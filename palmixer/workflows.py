# -*- coding: utf-8 -*-
"""Composite sample-handling sequences: make_sample and unload_sample.

Each transport step updates `state` itself (see the @_tracks decorator in
PAL12idb.py), so this module only sequences calls -- PAL12idb transport
functions, pump operations, and the carousel motor move -- and adds the
guards that decide whether a sequence is safe to start at all.

Guard checks (the `_check_*` methods) are pure reads of `state`: they never
move anything, so server.py calls them synchronously, before accepting a
`make_sample`/`unload_sample` command, to reply "ERROR: <reason>" right away
instead of accepting and failing several steps into a multi-minute
sequence. `make_sample`/`unload_sample` re-run the same check before moving,
so calling either directly (e.g. from a test) is still safe on its own.
"""

import threading
import time

from . import state


class WorkflowError(Exception):
    """Raised when a guard fails and the workflow refuses to start."""


class Workflows:
    def __init__(self, pal, robot, pump, motor, on_step=None, simulate=False):
        self.pal = pal
        self.robot = robot
        self.pump = pump
        self.motor = motor
        self.on_step = on_step or (lambda step, phase, detail="": None)
        self.simulate = simulate

    # -- step bookkeeping -----------------------------------------------------
    def _step(self, name, fn):
        self.on_step(name, "started", "")
        try:
            result = fn()
        except Exception as e:
            self.on_step(name, "failure", str(e))
            raise
        detail = result or ""
        self.on_step(name, "success", detail)
        return detail

    def _simulated_step(self, name, seconds=1.0, on_success=None):
        return self._step(name, lambda: self._sleep_and_report(name, seconds, on_success))

    def _sleep_and_report(self, name, seconds, on_success=None):
        time.sleep(seconds)
        if on_success is not None:
            on_success()
        return "simulated %s" % name

    def _pump_op(self, op):
        ok, detail = getattr(self.pump, op)()
        if not ok:
            raise WorkflowError(detail)
        return detail

    def _start_background_pump_op(self, op):
        """Fire `op` on a daemon thread; its own on_step trio lands whenever
        it finishes, without the caller waiting for it. Safe to overlap with
        later steps because Pump serializes its operations on its own lock --
        a later blocking pump call simply waits for this one."""
        def run():
            try:
                detail = self._pump_op(op)
                self.on_step(op, "success", detail)
            except Exception as e:
                self.on_step(op, "failure", str(e))
        self.on_step(op, "started", "(not awaited)")
        threading.Thread(target=run, daemon=True).start()

    @staticmethod
    def _require_known(value, what, allowed):
        if value == state.UNKNOWN:
            raise WorkflowError(
                '%s location is unknown; run "set_location %s <%s>" to reconcile'
                % (what, what, "|".join(allowed)))

    # -- make_sample ------------------------------------------------------------
    def _check_make_sample(self, slot):
        """Validate preconditions without moving anything. Returns the taught
        motor position for `slot`. Raises WorkflowError."""
        fc = state.get_flowcell_in_use()
        fc_loc = state.get_flowcell_location(fc)
        what = "flowcell_%d" % fc
        self._require_known(fc_loc, what, (state.FC_AT_CLEANING, state.FC_AT_BEAM))
        if fc_loc != state.FC_AT_CLEANING:
            raise WorkflowError(
                "flowcell %d must be at its cleaning station to make a sample "
                "(currently %s)" % (fc, fc_loc))

        mixer_loc = state.get_mixer_head()
        self._require_known(mixer_loc, state.WHAT_MIXER_HEAD,
                             (state.MIXER_AT_MIXER, state.MIXER_AT_CLEANING))

        try:
            return state.slot_position(slot)
        except KeyError as e:
            # str() on a KeyError is the repr of its argument, i.e. wrapped in
            # quotes; take the message itself so the reply reads cleanly.
            raise WorkflowError(e.args[0])

    def make_sample(self, slot):
        """Mix a fresh sample and load it at the beam. See the module plan
        for the full 9-step sequence; each PAL12idb call updates `state`."""
        target_position = self._check_make_sample(slot)
        mixer_loc = state.get_mixer_head()

        if self.simulate:
            fc = state.get_flowcell_in_use()
            steps = (
                ("mixer2cleaningstation", lambda: state.set_mixer_head(state.MIXER_AT_CLEANING)),
                ("rotate_carousel", lambda: state.set_carousel_slot(slot)),
                ("mixer2mixingstation", lambda: state.set_mixer_head(state.MIXER_AT_MIXER)),
                ("mix", None),
                ("mixer2cleaningstation", lambda: state.set_mixer_head(state.MIXER_AT_CLEANING)),
                ("clean_mixer", None),
                ("ready_flowcell_to_draw", lambda: state.set_flowcell_location(fc, state.FC_IN_GRIPPER)),
                ("draw_to_flowcell", None),
                ("load_sample_to_beam", lambda: state.set_flowcell_location(fc, state.FC_AT_BEAM)),
            )
            for step, on_success in steps:
                self._simulated_step(step, on_success=on_success)
            return "simulated make_sample(slot=%s) complete" % slot

        if mixer_loc == state.MIXER_AT_MIXER:
            self._step("mixer2cleaningstation", lambda: self.pal.mixer2cleaningstation(self.robot))

        self._step("rotate_carousel", lambda: self._rotate_carousel(slot, target_position))
        self._step("mixer2mixingstation", lambda: self.pal.mixer2mixingstation(self.robot))
        self._step("mix", lambda: self._pump_op("mix"))
        self._step("mixer2cleaningstation", lambda: self.pal.mixer2cleaningstation(self.robot))

        # No need to wait for the mixer to finish cleaning before moving on.
        self._start_background_pump_op("clean_mixer")

        self._step("ready_flowcell_to_draw", lambda: self.pal.ready_flowcell_to_draw(self.robot))
        # Blocks on the pump's lock until clean_mixer (if still running) finishes.
        self._step("draw_to_flowcell", lambda: self._pump_op("draw_to_flowcell"))
        self._step("load_sample_to_beam", lambda: self.pal.load_sample_to_beam(self.robot))

        return "make_sample(slot=%s) complete" % slot

    def _rotate_carousel(self, slot, target_position):
        self.motor.move_to(target_position)
        state.set_carousel_slot(slot)
        return "carousel at slot %s (%.4f)" % (slot, target_position)

    # -- unload_sample ------------------------------------------------------------
    def _check_unload_sample(self):
        fc = state.get_flowcell_in_use()
        fc_loc = state.get_flowcell_location(fc)
        what = "flowcell_%d" % fc
        self._require_known(fc_loc, what, (state.FC_AT_CLEANING, state.FC_AT_BEAM))
        if fc_loc != state.FC_AT_BEAM:
            raise WorkflowError(
                "flowcell %d must be at the sample table to unload "
                "(currently %s)" % (fc, fc_loc))

        mixer_loc = state.get_mixer_head()
        self._require_known(mixer_loc, state.WHAT_MIXER_HEAD,
                             (state.MIXER_AT_MIXER, state.MIXER_AT_CLEANING))

    def unload_sample(self):
        """Return the sample to the mixer, wash the flowcell, and clear the
        beam. See the module plan for the full 6-step sequence."""
        self._check_unload_sample()
        mixer_loc = state.get_mixer_head()

        if self.simulate:
            fc = state.get_flowcell_in_use()
            steps = (
                ("mixer2cleaningstation", lambda: state.set_mixer_head(state.MIXER_AT_CLEANING)),
                ("return_sample", lambda: state.set_flowcell_location(fc, state.FC_IN_GRIPPER)),
                ("aspirate_from_flowcell", None),
                ("wash_flowcell_after_return", lambda: state.set_flowcell_location(fc, state.FC_AT_CLEANING)),
                ("wash_flowcell", None),
                ("raise_to_sampletable_height", None),
            )
            for step, on_success in steps:
                self._simulated_step(step, on_success=on_success)
            return "simulated unload_sample complete"

        if mixer_loc == state.MIXER_AT_MIXER:
            self._step("mixer2cleaningstation", lambda: self.pal.mixer2cleaningstation(self.robot))

        self._step("return_sample", lambda: self.pal.return_sample(self.robot))
        self._step("aspirate_from_flowcell", lambda: self._pump_op("aspirate_from_flowcell"))
        self._step("wash_flowcell_after_return", lambda: self.pal.wash_flowcell_after_return(self.robot))
        self._step("wash_flowcell", lambda: self._pump_op("wash_flowcell"))
        self._step("raise_to_sampletable_height",
                    lambda: self.pal.raise_to_sampletable_height(self.robot))

        return "unload_sample complete"
