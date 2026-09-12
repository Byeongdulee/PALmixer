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

make_sample mixes a fresh vial and puts it in the beam:

    mixer2cleaningstation               (only if the head is at the mixer)
    rotate_carousel                     (to the slot being mixed)
    mixer2mixingstation
    mix                                 (pump; the sample ID is assigned here)
    mixer2cleaningstation
    clean_mixer                         (pump 5555, not awaited)
    advance_carousel                    (draw_offset_steps forward, awaited)
    ready_flowcell_to_draw              (onto sample_on_mixer_station)
    draw_to_flowcell                    (pump 5556, runs while 5555 cleans)
    load_sample_to_beam
    shake_sample                        (pump, auto, not awaited)
    park_at_transfer_point

A wait_for_<op> step appears only where a not-awaited op is still running on
the pump the next step needs; the mixer clean and the flowcell draw are on
different servers and overlap, so it does not appear between those two.

draw_and_load and draw_load_sample are make_sample's tail without the mixing,
for a vial that already holds what is wanted -- draw_load_sample additionally
turns the carousel to a named slot's drawing position first:

    advance_carousel                    (draw_load_sample only, to <slot>)
    mixer2cleaningstation               (only if the head is at the mixer)
    ready_flowcell_to_draw
    draw_to_flowcell                    (pump)
    load_sample_to_beam
    shake_sample                        (pump, auto, not awaited)
    park_at_transfer_point

unload_sample runs one of two sequences. The default discards the sample:

    load_flowcell_from_beam_to_cleaningstation
    wash_flowcell                       (pump, not awaited)
    raise_to_sampletable_height
    park_at_transfer_point

With `aspirate`, the sample is recovered into its vial first, which is what
this workflow did on every run before the flag existed:

    mixer2cleaningstation               (only if the head is at the mixer)
    return_sample
    aspirate_from_flowcell              (pump)
    wash_flowcell_after_return
    wash_flowcell                       (pump, awaited)
    raise_to_sampletable_height
    park_at_transfer_point

park_at_transfer_point (Workflows._park) runs only on a clean finish, as the
last step of each workflow -- never from a `finally`, and never after a step
above it raises. An arm that stopped partway through a sequence may still be
holding the flowcell or mid-descent, and a blind move to the corridor is not
obviously safe then; the operator's own judgement decides what to do with it,
the way every other mid-workflow failure already leaves that to them.

shake_sample (Workflows._start_auto_shake) is a loop, not a single pump call:
once the flowcell is loaded it re-fires `shake_sample` on a background thread
for as long as the sample sits at the beam, pinned to the flowcell that was
just loaded regardless of later changes to the "flowcell in use" selector.
Workflows.stop_auto_shake() ends it -- called at the start of unload_sample
(either sequence) and from the server whenever
load_flowcell_from_beam_to_cleaningstation runs on its own (the Experiment
tab's transport button), since the flowcell is about to leave the beam either
way. It also interrupts a shake started by the plain Shake Sample button, not
only one this loop started itself.

mixer2cleaningstation and mixer2mixingstation move the mixer head, which
clean_mixer washes by running liquid through it while it sits docked at the
cleaning station -- moving it away mid-wash risks spilling that, damaging the
tubing, or fouling the needle alignment. clean_mixer is fired without being
awaited (see above), so it can still be running well after the make_sample
that started it has returned and the server has gone idle again; nothing else
here waits for it to finish on its own the way a same-server pump op does.
Workflows.mixer_head_busy() answers whether it still is, checked fresh every
time (not cached), and Workflows._move_mixer_head() is the one place every
internal call to either transport goes through, refusing with a WorkflowError
if so. server.py additionally refuses outright, before ACCEPTED, for the two
cases an operator can trigger directly: the Experiment tab's own buttons for
either transport, and make_sample (_check_make_sample) -- so those get
"ERROR: ..." immediately rather than an ACCEPTED that then fails once the
sequence reaches the step that would have moved the head.
"""

import threading
import time

from . import state
# Imported as a bare name rather than through the module, because `pump` is
# also the name of a constructor argument and an attribute here.
from .pump import server_for as _pump_server_for

# How far the carousel advances between mixing a vial and drawing from it, in
# carousel steps -- `carousel.draw_offset_steps` in json/palmixer_config.json,
# 4 by default. The mixer head and the flowcell's draw point are not over the
# same slot: the head comes down where rotate_carousel left the carousel, and
# the flowcell reaches the vial some steps on. So once the head is clear and
# washing, the carousel is advanced this far to bring the vial just mixed round
# to the draw point (see Workflows._advance_carousel).
#
# Configured rather than hard-coded, and in steps rather than motor units, so
# the distance follows `carousel.step` and neither has to be changed in code.
def _draw_offset_steps():
    return state.get_carousel_draw_offset_steps()

# How long an auto-shake loop pauses between cycles once one finishes on its
# own (not stopped). Purely a breather -- stop_auto_shake() interrupts a
# cycle already in progress via Pump.stop_shaking() rather than waiting for
# this gap, so it does not control how quickly a stop takes effect.
AUTO_SHAKE_PAUSE_S = 0.5


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
        later steps because Pump serializes per server -- a later call to the
        *same* pump waits for this one, and a call to the other pump does not
        (the mixer and the flowcell are independent hardware)."""
        def run():
            try:
                detail = self._pump_op(op)
                self.on_step(op, "success", detail)
            except Exception as e:
                self.on_step(op, "failure", str(e))
        self.on_step(op, "started", "(not awaited)")
        thread = threading.Thread(target=run, daemon=True)
        self._background_pump = (op, thread)
        thread.start()

    def _background_pump_status(self):
        """(op, thread) of a still-running background pump op, or None.

        Freshly checks `thread.is_alive()` rather than trusting the bookkeeping
        left by _start_background_pump_op: nothing proactively clears that once
        the thread actually finishes, only the next caller that asks. Clears it
        here when it finds a dead thread, so a stale entry cannot linger and
        make something look busy that has long since finished.
        """
        entry = getattr(self, "_background_pump", None)
        if entry is None:
            return None
        op, thread = entry
        if not thread.is_alive():
            self._background_pump = None
            return None
        return entry

    def _await_background_pump(self, before_op):
        """Wait out a background pump op, but only if it would block `before_op`.

        Two ops on different pump servers cannot block each other, so waiting
        for one before the other is pure lost time -- the mixer clean and the
        flowcell draw are meant to overlap. Only a background op on the *same*
        server is a real constraint, and that one is waited for here rather
        than inside the pump call, so a wait of minutes is a named step instead
        of a step that appears to have stalled.
        """
        entry = self._background_pump_status()
        if entry is None:
            return
        op, thread = entry
        if _pump_server_for(op) != _pump_server_for(before_op):
            return                      # different pump: they run side by side
        self._background_pump = None
        self._step("wait_for_%s" % op, lambda: self._join_pump(thread, op))

    @staticmethod
    def _join_pump(thread, op):
        thread.join()
        # The op reports its own success or failure through its own on_step
        # trio; this step only ever says the waiting is over.
        return "%s finished; the pump is free" % op

    def mixer_head_busy(self):
        """Is the mixer head currently being washed (a background clean_mixer
        still running)?

        True while the head is docked at the cleaning station with liquid
        actively flowing through it -- moving it away mid-wash risks spilling
        it, damaging the tubing, or fouling the needle alignment. Unlike a
        pump-to-pump wait, nothing else here waits for this on its own, so
        anything that is about to move the head has to check fresh.
        """
        entry = self._background_pump_status()
        return entry is not None and entry[0] == "clean_mixer"

    def _move_mixer_head(self, name, fn):
        """Run a mixer-head transport (mixer2cleaningstation /
        mixer2mixingstation), refused while mixer_head_busy(). The one place
        every internal call to either goes through, so the refusal applies
        everywhere the head could be moved, not just the two cases with their
        own pre-flight check (the Experiment tab's buttons, in server.py, and
        make_sample, in _check_make_sample).
        """
        if self.mixer_head_busy():
            raise WorkflowError(
                "mixer head cannot be moved while it is being washed")
        return self._step(name, fn)

    def _start_auto_shake(self):
        """Begin shaking the flowcell just loaded, repeating until
        stop_auto_shake() is called. Fired after load_sample_to_beam in
        make_sample, draw_and_load, and draw_load_sample -- not awaited, so
        parking (and whatever the operator does next) runs while the sample
        is agitated at the beam.

        Pinned to the flowcell that was just loaded (Pump.shake_sample's
        `flowcell_id`), not whatever the "flowcell in use" selector reads
        later: switching that selector to work on the other flowcell must not
        silently retarget a shake already running for this one.
        """
        # Only one flowcell can physically be mid-shake at a time (the
        # flowcell dashboard serializes both its pumps through one recipe
        # thread), so a second auto-shake starting is a hand-off, not two
        # loops sharing the hardware.
        self.stop_auto_shake()
        fc = state.get_flowcell_in_use()
        stop_event = threading.Event()

        def run():
            try:
                while not stop_event.is_set():
                    ok, detail = self.pump.shake_sample(flowcell_id=fc)
                    if not ok:
                        self.on_step("shake_sample", "failure", detail)
                        return
                    if stop_event.wait(AUTO_SHAKE_PAUSE_S):
                        return
            except Exception as e:
                self.on_step("shake_sample", "failure", str(e))
                return
            self.on_step("shake_sample", "success", "stopped")

        thread = threading.Thread(target=run, daemon=True)
        self._auto_shake = (stop_event, thread)
        self.on_step("shake_sample", "started",
                    "(auto, flowcell %s, not awaited)" % fc)
        thread.start()

    def stop_auto_shake(self):
        """Stop shaking now, whatever started it -- an auto-shake loop or a
        one-off press of the plain Shake Sample button -- and stop an
        auto-shake loop from starting another cycle. Safe to call when
        nothing is shaking.

        Called at the start of unload_sample (either sequence) and by the
        server whenever load_flowcell_from_beam_to_cleaningstation runs on
        its own: once the flowcell is leaving the sample table, there is
        nothing left for a shake to agitate.
        """
        entry = getattr(self, "_auto_shake", None)
        if entry is not None:
            stop_event, _thread = entry
            self._auto_shake = None
            stop_event.set()
        return self.pump.stop_shaking()

    def _park(self):
        """Move the arm to the transfer point. The last step of a workflow
        that finished cleanly -- see the module plan for why only then.

        Every workflow ends here rather than wherever its last real step left
        the arm, so the next command, from either side of the cell, starts
        from a known, out-of-the-way pose instead of one specific to whatever
        ran last.
        """
        if self.simulate:
            self._simulated_step("park_at_transfer_point")
        else:
            self._step("park_at_transfer_point",
                        lambda: self.pal.goto_transfer_point(self.robot))

    # -- position requirements -------------------------------------------------
    # Composed from PAL12idb.TRANSPORT_STATIONS rather than listed separately,
    # so a change to which stations a transport function reads cannot leave a
    # stale second copy behind. server.py calls these before accepting a
    # workflow command, to refuse one that needs a station nobody has taught.
    def _stations(self, *transport_names):
        if self.pal is None:  # simulate mode: no PAL12idb, nothing to check
            return ()
        stations = ()
        for name in transport_names:
            stations += self.pal.TRANSPORT_STATIONS[name]
        return stations

    def stations_for_make_sample(self):
        # The opening mixer2cleaningstation is conditional, but step 5 runs it
        # unconditionally, so its stations are needed either way.
        return self._stations('mixer2cleaningstation', 'mixer2mixingstation',
                               'ready_flowcell_to_draw', 'load_sample_to_beam')

    def stations_for_draw_and_load(self):
        # Same conditional as make_sample's opener, and for the same reason:
        # the mixer cleaning station is only involved when the head is sitting
        # at the mixer and has to be parked before the flowcell can come in.
        if self.pal is None:
            return ()
        names = ('ready_flowcell_to_draw', 'load_sample_to_beam')
        if state.get_mixer_head() == state.MIXER_AT_MIXER:
            names = ('mixer2cleaningstation',) + names
        return self._stations(*names)

    def stations_for_unload_sample(self, aspirate=False):
        # raise_to_sampletable_height reads the sample table directly rather
        # than through a transport function, so it is not in the table above
        # and is added by hand to both paths.
        if self.pal is None:
            return ()
        if not aspirate:
            # One leg, sample table -> cleaning station. Nothing here goes
            # near the mixer, so neither mixer position is required.
            return (self._stations('load_flowcell_from_beam_to_cleaningstation')
                    + ('sample_table',))
        # The mixer head only has to be parked out of the way when it is
        # actually sitting at the mixer station; otherwise the opening
        # mixer2cleaningstation is skipped and the mixer cleaning station is
        # never touched, so requiring it would refuse a run that would work.
        names = ('return_sample', 'wash_flowcell_after_return')
        if state.get_mixer_head() == state.MIXER_AT_MIXER:
            names = ('mixer2cleaningstation',) + names
        return self._stations(*names) + ('sample_table',)

    @staticmethod
    def _require_known(value, what, allowed):
        if value == state.UNKNOWN:
            raise WorkflowError(
                '%s location is unknown; run "set_location %s <%s>" to reconcile'
                % (what, what, "|".join(allowed)))

    # -- make_sample ------------------------------------------------------------
    def _check_make_sample(self, slot, sample_id=None):
        """Validate preconditions without moving anything. Returns the taught
        motor position for `slot`. Raises WorkflowError."""
        # Slot range and sample ID first: both are cheap, and both are things
        # the operator can fix, so they should be refused before the state
        # guards start talking about where the hardware is.
        try:
            state.validate_slot(slot)
            if sample_id is not None:
                state.clean_sample_id(sample_id)
        except ValueError as e:
            raise WorkflowError(str(e))

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

        # Refused outright rather than accepted-then-failed: a wash left
        # running by a previous make_sample can still be going once this one
        # would otherwise reach mixer2mixingstation (make_sample's own
        # opening mixer2cleaningstation runs before this, so this alone would
        # not catch it there -- _move_mixer_head is the backstop for that gap
        # and for every other internal call to either mixer transport).
        if self.mixer_head_busy():
            raise WorkflowError(
                "cannot start make_sample while the mixer head is being washed")

        # Every slot position is derived from one taught reference, so a reference
        # taught on a different carousel sends the robot to where that one's slot
        # was. Refused rather than warned: the failure is a collision, not a bad
        # number. carousel.keep_reference_on_mount declares a repeatable mount and
        # turns this off.
        if state.carousel_reference_stale():
            raise WorkflowError(
                'the taught slot positions belong to carousel %r but %r is mounted. '
                'Teach one slot on this carousel ("teach_carousel_slot <n>"), or set '
                'carousel.keep_reference_on_mount if it seats repeatably.'
                % (state.reference_carousel_id() or "an unnamed one",
                   state.get_carousel_id()))

        try:
            return state.slot_position(slot)
        except KeyError as e:
            # str() on a KeyError is the repr of its argument, i.e. wrapped in
            # quotes; take the message itself so the reply reads cleanly.
            raise WorkflowError(e.args[0])

    def make_sample(self, slot, sample_id=None):
        """Mix a fresh sample and load it at the beam. See the module plan
        for the full step list; each PAL12idb call updates `state`.

        `sample_id` tags the carousel slot the sample is mixed in. None means
        assign one, which happens when the mix finishes rather than now -- an
        auto ID is a timestamp, and it should name the moment the sample came
        into existence, not the moment a run was requested several minutes of
        transport earlier. A slot consumed by a direct call (a script, a test)
        is never left anonymous either way."""
        target_position = self._check_make_sample(slot, sample_id)
        mixer_loc = state.get_mixer_head()

        if self.simulate:
            fc = state.get_flowcell_in_use()
            # The ID is minted inside the mix step, so it has to come back out
            # of the closure to reach the completion detail below.
            minted = []
            steps = (
                ("mixer2cleaningstation", lambda: state.set_mixer_head(state.MIXER_AT_CLEANING)),
                ("rotate_carousel", lambda: state.set_carousel_slot(slot)),
                ("mixer2mixingstation", lambda: state.set_mixer_head(state.MIXER_AT_MIXER)),
                ("mix", lambda: minted.append(self._assign_sample_id(slot, sample_id))),
                ("mixer2cleaningstation", lambda: state.set_mixer_head(state.MIXER_AT_CLEANING)),
                ("clean_mixer", None),
                ("advance_carousel", lambda: self._advance_carousel_state(slot)),
                ("ready_flowcell_to_draw", lambda: state.set_flowcell_location(fc, state.FC_IN_GRIPPER)),
                ("draw_to_flowcell", None),
                ("load_sample_to_beam", lambda: state.set_flowcell_location(fc, state.FC_AT_BEAM)),
                ("shake_sample", None),
            )
            for step, on_success in steps:
                self._simulated_step(step, on_success=on_success)
            self._park()
            return self._make_sample_detail("simulated make_sample", slot,
                                            minted[0] if minted else sample_id)

        if mixer_loc == state.MIXER_AT_MIXER:
            self._move_mixer_head("mixer2cleaningstation",
                                  lambda: self.pal.mixer2cleaningstation(self.robot))

        self._step("rotate_carousel", lambda: self._rotate_carousel(slot, target_position))
        self._move_mixer_head("mixer2mixingstation",
                              lambda: self.pal.mixer2mixingstation(self.robot))
        # A clean_mixer left running by the previous make_sample is on this
        # same pump, so it has to finish before this one can mix.
        self._await_background_pump("mix")
        self._step("mix", lambda: self._pump_op("mix"))
        # The vial is spent the moment it has been mixed, so the slot is marked
        # here rather than at the end: a later step failing does not un-consume
        # it, and the record must not depend on the run finishing. An unnamed
        # sample is named here too, for the same reason -- this is the instant
        # it became a sample.
        sample_id = self._assign_sample_id(slot, sample_id)
        self._move_mixer_head("mixer2cleaningstation",
                              lambda: self.pal.mixer2cleaningstation(self.robot))

        # No need to wait for the mixer to finish cleaning before moving on.
        self._start_background_pump_op("clean_mixer")

        # Now that the head is off the carousel and washing, turn the vial
        # round to the draw point. Awaited, unlike the wash: the flowcell comes
        # down onto whatever slot is underneath it, so the next step must not
        # start until the carousel has actually stopped there.
        self._step("advance_carousel",
                    lambda: self._advance_carousel(slot, target_position))

        self._step("ready_flowcell_to_draw", lambda: self.pal.ready_flowcell_to_draw(self.robot))
        # The clean_mixer started above is on the other pump and is left to run
        # alongside this. Only a flowcell-side leftover -- the wash fired by a
        # preceding unload_sample -- is waited for here.
        self._await_background_pump("draw_to_flowcell")
        self._step("draw_to_flowcell", lambda: self._pump_op("draw_to_flowcell"))
        self._step("load_sample_to_beam", lambda: self.pal.load_sample_to_beam(self.robot))
        self._start_auto_shake()
        self._park()

        return self._make_sample_detail("make_sample", slot, sample_id)

    def _assign_sample_id(self, slot, sample_id):
        """Name the sample just mixed into `slot` and record it. Returns the ID.

        `sample_id` None mints a timestamp one. Writing it through state is
        what publishes it: the write fires the tracking listener, so the ID
        reaches every client on the next tracking snapshot -- which is how the
        GUI fills in an ID it did not type.
        """
        if sample_id is None:
            sample_id = state.new_sample_id()
        return self._mark_slot_used(slot, sample_id)

    @staticmethod
    def _mark_slot_used(slot, sample_id):
        """Record which sample now occupies the carousel slot, marking it used.
        Overwrites an existing ID -- re-mixing a slot is allowed. Returns the
        ID as stored (state.set_sample_id normalises the text)."""
        return state.set_sample_id(slot, sample_id)

    @staticmethod
    def _make_sample_detail(what, slot, sample_id):
        detail = "%s(slot=%s, sample=%s) complete" % (what, slot, sample_id)
        if state.carousel_is_full():
            # Advisory: the carousel is a consumable and this run spent the last
            # slot. Nothing is blocked -- re-mixing a used slot stays legal --
            # but the operator needs to know a replacement is due.
            detail += " -- carousel full (%d/%s slots used), replace it and reset" % (
                state.used_slot_count(), state.get_carousel_size())
        return detail

    def _rotate_carousel(self, slot, target_position):
        # The advance that follows the mix has to be reachable too, and this is
        # the last moment at which finding out costs nothing: after the mix it
        # costs the vial. A motor record silently ignores a move past its soft
        # limits, so an unreachable advance is not something the run would
        # otherwise notice going wrong -- see Motor.check_in_range.
        self.motor.check_in_range(
            target_position + _draw_offset_steps() * state.require_carousel_step())
        self.motor.move_to(target_position)
        state.set_carousel_slot(slot)
        return "carousel at slot %s (%.4f)" % (slot, target_position)

    def _advance_carousel(self, slot, mixed_position):
        """Turn the carousel draw_offset_steps forward of `mixed_position`.

        Blocks until the motor stops: Motor.move_to waits on .DMOV, so this
        returns only once the carousel is actually there, and a stage that
        never gets there raises MotorError rather than letting the flowcell
        come down on a slot still in motion.

        Absolute, computed from the position this run mixed at rather than
        from wherever the motor happens to read -- the same reason
        rotate_carousel is absolute. A relative tweak would fold every
        rounding error of every previous run into the next one.
        """
        step = state.require_carousel_step()
        steps = _draw_offset_steps()
        target = mixed_position + steps * step
        self.motor.move_to(target)
        now = self._slot_after_advance(slot)
        if now is not None:
            state.set_carousel_slot(now)
        return "carousel advanced %d steps (%.4f) to %.4f (slot %s at the mixer, " \
               "slot %s at the draw point)" % (steps, steps * step, target,
                                               now if now is not None else "?", slot)

    def _advance_carousel_state(self, slot):
        """The bookkeeping half of _advance_carousel, for the simulate path:
        no motor, but the tracked slot still moves so the GUI shows what a real
        run would."""
        now = self._slot_after_advance(slot)
        if now is not None:
            state.set_carousel_slot(now)

    @staticmethod
    def _slot_after_advance(slot):
        """Which slot sits at the mixing point after the advance, or None.

        The tracked slot means "the slot the carousel is turned to", so it has
        to move with the carousel. It wraps: the carousel is a ring, and past
        the last slot the numbering comes back round to the first. None when
        the carousel size is not configured -- there is no ring to wrap around
        then, and a wrong slot number is worse than none.
        """
        size = state.get_carousel_size()
        if size == state.UNKNOWN:
            return None
        return (int(slot) - 1 + _draw_offset_steps()) % size + 1

    # -- draw_and_load ------------------------------------------------------------
    def _check_draw_and_load(self):
        """Validate preconditions without moving anything. Raises WorkflowError."""
        fc = state.get_flowcell_in_use()
        fc_loc = state.get_flowcell_location(fc)
        what = "flowcell_%d" % fc
        self._require_known(fc_loc, what, (state.FC_AT_CLEANING, state.FC_AT_BEAM))
        if fc_loc != state.FC_AT_CLEANING:
            raise WorkflowError(
                "flowcell %d must be at the cleaning station to draw into "
                "(currently %s)" % (fc, fc_loc))

        # Unlike the default unload, this one does go to the mixer -- to draw
        # from it -- so where the head is has to be known either way: to decide
        # whether to park it first, and because the flowcell is set down on the
        # station it would otherwise still be occupying.
        mixer_loc = state.get_mixer_head()
        self._require_known(mixer_loc, state.WHAT_MIXER_HEAD,
                             (state.MIXER_AT_MIXER, state.MIXER_AT_CLEANING))

    def draw_and_load(self):
        """Draw an already-mixed sample into the flowcell and put it in the beam.

        The tail of make_sample without the mixing: for a vial that already
        holds what is wanted -- one mixed by an earlier run, or a sample just
        recovered by `unload_sample aspirate`. Nothing here touches the
        carousel, so no slot is consumed and no sample ID is assigned; the
        vial the mixer is sitting over is whatever the last rotate left there.
        See the module plan for the step list.
        """
        self._check_draw_and_load()
        mixer_loc = state.get_mixer_head()

        if self.simulate:
            fc = state.get_flowcell_in_use()
            steps = (
                ("mixer2cleaningstation", lambda: state.set_mixer_head(state.MIXER_AT_CLEANING)),
                ("ready_flowcell_to_draw", lambda: state.set_flowcell_location(fc, state.FC_IN_GRIPPER)),
                ("draw_to_flowcell", None),
                ("load_sample_to_beam", lambda: state.set_flowcell_location(fc, state.FC_AT_BEAM)),
                ("shake_sample", None),
            )
            for step, on_success in steps:
                self._simulated_step(step, on_success=on_success)
            self._park()
            return "simulated draw_and_load complete"

        if mixer_loc == state.MIXER_AT_MIXER:
            self._move_mixer_head("mixer2cleaningstation",
                                  lambda: self.pal.mixer2cleaningstation(self.robot))

        self._step("ready_flowcell_to_draw", lambda: self.pal.ready_flowcell_to_draw(self.robot))
        # Awaited, unlike unload's wash: the arm is holding the flowcell down
        # on the mixer for the duration, and it must not be carried to the beam
        # until the draw has actually finished putting sample in it.
        self._await_background_pump("draw_to_flowcell")
        self._step("draw_to_flowcell", lambda: self._pump_op("draw_to_flowcell"))
        self._step("load_sample_to_beam", lambda: self.pal.load_sample_to_beam(self.robot))
        self._start_auto_shake()
        self._park()

        return "draw_and_load complete"

    # -- draw_load_sample ---------------------------------------------------------
    # draw_and_load's counterpart for a named slot: it turns the carousel to
    # `slot`'s *drawing* position (draw_offset_steps forward of its mixing
    # position -- see _advance_carousel) first, instead of drawing from
    # whatever vial the mixer was last left over. Still no mixing, so no slot
    # is consumed and no ID is minted -- the slot is expected to have been
    # prepared already.
    def _check_draw_load_sample(self, slot, sample_id=None):
        """Validate a preloaded slot without running the mixer."""
        try:
            state.validate_slot(slot)
            if sample_id is not None:
                state.clean_sample_id(sample_id)
        except ValueError as e:
            raise WorkflowError(str(e))
        fc = state.get_flowcell_in_use()
        self._require_known(state.get_flowcell_location(fc), "flowcell_%d" % fc,
                            (state.FC_AT_CLEANING,))
        if state.get_flowcell_location(fc) != state.FC_AT_CLEANING:
            raise WorkflowError("flowcell %d must be at its cleaning station" % fc)
        if state.get_mixer_head() == state.UNKNOWN:
            raise WorkflowError("mixer head location is unknown")
        try:
            return state.slot_position(slot)
        except KeyError as e:
            raise WorkflowError(e.args[0])

    def draw_load_sample(self, slot, sample_id=None):
        """Draw an already-prepared slot into the selected flow cell without mixing."""
        target_position = self._check_draw_load_sample(slot, sample_id)
        if self.simulate:
            fc = state.get_flowcell_in_use()
            for step, on_success in (
                    ("advance_carousel", lambda: self._advance_carousel_state(slot)),
                    ("ready_flowcell_to_draw", lambda: state.set_flowcell_location(
                        fc, state.FC_IN_GRIPPER)),
                    ("draw_to_flowcell", None),
                    ("load_sample_to_beam", lambda: state.set_flowcell_location(
                        fc, state.FC_AT_BEAM)),
                    ("shake_sample", None)):
                self._simulated_step(step, on_success=on_success)
            self._park()
            return self._make_sample_detail("simulated draw_load_sample", slot,
                                            sample_id or "preloaded")

        # Turn the carousel to `slot`'s *drawing* position -- draw_offset_steps
        # forward of its mixing position, the same offset make_sample uses for
        # the vial it just mixed (see _advance_carousel) -- before the
        # flowcell comes down onto it. This has to run first: ready_flowcell_
        # to_draw lowers the flowcell onto and bumps whatever is currently
        # under the seat, so the requested slot has to already be there, not
        # spun into place underneath a flowcell that has already landed.
        self._step("advance_carousel", lambda: self._advance_carousel(slot, target_position))
        self._step("ready_flowcell_to_draw", lambda: self.pal.ready_flowcell_to_draw(self.robot))
        # The wash a preceding unload_sample left running is on this same pump,
        # and the arm holds the flowcell down on the vial until the draw
        # returns -- so it is waited out here, as a named step, rather than
        # stalling inside the draw.
        self._await_background_pump("draw_to_flowcell")
        self._step("draw_to_flowcell", lambda: self._pump_op("draw_to_flowcell"))
        self._step("load_sample_to_beam", lambda: self.pal.load_sample_to_beam(self.robot))
        self._start_auto_shake()
        self._park()
        return self._make_sample_detail("draw_load_sample", slot, sample_id or "preloaded")

    # -- unload_sample ------------------------------------------------------------
    def _check_unload_sample(self, aspirate=False):
        fc = state.get_flowcell_in_use()
        fc_loc = state.get_flowcell_location(fc)
        what = "flowcell_%d" % fc
        self._require_known(fc_loc, what, (state.FC_AT_CLEANING, state.FC_AT_BEAM))
        if fc_loc != state.FC_AT_BEAM:
            raise WorkflowError(
                "flowcell %d must be at the sample table to unload "
                "(currently %s)" % (fc, fc_loc))

        # Where the mixer head is only matters when the sample is going back
        # into it. The default run never approaches the mixer, so an unknown
        # mixer position there is no reason to refuse a sequence that would
        # not have touched it.
        if aspirate:
            mixer_loc = state.get_mixer_head()
            self._require_known(mixer_loc, state.WHAT_MIXER_HEAD,
                                 (state.MIXER_AT_MIXER, state.MIXER_AT_CLEANING))

    def unload_sample(self, aspirate=False, wait_clean=True):
        """Take the flowcell off the beam, wash it, and clear the arm.

        By default the sample is not kept: the flowcell goes straight from the
        sample table to its cleaning station, and the wash takes the sample
        with it. `aspirate` recovers it first -- carry the flowcell to the
        mixer, push the contents back into the vial they were mixed in, and
        only then go on to the cleaning station -- which is what this workflow
        used to do on every run. See the module plan for both step lists.

        `wait_clean` only bites on the aspirate path, which is the only one
        that still holds the sequence open for the wash. The default path
        backgrounds that wash either way, so there is nothing there to skip.
        """
        self._check_unload_sample(aspirate)
        # The flowcell is coming off the beam either way, so any shake --
        # auto-started after a load, or a one-off Shake Sample press -- has
        # nothing left to agitate. Stopped before either sequence below runs,
        # not after: aspirate's own return_sample is the first real step, and
        # a shake still running while that happens would jostle the flowcell
        # exactly as it is being picked up.
        self.stop_auto_shake()
        mixer_loc = state.get_mixer_head()

        if self.simulate:
            fc = state.get_flowcell_in_use()
            if aspirate:
                steps = (
                    ("mixer2cleaningstation", lambda: state.set_mixer_head(state.MIXER_AT_CLEANING)),
                    ("return_sample", lambda: state.set_flowcell_location(fc, state.FC_IN_GRIPPER)),
                    ("aspirate_from_flowcell", None),
                    ("wash_flowcell_after_return", lambda: state.set_flowcell_location(fc, state.FC_AT_CLEANING)),
                    ("wash_flowcell", None),
                    ("raise_to_sampletable_height", None),
                )
            else:
                # wash_flowcell is awaited here where the real run does not.
                # Simulation exists to pace the GUI through the same named
                # steps, and a background sleep would only report them out of
                # order for no gain.
                steps = (
                    ("load_flowcell_from_beam_to_cleaningstation",
                     lambda: state.set_flowcell_location(fc, state.FC_AT_CLEANING)),
                    ("wash_flowcell", None),
                    ("raise_to_sampletable_height", None),
                )
            for step, on_success in steps:
                self._simulated_step(step, on_success=on_success)
            self._park()
            return "simulated unload_sample complete"

        if not aspirate:
            self._step("load_flowcell_from_beam_to_cleaningstation",
                        lambda: self.pal.load_flowcell_from_beam_to_cleaningstation(self.robot))
            # The wash needs the flowcell to be sitting in the station, not the
            # arm that put it there, and nothing after this touches the pump --
            # so it runs alongside the lift rather than holding the sequence
            # open. Pump serializes on its own lock, so whatever pump operation
            # comes next (the mix of the following make_sample, typically)
            # waits for it in its turn.
            self._start_background_pump_op("wash_flowcell")
            self._step("raise_to_sampletable_height",
                        lambda: self.pal.raise_to_sampletable_height(self.robot))
            self._park()
            return "unload_sample complete (sample discarded with the wash)"

        if mixer_loc == state.MIXER_AT_MIXER:
            self._move_mixer_head("mixer2cleaningstation",
                                  lambda: self.pal.mixer2cleaningstation(self.robot))

        self._step("return_sample", lambda: self.pal.return_sample(self.robot))
        # A wash left running by a previous unload is on this same pump.
        self._await_background_pump("aspirate_from_flowcell")
        self._step("aspirate_from_flowcell", lambda: self._pump_op("aspirate_from_flowcell"))
        self._step("wash_flowcell_after_return", lambda: self.pal.wash_flowcell_after_return(self.robot))
        if wait_clean:
            self._step("wash_flowcell", lambda: self._pump_op("wash_flowcell"))
        else:
            self._start_background_pump_op("wash_flowcell")
        self._step("raise_to_sampletable_height",
                    lambda: self.pal.raise_to_sampletable_height(self.robot))
        self._park()

        return "unload_sample complete"
