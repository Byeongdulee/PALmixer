# -*- coding: utf-8 -*-
"""PLACEHOLDER pump driver.

No real pump control package exists yet for the PALmixer mixer/flowcell
plumbing. This stub gives the server and GUI a stable interface to code
against now; swap the body of each method for a real driver later (a
dedicated ZMQ/serial pump service) without touching server.py or the GUI.

Every operation returns ``(ok: bool, detail: str)`` and is expected to take a
little real time, so callers (server.py's worker thread) see it as a genuine
blocking action -- just like a real pump move would be.

All operations serialize on a single lock. This is what makes it safe for a
workflow to fire ``clean_mixer()`` on a daemon thread and move on without
waiting for it: a later call (e.g. ``draw_to_flowcell()``) simply blocks on
the lock until the clean finishes, so two pump ops never run concurrently.

**Mixing speed** is a real settable now, even though the pump behind it is not.
``set_speed``/``speed`` give a client something to read and write, ``mix()``
uses whatever it currently holds, and every mix reports the speed it ran at --
so a campaign that varies mixing speed as a design axis records a value that
came from the pump rather than one it merely asked for. When a real driver
arrives, ``_apply_speed`` is the one method that has to talk to hardware.
"""

import threading
import time

from . import config

PUMP_ACTION_DURATION_S = 1.0  # placeholder "how long the pump pretends to run"

#: Fallbacks when json/palmixer_config.json has no ``pump`` section.
DEFAULT_SPEED_RPM = 800.0
DEFAULT_MIN_RPM = 0.0
DEFAULT_MAX_RPM = 3000.0


class PumpError(Exception):
    """Raised by a real driver on communication/hardware failure."""


class Pump:
    """PLACEHOLDER pump. Replace with a real driver when hardware is available."""

    def __init__(self, **kwargs):
        # A real driver would take connection info here (serial port, host:port, ...).
        self._initialized = False
        self._lock = threading.Lock()

        section = {}
        try:
            section = config.get_section("pump")
        except Exception:                       # noqa: BLE001 - defaults are fine
            pass
        self.min_rpm = float(section.get("min_rpm", DEFAULT_MIN_RPM))
        self.max_rpm = float(section.get("max_rpm", DEFAULT_MAX_RPM))
        self._speed_rpm = float(section.get("mixing_speed_rpm", DEFAULT_SPEED_RPM))

    # -- mixing speed ---------------------------------------------------------
    @property
    def speed(self):
        """The speed the next ``mix()`` will run at, in rpm."""
        return self._speed_rpm

    def set_speed(self, rpm):
        """Set the mixing speed. Returns ``(ok, detail)`` like every pump op.

        Bounds-checked rather than clamped: a campaign that asked for 5000 rpm on
        a 3000 rpm pump has a design space that does not match the hardware, and
        silently mixing at 3000 would put a value in the record that nothing
        actually ran at. Better to refuse and have the mismatch found now.
        """
        try:
            value = float(rpm)
        except (TypeError, ValueError):
            return False, "mixing speed must be a number, got %r" % (rpm,)
        if value != value:                      # NaN
            return False, "mixing speed must be a number, got %r" % (rpm,)
        if not self.min_rpm <= value <= self.max_rpm:
            return False, ("mixing speed must be %g..%g rpm, got %g"
                           % (self.min_rpm, self.max_rpm, value))
        with self._lock:
            self._speed_rpm = value
            ok, detail = self._apply_speed(value)
        return ok, detail

    def _apply_speed(self, rpm):
        """Where a real driver would talk to the hardware. The one seam to replace."""
        print("PLACEHOLDER Pump: mixing speed set to %g rpm (no hardware)." % rpm)
        return True, "mixing speed %g rpm (placeholder, no hardware)" % rpm

    def initialize(self):
        print("PLACEHOLDER Pump: initialize() -- no hardware connected.")
        self._initialized = True
        return True, "pump initialized (placeholder)"

    def mix(self):
        """Mix the sample in the mixer, at the current speed."""
        return self._do("mix", "at %g rpm" % self._speed_rpm)

    def clean_mixer(self):
        """Flush/clean the mixer."""
        return self._do("clean_mixer")

    def draw_to_flowcell(self):
        """Draw mixed solution from the mixer into the flowcell."""
        return self._do("draw_to_flowcell")

    def aspirate_from_flowcell(self):
        """Aspirate solution back out of the flowcell."""
        return self._do("aspirate_from_flowcell")

    def wash_flowcell(self):
        """Wash the flowcell after a sample has been returned to it."""
        return self._do("wash_flowcell")

    def _do(self, op, note=""):
        with self._lock:
            print("PLACEHOLDER Pump: running %r %s..." % (op, note))
            time.sleep(PUMP_ACTION_DURATION_S)
            print("PLACEHOLDER Pump: %r complete." % op)
            # The note carries the speed for a mix, so the value that ran ends up in
            # the workflow's step detail and from there in the sample's record --
            # rather than only the value someone asked for.
            return True, "%s complete%s (placeholder, no hardware)" % (
                op, " %s" % note if note else "")
