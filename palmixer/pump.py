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
"""

import threading
import time

PUMP_ACTION_DURATION_S = 1.0  # placeholder "how long the pump pretends to run"


class PumpError(Exception):
    """Raised by a real driver on communication/hardware failure."""


class Pump:
    """PLACEHOLDER pump. Replace with a real driver when hardware is available."""

    def __init__(self, **kwargs):
        # A real driver would take connection info here (serial port, host:port, ...).
        self._initialized = False
        self._lock = threading.Lock()

    def initialize(self):
        print("PLACEHOLDER Pump: initialize() -- no hardware connected.")
        self._initialized = True
        return True, "pump initialized (placeholder)"

    def mix(self):
        """Mix the sample in the mixer."""
        return self._do("mix")

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

    def _do(self, op):
        with self._lock:
            print("PLACEHOLDER Pump: running %r ..." % op)
            time.sleep(PUMP_ACTION_DURATION_S)
            print("PLACEHOLDER Pump: %r complete." % op)
            return True, "%s complete (placeholder, no hardware)" % op
