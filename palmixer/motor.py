# -*- coding: utf-8 -*-
"""EPICS motor control for the PALmixer forward/backward tweak.

Wraps the standard EPICS motor record tweak fields on a single motor PV
(default ``12idb:m6``):

    <pv>.TWV   tweak step value (engineering units)
    <pv>.TWF   tweak forward (put 1 to move +step)
    <pv>.TWR   tweak reverse (put 1 to move -step)
    <pv>.DMOV  done-moving flag (1 = motion complete)
    <pv>.RBV   readback value

caget() returns None if a PV is disconnected or the request times out; that
must never be treated as a valid position or done-flag, mirroring the guard
already used in PAL12idb.get_position().
"""

import time

FORWARD = "forward"
REVERSE = "reverse"


class MotorError(Exception):
    """Raised when a PV cannot be read/written or a move does not complete."""


class Motor:
    def __init__(self, pv="12idb:m6"):
        self.pv = pv

    def _caget(self, suffix):
        from epics import caget
        return caget("%s.%s" % (self.pv, suffix))

    def _caput(self, suffix, value):
        from epics import caput
        caput("%s.%s" % (self.pv, suffix), value)

    def read(self):
        """Return the current readback value (.RBV). Raises MotorError if disconnected."""
        rbv = self._caget("RBV")
        if rbv is None:
            raise MotorError("%s.RBV: no value read (PV disconnected?)" % self.pv)
        return rbv

    def wait_done(self, timeout=60.0, poll_interval=0.1):
        """Block until .DMOV reads 1, or raise MotorError after ``timeout`` seconds."""
        elapsed = 0.0
        while elapsed < timeout:
            dmov = self._caget("DMOV")
            if dmov is None:
                raise MotorError("%s.DMOV: no value read (PV disconnected?)" % self.pv)
            if int(dmov) == 1:
                return
            time.sleep(poll_interval)
            elapsed += poll_interval
        raise MotorError("%s: motion did not complete within %.1f s" % (self.pv, timeout))

    def tweak(self, direction, step, timeout=60.0):
        """Tweak the motor one step in ``direction`` ('forward' or 'reverse').

        Sets .TWV to ``step`` then pulses .TWF or .TWR, waits for .DMOV, and
        returns the new .RBV. Raises MotorError/ValueError on failure.
        """
        if direction not in (FORWARD, REVERSE):
            raise ValueError("direction must be %r or %r, got %r" % (FORWARD, REVERSE, direction))
        step = abs(float(step))
        self._caput("TWV", step)
        if direction == FORWARD:
            self._caput("TWF", 1)
        else:
            self._caput("TWR", 1)
        self.wait_done(timeout=timeout)
        return self.read()
