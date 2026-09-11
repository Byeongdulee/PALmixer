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
already used in PAL12idb.get_position(). caput() reports the same condition
the same way, and is checked for it -- pyepics only prints "cannot connect to
<pv>" and carries on otherwise, which would turn an unreachable IOC into a
tweak that looks accepted and then times out a minute later waiting on .DMOV.
"""

import time

from . import capath

FORWARD = "forward"
REVERSE = "reverse"


class MotorError(Exception):
    """Raised when a PV cannot be read/written or a move does not complete."""


class Motor:
    def __init__(self, pv="12idb:m6"):
        self.pv = pv

    def _caget(self, suffix):
        capath.ensure()
        from epics import caget
        return caget("%s.%s" % (self.pv, suffix))

    def _caput(self, suffix, value):
        capath.ensure()
        from epics import caput
        name = "%s.%s" % (self.pv, suffix)
        if caput(name, value) is None:
            raise MotorError(
                "%s: write failed (PV disconnected?). Check that the IOC for %s "
                "is running and reachable from this host." % (name, self.pv))

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

    def check_in_range(self, position):
        """Raise MotorError if `position` is outside the motor's soft limits.

        Worth doing before the write rather than after, because the motor
        record does not report the refusal in any way this class would
        otherwise notice: a .VAL outside .LLM/.HLM is simply not acted on, and
        .DMOV stays 1 throughout. The move then "succeeds" instantly with the
        motor exactly where it started -- which is how a carousel that never
        turned was reported as a completed step.

        Limits that cannot be read are not treated as a failure: an IOC that
        answers .VAL but not .LLM is not a case worth refusing a move over, and
        the .LVIO check after the write still catches the violation.
        """
        lo, hi = self._caget("LLM"), self._caget("HLM")
        if lo is None or hi is None or lo >= hi:
            return
        if not (lo <= float(position) <= hi):
            raise MotorError(
                "%s: %.4f is outside the soft limits [%.4f, %.4f] (%s). The motor "
                "record silently ignores a move past its limits, so this is "
                "refused here instead. Widen %s.LLM/.HLM, or fix the positions "
                "being asked for." % (self.pv, float(position), lo, hi,
                                      self._caget("EGU") or "engineering units",
                                      self.pv))

    def _check_no_limit_violation(self, position):
        """Backstop for a rejected move the range check did not predict --
        dial-vs-user limits, an offset changing underneath us. .LVIO is the
        motor record's own "the last move I was asked for violated a limit"."""
        if self._caget("LVIO"):
            raise MotorError(
                "%s: the move to %.4f violated a limit (%s.LVIO is set); the motor "
                "did not move." % (self.pv, float(position), self.pv))

    def move_to(self, position, timeout=120.0):
        """Move to an absolute position (.VAL), wait for .DMOV, return .RBV.

        Used for the carousel, which is taught as slot -> absolute position
        rather than moved with relative tweaks like the Experiment tab's
        forward/reverse buttons.

        Raises MotorError rather than returning if the target is out of range:
        see check_in_range for why that cannot be left to the motor record.
        """
        self.check_in_range(position)
        self._caput("VAL", float(position))
        self._check_no_limit_violation(position)
        self.wait_done(timeout=timeout)
        return self.read()
