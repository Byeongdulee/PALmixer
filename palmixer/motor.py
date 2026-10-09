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

import threading
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


class GalilMotor:
    """A Galil controller channel, exposing the same interface as :class:`Motor`.

    12idb:m6 was replaced by channel A of a Galil controller, driven over gclib
    (see ptychoSAXS/ptychosaxs/tw_galil.py). This is a drop-in for Motor: the
    server and workflows call read()/tweak()/move_to()/check_in_range() without
    knowing which one they hold.

    Positions and steps are Galil step counts (integers), not EPICS engineering
    units -- so the carousel reference and slot step have to be (re)taught in
    those counts against this motor. The connection is opened lazily and the
    motor turned on (SH<channel>) the first time it is used; gclib is imported
    there too, so this module stays importable on a host without it.
    """

    def __init__(self, ip, channel="A"):
        self.ip = ip
        self.channel = str(channel).strip().upper()
        self._g = None
        self._lock = threading.RLock()

    # -- connection ---------------------------------------------------------
    def _conn(self):
        """Open the controller once and enable the servo on this channel.

        gclib and the controller are only reachable where this actually runs,
        so both the import and the connect happen here rather than at
        construction -- a GUI-only or test host can hold a GalilMotor without a
        Galil present, exactly as Motor holds a PV without an IOC.
        """
        with self._lock:
            if self._g is not None:
                return self._g
            try:
                import gclib
            except Exception as e:
                # Not just ImportError: on Windows the pure-Python gclib wrapper
                # loads gclib.dll at import, so a missing native runtime raises a
                # TypeError/OSError here, not an ImportError. Either way the motor
                # cannot be driven, and the operator needs the same two things.
                raise MotorError(
                    "the Galil motor at %s cannot be driven -- importing gclib "
                    "failed (%s). Install the gclib Python package AND Galil's "
                    "native gclib runtime (gclib.dll must be found by the system)."
                    % (self.ip, e))
            try:
                g = gclib.py()
                g.GOpen(self.ip)
                g.GCommand("SH%s;" % self.channel)   # turn the motor on (servo here)
            except Exception as e:                    # gclib.GclibError and friends
                raise MotorError("could not open the Galil controller at %s "
                                 "(channel %s): %s" % (self.ip, self.channel, e))
            self._g = g
            return g

    def _command(self, cmd):
        g = self._conn()
        try:
            with self._lock:
                return g.GCommand(cmd)
        except Exception as e:
            raise MotorError("Galil command %r failed: %s" % (cmd, e))

    # -- the Motor interface ------------------------------------------------
    def read(self):
        """The channel's current reference position (RP), as an integer count."""
        raw = self._command("RP")          # all channels, CSV, like tw_galil
        parts = [p for p in str(raw).replace(":", "").split(",") if p.strip()]
        idx = ord(self.channel) - ord("A")
        try:
            return int(float(parts[idx]))
        except (IndexError, ValueError) as e:
            raise MotorError("could not read channel %s position from %r: %s"
                             % (self.channel, raw, e))

    def wait_done(self, timeout=120.0, poll_interval=0.1):
        """Block until this channel's move completes.

        gclib's GMotionComplete blocks on the controller itself; timeout is kept
        for interface parity with Motor (its own GOpen timeout bounds the wait).
        """
        g = self._conn()
        try:
            g.GMotionComplete(self.channel)
        except Exception as e:
            raise MotorError("waiting for Galil channel %s to finish moving "
                             "failed: %s" % (self.channel, e))

    def check_in_range(self, position):
        """No soft-limit pre-check for the Galil: unlike the EPICS motor record,
        a Galil move past a configured limit is reported (it errors or stops at
        the limit) rather than silently ignored, so there is no "looks done but
        never moved" failure to guard against here. Kept for interface parity."""
        return

    def tweak(self, direction, step, timeout=60.0):
        """Move one relative step (PR + BG) in ``direction``, wait, return the
        new position. ``step`` is a Galil step count."""
        if direction not in (FORWARD, REVERSE):
            raise ValueError("direction must be %r or %r, got %r"
                             % (FORWARD, REVERSE, direction))
        counts = abs(int(round(float(step))))
        if direction == REVERSE:
            counts = -counts
        self._command("PR%s=%i" % (self.channel, counts))
        self._command("BG%s" % self.channel)
        self.wait_done(timeout=timeout)
        return self.read()

    def move_to(self, position, timeout=120.0):
        """Move to an absolute step count (PA + BG), wait, return the new
        position. The taught carousel positions are read()/move_to() in the same
        counts, so they stay consistent."""
        self.check_in_range(position)
        self._command("PA%s=%i" % (self.channel, int(round(float(position)))))
        self._command("BG%s" % self.channel)
        self.wait_done(timeout=timeout)
        return self.read()
