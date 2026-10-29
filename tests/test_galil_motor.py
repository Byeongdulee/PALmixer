"""The Galil motor driver that replaced EPICS 12idb:m6 (carousel channel A).

GalilMotor is a drop-in for Motor -- read/tweak/move_to/check_in_range -- driving
a Galil controller channel over gclib. gclib and the controller are only present
on the beamline, so these inject a fake gclib and pin the command strings
(SH<channel> to turn on, SP/AC/DC to set the speed, PA/PR + BG to move, RP to
read, RP + TD to verify the arrival) and the config-driven selection between
the Galil and EPICS backends.
"""
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer import motor as motor_mod
from palmixer.motor import GalilMotor, Motor, MotorError


class FakeGalil:
    def __init__(self):
        self.opened = None
        self.commands = []
        self.motion_waits = []
        self.positions = [100, 0, 0, 0, 0, 0, 0, 0]   # A..H, RP
        self.steps = list(self.positions)              # A..H, TD
        self.motor_type = 2.0                          # a stepper, like the carousel
        self.stop_code = 1
        self.stop_short_at = None   # a limit/abort: RP and TD both end here
        self.steps_lost = 0         # pulses that never left the controller
        self._target = 0
        self._relative = False

    def GOpen(self, ip):
        self.opened = ip

    def GCommand(self, cmd):
        self.commands.append(cmd)
        if cmd == "RP":
            return ", ".join(str(p) for p in self.positions)
        if cmd == "TD":
            return ", ".join(str(p) for p in self.steps)
        if cmd.startswith("MG _MT"):
            return "%.4f" % self.motor_type
        if cmd.startswith("MG _SC"):
            return "%.4f" % self.stop_code
        if cmd.startswith("PAA="):
            self._target, self._relative = int(cmd.split("=")[1]), False
        elif cmd.startswith("PRA="):
            self._target, self._relative = int(cmd.split("=")[1]), True
        elif cmd.startswith("BGA"):
            end = self.positions[0] + self._target if self._relative else self._target
            if self.stop_short_at is not None:
                end = self.stop_short_at
            self.positions[0] = end
            self.steps[0] = end - self.steps_lost
        return ""

    def GMotionComplete(self, channel):
        self.motion_waits.append(channel)


@pytest.fixture
def fake_gclib(monkeypatch):
    fake = FakeGalil()
    mod = types.ModuleType("gclib")
    mod.py = lambda: fake
    monkeypatch.setitem(sys.modules, "gclib", mod)
    return fake


def test_connecting_opens_the_ip_and_turns_the_motor_on(fake_gclib):
    m = GalilMotor("10.54.122.161", "A")
    m.read()
    assert fake_gclib.opened == "10.54.122.161"
    assert "SHA;" in fake_gclib.commands          # the motor is enabled on connect


def test_read_returns_the_channel_count(fake_gclib):
    assert GalilMotor("x", "A").read() == 100


def test_read_picks_the_right_channel(fake_gclib):
    fake_gclib.positions = [1, 2, 3, 0, 0, 0, 0, 0]
    assert GalilMotor("x", "C").read() == 3


def test_move_to_is_absolute_then_begins_and_waits(fake_gclib):
    m = GalilMotor("x", "A")
    assert m.move_to(500) == 500
    assert "PAA=500" in fake_gclib.commands
    assert "BGA" in fake_gclib.commands
    assert fake_gclib.motion_waits == ["A"]


def test_tweak_forward_is_a_positive_relative_move(fake_gclib):
    m = GalilMotor("x", "A")
    assert m.tweak("forward", 25) == 125          # 100 + 25
    assert "PRA=25" in fake_gclib.commands


def test_tweak_reverse_is_a_negative_relative_move(fake_gclib):
    m = GalilMotor("x", "A")
    assert m.tweak("reverse", 25) == 75           # 100 - 25
    assert "PRA=-25" in fake_gclib.commands


def test_tweak_rejects_a_bad_direction(fake_gclib):
    with pytest.raises(ValueError):
        GalilMotor("x", "A").tweak("sideways", 1)


def test_check_in_range_is_a_noop(fake_gclib):
    GalilMotor("x", "A").check_in_range(999999)   # must not raise


# -- speed ----------------------------------------------------------------------
def test_move_to_sends_the_speed_and_accel_before_the_move(fake_gclib):
    GalilMotor("x", "A", speed=12500, accel=128000).move_to(500)
    cmds = fake_gclib.commands
    for profile in ("SPA=12500", "ACA=128000", "DCA=128000"):
        assert cmds.index(profile) < cmds.index("PAA=500") < cmds.index("BGA")


def test_tweak_sends_the_speed_too(fake_gclib):
    GalilMotor("x", "A", speed=12500).tweak("forward", 25)
    assert fake_gclib.commands.index("SPA=12500") < fake_gclib.commands.index("PRA=25")


def test_unset_speed_leaves_the_controller_alone(fake_gclib):
    GalilMotor("x", "A").move_to(500)
    assert not [c for c in fake_gclib.commands if c[:2] in ("SP", "AC", "DC")]


@pytest.mark.parametrize("bad", [0, -1, "-5"])
def test_a_non_positive_speed_is_refused(bad):
    with pytest.raises(ValueError, match="motor.speed"):
        GalilMotor("x", "A", speed=bad)


# -- arrival check --------------------------------------------------------------
@pytest.fixture
def no_settle(monkeypatch):
    monkeypatch.setattr(motor_mod, "_ARRIVAL_SETTLE_S", 0.0)


def test_move_to_checks_both_the_reference_and_the_step_count(fake_gclib):
    GalilMotor("x", "A").move_to(500)
    cmds = fake_gclib.commands
    assert cmds.index("BGA") < cmds.index("TD")


def test_a_move_stopped_short_raises_with_the_stop_code(fake_gclib, no_settle):
    fake_gclib.stop_short_at, fake_gclib.stop_code = 300, 2
    with pytest.raises(MotorError, match=r"did not reach 500: it ended at 300.*forward limit"):
        GalilMotor("x", "A").move_to(500)


def test_missing_step_pulses_raise_even_when_the_reference_arrived(fake_gclib, no_settle):
    fake_gclib.steps_lost = 40
    with pytest.raises(MotorError, match="step count 460"):
        GalilMotor("x", "A").move_to(500)


def test_a_tweak_stopped_short_raises(fake_gclib, no_settle):
    fake_gclib.stop_short_at, fake_gclib.stop_code = 110, 7
    with pytest.raises(MotorError, match=r"did not reach 125.*AB command"):
        GalilMotor("x", "A").tweak("forward", 25)


def test_a_servo_is_not_checked_against_its_encoder(fake_gclib):
    # On a servo TD reads an encoder, not a step count, so only RP is compared.
    fake_gclib.motor_type, fake_gclib.steps_lost = 1.0, 40
    assert GalilMotor("x", "A").move_to(500) == 500
    assert "TD" not in fake_gclib.commands


def test_a_missing_gclib_is_a_clear_motor_error(monkeypatch):
    monkeypatch.setitem(sys.modules, "gclib", None)   # import gclib -> ImportError
    with pytest.raises(MotorError, match="importing gclib failed"):
        GalilMotor("x", "A").read()


def test_a_failed_connection_is_a_clear_motor_error(monkeypatch):
    # The controller unreachable (GOpen raises) must surface as a MotorError, not
    # a raw gclib error escaping into a worker thread.
    broken = types.ModuleType("gclib")

    class _Py:
        def GOpen(self, ip):
            raise RuntimeError("TIMEOUT")
    broken.py = lambda: _Py()
    monkeypatch.setitem(sys.modules, "gclib", broken)
    with pytest.raises(MotorError, match="could not open the Galil controller"):
        GalilMotor("x", "A").read()


# -- config-driven selection ------------------------------------------------
def test_build_motor_picks_galil_by_default():
    from palmixer.server import _build_motor
    m = _build_motor({"type": "galil", "ip": "10.54.122.161", "channel": "A"})
    assert isinstance(m, GalilMotor) and m.ip == "10.54.122.161" and m.channel == "A"
    assert m.speed is None and m.accel is None


def test_build_motor_passes_the_speed_and_accel():
    from palmixer.server import _build_motor
    m = _build_motor({"type": "galil", "ip": "x", "speed": 12500, "accel": 128000})
    assert (m.speed, m.accel) == (12500, 128000)


def test_build_motor_can_still_pick_epics():
    from palmixer.server import _build_motor
    m = _build_motor({"type": "epics", "pv": "12idb:m6"})
    assert isinstance(m, Motor) and m.pv == "12idb:m6"


def test_build_motor_galil_requires_an_ip():
    from palmixer.server import _build_motor
    with pytest.raises(ValueError, match="motor.ip"):
        _build_motor({"type": "galil"})


def test_build_motor_rejects_an_unknown_type():
    from palmixer.server import _build_motor
    with pytest.raises(ValueError, match="galil.*epics|epics.*galil"):
        _build_motor({"type": "stepper"})
