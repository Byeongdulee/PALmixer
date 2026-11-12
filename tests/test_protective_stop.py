"""A protective stop at a transport's pickup or set-down: the safe stop, no search.

No robot -- a fake that is in protective stop until unlocked records what was
done to it, and pickup()/the set-down are stand-ins that raise the stop.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")


class MotionFault(Exception):
    pass


class StoppedRobot:
    """In protective stop (safety mode 3) until unlocked."""
    def __init__(self):
        self.mode = 3
        self.calls = []
    def get_safety_mode(self):
        return self.mode
    def unlock_protective_stop(self):
        self.calls.append("unlock")
        self.mode = 1
        return True
    def release_after_unlock(self):
        self.calls.append("open gripper")
    def mvr2z(self, dz, **k):
        self.calls.append(("lift", dz))


@pytest.fixture
def stopped(monkeypatch):
    monkeypatch.setattr(PAL12idb, "_MOTION_FAULTS", (MotionFault,))
    monkeypatch.setattr(PAL12idb, "locate_apriltag",
                        lambda *a, **k: pytest.fail("a protective stop must not search"))
    return StoppedRobot()


def _stop(*a, **k):
    raise MotionFault("Protective stopped")


def test_a_stop_while_picking_up_stops_safely_without_searching(stopped, monkeypatch):
    monkeypatch.setattr(PAL12idb, "pickup", _stop)
    with pytest.raises(PAL12idb.ProtectiveStopUnrecovered, match="while picking up"):
        PAL12idb._pickup_recovering(stopped, "cleaning_station", height=0.1)
    assert stopped.calls == ["unlock", "open gripper", ("lift", PAL12idb.RECOVERY_ABORT_LIFT)]


def test_a_stop_while_setting_down_stops_safely_without_searching(stopped):
    with pytest.raises(PAL12idb.ProtectiveStopUnrecovered, match="while setting down"):
        PAL12idb._dropdown_recovering(stopped, "cleaning_station", [0.4, -0.1, -0.2, 0, 3.14, 0],
                                      place=_stop)
    assert stopped.calls == ["unlock", "open gripper", ("lift", PAL12idb.RECOVERY_ABORT_LIFT)]


def test_the_stop_dialog_names_the_station(stopped, monkeypatch):
    monkeypatch.setattr(PAL12idb, "pickup", _stop)
    with pytest.raises(PAL12idb.ProtectiveStopUnrecovered) as raised:
        PAL12idb._pickup_recovering(stopped, "cleaning_station", height=0.1)
    message = str(raised.value)
    assert message.startswith(PAL12idb._commands.PROTECTIVE_STOP_UNRECOVERED)
    assert PAL12idb._commands.STATION_LABELS["cleaning_station"] in message


def test_other_failures_and_unnamed_seats_propagate_untouched(stopped, monkeypatch):
    def unreachable(*a, **k):
        raise ValueError("pose unreachable")
    monkeypatch.setattr(PAL12idb, "pickup", unreachable)
    with pytest.raises(ValueError):
        PAL12idb._pickup_recovering(stopped, "cleaning_station", height=0.1)
    monkeypatch.setattr(PAL12idb, "pickup", _stop)
    with pytest.raises(MotionFault):
        PAL12idb._pickup_recovering(stopped, None, height=0.1)
    assert stopped.calls == []                      # nothing was done to the robot
