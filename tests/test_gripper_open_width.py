"""The fingers open wider than release() before dropping onto the flowcell
cleaning station, so they clear a flowcell standing in a deep seat.

The Robotiq Hand-E takes a 0-255 position count (0 open, 255 closed) and
reports nothing back, so the widening is computed from the opening
robUR.release() commands rather than measured. These tests pin the arithmetic
and the call sites; nothing here talks to a robot.
"""
import ast
import inspect
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# PAL12idb imports camera_tools from the sibling robot repo.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
PAL12idb = pytest.importorskip("palmixer.PAL12idb",
                               reason="UR_12idb (camera_tools) not importable")

STROKE_M = 0.05          # Hand-E span
RELEASE_COUNT = 120      # what robUR.release() commands


def opening_mm(count):
    """Finger gap at a given count, for assertions written in millimetres."""
    return STROKE_M * (255 - count) / 255 * 1000


class FakeGripper:
    def __init__(self):
        self.counts = []

    def gripper_action(self, value):
        self.counts.append(value)


class FakeRobot:
    """Records gripper commands and relative Z moves in the order they happen."""

    def __init__(self, with_gripper=True):
        self.gripper = FakeGripper() if with_gripper else None
        self.events = []

    def release(self):
        self.events.append(("release", None))

    def grab(self):
        self.events.append(("grab", None))

    def mvr2z(self, dz):
        self.events.append(("mvr2z", dz))


def _robot(with_gripper=True):
    robot = FakeRobot(with_gripper)
    if with_gripper:
        original = robot.gripper.gripper_action

        def record(value):
            robot.events.append(("gripper_action", value))
            original(value)
        robot.gripper.gripper_action = record
    else:
        del robot.gripper
    return robot


def test_two_centimetres_wider_is_exactly_two_centimetres():
    count = PAL12idb.pickup_open_count(0.02)
    widened = opening_mm(count) - opening_mm(RELEASE_COUNT)
    assert widened == pytest.approx(20.0, abs=0.2), (
        "asked for 20 mm wider, got %.1f mm" % widened)


def test_the_configured_default_is_the_two_centimetres_asked_for():
    assert PAL12idb.cleaning_station_open_extra() == pytest.approx(0.02)
    assert PAL12idb.pickup_open_count() == PAL12idb.pickup_open_count(0.02)


def test_no_widening_means_the_original_release():
    """Zero must not be "open to count 120 by hand" -- it must be the plain
    release() call, so every non-cleaning-station pickup is untouched."""
    assert PAL12idb.pickup_open_count(0.0) is None
    robot = _robot()
    PAL12idb.pickup(robot, height=0.01)
    assert ("release", None) in robot.events
    assert not robot.gripper.counts, "a raw count was commanded for a plain pickup"


def test_the_wider_opening_happens_before_the_descent():
    """Opening after starting down would be too late -- the fingers are
    already alongside the flowcell by then."""
    robot = _robot()
    PAL12idb.pickup(robot, height=0.01, open_extra_m=0.02)
    kinds = [k for k, _ in robot.events]
    assert kinds == ["gripper_action", "mvr2z", "grab", "mvr2z"]
    assert ("release", None) not in robot.events
    assert robot.gripper.counts == [PAL12idb.pickup_open_count(0.02)]


def test_more_travel_than_the_gripper_has_clamps_to_fully_open():
    """A bad number should not refuse to pick the flowcell up."""
    assert PAL12idb.pickup_open_count(0.5) == PAL12idb.GRIPPER_OPEN_COUNT


def test_a_robot_with_no_raw_count_api_still_opens_its_gripper():
    """Simulated or older robot objects have release() and nothing finer."""
    robot = _robot(with_gripper=False)
    assert PAL12idb.open_gripper(robot, 0.02) is None
    assert ("release", None) in robot.events


# -- call sites -------------------------------------------------------------
# Structural rather than behavioural: the transports are @_tracks-decorated and
# write the real state ini, so running them in a test would scribble on the
# operator's tracked locations. What matters is the invariant -- a pickup that
# follows an approach to the cleaning station asks for the wider opening.

CLEANING_STATION_PICKUPS = ["ready_flowcell_to_draw", "flowcell_to_sample_on_mixer"]


def _pickup_kwargs(func):
    tree = ast.parse(inspect.getsource(func).lstrip())
    return [{kw.arg for kw in node.keywords}
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and getattr(node.func, "id", None) == "pickup"]


@pytest.mark.parametrize("name", CLEANING_STATION_PICKUPS)
def test_cleaning_station_transports_ask_for_the_wider_opening(name):
    calls = _pickup_kwargs(getattr(PAL12idb, name))
    assert calls, "%s no longer calls pickup()" % name
    assert all("open_extra_m" in kwargs for kwargs in calls), (
        "%s picks the flowcell up off the cleaning station without widening" % name)


def test_the_sample_table_pickup_is_left_alone():
    """return_sample collects the flowcell from the sample table, where there
    is no deep seat and the original opening is the tested one."""
    calls = _pickup_kwargs(PAL12idb.return_sample)
    assert calls and all("open_extra_m" not in kwargs for kwargs in calls)


def test_transport2sampletable_widens_only_when_coming_from_the_station():
    """pickup_from_above is set only for a cleaning station, so it doubles as
    the discriminator. Verify both branches really differ."""
    source = inspect.getsource(PAL12idb.transport2sampletable)
    assert "open_extra_m=cleaning_station_open_extra() if pickup_from_above else 0.0" \
        in " ".join(source.split())
