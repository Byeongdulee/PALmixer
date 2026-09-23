"""Jogging the arm by hand from the Configuration tab.

The position counterpart of tweak_orientation, and the missing half of "Set
Current Robot Position As": that records wherever the arm stands, so there has
to be a way to move it a millimetre without the teach pendant.

The frame matters and is the opposite of the rotation jog's. Position is
nudged in the **base** frame, because base X/Y/Z is what lands in
waypoints.ini; orientation is rotated in the **tool** frame, because there the
point is to swing the wrist without moving the tip.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
PAL12idb = pytest.importorskip("palmixer.PAL12idb",
                               reason="UR_12idb (camera_tools) not importable")
from palmixer import commands as cmd


START = [0.30, -0.15, 0.10, 2.231, -2.212, 0.0]


class FakeRobot:
    tcp = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]

    def __init__(self):
        self.pose = list(START)
        self.moves = []
        self.tcp_set = []

    def set_tcp(self, tcp):
        self.tcp_set.append(list(tcp))

    def get_pos(self):
        return list(self.pose)

    def moveto(self, target, acc=None, vel=None):
        self.moves.append((list(target), acc, vel))
        self.pose = list(target)


@pytest.mark.parametrize("axis,index", [("x", 0), ("y", 1), ("z", 2)])
def test_each_axis_moves_only_itself(axis, index):
    robot = FakeRobot()
    PAL12idb.tweak_position(robot, axis, 0.001)
    target, _acc, _vel = robot.moves[0]
    assert target[index] == pytest.approx(START[index] + 0.001)
    for other in range(6):
        if other != index:
            assert target[other] == pytest.approx(START[other]), (
                "jogging %s changed component %d" % (axis, other))


def test_orientation_is_untouched():
    """The whole point next to "Set Current Robot Position As": a taught
    orientation must survive a position correction."""
    robot = FakeRobot()
    PAL12idb.tweak_position(robot, "z", -0.002)
    target, _acc, _vel = robot.moves[0]
    assert target[3:] == pytest.approx(START[3:])


def test_the_step_is_exactly_what_gets_added():
    """The number in the spin box is the number the saved position changes by,
    so an operator can count clicks to a known offset."""
    robot = FakeRobot()
    for _ in range(4):
        PAL12idb.tweak_position(robot, "x", 0.0005)
    assert robot.get_pos()[0] == pytest.approx(START[0] + 0.002)


def test_the_gripper_tcp_is_asserted_first():
    """An aborted AprilTag search can leave the camera TCP live. Jogging in
    that frame would move the camera, not the gripper tip the taught positions
    are expressed in."""
    robot = FakeRobot()
    PAL12idb.tweak_position(robot, "x", 0.001)
    assert robot.tcp_set == [FakeRobot.tcp]


def test_it_runs_at_placement_speed_not_full_speed():
    """Close-quarters work next to a seat."""
    robot = FakeRobot()
    PAL12idb.tweak_position(robot, "y", 0.001)
    _target, acc, vel = robot.moves[0]
    assert (acc, vel) == (PAL12idb.placement_accel, PAL12idb.placement_speed)


@pytest.mark.parametrize("bad", [0.06, -0.06, 1.0, float("nan")])
def test_an_oversized_step_is_refused_without_moving(bad):
    """A jog is a nudge by eye with the arm millimetres from something it must
    not hit; a big number is a typo or a wrong unit, not an instruction."""
    robot = FakeRobot()
    with pytest.raises(ValueError):
        PAL12idb.tweak_position(robot, "x", bad)
    assert robot.moves == [], "the arm moved before the limit was checked"


def test_the_limit_boundary_itself_is_allowed():
    robot = FakeRobot()
    PAL12idb.tweak_position(robot, "x", PAL12idb.MAX_POSITION_TWEAK)
    assert robot.moves


def test_an_unknown_axis_is_refused():
    robot = FakeRobot()
    with pytest.raises(ValueError):
        PAL12idb.tweak_position(robot, "w", 0.001)
    assert robot.moves == []


def test_the_wire_command_carries_millimetres():
    """Millimetres on the wire and in the GUI, metres at the robot -- the one
    conversion lives in server.py."""
    assert cmd.tweak_position_command("x", -1.5) == "tweak_position x -1.5"
    assert cmd.POSITION_AXES == ("x", "y", "z")


def test_position_and_orientation_jogs_use_opposite_frames():
    """Pin the asymmetry, since it looks like an inconsistency until you know
    why: base for position, tool for rotation."""
    import inspect
    position = inspect.getsource(PAL12idb.tweak_position)
    orientation = inspect.getsource(PAL12idb.tweak_orientation)
    assert "coordinate='tcp'" in orientation
    assert "coordinate='tcp'" not in position
    assert "robot.get_pos()" in position      # base-frame read/add/moveto
