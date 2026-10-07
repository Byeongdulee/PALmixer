"""Searching for an AprilTag from where the arm already stands.

The Configuration tab's "Start from the current position" checkbox. For a
station that has moved far enough that its configured search reference no
longer looks anywhere near the tag: put the camera over it by eye and search
from there.

The point of the mode is that the arm does not move before the tilt grid
starts -- so these assert what step 1 of camera_tools.search_apriltag_by_tilt
does *not* do. put_camera2tcp() is the one to watch: it translates the arm by
the camera offset rather than reorienting it, so on a camera that already
looks down it would drag the view off the tag.
"""
from pathlib import Path
import math
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))

camera_tools = pytest.importorskip("camera_tools", reason="UR_12idb not importable")
m3d = pytest.importorskip("math3d")

from palmixer import commands as cmd

REF = [-0.22, -0.37, 0.12, -2.18860535, 2.25379435, 0.0]
FACE_DOWN = m3d.Transform([0.30, -0.15, 0.10, math.pi, 0.0, 0.0])
TILTED = m3d.Transform([0.30, -0.15, 0.10, 2.4, 0.9, 0.0])


class FakeRobot:
    """Records which step-1 moves were asked for. get_pose returns the same
    pose for either TCP; only its Z axis direction matters here."""

    tcp = [0.0, 0.0, 0.195, 0.0, 0.0, 0.0]
    camtcp = [0.0, 0.0433, 0.06, -math.pi / 6, 0.0, 0.0]

    def __init__(self, pose):
        self._pose = pose
        self._active = self.tcp
        self.calls = []
        self.camera = type("Cam", (), {"AT_physical_size": 0.012})()

    def get_tcp(self):
        return self._active

    def set_tcp(self, tcp):
        self._active = tcp

    def get_pose(self):
        return self._pose.copy()

    def Zalign(self):
        self.calls.append("Zalign")

    def moveto(self, target, **kw):
        self.calls.append("moveto")

    def put_camera2tcp(self):
        self.calls.append("put_camera2tcp")

    def set_pose(self, pose, **kw):
        self.calls.append("set_pose")


@pytest.fixture
def no_tag(monkeypatch):
    """Make detection always fail, so the search returns right after the tilt
    grid -- these tests are about step 1, not about finding anything."""
    monkeypatch.setattr(camera_tools, "_detect_apriltag", lambda *a, **k: None)


def _run(robot, **kw):
    return camera_tools.search_apriltag_by_tilt(robot, ref_pos=list(REF), **kw)


def test_the_normal_path_still_drives_to_the_reference(no_tag):
    """The default must be untouched: level, drive there, square the camera."""
    robot = FakeRobot(FACE_DOWN)
    _run(robot)
    assert robot.calls[:3] == ["Zalign", "moveto", "put_camera2tcp"]


def test_from_current_does_not_move_a_camera_that_already_faces_down(no_tag):
    """The whole point. No reference move, no levelling, and -- critically --
    no put_camera2tcp, which would shift the arm ~4 cm off the tag."""
    robot = FakeRobot(FACE_DOWN)
    _run(robot, from_current=True)
    assert "moveto" not in robot.calls
    assert "Zalign" not in robot.calls
    assert "put_camera2tcp" not in robot.calls


def test_from_current_squares_a_camera_that_does_not_face_down(no_tag):
    """The tilt grid sweeps only +/-10 degrees, so it cannot recover a wrist
    left at an arbitrary angle -- that case still gets brought face-down."""
    robot = FakeRobot(TILTED)
    _run(robot, from_current=True)
    assert "put_camera2tcp" in robot.calls
    assert "moveto" not in robot.calls, "it drove to the reference anyway"


def test_from_current_still_runs_the_tilt_grid(no_tag):
    """Skipping the approach must not skip the search itself."""
    robot = FakeRobot(FACE_DOWN)
    assert _run(robot, from_current=True) is False      # no tag, by the fixture
    assert robot.calls.count("set_pose") > 1, "the tilt grid did not run"


def test_a_camera_pointing_up_does_not_count_as_facing_down():
    """robUR.is_Z_aligned() takes an absolute value, so it calls straight-up
    aligned too. Face-down is a direction, not an axis."""
    assert camera_tools.camera_faces_down(FakeRobot(FACE_DOWN))
    up = m3d.Transform([0.30, -0.15, 0.10, 0.0, 0.0, 0.0])
    assert not camera_tools.camera_faces_down(FakeRobot(up))


def test_asking_leaves_the_active_tcp_as_it_found_it():
    """Callers read poses around this; a camera TCP left live would silently
    reinterpret the next one."""
    robot = FakeRobot(FACE_DOWN)
    robot.set_tcp(FakeRobot.tcp)
    camera_tools.camera_faces_down(robot)
    assert robot.get_tcp() is FakeRobot.tcp


# -- wire format -----------------------------------------------------------

def test_the_two_search_flags_are_independent_and_order_free():
    build = cmd.search_apriltag_command
    assert build("sample_table") == "search_apriltag sample_table"
    assert build("sample_table", from_current=True) == \
        "search_apriltag sample_table from_current"
    assert build("sample_table", skip_roll=True, from_current=True) == \
        "search_apriltag sample_table skip_roll from_current"


def test_the_tool_frame_swaps_take_no_arguments():
    assert cmd.put_camera_to_tcp_command() == "put_camera_to_tcp"
    assert cmd.put_tcp_to_camera_command() == "put_tcp_to_camera"
