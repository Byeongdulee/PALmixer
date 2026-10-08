"""Teaching the transfer point from the current pose.

The transfer point was a fixed module constant; it can now be taught and the
taught value lives in waypoints.ini. These pin the fallback-to-default, the
round-trip through the ini, that a teach reads the gripper-TCP pose, and the
server command's guards (busy refused, no-arg, simulate).
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
from palmixer import commands as cmd
from palmixer.server import PALmixerServer
PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")


@pytest.fixture
def ini(tmp_path, monkeypatch):
    # Point the module's waypoints.ini at a temp file so the test never touches
    # the real one.
    path = tmp_path / "waypoints.ini"
    monkeypatch.setattr(PAL12idb, "_INI_PATH", str(path))
    return path


class _Vec(list):
    def tolist(self):
        return list(self)


class _Pose:
    def __init__(self, v):
        self._v = v
    def get_pose_vector(self):
        return _Vec(self._v)


class FakeRobot:
    def __init__(self, pose):
        self.tcp = [0, 0, 0.195, 0, 0, 0]
        self._pose = pose
        self.tcp_sets = []
    def set_tcp(self, tcp):
        self.tcp_sets.append(list(tcp))
    def get_pose(self):
        return _Pose(self._pose)


def test_defaults_to_the_module_constant_when_untaught(ini):
    assert PAL12idb.get_transfer_point() == list(PAL12idb.transfer_point)


def test_teach_then_read_round_trips(ini):
    pose = [0.3, -0.2, 0.15, 2.1, -2.2, 0.0]
    stored = PAL12idb.record_current_transfer_point(FakeRobot(pose))
    assert stored == pose
    assert PAL12idb.get_transfer_point() == pytest.approx(pose)


def test_teach_reads_the_pose_with_the_gripper_tcp(ini):
    robot = FakeRobot([0.1, 0.2, 0.3, 0, 0, 0])
    PAL12idb.record_current_transfer_point(robot)
    assert robot.tcp_sets and robot.tcp_sets[-1] == robot.tcp


def test_a_malformed_entry_falls_back_to_default(ini):
    ini.write_text("[transfer_point]\nX = oops\n")
    assert PAL12idb.get_transfer_point() == list(PAL12idb.transfer_point)


def test_teaching_preserves_other_sections(ini):
    ini.write_text("[sample_table]\nX = 1.0\nY = 2.0\nZ = 3.0\n"
                   "Rx = 0.0\nRy = 0.0\nRz = 0.0\n")
    PAL12idb.record_current_transfer_point(FakeRobot([0.3, -0.2, 0.15, 0, 0, 0]))
    text = ini.read_text()
    assert "[sample_table]" in text and "[transfer_point]" in text


# -- server command ---------------------------------------------------------
def _server(**kw):
    s = object.__new__(PALmixerServer)
    s.simulate = kw.get("simulate", False)
    s.rob = kw.get("rob", object())
    s._busy = kw.get("busy", False)
    s._current_action = kw.get("action", None)
    s.PAL12idb = PAL12idb
    return s


def test_command_builder():
    assert cmd.set_transfer_point_here_command() == "set_transfer_point_here"


def test_takes_no_arguments():
    assert _server().fast_dispatch("set_transfer_point_here x").startswith(
        "ERROR: set_transfer_point_here takes no arguments")


def test_refused_while_busy():
    s = _server(busy=True, action="make_sample 3")
    assert "busy" in s.fast_dispatch("set_transfer_point_here")


def test_simulate_does_not_touch_the_robot():
    s = _server(simulate=True)
    assert s.fast_dispatch("set_transfer_point_here").startswith("OK")


def test_records_via_pal12idb(monkeypatch):
    calls = {}
    monkeypatch.setattr(PAL12idb, "record_current_transfer_point",
                        lambda rob: calls.setdefault("pose", [1, 2, 3, 0, 0, 0]))
    reply = _server().fast_dispatch("set_transfer_point_here")
    assert reply.startswith("OK taught transfer point")
    assert "pose" in calls
