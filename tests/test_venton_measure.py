"""The closed-loop per-tag measurement and the calibration loop around it.

No robot: a fake stands in for the centre/descend/project primitives. The point
here is the control flow -- centre, descend, re-centre, project; skip a tag that
cannot be acquired; average repeats; leave the arm at the overview pose -- not
the camera geometry (that lives in the robUR primitives these call).
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")


class FakeCamera:
    def __init__(self):
        self.AT_physical_size = None
        self.AT_preferred_id = None
        self.QRdistance = 0.2


class FakeRobot:
    """Centre/descend/project return scripted base positions per tag. `fail`
    is a set of tag ids whose centring fails."""
    def __init__(self, tag_base, fail=()):
        self.tag_base = tag_base          # {id: [x,y,z]}
        self.fail = set(fail)
        self.camera = FakeCamera()
        self.tcp = [0, 0, 0.195, 0, 0, 0]
        self._current = None
        self.set_pose_calls = 0
        self.home = object()
    def set_tcp(self, tcp): pass
    def get_pose(self): return self.home
    def set_pose(self, p, **k): self.set_pose_calls += 1
    def center_camera2apriltag(self, tag_id=None, **k):
        self._current = tag_id
        return tag_id not in self.fail
    def get_obj_position_from_camera_center(self, dist):
        base = self.tag_base[self._current]
        return (list(base), None)


@pytest.fixture
def no_descend(monkeypatch):
    """descend_to_apriltag needs the real camera machinery; stub it to a
    successful no-op so these tests exercise the loop, not the descent."""
    monkeypatch.setattr(PAL12idb, "descend_to_apriltag", lambda robot, **k: 0.2)


@pytest.fixture
def three_tags(monkeypatch):
    tags = {11: {"name": "venton_table_a", "table_xy_mm": [544.2, -28.0], "size_m": 0.0315},
            12: {"name": "venton_table_b", "table_xy_mm": [233.5, -28.0], "size_m": 0.0315},
            13: {"name": "venton_table_c", "table_xy_mm": [29.0, -230.0], "size_m": 0.0315}}
    monkeypatch.setattr(PAL12idb, "venton_table_tags", lambda: tags)
    return tags


def test_measure_returns_none_when_centering_fails(no_descend):
    robot = FakeRobot({7: [0.1, 0.2, 0.3]}, fail={7})
    assert PAL12idb._measure_tag_in_base(robot, 7, 0.0315) is None


def test_measure_projects_after_centering(no_descend):
    robot = FakeRobot({7: [0.11, 0.22, 0.33]})
    assert PAL12idb._measure_tag_in_base(robot, 7, 0.0315) == [0.11, 0.22, 0.33]


class MoveRobot:
    """A robot for the multi-pose open-loop calibration: records moves and
    returns a scripted open-loop view (via monkeypatched _read_tags_in_view).
    Only the motion API is needed here -- the camera read is stubbed."""
    class _Pose:
        def __init__(self, xyz): self.pos = list(xyz)
    def __init__(self, home=(0.3, -0.2, 0.5)):
        self.tcp = [0, 0, 0.195, 0, 0, 0]
        self.camtcp = [0, 0.0433, 0.06, -0.5236, 0, 0]
        self._home = self._Pose(home)
        self.moves = []
        self.set_pose_calls = 0
    def set_tcp(self, tcp): pass
    def get_pose(self): return self._home
    def set_pose(self, p, **k): self.set_pose_calls += 1
    def moveto(self, target, **k): self.moves.append(list(target))


def test_calibration_fits_a_clean_frame(three_tags, monkeypatch):
    # Every pose sees the same scripted base positions (a known transform of
    # the table coords), so the fit should be near-perfect and spread ~0.
    import math, numpy as np
    ang = math.radians(8.0)
    R = np.array([[math.cos(ang), -math.sin(ang), 0],
                  [math.sin(ang), math.cos(ang), 0], [0, 0, 1]])
    t = np.array([0.3, -0.2, 0.15])
    view = {tid: list((R @ (np.array(info["table_xy_mm"] + [0.0]) / 1000.0)) + t)
            for tid, info in three_tags.items()}
    monkeypatch.setattr(PAL12idb, "_read_tags_in_view", lambda robot, tags: dict(view))
    robot = MoveRobot()
    fit = PAL12idb.calibrate_venton_table(robot)
    assert fit["rmse_mm"] < 0.5
    assert len(fit["tags"]) == 3
    assert all(tg["spread_mm"] == pytest.approx(0.0) for tg in fit["tags"])
    assert all(tg["passes"] == len(PAL12idb.VENTON_CALIB_POSES) for tg in fit["tags"])
    assert fit["origin_m"] == pytest.approx(t, abs=1e-6)


def test_only_small_safe_xyz_moves_are_commanded(three_tags, monkeypatch):
    """Small single-axis steps, and never a downward Z step (that drove the
    camera into the table and protective-stopped). Each move is one axis, the
    step is small, and any Z move is upward."""
    monkeypatch.setattr(PAL12idb, "_read_tags_in_view",
                        lambda robot, tags: {t: [0.1, 0.1, 0.1] for t in tags})
    home = (0.3, -0.2, 0.5)
    robot = MoveRobot(home=home)
    PAL12idb.calibrate_venton_table(robot)
    for target in robot.moves:
        off = [target[i] - home[i] for i in range(3)]
        mag = [abs(o) for o in off]
        assert max(mag) <= 0.05 + 1e-9, "a move exceeded the small-step cap"
        assert sum(1 for o in mag if o > 1e-9) == 1, "a move mixed axes"
        assert off[2] >= -1e-9, "a downward Z step was commanded (collision risk)"


def test_a_tag_never_seen_fails_the_run(three_tags, monkeypatch):
    # tag 13 never appears in any view.
    monkeypatch.setattr(PAL12idb, "_read_tags_in_view",
                        lambda robot, tags: {11: [0.5, 0, 0.1], 12: [0.2, 0, 0.1]})
    with pytest.raises(RuntimeError, match="saw only 2 of 3"):
        PAL12idb.calibrate_venton_table(MoveRobot())


def test_the_arm_is_returned_to_the_start_at_the_end(three_tags, monkeypatch):
    monkeypatch.setattr(PAL12idb, "_read_tags_in_view",
                        lambda robot, tags: {t: [0.1, 0.1, 0.1] for t in tags})
    robot = MoveRobot()
    PAL12idb.calibrate_venton_table(robot)
    assert robot.set_pose_calls >= 1       # final return-home always runs
