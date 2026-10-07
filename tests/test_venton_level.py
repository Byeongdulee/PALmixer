"""The two-tag, level-assumption Venton fit (calibrate_venton_table_level).

The three-point position fit needs three non-collinear tags; this one trades the
third tag for two assumptions -- the table is level (normal = base +Z) and each
tag's +X runs along the table +X -- and so works with two. These pin the math:
the yaw comes from the tags' orientations, the origin from their positions, and a
wrong mounting convention shows up as a big residual. No robot and no camera: a
monkeypatched _read_tag_poses_in_view feeds scripted base poses.
"""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")
import numpy as np


def _tag_pose(R_table, origin_m, table_xy_mm):
    """A 4x4 base<-tag pose for a flat tag whose axes equal the table axes."""
    T = np.eye(4)
    T[:3, :3] = R_table
    T[:3, 3] = origin_m + R_table @ np.array(
        [table_xy_mm[0] / 1000.0, table_xy_mm[1] / 1000.0, 0.0])
    return T


@pytest.fixture
def two_tags(monkeypatch):
    tags = {12: {"name": "venton_table_b", "table_xy_mm": [233.5, -28.0], "size_m": 0.0315},
            13: {"name": "venton_table_c", "table_xy_mm": [29.0, -230.0], "size_m": 0.0315}}
    monkeypatch.setattr(PAL12idb, "venton_table_tags", lambda: dict(tags))
    return tags


def _level_R(yaw_deg):
    a = math.radians(yaw_deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_recovers_a_known_level_frame_from_two_tags(two_tags, monkeypatch):
    yaw, origin = 12.0, np.array([0.252, -0.143, 0.090])
    R = _level_R(yaw)
    poses = {tid: _tag_pose(R, origin, info["table_xy_mm"])
             for tid, info in two_tags.items()}
    monkeypatch.setattr(PAL12idb, "_read_tag_poses_in_view",
                        lambda robot, tags: (poses, np.eye(4)))
    fit = PAL12idb.calibrate_venton_table_level(object())
    assert fit["origin_m"] == pytest.approx(origin, abs=1e-6)
    assert fit["yaw_deg"] == pytest.approx(yaw, abs=1e-6)
    assert fit["normal"] == pytest.approx([0, 0, 1], abs=1e-9)   # level
    assert fit["rmse_mm"] == pytest.approx(0.0, abs=1e-6)
    # Positions imply the same yaw -> the mounting convention is consistent.
    assert fit["position_yaw_deg"] == pytest.approx(yaw, abs=1e-6)
    assert {t["id"] for t in fit["tags"]} == {12, 13}


def test_refuses_with_fewer_than_two_tags(two_tags, monkeypatch):
    R = _level_R(0.0)
    one = {12: _tag_pose(R, np.zeros(3), two_tags[12]["table_xy_mm"])}
    monkeypatch.setattr(PAL12idb, "_read_tag_poses_in_view",
                        lambda robot, tags: (one, np.eye(4)))
    with pytest.raises(RuntimeError, match="at least 2"):
        PAL12idb.calibrate_venton_table_level(object())


def test_unknown_tag_id_is_rejected(two_tags):
    with pytest.raises(RuntimeError, match="no configured Venton fiducial"):
        PAL12idb.calibrate_venton_table_level(object(), tag_ids=[12, 99])


def test_a_mounting_mismatch_shows_up_as_residual(two_tags, monkeypatch):
    # Tags physically yawed 90 deg from the table axes: the orientation fit then
    # disagrees with what the positions imply, and the position residual is large
    # -- which is exactly the signal that the tag-X == table-X assumption is off.
    R_tag = _level_R(90.0)
    origin = np.array([0.252, -0.143, 0.090])
    # positions follow the TRUE (level, zero-yaw) table; orientations are yawed.
    R_true = _level_R(0.0)
    poses = {}
    for tid, info in two_tags.items():
        T = _tag_pose(R_true, origin, info["table_xy_mm"])
        T[:3, :3] = R_tag                       # lie about the orientation only
        poses[tid] = T
    monkeypatch.setattr(PAL12idb, "_read_tag_poses_in_view",
                        lambda robot, tags: (poses, np.eye(4)))
    fit = PAL12idb.calibrate_venton_table_level(object())
    assert fit["yaw_deg"] == pytest.approx(90.0, abs=1e-6)
    assert fit["position_yaw_deg"] == pytest.approx(0.0, abs=1e-6)
    assert fit["rmse_mm"] > 50.0        # the convention error is loud, not silent


def test_positions_heading_is_immune_to_bad_tag_orientation(two_tags, monkeypatch):
    # Same garbage orientation as above, but heading='positions' ignores it and
    # recovers the true (level, zero-yaw) frame from the positions alone.
    origin = np.array([0.252, -0.143, 0.090])
    R_true, R_tag = _level_R(0.0), _level_R(90.0)
    poses = {}
    for tid, info in two_tags.items():
        T = _tag_pose(R_true, origin, info["table_xy_mm"])
        T[:3, :3] = R_tag
        poses[tid] = T
    monkeypatch.setattr(PAL12idb, "_read_tag_poses_in_view",
                        lambda robot, tags: (poses, np.eye(4)))
    fit = PAL12idb.calibrate_venton_table_level(object(), heading="positions")
    assert fit["method"] == "level+positions"
    assert fit["yaw_deg"] == pytest.approx(0.0, abs=1e-6)
    assert fit["origin_m"] == pytest.approx(origin, abs=1e-6)
    assert fit["rmse_mm"] == pytest.approx(0.0, abs=1e-6)


def test_an_unknown_heading_is_rejected(two_tags, monkeypatch):
    R = _level_R(0.0)
    poses = {tid: _tag_pose(R, np.zeros(3), info["table_xy_mm"])
             for tid, info in two_tags.items()}
    monkeypatch.setattr(PAL12idb, "_read_tag_poses_in_view",
                        lambda robot, tags: (poses, np.eye(4)))
    with pytest.raises(ValueError, match="heading must be"):
        PAL12idb.calibrate_venton_table_level(object(), heading="sideways")
