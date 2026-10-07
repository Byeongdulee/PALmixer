"""The Venton-table frame solver -- pure math, no robot.

Given points known in the table's surface frame and the same points measured in
the robot base frame, recover the transform between them. These tests build
synthetic measurements by applying a known transform, then check the solver
gets it back -- including with a deliberate scale error and noise, since that
is what a real camera delivers.
"""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer.calibration import fit_transform, table_frame_in_base, CalibrationError


def rot_z(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# The real recorded table fiducials (mm), with z=0 on the surface.
TABLE_MM = np.array([[544.2, -28.0, 0.0],
                     [233.5, -28.0, 0.0],
                     [29.0, -230.0, 0.0]])


def test_it_recovers_a_known_rigid_transform():
    R = rot_z(37.0)
    t = np.array([0.31, -0.42, 0.11])
    src = TABLE_MM / 1000.0
    tgt = (R @ src.T).T + t
    fit = fit_transform(src, tgt)
    assert fit["rmse"] < 1e-9
    assert np.allclose(fit["R"], R, atol=1e-9)
    assert np.allclose(fit["t"], t, atol=1e-9)
    assert fit["scale"] == pytest.approx(1.0, abs=1e-9)


def test_axes_are_orthonormal_and_right_handed():
    R = rot_z(20.0) @ rot_z(0.0)
    src = TABLE_MM / 1000.0
    tgt = (R @ src.T).T + np.array([0.1, 0.2, 0.3])
    f = table_frame_in_base(TABLE_MM, tgt)
    x, y, n = f["x_axis"], f["y_axis"], f["normal"]
    assert np.allclose([np.dot(x, y), np.dot(x, n), np.dot(y, n)], 0, atol=1e-9)
    assert np.allclose([np.linalg.norm(v) for v in (x, y, n)], 1, atol=1e-9)
    assert np.allclose(np.cross(x, y), n, atol=1e-9)        # right-handed


def test_scale_is_recovered_and_flagged():
    """A camera reading 3% long makes the measured triangle 3% larger. The
    similarity fit reports scale 1.03 instead of hiding it as a bias."""
    R = rot_z(-15.0)
    src = TABLE_MM / 1000.0
    tgt = (1.03 * (R @ src.T)).T + np.array([0.2, 0.1, 0.05])
    with_scale = fit_transform(src, tgt, with_scale=True)
    assert with_scale["scale"] == pytest.approx(1.03, abs=1e-6)
    assert with_scale["rmse"] < 1e-9
    # A rigid fit cannot absorb it, so it surfaces as residual instead.
    rigid = fit_transform(src, tgt, with_scale=False)
    assert rigid["scale"] == 1.0
    assert rigid["rmse"] > 1e-3


def test_origin_maps_to_the_translation():
    R = rot_z(55.0)
    t = np.array([0.5, -0.3, 0.2])
    tgt = (R @ (TABLE_MM / 1000.0).T).T + t
    f = table_frame_in_base(TABLE_MM, tgt)
    assert np.allclose(f["origin_m"], t, atol=1e-9)


def test_measurement_noise_shows_up_as_rmse_not_a_crash():
    rng = np.random.default_rng(0)
    R = rot_z(10.0)
    tgt = (R @ (TABLE_MM / 1000.0).T).T + np.array([0.1, 0.1, 0.1])
    tgt_noisy = tgt + rng.normal(scale=0.0005, size=tgt.shape)   # 0.5 mm
    f = table_frame_in_base(TABLE_MM, tgt_noisy)
    assert 0.0 < f["rmse_mm"] < 3.0                              # sub-mm noise -> small rmse
    assert f["max_error_mm"] >= f["rmse_mm"]


def test_extra_correspondences_are_accepted():
    R = rot_z(5.0)
    src = np.vstack([TABLE_MM, [[400.0, -130.0, 0.0]]]) / 1000.0
    tgt = (R @ src.T).T + np.array([0.0, 0.0, 0.4])
    fit = fit_transform(src, tgt)
    assert fit["residuals"].shape == (4, 3)
    assert fit["rmse"] < 1e-9


def test_collinear_points_are_refused():
    line = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [250.0, 0.0, 0.0]]) / 1000.0
    with pytest.raises(CalibrationError):
        fit_transform(line, line + np.array([1.0, 0.0, 0.0]))


def test_too_few_points_are_refused():
    with pytest.raises(CalibrationError):
        fit_transform(TABLE_MM[:2] / 1000.0, TABLE_MM[:2] / 1000.0)


def test_mismatched_shapes_are_refused():
    with pytest.raises(CalibrationError):
        fit_transform(TABLE_MM / 1000.0, TABLE_MM[:, :2] / 1000.0)


def test_a_reflection_is_never_returned():
    """A mirrored target must still yield a proper rotation (det +1), not a
    reflection that happens to fit the points."""
    src = TABLE_MM / 1000.0
    mirror = src.copy()
    mirror[:, 0] *= -1.0                 # flip X -> left-handed target
    fit = fit_transform(src, mirror)
    assert np.linalg.det(fit["R"]) == pytest.approx(1.0, abs=1e-9)


# -- venton_table_tags / summary (pure config + dict shaping, no robot) -----

def _pal():
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
    return pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")


def test_venton_tags_come_from_the_real_config():
    # Shape, not exact survey values -- those get tuned as the table is
    # re-measured, and pinning them here would turn a survey edit into a test
    # failure.
    tags = _pal().venton_table_tags()
    assert set(tags) == {11, 12, 13}
    for t in tags.values():
        assert len(t["table_xy_mm"]) == 2
        assert all(isinstance(v, float) for v in t["table_xy_mm"])
        assert t["size_m"] > 0


def test_tags_without_an_id_or_a_position_are_dropped(monkeypatch):
    PAL12idb = _pal()
    from palmixer import config
    section = {
        "has_both":  {"id": 7, "table_xy_mm": [10.0, 20.0], "size_m": 0.0315},
        "no_id":     {"table_xy_mm": [1.0, 2.0]},          # unsearchable
        "no_xy":     {"id": 8},                             # no anchor point
        "flowcell_1": {"id": 1, "size_m": 0.0095},          # a real tag, but no table pos
    }
    monkeypatch.setattr(config, "get_section",
                        lambda n: section if n == "apriltags" else {})
    assert set(PAL12idb.venton_table_tags()) == {7}


def test_summary_is_json_safe_plain_floats():
    import json
    PAL12idb = _pal()
    fit = table_frame_in_base(
        TABLE_MM, (rot_z(12.0) @ (TABLE_MM / 1000.0).T).T + np.array([0.1, 0.2, 0.3]))
    fit["tags"] = [{"id": 11, "name": "venton_table_a",
                    "table_xy_mm": [544.2, -28.0], "base_m": [0.1, 0.2, 0.3]}]
    s = PAL12idb.venton_calibration_summary(fit)
    json.dumps(s)                                  # must not raise on numpy types
    assert len(s["origin_m"]) == 3 and isinstance(s["scale"], float)
