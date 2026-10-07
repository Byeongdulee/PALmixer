"""Hand-eye solver: recover a known camera-in-flange transform (camtcp).

Synthetic, no robot. Build samples from a chosen true X (flange->camera) and a
fixed target, run the solver, check it gets X back -- clean, then with noise.
This is what lets the solver be trusted before it drives a real calibration,
since OpenCV's calibrateHandEye is absent from this build.
"""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer.calibration import (
    solve_hand_eye, transform_to_pose_vector, pose_vector_to_transform,
    _rotvec_to_rot, CalibrationError)


def transform(rotvec, t):
    T = np.eye(4)
    T[:3, :3] = _rotvec_to_rot(np.asarray(rotvec, float))
    T[:3, 3] = t
    return T


def make_samples(X, base_T_target, flange_poses):
    """cam_T_target_i = X^-1 @ flange_i^-1 @ base_T_target."""
    cam = []
    for F in flange_poses:
        cam.append(np.linalg.inv(X) @ np.linalg.inv(F) @ base_T_target)
    return flange_poses, cam


def varied_flange_poses(n=12, seed=0):
    """Flange poses with real orientation variety (hand-eye needs it)."""
    rng = np.random.default_rng(seed)
    poses = []
    for _ in range(n):
        rv = rng.uniform(-0.6, 0.6, size=3)         # up to ~34 deg
        t = rng.uniform(-0.3, 0.3, size=3) + np.array([0.3, -0.1, 0.4])
        poses.append(transform(rv, t))
    return poses


# A plausible camtcp-like truth: ~4 cm offset, 30 deg tilt about X.
X_TRUE = transform([-math.radians(30), 0.0, 0.0], [0.0, 0.0433, 0.06])
BASE_T_TARGET = transform([math.pi, 0.0, 0.3], [0.25, -0.14, 0.09])


def test_recovers_a_known_transform_exactly():
    flange, cam = make_samples(X_TRUE, BASE_T_TARGET, varied_flange_poses())
    out = solve_hand_eye(flange, cam)
    assert out["rot_residual_deg_max"] < 1e-6
    assert np.allclose(out["X"], X_TRUE, atol=1e-6)


def test_recovered_pose_vector_matches_camtcp_form():
    flange, cam = make_samples(X_TRUE, BASE_T_TARGET, varied_flange_poses())
    p = transform_to_pose_vector(solve_hand_eye(flange, cam)["X"])
    assert p[:3] == pytest.approx([0.0, 0.0433, 0.06], abs=1e-6)
    assert p[3] == pytest.approx(-math.radians(30), abs=1e-6)
    assert p[4:6] == pytest.approx([0.0, 0.0], abs=1e-6)


def test_tolerates_measurement_noise():
    flange, cam = make_samples(X_TRUE, BASE_T_TARGET, varied_flange_poses(n=20))
    rng = np.random.default_rng(1)
    noisy = []
    for T in cam:
        T2 = T.copy()
        T2[:3, 3] += rng.normal(scale=0.001, size=3)                  # 1 mm
        T2[:3, :3] = _rotvec_to_rot(rng.normal(scale=math.radians(0.5), size=3)) @ T2[:3, :3]
        noisy.append(T2)
    out = solve_hand_eye(flange, noisy)
    p = transform_to_pose_vector(out["X"])
    assert p[:3] == pytest.approx([0.0, 0.0433, 0.06], abs=5e-3)      # within 5 mm
    ang = np.linalg.norm(np.array(p[3:6]) - np.array([-math.radians(30), 0, 0]))
    assert math.degrees(ang) < 2.0                                    # within 2 deg


def test_rejects_too_few_samples():
    flange, cam = make_samples(X_TRUE, BASE_T_TARGET, varied_flange_poses(n=2))
    with pytest.raises(CalibrationError):
        solve_hand_eye(flange, cam)


def test_rejects_pure_translation():
    """No rotation variety -> rotation unconstrained -> refuse."""
    t_only = [transform([0, 0, 0], [0.3 + 0.05 * i, -0.1, 0.4]) for i in range(6)]
    flange, cam = make_samples(X_TRUE, BASE_T_TARGET, t_only)
    with pytest.raises(CalibrationError):
        solve_hand_eye(flange, cam)


def test_pose_vector_round_trips():
    p = [0.01, 0.0433, 0.06, -0.5236, 0.1, -0.2]
    assert transform_to_pose_vector(pose_vector_to_transform(p)) == pytest.approx(p, abs=1e-9)
