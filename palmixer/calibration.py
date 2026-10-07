# -*- coding: utf-8 -*-
"""Fit a coordinate frame from measured point correspondences.

Pure numpy, no robot and no camera: it takes points that are *known* in one
frame (the Venton table's surface frame, from hand measurement) and the *same*
points as measured in another (the robot base frame, from the camera), and
returns the rigid-or-similarity transform between them. PAL12idb collects the
measurements and calls in here; keeping the arithmetic separate is what lets it
be tested without hardware.

The estimator is Umeyama (1991): the least-squares similarity transform
(rotation, translation, and optionally a single uniform scale) between two
point sets. Three non-collinear correspondences are the minimum; more only
improve the fit.
"""
import math

import numpy as np


class CalibrationError(ValueError):
    """The correspondences do not determine a frame (too few, or collinear)."""


def fit_transform(source, target, with_scale=True):
    """Least-squares transform mapping ``source`` points onto ``target``.

    source, target: (N, 3) arrays of corresponding points, N >= 3, in the two
    frames and the *same unit*. source is the known frame (table), target is
    the measured one (robot base).

    Returns a dict:
        R          (3, 3) rotation, source axes expressed in the target frame
        t          (3,)   translation, the source origin in the target frame
        scale      float  uniform scale (1.0 when with_scale is False)
        residuals  (N, 3) target - transform(source), per point
        rmse       float  root-mean-square point error (target unit)
        max_error  float  largest single-point error

    With with_scale=True the scale absorbs a consistent distance error in the
    measured set -- so a camera that reads ~3% long shows up as scale ~1.03,
    which is a useful diagnostic rather than a hidden bias. With_scale=False
    forces a pure rigid fit, whose rmse then exposes that same error as
    residual instead.
    """
    P = np.asarray(source, dtype=float)
    Q = np.asarray(target, dtype=float)
    if P.shape != Q.shape or P.ndim != 2 or P.shape[1] != 3:
        raise CalibrationError("source and target must be matching (N, 3) arrays")
    n = P.shape[0]
    if n < 3:
        raise CalibrationError("need at least 3 correspondences, got %d" % n)

    muP, muQ = P.mean(axis=0), Q.mean(axis=0)
    Pc, Qc = P - muP, Q - muQ

    # Rank of the source spread: collinear (or coincident) points leave the
    # out-of-line axis unconstrained, so the frame is not determined. Caught
    # here rather than returning a transform with an arbitrary third axis.
    if np.linalg.matrix_rank(Pc, tol=1e-9) < 2:
        raise CalibrationError(
            "source points are collinear; a surface frame needs three points "
            "that are not in a line")

    cov = (Qc.T @ Pc) / n
    U, D, Vt = np.linalg.svd(cov)
    # Reflection fix: force a proper rotation (det +1), never a mirror.
    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt

    if with_scale:
        varP = (Pc ** 2).sum() / n
        scale = float((D * np.diag(S)).sum() / varP) if varP > 0 else 1.0
    else:
        scale = 1.0
    t = muQ - scale * R @ muP

    mapped = (scale * (R @ P.T)).T + t
    residuals = Q - mapped
    rmse = float(np.sqrt((residuals ** 2).sum(axis=1).mean()))
    max_error = float(np.sqrt((residuals ** 2).sum(axis=1)).max())
    return {"R": R, "t": t, "scale": scale,
            "residuals": residuals, "rmse": rmse, "max_error": max_error}


def table_frame_in_base(source_mm, target_m, with_scale=True):
    """Describe the Venton table's surface frame in robot base coordinates.

    source_mm : (N, 3) table-frame points in millimetres (z=0 on the surface).
    target_m  : (N, 3) the same points in the robot base frame, in metres.

    Converts the table points to metres, fits the transform, and returns the
    fit dict above plus:
        origin_m   (3,)   the table origin in base metres
        x_axis     (3,)   unit table +X in the base frame
        y_axis     (3,)   unit table +Y in the base frame
        normal     (3,)   unit table +Z (surface normal) in the base frame
        rmse_mm, max_error_mm : the fit errors back in millimetres, which is
                                the unit the table was measured in

    The axes come straight from the rotation columns, so they are orthonormal
    by construction; the normal is x_axis cross y_axis up to the fit.
    """
    P = np.asarray(source_mm, dtype=float) / 1000.0
    fit = fit_transform(P, np.asarray(target_m, dtype=float), with_scale=with_scale)
    R = fit["R"]
    fit.update(origin_m=fit["t"],
               x_axis=R[:, 0], y_axis=R[:, 1], normal=R[:, 2],
               rmse_mm=fit["rmse"] * 1000.0,
               max_error_mm=fit["max_error"] * 1000.0)
    return fit


# -- hand-eye calibration (eye-in-hand, AX = XB; Tsai-Lenz) ------------------
#
# Solves for the camera's pose in the robot flange frame -- i.e. camtcp -- from
# a set of (flange pose in base, target pose in camera) samples of a stationary
# target seen from varied robot poses. OpenCV's calibrateHandEye is absent from
# this build, so the classic Tsai-Lenz method is implemented here. Pure numpy;
# tested against synthetic data in tests/test_handeye.py.

def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _rot_to_rotvec(R):
    """Rotation matrix -> rotation vector (unit axis * angle), numpy only."""
    cos = (np.trace(R) - 1.0) / 2.0
    cos = max(-1.0, min(1.0, cos))
    angle = math.acos(cos)
    if angle < 1e-9:
        return np.zeros(3)
    if abs(angle - math.pi) < 1e-6:
        # Near 180 deg: axis from the symmetric part (R + I).
        w, V = np.linalg.eigh((R + np.eye(3)) / 2.0)
        axis = V[:, int(np.argmax(w))]
        return axis * angle
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * math.sin(angle)) * angle


def _rotvec_to_rot(rv):
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.eye(3)
    k = rv / angle
    K = _skew(k)
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def _tsai_vec(R):
    """Tsai's modified rotation vector P = 2 sin(theta/2) * axis."""
    rv = _rot_to_rotvec(R)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.zeros(3)
    return 2.0 * math.sin(angle / 2.0) * (rv / angle)


def solve_hand_eye(base_T_flange, cam_T_target):
    """Flange->camera transform (camtcp) from stationary-target samples.

    base_T_flange, cam_T_target: lists of N 4x4 homogeneous transforms -- the
    flange pose in the robot base, and the target pose in the camera optical
    frame, at each of N robot poses (same order). N >= 3, with varied
    orientations (pure translation does not constrain the rotation).

    Returns a 4x4 flange->camera transform X satisfying A X = X B, where A is
    gripper motion and B is camera-target motion between pose pairs. Also
    returns per-pair rotation residuals so the caller can judge the fit.
    """
    base_T_flange = [np.asarray(T, float) for T in base_T_flange]
    cam_T_target = [np.asarray(T, float) for T in cam_T_target]
    n = len(base_T_flange)
    if n != len(cam_T_target) or n < 3:
        raise CalibrationError("need >= 3 matching pose samples, got %d/%d"
                               % (n, len(cam_T_target)))

    # Motion pairs: A_ij = flange_j^-1 flange_i (gripper), B_ij = tgt_j tgt_i^-1
    # (camera), so A X = X B with X = flange->camera.
    A, B = [], []
    for i in range(n):
        for j in range(i + 1, n):
            A.append(np.linalg.inv(base_T_flange[j]) @ base_T_flange[i])
            B.append(cam_T_target[j] @ np.linalg.inv(cam_T_target[i]))

    # Rotation: skew(Pa + Pb) x = Pb - Pa, stacked over pairs; then recover R.
    M, d = [], []
    for Aij, Bij in zip(A, B):
        Pa, Pb = _tsai_vec(Aij[:3, :3]), _tsai_vec(Bij[:3, :3])
        if np.linalg.norm(_rot_to_rotvec(Aij[:3, :3])) < 1e-3:
            continue                     # a pair with no rotation says nothing
        M.append(_skew(Pa + Pb))
        d.append(Pb - Pa)
    if len(M) < 2:
        raise CalibrationError("samples have too little rotational variety; "
                               "vary the camera orientation between poses")
    M = np.vstack(M)
    d = np.concatenate(d)
    x, *_ = np.linalg.lstsq(M, d, rcond=None)
    Pcg = 2.0 * x / math.sqrt(1.0 + float(x @ x))
    npc2 = float(Pcg @ Pcg)
    Rx = ((1.0 - npc2 / 2.0) * np.eye(3)
          + 0.5 * (np.outer(Pcg, Pcg) + math.sqrt(max(0.0, 4.0 - npc2)) * _skew(Pcg)))

    # Translation: (Ra - I) tx = Rx tb - ta, stacked.
    C, e = [], []
    for Aij, Bij in zip(A, B):
        C.append(Aij[:3, :3] - np.eye(3))
        e.append(Rx @ Bij[:3, 3] - Aij[:3, 3])
    tx, *_ = np.linalg.lstsq(np.vstack(C), np.concatenate(e), rcond=None)

    X = np.eye(4)
    X[:3, :3] = Rx
    X[:3, 3] = tx

    # Per-pair rotation residual (deg): how well A_ij X = X B_ij holds.
    res = []
    for Aij, Bij in zip(A, B):
        err = np.linalg.inv(X) @ Aij @ X @ np.linalg.inv(Bij)
        res.append(math.degrees(float(np.linalg.norm(_rot_to_rotvec(err[:3, :3])))))
    return {"X": X, "rot_residual_deg_mean": float(np.mean(res)),
            "rot_residual_deg_max": float(np.max(res)), "pairs": len(A)}


def transform_to_pose_vector(T):
    """4x4 homogeneous -> [x, y, z, rx, ry, rz] (rotation vector), the UR TCP
    format camtcp uses."""
    T = np.asarray(T, float)
    rv = _rot_to_rotvec(T[:3, :3])
    return [float(T[0, 3]), float(T[1, 3]), float(T[2, 3]),
            float(rv[0]), float(rv[1]), float(rv[2])]


def pose_vector_to_transform(p):
    """[x, y, z, rx, ry, rz] -> 4x4 homogeneous."""
    T = np.eye(4)
    T[:3, :3] = _rotvec_to_rot(np.asarray(p[3:6], float))
    T[:3, 3] = np.asarray(p[0:3], float)
    return T


# -- ChArUco target pose (for hand-eye) -------------------------------------
#
# A ChArUco board gives an unambiguous, many-corner pose where a single
# AprilTag is bimodal and noisy -- the reason the AprilTag hand-eye solve had a
# 3.5 deg residual. cv2.aruco.CharucoDetector is present in this build even
# though calibrateHandEye is not.

def make_charuco_board(spec):
    """Build a cv2.aruco.CharucoBoard from a spec dict:
        {dictionary, squares_x, squares_y, square_length_m, marker_length_m}
    dictionary is an aruco dictionary name like 'DICT_4X4_50'.
    """
    import cv2
    dict_id = getattr(cv2.aruco, spec['dictionary'])
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    board = cv2.aruco.CharucoBoard(
        (int(spec['squares_x']), int(spec['squares_y'])),
        float(spec['square_length_m']), float(spec['marker_length_m']),
        dictionary)
    return board


def camera_matrix_from_f(camera_f, image_wh):
    """A pinhole matrix from a single focal length, principal point at centre.
    The rough fallback when no proper intrinsics have been calibrated."""
    w, h = image_wh
    return np.array([[camera_f, 0, w / 2.0], [0, camera_f, h / 2.0], [0, 0, 1.0]])


def detect_charuco_corners(gray, board):
    """(charuco_corners, charuco_ids) for an image, or (None, None). Shared by
    pose estimation and intrinsics calibration so both see the same detection."""
    import cv2
    detector = cv2.aruco.CharucoDetector(board)
    cc, ci, _m, _mi = detector.detectBoard(gray)
    return cc, ci


def charuco_board_pose(gray, board, camera_matrix, dist_coeffs=None, min_corners=6):
    """Pose of a ChArUco `board` in the camera optical frame from a grayscale
    image. Returns (cam_T_board 4x4, n_corners) or (None, n_corners).

    camera_matrix is the 3x3 intrinsic matrix and dist_coeffs the distortion
    (zeros if None). Pass the *calibrated* intrinsics here -- a wrong focal
    length biases the pose, which is what poisoned the first hand-eye solve.
    Fewer than `min_corners` matched is treated as no detection (a blurred
    frame mid-refocus).
    """
    import cv2
    K = np.asarray(camera_matrix, float)
    dist = np.zeros(5) if dist_coeffs is None else np.asarray(dist_coeffs, float)
    cc, ci = detect_charuco_corners(gray, board)
    if ci is None or len(ci) < min_corners:
        return None, (0 if ci is None else len(ci))
    obj_pts, img_pts = board.matchImagePoints(cc, ci)
    if obj_pts is None or len(obj_pts) < min_corners:
        return None, (0 if obj_pts is None else len(obj_pts))
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist)
    if not ok:
        return None, len(obj_pts)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    return T, len(obj_pts)


def calibrate_intrinsics(corner_views, image_wh, board, min_corners=6):
    """Camera intrinsics from several ChArUco views.

    corner_views: list of (charuco_corners, charuco_ids) from different camera
    poses. image_wh: (width, height). Returns a dict with camera_matrix (3x3
    list), dist_coeffs (list), fx/fy/cx/cy, rms reprojection error, and the
    number of views used. Needs several views with the board at varied
    positions, angles and distances.
    """
    import cv2
    obj_all, img_all = [], []
    for cc, ci in corner_views:
        if ci is None or len(ci) < min_corners:
            continue
        op, ip = board.matchImagePoints(cc, ci)
        if op is None or len(op) < min_corners:
            continue
        obj_all.append(np.asarray(op, np.float32))
        img_all.append(np.asarray(ip, np.float32))
    if len(obj_all) < 3:
        raise CalibrationError(
            "only %d usable ChArUco views; need >= 3 (more is better) at "
            "varied angles and distances" % len(obj_all))
    rms, K, dist, _rv, _tv = cv2.calibrateCamera(
        obj_all, img_all, tuple(image_wh), None, None)
    return {"camera_matrix": np.asarray(K).tolist(),
            "dist_coeffs": np.asarray(dist).ravel().tolist(),
            "fx": float(K[0, 0]), "fy": float(K[1, 1]),
            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "image_wh": [int(image_wh[0]), int(image_wh[1])],
            "rms_px": float(rms), "views": len(obj_all)}


def apriltag_pose_from_corners(corners, tag_size_m, camera_matrix, dist_coeffs=None):
    """cam_T_tag (4x4, optical frame) for one AprilTag, by solvePnP on its four
    corners with the *calibrated* camera matrix -- not the detector library's
    own pose, so it uses the real intrinsics. tag_size_m is the black-square
    edge. Returns None if the corners are unusable.

    Orientation from four coplanar points is bimodal, but the translation (tag
    centre in the camera) is stable -- and translation is all the Venton read
    uses.
    """
    import cv2
    s = float(tag_size_m) / 2.0
    obj = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]], np.float32)
    img = np.asarray(corners, np.float32).reshape(-1, 2)
    if img.shape[0] != 4:
        return None
    K = np.asarray(camera_matrix, float)
    dist = np.zeros(5) if dist_coeffs is None else np.asarray(dist_coeffs, float)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    return T
