"""The Venton axes overlay: projecting the calibrated frame into the image, the
command, frame persistence, and the server handler's guards.

The projection is pure (a base<-optical transform, a camera matrix, cv2), so it
is tested directly. The robot-coupled overlay read (venton_axes_overlay) is left
to the live run; what is pinned here is everything around it that can silently
break -- the maths, the on-disk frame round-trip, and the fast-command replies.
"""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
from palmixer import commands as cmd
from palmixer.server import PALmixerServer

PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")
import numpy as np


def _K(f=600.0, cx=320.0, cy=240.0):
    return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], float)


def test_point_on_axis_projects_to_principal_point():
    # base == optical: a point 1 m straight ahead lands on the principal point.
    uv, in_front = PAL12idb._project_base_points(
        np.eye(4), _K(), None, [[0.0, 0.0, 1.0]])
    assert uv[0] == pytest.approx([320.0, 240.0], abs=1e-6)
    assert in_front == [True]


def test_lateral_offset_shifts_by_f_times_angle():
    uv, _ = PAL12idb._project_base_points(
        np.eye(4), _K(f=600.0), None, [[0.1, 0.0, 1.0]])
    assert uv[0] == pytest.approx([320.0 + 600.0 * 0.1, 240.0], abs=1e-6)


def test_a_point_behind_the_lens_is_flagged():
    _uv, in_front = PAL12idb._project_base_points(
        np.eye(4), _K(), None, [[0.0, 0.0, -1.0]])
    assert in_front == [False]


def test_frame_round_trips_through_disk(tmp_path):
    path = tmp_path / "venton_frame.json"
    frame = {"origin_m": [0.1, 0.2, 0.3],
             "x_axis": [1.0, 0.0, 0.0], "y_axis": [0.0, 1.0, 0.0],
             "normal": [0.0, 0.0, 1.0]}
    PAL12idb.save_venton_frame(frame, path=str(path))
    back = PAL12idb.load_venton_frame(path=str(path))
    assert back == frame


def test_load_missing_frame_is_none(tmp_path):
    assert PAL12idb.load_venton_frame(path=str(tmp_path / "nope.json")) is None


def test_command_builder():
    assert cmd.venton_axes_command(640, 480) == "venton_axes 640 480"
    assert cmd.venton_axes_command(640, 480, 30) == "venton_axes 640 480 30"


# -- server handler ---------------------------------------------------------
def _server(**kw):
    s = object.__new__(PALmixerServer)
    s.simulate = kw.get("simulate", False)
    s.rob = kw.get("rob", object())
    s._busy = kw.get("busy", False)
    s.PAL12idb = PAL12idb
    s._venton_frame = kw.get("frame", None)
    return s


_FRAME = {"origin_m": [0.2, -0.1, 0.05], "x_axis": [1.0, 0.0, 0.0],
          "y_axis": [0.0, 1.0, 0.0], "normal": [0.0, 0.0, 1.0]}


def test_usage_error_on_too_few_args():
    assert _server().fast_dispatch("venton_axes 640").startswith("ERROR: usage")


def test_non_integer_size_is_rejected():
    assert "integer" in _server().fast_dispatch("venton_axes 640 tall")


def test_no_frame_calibrated(monkeypatch):
    # Nothing in memory and nothing on disk.
    monkeypatch.setattr(PAL12idb, "load_venton_frame", lambda *a, **k: None)
    reply = _server(frame=None).fast_dispatch("venton_axes 640 480")
    assert reply.startswith("ERROR: no Venton frame")


def test_busy_is_refused_without_touching_the_robot():
    reply = _server(frame=_FRAME, busy=True).fast_dispatch("venton_axes 640 480")
    assert reply.startswith("ERROR: busy")


def test_simulate_returns_overlay_json_centered_on_the_frame():
    reply = _server(frame=_FRAME, simulate=True).fast_dispatch("venton_axes 640 480")
    data = json.loads(reply)
    assert data["origin"] == [320.0, 240.0]
    assert set(data) >= {"origin", "x", "y", "z", "in_front"}
