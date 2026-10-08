"""Where the flowcell is released when it is set down.

Two seats get a low computed-Z release instead of the default 5 mm, and the
flowcell cleaning station stops being set down by feel (the deep-seat bump reads
the arm's own deceleration as contact there):

  * flowcell cleaning station: 2 mm above the grab point, no bump;
  * sample stage: 1 mm above the grab point.

dropdown_at's arithmetic is pinned directly; the cleaning-station transport's
switch away from the bump is checked structurally (running it would scribble on
the real tracked-location ini).
"""
import ast
import inspect
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")


class FakeRobot:
    def __init__(self):
        self.moves = []
        self.calls = []
    def moveto(self, p, **k):
        self.moves.append(list(p))
        self.calls.append("moveto")
    def release(self):
        self.calls.append("release")
    def mvr2z(self, dz):
        self.calls.append(("mvr2z", dz))
    def get_pos(self):
        return [0.0, 0.0, 0.5]


def _expected_release_z(taught_z, release_above):
    return (taught_z - PAL12idb.distance_gripper_tag
            - PAL12idb.grab_depth + release_above)


def test_the_release_constants():
    assert PAL12idb.cleaning_station_release_above == pytest.approx(0.002)
    assert PAL12idb.sampletable_release_above == pytest.approx(0.001)
    assert PAL12idb.dropdown_release_above == pytest.approx(0.005)   # unchanged default


def test_dropdown_at_defaults_to_five_millimetres():
    rob = FakeRobot()
    PAL12idb.dropdown_at(rob, [0.1, 0.2, 0.5])
    assert rob.moves[0][2] == pytest.approx(_expected_release_z(0.5, 0.005))
    assert "release" in rob.calls


def test_dropdown_at_honours_release_above():
    rob = FakeRobot()
    PAL12idb.dropdown_at(rob, [0.1, 0.2, 0.5], release_above=0.002)
    assert rob.moves[0][2] == pytest.approx(_expected_release_z(0.5, 0.002))


def test_dropdown_passes_release_above_through():
    rob = FakeRobot()
    PAL12idb.dropdown(rob, station_pos=[0.1, 0.2, 0.5], release_above=0.002)
    assert rob.moves[0][2] == pytest.approx(_expected_release_z(0.5, 0.002))


def test_dropdown_by_bump_ignores_release_above(monkeypatch):
    # by_bump feels for the seat; release_above has no say there.
    rob = FakeRobot()
    monkeypatch.setattr(rob, "bump", lambda **k: rob.calls.append("bump"), raising=False)
    PAL12idb.dropdown(rob, station_pos=[0.1, 0.2, 0.5], by_bump=True,
                      release_above=0.002)
    assert "bump" in rob.calls
    assert not rob.moves, "a computed-Z move was made on the bump path"


def test_sample_stage_drop_releases_one_millimetre_up(monkeypatch):
    monkeypatch.setattr(PAL12idb, "get_sampletable_position",
                        lambda: [0.1, 0.2, 0.5])
    rob = FakeRobot()
    PAL12idb.dropdown_sampletable(rob)
    assert rob.moves[0][2] == pytest.approx(_expected_release_z(0.5, 0.001))


def _transport_call(func):
    tree = ast.parse(inspect.getsource(func).lstrip())
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and getattr(n.func, "id", None) == "transport"]
    assert calls, "%s no longer calls transport()" % func.__name__
    return calls[0]


def test_cleaning_station_drop_is_two_mm_computed_z_not_bump():
    call = _transport_call(PAL12idb.load_flowcell_from_beam_to_cleaningstation)
    kwargs = {kw.arg: kw.value for kw in call.keywords}
    # No longer set down by feel...
    assert "drop_by_bump" not in kwargs or (
        isinstance(kwargs["drop_by_bump"], ast.Constant)
        and kwargs["drop_by_bump"].value is False), \
        "the cleaning-station drop still bumps"
    # ...but at the 2 mm computed-Z release.
    assert "drop_release_above" in kwargs, \
        "the cleaning-station drop does not pass drop_release_above"
    node = kwargs["drop_release_above"]
    name = getattr(node, "id", None) or getattr(node, "attr", None)
    assert name == "cleaning_station_release_above"
