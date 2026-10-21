"""The two carousel positions: carousel_on_mixer and carousel_in_stock.

One carousel carried between the mixer and a stock spot, its AprilTag at the
centre, so both positions read the same tag (apriltags.carousel). They are
reference points, not grab seats -- the search bumps down to measure the surface
and backs off like every other station, but does not grab. These pin the wiring:
both resolve to the shared tag, both round-trip through waypoints.ini, and both
are searchable/taught/goto.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
from palmixer import commands as cmd
PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")
from palmixer import config


CAROUSELS = ("carousel_on_mixer", "carousel_in_stock", "carousel_used_stack")


def test_both_are_searchable_stations():
    for s in CAROUSELS:
        assert s in cmd.STATIONS
        assert s in cmd.TAUGHT_STATIONS
        assert s in cmd.STATION_LABELS


def test_carousel_id_resolution():
    # Carousels are numbered 31-40 and any one may stand at any carousel
    # position, so all three accept the whole range.
    assert PAL12idb.apriltag_ids("carousel_on_mixer") == set(range(31, 41))
    assert PAL12idb.apriltag_ids("carousel_used_stack") == set(range(31, 41))
    assert PAL12idb.apriltag_ids("carousel_in_stock") == set(range(31, 41))


def test_the_carousel_tag_size():
    # The physical carousel tag measures 16 mm; the camera's distance scale/
    # offset correction (config camera.*) handles the measurement error, not the
    # tag size.
    for s in CAROUSELS:
        assert PAL12idb.apriltag_size(s) == pytest.approx(0.016)


def test_they_are_bumped_but_not_grabbed():
    # NO_GRAB means the gripper is not closed on them -- but they are still
    # bumped to anchor the recorded Z to the surface (the bump in locate_apriltag
    # runs for every station; only the grab is gated on NO_GRAB_STATIONS).
    for s in CAROUSELS:
        assert s in PAL12idb.NO_GRAB_STATIONS
    import inspect
    src = inspect.getsource(PAL12idb.locate_apriltag)
    # the grab is behind the NO_GRAB gate; the bump is not.
    assert "if pos not in NO_GRAB_STATIONS:" in src
    grab_i = src.index("robot.grab()")
    bump_i = src.index("robot.bump(z=-1, backoff=backoff)")
    gate_i = src.index("if pos not in NO_GRAB_STATIONS:")
    assert gate_i < grab_i < bump_i, "the bump must run outside the no-grab gate"
    # carousels back off by the teach backoff (60 mm), other stations as before.
    assert "CAROUSEL_TEACH_BACKOFF if pos in NO_GRAB_STATIONS" in src


def test_no_tag_id_guard_does_not_trip_for_them():
    # The ID-lock guard in locate_apriltag only raises when apriltag_id is None;
    # both carousels have a configured id, so a search would not be refused.
    for s in CAROUSELS:
        assert s in PAL12idb.APRILTAG_SUBJECT
        assert PAL12idb.apriltag_ids(s)


@pytest.fixture
def ini(tmp_path, monkeypatch):
    monkeypatch.setattr(PAL12idb, "_INI_PATH", str(tmp_path / "waypoints.ini"))


def test_positions_round_trip_through_waypoints(ini):
    mixer_pose = [0.40, 0.11, 0.22, 2.2, -2.2, 0.0]
    stock_pose = [0.10, 0.44, 0.22, 2.2, -2.2, 0.0]
    PAL12idb.store_station_position("carousel_on_mixer", mixer_pose)
    PAL12idb.store_station_position("carousel_in_stock", stock_pose)
    assert PAL12idb.get_station_position("carousel_on_mixer") == pytest.approx(mixer_pose)
    assert PAL12idb.get_station_position("carousel_in_stock") == pytest.approx(stock_pose)


def test_the_two_positions_are_stored_separately(ini):
    PAL12idb.store_station_position("carousel_on_mixer", [0.4, 0.1, 0.2, 0, 0, 0])
    PAL12idb.store_station_position("carousel_in_stock", [0.1, 0.4, 0.2, 0, 0, 0])
    assert PAL12idb.get_station_position("carousel_on_mixer") != \
        PAL12idb.get_station_position("carousel_in_stock")


def test_positions_are_independent(ini):
    # Independent taught poses and independent waypoint ids, whatever tags they
    # accept.
    assert PAL12idb.carousel_on_mixerID != PAL12idb.carousel_in_stockID
    assert PAL12idb.carousel_in_stockID != PAL12idb.carousel_used_stackID


def test_config_carries_the_shared_carousel_tag():
    entry = config.get_section("apriltags")["carousel"]
    assert entry["id_range"] == [31, 40]
    assert entry["size_m"] == pytest.approx(0.016)


def test_the_carousel_tag_has_a_90_degree_mounting_offset():
    # The tag is stuck on turned 90 deg about its normal; the search takes this
    # back out before recording so the stored heading is the carousel's frame.
    for s in CAROUSELS:
        assert PAL12idb.apriltag_rz_offset(s) == pytest.approx(90.0)


def test_the_mixer_head_tag_has_a_180_degree_mounting_offset():
    # The mixer-head tag is mounted facing the opposite direction (turned 180 deg
    # about its normal), so the search takes 180 deg back out before recording.
    for s in ("mixer_station", "mixer_cleaning_station"):
        assert PAL12idb.apriltag_rz_offset(s) == pytest.approx(180.0)


def test_other_stations_have_no_mounting_offset():
    for s in ("sample_table", "cleaning_station"):
        assert PAL12idb.apriltag_rz_offset(s) == pytest.approx(0.0)


def test_locate_apriltag_passes_the_offset_into_the_search():
    # The mounting offset is applied inside the tilt search now (so the descent
    # and the recorded pose are both on the seat frame), not re-corrected after.
    import inspect
    src = inspect.getsource(PAL12idb.locate_apriltag)
    assert "rz_offset=apriltag_rz_offset(pos)" in src
    assert "search_apriltag_by_tilt" in src


def test_the_tilt_search_squares_to_the_seat_frame():
    # camera_tools squares the camera to the tag PLUS the mounting offset, at
    # both heading-align rotations (after centring, and after the descent).
    import inspect
    ct = pytest.importorskip("camera_tools", reason="UR_12idb not importable")
    assert "rz_offset" in inspect.signature(ct.search_apriltag_by_tilt).parameters
    src = inspect.getsource(ct.search_apriltag_by_tilt)
    assert src.count("AT_euler[2]+rz_offset") == 2


def test_check_positions_defined_knows_them(ini):
    # goto/teach route through check_positions_defined; an unknown station there
    # KeyErrors instead of reporting "not taught", which is what a goto to an
    # untaught carousel did before they were added to _STATION_IDS.
    missing = PAL12idb.check_positions_defined(list(CAROUSELS))
    assert set(missing) == set(CAROUSELS)          # untaught -> reported, not raised
    PAL12idb.store_station_position("carousel_on_mixer", [0.4, 0.1, 0.2, 0, 0, 0])
    assert PAL12idb.check_positions_defined(["carousel_on_mixer"]) == []
