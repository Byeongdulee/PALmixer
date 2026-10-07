"""The physical AprilTags: which one each station's search looks for, and how
big it is.

Two facts drive the shape of this. A tag belongs to the thing it is stuck to,
not to the place -- the flowcell carries its tag to every station it visits,
so `cleaning_station` looks for a different tag depending on which flowcell is
in use. And `size_m` is the BLACK SQUARE's edge, not the printed sheet: a
tag36h11 is 8 cells of black inside a white quiet zone, so measuring the sheet
reads ~1.3x high and scales every distance with it.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))

PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")
urcamera = pytest.importorskip("common.urcamera", reason="UR_12idb not importable")
from palmixer import config


class Detection:
    def __init__(self, tag_id, center):
        self.tag_id = tag_id
        self.center = center


@pytest.fixture
def flowcell(monkeypatch):
    """Set which flowcell is in use without touching the state ini."""
    def use(n):
        monkeypatch.setattr(PAL12idb, "flowcell_ID", n)
    return use


@pytest.fixture
def tags(monkeypatch):
    """Drive the apriltags section without touching the real config file."""
    section = {}
    real = config.get_section
    monkeypatch.setattr(config, "get_section",
                        lambda name: section if name == "apriltags" else real(name))
    return section


# -- which tag -------------------------------------------------------------

def test_the_flowcell_stations_follow_the_flowcell_in_use(flowcell, tags):
    """The case a station-keyed mapping could not express, and the reason this
    is keyed by object. The sample table has no tag of its own -- what a
    search sees there is the flowcell that was transferred onto it, wearing
    the same tag it wore at its cleaning station."""
    tags.update({"flowcell_1": {"id": 1, "size_m": 0.0095},
                 "flowcell_2": {"id": 2, "size_m": 0.0095}})
    for n, expected in ((1, 1), (2, 2)):
        flowcell(n)
        assert PAL12idb.apriltag_id("cleaning_station") == expected
        assert PAL12idb.apriltag_id("sample_table") == expected


def test_the_mixer_stations_do_not_follow_the_flowcell(flowcell, tags):
    """The head's tag stays on the head, whichever flowcell is in use."""
    tags.update({"mixer_head": {"id": 25}})
    for n in (1, 2):
        flowcell(n)
        assert PAL12idb.apriltag_id("mixer_station") == 25
        assert PAL12idb.apriltag_id("mixer_cleaning_station") == 25


def test_both_mixer_stations_share_the_head_tag(flowcell, tags):
    """The head carries one tag between its two stations."""
    assert PAL12idb._apriltag_subject("mixer_station") == \
           PAL12idb._apriltag_subject("mixer_cleaning_station") == "mixer_head"


def test_an_unset_id_means_nearest_the_image_center(tags):
    """A real behaviour, not a missing value -- correct wherever one tag at a
    time is in view."""
    tags["sample_table"] = {"size_m": 0.0315}
    assert PAL12idb.apriltag_id("sample_table") is None
    tags["sample_table"] = {"id": None, "size_m": 0.0315}
    assert PAL12idb.apriltag_id("sample_table") is None


def test_a_station_with_no_tag_at_all_resolves_to_nothing():
    """sample_on_mixer_station is derived, not searched for."""
    assert PAL12idb._apriltag_subject("sample_on_mixer_station") is None
    assert PAL12idb.apriltag_id("sample_on_mixer_station") is None


# -- how big ---------------------------------------------------------------

def test_the_flowcell_black_squares_are_what_got_calipered(flowcell):
    """Against the real config, not a fixture. 9.5 mm replaced a 12 mm that
    was the printed sheet, which had every flowcell-tag distance 26% long."""
    for n in (1, 2):
        flowcell(n)
        assert PAL12idb.apriltag_size("cleaning_station") == pytest.approx(0.0095)


def test_the_venton_table_tags_are_recorded_but_inert():
    """The two 31.5 mm fiducials on the Venton table fix the other tags'
    positions relative to each other. Nothing searches for them, so no station
    may resolve to one -- but their numbers and sizes are worth not losing,
    and a later calibration from a few camera motions needs exactly this."""
    section = config.get_section("apriltags")
    venton = {k: v for k, v in section.items() if k.startswith("venton_table_")}
    assert {v["id"] for v in venton.values()} == {11, 12, 13}
    assert all(v["size_m"] == pytest.approx(0.0315) for v in venton.values())
    assert not set(PAL12idb.APRILTAG_SUBJECT.values()) & set(venton), \
        "a station resolves to a Venton fiducial; those are references, not seats"


def test_every_fiducial_carries_its_position_in_the_table_frame():
    """The survey data the planned calibration will need. Positions are
    millimetres from the table corner along the UR5's own X and Y, and the
    frame is written down beside them -- "the corner of the table" is only
    unambiguous while the person who picked it is standing there."""
    # Each fiducial carries a 2-D table position; the exact values are survey
    # data that gets re-measured, so this checks they are present and well
    # formed rather than pinning numbers a re-survey would change.
    section = config.get_section("apriltags")
    for name in ("venton_table_a", "venton_table_b", "venton_table_c"):
        xy = section[name]["table_xy_mm"]
        assert len(xy) == 2 and all(isinstance(v, (int, float)) for v in xy)
    frame = config.get_section("venton_table_frame")
    assert frame["units"] == "mm"
    assert "UR5" in frame["axes"] and "corner" in frame["origin"]


def test_no_two_fiducials_overlap():
    """Catches the transposed-coordinate class of error: id 13 first came in
    as (230, -29), which put its centre 7.8 mm from id 12 -- inside 12's own
    31.5 mm square. Two tags lying flat need at least their edge length
    between centres."""
    import math
    section = config.get_section("apriltags")
    placed = [(k, v["table_xy_mm"], v["size_m"] * 1000)
              for k, v in section.items() if v.get("table_xy_mm")]
    for i, (ka, pa, sa) in enumerate(placed):
        for kb, pb, sb in placed[i + 1:]:
            gap = math.dist(pa, pb)
            assert gap >= max(sa, sb), (
                "%s and %s are %.1f mm apart but are %.1f mm tags -- they "
                "would overlap" % (ka, kb, gap, max(sa, sb)))


def test_the_surveyed_separation_agrees_with_the_tape():
    """Two independent hand measurements of the same pair, which is what
    makes the camera's 3.5% excess attributable to the camera."""
    import math
    section = config.get_section("apriltags")
    gap = math.dist(section["venton_table_a"]["table_xy_mm"],
                    section["venton_table_b"]["table_xy_mm"])
    assert gap == pytest.approx(311.0, abs=1.0)


def test_there_is_no_tag_entry_for_a_station_that_has_no_tag():
    """The sample table carries nothing of its own; an entry for it would be
    a phantom that silently overrode the flowcell's real tag."""
    assert "sample_table" not in config.get_section("apriltags")


def test_the_mixer_head_size_is_still_the_unverified_inherited_one():
    """16 mm is the old hardcoded value, written the way the flowcell's 12 mm
    was -- the printed sheet, against a 9.5 mm black square. If 16 mm is the
    sheet here too the square is nearer 12.7 mm, which is 43 mm of standoff.
    This fails once someone calipers it, which is the point."""
    assert config.get_section("apriltags")["mixer_head"]["size_m"] == \
        pytest.approx(0.016), "mixer_head was measured -- update this test"


def test_a_missing_entry_falls_back_to_a_size_but_never_to_an_id(tags):
    """A guessed size is better than none; a guessed id would aim a search at
    the wrong tag."""
    assert PAL12idb.apriltag_size("cleaning_station") > 0
    assert PAL12idb.apriltag_id("cleaning_station") is None


@pytest.mark.parametrize("entry", [{"id": "two"}, {"size_m": "big"}, "nonsense"])
def test_a_nonsense_entry_falls_back_instead_of_raising(tags, entry, capsys):
    tags["sample_table"] = entry
    assert PAL12idb.apriltag_id("sample_table") is None
    assert PAL12idb.apriltag_size("sample_table") > 0


# -- selection, in UR_12idb ------------------------------------------------

def test_selectAT_prefers_the_named_tag_over_the_centered_one():
    near, far = Detection(12, (320.0, 240.0)), Detection(11, (80.0, 60.0))
    assert urcamera.selectAT([near, far], tag_id=11, center=(320, 240)).tag_id == 11


def test_selectAT_returns_nothing_when_the_named_tag_is_absent():
    """Not "the other one instead". A search over the wrong station must fail
    rather than quietly record a position from its neighbour."""
    assert urcamera.selectAT([Detection(12, (320.0, 240.0))],
                             tag_id=11, center=(320, 240)) is None


def test_selectAT_without_a_tag_id_still_takes_the_nearest():
    near, far = Detection(12, (300.0, 250.0)), Detection(11, (80.0, 60.0))
    assert urcamera.selectAT([near, far], tag_id=None, center=(320, 240)).tag_id == 12


def test_the_camera_carries_a_preferred_id_alongside_the_tag_size():
    """Both describe the tag being looked for rather than the frame in hand,
    so both are set once on the camera and read by every detection."""
    assert "AT_preferred_id" in urcamera.camera.__init__.__code__.co_names
    import inspect
    source = inspect.getsource(urcamera.camera.decodeAT)
    assert source.index("tag_id = self.AT_preferred_id") < source.index("selectAT")
