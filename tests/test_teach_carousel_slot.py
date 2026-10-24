"""Set Current Position publishes the taught and current slot together."""
from types import SimpleNamespace

import pytest

from palmixer import config, state
from palmixer.server import PALmixerServer


@pytest.fixture
def tracking(tmp_path, monkeypatch):
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))
    monkeypatch.setattr(config, "get_section", lambda name: {"size": 16, "step": -4500} if name == "carousel" else {})


@pytest.mark.parametrize("previous", [None, 12])
def test_set_current_position_updates_reference_and_gui_snapshot_atomically(tracking, monkeypatch, previous):
    if previous is not None:
        state.set_carousel_slot(previous)
    published = []
    monkeypatch.setattr(state, "_listeners", [lambda snapshot: published.append(snapshot)])
    server = object.__new__(PALmixerServer)
    server.simulate = False
    server.motor = SimpleNamespace(read=lambda: 85500)
    assert server.fast_dispatch("teach_carousel_slot 4") == "OK"
    snapshot = state.snapshot()
    assert snapshot["carousel_slot"] == 4
    assert snapshot["carousel_reference"] == (4, 85500)
    assert len(published) == 1
    assert published[0]["carousel_slot"] == 4
    assert published[0]["carousel_reference"] == (4, 85500)


def test_read_failure_preserves_existing_current_slot_and_reference(tracking):
    state.teach_carousel_slot(8, 67500)
    def fail():
        raise RuntimeError("motor offline")
    server = object.__new__(PALmixerServer)
    server.simulate = False
    server.motor = SimpleNamespace(read=fail)
    assert "motor offline" in server.fast_dispatch("teach_carousel_slot 4")
    assert state.get_carousel_slot() == 8
    assert state.carousel_reference() == (8, 67500)


@pytest.mark.parametrize("direction,position,expected", [
    ("forward", 90000, 3), ("reverse", 81000, 5),
    ("forward", 103500, 16), ("reverse", 31500, 16),
])
def test_jog_updates_current_slot_from_returned_motor_position(tracking, monkeypatch, direction, position, expected):
    state.teach_carousel_slot(4, 85500)
    updates = []
    monkeypatch.setattr(state, "_listeners", [lambda snapshot: updates.append(snapshot["carousel_slot"])])
    server = object.__new__(PALmixerServer)
    server.simulate = False
    server.motor = SimpleNamespace(tweak=lambda direction, step: position)
    assert server._motor_tweak(direction, 4500) == "motor now at %s" % position
    assert state.get_carousel_slot() == expected
    assert updates == [expected]
    assert state.carousel_reference() == (4, 85500)


def test_two_half_slot_jogs_clear_then_restore_slot(tracking):
    state.teach_carousel_slot(4, 85500)
    server = object.__new__(PALmixerServer)
    server.simulate = False
    positions = iter([87750, 90000])
    server.motor = SimpleNamespace(tweak=lambda direction, step: next(positions))
    server._motor_tweak("forward", 2250)
    assert state.get_carousel_slot() == state.UNKNOWN
    server._motor_tweak("forward", 2250)
    assert state.get_carousel_slot() == 3


def test_failed_jog_clears_stale_slot(tracking):
    state.teach_carousel_slot(4, 85500)
    def fail(direction, step):
        raise RuntimeError("motor stopped partway")
    server = object.__new__(PALmixerServer)
    server.simulate = False
    server.motor = SimpleNamespace(tweak=fail)
    with pytest.raises(RuntimeError, match="stopped partway"):
        server._motor_tweak("forward", 4500)
    assert state.get_carousel_slot() == state.UNKNOWN


def test_readback_tracking_supports_positive_direction(tracking, monkeypatch):
    monkeypatch.setattr(config, "get_section", lambda name: {"size": 16, "step": 4500} if name == "carousel" else {})
    state.teach_carousel_slot(4, 85500)
    assert state.update_carousel_slot_from_position(90000) == 5
    assert state.update_carousel_slot_from_position(81000) == 3


@pytest.mark.parametrize("position", [None, float('nan'), float('inf'), 'bad', 85501])
def test_invalid_or_unaligned_readback_is_unknown(tracking, position):
    state.teach_carousel_slot(4, 85500)
    assert state.update_carousel_slot_from_position(position) == state.UNKNOWN


def test_readback_without_valid_reference_is_unknown(tracking, monkeypatch):
    state.set_carousel_slot(4)
    assert state.update_carousel_slot_from_position(85500) == state.UNKNOWN
    state.teach_carousel_slot(4, 85500)
    monkeypatch.setattr(state, "carousel_reference_stale", lambda: True)
    assert state.update_carousel_slot_from_position(85500) == state.UNKNOWN
