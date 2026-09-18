"""Clear sample labels without moving or reteaching the robot/carousel."""
import threading
from uuid import uuid4

import pytest

from palmixer import commands, state
from palmixer.server import PALmixerServer


@pytest.fixture
def tracked(monkeypatch, tmp_path):
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))
    monkeypatch.setattr(state, "get_carousel_size", lambda: 8)
    monkeypatch.setattr(state, "get_carousel_step", lambda: 4.2)
    state.mount_carousel("tray-A", {1: "sample-one", 2: "sample-two"})
    state.set_sample_uid(1, "sample-one", str(uuid4()))
    state.teach_carousel_slot(2, 15.5)
    state.set_carousel_slot(3)
    instance = object.__new__(PALmixerServer)
    instance._busy = False
    instance._busy_lock = threading.Lock()
    return instance


def test_clear_slots_preserves_reference_identity_positions_and_all_other_tracking(tracked):
    before = state.snapshot()
    assert before["carousel_samples"] and before["carousel_sample_uids"]
    assert tracked.fast_dispatch(commands.clear_slots_command()) == "OK"
    after = state.snapshot()
    assert after == dict(before, carousel_samples={}, carousel_sample_uids={}, carousel_full=False)
    assert after["carousel_reference"] == (2, 15.5)
    assert after["carousel_id"] == "tray-A"
    assert after["carousel_slot"] == 3
    # Metadata-only server fixture has no motor, robot or pump at all.
    assert tracked.fast_dispatch("clear_slots") == "OK"


def test_busy_clear_is_refused_without_changing_any_state(tracked):
    before = state.snapshot()
    tracked._busy = True
    assert "busy" in tracked.fast_dispatch("clear_slots")
    assert state.snapshot() == before


def test_clear_rejects_unexpected_arguments(tracked):
    before = state.snapshot()
    assert tracked.fast_dispatch("clear_slots 1").startswith("ERROR:")
    assert state.snapshot() == before
