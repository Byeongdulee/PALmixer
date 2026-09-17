"""Hidden UUIDs survive slot tracking and guard legacy PVapp updates."""
from uuid import uuid4

import pytest

from palmixer import state, pvapp
from palmixer.server import PALmixerServer
from palmixer.mixing_records import MixingJournal
from test_mixing_records import remote, finished_record
from test_pvapp import updater


@pytest.fixture
def tracking(monkeypatch, tmp_path):
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))
    monkeypatch.setattr(state, "get_carousel_size", lambda: 8)


def test_tag_uuid_survives_restart_and_rejects_conflicting_identity(tracking):
    uid = str(uuid4())
    state.set_sample_id(1, "S1")
    state.set_sample_uid(1, "S1", uid)
    state.set_sample_id(1, "S1")
    assert state.snapshot()["carousel_sample_uids"] == {1: uid}
    with pytest.raises(ValueError, match="conflicts"):
        state.set_sample_uid(1, "S1", str(uuid4()))
    with pytest.raises(ValueError, match="no longer"):
        state.set_sample_uid(1, "another", uid)
    assert state.carousel_sample_uids() == {1: uid}
    state.set_sample_id(1, "new-vial")
    assert state.carousel_sample_uids() == {}
    state.set_sample_uid(1, "new-vial", str(uuid4()))
    state.mount_carousel("another-tray", {1: "another-vial"})
    assert state.carousel_sample_uids() == {}


def test_uuid_command_is_metadata_only_and_busy_changes_are_refused(tracking):
    server = object.__new__(PALmixerServer)
    server._busy = False
    state.set_sample_id(1, "my sample")
    uid = str(uuid4())
    assert server.fast_dispatch("set_sample_uid 1 %s my sample" % uid) == "OK"
    assert state.carousel_sample_uids() == {1: uid}
    server._busy = True
    assert "busy" in server.fast_dispatch("set_sample_uid 1 %s my sample" % uid)
    state.clear_sample_id(1)
    assert state.carousel_sample_uids() == {}


def test_mixing_uuid_matches_pvapp_and_conflict_does_not_write(remote):
    storage, puts, _ = remote
    uid = str(uuid4())
    storage["data"]["sample_uid"] = uid
    record = dict(finished_record(), sample_uid=uid)
    updater().confirm_mix("S1", 3, 1, mixing_record=record)
    assert storage["data"]["mixing_records"][0]["sample_uid"] == uid
    before = len(puts)
    with pytest.raises(pvapp.PVappError, match="sample_uid conflict"):
        updater().confirm_mix("S1", 3, 1, mixing_record=dict(record, sample_uid=str(uuid4())))
    assert len(puts) == before


def test_legacy_mix_inherits_registered_uuid_in_durable_journal(remote, tmp_path):
    import json
    storage, _, _ = remote
    uid = str(uuid4())
    storage["data"]["sample_uid"] = uid
    journal = MixingJournal(tmp_path)
    record = finished_record()
    journal.save(record)
    journal.sync_pending(updater())
    saved = json.loads((tmp_path / (record["run_id"] + ".json")).read_text())
    assert saved["record"]["sample_uid"] == uid
    assert saved["pvapp_sync"]["status"] == "synced"
