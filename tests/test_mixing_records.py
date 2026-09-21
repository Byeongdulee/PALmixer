"""Mix provenance tests: fake pump/HTTP only, never real hardware or PVapp."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from threading import Event, Lock
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer import pump, pvapp, state
from palmixer.mixing_records import MixingJournal, new_record
from palmixer.server import PALmixerServer
from test_pvapp import Response, updater


PARAMETERS = {"mixing_speeds_ml_min": [37.5, 112.5], "start_delay_ms": [0, 49],
              "mixing_volumes_ul": [400, 1200],
              "effective_mixing_speeds_ml_min": [37.4, 112.4],
              "mixing_ttl": {"duration_ms": 20, "program_start_delay_ms": [20, 69]}}


class Endpoint:
    def __init__(self, replies):
        self.replies, self.requests, self.lock = list(replies), [], Lock()

    def request(self, request):
        self.requests.append(deepcopy(request))
        # The last reply repeats instead of running out. `status` is a
        # question, not a queue: a real dashboard asked twice in a row answers
        # the same thing twice, and the wait re-polls an inconclusive reading
        # rather than trusting a single sample of it.
        response = self.replies.pop(0) if len(self.replies) > 1 else self.replies[0]
        if isinstance(response, Exception):
            raise response
        return response


def driver(terminal="COMPLETE"):
    p = pump.Pump(simulate=True)
    p.simulate = False
    p._poll_interval_s = 0
    # Short, so the cases that deliberately never produce completion evidence
    # spend milliseconds in the re-poll window rather than the real 10 s.
    p._startup_grace_s = 0.05
    p._mixer = Endpoint([
        {"ok": True, "queued": True, "parameters": deepcopy(PARAMETERS)},
        {"ok": True, "operation_active": False,
         "operation": "Purpose: MAKE SAMPLE\nStatus: " + terminal,
         "make_sample": {"current_settings": {"mixing_speeds_ml_min": [1, 2]}}}])
    return p


def test_pump_records_accepted_snapshot_not_later_dashboard_values():
    p = driver()
    updates = []
    p.on_mix_record = updates.append
    ok, _ = p.mix()
    assert ok
    record = p.last_mix_record
    assert record["settings"] == PARAMETERS
    assert record["status"] == "completed"
    assert record["started_at"] <= record["accepted_at"] <= record["finished_at"]
    assert record["elapsed_s"] >= 0
    assert record["pump_request_id"] == p._mixer.requests[0]["id"]
    assert updates[0]["status"] == "accepted" and "finished_at" not in updates[0]
    assert updates[1]["status"] == "completed"
    assert "planned, not measured" in record["settings_source"]


@pytest.mark.parametrize("terminal,status", [("ERROR during mixing", "failed"),
    ("ABORTED during aspiration", "stopped"), ("STOP acknowledged by both pumps", "stopped"),
    ("CANCELLED", "stopped"), ("Connected", "unknown"), ("", "unknown")])
def test_idle_is_not_enough_to_claim_success(terminal, status):
    p = driver(terminal)
    ok, _ = p.mix()
    assert not ok and p.last_mix_record["status"] == status
    assert p.last_mix_record["settings"] == PARAMETERS
    assert p.last_mix_record["finished_at"]


@pytest.mark.parametrize("when", ["send", "wait", "timeout"])
def test_lost_contact_or_timeout_has_unknown_outcome(when):
    p = driver()
    if when == "send":
        p._mixer.replies = [pump.PumpError("no response")]
    elif when == "wait":
        p._mixer.replies[1] = pump.PumpError("lost connection")
    else:
        p._mixer.replies[1] = {"ok": True, "operation_active": True}
        p._op_timeout_s = -1
    assert not p.mix()[0]
    assert p.last_mix_record["status"] == "unknown"


def test_rejected_and_simulated_pump_operations_are_not_completed():
    p = driver()
    p._mixer.replies = [{"ok": False, "error": "not initialized"}]
    assert not p.mix()[0]
    assert p.last_mix_record["status"] == "failed" and p.last_mix_record["settings"] is None
    simulated = pump.Pump(simulate=True)
    assert simulated.mix()[0]
    assert simulated.last_mix_record["status"] == "simulated"
    assert simulated.last_mix_record["settings"] is None


def test_other_dashboard_operation_completion_does_not_confirm_this_mix():
    p = driver()
    p._mixer.replies[1]["operation"] = "Purpose: Cleaning\nStatus: COMPLETE"
    assert not p.mix()[0]
    assert p.last_mix_record["status"] == "unknown"


def finished_record(sample_id="S1", status="completed"):
    record = new_record(sample_id, 3, "C1", 1)
    record.update(workflow_status="completed", mixing_status=status, finished_at="2026-09-17T12:01:00+00:00",
        pump={"settings": deepcopy(PARAMETERS), "finished_at": "2026-09-17T12:00:00+00:00", "status": status})
    return record


@pytest.fixture
def remote(monkeypatch):
    storage = {"sample_id": "S1", "data": {"design": {"DSPC": 2},
        "actual_organic_components": {"DSPC": 2, "PEG-lipid": .2},
        "scattering": {"scan_id": "scan-1"}}}
    puts = []
    def put(url, **kwargs):
        puts.append(deepcopy(kwargs["json"]))
        storage["data"] = deepcopy(kwargs["json"]["data"])
        return Response(200, deepcopy(storage))
    requests = SimpleNamespace(get=lambda *a, **k: Response(200, deepcopy(storage)), put=put)
    monkeypatch.setitem(sys.modules, "requests", requests)
    return storage, puts, requests


def test_pvapp_appends_runs_and_retry_does_not_duplicate_or_erase_data(remote):
    storage, puts, _ = remote
    original = deepcopy(storage["data"])
    client = updater()
    first, second = finished_record(), finished_record()
    for record in (first, first, second):
        client.confirm_mix("S1", 3, 1, mixing_record=record)
    data = storage["data"]
    assert all(data[key] == value for key, value in original.items())
    assert [r["run_id"] for r in data["mixing_records"]] == [first["run_id"], second["run_id"]]
    assert data["mixed_at"] == first["pump"]["finished_at"]


def test_failed_mix_preserves_previous_success_and_simulation_is_not_published(remote, tmp_path):
    storage, puts, _ = remote
    client = updater()
    good, bad = finished_record(), finished_record(status="stopped")
    client.confirm_mix("S1", 3, 1, mixing_record=good)
    client.confirm_mix("S1", 4, 2, mixing_record=bad)
    assert storage["data"]["mixed_slot"] == 3
    assert storage["data"]["mixing_records"][-1]["mixing_status"] == "stopped"
    journal = MixingJournal(tmp_path)
    simulated = finished_record()
    simulated["simulated"] = True
    journal.save(simulated)
    journal.sync_pending(client)
    assert len(puts) == 2


def test_retry_of_older_run_does_not_regress_latest_mix_summary(remote):
    storage, _, _ = remote
    client = updater()
    old, recent = finished_record(), finished_record()
    recent["pump"]["finished_at"] = "2026-09-17T13:00:00+00:00"
    recent.update(slot=4, flowcell=2)
    client.confirm_mix("S1", 4, 2, mixing_record=recent)
    client.confirm_mix("S1", 3, 1, mixing_record=old)
    assert storage["data"]["mixed_at"] == recent["pump"]["finished_at"]
    assert storage["data"]["mixed_slot"] == 4 and storage["data"]["flowcell"] == 2
    assert len(storage["data"]["mixing_records"]) == 2


def test_retry_after_lost_http_ack_and_restart_is_idempotent(remote, tmp_path):
    storage, puts, requests = remote
    record = finished_record()
    journal = MixingJournal(tmp_path)
    journal.save(record)
    original_put = requests.put
    def lost_ack(*args, **kwargs):
        original_put(*args, **kwargs)
        raise RuntimeError("reply lost after commit")
    requests.put = lost_ack
    journal.sync_pending(updater())
    saved = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert saved["pvapp_sync"]["status"] == "pending"
    assert "reply lost" in saved["pvapp_sync"]["error"]
    requests.put = original_put
    MixingJournal(tmp_path).sync_pending(updater())
    assert len(storage["data"]["mixing_records"]) == 1
    saved = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert saved["pvapp_sync"]["status"] == "synced"


def test_startup_marks_interrupted_operation_unknown_and_preserves_snapshot(tmp_path):
    journal = MixingJournal(tmp_path)
    record = new_record("S1", 3, "C1", 1)
    record.update(mixing_status="running", pump={"status": "accepted", "settings": PARAMETERS})
    journal.save(record)
    (tmp_path / "corrupt.json").write_text("not json")
    (tmp_path / "bad-envelope.json").write_text('{"record": []}')
    journal.recover_interrupted()
    saved = json.loads((tmp_path / (record["run_id"] + ".json")).read_text())["record"]
    assert saved["workflow_status"] == "interrupted" and saved["mixing_status"] == "unknown"
    assert saved["pump"]["settings"] == PARAMETERS
    assert saved["finished_at"] is None  # no invented physical finish time


@pytest.mark.parametrize("failure", [None, "before_mix", "during_mix", "after_mix"])
def test_server_records_partial_workflow_and_keeps_prepared_sample_identity(tmp_path, monkeypatch, failure):
    instance = object.__new__(PALmixerServer)
    instance.simulate = False
    instance._mixing_journal = MixingJournal(tmp_path)
    instance._active_mix_record = instance._mix_record_thread = None
    instance._last_mixing_record = None
    instance._mix_sync_wake = Event()
    instance._current_step = instance._current_trace = None
    instance._beamline = "test"
    instance._mqtt = SimpleNamespace(publish=lambda *a, **k: None)
    instance.pump = driver("STOP acknowledged" if failure == "during_mix" else "COMPLETE")
    instance.pump.on_mix_record = instance._pump_mix_recorded
    monkeypatch.setattr(state, "get_sample_id", lambda slot: "S1")
    monkeypatch.setattr(state, "get_carousel_id", lambda: "C1")
    monkeypatch.setattr(state, "get_flowcell_in_use", lambda: 1)
    def run(slot, sample_id):
        assert slot == 3 and sample_id == "S1"
        if failure == "before_mix":
            raise RuntimeError("robot unavailable")
        instance._publish_step("mix", "started")
        ok, detail = instance.pump.mix()
        instance._publish_step("mix", "success" if ok else "failure", detail)
        if not ok:
            raise RuntimeError(detail)
        if failure == "after_mix":
            raise RuntimeError("transport failed after mixing")
        return "complete"
    instance.workflows = SimpleNamespace(make_sample=run)
    if failure:
        with pytest.raises(RuntimeError):
            instance._make_sample_recorded(3, None)
    else:
        instance._make_sample_recorded(3, None)
    envelope = json.loads(next(tmp_path.glob("*.json")).read_text())
    record = envelope["record"]
    assert record["sample_id"] == "S1" and record["slot"] == 3
    assert record["workflow_status"] == ("failed" if failure else "completed")
    assert record["mixing_status"] == ({"before_mix": "not_started", "during_mix": "stopped"}.get(failure, "completed"))
    assert instance._active_mix_record is None and instance._mix_sync_wake.is_set()
    assert envelope["pvapp_sync"]["status"] == "pending"
    if failure != "before_mix":
        assert record["pump"]["settings"] == PARAMETERS


@pytest.mark.parametrize("status,ok", [("Complete: draw to flowcell", True),
    ("Error", False), ("Stopped - recheck position", False), ("Ready", False),
    ("Complete: wash flowcell", False)])
def test_flowcell_draw_requires_its_own_success_status(status, ok, monkeypatch):
    p = pump.Pump(simulate=True)
    p.simulate = False
    p._poll_interval_s = 0
    p._startup_grace_s = 0.05   # "Ready" is not a verdict; it gets re-polled
    monkeypatch.setattr(state, "get_flowcell_in_use", lambda: 1)
    p._flowcell = Endpoint([{"ok": True}, {"ok": True, "operation_active": False,
        "pumps": [{"device": 0, "status": status},
                  {"device": 1, "status": "Complete: draw to flowcell"}]}])
    assert p.draw_to_flowcell()[0] is ok


@pytest.mark.parametrize("failed", [False, True])
def test_worker_result_is_correlated_even_when_idle_after_failure(monkeypatch, failed):
    from palmixer.workflows import Workflows
    instance = object.__new__(PALmixerServer)
    instance.workflows = Workflows(None, None, None, None)
    instance.workflows._step("park_at_transfer_point", lambda: "parked")
    instance.simulate = False
    instance._busy = True
    instance._busy_lock = Lock()
    instance._current_action = "draw_load_sample 1 (S1)"
    instance._current_trace = "job1"
    instance._current_step = "draw_to_flowcell"
    instance._beamline = "test"
    instance._mqtt = SimpleNamespace(publish=lambda *a, **k: None)
    monkeypatch.setattr(state, "snapshot", lambda: {"flowcell_1": "beam"})
    def run():
        if failed:
            raise RuntimeError("draw failed")
        return "done"
    instance._run_action(run, instance._current_action, "job1", "draw_load_sample 1 S1")
    assert instance._last_result["action_id"] == "job1"
    assert instance._last_result["command"] == "draw_load_sample 1 S1"
    assert instance._last_result["ok"] is (not failed)
    assert instance._last_result["state"] == {"flowcell_1": "beam"}
    assert instance._busy is False
    assert not instance.workflows.cleanup_snapshot()["parked"]


def test_draw_load_refuses_relabeling_an_existing_sample(monkeypatch):
    from palmixer.workflows import Workflows, WorkflowError
    monkeypatch.setattr(state, "validate_slot", lambda slot: slot)
    monkeypatch.setattr(state, "get_sample_id", lambda slot: "OTHER")
    workflow = object.__new__(Workflows)
    with pytest.raises(WorkflowError, match="OTHER"):
        workflow._check_draw_load_sample(1, "S1")


def test_acceptance_and_finished_result_share_the_same_action_id(monkeypatch):
    import palmixer.server as server_module
    from palmixer.workflows import Workflows
    instance = object.__new__(PALmixerServer)
    instance.workflows = Workflows(None, None, None, None)
    instance.simulate = False
    instance._busy = False
    instance._busy_lock = Lock()
    instance._beamline = "test"
    instance._mqtt = SimpleNamespace(publish=lambda *a, **k: None)
    instance._resolve = lambda *args: (lambda: "done", "draw_load_sample 1 (S1)")
    monkeypatch.setattr(state, "snapshot", lambda: {"flowcell_1": "beam"})
    monkeypatch.setattr(server_module.threading, "Thread", lambda target, args, **kw:
                        SimpleNamespace(start=lambda: target(*args)))
    reply = instance.dispatch("draw_load_sample 1 S1")
    assert reply == "ACCEPTED action_id=" + instance._last_result["action_id"]
    assert instance._last_result["command"] == "draw_load_sample 1 S1"
    assert instance._last_result["ok"] is True and instance._busy is False
