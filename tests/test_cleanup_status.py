import threading
from types import SimpleNamespace

from palmixer.workflows import Workflows


def test_cleanup_waits_for_both_background_operations_and_parking():
    wash, mixer = threading.Event(), threading.Event()
    def run(event):
        assert event.wait(3)
        return True, "done"
    workflow = Workflows(None, None, SimpleNamespace(wash_flowcell=lambda: run(wash),
        clean_mixer=lambda: run(mixer)), None)
    assert not workflow.cleanup_snapshot()["wash_complete"]
    workflow._start_background_pump_op("clean_mixer")
    mixer_thread = workflow._background_pump[1]
    workflow._start_background_pump_op("wash_flowcell")
    wash_thread = workflow._background_pump[1]
    workflow._step("park_at_transfer_point", lambda: "parked")
    assert workflow.cleanup_snapshot()["parked"] and workflow.cleanup_snapshot()["active"]
    wash.set()
    wash_thread.join(3)
    assert workflow.cleanup_snapshot()["wash_complete"]
    assert workflow.cleanup_snapshot()["active"]  # mixer wash is still running
    mixer.set()
    mixer_thread.join(3)
    assert not workflow.cleanup_snapshot()["active"]
    assert not workflow.cleanup_snapshot()["problem"]


def test_failed_background_wash_is_retained_after_thread_exits_and_restart_is_unknown():
    workflow = Workflows(None, None, SimpleNamespace(wash_flowcell=lambda: (False, "pump lost")), None)
    workflow._start_background_pump_op("wash_flowcell")
    workflow._background_pump[1].join(3)
    workflow._step("park_at_transfer_point", lambda: "parked")
    snapshot = workflow.cleanup_snapshot()
    assert "pump lost" in snapshot["problem"] and not snapshot["wash_complete"]
    restarted = Workflows(None, None, None, None).cleanup_snapshot()
    assert restarted["session"] != snapshot["session"]
    assert not restarted["parked"] and not restarted["wash_complete"]
