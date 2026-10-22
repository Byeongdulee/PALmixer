"""Preparation scheduling and the real lift function, with no UR dependency."""
import ast
import functools
from pathlib import Path
from types import SimpleNamespace
import threading
from uuid import uuid4

import pytest

from palmixer.preparation import PreparationCoordinator, PreparationError
from palmixer.workflows import Workflows


def binding(slot=1, previous=None):
    uid = str(uuid4())
    return dict(placement_id=str(uuid4()), sample_uid=uid, order_id=uid,
                carousel_id="carousel-A", slot=slot, release_after_sample_uid=previous)


def released(coordinator, value):
    coordinator.queue(value)
    coordinator.open_initial(value["placement_id"])
    coordinator.wait()
    coordinator.release(value["placement_id"], value["sample_uid"], completion_id="done-A",
                        prepared=True, pal_tool_clear=True)


def lift_function(robot, log, *, travel=None):
    """Compile the actual transport and tracking wrappers without importing SDKs."""
    path = Path(__file__).parents[1] / "palmixer" / "PAL12idb.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names = {"_tracks", "_flowcell_setter", "load_sample_to_beam"}
    module = ast.Module(body=[node for node in tree.body
                             if isinstance(node, ast.FunctionDef) and node.name in names], type_ignores=[])
    locations = []
    state = SimpleNamespace(FC_AT_BEAM="at_beam", UNKNOWN="unknown",
                            set_flowcell_location=lambda fc, where: locations.append(where))
    def transfer(robot, position):
        log.append(("transfer", position))
        if travel:
            travel()
    env = dict(functools=functools, state=state, flowcell_ID=1, needle_clear_height=.08,
               distance_gripper_tag=.02, sampletable_clear_height=.1,
               _align_sample_table_if_enabled=lambda: "restore",
               get_sampletable_position=lambda: [1, 2, 3, 4, 5, 6],
               move_via_transferpoint=transfer,
               _dropdown_recovering=lambda robot, station, position, place: place(),
               dropdown_sampletable=lambda robot: log.append(("drop",)),
               _restore_stage=lambda *args: log.append(("restore",)))
    exec(compile(module, str(path), "exec"), env)
    return env["load_sample_to_beam"], locations


@pytest.mark.parametrize("return_value", [None, False, True])
def test_real_lift_ignores_return_value_and_preserves_movements(return_value):
    log = []
    robot = SimpleNamespace(mvr2z=lambda height: log.append(("lift", height)) or return_value)
    fn, locations = lift_function(robot, log)
    fn(robot)
    baseline = list(log)
    log.clear()
    locations.clear()
    def notification():
        assert not locations  # FC_AT_BEAM must not be set at the lift milestone.
        log.append(("notification",))
    fn(robot, on_lift_complete=notification)
    assert log[0:2] == [("lift", .08), ("notification",)]
    assert [entry for entry in log if entry[0] != "notification"] == baseline
    assert [entry[0] for entry in baseline] == ["lift", "transfer", "drop", "restore"]
    assert locations == ["at_beam"]


def test_real_lift_exception_keeps_existing_tracking_and_does_not_notify():
    log = []
    def fail(height):
        raise RuntimeError("original lift error")
    robot = SimpleNamespace(mvr2z=fail)
    fn, locations = lift_function(robot, log)
    with pytest.raises(RuntimeError, match="original lift error"):
        fn(robot, on_lift_complete=lambda: log.append("notified"))
    assert log == []
    assert locations == ["unknown"]


def test_notification_exception_cannot_interrupt_existing_beam_transfer(caplog):
    log = []
    robot = SimpleNamespace(mvr2z=lambda height: log.append(("lift", height)))
    fn, locations = lift_function(robot, log)
    def fail():
        raise RuntimeError("notification only")
    fn(robot, on_lift_complete=fail)
    assert [entry[0] for entry in log] == ["lift", "transfer", "drop", "restore"]
    assert locations == ["at_beam"]
    assert "notification only" in caplog.text


def test_restart_remembers_interrupted_sample_even_without_next_request(tmp_path):
    path = tmp_path / "state.json"
    c = PreparationCoordinator(path, lambda p: None, lambda p: None)
    a = binding()
    released(c, a)
    c.begin_sample(a["sample_uid"], 1, "A", carousel_id="carousel-A")
    c.close()
    restarted = PreparationCoordinator(path, lambda p: None, lambda p: None)
    assert "interrupted" in restarted.snapshot()["error"]
    with pytest.raises(PreparationError):
        restarted.queue(binding(2))
    restarted.close()


def test_initial_head_clearance_exception_never_dispatches_carousel(tmp_path, monkeypatch):
    from palmixer import state
    moves = []
    def fail(robot):
        raise RuntimeError("head movement failed")
    wf = Workflows(SimpleNamespace(mixer2cleaningstation=fail), None, None, None)
    c = PreparationCoordinator(tmp_path / "state.json", moves.append, lambda p: None)
    wf.preparation = c
    monkeypatch.setattr(wf, "_check_make_sample", lambda *a: 0)
    monkeypatch.setattr(wf, "_move_mixer_head", lambda label, action: action())
    monkeypatch.setattr(state, "get_mixer_head", lambda: state.MIXER_AT_MIXER)
    a = binding()
    c.queue(a)
    with pytest.raises(RuntimeError, match="head movement failed"):
        wf.prepare_initial(a["placement_id"])
    assert moves == []
    assert c.get(a["placement_id"])["phase"] == "uncertain"
    c.close()


def test_sample_failure_during_positioning_keeps_next_preparation_uncertain(tmp_path):
    entered, finish = threading.Event(), threading.Event()
    sent = []
    a = binding()
    b = binding(2, a["sample_uid"])
    def move(p):
        if p == b:
            entered.set()
            assert finish.wait(3)
    c = PreparationCoordinator(tmp_path / "state.json", move, sent.append)
    released(c, a)
    c.queue(b)
    c.begin_sample(a["sample_uid"], 1, "A", carousel_id="carousel-A")
    c.lift_complete(a["sample_uid"], "A")
    assert entered.wait(2)
    c.finish_sample(a["sample_uid"], "A", RuntimeError("beam transfer failed"))
    finish.set()
    c.wait()
    assert sent == [a]
    assert c.get(a["placement_id"])["workflow_outcome"]["status"] == "failed"
    assert c.get(b["placement_id"])["phase"] == "uncertain"
    c.close()


def test_pending_next_placement_waits_for_matching_lift_and_motor(tmp_path):
    moves, notifications = [], []
    motor_started, motor_finish = threading.Event(), threading.Event()
    a = binding()
    b = binding(2, a["sample_uid"])
    def move(value):
        moves.append(value["slot"])
        if value == b:
            motor_started.set()
            assert motor_finish.wait(3)
    coordinator = PreparationCoordinator(tmp_path / "state.json", move, notifications.append)
    released(coordinator, a)
    coordinator.queue(b)
    assert moves == [1]  # Idle gap before make_sample is NOT a release point.
    coordinator.begin_sample(a["sample_uid"], 1, "action-A", carousel_id="carousel-A")
    assert moves == [1]
    coordinator.lift_complete(a["sample_uid"], "action-A")
    assert motor_started.wait(2)
    assert notifications == [a]
    coordinator.lift_complete(a["sample_uid"], "action-A")  # Duplicate milestone.
    motor_finish.set()
    coordinator.wait()
    assert moves == [1, 2]
    assert notifications == [a, b]
    assert coordinator.get(b["placement_id"])["phase"] == "slot_ready"
    coordinator.finish_sample(a["sample_uid"], "action-A")
    with pytest.raises(PreparationError):
        coordinator.require_idle()  # Ownership survives the parent workflow.
    with pytest.raises(PreparationError):
        coordinator.release(b["placement_id"], b["sample_uid"], completion_id="B",
                            prepared=True, pal_tool_clear=False)
    coordinator.release(b["placement_id"], b["sample_uid"], completion_id="B",
                        prepared=True, pal_tool_clear=True)
    coordinator.close()


def test_late_request_uses_unconsumed_window_only(tmp_path):
    moves = []
    coordinator = PreparationCoordinator(tmp_path / "state.json", moves.append, lambda p: None)
    a = binding()
    released(coordinator, a)
    coordinator.begin_sample(a["sample_uid"], 1, "A", carousel_id="carousel-A")
    coordinator.lift_complete(a["sample_uid"], "A")
    coordinator.finish_sample(a["sample_uid"], "A")
    b = binding(2, a["sample_uid"])
    coordinator.queue(b)
    coordinator.wait()
    assert [p["slot"] for p in moves] == [1, 2]
    coordinator.close()


def test_invalidated_window_does_not_release_late_request(tmp_path):
    moves = []
    c = PreparationCoordinator(tmp_path / "state.json", moves.append, lambda p: None)
    a = binding()
    released(c, a)
    c.begin_sample(a["sample_uid"], 1, "A", carousel_id="carousel-A")
    c.lift_complete(a["sample_uid"], "A")
    c.finish_sample(a["sample_uid"], "A")
    c.require_idle()
    c.queue(binding(2, a["sample_uid"]))
    assert len(moves) == 1
    c.close()


def test_notification_retry_never_repeats_motor_motion(tmp_path):
    moves, deliveries = [], []
    def notify(value):
        deliveries.append(value)
        if len(deliveries) == 1:
            raise TimeoutError("ack lost")
    c = PreparationCoordinator(tmp_path / "state.json", moves.append, notify)
    a = binding()
    c.queue(a)
    c.open_initial(a["placement_id"])
    c.wait()
    assert c.get(a["placement_id"])["phase"] == "ready_unacknowledged"
    c.retry_ready(a["placement_id"])
    c.wait()
    assert moves == [a]
    assert deliveries == [a, a]
    assert c.get(a["placement_id"])["phase"] == "slot_ready"
    c.close()


def test_lift_does_not_wait_for_persistence_on_positioning_worker(tmp_path, monkeypatch):
    entered, finish = threading.Event(), threading.Event()
    moves = []
    c = PreparationCoordinator(tmp_path / "state.json", moves.append, lambda p: None)
    a = binding()
    released(c, a)
    b = binding(2, a["sample_uid"])
    c.queue(b)
    c.begin_sample(a["sample_uid"], 1, "A", carousel_id="carousel-A")
    original_save = c._save
    def delayed_save():
        entered.set()
        assert finish.wait(3)
        original_save()
    monkeypatch.setattr(c, "_save", delayed_save)
    try:
        c.lift_complete(a["sample_uid"], "A")
        assert entered.wait(2)
        assert moves == [a]  # Durable write must finish before moving B.
    finally:
        finish.set()
        c.wait()
        c.close()
    assert moves == [a, b]


def test_wrong_mounted_carousel_and_workflow_replay_are_refused(tmp_path):
    c = PreparationCoordinator(tmp_path / "state.json", lambda p: None, lambda p: None)
    a = binding()
    released(c, a)
    with pytest.raises(PreparationError, match="predecessor"):
        c.queue(binding(2))  # A is prepared but has not yet mixed or lifted.
    with pytest.raises(PreparationError):
        c.begin_sample(a["sample_uid"], 1, "A", carousel_id="another-tray")
    c.begin_sample(a["sample_uid"], 1, "A", carousel_id="carousel-A")
    c.finish_sample(a["sample_uid"], "A")
    with pytest.raises(PreparationError):
        c.begin_sample(a["sample_uid"], 1, "A", carousel_id="carousel-A")
    c.close()


def test_motor_failure_does_not_notify_or_replay_on_restart(tmp_path):
    sent = []
    def fail(value):
        raise RuntimeError("motor failed")
    path = tmp_path / "state.json"
    c = PreparationCoordinator(path, fail, sent.append)
    a = binding()
    c.queue(a)
    c.open_initial(a["placement_id"])
    c.wait()
    assert c.get(a["placement_id"])["phase"] == "uncertain"
    c.close()
    restored = PreparationCoordinator(path, lambda value: pytest.fail("replayed"), sent.append)
    assert restored.queue(a)["phase"] == "uncertain"
    assert sent == []
    with pytest.raises(PreparationError):
        restored.queue(binding(2))
    restored.close()


def test_duplicate_conflict_cancel_and_failed_lift_context(tmp_path):
    c = PreparationCoordinator(tmp_path / "state.json", lambda p: None, lambda p: None)
    a = binding()
    assert c.queue(a) == c.queue(a)
    with pytest.raises(PreparationError):
        c.queue(dict(a, slot=2))
    c.cancel(a["placement_id"])
    assert c.queue(a)["phase"] == "cancelled"
    # An unexpected callback latches a coordination failure, never raising on UR.
    c.lift_complete(a["sample_uid"], "wrong-action")
    assert c.snapshot()["error"]
    c.close()


def test_real_transport_overlaps_next_positioning(tmp_path):
    log = []
    travel_entered, travel_finish = threading.Event(), threading.Event()
    b_ready = threading.Event()
    a = binding()
    b = binding(2, a["sample_uid"])
    c = PreparationCoordinator(tmp_path / "state.json", lambda p: log.append(("motor", p["slot"])),
                               lambda p: b_ready.set() if p == b else None)
    released(c, a)
    c.queue(b)
    c.begin_sample(a["sample_uid"], 1, "A", carousel_id="carousel-A")
    def travel():
        travel_entered.set()
        assert travel_finish.wait(3)
    robot = SimpleNamespace(mvr2z=lambda height: log.append(("lift", height)))
    fn, locations = lift_function(robot, log, travel=travel)
    thread = threading.Thread(target=lambda: fn(robot, on_lift_complete=lambda:
                                                c.lift_complete(a["sample_uid"], "A")))
    thread.start()
    try:
        assert travel_entered.wait(2)
        assert b_ready.wait(2)
        assert thread.is_alive()
        assert locations == []
        assert log.index(("lift", .08)) < log.index(("motor", 2))
    finally:
        travel_finish.set()
        thread.join(3)
        c.wait()
        c.close()
    assert locations == ["at_beam"]


@pytest.mark.parametrize("step", [4500, -4500])
def test_preparation_target_is_absolute_and_updates_slot_at_mixer(tmp_path, monkeypatch, step):
    from palmixer import state
    moved, tracked = [], []
    wf = Workflows(None, None, None, SimpleNamespace(check_in_range=lambda p: None, move_to=moved.append))
    c = PreparationCoordinator(tmp_path / "state.json",
                               lambda p: wf.position_for_preparation(p, offset_steps=-4), lambda p: None)
    wf.preparation = c
    monkeypatch.setattr(state, "validate_slot", lambda slot: slot)
    monkeypatch.setattr(state, "get_carousel_id", lambda: "carousel-A")
    monkeypatch.setattr(state, "carousel_reference_stale", lambda: False)
    monkeypatch.setattr(state, "get_mixer_head", lambda: state.MIXER_AT_CLEANING)
    monkeypatch.setattr(state, "slot_position", lambda slot: 100 + (slot - 1) * step)
    monkeypatch.setattr(state, "require_carousel_step", lambda: step)
    monkeypatch.setattr(state, "get_carousel_size", lambda: 16)
    monkeypatch.setattr(state, "set_carousel_slot", tracked.append)
    a = binding(2)
    c.queue(a)
    c.open_initial(a["placement_id"])
    c.wait()
    assert moved == [100 - 3 * step]
    assert tracked == [14]
    c.close()
