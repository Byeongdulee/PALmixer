"""The prefill gap is pulled back as the flowcell is lifted off its station.

The FlowCell wash ends with a prefill that leaves the syringe on the sample
port holding a set volume (180 uL with apssector12_pump_control's shipped
recipe); the 50 uL gap is pulled back separately, by the pump's pull_back_gap,
once ready_flowcell_to_draw has lifted the flowcell clear of its cleaning
station -- the needle is in air there -- and before it travels to the vial.

What has to hold:

- the flowcell does not leave its station while its own pump is busy (the
  wash pushes liquid out through the needle), whoever started the wash;
- the pull-back happens after the lift and before the approach, and a
  refusal keeps the flowcell away from the vial;
- the reasons a pull-back would be refused are caught before anything moves:
  dashboard down, a dashboard without the locked recipe, or a syringe that
  does not hold what a finished wash leaves;
- a pull-back is only a success on the flowcell's own "Complete" status.
"""
from copy import deepcopy
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer import pump, state
from palmixer.workflows import Workflows, WorkflowError

RECIPE = {"locked": True, "prefill_final_ul": 180.0, "pulled_back_ul": 230.0,
          "position_tolerance_ul": 0.5}


@pytest.fixture
def tracked(monkeypatch, tmp_path):
    """An isolated state file: flowcell 1 in use, seated, mixer head docked."""
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))
    state.set_flowcell_in_use(1)
    state.set_flowcell_location(1, state.FC_AT_CLEANING)
    state.set_mixer_head(state.MIXER_AT_CLEANING)


class FakeFlowcellPump:
    """The flowcell dashboard as Workflows sees it through Pump."""

    _op_timeout_s = 5.0

    def __init__(self, position=180.0, busy=False, other_busy=False,
                 unreachable=False, recipe=RECIPE, pull_back=None):
        self.position = position
        self.busy = busy
        self.other_busy = other_busy
        self.unreachable = unreachable
        self.recipe = recipe
        self.pull_back = pull_back or (True, "pull_back_gap (device 0) complete")
        self.events = []

    @staticmethod
    def flowcell_device(flowcell_id=None):
        return pump.Pump.flowcell_device(flowcell_id)

    def status_snapshot(self):
        if self.unreachable:
            flowcell = {"error": "tcp://127.0.0.1:5556: Resource temporarily unavailable",
                        "pumps": [{"name": "flowcell2", "status": "no reply", "position": "--"},
                                  {"name": "flowcell3", "status": "no reply", "position": "--"}]}
        else:
            flowcell = {"error": None, "recipe": deepcopy(self.recipe),
                        "operation_active": self.busy or self.other_busy,
                        "pumps": [{"device": 0, "operation_active": self.busy,
                                   "position_ul": self.position},
                                  {"device": 1, "operation_active": self.other_busy,
                                   "position_ul": 0.0}]}
        return {"servers": {"mixer": {"operation_active": False}, "flowcell": flowcell}}

    def pull_back_gap(self):
        self.events.append("pull_back_gap")
        return self.pull_back

    def draw_to_flowcell(self):
        self.events.append("draw_to_flowcell")
        return True, "draw_to_flowcell (device 0) complete"

    def shake_sample(self, flowcell_id=None):
        time.sleep(0.05)
        return True, "shaken"

    def stop_shaking(self):
        return True, "stopped"


class FakePal:
    """The two PAL12idb transports a draw uses, recording the arm's order."""

    def __init__(self, events):
        self.events = events

    def ready_flowcell_to_draw(self, robot, before_approach=None):
        self.events.append("pickup")            # lifted clear of the station
        if before_approach is not None:
            before_approach()
        self.events.append("approach")          # carried to the vial, lowered

    def load_sample_to_beam(self, robot):
        self.events.append("load_sample_to_beam")

    def goto_transfer_point(self, robot):
        self.events.append("park")

    def mixer2cleaningstation(self, robot):
        self.events.append("mixer2cleaningstation")


def _workflow(fake_pump):
    log = []
    pal = FakePal(fake_pump.events)
    workflow = Workflows(pal, object(), fake_pump, None,
                         on_step=lambda step, phase, detail="": log.append((step, phase, detail)))
    return workflow, log


def _phases(log):
    return [(step, phase) for step, phase, _detail in log]


# -- the draw sequence --------------------------------------------------------
def test_draw_pulls_the_gap_back_after_the_lift_and_before_the_vial(tracked):
    fake = FakeFlowcellPump()
    workflow, log = _workflow(fake)
    try:
        workflow.draw_and_load()
    finally:
        workflow.stop_auto_shake()
    assert fake.events[:5] == ["pickup", "pull_back_gap", "approach",
                               "draw_to_flowcell", "load_sample_to_beam"]
    detail = next(d for s, p, d in log if (s, p) == ("ready_flowcell_to_draw", "success"))
    assert "gap pulled back after the lift" in detail
    # Folded into the transport's step, not a step of its own.
    assert not any(step == "pull_back_gap" for step, _ in _phases(log))


def test_a_refused_pull_back_keeps_the_flowcell_away_from_the_vial(tracked):
    fake = FakeFlowcellPump(pull_back=(False, "pull_back_gap (device 0): completion "
                                              "unverified (Error)"))
    workflow, log = _workflow(fake)
    with pytest.raises(WorkflowError, match="stopped above its cleaning station"):
        workflow.draw_and_load()
    assert fake.events == ["pickup", "pull_back_gap"]
    assert ("ready_flowcell_to_draw", "failure") in _phases(log)
    assert not any(step == "draw_to_flowcell" for step, _ in _phases(log))


@pytest.mark.parametrize("fake_kwargs,message", [
    ({"position": 0.0}, "holds 0.0 uL, not the 180 uL a finished wash leaves"),
    ({"position": None}, "does not know how much"),
    ({"unreachable": True}, "not answering"),
    ({"recipe": None}, "predates apssector12_pump_control 3.0.0"),
])
def test_a_draw_that_could_not_pull_back_is_refused_before_anything_moves(
        tracked, fake_kwargs, message):
    """After a FlowCellControl restart, for instance, both syringes are homed
    to 0 uL: the draw is refused when accepted, not with the flowcell lifted."""
    fake = FakeFlowcellPump(**fake_kwargs)
    workflow, log = _workflow(fake)
    for call in (workflow._check_draw_and_load, workflow.draw_and_load):
        with pytest.raises(WorkflowError, match=message):
            call()
    assert fake.events == []
    assert log == []


def test_make_sample_is_refused_before_mixing(tracked, monkeypatch):
    fake = FakeFlowcellPump(position=0.0)
    fake.mix = lambda: pytest.fail("mixed a sample the flowcell could not take")
    workflow, _log = _workflow(fake)
    monkeypatch.setattr(workflow, "_check_make_sample", lambda slot, sample_id=None: 0.0)
    with pytest.raises(WorkflowError, match="Run Wash \\+ prefill on flowcell 1"):
        workflow._make_sample(3)
    assert fake.events == []


@pytest.mark.parametrize("position", [180.0, 180.4, 230.0, 229.6])
def test_a_syringe_at_either_volume_may_be_drawn_with(tracked, position):
    workflow, _log = _workflow(FakeFlowcellPump(position=position))
    workflow.check_flowcell_can_draw(wash_may_still_run=False)


def test_a_wash_still_running_at_acceptance_is_judged_when_it_ends(tracked):
    fake = FakeFlowcellPump(position=None, busy=True)
    workflow, _log = _workflow(fake)
    workflow.check_flowcell_can_draw()              # the wash will leave 180 uL
    with pytest.raises(WorkflowError):
        workflow.check_flowcell_can_draw(wash_may_still_run=False)


# -- waiting for the flowcell's own pump --------------------------------------
def test_the_flowcell_waits_out_our_own_wash_in_its_station(tracked):
    release = threading.Event()
    fake = FakeFlowcellPump()
    fake.wash_flowcell = lambda: (release.wait(5), "washed")
    workflow, log = _workflow(fake)
    workflow._start_background_pump_op("wash_flowcell")
    waiter = threading.Thread(target=workflow.await_flowcell_ready, daemon=True)
    waiter.start()
    time.sleep(0.3)
    assert waiter.is_alive(), "the flowcell was cleared to leave mid-wash"
    assert ("wait_for_wash_flowcell", "started") in _phases(log)
    release.set()
    waiter.join(5)
    assert not waiter.is_alive()
    assert ("wait_for_wash_flowcell", "success") in _phases(log)


def test_a_wash_palmixer_did_not_start_still_holds_the_flowcell(tracked):
    fake = FakeFlowcellPump(busy=True)
    workflow, log = _workflow(fake)
    assert workflow.flowcell_pump_running()
    waiter = threading.Thread(target=workflow.await_flowcell_ready, daemon=True)
    waiter.start()
    time.sleep(0.3)
    assert waiter.is_alive()
    assert ("wait_for_flowcell_pump", "started") in _phases(log)
    fake.busy = False
    waiter.join(5)
    assert not waiter.is_alive()
    assert ("wait_for_flowcell_pump", "success") in _phases(log)


def test_the_other_flowcell_being_busy_does_not_hold_this_one(tracked):
    workflow, log = _workflow(FakeFlowcellPump(other_busy=True))
    assert not workflow.flowcell_pump_running()
    started = time.time()
    workflow.await_flowcell_ready()
    assert time.time() - started < 1.0
    assert log == []


def test_a_pump_that_never_goes_idle_is_refused_not_waited_on_forever(tracked):
    fake = FakeFlowcellPump(busy=True)
    fake._op_timeout_s = 0.05
    workflow, _log = _workflow(fake)
    with pytest.raises(WorkflowError, match="refusing to take the flowcell out"):
        workflow.await_flowcell_ready()


# -- the Experiment tab's button ----------------------------------------------
def test_the_experiment_tab_button_follows_the_same_rule(tracked):
    fake = FakeFlowcellPump()
    workflow, _log = _workflow(fake)
    detail = workflow.ready_flowcell_to_draw()
    assert fake.events == ["pickup", "pull_back_gap", "approach"]
    assert "gap pulled back" in detail
    with pytest.raises(WorkflowError, match="holds 0.0 uL"):
        _workflow(FakeFlowcellPump(position=0.0))[0].ready_flowcell_to_draw()


def test_the_server_routes_the_button_through_workflows():
    from palmixer.server import PALmixerServer
    server = object.__new__(PALmixerServer)
    server.simulate = False
    server.rob = object()
    server.workflows = SimpleNamespace(
        ready_flowcell_to_draw=lambda: "gap pulled back after the lift: ok")
    server.PAL12idb = SimpleNamespace(
        ready_flowcell_to_draw=lambda robot: pytest.fail("transport called directly"))
    assert server._run_transport("ready_flowcell_to_draw") == (
        "ready_flowcell_to_draw complete; gap pulled back after the lift: ok")


# -- the pump client ----------------------------------------------------------
class Endpoint:
    def __init__(self, replies):
        self.replies, self.requests, self.lock = list(replies), [], threading.Lock()

    def request(self, request):
        self.requests.append(deepcopy(request))
        return self.replies.pop(0) if len(self.replies) > 1 else self.replies[0]


def _client(monkeypatch, replies, flowcell_in_use=1):
    p = pump.Pump(simulate=True)
    p.simulate = False
    p._poll_interval_s = 0
    p._startup_grace_s = 0.05
    p._op_timeout_s = 2.0
    monkeypatch.setattr(state, "get_flowcell_in_use", lambda: flowcell_in_use)
    p._flowcell = Endpoint(replies)
    return p


def _status(device_status, device=0, other_busy=False):
    return {"ok": True, "operation_active": other_busy, "pumps": [
        {"device": device, "operation_active": False, "status": device_status},
        {"device": 1 - device, "operation_active": other_busy,
         "status": "Running: flow sample" if other_busy else "Ready"}]}


def test_pull_back_names_only_the_command_and_the_flowcell(monkeypatch):
    p = _client(monkeypatch, [{"ok": True}, _status("Complete: pull back gap", device=1)],
                flowcell_in_use=2)
    ok, detail = p.pull_back_gap()
    assert ok, detail
    request = p._flowcell.requests[0]
    assert {k: v for k, v in request.items() if k != "id"} == {
        "command": "pull_back_gap", "device": 1, "confirmed": True}
    assert pump.server_for("pull_back_gap") == pump.FLOWCELL_SERVER


@pytest.mark.parametrize("status,ok", [
    ("Complete: pull back gap", True), ("Error", False),
    ("Stopped - recheck position", False), ("Ready", False),
    ("Complete: wash flowcell", False), ("Complete: draw to flowcell", False)])
def test_a_pull_back_succeeds_only_on_its_own_complete_status(monkeypatch, status, ok):
    """A refusal (syringe not at the prefill volume) ends idle with "Error";
    without this check it would read as done once the grace ran out."""
    p = _client(monkeypatch, [{"ok": True}, _status(status)])
    assert p.pull_back_gap()[0] is ok


def test_the_other_flowcell_busy_does_not_hold_a_finished_pull_back(monkeypatch):
    p = _client(monkeypatch, [{"ok": True}, _status("Complete: pull back gap", other_busy=True)])
    started = time.monotonic()
    ok, detail = p.pull_back_gap()
    assert ok, detail
    assert time.monotonic() - started < 1.0


def test_a_rejected_pull_back_is_a_failure(monkeypatch):
    p = _client(monkeypatch, [{"ok": False, "error": "unknown command 'pull_back_gap'"}])
    ok, detail = p.pull_back_gap()
    assert not ok and "rejected" in detail and "unknown command" in detail


def test_flowcell_status_carries_position_trust_and_recipe(monkeypatch):
    p = _client(monkeypatch, [{}])
    p._flowcell_status = Endpoint([{
        "ok": True, "connected": True, "operation_active": False, "recipe": RECIPE,
        "pumps": [{"device": 0, "name": "flowcell2", "operation_active": False,
                   "position_ul": 180.0, "position_trusted": True,
                   "trust_reason": "Homed by Wash + prefill flow cell",
                   "status": "Complete: wash flowcell",
                   "position": "Position: 180.0 / 1000 uL | valve: port 3 (sample)"},
                  {"device": 1, "name": "flowcell3", "operation_active": False,
                   "position_ul": None, "position_trusted": False,
                   "trust_reason": "Pull back gap failed", "status": "Error"}]}])
    entry = p._flowcell_status_snapshot()
    assert entry["recipe"] == RECIPE
    first, second = entry["pumps"]
    assert (first["device"], first["operation_active"], first["position_ul"]) == (0, False, 180.0)
    assert first["trust_reason"] is None                # trusted: no reason to report
    assert (second["position_ul"], second["trust_reason"]) == (None, "Pull back gap failed")


def test_an_untrusted_position_is_refused_with_the_dashboards_reason(tracked):
    fake = FakeFlowcellPump(position=None)
    snapshot = fake.status_snapshot
    def with_reason():
        status = snapshot()
        status["servers"]["flowcell"]["pumps"][0]["trust_reason"] = "Emergency stop"
        return status
    fake.status_snapshot = with_reason
    with pytest.raises(WorkflowError, match="position untrusted: Emergency stop"):
        _workflow(fake)[0].check_flowcell_can_draw(wash_may_still_run=False)


# -- the transport hook --------------------------------------------------------
@pytest.fixture
def pal12idb(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[2] / "UR_12idb"))
    module = pytest.importorskip("palmixer.PAL12idb",
                                 reason="UR_12idb (camera_tools) not importable")
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))
    state.set_flowcell_location(1, state.FC_AT_CLEANING)
    events = []
    monkeypatch.setattr(module, "flowcell_ID", 1)
    monkeypatch.setattr(module, "get_cleaningstation_position", lambda n: [0.1, 0.2, 0.3])
    monkeypatch.setattr(module, "get_sample_on_mixerstation_position", lambda: [0.4, 0.5, 0.6])
    monkeypatch.setattr(module, "cleaning_station_open_extra", lambda: 0.0)
    monkeypatch.setattr(module, "move_to_cleaningstation",
                        lambda robot, station, via_transferpoint=True: events.append("to_station"))
    monkeypatch.setattr(module, "_pickup_recovering",
                        lambda robot, station, height, open_extra_m=0.0: events.append("pickup"))
    robot = SimpleNamespace(moveto=lambda p: events.append("moveto"),
                            bump=lambda z, backoff: events.append("bump"))
    return module, robot, events


def test_the_hook_runs_after_the_lift_and_before_the_approach(pal12idb):
    module, robot, events = pal12idb
    module.ready_flowcell_to_draw(robot, before_approach=lambda: events.append("hook"))
    assert events == ["to_station", "pickup", "hook", "moveto", "bump"]
    assert state.get_flowcell_location(1) == state.FC_IN_GRIPPER


def test_a_failing_hook_stops_the_transport_where_it_is(pal12idb):
    module, robot, events = pal12idb

    def refuse():
        events.append("hook")
        raise WorkflowError("gap not pulled back")

    with pytest.raises(WorkflowError):
        module.ready_flowcell_to_draw(robot, before_approach=refuse)
    assert events == ["to_station", "pickup", "hook"]
    assert state.get_flowcell_location(1) == state.UNKNOWN


def test_without_a_hook_the_transport_is_unchanged(pal12idb):
    module, robot, events = pal12idb
    module.ready_flowcell_to_draw(robot)
    assert events == ["to_station", "pickup", "moveto", "bump"]
