"""A mixer-head move ordered mid-wash waits for the wash instead of failing.

The head is docked at its cleaning station with liquid running through it, so
moving it right away would spill -- but the order itself is fine, it has just
arrived early. Every path to the head therefore holds until clean_mixer
finishes rather than refusing (see "Mixer wash safety" in the README).
"""
import threading
import time

from palmixer.workflows import Workflows, WorkflowError


class FakeMixingDashboard:
    """Stands in for the pump-dashboard ZMQ status reply. `active` is the
    hardware truth every check here is really about: pump0/pump1 running."""

    def __init__(self, active=False, unreachable=False):
        self.active = active
        self.unreachable = unreachable
        self.polls = 0
        self._op_timeout_s = 5.0

    def status_snapshot(self):
        self.polls += 1
        if self.unreachable:
            raise RuntimeError("pump dashboard not answering")
        return {"servers": {"mixer": {"operation_active": self.active},
                            "flowcell": {"operation_active": False}}}


def _logging_workflow(pump=None):
    log = []
    workflow = Workflows(None, None, pump, None,
                         on_step=lambda step, phase, detail="": log.append((step, phase)))
    return workflow, log


def _washing_workflow():
    """A Workflows with a clean_mixer running that finishes only when the
    returned event is set, plus the step log every assertion reads."""
    release = threading.Event()
    pump = FakeMixingDashboard()
    pump.clean_mixer = lambda: (release.wait(5), "washed")
    workflow, log = _logging_workflow(pump)
    workflow._start_background_pump_op("clean_mixer")
    assert workflow.mixer_head_busy()
    return workflow, release, log


def test_move_waits_for_the_wash_rather_than_raising():
    workflow, release, log = _washing_workflow()
    moved = threading.Event()

    def move():
        workflow._move_mixer_head("mixer2mixingstation", lambda: moved.set() or "at mixer")

    mover = threading.Thread(target=move, daemon=True)
    mover.start()

    # The move must not happen while the wash is running. Long enough that a
    # move that ignored the wash would have landed by now.
    time.sleep(0.3)
    assert not moved.is_set(), "the head moved while it was still being washed"
    assert ("wait_for_clean_mixer", "started") in log
    assert workflow.mixer_head_busy(), "the head should read busy for the whole wait"

    release.set()
    mover.join(5)
    assert moved.is_set(), "the head never moved after the wash finished"
    assert ("wait_for_clean_mixer", "success") in log
    assert ("mixer2mixingstation", "success") in log
    # The wait is reported before the move, not merged into it: a hold of
    # minutes has to be legible as waiting, not as a stalled transport.
    assert log.index(("wait_for_clean_mixer", "success")) < \
           log.index(("mixer2mixingstation", "started"))
    assert not workflow.mixer_head_busy()


def test_pumps_running_that_palmixer_did_not_start_still_hold_the_head():
    """The case that actually happened on the beamline: pump0 and pump1 were
    washing the head, but the wash was not one this process had fired, so
    there was no background thread to join. The head must still not move --
    the dashboard's own operation_active is what decides."""
    dashboard = FakeMixingDashboard(active=True)
    workflow, log = _logging_workflow(dashboard)
    assert not workflow._background_pump_threads(), "no local bookkeeping, by design"
    assert workflow.mixer_head_busy(), "an outside wash must still pin the head"

    moved = threading.Event()
    mover = threading.Thread(target=lambda: workflow._move_mixer_head(
        "mixer2mixingstation", lambda: moved.set() or "at mixer"), daemon=True)
    mover.start()
    time.sleep(0.3)
    assert not moved.is_set(), "the head moved while pump0/pump1 were running"
    assert ("wait_for_mixer_pumps", "started") in log

    dashboard.active = False        # the dashboard finishes the wash
    mover.join(5)
    assert moved.is_set(), "the head never moved after the pumps went idle"
    assert ("wait_for_mixer_pumps", "success") in log
    assert ("mixer2mixingstation", "success") in log


def test_a_flowcell_wash_does_not_displace_a_tracked_mixer_wash():
    """One slot used to hold the background op, so starting a wash_flowcell
    erased a clean_mixer that was still running -- after which nothing waited
    for it. They are on different servers and are meant to overlap."""
    release = threading.Event()
    pump = FakeMixingDashboard()
    pump.clean_mixer = lambda: (release.wait(5), "washed")
    pump.wash_flowcell = lambda: (release.wait(5), "washed")
    workflow, _log = _logging_workflow(pump)
    workflow._start_background_pump_op("clean_mixer")
    workflow._start_background_pump_op("wash_flowcell")
    assert workflow.mixer_head_busy(), "the flowcell wash erased the mixer wash"
    release.set()


def test_timeout_refuses_the_move_rather_than_moving_anyway():
    """An operation still active past the pump's own timeout is a fault. The
    head stays put and the move fails loudly -- driving it out of a station
    that may still have liquid running through it is the worse mistake."""
    dashboard = FakeMixingDashboard(active=True)
    dashboard._op_timeout_s = 0.05      # the pump's own per-op timeout
    workflow, log = _logging_workflow(dashboard)
    moved = []
    try:
        workflow._move_mixer_head("mixer2mixingstation", lambda: moved.append(1))
    except WorkflowError as e:
        assert "refusing to move the mixer head" in str(e)
    else:
        raise AssertionError("the move went ahead despite the pumps never going idle")
    assert not moved, "the head moved after the wait timed out"
    assert ("mixer2mixingstation", "started") not in log


def test_an_unreachable_dashboard_does_not_strand_the_robot():
    """Indistinguishable from a dashboard that is switched off. Refusing to
    ever move the head because a status socket is down would be worse."""
    workflow, log = _logging_workflow(FakeMixingDashboard(unreachable=True))
    assert not workflow.mixer_head_busy()
    workflow._move_mixer_head("mixer2mixingstation", lambda: "at mixer")
    assert ("mixer2mixingstation", "success") in log


def test_await_is_a_no_op_when_nothing_is_washing():
    workflow, log = _logging_workflow(FakeMixingDashboard(active=False))
    workflow.await_mixer_wash()
    assert log == [], "a wait step was reported with no wash running"
    workflow._move_mixer_head("mixer2cleaningstation", lambda: "docked")
    assert ("mixer2cleaningstation", "success") in log


def test_a_flowcell_wash_does_not_hold_the_mixer_head():
    """Only the mixing server pins the head. wash_flowcell runs on the other
    pump and has nothing to do with where the head is."""
    pump = FakeMixingDashboard()
    pump.wash_flowcell = lambda: (time.sleep(2), "washed")
    workflow, log = _logging_workflow(pump)
    workflow._start_background_pump_op("wash_flowcell")
    assert not workflow.mixer_head_busy()
    started = time.time()
    workflow._move_mixer_head("mixer2mixingstation", lambda: "at mixer")
    assert time.time() - started < 1.0, "the head waited for an unrelated wash"
    assert ("mixer2mixingstation", "success") in log
    assert not any(step.startswith("wait_for") for step, _ in log)


def test_a_failing_wash_releases_the_head_instead_of_pinning_it():
    """The move still happens after a wash that reports failure -- the wash's
    own failure is reported by its own step trio, separately."""
    pump = FakeMixingDashboard()
    pump.clean_mixer = lambda: (False, "pump lost")
    workflow, log = _logging_workflow(pump)
    workflow._start_background_pump_op("clean_mixer")
    workflow._move_mixer_head("mixer2mixingstation", lambda: "at mixer")
    assert ("clean_mixer", "failure") in log
    assert ("mixer2mixingstation", "success") in log
    assert not workflow.mixer_head_busy()
