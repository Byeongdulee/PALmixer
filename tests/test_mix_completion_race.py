"""A dashboard that is slow to start (or slow to report finishing) must not
abandon a mix that is running fine.

The failure this covers, seen on the beamline: the pumps moved, the sample was
mixed, and PALmixer still failed the run with

    sample_make: unknown (Purpose: Make Sample)

because it polled `status` into the gap between the dashboard accepting the
command and the dashboard raising operation_active. "Not active" was read as
"already finished", and with no Status line yet there was no completion
evidence to find. make_sample aborted at step 4 of 11.
"""
from pathlib import Path
import sys
import threading

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer.pump import Pump


COMPLETE = "Purpose: Make Sample\nStatus: COMPLETE - plungers at 0 uL."
ACCEPTED_NOT_STARTED = "Purpose: Make Sample"      # no Status line yet
RUNNING = "Purpose: Make Sample\nStatus: RUNNING - dispensing."


class FakeEndpoint:
    """Replays a scripted sequence of `status` replies, one per poll."""

    def __init__(self, statuses):
        self.statuses = list(statuses)
        self.polls = 0
        self.lock = threading.Lock()

    def request(self, payload):
        if payload.get("command") != "status":
            return {"ok": True, "queued": True, "parameters": {"speed": 1.0}}
        self.polls += 1
        # Clamps rather than pops: the point of these tests is how many times
        # the wait re-polls, which is not known up front.
        active, operation = self.statuses[min(self.polls - 1, len(self.statuses) - 1)]
        return {"ok": True, "operation_active": active, "operation": operation,
                "pumps": []}


def _pump(statuses):
    """A Pump wired to a scripted dashboard. Built the way the other pump
    tests build one: construct in simulate mode, then switch simulate off so
    the real request/poll path runs against the fake endpoint."""
    p = Pump(simulate=True)
    p.simulate = False
    p._poll_interval_s = 0
    p._op_timeout_s = 5.0
    p._startup_grace_s = 0.2
    endpoint = FakeEndpoint(statuses)
    p._mixer = endpoint
    return p, endpoint


def test_a_slow_starting_dashboard_no_longer_fails_a_good_mix():
    """Three polls of 'accepted but not started', then it runs, then COMPLETE.
    This is the exact sequence that produced the beamline failure."""
    pump, endpoint = _pump([
        (False, ACCEPTED_NOT_STARTED),   # <-- used to be read as "finished"
        (False, ACCEPTED_NOT_STARTED),
        (False, ACCEPTED_NOT_STARTED),
        (True, RUNNING),
        (True, RUNNING),
        (False, COMPLETE),
    ])
    ok, detail = pump.mix()
    assert ok, detail
    assert pump.last_mix_record["status"] == "completed"
    assert endpoint.polls >= 6, "it stopped polling before the mix had begun"


def test_the_old_behaviour_is_what_failed_on_the_beamline():
    """Proof that the test above is testing the right thing. startup_grace_s
    of 0 is exactly the pre-fix code path, and it reproduces the reported
    message verbatim off the same scripted dashboard."""
    statuses = ([(False, ACCEPTED_NOT_STARTED)] * 3
                + [(True, RUNNING), (True, RUNNING), (False, COMPLETE)])
    pump, endpoint = _pump(statuses)
    pump._startup_grace_s = 0
    ok, detail = pump.mix()
    assert not ok
    assert detail == "sample_make: unknown (Purpose: Make Sample)"
    assert endpoint.polls == 1, "it gave up on the very first poll"


def test_a_late_terminal_status_line_is_waited_for():
    """The mirror case: operation_active drops a beat before the dashboard
    writes Status: COMPLETE. The first idle reading has no evidence in it."""
    pump, _endpoint = _pump([
        (True, RUNNING),
        (False, ACCEPTED_NOT_STARTED),   # idle, but the status line has not landed
        (False, ACCEPTED_NOT_STARTED),
        (False, COMPLETE),
    ])
    ok, detail = pump.mix()
    assert ok, detail
    assert pump.last_mix_record["status"] == "completed"


def test_a_genuine_stop_is_still_reported_immediately():
    """An explicit STOP is the dashboard saying something definite. It must
    not be retried in the hope it changes its mind, and must still fail."""
    stopped = "Purpose: Make Sample\nStatus: STOPPED by operator."
    pump, endpoint = _pump([(True, RUNNING), (False, stopped), (False, COMPLETE)])
    ok, detail = pump.mix()
    assert not ok
    assert "stopped" in detail
    assert pump.last_mix_record["status"] == "stopped"
    assert endpoint.polls == 2, "a definite STOP was re-polled"


def test_a_dashboard_that_never_starts_still_fails_after_the_grace():
    """The guard has not been removed -- it has been made patient. A command
    that is accepted and then simply never runs still fails the workflow."""
    pump, _endpoint = _pump([(False, ACCEPTED_NOT_STARTED)])
    ok, detail = pump.mix()
    assert not ok
    assert "unknown" in detail and "Purpose: Make Sample" in detail
    assert pump.last_mix_record["status"] == "unknown"


def test_a_mix_that_errors_is_failed_not_retried():
    errored = "Purpose: Make Sample\nStatus: ERROR - syringe overrun."
    pump, endpoint = _pump([(True, RUNNING), (False, errored)])
    ok, detail = pump.mix()
    assert not ok and "failed" in detail
    assert endpoint.polls == 2, "a definite ERROR was re-polled"


@pytest.mark.parametrize("operation,expected", [
    (COMPLETE, "completed"),
    (ACCEPTED_NOT_STARTED, "unknown"),
    ("Purpose: Clean All\nStatus: COMPLETE", "unknown"),   # wrong purpose
])
def test_completion_verdicts_are_unchanged(operation, expected):
    """The verdict function itself is untouched -- only when it gets asked."""
    from palmixer.pump import _mix_completion
    assert _mix_completion(operation) == expected
