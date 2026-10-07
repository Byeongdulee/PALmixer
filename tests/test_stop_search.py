"""Stop Search must never wedge the command loop.

stop_search is a fast command: it is answered inline on the single ZMQ reply
thread, on purpose, so it can reach the robot while a search worker is holding
everything else. That also means anything it does synchronously that can block
freezes *every* command, status included.

The abort itself is the stop event, which is instant. The stopj() that halts
the move in flight goes through secmon.send_program, whose wait is unbounded
and depends on the secondary-interface receive loop draining its queue -- so on
a controller whose streams drop out it can hang indefinitely. These tests pin
that stopj is fired off the reply thread and cannot hold it.
"""
import threading
import time
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer import commands as cmd
from palmixer.server import PALmixerServer


class FakeURRobot:
    """`rob.robot` -- the thing stopj() is called on. `block` lets a test make
    stopj hang the way a stalled secmon.send_program does."""

    def __init__(self, block=None):
        self.block = block          # a threading.Event to wait on, or None
        self.stopj_calls = 0
        self.stopj_entered = threading.Event()

    def stopj(self, acc=1.5):
        self.stopj_calls += 1
        self.stopj_entered.set()
        if self.block is not None:
            self.block.wait(5)      # stand in for an unbounded send_program wait


class FakeRob:
    def __init__(self, block=None):
        self.robot = FakeURRobot(block)


def _server(current_action, rob):
    s = object.__new__(PALmixerServer)
    s.simulate = False
    s.rob = rob
    s._current_action = current_action
    s._search_stop_event = threading.Event()
    return s


def test_refused_when_no_search_is_running():
    s = _server(None, FakeRob())
    assert s.fast_dispatch(cmd.stop_search_command()) == "ERROR: no AprilTag search is running"
    assert s.rob.robot.stopj_calls == 0


def test_a_different_action_running_is_not_a_search():
    s = _server("make_sample 3", FakeRob())
    assert "no AprilTag search" in s.fast_dispatch(cmd.stop_search_command())


def test_it_sets_the_stop_event_and_returns_ok():
    s = _server("search_apriltag mixer_station", FakeRob())
    assert not s._search_stop_event.is_set()
    assert s.fast_dispatch(cmd.stop_search_command()) == "OK"
    assert s._search_stop_event.is_set(), "the search was not told to stop"


def test_a_hanging_stopj_does_not_block_the_reply():
    """The regression: the reply must come back immediately even if stopj
    never returns. Before the fix this call blocked until stopj did."""
    block = threading.Event()               # never set -> stopj hangs
    s = _server("search_apriltag mixer_station", FakeRob(block))
    t0 = time.time()
    reply = s.fast_dispatch(cmd.stop_search_command())
    elapsed = time.time() - t0
    assert reply == "OK"
    assert elapsed < 0.5, "stop_search blocked on stopj for %.1fs" % elapsed
    # the event is set regardless of stopj's fate -- that is what stops the search
    assert s._search_stop_event.is_set()
    # and stopj really was dispatched, just not awaited
    assert s.rob.robot.stopj_entered.wait(2), "stopj was never fired"
    block.set()


def test_stopj_is_still_actually_sent():
    """Firing it off-thread must not mean dropping it -- a healthy stopj still
    reaches the robot, just without the reply waiting on it."""
    s = _server("search_apriltag cleaning_station", FakeRob())
    assert s.fast_dispatch(cmd.stop_search_command()) == "OK"
    assert s.rob.robot.stopj_entered.wait(2)
    assert s.rob.robot.stopj_calls == 1


def test_simulate_sets_the_event_without_touching_the_robot():
    s = _server("search_apriltag sample_table", FakeRob())
    s.simulate = True
    assert s.fast_dispatch(cmd.stop_search_command()) == "OK"
    assert s._search_stop_event.is_set()
    assert s.rob.robot.stopj_calls == 0, "stopj was sent in simulate mode"


def test_a_raising_stopj_is_swallowed_not_surfaced():
    """Off the reply thread there is no caller to return an error to, and a
    stop that set the event has already done its job -- so a stopj that throws
    is logged, not propagated into a crashed thread that could mask it."""
    class Raises(FakeRob):
        def __init__(self):
            super().__init__()
            def boom(acc=1.5):
                self.robot.stopj_entered.set()
                raise RuntimeError("controller refused stopj")
            self.robot.stopj = boom
    s = _server("search_apriltag mixer_station", Raises())
    assert s.fast_dispatch(cmd.stop_search_command()) == "OK"
    assert s.rob.robot.stopj_entered.wait(2)
