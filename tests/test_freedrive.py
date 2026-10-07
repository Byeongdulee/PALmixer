"""The freedrive (hand-guiding) toggle.

freedrive is a fast command, answered inline on the ZMQ reply thread like
unlock_stop: turning it *off* must work even while the server is holding busy,
because that is a recovery. Turning it *on* is refused while a worker action is
running -- releasing the arm's brakes mid-move is dangerous. These pin that
contract and the on/off plumbing down to robUR.set_freedrive.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer import commands as cmd
from palmixer.server import PALmixerServer


class FakeRob:
    """Records set_freedrive calls. `raise_on` lets a test make it throw."""
    def __init__(self, raise_on=None):
        self.calls = []
        self.raise_on = raise_on

    def set_freedrive(self, on, timeout=600):
        if self.raise_on is not None:
            raise self.raise_on
        self.calls.append(bool(on))


def _server(rob, busy=False, simulate=False):
    s = object.__new__(PALmixerServer)
    s.simulate = simulate
    s.rob = rob
    s._busy = busy
    return s


def test_command_builder_shapes_the_wire_string():
    assert cmd.freedrive_command(True) == "freedrive on"
    assert cmd.freedrive_command(False) == "freedrive off"


def test_turning_it_on_when_idle_reaches_the_robot():
    rob = FakeRob()
    s = _server(rob)
    assert s.fast_dispatch(cmd.freedrive_command(True)) == "OK freedrive on"
    assert rob.calls == [True]


def test_turning_it_off_reaches_the_robot():
    rob = FakeRob()
    s = _server(rob)
    assert s.fast_dispatch(cmd.freedrive_command(False)) == "OK freedrive off"
    assert rob.calls == [False]


def test_turning_it_on_is_refused_while_busy():
    rob = FakeRob()
    s = _server(rob, busy=True)
    reply = s.fast_dispatch(cmd.freedrive_command(True))
    assert reply.startswith("ERROR")
    assert "busy" in reply
    assert rob.calls == [], "the robot was touched despite the refusal"


def test_turning_it_off_still_works_while_busy():
    # The recovery case: a run is holding the server busy and the operator
    # needs the arm stiff again.
    rob = FakeRob()
    s = _server(rob, busy=True)
    assert s.fast_dispatch(cmd.freedrive_command(False)) == "OK freedrive off"
    assert rob.calls == [False]


def test_bad_argument_is_rejected_without_touching_the_robot():
    rob = FakeRob()
    s = _server(rob)
    assert s.fast_dispatch("freedrive sideways").startswith("ERROR: usage")
    assert s.fast_dispatch("freedrive").startswith("ERROR: usage")
    assert s.fast_dispatch("freedrive on off").startswith("ERROR: usage")
    assert rob.calls == []


def test_simulate_does_not_touch_the_robot():
    rob = FakeRob()
    s = _server(rob, simulate=True)
    assert s.fast_dispatch(cmd.freedrive_command(True)) == "OK simulated freedrive on"
    assert rob.calls == []


def test_a_raising_robot_is_reported_not_propagated():
    rob = FakeRob(raise_on=RuntimeError("not in remote control"))
    s = _server(rob)
    reply = s.fast_dispatch(cmd.freedrive_command(True))
    assert reply.startswith("ERROR: could not turn freedrive on")
    assert "not in remote control" in reply
