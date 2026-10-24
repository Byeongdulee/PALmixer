"""PAL loading uses sample-slot arguments; fake motors only."""
import threading
from types import SimpleNamespace

import pytest

from palmixer import commands as cmd, config, state
from palmixer.server import PALmixerServer
from palmixer.workflows import Workflows, WorkflowError


@pytest.fixture
def loading(tmp_path, monkeypatch):
    geometry = dict(size=16, step=4500, preparation_offset_steps=-4)
    monkeypatch.setattr(config, "get_section", lambda section: geometry if section == "carousel" else {})
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))
    state.mount_carousel("carousel-A", {})
    state.teach_carousel_slot(1, 27000)
    state.set_carousel_slot(6)
    state.set_mixer_head(state.MIXER_AT_CLEANING)
    state.set_flowcell_in_use(1)
    state.set_flowcell_location(1, state.FC_AT_CLEANING)
    moves = []
    def move_to(position):
        assert state.get_carousel_slot() == 6
        moves.append(position)
        return position
    motor = SimpleNamespace(check_in_range=lambda position: None, move_to=move_to)
    wf = Workflows(None, None, None, motor)
    return wf, moves, geometry


@pytest.mark.parametrize("sample_slot,position_slot,target", [(8, 4, 40500), (16, 12, 76500), (1, 13, 81000)])
def test_public_function_applies_offset_and_keeps_sample_identity(loading, sample_slot, position_slot, target):
    wf, moves, _ = loading
    state.set_sample_id(sample_slot, "prepared-vial")
    assert wf.move_to_pal_loading(sample_slot) == target
    assert moves == [target]
    assert state.get_carousel_slot() == position_slot
    assert state.get_sample_id(sample_slot) == "prepared-vial"
    assert not state.get_sample_id(position_slot)


def test_function_reads_configured_offset(loading):
    wf, moves, geometry = loading
    geometry["preparation_offset_steps"] = -2
    assert wf.move_to_pal_loading(8) == 49500
    assert state.get_carousel_slot() == 6
    assert moves == [49500]


@pytest.mark.parametrize("method", ["move_to_pal_loading", "move_to_mixing"])
@pytest.mark.parametrize("slot", [0, 17, 8.5, True])
def test_invalid_slot_never_moves(loading, slot, method):
    wf, moves, _ = loading
    with pytest.raises((ValueError, WorkflowError)):
        getattr(wf, method)(slot)
    assert moves == []


@pytest.mark.parametrize("obstacle", ["mixer", "flowcell", "reference", "coordination", "offset"])
def test_loading_refuses_uncleared_or_unconfigured_equipment(loading, obstacle):
    wf, moves, geometry = loading
    if obstacle == "mixer":
        state.set_mixer_head(state.MIXER_AT_MIXER)
    elif obstacle == "flowcell":
        state.set_flowcell_location(1, state.UNKNOWN)
    elif obstacle == "reference":
        state.reset_carousel()
    elif obstacle == "coordination":
        wf.preparation = object()
    else:
        geometry.pop("preparation_offset_steps")
    with pytest.raises((ValueError, WorkflowError)):
        wf.move_to_pal_loading(8)
    assert moves == []


def test_failed_move_does_not_update_tracked_slot(loading):
    wf, moves, _ = loading
    def fail(position):
        raise RuntimeError("motor failed")
    wf.motor.move_to = fail
    with pytest.raises(RuntimeError, match="motor failed"):
        wf.move_to_pal_loading(8)
    assert state.get_carousel_slot() == 6


def test_simulation_does_not_touch_motor(loading):
    wf, moves, _ = loading
    wf.simulate = True
    wf.motor = None
    assert wf.move_to_pal_loading(8) == 40500
    assert state.get_carousel_slot() == 4
    assert wf.move_to_mixing(8) == 58500
    assert state.get_carousel_slot() == 8


@pytest.mark.parametrize("slot,loading_slot,mixing_target", [(8, 4, 58500), (16, 12, 94500), (1, 13, 27000)])
def test_loading_then_mixing_returns_to_sample_slot_without_offset(loading, slot, loading_slot, mixing_target):
    wf, moves, geometry = loading
    state.set_sample_id(slot, "my-vial")
    wf.move_to_pal_loading(slot)
    assert state.get_carousel_slot() == loading_slot
    def move_back(position):
        assert state.get_carousel_slot() == loading_slot
        moves.append(position)
    wf.motor.move_to = move_back
    geometry.pop("preparation_offset_steps")  # Return never needs the loading offset.
    assert wf.move_to_mixing(slot) == mixing_target
    assert moves[-1] == mixing_target
    assert state.get_carousel_slot() == slot
    assert state.get_sample_id(slot) == "my-vial"


def test_failed_return_retains_loading_position(loading):
    wf, moves, _ = loading
    wf.move_to_pal_loading(8)
    def fail(position):
        raise RuntimeError("return failed")
    wf.motor.move_to = fail
    with pytest.raises(RuntimeError, match="return failed"):
        wf.move_to_mixing(8)
    assert state.get_carousel_slot() == 4


@pytest.mark.parametrize("obstacle", ["mixer", "flowcell", "coordination"])
def test_return_keeps_clearance_and_coordination_guards(loading, obstacle):
    wf, moves, _ = loading
    if obstacle == "mixer":
        state.set_mixer_head(state.MIXER_AT_MIXER)
    elif obstacle == "flowcell":
        state.set_flowcell_location(1, state.UNKNOWN)
    else:
        wf.preparation = object()
    with pytest.raises(WorkflowError):
        wf.move_to_mixing(8)
    assert moves == []


def test_automatic_mixing_returns_from_pal_loading_before_head_descends(loading, monkeypatch):
    wf, moves, _ = loading
    wf.move_to_pal_loading(8)
    wf.motor.move_to = moves.append
    class StopBeforeMixing(Exception):
        pass
    def lower_head(name, action):
        assert name == "mixer2mixingstation"
        assert moves == [40500, 58500]
        assert state.get_carousel_slot() == 8
        raise StopBeforeMixing()
    monkeypatch.setattr(wf, "_move_mixer_head", lower_head)
    with pytest.raises(StopBeforeMixing):
        wf._make_sample(8)


@pytest.mark.parametrize("command,target,position_slot", [
    (cmd.MOVE_TO_PAL_LOADING, 40500, 4), (cmd.MOVE_TO_MIXING, 58500, 8),
])
def test_command_routes_to_function_and_busy_server_refuses(loading, command, target, position_slot):
    wf, moves, _ = loading
    server = object.__new__(PALmixerServer)
    server.workflows = wf
    server._busy_lock = threading.Lock()
    server._busy = True
    server._current_action = "other action"
    assert cmd.move_to_pal_loading_command(8) == "move_to_pal_loading 8"
    assert cmd.move_to_mixing_command(8) == "move_to_mixing 8"
    message = command + " 8"
    assert server.dispatch(message).startswith("ERROR: busy")
    assert moves == []
    server.preparation_api = SimpleNamespace(active=True)
    assert "owns the equipment" in server.dispatch(message)
    server.preparation_api.active = False
    action, label = server._resolve(command, ["8"])
    assert label == message
    assert "sample slot 8 (carousel slot %d" % position_slot in action()
    assert moves == [target]


@pytest.mark.parametrize("args", [[], ["8", "4"], ["eight"], ["8.5"], ["17"]])
@pytest.mark.parametrize("command", [cmd.MOVE_TO_PAL_LOADING, cmd.MOVE_TO_MIXING])
def test_command_validates_arguments_before_motion(loading, args, command):
    wf, moves, _ = loading
    server = object.__new__(PALmixerServer)
    server.workflows = wf
    with pytest.raises(ValueError):
        server._resolve(command, args)
    assert moves == []
