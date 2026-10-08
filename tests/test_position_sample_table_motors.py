"""The 'position sample_table motors' toggle.

Four transports drive the DAQ sample-stage motors (sth/sav/stv) to the recorded
position before approaching the sample table. This toggle gates that: on (the
default) keeps it, off leaves those EPICS motors untouched so a transport runs
even when the DAQ GUI is unreachable. These pin the gate helper, the state
persistence, and the server command.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
from palmixer import commands as cmd
from palmixer.server import PALmixerServer
PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")


@pytest.fixture
def daq_spy(monkeypatch):
    """Record whether the DAQ motors were read/aligned, without a DAQ GUI."""
    calls = {"get_pos": 0, "align": 0}

    def get_pos():
        calls["get_pos"] += 1
        return [1.0, 2.0, 3.0]

    def align():
        calls["align"] += 1
        return {"sth": 1.0, "sav": 2.0, "stv": 3.0}

    monkeypatch.setattr(PAL12idb.daq_client, "get_pos", get_pos)
    monkeypatch.setattr(PAL12idb.daq_client, "align_sample_table", align)
    return calls


@pytest.fixture(autouse=True)
def restore_flag():
    saved = PAL12idb.position_sample_table_motors
    yield
    PAL12idb.position_sample_table_motors = saved


def test_on_reads_and_aligns_and_returns_the_restore_position(daq_spy, monkeypatch):
    monkeypatch.setattr(PAL12idb, "position_sample_table_motors", True)
    restore_to = PAL12idb._align_sample_table_if_enabled()
    assert restore_to == [1.0, 2.0, 3.0]
    assert daq_spy == {"get_pos": 1, "align": 1}


def test_off_touches_nothing_and_returns_none(daq_spy, monkeypatch):
    monkeypatch.setattr(PAL12idb, "position_sample_table_motors", False)
    assert PAL12idb._align_sample_table_if_enabled() is None
    assert daq_spy == {"get_pos": 0, "align": 0}, "the motors were touched while off"


def test_restore_stage_is_a_noop_on_none(monkeypatch):
    # The partner call in every transport: when alignment was skipped there is
    # nothing to put back, and it must not call the DAQ at all.
    touched = []
    monkeypatch.setattr(PAL12idb.daq_client, "set_pos", lambda p: touched.append(p))
    PAL12idb._restore_stage(None, "a skipped transport")
    assert touched == []


# -- state persistence ------------------------------------------------------
def test_state_round_trips_and_defaults_on(tmp_path, monkeypatch):
    from palmixer import state
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "s.ini"))
    assert state.get_position_sample_table_motors() is True       # default
    state.set_position_sample_table_motors(False)
    assert state.get_position_sample_table_motors() is False
    state.set_position_sample_table_motors(True)
    assert state.get_position_sample_table_motors() is True


def test_snapshot_carries_the_flag(tmp_path, monkeypatch):
    from palmixer import state
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "s.ini"))
    state.set_position_sample_table_motors(False)
    assert state.snapshot()["position_sample_table_motors"] is False


# -- server command ---------------------------------------------------------
def _server():
    s = object.__new__(PALmixerServer)
    s.simulate = False
    s.PAL12idb = PAL12idb
    return s


def test_command_builder():
    assert cmd.set_position_sample_table_motors_command(True) == \
        "set_position_sample_table_motors on"
    assert cmd.set_position_sample_table_motors_command(False) == \
        "set_position_sample_table_motors off"


def test_server_rejects_a_bad_argument():
    assert _server().fast_dispatch(
        "set_position_sample_table_motors maybe").startswith("ERROR: usage")


def test_server_sets_the_flag_through_pal12idb(restore_flag, monkeypatch, tmp_path):
    from palmixer import state
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "s.ini"))
    s = _server()
    assert s.fast_dispatch("set_position_sample_table_motors off").startswith("OK")
    assert PAL12idb.position_sample_table_motors is False
    assert state.get_position_sample_table_motors() is False
    assert s.fast_dispatch("set_position_sample_table_motors on").startswith("OK")
    assert PAL12idb.position_sample_table_motors is True
