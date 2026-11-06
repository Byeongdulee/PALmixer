"""The RedBoard trigger: its supervisor (palmixer/redboard.py), the two hooks
that restart it as the mixer head sets off for the carousel, and make_sample
refusing to mix without it.

The supervisor tests launch real child processes -- stand-in scripts that
print what redboard_trigger.py prints -- and never the real script, which
would take the serial port from a real board.
"""
import os
import subprocess
import sys
import textwrap
import threading
import time
from types import SimpleNamespace

import pytest

from palmixer import config, redboard, state, workflows
from palmixer.redboard import RedBoardTrigger, TriggerNotArmed
from palmixer.server import PALmixerServer
from palmixer.workflows import Workflows, WorkflowError

ARMED = "ARMED: port=COM99 threshold=3.00V delay=0.26s pulse=0.39s lockout=10.00s locked=%d"


def _armed(locked=0, volts="0.10", port="COM99"):
    """What the script prints on arming: the board's status reply, then ARMED."""
    return ('print("A0: %sV, locked=%d", flush=True)\nprint("%s", flush=True)\n'
            % (volts, locked, (ARMED % locked).replace("COM99", port)))


POLLING = """
while True:
    print("A0: 0.10V, locked=0", flush=True)
    time.sleep(0.2)
"""

ARMED_AND_POLLING = 'print("Probing COM99...", flush=True)\n' + _armed() + POLLING

NO_BOARD = """
print("Probing COM99...", flush=True)
print("Error: No board running the trigger sketch found.", file=sys.stderr, flush=True)
sys.exit(1)
"""

BROKEN_EDIT = """
def configure(:
    pass
"""

LOCKED_THEN_READY = _armed(locked=1) + """
for _ in range(4):
    print("A0: 0.10V, locked=1", flush=True)
    time.sleep(0.2)
print("READY: monitoring A0", flush=True)
""" + POLLING

# A0 held above the threshold: the level-triggered firmware fires again the
# moment each lockout ends, so READY is followed within a millisecond by the
# next TRIGGER -- here in a single write, and with a status reply squeezed in.
STUCK_HIGH_ONE_WRITE = _armed(locked=1, volts="3.40") + """
while True:
    time.sleep(0.3)
    sys.stdout.write("READY: monitoring A0\\nTRIGGER: 3.40V\\n")
    sys.stdout.flush()
    print("LOCKOUT: 10.00s", flush=True)
    print("A0: 3.40V, locked=1", flush=True)
"""

STUCK_HIGH_WITH_A_POLL = _armed(locked=1, volts="3.40") + """
while True:
    time.sleep(0.3)
    print("READY: monitoring A0", flush=True)
    print("A0: 3.40V, locked=0", flush=True)
    time.sleep(0.001)
    print("TRIGGER: 3.40V", flush=True)
    print("LOCKOUT: 10.00s", flush=True)
"""

HIGH_BUT_UNLOCKED = _armed(locked=0, volts="3.40") + """
while True:
    print("A0: 3.40V, locked=0", flush=True)
    time.sleep(0.2)
"""

STILL_SEARCHING = """
print("Probing COM99...", flush=True)
time.sleep(60)
"""

ARMED_THEN_SILENT = _armed() + "time.sleep(60)\n"

LOSES_THE_PORT = _armed() + """
time.sleep(0.3)
print("DISCONNECTED: device reports readiness to read but returned no data", flush=True)
time.sleep(0.6)
""" + _armed(port="COM18") + POLLING

TRIGGERS = _armed() + """
print("TRIGGER: 3.21V", flush=True)
print("LOCKOUT: 10.00s", flush=True)
time.sleep(60)
"""

TAKEN_OVER = _armed() + """
import os
time.sleep(0.5)
os._exit(15)
"""


def _settings(tmp_path, body, **overrides):
    script = tmp_path / "stand_in_trigger.py"
    script.write_text("import sys, time\n" + textwrap.dedent(body))
    settings = {"enabled": True, "script": str(script), "python": sys.executable,
                "port": "", "armed_timeout_s": 5.0,
                "log_file": str(tmp_path / "logs" / "redboard.log")}
    settings.update(overrides)
    return settings


@pytest.fixture
def supervised(tmp_path):
    """make(body, **overrides) -> (trigger, settings): a RedBoardTrigger
    running `body` as its script; every one made is stopped at the end."""
    made = []

    def make(body, **overrides):
        settings = _settings(tmp_path, body, **overrides)
        trigger = RedBoardTrigger(settings=lambda: settings)
        made.append(trigger)
        return trigger, settings

    yield make
    for trigger in made:
        trigger.stop()


def _wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("timed out waiting")
        time.sleep(0.02)


# -- the supervisor ------------------------------------------------------------
def test_restart_launches_the_script_and_reports_it_armed(supervised, tmp_path):
    trigger, _ = supervised(ARMED_AND_POLLING)
    trigger.restart("mixer2mixingstation")
    detail = trigger.await_armed(10)
    assert "port=COM99" in detail and "A0 0.10 V, not locked out" in detail
    assert "locked=" not in detail, "the stale locked= of the ARMED line"
    snapshot = trigger.snapshot()
    assert snapshot["state"] == redboard.ARMED and snapshot["ready"]
    assert snapshot["port"] == "COM99" and snapshot["locked"] is False
    assert snapshot["threshold_volts"] == 3.0
    assert snapshot["restarts"] == 1 and snapshot["pid"]
    log = (tmp_path / "logs" / "redboard.log").read_text()
    assert "restart for mixer2mixingstation" in log and "ARMED:" in log


def test_each_restart_replaces_the_previous_instance(supervised):
    trigger, _ = supervised(ARMED_AND_POLLING)
    trigger.restart("first move")
    trigger.await_armed(10)
    first = trigger._proc
    trigger.restart("second move")
    trigger.await_armed(10)
    assert trigger._proc is not first
    assert first.poll() is not None, "the previous instance was left running"
    assert trigger.snapshot()["restarts"] == 2


def test_a_script_that_exits_fails_fast_with_its_reason(supervised):
    trigger, _ = supervised(NO_BOARD)
    trigger.restart("mixer2mixingstation")
    started = time.monotonic()
    with pytest.raises(TriggerNotArmed) as caught:
        trigger.await_armed(30)
    assert time.monotonic() - started < 10, "a script that has died should not be waited on"
    assert "exited with code 1" in str(caught.value)
    assert "No board running the trigger sketch found" in str(caught.value)
    assert trigger.snapshot()["state"] == redboard.FAILED


def test_a_crash_names_its_exception(supervised):
    """A bad edit to the script fails make_sample; the reason must say why."""
    trigger, _ = supervised(BROKEN_EDIT)
    trigger.restart("mixer2mixingstation")
    with pytest.raises(TriggerNotArmed, match="exited with code 1: SyntaxError"):
        trigger.await_armed(30)


def test_a_copy_taken_over_by_another_says_so(supervised):
    trigger, _ = supervised(TAKEN_OVER)
    trigger.restart("mixer2mixingstation")
    trigger.await_armed(10)
    _wait_for(lambda: trigger.snapshot()["state"] == redboard.FAILED)
    assert "stopped by another copy" in trigger.snapshot()["detail"]


def test_a_lockout_is_waited_out(supervised):
    """Opening the port does not reset the board, so a lockout already running
    survives the restart; the board is not ready until it has ended."""
    trigger, _ = supervised(LOCKED_THEN_READY)
    trigger.restart("mixer2mixingstation")
    _wait_for(lambda: trigger.snapshot()["state"] == redboard.ARMED)
    assert not trigger.is_armed(), "configured but locked out is not ready"
    trigger.await_armed(10)
    assert trigger.snapshot()["locked"] is False


@pytest.mark.parametrize("body", [STUCK_HIGH_ONE_WRITE, STUCK_HIGH_WITH_A_POLL],
                         ids=["ready-and-trigger-in-one-write", "a-poll-between"])
def test_an_input_stuck_high_never_reads_as_armed(supervised, body):
    """With A0 above the threshold the board refires the instant each lockout
    ends; the READY in between must not let the mix through."""
    trigger, _ = supervised(body)
    trigger.restart("mixer2mixingstation")
    with pytest.raises(TriggerNotArmed, match="threshold|lockout|status report"):
        trigger.await_armed(2.5)
    assert trigger.snapshot()["last_trigger"] is not None


def test_an_input_above_the_threshold_is_named(supervised):
    trigger, _ = supervised(HIGH_BUT_UNLOCKED)
    trigger.restart("mixer2mixingstation")
    with pytest.raises(TriggerNotArmed, match="A0 reads 3.40 V, at or above the 3.00 V threshold"):
        trigger.await_armed(1)


def test_an_arming_that_never_comes_times_out(supervised):
    trigger, _ = supervised(STILL_SEARCHING)
    trigger.restart("mixer2mixingstation")
    with pytest.raises(TriggerNotArmed, match="has not confirmed its configuration"):
        trigger.await_armed(0.5)


def test_a_board_gone_quiet_is_not_armed(supervised, monkeypatch):
    monkeypatch.setattr(redboard, "STALE_AFTER_S", 1.0)
    trigger, _ = supervised(ARMED_THEN_SILENT)
    trigger.restart("mixer2mixingstation")
    trigger.await_armed(10)
    time.sleep(1.3)
    assert not trigger.is_armed()
    with pytest.raises(TriggerNotArmed, match="no word from the board"):
        trigger.await_armed(0.3)


def test_a_lost_port_is_not_armed_until_the_script_rearms(supervised):
    trigger, _ = supervised(LOSES_THE_PORT)
    trigger.restart("mixer2mixingstation")
    trigger.await_armed(10)
    _wait_for(lambda: not trigger.is_armed())
    assert "reconnecting" in trigger.snapshot()["detail"]
    assert "port=COM18" in trigger.await_armed(10)


def test_triggers_are_recorded(supervised):
    trigger, _ = supervised(TRIGGERS)
    trigger.restart("mixer2mixingstation")
    _wait_for(lambda: trigger.snapshot()["last_trigger"] is not None)
    snapshot = trigger.snapshot()
    assert snapshot["last_trigger"]["volts"] == 3.21
    assert snapshot["locked"] is True and not snapshot["ready"]


def test_turning_it_off_stops_the_script_and_frees_the_port(supervised, monkeypatch):
    monkeypatch.setattr(redboard, "WATCH_INTERVAL_S", 0.1)
    trigger, settings = supervised(ARMED_AND_POLLING)
    trigger.restart("mixer2mixingstation")
    trigger.await_armed(10)
    child = trigger._proc
    settings["enabled"] = False
    _wait_for(lambda: child.poll() is not None)
    assert trigger.snapshot()["state"] == redboard.OFF
    settings["enabled"] = True                # and on again, at the next move
    trigger.restart("mixer2mixingstation")
    trigger.await_armed(10)


def test_stop_ends_the_child_for_good(supervised):
    trigger, _ = supervised(ARMED_AND_POLLING)
    trigger.restart("mixer2mixingstation")
    trigger.await_armed(10)
    child = trigger._proc
    trigger.stop()
    assert child.poll() is not None
    trigger.restart("a move during shutdown")
    time.sleep(0.3)
    assert trigger._proc is child, "a child was launched after stop()"
    assert trigger.snapshot()["state"] == redboard.STOPPED
    with pytest.raises(TriggerNotArmed):
        trigger.await_armed(0.1)


def test_the_log_is_rotated_between_sessions(supervised, tmp_path, monkeypatch):
    monkeypatch.setattr(redboard, "LOG_MAX_BYTES", 100)
    log = tmp_path / "logs" / "redboard.log"
    log.parent.mkdir()
    log.write_text("x" * 200)
    (tmp_path / "logs" / "redboard.log.1").write_text("older")
    trigger, _ = supervised(ARMED_AND_POLLING)
    trigger.restart("mixer2mixingstation")
    trigger.await_armed(10)
    assert (tmp_path / "logs" / "redboard.log.1").read_text() == "x" * 200
    assert (tmp_path / "logs" / "redboard.log.2").read_text() == "older"
    assert "ARMED:" in log.read_text()


@pytest.mark.skipif(sys.platform != "win32", reason="Windows file sharing")
def test_a_log_held_open_elsewhere_is_left_alone(supervised, tmp_path, monkeypatch):
    """A program holding the log without delete sharing makes the rotation
    impossible; the backups must not be shifted (and lost) regardless."""
    monkeypatch.setattr(redboard, "LOG_MAX_BYTES", 100)
    logs = tmp_path / "logs"
    logs.mkdir()
    (logs / "redboard.log").write_text("x" * 200)
    for n in range(1, 6):
        (logs / ("redboard.log.%d" % n)).write_text("backup %d" % n)
    holder = subprocess.Popen(
        [sys.executable, "-c", "import sys, time; f = open(sys.argv[1]); print('open', flush=True); "
                               "time.sleep(30)", str(logs / "redboard.log")],
        stdout=subprocess.PIPE, text=True)
    try:
        assert holder.stdout.readline().strip() == "open"
        trigger, _ = supervised(ARMED_AND_POLLING)
        trigger.restart("mixer2mixingstation")
        trigger.await_armed(10)
        for n in range(1, 6):
            assert (logs / ("redboard.log.%d" % n)).read_text() == "backup %d" % n
        assert (logs / "redboard.log").read_text().startswith("x" * 200)
    finally:
        holder.kill()
        holder.wait()


def test_a_log_that_cannot_be_written_does_not_stop_arming(supervised, tmp_path):
    (tmp_path / "not_a_folder").write_text("")
    trigger, _ = supervised(ARMED_AND_POLLING,
                            log_file=str(tmp_path / "not_a_folder" / "redboard.log"))
    trigger.restart("mixer2mixingstation")
    trigger.await_armed(10)


def test_off_or_simulated_never_launches():
    launched = []

    def popen(*args, **kwargs):
        launched.append(args)

    off = RedBoardTrigger(settings=lambda: {"enabled": False}, popen=popen)
    simulated = RedBoardTrigger(simulate=True, settings=lambda: {"enabled": True}, popen=popen)
    off.restart("mixer2mixingstation")
    simulated.restart("mixer2mixingstation")
    time.sleep(0.2)
    assert launched == []
    assert not off.enabled and not simulated.enabled
    assert simulated.snapshot()["state"] == redboard.OFF


def test_a_launch_failure_is_recorded_not_raised(tmp_path):
    """restart() runs on the thread driving the robot: whatever goes wrong
    starting the script must come back through await_armed(), not as an
    exception in the arm's way."""
    def popen(*args, **kwargs):
        raise OSError("interpreter not found")

    settings = _settings(tmp_path, "pass")
    trigger = RedBoardTrigger(settings=lambda: settings, popen=popen)
    trigger.restart("mixer2mixingstation")
    with pytest.raises(TriggerNotArmed, match="could not start .*interpreter not found"):
        trigger.await_armed(5)
    trigger.stop()


def test_a_missing_script_is_reported(tmp_path):
    settings = _settings(tmp_path, "pass", script=str(tmp_path / "missing.py"))
    trigger = RedBoardTrigger(settings=lambda: settings)
    trigger.restart("mixer2mixingstation")
    with pytest.raises(TriggerNotArmed, match="no RedBoard script at"):
        trigger.await_armed(5)
    trigger.stop()


def test_the_command_line(tmp_path):
    """The script's own restart-after-trigger is turned off -- restarting is
    the supervisor's job -- and a relative script path is the repo copy."""
    trigger = RedBoardTrigger(settings=lambda: {})
    script = tmp_path / "stand_in_trigger.py"
    script.write_text("pass")
    argv, cwd = trigger._command({"script": str(script), "python": "py.exe", "port": "COM17"})
    assert argv == ["py.exe", "-u", str(script), "--restart-after-trigger", "0", "--port", "COM17"]
    assert cwd == str(tmp_path)
    argv, _ = trigger._command({})
    assert argv[0] == sys.executable
    assert argv[2] == os.path.join(redboard._ROOT_DIR, "redboard", "redboard_trigger.py")


# -- the config ------------------------------------------------------------------
def test_the_config_keeps_it_off_unless_turned_on(monkeypatch):
    monkeypatch.delenv("PALMIXER_REDBOARD_ENABLED", raising=False)
    monkeypatch.setattr(config, "_load_file", lambda: {})
    assert config.get_section("redboard")["enabled"] is False
    monkeypatch.setenv("PALMIXER_REDBOARD_ENABLED", "1")
    assert config.get_section("redboard")["enabled"] is True
    monkeypatch.setattr(config, "_load_file", lambda: {"redboard": {"enabled": True}})
    monkeypatch.setenv("PALMIXER_REDBOARD_ENABLED", "0")
    assert config.get_section("redboard")["enabled"] is False


def test_a_config_that_stops_parsing_keeps_its_last_good_settings(tmp_path, monkeypatch):
    """A typo, or a save caught half-written, must not quietly switch off the
    check make_sample makes before it mixes."""
    path = tmp_path / "palmixer_config.json"
    monkeypatch.setattr(config, "_CONFIG_PATH", str(path))
    monkeypatch.setattr(config, "_last_good_text", None)
    monkeypatch.setattr(config, "_last_warning", None)
    monkeypatch.delenv("PALMIXER_REDBOARD_ENABLED", raising=False)
    path.write_text('{"redboard": {"enabled": true}}')
    assert config.get_section("redboard")["enabled"] is True
    path.write_text('{"redboard": {"enabled": true,}}')
    assert config.get_section("redboard")["enabled"] is True
    path.write_text("")
    assert config.get_section("redboard")["enabled"] is True
    path.write_text('{"redboard": {"enabled": false}}')
    assert config.get_section("redboard")["enabled"] is False


# -- the workflow and the server -------------------------------------------------
class FakeTrigger:
    """Stands in for RedBoardTrigger. `armed`: ready right now. `failure`:
    what await_armed() raises; None means an arming in progress that completes."""

    def __init__(self, enabled=True, armed=True, failure=None, events=None):
        self.enabled, self.armed, self.failure = enabled, armed, failure
        self.events = events if events is not None else []

    def restart(self, reason):
        self.events.append(("restart", reason))

    def is_armed(self):
        return self.armed

    def await_armed(self, timeout_s=None):
        self.events.append(("await_armed",))
        if self.failure:
            raise TriggerNotArmed(self.failure)
        self.armed = True
        return "port=COM99; A0 0.10 V, not locked out"


class StopAtMix(Exception):
    """Raised by the fake pump's mix: everything these tests check happens before it."""


@pytest.fixture
def make_sample(tmp_path, monkeypatch):
    """build(trigger) -> a Workflows that runs make_sample's real (not
    simulated) branch on fakes, as far as the mix; and the event list every
    fake -- steps, moves, the mix, the trigger -- appends to."""
    geometry = dict(size=16, step=4500, draw_offset_steps=4)
    monkeypatch.setattr(config, "get_section",
                        lambda section: geometry if section == "carousel" else {})
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))
    state.mount_carousel("carousel-A", {})
    state.teach_carousel_slot(1, 27000)
    state.set_carousel_slot(1)
    state.set_mixer_head(state.MIXER_AT_CLEANING)
    state.set_flowcell_in_use(1)
    state.set_flowcell_location(1, state.FC_AT_CLEANING)
    events = []

    def mix():
        events.append(("pump", "mix"))
        raise StopAtMix()

    pal = SimpleNamespace(
        mixer2mixingstation=lambda robot: events.append(("move", "mixer2mixingstation")),
        mixer2cleaningstation=lambda robot: events.append(("move", "mixer2cleaningstation")))
    motor = SimpleNamespace(check_in_range=lambda position: None, move_to=lambda position: None)

    def build(trigger):
        if trigger is not None:
            trigger.events = events
        return Workflows(pal, None, SimpleNamespace(mix=mix), motor, trigger=trigger,
                         on_step=lambda step, phase, detail="": events.append((step, phase)))

    return build, events


def test_make_sample_restarts_the_trigger_as_the_head_sets_off(make_sample):
    build, events = make_sample
    with pytest.raises(StopAtMix):
        build(FakeTrigger(armed=True))._make_sample(8)
    assert events.count(("restart", "mixer2mixingstation")) == 1
    assert (events.index(("restart", "mixer2mixingstation"))
            < events.index(("mixer2mixingstation", "started"))
            < events.index(("move", "mixer2mixingstation")))
    assert ("pump", "mix") in events
    # Armed already -- the usual case -- so no step of its own and no wait.
    assert ("wait_for_redboard", "started") not in events
    assert ("await_armed",) not in events


def test_make_sample_will_not_mix_without_an_armed_trigger(make_sample):
    build, events = make_sample
    trigger = FakeTrigger(armed=False, failure="the RedBoard script (pid 7) exited "
                                               "with code 1: No board found")
    with pytest.raises(WorkflowError) as caught:
        build(trigger)._make_sample(8)
    message = str(caught.value)
    assert "not armed" in message and "No board found" in message
    assert "Stopped before mixing" in message and "run make_sample again" in message
    assert ("pump", "mix") not in events, "mixed without the trigger"
    assert (events.index(("move", "mixer2mixingstation"))
            < events.index(("wait_for_redboard", "started"))
            < events.index(("wait_for_redboard", "failure")))
    assert not state.get_sample_id(8), "a refused mix must leave the slot untagged"


def test_a_coordinated_refusal_says_how_to_recover(make_sample):
    """A failed coordinated make ends the PALsystem run; make_sample cannot
    simply be repeated until that run is cleared."""
    build, _ = make_sample
    wf = build(FakeTrigger(armed=False, failure="no board"))
    wf.preparation = object()
    with pytest.raises(WorkflowError, match="clear the interrupted run in PALsystem"):
        wf._wait_trigger_armed()


def test_an_arming_still_in_progress_is_waited_for(make_sample):
    build, events = make_sample
    with pytest.raises(StopAtMix):
        build(FakeTrigger(armed=False))._make_sample(8)
    assert events.index(("wait_for_redboard", "success")) < events.index(("pump", "mix"))


def test_a_trigger_switched_off_never_holds_the_mix(make_sample):
    build, events = make_sample
    with pytest.raises(StopAtMix):
        build(FakeTrigger(enabled=False, armed=False))._make_sample(8)
    assert ("await_armed",) not in events and ("pump", "mix") in events


def test_without_a_trigger_make_sample_is_unchanged(make_sample):
    build, events = make_sample
    with pytest.raises(StopAtMix):
        build(None)._make_sample(8)
    assert ("pump", "mix") in events


def test_only_the_move_to_the_carousel_restarts_the_trigger():
    events = []
    wf = Workflows(None, None, None, None, trigger=FakeTrigger(events=events))
    wf._move_mixer_head("mixer2cleaningstation", lambda: "parked")
    assert events == []
    wf._move_mixer_head("mixer2mixingstation", lambda: "seated")
    assert events == [("restart", "mixer2mixingstation")]


def test_the_restart_waits_out_a_wash_like_the_move(monkeypatch):
    """A wash can hold the head for minutes; the restart belongs to the move
    actually starting, not to the order for it."""
    monkeypatch.setattr(workflows, "MIXER_PUMP_POLL_S", 0.05)
    washing = [True]
    dashboard = SimpleNamespace(
        _op_timeout_s=5.0,
        status_snapshot=lambda: {"servers": {"mixer": {"operation_active": washing[0]}}})
    events = []
    wf = Workflows(None, None, dashboard, None, trigger=FakeTrigger(events=events),
                   on_step=lambda step, phase, detail="": events.append((step, phase)))
    mover = threading.Thread(target=wf._move_mixer_head,
                             args=("mixer2mixingstation", lambda: "seated"), daemon=True)
    mover.start()
    time.sleep(0.3)
    assert ("restart", "mixer2mixingstation") not in events, "restarted mid-wash"
    washing[0] = False
    mover.join(5)
    assert (events.index(("wait_for_mixer_pumps", "success"))
            < events.index(("restart", "mixer2mixingstation"))
            < events.index(("mixer2mixingstation", "started")))


def test_the_experiment_tab_button_restarts_the_trigger_before_moving():
    events = []
    server = object.__new__(PALmixerServer)
    server.simulate = False
    server.rob = None
    server.workflows = Workflows(None, None, None, None, trigger=FakeTrigger(events=events))
    server.PAL12idb = SimpleNamespace(
        mixer2mixingstation=lambda rob: events.append(("move", "mixer2mixingstation")),
        mixer2cleaningstation=lambda rob: events.append(("move", "mixer2cleaningstation")))
    server._run_transport("mixer2cleaningstation")
    server._run_transport("mixer2mixingstation")
    assert events == [("move", "mixer2cleaningstation"),
                      ("restart", "mixer2mixingstation (Experiment tab)"),
                      ("move", "mixer2mixingstation")]


def test_server_stop_stops_the_trigger():
    def bare_server():
        server = object.__new__(PALmixerServer)
        server._stopping = threading.Event()
        server._mix_sync_wake = threading.Event()
        server.zmq_server = SimpleNamespace(stop=lambda: None)
        server._mqtt = SimpleNamespace(close=lambda: None)
        return server

    stopped = []
    server = bare_server()
    server.redboard = SimpleNamespace(stop=lambda: stopped.append(True))
    server.stop()
    assert stopped == [True]
    bare_server().stop()        # one built without a trigger stops cleanly too
