"""redboard/redboard_trigger.py, the RedBoard's host script: everything that
does not need a board.

Nothing here opens a serial port, and nothing calls main() or
stop_other_instances(): run for real, those would take the port from a real
board and stop a copy someone is running.
"""
import importlib.util
import os
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("serial")

_SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "redboard", "redboard_trigger.py")

UPDATED = "UPDATED: Threshold=3.00V, Delay=0.26s, OpTime=0.39s, Lockout=10.00s"
CONFIG_LINE = b"3.0,0.26,0.39,10.0\n"


@pytest.fixture(scope="module")
def rt():
    spec = importlib.util.spec_from_file_location("redboard_trigger", _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeSerial:
    """Answers each readline() with the next scripted reply -- a str is a whole
    line, bytes are returned as they are -- then b"" (a read timeout)."""

    def __init__(self, replies, port="COM99"):
        self.replies = list(replies)
        self.written = []
        self.port = port
        self.closed = False

    def write(self, data):
        self.written.append(data)

    def readline(self):
        if not self.replies:
            return b""
        reply = self.replies.pop(0)
        return reply if isinstance(reply, bytes) else (reply + "\n").encode()

    def close(self):
        self.closed = True


# -- whole lines ---------------------------------------------------------------
def test_half_lines_are_joined_before_they_are_read(rt):
    """readline() can time out mid-line; a split TRIGGER must still read as one."""
    board = rt.Board(FakeSerial([b"TRIG", b"", b"GER: 3.12V\r\n"]))
    assert [board.line() for _ in range(3)] == ["", "", "TRIGGER: 3.12V"]


def test_stray_bytes_stay_printable(rt):
    assert rt.Board(FakeSerial([b"A0: 0.10V\xff\n"])).line() == "A0: 0.10V\\xff"


# -- configuring ---------------------------------------------------------------
def test_configure_confirms_what_the_board_took(rt):
    conn = FakeSerial([UPDATED, "A0: 0.12V, locked=0"])
    assert rt.configure(rt.Board(conn)) is False
    assert conn.written == [CONFIG_LINE, b"?\n"]


def test_configure_reports_a_lockout_in_progress(rt):
    assert rt.configure(rt.Board(FakeSerial([UPDATED, "A0: 3.40V, locked=1"]))) is True


def test_lines_ahead_of_the_acknowledgement_are_passed_on(rt, capsys):
    rt.configure(rt.Board(FakeSerial(["TRIGGER: 3.20V", UPDATED, "A0: 0.12V, locked=1"])))
    assert "TRIGGER: 3.20V" in capsys.readouterr().out


def test_an_unanswered_configuration_is_sent_once_more(rt, monkeypatch):
    monkeypatch.setattr(rt, "CONFIG_TIMEOUT", 0.05)

    class AnswersTheSecond(FakeSerial):
        def write(self, data):
            super().write(data)
            if self.written.count(CONFIG_LINE) == 2:
                self.replies += [UPDATED, "A0: 0.12V, locked=0"]

    conn = AnswersTheSecond([])
    assert rt.configure(rt.Board(conn)) is False
    assert conn.written == [CONFIG_LINE, CONFIG_LINE, b"?\n"]


def test_a_missing_acknowledgement_is_an_error(rt, monkeypatch):
    monkeypatch.setattr(rt, "CONFIG_TIMEOUT", 0.05)
    conn = FakeSerial([])
    with pytest.raises(RuntimeError, match="did not acknowledge"):
        rt.configure(rt.Board(conn))
    assert conn.written == [CONFIG_LINE, CONFIG_LINE]


def test_a_different_configuration_is_an_error(rt):
    # The sketch's own defaults: what a board that missed the line would hold.
    with pytest.raises(RuntimeError, match="different configuration"):
        rt.configure(rt.Board(FakeSerial(["UPDATED: Threshold=2.50V, Delay=0.10s, "
                                          "OpTime=0.40s, Lockout=10.00s"])))


def test_armed_line_names_the_port_and_the_settings(rt, monkeypatch, capsys):
    monkeypatch.setattr(rt, "open_board", lambda port, timeout_s: rt.Board(FakeSerial(
        [UPDATED, "A0: 0.12V, locked=0"], port="COM17")))
    rt.connect_and_arm(None, 1)
    assert ("ARMED: port=COM17 threshold=3.00V delay=0.26s pulse=0.39s "
            "lockout=10.00s locked=0") in capsys.readouterr().out


def test_a_failed_configuration_closes_the_port(rt, monkeypatch):
    conn = FakeSerial([])
    monkeypatch.setattr(rt, "CONFIG_TIMEOUT", 0.05)
    monkeypatch.setattr(rt, "open_board", lambda port, timeout_s: rt.Board(conn))
    with pytest.raises(RuntimeError):
        rt.connect_and_arm(None, 1)
    assert conn.closed


# -- one copy at a time ------------------------------------------------------------
@pytest.mark.parametrize("cmdline,script", [
    (["python", "redboard_trigger.py"], "redboard_trigger.py"),
    (["python", "-u", "-X", "utf8", "run.py", "--port", "COM17"], "run.py"),
    (["python", "-Xutf8", "-W", "ignore", "-uB", "run.py"], "run.py"),
    (["python", "-I", "--", "run.py"], "run.py"),
    (["python", "-c", "import run"], None),
    (["python", "-uc", "import run"], None),
    (["pythonw", "-m", "idlelib", "run.py"], None),
    (["python"], None),
])
def test_the_script_a_python_command_line_runs(rt, cmdline, script):
    assert rt.script_argument(cmdline) == script


@pytest.mark.parametrize("name,cmdline,expected", [
    ("python.exe", ["python", "redboard_trigger.py"], True),
    ("python.exe", ["C:\\env\\python.exe", "-u", "C:\\PALmixer\\redboard\\redboard_trigger.py",
                    "--restart-after-trigger", "0"], True),
    ("pythonw.exe", ["pythonw", "C:/Temp/adruino/REDBOARD_TRIGGER.PY"], True),
    # Editors that only have the file open are left alone.
    ("Code.exe", ["Code.exe", "C:\\PALmixer\\redboard\\redboard_trigger.py"], False),
    ("pythonw.exe", ["pythonw.exe", "-m", "idlelib", "C:\\PALmixer\\redboard\\redboard_trigger.py"],
     False),
    ("pythonw.exe", ["pythonw.exe", "C:\\Anaconda3\\Scripts\\spyder-script.py",
                     "C:\\PALmixer\\redboard\\redboard_trigger.py"], False),
    ("cmd.exe", ["cmd.exe", "/c", "python", "redboard_trigger.py"], False),
    ("python.exe", ["python", "-m", "pytest", "tests/test_redboard_script.py"], False),
    ("python.exe", ["python", "redboard_trigger.py.bak"], False),
    (None, None, False),
])
def test_which_processes_count_as_another_copy(rt, name, cmdline, expected):
    assert rt.is_other_instance(name, cmdline, "redboard_trigger.py") is expected


class FakeProcess:
    def __init__(self, pid, name, cmdline, wait_error=None):
        self.pid = pid
        self.info = {"pid": pid, "name": name, "cmdline": cmdline}
        self.wait_error = wait_error
        self.calls = []

    def terminate(self):
        self.calls.append("terminate")

    def kill(self):
        self.calls.append("kill")

    def wait(self, timeout=None):
        self.calls.append("wait")
        if self.wait_error is not None and self.calls.count("wait") == 1:
            raise self.wait_error
        return 0


def fake_psutil(processes, me=100, parents=(50,)):
    """Just enough of psutil for stop_other_instances()."""
    class Error(Exception):
        pass

    class NoSuchProcess(Error):
        pass

    class AccessDenied(Error):
        pass

    class TimeoutExpired(Error):
        pass

    return SimpleNamespace(
        Error=Error, NoSuchProcess=NoSuchProcess, AccessDenied=AccessDenied,
        TimeoutExpired=TimeoutExpired, process_iter=lambda attrs: list(processes),
        Process=lambda: SimpleNamespace(
            pid=me, parents=lambda: [SimpleNamespace(pid=p) for p in parents]))


def test_the_takeover_stops_other_running_copies_only(rt):
    running = ["python", "redboard_trigger.py"]
    me, parent = FakeProcess(100, "python.exe", running), FakeProcess(50, "python.exe", running)
    copy = FakeProcess(200, "python.exe", running)
    idle = FakeProcess(300, "pythonw.exe", ["pythonw", "-m", "idlelib", "redboard_trigger.py"])
    rt.stop_other_instances(fake_psutil([me, parent, copy, idle]))
    assert copy.calls == ["terminate", "wait"]
    assert me.calls == parent.calls == idle.calls == []


def test_a_copy_that_cannot_be_stopped_is_reported_not_fatal(rt, capsys):
    """One started as administrator: psutil may not even wait on it."""
    ps = fake_psutil([])
    elevated = FakeProcess(200, "python.exe", ["python", "redboard_trigger.py"],
                           wait_error=ps.AccessDenied("pid 200"))
    ps.process_iter = lambda attrs: [elevated]
    rt.stop_other_instances(ps)
    assert "Close it by hand" in capsys.readouterr().out


def test_a_copy_that_ignores_terminate_is_killed(rt):
    ps = fake_psutil([])
    stubborn = FakeProcess(200, "python.exe", ["python", "redboard_trigger.py"],
                           wait_error=ps.TimeoutExpired(5))
    ps.process_iter = lambda attrs: [stubborn]
    rt.stop_other_instances(ps)
    assert stubborn.calls == ["terminate", "wait", "kill", "wait"]


# -- finding the board -----------------------------------------------------------


def test_only_board_like_ports_are_probed(rt, monkeypatch):
    """A probe writes to the port; the pumps' RS-232 adapters must not get it."""
    ports = [
        SimpleNamespace(device="COM14", description="Prolific PL2303GS USB Serial COM Port (COM14)",
                        manufacturer="Prolific", vid=0x067B),
        SimpleNamespace(device="COM17", description="Arduino Leonardo ETH (COM17)",
                        manufacturer="Arduino Srl (www.arduino.org)", vid=0x2A03),
        SimpleNamespace(device="COM5", description="USB Serial Port (COM5)",
                        manufacturer="FTDI", vid=0x0403),
        SimpleNamespace(device="COM20", description="Communications Port (COM20)",
                        manufacturer=None, vid=0x1B4F),
    ]
    monkeypatch.setattr(rt.serial.tools.list_ports, "comports", lambda: ports)
    assert rt.candidate_ports() == ["COM17", "COM20"]
    assert rt.candidate_ports("COM5") == ["COM5"], "a named port is always tried"


def test_open_board_gives_up_after_its_timeout(rt, monkeypatch):
    tried = []
    monkeypatch.setattr(rt, "candidate_ports", lambda port: ["COM17"])
    monkeypatch.setattr(rt, "probe", lambda device: tried.append(device))
    with pytest.raises(RuntimeError, match="No board running the trigger sketch found on COM17"):
        rt.open_board("COM17", timeout_s=0)
    assert tried == ["COM17"]


def test_open_board_retries_a_port_not_yet_released(rt, monkeypatch):
    """The copy just stopped can take a moment to let go of the port."""
    board = rt.Board(FakeSerial([]))
    answers = [None, board]
    monkeypatch.setattr(rt, "candidate_ports", lambda port: ["COM17"])
    monkeypatch.setattr(rt, "probe", lambda device: answers.pop(0))
    assert rt.open_board(None, timeout_s=5) is board


# -- streaming ---------------------------------------------------------------------
def test_run_session_restarts_after_a_trigger_when_asked(rt):
    conn = FakeSerial(["TRIGGER: 3.20V"])
    started = time.monotonic()
    rt.run_session(rt.Board(conn), 0.05)
    assert time.monotonic() - started >= 0.05
    assert b"?\n" in conn.written, "A0 was never polled"


def test_run_session_does_not_restart_itself_when_told_not_to(rt):
    """--restart-after-trigger 0, as PALmixer runs it: only losing the board ends it."""
    class LosesThePort(FakeSerial):
        reads = 0

        def readline(self):
            self.reads += 1
            if self.reads > 200:
                raise rt.serial.SerialException("port lost")
            return super().readline()

    with pytest.raises(rt.serial.SerialException, match="port lost"):
        rt.run_session(rt.Board(LosesThePort(["TRIGGER: 3.20V"])), 0)


def test_a_board_that_stops_answering_counts_as_lost(rt, monkeypatch):
    """Polled once a second, the board always answers; silence means it hung."""
    monkeypatch.setattr(rt, "SILENCE_TIMEOUT", 0.05)
    with pytest.raises(rt.serial.SerialException, match="has not answered"):
        rt.run_session(rt.Board(FakeSerial([])), 0)


def test_defaults_match_the_standalone_behaviour(rt):
    args = rt.parse_args([])
    assert args.port is None
    assert args.restart_after_trigger == rt.RESTART_AFTER_TRIGGER == 20.0
    args = rt.parse_args(["--port", "COM17", "--restart-after-trigger", "0"])
    assert (args.port, args.restart_after_trigger) == ("COM17", 0.0)
