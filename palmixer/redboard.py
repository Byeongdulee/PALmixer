# -*- coding: utf-8 -*-
"""Restart the RedBoard pulse-trigger script each time the mixer head heads for CRSL.

The RedBoard is a native-USB Arduino (an ATmega32U4; it enumerates as an
"Arduino Leonardo") that watches an analog signal on A0 and, when it crosses
a threshold, waits a set delay, fires a pulse on D7, and then locks itself out
for a while. The firmware does the triggering on its own. Its host script,
redboard/redboard_trigger.py, finds the board, pushes the four operating
parameters into it -- they live in the board's RAM only -- and reports what
the board says. See redboard/README.md.

What PALmixer does with it
--------------------------
- Every real start of mixer2mixingstation -- the robot carrying the mixer head
  off its cleaning station and onto the carousel -- calls restart(). The
  instance started last time is stopped and a fresh one launched, so the board
  is configured from the script as it is on disk *now* and the session
  watching it is new. Both callers make the call after their wash wait, so it
  marks the move actually starting, not the order for it: Workflows.
  _move_mixer_head (make_sample, standalone or coordinated) and Server.
  _run_transport (the Experiment tab's button).
- make_sample calls await_armed() just before 'mix' and will not mix unless
  the board is *armed*: it has acknowledged the configuration, is not locked
  out, is still answering, and reads A0 below its threshold -- the firmware
  is level-triggered, so an input already above the threshold would fire it
  on its own, not on the mix. A sample mixed without its trigger is the
  expensive way to find out the board was unplugged.

The move takes 17-30 s; finding and configuring the board takes about 3 s, so
in normal running the check before 'mix' passes without waiting.

restart() returns at once and never raises. Stopping the old instance and
starting the new one happen on a background thread, because the caller is the
thread driving the robot, and the arm must not wait on -- or be stopped by --
anything here. A failure there is recorded, and await_armed() reports it.

Only one process can hold a serial port on Windows, so the next instance can
connect only once the last has let go. The supervisor waits for its own
previous child to exit before launching the next, and the script itself stops
any other copy of redboard_trigger.py still running -- one started by hand
from an Anaconda Prompt, or an orphan of a server that died -- before it
looks for the board. An orphan does not outlive its server by much anyway:
its output goes to a pipe only the server reads, and its next line (one a
second) fails once that end is gone. Turning the trigger off in the config
stops the running script within WATCH_INTERVAL_S, so the port is free for an
upload of the sketch or a run by hand; stop() ends it for good.

What a restart does not do: opening the port does not reset a native-USB
board, so a lockout already running survives it -- the firmware ends a
lockout only on its own timer. That is why being armed includes the board
reporting itself unlocked; a lockout is a few seconds and ends by itself.

Everything the script prints goes to logs/redboard_trigger.log, rotated
between sessions. Arming, triggers and failures are also printed on the
server console, and the latest state rides on get_state as ``redboard``.
"""

import logging
import os
import re
import subprocess
import sys
import threading
import time

from . import config
from .mixing_records import utc_now

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: States, as reported in snapshot()["state"].
OFF = "off"              # disabled in the config, or --simulate
IDLE = "idle"            # enabled, not started yet this session
STARTING = "starting"    # restarted; the board has not confirmed its configuration yet
ARMED = "armed"          # the board acknowledged the configuration
FAILED = "failed"        # the last restart did not end armed; `detail` says why
STOPPED = "stopped"      # the server is shutting down

#: How long a terminated instance gets to exit before it is killed outright.
EXIT_WAIT_S = 5.0

#: The script asks the board for its status every second and prints each
#: reply. No word for this long means the session watching the board has
#: stalled, whatever its last message said.
STALE_AFTER_S = 5.0

#: How often a running script is checked against redboard.enabled, so turning
#: the trigger off frees the serial port within seconds.
WATCH_INTERVAL_S = 2.0

#: The log is rotated only between sessions, never mid-line, and only when the
#: live file can be moved aside first -- see _rotate().
LOG_MAX_BYTES = 5 * 1024 * 1024
LOG_BACKUPS = 5

#: No console window for the child. 0 off Windows, where the flag does not exist.
_CREATE_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)

#: Exit status of a process ended by psutil's terminate() on Windows -- what a
#: copy of the script started by hand does to the one running here.
_TERMINATED_BY_ANOTHER_COPY = 15

_A0_RE = re.compile(r"^A0:\s*([-+0-9.]+)\s*V,\s*locked=([01])")
_TRIGGER_RE = re.compile(r"^TRIGGER:\s*([-+0-9.]+)\s*V")
_PORT_RE = re.compile(r"\bport=(\S+)")
_LOCKED_RE = re.compile(r"\blocked=([01])")
_THRESHOLD_RE = re.compile(r"\bthreshold=([-+0-9.]+)V")
# The last line of a Python traceback: "SyntaxError: invalid syntax",
# "ModuleNotFoundError: No module named 'serial'", ...
_EXCEPTION_RE = re.compile(r"^[A-Za-z_][\w.]*(Error|Exception)\b")


class TriggerNotArmed(Exception):
    """await_armed() gave up: the board is not armed."""


class RedBoardTrigger:
    """Owns the redboard_trigger.py child process for the server's lifetime.

    `settings` returns the ``redboard`` config section and is read afresh on
    every use, so turning the trigger off in json/palmixer_config.json takes
    effect without restarting the server. `popen` and `clock` are for tests.
    """

    def __init__(self, simulate=False, settings=None, popen=subprocess.Popen,
                 clock=time.monotonic):
        self.simulate = simulate
        self._settings = settings or (lambda: config.get_section("redboard"))
        self._popen = popen
        self._clock = clock
        # _cond guards every field below; _launch_lock serializes stopping
        # and starting children, which can each take seconds.
        self._cond = threading.Condition()
        self._launch_lock = threading.Lock()
        self._generation = 0         # bumped by every restart, retirement and stop()
        self._stopped = False        # stop() is final
        self._watching = False       # the enabled-watcher thread has been started
        self._wake = threading.Event()   # ends the watcher
        self._proc = None
        self._state = OFF if simulate else IDLE
        self._detail = ""
        self._restarts = 0
        self._log_lock = threading.Lock()
        self._log_handler = None
        self._reset_session()

    # -- the two calls the robot side makes --------------------------------------
    @property
    def enabled(self):
        """True when the config turns the trigger on and this is not --simulate."""
        return not self.simulate and bool(self._settings().get("enabled"))

    def restart(self, reason):
        """Stop the running instance and start a fresh one, without waiting.

        Called from the thread driving the robot, so this only records the
        request and hands the work to a background thread: it never blocks and
        never raises. Does nothing once stop() has been called.
        """
        try:
            if not self.enabled:
                return
            with self._cond:
                if self._stopped:
                    return
                self._generation += 1
                generation = self._generation
                self._restarts += 1
                self._reset_session()
                self._state, self._detail = STARTING, "restarting for %s" % reason
                self._cond.notify_all()
            print("RedBoard: restarting the trigger script for %s" % reason)
            threading.Thread(target=self._relaunch, args=(generation, reason),
                             name="redboard-restart", daemon=True).start()
        except Exception as e:      # never into the robot's thread
            self._fail(None, "could not restart the RedBoard script: %s" % e)

    def await_armed(self, timeout_s=None):
        """Block until the board is armed; return a description of how it is set.

        Raises TriggerNotArmed at once if the last restart failed -- the script
        exited, or could not be launched -- since waiting cannot fix that. A
        restart still finding the board, a lockout still running out, or A0
        still above the threshold is waited for up to `timeout_s` (the
        config's armed_timeout_s by default).
        """
        if timeout_s is None:
            timeout_s = float(self._settings().get("armed_timeout_s", 15.0))
        deadline = self._clock() + timeout_s
        with self._cond:
            while True:
                problem = self._not_ready()
                if problem is None:
                    return "%s; A0 %.2f V, not locked out" % (self._detail, self._a0_volts)
                if self._state not in (STARTING, ARMED):
                    raise TriggerNotArmed(problem)
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise TriggerNotArmed("%s, after waiting %.0f s" % (problem, timeout_s))
                # Woken by every line from the board; the cap re-checks the
                # staleness clock, which nothing notifies.
                self._cond.wait(min(remaining, 0.5))

    def is_armed(self):
        """Armed right now -- await_armed() would not wait."""
        with self._cond:
            return self._not_ready() is None

    # -- status and shutdown ---------------------------------------------------
    def snapshot(self):
        """The current state, JSON-serializable, for get_state."""
        enabled = self.enabled
        with self._cond:
            problem = self._not_ready()
            return {
                "enabled": enabled,
                "state": self._state,
                "ready": problem is None,
                "detail": self._detail if problem is None else problem,
                "pid": self._pid,
                "port": self._port,
                "armed_at": self._armed_at,
                "a0_volts": self._a0_volts,
                "threshold_volts": self._threshold,
                "locked": self._locked,
                "last_trigger": dict(self._last_trigger) if self._last_trigger else None,
                "restarts": self._restarts,
            }

    def stop(self):
        """Stop the child for good; later restart() calls do nothing. Called
        from Server.stop()."""
        with self._cond:
            self._stopped = True
            self._generation += 1
            if self._state != OFF:
                self._state, self._detail = STOPPED, "the server is shutting down"
            self._cond.notify_all()
        self._wake.set()
        with self._launch_lock:
            try:
                self._stop_child()
            except Exception as e:
                self._log("--- could not stop the RedBoard script: %s" % e)
        with self._log_lock:
            self._close_log()

    # -- launching ---------------------------------------------------------------
    def _relaunch(self, generation, reason):
        try:
            with self._launch_lock:
                with self._cond:
                    if self._stopped or generation != self._generation:
                        return              # superseded, retired, or stopped
                self._stop_child()
                settings = self._settings()
                argv, cwd = self._command(settings)
                self._start_log(settings)
                self._log("--- restart for %s: %s" % (reason, subprocess.list2cmdline(argv)))
                proc = self._popen(argv, cwd=cwd, env=_child_env(),
                                   stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                   errors="replace", bufsize=1,
                                   creationflags=_CREATE_NO_WINDOW)
                with self._cond:
                    # Recorded even if a newer restart came in meanwhile: that
                    # one is waiting on _launch_lock, and stops this child first.
                    self._proc = proc
                    if generation == self._generation:
                        self._pid = proc.pid
                        self._detail = ("pid %d started for %s; looking for the board"
                                        % (proc.pid, reason))
                    start_watch, self._watching = not self._watching, True
                threading.Thread(target=self._read_output, args=(proc, generation),
                                 name="redboard-output", daemon=True).start()
                if start_watch:
                    threading.Thread(target=self._watch, name="redboard-watch",
                                     daemon=True).start()
        except Exception as e:
            self._fail(generation, "could not start the RedBoard script: %s" % e)

    def _command(self, settings):
        """The child's argv and working directory, from the config section."""
        script = settings.get("script") or "redboard/redboard_trigger.py"
        script = os.path.normpath(os.path.join(_ROOT_DIR, script))  # unless absolute
        if not os.path.isfile(script):
            raise FileNotFoundError("no RedBoard script at %s (redboard.script in "
                                    "json/palmixer_config.json)" % script)
        # -u so each line reaches the pipe as it is printed. The script's own
        # restart-after-trigger is turned off: restarts are this module's job.
        argv = [settings.get("python") or sys.executable, "-u", script,
                "--restart-after-trigger", "0"]
        if settings.get("port"):
            argv += ["--port", str(settings["port"])]
        return argv, os.path.dirname(script)

    def _stop_child(self):
        """Terminate the current child and wait for it to exit. Under _launch_lock."""
        with self._cond:
            proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        self._log("--- stopping pid %d" % proc.pid)
        proc.terminate()
        try:
            proc.wait(EXIT_WAIT_S)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(EXIT_WAIT_S)

    def _watch(self):
        """Stop the running script once the config turns the trigger off."""
        while not self._wake.wait(WATCH_INTERVAL_S):
            try:
                if not self.enabled:
                    self._retire()
            except Exception as e:
                self._log("--- enabled check failed: %s" % e)

    def _retire(self):
        """Stop the script because the trigger was turned off. Unlike stop(),
        the next restart() -- once it is turned back on -- starts it again."""
        with self._cond:
            if self._stopped or self._proc is None or self._proc.poll() is not None:
                return
            self._generation += 1
            self._state, self._detail = OFF, "turned off in the config (redboard.enabled)"
            self._pid = None
            self._cond.notify_all()
        with self._launch_lock:
            self._stop_child()
        print("RedBoard: turned off in the config; trigger script stopped, serial port free")

    # -- the child's output ------------------------------------------------------
    def _read_output(self, proc, generation):
        try:
            for raw in proc.stdout:
                line = raw.strip()
                if line:
                    self._log("[%d] %s" % (proc.pid, line))
                    self._on_line(generation, line)
        except Exception as e:      # the pipe breaking as the child is killed, say
            self._log("[%d] output stopped: %s" % (proc.pid, e))
        finally:
            try:
                proc.stdout.close()
            except Exception:
                pass
        code = proc.wait()
        self._log("[%d] exited with code %s" % (proc.pid, code))
        self._on_exit(generation, proc, code)

    def _on_line(self, generation, line):
        announce = None
        with self._cond:
            if generation != self._generation:
                return
            now = self._clock()
            a0 = _A0_RE.match(line)
            if a0:
                self._last_heard = now
                self._a0_volts = float(a0.group(1))
                self._locked = a0.group(2) == "1"
            elif line.startswith("ARMED:"):
                self._last_heard = now
                text = line[len("ARMED:"):].strip()
                port, locked = _PORT_RE.search(text), _LOCKED_RE.search(text)
                threshold = _THRESHOLD_RE.search(text)
                if port:
                    self._port = port.group(1)
                if locked:
                    self._locked = locked.group(1) == "1"
                if threshold:
                    self._threshold = float(threshold.group(1))
                self._armed_at = utc_now()
                # Without its locked=, which goes stale; the live state is
                # added wherever this is shown as armed.
                self._state = ARMED
                self._detail = re.sub(r"\s*\blocked=[01]", "", text)
                announce = "RedBoard: armed (%s)" % text
            elif line.startswith("TRIGGER:"):
                self._last_heard = now
                self._locked = True
                volts = _TRIGGER_RE.match(line)
                self._last_trigger = {"at": utc_now(),
                                      "volts": float(volts.group(1)) if volts else None}
                announce = "RedBoard: %s" % line
            elif line.startswith("LOCKOUT:"):
                self._last_heard = now
                self._locked = True
            elif line.startswith("READY:"):
                # The lockout is over -- but with A0 still above the threshold,
                # the level-triggered firmware fires again on its very next
                # pass. Only the next status reply says which, so until then
                # the board is not counted as ready.
                self._last_heard = now
                self._locked = None
                self._a0_volts = None
            elif line.startswith("UPDATED:"):
                self._last_heard = now
            elif line.startswith("DISCONNECTED:"):
                # The script looks for the board again by itself; until it
                # says ARMED once more, nothing it said before counts.
                self._state = STARTING
                self._detail = "reconnecting: %s" % line[len("DISCONNECTED:"):].strip()
                self._locked = None
                self._a0_volts = None
                announce = "RedBoard: port lost (%s); reconnecting" % self._detail
            elif line.startswith("Error:"):
                self._last_error = line[len("Error:"):].strip()
            elif _EXCEPTION_RE.match(line):
                # A crash rather than a reported failure -- a bad edit, a
                # missing module. The last such line of a traceback is the
                # best reason there is.
                self._last_exception = line
            self._cond.notify_all()
        if announce:
            print(announce)

    def _on_exit(self, generation, proc, code):
        with self._cond:
            if generation != self._generation:
                return      # stopped on purpose: replaced, retired, or shut down
            reason = self._last_error or self._last_exception
            if not reason:
                reason = ("stopped by another copy of redboard_trigger.py (one started by hand?)"
                          if code == _TERMINATED_BY_ANOTHER_COPY else "it reported no error")
            detail = ("the RedBoard script (pid %d) exited with code %s: %s"
                      % (proc.pid, code, reason))
            self._state, self._detail, self._pid = FAILED, detail, None
            self._cond.notify_all()
        print("RedBoard: %s" % detail)

    def _fail(self, generation, detail):
        """Record a failed restart, unless a newer one has already superseded it."""
        with self._cond:
            if generation is not None and generation != self._generation:
                return
            self._state, self._detail = FAILED, detail
            self._cond.notify_all()
        self._log("--- " + detail)
        print("RedBoard: %s" % detail)

    # -- readiness ---------------------------------------------------------------
    def _reset_session(self):
        """Forget everything the previous child reported. Under _cond."""
        self._pid = None
        self._port = None
        self._armed_at = None
        self._locked = None
        self._a0_volts = None
        self._threshold = None
        self._last_heard = None
        self._last_trigger = None
        self._last_error = ""
        self._last_exception = ""

    def _not_ready(self):
        """Why the board cannot be relied on right now, or None. Under _cond."""
        if self._state == ARMED:
            if self._last_heard is None or self._clock() - self._last_heard > STALE_AFTER_S:
                return "no word from the board for over %.0f s" % STALE_AFTER_S
            if self._locked is None or self._a0_volts is None:
                return "waiting for the board's next status report"
            if self._threshold is not None and self._a0_volts >= self._threshold:
                return ("A0 reads %.2f V, at or above the %.2f V threshold: the board would "
                        "fire on its own, not on the mix (is the trigger input stuck high?)"
                        % (self._a0_volts, self._threshold))
            if self._locked:
                return "the board is still in its post-trigger lockout"
            return None
        if self._state == STARTING:
            return "the board has not confirmed its configuration yet (%s)" % self._detail
        if self._state == FAILED:
            return self._detail
        if self._state == OFF:
            return "the RedBoard trigger is off (redboard.enabled, or --simulate)"
        if self._state == STOPPED:
            return "the RedBoard supervisor has been stopped"
        return "the RedBoard script has not been started yet"

    # -- log -------------------------------------------------------------------
    def _start_log(self, settings):
        """Open the log for a new session, rotating it first if it has grown.

        Never raises: the trigger works without a log, and the check before
        'mix' must not refuse because of a file.
        """
        path = os.path.normpath(os.path.join(  # unless absolute
            _ROOT_DIR, settings.get("log_file") or "logs/redboard_trigger.log"))
        problem = None
        with self._log_lock:
            self._close_log()
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                if os.path.exists(path) and os.path.getsize(path) > LOG_MAX_BYTES:
                    _rotate(path)
                handler = logging.FileHandler(path, encoding="utf-8", delay=True)
                handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
                self._log_handler = handler
            except OSError as e:
                problem = e
        if problem is not None:
            print("RedBoard: cannot write the log at %s (%s); running without it"
                  % (path, problem))

    def _close_log(self):
        """Under _log_lock."""
        if self._log_handler is not None:
            self._log_handler.close()
            self._log_handler = None

    def _log(self, message):
        """Append one line to the session log; the log never breaks the supervisor."""
        with self._log_lock:
            handler = self._log_handler
            if handler is None:
                return
            try:
                handler.handle(logging.makeLogRecord(
                    {"msg": message, "levelno": logging.INFO, "levelname": "INFO"}))
            except Exception:
                pass


def _rotate(path):
    """Move `path` to `path`.1, shifting older backups up and dropping the oldest.

    The live log is moved aside first, and the backups are shifted only once
    that has worked. A program holding the file open without delete sharing
    makes the move fail, and shifting first would then lose a backup every
    session while rotating nothing. Leaves everything as it was if the live
    log cannot be moved.
    """
    aside = path + ".rotating"
    try:
        os.replace(path, aside)
    except OSError:
        return
    for n in range(LOG_BACKUPS - 1, 0, -1):
        try:
            os.replace("%s.%d" % (path, n), "%s.%d" % (path, n + 1))
        except OSError:
            pass                    # missing, or held open
    try:
        os.replace(aside, path + ".1")
    except OSError:
        try:
            os.replace(aside, path)  # put it back rather than lose it
        except OSError:
            pass


def _child_env():
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUNBUFFERED"] = "1"
    return env
