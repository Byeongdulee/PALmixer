r"""Host-side controller for the RedBoard pulse trigger.

The board does the triggering itself: when A0 rises above THRESHOLD_VOLTAGE it
waits DELAYTIME, drives D7 high for OPTIME, then ignores A0 for LOCKOUT_TIME.
It keeps doing so whether or not this script is running. What the script does
is find the board, push those four parameters into it -- they live in the
board's RAM only, so a power cycle reverts them to the sketch's defaults until
a host configures it again -- and print everything the board reports.

It speaks plain serial at 9600 baud, NOT Firmata. The board must have
firmware/redboard_trigger/redboard_trigger.ino uploaded (formerly
sketch_sep11a.ino), not StandardFirmata.

Protocol
    host  -> board : "<threshold>,<delay>,<optime>,<lockout>\n"  configure
                     "?\n"                                       query A0 now
    board -> host  : "UPDATED: ..."            configuration acknowledged
                     "A0: <v>V, locked=<0|1>"  reply to a query
                     "TRIGGER: <v>V"           threshold crossed
                     "LOCKOUT: <n>s"           pulse done, dead time started
                     "READY: monitoring A0"    lockout expired

Lines this script adds, for whoever reads its output (PALmixer parses them):
    "ARMED: port=<p> threshold=<v>V delay=<s>s pulse=<s>s lockout=<s>s locked=<0|1>"
                        the board acknowledged exactly the configuration sent
    "DISCONNECTED: ..." the port was lost, or the board stopped answering;
                        looking for it again
    "Error: ..."        why it is giving up, just before exiting with status 1

Running it
    python redboard_trigger.py                  from an Anaconda Prompt
    python redboard_trigger.py --port COM17     skip the port search

PALmixer runs it the same way, restarting it every time the robot starts
carrying the mixer head onto the carousel (palmixer/redboard.py), with
--restart-after-trigger 0 because it does the restarting itself. Whoever
starts it, it first stops any other copy already running: only one program
can hold the serial port, so a copy left running would leave this one with
no board to find.

Opening the port does not reset this native-USB (ATmega32U4) board: the
sketch keeps running throughout, including any lockout already in progress.

Exit status: 0 when stopped with Ctrl-C; 1 when no board is found, the board
does not acknowledge the configuration, or the board is lost and not found
again within --connect-timeout.
"""

import argparse
import os
import re
import sys
import time

import serial
import serial.tools.list_ports

# Operational parameters pushed to the board at startup -- edit these.
THRESHOLD_VOLTAGE = 3.0  # V
DELAYTIME = 0.26         # s, dead time between the trigger and the pulse
OPTIME = 0.39            # s, pulse width
LOCKOUT_TIME = 10.0      # s, dead time after each pulse

BAUD_RATE = 9600         # never 1200: closing a port opened at 1200 baud drops a
                         # 32U4 board into its bootloader
SETTLE_DELAY = 2.0       # s -- after opening a port, before talking to it. This
                         # board does not reboot when the port opens, but one that
                         # does (a classic RedBoard) needs this long to boot, and
                         # it outlasts the sketch's 1 s wait on a half-received line.
HANDSHAKE_TIMEOUT = 2.0  # s -- how long to wait for the reply to "?"
CONFIG_TIMEOUT = 2.0     # s -- how long to wait for "UPDATED:" after configuring
WRITE_TIMEOUT = 1.0      # s -- a write the board does not take in this long means it hung
POLL_INTERVAL = 1.0      # s -- how often to ask the board for the A0 value
SILENCE_TIMEOUT = 5.0    # s -- no reply to the polls for this long: reconnect
RESTART_AFTER_TRIGGER = 20.0  # s -- reconnect and reconfigure this long after a trigger
CONNECT_TIMEOUT = 15.0   # s -- keep looking for the board this long before giving up.
                         # Covers a port still being released by a copy just
                         # stopped, and a board still enumerating after a replug.

# Which ports are worth probing. A probe writes "?" to the port, and an
# unrelated device -- the pumps' RS-232 adapters are on this machine too -- has
# no business receiving it, so only ports that look like the board are tried
# unless --port names one.
BOARD_KEYWORDS = ("arduino", "redboard", "sparkfun", "ch340", "usb serial device")
BOARD_VIDS = {0x2341, 0x2A03, 0x1B4F, 0x1A86}  # Arduino LLC, Arduino Srl, SparkFun, WCH (CH340)

# The board stores the two times as whole milliseconds and prints every value
# to two decimals, so what it acknowledges can be up to 0.01 off what was sent.
# Anything further off is a different configuration.
CONFIG_TOLERANCE = 0.011


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Configure the RedBoard pulse trigger and print what it reports.")
    parser.add_argument("--port", help="the board's serial port, e.g. COM17 (default: find it)")
    parser.add_argument("--restart-after-trigger", type=float, default=RESTART_AFTER_TRIGGER,
                        metavar="S", help="reconnect and reconfigure S seconds after a "
                                          "trigger; 0 never (default: %(default)g)")
    parser.add_argument("--connect-timeout", type=float, default=CONNECT_TIMEOUT, metavar="S",
                        help="give up when no board is found within S seconds "
                             "(default: %(default)g)")
    return parser.parse_args(argv)


class Board:
    """An open connection to the trigger sketch.

    line() hands back whole lines only. pyserial's readline() applies its
    timeout to the whole line, so it can return half of one when the timeout
    lands mid-line -- a TRIGGER split into "TRIG" and "GER: 3.12V" would go
    unrecognized -- and the rest is kept here until its newline arrives.
    """

    def __init__(self, conn):
        self.conn = conn
        self.port = conn.port
        self._partial = b""

    def send(self, text):
        self.conn.write(text.encode())

    def line(self):
        """The next whole line, stripped; "" if none finished within the read timeout."""
        chunk = self.conn.readline()
        if not chunk.endswith(b"\n"):
            self._partial += chunk
            return ""
        line, self._partial = self._partial + chunk, b""
        # backslashreplace keeps any stray byte printable on every console and pipe.
        return line.decode(errors="backslashreplace").strip()

    def close(self):
        self.conn.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()


# -- one copy at a time -------------------------------------------------------
def script_argument(cmdline):
    """The script a Python command line runs, or None for -c, -m, or none.

    Only the interpreter's own script counts. An editor that merely has the
    file open -- "pythonw -m idlelib redboard_trigger.py", "pythonw
    spyder-script.py redboard_trigger.py" -- is running something else.
    """
    args = list(cmdline or [])[1:]
    while args:
        arg = args.pop(0)
        if arg == "--":
            return args[0] if args else None
        if arg.startswith("--"):
            if arg == "--check-hash-based-pycs" and args:
                args.pop(0)                 # its value
            continue
        if arg.startswith("-") and len(arg) > 1:
            letters = arg[1:]               # combined flags, as in -uB
            for i, letter in enumerate(letters):
                if letter in "cm":
                    return None             # code or a module, not a script
                if letter in "WX":          # take a value: -X utf8, or -Xutf8
                    if i == len(letters) - 1 and args:
                        args.pop(0)
                    break
            continue
        return arg
    return None


def is_other_instance(name, cmdline, script_name):
    """Is this process (executable `name`, arguments `cmdline`) a Python
    running a script called `script_name`?"""
    if not (name or "").lower().startswith("python"):
        return False
    script = script_argument(cmdline)
    return script is not None and os.path.basename(script).lower() == script_name.lower()


def stop_other_instances(ps=None):
    """Stop any other copy of this script, so this one can have the serial port.

    Matches Python processes running a file named like this one, wherever it
    lives -- a copy started by hand, or one left behind by a PALmixer server
    that died -- but never this process or one of its parents. Needs psutil
    (`ps`, for tests); without it nothing is stopped, and a port held by
    another copy shows up as "unavailable" in the probe. A copy that cannot be
    stopped -- one started as administrator -- is reported and left; the probe
    then finds its port busy.
    """
    if ps is None:
        try:
            import psutil as ps
        except ImportError:
            return
    try:
        me = ps.Process()
        skip = {me.pid} | {parent.pid for parent in me.parents()}
    except ps.Error:
        skip = {os.getpid()}
    script_name = os.path.basename(__file__)
    others = [proc for proc in ps.process_iter(["pid", "name", "cmdline"])
              if proc.info["pid"] not in skip
              and is_other_instance(proc.info["name"], proc.info["cmdline"], script_name)]
    for proc in others:
        print(f"Stopping the copy already running (pid {proc.pid})", flush=True)
        try:
            proc.terminate()
        except ps.Error:
            pass                            # judged by the wait below
    for proc in others:
        try:
            proc.wait(timeout=5)
        except ps.NoSuchProcess:
            pass
        except ps.TimeoutExpired:
            try:
                proc.kill()
                proc.wait(timeout=5)
            except ps.Error as exc:
                print(f"Could not stop pid {proc.pid}: {exc!r}", flush=True)
        except ps.Error as exc:
            print(f"Could not stop the copy already running (pid {proc.pid}): {exc!r} -- "
                  "started as administrator? Close it by hand.", flush=True)


# -- finding the board --------------------------------------------------------
def looks_like_board(info):
    """Could this port (a serial.tools.list_ports entry) be the trigger board?"""
    text = f"{info.description} {info.manufacturer or ''}".lower()
    return info.vid in BOARD_VIDS or any(word in text for word in BOARD_KEYWORDS)


def candidate_ports(port=None):
    """The ports to probe: the one named, or every port that looks like the board."""
    if port:
        return [port]
    return [info.device for info in serial.tools.list_ports.comports() if looks_like_board(info)]


def probe(device):
    """Open `device` and return a Board if the trigger sketch answers there,
    else None. The sketch is the only thing that answers "?" with an "A0:"
    line, so this won't latch onto an unrelated serial device."""
    print(f"Probing {device}...", flush=True)
    try:
        conn = serial.Serial(device, BAUD_RATE, timeout=0.1, write_timeout=WRITE_TIMEOUT)
    except (OSError, serial.SerialException) as exc:
        print(f"  {device} unavailable: {exc}", flush=True)  # busy, or gone
        return None
    board = Board(conn)
    try:
        time.sleep(SETTLE_DELAY)
        conn.reset_input_buffer()
        board.send("?\n")
        if read_until(board, "A0:", HANDSHAKE_TIMEOUT, echo=False) is not None:
            print(f"Found board on {device}", flush=True)
            return board
    except (OSError, serial.SerialException) as exc:
        print(f"  {device} failed: {exc}", flush=True)
    board.close()
    return None


def open_board(port=None, timeout_s=CONNECT_TIMEOUT):
    """Find the port running the trigger sketch, retrying for up to timeout_s.
    Returns a Board; raises RuntimeError."""
    deadline = time.monotonic() + timeout_s
    while True:
        for device in candidate_ports(port):
            board = probe(device)
            if board is not None:
                return board
        if time.monotonic() >= deadline:
            raise RuntimeError(
                "No board running the trigger sketch found%s. Check the USB cable, make "
                "sure redboard_trigger.ino is uploaded, and close the Arduino Serial "
                "Monitor or anything else holding the port."
                % (f" on {port}" if port else ""))
        time.sleep(1.0)


# -- talking to it ------------------------------------------------------------
def read_until(board, prefix, timeout_s, echo=True):
    """Return the first line from the board starting with `prefix`, or None
    after timeout_s. Other lines are printed (if `echo`) rather than lost."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = board.line()
        if line.startswith(prefix):
            return line
        if line and echo:
            print(line, flush=True)
    return None


def configure(board):
    """Push the operational parameters and confirm the board took them.

    Returns True if the board is in a post-trigger lockout right now. Raises
    RuntimeError if the board does not acknowledge, or acknowledges different
    values.
    """
    sent = (THRESHOLD_VOLTAGE, DELAYTIME, OPTIME, LOCKOUT_TIME)
    # Sent a second time if unanswered: the sketch drops a line that arrives
    # glued to a stray fragment another program left on the port.
    for _ in range(2):
        board.send(f"{THRESHOLD_VOLTAGE},{DELAYTIME},{OPTIME},{LOCKOUT_TIME}\n")
        reply = read_until(board, "UPDATED:", CONFIG_TIMEOUT)
        if reply is not None:
            break
    else:
        raise RuntimeError("the board did not acknowledge the configuration")
    print(reply, flush=True)
    got = [float(value) for value in re.findall(r"=\s*([-+0-9.]+)", reply)]
    if len(got) != len(sent) or any(abs(g - s) > CONFIG_TOLERANCE for g, s in zip(got, sent)):
        raise RuntimeError(f"the board acknowledged a different configuration: {reply}")
    board.send("?\n")
    status = read_until(board, "A0:", HANDSHAKE_TIMEOUT)
    if status is None:
        raise RuntimeError("the board stopped answering after it was configured")
    print(status, flush=True)
    return status.endswith("locked=1")


def connect_and_arm(port, connect_timeout):
    """Open the board, configure it, and announce it armed. Returns the Board."""
    board = open_board(port, connect_timeout)
    try:
        locked = configure(board)
    except BaseException:
        board.close()
        raise
    print(f"ARMED: port={board.port} threshold={THRESHOLD_VOLTAGE:.2f}V "
          f"delay={DELAYTIME:.2f}s pulse={OPTIME:.2f}s lockout={LOCKOUT_TIME:.2f}s "
          f"locked={int(locked)}", flush=True)
    return board


def run_session(board, restart_after_trigger):
    """Print the board's output, polling A0 every POLL_INTERVAL seconds.

    Returns restart_after_trigger seconds after the first TRIGGER line (never,
    if that is 0); later triggers do not push the restart back. Raises
    serial.SerialException if the port is lost, or if the board stops
    answering for SILENCE_TIMEOUT seconds.
    """
    next_poll = last_heard = time.monotonic()
    restart_at = None
    while True:
        line = board.line()
        now = time.monotonic()
        if line:
            last_heard = now
            print(line, flush=True)
            if line.startswith("TRIGGER:") and restart_after_trigger > 0 and restart_at is None:
                restart_at = now + restart_after_trigger
                print(f"Restarting in {restart_after_trigger:g} s", flush=True)
        elif now - last_heard > SILENCE_TIMEOUT:
            raise serial.SerialException(
                f"the board has not answered for {SILENCE_TIMEOUT:g} s")

        if restart_at is not None and now >= restart_at:
            return

        if now >= next_poll:
            board.send("?\n")
            next_poll = now + POLL_INTERVAL


def main(argv=None):
    args = parse_args(argv)
    stop_other_instances()
    while True:
        with connect_and_arm(args.port, args.connect_timeout) as board:
            print("Ctrl-C to stop.\n", flush=True)
            try:
                run_session(board, args.restart_after_trigger)
                print("\nRestarting...\n", flush=True)
            except serial.SerialException as exc:
                print(f"DISCONNECTED: {exc}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    except (RuntimeError, serial.SerialException) as exc:
        print(f"Error: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
