# -*- coding: utf-8 -*-
"""Pump driver: a ZMQ client to the apssector12_pump_control dashboards.

The real pump software lives in a separate repo (``apssector12_pump_control``)
and runs two loopback-only ZeroMQ REP servers, started by its Tk dashboards:

    Main pump / mixing   PumpControl.cmd      tcp://127.0.0.1:5555
    FlowCell             FlowCellControl.cmd  tcp://127.0.0.1:5556

This class is a thin REQ client to those two servers. It keeps PALmixer's pump
interface unchanged -- every op still returns ``(ok: bool, detail: str)`` and
blocks until the pump is done -- so server.py, workflows.py and the GUI need no
changes. The pump dashboards keep ownership of the serial (COM) ports and their
own safety confirmations (syringe sizes, position trust); PALmixer only sends
commands.

Command mapping (see the pump repo's ZMQ_COMMANDS.md / FLOWCELL_ZMQ_COMMANDS.md):

    mix()                    -> 5555  sample_make
    clean_mixer()            -> 5555  clean_all
    draw_to_flowcell()       -> 5556  draw_to_flowcell   (+ device)
    aspirate_from_flowcell() -> 5556  aspirate_from_flowcell (+ device)
    wash_flowcell()          -> 5556  wash_flowcell      (+ device)
    shake_sample()           -> 5556  flow2_sample / flow3_sample (device is
                                      in the command name, not an argument)
    stop_shaking()           -> 5556  stop_shaking  (active shake only; own
                                      socket, no wait)

Flow-cell selection is the 0-based ``device`` int the flowcell server expects.
PALmixer tracks the in-use flowcell as an id (1 or 2) in state; this maps
id 1 -> device 0 (flowcell2) and id 2 -> device 1 (flowcell3), so the operator's
existing "which flowcell" choice drives the pump automatically.

**Queued, not done.** A ZMQ reply means the command was accepted and queued, not
that motion finished. To keep the blocking ``(ok, detail)`` contract every
motion op waits: it sends the command, then polls the server's ``status`` until
``operation_active`` goes false before returning.

**One lock per server, not one lock overall.** The whole send+poll runs under
the lock belonging to the endpoint it is talking to. Within a server that
serializes everything, which is required: the REQ socket cannot be shared, and
neither can the pump behind it. Across the two it does not, which is the point
-- the mixer and the flowcell are separate hardware, so cleaning the mixer
(5555) and drawing into the flowcell (5556) run side by side. A single lock
across both used to make the draw wait out the whole mixer clean for nothing.

So ``clean_mixer()`` fired on a daemon thread no longer holds up
``draw_to_flowcell()``; a background ``wash_flowcell()`` still holds up the next
flowcell op, because that really is the same pump.

**Mixing speed is advisory.** The mixing server's ``sample_make`` uses the
volumes and speeds currently shown in the pump dashboard and rejects overrides,
so PALmixer cannot set the mix speed over ZMQ. ``speed``/``set_speed`` are kept
as a local record only: the value is still reported and folded into each mix's
detail (so the sample record carries a speed), but the dashboard sets the actual
hardware speed.

Connection settings come from the ``pump`` section of json/palmixer_config.json
(host, mixer_port, flowcell_port, timeouts). Note the pump dashboards read the
env vars ``PALMIXER_ZMQ_PORT`` / ``PALMIXER_MQTT_PORT`` for *their own* ports --
the same names PALmixer uses for its control plane -- so this client uses its own
``PALMIXER_PUMP_*`` overrides and never those.
"""

import threading
import time
import uuid

from . import config
from . import state

#: Fallbacks when json/palmixer_config.json has no ``pump`` section.
DEFAULT_SPEED_RPM = 800.0
DEFAULT_MIN_RPM = 0.0
DEFAULT_MAX_RPM = 3000.0
DEFAULT_HOST = "127.0.0.1"
DEFAULT_MIXER_PORT = 5555
DEFAULT_FLOWCELL_PORT = 5556
DEFAULT_REQUEST_TIMEOUT_S = 5.0
DEFAULT_POLL_INTERVAL_S = 0.5
DEFAULT_OPERATION_TIMEOUT_S = 600.0
#: Ceiling on how long a status poll waits for a reply. Deliberately shorter
#: than an operation's timeout: a status read that hangs leaves a stale panel,
#: which is worth far less than the seconds it would spend waiting, and a
#: server that is down should show as down promptly rather than eventually.
STATUS_TIMEOUT_S = 2.0


class PumpError(Exception):
    """Raised on a ZMQ transport failure talking to a pump server."""


def _new_id():
    return "palmixer-%s" % uuid.uuid4().hex[:8]


#: Longest status text kept for the GUI panel. The mixing server's `operation`
#: runs to several lines of prose; a status column is not where that belongs.
STATUS_TEXT_LIMIT = 90


def _operation_summary(operation):
    """One line out of the mixing server's multi-line `operation` report.

    That field reads like

        Purpose: Cleaning - Clean everything
        Selected: both pumps (pump0 and pump1)
        Status: COMPLETE - selected plunger(s) are at 0 uL.
        Actual elapsed: 5 min 50 s; ...

    of which the "Status:" line is the part that answers "what is it doing".
    Falls back to the first non-empty line for a report shaped differently, so
    an unfamiliar format degrades to something rather than nothing.
    """
    lines = [ln.strip() for ln in str(operation or "").splitlines() if ln.strip()]
    if not lines:
        return ""
    chosen = next((ln[len("Status:"):].strip() for ln in lines
                   if ln.startswith("Status:")), lines[0])
    if len(chosen) > STATUS_TEXT_LIMIT:
        chosen = chosen[:STATUS_TEXT_LIMIT - 3] + "..."
    return chosen


def _mixer_pump_names(reply):
    """The mixing server's two pump names, however it happens to report them.

    They live under make_sample.current_settings rather than at the top level,
    since that server has no per-pump list the way the flowcell one does.
    """
    try:
        names = reply["make_sample"]["current_settings"]["pump_names"]
        if names:
            return [str(n) for n in names]
    except (KeyError, TypeError):
        pass
    return ["pump0", "pump1"]


def _mixer_position(position_ul, trusted):
    """Format one mixing pump's plunger position.

    An untrusted position is called out rather than shown as a bare number: the
    server means "I have not homed since something could have moved this", and
    a number presented like any other would read as fact.
    """
    if position_ul is None:
        return "--"
    text = "%g uL" % float(position_ul)
    return text if trusted else text + " (untrusted)"


#: Which pump server each operation talks to. The two are independent hardware
#: on independent ports, so this is also what decides which operations can run
#: at the same time -- see Pump._run_remote and workflows._await_background_pump.
MIXER_SERVER = "mixer"
FLOWCELL_SERVER = "flowcell"
OP_SERVERS = {
    "mix": MIXER_SERVER,
    "clean_mixer": MIXER_SERVER,
    "draw_to_flowcell": FLOWCELL_SERVER,
    "aspirate_from_flowcell": FLOWCELL_SERVER,
    "wash_flowcell": FLOWCELL_SERVER,
    "shake_sample": FLOWCELL_SERVER,
}


def server_for(op):
    """Which pump server ``op`` runs on, or None for an unknown name.

    Two ops that answer differently here cannot block each other. Callers use
    that to decide whether waiting for one before starting the other is a real
    constraint or just lost time.
    """
    return OP_SERVERS.get(op)


class _Endpoint:
    """One REQ socket to one pump server, with lazy-pirate recovery.

    A REQ socket that misses a reply is stuck (it cannot send again), so on any
    send/recv failure the socket is dropped and reopened on the next call.

    Carries the lock for its own server. The socket cannot be used by two
    threads at once and neither can the pump behind it, but that is a
    per-server constraint, not a global one -- so the lock lives here, with the
    thing it actually protects, rather than on Pump.
    """

    def __init__(self, zmq, ctx, host, port, timeout_ms):
        self._zmq = zmq
        self._ctx = ctx
        self._addr = "tcp://%s:%d" % (host, port)
        self._timeout_ms = timeout_ms
        self._sock = None
        self.lock = threading.Lock()

    def _open(self):
        sock = self._ctx.socket(self._zmq.REQ)
        sock.setsockopt(self._zmq.LINGER, 0)
        sock.setsockopt(self._zmq.RCVTIMEO, self._timeout_ms)
        sock.setsockopt(self._zmq.SNDTIMEO, self._timeout_ms)
        sock.connect(self._addr)
        self._sock = sock

    def request(self, payload):
        """Send one JSON request and return the reply dict.

        Raises :class:`PumpError` on any transport failure (including timeout),
        after dropping the now-unusable socket.
        """
        if self._sock is None:
            self._open()
        try:
            self._sock.send_json(payload)
            return self._sock.recv_json()
        except Exception as e:                     # noqa: BLE001 - all transport errs
            self.close()
            raise PumpError("%s: %s" % (self._addr, e))

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close(0)
            except Exception:                      # noqa: BLE001
                pass
            self._sock = None


class Pump:
    """ZMQ client to the apssector12_pump_control mixing + flowcell servers."""

    def __init__(self, simulate=False, **kwargs):
        self.simulate = simulate
        # Guards the advisory mixing speed only. The pump servers have a lock
        # each, on their endpoint; this one protects a float and must not be
        # held across a pump operation, or it would serialize the two servers
        # again through the back door.
        self._speed_lock = threading.Lock()

        section = {}
        try:
            section = config.get_section("pump")
        except Exception:                          # noqa: BLE001 - defaults are fine
            pass
        self.min_rpm = float(section.get("min_rpm", DEFAULT_MIN_RPM))
        self.max_rpm = float(section.get("max_rpm", DEFAULT_MAX_RPM))
        self._speed_rpm = float(section.get("mixing_speed_rpm", DEFAULT_SPEED_RPM))

        self._host = section.get("host", DEFAULT_HOST)
        self._mixer_port = int(section.get("mixer_port", DEFAULT_MIXER_PORT))
        self._flowcell_port = int(section.get("flowcell_port", DEFAULT_FLOWCELL_PORT))
        self._req_timeout_s = float(
            section.get("request_timeout_s", DEFAULT_REQUEST_TIMEOUT_S))
        self._poll_interval_s = float(
            section.get("poll_interval_s", DEFAULT_POLL_INTERVAL_S))
        self._op_timeout_s = float(
            section.get("operation_timeout_s", DEFAULT_OPERATION_TIMEOUT_S))

        # _ctx is kept on the instance purely to hold the context alive: its
        # sockets die with it, so letting it fall out of scope here would break
        # both endpoints. Nothing else reads it -- the process owns it until exit.
        self._ctx = None
        self._mixer = None
        self._flowcell = None
        # A second socket to the flowcell server, used only by stop_shaking.
        # It has to be separate: a stop exists to reach a pump that is in the
        # middle of an operation, and that operation holds _flowcell's lock and
        # owns its REQ socket for its whole duration. Sharing either would make
        # the stop wait for the very thing it is trying to interrupt.
        self._flowcell_stop = None
        # Sockets for the status poll, again separate from the operation ones.
        # Status is most worth reading exactly while an operation is running,
        # and that operation owns its endpoint for its whole duration -- so a
        # poll sharing it would only ever report between operations. A shorter
        # timeout than an operation gets, too: a status read that hangs for
        # five seconds is a stale panel, not a failed move.
        self._mixer_status = None
        self._flowcell_status = None
        if not self.simulate:
            try:
                import zmq
            except ImportError as e:
                print("Pump: pyzmq not installed (%s); pump ops will fail "
                      "until it is." % e)
            else:
                self._ctx = zmq.Context()
                tmo = int(self._req_timeout_s * 1000)
                self._mixer = _Endpoint(
                    zmq, self._ctx, self._host, self._mixer_port, tmo)
                self._flowcell = _Endpoint(
                    zmq, self._ctx, self._host, self._flowcell_port, tmo)
                self._flowcell_stop = _Endpoint(
                    zmq, self._ctx, self._host, self._flowcell_port, tmo)
                stat_tmo = int(min(self._req_timeout_s, STATUS_TIMEOUT_S) * 1000)
                self._mixer_status = _Endpoint(
                    zmq, self._ctx, self._host, self._mixer_port, stat_tmo)
                self._flowcell_status = _Endpoint(
                    zmq, self._ctx, self._host, self._flowcell_port, stat_tmo)

    # -- mixing speed (advisory) ---------------------------------------------
    @property
    def speed(self):
        """The advisory mixing speed, in rpm (see module docstring)."""
        return self._speed_rpm

    def set_speed(self, rpm):
        """Record the advisory mixing speed. Returns ``(ok, detail)``.

        Bounds-checked rather than clamped: a campaign that asked for 5000 rpm on
        a 3000 rpm pump has a design space that does not match the hardware, so
        recording a value nothing could run at is worse than refusing now. The
        actual hardware speed is set in the pump dashboard, not here.
        """
        try:
            value = float(rpm)
        except (TypeError, ValueError):
            return False, "mixing speed must be a number, got %r" % (rpm,)
        if value != value:                          # NaN
            return False, "mixing speed must be a number, got %r" % (rpm,)
        if not self.min_rpm <= value <= self.max_rpm:
            return False, ("mixing speed must be %g..%g rpm, got %g"
                           % (self.min_rpm, self.max_rpm, value))
        with self._speed_lock:
            self._speed_rpm = value
            ok, detail = self._apply_speed(value)
        return ok, detail

    def _apply_speed(self, rpm):
        """Record the advisory speed only -- the dashboard owns real mix speed."""
        return True, "mixing speed %g rpm (advisory; dashboard sets actual)" % rpm

    # -- pump operations ------------------------------------------------------
    def mix(self):
        """Mix the sample in the mixer (mixing server ``sample_make``)."""
        # Advisory speed is folded into the detail so the sample record carries
        # a mixing speed even though the dashboard sets the actual value.
        return self._run_remote(self._mixer, "sample_make", None,
                                "mix at %g rpm" % self._speed_rpm)

    def clean_mixer(self):
        """Flush/clean the mixer (mixing server ``clean_all``)."""
        return self._run_remote(self._mixer, "clean_all", None, "clean_mixer")

    def draw_to_flowcell(self):
        """Draw mixed solution from the mixer into the in-use flowcell."""
        d = self._flowcell_device()
        return self._run_remote(self._flowcell, "draw_to_flowcell",
                                {"device": d},
                                "draw_to_flowcell (device %d)" % d)

    def aspirate_from_flowcell(self):
        """Expel solution back out of the in-use flowcell.

        (The flowcell server's ``aspirate_from_flowcell`` is a legacy name that
        actually expels; PALmixer keeps the same name for continuity.)
        """
        d = self._flowcell_device()
        return self._run_remote(self._flowcell, "aspirate_from_flowcell",
                                {"device": d},
                                "aspirate_from_flowcell (device %d)" % d)

    def wash_flowcell(self):
        """Wash the in-use flowcell after a sample has been returned to it."""
        d = self._flowcell_device()
        return self._run_remote(self._flowcell, "wash_flowcell",
                                {"device": d},
                                "wash_flowcell (device %d)" % d)

    def shake_sample(self):
        """Shake the sample in the in-use flowcell by cycling it back and forth.

        The one flowcell op whose target is named by the command rather than by
        a ``device`` field: the flowcell server has ``flow2_sample`` for
        flowcell2 and ``flow3_sample`` for flowcell3, and sets the device from
        the name itself. So this picks the command instead of the argument --
        id 1 -> ``flow2_sample``, id 2 -> ``flow3_sample``, the same mapping
        _flowcell_device() makes.

        Volume and cycle count come from the flowcell dashboard's own settings,
        like every other op's speeds and ports.
        """
        d = self._flowcell_device()
        command = "flow2_sample" if d == 0 else "flow3_sample"
        return self._run_remote(self._flowcell, command, None,
                                "shake_sample (%s)" % command)

    def stop_shaking(self):
        """Ask the flowcell server to end the active shake. Returns (ok, detail).

        Sends ``stop_shaking``, not the dashboard's plainer ``stop``:
        ``stop_shaking`` only touches an active ``flow2_sample``/
        ``flow3_sample`` cycle and does nothing (accepted=False) if whatever is
        running is not one -- the dashboard's own ``stop`` would instead finish
        *any* active recipe, which is not what a button labelled "Stop Shake"
        should be able to reach into and interrupt. Only one flowcell action
        runs at a time regardless of device, so this always means "whichever
        shake is running now", not "both flowcells at once".

        Not a motion op and deliberately not built like one: it goes out on its
        own socket rather than the one the running operation holds (see
        _flowcell_stop), and it does not poll for idle afterwards. Both follow
        from what it is for -- a stop that queued behind the shake it means to
        interrupt, or that then blocked for as long as that shake had left,
        would be no stop at all.

        Also accepted without the ``confirmed`` flag every other
        state-changing command needs, same as the dashboard's ``stop``.
        """
        if self.simulate:
            return True, "stop_shaking requested (simulate)"
        endpoint = self._flowcell_stop
        if endpoint is None:
            return False, "stop_shaking: pump transport unavailable (pyzmq missing)"
        with endpoint.lock:
            try:
                reply = endpoint.request({"command": "stop_shaking", "id": _new_id()})
            except PumpError as e:
                return False, "stop_shaking failed: %s" % e
        if not isinstance(reply, dict) or not reply.get("ok", False):
            err = (reply.get("error") if isinstance(reply, dict)
                   else None) or ("bad reply %r" % (reply,))
            return False, "stop_shaking rejected: %s" % err
        if not reply.get("accepted", True):
            return True, "stop_shaking sent, but nothing was shaking"
        return True, "stop_shaking requested"

    # -- status ---------------------------------------------------------------
    def status_snapshot(self):
        """Both servers' `status`, flattened to one row per physical pump.

        Four rows: pump0 and pump1 on the mixing server, flowcell2 and
        flowcell3 on the flowcell server. The two servers report quite
        differently -- the flowcell one has a per-pump list, the mixing one has
        parallel arrays and a single shared operation string -- so the shaping
        happens here rather than in the GUI, which then only has rows to draw.

        Never raises. A server that cannot be reached becomes an ``error`` on
        its entry and rows that say so, because "the pump dashboard is not
        answering" is exactly the thing the panel exists to show.
        """
        return {"servers": {"mixer": self._mixer_status_snapshot(),
                            "flowcell": self._flowcell_status_snapshot()},
                "time": time.time()}

    def _poll_status(self, endpoint, label):
        """One `status` request. Returns (reply_dict, error_string)."""
        if self.simulate:
            return None, None
        if endpoint is None:
            return None, "pump transport unavailable (pyzmq missing)"
        with endpoint.lock:
            try:
                reply = endpoint.request({"command": "status", "id": _new_id()})
            except PumpError as e:
                return None, str(e)
        if not isinstance(reply, dict) or not reply.get("ok", False):
            err = (reply.get("error") if isinstance(reply, dict) else None)
            return None, err or ("bad reply from %s" % label)
        return reply, None

    def _mixer_status_snapshot(self):
        reply, error = self._poll_status(self._mixer_status, "the mixing server")
        entry = {"port": self._mixer_port, "error": error,
                 "connected": False, "operation_active": False, "detail": "",
                 "pumps": []}
        if self.simulate:
            entry.update(connected=True, detail="simulate",
                         pumps=[{"name": n, "status": "simulate", "position": "--"}
                                for n in ("pump0", "pump1")])
            return entry
        if reply is None:
            entry["pumps"] = [{"name": n, "status": "no reply", "position": "--"}
                              for n in ("pump0", "pump1")]
            return entry
        entry["connected"] = bool(reply.get("connected"))
        entry["operation_active"] = bool(reply.get("operation_active"))
        entry["detail"] = _operation_summary(reply.get("operation"))
        # Parallel arrays, not a list of pumps -- and the operation string is
        # the server's, shared by both, so each row repeats it.
        names = _mixer_pump_names(reply)
        positions = reply.get("positions_ul") or []
        trusted = reply.get("positions_trusted") or []
        for i, name in enumerate(names):
            entry["pumps"].append({
                "name": name,
                "status": entry["detail"] or ("running" if entry["operation_active"]
                                              else "idle"),
                "position": _mixer_position(positions[i] if i < len(positions) else None,
                                            trusted[i] if i < len(trusted) else True),
            })
        return entry

    def _flowcell_status_snapshot(self):
        reply, error = self._poll_status(self._flowcell_status, "the flowcell server")
        entry = {"port": self._flowcell_port, "error": error,
                 "connected": False, "operation_active": False, "detail": "",
                 "stop_requested": False, "pumps": []}
        if self.simulate:
            entry.update(connected=True, detail="simulate",
                         pumps=[{"name": n, "status": "simulate", "position": "--"}
                                for n in ("flowcell2", "flowcell3")])
            return entry
        if reply is None:
            entry["pumps"] = [{"name": n, "status": "no reply", "position": "--"}
                              for n in ("flowcell2", "flowcell3")]
            return entry
        entry["connected"] = bool(reply.get("connected"))
        entry["operation_active"] = bool(reply.get("operation_active"))
        entry["stop_requested"] = bool(reply.get("emergency_stop_requested"))
        for i, pump in enumerate(reply.get("pumps") or []):
            entry["pumps"].append({
                "name": str(pump.get("name") or "flowcell%d" % (i + 2)),
                "status": str(pump.get("status") or ""),
                # "Position: 430.0 / 1000 uL  |  valve: 3" -- already meant for
                # a person to read, so it is passed through as it comes.
                "position": str(pump.get("position") or ""),
            })
        return entry

    # -- internals ------------------------------------------------------------
    @staticmethod
    def _flowcell_device():
        """Map PALmixer's in-use flowcell id (1/2) to the 0-based device int.

        id 1 -> device 0 (flowcell2), id 2 -> device 1 (flowcell3).
        """
        return 0 if state.get_flowcell_in_use() == 1 else 1

    def _run_remote(self, endpoint, command, extra, label):
        """Send one motion command and block until that server goes idle.

        Returns ``(ok, detail)``. Held under the endpoint's own lock for the
        entire send+poll, so two ops on the *same* pump never run concurrently
        -- and two ops on different pumps freely do.
        """
        if self.simulate:
            return True, "%s (simulate, no hardware)" % label
        if endpoint is None:
            return False, "%s: pump transport unavailable (pyzmq missing)" % label

        payload = {"command": command, "confirmed": True, "id": _new_id()}
        if extra:
            payload.update(extra)

        with endpoint.lock:
            try:
                reply = endpoint.request(payload)
            except PumpError as e:
                return False, "%s failed: %s" % (label, e)
            if not isinstance(reply, dict) or not reply.get("ok", False):
                err = (reply.get("error") if isinstance(reply, dict)
                       else None) or ("bad reply %r" % (reply,))
                return False, "%s rejected: %s" % (label, err)
            return self._wait_idle(endpoint, label)

    def _wait_idle(self, endpoint, label):
        """Poll ``status`` until ``operation_active`` is false (or timeout)."""
        deadline = time.monotonic() + self._op_timeout_s
        while True:
            time.sleep(self._poll_interval_s)
            try:
                st = endpoint.request({"command": "status", "id": _new_id()})
            except PumpError as e:
                return False, "%s: lost contact while waiting (%s)" % (label, e)
            if not isinstance(st, dict) or not st.get("ok", False):
                return False, "%s: status query failed: %r" % (label, st)
            if not st.get("operation_active", False):
                return True, "%s complete" % label
            if time.monotonic() > deadline:
                return False, ("%s: still active after %g s"
                               % (label, self._op_timeout_s))
