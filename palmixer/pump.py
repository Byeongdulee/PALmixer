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

Flow-cell selection is the 0-based ``device`` int the flowcell server expects.
PALmixer tracks the in-use flowcell as an id (1 or 2) in state; this maps
id 1 -> device 0 (flowcell2) and id 2 -> device 1 (flowcell3), so the operator's
existing "which flowcell" choice drives the pump automatically.

**Queued, not done.** A ZMQ reply means the command was accepted and queued, not
that motion finished. To keep the blocking ``(ok, detail)`` contract every
motion op waits: it sends the command, then polls the server's ``status`` until
``operation_active`` goes false before returning. The whole send+poll runs under
a single lock, so two pump ops never overlap -- which is what lets a workflow
fire ``clean_mixer()`` on a daemon thread and have a later ``draw_to_flowcell()``
simply block until the clean has finished (see workflows.py make_sample).

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


class PumpError(Exception):
    """Raised on a ZMQ transport failure talking to a pump server."""


def _new_id():
    return "palmixer-%s" % uuid.uuid4().hex[:8]


class _Endpoint:
    """One REQ socket to one pump server, with lazy-pirate recovery.

    A REQ socket that misses a reply is stuck (it cannot send again), so on any
    send/recv failure the socket is dropped and reopened on the next call.
    """

    def __init__(self, zmq, ctx, host, port, timeout_ms):
        self._zmq = zmq
        self._ctx = ctx
        self._addr = "tcp://%s:%d" % (host, port)
        self._timeout_ms = timeout_ms
        self._sock = None

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
        self._lock = threading.Lock()

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
        with self._lock:
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

    # -- internals ------------------------------------------------------------
    @staticmethod
    def _flowcell_device():
        """Map PALmixer's in-use flowcell id (1/2) to the 0-based device int.

        id 1 -> device 0 (flowcell2), id 2 -> device 1 (flowcell3).
        """
        return 0 if state.get_flowcell_in_use() == 1 else 1

    def _run_remote(self, endpoint, command, extra, label):
        """Send one motion command and block until the server goes idle.

        Returns ``(ok, detail)``. Held under ``self._lock`` for the entire
        send+poll so two pump ops never run concurrently.
        """
        if self.simulate:
            return True, "%s (simulate, no hardware)" % label
        if endpoint is None:
            return False, "%s: pump transport unavailable (pyzmq missing)" % label

        payload = {"command": command, "confirmed": True, "id": _new_id()}
        if extra:
            payload.update(extra)

        with self._lock:
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
