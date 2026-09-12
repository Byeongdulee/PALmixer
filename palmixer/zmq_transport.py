# -*- coding: utf-8 -*-
"""ZeroMQ REQ/REP control plane for PALmixer.

Mirrors the ZMQClient / ZMQCommandServer pair used in APS12_SAXSDaq
(tools/zmq_client.py, tools/zmq_server.py):

- Strict REQ/REP, one command string -> one reply string.
- A fresh REQ socket per client call (a timed-out REQ socket is unusable, so
  per-call sockets keep a dropped reply cleanly retryable).
- pyzmq is imported lazily so this module (and anything that just wants the
  command-name constants) imports fine even where pyzmq is not installed.

A ZMQ REP socket only ever has one request in flight at a time (that is
enforced by the REQ/REP pattern itself, independent of any threading on the
server side), so ``dispatch_fn`` must return quickly. For long-running
hardware actions (robot moves, motor tweaks, pump ops), ``dispatch_fn`` is
expected to start the work on its own background thread and reply
"ACCEPTED" immediately, reporting completion asynchronously over MQTT -- see
palmixer/server.py, which owns that worker thread and the busy/idle state.
``fast_dispatch_fn`` is an optional separate path for trivially-fast,
read-only commands (e.g. "status"); everything else goes to ``dispatch_fn``.
"""

import threading


class ZMQError(Exception):
    """Raised when a command cannot be delivered or no reply arrives in time."""


class ZMQClient:
    """Thin client used by the GUI to talk to :class:`ZMQCommandServer`."""

    def __init__(self, host="localhost", port=9880, timeout_ms=30000):
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms

    def send(self, command, timeout_ms=None):
        """Send one command string, return the server's reply string.

        Raises ZMQError on connection failure or timeout (no reply within
        timeout_ms). The caller decides whether to retry.
        """
        try:
            import zmq
        except ImportError as e:
            raise ZMQError("pyzmq not installed: %s" % e)

        tmo = self.timeout_ms if timeout_ms is None else timeout_ms
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        try:
            sock.connect("tcp://%s:%d" % (self.host, self.port))
            sock.send_string(command)
            poller = zmq.Poller()
            poller.register(sock, zmq.POLLIN)
            if dict(poller.poll(tmo)).get(sock) == zmq.POLLIN:
                return sock.recv_string()
            raise ZMQError(
                "no reply within %d ms from %s:%d (command: %r)"
                % (tmo, self.host, self.port, command)
            )
        finally:
            sock.close(0)

    def status(self, timeout_ms=5000):
        """Return the server's status string ('IDLE'/'BUSY'), or '' on error."""
        try:
            return self.send("status", timeout_ms=timeout_ms)
        except ZMQError:
            return ""


class ZMQCommandServer:
    """REQ/REP ZeroMQ server that exposes PALmixer commands to the GUI.

    Protocol (space-delimited, no newline):
        client sends:  "<command> [arg1] [arg2] ..."
        server replies: "OK", "ACCEPTED", "ERROR: <reason>", or a value string

    The command vocabulary lives entirely in ``dispatch_fn``/``fast_dispatch_fn``;
    this class only provides the transport and a background poll loop on a
    single daemon thread.
    """

    def __init__(self, dispatch_fn, port=9880, cmd_timeout=10,
                 fast_dispatch_fn=None):
        self._dispatch = dispatch_fn
        self._fast_dispatch = fast_dispatch_fn
        self._port = port
        self._cmd_timeout = cmd_timeout
        self._thread = None
        self._running = False

    @property
    def port(self):
        return self._port

    def start(self, port=None):
        """Bind the REP socket, then serve on a background thread.

        The bind happens here, on the caller's thread, so that a failure --
        almost always the port still held by an older server -- is raised at
        the caller. Bound inside ``_serve`` instead, the exception killed only
        the daemon thread, while ``start`` had already announced it was
        listening and the caller's ``while True`` loop kept a process alive
        that answered nothing.
        """
        if port is not None:
            self._port = port
        if self._thread and self._thread.is_alive():
            return

        try:
            import zmq
        except ImportError as e:
            raise ZMQError("pyzmq not installed - run: pip install pyzmq (%s)" % e)

        ctx = zmq.Context()
        sock = ctx.socket(zmq.REP)
        try:
            sock.bind("tcp://*:%d" % self._port)
        except zmq.ZMQError as e:
            sock.close(0)
            ctx.term()
            raise ZMQError("cannot bind tcp://*:%d (%s) -- another server is "
                           "probably already running on that port"
                           % (self._port, e))
        sock.setsockopt(zmq.RCVTIMEO, 500)  # poll every 500 ms for clean shutdown

        self._running = True
        self._thread = threading.Thread(target=self._serve, args=(ctx, sock),
                                        daemon=True)
        self._thread.start()
        print("ZMQ command server listening on tcp://*:%d" % self._port)

    def stop(self):
        self._running = False

    def is_alive(self):
        return self._thread is not None and self._thread.is_alive()

    def _serve(self, ctx, sock):
        """Poll the socket bound by :meth:`start` until :meth:`stop`."""
        import zmq

        while self._running:
            try:
                msg = sock.recv_string()
            except zmq.Again:
                continue
            except Exception as e:
                print("ZMQ recv error: %s" % e)
                break

            reply = None
            if self._fast_dispatch is not None:
                try:
                    reply = self._fast_dispatch(msg)
                except Exception as e:
                    reply = "ERROR: %s" % e

            if reply is None:
                # dispatch_fn must return promptly (see module docstring):
                # long actions are expected to be started on a background
                # thread by the caller-supplied dispatch_fn itself.
                result = [None]
                done = threading.Event()

                def _cmd(m=msg):
                    try:
                        result[0] = self._dispatch(m)
                    except Exception as e:
                        result[0] = "ERROR: %s" % e
                    finally:
                        done.set()

                threading.Thread(target=_cmd, daemon=True).start()
                done.wait(timeout=self._cmd_timeout)
                reply = result[0] if result[0] is not None else "TIMEOUT"

            sock.send_string(reply)

        sock.close()
        ctx.term()
        print("ZMQ command server stopped")
