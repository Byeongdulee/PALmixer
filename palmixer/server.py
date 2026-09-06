# -*- coding: utf-8 -*-
"""Headless PALmixer server.

Owns the UR3 robot (via UR_12idb), the PAL12idb transport/AprilTag library,
the placeholder pump, and the EPICS motor. Exposes them to the PyQt GUI (or
any other ZMQ client) as a small command vocabulary (see commands.py) over a
ZMQ REQ/REP socket, and reports motion start/success/failure over MQTT (see
mqtt_status.py).

Run directly:
    python -m palmixer.server              # drives real hardware
    python -m palmixer.server --simulate   # no robot/motor/pump I/O; for GUI/dev testing
"""

import argparse
import sys
import threading
import time

from . import commands as cmd
from . import config
from .mqtt_status import (
    MQTTPublisher, PHASE_FAILURE, PHASE_STARTED, PHASE_SUCCESS,
    motion_payload, motion_topic, new_trace, state_payload, state_topic,
)
from .pump import Pump
from .motor import Motor
from .zmq_transport import ZMQCommandServer

STATE_IDLE = "IDLE"
STATE_BUSY = "BUSY"


class PALmixerServer:
    """Wires the ZMQ command server to robot/pump/motor actions, with MQTT status."""

    def __init__(self, simulate=False):
        self.simulate = simulate
        cfg = config.get_config()
        self._beamline = cfg["mqtt"].get("beamline", "12idb")

        self._busy_lock = threading.Lock()
        self._busy = False
        self._current_action = None

        self._mqtt = MQTTPublisher(
            host=cfg["mqtt"].get("host"), port=cfg["mqtt"].get("port"),
            client_id_prefix="palmixer-server",
        )

        self.rob = None
        self.PAL12idb = None
        self.pump = Pump()
        self.pump.initialize()
        self.motor = Motor(cfg["motor"].get("pv", "12idb:m6"))

        if not self.simulate:
            ur12idb_path = cfg["robot"].get("ur12idb_path")
            if ur12idb_path and ur12idb_path not in sys.path:
                sys.path.insert(0, ur12idb_path)
            import robot12idb  # noqa: E402  (path must be set first)
            from . import PAL12idb as pal

            self.rob = robot12idb.UR3(name=cfg["robot"].get("name", "UR3"))
            self.PAL12idb = pal
        else:
            print("PALmixerServer: --simulate mode, no hardware will be touched.")

        self.zmq_server = ZMQCommandServer(
            dispatch_fn=self.dispatch,
            fast_dispatch_fn=self.fast_dispatch,
            port=cfg["zmq"].get("port", 9880),
        )

    # -- status ------------------------------------------------------------
    def fast_dispatch(self, msg):
        """Read-only commands, answered without touching hardware. Returns
        None for anything else so it falls through to dispatch()."""
        if msg.strip() == cmd.STATUS:
            return STATE_BUSY if self._busy else STATE_IDLE
        return None

    def _publish_state(self, state, action=None):
        self._mqtt.publish(state_topic(self._beamline), state_payload(state, action),
                            qos=0, retain=True)

    # -- command dispatch ----------------------------------------------------
    def dispatch(self, msg):
        """Parse one command string and either run it or reject it as busy.

        Must return promptly: hardware actions are started on a background
        thread and this returns "ACCEPTED" right away (see module docstring
        of zmq_transport.py for why that's required).
        """
        parts = msg.strip().split()
        if not parts:
            return "ERROR: empty command"
        name, args = parts[0], parts[1:]

        try:
            action_fn, action_label = self._resolve(name, args)
        except ValueError as e:
            return "ERROR: %s" % e

        with self._busy_lock:
            if self._busy:
                return "ERROR: busy (running %r)" % self._current_action
            self._busy = True
            self._current_action = action_label

        trace = new_trace()
        self._publish_state(STATE_BUSY, action_label)
        self._mqtt.publish(motion_topic(self._beamline),
                            motion_payload(action_label, PHASE_STARTED, True, trace=trace))

        threading.Thread(target=self._run_action, args=(action_fn, action_label, trace),
                          daemon=True).start()
        return "ACCEPTED"

    def _resolve(self, name, args):
        """Map a command name + args to a zero-arg callable and a human label.

        Raises ValueError for anything unrecognized or malformed.
        """
        if name == cmd.SEARCH_APRILTAG:
            if len(args) != 1 or args[0] not in cmd.STATIONS:
                raise ValueError("usage: %s <%s>" % (cmd.SEARCH_APRILTAG, "|".join(cmd.STATIONS)))
            station = args[0]
            label = "%s %s" % (cmd.SEARCH_APRILTAG, station)
            return (lambda: self._search_apriltag(station)), label

        if name in cmd.TRANSPORT_FUNCTIONS:
            if args:
                raise ValueError("%s takes no arguments" % name)
            return (lambda: self._run_transport(name)), name

        if name == cmd.MOTOR_TWEAK:
            if len(args) != 2 or args[0] not in (cmd.MOTOR_FORWARD, cmd.MOTOR_REVERSE):
                raise ValueError("usage: %s <forward|reverse> <step>" % cmd.MOTOR_TWEAK)
            direction, step_str = args
            try:
                step = float(step_str)
            except ValueError:
                raise ValueError("step must be numeric, got %r" % step_str)
            label = "%s %s %s" % (cmd.MOTOR_TWEAK, direction, step)
            return (lambda: self._motor_tweak(direction, step)), label

        if name == cmd.PUMP:
            if len(args) != 1 or args[0] not in cmd.PUMP_OPS:
                raise ValueError("usage: %s <%s>" % (cmd.PUMP, "|".join(cmd.PUMP_OPS)))
            op = args[0]
            label = "%s %s" % (cmd.PUMP, op)
            return (lambda: self._run_pump(op)), label

        raise ValueError("unknown command %r" % name)

    def _run_action(self, action_fn, action_label, trace):
        """Runs on a background thread: execute the action, publish the
        outcome, and clear the busy flag."""
        try:
            detail = action_fn()
            self._mqtt.publish(motion_topic(self._beamline),
                                motion_payload(action_label, PHASE_SUCCESS, True,
                                               detail=detail or "", trace=trace))
        except Exception as e:
            print("PALmixerServer: action %r failed: %s" % (action_label, e))
            self._mqtt.publish(motion_topic(self._beamline),
                                motion_payload(action_label, PHASE_FAILURE, False,
                                               detail=str(e), trace=trace))
        finally:
            with self._busy_lock:
                self._busy = False
                self._current_action = None
            self._publish_state(STATE_IDLE)

    # -- individual actions --------------------------------------------------
    def _search_apriltag(self, station):
        if self.simulate:
            time.sleep(2)
            return "simulated AprilTag search for %s" % station
        pos = self.PAL12idb.locate_apriltag(self.rob, pos=station)
        return "found %s at %s" % (station, pos)

    def _run_transport(self, name):
        if self.simulate:
            time.sleep(2)
            return "simulated transport %s" % name
        getattr(self.PAL12idb, name)(self.rob)
        return "%s complete" % name

    def _motor_tweak(self, direction, step):
        if self.simulate:
            time.sleep(0.5)
            return "simulated motor tweak %s by %s" % (direction, step)
        new_pos = self.motor.tweak(direction, step)
        return "motor now at %s" % new_pos

    def _run_pump(self, op):
        ok, detail = getattr(self.pump, op)()
        if not ok:
            raise RuntimeError(detail)
        return detail

    # -- lifecycle ------------------------------------------------------------
    def start(self):
        self._publish_state(STATE_IDLE)
        self.zmq_server.start()

    def stop(self):
        self.zmq_server.stop()
        self._mqtt.close()


def main():
    parser = argparse.ArgumentParser(description="PALmixer headless control server")
    parser.add_argument("--simulate", action="store_true",
                         help="do not touch real robot/motor hardware")
    args = parser.parse_args()

    server = PALmixerServer(simulate=args.simulate)
    server.start()
    print("PALmixer server running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
