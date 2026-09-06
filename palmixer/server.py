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
import json
import sys
import threading
import time

from . import commands as cmd
from . import config
from . import state
from .mqtt_status import (
    MQTTPublisher, PHASE_FAILURE, PHASE_STARTED, PHASE_SUCCESS,
    motion_payload, motion_topic, new_trace, state_payload, state_topic,
    tracking_topic,
)
from .pump import Pump
from .motor import Motor
from .workflows import Workflows, WorkflowError
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
        self._current_trace = None  # trace id of the action running now, for _publish_step

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

        self.workflows = Workflows(self.PAL12idb, self.rob, self.pump, self.motor,
                                    on_step=self._publish_step, simulate=self.simulate)
        state.add_listener(self._publish_tracking)

        self.zmq_server = ZMQCommandServer(
            dispatch_fn=self.dispatch,
            fast_dispatch_fn=self.fast_dispatch,
            port=cfg["zmq"].get("port", 9880),
        )

    # -- status ------------------------------------------------------------
    def fast_dispatch(self, msg):
        """Read-only commands, answered without touching hardware -- including
        while a multi-minute workflow is running, since the ZMQ REP socket is
        strictly one-in-flight and dispatch() occupies it for the duration.
        Returns None for anything else so it falls through to dispatch()."""
        parts = msg.strip().split()
        if not parts:
            return None
        name, args = parts[0], parts[1:]

        if name == cmd.STATUS:
            return STATE_BUSY if self._busy else STATE_IDLE

        if name == cmd.GET_STATE:
            return json.dumps(state.snapshot())

        if name == cmd.SET_FLOWCELL:
            if len(args) != 1:
                return "ERROR: usage: %s <%s>" % (
                    cmd.SET_FLOWCELL, "|".join(str(i) for i in cmd.FLOWCELL_IDS))
            try:
                fc = int(args[0])
            except ValueError:
                return "ERROR: flowcell id must be an integer, got %r" % args[0]
            if fc not in cmd.FLOWCELL_IDS:
                return "ERROR: flowcell id must be one of %s" % (list(cmd.FLOWCELL_IDS),)
            if self.PAL12idb is not None:
                self.PAL12idb.set_flowcell_ID(fc)
            state.set_flowcell_in_use(fc)
            return "OK"

        if name == cmd.SET_LOCATION:
            if len(args) != 2 or args[0] not in cmd.LOCATION_KEYS:
                return "ERROR: usage: %s <%s> <value>" % (
                    cmd.SET_LOCATION, "|".join(cmd.LOCATION_KEYS))
            what, value = args
            try:
                state.set_location(what, value)
            except ValueError as e:
                return "ERROR: %s" % e
            return "OK"

        if name == cmd.TEACH_CAROUSEL_SLOT:
            if len(args) != 1:
                return "ERROR: usage: %s <slot>" % cmd.TEACH_CAROUSEL_SLOT
            try:
                slot = int(args[0])
            except ValueError:
                return "ERROR: slot must be an integer, got %r" % args[0]
            if self.simulate:
                position = float(slot)  # arbitrary, distinct per slot; no real motor to read
            else:
                try:
                    position = self.motor.read()
                except Exception as e:
                    return "ERROR: could not read motor position: %s" % e
            state.teach_carousel_slot(slot, position)
            return "OK"

        return None

    def _publish_state(self, state_value, action=None):
        self._mqtt.publish(state_topic(self._beamline), state_payload(state_value, action),
                            qos=0, retain=True)

    def _publish_step(self, step, phase, detail=""):
        """Wired as Workflows' on_step: one motion message per workflow step,
        correlated to the parent make_sample/unload_sample action by trace id."""
        self._mqtt.publish(motion_topic(self._beamline),
                            motion_payload(step, phase, phase != PHASE_FAILURE,
                                           detail=detail, trace=self._current_trace))

    def _publish_tracking(self, snapshot):
        self._mqtt.publish(tracking_topic(self._beamline), snapshot, qos=0, retain=True)

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
        self._current_trace = trace
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

        if name in (cmd.PUSH_POSITIONS, cmd.PULL_POSITIONS):
            if args:
                raise ValueError("%s takes no arguments" % name)
            return (lambda: self._sync_positions(name)), name

        if name in cmd.TRANSPORT_FUNCTIONS:
            if args:
                raise ValueError("%s takes no arguments" % name)
            self._require_positions(self.PAL12idb.TRANSPORT_STATIONS[name] if self.PAL12idb else ())
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

        if name == cmd.MAKE_SAMPLE:
            if len(args) != 1:
                raise ValueError("usage: %s <slot>" % cmd.MAKE_SAMPLE)
            try:
                slot = int(args[0])
            except ValueError:
                raise ValueError("slot must be an integer, got %r" % args[0])
            self._require_positions(self.workflows.stations_for_make_sample())
            try:
                self.workflows._check_make_sample(slot)
            except WorkflowError as e:
                raise ValueError(str(e))
            label = "%s %s" % (cmd.MAKE_SAMPLE, slot)
            return (lambda: self.workflows.make_sample(slot)), label

        if name == cmd.UNLOAD_SAMPLE:
            if args:
                raise ValueError("%s takes no arguments" % cmd.UNLOAD_SAMPLE)
            self._require_positions(self.workflows.stations_for_unload_sample())
            try:
                self.workflows._check_unload_sample()
            except WorkflowError as e:
                raise ValueError(str(e))
            return (lambda: self.workflows.unload_sample()), cmd.UNLOAD_SAMPLE

        raise ValueError("unknown command %r" % name)

    def _require_positions(self, station_keys):
        """Refuse up front (as a synchronous ERROR reply) rather than letting
        a transport call fail deep inside a multi-minute action once it is
        already ACCEPTED. No-op in simulate mode, where there is no PAL12idb
        and therefore nothing to check.

        Safe to call from the reply path: check_positions_defined() reads
        waypoints.ini and does no Channel Access, so it cannot stall here."""
        if self.simulate or self.PAL12idb is None or not station_keys:
            return
        missing = self.PAL12idb.check_positions_defined(station_keys)
        if missing:
            labels = [cmd.STATION_LABELS.get(s, s) for s in missing]
            raise ValueError(cmd.position_not_configured_error(labels))

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
            self._current_trace = None
            self._publish_state(STATE_IDLE)

    # -- individual actions --------------------------------------------------
    def _search_apriltag(self, station):
        if self.simulate:
            time.sleep(2)
            return "simulated AprilTag search for %s" % station
        pos = self.PAL12idb.locate_apriltag(self.rob, pos=station)
        return "found %s at %s" % (station, pos)

    def _sync_positions(self, name):
        """Explicit waypoints.ini <-> EPICS PV sync, the only Channel Access
        this package does on the waypoint PVs. Runs on the worker thread
        because a disconnected PV makes it slow, which is exactly why moves
        read the ini instead (see PAL12idb.get_position)."""
        if self.simulate:
            time.sleep(0.5)
            return "simulated %s" % name
        if name == cmd.PUSH_POSITIONS:
            return self.PAL12idb.push_positions_to_pvs()
        return self.PAL12idb.pull_positions_from_pvs()

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
        self._publish_tracking(state.snapshot())
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
