# -*- coding: utf-8 -*-
"""Headless PALmixer server.

Owns the UR3 robot (via UR_12idb), the PAL12idb transport/AprilTag library,
the pump (a ZMQ client to apssector12_pump_control), and the EPICS motor.
Exposes them to the PyQt GUI (or
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
        # How the last action ended -- {action, ok, detail, time}, success as
        # well as failure. Rides on the get_state snapshot so a client learns
        # about it from its ordinary poll: the motion MQTT topic is the only
        # other place an async outcome is reported, and it is silent whenever
        # the broker is unreachable or paho is not installed -- which left the
        # GUI showing nothing at all for a search that had already given up.
        self._last_result = None
        self._search_stop_event = None  # set while a search_apriltag action is running

        self._mqtt = MQTTPublisher(
            host=cfg["mqtt"].get("host"), port=cfg["mqtt"].get("port"),
            client_id_prefix="palmixer-server",
        )

        self.rob = None
        self.PAL12idb = None
        self.pump = Pump(simulate=self.simulate)
        self.motor = Motor(cfg["motor"].get("pv", "12idb:m6"))

        if not self.simulate:
            ur12idb_path = cfg["robot"].get("ur12idb_path")
            if ur12idb_path and ur12idb_path not in sys.path:
                sys.path.insert(0, ur12idb_path)
            import robot12idb  # noqa: E402  (path must be set first)
            from . import PAL12idb as pal

            # Pass `ip` explicitly so the connection uses *our* config
            # (json/palmixer_config.json, overridable with PALMIXER_ROBOT_IP)
            # instead of UR_12idb's own list_of_robots.json -- that file is
            # a default for other UR_12idb consumers, not a second place
            # this package's robot address has to be kept in sync.
            self.rob = robot12idb.UR3(name=cfg["robot"].get("name", "UR3"),
                                       ip=cfg["robot"].get("ip"))
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
            # The pump's speed is a live setting rather than tracked state, so it is
            # added here (where the pump is) rather than in state.py -- but it rides
            # on the same snapshot, because a client polling for "what is the machine
            # set to" should not need a second round trip for it.
            return json.dumps(dict(state.snapshot(),
                                   mixing_speed=self.pump.speed,
                                   mixing_speed_limits=[self.pump.min_rpm,
                                                        self.pump.max_rpm],
                                   last_result=self._last_result))

        if name == cmd.STOP_SEARCH:
            return self._stop_search()

        if name in (cmd.SET_POSITION_HERE, cmd.SET_ORIENTATION_HERE):
            return self._set_position_here(args, orientation_only=(
                name == cmd.SET_ORIENTATION_HERE))

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
            # Both checked before the motor read, so a bad slot or an
            # unconfigured step says so instead of reporting whatever the
            # hardware read had to say.
            try:
                slot = state.validate_slot(args[0])
                state.require_carousel_step()
            except ValueError as e:
                return "ERROR: %s" % e
            if self.simulate:
                position = float(slot)  # arbitrary, distinct per slot; no real motor to read
            else:
                try:
                    position = self.motor.read()
                except Exception as e:
                    return "ERROR: could not read motor position: %s" % e
            state.teach_carousel_slot(slot, position)
            return "OK"

        if name == cmd.RESET_CAROUSEL:
            if args:
                return "ERROR: %s takes no arguments" % cmd.RESET_CAROUSEL
            state.reset_carousel()
            return "OK"

        # The sample ID is the rest of the line, so it may contain spaces.
        # Rejoining the split args normalises internal whitespace, which is
        # what state.set_sample_id would do anyway.
        if name == cmd.SET_SAMPLE_ID:
            if len(args) < 2:
                return "ERROR: usage: %s <slot> <sample id>" % cmd.SET_SAMPLE_ID
            try:
                state.set_sample_id(args[0], " ".join(args[1:]))
            except ValueError as e:
                return "ERROR: %s" % e
            return "OK"

        if name == cmd.GET_SAMPLE_ID:
            if len(args) != 1:
                return "ERROR: usage: %s <slot>" % cmd.GET_SAMPLE_ID
            try:
                sample_id = state.get_sample_id(args[0])
            except ValueError as e:
                return "ERROR: %s" % e
            # An unused slot is a fact, not an error -- report it with the
            # vocabulary's first-class "unknown", as the locations do.
            return sample_id if sample_id else state.UNKNOWN

        if name == cmd.CLEAR_SAMPLE_ID:
            if len(args) != 1:
                return "ERROR: usage: %s <slot>" % cmd.CLEAR_SAMPLE_ID
            try:
                state.clear_sample_id(args[0])
            except ValueError as e:
                return "ERROR: %s" % e
            return "OK"

        if name == cmd.GET_CAROUSEL_ID:
            if args:
                return "ERROR: %s takes no arguments" % cmd.GET_CAROUSEL_ID
            return state.get_carousel_id()

        # Fast, like set_flowcell: it is a setpoint the next mix picks up, not an
        # action. Answering it mid-workflow is harmless -- and refused below when it
        # would land between the guard and the mix of a running make_sample.
        if name == cmd.GET_MIXING_SPEED:
            if args:
                return "ERROR: %s takes no arguments" % cmd.GET_MIXING_SPEED
            return "%g" % self.pump.speed

        if name == cmd.SET_MIXING_SPEED:
            if len(args) != 1:
                return "ERROR: usage: %s <rpm>" % cmd.SET_MIXING_SPEED
            if self._busy:
                # make_sample reads the speed when it reaches its mix step, so a
                # change landing mid-run would apply to a sample the caller thought
                # was already settled -- and the record would name the wrong speed.
                return ("ERROR: busy (running %r); the mixing speed cannot change "
                        "mid-action" % self._current_action)
            ok, detail = self.pump.set_speed(args[0])
            return ("OK %s" % detail) if ok else ("ERROR: %s" % detail)

        if name == cmd.MOUNT_CAROUSEL:
            return self._mount_carousel(args)

        return None

    def _mount_carousel(self, args):
        """Declare which carousel is now in the machine, and everything on it.

        One transition, not a clear plus N tags: a swap that half-applied would
        leave this server's slot table disagreeing with the physical carousel,
        and every sample measured afterwards would carry the wrong ID.

        Refused while an action is running. It is a fast command, so the server
        would happily answer mid-workflow -- but `make_sample` re-reads the slot
        table as it goes, so replacing the carousel underneath a running one
        would mix from a slot that is no longer there.
        """
        if len(args) not in (1, 2):
            return "ERROR: usage: %s <carousel id> [base64 {slot: sample_id}]" \
                % cmd.MOUNT_CAROUSEL
        if self._busy:
            return "ERROR: busy (running %r); the carousel cannot change mid-action" \
                % self._current_action
        try:
            samples = cmd.decode_inventory(args[1]) if len(args) == 2 else {}
            carousel_id = state.mount_carousel(args[0], samples)
        except ValueError as e:
            return "ERROR: %s" % e
        detail = "mounted carousel %s with %d sample(s)" % (carousel_id, len(samples))
        if state.carousel_reference_stale():
            # Not refused here -- mounting is bookkeeping and always allowed. The
            # workflows refuse, which is where a wrong position would do harm.
            detail += ("; the taught slot positions belong to carousel %r, so teach "
                       "one slot on this one before making a sample"
                       % state.reference_carousel_id())
        print(detail)
        return "OK %s" % detail

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
            skip_roll = len(args) == 2 and args[1] == cmd.SKIP_ROLL
            if skip_roll:
                args = args[:1]
            if len(args) != 1 or args[0] not in cmd.STATIONS:
                raise ValueError("usage: %s <%s> [%s]" % (
                    cmd.SEARCH_APRILTAG, "|".join(cmd.STATIONS), cmd.SKIP_ROLL))
            station = args[0]
            label = "%s %s%s" % (cmd.SEARCH_APRILTAG, station,
                                 " (skip roll)" if skip_roll else "")
            # Created here (on the synchronous reply path, before ACCEPTED is
            # sent) so a stop_search sent right after is guaranteed to find it.
            self._search_stop_event = threading.Event()
            return (lambda: self._search_apriltag(station, skip_roll)), label

        if name in (cmd.PUSH_POSITIONS, cmd.PULL_POSITIONS):
            if args:
                raise ValueError("%s takes no arguments" % name)
            return (lambda: self._sync_positions(name)), name

        if name in cmd.TRANSPORT_FUNCTIONS:
            if args:
                raise ValueError("%s takes no arguments" % name)
            self._require_positions(self.PAL12idb.TRANSPORT_STATIONS[name] if self.PAL12idb else ())
            return (lambda: self._run_transport(name)), name

        if name == cmd.GOTO_POSITION:
            if len(args) != 1 or args[0] not in cmd.STATIONS:
                raise ValueError("usage: %s <%s>" % (cmd.GOTO_POSITION, "|".join(cmd.STATIONS)))
            station = args[0]
            # Refuse an untaught station here rather than letting the move fail
            # once it has already been ACCEPTED -- same check, and the same
            # "search its AprilTag first" message, the transports get.
            self._require_positions((station,))
            label = "%s %s" % (cmd.GOTO_POSITION, station)
            return (lambda: self._goto_position(station)), label

        if name == cmd.TWEAK_ORIENTATION:
            if len(args) != 2 or args[0].lower() not in cmd.ORIENTATION_AXES:
                raise ValueError("usage: %s <%s> <degrees>" % (
                    cmd.TWEAK_ORIENTATION, "|".join(cmd.ORIENTATION_AXES)))
            axis = args[0].lower()
            try:
                degrees = float(args[1])
            except ValueError:
                raise ValueError("degrees must be numeric, got %r" % args[1])
            label = "%s %s %s" % (cmd.TWEAK_ORIENTATION, axis, degrees)
            return (lambda: self._tweak_orientation(axis, degrees)), label

        if name == cmd.ZALIGN:
            if args:
                raise ValueError("%s takes no arguments" % cmd.ZALIGN)
            return self._zalign, cmd.ZALIGN

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
            if not args:
                raise ValueError("usage: %s <slot> [sample id]" % cmd.MAKE_SAMPLE)
            try:
                slot = int(args[0])
            except ValueError:
                raise ValueError("slot must be an integer, got %r" % args[0])
            # Assigned here rather than deep in the workflow so the ID the slot
            # will be tagged with appears in the ACCEPTED log line and in every
            # MQTT motion payload for the run.
            sample_id = " ".join(args[1:]) if len(args) > 1 else state.new_sample_id()
            self._require_positions(self.workflows.stations_for_make_sample())
            try:
                self.workflows._check_make_sample(slot, sample_id)
            except WorkflowError as e:
                raise ValueError(str(e))
            label = "%s %s (%s)" % (cmd.MAKE_SAMPLE, slot, sample_id)
            return (lambda: self.workflows.make_sample(slot, sample_id)), label

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
            self._last_result = {"action": action_label, "ok": True,
                                 "detail": detail or "", "time": time.time()}
            self._mqtt.publish(motion_topic(self._beamline),
                                motion_payload(action_label, PHASE_SUCCESS, True,
                                               detail=detail or "", trace=trace))
        except Exception as e:
            print("PALmixerServer: action %r failed: %s" % (action_label, e))
            self._last_result = {"action": action_label, "ok": False,
                                 "detail": str(e), "time": time.time()}
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
    def _stop_search(self):
        """Abort an in-progress search_apriltag: sets the cooperative stop
        event the search loop polls between moves, and -- the part that
        actually matters for how quickly it stops -- tells the robot
        controller to halt whatever move is in flight right now, rather than
        waiting for that move to finish on its own."""
        if (self._current_action is None
                or not self._current_action.startswith(cmd.SEARCH_APRILTAG + " ")):
            return "ERROR: no AprilTag search is running"
        event = self._search_stop_event
        if event is not None:
            event.set()
        if not self.simulate and self.rob is not None:
            try:
                self.rob.robot.stopj()
            except Exception as e:
                return "ERROR: failed to stop robot: %s" % e
        return "OK"

    def _set_position_here(self, args, orientation_only=False):
        """Teach a station from the robot's current pose.

        A fast command rather than an action: it reads a pose and writes
        waypoints.ini, with no motion of its own. Refused while busy, though --
        unlike the other fast commands this one samples where the arm *is*, and
        a pose read out of the middle of a move records a point the robot was
        only passing through.

        orientation_only keeps the station's taught X/Y/Z and replaces just
        RX/RY/RZ, for a station the AprilTag search places well but cannot
        orient.
        """
        verb = cmd.SET_ORIENTATION_HERE if orientation_only else cmd.SET_POSITION_HERE
        if len(args) != 1 or args[0] not in cmd.STATIONS:
            return "ERROR: usage: %s <%s>" % (verb, "|".join(cmd.STATIONS))
        station = args[0]
        if self._busy:
            return ("ERROR: busy (running %r); the robot has to be standing still "
                    "to teach from it" % self._current_action)
        if self.simulate:
            return "OK simulated %s of %s" % (verb, station)
        record = (self.PAL12idb.record_current_orientation if orientation_only
                  else self.PAL12idb.record_current_position)
        try:
            pose = record(self.rob, station)
        except Exception as e:
            return "ERROR: could not record %s: %s" % (station, e)
        detail = "taught %s%s at %s" % (station,
                                        " orientation" if orientation_only else "",
                                        pose)
        print(detail)
        return "OK %s" % detail

    def _search_apriltag(self, station, skip_roll=False):
        if self.simulate:
            # Sleep in small slices so a stop_search sent during a --simulate
            # run is also honored, instead of only being useful against real
            # hardware.
            event = self._search_stop_event
            for _ in range(20):
                if event is not None and event.is_set():
                    raise RuntimeError("AprilTag search for %s was stopped by the operator" % station)
                time.sleep(0.1)
            return "simulated AprilTag search for %s%s" % (
                station, " (skip roll)" if skip_roll else "")
        pos = self.PAL12idb.locate_apriltag(
            self.rob, pos=station, stop_event=self._search_stop_event,
            skip_roll=skip_roll)
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

    def _goto_position(self, station):
        if self.simulate:
            time.sleep(1)
            return "simulated goto %s" % station
        pose = self.PAL12idb.goto_station(self.rob, station)
        return "at %s (%s)" % (station, pose)

    def _tweak_orientation(self, axis, degrees):
        if self.simulate:
            time.sleep(0.3)
            return "simulated %s rotation of %s deg" % (axis, degrees)
        pose = self.PAL12idb.tweak_orientation(self.rob, axis, degrees)
        return "rotated %s by %s deg, now at %s" % (axis, degrees, pose)

    def _zalign(self):
        """Level the tool Z axis (robUR.Zalign). Runs through the same busy /
        ACCEPTED path as a transport: it is a real move, slow by design
        (acc=vel=0.05), so it must not hold up the reply thread."""
        if self.simulate:
            time.sleep(0.5)
            return "simulated zalign"
        pose = self.rob.Zalign()
        return "z-aligned at %s" % (pose,)

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
