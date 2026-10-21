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
import os
import sys
import threading
import time
from copy import deepcopy

from . import commands as cmd
from . import config
from . import pvapp
from . import state
from .mixing_records import MixingJournal, new_record, utc_now
from .mqtt_status import (
    MQTTPublisher, PHASE_FAILURE, PHASE_STARTED, PHASE_SUCCESS,
    motion_payload, motion_topic, new_trace, state_payload, state_topic,
    tracking_topic,
)
from .pump import Pump
from .motor import Motor, GalilMotor
from .workflows import Workflows, WorkflowError
from .zmq_transport import ZMQCommandServer, ZMQError

STATE_IDLE = "IDLE"
STATE_BUSY = "BUSY"

#: How often the pump dashboards are asked for their status. A little under the
#: GUI's own 3 s get_state poll, so a panel refresh rarely shows the same
#: snapshot twice, without making the dashboards answer more often than anyone
#: reads.
PUMP_STATUS_INTERVAL_S = 2.0


def _robot_model(robot12idb, name):
    """The robot12idb class for `robot.name` in the config -- UR3, UR5, ...

    The model is a real difference in tool geometry, not a label: the UR5
    carries a tool changer, putting its TCP 45 mm further out than the UR3's,
    and its camera TCP with it. Building the wrong class drives the arm with
    the other one's offsets, so every taught descent lands 45 mm out. `name`
    also selects UR_12idb's own ini/<name>.ini, so the two follow each other
    instead of being configured separately.

    Matched case-insensitively, and an unknown name is refused with the list of
    models this UR_12idb actually has, rather than falling back to the UR3 --
    silently driving a UR5 as a UR3 is exactly the failure this exists to stop.
    """
    base = getattr(robot12idb, "UR_cam_grip", None)
    models = {n: cls for n, cls in vars(robot12idb).items()
              if isinstance(cls, type) and base is not None
              and issubclass(cls, base) and cls is not base}
    for model_name, cls in models.items():
        if model_name.upper() == name.upper():
            return cls
    raise ValueError(
        "robot.name is %r, which is not a robot model in this UR_12idb "
        "checkout. Available: %s. Set robot.name in "
        "json/palmixer_config.json to the arm actually connected -- note that "
        "the models differ in tool geometry, so taught positions do not carry "
        "between them." % (name, ", ".join(sorted(models)) or "none found"))


def _build_motor(motor_cfg):
    """The motor driver for the `motor` config section.

    type "galil" drives a Galil controller channel (ip + channel) -- 12idb:m6
    was replaced by channel A of the Galil; type "epics" drives an EPICS motor
    record (pv). Defaults to galil, the hardware actually installed.
    """
    kind = str(motor_cfg.get("type", "galil")).strip().lower()
    if kind == "epics":
        return Motor(motor_cfg.get("pv", "12idb:m6"))
    if kind == "galil":
        ip = motor_cfg.get("ip")
        if not ip:
            raise ValueError("motor.type is 'galil' but motor.ip is not set in "
                             "json/palmixer_config.json")
        return GalilMotor(ip, motor_cfg.get("channel", "A"))
    raise ValueError("motor.type must be 'galil' or 'epics', got %r" % (kind,))


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
        # Which step of a multi-step workflow is running right now, so the GUI
        # can name it in the Server state box. Rides on the get_state snapshot
        # for the same reason _last_result does: the motion MQTT topic is the
        # only other place a step is reported, and it is silent whenever paho
        # is missing or the broker is unreachable -- which left a ten-minute
        # make_sample showing nothing but "BUSY" for its whole run.
        self._current_step = None
        # Latest pump status, refreshed by a background thread and served from
        # here. Polled rather than fetched on demand because get_state runs on
        # the ZMQ reply thread: two round trips to the pump dashboards there
        # would stall every other command behind them, and a dashboard that is
        # down would stall them for its whole timeout.
        self._pump_status = None
        self._stopping = threading.Event()
        # How the last action ended -- {action, ok, detail, time}, success as
        # well as failure. Rides on the get_state snapshot so a client learns
        # about it from its ordinary poll: the motion MQTT topic is the only
        # other place an async outcome is reported, and it is silent whenever
        # the broker is unreachable or paho is not installed -- which left the
        # GUI showing nothing at all for a search that had already given up.
        self._last_result = None
        self._mixing_journal = MixingJournal()
        self._active_mix_record = None
        self._mix_record_thread = None
        self._last_mixing_record = None
        self._mix_sync_wake = threading.Event()
        self._search_stop_event = None  # set while a search_apriltag action is running

        self._mqtt = MQTTPublisher(
            host=cfg["mqtt"].get("host"), port=cfg["mqtt"].get("port"),
            client_id_prefix="palmixer-server",
        )

        self.rob = None
        self.PAL12idb = None
        self.pump = Pump(simulate=self.simulate)
        self.pump.on_mix_record = self._pump_mix_recorded
        self.motor = _build_motor(cfg["motor"])
        #: Appends locally journaled mixing outcomes to PVapp. Credentials come
        #: from the environment at startup and may be replaced for the process's
        #: lifetime by a campaign's set_credentials -- see PVappUpdater.
        self.pvapp = pvapp.PVappUpdater()

        if not self.simulate:
            ur12idb_path = cfg["robot"].get("ur12idb_path")
            if ur12idb_path and ur12idb_path not in sys.path:
                sys.path.insert(0, ur12idb_path)
            try:
                import robot12idb  # noqa: E402  (path must be set first)
            except ImportError as e:
                # The bare ModuleNotFoundError names only `robot12idb`, which
                # says nothing about *which* path was searched -- and the usual
                # cause is a ur12idb_path belonging to a different machine. A
                # failure naming some *other* module is UR_12idb's own import
                # of a dependency: the checkout was found, the environment is
                # short a package, and pointing the path elsewhere would not
                # help.
                if getattr(e, "name", None) in (None, "robot12idb"):
                    hint = ("Point PALMIXER_UR12IDB_PATH (or robot.ur12idb_path "
                            "in json/palmixer_config.json) at your UR_12idb "
                            "checkout")
                else:
                    hint = ("UR_12idb was found there but needs %s; install it "
                            "(the aps12robot environment has it)" % e.name)
                raise ImportError("cannot import robot12idb from %r (%s). %s, "
                                  "or start the server with --simulate."
                                  % (ur12idb_path, e, hint)) from e
            from . import PAL12idb as pal

            # `robot.name` picks the class, not just the ini file. The models
            # differ in tool geometry -- the UR5 carries a tool changer, so its
            # TCP sits 45 mm further out than the UR3's -- and driving one
            # through the other's class puts every descent and every camera
            # standoff wrong by that much.
            robot_name = str(cfg["robot"].get("name") or "UR3").strip()
            robot_class = _robot_model(robot12idb, robot_name)
            # Pass `ip` explicitly so the connection uses *our* config
            # (json/palmixer_config.json, overridable with PALMIXER_ROBOT_IP)
            # instead of UR_12idb's own list_of_robots.json -- that file is
            # a default for other UR_12idb consumers, not a second place
            # this package's robot address has to be kept in sync.
            self.rob = robot_class(name=robot_name, ip=cfg["robot"].get("ip"))
            # Printed because taught positions are only valid for the geometry
            # they were taught on: seeing the wrong TCP here is the cheapest
            # moment to catch a config pointed at the other arm.
            print("PALmixerServer: %s at %s, tool TCP %s"
                  % (robot_name, cfg["robot"].get("ip"),
                     getattr(robot_class, "tcp", "unknown")))
            # Apply calibrated camera intrinsics (if any) so every distance
            # measurement uses the real focal length, not the assumed one. A
            # no-op when the camera has never been calibrated.
            try:
                pal.apply_camera_intrinsics(self.rob)
            except Exception as e:
                print("PALmixerServer: could not apply camera intrinsics: %s" % e)
            # Apply the calibrated AprilTag-distance correction (scale + offset)
            # to the camera, so QRdistance reads true distance everywhere (the
            # raw pinhole value has a small scale and offset error). No-op when
            # unset. See camera.distance_scale / distance_offset_m in the config.
            try:
                cam_cfg = cfg.get("camera", {}) or {}
                scale = float(cam_cfg.get("distance_scale", 1.0))
                offset = float(cam_cfg.get("distance_offset_m", 0.0))
                self.rob.camera.dist_scale = scale
                self.rob.camera.dist_offset = offset
                print("PALmixerServer: AT distance correction scale=%.4f offset=%+.4f m"
                      % (scale, offset))
            except Exception as e:
                print("PALmixerServer: could not set AT distance correction: %s" % e)
            # Software joint guard: refuse joint moves that drive the shoulder
            # (joint 1) below this angle. Lives on robUR.movej; set it from
            # config here (robot.shoulder_min_deg; default -150). None disables.
            try:
                rob_cfg = cfg.get("robot", {}) or {}
                if "shoulder_min_deg" in rob_cfg:
                    lim = rob_cfg.get("shoulder_min_deg")
                    self.rob.shoulder_min_deg = (None if lim is None else float(lim))
                lim_now = getattr(self.rob, "shoulder_min_deg", None)
                print("PALmixerServer: shoulder joint guard = %s"
                      % ("off" if lim_now is None else ("%.1f deg" % lim_now)))
            except Exception as e:
                print("PALmixerServer: could not set shoulder guard: %s" % e)
            # Activate the gripper now so the first grab does not pay the slow
            # activation handshake mid-move. Non-fatal: a gripper that will not
            # activate at startup must not stop the server from coming up.
            try:
                pal.activate_gripper(self.rob)
                print("PALmixerServer: gripper activated")
            except Exception as e:
                print("PALmixerServer: could not activate gripper: %s" % e)
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
                                   action_result_version=1,
                                   simulated=self.simulate,
                                   busy=bool(self._busy),
                                   current_action_id=self._current_trace,
                                   mixing_speed=self.pump.speed,
                                   mixing_speed_limits=[self.pump.min_rpm,
                                                        self.pump.max_rpm],
                                   current_action=self._current_action,
                                   current_step=self._current_step,
                                   cleanup=self.workflows.cleanup_snapshot(),
                                   pump_status=self._pump_status,
                                   last_mixing_record=self._last_mixing_record,
                                   last_result=self._last_result))

        if name == cmd.SET_CREDENTIALS:
            return self._set_credentials(args)

        if name == cmd.STOP_SEARCH:
            return self._stop_search()

        if name == cmd.FIT_VENTON_TABLE:
            # Fast: no motion, just fits the points measure_venton_tag has
            # collected. Answered inline so the operator gets the frame back
            # without it riding the worker path.
            if args:
                return "ERROR: %s takes no arguments" % cmd.FIT_VENTON_TABLE
            if self.simulate:
                return "simulated venton fit"
            try:
                fit = self.PAL12idb.venton_fit_from_measurements(self._venton_points())
            except Exception as e:
                return "ERROR: %s" % e
            return self._venton_reply(fit)

        if name == cmd.CLEAR_VENTON_MEASUREMENTS:
            if args:
                return "ERROR: %s takes no arguments" % cmd.CLEAR_VENTON_MEASUREMENTS
            self._venton_points().clear()
            return "OK (Venton measurements cleared)"

        if name == cmd.VENTON_AXES:
            return self._venton_axes(args)

        if name == cmd.GET_POSE:
            frame = "gripper"
            if args:
                if len(args) != 1 or args[0].lower() not in ("gripper", "camera"):
                    return "ERROR: usage: %s [gripper|camera]" % cmd.GET_POSE
                frame = args[0].lower()
            return self._get_pose(frame)

        if name == cmd.GET_TCP:
            if args:
                return "ERROR: %s takes no arguments" % cmd.GET_TCP
            return self._get_tcp()

        if name == cmd.GET_JOINTS:
            if args:
                return "ERROR: %s takes no arguments" % cmd.GET_JOINTS
            return self._get_joints()

        if name == cmd.TAG_DISTANCE:
            return self._tag_distance(args)

        if name == cmd.UNLOCK_STOP:
            if args:
                return "ERROR: %s takes no arguments" % cmd.UNLOCK_STOP
            return self._unlock_stop()

        if name == cmd.FREEDRIVE:
            return self._freedrive(args)

        if name == cmd.STOP_PUMP:
            if args:
                return "ERROR: %s takes no arguments" % cmd.STOP_PUMP
            # Fast, and deliberately not guarded by the busy flag: the whole
            # point is to reach a pump operation that is running, which is
            # exactly when dispatch() would refuse a worker command. Pump sends
            # it on a socket that operation is not holding. Harmless when
            # nothing is shaking -- the flowcell server just reports it was
            # not accepted rather than touching an unrelated action.
            ok, detail = self.pump.stop_shaking()
            return ("OK %s" % detail) if ok else ("ERROR: %s" % detail)

        if name in (cmd.SET_POSITION_HERE, cmd.SET_ORIENTATION_HERE):
            return self._set_position_here(args, orientation_only=(
                name == cmd.SET_ORIENTATION_HERE))

        if name == cmd.SET_TRANSFER_POINT_HERE:
            return self._set_transfer_point_here(args)

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

        if name == cmd.SET_POSITION_SAMPLE_TABLE_MOTORS:
            if len(args) != 1 or args[0] not in (cmd.TOGGLE_ON, cmd.TOGGLE_OFF):
                return "ERROR: usage: %s %s|%s" % (
                    cmd.SET_POSITION_SAMPLE_TABLE_MOTORS, cmd.TOGGLE_ON, cmd.TOGGLE_OFF)
            on = args[0] == cmd.TOGGLE_ON
            # Through PAL12idb so its module global and the persisted state move
            # together (set_position_sample_table_motors writes both); fall back
            # to state alone if the robot module is not loaded (--simulate GUI).
            if self.PAL12idb is not None:
                self.PAL12idb.set_position_sample_table_motors(on)
            else:
                state.set_position_sample_table_motors(on)
            return "OK sample-table motor positioning %s" % args[0]

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

        if name == cmd.SET_SAMPLE_UID:
            if len(args) < 3:
                return "ERROR: usage: set_sample_uid <slot> <sample_uid> <sample id>"
            if self._busy:
                return "ERROR: busy; sample identity cannot change during an action"
            try:
                state.set_sample_uid(args[0], " ".join(args[2:]), args[1])
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

        if name == cmd.CLEAR_SLOTS:
            if args:
                return "ERROR: clear_slots takes no arguments"
            with self._busy_lock:
                if self._busy:
                    return "ERROR: busy; slot labels cannot be cleared during an action"
                state.clear_slots()
            return "OK"

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
        correlated to the parent make_sample/unload_sample action by trace id.

        Also records the step as the current one, for clients that poll
        get_state rather than listen to MQTT. Only "started" sets it: a step
        that has finished is no longer what the machine is doing, and the next
        one announces itself a moment later. Background steps (the not-awaited
        wash and mixer clean) report their success out of order by design, so
        their late "success" must not clear a step that is genuinely running --
        hence only clearing when the name still matches.
        """
        self._record_mixing_step(step, phase, detail)
        if phase == PHASE_STARTED:
            self._current_step = step
        elif self._current_step == step:
            self._current_step = None
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

        threading.Thread(target=self._run_action,
                          args=(action_fn, action_label, trace, " ".join(parts)),
                          daemon=True).start()
        return "ACCEPTED action_id=%s" % trace

    def _resolve(self, name, args):
        """Map a command name + args to a zero-arg callable and a human label.

        Raises ValueError for anything unrecognized or malformed.
        """
        if name == cmd.SEARCH_APRILTAG:
            # Both flags are optional and order-independent, so they are pulled
            # out by name rather than by position. An unrecognised extra word
            # is left in `args` and falls through to the usage error below --
            # a misspelled flag must not be read as "no flag" and silently
            # drive the arm across the cell.
            flags = set(args[1:])
            skip_roll = cmd.SKIP_ROLL in flags
            from_current = cmd.FROM_CURRENT in flags
            flags -= {cmd.SKIP_ROLL, cmd.FROM_CURRENT}
            if len(args) < 1 or args[0] not in cmd.STATIONS or flags:
                raise ValueError("usage: %s <%s> [%s] [%s]" % (
                    cmd.SEARCH_APRILTAG, "|".join(cmd.STATIONS),
                    cmd.SKIP_ROLL, cmd.FROM_CURRENT))
            station = args[0]
            label = "%s %s%s%s" % (
                cmd.SEARCH_APRILTAG, station,
                " (skip roll)" if skip_roll else "",
                " (from current position)" if from_current else "")
            # Created here (on the synchronous reply path, before ACCEPTED is
            # sent) so a stop_search sent right after is guaranteed to find it.
            self._search_stop_event = threading.Event()
            return (lambda: self._search_apriltag(
                station, skip_roll, from_current)), label

        if name in (cmd.PUT_CAMERA_TO_TCP, cmd.PUT_TCP_TO_CAMERA):
            if args:
                raise ValueError("%s takes no arguments" % name)
            return (lambda: self._swap_tool_frame(name)), name

        if name in (cmd.PUSH_POSITIONS, cmd.PULL_POSITIONS):
            if args:
                raise ValueError("%s takes no arguments" % name)
            return (lambda: self._sync_positions(name)), name

        if name in cmd.TRANSPORT_FUNCTIONS:
            if args:
                raise ValueError("%s takes no arguments" % name)
            # A mixer-head transport is deliberately *not* refused here while
            # the head is being washed. It is accepted and the worker waits
            # out the wash (_run_transport -> Workflows.await_mixer_wash), so
            # the head goes where it was told to go as soon as it can rather
            # than the operator having to notice the refusal and press again.
            self._require_positions(self.PAL12idb.TRANSPORT_STATIONS[name] if self.PAL12idb else ())
            return (lambda: self._run_transport(name)), name

        if name == cmd.GOTO_POSITION:
            if len(args) != 1 or args[0] not in cmd.TAUGHT_STATIONS:
                raise ValueError("usage: %s <%s>" % (
                    cmd.GOTO_POSITION, "|".join(cmd.TAUGHT_STATIONS)))
            station = args[0]
            # Refuse an untaught station here rather than letting the move fail
            # once it has already been ACCEPTED -- same check, and the same
            # "search its AprilTag first" message, the transports get.
            self._require_positions((station,))
            label = "%s %s" % (cmd.GOTO_POSITION, station)
            return (lambda: self._goto_position(station)), label

        if name == cmd.GOTO_TRANSFER_POINT:
            if args:
                raise ValueError("%s takes no arguments" % cmd.GOTO_TRANSFER_POINT)
            # No _require_positions: the transfer point is a fixed pose in
            # PAL12idb, not a taught one, so there is nothing that could be
            # unconfigured about it.
            return self._goto_transfer_point, cmd.GOTO_TRANSFER_POINT

        if name == cmd.UNLOAD_CAROUSEL:
            if args:
                raise ValueError("%s takes no arguments" % cmd.UNLOAD_CAROUSEL)
            self._require_positions(('carousel_on_mixer', 'carousel_used_stack'))
            return self._unload_carousel, cmd.UNLOAD_CAROUSEL

        if name == cmd.PLACE_NEW_CAROUSEL:
            if args:
                raise ValueError("%s takes no arguments" % cmd.PLACE_NEW_CAROUSEL)
            self._require_positions(('carousel_in_stock', 'carousel_on_mixer'))
            return self._place_new_carousel, cmd.PLACE_NEW_CAROUSEL

        if name == cmd.CALIBRATE_VENTON_TABLE:
            if args:
                raise ValueError("%s takes no arguments" % cmd.CALIBRATE_VENTON_TABLE)
            return self._calibrate_venton_table, cmd.CALIBRATE_VENTON_TABLE

        if name == cmd.CALIBRATE_VENTON_LEVEL:
            heading, id_args = "orientation", list(args)
            modes = [a for a in id_args if a in cmd.VENTON_HEADING_MODES]
            if len(modes) > 1:
                raise ValueError("give at most one of %s"
                                 % "|".join(cmd.VENTON_HEADING_MODES))
            if modes:
                heading = modes[0]
                id_args = [a for a in id_args if a != heading]
            try:
                tag_ids = [int(a) for a in id_args] or None
            except ValueError:
                raise ValueError("usage: %s [%s] [tag_id ...]  (ids are integers)"
                                 % (cmd.CALIBRATE_VENTON_LEVEL,
                                    "|".join(cmd.VENTON_HEADING_MODES)))
            label = "%s %s%s" % (
                cmd.CALIBRATE_VENTON_LEVEL, heading,
                " " + " ".join(str(t) for t in tag_ids) if tag_ids else "")
            return (lambda: self._calibrate_venton_level(tag_ids, heading)), label

        if name == cmd.GOTO_XYZ:
            if len(args) != 3:
                raise ValueError("usage: %s <x> <y> <z>  (metres)" % cmd.GOTO_XYZ)
            try:
                xyz = [float(v) for v in args]
            except ValueError:
                raise ValueError("%s needs three numeric coordinates" % cmd.GOTO_XYZ)
            return (lambda: self._goto_xyz(xyz)),\
                   "%s %s" % (cmd.GOTO_XYZ, xyz)

        if name == cmd.CALIBRATE_CAMTCP:
            # No arg -> ChArUco board (default). 'apriltag [id]' -> single tag.
            target, tag_id = 'charuco', 13
            if args and args[0].lower() == 'apriltag':
                target = 'apriltag'
                if len(args) > 1:
                    try:
                        tag_id = int(args[1])
                    except ValueError:
                        raise ValueError("tag id must be an integer, got %r" % args[1])
            elif args:
                raise ValueError("usage: %s [apriltag [tag_id]]" % cmd.CALIBRATE_CAMTCP)
            return (lambda: self._calibrate_camtcp(target, tag_id)),\
                   "%s %s" % (cmd.CALIBRATE_CAMTCP, target)

        if name == cmd.CALIBRATE_CAMERA_INTRINSICS:
            if args:
                raise ValueError("%s takes no arguments" % cmd.CALIBRATE_CAMERA_INTRINSICS)
            return self._calibrate_camera_intrinsics, cmd.CALIBRATE_CAMERA_INTRINSICS

        if name == cmd.ESTIMATE_VENTON_TABLE:
            if args:
                raise ValueError("%s takes no arguments" % cmd.ESTIMATE_VENTON_TABLE)
            return self._estimate_venton_table, cmd.ESTIMATE_VENTON_TABLE

        if name == cmd.ALIGN_CAMERA_NORTH:
            # Optional direction: a name (+x/-x/+y/-y) or a bare azimuth in
            # degrees; defaults to +x. Resolved to an azimuth here so the robot
            # layer only ever sees a number.
            direction = args[0].lower() if args else "+x"
            if len(args) > 1:
                raise ValueError("usage: %s [%s|<degrees>]" % (
                    cmd.ALIGN_CAMERA_NORTH, "|".join(cmd.CAMERA_DIRECTIONS)))
            if self.PAL12idb is not None and direction in self.PAL12idb.CAMERA_AZIMUTH:
                azimuth = self.PAL12idb.CAMERA_AZIMUTH[direction]
            else:
                try:
                    azimuth = float(direction)
                except ValueError:
                    raise ValueError("usage: %s [%s|<degrees>]" % (
                        cmd.ALIGN_CAMERA_NORTH, "|".join(cmd.CAMERA_DIRECTIONS)))
            label = "%s %s" % (cmd.ALIGN_CAMERA_NORTH, direction)
            return (lambda: self._align_camera_north(azimuth)), label

        if name == cmd.MEASURE_VENTON_TAG:
            if len(args) != 1:
                raise ValueError("usage: %s <tag_id>" % cmd.MEASURE_VENTON_TAG)
            try:
                tag_id = int(args[0])
            except ValueError:
                raise ValueError("tag id must be an integer, got %r" % args[0])
            return (lambda: self._measure_venton_tag(tag_id)),\
                   "%s %d" % (cmd.MEASURE_VENTON_TAG, tag_id)

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

        if name == cmd.TWEAK_POSITION:
            if len(args) != 2 or args[0].lower() not in cmd.POSITION_AXES:
                raise ValueError("usage: %s <%s> <millimetres>" % (
                    cmd.TWEAK_POSITION, "|".join(cmd.POSITION_AXES)))
            axis = args[0].lower()
            try:
                millimetres = float(args[1])
            except ValueError:
                raise ValueError("millimetres must be numeric, got %r" % args[1])
            # Bounded again here, on the synchronous accept path, so an
            # over-large jog is refused before ACCEPTED rather than failing
            # once the operator thinks the arm is already moving. PAL12idb
            # enforces the same limit as the backstop for a direct call.
            if self.PAL12idb is not None:
                limit_mm = self.PAL12idb.MAX_POSITION_TWEAK * 1000.0
                if not abs(millimetres) <= limit_mm:
                    raise ValueError(
                        "jog of %g mm exceeds the %g mm single-step limit"
                        % (millimetres, limit_mm))
            label = "%s %s %s mm" % (cmd.TWEAK_POSITION, axis, millimetres)
            return (lambda: self._tweak_position(axis, millimetres)), label

        if name == cmd.ZALIGN:
            if args:
                raise ValueError("%s takes no arguments" % cmd.ZALIGN)
            return self._zalign, cmd.ZALIGN

        if name == cmd.FACE_CAMERA_DOWN:
            if args:
                raise ValueError("%s takes no arguments" % cmd.FACE_CAMERA_DOWN)
            return self._face_camera_down, cmd.FACE_CAMERA_DOWN

        if name == cmd.RELEASE_GRIPPER:
            if args:
                raise ValueError("%s takes no arguments" % cmd.RELEASE_GRIPPER)
            return self._release_gripper, cmd.RELEASE_GRIPPER

        if name == cmd.GRAB_GRIPPER:
            if args:
                raise ValueError("%s takes no arguments" % cmd.GRAB_GRIPPER)
            return self._grab_gripper, cmd.GRAB_GRIPPER

        if name == cmd.BUMP_Z:
            if len(args) > 1:
                raise ValueError("usage: %s [backoff_mm]" % cmd.BUMP_Z)
            try:
                backoff_mm = float(args[0]) if args else 0.0
            except ValueError:
                raise ValueError("backoff_mm must be numeric, got %r" % args[0])
            return (lambda: self._bump_z(backoff_mm)), "%s %g mm" % (cmd.BUMP_Z, backoff_mm)

        if name == cmd.SCAN_TAGS:
            if len(args) not in (0, 3):
                raise ValueError("usage: %s [<start_deg> <end_deg> <speed_deg_s>]"
                                 % cmd.SCAN_TAGS)
            try:
                sweep = ([float(a) for a in args] if args else [-45.0, 45.0, 1.0])
            except ValueError:
                raise ValueError("scan_tags args must be numeric, got %r" % (args,))
            label = "%s %g %g %g deg/s" % (cmd.SCAN_TAGS, sweep[0], sweep[1], sweep[2])
            return (lambda: self._scan_tags(*sweep)), label

        if name == cmd.MEASURE_TAG_HEIGHTS:
            try:
                only_ids = [int(a) for a in args] if args else None
            except ValueError:
                raise ValueError("%s ids must be integers, got %r"
                                 % (cmd.MEASURE_TAG_HEIGHTS, args))
            label = (cmd.MEASURE_TAG_HEIGHTS if not only_ids
                     else "%s %s" % (cmd.MEASURE_TAG_HEIGHTS, only_ids))
            return (lambda: self._measure_tag_heights(only_ids)), label

        if name == cmd.RASTER_TAG_SCAN:
            if len(args) not in (0, 2):
                raise ValueError("usage: %s [base_from base_to]"
                                 % cmd.RASTER_TAG_SCAN)
            try:
                g = ([float(a) for a in args] if args else [-80.0, 0.0])
            except ValueError:
                raise ValueError("raster_tag_scan args must be numeric, got %r" % (args,))
            label = "%s base %g..%g" % (cmd.RASTER_TAG_SCAN, g[0], g[1])
            return (lambda: self._raster_tag_scan(*g)), label

        if name == cmd.LOCATE_MIXER_CAROUSEL:
            if len(args) not in (0, 2):
                raise ValueError("usage: %s [base_from base_to]"
                                 % cmd.LOCATE_MIXER_CAROUSEL)
            try:
                g = ([float(a) for a in args] if args else [-100.0, 20.0])
            except ValueError:
                raise ValueError("locate_mixer_carousel args must be numeric, got %r"
                                 % (args,))
            label = "%s base %g..%g" % (cmd.LOCATE_MIXER_CAROUSEL, g[0], g[1])
            return (lambda: self._locate_mixer_carousel(*g)), label

        if name == cmd.READ_TAG:
            if len(args) != 1:
                raise ValueError("usage: %s <tag_id>" % cmd.READ_TAG)
            try:
                tag_id = int(args[0])
            except ValueError:
                raise ValueError("read_tag tag id must be an integer, got %r" % (args,))
            label = "%s %d" % (cmd.READ_TAG, tag_id)
            return (lambda: self._read_tag(tag_id)), label

        if name == cmd.READ_TAGS:
            if args:
                raise ValueError("usage: %s (no args)" % cmd.READ_TAGS)
            return (lambda: self._read_tags()), cmd.READ_TAGS

        if name == cmd.SURVEY_TAGS:
            if args:
                raise ValueError("usage: %s (no args)" % cmd.SURVEY_TAGS)
            return (lambda: self._survey_tags()), cmd.SURVEY_TAGS

        if name == cmd.MOVE_TO_XYZ:
            if len(args) not in (3, 4):
                raise ValueError("usage: %s <x> <y> <z> [gripper|camera]"
                                 % cmd.MOVE_TO_XYZ)
            try:
                x, y, zc = float(args[0]), float(args[1]), float(args[2])
            except ValueError:
                raise ValueError("move_to_xyz x/y/z must be numeric, got %r" % (args,))
            frame = args[3].lower() if len(args) == 4 else "gripper"
            if frame not in ("gripper", "camera"):
                raise ValueError("frame must be gripper or camera, got %r" % frame)
            label = "%s %.4f %.4f %.4f %s" % (cmd.MOVE_TO_XYZ, x, y, zc, frame)
            return (lambda: self._move_to_xyz(x, y, zc, frame)), label

        if name == cmd.SET_BASE_JOINT:
            if len(args) not in (1, 2):
                raise ValueError("usage: %s <deg> [speed_deg_s]" % cmd.SET_BASE_JOINT)
            try:
                deg = float(args[0])
                speed = float(args[1]) if len(args) == 2 else None
            except ValueError:
                raise ValueError("set_base_joint args must be numeric, got %r" % (args,))
            if speed is not None and speed <= 0:
                raise ValueError("speed_deg_s must be positive, got %g" % speed)
            label = "%s %g deg" % (cmd.SET_BASE_JOINT, deg)
            return (lambda: self._set_base_joint(deg, speed)), label

        if name == cmd.SET_JOINT:
            if len(args) not in (2, 3):
                raise ValueError("usage: %s <index 0-5> <deg> [speed_deg_s]"
                                 % cmd.SET_JOINT)
            try:
                index = int(args[0])
                deg = float(args[1])
                speed = float(args[2]) if len(args) == 3 else None
            except ValueError:
                raise ValueError("set_joint args must be numeric, got %r" % (args,))
            if index not in range(6):
                raise ValueError("joint index must be 0-5, got %d" % index)
            if speed is not None and speed <= 0:
                raise ValueError("speed_deg_s must be positive, got %g" % speed)
            label = "%s %d -> %g deg" % (cmd.SET_JOINT, index, deg)
            return (lambda: self._set_joint(index, deg, speed)), label

        if name == cmd.RECONFIGURE:
            if len(args) not in (6, 7):
                raise ValueError("usage: %s <j0..j5 deg> [speed_deg_s]" % cmd.RECONFIGURE)
            try:
                joints = [float(v) for v in args[:6]]
                speed = float(args[6]) if len(args) == 7 else None
            except ValueError:
                raise ValueError("reconfigure args must be numeric, got %r" % (args,))
            if speed is not None and speed <= 0:
                raise ValueError("speed_deg_s must be positive, got %g" % speed)
            label = "%s %s deg" % (cmd.RECONFIGURE, joints)
            return (lambda: self._reconfigure(joints, speed)), label

        if name == cmd.SWING_BASE:
            if len(args) != 3:
                raise ValueError("usage: %s <start_deg> <end_deg> <speed_deg_s>"
                                 % cmd.SWING_BASE)
            try:
                start_deg, end_deg, speed = (float(args[0]), float(args[1]),
                                             float(args[2]))
            except ValueError:
                raise ValueError("swing_base args must be numeric, got %r" % (args,))
            if speed <= 0:
                raise ValueError("speed_deg_s must be positive, got %g" % speed)
            label = "%s %g %g %g deg/s" % (cmd.SWING_BASE, start_deg, end_deg, speed)
            return (lambda: self._swing_base(start_deg, end_deg, speed)), label

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
            # With no explicit ID, retain a prepared vial's slot tag. Only an
            # untagged vial needs a new ID, minted by the workflow after mixing.
            sample_id = " ".join(args[1:]) if len(args) > 1 else None
            self._require_positions(self.workflows.stations_for_make_sample())
            try:
                self.workflows._check_make_sample(slot, sample_id)
            except WorkflowError as e:
                raise ValueError(str(e))
            label = "%s %s%s" % (cmd.MAKE_SAMPLE, slot,
                                 " (%s)" % sample_id if sample_id else "")

            def run_make_sample():
                return self._make_sample_recorded(slot, sample_id)
            return run_make_sample, label

        if name == cmd.DRAW_LOAD_SAMPLE:
            if not args:
                raise ValueError("usage: %s <slot> [sample id]" % cmd.DRAW_LOAD_SAMPLE)
            try:
                slot = int(args[0])
            except ValueError:
                raise ValueError("slot must be an integer, got %r" % args[0])
            sample_id = " ".join(args[1:]) if len(args) > 1 else None
            self._require_positions(self.workflows._stations(
                "ready_flowcell_to_draw", "load_sample_to_beam"))
            try:
                self.workflows._check_draw_load_sample(slot, sample_id)
            except WorkflowError as e:
                raise ValueError(str(e))
            label = "%s %s (%s)" % (cmd.DRAW_LOAD_SAMPLE, slot,
                                     sample_id or "preloaded")
            return (lambda: self.workflows.draw_load_sample(slot, sample_id)), label

        if name == cmd.DRAW_AND_LOAD:
            if args:
                raise ValueError("%s takes no arguments" % cmd.DRAW_AND_LOAD)
            self._require_positions(self.workflows.stations_for_draw_and_load())
            try:
                self.workflows._check_draw_and_load()
            except WorkflowError as e:
                raise ValueError(str(e))
            return (lambda: self.workflows.draw_and_load()), cmd.DRAW_AND_LOAD

        if name == cmd.UNLOAD_SAMPLE:
            aspirate = args == [cmd.ASPIRATE]
            if args and not aspirate:
                raise ValueError("usage: %s [%s]" % (cmd.UNLOAD_SAMPLE, cmd.ASPIRATE))
            # Both the required positions and the guard depend on the flag:
            # without it nothing goes near the mixer, so neither its stations
            # nor its tracked location can refuse the run.
            self._require_positions(
                self.workflows.stations_for_unload_sample(aspirate))
            try:
                self.workflows._check_unload_sample(aspirate)
            except WorkflowError as e:
                raise ValueError(str(e))
            label = "%s%s" % (cmd.UNLOAD_SAMPLE, " (aspirate)" if aspirate else "")
            return (lambda: self.workflows.unload_sample(aspirate)), label

        if name == cmd.UNLOAD_SAMPLE_ASYNC:
            if args:
                raise ValueError("%s takes no arguments" % cmd.UNLOAD_SAMPLE_ASYNC)
            self._require_positions(self.workflows.stations_for_unload_sample())
            try:
                self.workflows._check_unload_sample()
            except WorkflowError as e:
                raise ValueError(str(e))
            return (lambda: self.workflows.unload_sample(wait_clean=False)), \
                   cmd.UNLOAD_SAMPLE_ASYNC

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

    def _run_action(self, action_fn, action_label, trace, command=None):
        """Runs on a background thread: execute the action, publish the
        outcome, and clear the busy flag."""
        try:
            # Direct transports and manual moves also invalidate an earlier park.
            # Only a completed workflow parking step can establish it again.
            self.workflows._cleanup_begin("worker_action")
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
            self._last_result.update(action_id=trace, command=command or action_label,
                                     finished_at=utc_now(), simulated=self.simulate,
                                     state=state.snapshot())
            with self._busy_lock:
                self._busy = False
                self._current_action = None
                self._current_trace = None
                self._current_step = None
            self._publish_state(STATE_IDLE)

    # -- individual actions --------------------------------------------------
    def _set_credentials(self, args):
        """A campaign handing over the PVapp login it already has, so it does not
        have to be exported separately on this host too. Takes effect on the *next*
        PVapp confirmation, in memory only -- never logged, never returned, never on
        the get_state snapshot. The only trace this leaves is a console line naming
        the username, never the password.
        """
        if len(args) != 1:
            return "ERROR: usage: %s <base64 json>" % cmd.SET_CREDENTIALS
        try:
            payload = cmd.decode_credentials(args[0])
        except ValueError as e:
            return "ERROR: %s" % e
        username = str(payload.get("username") or "").strip()
        badge = str(payload.get("badge") or "").strip()
        gup = str(payload.get("gup") or "").strip()
        # Either route is a credential on its own, so "empty" now means neither was sent.
        # The badge pair needs no password; a badge without its proposal is not a
        # credential and is refused here rather than a round trip later.
        if badge and not gup:
            return "ERROR: badge needs the proposal (gup) it belongs to"
        if gup and not badge:
            return "ERROR: gup needs the badge it belongs to"
        if not username and not badge:
            return "ERROR: send username, or badge and gup"
        if str(payload.get("source") or "campaign").strip().lower() == "user":
            # The GUI's User tab: a fallback, never an override, so a person setting it
            # cannot take a running campaign's mixes out from under its proposal.
            self.pvapp.user_badge, self.pvapp.user_gup = badge, gup
            print("PALmixerServer: User tab set badge %s on proposal %s%s"
                  % (badge, gup, " (held in reserve; a campaign's own are in use)"
                     if self.pvapp.badge else ""))
            return "OK"

        # Whichever route was sent REPLACES the other: the updater prefers the pair, so
        # a username accepted here while a configured pair remained would be ignored.
        if username:
            self.pvapp.username = username
            self.pvapp.password = str(payload.get("password") or "")
            self.pvapp.badge = self.pvapp.gup = ""
            self.pvapp.password_only = True
        else:
            self.pvapp.badge, self.pvapp.gup = badge, gup
            self.pvapp.username = self.pvapp.password = ""
            self.pvapp.password_only = False
        print("PALmixerServer: PVapp credentials received for %s"
              % ("badge %s on proposal %s" % (badge, gup) if badge else username))
        return "OK"

    def _save_mixing_record(self, record):
        try:
            self._mixing_journal.save(record)
        except (OSError, ValueError, TypeError) as exc:
            print("WARNING: could not save mixing run %s locally: %s" % (record["run_id"], exc))

    def _record_mixing_step(self, step, phase, detail):
        record = self._active_mix_record
        # Background washes can finish during a later sample. Only this
        # workflow's own thread can add its ordered step observations.
        if record is None or self._mix_record_thread != threading.get_ident():
            return
        record["steps"].append({"step": step, "phase": phase, "at": utc_now(), "detail": str(detail)})
        if step == "mix":
            if phase == PHASE_STARTED:
                record["mixing_status"] = "running"
                self.pump.last_mix_record = None
            else:
                pump_record = None if self.simulate else self.pump.last_mix_record
                record["pump"] = deepcopy(pump_record)
                record["mixing_status"] = ((pump_record or {}).get("status") or
                    ("simulated" if self.simulate else "unknown"))
            self._save_mixing_record(record)

    def _pump_mix_recorded(self, pump_record):
        record = self._active_mix_record
        if record is not None and self._mix_record_thread == threading.get_ident():
            record["pump"] = deepcopy(pump_record)
            record["mixing_status"] = ("running" if pump_record["status"] == "accepted"
                                       else pump_record["status"])
            self._save_mixing_record(record)

    def _make_sample_recorded(self, slot, sample_id):
        # A PALsystem-prepared vial already has an identity. Keep it when the
        # GUI leaves its optional sample-ID box blank.
        sample_id = sample_id or state.get_sample_id(slot)
        record = new_record(sample_id, slot, state.get_carousel_id(),
                            state.get_flowcell_in_use(), simulated=self.simulate,
                            sample_uid=(state.carousel_sample_uids().get(int(slot), "")
                                        if state.get_sample_id(slot) == sample_id else ""))
        record["action_id"] = self._current_trace
        self._active_mix_record = record
        self._mix_record_thread = threading.get_ident()
        self._save_mixing_record(record)
        try:
            detail = self.workflows.make_sample(slot, sample_id)
            record.update(workflow_status="completed", detail=detail)
            return detail
        except Exception as exc:
            record.update(workflow_status="failed", error=str(exc))
            raise
        finally:
            record["sample_id"] = sample_id or state.get_sample_id(slot)
            record["finished_at"] = utc_now()
            self._save_mixing_record(record)
            self._last_mixing_record = deepcopy(record)
            self._active_mix_record = self._mix_record_thread = None
            self._mix_sync_wake.set()

    def _sync_mixing_records(self):
        """HTTP retries never hold up robot/pump operations or the GUI."""
        while not self._stopping.is_set():
            self._mix_sync_wake.clear()
            self._mixing_journal.sync_pending(self.pvapp)
            self._mix_sync_wake.wait(30)

    def _stop_search(self):
        """Abort an in-progress search_apriltag: sets the cooperative stop
        event the search loop polls between moves, and -- to halt the move in
        flight sooner than that -- tells the controller to stop now.

        The event is the part that actually ends the search, and setting it is
        instant and cannot block, so it is done first and unconditionally. The
        stopj() only makes the stop *faster*; it goes through
        secmon.send_program, whose condition.wait() is unbounded and only
        returns once the secondary-interface receive loop next drains its
        queue. A stalled 30002 socket therefore freezes that call -- and
        because stop_search is a fast command answered inline on the single
        ZMQ reply thread, freezing it here freezes every command, status
        included (the "server stops responding after Stop Search" seen on a
        controller whose realtime/secondary streams were dropping out). So the
        stopj is fired on a throwaway thread: the reply goes out either way,
        and the event above has already guaranteed the search will stop."""
        if (self._current_action is None
                or not self._current_action.startswith(cmd.SEARCH_APRILTAG + " ")):
            return "ERROR: no AprilTag search is running"
        event = self._search_stop_event
        if event is not None:
            event.set()
        if not self.simulate and self.rob is not None:
            threading.Thread(target=self._fire_stopj, daemon=True).start()
        return "OK"

    def _fire_stopj(self):
        """Send stopj() off the reply thread. Best effort: a failure or a hang
        here must not be able to wedge the command loop, so it is reported to
        the log rather than to the caller, who has already been told OK on the
        strength of the stop event."""
        try:
            self.rob.robot.stopj()
        except Exception as e:
            print("PALmixerServer: stopj after stop_search failed: %s" % e)

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
        # TAUGHT_STATIONS, not STATIONS: teaching by hand is the only way a
        # station with no AprilTag of its own gets a position at all.
        if len(args) != 1 or args[0] not in cmd.TAUGHT_STATIONS:
            return "ERROR: usage: %s <%s>" % (verb, "|".join(cmd.TAUGHT_STATIONS))
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

    def _set_transfer_point_here(self, args):
        """Teach the transfer point from the robot's current pose. Fast, like
        _set_position_here: reads a pose and writes waypoints.ini, no motion --
        and refused while busy for the same reason, a pose sampled mid-move
        records a point the arm was only passing through."""
        if args:
            return "ERROR: %s takes no arguments" % cmd.SET_TRANSFER_POINT_HERE
        if self._busy:
            return ("ERROR: busy (running %r); the robot has to be standing still "
                    "to teach from it" % self._current_action)
        if self.simulate:
            return "OK simulated %s" % cmd.SET_TRANSFER_POINT_HERE
        if self.rob is None:
            return "ERROR: no robot connection"
        try:
            pose = self.PAL12idb.record_current_transfer_point(self.rob)
        except Exception as e:                           # noqa: BLE001
            return "ERROR: could not record the transfer point: %s" % e
        detail = "taught transfer point at %s" % (pose,)
        print(detail)
        return "OK %s" % detail

    def _search_apriltag(self, station, skip_roll=False, from_current=False):
        if self.simulate:
            # Sleep in small slices so a stop_search sent during a --simulate
            # run is also honored, instead of only being useful against real
            # hardware.
            event = self._search_stop_event
            for _ in range(20):
                if event is not None and event.is_set():
                    raise RuntimeError("AprilTag search for %s was stopped by the operator" % station)
                time.sleep(0.1)
            return "simulated AprilTag search for %s%s%s" % (
                station, " (skip roll)" if skip_roll else "",
                " (from current position)" if from_current else "")
        pos = self.PAL12idb.locate_apriltag(
            self.rob, pos=station, stop_event=self._search_stop_event,
            skip_roll=skip_roll, from_current=from_current)
        return "found %s at %s" % (station, pos)

    def _swap_tool_frame(self, name):
        """put_camera_to_tcp / put_tcp_to_camera. A worker action: both are
        real ~4 cm moves, not frame bookkeeping."""
        if self.simulate:
            time.sleep(0.5)
            return "simulated %s" % name
        if name == cmd.PUT_CAMERA_TO_TCP:
            pose = self.PAL12idb.put_camera_to_tcp(self.rob)
            return "camera moved onto the tool pose, now at %s" % (pose,)
        pose = self.PAL12idb.put_tcp_to_camera(self.rob)
        return "tool moved onto the camera pose, now at %s" % (pose,)

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
        if name == "load_flowcell_from_beam_to_cleaningstation":
            # The flowcell is leaving the sample table -- same reason
            # unload_sample stops a shake at its own start (see
            # Workflows.stop_auto_shake), but this is the path reached when
            # the Experiment tab's transport button is pressed directly
            # rather than through unload_sample.
            self.workflows.stop_auto_shake()
        if name in cmd.MIXER_HEAD_TRANSPORTS:
            # The head is docked at its cleaning station with liquid running
            # through it; moving it now would spill. Wait the wash out rather
            # than refuse -- the order is a fine one, it has just arrived
            # early. Reported as its own "wait_for_clean_mixer" step so a wait
            # of minutes does not look like a stalled robot. This is the only
            # path that bypasses Workflows._move_mixer_head (the Experiment
            # tab's two buttons calling PAL12idb directly), so the wait has to
            # be repeated here.
            self.workflows.await_mixer_wash()
        getattr(self.PAL12idb, name)(self.rob)
        return "%s complete" % name

    def _unload_carousel(self):
        if self.simulate:
            time.sleep(2)
            state.set_carousel_on_mixer(None)
            return "simulated unload_carousel (mixer now empty)"
        pose = self.PAL12idb.unload_current_carousel(self.rob)
        return "carousel unloaded to the used stack (arm at %s)" % (pose,)

    def _place_new_carousel(self):
        if self.simulate:
            time.sleep(2)
            return "simulated place_new_carousel"
        pose = self.PAL12idb.place_new_carousel(self.rob)
        return "new carousel placed on the mixer (tag %s, arm at %s)" % (
            state.get_carousel_on_mixer(), pose)

    def _goto_position(self, station):
        if self.simulate:
            time.sleep(1)
            return "simulated goto %s" % station
        pose = self.PAL12idb.goto_station(self.rob, station)
        return "at %s (%s)" % (station, pose)

    def _goto_transfer_point(self):
        if self.simulate:
            time.sleep(1)
            return "simulated goto transfer point"
        pose = self.PAL12idb.goto_transfer_point(self.rob)
        return "at the transfer point (%s)" % (pose,)

    def _goto_xyz(self, xyz):
        if self.simulate:
            time.sleep(0.5)
            return "simulated goto xyz %s" % (xyz,)
        pose = self.PAL12idb.goto_xyz(self.rob, xyz)
        return "tool at %s" % (pose,)

    def _calibrate_camera_intrinsics(self):
        if self.simulate:
            time.sleep(1)
            return "simulated camera intrinsics calibration"
        r = self.PAL12idb.calibrate_camera_intrinsics(self.rob)
        return ("camera intrinsics from %d ChArUco views: fx=%.1f fy=%.1f "
                "cx=%.1f cy=%.1f, reprojection rms %.3f px (assumed f was %.0f). "
                "Stored; now used by the ChArUco pose. Re-run calibrate_camtcp."
                % (r['views'], r['fx'], r['fy'], r['cx'], r['cy'],
                   r['rms_px'], r['assumed_f']))

    def _calibrate_camtcp(self, target, tag_id):
        if self.simulate:
            time.sleep(1)
            return "simulated camtcp calibration (%s)" % target
        r = self.PAL12idb.run_handeye_calibration(self.rob, tag_id=tag_id, target=target)
        return ("camtcp (flange->camera) solved from %d %s views: %s | "
                "current camtcp %s | rot residual mean %.3f deg, max %.3f deg "
                "over %d pairs. NOT applied -- review, then set robot camtcp."
                % (r['samples'], target, r['camtcp'], r['current_camtcp'],
                   r['rot_residual_deg_mean'], r['rot_residual_deg_max'], r['pairs']))

    def _calibrate_venton_table(self):
        if self.simulate:
            time.sleep(1)
            return "simulated Venton table calibration"
        fit = self.PAL12idb.calibrate_venton_table(self.rob)
        self._set_venton_frame(fit)
        return self._venton_reply(fit)

    def _estimate_venton_table(self):
        if self.simulate:
            time.sleep(1)
            return "simulated Venton table estimate"
        fit = self.PAL12idb.estimate_venton_table(self.rob)
        self._set_venton_frame(fit)
        return "ESTIMATE (open-loop, rough) -- " + self._venton_reply(fit)

    def _set_venton_frame(self, fit):
        """Remember the latest calibrated Venton frame (and persist it), so the
        axes overlay can be drawn later without re-running a calibration."""
        self._venton_frame = {k: [float(v) for v in fit[k]]
                              for k in ('origin_m', 'x_axis', 'y_axis', 'normal')}
        try:
            self.PAL12idb.save_venton_frame(self._venton_frame)
        except Exception:                                # noqa: BLE001
            pass        # a non-writable json dir must not fail the calibration

    def _venton_frame_or_load(self):
        """The in-memory Venton frame, falling back to the persisted one."""
        frame = getattr(self, "_venton_frame", None)
        if frame is not None:
            return frame
        return self.PAL12idb.load_venton_frame() if self.PAL12idb else None

    def _get_joints(self):
        """Report the six joint angles in degrees (base joint first). A fast read
        -- refused while the robot is moving."""
        import math
        if self.simulate:
            return "joints (deg): [0.00, 0.00, 0.00, 0.00, 0.00, 0.00] [simulated]"
        if self.rob is None:
            return "ERROR: no robot connection"
        if self._busy:
            return ("ERROR: busy -- cannot read the joints while the robot is "
                    "moving; try again when it is idle")
        j = [math.degrees(a) for a in self.rob.getj()]
        return ("joints (deg): [%s]  (base = %.2f deg)"
                % (", ".join("%.2f" % a for a in j), j[0]))

    def _get_tcp(self):
        """Report the active tool TCP vector and whether it is the gripper or
        camera frame. A fast read -- refused while the robot is moving."""
        if self.simulate:
            return "tcp gripper: [0.0000, 0.0000, 0.1950, 0.0000, 0.0000, 0.0000] [simulated]"
        if self.rob is None:
            return "ERROR: no robot connection"
        if self._busy:
            return ("ERROR: busy -- cannot read the TCP while the robot is "
                    "moving; try again when it is idle")
        tcp = list(self.rob.get_tcp())
        def _close(a, b):
            return all(abs(float(x) - float(y)) < 1e-4 for x, y in zip(a, b))
        which = ("gripper" if _close(tcp, self.rob.tcp)
                 else "camera" if _close(tcp, self.rob.camtcp) else "other")
        return "tcp %s: [%s]" % (which, ", ".join("%.4f" % x for x in tcp))

    def _get_pose(self, frame="gripper"):
        """Report the current TCP pose, in the gripper frame (default) or the
        camera frame (frame="camera").

        A fast read -- refused while the robot is moving, since a pose read
        mid-move is a moving target. Reads with the requested TCP active, then
        restores the gripper TCP (set_tcp changes only the reference frame, not
        the arm). See cmd.GET_POSE.
        """
        if self.simulate:
            return ("pose %s (m, rad): [0.0000, 0.0000, 0.3000, 0.0000, 0.0000, 0.0000] [simulated]"
                    % frame)
        if self.rob is None:
            return "ERROR: no robot connection"
        if self._busy:
            return ("ERROR: busy -- cannot read the pose while the robot is "
                    "moving; try again when it is idle")
        self.rob.set_tcp(self.rob.camtcp if frame == "camera" else self.rob.tcp)
        v = self.rob.get_pose().get_pose_vector().tolist()
        self.rob.set_tcp(self.rob.tcp)                    # leave the gripper TCP active
        return "pose %s (m, rad): [%s]" % (frame, ", ".join("%.4f" % x for x in v))

    def _tag_distance(self, args):
        """Report the camera-to-tag distance for the AprilTag currently in view.

        Optional station arg (args[0]) sets the tag size/ids so the distance is
        scaled for that station's tag; without it the camera's current settings
        are used. A fast read -- refused while the robot is moving, since a frame
        grabbed mid-move is unreliable. See cmd.TAG_DISTANCE.
        """
        if len(args) > 1:
            return "ERROR: usage: %s [station]" % cmd.TAG_DISTANCE
        station = args[0] if args else None
        if self.simulate:
            return "tag ? at 0.2000 m (tag size 0.0064 m) [simulated]"
        if self.rob is None:
            return "ERROR: no robot connection"
        if self._busy:
            return ("ERROR: busy -- cannot read the tag distance while the robot "
                    "is moving; try again when it is idle")
        pal = self.PAL12idb
        if station is not None:
            if station not in pal.APRILTAG_SUBJECT:
                return "ERROR: unknown station %r" % station
            self.rob.camera.AT_physical_size = pal.apriltag_size(station)
            self.rob.camera.AT_preferred_id = pal.apriltag_ids(station)
        try:
            det = pal.camera_tools._detect_apriltag(self.rob, settle=5)
        except Exception as e:                           # noqa: BLE001
            return "ERROR: %s" % e
        if det is None:
            seen = list(getattr(self.rob.camera, "AT_ids", []) or [])
            return "ERROR: no AprilTag in view (tags seen: %s)" % seen
        return "tag %s at %.4f m (tag size %.4f m)" % (
            self.rob.camera.AT_id, self.rob.camera.QRdistance,
            self.rob.camera.AT_physical_size)

    def _venton_axes(self, args):
        """Project the stored Venton frame into the current view -- see
        cmd.VENTON_AXES. Fast and synchronous: the GUI calls it once when the
        overlay toggle goes on, then freezes the result."""
        if not (2 <= len(args) <= 3):
            return "ERROR: usage: %s <width> <height> [axis_len_mm]" % cmd.VENTON_AXES
        try:
            w, h = int(args[0]), int(args[1])
            axis_len_m = (float(args[2]) / 1000.0) if len(args) == 3 else 0.05
        except ValueError:
            return "ERROR: %s needs integer width/height (and a numeric mm)" % cmd.VENTON_AXES
        frame = self._venton_frame_or_load()
        if frame is None:
            return ("ERROR: no Venton frame calibrated yet -- run "
                    "calibrate_venton_level (or calibrate_venton_table) first")
        if self.simulate:
            return json.dumps({"origin": [w / 2.0, h / 2.0],
                               "x": [w / 2.0 + 40, h / 2.0],
                               "y": [w / 2.0, h / 2.0 + 40],
                               "z": [w / 2.0, h / 2.0 - 40],
                               "in_front": [True, True, True, True],
                               "axis_len_m": axis_len_m, "image_wh": [w, h]})
        if self.rob is None:
            return "ERROR: no robot connection"
        if self._busy:
            return ("ERROR: busy -- cannot read the camera pose while the robot "
                    "is moving; try again when it is idle")
        try:
            overlay = self.PAL12idb.venton_axes_overlay(
                self.rob, frame, (w, h), axis_len_m)
        except Exception as e:                           # noqa: BLE001
            return "ERROR: %s" % e
        return json.dumps(overlay)

    #: Where the level calibration drops its annotated frame (red origin dot).
    _VENTON_DOT_PATH = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "venton_origin_dot.png")

    def _calibrate_venton_level(self, tag_ids, heading="orientation"):
        if self.simulate:
            time.sleep(1)
            return "simulated level Venton calibration (%s, tags %s)" % (
                heading, tag_ids or "all")
        fit = self.PAL12idb.calibrate_venton_table_level(
            self.rob, tag_ids=tag_ids, heading=heading)
        self._set_venton_frame(fit)
        # Project the found origin back into a fresh frame and mark it, so the
        # result can be eyeballed against the table, not just read as numbers.
        # A failed render must not lose the calibration -- report it and go on.
        dot = "not rendered"
        try:
            _path, pixel = self.PAL12idb.render_point_on_camera(
                self.rob, fit['origin_m'], self._VENTON_DOT_PATH, label="(0,0)")
            dot = ("origin dot at pixel (%.0f, %.0f) in %s" % (pixel[0], pixel[1], _path)
                   if pixel else "origin projects OUTSIDE the frame; saved %s" % _path)
        except Exception as e:                           # noqa: BLE001
            dot = "could not render origin dot: %s" % e
        s = self.PAL12idb.venton_calibration_summary(fit)
        pos_yaw = fit.get('position_yaw_deg')
        return ("%s fit over tags %s: origin %s m, X %s, Y %s, normal %s; "
                "yaw %.2f deg (positions imply %s), rmse %.2f mm (max %.2f mm); %s"
                % (fit.get('method', 'level').upper(), [t['id'] for t in s['tags']],
                   s['origin_m'], s['x_axis'], s['y_axis'], s['normal'],
                   fit['yaw_deg'],
                   ("%.2f deg" % pos_yaw) if pos_yaw is not None else "n/a",
                   s['rmse_mm'], s['max_error_mm'], dot))

    def _venton_points(self):
        """Base-frame positions collected one tag at a time by
        measure_venton_tag, keyed by tag id. Lives on the server so it persists
        across commands until fit or clear."""
        if not hasattr(self, "_venton_pts"):
            self._venton_pts = {}
        return self._venton_pts

    def _measure_venton_tag(self, tag_id):
        if self.simulate:
            time.sleep(0.5)
            self._venton_points()[tag_id] = [0.0, 0.0, 0.0]
            return "simulated measure of tag %d" % tag_id
        pos = self.PAL12idb.measure_venton_tag(self.rob, tag_id)
        if pos is None:
            raise RuntimeError(
                "could not acquire tag %d -- aim the camera so it is near the "
                "image centre and in focus, then try again" % tag_id)
        self._venton_points()[tag_id] = pos
        got = sorted(self._venton_points())
        more = [t for t in (self.PAL12idb.venton_table_tags()) if t not in got]
        return ("tag %d at [%.4f, %.4f, %.4f] m; collected %s%s"
                % (tag_id, pos[0], pos[1], pos[2], got,
                   "; still need %s, then fit_venton_table" % more if more
                   else "; run fit_venton_table"))

    def _align_camera_north(self, azimuth):
        if self.simulate:
            time.sleep(0.3)
            return "simulated align camera north to %.1f deg" % azimuth
        before, after = self.PAL12idb.align_camera_north(self.rob, target_az_deg=azimuth)
        return ("camera north azimuth %.1f -> %.1f deg (target %.1f)"
                % (before, after, azimuth))

    def _venton_reply(self, fit):
        s = self.PAL12idb.venton_calibration_summary(fit)
        return ("Venton table frame: origin %s m, X %s, Y %s, normal %s; "
                "scale %.4f, fit rmse %.2f mm (max %.2f mm) over %d tags %s"
                % (s['origin_m'], s['x_axis'], s['y_axis'], s['normal'],
                   s['scale'], s['rmse_mm'], s['max_error_mm'],
                   len(s['tags']), [t['id'] for t in s['tags']]))

    def _tweak_orientation(self, axis, degrees):
        if self.simulate:
            time.sleep(0.3)
            return "simulated %s rotation of %s deg" % (axis, degrees)
        pose = self.PAL12idb.tweak_orientation(self.rob, axis, degrees)
        return "rotated %s by %s deg, now at %s" % (axis, degrees, pose)

    def _tweak_position(self, axis, millimetres):
        """Jog the arm along a base axis. Same worker path as the orientation
        tweak: a real move, so it must not run on the reply thread."""
        if self.simulate:
            time.sleep(0.3)
            return "simulated %s jog of %s mm" % (axis, millimetres)
        pose = self.PAL12idb.tweak_position(self.rob, axis, millimetres / 1000.0)
        return "moved %s by %s mm, now at %s" % (axis, millimetres, pose)

    def _zalign(self):
        """Level the tool Z axis (robUR.Zalign). Runs through the same busy /
        ACCEPTED path as a transport: it is a real move, slow by design
        (acc=vel=0.05), so it must not hold up the reply thread."""
        if self.simulate:
            time.sleep(0.5)
            return "simulated zalign"
        pose = self.rob.Zalign()
        return "z-aligned at %s" % (pose,)

    def _release_gripper(self):
        """Open the gripper where the arm is standing.

        A worker action, so it cannot be pressed part-way through a transport.
        It can still be pressed with something held -- ready_flowcell_to_draw
        and flowcell_to_sample_on_mixer both finish holding the flowcell -- so
        anything the tracking says is in the gripper becomes unknown: after
        this it is wherever it fell, which is not something this can know.
        """
        if self.simulate:
            time.sleep(0.2)
            what = "simulated gripper release"
        else:
            # Both activate_gripper() and release() are URScript programs, and
            # the controller runs none while it is stopped -- so this used to
            # send two commands into the void and report "gripper released"
            # anyway. Clear a protective stop first, then release, then look
            # again: a worker action can afford the ~6 s the hold-off costs,
            # and a release that did not happen must not be reported as one.
            mode = self._safety_mode()
            if mode == 3:
                print("PALmixerServer: protective stopped; clearing it before "
                      "opening the gripper.")
                if not self.rob.unlock_protective_stop():
                    raise RuntimeError(
                        "the protective stop would not clear, so the gripper "
                        "cannot be opened -- clear the cause and try again")
            elif mode is not None and mode > 2:
                # A safeguard stop or an e-stop is not ours to release: that is
                # a door open or a button pressed, and unlocking it from here
                # would be undoing someone's deliberate safing.
                raise RuntimeError(
                    "the robot is in safety mode %d, not a protective stop -- "
                    "the gripper cannot be opened until that is cleared at the "
                    "robot" % mode)
            self.PAL12idb.activate_gripper(self.rob)
            self.rob.release()
            if self._safety_mode() not in (1, 2, None):
                raise RuntimeError(
                    "the robot stopped while releasing, so the gripper command "
                    "was discarded -- clear the cause and try again")
            what = "gripper released"
        # Same bookkeeping and the same wording either way: a simulated run is
        # there to rehearse what the real one reports.
        dropped = self._forget_held_flowcell()
        if dropped:
            return ("%s; flowcell %d was being held, so its location is now "
                    "unknown -- reconcile it with set_location" % (what, dropped))
        return what

    def _bump_z(self, backoff_mm=0.0):
        """Bump straight down until the tool touches, then back off `backoff_mm`.

        A worker action (a real force-sensed descent). The gripper TCP is made
        active so the Z it stops at is in the frame the taught positions use. With
        backoff_mm the tool retreats that far above the contact (ending there);
        the measured surface, travel and final height are stored as last_result.
        """
        if self.simulate:
            time.sleep(0.3)
            return "simulated bump; surface z = 0.0000 m (backoff %g mm)" % backoff_mm
        backoff = backoff_mm / 1000.0
        self.rob.set_tcp(self.rob.tcp)
        # Fully shut the gripper first so the fingertips make a clean, defined
        # contact -- a surface measured with an open/part-open gripper is not the
        # same height.
        self.PAL12idb.close_gripper_fully(self.rob)
        z0 = self.rob.get_pos()[2]               # before the descent
        self.rob.bump(z=-1, backoff=backoff)
        z_final = self.rob.get_pos()[2]          # after the backoff
        surface = z_final - backoff               # the contact point
        travel = z0 - surface                    # how far it descended to touch
        if backoff_mm:
            return ("bumped; traveled %.4f m down to surface z = %.4f m, backed "
                    "off %g mm to z = %.4f m" % (travel, surface, backoff_mm, z_final))
        return ("bumped; traveled %.4f m down to contact (surface z = %.4f m)"
                % (travel, surface))

    def _grab_gripper(self):
        """Close the gripper where the arm is standing.

        The mirror of _release_gripper: a worker action, refused mid-transport.
        Closing holds nothing new as far as the tracking is concerned, so there
        is no held-flowcell bookkeeping -- it just shuts the jaws. Like release,
        it clears a protective stop first (the controller runs no URScript while
        stopped) and verifies the robot did not stop during the close, so a
        command that did not happen is not reported as if it did.
        """
        if self.simulate:
            time.sleep(0.2)
            return "simulated gripper grab"
        mode = self._safety_mode()
        if mode == 3:
            print("PALmixerServer: protective stopped; clearing it before "
                  "closing the gripper.")
            if not self.rob.unlock_protective_stop():
                raise RuntimeError(
                    "the protective stop would not clear, so the gripper "
                    "cannot be closed -- clear the cause and try again")
        elif mode is not None and mode > 2:
            raise RuntimeError(
                "the robot is in safety mode %d, not a protective stop -- the "
                "gripper cannot be closed until that is cleared at the robot"
                % mode)
        self.PAL12idb.activate_gripper(self.rob)
        self.rob.grab()
        if self._safety_mode() not in (1, 2, None):
            raise RuntimeError(
                "the robot stopped while grabbing, so the gripper command was "
                "discarded -- clear the cause and try again")
        return "gripper grabbed"

    def _safety_mode(self):
        """The robot's safety mode, or None if it cannot be read.

        1 NORMAL / 2 REDUCED are the only two in which the controller will run
        a URScript program; 3 is a protective stop, and 5-11 are safeguard
        stops, emergency stops, violations and faults. Reads never raise on a
        stopped robot (they go to the cached secmon dict), but the connection
        itself can be down, hence the None.
        """
        try:
            return int(self.rob.get_safety_mode())
        except Exception:                               # noqa: BLE001
            try:
                return 3 if self.rob.is_protective_stopped() else 1
            except Exception:                           # noqa: BLE001
                return None

    @staticmethod
    def _forget_held_flowcell():
        """Mark a flowcell the tracking believes is in the gripper as unknown.
        Returns its id, or None if nothing was being held."""
        for fc in state.FLOWCELL_IDS:
            if state.get_flowcell_location(fc) == state.FC_IN_GRIPPER:
                state.set_flowcell_location(fc, state.UNKNOWN)
                return fc
        return None

    def _unlock_stop(self):
        """Clear a protective stop. A fast command -- see cmd.UNLOCK_STOP.

        Answered synchronously, so it is kept short: one dashboard call and a
        brief look at whether it took. The look is worth the half second,
        because "I pressed it and nothing said otherwise" is not the same as
        knowing the robot is free again.
        """
        if self.simulate:
            return "OK simulated protective-stop unlock"
        if self.rob is None:
            return "ERROR: no robot connection"
        try:
            # The dashboard says whether the controller accepted the release;
            # robUR.unlock_stop() throws that answer away, so it is asked
            # directly -- the same one-level reach _stop_search makes for
            # stopj(). Without it a refusal and a release that did not take
            # hold are indistinguishable, and they need different advice.
            accepted = bool(self.rob.dashboard.unlock())
        except Exception as e:                          # noqa: BLE001
            return "ERROR: could not unlock the protective stop: %s" % e
        # secmon lags the unlock slightly, so give it a moment before asking.
        time.sleep(0.5)
        try:
            still_stopped = self.rob.is_protective_stopped()
        except Exception:                               # noqa: BLE001
            return "OK protective-stop unlock sent (could not read the state back)"
        if not still_stopped:
            return "OK protective stop cleared"
        if not accepted:
            # The common case, and the one that used to look like a dead
            # button: UR refuses to release a protective stop until about 5 s
            # after it tripped, and this command is answered inline on the
            # socket thread, so it cannot sit and wait out the hold-off
            # without also stalling status and stop_search.
            return ("OK unlock refused by the controller -- it will not release "
                    "a protective stop until about 5 s after it trips. Wait a "
                    "moment and press again.")
        return ("OK protective-stop unlock sent and accepted, but the robot "
                "still reports a stop -- the cause is probably still there; "
                "clear it and press again")

    def _freedrive(self, args):
        """Toggle freedrive (hand-guiding) -- see cmd.FREEDRIVE.

        Fast, like _unlock_stop: ending freedrive must work even while the
        server is holding busy, because that is a recovery. Starting it is
        refused while a worker action is running -- letting the arm's brakes go
        in the middle of a move is dangerous.
        """
        if len(args) != 1 or args[0] not in (cmd.FREEDRIVE_ON, cmd.FREEDRIVE_OFF):
            return "ERROR: usage: %s %s|%s" % (
                cmd.FREEDRIVE, cmd.FREEDRIVE_ON, cmd.FREEDRIVE_OFF)
        on = args[0] == cmd.FREEDRIVE_ON
        if self.simulate:
            return "OK simulated freedrive %s" % args[0]
        if self.rob is None:
            return "ERROR: no robot connection"
        if on and self._busy:
            return ("ERROR: cannot enter freedrive while the robot is busy -- "
                    "stop the running action first")
        try:
            self.rob.set_freedrive(on)
        except Exception as e:                           # noqa: BLE001
            return "ERROR: could not turn freedrive %s: %s" % (args[0], e)
        return "OK freedrive %s" % args[0]

    def _face_camera_down(self):
        """Level the camera to point straight down, keeping its position/heading.

        Zalign on the camera TCP, so the camera (not the gripper) is held in place
        and the wrist swings to bring the camera's Z straight down; the gripper TCP
        is restored afterwards. A worker move.
        """
        if self.simulate:
            time.sleep(0.5)
            return "simulated face camera down"
        self.rob.set_tcp(self.rob.camtcp)
        pose = self.rob.Zalign()
        self.rob.set_tcp(self.rob.tcp)
        return "camera faced down at %s" % (pose,)

    def _measure_tag_heights(self, only_ids=None):
        """Bump-measure tags from the last raster scan and report the table with
        the measured surface Z added. `only_ids` limits it to a subset (and skips
        the initial put_tcp2camera, for continuing after a stop). A worker motion.
        """
        scan = getattr(self, "_last_scan", None)
        if not scan:
            return ("ERROR: no raster scan to measure -- run raster_tag_scan first")
        if self.simulate:
            time.sleep(0.5)
            return "simulated tag-height measurement"
        tags_xyz = {tid: scan[tid]["xyz"] for tid in scan}
        measured = self.PAL12idb.measure_tag_heights(
            self.rob, tags_xyz, only_ids=only_ids,
            put_tcp2cam=(not only_ids))
        lines = ["tag-heights: scan xyz + bump-measured surface Z:"]
        for tid in sorted(scan):
            x, y, zc = scan[tid]["xyz"]
            mz = measured.get(tid)
            mz_s = "%.4f" % mz if mz is not None else "?"
            dz = ("%+.4f" % (mz - zc)) if mz is not None else "?"
            lines.append("  tag %2d  scan (%.4f, %.4f, %.4f)  bump Z %s  (dz %s)"
                         % (tid, x, y, zc, mz_s, dz))
        report = "\n".join(lines)
        print(report)
        return report

    def _raster_tag_scan(self, base_from, base_to):
        """Cylindrical raster scan for AprilTags (camera must face down); report
        every tag found as id -> base-frame (x, y, z). A worker motion."""
        if self.simulate:
            time.sleep(0.5)
            return "simulated raster tag scan (no tags)"
        found = self.PAL12idb.raster_tag_scan(
            self.rob, base_from=base_from, base_to=base_to)
        self._last_scan = found                    # kept for measure_tag_heights
        if not found:
            print("raster_tag_scan: no AprilTags found")
            return "raster_tag_scan: no AprilTags found"
        lines = ["raster_tag_scan: found %d tag(s):" % len(found)]
        for tid in sorted(found):
            x, y, z = found[tid]["xyz"]
            lines.append("  tag %2d at (%.4f, %.4f, %.4f) m  [%d sightings, seen at %s]"
                         % (tid, x, y, z, found[tid]["n"], found[tid]["seen_at"]))
        report = "\n".join(lines)
        print(report)
        return report

    def _locate_mixer_carousel(self, base_from, base_to):
        """Raster-scan, identify the carousel on the mixer (tag ~65 mm -X/-Y from
        the mixer-head tag), store its tag centre, and report the sample position.
        A worker motion. Keeps self._last_scan so measure_tag_heights still works."""
        if self.simulate:
            time.sleep(0.5)
            return "simulated locate_mixer_carousel (no tags)"
        res = self.PAL12idb.locate_mixer_carousel(
            self.rob, base_from=base_from, base_to=base_to)
        self._last_scan = res.get("tags") or {}    # kept for measure_tag_heights
        report = res.get("summary") or "locate_mixer_carousel: no result"
        print(report)
        return report

    def _read_tag(self, tag_id):
        """Read one AprilTag's base (x, y, z) from the current camera view (no
        motion); report it or say it is not in view. A worker call (it touches
        the camera/TCP), but it does not move the arm."""
        if self.simulate:
            time.sleep(0.2)
            return "simulated read_tag %d (none)" % tag_id
        pose = self.PAL12idb.read_tag_in_view(self.rob, tag_id)
        if pose is None:
            return ("tag %d not in the current camera view -- aim the camera at it "
                    "first (face_camera_down / move over it)." % tag_id)
        return ("tag %d at (%.4f, %.4f, %.4f) m, rotvec (%.4f, %.4f, %.4f) (base)"
                % (tag_id, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))

    def _read_tags(self):
        """Read every AprilTag in the current camera view (no motion); report each
        id's base (x, y, z) and role. A worker call; it does not move the arm."""
        if self.simulate:
            time.sleep(0.2)
            return "simulated read_tags (none)"
        found = self.PAL12idb.read_tags_in_view(self.rob)
        if not found:
            return ("no AprilTags in the current camera view -- aim the camera "
                    "(face_camera_down / move over them) first.")
        lines = ["tags in the current view (%d):" % len(found)]
        for tid in sorted(found):
            x, y, z = found[tid]
            role = self.PAL12idb._tag_role(tid)
            lines.append("  tag %2d%s at (%.4f, %.4f, %.4f) m"
                         % (tid, (" (%s)" % role) if role else "", x, y, z))
        report = "\n".join(lines)
        print(report)
        return report

    def _survey_tags(self):
        """After a raster scan, center the camera on the mixer head, the carousel
        on the mixing station, and flowcells 1 & 2, and bump the gripper down to
        measure each one's surface Z. Uses the most recent raster scan's positions
        (self._last_scan). A worker motion; reports centered X,Y + bumped Z."""
        scan = getattr(self, "_last_scan", None)
        if not scan:
            return ("no raster scan to survey -- run raster_tag_scan or "
                    "locate_mixer_carousel first.")
        if self.simulate:
            time.sleep(0.5)
            return "simulated survey_tags (no motion)"
        res = self.PAL12idb.survey_key_tags(self.rob, scan)
        if not res:
            return "survey_tags: none of the target tags were in the last scan."
        lines = ["survey (centered X,Y + bumped Z):"]
        for tid in sorted(res):
            r = res[tid]
            cx, cy, _cz = r["centered_xyz"]
            lines.append("  %-20s tag %2d: centered (%.4f, %.4f), bump Z %.4f m"
                         % (r["role"], tid, cx, cy, r["bumped_z"]))
        report = "\n".join(lines)
        print(report)
        return report

    def _scan_tags(self, start_deg=-45.0, end_deg=45.0, speed_deg_s=1.0):
        """Swing the base and record every AprilTag's base-frame position.

        A worker motion. Returns (and prints) the found tags as id -> (x, y, z).
        """
        if self.simulate:
            time.sleep(0.5)
            return "simulated tag scan (no tags)"
        found = self.PAL12idb.scan_apriltags_during_base_swing(
            self.rob, start_deg=start_deg, end_deg=end_deg, speed_deg_s=speed_deg_s)
        if not found:
            print("scan_tags: no AprilTags found during the swing")
            return "scan_tags: no AprilTags found"
        lines = ["scan_tags: found %d tag(s):" % len(found)]
        for tid in sorted(found):
            x, y, z = found[tid]["xyz"]
            lines.append("  tag %2d at (%.4f, %.4f, %.4f) m  [%d sightings]"
                         % (tid, x, y, z, found[tid]["n"]))
        report = "\n".join(lines)
        print(report)
        return report

    def _move_to_xyz(self, x, y, z, frame="gripper"):
        """Move the chosen tool (gripper|camera) to base-frame X,Y,Z keeping the
        current orientation. Goes over the X,Y at the current height first, then to
        Z, so it does not sweep a diagonal into anything. A worker motion."""
        if self.simulate:
            time.sleep(0.3)
            return "simulated %s move to (%.4f, %.4f, %.4f)" % (frame, x, y, z)
        self.rob.set_tcp(self.rob.camtcp if frame == "camera" else self.rob.tcp)
        cur = self.rob.get_pos()
        self.rob.moveto([x, y, cur[2]])          # over the X,Y at the current height
        self.rob.moveto([x, y, z])               # then to the target Z
        self.rob.set_tcp(self.rob.tcp)           # leave the gripper TCP active
        return "%s at (%.4f, %.4f, %.4f)" % (frame, x, y, z)

    def _set_base_joint(self, deg, speed_deg_s=None):
        """Move the base joint (joint 0) to an ABSOLUTE angle (deg), holding the
        other joints. A worker motion. Default speed is moderate."""
        import math
        if self.simulate:
            time.sleep(0.5)
            return "simulated base joint -> %g deg" % deg
        j = list(self.rob.getj())
        j[0] = math.radians(deg)
        vel = math.radians(speed_deg_s) if speed_deg_s else 0.2
        self.rob.movej(j, acc=0.3, vel=vel, wait=True)
        return "base joint set to %g deg" % deg

    def _set_joint(self, index, deg, speed_deg_s=None):
        """Move one joint (index 0-5) to an ABSOLUTE angle (deg), holding the
        others. A worker motion. Default speed is moderate."""
        import math
        if self.simulate:
            time.sleep(0.5)
            return "simulated joint %d -> %g deg" % (index, deg)
        j = list(self.rob.getj())
        j[index] = math.radians(deg)
        lim = getattr(self.rob, "shoulder_min_deg", None)
        if index == 1 and lim is not None and deg < lim - 1e-6:
            raise ValueError("shoulder target %g deg is below the safety limit "
                             "%g deg; refused" % (deg, lim))
        vel = math.radians(speed_deg_s) if speed_deg_s else 0.2
        self.rob.movej(j, acc=0.3, vel=vel, wait=True)
        return "joint %d set to %g deg" % (index, deg)

    def _reconfigure(self, joints_deg, speed_deg_s=None):
        """Move to an alternate IK branch: record the current TCP pose, movej to
        the six given joint angles (deg), then movel back to the recorded TCP so
        the actual tool pose is preserved (absorbs controller calibration). A
        worker motion."""
        import math
        if self.simulate:
            time.sleep(0.5)
            return "simulated reconfigure -> %s deg" % (joints_deg,)
        lim = getattr(self.rob, "shoulder_min_deg", None)
        if lim is not None and joints_deg[1] < lim - 1e-6:
            raise ValueError("shoulder target %g deg is below the safety limit "
                             "%g deg; refused" % (joints_deg[1], lim))
        pose = list(self.rob.get_pose().get_pose_vector())   # exact current TCP (base)
        q = [math.radians(v) for v in joints_deg]
        vel = math.radians(speed_deg_s) if speed_deg_s else 0.3
        self.rob.movej(q, acc=0.3, vel=vel, wait=True)        # alternate branch (nominal)
        self.rob.movel(pose, acc=0.3, vel=0.1, wait=True)     # refine to exact TCP, same branch
        now = [round(math.degrees(v), 3) for v in self.rob.getj()]
        return ("reconfigured to joints %s deg; TCP restored to %s" %
                (now, [round(v, 5) for v in pose]))

    def _swing_base(self, start_deg, end_deg, speed_deg_s):
        """Swing the base joint (joint 0) from current+start_deg to current+end_deg.

        Only joint 0 moves (the other joints are held at their current angles).
        It first positions to the start angle at a moderate speed, then swings to
        the end angle at speed_deg_s deg/s. A worker motion -- a real, large move.
        """
        import math
        if self.simulate:
            time.sleep(0.5)
            return ("simulated base swing %+g -> %+g deg at %g deg/s"
                    % (start_deg, end_deg, speed_deg_s))
        j0 = self.rob.getj()
        base = j0[0]
        start = list(j0); start[0] = base + math.radians(start_deg)
        end = list(j0);   end[0] = base + math.radians(end_deg)
        swing_vel = math.radians(speed_deg_s)
        # Position to the start angle at a moderate speed, then swing at the asked speed.
        self.rob.movej(start, acc=0.3, vel=0.2, wait=True)
        self.rob.movej(end, acc=0.3, vel=swing_vel, wait=True)
        return ("swung base %+g -> %+g deg at %g deg/s"
                % (start_deg, end_deg, speed_deg_s))

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

    # -- pump status polling ---------------------------------------------------
    def _poll_pump_status(self):
        """Refresh self._pump_status forever, on its own thread.

        Its own thread, and its own sockets inside Pump, so neither the ZMQ
        reply path nor a running pump operation ever waits on it -- and so the
        panel keeps updating *during* an operation, which is when it is worth
        looking at. Failures are data, not errors: status_snapshot() reports an
        unreachable dashboard as such, which is what the panel should show.
        """
        while not self._stopping.is_set():
            try:
                self._pump_status = self.pump.status_snapshot()
            except Exception as e:                       # noqa: BLE001
                self._pump_status = {"error": str(e), "time": time.time()}
            self._stopping.wait(PUMP_STATUS_INTERVAL_S)

    # -- lifecycle ------------------------------------------------------------
    def start(self):
        self._mixing_journal.recover_interrupted()
        self._publish_state(STATE_IDLE)
        self._publish_tracking(state.snapshot())
        threading.Thread(target=self._poll_pump_status, daemon=True).start()
        threading.Thread(target=self._sync_mixing_records, daemon=True).start()
        self.zmq_server.start()

    def stop(self):
        self._stopping.set()
        self._mix_sync_wake.set()
        self.zmq_server.stop()
        self._mqtt.close()


def main():
    parser = argparse.ArgumentParser(description="PALmixer headless control server")
    parser.add_argument("--simulate", action="store_true",
                         help="do not touch real robot/motor hardware")
    args = parser.parse_args()

    server = PALmixerServer(simulate=args.simulate)
    try:
        server.start()
    except ZMQError as e:
        # No client can reach this server, so do not hold the port-less process
        # open in the loop below as if one could.
        print("\nFAILED TO START: %s" % e)
        server.stop()
        return 1
    print("PALmixer server running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
