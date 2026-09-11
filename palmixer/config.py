# -*- coding: utf-8 -*-
"""Configuration for PALmixer.

Reads ``json/palmixer_config.json`` (next to the package) and layers environment
variable overrides on top, mirroring the ``app_config`` conventions used in
APS12_SAXSDaq. Everything has a sensible default so the module imports and the
GUI/server run even without the JSON file present.

Sections:
    zmq   {host, port}                          -- control plane (REQ/REP)
    mqtt  {host, port, prefix, beamline}        -- telemetry (pub/sub)
    robot {name, ip, ur12idb_path}              -- UR robot + camera
           ur12idb_path is per-OS: {"linux": ..., "windows": ...}, or a list
           of candidates, or a single path (see _resolve_ur12idb_path)
    motor {pv}                                  -- EPICS motor PV base
    carousel {size, step}                       -- carousel geometry
    pump  {host, mixer_port, flowcell_port, ...}-- apssector12_pump_control ZMQ
    daq   {host, port, request_timeout_s}       -- beamline DAQ GUI ZMQ (sample stage motors)

Env overrides (take precedence over the JSON):
    PALMIXER_ZMQ_HOST   PALMIXER_ZMQ_PORT
    PALMIXER_MQTT_HOST  PALMIXER_MQTT_PORT
    PALMIXER_ROBOT_IP   PALMIXER_UR12IDB_PATH
    PALMIXER_MOTOR_PV
    PALMIXER_CAROUSEL_SIZE  PALMIXER_CAROUSEL_STEP
    PALMIXER_PUMP_HOST  PALMIXER_PUMP_MIXER_PORT  PALMIXER_PUMP_FLOWCELL_PORT
    PALMIXER_DAQ_HOST   PALMIXER_DAQ_PORT

The pump section points at the two loopback ZMQ servers run by the
apssector12_pump_control dashboards (mixing on 5555, flowcell on 5556). Its
ports are configured *here*, not via PALMIXER_ZMQ_PORT: the pump dashboards
themselves read PALMIXER_ZMQ_PORT / PALMIXER_MQTT_PORT for their own ports (the
same env names this package uses for its control plane), so the pump client is
given its own PALMIXER_PUMP_* overrides to avoid the collision. mixing_speed_rpm
/ min_rpm / max_rpm are advisory only -- the pump dashboard sets the real mix
speed (see palmixer/pump.py).

The carousel section describes the *hardware*: how many slots the carousel has
and how far the motor moves between two adjacent ones. Both defaults here are
0, meaning "not configured" -- a wrong step would drive the carousel to the
wrong slot, so an absent one refuses the move instead of guessing. What is in
each slot right now (the taught reference position and the sample IDs) is
live state and lives in palmixer/palmixer_state.ini, not here.
"""

import json
import os
import sys

# Repo root holds json/palmixer_config.json ; this file lives in <root>/palmixer/.
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.dirname(_PKG_DIR)
_CONFIG_PATH = os.path.join(_ROOT_DIR, "json", "palmixer_config.json")

_DEFAULTS = {
    "zmq": {"host": "localhost", "port": 9880},
    "mqtt": {"host": "localhost", "port": 1883, "prefix": "aps12", "beamline": "12idb"},
    "robot": {
        "name": "UR3",
        "ip": "UR3-12idb.xray.aps.anl.gov",
        # One checkout of UR_12idb per operating system: the beamline Linux
        # host keeps it under S12STAFF, the Windows control machine under the
        # user's GitHub folder. Both are named here rather than one being the
        # value and the other a local edit, so the same committed config works
        # on either. See _resolve_ur12idb_path for the accepted forms.
        "ur12idb_path": {
            "linux": "/home/beams15/S12STAFF/python_codes/UR_12idb",
            "windows": "C:/Users/s12idb/Documents/GitHub/UR_12idb",
        },
    },
    "motor": {"pv": "12idb:m6"},
    # `step` is degrees of rotation between adjacent slots -- 12.m6 is a rotary
    # stage, so its engineering units and the carousel's angular pitch are the
    # same number. PVapp's "CRS" holder type describes the same hardware
    # (app/holders.py) and must agree: radius and tube diameter are carried here
    # too so the two can be compared without opening both repos.
    "carousel": {"size": 0, "step": 0.0, "draw_offset_steps": 4,
                 "radius_mm": 50.0, "tube_diameter_mm": 10.0},
    # ZMQ endpoints of the apssector12_pump_control dashboards. speed_* are
    # advisory (the dashboard owns the real mix speed); timeouts govern the
    # REQ/REP round-trip and the poll-to-completion wait in palmixer/pump.py.
    "pump": {"mixing_speed_rpm": 800.0, "min_rpm": 0.0, "max_rpm": 3000.0,
             "host": "sec12b02.xray.aps.anl.gov", "mixer_port": 5555, "flowcell_port": 5556,
             "request_timeout_s": 5.0, "poll_interval_s": 0.5,
             "operation_timeout_s": 600.0},
    # ZMQ endpoint of the beamline DAQ GUI -- a different server from this
    # package's own "zmq" section above. Its getpos/move/status commands read
    # and drive the sample-stage motors, which is a separate control system
    # from the robot: see palmixer/daq_client.py, which records their position
    # whenever the robot's sample_table waypoint is taught and restores it
    # before every transport that touches the sample table.
    "daq": {"host": "purple.xray.aps.anl.gov", "port": 9876,
            "request_timeout_s": 5.0},
}


def _os_key():
    """This machine's key in a per-OS config mapping."""
    if sys.platform.startswith("win"):
        return "windows"
    if sys.platform == "darwin":
        return "macos"
    return "linux"


def _resolve_ur12idb_path(value):
    """Reduce robot.ur12idb_path to the one path to put on sys.path here.

    Accepts, in the JSON or as a default:
      * a mapping {"linux": ..., "windows": ..., "macos": ...} -- this OS's
        entry is preferred, the others are tried only as a last resort;
      * a list of candidate paths, tried in order;
      * a plain string (what PALMIXER_UR12IDB_PATH always gives).

    The first candidate that exists on disk wins. A path belonging to another
    machine is worse than none at all -- sys.path gains a dead entry and
    ``import robot12idb`` then fails with a bare ModuleNotFoundError naming
    only the module -- so if nothing matches, a UR_12idb checkout sitting next
    to this repo is used. Failing even that, this OS's configured path is
    returned unchanged, so the import error names the path that was meant.
    """
    if isinstance(value, dict):
        preferred = value.get(_os_key())
        candidates = [preferred] + [v for k, v in sorted(value.items())
                                    if k != _os_key()]
    elif isinstance(value, (list, tuple)):
        candidates = list(value)
        preferred = candidates[0] if candidates else None
    else:
        preferred = value
        candidates = [value]

    for path in candidates:
        if path and os.path.isdir(path):
            return path

    sibling = os.path.join(os.path.dirname(_ROOT_DIR), "UR_12idb")
    if os.path.isdir(sibling):
        return sibling
    return preferred


def _load_file():
    """Return the parsed JSON config, or {} if it is missing/unreadable."""
    try:
        with open(_CONFIG_PATH) as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except Exception as e:  # malformed JSON should not crash the app
        print("WARNING: could not read %s (%s); using defaults" % (_CONFIG_PATH, e))
        return {}


def _merged():
    """Defaults <- file <- env, deep-merged one level."""
    cfg = {k: dict(v) for k, v in _DEFAULTS.items()}
    for section, values in _load_file().items():
        if isinstance(values, dict):
            cfg.setdefault(section, {}).update(values)
        else:
            cfg[section] = values

    env = os.environ.get
    if env("PALMIXER_ZMQ_HOST"):
        cfg["zmq"]["host"] = env("PALMIXER_ZMQ_HOST")
    if env("PALMIXER_ZMQ_PORT"):
        cfg["zmq"]["port"] = int(env("PALMIXER_ZMQ_PORT"))
    if env("PALMIXER_MQTT_HOST"):
        cfg["mqtt"]["host"] = env("PALMIXER_MQTT_HOST")
    if env("PALMIXER_MQTT_PORT"):
        cfg["mqtt"]["port"] = int(env("PALMIXER_MQTT_PORT"))
    if env("PALMIXER_ROBOT_IP"):
        cfg["robot"]["ip"] = env("PALMIXER_ROBOT_IP")
    if env("PALMIXER_UR12IDB_PATH"):
        cfg["robot"]["ur12idb_path"] = env("PALMIXER_UR12IDB_PATH")
    if env("PALMIXER_MOTOR_PV"):
        cfg["motor"]["pv"] = env("PALMIXER_MOTOR_PV")
    if env("PALMIXER_CAROUSEL_SIZE"):
        cfg["carousel"]["size"] = env("PALMIXER_CAROUSEL_SIZE")
    if env("PALMIXER_CAROUSEL_STEP"):
        cfg["carousel"]["step"] = env("PALMIXER_CAROUSEL_STEP")
    # NB: deliberately NOT PALMIXER_ZMQ_PORT -- that names this package's own
    # control plane, and the pump dashboards reuse it for their ports too.
    if env("PALMIXER_PUMP_HOST"):
        cfg["pump"]["host"] = env("PALMIXER_PUMP_HOST")
    if env("PALMIXER_PUMP_MIXER_PORT"):
        cfg["pump"]["mixer_port"] = int(env("PALMIXER_PUMP_MIXER_PORT"))
    if env("PALMIXER_PUMP_FLOWCELL_PORT"):
        cfg["pump"]["flowcell_port"] = int(env("PALMIXER_PUMP_FLOWCELL_PORT"))
    if env("PALMIXER_DAQ_HOST"):
        cfg["daq"]["host"] = env("PALMIXER_DAQ_HOST")
    if env("PALMIXER_DAQ_PORT"):
        cfg["daq"]["port"] = int(env("PALMIXER_DAQ_PORT"))

    # Collapse the per-OS mapping to a single path, so every consumer
    # (server.py's sys.path insert, the GUI's AprilTag detector import) still
    # sees a plain string and none of them has to know about the platform.
    cfg["robot"]["ur12idb_path"] = _resolve_ur12idb_path(
        cfg["robot"].get("ur12idb_path"))
    return cfg


def get_section(name):
    """Return one config section as a dict (empty dict if unknown)."""
    return dict(_merged().get(name, {}))


def get_config():
    """Return the whole merged config."""
    return _merged()
