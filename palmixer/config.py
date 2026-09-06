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
    motor {pv}                                  -- EPICS motor PV base

Env overrides (take precedence over the JSON):
    PALMIXER_ZMQ_HOST   PALMIXER_ZMQ_PORT
    PALMIXER_MQTT_HOST  PALMIXER_MQTT_PORT
    PALMIXER_ROBOT_IP   PALMIXER_UR12IDB_PATH
    PALMIXER_MOTOR_PV
"""

import json
import os

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
        "ur12idb_path": "/home/beams15/S12STAFF/python_codes/UR_12idb",
    },
    "motor": {"pv": "12idb:m6"},
}


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
    return cfg


def get_section(name):
    """Return one config section as a dict (empty dict if unknown)."""
    return dict(_merged().get(name, {}))


def get_config():
    """Return the whole merged config."""
    return _merged()
