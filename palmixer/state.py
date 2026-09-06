# -*- coding: utf-8 -*-
"""Persistent tracking state for PALmixer: what is where, and the carousel table.

Four things are tracked, backed by ``palmixer_state.ini`` next to this file so
they survive a server restart:

    A) the mixer head's location   -- at the mixer, or at its cleaning station
    B) flowcell 1 and 2 locations  -- independently, at the beam or at cleaning
    C) the carousel slot last moved to (the EPICS motor holds the true position)
    D) which flowcell is in use    -- PAL12idb's flowcell_ID

Plus the taught slot -> motor position table the carousel moves use.

IMPORTANT -- this module must stay dependency-free (stdlib only). PAL12idb.py
imports ``camera_tools`` at module scope, which only resolves inside the
beamline ``aps12robot`` environment, so server.py imports PAL12idb only when
not simulating. Keeping the store here means simulate mode, and anything else
that just wants to read state, can import it anywhere. PAL12idb imports this
module; never the reverse.

Every location has an ``unknown`` value and that is the default. On first run,
and after any failed move or manual intervention, ``unknown`` is the honest
answer -- recording a location that was never reached is worse than admitting
we do not know. Callers are expected to refuse to move rather than guess.
"""

import configparser
import os
import threading

# -- location vocabulary ----------------------------------------------------
UNKNOWN = "unknown"

MIXER_AT_MIXER = "mixer_station"
MIXER_AT_CLEANING = "mixer_cleaning_station"
MIXER_LOCATIONS = (MIXER_AT_MIXER, MIXER_AT_CLEANING, UNKNOWN)

FC_AT_CLEANING = "cleaning_station"
FC_AT_BEAM = "sample_table"
FC_IN_GRIPPER = "gripper"  # transient: held by the robot, in transit at the mixer
FC_LOCATIONS = (FC_AT_CLEANING, FC_AT_BEAM, FC_IN_GRIPPER, UNKNOWN)

FLOWCELL_IDS = (1, 2)

# Names used on the wire by "set_location <what> <value>".
WHAT_MIXER_HEAD = "mixer_head"
WHAT_FLOWCELL_1 = "flowcell_1"
WHAT_FLOWCELL_2 = "flowcell_2"
LOCATION_KEYS = (WHAT_MIXER_HEAD, WHAT_FLOWCELL_1, WHAT_FLOWCELL_2)

_INI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "palmixer_state.ini")
_TRACKING = "tracking"
_CAROUSEL = "carousel"

# Re-entrant: the setters below call the getters while holding it.
_lock = threading.RLock()
_listeners = []


# -- ini plumbing -----------------------------------------------------------
def _read():
    parser = configparser.ConfigParser()
    parser.read(_INI_PATH)
    return parser


def _write(parser):
    # Written on every transition, so use a temp file + replace: a crash
    # mid-write must not leave a half-truncated state file behind.
    tmp = _INI_PATH + ".tmp"
    with open(tmp, "w") as fp:
        parser.write(fp)
    os.replace(tmp, _INI_PATH)


def _get(section, key, default=UNKNOWN):
    parser = _read()
    if not parser.has_option(section, key):
        return default
    value = parser.get(section, key).strip()
    return value or default


def _set(section, key, value):
    """Set one key, preserving every other section/key in the file."""
    with _lock:
        parser = _read()
        if not parser.has_section(section):
            parser.add_section(section)
        parser.set(section, key, str(value))
        _write(parser)
    _notify()


# -- change notification ----------------------------------------------------
def add_listener(fn):
    """Register ``fn(snapshot_dict)``, called after every change.

    server.py uses this to publish the tracking state over MQTT without every
    call site having to remember to do it. Listener errors are swallowed --
    telemetry must never break hardware control.
    """
    _listeners.append(fn)


def _notify():
    # Called with the lock released: a listener publishing over MQTT should
    # never be able to stall a state write.
    snap = snapshot()
    for fn in list(_listeners):
        try:
            fn(snap)
        except Exception as e:
            print("WARNING: state listener failed: %s" % e)


# -- A) mixer head ----------------------------------------------------------
def get_mixer_head():
    return _get(_TRACKING, WHAT_MIXER_HEAD)


def set_mixer_head(location):
    if location not in MIXER_LOCATIONS:
        raise ValueError("mixer head location must be one of %s, got %r"
                         % (list(MIXER_LOCATIONS), location))
    _set(_TRACKING, WHAT_MIXER_HEAD, location)


# -- B) flowcell locations --------------------------------------------------
def _flowcell_key(flowcell_id):
    fc = int(flowcell_id)
    if fc not in FLOWCELL_IDS:
        raise ValueError("flowcell id must be one of %s, got %r"
                         % (list(FLOWCELL_IDS), flowcell_id))
    return "flowcell_%d" % fc


def get_flowcell_location(flowcell_id):
    return _get(_TRACKING, _flowcell_key(flowcell_id))


def set_flowcell_location(flowcell_id, location):
    if location not in FC_LOCATIONS:
        raise ValueError("flowcell location must be one of %s, got %r"
                         % (list(FC_LOCATIONS), location))
    _set(_TRACKING, _flowcell_key(flowcell_id), location)


def set_location(what, location):
    """Operator reconcile: declare where something actually is.

    ``what`` is one of LOCATION_KEYS. This is the way out of ``unknown`` after
    a manual intervention -- deliberately a separate, explicit action rather
    than a force flag on the workflows.
    """
    if what == WHAT_MIXER_HEAD:
        set_mixer_head(location)
    elif what in (WHAT_FLOWCELL_1, WHAT_FLOWCELL_2):
        set_flowcell_location(int(what.rsplit("_", 1)[1]), location)
    else:
        raise ValueError("cannot set %r; expected one of %s"
                         % (what, list(LOCATION_KEYS)))


# -- C) carousel ------------------------------------------------------------
def get_carousel_slot():
    """The slot last moved to, as an int, or UNKNOWN. Int rather than the raw
    ini string so the JSON snapshot types it like flowcell_in_use."""
    value = _get(_TRACKING, "carousel_slot", UNKNOWN)
    if value == UNKNOWN:
        return UNKNOWN
    try:
        return int(value)
    except ValueError:
        return UNKNOWN


def set_carousel_slot(slot):
    _set(_TRACKING, "carousel_slot", int(slot))


def carousel_slots():
    """The taught slot -> motor position table, as ``{int: float}``."""
    parser = _read()
    if not parser.has_section(_CAROUSEL):
        return {}
    slots = {}
    for key, value in parser.items(_CAROUSEL):
        if not key.startswith("slot_"):
            continue
        try:
            slots[int(key.split("_", 1)[1])] = float(value)
        except ValueError:
            continue  # ignore a hand-edited line we cannot parse
    return slots


def teach_carousel_slot(slot, position):
    """Record ``position`` (motor engineering units) as carousel slot ``slot``."""
    _set(_CAROUSEL, "slot_%d" % int(slot), repr(float(position)))


def slot_position(slot):
    """Motor position for ``slot``. Raises KeyError if it has not been taught."""
    slots = carousel_slots()
    key = int(slot)
    if key not in slots:
        raise KeyError("carousel slot %d has not been taught (known slots: %s)"
                       % (key, sorted(slots) or "none"))
    return slots[key]


# -- D) flowcell in use -----------------------------------------------------
def get_flowcell_in_use():
    value = _get(_TRACKING, "flowcell_in_use", "1")
    try:
        fc = int(value)
    except ValueError:
        return 1
    return fc if fc in FLOWCELL_IDS else 1


def set_flowcell_in_use(flowcell_id):
    fc = int(flowcell_id)
    if fc not in FLOWCELL_IDS:
        raise ValueError("flowcell id must be one of %s, got %r"
                         % (list(FLOWCELL_IDS), flowcell_id))
    _set(_TRACKING, "flowcell_in_use", fc)


# -- the whole picture ------------------------------------------------------
def snapshot():
    """Everything tracked, as a plain dict. This is what goes on the wire."""
    return {
        WHAT_MIXER_HEAD: get_mixer_head(),
        WHAT_FLOWCELL_1: get_flowcell_location(1),
        WHAT_FLOWCELL_2: get_flowcell_location(2),
        "flowcell_in_use": get_flowcell_in_use(),
        "carousel_slot": get_carousel_slot(),
        "carousel_slots": carousel_slots(),
    }
