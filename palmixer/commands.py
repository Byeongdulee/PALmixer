# -*- coding: utf-8 -*-
"""Shared ZMQ command vocabulary for the PALmixer client (GUI) and server.

Keeping the command names, station names, and pump op names in one module
means the GUI buttons and the server dispatcher can never drift apart.

Wire protocol (mirrors APS12_SAXSDaq's ZMQCommandServer): plain space-delimited
strings, one request -> one reply.

    "status"                              -> "IDLE" | "BUSY"
    "search_apriltag <station>"           -> "ACCEPTED" | "ERROR: <reason>"
    "<transport_name>"                    -> "ACCEPTED" | "ERROR: <reason>"
    "motor_tweak forward|reverse <step>"  -> "ACCEPTED" | "ERROR: <reason>"
    "pump <op>"                           -> "ACCEPTED" | "ERROR: <reason>"
    "make_sample <slot>"                  -> "ACCEPTED" | "ERROR: <reason>"
    "unload_sample"                       -> "ACCEPTED" | "ERROR: <reason>"
    "set_flowcell <1|2>"                  -> "OK" | "ERROR: <reason>"
    "get_state"                           -> "<JSON snapshot>"
    "set_location <what> <value>"         -> "OK" | "ERROR: <reason>"
    "teach_carousel_slot <n>"             -> "OK" | "ERROR: <reason>"
"""

from . import state as _state

STATUS = "status"

# AprilTag search stations. These are exactly the `pos` strings that
# PAL12idb.locate_apriltag() understands.
STATION_SAMPLE_TABLE = "sample_table"
STATION_CLEANING_STATION = "cleaning_station"
STATION_MIXER_STATION = "mixer_station"
STATION_MIXER_CLEANING_STATION = "mixer_cleaning_station"

STATIONS = (
    STATION_SAMPLE_TABLE,
    STATION_CLEANING_STATION,
    STATION_MIXER_STATION,
    STATION_MIXER_CLEANING_STATION,
)

STATION_LABELS = {
    STATION_SAMPLE_TABLE: "Sample Table",
    STATION_CLEANING_STATION: "Flowcell Cleaning Station",
    STATION_MIXER_STATION: "Mixer Station",
    STATION_MIXER_CLEANING_STATION: "Mixer Cleaning Station",
}

SEARCH_APRILTAG = "search_apriltag"

# Explicit sync between waypoints.ini (which every move reads) and the
# 12idUR:WaypointL:* EPICS PVs (the beamline-wide interchange). These are the
# only commands that do Channel Access on the waypoint PVs.
PUSH_POSITIONS = "push_positions"
PULL_POSITIONS = "pull_positions"

# The 8 "actual transport functions" defined in PAL12idb.py.
TRANSPORT_FUNCTIONS = (
    "mixer2cleaningstation",
    "mixer2mixingstation",
    "load_flowcell_from_cleaningstation_to_beam",
    "load_flowcell_from_beam_to_cleaningstation",
    "ready_flowcell_to_draw",
    "load_sample_to_beam",
    "return_sample",
    "wash_flowcell_after_return",
)

TRANSPORT_LABELS = {
    "mixer2cleaningstation": "Mixer -> Cleaning Station",
    "mixer2mixingstation": "Cleaning Station -> Mixer",
    "load_flowcell_from_cleaningstation_to_beam": "Cleaning Station -> Beam",
    "load_flowcell_from_beam_to_cleaningstation": "Beam -> Cleaning Station",
    "ready_flowcell_to_draw": "Ready Flowcell to Draw",
    "load_sample_to_beam": "Load Sample to Beam",
    "return_sample": "Return Sample from Beam",
    "wash_flowcell_after_return": "Wash Flowcell After Return",
}

MOTOR_TWEAK = "motor_tweak"
MOTOR_FORWARD = "forward"
MOTOR_REVERSE = "reverse"

PUMP = "pump"

# Pump operations requested for the Experiment tab.
PUMP_MIX = "mix"
PUMP_CLEAN_MIXER = "clean_mixer"
PUMP_DRAW_TO_FLOWCELL = "draw_to_flowcell"
PUMP_ASPIRATE_FROM_FLOWCELL = "aspirate_from_flowcell"
PUMP_WASH_FLOWCELL = "wash_flowcell"

PUMP_OPS = (
    PUMP_MIX,
    PUMP_CLEAN_MIXER,
    PUMP_DRAW_TO_FLOWCELL,
    PUMP_ASPIRATE_FROM_FLOWCELL,
    PUMP_WASH_FLOWCELL,
)

PUMP_LABELS = {
    PUMP_MIX: "Mix",
    PUMP_CLEAN_MIXER: "Clean Mixer",
    PUMP_DRAW_TO_FLOWCELL: "Draw to Flowcell",
    PUMP_ASPIRATE_FROM_FLOWCELL: "Aspirate from Flowcell",
    PUMP_WASH_FLOWCELL: "Wash Flowcell",
}

# -- automation workflows (make a full sample / unload a sample) -----------
MAKE_SAMPLE = "make_sample"
UNLOAD_SAMPLE = "unload_sample"

# -- tracking / state commands ----------------------------------------------
SET_FLOWCELL = "set_flowcell"
GET_STATE = "get_state"
SET_LOCATION = "set_location"
TEACH_CAROUSEL_SLOT = "teach_carousel_slot"

FLOWCELL_IDS = _state.FLOWCELL_IDS               # (1, 2)
LOCATION_KEYS = _state.LOCATION_KEYS             # mixer_head, flowcell_1, flowcell_2
MIXER_LOCATIONS = _state.MIXER_LOCATIONS
FC_LOCATIONS = _state.FC_LOCATIONS
UNKNOWN = _state.UNKNOWN
WHAT_MIXER_HEAD = _state.WHAT_MIXER_HEAD
WHAT_FLOWCELL_1 = _state.WHAT_FLOWCELL_1
WHAT_FLOWCELL_2 = _state.WHAT_FLOWCELL_2


# Marker prefix on a "position not configured" ERROR reply, shared between
# server.py (which builds the message) and the GUI (which matches it to
# decide whether to pop up a dialog instead of just logging it).
POSITION_NOT_CONFIGURED = "position not configured"


def position_not_configured_error(missing_labels):
    """Build the ERROR detail for a transport/workflow command that needs a
    station position no one has taught yet (see PAL12idb.check_positions_defined)."""
    return "%s: %s -- use the Configuration tab to search its AprilTag first" % (
        POSITION_NOT_CONFIGURED, ", ".join(missing_labels))


def search_apriltag_command(station):
    """Build the wire command for searching a station's AprilTag."""
    return "%s %s" % (SEARCH_APRILTAG, station)


def push_positions_command():
    """waypoints.ini -> EPICS waypoint PVs."""
    return PUSH_POSITIONS


def pull_positions_command():
    """EPICS waypoint PVs -> waypoints.ini."""
    return PULL_POSITIONS


def motor_tweak_command(direction, step):
    """Build the wire command for a motor tweak in ``direction`` by ``step``."""
    return "%s %s %s" % (MOTOR_TWEAK, direction, step)


def pump_command(op):
    """Build the wire command for a pump operation."""
    return "%s %s" % (PUMP, op)


def make_sample_command(slot):
    """Build the wire command to run the full make-a-sample sequence."""
    return "%s %s" % (MAKE_SAMPLE, slot)


def unload_sample_command():
    """Build the wire command to run the full unload-sample sequence."""
    return UNLOAD_SAMPLE


def set_flowcell_command(flowcell_id):
    """Build the wire command to change which flowcell is in use."""
    return "%s %s" % (SET_FLOWCELL, flowcell_id)


def get_state_command():
    """Build the wire command to fetch the tracking state snapshot."""
    return GET_STATE


def set_location_command(what, location):
    """Build the wire command to reconcile a tracked item to ``location``."""
    return "%s %s %s" % (SET_LOCATION, what, location)


def teach_carousel_slot_command(slot):
    """Build the wire command to record the current motor position as ``slot``."""
    return "%s %s" % (TEACH_CAROUSEL_SLOT, slot)
