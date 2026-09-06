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
"""

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

PUMP_OPS = (
    PUMP_MIX,
    PUMP_CLEAN_MIXER,
    PUMP_DRAW_TO_FLOWCELL,
    PUMP_ASPIRATE_FROM_FLOWCELL,
)

PUMP_LABELS = {
    PUMP_MIX: "Mix",
    PUMP_CLEAN_MIXER: "Clean Mixer",
    PUMP_DRAW_TO_FLOWCELL: "Draw to Flowcell",
    PUMP_ASPIRATE_FROM_FLOWCELL: "Aspirate from Flowcell",
}


def search_apriltag_command(station):
    """Build the wire command for searching a station's AprilTag."""
    return "%s %s" % (SEARCH_APRILTAG, station)


def motor_tweak_command(direction, step):
    """Build the wire command for a motor tweak in ``direction`` by ``step``."""
    return "%s %s %s" % (MOTOR_TWEAK, direction, step)


def pump_command(op):
    """Build the wire command for a pump operation."""
    return "%s %s" % (PUMP, op)
