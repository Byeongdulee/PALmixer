# -*- coding: utf-8 -*-
"""Shared ZMQ command vocabulary for the PALmixer client (GUI) and server.

Keeping the command names, station names, and pump op names in one module
means the GUI buttons and the server dispatcher can never drift apart.

Wire protocol (mirrors APS12_SAXSDaq's ZMQCommandServer): plain space-delimited
strings, one request -> one reply.

Worker acceptances shown as ACCEPTED below include ``action_id=<id>``. Match
that ID and command against ``get_state.last_result`` to verify completion;
IDLE alone says nothing about success.

    "status"                              -> "IDLE" | "BUSY"
    "search_apriltag <station> [skip_roll]" -> "ACCEPTED" | "ERROR: <reason>"
    "stop_search"                         -> "OK" | "ERROR: <reason>"
    "set_position_here <station>"         -> "OK <pose>" | "ERROR: <reason>"
    "set_orientation_here <station>"      -> "OK <pose>" | "ERROR: <reason>"
    "tweak_orientation x|y|z <degrees>"   -> "ACCEPTED" | "ERROR: <reason>"
    "tweak_position x|y|z <millimetres>"  -> "ACCEPTED" | "ERROR: <reason>"
    "put_camera_to_tcp"                   -> "ACCEPTED" | "ERROR: <reason>"
    "put_tcp_to_camera"                   -> "ACCEPTED" | "ERROR: <reason>"
    "align_camera_north +x|-x|+y|-y|<deg>"-> "ACCEPTED" | "ERROR: <reason>"
    "calibrate_venton_table"              -> "ACCEPTED" | "ERROR: <reason>"
    "calibrate_venton_level [id ...]"     -> "ACCEPTED" | "ERROR: <reason>"
    "estimate_venton_table"               -> "ACCEPTED" | "ERROR: <reason>"
    "venton_axes <w> <h> [mm]"            -> "<json pixels>" | "ERROR: <reason>"
    "measure_venton_tag <id>"             -> "ACCEPTED" | "ERROR: <reason>"
    "fit_venton_table"                    -> "<frame>" | "ERROR: <reason>"
    "clear_venton_measurements"           -> "OK ..." | "ERROR: <reason>"
    "goto_position <station>"             -> "ACCEPTED" | "ERROR: <reason>"
    "goto_transfer_point"                 -> "ACCEPTED" | "ERROR: <reason>"
    "set_transfer_point_here"             -> "OK <pose>" | "ERROR: <reason>"
    "<transport_name>"                    -> "ACCEPTED" | "ERROR: <reason>"
    "zalign"                              -> "ACCEPTED" | "ERROR: <reason>"
    "release_gripper"                     -> "ACCEPTED" | "ERROR: <reason>"
    "unlock_stop"                         -> "OK <detail>" | "ERROR: <reason>"
    "freedrive on|off"                    -> "OK <detail>" | "ERROR: <reason>"
    "motor_tweak forward|reverse <step>"  -> "ACCEPTED" | "ERROR: <reason>"
    "pump <op>"                           -> "ACCEPTED" | "ERROR: <reason>"
    "stop_pump"                           -> "OK <detail>" | "ERROR: <reason>"
    "make_sample <slot> [sample id]"      -> "ACCEPTED" | "ERROR: <reason>"
    "draw_and_load"                       -> "ACCEPTED" | "ERROR: <reason>"
    "draw_load_sample <slot> [sample id]" -> "ACCEPTED" | "ERROR: <reason>"
    "unload_sample [aspirate]"            -> "ACCEPTED" | "ERROR: <reason>"
    "set_flowcell <1|2>"                  -> "OK" | "ERROR: <reason>"
    "set_position_sample_table_motors on|off" -> "OK" | "ERROR: <reason>"
    "get_state"                           -> "<JSON snapshot>"
    "set_location <what> <value>"         -> "OK" | "ERROR: <reason>"
    "reset_carousel"                      -> "OK" | "ERROR: <reason>"
    "teach_carousel_slot <n>"             -> "OK" | "ERROR: <reason>"
    "set_sample_id <slot> <sample id>"    -> "OK" | "ERROR: <reason>"
    "get_sample_id <slot>"                -> "<sample id>" | "unknown" | "ERROR: <reason>"
    "clear_sample_id <slot>"              -> "OK" | "ERROR: <reason>"
    "clear_slots"                         -> "OK" | "ERROR: <reason>" (keeps taught reference)
    "set_credentials <base64 json>"       -> "OK" | "ERROR: <reason>"

A sample ID is the rest of the line, so it may contain spaces; runs of
whitespace in it collapse to one. Every other argument is a single token.

``set_credentials`` hands PALmixer the PVapp login a campaign already has, for
confirming a mix once ``make_sample`` finishes (see :mod:`palmixer.pvapp`). Fast,
like ``get_sample_id`` -- it takes effect on the next confirmation, not whatever
mix is already running. Base64'd for the same reason ``mount_carousel`` is: a
password may hold characters this whitespace-delimited wire can't carry raw. That
is encoding, not secrecy -- this socket is unauthenticated plain text like every
other command on it. Held in memory only; never logged, returned, or persisted.
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

# The sample seat beside the mixer cleaning station. Taught like the four
# above, but not searchable: it carries no AprilTag of its own, so it is left
# out of STATIONS (which is exactly the set locate_apriltag understands) and
# reached instead by the flowcell_to_sample_on_mixer transport, which parks the
# flowcell over it to be jogged on and recorded by hand.
STATION_SAMPLE_ON_MIXER = "sample_on_mixer_station"

# Every station with a taught position: what "go to", "teach here", and "save
# orientation" work on. A superset of STATIONS.
TAUGHT_STATIONS = STATIONS + (STATION_SAMPLE_ON_MIXER,)

STATION_LABELS = {
    STATION_SAMPLE_TABLE: "Sample Table",
    STATION_CLEANING_STATION: "Flowcell Cleaning Station",
    STATION_MIXER_STATION: "Mixer Station",
    STATION_MIXER_CLEANING_STATION: "Mixer Cleaning Station",
    STATION_SAMPLE_ON_MIXER: "Sample on Mixer Station",
}

SEARCH_APRILTAG = "search_apriltag"
STOP_SEARCH = "stop_search"

# Optional trailing flag on search_apriltag. It tells the server to keep the
# camera face-normal-down (level) after finding the tag, instead of tipping it
# face-down or squaring it to a tilted tag -- see the roll skip in
# camera_tools.search_apriltag_by_tilt. Position is recorded either way; only
# the recorded orientation differs.
SKIP_ROLL = "skip_roll"

# Optional trailing flag on search_apriltag. Search from wherever the arm is
# standing instead of driving to the station's configured reference first --
# for a station that has moved far enough that the reference no longer looks
# anywhere near its tag. Nothing is read from waypoints.ini or search_refs, so
# it also works when a stored position is wrong in a way that would send the
# arm somewhere unhelpful. Combinable with SKIP_ROLL; order does not matter.
FROM_CURRENT = "from_current"

# Swap which of the two tool frames is standing on the current pose: move the
# camera onto the gripper tip's pose, or the tip onto the camera's. Both
# translate the arm by the camera offset without reorienting it. PUT_CAMERA_TO_TCP
# is how an operator gets the camera looking down before a FROM_CURRENT search,
# which will not reorient it for them.
PUT_CAMERA_TO_TCP = "put_camera_to_tcp"
PUT_TCP_TO_CAMERA = "put_tcp_to_camera"

# Teach a station from where the robot is standing instead of by searching for
# its AprilTag -- for a station whose tag is obscured, or one being nudged off
# a position that is already nearly right. Records exactly what the search
# records (PAL12idb.record_current_position), so the two are interchangeable.
SET_POSITION_HERE = "set_position_here"

# Teach a station's orientation alone, keeping the X/Y/Z the AprilTag search
# found. For the flowcell cleaning station, where the flowcell does not sit
# level and its 12 mm tag is too small to resolve the tilt from -- position is
# measured well there, orientation is not.
#
# No GUI button sends this any more: the Configuration tab's jog panel can now
# move the position too, so saving the orientation alone would discard half of
# what was just jogged, and its save button sends SET_POSITION_HERE instead.
# Kept in the vocabulary because the operation is still a real one over ZMQ --
# re-teaching a seat angle without disturbing a position that is known good.
SET_ORIENTATION_HERE = "set_orientation_here"
TWEAK_ORIENTATION = "tweak_orientation"
ORIENTATION_AXES = ("x", "y", "z")

# The position counterpart of TWEAK_ORIENTATION: nudge the arm along a base
# axis so a taught X/Y/Z can be corrected by eye before SET_POSITION_HERE
# records it. Base frame, not tool frame, because the numbers being corrected
# are base frame -- see PAL12idb.tweak_position. There is no
# "set_position_here for one axis": position is taught whole.
TWEAK_POSITION = "tweak_position"
POSITION_AXES = ("x", "y", "z")

# Drive to a station's taught position and stop there, without picking
# anything up -- the way to check by eye what a search or a manual teach
# actually recorded (PAL12idb.goto_station).
GOTO_POSITION = "goto_position"

# Drive to the corridor waypoint every cross-cell leg routes through. A
# separate command rather than a GOTO_POSITION station, because it is not a
# taught position at all: nothing to search for, nothing in waypoints.ini, and
# so nothing for the position pre-check to refuse.
GOTO_TRANSFER_POINT = "goto_transfer_point"

# Teach the transfer point from where the arm is standing now. The transfer
# point was a fixed corridor pose; this records the current pose as it instead
# (PAL12idb.record_current_transfer_point -> waypoints.ini [transfer_point]), so
# goto_transfer_point and every cross-cell leg route through the new one. Fast
# bookkeeping like set_position_here -- no move.
SET_TRANSFER_POINT_HERE = "set_transfer_point_here"

# Measure the Venton table's surface frame in robot base coordinates by reading
# the table fiducials (apriltags.venton_table_* with id + table_xy_mm) from the
# current view. A worker action -- it moves the arm to centre on each tag. The
# reply carries the fitted frame and the per-tag residual.
CALIBRATE_VENTON_TABLE = "calibrate_venton_table"

# Move the tool TCP to an absolute base-frame x/y/z (metres), keeping the
# current orientation. A general positioning move -- e.g. to hover over the
# Venton origin a calibration just found. Worker action.
GOTO_XYZ = "goto_xyz"

# Hand-eye calibration of the camera-in-flange transform (camtcp): drive to
# varied poses, read one tag's pose at each, solve AX=XB. Optional tag id arg
# (default 13). Reports the solved camtcp for review; does not apply it.
CALIBRATE_CAMTCP = "calibrate_camtcp"

# Camera intrinsics (focal length + distortion) from ChArUco views at varied
# poses. Stored and then used by every ChArUco pose, replacing the assumed
# focal length. Run this BEFORE calibrate_camtcp. No arguments.
CALIBRATE_CAMERA_INTRINSICS = "calibrate_camera_intrinsics"

# Two-tag Venton calibration: assume the table is LEVEL (surface normal = base
# +Z) and take the in-plane heading from each tag's own measured orientation,
# instead of needing three non-collinear tag positions. No motion. Optional
# trailing tag ids restrict it to those fiducials (e.g. the two currently in
# view); with none it uses every configured Venton tag it can see. Also drops a
# red dot on the table origin in a saved camera frame for a visual check.
CALIBRATE_VENTON_LEVEL = "calibrate_venton_level"

# Quick, no-motion estimate of the same frame: read the fiducials from the
# current view and fit, without centring or descending. Carries the camera's
# mount-geometry error, so it is a rough answer (its reply says how rough);
# calibrate_venton_table is the accurate, moving version.
ESTIMATE_VENTON_TABLE = "estimate_venton_table"

# Project the last-calibrated Venton frame (origin + X/Y/Z axes) into the
# current camera view and return the pixel positions, for the GUI to draw the
# coordinate axes on the live image. Fast and no-motion: it reads the arm pose
# and projects, computing the overlay once at the moment the operator toggles it
# on. Args are the width/height of the frame being drawn on (so the camera
# matrix matches it) and an optional axis length in millimetres (default 50).
VENTON_AXES = "venton_axes"

# Spin the camera in place so its image-up (north) axis points along a named
# robot direction (+x/-x/+y/-y, or a bare azimuth in degrees; default +x) -- a
# setup step that fixes the camera's in-plane heading before a Venton read.
# Worker action: it moves the wrist.
ALIGN_CAMERA_NORTH = "align_camera_north"
CAMERA_DIRECTIONS = ("+x", "-x", "+y", "-y")

# Operator-assisted Venton calibration: the operator jogs the camera over one
# tag at a time, measure_venton_tag localizes it closed-loop (centre + descend),
# fit_venton_table fits the frame once three are collected, clear starts over.
MEASURE_VENTON_TAG = "measure_venton_tag"       # arg: tag id
FIT_VENTON_TABLE = "fit_venton_table"
CLEAR_VENTON_MEASUREMENTS = "clear_venton_measurements"

# Explicit sync between waypoints.ini (which every move reads) and the
# 12idUR:WaypointL:* EPICS PVs (the beamline-wide interchange). These are the
# only commands that do Channel Access on the waypoint PVs.
PUSH_POSITIONS = "push_positions"
PULL_POSITIONS = "pull_positions"

# The "actual transport functions" defined in PAL12idb.py.
TRANSPORT_FUNCTIONS = (
    "mixer2cleaningstation",
    "mixer2mixingstation",
    "load_flowcell_from_cleaningstation_to_beam",
    "load_flowcell_from_beam_to_cleaningstation",
    "ready_flowcell_to_draw",
    "load_sample_to_beam",
    "return_sample",
    "wash_flowcell_after_return",
    "flowcell_to_sample_on_mixer",
)

# The two transports that physically move the mixer head. Held by the server
# until a wash in progress finishes (Workflows.await_mixer_wash) rather than
# refused -- moving the head off the cleaning station mid-wash risks spilling
# the liquid actively flowing through it, damaging the tubing, or fouling the
# needle alignment, but the move itself is wanted, just not yet.
MIXER_HEAD_TRANSPORTS = ("mixer2cleaningstation", "mixer2mixingstation")

TRANSPORT_LABELS = {
    "mixer2cleaningstation": "Mixer -> Cleaning Station",
    "mixer2mixingstation": "Cleaning Station -> Mixer",
    "load_flowcell_from_cleaningstation_to_beam": "Cleaning Station -> Beam",
    "load_flowcell_from_beam_to_cleaningstation": "Beam -> Cleaning Station",
    "ready_flowcell_to_draw": "Ready Flowcell to Draw",
    "load_sample_to_beam": "Load Sample to Beam",
    "return_sample": "Return Sample from Beam",
    "wash_flowcell_after_return": "Wash Flowcell After Return",
    "flowcell_to_sample_on_mixer": "Flowcell -> Sample on Mixer (hold)",
}

# Straighten the tool's Z axis to point straight down, keeping the current
# heading and position (robUR.Zalign). A recovery action rather than part of
# any sequence: something that leaves the wrist tilted -- an aborted AprilTag
# tilt search, a manual jog on the pendant -- makes the next transport's
# vertical moves go off at an angle, and this puts it back.
ZALIGN = "zalign"

# Manual robot recovery, both on the Experiment tab's Robot panel.
#
# RELEASE_GRIPPER opens the jaws where the arm stands. A worker command like
# any other hardware action, so it is refused while one is running -- pressed
# mid-carry it would drop whatever is being carried.
#
# UNLOCK_STOP clears a protective stop (robUR.unlock_stop -> dashboard.unlock).
# Fast, and deliberately not gated on the busy flag: a protective stop happens
# *during* a move, and the action thread it interrupted may still be sitting
# there holding the server busy, which is exactly when this is needed. It
# reaches the robot over the dashboard socket rather than the motion one, so a
# stuck move does not block it either.
RELEASE_GRIPPER = "release_gripper"
UNLOCK_STOP = "unlock_stop"

# Hand-guiding (freedrive): release the arm's brakes so an operator can push it
# to a pose by hand, then re-stiffen it. A toggle on the Experiment tab's Robot
# panel. Like UNLOCK_STOP it is answered on the fast path and not gated on the
# busy flag, so turning freedrive *off* always works -- that is a recovery.
# Turning it *on* is refused while a worker action is running: letting the
# brakes go mid-move is dangerous. The controller re-stiffens on its own after a
# while (see robUR.set_freedrive), so a toggle left on fails safe.
FREEDRIVE = "freedrive"
FREEDRIVE_ON = "on"
FREEDRIVE_OFF = "off"

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

# Cycle the sample back and forth inside the in-use flowcell. On the flowcell
# server this is flow2_sample or flow3_sample -- the device is part of the
# command name there rather than an argument, so Pump picks the name.
PUMP_SHAKE_SAMPLE = "shake_sample"

PUMP_OPS = (
    PUMP_MIX,
    PUMP_CLEAN_MIXER,
    PUMP_DRAW_TO_FLOWCELL,
    PUMP_ASPIRATE_FROM_FLOWCELL,
    PUMP_WASH_FLOWCELL,
    PUMP_SHAKE_SAMPLE,
)

# The subset the Experiment tab lays out as its Pump row. Shaking lives on the
# Automation tab instead, next to the workflows it belongs with, so it is left
# out here -- the server still accepts it as an ordinary `pump <op>`.
EXPERIMENT_PUMP_OPS = (
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
    PUMP_SHAKE_SAMPLE: "Shake Sample",
}

# Stop an active shake_sample now. A command of its own rather than a pump op,
# because it is the one pump command that has to be answerable *while* a pump
# op is running -- so the server takes it on the fast path, like stop_search,
# and Pump sends it on a socket the running op is not holding. Reaches the
# flowcell server's own stop_shaking, which only interrupts an active
# flow2_sample/flow3_sample cycle -- an unrelated draw/wash/aspirate keeps
# running.
STOP_PUMP = "stop_pump"

# -- automation workflows (make a full sample / unload a sample) -----------
MAKE_SAMPLE = "make_sample"
DRAW_LOAD_SAMPLE = "draw_load_sample"
UNLOAD_SAMPLE = "unload_sample"
UNLOAD_SAMPLE_ASYNC = "unload_sample_async"

# make_sample's tail without the mixing: draw from the vial the mixer is
# already sitting over and put the flowcell in the beam. For a vial that
# already holds what is wanted -- mixed by an earlier run, or recovered by
# "unload_sample aspirate" -- so it consumes no carousel slot and assigns no
# sample ID, unlike make_sample.
DRAW_AND_LOAD = "draw_and_load"

# Optional trailing flag on unload_sample. Without it the flowcell comes off
# the beam and goes straight to its cleaning station to be washed, and the
# sample in it is discarded with the wash. With it, the sample is recovered
# first: the flowcell is carried to the mixer, its contents pushed back into
# the vial they were mixed in, and only then does it go to be washed. That
# recovery is three extra legs and a pump operation, and it is only worth
# running when the sample is wanted back, so it is off by default.
ASPIRATE = "aspirate"

# -- tracking / state commands ----------------------------------------------
SET_FLOWCELL = "set_flowcell"
GET_STATE = "get_state"
SET_LOCATION = "set_location"

# Whether the sample-table transports drive the DAQ sample-stage motors
# (sth/sav/stv) to the recorded position before approaching the sample table.
# A persisted setting, not an action -- "on" keeps the long-standing behaviour,
# "off" leaves those EPICS motors untouched so a transport runs even when the
# DAQ GUI is unreachable. Fast; reported in get_state as
# position_sample_table_motors.
SET_POSITION_SAMPLE_TABLE_MOTORS = "set_position_sample_table_motors"
TOGGLE_ON = "on"
TOGGLE_OFF = "off"

# -- carousel slot inventory -------------------------------------------------
# The carousel is a consumable: a fixed number of slots, each spent once a
# sample has been mixed in it, and the whole thing swapped out when full. Its
# geometry (size, step) is configuration, not a command -- see config.py.
TEACH_CAROUSEL_SLOT = "teach_carousel_slot"
RESET_CAROUSEL = "reset_carousel"
CLEAR_SLOTS = "clear_slots"
SET_SAMPLE_ID = "set_sample_id"
SET_SAMPLE_UID = "set_sample_uid"
GET_SAMPLE_ID = "get_sample_id"
CLEAR_SAMPLE_ID = "clear_sample_id"

# -- swapping the whole carousel ---------------------------------------------
# A batch of samples is prepared onto a carousel off-line and swapped in whole.
# One command carries the ID *and* the inventory so the swap lands as a single
# transition: a half-applied one would leave this server's idea of each slot
# disagreeing with the physical carousel, and every sample measured afterwards
# would carry the wrong ID with nothing to reveal it.
MOUNT_CAROUSEL = "mount_carousel"
GET_CAROUSEL_ID = "get_carousel_id"

# -- pump settings -----------------------------------------------------------
# Mixing speed is a property of the pump, not of a sample, so it is set and read
# like the flowcell selector rather than passed to make_sample: set it, then mix.
# That keeps make_sample's signature stable and means the GUI's own "Mix" button
# runs at the same speed the automation would.
SET_MIXING_SPEED = "set_mixing_speed"
GET_MIXING_SPEED = "get_mixing_speed"

# -- PVapp -------------------------------------------------------------------
# The sample register's login, handed over by a campaign rather than exported
# separately on this host. Only used to confirm a mix after make_sample; see
# palmixer/pvapp.py.
SET_CREDENTIALS = "set_credentials"

FLOWCELL_IDS = _state.FLOWCELL_IDS               # (1, 2)
LOCATION_KEYS = _state.LOCATION_KEYS             # mixer_head, flowcell_1, flowcell_2
MIXER_LOCATIONS = _state.MIXER_LOCATIONS
FC_LOCATIONS = _state.FC_LOCATIONS
UNKNOWN = _state.UNKNOWN
MAX_SAMPLE_ID_LEN = _state.MAX_SAMPLE_ID_LEN
WHAT_MIXER_HEAD = _state.WHAT_MIXER_HEAD
WHAT_FLOWCELL_1 = _state.WHAT_FLOWCELL_1
WHAT_FLOWCELL_2 = _state.WHAT_FLOWCELL_2


# Marker prefix on a "position not configured" ERROR reply, shared between
# server.py (which builds the message) and the GUI (which matches it to
# decide whether to pop up a dialog instead of just logging it).
POSITION_NOT_CONFIGURED = "position not configured"

# Marker prefix on the failure detail of a transport that hit a protective stop
# and could not be recovered (PAL12idb.ProtectiveStopUnrecovered). Matched by
# the GUI the same way POSITION_NOT_CONFIGURED is, but on the *async* failure
# routes -- the MQTT motion topic and the get_state last_result poll -- since a
# transport fails long after its command was answered with ACCEPTED. The robot
# has already been put somewhere safe by the time this appears; the dialog is
# to fetch the operator, not to ask them to catch anything.
PROTECTIVE_STOP_UNRECOVERED = "protective stop not recovered"


def is_protective_stop_failure(detail):
    """Whether a failure detail carries the unrecovered-protective-stop marker.

    Matched against the raw detail rather than the composed banner text, so it
    works the same on both async routes (the MQTT motion payload and the
    get_state last_result), which wrap it differently.
    """
    return PROTECTIVE_STOP_UNRECOVERED in str(detail or "")


def position_not_configured_error(missing_labels):
    """Build the ERROR detail for a transport/workflow command that needs a
    station position no one has taught yet (see PAL12idb.check_positions_defined)."""
    return "%s: %s -- use the Configuration tab to search its AprilTag first" % (
        POSITION_NOT_CONFIGURED, ", ".join(missing_labels))


def search_apriltag_command(station, skip_roll=False, from_current=False):
    """Build the wire command for searching a station's AprilTag.

    ``skip_roll`` appends the SKIP_ROLL flag, keeping the camera level (face
    straight down) instead of rolling it face-down / squaring it to the tag.
    ``from_current`` appends FROM_CURRENT, searching from where the arm stands
    rather than driving to the station's configured reference first.
    """
    parts = [SEARCH_APRILTAG, station]
    if skip_roll:
        parts.append(SKIP_ROLL)
    if from_current:
        parts.append(FROM_CURRENT)
    return " ".join(parts)


def put_camera_to_tcp_command():
    """Build the wire command to move the camera onto the gripper tip's pose."""
    return PUT_CAMERA_TO_TCP


def put_tcp_to_camera_command():
    """Build the wire command to move the gripper tip onto the camera's pose."""
    return PUT_TCP_TO_CAMERA


def stop_search_command():
    """Build the wire command to abort an in-progress AprilTag search."""
    return STOP_SEARCH


def set_position_here_command(station):
    """Build the wire command to teach ``station`` from the robot's current pose."""
    return "%s %s" % (SET_POSITION_HERE, station)


def set_orientation_here_command(station):
    """Build the wire command to give ``station`` the tool's current orientation,
    keeping its taught position."""
    return "%s %s" % (SET_ORIENTATION_HERE, station)


def tweak_orientation_command(axis, degrees):
    """Build the wire command to rotate the tool about its own ``axis``."""
    return "%s %s %s" % (TWEAK_ORIENTATION, axis, degrees)


def tweak_position_command(axis, millimetres):
    """Build the wire command to jog the arm along base-frame ``axis``.

    Millimetres on the wire, metres everywhere the robot is driven: the jog is
    an operator typing a step they can see, and 0.5 reads better than 0.0005
    both in the GUI and in the status log. PAL12idb.tweak_position takes
    metres, and server.py does the one conversion.
    """
    return "%s %s %s" % (TWEAK_POSITION, axis, millimetres)


def goto_position_command(station):
    """Build the wire command to drive to ``station``'s taught position."""
    return "%s %s" % (GOTO_POSITION, station)


def goto_transfer_point_command():
    """Build the wire command to drive to the transfer point."""
    return GOTO_TRANSFER_POINT


def set_transfer_point_here_command():
    """Build the wire command to teach the transfer point from the current pose."""
    return SET_TRANSFER_POINT_HERE


def calibrate_venton_table_command():
    """Build the wire command to calibrate the Venton table frame."""
    return CALIBRATE_VENTON_TABLE


def goto_xyz_command(x, y, z):
    """Build the wire command to move the tool TCP to absolute base x/y/z (m)."""
    return "%s %s %s %s" % (GOTO_XYZ, x, y, z)


def calibrate_camtcp_command(tag_id=13):
    """Build the wire command to run hand-eye camtcp calibration on `tag_id`."""
    return "%s %s" % (CALIBRATE_CAMTCP, tag_id)


def calibrate_camera_intrinsics_command():
    """Build the wire command to calibrate camera intrinsics from ChArUco."""
    return CALIBRATE_CAMERA_INTRINSICS


def estimate_venton_table_command():
    """Build the wire command for the quick no-motion Venton estimate."""
    return ESTIMATE_VENTON_TABLE


def venton_axes_command(width, height, axis_len_mm=None):
    """Build the wire command for the Venton axes overlay pixels in a
    ``width`` x ``height`` frame (optionally sized ``axis_len_mm`` long)."""
    if axis_len_mm is not None:
        return "%s %d %d %g" % (VENTON_AXES, int(width), int(height), axis_len_mm)
    return "%s %d %d" % (VENTON_AXES, int(width), int(height))


VENTON_HEADING_MODES = ("orientation", "positions")


def calibrate_venton_level_command(tag_ids=None, heading=None):
    """Build the wire command for the level two-tag Venton fit.

    ``tag_ids`` optionally restricts the fit to those fiducials (the two in view,
    say); omit it to use every configured Venton tag the camera can see.
    ``heading`` picks where the in-plane yaw comes from: ``"orientation"`` (each
    tag's own measured pose) or ``"positions"`` (the robust position-only fit);
    omit it for the server default.
    """
    parts = [CALIBRATE_VENTON_LEVEL]
    if heading:
        parts.append(heading)
    if tag_ids:
        parts.extend(str(int(t)) for t in tag_ids)
    return " ".join(parts)


def align_camera_north_command(direction="+x"):
    """Build the wire command to point the camera's north axis along `direction`
    (+x/-x/+y/-y, or an azimuth in degrees)."""
    return "%s %s" % (ALIGN_CAMERA_NORTH, direction)


def measure_venton_tag_command(tag_id):
    """Build the wire command to closed-loop measure one Venton fiducial."""
    return "%s %s" % (MEASURE_VENTON_TAG, tag_id)


def fit_venton_table_command():
    """Build the wire command to fit the table frame from collected tags."""
    return FIT_VENTON_TABLE


def clear_venton_measurements_command():
    """Build the wire command to discard collected Venton measurements."""
    return CLEAR_VENTON_MEASUREMENTS


def push_positions_command():
    """waypoints.ini -> EPICS waypoint PVs."""
    return PUSH_POSITIONS


def pull_positions_command():
    """EPICS waypoint PVs -> waypoints.ini."""
    return PULL_POSITIONS


def zalign_command():
    """Build the wire command to level the tool Z axis straight down."""
    return ZALIGN


def release_gripper_command():
    """Build the wire command to open the gripper where the arm stands."""
    return RELEASE_GRIPPER


def unlock_stop_command():
    """Build the wire command to clear the robot's protective stop."""
    return UNLOCK_STOP


def freedrive_command(on):
    """Build the wire command to enter (``on`` true) or leave freedrive mode."""
    return "%s %s" % (FREEDRIVE, FREEDRIVE_ON if on else FREEDRIVE_OFF)


def motor_tweak_command(direction, step):
    """Build the wire command for a motor tweak in ``direction`` by ``step``."""
    return "%s %s %s" % (MOTOR_TWEAK, direction, step)


def pump_command(op):
    """Build the wire command for a pump operation."""
    return "%s %s" % (PUMP, op)


def shake_sample_command():
    """Build the wire command to shake the sample in the in-use flowcell."""
    return pump_command(PUMP_SHAKE_SAMPLE)


def stop_pump_command():
    """Build the wire command to stop an active shake_sample."""
    return STOP_PUMP


def make_sample_command(slot, sample_id=None):
    """Build the wire command to run the full make-a-sample sequence.

    ``sample_id`` tags the slot the sample is mixed in; when it is omitted (or
    blank) the server assigns a timestamp ID, so a used slot is never
    anonymous."""
    if sample_id and str(sample_id).strip():
        return "%s %s %s" % (MAKE_SAMPLE, slot, str(sample_id).strip())
    return "%s %s" % (MAKE_SAMPLE, slot)


def draw_and_load_command():
    """Build the wire command to draw an already-mixed sample and load it."""
    return DRAW_AND_LOAD


def draw_load_sample_command(slot, sample_id=None):
    """Build the wire command to turn the carousel to ``slot``'s drawing
    position (draw_offset_steps forward of its mixing position) and draw it.

    Unlike ``draw_and_load``, which draws from wherever the carousel already
    happens to be sitting, this rotates there first -- for a specific,
    already-prepared slot rather than whatever a previous rotate left under
    the seat. ``sample_id`` is a label only: no slot is consumed and no ID is
    recorded in the carousel's inventory."""
    if sample_id and str(sample_id).strip():
        return "%s %s %s" % (DRAW_LOAD_SAMPLE, slot, str(sample_id).strip())
    return "%s %s" % (DRAW_LOAD_SAMPLE, slot)


def unload_sample_command(aspirate=False):
    """Build the wire command to run the unload-sample sequence.

    ``aspirate`` appends the ASPIRATE flag, recovering the sample into its
    vial before the flowcell is washed instead of discarding it.
    """
    if aspirate:
        return "%s %s" % (UNLOAD_SAMPLE, ASPIRATE)
    return UNLOAD_SAMPLE


def set_flowcell_command(flowcell_id):
    """Build the wire command to change which flowcell is in use."""
    return "%s %s" % (SET_FLOWCELL, flowcell_id)


def get_state_command():
    """Build the wire command to fetch the tracking state snapshot."""
    return GET_STATE


def set_position_sample_table_motors_command(on):
    """Build the wire command to turn DAQ sample-table motor positioning on/off."""
    return "%s %s" % (SET_POSITION_SAMPLE_TABLE_MOTORS,
                      TOGGLE_ON if on else TOGGLE_OFF)


def set_location_command(what, location):
    """Build the wire command to reconcile a tracked item to ``location``."""
    return "%s %s %s" % (SET_LOCATION, what, location)


def teach_carousel_slot_command(slot):
    """Build the wire command to record the current motor position as ``slot``."""
    return "%s %s" % (TEACH_CAROUSEL_SLOT, slot)


def reset_carousel_command():
    """Build the wire command to replace the carousel: drop the taught
    reference position and every sample ID."""
    return RESET_CAROUSEL


def clear_slots_command():
    """Clear all sample labels/UUIDs, preserving the taught carousel reference."""
    return CLEAR_SLOTS


def set_sample_id_command(slot, sample_id):
    """Build the wire command to tag ``slot`` with ``sample_id``, marking it used."""
    return "%s %s %s" % (SET_SAMPLE_ID, slot, str(sample_id).strip())


def get_sample_id_command(slot):
    """Build the wire command to read back ``slot``'s sample ID."""
    return "%s %s" % (GET_SAMPLE_ID, slot)


def clear_sample_id_command(slot):
    """Build the wire command to drop ``slot``'s sample ID, marking it unused."""
    return "%s %s" % (CLEAR_SAMPLE_ID, slot)


def mount_carousel_command(carousel_id, samples=None):
    """Build the wire command declaring which carousel is now in the machine.

    ``samples`` is ``{slot: sample_id}`` for everything on it. Base64'd because
    the rest of this protocol is whitespace-delimited and a sample ID may contain
    spaces -- the same reason PALsystem base64s its prep payloads.
    """
    import base64
    import json

    payload = {str(int(slot)): str(sample_id)
               for slot, sample_id in dict(samples or {}).items()}
    token = base64.b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")
    return "%s %s %s" % (MOUNT_CAROUSEL, carousel_id, token)


def decode_inventory(token):
    """The inverse. Returns ``{int slot: str sample_id}``. Raises ValueError."""
    import base64
    import json

    try:
        raw = base64.b64decode(token, validate=True)
        payload = json.loads(raw.decode("utf-8"))
    except Exception as e:
        raise ValueError("inventory is not base64'd JSON: %s" % e)
    if not isinstance(payload, dict):
        raise ValueError("inventory must be a JSON object of {slot: sample_id}")
    out = {}
    for slot, sample_id in payload.items():
        try:
            out[int(slot)] = str(sample_id)
        except (TypeError, ValueError):
            raise ValueError("inventory slot %r is not an integer" % (slot,))
    return out


def get_carousel_id_command():
    return GET_CAROUSEL_ID


def set_credentials_command(username="", password="", badge="", gup="",
                            source="campaign"):
    """Build the wire command handing PALmixer a PVapp login.

    Two shapes, matching PVapp's two routes: a ``username``/``password`` pair, or a
    ``badge``/``gup`` pair that needs no password at all on the beamline network. Send
    whichever the campaign itself is using; sending both is allowed but pointless, since
    the badge pair wins on the receiving side.

    Base64'd like ``mount_carousel``'s inventory -- not for secrecy, only because a
    password may contain characters this whitespace-delimited wire can't carry raw.

    ``source`` picks the slot: ``"campaign"`` is the run's own identity and outranks
    everything, ``"user"`` is what was typed into the GUI's User tab and is used only
    while no campaign has provided one.
    """
    import base64
    import json

    payload = {"username": str(username), "password": str(password),
               "source": str(source)}
    if badge not in (None, "") and gup not in (None, ""):
        payload["badge"], payload["gup"] = str(badge), str(gup)
    token = base64.b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")
    return "%s %s" % (SET_CREDENTIALS, token)


def decode_credentials(token):
    """The inverse. Returns ``{"username": ..., "password": ...}``. Raises ValueError.

    The error deliberately says nothing about the payload's contents: a malformed
    credential blob should not put any part of itself in a log.
    """
    import base64
    import json

    try:
        raw = base64.b64decode(token, validate=True)
        payload = json.loads(raw.decode("utf-8"))
    except Exception:
        raise ValueError("credentials are not base64'd JSON")
    if not isinstance(payload, dict):
        raise ValueError("credentials must be a JSON object")
    return payload


def set_mixing_speed_command(rpm):
    """Build the wire command setting the speed the next mix will run at."""
    return "%s %s" % (SET_MIXING_SPEED, rpm)


def get_mixing_speed_command():
    return GET_MIXING_SPEED
