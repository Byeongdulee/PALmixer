import configparser
import functools
import os
import time

import camera_tools

try:
    from . import state
except ImportError:
    import state

try:
    from . import daq_client
except ImportError:
    import daq_client

ref_mixer_cleantable = [0.4, 0.1, 0.1, 2.231, -2.212, 0]
ref_cleanstation = [0.38, -0.16, -0.1, 2.231, -2.212, 0]
ref_sampletable = [-0.22, -0.37, 0.12, -2.18860535, 2.25379435, 0]
# Intermediate pose for the sample table <-> mixer side traverse. A direct
# point-to-point move between those two regions sweeps the arm through what
# sits between them, so every such leg is routed through this pose instead
# (see move_via_transferpoint). Not a taught position: it is a fixed corridor
# waypoint, so it lives here rather than in waypoints.ini.
transfer_point = [0.25, -0.16, 0.1, 2.231, -2.212, 0]
# How close to a leg's destination the arm has to already be for that corridor
# to buy nothing. A move starting within a centimetre of where it is going is
# not crossing between the two regions, so it does not need routing; the usual
# case is the empty approach at the head of a transport, with the gripper left
# standing at the station by whatever ran before it. See already_at().
transfer_point_skip_radius = 0.01
distance_gripper_tag = 0.05
# move sth out by 50 mm
# move sav down by 30 mm
# after grap, have to move 0.20 m up to clear the needle.
sample_table = []
cleaning_station1 = []
cleaning_station2 = []
mixer_cleaning_station = []
mixer_station = []
sample_on_mixer_station = []
# Where sample_on_mixer_station sits relative to the mixer cleaning station,
# as (dX, dY, dZ) in metres. Only a starting point: it is what
# get_sample_on_mixerstation_position() returns until the station has been
# taught, so the first approach lands close enough to nudge onto the seat by
# hand rather than having to be jogged there from scratch. Once it is taught
# (Configuration tab), the taught pose is used and this is not consulted again.
#
# Measured, not estimated: the seat was found by hand at
# [0.359, -0.039, -0.142, 1.444, 2.790, 0] with the mixer cleaning station
# taught at [0.398548, 0.050382, -0.162180, 1.443748, 2.789828, 0.000389], and
# these are the differences. The two poses' orientations agree to under
# 0.0004 rad, which is why the fallback keeps the mixer cleaning station's.
sample_on_mixer_offset = (-0.039548, -0.090382, 0.020180)
needle_clear_height = 0.16
# Clearance held over the sample table on approach. Lower than
# needle_clear_height: the full lift overshoots the table, and nothing at the
# table needs that much room. Only the approach is shortened -- the lift at
# whichever station the flowcell came from still uses needle_clear_height.
sampletable_clear_height = needle_clear_height - 0.03
mixer_height = 0.05
grab_depth = 0.01
# How far above the flowcell cleaning station the arm is brought before it
# comes down onto the pose, so the final approach there is straight down
# rather than a diagonal in from wherever the arm was. See
# move_to_cleaningstation. Only this station gets the treatment: it is the one
# the arm reaches into from the side, and the deepest of them.
cleaningstation_approach_lift = 0.05
# How far to lift off the contact a bump found before opening the gripper, for
# the stations that are set down by feel rather than at a computed Z (see
# dropdown_by_bump and BUMP_RELEASE_STATIONS). Small on purpose: the point of
# feeling for the seat is to release at the seat, and anything much larger
# gives back the height the bump was there to find.
bump_release_backoff = 0.002
# How far below the taught pose the fast descent stops, handing over to the
# bump. A bump creeps -- it has to, to read contact -- so feeling out the whole
# carry clearance costs most of a minute for travel that is over known empty
# air. This closes that at ordinary speed and leaves the bump the last few
# millimetres, which is the part where the seat actually is: the release lands
# near taught_z - 0.055, so from here the bump has about 25 mm to find.
#
# Open-loop, but no more so than dropdown_at, which drives blind all the way to
# the release height; this stops 25 mm short of it.
bump_approach_clearance = 0.03
# The stations set down that way. Documentation rather than a lookup -- each
# transport passes drop_by_bump itself, since it already knows which end it is
# releasing at -- but the two places that do have to agree with each other, so
# the list of them lives here.
BUMP_RELEASE_STATIONS = ('mixer_cleaning_station', 'sample_on_mixer_station',
                         'cleaning_station')
# Physical edge length of the AprilTag each station shows the camera: the tag
# on the flowcell is 12 mm, the one on the mixer head is 16 mm. The camera
# measures distance as AT_physical_size / (tag edge in pixels) * focal length
# (urcamera.getATdistance), so this scales every distance reading linearly --
# with the robot's default 7.5 mm left in place the camera believes it is much
# closer to the tag than it is, and a descent aimed at 0.2 m stops well short.
flowcell_apriltag_size = 0.012
mixer_apriltag_size = 0.016
APRILTAG_SIZES = {
    'sample_table': flowcell_apriltag_size,
    'cleaning_station': flowcell_apriltag_size,
    'mixer_station': mixer_apriltag_size,
    'mixer_cleaning_station': mixer_apriltag_size,
}
# Camera-to-tag distance the close-range pass of locate_apriltag works from.
apriltag_view_distance = 0.2
# Recovery for a tag that drops out of view partway through a teach, after the
# search has already found it: creep the camera down a step at a time and look
# again, rather than abandoning a configuration that was nearly done. Down,
# because it grows the tag in frame -- the readings that go missing are the
# ones taken from too far off for the tag to resolve. The travel cap is what
# keeps this a nudge: 0.01 m cannot reach anything from a standoff measured in
# tenths of a metre, so a tag that is gone for some other reason gives up here
# instead of walking the camera into the station.
apriltag_reacquire_step = 0.001
apriltag_reacquire_max_travel = 0.01
# Seconds to let the camera sharpen after one of those steps before asking it
# to read a tag. The arm's camera is the robot's own IP camera (HTTP :4242),
# whose focus cannot be driven from here -- urcamera's focus()/scanfocus() are
# USB-only and raise NoUSBCameraException on it -- so waiting one out is all
# there is; see wait_for_camera_focus().
#
# Both of these are per step, and a tag that is really gone pays them ten
# times over before the travel cap stops it, so neither is the 5 s that
# _detect_apriltag waits by default: that would be nearly two minutes of an
# operator's teach spent on a tag that was not coming back. A 1 mm move is
# small enough that the camera has little to re-converge on, and the look
# after it starts from an image already judged sharp.
camera_focus_timeout = 2.0
apriltag_reacquire_settle = 2.0
# Stations whose tag is not lying flat, so the camera has to be squared to the
# tag's own normal rather than tipped face-down: the flowcell does not sit
# level in its cleaning station, so its tag comes up at an angle. For these the
# search skips Zalign() as well -- levelling the tool first would square it to
# a surface the tag is not parallel to. See camera_tools.search_apriltag_by_tilt.
APRILTAG_NOT_FLAT = ('cleaning_station',)
# Speed and acceleration for setting the flowcell down on the sample stage:
# half of what every other move runs at. Imported from robUR rather than
# hard-coded to 0.05, so this stays half of whatever the default becomes.
try:
    from common.robUR import DEFAULT_ACCEL, DEFAULT_SPEED
except ImportError:
    # robUR is only importable where UR_12idb is on the path -- the server adds
    # it before importing this module, but the GUI-side imports of the shared
    # constants below do not need the robot driver.
    DEFAULT_ACCEL = DEFAULT_SPEED = 0.1
placement_speed = DEFAULT_SPEED / 2
placement_accel = DEFAULT_ACCEL / 2
flowcell_ID = state.get_flowcell_in_use()
from . import capath
capath.ensure()                 # must precede the epics import: see capath.py
from epics import caget, caput
sampletableID = 1
cleaningstationID1 = 2
cleaningstationID2 = 3
mixerstationID = 4
mixer_cleaningstationID = 5
sample_on_mixerstationID = 6


def set_flowcell_ID(n):
    """Change which flowcell (1 or 2) the transport functions act on."""
    global flowcell_ID
    flowcell_ID = int(n)
    state.set_flowcell_in_use(flowcell_ID)

# Field names as the 12idUR IOC actually spells them: the rotations are mixed
# case. Uppercase RX/RY/RZ named PVs that do not exist, so every push reported
# "PV not writable" and every pull "no value from PV" for those three. Matches
# UR_12idb's own copy of this file (nmarks, 6546228). These strings are also
# the waypoints.ini section keys, and configparser lowercases those on write,
# so the ini is unaffected by the case change.
_POSITION_FIELDS = ('X', 'Y', 'Z', 'Rx', 'Ry', 'Rz')

# waypoints.ini, next to this file, is the runtime source of truth for taught
# positions: get_position()/set_position() touch the file and nothing else.
# That keeps Channel Access off the control path entirely -- a caget on a
# disconnected PV blocks for seconds, and get_position() is reached from
# check_positions_defined() on the synchronous ZMQ reply path, where a stall
# would time the client out (or worse, let a command be started after the
# client had already given up on it).
#
# The 12idUR:WaypointL:<ID>:<field> PVs are an interchange with the rest of
# the beamline rather than a store this package reads: sync them explicitly
# with push_positions_to_pvs() / pull_positions_from_pvs().
_INI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'waypoints.ini')

_ID_NAMES = {
    sampletableID: 'sample_table',
    cleaningstationID1: 'cleaning_station_1',
    cleaningstationID2: 'cleaning_station_2',
    mixerstationID: 'mixer_station',
    mixer_cleaningstationID: 'mixer_cleaning_station',
    sample_on_mixerstationID: 'sample_on_mixer_station',
}


def _ini_section(ID):
    return _ID_NAMES.get(ID, 'waypoint_%s' % ID)


def _read_ini_position(ID):
    # Returns the cached position for ID, or None if there is no usable entry.
    parser = configparser.ConfigParser()
    parser.read(_INI_PATH)
    section = _ini_section(ID)
    if not parser.has_section(section):
        return None
    try:
        return [parser.getfloat(section, f) for f in _POSITION_FIELDS]
    except (ValueError, configparser.NoOptionError):
        return None


# Marks an ini entry this package computed from another station rather than
# one an operator taught, so a later re-teach of that other station can refresh
# it without ever overwriting a hand-tuned position. Only the value of
# _write_ini_position's derived_from argument ever sets it, and any write
# without that argument -- a manual teach, an EPICS pull -- clears it, which is
# exactly right: both of those make the entry the operator's, not ours.
_DERIVED_FROM_KEY = 'derived_from'


def _write_ini_position(ID, pos, derived_from = None):
    # Merge into the existing file rather than overwriting it, so other
    # stations' cached positions survive.
    parser = configparser.ConfigParser()
    parser.read(_INI_PATH)
    section = _ini_section(ID)
    if not parser.has_section(section):
        parser.add_section(section)
    for f, v in zip(_POSITION_FIELDS, pos):
        parser.set(section, f, repr(float(v)))
    if derived_from:
        parser.set(section, _DERIVED_FROM_KEY, str(derived_from))
    else:
        parser.remove_option(section, _DERIVED_FROM_KEY)
    with open(_INI_PATH, 'w') as fp:
        parser.write(fp)


def _derived_from(ID):
    """Which station's position `ID`'s entry was computed from, or None.

    None covers both "taught by hand" and "no entry at all": neither is
    something this package may recompute.
    """
    parser = configparser.ConfigParser()
    parser.read(_INI_PATH)
    section = _ini_section(ID)
    if not parser.has_section(section):
        return None
    return parser.get(section, _DERIVED_FROM_KEY, fallback=None) or None


def _pv_name(ID, field):
    return f'12idUR:WaypointL:{ID}:{field}'


def get_position(ID):
    """The taught position for waypoint `ID`, read from waypoints.ini.

    Raises RuntimeError if the station has never been taught. Does no Channel
    Access by design (see the note on _INI_PATH); to bring in a position
    taught elsewhere, run pull_positions_from_pvs() first.
    """
    pos = _read_ini_position(ID)
    if pos is None:
        raise RuntimeError(
            f'waypoint {ID} ({_ini_section(ID)}) has no position in {_INI_PATH}; '
            f'search its AprilTag to teach it, or pull it from EPICS')
    return pos

def set_position(ID, pos):
    """Record a taught position in waypoints.ini.

    Does not write the EPICS PVs -- push_positions_to_pvs() does that, so
    publishing to the rest of the beamline stays an explicit operator action.
    """
    _write_ini_position(ID, pos)


# -- EPICS waypoint sync -------------------------------------------------------
# The only two functions here that do Channel Access on the waypoint PVs.
# Both are operator-initiated (Configuration tab), never part of a move.
WAYPOINT_IDS = (sampletableID, cleaningstationID1, cleaningstationID2,
                mixerstationID, mixer_cleaningstationID, sample_on_mixerstationID)


def _sync_summary(verb, done, skipped, failed, failed_label):
    """Report what a push/pull actually managed, and raise only if it managed
    nothing at all.

    A partial result is normal, not an error: on a fresh install most
    waypoints have never been taught on either side. Only a sync where every
    single station failed points at something the operator must fix (the IOC
    being down, typically), so that is the case worth raising on.
    """
    parts = ["%s %d/%d (%s)" % (verb, len(done), len(WAYPOINT_IDS),
                                 ", ".join(done) if done else "none")]
    if skipped:
        parts.append("not taught yet: %s" % ", ".join(skipped))
    if failed:
        parts.append("%s: %s" % (failed_label, ", ".join(failed)))
    summary = "; ".join(parts)
    if failed and not done:
        raise RuntimeError(summary)
    return summary


def push_positions_to_pvs():
    """Copy every taught position in waypoints.ini out to its EPICS PVs.

    Stations with no ini entry are skipped -- there is nothing to push, and
    that is the normal state of a fresh install rather than an error.
    Returns a summary naming any station whose PVs could not be written;
    raises RuntimeError only if every station failed.
    """
    pushed, skipped, failed = [], [], []
    for ID in WAYPOINT_IDS:
        name = _ini_section(ID)
        pos = _read_ini_position(ID)
        if pos is None:
            skipped.append(name)
            continue
        # Attempt every field before judging: caput() returns None for a PV it
        # could not reach, and short-circuiting would leave a half-written set.
        results = [caput(_pv_name(ID, f), v) for f, v in zip(_POSITION_FIELDS, pos)]
        (pushed if all(r is not None for r in results) else failed).append(name)
    return _sync_summary("pushed", pushed, skipped, failed, "PV not writable")


def pull_positions_from_pvs():
    """Import positions from the EPICS waypoint PVs into waypoints.ini.

    A station is only written when all six of its fields read back, so a
    partial EPICS outage cannot overwrite a good taught position with junk.
    caget() returns None both for a PV that is unreachable and for one that
    was never populated, so those two cases share a bucket in the summary.
    Raises RuntimeError only if no station read back at all.
    """
    pulled, failed = [], []
    for ID in WAYPOINT_IDS:
        name = _ini_section(ID)
        pos = [caget(_pv_name(ID, f)) for f in _POSITION_FIELDS]
        if any(v is None for v in pos):
            failed.append(name)
            continue
        _write_ini_position(ID, pos)
        pulled.append(name)
    # A pull can move the station the sample seat is measured from. If the PVs
    # also carried a seat position, the write above already took it and cleared
    # its derived marker, so this leaves it alone; if they did not, the seat is
    # still ours and is brought back into agreement with what was just pulled.
    refresh_sample_on_mixerstation_position()
    return _sync_summary("pulled", pulled, (), failed, "no value from PV")

def get_sampletable_position():
    global sample_table, sampletableID
    sample_table = get_position(sampletableID)
    return sample_table

def set_sampletable_position(pos):
    global sample_table,sampletableID
    sample_table = pos
    set_position(sampletableID, pos)

def get_cleaningstation_position(ID):
    global cleaning_station1, cleaning_station2
    if ID == 1:
        cleaning_station1 = get_position(cleaningstationID1)
        return cleaning_station1
    else:
        cleaning_station2 = get_position(cleaningstationID2)
        return cleaning_station2

def set_cleaningstation_position(ID, pos):
    global cleaning_station1, cleaning_station2
    if ID == 1:
        cleaning_station1 = pos
        set_position(cleaningstationID1, pos)
    else:
        cleaning_station2 = pos
        set_position(cleaningstationID2, pos)

def get_mixerstation_position():
    global mixer_station
    mixer_station = get_position(mixerstationID)
    return mixer_station

def set_mixerstation_position(pos):
    global mixer_station
    mixer_station = pos
    set_position(mixerstationID, pos)

def get_mixer_cleaningstation_position():
    global mixer_cleaning_station
    mixer_cleaning_station = get_position(mixer_cleaningstationID)
    return mixer_cleaning_station

def set_mixer_cleaningstation_position(pos):
    global mixer_cleaning_station
    mixer_cleaning_station = pos
    set_position(mixer_cleaningstationID, pos)
    # The sample seat is measured from this station, so teaching this one is
    # what defines it -- and what redefines it when this is taught again.
    refresh_sample_on_mixerstation_position()

def _sample_on_mixer_from(mixer_cleaning_pos):
    """The sample seat's pose, computed from the mixer cleaning station's.

    The offset is position-only: the two seats are the same piece of hardware
    a few centimetres apart, and the measurement this offset came from found
    their orientations agreeing to under 0.0004 rad, so the orientation is
    carried across unchanged.
    """
    pos = list(mixer_cleaning_pos)
    for i, d in enumerate(sample_on_mixer_offset):
        pos[i] = pos[i] + d
    return pos

def refresh_sample_on_mixerstation_position():
    """Define the sample seat from the mixer cleaning station, if it is ours
    to define. Returns the pose now in effect, or None if there is none yet.

    Called whenever the mixer cleaning station is taught, so the seat exists
    from the moment the station it hangs off does and follows it if that
    station is ever re-taught. A position an operator taught by hand is left
    strictly alone -- that is the point of the derived_from marker: without it
    there would be no way to tell a value this wrote last time from one
    somebody spent a jogging session on, and re-searching the mixer cleaning
    station would quietly throw the latter away.
    """
    try:
        base = get_mixer_cleaningstation_position()
    except RuntimeError:
        return None                      # nothing to derive from yet
    existing = _read_ini_position(sample_on_mixerstationID)
    if existing is not None and _derived_from(sample_on_mixerstationID) is None:
        return existing                  # taught by hand; not ours to touch
    global sample_on_mixer_station
    sample_on_mixer_station = _sample_on_mixer_from(base)
    _write_ini_position(sample_on_mixerstationID, sample_on_mixer_station,
                        derived_from=_ini_section(mixer_cleaningstationID))
    return sample_on_mixer_station

def get_sample_on_mixerstation_position():
    """The pose of the sample seat on the mixer station.

    Normally just the ini entry, like every other station: it is written the
    moment the mixer cleaning station is taught (see
    refresh_sample_on_mixerstation_position). The computed fallback below is
    for the one case that misses -- an installation whose mixer cleaning
    station was taught before this station existed -- so the seat is usable
    there without making the operator re-search a station that is already
    right.

    Raises RuntimeError (from get_mixer_cleaningstation_position) if neither
    this station nor the one it is measured from has ever been taught.
    """
    global sample_on_mixer_station
    pos = _read_ini_position(sample_on_mixerstationID)
    if pos is None:
        pos = _sample_on_mixer_from(get_mixer_cleaningstation_position())
    sample_on_mixer_station = pos
    return pos

def set_sample_on_mixerstation_position(pos):
    # No derived_from: this is the manual teach, and writing without the
    # marker is what makes the entry the operator's, so a later re-teach of
    # the mixer cleaning station leaves it alone.
    global sample_on_mixer_station
    sample_on_mixer_station = pos
    set_position(sample_on_mixerstationID, pos)

# -- position readiness checks -----------------------------------------------
# Station keys match commands.TAUGHT_STATIONS / commands.STATION_LABELS, so callers
# on the server side can report a missing position using the same label the
# Configuration tab's AprilTag-search buttons use.
_STATION_IDS = {
    'sample_table': lambda: sampletableID,
    'mixer_station': lambda: mixerstationID,
    'mixer_cleaning_station': lambda: mixer_cleaningstationID,
    'sample_on_mixer_station': lambda: sample_on_mixerstationID,
    # Resolved at call time, like the transport functions do: which physical
    # cleaning station this means depends on the flowcell currently in use.
    'cleaning_station': lambda: cleaningstationID1 if flowcell_ID == 1 else cleaningstationID2,
}

# A station that can be reached without having been taught, and the station it
# is derived from when it has not been. Only sample_on_mixer_station has one
# (see get_sample_on_mixerstation_position), and it is here rather than in that
# function so check_positions_defined() gives the same answer the move will:
# without it, "go to the sample seat" would be refused as unconfigured on the
# very first press, which is the press that exists to get the arm close enough
# to teach it.
_STATION_FALLBACKS = {
    'sample_on_mixer_station': 'mixer_cleaning_station',
}

# Which station positions each transport function reads via get_position().
# workflows.py composes its own requirements from this rather than keeping a
# second list, so the two cannot drift apart.
TRANSPORT_STATIONS = {
    'mixer2cleaningstation': ('mixer_station', 'mixer_cleaning_station'),
    'mixer2mixingstation': ('mixer_cleaning_station', 'mixer_station'),
    'load_flowcell_from_cleaningstation_to_beam': ('cleaning_station', 'sample_table'),
    'load_flowcell_from_beam_to_cleaningstation': ('sample_table', 'cleaning_station'),
    'ready_flowcell_to_draw': ('cleaning_station', 'sample_on_mixer_station'),
    'load_sample_to_beam': ('sample_table',),
    'return_sample': ('sample_table', 'sample_on_mixer_station'),
    'wash_flowcell_after_return': ('cleaning_station',),
    'flowcell_to_sample_on_mixer': ('cleaning_station', 'sample_on_mixer_station'),
}


def check_positions_defined(station_keys):
    """Which of `station_keys` have not been taught yet (no waypoints.ini entry).

    Returns a list of station-key strings (empty if all are configured). A
    plain file read, with no Channel Access and no side effects, so it is
    cheap enough to run on the synchronous ZMQ reply path before accepting a
    command. An unrecognized station key is a caller bug and raises KeyError
    rather than being quietly reported as unconfigured."""
    missing = []
    for key in dict.fromkeys(station_keys):  # de-dup, keep first-seen order
        if _read_ini_position(_STATION_IDS[key]()) is not None:
            continue
        # A station with a fallback counts as configured as long as the station
        # it is derived from is, since that is what the move will actually use.
        # When neither is taught the fallback's name is the one reported: it is
        # the position the operator has to go and teach.
        fallback = _STATION_FALLBACKS.get(key)
        if fallback is None:
            missing.append(key)
        elif _read_ini_position(_STATION_IDS[fallback]()) is None:
            missing.append(fallback)
    return missing

# Basic operation functions
# Which robot object the gripper has already been activated on. Keyed on the
# object rather than a plain bool so that a reconnect -- which hands out a new
# robot object talking to a freshly powered gripper -- activates again instead
# of the flag from the previous connection suppressing it.
_gripper_activated_on = None


def activate_gripper(robot):
    """Activate the gripper, unless it has already been activated on `robot`.

    Activation is a slow handshake and only has to happen once, so every
    transport routine calls this instead of robot.activate_gripper() directly.
    Call reset_gripper_activation() to force the next call to run it again.
    """
    global _gripper_activated_on
    if _gripper_activated_on is robot:
        return
    robot.activate_gripper()
    _gripper_activated_on = robot


def reset_gripper_activation():
    """Forget that the gripper was activated, so the next call activates it."""
    global _gripper_activated_on
    _gripper_activated_on = None


def pickup(robot, height = needle_clear_height):
    robot.release()
    # cm go deeper from the standard height
    robot.mvr2z(-grab_depth-distance_gripper_tag)
    robot.grab()
    # move high enough so the needle is cleared
    robot.mvr2z(height)

def dropdown_at(robot, station_pos, acc=None, vel=None):
    """Descend to `station_pos`'s release height and let go.

    The release Z is 0.005 m above where pickup() grabs -- pickup() descends
    distance_gripper_tag + grab_depth from the taught pose, so this is
    (taught Z - distance_gripper_tag) - grab_depth + 0.005.

    The full taught pose is sent rather than a relative Z move, so where the
    flowcell is set down does not depend on the clearance it was carried in
    at. No bump: feeling for contact was releasing high, because anything the
    gripper brushes on the way -- a lip on the station, a slightly cocked
    flowcell, the arm's own deceleration -- reads as arrival.
    """
    p = list(station_pos)
    p[2] = p[2]-distance_gripper_tag-grab_depth+0.005
    if acc is None or vel is None:
        robot.moveto(p)
    else:
        robot.moveto(p, acc=acc, vel=vel)
    robot.release()
    # move back up to standard height
    robot.mvr2z(distance_gripper_tag)

def bump_approach(robot, station_pos, clearance = bump_approach_clearance):
    """Drop at ordinary speed to `clearance` below `station_pos`, ready to bump.

    Returns how far it moved (0.0 if it did not). Only ever moves down, and
    only as far as a height computed from the taught pose -- called with the
    arm already at or below that height it does nothing, so it cannot lift a
    piece that is on its way into a seat.

    A pose that cannot be read means no move: the bump then feels out the whole
    distance, which is slow but correct. That is the right way to fail, since
    the alternative is a blind relative move computed from a Z nobody knows.
    """
    try:
        now_z = float(robot.get_pos()[2])
    except Exception:
        return 0.0
    dz = (float(station_pos[2]) - clearance) - now_z
    if dz >= 0:
        return 0.0
    robot.mvr2z(dz)
    return dz


def dropdown_by_bump(robot, station_pos = None, backoff = bump_release_backoff,
                     lift = distance_gripper_tag):
    """Descend until something is felt, release there, and lift back off.

    Given the destination's taught pose, the descent is done in two parts: an
    ordinary move down to bump_approach_clearance below it, then the bump for
    the rest (see bump_approach). Without one the bump does the whole descent,
    which is correct but slow.

    The alternative to dropdown_at()'s computed Z, for the seats listed in
    BUMP_RELEASE_STATIONS: the two on the mixer side and the flowcell cleaning
    station. All three are deep sockets whose Z depends on how the piece is
    sitting in the gripper, so a release aimed at a fixed height either leaves
    the piece hanging or drives the gripper into the socket; feeling for the
    bottom lands on it either way.

    That is the opposite trade from the sample table, which is why this is a
    separate function rather than a change to dropdown_at(): out at the table
    a bump reads contact off anything the gripper brushes on the way in, and
    releasing high there drops the flowcell onto the stage. What makes the
    difference is the approach -- these three are come down on vertically, from
    directly overhead, so there is nothing to brush on the way (see
    move_to_cleaningstation for the flowcell station's).

    `backoff` is how far up the bump itself retreats after touching, so the
    gripper is not pressing down when it opens; `lift` is the clearance the
    arm ends at, matching what dropdown()/dropdown_at() leave.
    """
    if station_pos is not None:
        bump_approach(robot, station_pos)
    robot.bump(z=-1, backoff=backoff)
    robot.release()
    robot.mvr2z(lift)

def dropdown(robot, height = needle_clear_height, station_pos = None,
             by_bump = False):
    # Release onto a station. Given the destination's taught pose, this
    # descends to a computed Z (dropdown_at); without one it falls back to a
    # relative move down from wherever the caller left the arm, which is only
    # right if that was the travel height `height` describes.
    #
    # by_bump feels for the seat instead: the taught Z decides only where the
    # fast part of the descent stops, not where the gripper opens.
    if by_bump:
        return dropdown_by_bump(robot, station_pos)
    if station_pos is not None:
        return dropdown_at(robot, station_pos)
    # move down
    robot.mvr2z(-1*(height+grab_depth-0.005))
    robot.release()
    # move back up to standard height
    robot.mvr2z(distance_gripper_tag)

def dropdown_sampletable(robot):
    # The sample table drop, at half speed: it is the one placement onto the
    # beamline stage, open-loop, so anything the taught position is off by is
    # taken up by the hardware. Halving the approach is cheap -- a 0.06 m move
    # once per sample.
    return dropdown_at(robot, get_sampletable_position(),
                       acc=placement_accel, vel=placement_speed)

def transport2sampletable(robot, p1, height = needle_clear_height,
                          via_transferpoint = False, pickup_from_above = False):
    # transport() specialized to the sample table as the destination: the drop
    # is dropdown_sampletable() rather than the bump-for-contact dropdown(),
    # and the destination comes from the taught sample table position rather
    # than from the caller. The robot is assumed to be empty.
    #
    # via_transferpoint routes both legs -- the empty approach to p1 and the
    # carry to the table -- through transfer_point; see _move_leg.
    # pickup_from_above drops onto p1 vertically at the end of the first leg
    # instead of arriving on it directly; see move_to_cleaningstation, which is
    # the only p1 that wants it.
    activate_gripper(robot)
    if pickup_from_above:
        move_to_cleaningstation(robot, p1, via_transferpoint)
    else:
        _move_leg(robot, p1, via_transferpoint)
    pickup(robot, height=height)
    p2 = list(get_sampletable_position())
    p2[2] = p2[2]-distance_gripper_tag+sampletable_clear_height
    _move_leg(robot, p2, via_transferpoint)
    dropdown_sampletable(robot)

def test_pickup(robot, height=needle_clear_height):
    pickup(robot, height=height)
    dropdown(robot, height=height)

def test_mixer_pickup(robot):
    test_pickup(robot, height=mixer_height)

def goto_default(robot):
    # ref_mixer_cleantable is on the mixer side, so this is a cross-cell move
    # from anywhere at the sample table and goes through the corridor. Sent as
    # a copy, so the robot API cannot write back into the module constant.
    move_via_transferpoint(robot, list(ref_mixer_cleantable))

def already_at(robot, target, radius = transfer_point_skip_radius):
    """Is the TCP already within `radius` m of `target`'s X/Y/Z?

    Position only. An orientation that still has to change is taken up by the
    move itself, and the corridor is about the ground the arm covers, not how
    the tool is turned. Reads get_pos() -- the actual TCP position, the same
    source return_sample() compares against a taught pose.

    Anything unreadable answers False, so a pose that cannot be fetched costs
    the detour rather than skipping one that was needed. That is also what
    happens when the camera TCP is still active: the reading is then offset by
    the camera mount and simply fails to match, which is the harmless direction
    to be wrong in.
    """
    try:
        now = robot.get_pos()
        d = sum((float(now[i]) - float(target[i])) ** 2 for i in range(3)) ** 0.5
    except Exception:
        return False
    return d <= radius

def move_via_transferpoint(robot, target):
    # Move to `target` by way of transfer_point. Used for every leg that
    # crosses between the sample table and the rest of the cell -- the mixer,
    # the mixer cleaning station, or the flowcell cleaning station -- in either
    # direction and whether or not the gripper is holding anything.
    # transfer_point is sent as a copy so a caller cannot mutate the module
    # constant through the robot API.
    #
    # Unless the arm is standing at `target` already: then it is not crossing
    # between the two regions at all, and going out to the corridor and back
    # is travel for its own sake. The final moveto still runs -- there can be
    # a centimetre and an orientation left to take up -- it just goes direct.
    if not already_at(robot, target):
        robot.moveto(list(transfer_point))
    robot.moveto(target)

def _move_leg(robot, target, via_transferpoint):
    # One leg of a transport, routed through transfer_point or not. Both legs
    # of a transport get the same treatment: the routing is a property of the
    # two stations involved, and a standalone command can be run with the arm
    # parked anywhere, so the empty approach has to clear the same obstacles
    # the carry does -- except when the arm turns out to be at the leg's
    # destination already, which move_via_transferpoint checks for.
    if via_transferpoint:
        move_via_transferpoint(robot, target)
    else:
        robot.moveto(target)

def move_to_cleaningstation(robot, cleaning_station, via_transferpoint = True):
    """Arrive over the flowcell cleaning station and come straight down onto it.

    Every other station is arrived at by sending the taught pose and letting
    the controller take whatever straight line it likes to get there. That
    line comes in at an angle -- the corridor sits ~0.1 m up and the flowcell
    cleaning station ~0.23 m down -- so the gripper closes on the station
    sideways, sweeping the last stretch through the space the flowcell and its
    fittings occupy. Splitting it in two makes the final approach purely
    vertical: across at cleaningstation_approach_lift above the pose, then
    down the standoff and nothing else.

    Only the descent is new; the leg that gets above the station is the
    ordinary one and is still routed through the corridor unless the caller
    says otherwise. Standing on the station already, this is just the vertical
    leg -- going out to the corridor and back to reach a pose the arm is on
    would be travel for its own sake, the same judgement move_via_transferpoint
    makes.
    """
    target = list(cleaning_station)
    if not already_at(robot, target):
        above = list(target)
        above[2] = above[2] + cleaningstation_approach_lift
        _move_leg(robot, above, via_transferpoint)
    robot.moveto(target)

def transport(robot, p1, p2, height = needle_clear_height, drop_height = None,
              via_transferpoint = False, drop_by_bump = False):
    # picking up the flowcell at p1 and dropping it at p2. The robot is assumed to be empty.
    # assuming robot is empty
    # `drop_height` is the clearance held over p2, and defaults to `height`.
    # It is separate so a station that wants a lower approach (the sample
    # table) can have one without also shortening the lift at p1, which has to
    # stay tall enough to clear the needle.
    #
    # `drop_by_bump` sets down by feel rather than at p2's computed release Z;
    # see dropdown_by_bump. The approach above p2 is unchanged either way -- it
    # is only the last few millimetres that differ.
    if drop_height is None:
        drop_height = height
    activate_gripper(robot)
    _move_leg(robot, p1, via_transferpoint)
    # `height` has to reach pickup()/dropdown() too, not just the travel Z
    # below: the mixer head only needs to clear its post by mixer_height, and
    # lifting it the full needle_clear_height instead swings it far higher
    # than the move requires.
    pickup(robot, height=height)
    # Z position should be the needle cleared position. Work on a copy: writing
    # p2[2] in place edits the caller's list, so a taught position passed in
    # (sample_table / cleaning_station) would creep upward on every transport.
    p2 = list(p2)
    destination = list(p2)          # the taught pose, before the clearance
    p2[2] = p2[2]-distance_gripper_tag+drop_height
    _move_leg(robot, p2, via_transferpoint)
    dropdown(robot, height=drop_height, station_pos=destination,
             by_bump=drop_by_bump)
# State tracking. Every transport function below is wrapped with @_tracks so
# that wherever it is called from -- a workflow or a single button on the
# Experiment tab -- the tracked location is updated the same way. The state
# is recorded only after the motion returns; if it raises, the tracked item
# is set to state.UNKNOWN instead, since a motion that failed partway leaves
# the hardware in a position nobody actually knows.
def _tracks(get_setter, value):
    """Decorator: record `value` via get_setter()'s setter on success, else UNKNOWN.

    `get_setter` is called at call-time, not decoration time, so it can look
    at module state (like the current flowcell_ID) as of the moment the
    transport function actually runs.
    """
    def decorator(fn):
        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            setter = get_setter()
            try:
                result = fn(*args, **kwargs)
            except Exception:
                setter(state.UNKNOWN)
                raise
            setter(value)
            return result
        return wrapper
    return decorator


def _flowcell_setter():
    fc = flowcell_ID
    return lambda location: state.set_flowcell_location(fc, location)


def _restore_stage(restore_to, what):
    """Put the DAQ sample-stage motors back where they were before we aligned.

    align_sample_table() drives the stage to where the robot's sample_table
    waypoint was taught, because that is where the gripper knows how to reach.
    That is a handoff position, not a measuring position -- leaving the stage
    parked there means the next acquisition looks at wherever the robot likes
    rather than wherever the beam was set up to look, and the data comes back
    perfectly plausible and wrong.

    Best-effort, and deliberately so: every caller is wrapped in @_tracks, which
    records the flowcell's location as UNKNOWN if the function raises. By the
    time this runs the robot has already done its physical work and the flowcell
    really is where it was put, so raising here would condemn a cell that is
    exactly where it should be and block everything after it. The warning is
    loud instead, and it says what is wrong with the data rather than only what
    failed -- if the DAQ GUI were unreachable, align_sample_table() at the top of
    the same function would already have raised before anything moved.
    """
    if restore_to is None:
        return
    try:
        daq_client.set_pos(restore_to)
    except Exception as e:
        print('PAL12idb: could not restore the DAQ sample-table motors after %s: '
              '%s\n  The stage is still at the robot handoff position, so anything '
              'measured now is at the wrong place. Put %s back by hand before '
              'acquiring.' % (what, e, dict(zip(daq_client.MOTORS, restore_to))))


# Actual transport functions. These functions are used to move the flowcell between the cleaning station, mixer station, and sample table.
# mixer head to its cleaning station. The robot is assumed to be empty.
@_tracks(lambda: state.set_mixer_head, state.MIXER_AT_CLEANING)
def mixer2cleaningstation(robot):
    # Set down by feel: the mixer cleaning station is one of the two seats a
    # computed release Z does not suit (BUMP_RELEASE_STATIONS).
    transport(robot, get_mixerstation_position(), get_mixer_cleaningstation_position(),
              height=mixer_height, drop_by_bump=True)

# mixer head to the mixer station. The robot is assumed to be empty.
@_tracks(lambda: state.set_mixer_head, state.MIXER_AT_MIXER)
def mixer2mixingstation(robot):
    transport(robot, get_mixer_cleaningstation_position(), get_mixerstation_position(), height=mixer_height)

# Bring the flowcell parked at the cleaning station to the beam, ready for data collection. The robot is assumed to be empty.
# This is for measuring water background. The flowcell is not loaded with sample.
@_tracks(_flowcell_setter, state.FC_AT_BEAM)
def load_flowcell_from_cleaningstation_to_beam(robot):
    # Captured before align_sample_table() moves anything, so the stage can go
    # back to the measuring position once the flowcell is seated -- see
    # _restore_stage(). Read first for the same reason as in
    # load_flowcell_from_beam_to_cleaningstation: it fails here, before either
    # the stage or the robot has moved, if the DAQ GUI cannot be reached.
    restore_to = daq_client.get_pos()
    # Bring the DAQ sample stage back to where it was when the robot's
    # sample_table waypoint was taught, before the robot approaches it --
    # see daq_client.align_sample_table(). Raises before anything moves if
    # that has never been recorded, rather than placing the flowcell at a
    # sample table the beam is not actually looking at.
    daq_client.align_sample_table()
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # cleaning station -> sample table: crosses between the two regions.
    transport2sampletable(robot, cleaning_station, height=needle_clear_height,
                          via_transferpoint=True, pickup_from_above=True)
    # The flowcell is seated and the gripper is clear, so the stage is free to
    # go back to where the beam is set up to look -- and it has to happen here,
    # before the background acquisition that follows this call.
    _restore_stage(restore_to, 'loading the flowcell to the beam')

# Bring the flowcell from the beam to the cleaning station.
@_tracks(_flowcell_setter, state.FC_AT_CLEANING)
def load_flowcell_from_beam_to_cleaningstation(robot):
    # Captured before align_sample_table() moves anything, so the DAQ stage
    # can be put back once the flowcell has actually left the sample table --
    # see the restore at the end of this function. Raises here (before either
    # the stage or the robot has moved) if the DAQ GUI cannot be reached,
    # which is also what align_sample_table() is about to need.
    restore_to = daq_client.get_pos()
    # Same alignment, on the way off the sample table: see
    # daq_client.align_sample_table().
    daq_client.align_sample_table()
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # sample table -> cleaning station: crosses between the two regions.
    # Set down by feel, like the two mixer-side seats (BUMP_RELEASE_STATIONS):
    # how deep the flowcell sits in its station depends on how it is held, so
    # the seat is found rather than computed.
    transport(robot, get_sampletable_position(), cleaning_station,
              height=needle_clear_height, via_transferpoint=True,
              drop_by_bump=True)
    # The flowcell is off the sample table now, so nothing is left for the
    # stage to be aligned to it for; put it back where it was before this
    # function touched it, rather than leaving it parked at the sample table
    # position indefinitely.
    _restore_stage(restore_to, 'returning the flowcell to its cleaning station')

# Bring the flowcell parked at the cleaning station to the sample seat, ready
# to draw solution from the vial standing there. The robot is assumed to be
# empty.
#
# The seat, not the mixer station: the mixer head comes down where
# rotate_carousel leaves the carousel, and the flowcell draws a few slots round
# from there, which is what sample_on_mixer_station is. make_sample turns the
# vial it just mixed round to this seat (carousel.draw_offset_steps) before
# calling this, so the flowcell descends onto that vial.
@_tracks(_flowcell_setter, state.FC_IN_GRIPPER)
def ready_flowcell_to_draw(robot):
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # draw solution and put it in the beam
    # assuming the robot is empty.
    # Straight down onto the station at the end, as everywhere else it is
    # approached. Not routed through the corridor: this step runs with the arm
    # already on the mixer side, which is the same side the station is on.
    move_to_cleaningstation(robot, cleaning_station, via_transferpoint=False)
    pickup(robot)
    p2 = list(get_sample_on_mixerstation_position())
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
    robot.bump(z=-1,backoff=0.002)

# after drawing, the robot is holding the flowcell. Move it to the beam and drop it.
@_tracks(_flowcell_setter, state.FC_AT_BEAM)
def load_sample_to_beam(robot):
    # Captured before the alignment moves the stage: this is the position the
    # sample is actually measured at, and it is what the stage goes back to once
    # the flowcell is down. See _restore_stage().
    restore_to = daq_client.get_pos()
    # See daq_client.align_sample_table().
    daq_client.align_sample_table()
    robot.mvr2z(needle_clear_height)
    p2 = list(get_sampletable_position())
    p2[2] = p2[2]-distance_gripper_tag+sampletable_clear_height
    # mixer station -> sample table: crosses between the two regions.
    move_via_transferpoint(robot, p2)
    dropdown_sampletable(robot)
    # Last step of the load, and the last thing before the campaign acquires:
    # the sample is in the beam only once the stage is back where the beam is.
    _restore_stage(restore_to, 'loading the sample to the beam')

# after data collection, pick up the flowcell from the beam and move it to the mixer to aspirate.
@_tracks(_flowcell_setter, state.FC_IN_GRIPPER)
def return_sample(robot):
    # Captured before the alignment, and restored at the end once the flowcell
    # has left the sample table. Without this the stage would be left standing
    # at the handoff position, and the *next* load_sample_to_beam would read
    # that as the position to go back to -- so the measuring position would be
    # lost after the first sample and every one after it measured in the wrong
    # place. See _restore_stage().
    restore_to = daq_client.get_pos()
    # See daq_client.align_sample_table().
    daq_client.align_sample_table()
    # move up to the sample table height.
    p = robot.get_pos()
    sample_table_pos = get_sampletable_position()
    p[2] = sample_table_pos[2]
    robot.moveto(p)
    # move to the sample table. Routed through the transfer point because the
    # gripper is not necessarily starting from the sample table side: in
    # unload_sample this runs straight after mixer2cleaningstation, so the
    # approach can begin at the mixer cleaning station.
    move_via_transferpoint(robot, sample_table_pos)
    pickup(robot)
    # The sample seat, matching ready_flowcell_to_draw: the sample goes back
    # into the vial it was drawn from, and that vial is standing at the seat,
    # not under the mixer head. Aspirating over the mixer station would push it
    # into whichever vial the carousel happens to have left there.
    p2 = list(get_sample_on_mixerstation_position())
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    # sample table -> mixer side: crosses between the two regions.
    move_via_transferpoint(robot, p2)
    robot.bump(z=-1,backoff=0.005)
    # The flowcell is off the sample table and over the mixer side, so the stage
    # is free again -- and has to be put back, or the next load would take the
    # handoff position for the measuring one.
    _restore_stage(restore_to, 'returning the sample from the beam')

# after aspirating, move the flowcell to the cleaning station and drop it.
@_tracks(_flowcell_setter, state.FC_AT_CLEANING)
def wash_flowcell_after_return(robot):
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    robot.mvr2z(needle_clear_height)
    move_to_cleaningstation(robot, cleaning_station, via_transferpoint=False)
    # By feel, as at the other end of this station's use: where the flowcell is
    # let go is where the seat turns out to be, not a computed height. The
    # taught pose is still passed, for the fast part of the descent.
    dropdown(robot, station_pos=cleaning_station, by_bump=True)

# Pick the flowcell up from its cleaning station and hold it over the sample
# seat on the mixer station, without letting go. The teaching approach for that
# seat: it shows no AprilTag, so the search cannot find it, and until it has
# been taught the target is the mixer cleaning station shifted by
# sample_on_mixer_offset. That puts the flowcell within a nudge of the seat, to
# be jogged onto it by hand and recorded from the Configuration tab.
#
# The arm ends at the seat's reference pose -- the pose a transport starts
# from, distance_gripper_tag + grab_depth above the grab point, with the
# flowcell hanging that far clear of the seat. That is deliberately the pose
# "Set Current Robot Position As" wants recorded, so a jog from here can be
# saved as-is without the operator having to add the standoff back in.
@_tracks(_flowcell_setter, state.FC_IN_GRIPPER)
def flowcell_to_sample_on_mixer(robot):
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # Read before anything moves: untaught and with no mixer cleaning station
    # to derive it from, this raises, and it should do so with the arm still
    # standing where it started rather than holding the flowcell somewhere.
    target = list(get_sample_on_mixerstation_position())
    activate_gripper(robot)
    # Both legs routed: a standalone button can be pressed with the arm parked
    # anywhere, including at the sample table. Down onto the station vertically
    # at the end of the first one.
    move_to_cleaningstation(robot, cleaning_station)
    pickup(robot)
    approach = list(target)
    approach[2] = target[2]-distance_gripper_tag+needle_clear_height
    _move_leg(robot, approach, True)
    # Half speed for the descent, as at the sample table: on the first press
    # the target is a computed guess rather than a taught pose, and this is the
    # leg that closes on it.
    robot.moveto(target, acc=placement_accel, vel=placement_speed)

# after washing, raise the robot to the sample table's height so it is clear
# of the cleaning station before the next command moves it elsewhere. Uses the
# full 6-element pose rather than robot.get_pos() (which is position only, and
# which moveto() would have to backfill the orientation for) so the pose sent
# is explicit here rather than reconstructed inside moveto().
def raise_to_sampletable_height(robot):
    p = robot.get_pose().get_pose_vector().tolist()
    p[2] = get_sampletable_position()[2]
    robot.moveto(p)

## Configuration functions. These functions are used to locate the positions of the sample table and cleaning station using AprilTags.
def store_station_position(pos, v):
    """Record `v` as station `pos`'s taught position.

    The one place a station name is turned into a setter, so the AprilTag
    search and the manual teach cannot record to different places. `pos` is a
    commands.STATIONS name; which physical cleaning station 'cleaning_station'
    means depends on the flowcell in use, resolved here at call time.

    Raises ValueError on an unknown station rather than returning quietly:
    silently recording nothing would look exactly like a successful teach.
    """
    if pos == 'sample_table':
        set_sampletable_position(v)
    elif pos == 'cleaning_station':
        set_cleaningstation_position(1 if flowcell_ID == 1 else 2, v)
    elif pos == 'mixer_cleaning_station':
        set_mixer_cleaningstation_position(v)
    elif pos == 'mixer_station':
        set_mixerstation_position(v)
    elif pos == 'sample_on_mixer_station':
        set_sample_on_mixerstation_position(v)
    else:
        raise ValueError('unknown station %r' % (pos,))
    if pos == 'sample_table':
        # The beamline DAQ sample-stage motors are a separate control system
        # from the robot (see daq_client.py): capture where they are the
        # moment the robot's own idea of sample_table is (re)established, so
        # align_sample_table() can put them back exactly here later rather
        # than wherever something else left them. Best-effort -- the robot
        # position just taught is worth keeping even if the DAQ GUI happens to
        # be unreachable right now; the transports that need the DAQ position
        # will raise clearly on their own if it was never recorded.
        try:
            daq_client.record_positions()
        except Exception as e:
            print('PAL12idb: could not record DAQ sample-table positions: %s' % e)
    return v

def get_station_position(pos):
    """The taught position for station `pos` (a commands.STATIONS name).

    The read counterpart of store_station_position(), resolving
    'cleaning_station' against the flowcell in use the same way. Raises
    RuntimeError (from get_position) if the station has never been taught.
    """
    if pos == 'sample_table':
        return get_sampletable_position()
    if pos == 'cleaning_station':
        return get_cleaningstation_position(1 if flowcell_ID == 1 else 2)
    if pos == 'mixer_cleaning_station':
        return get_mixer_cleaningstation_position()
    if pos == 'mixer_station':
        return get_mixerstation_position()
    if pos == 'sample_on_mixer_station':
        return get_sample_on_mixerstation_position()
    raise ValueError('unknown station %r' % (pos,))

ORIENTATION_AXES = ('x', 'y', 'z')

def tweak_orientation(robot, axis, degrees):
    """Rotate the tool `degrees` about its own `axis`, leaving position alone.

    The jog behind the Configuration tab's orientation teaching. Rotating in
    the TCP frame keeps the gripper tip where it is and swings the wrist
    around it, so the operator can match the tool to a tilted seat without
    losing the position the AprilTag search found.
    """
    axis = str(axis).lower()
    if axis not in ORIENTATION_AXES:
        raise ValueError('axis must be one of %s, got %r' % (ORIENTATION_AXES, axis))
    robot.set_tcp(robot.tcp)
    {'x': robot.rotx, 'y': robot.roty, 'z': robot.rotz}[axis](
        float(degrees), coordinate='tcp')
    return robot.get_pose().get_pose_vector().tolist()

def goto_station(robot, pos):
    """Move the robot to station `pos`'s taught position, via transfer_point.

    A manual "take me there" for checking a taught position by eye, not part
    of any sequence -- so unlike the transports, which know both ends of the
    leg they are running, this goes through transfer_point. It can be pressed
    with the arm standing anywhere, and the corridor is the one route that does
    not depend on knowing where it started. Pressed for the station the arm is
    already at, it stays put rather than making the round trip (already_at()).

    The arm ends at the taught pose itself: the same pose a transport starts
    from, distance_gripper_tag + grab_depth above the grab point. It does not
    grab, release, or descend.
    """
    target = get_station_position(pos)
    if pos == 'cleaning_station':
        # Down onto it, like every move that goes there for real -- checking a
        # taught position by eye is worth nothing if the trip to it does not
        # take the path the transports take.
        move_to_cleaningstation(robot, target)
    else:
        move_via_transferpoint(robot, target)
    return target

def goto_transfer_point(robot):
    """Move the arm to the corridor waypoint, and stop there.

    Not a taught station: transfer_point is a fixed pose in this module, so
    there is nothing to teach and nothing to check for. This is the manual
    "park it out of the way" / "start from somewhere known" -- from the
    corridor every station is one ordinary move away, which is the same
    property move_via_transferpoint relies on.

    Sent direct rather than through move_via_transferpoint, which would be
    circular. The gripper TCP is asserted first because transfer_point is
    expressed in that frame: an aborted AprilTag search can leave the camera
    TCP active, and that would put the camera on the pose with the gripper an
    offset away from it.
    """
    robot.set_tcp(robot.tcp)
    target = list(transfer_point)
    robot.moveto(target)
    return target

def record_current_orientation(robot, pos):
    """Replace station `pos`'s taught orientation, keeping its taught X/Y/Z.

    For a station the AprilTag search locates well but cannot orient: the
    flowcell does not sit level in its cleaning station, and its 12 mm tag is
    too small at any workable standoff to resolve which way it is tilted (the
    pose solver's two solutions differ by more than the tilt itself). Position
    is unaffected by that -- centring on the tag is a 1-2 px measurement -- so
    only RX/RY/RZ are taken from where the arm is now.

    The seat angle is fixed station geometry, so this is a once-per-setup
    operation, not something to redo per teach. Raises RuntimeError if the
    station has no taught position yet: there is no X/Y/Z to keep.
    """
    taught = get_station_position(pos)
    robot.set_tcp(robot.tcp)
    current = robot.get_pose().get_pose_vector().tolist()
    return store_station_position(pos, list(taught[:3]) + list(current[3:]))

def record_current_position(robot, pos):
    """Record where the robot is standing right now as station `pos`.

    The manual alternative to locate_apriltag(): same stations, same stored
    value -- the pose read with the gripper TCP active -- but taken from
    wherever the arm has been jogged to instead of from an AprilTag search.

    What gets stored is a reference pose, not the grab point: pickup()
    descends distance_gripper_tag + grab_depth from it. Jogging the gripper
    onto the object and pressing this would teach a position that much too
    low, so the arm has to be left where a search would leave it.
    """
    robot.set_tcp(robot.tcp)
    return store_station_position(pos, robot.get_pose().get_pose_vector().tolist())

def wait_for_camera_focus(robot, timeout = camera_focus_timeout,
                          poll_interval = 0.2, stop_event = None):
    """Pull frames until the image is sharp again, or `timeout` runs out.

    Returns True if a sharp frame was seen. False means the wait ran out (or
    the operator aborted) -- not a reason to stop by itself, since a tag can
    still decode from a frame urcamera calls blurry: isblurry() judges the
    centre crop against a fixed variance-of-Laplacian threshold, which is a
    coarser question than whether this particular tag resolves.

    Nothing here commands the focus. The arm's camera is the robot's own IP
    camera, so urcamera's focus()/scanfocus()/autofocus() all raise
    NoUSBCameraException on it; the camera refocuses by itself and the only
    move available is to keep asking it for frames until it has.
    """
    t0 = time.time()
    while True:
        if stop_event is not None and stop_event.is_set():
            return False
        # Reuse a live display loop's frame if showcamera is already capturing,
        # the way camera_tools._detect_apriltag does.
        if not robot.camera._running:
            robot.camera.capture()
        # isblurry() answers None when there is no frame to judge at all, which
        # is a reason to keep waiting rather than to call the image sharp.
        if robot.camera.isblurry() is False:
            return True
        if time.time() - t0 >= timeout:
            print("Camera still out of focus after {:.1f} s.".format(timeout))
            return False
        time.sleep(poll_interval)


def reacquire_apriltag(robot, step = apriltag_reacquire_step,
                       max_travel = apriltag_reacquire_max_travel,
                       stop_event = None):
    """Creep the camera down in `step` moves until the AprilTag reads again.

    For a tag lost partway through a teach, once the search has already put
    the camera on it. Returns (regained, travelled): whether a tag was read,
    and how far down this actually moved -- the caller needs the travel to
    keep its own descent budget honest, and it is not simply max_travel, since
    the loop stops at the step that succeeds.

    Every step waits for focus before looking. A move this small is over long
    before the camera has caught up with it, so reading a frame straight away
    mostly measures the blur rather than the tag, and a run of those would
    spend the whole travel allowance without ever giving the tag a fair look.
    """
    travelled = 0.0
    for _ in range(int(round(max_travel / step))):
        if stop_event is not None and stop_event.is_set():
            return False, travelled
        robot.mvr2z(-step)
        travelled = travelled + step
        wait_for_camera_focus(robot, stop_event=stop_event)
        if camera_tools._detect_apriltag(robot, settle=apriltag_reacquire_settle,
                                         stop_event=stop_event) is not None:
            print("Regained the AprilTag {:.3f} m down.".format(travelled))
            return True, travelled
    print("No AprilTag after {:.3f} m down; giving up.".format(travelled))
    return False, travelled


def descend_to_apriltag(robot, distance = apriltag_view_distance, tolerance = 0.005,
                        max_steps = 4, max_descent = 0.4, stop_event = None):
    """Move the camera until it sits `distance` m from the AprilTag in view.

    Returns the last measured camera-to-tag distance, or None if no tag could
    be read at all. robot.camera.AT_physical_size has to already be set to the
    size of the tag being looked at -- locate_apriltag() does that from
    APRILTAG_SIZES -- since the measurement is directly proportional to it.

    Measure and move is iterated rather than done once, because each step is
    only as good as the single frame behind it; the loop stops as soon as the
    tag is within `tolerance` of the target. max_descent caps the total
    downward travel this can ever command, so one bad reading (a misread tag,
    a stale AT_physical_size) cannot walk the arm down into the station.
    """
    # _detect_apriltag is camera_tools' own helper rather than public API, but
    # it is what every distance check in that module uses: it re-polls until
    # the tag reads, reuses a live display loop's frame if one is running, and
    # honours stop_event instead of waiting out its settle timeout.
    measured = None
    descended = 0.0
    for _ in range(max_steps):
        if stop_event is not None and stop_event.is_set():
            return measured
        if camera_tools._detect_apriltag(robot, stop_event=stop_event) is None:
            # Lost it mid-descent. Nudge down and look again before giving up,
            # within whatever is left of the same budget: max_descent is there
            # so that no reading, good or missing, can walk the arm into the
            # station, and recovery that could add its 10 mm on top of a
            # spent budget would be a way around it. A budget of nothing left
            # means no nudge, and this gives up exactly as it used to.
            regained, recovered = reacquire_apriltag(
                robot, max_travel=min(apriltag_reacquire_max_travel,
                                      max(0.0, max_descent - descended)),
                stop_event=stop_event)
            descended = descended + recovered
            if not regained:
                return measured
        measured = robot.camera.QRdistance
        step = measured - distance
        print(f"AprilTag is {measured:.3f} m from the camera (target {distance:.3f} m).")
        if abs(step) <= tolerance:
            break
        if step > 0 and descended + step > max_descent:
            step = max_descent - descended
            if step <= 0:
                print(f"Stopping at the {max_descent:.3f} m descent limit.")
                break
        robot.mvr2z(-step)
        descended = descended + step
    return measured

def locate_apriltag(robot, pos = '', stop_event=None, skip_roll=False):
    # Record the taught position of a station. Returns the pose it found, and
    # also stores it in sample_table / cleaning_station. stop_event, if given,
    # is a threading.Event the caller can set (alongside stopping the robot)
    # to abort the search early; see camera_tools.search_apriltag_by_tilt.
    # skip_roll leaves the camera face-normal-down (level) instead of rolling
    # it face-down / squaring it to a tilted tag: position is still recorded,
    # only the orientation differs (teach a tilted seat's angle by hand).
    global sample_table, cleaning_station1, cleaning_station2, mixer_cleaning_station, mixer_station
    ref_pos = []
    if pos == 'sample_table':
        ref_pos = ref_sampletable
    if pos == 'cleaning_station':
        ref_pos = ref_cleanstation
    if pos == 'mixer_cleaning_station':
        ref_pos = ref_mixer_cleantable
    if pos == 'mixer_station':
        ref_pos = ref_mixer_cleantable
    print(f"Looking for {pos} ....")
    if len(ref_pos)==0:
        ref_pos = ref_sampletable
    # Tell the camera how big the tag it is about to look at actually is,
    # before anything measures a distance from it. This is read by every
    # distance-based step downstream, not just the descent below: the tilt
    # search's own "descend until the tag is 0.2 m away" loop, the pivot point
    # roll_around_tag() sets, and the pixels-to-meters conversion in
    # center_camera2apriltag() all scale with it.
    robot.camera.AT_physical_size = APRILTAG_SIZES.get(pos, flowcell_apriltag_size)
    # Arrive at the reference pose through the corridor, like every other move
    # that crosses the cell. search_apriltag_by_tilt() opens with a direct
    # moveto(ref_pos), which from the far side sweeps the arm through whatever
    # lies between: both mixer stations reference ref_mixer_cleantable, so a
    # search for either one started at the sample table crosses the whole cell,
    # and a sample table search started at the mixer does the same in reverse.
    # Getting there first leaves that moveto with nowhere left to travel.
    #
    # With the gripper TCP, which is the frame ref_pos is expressed in and the
    # one the search itself sets before moving. A camera TCP left active by an
    # earlier step would otherwise put the camera on the reference pose and the
    # gripper an offset away from it.
    robot.set_tcp(robot.tcp)
    move_via_transferpoint(robot, list(ref_pos))
    # One pass. This used to run the search twice, closing in between them, so
    # that the position was recorded from a close-range detection rather than
    # from wherever the first pass happened to stop. The search now arrives
    # there by itself: its descent loop reaches a true standoff (it only
    # stopped short because AT_physical_size was wrong, which is set above),
    # and the align_to_tag path closes to AT_SQUARE_UP_DISTANCE and squares up
    # from there. A second pass re-ran the tilt grid, the roll, and -- for a
    # tilted tag -- the whole probe-and-square loop, from a reference already
    # sitting on the tag, for a result the first pass had already reached.
    found = camera_tools.search_apriltag_by_tilt(
        robot, ref_pos=ref_pos, align_to_tag=(pos in APRILTAG_NOT_FLAT),
        stop_event=stop_event, skip_roll=skip_roll)
    if not found:
        # Previously this fell through to grab/bump/record a position even
        # on a failed or aborted search -- since the robot could be
        # anywhere the tilt search left it. Stop here instead: no position
        # is worth recording without a confirmed tag detection.
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError(f"AprilTag search for {pos} was stopped by the operator")
        # Say which step gave up and why, not just that the search did.
        # camera_tools records the reason as it bails (no tag in the tilt
        # range, the squaring not converging, the tilt response not being
        # invertible, ...); without it this read "failed to find a tag" even
        # when the tag had been found and it was the alignment that failed.
        reason = getattr(camera_tools, 'last_search_failure', None)
        raise RuntimeError("AprilTag alignment for %s failed: %s"
                           % (pos, reason or "no reason reported"))
    # Settle on a known standoff before the grab. Not for the recorded value --
    # the bump below feels for contact, so it lands in the same place either
    # way -- but so the bump always travels about the same distance, whether
    # the search left off at its own descent granularity or at the closer
    # AT_SQUARE_UP_DISTANCE a tilted tag needs.
    print(f"Settling at the working standoff for {pos} ....")
    if descend_to_apriltag(robot, stop_event=stop_event) is None:
        raise RuntimeError(f"Lost the AprilTag for {pos} while settling")
    # The other place a teach can lose the tag: centring returns False when it
    # cannot find one to centre on. That was being discarded, so a lost tag
    # here fell through to the grab and recorded a position nothing had
    # confirmed -- the very thing the search's own `not found` branch refuses
    # to do. Nudge down for it and try once more; still nothing, and this stops
    # like every other lost-tag path rather than teaching a guess.
    if not robot.center_camera2apriltag(tol_m=0.0002):
        if not reacquire_apriltag(robot, stop_event=stop_event)[0]:
            raise RuntimeError(f"Lost the AprilTag for {pos} while centring")
        if not robot.center_camera2apriltag(tol_m=0.0002):
            raise RuntimeError(f"Could not centre the camera on {pos}'s AprilTag")
    robot.put_tcp2camera()

    robot.grab()
    robot.bump(z=-1,backoff=distance_gripper_tag)
    robot.set_tcp(robot.tcp)
    p = robot.get_pose()
    v = p.get_pose_vector().tolist()
    store_station_position(pos, v)
    return v
