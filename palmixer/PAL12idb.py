import configparser
import functools
import os

import camera_tools

try:
    from . import state
except ImportError:
    import state

ref_mixer_cleantable = [0.4, 0.1, 0.1, 2.231, -2.212, 0]
ref_cleanstation = [0.38, -0.16, -0.1, 2.231, -2.212, 0]
ref_sampletable = [-0.22, -0.37, 0.12, -2.18860535, 2.25379435, 0]
# Intermediate pose for the sample table <-> mixer side traverse. A direct
# point-to-point move between those two regions sweeps the arm through what
# sits between them, so every such leg is routed through this pose instead
# (see move_via_transferpoint). Not a taught position: it is a fixed corridor
# waypoint, so it lives here rather than in waypoints.ini.
transfer_point = [0.25, -0.16, 0.1, 2.231, -2.212, 0]
distance_gripper_tag = 0.05
# move sth out by 50 mm
# move sav down by 30 mm
# after grap, have to move 0.20 m up to clear the needle.
sample_table = []
cleaning_station1 = []
cleaning_station2 = []
mixer_cleaning_station = []
mixer_station = []
needle_clear_height = 0.16
# Clearance held over the sample table on approach. Lower than
# needle_clear_height: the full lift overshoots the table, and nothing at the
# table needs that much room. Only the approach is shortened -- the lift at
# whichever station the flowcell came from still uses needle_clear_height.
sampletable_clear_height = needle_clear_height - 0.03
mixer_height = 0.05
grab_depth = 0.01
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
from epics import caget, caput
sampletableID = 1
cleaningstationID1 = 2
cleaningstationID2 = 3
mixerstationID = 4
mixer_cleaningstationID = 5


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


def _write_ini_position(ID, pos):
    # Merge into the existing file rather than overwriting it, so other
    # stations' cached positions survive.
    parser = configparser.ConfigParser()
    parser.read(_INI_PATH)
    section = _ini_section(ID)
    if not parser.has_section(section):
        parser.add_section(section)
    for f, v in zip(_POSITION_FIELDS, pos):
        parser.set(section, f, repr(float(v)))
    with open(_INI_PATH, 'w') as fp:
        parser.write(fp)


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
                mixerstationID, mixer_cleaningstationID)


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

# -- position readiness checks -----------------------------------------------
# Station keys match commands.STATIONS / commands.STATION_LABELS, so callers
# on the server side can report a missing position using the same label the
# Configuration tab's AprilTag-search buttons use.
_STATION_IDS = {
    'sample_table': lambda: sampletableID,
    'mixer_station': lambda: mixerstationID,
    'mixer_cleaning_station': lambda: mixer_cleaningstationID,
    # Resolved at call time, like the transport functions do: which physical
    # cleaning station this means depends on the flowcell currently in use.
    'cleaning_station': lambda: cleaningstationID1 if flowcell_ID == 1 else cleaningstationID2,
}

# Which station positions each transport function reads via get_position().
# workflows.py composes its own requirements from this rather than keeping a
# second list, so the two cannot drift apart.
TRANSPORT_STATIONS = {
    'mixer2cleaningstation': ('mixer_station', 'mixer_cleaning_station'),
    'mixer2mixingstation': ('mixer_cleaning_station', 'mixer_station'),
    'load_flowcell_from_cleaningstation_to_beam': ('cleaning_station', 'sample_table'),
    'load_flowcell_from_beam_to_cleaningstation': ('sample_table', 'cleaning_station'),
    'ready_flowcell_to_draw': ('cleaning_station', 'mixer_station'),
    'load_sample_to_beam': ('sample_table',),
    'return_sample': ('sample_table', 'mixer_station'),
    'wash_flowcell_after_return': ('cleaning_station',),
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
        if _read_ini_position(_STATION_IDS[key]()) is None:
            missing.append(key)
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

def dropdown(robot, height = needle_clear_height):
    # move down
    #robot.mvr2z(-1*(height+grab_depth-0.005))
    robot.bump(z=-1, backoff=0)
    robot.release()
    # move back up to standard height
    robot.mvr2z(distance_gripper_tag)

def dropdown_sampletable(robot):
    # Release onto the sample table without bumping for contact: descend to a
    # Z computed from the taught position and let go there. The full taught
    # pose is sent rather than a relative Z move, so where the flowcell is set
    # down does not depend on the clearance it was carried in at.
    #
    # The release Z is 0.005 m above where pickup() grabs -- pickup() descends
    # distance_gripper_tag + grab_depth from the taught pose, so this is
    # (taught Z - distance_gripper_tag) - grab_depth + 0.005. Same landing
    # height the relative-move dropdown() used before it was changed to bump.
    #
    # The descent runs at half speed. It is the one move that sets the
    # flowcell down on the beamline stage, open-loop -- no bump to feel for
    # contact -- so anything the taught position is off by is taken up by the
    # hardware. Halving the approach is cheap here: it is a 0.06 m move once
    # per sample.
    p = list(get_sampletable_position())
    p[2] = p[2]-distance_gripper_tag-grab_depth+0.005
    robot.moveto(p, acc=placement_accel, vel=placement_speed)
    robot.release()
    # move back up to standard height
    robot.mvr2z(distance_gripper_tag)

def transport2sampletable(robot, p1, height = needle_clear_height,
                          via_transferpoint = False):
    # transport() specialized to the sample table as the destination: the drop
    # is dropdown_sampletable() rather than the bump-for-contact dropdown(),
    # and the destination comes from the taught sample table position rather
    # than from the caller. The robot is assumed to be empty.
    #
    # via_transferpoint routes both legs -- the empty approach to p1 and the
    # carry to the table -- through transfer_point; see _move_leg.
    activate_gripper(robot)
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
    robot.moveto(ref_mixer_cleantable)

def move_via_transferpoint(robot, target):
    # Move to `target` by way of transfer_point. Used for every leg that
    # crosses between the sample table and the rest of the cell -- the mixer,
    # the mixer cleaning station, or the flowcell cleaning station -- in either
    # direction and whether or not the gripper is holding anything.
    # transfer_point is sent as a copy so a caller cannot mutate the module
    # constant through the robot API.
    robot.moveto(list(transfer_point))
    robot.moveto(target)

def _move_leg(robot, target, via_transferpoint):
    # One leg of a transport, routed through transfer_point or not. Both legs
    # of a transport get the same treatment: the routing is a property of the
    # two stations involved, and the robot's starting pose is not knowable here
    # (a standalone command can be run with the arm parked anywhere), so the
    # empty approach has to clear the same obstacles the carry does.
    if via_transferpoint:
        move_via_transferpoint(robot, target)
    else:
        robot.moveto(target)

def transport(robot, p1, p2, height = needle_clear_height, drop_height = None,
              via_transferpoint = False):
    # picking up the flowcell at p1 and dropping it at p2. The robot is assumed to be empty.
    # assuming robot is empty
    # `drop_height` is the clearance held over p2, and defaults to `height`.
    # It is separate so a station that wants a lower approach (the sample
    # table) can have one without also shortening the lift at p1, which has to
    # stay tall enough to clear the needle.
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
    p2[2] = p2[2]-distance_gripper_tag+drop_height
    _move_leg(robot, p2, via_transferpoint)
    dropdown(robot, height=drop_height)
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


# Actual transport functions. These functions are used to move the flowcell between the cleaning station, mixer station, and sample table.
# mixer head to its cleaning station. The robot is assumed to be empty.
@_tracks(lambda: state.set_mixer_head, state.MIXER_AT_CLEANING)
def mixer2cleaningstation(robot):
    transport(robot, get_mixerstation_position(), get_mixer_cleaningstation_position(), height=mixer_height)

# mixer head to the mixer station. The robot is assumed to be empty.
@_tracks(lambda: state.set_mixer_head, state.MIXER_AT_MIXER)
def mixer2mixingstation(robot):
    transport(robot, get_mixer_cleaningstation_position(), get_mixerstation_position(), height=mixer_height)

# Bring the flowcell parked at the cleaning station to the beam, ready for data collection. The robot is assumed to be empty.
# This is for measuring water background. The flowcell is not loaded with sample.
@_tracks(_flowcell_setter, state.FC_AT_BEAM)
def load_flowcell_from_cleaningstation_to_beam(robot):
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # cleaning station -> sample table: crosses between the two regions.
    transport2sampletable(robot, cleaning_station, height=needle_clear_height,
                          via_transferpoint=True)

# Bring the flowcell from the beam to the cleaning station.
@_tracks(_flowcell_setter, state.FC_AT_CLEANING)
def load_flowcell_from_beam_to_cleaningstation(robot):
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # sample table -> cleaning station: crosses between the two regions.
    transport(robot, get_sampletable_position(), cleaning_station,
              height=needle_clear_height, via_transferpoint=True)

# Bring the flowcell parked at the cleaning station to the mixing station,
# Ready to draw solution from the mixer. The robot is assumed to be empty.
@_tracks(_flowcell_setter, state.FC_IN_GRIPPER)
def ready_flowcell_to_draw(robot):
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # draw solution and put it in the beam
    # assuming the robot is empty.
    robot.moveto(cleaning_station)
    pickup(robot)
    p2 = list(get_mixerstation_position())
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
    robot.bump(z=-1,backoff=0.002)

# after drawing, the robot is holding the flowcell. Move it to the beam and drop it.
@_tracks(_flowcell_setter, state.FC_AT_BEAM)
def load_sample_to_beam(robot):
    robot.mvr2z(needle_clear_height)
    p2 = list(get_sampletable_position())
    p2[2] = p2[2]-distance_gripper_tag+sampletable_clear_height
    # mixer station -> sample table: crosses between the two regions.
    move_via_transferpoint(robot, p2)
    dropdown_sampletable(robot)

# after data collection, pick up the flowcell from the beam and move it to the mixer to aspirate.
@_tracks(_flowcell_setter, state.FC_IN_GRIPPER)
def return_sample(robot):
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
    p2 = list(get_mixerstation_position())
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    # sample table -> mixer station: crosses between the two regions.
    move_via_transferpoint(robot, p2)
    robot.bump(z=-1,backoff=0.005)

# after aspirating, move the flowcell to the cleaning station and drop it.
@_tracks(_flowcell_setter, state.FC_AT_CLEANING)
def wash_flowcell_after_return(robot):
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    robot.mvr2z(needle_clear_height)
    robot.moveto(cleaning_station)
    dropdown(robot, height=mixer_height)

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
    else:
        raise ValueError('unknown station %r' % (pos,))
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
    leg they are running, this always goes through transfer_point. It can be
    pressed with the arm standing anywhere, and the corridor is the one route
    that does not depend on knowing where it started.

    The arm ends at the taught pose itself: the same pose a transport starts
    from, distance_gripper_tag + grab_depth above the grab point. It does not
    grab, release, or descend.
    """
    target = get_station_position(pos)
    move_via_transferpoint(robot, target)
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
    robot.put_tcp2camera()

    robot.grab()
    robot.bump(z=-1,backoff=distance_gripper_tag)
    robot.set_tcp(robot.tcp)
    p = robot.get_pose()
    v = p.get_pose_vector().tolist()
    store_station_position(pos, v)
    return v
