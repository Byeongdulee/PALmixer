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
distance_gripper_tag = 0.05
# move sth out by 50 mm
# move sav down by 30 mm
# after grap, have to move 0.20 m up to clear the needle.
sample_table = []
cleaning_station1 = []
cleaning_station2 = []
mixer_cleaning_station = []
mixer_station = []
needle_clear_height = 0.20
mixer_height = 0.05
grab_depth = 0.01
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

_POSITION_FIELDS = ('X', 'Y', 'Z', 'RX', 'RY', 'RZ')

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
def pickup(robot, height = needle_clear_height):
    robot.release()
    # cm go deeper from the standard height
    robot.mvr2z(-grab_depth-distance_gripper_tag)
    robot.grab()
    # move high enough so the needle is cleared
    robot.mvr2z(height)

def dropdown(robot, height = needle_clear_height):
    # move down
    robot.mvr2z(-1*(height+grab_depth-0.005))
    robot.release()
    # move back up to standard height
    robot.mvr2z(distance_gripper_tag)

def test_pickup(robot, height=needle_clear_height):
    pickup(robot, height=height)
    dropdown(robot, height=height)

def test_mixer_pickup(robot):
    test_pickup(robot, height=mixer_height)

def goto_default(robot):
    robot.moveto(ref_mixer_cleantable)

def transport(robot, p1, p2, height = needle_clear_height):
    # picking up the flowcell at p1 and dropping it at p2. The robot is assumed to be empty.
    # assuming robot is empty
    robot.activate_gripper()
    robot.moveto(p1)
    pickup(robot)
    # Z position should be the needle cleared position. Work on a copy: writing
    # p2[2] in place edits the caller's list, so a taught position passed in
    # (sample_table / cleaning_station) would creep upward on every transport.
    p2 = list(p2)
    p2[2] = p2[2]-distance_gripper_tag+height
    robot.moveto(p2)
    dropdown(robot)
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
    transport(robot, cleaning_station, get_sampletable_position(), height=needle_clear_height)

# Bring the flowcell from the beam to the cleaning station.
@_tracks(_flowcell_setter, state.FC_AT_CLEANING)
def load_flowcell_from_beam_to_cleaningstation(robot):
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    transport(robot, get_sampletable_position(), cleaning_station, height=needle_clear_height)

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
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
    dropdown(robot)

# after data collection, pick up the flowcell from the beam and move it to the mixer to aspirate.
@_tracks(_flowcell_setter, state.FC_IN_GRIPPER)
def return_sample(robot):
    # move up to the sample table height.
    p = robot.get_pos()
    sample_table_pos = get_sampletable_position()
    p[2] = sample_table_pos[2]
    robot.moveto(p)
    # move to the sample table
    robot.moveto(sample_table_pos)
    pickup(robot)
    p2 = list(get_mixerstation_position())
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
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
def locate_apriltag(robot, pos = '', stop_event=None):
    # Record the taught position of a station. Returns the pose it found, and
    # also stores it in sample_table / cleaning_station. stop_event, if given,
    # is a threading.Event the caller can set (alongside stopping the robot)
    # to abort the search early; see camera_tools.search_apriltag_by_tilt.
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
    found = camera_tools.search_apriltag_by_tilt(robot, ref_pos=ref_pos, stop_event=stop_event)
    if not found:
        # Previously this fell through to grab/bump/record a position even on
        # a failed or aborted search -- since the robot could be anywhere the
        # tilt search left it. Stop here instead: no position is worth
        # recording without a confirmed tag detection.
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError(f"AprilTag search for {pos} was stopped by the operator")
        raise RuntimeError(f"AprilTag search for {pos} failed to find a tag")
    robot.put_tcp2camera()

    robot.grab()
    robot.bump(z=-1,backoff=distance_gripper_tag)
    robot.set_tcp(robot.tcp)
    p = robot.get_pose()
    v = p.get_pose_vector().tolist()
    if pos == 'sample_table':
        set_sampletable_position(v)
    if pos == 'cleaning_station':
        if flowcell_ID == 1:
            set_cleaningstation_position(1, v)
        else:
            set_cleaningstation_position(2, v)
    if pos == 'mixer_cleaning_station':
        set_mixer_cleaningstation_position(v)
    if pos == 'mixer_station':
        set_mixerstation_position(v)
    return v
