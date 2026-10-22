import configparser
import functools
import math
import os
import time

import numpy as _np

import camera_tools

try:
    from . import state
except ImportError:
    import state

try:
    from . import daq_client
except ImportError:
    import daq_client

try:
    from . import config as _config
except ImportError:
    import config as _config

try:
    from . import commands as _commands
except ImportError:
    import commands as _commands

try:
    from . import calibration as _calibration
except ImportError:
    import calibration as _calibration

# Where an AprilTag search stands to start looking -- a camera standoff above
# each station, not a grab pose. locate_apriltag() drives here first and
# search_apriltag_by_tilt() then tilts around from here hunting the tag.
#
# These are the `search_refs` section of json/palmixer_config.json; the names
# below are kept because goto_default() and the rest of this module refer to
# them, but they are now just the config read once at import. Read live with
# search_reference(), which is what every search actually calls -- config.py
# re-reads the file on every access, so an edit there takes effect without a
# restart, the same as waypoints.ini.
def _ref(station, fallback):
    try:
        pose = _config.get_section('search_refs').get(station)
        return [float(v) for v in pose] if pose else list(fallback)
    except Exception:
        return list(fallback)

ref_mixer_cleantable = _ref('mixer_cleaning_station',
                            [0.4, 0.1, 0.1, 2.231, -2.212, 0])
ref_cleanstation = _ref('cleaning_station', [0.38, -0.16, -0.1, 2.231, -2.212, 0])
ref_sampletable = _ref('sample_table',
                       [-0.22, -0.37, 0.12, -2.18860535, 2.25379435, 0])
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
# Absolute base-frame Z (gripper TCP) to lift to before swinging the arm around
# the cell -- a safe travel height clear of everything on the bench. Unlike the
# relative *_clear_height / *_lift values, this is an absolute Z to move the arm
# TO, not a relative lift. Per robot model: the UR3's shorter reach may want a
# different value than the UR5 (adjust the UR3 entry when known).
SAFE_SWING_Z_BY_MODEL = {'UR5': 0.45, 'UR3': 0.5}


# Approximate UR5 reach model for estimating the max planar radius at a height.
# Sphere about the shoulder: the wrist centre must lie within UR5_ARM_REACH
# (|a2|+|a3|) of the shoulder, which sits UR5_SHOULDER_Z above the base.
UR5_SHOULDER_Z = 0.16       # shoulder (joint-2) height above the base, m
UR5_ARM_REACH = 0.817       # shoulder -> wrist reach |a2| + |a3|, m
# raster_tag_scan step sizing: advance this fraction of the camera footprint per
# step (so adjacent stops overlap), and assume the tags sit around this bench
# height (for the camera->bench distance that sets the footprint). Both the
# angular and radial steps scale with this, so halving it halves the step size
# (denser scan, ~4x the stops) -- 0.35 after the 0.7 scan missed the mixer head.
CAMERA_FOV_OVERLAP = 0.35
CAMERA_SCAN_BENCH_Z = 0.10


def max_rxy_at_z(z, tool_len=0.0, reach=UR5_ARM_REACH, shoulder_z=UR5_SHOULDER_Z,
                 margin=0.0):
    """Approximate the maximum planar radius Rxy the tool tip can reach at height
    `z`, for a tool pointing straight down.

    Sphere model: the wrist centre must lie within `reach` of the shoulder at
    (0, 0, shoulder_z). With the tool pointing down the wrist is `tool_len` above
    the tip and directly over it, so

        Rxy_max = sqrt(reach^2 - (z + tool_len - shoulder_z)^2) - margin.

    APPROXIMATE: it ignores the small wrist lateral offsets (d4/d5) and the
    shoulder offset, so treat it as an upper bound to sanity-check scan radii, not
    a hard limit -- verify against the robot and pull it in with `margin`. Returns
    0 when the height is outside the reachable band."""
    dz = (z + tool_len) - shoulder_z
    inside = reach * reach - dz * dz
    if inside <= 0.0:
        return 0.0
    return max(0.0, math.sqrt(inside) - margin)


def safe_swing_z():
    """The safe swing Z (absolute base Z) for the configured robot model.

    Reads robot.name from the config and looks it up in SAFE_SWING_Z_BY_MODEL,
    defaulting to the UR5 value for an unrecognised model."""
    name = str(_config.get_section('robot').get('name') or 'UR5').strip().upper()
    return SAFE_SWING_Z_BY_MODEL.get(name, SAFE_SWING_Z_BY_MODEL['UR5'])
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
# dropdown_by_bump and BUMP_RELEASE_STATIONS). 0 means release right at the seat
# the bump felt (no retreat before opening).
bump_release_backoff = 0.0
# Default release height above the grab point for a computed-Z drop
# (dropdown_at). The flowcell cleaning station overrides it to 2 mm: it is set
# down at a known Z rather than by feel, 2 mm above where the flowcell was
# grabbed, then opened. It is come down on purely vertically
# (move_to_cleaningstation), so there is nothing to brush on the way and a known
# height is both safe and gentler than the deep-seat bump the mixer side uses.
dropdown_release_above = 0.005
cleaning_station_release_above = 0.002
# The sample stage drop releases 1 mm above the grab point: the flowcell is set
# onto the beamline stage open-loop, and a low release lands it gently where the
# taught pose puts it rather than dropping it the default height.
sampletable_release_above = 0.001
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
# How far above the tag surface an AprilTag search descends (using the measured,
# calibrated tag distance) before the teach bump feels for the surface. Keeps the
# bump travel short and consistent regardless of the camera standoff the search
# left off at. See locate_apriltag.
APRILTAG_BUMP_STANDOFF = 0.030
# The stations set down that way. Documentation rather than a lookup -- each
# transport passes drop_by_bump itself, since it already knows which end it is
# releasing at -- but the two places that do have to agree with each other, so
# the list of them lives here.
BUMP_RELEASE_STATIONS = ('mixer_station', 'mixer_cleaning_station',
                         'sample_on_mixer_station', 'cleaning_station')
# Physical edge length of the AprilTag each station shows the camera. The
# camera measures distance as AT_physical_size / (tag edge in pixels) * focal
# length (urcamera.getATdistance), so this scales every distance reading
# linearly -- with the robot's default 7.5 mm left in place the camera
# believes it is much closer to the tag than it is, and a descent aimed at
# 0.2 m stops well short.
#
# **This is the BLACK SQUARE's edge, not the printed tag's.** A tag36h11 is
# 8 cells of black inside a white quiet zone, and the detector's tag_size
# means the black border's outer edge. Measuring the sheet instead reads
# about 1.3x high, and every distance with it: the flowcell value here was
# 12 mm (the sheet) against a black square that calipers at 9.5 mm, which had
# every flowcell-tag distance 26% long.
# Whose tag each station's search is actually looking at. A tag belongs to the
# thing it is stuck to, not to the place: the flowcell carries its tag to every
# station it visits, so one station's search looks for a different tag
# depending on which flowcell is in use, while the sample table and the two
# mixer stations each have a tag that stays put.
#
# 'flowcell' is resolved to flowcell_1 / flowcell_2 at call time by the
# flowcell in use, the same way 'cleaning_station' resolves to a position.
APRILTAG_SUBJECT = {
    # The sample table has no tag of its own: what the search sees there is
    # the flowcell that was transferred onto it, wearing the same tag it wore
    # at its cleaning station. So this station and cleaning_station look for
    # the *same* tag, and which one depends on the flowcell in use.
    'sample_table': 'flowcell',
    'cleaning_station': 'flowcell',
    'mixer_station': 'mixer_head',
    'mixer_cleaning_station': 'mixer_head',
    # Carousel positions read the carousel tag at the centre. Carousels are
    # numbered 31-40, and any one of them may be standing at any carousel
    # position -- on the mixer, in stock, or on the used stack -- so all three
    # use the shared 'carousel' entry (id_range 31-40) and accept any carousel.
    'carousel_on_mixer': 'carousel',
    'carousel_in_stock': 'carousel',
    'carousel_used_stack': 'carousel',
}

# Stations that are a reference point, not something to grab: the search still
# bumps down to the surface and backs off the same distance every other station
# does (so the recorded Z is anchored to the real surface), but it does NOT grab
# -- there is nothing to pick up at a carousel centre. See locate_apriltag.
NO_GRAB_STATIONS = ('carousel_on_mixer', 'carousel_in_stock', 'carousel_used_stack')
# Fallbacks for the `apriltags` config section, used when it says nothing about
# a tag. Only sizes: an unconfigured *number* means "whichever tag is nearest
# the image center", which is a real behaviour rather than a missing value.
APRILTAG_FALLBACK_SIZES = {
    'flowcell_1': 0.0095,
    'flowcell_2': 0.0095,
    'sample_table': 0.0315,
    'mixer_head': 0.016,
    'carousel': 0.016,
}
DEFAULT_APRILTAG_SIZE = 0.0095


def _apriltag_subject(station):
    """Which entry in the `apriltags` config a station's search reads."""
    subject = APRILTAG_SUBJECT.get(station)
    if subject == 'flowcell':
        return 'flowcell_%d' % (1 if flowcell_ID == 1 else 2)
    return subject


def _apriltag_entry(station):
    subject = _apriltag_subject(station)
    if subject is None:
        return None, {}
    try:
        entry = _config.get_section('apriltags').get(subject) or {}
    except Exception:
        entry = {}
    return subject, entry if isinstance(entry, dict) else {}


def apriltag_size(station):
    """Black-square edge length of the tag `station`'s search looks for.

    **The black square, not the printed sheet.** A tag36h11 is 8 cells of
    black inside a white quiet zone and the detector's tag_size means the
    black border's outer edge; measuring the sheet reads about 1.3x high, and
    every distance with it. The flowcell entry was 12 mm (the sheet) against a
    square that calipers at 9.5 mm, which had every flowcell-tag distance 26%
    long.

    The camera measures distance as AT_physical_size / (tag edge in pixels) *
    focal length (urcamera.getATdistance), so this scales every reading
    linearly -- a search aimed at a 0.2 m standoff stops somewhere else
    entirely if it is wrong.
    """
    subject, entry = _apriltag_entry(station)
    try:
        size = entry.get('size_m')
        if size is not None:
            return float(size)
    except (TypeError, ValueError):
        print("PAL12idb: ignoring non-numeric apriltags.%s.size_m" % subject)
    return APRILTAG_FALLBACK_SIZES.get(subject, DEFAULT_APRILTAG_SIZE)


def apriltag_id(station):
    """The AprilTag number `station`'s search looks for, or None.

    None means "whichever tag is nearest the image center", decided afresh on
    every frame. That is correct only while one tag at a time can be in view;
    where two can, a search that tilts, rolls and descends over many frames
    can change its mind about which it is measuring partway through.

    A non-numeric entry is ignored with a warning rather than raised on: a
    typo here must not break a search that worked yesterday, and the tag
    actually used is reported either way.
    """
    subject, entry = _apriltag_entry(station)
    try:
        value = entry.get('id')
        return None if value is None else int(value)
    except (TypeError, ValueError):
        print("PAL12idb: ignoring non-numeric apriltags.%s.id" % subject)
        return None


def apriltag_rz_offset(station):
    """Degrees the station's tag is mounted turned about its own normal (Z).

    0.0 when unset. The search squares the recorded heading to the tag, so a tag
    that is stuck on rotated about its normal records a heading that much off the
    seat's real frame; this is subtracted back out before the pose is stored (see
    locate_apriltag). The carousel tag is mounted at +90 deg -- apriltags.carousel
    carries rz_offset_deg. A non-numeric entry is ignored with a warning, like id.
    """
    subject, entry = _apriltag_entry(station)
    try:
        value = entry.get('rz_offset_deg')
        return 0.0 if value is None else float(value)
    except (TypeError, ValueError):
        print("PAL12idb: ignoring non-numeric apriltags.%s.rz_offset_deg" % subject)
        return 0.0


def apriltag_ids(station):
    """The set of AprilTag numbers `station`'s search will accept, or None.

    Most stations name one tag (apriltags.<subject>.id). A seat that can show any
    of several tags names a range instead -- apriltags.<subject>.id_range
    [lo, hi], inclusive -- or an explicit apriltags.<subject>.ids list. The
    carousel positions use id_range 31-40: the carousel standing there (at the
    mixer or in stock) may be any one of them. None means no id is configured --
    the search then refuses rather than locking onto the nearest tag (see
    locate_apriltag). selectAT takes the set and matches any member.
    """
    subject, entry = _apriltag_entry(station)
    rng = entry.get('id_range')
    if rng is not None:
        try:
            lo, hi = int(rng[0]), int(rng[1])
            return set(range(min(lo, hi), max(lo, hi) + 1))
        except (TypeError, ValueError, IndexError):
            print("PAL12idb: ignoring malformed apriltags.%s.id_range" % subject)
    ids = entry.get('ids')
    if ids is not None:
        try:
            out = set(int(i) for i in ids)
            if out:
                return out
        except (TypeError, ValueError):
            print("PAL12idb: ignoring malformed apriltags.%s.ids" % subject)
    single = apriltag_id(station)
    return {single} if single is not None else None


def venton_table_tags():
    """Configured Venton-table fiducials usable for calibration, keyed by id.

    Every ``apriltags.*`` entry that has BOTH a numeric ``id`` and a
    ``table_xy_mm`` -- a tag with no id cannot be singled out in a frame, and
    one with no table position has no known point to anchor the fit to, so
    either missing drops it. Each value is {name, table_xy_mm, size_m}.
    """
    tags = {}
    try:
        section = _config.get_section('apriltags')
    except Exception:
        return tags
    for name, entry in section.items():
        if not isinstance(entry, dict):
            continue
        tid, xy = entry.get('id'), entry.get('table_xy_mm')
        if tid is None or not xy:
            continue
        try:
            size = entry.get('size_m')
            tags[int(tid)] = {
                'name': name,
                'table_xy_mm': [float(xy[0]), float(xy[1])],
                'size_m': float(size) if size is not None else DEFAULT_APRILTAG_SIZE}
        except (TypeError, ValueError, IndexError):
            print("PAL12idb: skipping malformed apriltags.%s for calibration" % name)
    return tags


def _measure_tag_in_base(robot, tag_id, size_m):
    """Closed-loop base-frame position of one tag: centre, descend close,
    re-centre, then project along the optical axis.

    Returns a 3-list (metres) or None if the tag could not be acquired. This is
    the path stations are taught with (center_camera2apriltag +
    descend_to_apriltag), and the reason it is used here instead of a one-shot
    open-loop read: an off-axis tag placed from a single frame is corrupted by
    any error in the camera's mount geometry (camtcp), which showed up as a
    ~120 mm spread between views. Centring drives the tag onto the optical axis,
    where that error cancels, and descending to a short standoff shrinks what
    little remains (lateral error scales with distance). What is left is the
    accuracy the whole system already teaches positions with.
    """
    robot.camera.AT_physical_size = size_m
    robot.camera.AT_preferred_id = tag_id
    try:
        if not robot.center_camera2apriltag(tag_id=tag_id):
            return None
        if descend_to_apriltag(robot) is None:
            return None
        # Re-centre at the close standoff: the descent re-detects but does not
        # re-centre, and the projection below is only on-axis if the tag is.
        if not robot.center_camera2apriltag(tag_id=tag_id):
            return None
        pos, _campos = robot.get_obj_position_from_camera_center(robot.camera.QRdistance)
        return [float(pos[0]), float(pos[1]), float(pos[2])]
    finally:
        robot.set_tcp(robot.tcp)        # always leave the gripper TCP active


def _flange_T_optical(robot):
    """flange -> camera optical frame, from the (validated, mechanical) camtcp
    and the fixed optical convention.

    camtcp is flange -> camera-TCP frame; the camera's optical frame (solvePnP:
    x right, y down, z forward) differs from it by a 180 deg turn about the
    shared viewing (Z) axis -- measured by the hand-eye calibration as 173.9 deg
    about ~Z, i.e. a clean axis flip with the few degrees being solve noise. So
    optical = camtcp . Rz(180). This replaces get_camera_vector's hard-coded
    mount angles with the real camtcp and an exact convention.
    """
    camtcp_T = _calibration.pose_vector_to_transform(
        [float(v) for v in robot.camtcp])
    flip = _np.eye(4)
    flip[0, 0] = -1.0          # Rz(180): optical x,y are camtcp x,y flipped
    flip[1, 1] = -1.0
    return camtcp_T @ flip


def _read_tags_in_view(robot, tags):
    """Base-frame position of every configured tag in the current view -- no
    motion. Returns {id: [x, y, z]} (metres).

    Each tag's position is the proper chain of transforms, not a reconstruction:
    base <- flange (robot kinematics) <- optical (validated camtcp + convention,
    _flange_T_optical) <- tag (solvePnP on the tag corners with the *calibrated*
    camera matrix, apriltag_pose_from_corners). No get_camera_vector, no assumed
    focal length -- so it is bound only by the camtcp and intrinsics accuracy,
    which are now calibrated. Tag orientation is bimodal but position is stable,
    and position is all this uses.
    """
    robot.set_tcp(robot.tcp)
    found = {}
    try:
        base_T_flange = _flange_transform(robot)      # restores tool TCP
        flange_T_optical = _flange_T_optical(robot)
        base_T_optical = base_T_flange @ flange_T_optical
        for tid, info in tags.items():
            robot.camera.AT_physical_size = info['size_m']
            robot.camera.AT_preferred_id = tid
            r = camera_tools._detect_apriltag(robot, tag_id=tid)
            if r is None:
                continue
            img = robot.camera.image
            K, dist = _camera_matrix_and_dist(robot, (img.shape[1], img.shape[0]))
            cam_T_tag = _calibration.apriltag_pose_from_corners(
                r.corners, info['size_m'], K, dist_coeffs=dist)
            if cam_T_tag is None:
                continue
            base = (base_T_optical @ cam_T_tag)[:3, 3]
            found[tid] = [float(base[0]), float(base[1]), float(base[2])]
    finally:
        robot.set_tcp(robot.tcp)        # always leave the gripper TCP active
    return found


def _all_configured_tags():
    """Every AprilTag the config knows about, as {id: {'size_m': ...}}.

    Collapses each apriltags.<subject> entry's id / ids / id_range into individual
    ids, each mapped to that subject's physical size -- so a scan can look for all
    of them and scale each tag's pose by the right size."""
    tags = {}
    for subject, entry in (_config.get_section("apriltags") or {}).items():
        size = entry.get("size_m")
        if size is None:
            continue
        ids = set()
        rng = entry.get("id_range")
        if rng:
            try:
                ids |= set(range(int(rng[0]), int(rng[1]) + 1))
            except (TypeError, ValueError, IndexError):
                pass
        for i in entry.get("ids", []) or []:
            try:
                ids.add(int(i))
            except (TypeError, ValueError):
                pass
        if entry.get("id") is not None:
            try:
                ids.add(int(entry["id"]))
            except (TypeError, ValueError):
                pass
        for tid in ids:
            tags[tid] = {"size_m": float(size)}
    return tags


def scan_apriltags_during_base_swing(robot, start_deg=-45.0, end_deg=45.0,
                                     speed_deg_s=10.0, step_deg=30.0,
                                     settle_s=3.0):
    """Sweep the base joint start->end deg (relative) and, across the sweep, find
    every configured AprilTag and record its base-frame position.

    Stepped rather than continuous: the base is moved in small CAROUSEL-free steps
    (step_deg) and all tags in view are read at each stop. Stationary reads avoid
    motion blur / frame-lag, so the positions are accurate. Each stop captures one
    frame and decodes ALL tags in it (not a slow per-tag detection); each tag's
    pose is scaled by its configured size and mapped base <- optical <- tag like
    _read_tags_in_view. Returns {id: {'xyz': [x,y,z], 'n': sightings}} averaged.
    """
    tags = _all_configured_tags()
    robot.set_tcp(robot.tcp)
    j0 = robot.getj()
    base = j0[0]
    flange_T_optical = _flange_T_optical(robot)
    step_vel = math.radians(max(speed_deg_s, 5.0))          # move between stops; scan is at the stop
    sightings = {}                                          # id -> list of [x,y,z]

    def scan_here():
        try:
            # Let the camera settle/refocus after the move, and flush a stale
            # frame or two, before reading -- otherwise the first frame at a new
            # stop is blurry and tags are missed.
            if settle_s:
                time.sleep(settle_s)
            camera_tools._flush_frames(robot, 2)
            base_T_optical = _flange_transform(robot) @ flange_T_optical
            robot.camera.capture()
            robot.camera.decodeAT(tag_id=None)             # all tags in this frame
            dets = list(getattr(robot.camera, "AT_detections", []) or [])
            img = robot.camera.image
            if img is None or not dets:
                return
            K, dist = _camera_matrix_and_dist(robot, (img.shape[1], img.shape[0]))
            for d in dets:
                tid = getattr(d, "tag_id", None)
                if tid not in tags:
                    continue
                cam_T_tag = _calibration.apriltag_pose_from_corners(
                    d.corners, tags[tid]["size_m"], K, dist_coeffs=dist)
                if cam_T_tag is None:
                    continue
                p = (base_T_optical @ cam_T_tag)[:3, 3]
                sightings.setdefault(tid, []).append(
                    [float(p[0]), float(p[1]), float(p[2])])
        except Exception as e:                             # noqa: BLE001
            print("scan: frame skipped (%s)" % e)

    try:
        nsteps = max(1, int(round(abs(end_deg - start_deg) / float(step_deg))))
        for i in range(nsteps + 1):
            ang = start_deg + (end_deg - start_deg) * i / nsteps
            q = list(j0); q[0] = base + math.radians(ang)
            robot.movej(q, acc=0.3, vel=step_vel, wait=True)
            scan_here()
    finally:
        robot.set_tcp(robot.tcp)
    result = {}
    for tid, pts in sightings.items():
        arr = _np.array(pts)
        result[tid] = {"xyz": arr.mean(axis=0).tolist(), "n": len(pts)}
    return result


def raster_tag_scan(robot, base_from=-80.0, base_to=0.0,
                    r_inner=0.30, reach_margin=0.10, z=None, settle_s=3.0):
    """Cylindrical-coordinate raster scan for AprilTags -- the CAMERA MUST FACE
    STRAIGHT DOWN. Moves over a (base-joint angle x radial Rxy) grid at a fixed Z
    and records every configured tag's base-frame position.

    The camera must be looking straight down: this is a cylindrical scan (base
    angle + radius) of the bench below. If the camera already faces down, the scan
    just uses the camera TCP and does NOT put_camera2tcp (which would disturb the
    down orientation -- that movement is why repeat scans drifted); otherwise it
    falls back to put_camera2tcp. Face the camera down first (face_camera_down)
    for consistent results.

    Base joint from base_from to base_to; the angular step AND the radial step are
    derived from the camera's field of view so adjacent stops overlap (see
    CAMERA_FOV_OVERLAP). Radius spans r_inner..(max reachable Rxy at this Z -
    reach_margin), swept serpentine. Z defaults to the safe swing height.
    Positioning is in the CAMERA TCP. Each stop settles, captures one frame,
    decodes ALL tags and maps every tag from the CAMERA frame into BASE coordinates
    (base <- optical <- tag). Returns {id: {'xyz', 'n', 'seen_at'}}.
    """
    tags = _all_configured_tags()
    if z is None:
        z = safe_swing_z()

    # Face straight down: if it already does, just use the camera TCP -- do NOT
    # put_camera2tcp (that move disturbs the down orientation, which is what made
    # repeat scans drift). Otherwise fall back to put_camera2tcp.
    if camera_tools.camera_faces_down(robot):
        robot.set_tcp(robot.camtcp)
        print("raster_tag_scan: camera already faces down; using the camera TCP "
              "(skipping put_camera2tcp)")
    else:
        print("raster_tag_scan: camera not facing down; doing put_camera2tcp")
        robot.put_camera2tcp()
    flange_T_optical = _flange_T_optical(robot)

    # Step sizes from the camera's field of view, so adjacent stops overlap and
    # leave no gaps. Footprint on the bench at camera->bench distance d is
    # (W/f_x, H/f_y) * d; use the smaller side (conservative, orientation-agnostic)
    # and cover CAMERA_FOV_OVERLAP of it per step.
    robot.camera.capture()
    img = robot.camera.image
    W, H = (img.shape[1], img.shape[0]) if img is not None else (1280, 720)
    K, _dist = _camera_matrix_and_dist(robot, (W, H))
    fx, fy = float(K[0, 0]), float(K[1, 1])
    d = max(0.05, z - CAMERA_SCAN_BENCH_Z)                  # camera -> bench distance
    foot = min(W / fx, H / fy) * d                         # smaller footprint side (m)
    camera_tool = abs(float(robot.camtcp[2])) + 0.08       # camera tip + wrist, down
    r_outer = max_rxy_at_z(z, tool_len=camera_tool) - reach_margin

    def grid(a, b, step):
        n = max(1, int(round(abs(b - a) / float(step))))
        return [a + (b - a) * i / n for i in range(n + 1)]
    # Radius-major, INNER first then outward. The angular step is per-radius: the
    # arc at radius R must stay within the footprint, so Rxy * dtheta <= overlap *
    # foot -> dtheta = overlap * foot / R. Small R -> large angular step; large R
    # -> small step.
    radial_step = max(0.01, CAMERA_FOV_OVERLAP * foot)
    radii = (grid(r_inner, r_outer, radial_step) if r_outer > r_inner
             else [r_inner])

    def base_step_for(R):
        return max(2.0, math.degrees(CAMERA_FOV_OVERLAP * foot / max(R, 1e-6)))
    print("raster_tag_scan: Z=%.3f d=%.3f footprint=%.3f m, radial step=%.3f m, "
          "Rxy=%s (inner-first), angular step %.1f->%.1f deg (inner->outer)"
          % (z, d, foot, radial_step, ["%.3f" % r for r in radii],
             base_step_for(radii[0]), base_step_for(radii[-1])))

    sightings = {}

    def scan_here(ang, R):
        try:
            if settle_s:
                time.sleep(settle_s)
            camera_tools._flush_frames(robot, 2)
            base_T_optical = _flange_transform(robot) @ flange_T_optical  # resets to gripper TCP
            robot.camera.capture()
            robot.camera.decodeAT(tag_id=None)
            dets = list(getattr(robot.camera, "AT_detections", []) or [])
            img = robot.camera.image
            if img is None or not dets:
                return
            K, dist = _camera_matrix_and_dist(robot, (img.shape[1], img.shape[0]))
            for d in dets:
                tid = getattr(d, "tag_id", None)
                if tid not in tags:
                    continue
                cam_T_tag = _calibration.apriltag_pose_from_corners(
                    d.corners, tags[tid]["size_m"], K, dist_coeffs=dist)
                if cam_T_tag is None:
                    continue
                p = (base_T_optical @ cam_T_tag)[:3, 3]
                sightings.setdefault(tid, []).append(
                    {"xyz": [float(p[0]), float(p[1]), float(p[2])],
                     "base": ang, "R": R})
        except Exception as e:                           # noqa: BLE001
            print("raster: frame skipped (%s)" % e)

    try:
        for i, R in enumerate(radii):
            # Per-radius angular step (bigger at the inner radius), swept in the
            # opposite direction each radius step (serpentine) as it moves outward.
            angles = grid(base_from, base_to, base_step_for(R))
            if i % 2 == 1:
                angles = list(reversed(angles))
            for ang in angles:
                q = list(robot.getj())
                q[0] = math.radians(ang)
                robot.movej(q, acc=0.3, vel=0.3, wait=True)   # set the base joint, keep the wrist
                # Position the CAMERA at (Rxy, Z) along its current azimuth.
                # scan_here resets to the gripper TCP, so re-assert camtcp here.
                robot.set_tcp(robot.camtcp)
                cur = robot.get_pos()
                phi = math.atan2(cur[1], cur[0])
                robot.moveto([R * math.cos(phi), R * math.sin(phi), z])
                scan_here(ang, R)
    finally:
        robot.set_tcp(robot.tcp)                          # leave the gripper TCP active
    result = {}
    for tid, lst in sightings.items():
        arr = _np.array([s["xyz"] for s in lst])
        seen_at = sorted({(round(s["base"], 1), round(s["R"], 3)) for s in lst})
        result[tid] = {"xyz": arr.mean(axis=0).tolist(), "n": len(lst),
                       "seen_at": seen_at}
    return result


def _tag_role(tid):
    """Human label for an AprilTag id from the apriltags config -- 'flowcell 1',
    'Venton table b', 'mixer head', 'carousel', ... -- or '' if the id is not
    configured. Lets a scan report say what each tag is, not just its number."""
    nice = {'flowcell_1': 'flowcell 1', 'flowcell_2': 'flowcell 2',
            'venton_table_a': 'Venton table a', 'venton_table_b': 'Venton table b',
            'venton_table_c': 'Venton table c', 'mixer_head': 'mixer head',
            'carousel': 'carousel'}
    try:
        at = _config.get_section('apriltags') or {}
    except Exception:
        return ''
    for key, entry in at.items():
        if not isinstance(entry, dict):
            continue
        ids = set()
        if entry.get('id') is not None:
            try:
                ids.add(int(entry['id']))
            except (TypeError, ValueError):
                pass
        for i in (entry.get('ids') or []):
            try:
                ids.add(int(i))
            except (TypeError, ValueError):
                pass
        rng = entry.get('id_range')
        if rng:
            try:
                ids.update(range(int(rng[0]), int(rng[1]) + 1))
            except (TypeError, ValueError, IndexError):
                pass
        if tid in ids:
            return nice.get(key, key)
    return ''


def _rotvec_from_R(R):
    """Axis-angle rotation vector (rx, ry, rz) from a 3x3 rotation matrix --
    the UR convention, matching get_pose's orientation part."""
    ang = math.acos(max(-1.0, min(1.0, (float(_np.trace(R)) - 1.0) / 2.0)))
    if ang < 1e-9:
        return [0.0, 0.0, 0.0]
    v = _np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) \
        / (2.0 * math.sin(ang))
    return [float(x) for x in (v * ang)]


def read_tag_in_view(robot, tag_id, settle=5):
    """Detect one AprilTag in the CURRENT camera view (no motion) and return its
    base-frame pose [x, y, z, rx, ry, rz] (position + axis-angle rotation vector),
    or None if it is not seen. Uses the tag's configured size
    (DEFAULT_APRILTAG_SIZE if the id is not configured). The camera must already
    be pointing at the tag -- this does not move the arm."""
    size = (_all_configured_tags().get(tag_id) or {}).get('size_m',
                                                          DEFAULT_APRILTAG_SIZE)
    poses, _ = _read_tag_poses_in_view(robot, {tag_id: {'size_m': size}})
    if tag_id not in poses:
        return None
    T = poses[tag_id]
    p = T[:3, 3]
    rv = _rotvec_from_R(T[:3, :3])
    return [float(p[0]), float(p[1]), float(p[2]), rv[0], rv[1], rv[2]]


def read_tags_in_view(robot):
    """Read ALL configured AprilTags in the CURRENT camera view (no motion) and
    return {id: [x, y, z]} in base coordinates. One frame, decode every tag at
    once (fast -- no per-tag settle), each scaled by its configured size. The
    camera must already be pointing where you want to read; this does not move
    the arm."""
    cfg = _all_configured_tags()
    robot.set_tcp(robot.tcp)
    base_T_optical = _flange_transform(robot) @ _flange_T_optical(robot)
    camera_tools._flush_frames(robot, 2)                # drop stale frames first
    robot.camera.capture()
    robot.camera.decodeAT(tag_id=None)                  # every tag in the frame
    dets = list(getattr(robot.camera, "AT_detections", []) or [])
    img = robot.camera.image
    out = {}
    if img is None or not dets:
        return out
    K, dist = _camera_matrix_and_dist(robot, (img.shape[1], img.shape[0]))
    for d in dets:
        tid = getattr(d, "tag_id", None)
        if tid not in cfg:
            continue
        cam_T_tag = _calibration.apriltag_pose_from_corners(
            d.corners, cfg[tid]["size_m"], K, dist_coeffs=dist)
        if cam_T_tag is None:
            continue
        p = (base_T_optical @ cam_T_tag)[:3, 3]
        out[tid] = [float(p[0]), float(p[1]), float(p[2])]
    return out


def _venton_axes_from_scan(tags):
    """Fit the LEVEL Venton table frame from raster-scan tag positions.

    Needs >=2 Venton-table fiducials (apriltags.venton_table_* -- ids with a
    table_xy_mm) present in `tags`. With the surface level the only free rotation
    is a yaw about base Z, so two or more tag positions pin the frame: a 2D rigid
    fit of the known table (x, y) onto the measured base (x, y). Returns
    {origin_m, x_axis, y_axis, normal, yaw_deg, rmse_mm, ids} in base coordinates,
    or None if fewer than two are present (or they are coincident)."""
    vt = venton_table_tags()
    seen = [t for t in sorted(tags) if t in vt]
    if len(seen) < 2:
        return None
    tc = _np.array([[vt[t]['table_xy_mm'][0] / 1000.0,
                     vt[t]['table_xy_mm'][1] / 1000.0] for t in seen])
    bc = _np.array([[tags[t]['xyz'][0], tags[t]['xyz'][1]] for t in seen])
    tcm, bcm = tc.mean(axis=0), bc.mean(axis=0)
    tcc, bcc = tc - tcm, bc - bcm
    if _np.linalg.norm(tcc) < 1e-9 or _np.linalg.norm(bcc) < 1e-9:
        return None
    num = float(_np.sum(tcc[:, 0] * bcc[:, 1] - tcc[:, 1] * bcc[:, 0]))
    den = float(_np.sum(tcc[:, 0] * bcc[:, 0] + tcc[:, 1] * bcc[:, 1]))
    yaw = math.atan2(num, den)
    c, s = math.cos(yaw), math.sin(yaw)
    x_axis = [c, s, 0.0]
    y_axis = [-s, c, 0.0]
    ox = float(bcm[0] - (c * tcm[0] - s * tcm[1]))
    oy = float(bcm[1] - (s * tcm[0] + c * tcm[1]))
    oz = float(_np.mean([tags[t]['xyz'][2] for t in seen]))
    res = []
    for i in range(len(seen)):
        px = ox + c * tc[i, 0] - s * tc[i, 1]
        py = oy + s * tc[i, 0] + c * tc[i, 1]
        res.append(math.hypot(px - bc[i, 0], py - bc[i, 1]))
    rmse_mm = (sum(r * r for r in res) / len(res)) ** 0.5 * 1000.0
    return {'origin_m': [ox, oy, oz], 'x_axis': x_axis, 'y_axis': y_axis,
            'normal': [0.0, 0.0, 1.0], 'yaw_deg': math.degrees(yaw),
            'rmse_mm': rmse_mm, 'ids': seen}


def _to_venton_frame(xyz, vframe):
    """Base-frame xyz -> Venton-table-frame xyz: v = R^T (base - origin), with
    R = [x_axis | y_axis | normal]."""
    o, x, y, n = (vframe['origin_m'], vframe['x_axis'], vframe['y_axis'],
                  vframe['normal'])
    d = [xyz[i] - o[i] for i in range(3)]
    return [sum(x[i] * d[i] for i in range(3)),
            sum(y[i] * d[i] for i in range(3)),
            sum(n[i] * d[i] for i in range(3))]


def _venton_classify(tags, vframe, tol_m=0.015):
    """Classify scanned tags by LOCATION using the Venton-frame references stored
    in mixer_head_refs.json (venton_frame_refs). Transforms each carousel / mixer-
    head tag into the current scan's Venton frame and compares to the recorded
    locations, so a carousel is identified as 'on the mixing station' by where it
    is -- bench-anchored, independent of which mixer-head tag (25/26) is present.

    Returns (lines, mixer_carousel_id, carousel_xyz, sample_xyz). Empty/None when
    there is no Venton frame (fewer than two Venton tags) or no recorded refs."""
    import json
    if not vframe:
        return [], None, None, None
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'json', 'mixer_head_refs.json')
    if not os.path.exists(path):
        return [], None, None, None
    try:
        refs = (json.load(open(path, encoding='utf-8')).get('venton_frame_refs') or {})
    except Exception:
        return [], None, None, None
    car_ref = refs.get('carousel_on_mixer_venton_xyz')
    ms_ref = refs.get('mixing_station_mixerhead_venton_xyz')
    cl_ref = refs.get('mixer_on_cleaning_station_venton_xyz')

    def d2(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    car_ids = apriltag_ids('carousel_on_mixer') or set()
    head_ids = apriltag_ids('mixer_station') or set()
    lines = ["Venton-frame classification (bench-anchored, tol %d mm):" % (tol_m * 1000)]
    cid = cxyz = sxyz = None
    best = None
    for t in sorted(tags):
        if t in car_ids and car_ref:
            v = _to_venton_frame(tags[t]['xyz'], vframe)
            dd = d2(v, car_ref)
            on = dd <= tol_m
            lines.append("  carousel %d: venton (%.3f, %.3f) -> %s mixing station "
                         "(%.1f mm from ref)" % (t, v[0], v[1],
                         "ON the" if on else "not on the", dd * 1000))
            if on and (best is None or dd < best[0]):
                best = (dd, t)
    if best is not None:
        cid = best[1]
        cxyz = list(tags[cid]['xyz'])
        sxyz = [cxyz[0], cxyz[1] + SAMPLE_OFFSET_Y, cxyz[2]]
    for t in sorted(tags):
        if t in head_ids:
            v = _to_venton_frame(tags[t]['xyz'], vframe)
            where = "position not recognized"
            if ms_ref and d2(v, ms_ref) <= tol_m:
                where = "MIXING station"
            elif cl_ref and d2(v, cl_ref) <= tol_m:
                where = "CLEANING station"
            lines.append("  mixer-head tag %d: venton (%.3f, %.3f) -> %s"
                         % (t, v[0], v[1], where))
    return lines, cid, cxyz, sxyz


def _mixer_model_summary(tags):
    """Using the stored mixer-area model (json/mixer_head_refs.json), fill in all
    four reference points from whatever model tag the scan saw -- mixing station
    (mixer-head tag), carousel on mixer, sample on mixer station, and mixer on
    cleaning station -- labelling each measured or derived. Returns summary lines
    ([] if there is no model file). All offsets are relative to the mixer-head tag
    on the mixing station (the model anchor)."""
    import json
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'json', 'mixer_head_refs.json')
    if not os.path.exists(path):
        return []
    try:
        m = json.load(open(path, encoding='utf-8'))
    except Exception:
        return []
    head_tag = int(m.get('mixer_head_tag', 26))
    car_off = (m.get('carousel_on_mixer') or {}).get('pos_offset_from_mixing_tag26')
    car_abs = (m.get('carousel_on_mixer') or {}).get('tag_pose')
    samp_off = (m.get('sample_on_mixer_station') or {}).get('pos_offset_from_carousel_tag')
    clean_off = (m.get('mixer_on_cleaning_station') or {}).get('pos_offset_from_mixing_tag26')

    def dist(a, b):
        return (sum((a[i] - b[i]) ** 2 for i in range(3))) ** 0.5

    # Anchor = the mixing-station mixer-head tag position. Measured directly if the
    # head tag is in view; otherwise derived from a carousel seen at the model's
    # carousel-on-mixer location (so any single visible model tag anchors it).
    anchor, anchor_measured = None, False
    if head_tag in tags:
        anchor, anchor_measured = list(tags[head_tag]['xyz']), True
    elif car_off and car_abs:
        for t in sorted(tags):
            if 31 <= t <= 40 and dist(tags[t]['xyz'], car_abs[:3]) < 0.025:
                anchor = [tags[t]['xyz'][i] - car_off[i] for i in range(3)]
                break

    lines = ["mixer-area model (from mixer_head_refs.json) -- all four points:"]
    if anchor is None:
        lines.append("  no model anchor in view (need mixer-head tag %d or the "
                     "carousel on the mixer)." % head_tag)
        return lines
    lines.append("  mixing station (tag %d): (%.4f, %.4f, %.4f) m  [%s]" % (
        head_tag, anchor[0], anchor[1], anchor[2],
        "measured" if anchor_measured else "derived"))
    car = None
    if car_off:
        car = [anchor[i] + car_off[i] for i in range(3)]
        meas = any(31 <= t <= 40 and dist(tags[t]['xyz'], car) < 0.025 for t in tags)
        lines.append("  carousel on mixer: (%.4f, %.4f, %.4f) m  [%s]" % (
            car[0], car[1], car[2], "measured" if meas else "derived"))
    if car and samp_off:
        s = [car[i] + samp_off[i] for i in range(3)]
        lines.append("  sample on mixer station: (%.4f, %.4f, %.4f) m  [derived]"
                     % (s[0], s[1], s[2]))
    if clean_off:
        cl = [anchor[i] + clean_off[i] for i in range(3)]
        lines.append("  mixer on cleaning station: (%.4f, %.4f, %.4f) m  [derived]"
                     % (cl[0], cl[1], cl[2]))
    else:
        lines.append("  mixer on cleaning station: not recorded yet (put the head "
                     "there and read tag %d)." % head_tag)
    return lines


def locate_mixer_carousel(robot, base_from=-100.0, base_to=20.0):
    """Raster-scan for tags, identify the carousel on the mixer, store its tag
    centre, and compute the sample position.

    The carousel on the mixer sits at MIXER_CAROUSEL_DX / _DY from the mixer-head
    tag; a carousel tag (31-40) within MIXER_CAROUSEL_MATCH_TOL of (mixer head +
    that offset) is taken as the one on the mixer. Its tag centre is stored
    (state.set_carousel_mixer_seat) so sample calcs reuse it, and the sample is
    SAMPLE_OFFSET_Y along base +Y from the centre. No robot move beyond the scan.

    Returns {'tags', 'mixer_carousel_id', 'carousel_xyz', 'sample_xyz',
    'summary'}. mixer_carousel_id is None (and nothing is stored) when no carousel
    tag is within tolerance.
    """
    tags = raster_tag_scan(robot, base_from, base_to)
    head_ids = apriltag_ids('mixer_station') or set()
    car_ids = apriltag_ids('carousel_on_mixer') or set()
    heads = {t: tags[t]['xyz'] for t in tags if t in head_ids}
    cars = {t: tags[t]['xyz'] for t in tags if t in car_ids}

    lines = ["locate_mixer_carousel: found %d tag(s):" % len(tags)]
    for t in sorted(tags):
        x, y, z = tags[t]['xyz']
        role = _tag_role(t)
        tag_label = "%2d%s" % (t, (" (%s)" % role) if role else "")
        lines.append("  tag %s at (%.4f, %.4f, %.4f) m" % (tag_label, x, y, z))

    result = {'tags': tags, 'mixer_carousel_id': None,
              'carousel_xyz': None, 'sample_xyz': None, 'venton_frame': None}

    # Venton table axes, when its fiducials are in the scan (level fit).
    vframe = _venton_axes_from_scan(tags)
    if vframe:
        result['venton_frame'] = vframe
        lines.append("Venton table axes (level fit, tags %s): origin "
                     "(%.4f, %.4f, %.4f) m, yaw %.3f deg, x_axis [%.5f, %.5f, "
                     "%.5f], y_axis [%.5f, %.5f, %.5f] (rmse %.1f mm)" % (
                         vframe['ids'], vframe['origin_m'][0],
                         vframe['origin_m'][1], vframe['origin_m'][2],
                         vframe['yaw_deg'],
                         vframe['x_axis'][0], vframe['x_axis'][1], vframe['x_axis'][2],
                         vframe['y_axis'][0], vframe['y_axis'][1], vframe['y_axis'][2],
                         vframe['rmse_mm']))

    # Fill in all four mixer-area points from whatever model tag was seen.
    lines += _mixer_model_summary(tags)

    # Primary identification: Venton-frame classification (bench-anchored) -- it
    # places the carousel on the mixing station by LOCATION in the fixed Venton
    # frame, so it works regardless of which mixer-head tag (25/26) is present and
    # does not rely on the -65/-65 estimate. The -65/-65 block below is the
    # fallback when no Venton frame / refs are available.
    vclass_lines, vcid, vcxyz, vsxyz = _venton_classify(tags, vframe)
    if vclass_lines:
        lines += vclass_lines
    if vcid is not None:
        state.set_carousel_mixer_seat(vcxyz)
        result.update(mixer_carousel_id=vcid, carousel_xyz=vcxyz, sample_xyz=vsxyz)
        lines.append("carousel %d is ON THE MIXING STATION (Venton-frame match) -- "
                     "stored as the carousel seat; sample at (%.4f, %.4f, %.4f) m."
                     % (vcid, vsxyz[0], vsxyz[1], vsxyz[2]))
        result['summary'] = "\n".join(lines)
        return result

    if not heads:
        lines.append("no mixer-head tag (%s) in view -- cannot locate the mixer "
                     "carousel." % sorted(head_ids))
        result['summary'] = "\n".join(lines)
        return result
    if not cars:
        lines.append("no carousel tag (%s) in view." % sorted(car_ids))
        result['summary'] = "\n".join(lines)
        return result

    # Nearest carousel to any mixer-head expected point (head + offset). Checking
    # both 25 and 26 is robust to which mixer-head tag the raster happened to see.
    best = None   # (dist, car_id, head_id)
    for ht, (hx, hy, _hz) in heads.items():
        ex, ey = hx + MIXER_CAROUSEL_DX, hy + MIXER_CAROUSEL_DY
        for ct, (cx, cy, _cz) in cars.items():
            d = math.hypot(cx - ex, cy - ey)
            if best is None or d < best[0]:
                best = (d, ct, ht)
    dist, cid, hid = best
    if dist <= MIXER_CAROUSEL_MATCH_TOL:
        cx, cy, cz = tags[cid]['xyz']
        state.set_carousel_mixer_seat([cx, cy, cz])
        sx, sy, sz = cx, cy + SAMPLE_OFFSET_Y, cz
        result.update(mixer_carousel_id=cid, carousel_xyz=[cx, cy, cz],
                      sample_xyz=[sx, sy, sz])
        lines.append("carousel ON THE MIXER: tag %d at (%.4f, %.4f, %.4f) m "
                     "(%.1f mm from mixer-head %d + offset) -- stored as the "
                     "carousel seat." % (cid, cx, cy, cz, dist * 1000, hid))
        lines.append("sample position (tag + %.0f mm +Y): (%.4f, %.4f, %.4f) m"
                     % (SAMPLE_OFFSET_Y * 1000, sx, sy, sz))
    else:
        lines.append("no carousel within %.0f mm of the expected mixer offset "
                     "(nearest: tag %d, %.1f mm off vs mixer-head %d). Nothing "
                     "stored." % (MIXER_CAROUSEL_MATCH_TOL * 1000, cid,
                                  dist * 1000, hid))
    result['summary'] = "\n".join(lines)
    return result


def measure_tag_heights(robot, tags_xyz, z=None, only_ids=None, put_tcp2cam=True):
    """Bump the gripper straight down at each tag's X,Y to measure the actual
    surface Z, and return {id: measured_surface_z} (base frame).

    `tags_xyz` is {id: [x, y, z]} (e.g. from raster_tag_scan). `only_ids` limits it
    to those ids (e.g. to finish a subset after a stop). Operator recipe: with
    put_tcp2cam, put the TCP to the camera first; then ALWAYS rise to the safe
    swing Z before any lateral move, go over each tag at that Z, and bump down
    (gripper fully shut) to feel the real surface. Pass put_tcp2cam=False when
    continuing after a protective stop so it does not re-descend onto a fixture.
    """
    if z is None:
        z = safe_swing_z()
    # Tap with the GRIPPER tip pointing straight down: make the gripper TCP active,
    # rise to the safe swing Z, then Zalign so the tool Z axis is level-down. Only
    # then is a straight-down bump safe -- a tilted tip catches fixture edges and
    # trips a protective stop (put_tcp2cam is accepted for back-compat but unused;
    # the camera TCP is wrong for a bump).
    robot.set_tcp(robot.tcp)                      # gripper frame for the bump/height
    cur = robot.get_pos()
    robot.moveto([cur[0], cur[1], z])             # lift straight up before any lateral move
    robot.Zalign()                                # gripper tip straight down (keeps xy, yaw)
    measured = {}
    ids = [t for t in sorted(tags_xyz) if (only_ids is None or t in only_ids)]
    for tid in ids:
        x, y = float(tags_xyz[tid][0]), float(tags_xyz[tid][1])
        robot.moveto([x, y, z])                   # over the tag at the safe swing Z (stays down)
        close_gripper_fully(robot)                # clean fingertip contact
        robot.bump(z=-1, backoff=0.0)             # feel the real surface, straight down
        measured[tid] = float(robot.get_pos()[2])
        print("measure_tag_heights: tag %d surface z = %.4f m" % (tid, measured[tid]))
        robot.moveto([x, y, z])                   # back up to the safe swing Z
    return measured


def _survey_targets(scan_tags):
    """Pick the survey targets from a raster scan, each as (id, role): the mixer
    head (whichever of 25/26 is present), the carousel on the mixing station
    (Venton-classified, else the carousel nearest the mixer head), and flowcells
    1 and 2. Only ids actually in the scan are returned."""
    targets = []
    head_ids = apriltag_ids('mixer_station') or set()
    head = None
    for t in sorted(scan_tags):
        if t in head_ids:
            targets.append((t, 'mixer head'))
            head = scan_tags[t]['xyz']
            break
    # carousel on the mixer: Venton classification first, else nearest to the head
    _, cid, _, _ = _venton_classify(scan_tags, _venton_axes_from_scan(scan_tags))
    if cid is None and head is not None:
        car_ids = apriltag_ids('carousel_on_mixer') or set()
        cars = [t for t in scan_tags if t in car_ids]
        if cars:
            cid = min(cars, key=lambda t: (scan_tags[t]['xyz'][0] - head[0]) ** 2
                      + (scan_tags[t]['xyz'][1] - head[1]) ** 2)
    if cid is not None:
        targets.append((cid, 'carousel on mixer'))
    for fid, role in ((1, 'flowcell 1'), (2, 'flowcell 2')):
        if fid in scan_tags:
            targets.append((fid, role))
    return targets


def survey_key_tags(robot, scan_tags, z=None):
    """After a raster scan, visit the mixer head, the carousel on the mixing
    station, and flowcells 1 & 2; for each, center the camera on the tag (the
    search/centre procedure) and bump the gripper straight down to measure the
    true surface Z. Returns {id: {'role', 'centered_xyz', 'bumped_z'}}. Rises to
    the safe swing Z before every lateral move; reports, stores nothing.

    `scan_tags` is {id: {'xyz': [...]}} from raster_tag_scan -- the coarse
    positions used as the go-to hint for each tag."""
    if z is None:
        z = safe_swing_z()
    sizes = _all_configured_tags()
    results = {}
    for tid, role in _survey_targets(scan_tags):
        hint = scan_tags[tid]['xyz']
        # 1. Safe: rise straight up, bring the camera face-down, go over the hint.
        robot.set_tcp(robot.tcp)
        cur = robot.get_pos()
        robot.moveto([cur[0], cur[1], z])
        if not camera_tools.camera_faces_down(robot):
            robot.put_camera2tcp()
        robot.set_tcp(robot.camtcp)
        robot.moveto([hint[0], hint[1], z])          # camera over the tag at safe Z
        # 2. Search/centre on this tag.
        robot.camera.AT_physical_size = (sizes.get(tid) or {}).get('size_m',
                                                                   DEFAULT_APRILTAG_SIZE)
        robot.camera.AT_preferred_id = tid
        robot.center_camera2apriltag(tag_id=tid, tol_m=CAROUSEL_CENTER_TOL_M,
                                     settle=CAROUSEL_CENTER_SETTLE,
                                     max_iter=CAROUSEL_CENTER_ITERS)
        # 3. Accurate centred position (base frame) from the current view.
        pose = read_tag_in_view(robot, tid)
        centered = pose[:3] if pose else list(hint[:3])
        # 4. Gripper straight down at the centred X,Y, bump to feel the surface.
        robot.set_tcp(robot.tcp)
        cur = robot.get_pos()
        robot.moveto([cur[0], cur[1], z])            # rise before the frame-shift lateral move
        robot.moveto([centered[0], centered[1], z])  # gripper over the tag at safe Z
        robot.Zalign()                               # tip straight down
        close_gripper_fully(robot)
        robot.bump(z=-1, backoff=0.0)
        bz = float(robot.get_pos()[2])
        robot.moveto([centered[0], centered[1], z])  # rise clear
        results[tid] = {'role': role,
                        'centered_xyz': [round(centered[0], 5), round(centered[1], 5),
                                         round(centered[2], 5)],
                        'bumped_z': round(bz, 5)}
        print("survey: %s (tag %d) centred (%.4f, %.4f), bump Z %.4f m"
              % (role, tid, centered[0], centered[1], bz))
    return results


def _read_tag_poses_in_view(robot, tags):
    """Like _read_tags_in_view, but keep each tag's full base pose (4x4) and
    hand back the base<-optical transform used.

    Returns (poses, base_T_optical) where poses = {id: 4x4 base_T_tag}. The
    position part of each pose is exactly what _read_tags_in_view returns; the
    rotation part is the tag's orientation in base, which the level fit uses for
    the in-plane heading (and which position-only fits deliberately ignore
    because it is bimodal). No motion. base_T_optical is returned so a caller
    can project base points back into the same image (the red origin dot).
    """
    robot.set_tcp(robot.tcp)
    poses = {}
    base_T_optical = None
    try:
        base_T_flange = _flange_transform(robot)      # restores tool TCP
        flange_T_optical = _flange_T_optical(robot)
        base_T_optical = base_T_flange @ flange_T_optical
        for tid, info in tags.items():
            robot.camera.AT_physical_size = info['size_m']
            robot.camera.AT_preferred_id = tid
            r = camera_tools._detect_apriltag(robot, tag_id=tid)
            if r is None:
                continue
            img = robot.camera.image
            K, dist = _camera_matrix_and_dist(robot, (img.shape[1], img.shape[0]))
            cam_T_tag = _calibration.apriltag_pose_from_corners(
                r.corners, info['size_m'], K, dist_coeffs=dist)
            if cam_T_tag is None:
                continue
            poses[tid] = base_T_optical @ cam_T_tag
    finally:
        robot.set_tcp(robot.tcp)        # always leave the gripper TCP active
    return poses, base_T_optical


def _yaw_from_positions(poses, tags):
    """Table yaw (rad) from the tag POSITIONS alone, under the level assumption.

    A 2D rigid fit of the known table (x, y) onto the measured base (x, y): with
    the surface level the only free rotation is a yaw about Z, so two or more tag
    positions pin it. Returns None if the points are coincident. This is the
    robust heading -- it does not touch the bimodal tag orientation.
    """
    ids = sorted(poses)
    tc = _np.array([[tags[t]['table_xy_mm'][0] / 1000.0,
                     tags[t]['table_xy_mm'][1] / 1000.0] for t in ids])
    bc = _np.array([[float(poses[t][0, 3]), float(poses[t][1, 3])] for t in ids])
    tc = tc - tc.mean(axis=0)
    bc = bc - bc.mean(axis=0)
    if _np.linalg.norm(tc) < 1e-9 or _np.linalg.norm(bc) < 1e-9:
        return None
    num = float(_np.sum(tc[:, 0] * bc[:, 1] - tc[:, 1] * bc[:, 0]))
    den = float(_np.sum(tc[:, 0] * bc[:, 0] + tc[:, 1] * bc[:, 1]))
    return math.atan2(num, den)


def calibrate_venton_table_level(robot, tag_ids=None, heading='orientation'):
    """Estimate the Venton table frame assuming the surface is LEVEL.

    ``heading`` picks where the one free rotation (yaw about Z) comes from:

      * ``'orientation'`` -- each tag's own measured +X axis (what the operator
        asked for). Honest but only as good as solvePnP's orientation on a flat
        tag, which is bimodal; when it is unreliable the per-tag residual and the
        gap to the position-derived yaw below both blow up, so a bad fit is loud.
      * ``'positions'`` -- the tag positions alone (``_yaw_from_positions``). The
        robust path: it never touches the shaky tag orientation, and two tags are
        enough once the surface is assumed level.

    This is the two-tag path: the three-point position fit (estimate_venton_table
    / calibrate_venton_table) needs three non-collinear tags because two points
    only ever give a line. Here the missing information comes from two added
    assumptions instead of a third tag:

      * the table is level, so the surface normal is base +Z and the frame's
        only free rotation is a yaw about Z;
      * each flat-lying tag's +X axis runs along the table +X, so the tag's own
        orientation gives that yaw directly.

    For every visible tag the tag's +X axis is projected onto the horizontal
    plane and its azimuth taken; the azimuths are averaged (circular mean) into
    one table yaw. The origin is each tag's base position minus the yaw-rotated
    table offset, averaged over the tags. One tag is enough in principle; two or
    more average down the per-tag noise and give a residual.

    Returns a fit dict shaped like estimate_venton_table's (origin_m, x_axis,
    y_axis, normal, scale=1.0, rmse_mm, max_error_mm, tags) plus 'method',
    'yaw_deg', the per-tag 'headings_deg', and 'position_yaw_deg' -- the yaw the
    tag *positions* alone imply, as an independent cross-check on the tag-X ==
    table-X mounting assumption (a big gap between it and yaw_deg means the tags
    are not mounted square to the table). No motion.
    """
    tags = venton_table_tags()
    if tag_ids is not None:
        want = [int(t) for t in tag_ids]
        tags = {t: tags[t] for t in want if t in tags}
        missing = [t for t in want if t not in tags]
        if missing:
            raise RuntimeError("no configured Venton fiducial for id(s) %s" % missing)
    if not tags:
        raise RuntimeError("no Venton fiducials configured")

    poses, _ = _read_tag_poses_in_view(robot, tags)
    if len(poses) < 2:
        raise RuntimeError(
            "saw only %d usable Venton fiducial(s) in the current view; need at "
            "least 2 for the level/orientation fit -- aim the camera so two tags "
            "are in frame and in focus" % len(poses))

    # Per-tag yaw from each tag's +X axis flattened onto the horizontal plane --
    # computed either way, because it is reported as a per-tag diagnostic even
    # when the robust position yaw is the one used.
    headings = {}
    for tid, T in poses.items():
        x_axis = _np.array(T[:3, 0], dtype=float)
        x_axis[2] = 0.0
        if _np.linalg.norm(x_axis) < 1e-6:
            continue            # tag edge-on: its yaw is undefined, skip it
        headings[tid] = math.atan2(float(x_axis[1]), float(x_axis[0]))

    pos_yaw = _yaw_from_positions(poses, tags)
    if heading == 'positions':
        if pos_yaw is None:
            raise RuntimeError("tag positions are coincident; cannot derive a "
                               "heading from them")
        yaw = pos_yaw
    elif heading == 'orientation':
        if not headings:
            raise RuntimeError("no tag gave a usable horizontal heading")
        # Circular mean of the per-tag headings.
        sin_sum = sum(math.sin(h) for h in headings.values())
        cos_sum = sum(math.cos(h) for h in headings.values())
        yaw = math.atan2(sin_sum, cos_sum)
    else:
        raise ValueError("heading must be 'orientation' or 'positions', got %r"
                         % (heading,))
    c, s = math.cos(yaw), math.sin(yaw)
    R = _np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def table_offset_m(txy):
        return R @ _np.array([txy[0] / 1000.0, txy[1] / 1000.0, 0.0])

    # Origin: average of (tag position - rotated table offset).
    origins = [_np.array(T[:3, 3], float) - table_offset_m(tags[tid]['table_xy_mm'])
               for tid, T in poses.items()]
    origin = _np.mean(origins, axis=0)

    # Per-tag residual: how far each measured position is from the fitted frame.
    tags_out, errs = [], []
    for tid in sorted(poses):
        p = _np.array(poses[tid][:3, 3], float)
        pred = origin + table_offset_m(tags[tid]['table_xy_mm'])
        err = float(_np.linalg.norm(p - pred))
        errs.append(err)
        tags_out.append({'id': tid, 'name': tags[tid]['name'],
                         'table_xy_mm': list(tags[tid]['table_xy_mm']),
                         'base_m': [float(v) for v in p],
                         'error_mm': err * 1000.0})
    rmse_mm = math.sqrt(sum(e * e for e in errs) / len(errs)) * 1000.0
    max_error_mm = max(errs) * 1000.0

    # The position-derived yaw is always reported as a cross-check: when the
    # orientation heading is in use, a big gap to this means the tag-X == table-X
    # mounting assumption (or solvePnP's orientation) is off.
    position_yaw_deg = None if pos_yaw is None else (
        (math.degrees(pos_yaw) + 180.0) % 360.0 - 180.0)

    return {'method': 'level+' + heading,
            'origin_m': origin, 'x_axis': R[:, 0], 'y_axis': R[:, 1],
            'normal': R[:, 2], 'scale': 1.0,
            'rmse_mm': rmse_mm, 'max_error_mm': max_error_mm,
            'yaw_deg': math.degrees(yaw),
            'headings_deg': {t: math.degrees(h) for t, h in headings.items()},
            'position_yaw_deg': position_yaw_deg,
            'tags': tags_out}


def render_point_on_camera(robot, base_xyz, path, base_T_optical=None,
                           radius=9, label=None):
    """Capture the current camera frame, project the base-frame point
    ``base_xyz`` (metres) into it, draw a red dot there, and save a PNG to
    ``path``. No motion.

    Returns (path, (u, v)) with the pixel the point landed on, or (path, None)
    if it falls behind the camera or outside the frame (the dot is still drawn
    when it is merely near an edge, so the operator can see which way it is off).
    ``base_T_optical`` may be passed in to reuse the transform a calibration
    already computed; otherwise it is read fresh from the current pose.
    """
    import cv2
    from PIL import Image, ImageDraw

    robot.set_tcp(robot.tcp)
    if base_T_optical is None:
        base_T_flange = _flange_transform(robot)
        base_T_optical = base_T_flange @ _flange_T_optical(robot)

    robot.camera.capture()
    img = _np.array(robot.camera.image)                 # RGB (H, W, 3)
    K, dist = _camera_matrix_and_dist(robot, (img.shape[1], img.shape[0]))

    optical_T_base = _np.linalg.inv(base_T_optical)
    pt = _np.array([float(base_xyz[0]), float(base_xyz[1]), float(base_xyz[2]), 1.0])
    in_cam = optical_T_base @ pt
    rvec, _ = cv2.Rodrigues(optical_T_base[:3, :3])
    tvec = optical_T_base[:3, 3]
    uv, _ = cv2.projectPoints(
        _np.array([[pt[0], pt[1], pt[2]]], dtype=float), rvec, tvec, K,
        _np.zeros(5) if dist is None else _np.asarray(dist, float))
    u, v = (float(x) for x in uv.reshape(2))

    h, w = img.shape[:2]
    pil = Image.fromarray(img)
    draw = ImageDraw.Draw(pil)
    if in_cam[2] > 0:       # only draw a point that is actually in front of the lens
        draw.ellipse([u - radius, v - radius, u + radius, v + radius],
                     fill=(255, 0, 0), outline=(255, 255, 255))
        if label:
            draw.text((u + radius + 3, v - radius), str(label), fill=(255, 0, 0))
    pil.save(path)
    on_frame = bool(in_cam[2] > 0 and 0 <= u < w and 0 <= v < h)
    return path, ((u, v) if on_frame else None)


def _project_base_points(base_T_optical, K, dist, points):
    """Project base-frame 3D points into the image. Returns (uv, in_front):
    uv is a list of [u, v] pixels, in_front a list of bools (False = behind the
    lens, where the projection is meaningless)."""
    import cv2
    optical_T_base = _np.linalg.inv(base_T_optical)
    rvec, _ = cv2.Rodrigues(optical_T_base[:3, :3])
    tvec = optical_T_base[:3, 3]
    objp = _np.array([[float(p[0]), float(p[1]), float(p[2])] for p in points],
                     dtype=float)
    uv, _ = cv2.projectPoints(objp, rvec, tvec, K,
                              _np.zeros(5) if dist is None else _np.asarray(dist, float))
    uv = uv.reshape(-1, 2)
    in_front = []
    for p in points:
        cp = optical_T_base @ _np.array([float(p[0]), float(p[1]), float(p[2]), 1.0])
        in_front.append(bool(cp[2] > 0))
    return [[float(u), float(v)] for u, v in uv], in_front


def venton_axes_overlay(robot, frame, image_wh, axis_len_m=0.05):
    """Pixel positions of the Venton frame's origin and its X/Y/Z axis tips in
    the current view, for drawing the coordinate axes on the live image. No
    motion.

    ``frame`` carries origin_m/x_axis/y_axis/normal in base coordinates (a
    calibration result or the stored one); ``image_wh`` is the (width, height) of
    the frame the overlay will be drawn on, so the camera matrix matches it.
    Returns {origin, x, y, z: [u, v]} plus 'in_front' (per point) and the
    'axis_len_m' used. Computed once; the caller freezes it until the operator
    turns the overlay off, so it does not track the arm.
    """
    robot.set_tcp(robot.tcp)
    base_T_flange = _flange_transform(robot)
    base_T_optical = base_T_flange @ _flange_T_optical(robot)
    K, dist = _camera_matrix_and_dist(robot, (int(image_wh[0]), int(image_wh[1])))
    o = _np.array([float(v) for v in frame['origin_m']])
    x = _np.array([float(v) for v in frame['x_axis']])
    y = _np.array([float(v) for v in frame['y_axis']])
    z = _np.array([float(v) for v in frame['normal']])
    uv, in_front = _project_base_points(
        base_T_optical, K, dist,
        [o, o + axis_len_m * x, o + axis_len_m * y, o + axis_len_m * z])
    return {'origin': uv[0], 'x': uv[1], 'y': uv[2], 'z': uv[3],
            'in_front': in_front, 'axis_len_m': axis_len_m,
            'image_wh': [int(image_wh[0]), int(image_wh[1])]}


#: Where the last calibrated Venton frame is kept, so the axes overlay survives a
#: server restart without re-running a calibration.
_VENTON_FRAME_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'json', 'venton_frame.json')


def save_venton_frame(frame, path=_VENTON_FRAME_PATH):
    """Persist a calibrated Venton frame (origin + axes, base coordinates)."""
    import json
    data = {k: [float(v) for v in frame[k]]
            for k in ('origin_m', 'x_axis', 'y_axis', 'normal')}
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    return path


def load_venton_frame(path=_VENTON_FRAME_PATH):
    """The stored Venton frame dict, or None if none has been saved."""
    try:
        import json
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


#: Azimuths in the base XY plane for the named robot directions, for pointing
#: the camera's image-up (north) axis along one of them.
CAMERA_AZIMUTH = {'+x': 0.0, '+y': 90.0, '-x': 180.0, '-y': -90.0}


def align_camera_north(robot, target_az_deg=0.0):
    """Spin the camera in place so its image-up (north) axis points at
    `target_az_deg` in the base XY plane (0 = +X, 90 = +Y, -90 = -Y, 180 = -X),
    keeping it looking at the same spot. Returns (before, after) north azimuths
    in degrees.

    Rotated about the CAMERA's own axis (camtcp Z), not base Z: a base-Z
    rotation orbits the whole arm about the base and, for anything but a tiny
    angle, swings the camera off the table. Rotating about the camera axis
    spins the image in place and keeps the tags in frame. North's azimuth in
    the base XY plane is atan2(ny, nx). The camera axis points down (camtcp Z
    is ~-Z base), so a positive rotz about it DECREASES that azimuth -- verified
    on the robot, a +89 deg rotz moved north from -179 to +92. So the turn that
    lands north on the target is rotz(azimuth - target), normalized to the
    shortest direction so it never spins the long way round.
    """
    _camvec, northv, _eastv = robot.get_camera_vector()
    azimuth = math.degrees(math.atan2(float(northv[1]), float(northv[0])))
    delta = (azimuth - target_az_deg + 180.0) % 360.0 - 180.0   # shortest spin
    robot.set_tcp(robot.camtcp)
    robot.rotz(delta, coordinate='tcp')
    robot.set_tcp(robot.tcp)
    _camvec, northv, _eastv = robot.get_camera_vector()
    after = math.degrees(math.atan2(float(northv[1]), float(northv[0])))
    return azimuth, after


def goto_xyz(robot, xyz):
    """Move the tool TCP to absolute base-frame `xyz` (metres), keeping the
    current orientation. A general positioning move -- e.g. hovering over a
    calibrated Venton origin. Straight-line (movel) from wherever the arm is, so
    the caller is responsible for the path being clear. Returns the pose."""
    robot.set_tcp(robot.tcp)
    robot.moveto([float(xyz[0]), float(xyz[1]), float(xyz[2])],
                 acc=placement_accel, vel=placement_speed)
    return list(robot.get_pos())


def rotate_base_z(robot, degrees):
    """Rotate the arm about the base Z axis by `degrees`. Orbits the TCP about
    the base -- a large angle moves the camera a long way. Here mainly to undo
    such an orbit; align_camera_north spins in place instead."""
    robot.set_tcp(robot.tcp)
    robot.rotz(float(degrees), coordinate='base')
    return list(robot.get_pos())


def estimate_venton_table(robot, with_scale=True):
    """Quick, no-motion estimate of the Venton table frame in base coordinates.

    Reads the fiducials from the current view open-loop (_read_tags_in_view)
    and fits the transform -- no centring, no descending, no guard. Returns the
    same fit dict as calibrate_venton_table (origin_m, x_axis, y_axis, normal,
    scale, rmse_mm, tags), but it is an ESTIMATE: the open-loop read carries
    the camera mount-geometry error, so rmse_mm (and the scale) report how
    rough it is. For an accurate frame use calibrate_venton_table.
    """
    tags = venton_table_tags()
    if len(tags) < 3:
        raise RuntimeError("need at least 3 configured Venton fiducials; have %d"
                           % len(tags))
    view = _read_tags_in_view(robot, tags)
    if len(view) < 3:
        raise RuntimeError("saw only %d of %d fiducials in the current view; "
                           "aim the camera so all three are visible"
                           % (len(view), len(tags)))
    measured, source_mm, target_m = [], [], []
    for tid in sorted(view):
        info = tags[tid]
        source_mm.append([info['table_xy_mm'][0], info['table_xy_mm'][1], 0.0])
        target_m.append(view[tid])
        measured.append({'id': tid, 'name': info['name'],
                         'table_xy_mm': info['table_xy_mm'], 'base_m': view[tid]})
    fit = _calibration.table_frame_in_base(source_mm, target_m, with_scale=with_scale)
    fit['tags'] = measured
    return fit


#: Poses for hand-eye data collection, each applied relative to the start pose:
#: a tool-frame rotation ('rot', axis, deg) or a base translation ('xyz', dx,
#: dy, dz). The rotations give the orientation variety the hand-eye solve needs;
#: the translations help the translation part. Kept small so tag 13 stays in
#: view (rotations pivot about the tool TCP, so the camera barely shifts).
HANDEYE_ROT_DEG = 15.0
HANDEYE_TRANS_M = 0.05
HANDEYE_POSES = (
    ('home',),
    ('rot', 'x', HANDEYE_ROT_DEG), ('rot', 'x', -HANDEYE_ROT_DEG),
    ('rot', 'y', HANDEYE_ROT_DEG), ('rot', 'y', -HANDEYE_ROT_DEG),
    ('rot', 'z', HANDEYE_ROT_DEG), ('rot', 'z', -HANDEYE_ROT_DEG),
    ('xyz', HANDEYE_TRANS_M, 0.0, 0.0), ('xyz', -HANDEYE_TRANS_M, 0.0, 0.0),
    ('xyz', 0.0, HANDEYE_TRANS_M, 0.0), ('xyz', 0.0, -HANDEYE_TRANS_M, 0.0),
    ('xyz', 0.0, 0.0, HANDEYE_TRANS_M), ('xyz', 0.0, 0.0, -HANDEYE_TRANS_M),
)


def _flange_transform(robot):
    """base -> flange (tool0) as a 4x4, by reading the pose with a zero TCP.
    camtcp is expressed relative to this flange, so this is the frame the
    hand-eye X (flange -> camera) is solved in."""
    robot.set_tcp([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    try:
        fv = list(robot.get_pose().get_pose_vector())
    finally:
        robot.set_tcp(robot.tcp)
    return _calibration.pose_vector_to_transform(fv)


#: After a move the IP camera refocuses, so the first frames decode blurred or
#: not at all. The ChArUco capture re-grabs for up to this long, taking the
#: first frame that yields enough corners, before giving up on a pose.
CHARUCO_REFOCUS_S = 4.0


#: Calibrated camera intrinsics live here (git-ignored measured state, like the
#: other *_positions files). Written by calibrate_camera_intrinsics, read by the
#: ChArUco pose so it uses the real focal length and distortion instead of the
#: assumed f.
_INTRINSICS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'json', 'camera_intrinsics.json')


def load_camera_intrinsics():
    """The stored intrinsics dict, or None if never calibrated."""
    try:
        import json
        with open(_INTRINSICS_PATH) as f:
            return json.load(f)
    except Exception:
        return None


def save_camera_intrinsics(data):
    import json
    with open(_INTRINSICS_PATH, 'w') as f:
        json.dump(data, f, indent=2)


def apply_camera_intrinsics(robot):
    """Apply the stored calibrated focal length to the whole system, so every
    distance measurement (AprilTag QRdistance, descends, the Venton read) uses
    it instead of the assumed f.

    The camera recomputes self.camera_f from a module-level base on every
    capture (urcamera.capture: camera_f / default_imgH * frame_width), so a
    one-off instance override would be wiped the next frame. Instead the module
    base is set so that recompute lands on the calibrated focal length at the
    calibrated frame width. The ChArUco pose already uses the full stored matrix
    and is unaffected. Returns the applied focal length, or None if no
    intrinsics are stored. Safe to call at startup.
    """
    intr = load_camera_intrinsics()
    if not intr or not intr.get('fx'):
        return None
    import sys
    f_eff = 0.5 * (float(intr['fx']) + float(intr['fy']))
    wh = intr.get('image_wh') or [getattr(robot.camera, 'imgH', 640), 0]
    width = float(wh[0]) or 640.0
    cam_mod = sys.modules[type(robot.camera).__module__]
    # capture() does: self.camera_f = cam_mod.camera_f / cam_mod.default_imgH * frame_width
    cam_mod.camera_f = f_eff * float(cam_mod.default_imgH) / width
    robot.camera.camera_f = f_eff
    print("PAL12idb: applied calibrated focal length %.1f px (was assumed ~810) "
          "to the camera; distance measurements now use it." % f_eff)
    return f_eff


def _camera_matrix_and_dist(robot, image_wh):
    """(camera_matrix 3x3, dist_coeffs) to use for ChArUco pose: the calibrated
    intrinsics if present (and matching this frame size), else a rough pinhole
    from the assumed focal length. Warns once if the stored size differs."""
    intr = load_camera_intrinsics()
    if intr and intr.get('camera_matrix'):
        stored_wh = intr.get('image_wh')
        if stored_wh and tuple(stored_wh) != tuple(image_wh):
            print("PAL12idb: WARNING camera intrinsics were calibrated at %s "
                  "but frames are %s; the focal length will not match."
                  % (stored_wh, list(image_wh)))
        return (_np.asarray(intr['camera_matrix'], float),
                _np.asarray(intr.get('dist_coeffs') or [], float))
    return (_calibration.camera_matrix_from_f(robot.camera.camera_f, image_wh), None)


def _charuco_board_and_f(robot):
    spec = _config.get_section('charuco')
    board = _calibration.make_charuco_board(spec)
    return board, spec, float(robot.camera.camera_f)


def _charuco_gray(robot):
    """Current camera frame as grayscale, or None. Captures if idle."""
    import cv2
    if not robot.camera._running:
        robot.camera.capture()
    img = robot.camera.image
    if img is None:
        return None
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) if img.ndim == 3 else img


def _detect_charuco_in_base(robot, board, min_corners, refocus_s):
    """Capture (retrying through the post-move refocus) and return
    (cam_T_board 4x4, n_corners), or (None, best_n) if the board never came in
    sharp enough. Uses the calibrated intrinsics when available."""
    deadline = time.time() + refocus_s
    best_n = 0
    while True:
        gray = _charuco_gray(robot)
        if gray is not None:
            K, dist = _camera_matrix_and_dist(robot, (gray.shape[1], gray.shape[0]))
            T, n = _calibration.charuco_board_pose(
                gray, board, K, dist_coeffs=dist, min_corners=min_corners)
            best_n = max(best_n, n)
            if T is not None:
                return T, n
        if time.time() >= deadline:
            return None, best_n
        time.sleep(0.3)         # let the camera refocus, then look again


#: Poses for intrinsics calibration: the board should appear across the image at
#: varied angles AND distances (the Z steps), which is what constrains focal
#: length and distortion. Relative to the start pose; same ('rot'/'xyz') format.
INTRINSICS_POSES = (
    ('home',),
    ('rot', 'x', 15.0), ('rot', 'x', -15.0),
    ('rot', 'y', 15.0), ('rot', 'y', -15.0),
    ('rot', 'z', 12.0), ('rot', 'z', -12.0),
    ('xyz', 0.06, 0.0, 0.0), ('xyz', -0.06, 0.0, 0.0),
    ('xyz', 0.0, 0.06, 0.0), ('xyz', 0.0, -0.06, 0.0),
    ('xyz', 0.0, 0.0, 0.07), ('xyz', 0.0, 0.0, -0.05),   # distance variety
    ('xyz', 0.05, 0.05, 0.0), ('xyz', -0.05, -0.05, 0.05),
)


def _apply_relative_pose(robot, home, hx, hy, hz, pose):
    """Move to one HANDEYE/INTRINSICS pose entry relative to home."""
    robot.set_tcp(robot.tcp)
    robot.set_pose(home)
    kind = pose[0]
    if kind == 'rot':
        _, axis, deg = pose
        {'x': robot.rotx, 'y': robot.roty, 'z': robot.rotz}[axis](deg, coordinate='tcp')
    elif kind == 'xyz':
        _, dx, dy, dz = pose
        if dx or dy or dz:
            robot.moveto([hx + dx, hy + dy, hz + dz])


def _detect_charuco_corners_refocus(robot, board, min_corners, refocus_s):
    """(charuco_corners, charuco_ids, n) once the board is sharp, or (None,
    None, best_n) after `refocus_s`. Waits out the post-move camera refocus."""
    deadline = time.time() + refocus_s
    best = (None, None, 0)
    while True:
        gray = _charuco_gray(robot)
        if gray is not None:
            cc, ci = _calibration.detect_charuco_corners(gray, board)
            n = 0 if ci is None else len(ci)
            if n > best[2]:
                best = (cc, ci, n)
            if n >= min_corners:
                return cc, ci, n
        if time.time() >= deadline:
            return None, None, best[2]
        time.sleep(0.3)


def collect_charuco_corner_views(robot, poses=INTRINSICS_POSES, settle_s=1.5,
                                 refocus_s=CHARUCO_REFOCUS_S):
    """Drive to each pose and collect (charuco_corners, charuco_ids) for
    intrinsics. Returns (views, image_wh). Leaves the arm at the start pose."""
    board, spec, _f = _charuco_board_and_f(robot)
    min_corners = int(spec.get('min_corners', 6))
    robot.set_tcp(robot.tcp)
    home = robot.get_pose()
    hx, hy, hz = float(home.pos[0]), float(home.pos[1]), float(home.pos[2])
    views, image_wh = [], None
    try:
        for pose in poses:
            _apply_relative_pose(robot, home, hx, hy, hz, pose)
            time.sleep(settle_s)
            gray = _charuco_gray(robot)
            if gray is not None and image_wh is None:
                image_wh = (gray.shape[1], gray.shape[0])
            cc, ci, n = _detect_charuco_corners_refocus(robot, board, min_corners, refocus_s)
            label = pose[0] if len(pose) == 1 else "%s" % (pose,)
            if ci is None:
                print("PAL12idb: intrinsics pose %s -- board not sharp (best %d), "
                      "skipped" % (label, n))
                continue
            views.append((cc, ci))
            print("PAL12idb: intrinsics pose %-18s %d corners" % (label, n))
    finally:
        robot.set_tcp(robot.tcp)
        robot.set_pose(home)
    return views, (image_wh or (640, 480))


def calibrate_camera_intrinsics(robot):
    """Calibrate the camera matrix + distortion from ChArUco views at varied
    poses, store them (camera_intrinsics.json), and return the result dict. The
    stored intrinsics are then used by every ChArUco pose, replacing the assumed
    focal length -- which is what the hand-eye solve and the distance scale need.

    Moves the robot. Leave the board comfortably in view first."""
    board, spec, _f = _charuco_board_and_f(robot)
    views, image_wh = collect_charuco_corner_views(robot)
    result = _calibration.calibrate_intrinsics(
        views, image_wh, board, min_corners=int(spec.get('min_corners', 6)))
    result['assumed_f'] = float(robot.camera.camera_f)
    save_camera_intrinsics(result)
    apply_camera_intrinsics(robot)      # take effect now, no restart needed
    return result


def collect_handeye_charuco(robot, poses=HANDEYE_POSES, settle_s=1.5,
                            refocus_s=CHARUCO_REFOCUS_S):
    """Like collect_handeye_samples but the target is the ChArUco board, whose
    pose is unambiguous. Returns (flanges, cams, labels). Waits out the camera
    refocus after each move before trusting a frame."""
    board, spec, _f = _charuco_board_and_f(robot)
    min_corners = int(spec.get('min_corners', 6))
    robot.set_tcp(robot.tcp)
    home = robot.get_pose()
    hx, hy, hz = float(home.pos[0]), float(home.pos[1]), float(home.pos[2])
    flanges, cams, labels = [], [], []
    try:
        for pose in poses:
            robot.set_tcp(robot.tcp)
            robot.set_pose(home)
            kind = pose[0]
            if kind == 'rot':
                _, axis, deg = pose
                {'x': robot.rotx, 'y': robot.roty, 'z': robot.rotz}[axis](
                    deg, coordinate='tcp')
            elif kind == 'xyz':
                _, dx, dy, dz = pose
                if dx or dy or dz:
                    robot.moveto([hx + dx, hy + dy, hz + dz])
            time.sleep(settle_s)
            C, n = _detect_charuco_in_base(robot, board, min_corners, refocus_s)
            label = pose[0] if len(pose) == 1 else "%s" % (pose,)
            if C is None:
                print("PAL12idb: hand-eye pose %s -- ChArUco not sharp enough "
                      "(best %d corners), skipped" % (label, n))
                continue
            flanges.append(_flange_transform(robot))
            cams.append(C)
            labels.append(label)
            print("PAL12idb: hand-eye pose %-6s ChArUco %d corners" % (label, n))
    finally:
        robot.set_tcp(robot.tcp)
        robot.set_pose(home)
    return flanges, cams, labels


def collect_handeye_samples(robot, tag_id, poses=HANDEYE_POSES, settle_s=1.0):
    """Drive to each pose and record (base->flange, camera->tag) for `tag_id`.

    Returns (flanges, cams, labels): matched lists of 4x4 transforms and the
    pose labels that produced them. A pose where the tag is not detected (or has
    no solvePnP pose) is skipped. Leaves the arm at the start pose, tool TCP
    active. The camera->tag pose is the detector's solvePnP output (optical
    frame: x right, y down, z forward), scaled by the tag's configured size.
    """
    tags = venton_table_tags()
    size_m = tags.get(tag_id, {}).get('size_m', DEFAULT_APRILTAG_SIZE)
    robot.set_tcp(robot.tcp)
    home = robot.get_pose()
    hx, hy, hz = float(home.pos[0]), float(home.pos[1]), float(home.pos[2])
    flanges, cams, labels = [], [], []
    try:
        for pose in poses:
            robot.set_tcp(robot.tcp)
            robot.set_pose(home)
            kind = pose[0]
            if kind == 'rot':
                _, axis, deg = pose
                {'x': robot.rotx, 'y': robot.roty, 'z': robot.rotz}[axis](
                    deg, coordinate='tcp')
            elif kind == 'xyz':
                _, dx, dy, dz = pose
                if dx or dy or dz:
                    robot.moveto([hx + dx, hy + dy, hz + dz])
            time.sleep(settle_s)
            robot.camera.AT_physical_size = size_m
            robot.camera.AT_preferred_id = tag_id
            r = camera_tools._detect_apriltag(robot, tag_id=tag_id)
            if r is None or getattr(r, 'pose_R', None) is None:
                print("PAL12idb: hand-eye pose %s -- tag %d not usable, skipped"
                      % (pose[0] if len(pose) == 1 else pose, tag_id))
                continue
            C = _np.eye(4)
            C[:3, :3] = _np.asarray(r.pose_R, float)
            C[:3, 3] = _np.asarray(r.pose_t, float).ravel()
            flanges.append(_flange_transform(robot))
            cams.append(C)
            labels.append(pose[0] if len(pose) == 1 else "%s" % (pose,))
    finally:
        robot.set_tcp(robot.tcp)
        robot.set_pose(home)
    return flanges, cams, labels


def run_handeye_calibration(robot, tag_id=13, target='charuco'):
    """Collect target views from varied poses and solve for camtcp.

    target='charuco' (default) uses the ChArUco board -- an unambiguous,
    many-corner pose, far better than a single AprilTag (whose bimodal pose
    gave a 3.5 deg residual). target='apriltag' falls back to `tag_id`.

    Returns a dict: `camtcp` (the solved [x,y,z,rx,ry,rz] flange->camera pose
    vector), `current_camtcp` (what the robot uses now, for comparison),
    `samples`, and the solve's rotation residual. Does NOT write the result --
    it is reported for review; apply it by editing the robot's camtcp.

    Moves the robot (small rotations/translations about the start pose). Leave
    the target comfortably in view first.
    """
    if target == 'charuco':
        flanges, cams, labels = collect_handeye_charuco(robot)
        what = "the ChArUco board"
    else:
        flanges, cams, labels = collect_handeye_samples(robot, tag_id)
        what = "tag %d" % tag_id
    if len(flanges) < 3:
        raise RuntimeError(
            "only %d usable views of %s; need >= 3 (and varied orientation). "
            "Aim the camera so the target stays in view across small tilts."
            % (len(flanges), what))
    out = _calibration.solve_hand_eye(flanges, cams)
    camtcp = _calibration.transform_to_pose_vector(out['X'])
    return {'camtcp': [round(v, 6) for v in camtcp],
            'current_camtcp': [round(float(v), 6) for v in robot.camtcp],
            'samples': len(flanges),
            'rot_residual_deg_mean': round(out['rot_residual_deg_mean'], 4),
            'rot_residual_deg_max': round(out['rot_residual_deg_max'], 4),
            'pairs': out['pairs'], 'labels': labels}


def measure_venton_tag(robot, tag_id):
    """Closed-loop measure one Venton fiducial from the current pose, for the
    operator-assisted calibration: the operator jogs the camera so the tag sits
    near the image centre, then this centres precisely, descends close,
    re-centres and projects (_measure_tag_in_base).

    Near-centre to start keeps the centring move small (the large move from an
    off-centre tag is what failed the fully automatic version), and ending
    directly over the tag is what makes the result independent of where the
    camera was -- the property the open-loop estimate lacked. Returns the base
    [x, y, z] in metres, or None if the tag could not be acquired.
    """
    tags = venton_table_tags()
    if tag_id not in tags:
        raise RuntimeError("tag %d is not a configured Venton fiducial %s"
                           % (tag_id, sorted(tags)))
    return _measure_tag_in_base(robot, tag_id, tags[tag_id]['size_m'])


def venton_fit_from_measurements(measurements, with_scale=True):
    """Fit the table frame from {id: base_xyz} points collected one at a time
    by measure_venton_tag. Needs >= 3 non-collinear tags. No residual guard:
    the operator is iterating and sees rmse_mm in the reply to judge it."""
    tags = venton_table_tags()
    source_mm, target_m, used = [], [], []
    for tid in sorted(measurements):
        if tid not in tags:
            continue
        info = tags[tid]
        source_mm.append([info['table_xy_mm'][0], info['table_xy_mm'][1], 0.0])
        target_m.append(measurements[tid])
        used.append({'id': tid, 'name': info['name'],
                     'table_xy_mm': info['table_xy_mm'],
                     'base_m': [round(float(v), 6) for v in measurements[tid]]})
    if len(used) < 3:
        raise RuntimeError("have %d measured tags; need 3 non-collinear "
                           "(measure each with measure_venton_tag <id>)" % len(used))
    fit = _calibration.table_frame_in_base(source_mm, target_m, with_scale=with_scale)
    fit['tags'] = used
    return fit


#: Camera poses to read the fiducials from, each a small translation (metres,
#: base frame) off wherever the arm starts. Deliberately ONLY +/-50 mm along X,
#: Y and Z -- no descent to a standoff (that dropped the tags out of view) and
#: no large centring move (that would not execute). Small bounded moves that
#: run reliably, with every tag staying in frame at each. The first entry is
#: the start pose itself.
VENTON_CALIB_POSES = (
    ('home', 0.0, 0.0, 0.0),
    ('+X30', 0.03, 0.0, 0.0),
    ('-X30', -0.03, 0.0, 0.0),
    ('+Y30', 0.0, 0.03, 0.0),
    ('-Y30', 0.0, -0.03, 0.0),
    ('+Z30', 0.0, 0.0, 0.03),      # up only -- never step down toward the table
)

#: Pause after each move before reading, so the camera has refocused.
VENTON_SETTLE_S = 1.0


def calibrate_venton_table(robot, poses=VENTON_CALIB_POSES, with_scale=True,
                           settle_s=VENTON_SETTLE_S):
    """Find the Venton table's surface frame in robot base coordinates.

    Reads the configured fiducials (venton_table_tags) open-loop from several
    small camera offsets (`poses`: +/-50 mm along X, Y, Z from the start pose,
    no descent, no centring) and fits one transform over all the readings
    (calibration.table_frame_in_base). Each tag seen at each pose adds a
    correspondence; the fit averages them, and the per-tag spread across poses
    is reported as a quality number.

    Open-loop here because the closed-loop path cannot run on this setup: the
    descent to a standoff drops the tags out of view, and a large centring move
    does not complete. These small +/-50 mm moves do run reliably and keep all
    tags in frame. The reading still carries the camera mount-geometry error,
    so the result is an estimate -- rmse_mm and the per-tag spread say how
    rough -- not a taught-accuracy frame.

    Returns the fit dict (origin_m, x_axis, y_axis, normal, scale, rmse_mm) plus
    'tags'. Raises RuntimeError only if fewer than three tags were seen across
    all poses; no residual guard, so the operator sees the numbers and judges.

    MOVES the robot -- only the small offsets above, returning to the start pose
    at the end.
    """
    tags = venton_table_tags()
    if len(tags) < 3:
        raise RuntimeError(
            "need at least 3 configured Venton fiducials (id + table_xy_mm); "
            "have %d" % len(tags))
    # The TCP is the camera centre (camtcp) for the moves, so the +/-50 mm
    # offsets translate the camera itself -- a clean, known camera baseline
    # between views rather than a tool move that drags the camera along an arc.
    # Re-asserted before every move because _read_tags_in_view switches back to
    # the tool TCP internally (get_camera_vector rebuilds the view direction
    # from the tool orientation).
    robot.set_tcp(robot.camtcp)
    home = robot.get_pose()                      # camera-centre pose
    hx, hy, hz = float(home.pos[0]), float(home.pos[1]), float(home.pos[2])
    per_tag = {tid: [] for tid in tags}
    try:
        for label, dx, dy, dz in poses:
            robot.set_tcp(robot.camtcp)             # moves are of the camera centre
            robot.set_pose(home)                    # full return, then one small offset
            if dx or dy or dz:
                robot.moveto([hx + dx, hy + dy, hz + dz])
            time.sleep(settle_s)
            view = _read_tags_in_view(robot, tags)  # manages its own TCP for the read
            for tid, base in view.items():
                per_tag[tid].append(base)
            print("PAL12idb: Venton pose %-5s saw tags %s" % (label, sorted(view)))
    finally:
        robot.set_tcp(robot.camtcp)
        robot.set_pose(home)                        # always leave the camera where it started
        robot.set_tcp(robot.tcp)

    measured, source_mm, target_m = [], [], []
    for tid in sorted(tags):
        views = per_tag[tid]
        if not views:
            print("PAL12idb: Venton tag %d (%s) not seen from any pose"
                  % (tid, tags[tid]['name']))
            continue
        avg = [sum(v[i] for v in views) / len(views) for i in range(3)]
        spread_mm = 1000.0 * max(
            (sum((a - b) ** 2 for a, b in zip(v, avg)) ** 0.5 for v in views),
            default=0.0)
        info = tags[tid]
        source_mm.append([info['table_xy_mm'][0], info['table_xy_mm'][1], 0.0])
        target_m.append(avg)
        measured.append({'id': tid, 'name': info['name'],
                         'table_xy_mm': info['table_xy_mm'], 'base_m': avg,
                         'passes': len(views), 'spread_mm': round(spread_mm, 2)})
    if len(measured) < 3:
        raise RuntimeError(
            "saw only %d of %d configured Venton fiducials across all poses; "
            "need 3 non-collinear to fit a frame" % (len(measured), len(tags)))
    fit = _calibration.table_frame_in_base(source_mm, target_m, with_scale=with_scale)
    fit['tags'] = measured
    return fit


def venton_calibration_summary(fit):
    """A plain-float, JSON-safe view of a calibrate_venton_table result, for
    reporting over ZMQ or logging. numpy arrays become lists; nothing here
    touches the robot."""
    rnd = lambda v, n=6: [round(float(x), n) for x in v]
    return {
        'origin_m': rnd(fit['origin_m']),
        'x_axis': rnd(fit['x_axis']),
        'y_axis': rnd(fit['y_axis']),
        'normal': rnd(fit['normal']),
        'scale': round(float(fit['scale']), 6),
        'rmse_mm': round(float(fit['rmse_mm']), 3),
        'max_error_mm': round(float(fit['max_error_mm']), 3),
        'tags': [{'id': t['id'], 'name': t['name'],
                  'table_xy_mm': t['table_xy_mm'], 'base_m': rnd(t['base_m']),
                  'passes': t.get('passes'), 'spread_mm': t.get('spread_mm')}
                 for t in fit['tags']],
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

# -- protective-stop recovery ------------------------------------------------
# A protective stop does not arrive as one exception type. The robot raises it
# two different ways, from two different modules, and a stop that kills a
# running program surfaces as a third class that merely shares a name with the
# first:
#   * common.robUR.RobotException(SafetyStatus(mode)) -- the `if mode > 2` guard
#     that opens every motion wrapper in robUR, raised *before* anything moves.
#   * urxe.ursecmon.ProtectiveStopException -- raised mid-move, from the
#     secondary monitor's wait().
#   * urx.urrobot.RobotException -- "Robot stopped" / "Goal not reached ...".
#     A *different class* from robUR's despite the identical name, so catching
#     one does not catch the other.
#   * ursecmon.TimeoutException -- a program send that did not land.
# Collected here so the catch sites read as one thing. Empty tuple when
# UR_12idb is absent, which makes the isinstance() below simply never match.
_MOTION_FAULTS = ()
for _mod, _name in (('common.robUR', 'RobotException'),
                    ('urxe.ursecmon', 'ProtectiveStopException'),
                    ('urxe.ursecmon', 'TimeoutException'),
                    ('urx.urrobot', 'RobotException')):
    try:
        _exc = getattr(__import__(_mod, fromlist=[_name]), _name)
    except Exception:                      # noqa: BLE001 - absent is fine
        continue
    if isinstance(_exc, type) and issubclass(_exc, BaseException):
        _MOTION_FAULTS = _MOTION_FAULTS + (_exc,)

# How far above a seat the recovery search stands to look for its AprilTag.
# The taught pose with this added to Z, at the taught orientation -- not
# search_reference(), which is a tuned camera standoff for a teach. Recovery is
# already at the seat when it starts, so this is a straight lift off it.
#
# 100 mm, not the 200 mm first tried. The tag sits distance_gripper_tag (50 mm)
# below the taught pose, so this leaves the camera ~0.15 m off it -- close to
# the standoffs the search_refs poses use, and within reach of the 12 mm
# flowcell tags. At 200 mm the camera stood ~0.25 m off, further out than any
# tuned reference, and a 12 mm tag may not resolve from there. Being nearer
# than the 0.2 m working standoff costs nothing: descend_to_apriltag steps
# either way (step = measured - distance), so it simply backs off to it.
RECOVERY_SEARCH_LIFT = 0.10
# The lift between re-grabbing a mis-seated piece and setting it back down, so
# the second attempt comes down onto the seat instead of dragging across it.
RECOVERY_RESEAT_LIFT = 0.05
# Where the arm retreats to when recovery has given up, before the operator is
# asked to come and look. Enough to be clear of the seat, small enough not to
# be a move of its own.
RECOVERY_ABORT_LIFT = 0.10


class ProtectiveStopUnrecovered(RuntimeError):
    """A protective stop that recovery could not get past.

    Raised only after the arm has been put somewhere safe (gripper released
    where that is wanted, lifted clear). Carries
    commands.PROTECTIVE_STOP_UNRECOVERED at the front of its message, which is
    what makes the GUI raise a dialog rather than only colouring a banner.
    """


def _is_protective_stop(robot, exc):
    """True only if `exc` is a motion fault *and* the robot really is stopped.

    The exception type alone is not enough: robUR raises its RobotException for
    an unreachable pose and a TCP-set timeout as well, and "recovering" from
    one of those would unlock nothing and then drive the arm around for no
    reason. The safety mode is the authority, so it decides; the type only
    keeps unrelated failures (a lost camera, a bad station name) out.
    """
    if _MOTION_FAULTS and not isinstance(exc, _MOTION_FAULTS):
        return False
    try:
        return robot.get_safety_mode() > 2
    except Exception:                      # noqa: BLE001 - fall back to secmon
        try:
            return bool(robot.is_protective_stopped())
        except Exception:                  # noqa: BLE001 - cannot tell: not one
            return False


def _recoverable_station(station):
    """Whether `station` has an AprilTag a recovery search could find.

    False for sample_on_mixer_station: its pose is derived from the mixer
    cleaning station by a fixed offset (see _sample_on_mixer_from), and there
    is no tag on the seat itself to search for.
    """
    return station in APRILTAG_SUBJECT
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
carousel_on_mixerID = 7
carousel_in_stockID = 8
carousel_used_stackID = 9


def set_flowcell_ID(n):
    """Change which flowcell (1 or 2) the transport functions act on."""
    global flowcell_ID
    flowcell_ID = int(n)
    state.set_flowcell_in_use(flowcell_ID)


# Whether the sample-table transports drive the DAQ sample-stage motors
# (sth/sav/stv) to the recorded position. Read live from state so the toggle
# survives a restart; kept as a module global, synced by set_position_sample_
# table_motors(), the same way flowcell_ID is.
position_sample_table_motors = state.get_position_sample_table_motors()


def set_position_sample_table_motors(on):
    """Turn DAQ sample-table motor positioning on or off for the transports."""
    global position_sample_table_motors
    position_sample_table_motors = bool(on)
    state.set_position_sample_table_motors(position_sample_table_motors)

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
    carousel_on_mixerID: 'carousel_on_mixer',
    carousel_in_stockID: 'carousel_in_stock',
    carousel_used_stackID: 'carousel_used_stack',
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


# The transfer point was a fixed module constant (transfer_point); it can now be
# taught, and a taught value is kept in waypoints.ini next to the station
# positions -- its own section rather than a waypoint ID, since it is not one.
# The module constant stays as the default used until it is taught.
_TRANSFER_POINT_SECTION = 'transfer_point'


def get_transfer_point():
    """The transfer point pose: the taught value from waypoints.ini if present,
    else the module default. Read live (like get_position) so a re-teach takes
    effect without a restart."""
    parser = configparser.ConfigParser()
    parser.read(_INI_PATH)
    if parser.has_section(_TRANSFER_POINT_SECTION):
        try:
            return [parser.getfloat(_TRANSFER_POINT_SECTION, f)
                    for f in _POSITION_FIELDS]
        except (ValueError, configparser.NoOptionError):
            pass        # malformed entry -> fall back to the default below
    return list(transfer_point)


def store_transfer_point(pose):
    """Persist the transfer point pose to waypoints.ini. Merges into the file so
    the station positions beside it survive."""
    parser = configparser.ConfigParser()
    parser.read(_INI_PATH)
    if not parser.has_section(_TRANSFER_POINT_SECTION):
        parser.add_section(_TRANSFER_POINT_SECTION)
    for f, v in zip(_POSITION_FIELDS, pose):
        parser.set(_TRANSFER_POINT_SECTION, f, repr(float(v)))
    with open(_INI_PATH, 'w') as fp:
        parser.write(fp)
    return list(pose)


def record_current_transfer_point(robot):
    """Record where the arm is standing now as the transfer point.

    Read with the gripper TCP active, like record_current_position, so the
    stored pose is in the same frame goto_transfer_point and
    move_via_transferpoint drive to. Unlike a station teach, the transfer point
    is a pose the arm goes *to* directly -- no descent is applied from it -- so
    what is jogged to is exactly what is stored.
    """
    robot.set_tcp(robot.tcp)
    pose = robot.get_pose().get_pose_vector().tolist()
    return store_transfer_point(pose)


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

def get_carousel_on_mixer_position():
    return get_position(carousel_on_mixerID)

def set_carousel_on_mixer_position(pos):
    set_position(carousel_on_mixerID, pos)

def get_carousel_in_stock_position():
    return get_position(carousel_in_stockID)

def set_carousel_in_stock_position(pos):
    set_position(carousel_in_stockID, pos)

def get_carousel_used_stack_position():
    return get_position(carousel_used_stackID)

def set_carousel_used_stack_position(pos):
    set_position(carousel_used_stackID, pos)

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
    'carousel_on_mixer': lambda: carousel_on_mixerID,
    'carousel_in_stock': lambda: carousel_in_stockID,
    'carousel_used_stack': lambda: carousel_used_stackID,
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


#: Robotiq position counts, 0 fully open and 255 fully closed. Fixed by the
#: gripper's protocol, not a preference.
GRIPPER_OPEN_COUNT = 0
GRIPPER_CLOSE_COUNT = 255          # Robotiq count for a fully-shut gripper
GRIPPER_CLOSED_COUNT = 255


def _open_extra(key):
    """A `gripper.<key>_open_extra_m` from config, read live so it can be
    retuned without a restart. 0.0 when absent -- an unset widening means the
    ordinary release(), not a failure."""
    return float(_config.get_section('gripper').get(key, 0.0) or 0.0)


def cleaning_station_open_extra():
    """How much wider than release() to open before a flowcell cleaning
    station pickup. The flowcell stands in a deep seat and the fingers pass
    down either side of it."""
    return _open_extra('cleaning_station_open_extra_m')


def mixer_head_open_extra():
    """How much wider than release() to open before taking the mixer head.

    Separate from the flowcell's widening rather than sharing one number: the
    two are avoiding different obstructions, so a change made for one seat
    should not silently move the other.
    """
    return _open_extra('mixer_head_open_extra_m')


def carousel_open_extra():
    """How much wider than release() to open before grabbing a carousel. The
    config default (0.05 m, the full stroke) clamps to a full open in
    pickup_open_count -- the fingers reach around the carousel rim."""
    return _open_extra('carousel_open_extra_m')


def pickup_open_count(extra_m=None):
    """The gripper count to descend at, `extra_m` wider than robot.release().

    The Hand-E takes a position count and reports nothing back, so "wider than
    the current opening" cannot be measured -- it is computed from the opening
    robUR.release() commands (`gripper.release_count`) and the finger travel
    per count (`gripper.stroke_m` / 255).

    Returns None when no widening is asked for, meaning "just call release()".
    Clamped to fully open: asking for more travel than the gripper has is a
    number being wrong, not a reason to refuse to pick the flowcell up.
    """
    section = _config.get_section('gripper')
    if extra_m is None:
        extra_m = float(section.get('cleaning_station_open_extra_m', 0.0) or 0.0)
    if extra_m <= 0:
        return None
    stroke_m = float(section.get('stroke_m', 0.05) or 0.05)
    release_count = float(section.get('release_count', 120))
    counts_per_m = GRIPPER_CLOSED_COUNT / stroke_m
    return int(max(GRIPPER_OPEN_COUNT,
                   min(GRIPPER_CLOSED_COUNT,
                       round(release_count - extra_m * counts_per_m))))


def open_gripper(robot, extra_m=None):
    """Open the gripper, optionally `extra_m` wider than robot.release().

    Falls back to a plain release() when there is no widening to do, or when
    the robot object exposes no way to command a raw count -- a simulated or
    older robot should still open its gripper, just not by a tuned amount.
    """
    count = pickup_open_count(extra_m)
    gripper = getattr(robot, 'gripper', None)
    action = getattr(gripper, 'gripper_action', None)
    if count is None or action is None:
        robot.release()
        return None
    action(count)
    return count


def close_gripper_fully(robot):
    """Fully shut the gripper (count 255), for a clean fingertip contact.

    grab()/close_gripper() is a force close that can stop short of fully shut --
    fine for gripping an object, but when we bump the closed fingertips onto a
    surface to measure its height we want them all the way together. Command the
    full-close count directly; fall back to grab() on a robot that exposes no raw
    count (a simulated or older robot still closes, just not to a tuned amount).
    """
    gripper = getattr(robot, 'gripper', None)
    action = getattr(gripper, 'gripper_action', None)
    if action is None:
        robot.grab()
        return
    action(GRIPPER_CLOSE_COUNT)


def pickup(robot, height = needle_clear_height, open_extra_m = 0.0):
    """Descend, close on the piece, and lift clear.

    `open_extra_m` opens the fingers that much wider than release() does
    before descending. Used when dropping onto the flowcell cleaning station,
    where the fingers pass down either side of a flowcell standing in a deep
    seat and a wider gap is the difference between clearing it and nudging it.
    Zero everywhere else, which is the original release()-and-descend.
    """
    open_gripper(robot, open_extra_m)
    # cm go deeper from the standard height
    robot.mvr2z(-grab_depth-distance_gripper_tag)
    robot.grab()
    # move high enough so the needle is cleared
    robot.mvr2z(height)

def dropdown_at(robot, station_pos, acc=None, vel=None,
                release_above=dropdown_release_above):
    """Descend to `station_pos`'s release height and let go.

    The release Z is `release_above` m above where pickup() grabs -- pickup()
    descends distance_gripper_tag + grab_depth from the taught pose, so this is
    (taught Z - distance_gripper_tag) - grab_depth + release_above. The default
    is 5 mm; the flowcell cleaning station passes 2 mm
    (cleaning_station_release_above).

    The full taught pose is sent rather than a relative Z move, so where the
    flowcell is set down does not depend on the clearance it was carried in
    at. No bump: feeling for contact was releasing high, because anything the
    gripper brushes on the way -- a lip on the station, a slightly cocked
    flowcell, the arm's own deceleration -- reads as arrival.
    """
    p = list(station_pos)
    p[2] = p[2]-distance_gripper_tag-grab_depth+release_above
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
             by_bump = False, release_above = dropdown_release_above):
    # Release onto a station. Given the destination's taught pose, this
    # descends to a computed Z (dropdown_at); without one it falls back to a
    # relative move down from wherever the caller left the arm, which is only
    # right if that was the travel height `height` describes.
    #
    # by_bump feels for the seat instead: the taught Z decides only where the
    # fast part of the descent stops, not where the gripper opens. release_above
    # is the computed-Z release height above the grab point (dropdown_at); it has
    # no effect when by_bump is set.
    if by_bump:
        return dropdown_by_bump(robot, station_pos)
    if station_pos is not None:
        return dropdown_at(robot, station_pos, release_above=release_above)
    # move down
    robot.mvr2z(-1*(height+grab_depth-0.005))
    robot.release()
    # move back up to standard height
    robot.mvr2z(distance_gripper_tag)

def dropdown_sampletable(robot):
    # The sample table drop, at half speed: it is the one placement onto the
    # beamline stage, open-loop, so anything the taught position is off by is
    # taken up by the hardware. Halving the approach is cheap -- a 0.06 m move
    # once per sample. Releases 1 mm above the grab point, lower than the default,
    # to set the flowcell down gently onto the stage.
    return dropdown_at(robot, get_sampletable_position(),
                       acc=placement_accel, vel=placement_speed,
                       release_above=sampletable_release_above)


# -- protective-stop recovery -------------------------------------------------
# A protective stop used to end a transport outright: the exception propagated,
# @_tracks marked the piece's location unknown, and the operator had to unlock
# the robot by hand and re-teach. These sequences try to get past one on their
# own, and when they cannot, put the arm somewhere safe and fetch someone.
#
# Recovery is scoped to the single pickup or dropdown that failed, never to the
# transport as a whole. Re-running a transport would be wrong twice over: the
# recovered pose is deliberately not stored, so the retry would go straight back
# to the taught pose the stop just disproved; and at a target-end stop the piece
# is already at the target, so the retry would drive back to p1 and pick up air.
#
# One attempt, then stop. A protective stop raised *during* recovery is not
# recovered again -- whatever tripped it is evidently still there.
#
# What locate_apriltag leaves behind matters here, and is not obvious: it ends
# with put_tcp2camera(), which puts the *gripper tip* where the *camera* was --
# one camera standoff above the tag -- so its trailing grab() closes on empty
# air and its bump() only touches the piece and backs off. It returns parked on
# the reference pose with the fingers closed and empty. That is exactly the
# state _move_leg() leaves for pickup(), which opens the fingers before
# descending, so a search composes straight into a pickup.

def _stop_is_cleared(robot):
    """True if the robot is out of protective stop (safety mode 1 or 2)."""
    try:
        return robot.get_safety_mode() <= 2
    except Exception:                      # noqa: BLE001 - fall back to secmon
        try:
            return not robot.is_protective_stopped()
        except Exception:                  # noqa: BLE001 - cannot tell
            return False


def _release_and_require_clear(robot, station):
    """robot.release_after_unlock(), then check it actually worked.

    release_after_unlock (robUR.py:711) prints and carries on when the unlock
    fails, and the release() it then sends is URScript the controller discards
    while stopped -- so it can return normally having neither cleared the stop
    nor opened the gripper. Recovery would go on believing the fingers were
    open when they are still clamped on the piece, so the mode is re-read here
    and a stop that did not clear ends the attempt.
    """
    robot.release_after_unlock()
    if not _stop_is_cleared(robot):
        raise RuntimeError("the protective stop did not clear, so the gripper "
                           "release was discarded by the controller")


def _abort_protective_stop(robot, station, reason, release=True):
    """Put the arm somewhere safe, then raise ProtectiveStopUnrecovered.

    Always raises. `release` opens the gripper first, which is what is wanted
    at a seat (the piece is on or nearly on it) and, by the operator's choice,
    mid-travel too. Every step is best-effort and individually reported: this
    runs *because* something already went wrong, so a failure to lift must not
    replace the diagnosis with a new one -- but the operator does need to know
    whether the arm really lifted and whether the fingers really opened.
    """
    done = []
    try:
        robot.unlock_protective_stop()
        done.append("cleared the stop" if _stop_is_cleared(robot)
                    else "could NOT clear the stop")
    except Exception as e:                 # noqa: BLE001
        done.append("could NOT clear the stop (%s)" % e)
    if release:
        try:
            robot.release_after_unlock()
            done.append("opened the gripper" if _stop_is_cleared(robot)
                        else "gripper release was DISCARDED (still stopped)")
        except Exception as e:             # noqa: BLE001
            done.append("could NOT open the gripper (%s)" % e)
    else:
        done.append("left the gripper CLOSED -- it may still hold the piece")
    try:
        robot.mvr2z(RECOVERY_ABORT_LIFT)
        done.append("lifted %.2f m" % RECOVERY_ABORT_LIFT)
    except Exception as e:                 # noqa: BLE001
        done.append("could NOT lift (%s) -- THE ARM HAS NOT MOVED" % e)
    raise ProtectiveStopUnrecovered(
        "%s at %s: %s. The robot then: %s. Check the station and whatever the "
        "arm was carrying before running anything else."
        % (_commands.PROTECTIVE_STOP_UNRECOVERED,
           _commands.STATION_LABELS.get(station, station or "an unnamed position"),
           reason, "; ".join(done)))


def _recovery_search_pose(station):
    """The taught seat pose lifted RECOVERY_SEARCH_LIFT, orientation kept.

    Not search_reference(): that substitutes a camera standoff and attitude
    tuned for teaching a station from across the cell. Recovery is already
    standing on the seat, so this is a straight lift off it and nothing else
    moves.
    """
    above = list(get_station_position(station))
    above[2] = above[2] + RECOVERY_SEARCH_LIFT
    return above


def _search_seat_again(robot, station):
    """Stand over `station` and re-find its AprilTag, without storing anything.

    Returns the found pose. store=False is the whole point: a recovery search
    is a guess made under duress, and letting it overwrite a taught position in
    waypoints.ini would quietly replace a good teach with it.
    """
    above = _recovery_search_pose(station)
    robot.set_tcp(robot.tcp)               # an aborted search can leave camtcp live
    robot.moveto(above)                    # at the seat already, so effectively a lift
    return locate_apriltag(robot, pos=station, ref_pos=above, store=False,
                           via_transferpoint=False)


def _recover_at_start(robot, station, height, open_extra_m = 0.0):
    """Protective stop while picking up: re-find the seat, then pick up.

    Unlock, stand RECOVERY_SEARCH_LIFT over the taught seat, search for its
    AprilTag from there, and pick up from the pose that finds. locate_apriltag
    leaves the arm on that pose with the fingers closed and empty, which is
    exactly where pickup() expects to begin, so the pickup is the ordinary one.
    """
    if not robot.unlock_protective_stop():
        _abort_protective_stop(robot, station,
                               "the protective stop would not clear")
    if not _recoverable_station(station):
        _abort_protective_stop(
            robot, station,
            "it carries no AprilTag, so there is nothing to search for")
    print("PAL12idb: recovering at %s -- standing %.2f m over the seat to look "
          "for its AprilTag." % (station, RECOVERY_SEARCH_LIFT))
    found = _search_seat_again(robot, station)
    pickup(robot, height=height, open_extra_m=open_extra_m)
    return found


def _recover_at_target(robot, station, station_pos, place):
    """Protective stop while setting down: let go, re-find the piece, re-seat it.

    Unlock and open the gripper (the piece is at or nearly at its seat), back
    off RECOVERY_SEARCH_LIFT, and search for the tag from over the taught seat.
    Then pick the piece back up, lift RECOVERY_RESEAT_LIFT, and set it down at
    the *taught* pose -- so the second attempt lands where the first was aiming
    rather than wherever the stop left things.

    `place` is the station's own set-down, passed in so the retry uses it too:
    the sample table is placed at half speed and the deep seats are placed by
    feel, and a recovery that fell back to a plain dropdown() would set the
    piece down differently from the way that station is meant to be.
    """
    _release_and_require_clear(robot, station)
    robot.mvr2z(RECOVERY_SEARCH_LIFT)
    if not _recoverable_station(station):
        _abort_protective_stop(
            robot, station,
            "it carries no AprilTag, so there is nothing to search for")
    print("PAL12idb: recovering at %s -- released, backed off %.2f m, looking "
          "for the AprilTag again." % (station, RECOVERY_SEARCH_LIFT))
    _search_seat_again(robot, station)
    pickup(robot, height=RECOVERY_RESEAT_LIFT)
    place()


def _pickup_recovering(robot, station, height, open_extra_m = 0.0):
    """pickup(), recovering once from a protective stop at the seat.

    With `station` None -- a call site that has not been told which seat it is
    working on -- this is exactly pickup(), so an unconverted caller keeps
    today's fail-fast behaviour rather than recovering against a seat nobody
    named.
    """
    try:
        pickup(robot, height=height, open_extra_m=open_extra_m)
    except Exception as e:
        if station is None or not _is_protective_stop(robot, e):
            raise
        print("PAL12idb: protective stop picking up at %s (%s); recovering."
              % (station, e))
        try:
            _recover_at_start(robot, station, height, open_extra_m)
        except ProtectiveStopUnrecovered:
            raise
        except Exception as e2:            # noqa: BLE001 - one attempt only
            _abort_protective_stop(robot, station, "recovery failed: %s" % e2)


def _dropdown_recovering(robot, station, station_pos,
                         height = needle_clear_height, by_bump = False,
                         place = None, release_above = dropdown_release_above):
    """Set down at `station`, recovering once from a protective stop there.

    `place` overrides how: transport2sampletable places at half speed via
    dropdown_sampletable(), and the recovery has to re-place the same way.
    Defaults to the ordinary dropdown(). `release_above` is passed to that
    default dropdown() (the computed-Z release height above the grab point).
    """
    if place is None:
        def place():
            dropdown(robot, height=height, station_pos=station_pos,
                     by_bump=by_bump, release_above=release_above)
    try:
        place()
    except Exception as e:
        if station is None or not _is_protective_stop(robot, e):
            raise
        print("PAL12idb: protective stop setting down at %s (%s); recovering."
              % (station, e))
        try:
            _recover_at_target(robot, station, station_pos, place)
        except ProtectiveStopUnrecovered:
            raise
        except Exception as e2:            # noqa: BLE001 - one attempt only
            _abort_protective_stop(robot, station, "recovery failed: %s" % e2)


def _move_leg_recovering(robot, target, via_transferpoint, station = None):
    """_move_leg(), abandoning safely on a protective stop in mid-travel.

    No search here: a stop between seats leaves the arm somewhere with no tag
    to re-find and no seat to re-measure against, so there is nothing to
    recover to. The arm is released and lifted clear and the operator called.
    Releasing drops whatever is being carried -- chosen deliberately over
    leaving a stalled arm holding a piece out over the cell.
    """
    try:
        _move_leg(robot, target, via_transferpoint)
    except Exception as e:
        if not _is_protective_stop(robot, e):
            raise
        _abort_protective_stop(
            robot, station,
            "protective stop in mid-travel, with no seat to re-measure "
            "against (%s)" % e)


def transport2sampletable(robot, p1, height = needle_clear_height,
                          via_transferpoint = False, pickup_from_above = False,
                          p1_station = None):
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
    #
    # `p1_station` names the seat picked up from, so a protective stop there can
    # be recovered. The destination is always the sample table, so that end
    # names itself.
    activate_gripper(robot)
    try:
        if pickup_from_above:
            move_to_cleaningstation(robot, p1, via_transferpoint)
        else:
            _move_leg(robot, p1, via_transferpoint)
    except Exception as e:
        if not _is_protective_stop(robot, e):
            raise
        _abort_protective_stop(
            robot, p1_station,
            "protective stop on the approach, before anything was picked "
            "up (%s)" % e)
    # pickup_from_above is only ever set for a flowcell cleaning station (see
    # the note above), so it doubles as "this is the pickup that wants the
    # wider opening" -- no second flag saying the same thing twice.
    _pickup_recovering(
        robot, p1_station, height=height,
        open_extra_m=cleaning_station_open_extra() if pickup_from_above else 0.0)
    p2 = list(get_sampletable_position())
    p2[2] = p2[2]-distance_gripper_tag+sampletable_clear_height
    _move_leg_recovering(robot, p2, via_transferpoint, 'sample_table')
    _dropdown_recovering(robot, 'sample_table', get_sampletable_position(),
                         place=lambda: dropdown_sampletable(robot))

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
    # The transfer point is read live (get_transfer_point: the taught value, or
    # the module default), so re-teaching it takes effect at once.
    #
    # Unless the arm is standing at `target` already: then it is not crossing
    # between the two regions at all, and going out to the corridor and back
    # is travel for its own sake. The final moveto still runs -- there can be
    # a centimetre and an orientation left to take up -- it just goes direct.
    if not already_at(robot, target):
        robot.moveto(get_transfer_point())
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
              via_transferpoint = False, drop_by_bump = False,
              p1_station = None, p2_station = None, open_extra_m = 0.0,
              drop_release_above = dropdown_release_above):
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
    #
    # `p1_station` / `p2_station` name the two seats, which is what lets a
    # protective stop at either end be recovered from (the recovery has to know
    # which AprilTag to look for and which taught pose to stand over). Left
    # None, the wrappers below are exactly pickup()/dropdown()/_move_leg() and
    # nothing recovers -- so an un-named caller keeps today's fail-fast
    # behaviour instead of recovering against a seat nobody identified.
    activate_gripper(robot)
    _move_leg_recovering(robot, p1, via_transferpoint, p1_station)
    # `height` has to reach pickup()/dropdown() too, not just the travel Z
    # below: the mixer head only needs to clear its post by mixer_height, and
    # lifting it the full needle_clear_height instead swings it far higher
    # than the move requires.
    #
    # `open_extra_m` widens the fingers for the pickup at p1 only. It is the
    # caller's to set rather than derived from p1_station, because what has to
    # be cleared is the thing being picked up, not the seat it stands in: the
    # same station appears as p1 holding the mixer head and as p2 holding
    # nothing.
    _pickup_recovering(robot, p1_station, height=height,
                       open_extra_m=open_extra_m)
    # Z position should be the needle cleared position. Work on a copy: writing
    # p2[2] in place edits the caller's list, so a taught position passed in
    # (sample_table / cleaning_station) would creep upward on every transport.
    p2 = list(p2)
    destination = list(p2)          # the taught pose, before the clearance
    p2[2] = p2[2]-distance_gripper_tag+drop_height
    _move_leg_recovering(robot, p2, via_transferpoint, p2_station)
    _dropdown_recovering(robot, p2_station, destination, height=drop_height,
                         by_bump=drop_by_bump, release_above=drop_release_above)
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


def _align_sample_table_if_enabled():
    """Put the DAQ sample-stage motors on the recorded sample-table position and
    return where they were (for _restore_stage), or do nothing and return None
    when sample-table motor positioning is turned off.

    Off means exactly that -- no get_pos, no align, no restore: the sth/sav/stv
    EPICS motors are not touched at all. So a transport runs even when the DAQ
    GUI is unreachable or the positions were never recorded, instead of
    align_sample_table() raising before anything moves. On, the behaviour is
    unchanged: capture where the stage is, align it, and the caller puts it back
    afterwards with _restore_stage().
    """
    if not position_sample_table_motors:
        print("PAL12idb: sample-table motor positioning is off; not moving "
              "sth/sav/stv.")
        return None
    restore_to = daq_client.get_pos()
    daq_client.align_sample_table()
    return restore_to


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
              height=mixer_height, drop_by_bump=True,
              p1_station='mixer_station', p2_station='mixer_cleaning_station',
              open_extra_m=mixer_head_open_extra())

# mixer head to the mixer station. The robot is assumed to be empty.
@_tracks(lambda: state.set_mixer_head, state.MIXER_AT_MIXER)
def mixer2mixingstation(robot):
    # Set down by feel here too: the mixer station is a deep socket like the
    # cleaning station, so bump for the seat rather than a computed release Z.
    transport(robot, get_mixer_cleaningstation_position(), get_mixerstation_position(),
              height=mixer_height, drop_by_bump=True,
              p1_station='mixer_cleaning_station', p2_station='mixer_station',
              open_extra_m=mixer_head_open_extra())

# Bring the flowcell parked at the cleaning station to the beam, ready for data collection. The robot is assumed to be empty.
# This is for measuring water background. The flowcell is not loaded with sample.
@_tracks(_flowcell_setter, state.FC_AT_BEAM)
def load_flowcell_from_cleaningstation_to_beam(robot):
    # Bring the DAQ sample stage back to where it was when the robot's
    # sample_table waypoint was taught, before the robot approaches it, and
    # capture where it was so _restore_stage() can put it back once the flowcell
    # is seated. Skipped entirely when sample-table motor positioning is off (the
    # stage is then left wherever it stands) -- see _align_sample_table_if_enabled().
    restore_to = _align_sample_table_if_enabled()
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # cleaning station -> sample table: crosses between the two regions.
    transport2sampletable(robot, cleaning_station, height=needle_clear_height,
                          via_transferpoint=True, pickup_from_above=True,
                          p1_station='cleaning_station')
    # The flowcell is seated and the gripper is clear, so the stage is free to
    # go back to where the beam is set up to look -- and it has to happen here,
    # before the background acquisition that follows this call.
    _restore_stage(restore_to, 'loading the flowcell to the beam')

# Bring the flowcell from the beam to the cleaning station.
@_tracks(_flowcell_setter, state.FC_AT_CLEANING)
def load_flowcell_from_beam_to_cleaningstation(robot):
    # Align the DAQ stage on the way off the sample table too, capturing where
    # it was so it can be put back once the flowcell has actually left. Skipped
    # entirely when sample-table motor positioning is off --
    # see _align_sample_table_if_enabled().
    restore_to = _align_sample_table_if_enabled()
    if flowcell_ID == 1:
        cleaning_station = get_cleaningstation_position(1)
    else:
        cleaning_station = get_cleaningstation_position(2)
    # sample table -> cleaning station: crosses between the two regions.
    # Set down at a computed Z, 2 mm above where the flowcell was grabbed, then
    # opened -- not by feel. It is come down on purely vertically
    # (move_to_cleaningstation), so there is nothing to brush on the way, and a
    # known height avoids the deep-seat bump reading the arm's own deceleration
    # as contact.
    transport(robot, get_sampletable_position(), cleaning_station,
              height=needle_clear_height, via_transferpoint=True,
              drop_release_above=cleaning_station_release_above,
              p1_station='sample_table', p2_station='cleaning_station')
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
    _pickup_recovering(robot, 'cleaning_station', height=needle_clear_height,
                       open_extra_m=cleaning_station_open_extra())
    p2 = list(get_sample_on_mixerstation_position())
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
    robot.bump(z=-1,backoff=0.002)

# after drawing, the robot is holding the flowcell. Move it to the beam and drop it.
@_tracks(_flowcell_setter, state.FC_AT_BEAM)
def load_sample_to_beam(robot, on_lift_complete=None):
    # Align the DAQ stage on the sample-table position and capture the measuring
    # position it was at, so _restore_stage() can return it there once the
    # flowcell is down. Skipped entirely when sample-table motor positioning is
    # off -- see _align_sample_table_if_enabled().
    restore_to = _align_sample_table_if_enabled()
    robot.mvr2z(needle_clear_height)
    if on_lift_complete is not None:
        # The scheduler hook is nonthrowing and only wakes its own worker.
        # Preserve the existing movement's normal-return/exception behavior.
        try:
            on_lift_complete()
        except Exception:
            # Notification failures must not interrupt the established UR route.
            import logging
            logging.getLogger(__name__).exception("Preparation lift notification failed")
    p2 = list(get_sampletable_position())
    p2[2] = p2[2]-distance_gripper_tag+sampletable_clear_height
    # mixer station -> sample table: crosses between the two regions.
    move_via_transferpoint(robot, p2)
    _dropdown_recovering(robot, 'sample_table', get_sampletable_position(),
                         place=lambda: dropdown_sampletable(robot))
    # Last step of the load, and the last thing before the campaign acquires:
    # the sample is in the beam only once the stage is back where the beam is.
    _restore_stage(restore_to, 'loading the sample to the beam')

# after data collection, pick up the flowcell from the beam and move it to the mixer to aspirate.
@_tracks(_flowcell_setter, state.FC_IN_GRIPPER)
def return_sample(robot):
    # Align the DAQ stage and capture where it was, restored at the end once the
    # flowcell has left the sample table. Without the restore the stage would be
    # left at the handoff position and the next load_sample_to_beam would read
    # that as the measuring position, losing it for every later sample. Skipped
    # entirely when sample-table motor positioning is off --
    # see _align_sample_table_if_enabled().
    restore_to = _align_sample_table_if_enabled()
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
    _pickup_recovering(robot, 'sample_table', height=needle_clear_height)
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
    _dropdown_recovering(robot, 'cleaning_station', cleaning_station,
                         by_bump=True)

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
    _pickup_recovering(robot, 'cleaning_station', height=needle_clear_height,
                       open_extra_m=cleaning_station_open_extra())
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


# -- carousel swap (unload the spent one / place a fresh one) ------------------
# Camera-guided pick-and-place of a whole carousel between the mixer, the stock
# position, and the used-carousel stack. All relative moves are BASE frame.
CAROUSEL_CLEAR_LIFT = 0.120          # straight-up clearance for carrying (m)
CAROUSEL_SNAP_LIFT = 0.003           # straight up right after grabbing, before the diagonal (m)
CAROUSEL_LOOK_LIFT = 0.100           # lift above the mixer seat to look (m)
CAROUSEL_STACK_ABOVE = 0.100         # unload: stand this far above a found used-stack tag before bumping (m)
# Identify which carousel is on the mixer from a raster scan: the carousel-on-
# mixer tag sits this far from the mixer-head tag, and a carousel tag must land
# within MIXER_CAROUSEL_MATCH_TOL of that expected point to count. The sample on
# that carousel is SAMPLE_OFFSET_Y along base +Y from the carousel tag centre.
MIXER_CAROUSEL_DX = -0.065           # carousel-on-mixer tag X offset from the mixer head (m)
MIXER_CAROUSEL_DY = -0.065           # carousel-on-mixer tag Y offset from the mixer head (m)
MIXER_CAROUSEL_MATCH_TOL = 0.010     # max distance from (mixer head + offset) to count (m)
SAMPLE_OFFSET_Y = 0.062              # sample is this far along base +Y from the carousel tag centre (m)
# The 45 deg seat-clearance move after grab: 10 mm total, so cos45 along each of
# -X and +Z. Sliding the carousel out from under its retainer while lifting.
# Two-segment slide that clears the carousel retainer after the snap-lift: a
# short STEEP segment to lift the carousel clear of the surface first (a single
# shallow slide was glancing the surface), then a long SHALLOW segment to travel
# out from under the retainer. Each is (travel_m, angle_deg_above_horizontal) and
# runs out-and-up (-X / +Z). Place retraces the SAME segments in reverse
# (reversed order, each +X / -Z) so a new carousel seats along the same path.
CAROUSEL_SLIDE_SEGMENTS = (
    (0.002, 60.0),   # steep: 2 mm at 60 deg -- lift clear of the surface first
    (0.020, 30.0),   # shallow: 20 mm at 30 deg -- travel out from under the retainer
)
# Per-segment (dx, dz) components, and the totals used to offset the place start.
CAROUSEL_SLIDE_STEPS = [
    (t * math.cos(math.radians(a)), t * math.sin(math.radians(a)))
    for (t, a) in CAROUSEL_SLIDE_SEGMENTS
]
CAROUSEL_SLIDE_TOTAL_DX = sum(dx for dx, _dz in CAROUSEL_SLIDE_STEPS)   # total horizontal (m)
CAROUSEL_SLIDE_TOTAL_DZ = sum(dz for _dx, dz in CAROUSEL_SLIDE_STEPS)   # total vertical (m)
# A carousel position is taught bump-to-surface then backed off by this, so the
# taught Z sits this far ABOVE the grab surface -- a safe height to move to before
# the grab bumps down to find the real surface.
CAROUSEL_TEACH_BACKOFF = 0.060
CAROUSEL_BUMP_LIFT = 0.001           # up after the grab bump finds the surface
CAROUSEL_GRAB_UNDER = 0.012          # down after opening, to reach under the rim (11 mm net below surface)
# Place: how far above the surface the carousel should stand after the reverse
# 45 deg landing -- the bump then seats the rest. The gripper start Z is worked
# back from this (see place_new_carousel).
CAROUSEL_PLACE_STANDOFF = 0.005
# The carousel acquire spends longer than the default on centring: the arm
# arrives right after a move with the camera still refocusing, and a mis-centred
# grab is costly. Longer detect/centre settle (seconds the camera keeps
# refocusing) and more centring passes than the defaults, plus a confirming pass.
CAROUSEL_CENTER_SETTLE = 10          # s to keep recapturing while the camera focuses
CAROUSEL_CENTER_ITERS = 8            # centring passes (the default is 4)
CAROUSEL_CENTER_TOL_M = 0.0002       # how close to centred counts as centred (m)
CAROUSEL_LOAD_APPROACH_M = 0.200     # load: approach to this camera-to-tag distance before the grab


def _fmt_m(x):
    """Format a distance in metres for a log line, or '?' when it is unknown."""
    try:
        return "%.4f" % float(x)
    except (TypeError, ValueError):
        return "?"


def _pose_str(robot):
    """Current pose as a short string for debug prints.

    Full [x, y, z, rx, ry, rz] when the robot exposes get_pose(); otherwise the
    xyz from get_pos() (so a simulated/fake robot in tests still works)."""
    try:
        v = robot.get_pose().get_pose_vector().tolist()
    except Exception:                                    # noqa: BLE001
        try:
            v = list(robot.get_pos())
        except Exception:                                # noqa: BLE001
            return "?"
    return "[" + ", ".join("%.4f" % x for x in v) + "]"


def _acquire_carousel(robot, station, approach_distance_m=None):
    """Centre and square the camera on the carousel at `station`.

    Returns (tag_id, camera_to_tag_distance_m). Raises RuntimeError if no
    acceptable carousel tag is in view. Mirrors locate_apriltag's centre+square
    (including the tag mounting offset, apriltag_rz_offset) but does not drive to
    a reference or grab -- the caller has already put the camera over the seat.

    When approach_distance_m is set, after centring at the look height the camera
    is driven down to that camera-to-tag distance and re-centred there (a closer,
    steadier view before the grab). Used for loading, where a precise centre
    matters most; the grab then takes over (put_tcp2camera, move down).
    """
    robot.set_tcp(robot.tcp)
    robot.camera.AT_physical_size = apriltag_size(station)
    robot.camera.AT_preferred_id = apriltag_ids(station)
    # Give the camera time to refocus before the first read -- the arm has just
    # arrived over the seat and the first frames are usually blurry.
    if camera_tools._detect_apriltag(robot, settle=CAROUSEL_CENTER_SETTLE) is None:
        seen = list(getattr(robot.camera, 'AT_ids', []) or [])
        raise RuntimeError(
            "no carousel at %s: looked for tag(s) %s, tags in view %s"
            % (station, sorted(apriltag_ids(station) or []), seen))

    def _center():
        # A long settle so each centring pass waits out the camera's refocus,
        # and extra passes so it actually converges on the tag centre.
        return robot.center_camera2apriltag(
            tol_m=CAROUSEL_CENTER_TOL_M, settle=CAROUSEL_CENTER_SETTLE,
            max_iter=CAROUSEL_CENTER_ITERS)

    # No descend-to-standoff: centre and square from the look height, then the
    # grab moves to the taught height and bumps for the surface.
    if not _center():
        if not reacquire_apriltag(robot)[0] or not _center():
            raise RuntimeError("could not centre the camera on the carousel at %s"
                               % station)
    # Square the heading to the seat frame -- the tag is mounted turned about its
    # normal (apriltag_rz_offset), the same correction the search applies.
    if robot.camera.AT_euler is not None:
        robot.rotate_around_Zaxis_camera(
            180 + robot.camera.AT_euler[2] + apriltag_rz_offset(station))
        _center()
    if approach_distance_m is not None:
        # Approach to the requested camera-to-tag distance for a closer, steadier
        # view, then re-centre there before the grab commits. Print the distances
        # so the measurement can be checked against the real stand-off.
        before = robot.camera.QRdistance
        reached = camera_tools.approach_tag_distance(robot, approach_distance_m)
        _center()
        print("PAL12idb: carousel load approach to %s -- tag distance was %s m, "
              "reached %s m, %s m after re-centre"
              % (station, _fmt_m(before), _fmt_m(reached),
                 _fmt_m(robot.camera.QRdistance)))
    else:
        # One more confirming pass: with the camera settled and the heading
        # squared, make sure it is really centred before the grab commits here.
        _center()
    return robot.camera.AT_id, robot.camera.QRdistance


def _grab_carousel(robot, station, tag_distance):
    """Grab the carousel the camera is centred on, at `station`.

    Using the measured (calibrated) `tag_distance`, come straight down to
    APRILTAG_BUMP_STANDOFF (30 mm) above the tag surface, then close the gripper
    and bump to feel the actual surface -- same approach as every AprilTag search
    now. Then back up 1 mm, open wide, descend under the rim, and close to grip.

    Switches to the gripper TCP first and stays there: put_tcp2camera leaves the
    CAMERA TCP active, so every height read / move after this must be in the
    gripper frame -- otherwise the carousel is carried (and dropped) a camtcp
    offset away from where it should be.
    """
    robot.put_tcp2camera()                              # gripper over the tag centre
    robot.set_tcp(robot.tcp)                            # gripper frame for all height work
    if tag_distance is not None:
        # Down to 30 mm above the tag surface using the measured distance, then the
        # bump below feels the real surface for the last 30 mm.
        robot.mvr2z(-max(0.0, tag_distance - APRILTAG_BUMP_STANDOFF))
    close_gripper_fully(robot)                          # fully shut for a clean bump contact
    robot.bump(z=-1, backoff=0.0)                       # feel the real surface
    if station == 'carousel_on_mixer':
        # Record the true mixer seat the bump just found (gripper frame): the
        # camera-centred X,Y and the bumped surface Z. place_new_carousel reuses it
        # so it lands exactly where this grabbed -- the taught pose's X,Y are a
        # millimetre or two off (not camera-centred) and its Z sits too low (the
        # teach backoff was measured along the camera TCP).
        state.set_carousel_mixer_seat(robot.get_pos()[:3])
    robot.mvr2z(CAROUSEL_BUMP_LIFT)                     # up 1 mm
    open_gripper(robot, carousel_open_extra())          # open wide
    robot.mvr2z(-CAROUSEL_GRAB_UNDER)                   # down under the rim
    robot.grab()                                        # close to grip


def _carousel_lift(robot):
    """The seat-clearance lift after grabbing on the mixer: up SNAP_LIFT, then a
    two-segment slide (steep to lift clear of the surface, then shallow to travel
    out from under the retainer -- CAROUSEL_SLIDE_STEPS, each -X / +Z), then up
    CLEAR_LIFT. Prints the pose after each step for debugging."""
    robot.mvr2z(CAROUSEL_SNAP_LIFT)
    print("unload lift: after snap-lift   %s" % _pose_str(robot))
    for dx, dz in CAROUSEL_SLIDE_STEPS:          # steep clear-off, then shallow travel-out
        robot.translate([-dx, 0.0, dz])
    print("unload lift: after slide-out   %s" % _pose_str(robot))
    robot.mvr2z(CAROUSEL_CLEAR_LIFT)
    print("unload lift: after clear-lift  %s" % _pose_str(robot))


def _look_for_used_stack_tag(robot):
    """Look over the used-carousel stack for a tag before unloading.

    Returns (pose, distance): the gripper pose [x, y, z, ...] positioned over the
    tag (at the look height) and the corrected camera-to-tag distance, so the
    unload can set the new used carousel down on TOP of the stack (100 mm above
    that tag, then bump). Returns None when the used stack is untaught or empty --
    the unload then uses its normal taught-position drop. Non-fatal by design: any
    trouble here means 'no tag'.
    """
    try:
        used = list(get_station_position('carousel_used_stack'))
    except (RuntimeError, ValueError):
        return None
    robot.set_tcp(robot.tcp)
    clear_z = used[2] + CAROUSEL_CLEAR_LIFT
    used_over = list(used)
    used_over[2] = clear_z
    # Route OVER the used stack via the transfer point -- never a blind descent at
    # wherever the arm happens to be (that drove straight down into the mixer
    # carousel and protective-stopped). The transfer point keeps it clear until it
    # is above the stack, then it comes down to the look height.
    move_via_transferpoint(robot, used_over)           # safely over the used-stack centre
    robot.put_camera2tcp()                             # camera onto that pose
    robot.camera.AT_physical_size = apriltag_size('carousel_used_stack')
    robot.camera.AT_preferred_id = apriltag_ids('carousel_used_stack')
    if camera_tools._detect_apriltag(robot, settle=CAROUSEL_CENTER_SETTLE) is None:
        robot.set_tcp(robot.tcp)
        print("unload: no tag on the used stack -- using the normal taught drop")
        return None
    # Centre on the top carousel's tag so the gripper can be put straight over it,
    # and read the corrected distance (how far the tag is below the camera).
    robot.center_camera2apriltag(tol_m=CAROUSEL_CENTER_TOL_M,
                                 settle=CAROUSEL_CENTER_SETTLE,
                                 max_iter=CAROUSEL_CENTER_ITERS)
    # Centre only -- the drop does NOT match the heading of the carousel below; it
    # uses the taught used-stack orientation (see unload_current_carousel). We only
    # need the found tag's X,Y and its distance (how far below the camera it is).
    dist = robot.camera.QRdistance
    robot.put_tcp2camera()                             # gripper over the tag (where the camera was)
    robot.set_tcp(robot.tcp)
    pose = list(robot.get_pos())
    print("unload: found a carousel on the used stack, tag distance %s m -- "
          "will stack on top" % _fmt_m(dist))
    return (pose, dist)


@_tracks(lambda: state.set_carousel_on_mixer, state.UNKNOWN)
def unload_current_carousel(robot):
    """Take the carousel off the mixer and set it on the used-carousel stack.

    First look over the used stack for a tag: if the stack already holds a
    carousel, the one being unloaded is set down on TOP of it (straight down to
    100 mm above that tag, then bump); if the stack is empty (no tag), the drop is
    the normal taught used-stack position. Then grab the mixer carousel, lift clear
    on the 45 deg path, carry over the used stack, and set it down by feel. MOVES
    the robot. Requires carousel_on_mixer and carousel_used_stack to be taught.
    """
    robot.set_tcp(robot.tcp)
    # Look over the used stack before grabbing: is there already a carousel there?
    stack_tag = _look_for_used_stack_tag(robot)
    move_via_transferpoint(robot, list(get_station_position('carousel_on_mixer')))
    robot.mvr2z(CAROUSEL_LOOK_LIFT)
    # Put the CAMERA over the carousel centre (not just face it down in place):
    # the gripper is over the centre now, so put_camera2tcp moves the camera onto
    # that pose. Levelling the camera where it stands would leave it the camtcp
    # offset away from the centre, and the tag would fall outside the view.
    robot.put_camera2tcp()
    _tag, dist = _acquire_carousel(robot, 'carousel_on_mixer')
    _grab_carousel(robot, 'carousel_on_mixer', dist)
    _carousel_lift(robot)
    robot.set_tcp(robot.tcp)                 # gripper frame: the carousel is carried, not the camera
    clear_z = robot.get_pos()[2]
    if stack_tag is not None:
        # Stack on top of the carousel already on the used stack: carry over its
        # tag, go straight down to 100 mm above it, then bump to set down by feel.
        # Use the TAUGHT used-stack orientation (not the found tag's, and not the
        # camera-centring pose) -- it faces straight down, so the carousel is set
        # down at the recorded heading AND the bump's -Z travels straight down.
        over, dist = stack_tag
        heading = list(get_station_position('carousel_used_stack'))[3:]
        above = [over[0], over[1], clear_z] + heading   # over the found tag, clear height, taught heading
        robot.moveto(above)
        print("unload stack: over the tag %s" % _pose_str(robot))
        drop_z = over[2] - dist + CAROUSEL_STACK_ABOVE  # 100 mm above the tag (tag is dist below `over`)
        robot.moveto([over[0], over[1], drop_z] + heading)   # straight down to 100 mm above the tag
        print("unload stack: 100 mm above the tag %s" % _pose_str(robot))
        robot.bump(z=-1, backoff=0.0)                   # bump straight down (long travel) onto the stack
        print("unload stack: after bump %s" % _pose_str(robot))
        robot.release()                                 # open as soon as it bumps -- do not force further down
        robot.moveto(above)                             # back up to the clear height
    else:
        # Empty stack: drop at the taught used-stack pose (full pose -> the stack's
        # own heading, not the mixer heading it was grabbed with).
        used_clear = list(get_station_position('carousel_used_stack'))
        used_clear[2] = clear_z
        robot.moveto(used_clear)                         # over the used stack, at its heading
        robot.bump(z=-1, backoff=0.0)                    # set down by feel
        robot.release()                                  # open as soon as it bumps -- do not force further down
        robot.moveto(used_clear)                         # back up to the clear height
    return list(robot.get_pos())


def place_new_carousel(robot):
    """Bring a fresh carousel from the stock position onto the mixer.

    Carry clear, swing the camera over the stock seat, grab the new carousel,
    carry it to 120 mm above the mixer, and set it down onto the mixer seat --
    sliding in along the reverse (+X / -Z) of the unload lift and bumping to the
    seat. MOVES the robot. Requires carousel_in_stock and carousel_on_mixer
    taught.

    Not @_tracks-decorated like the transports: the tracked value is the tag id
    of the carousel it grabs, known only at runtime. The mixer is marked unknown
    up front (it is mid-swap, so a failure leaves it honestly unknown) and set to
    the grabbed tag once it is seated.
    """
    robot.set_tcp(robot.tcp)
    state.set_carousel_on_mixer(None)       # mid-swap: unknown until it is seated
    mixer = list(get_station_position('carousel_on_mixer'))
    clear_z = mixer[2] + CAROUSEL_CLEAR_LIFT
    now = robot.get_pos()
    robot.moveto([now[0], now[1], clear_z])             # up to the carry height
    # Take the GRIPPER over the stock centre at the clear height, using the stock
    # pose's own (down-facing) orientation, then put the CAMERA onto that pose.
    # Same reason as unload: the camera has to be over the tag, not the camtcp
    # offset away from it, or the tag falls outside the view.
    stock = get_station_position('carousel_in_stock')
    stock_over = list(stock)
    stock_over[2] = clear_z
    robot.moveto(stock_over)                            # gripper over stock centre, facing down
    robot.put_camera2tcp()                              # camera onto that pose
    tag, dist = _acquire_carousel(robot, 'carousel_in_stock',
                                  approach_distance_m=CAROUSEL_LOAD_APPROACH_M)
    _grab_carousel(robot, 'carousel_in_stock', dist)
    robot.set_tcp(robot.tcp)                 # gripper frame: the carousel is carried, not the camera
    robot.mvr2z(CAROUSEL_CLEAR_LIFT)                     # lift clear of the stock seat
    # Reuse the REAL mixer seat measured by unload's grab (gripper frame): the
    # camera-centred X,Y and the bumped surface Z. Fall back to the taught pose
    # only if no unload has measured it. Using the measured X,Y (not the taught
    # X,Y, which is a mm or two off) lets the landing be the exact reverse of the
    # lift -- same two-segment slide travel (SLIDE_STEPS), landing right on the seat.
    seat = state.get_carousel_mixer_seat()
    if seat is not None:
        seat_x, seat_y, surface_z = seat
    else:
        seat_x, seat_y = mixer[0], mixer[1]
        surface_z = mixer[2] - CAROUSEL_TEACH_BACKOFF
    # Make the place start coincide EXACTLY with the unload lift's "after slide-out"
    # point, so place reverses the lift step for step: straight down to this start,
    # slide in 30 deg then 60 deg (the lift's segments reversed), then straight down
    # (the bump) through the snap-lift to seat. The lift ends CAROUSEL_SNAP_LIFT +
    # TOTAL_DZ above the grab point (net_under below the measured surface), offset
    # back by TOTAL_DX in X -- so the start is that same point.
    net_under = CAROUSEL_GRAB_UNDER - CAROUSEL_BUMP_LIFT   # carousel hangs this far below the gripper
    start = list(mixer)                                   # taught orientation (3:6)
    start[0] = seat_x - CAROUSEL_SLIDE_TOTAL_DX          # back along -X so the reverse slide reaches the seat X
    start[1] = seat_y
    start[2] = surface_z + CAROUSEL_SLIDE_TOTAL_DZ + CAROUSEL_SNAP_LIFT - net_under
    carry = list(start)
    carry[2] = clear_z
    robot.moveto(carry)                                  # carry above the landing start, seat heading
    print("place seating: after carry-above %s" % _pose_str(robot))
    robot.moveto(start)                                  # down to the stand-off, offset back along -X
    print("place seating: after down-to-start %s" % _pose_str(robot))
    # Reverse slide-in landing: retrace the unload lift's two-segment slide in
    # reverse (segments reversed, each +X / -Z), landing on the measured seat X,Y
    # along the same trajectory.
    for dx, dz in reversed(CAROUSEL_SLIDE_STEPS):
        robot.translate([dx, 0.0, -dz])
    print("place seating: after slide-in landing %s" % _pose_str(robot))
    robot.bump(z=-1, backoff=0.0)                        # bump down to seat at the real surface
    print("place seating: after bump %s" % _pose_str(robot))
    robot.release()
    print("place seating: after release %s" % _pose_str(robot))
    robot.mvr2z(CAROUSEL_CLEAR_LIFT)                     # retract clear
    print("place seating: after retract %s" % _pose_str(robot))
    # Record which carousel is now on the mixer (overrides the @_tracks default).
    state.set_carousel_on_mixer(tag)
    return list(robot.get_pos())


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
    elif pos == 'carousel_on_mixer':
        set_carousel_on_mixer_position(v)
    elif pos == 'carousel_in_stock':
        set_carousel_in_stock_position(v)
    elif pos == 'carousel_used_stack':
        set_carousel_used_stack_position(v)
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
    if pos == 'carousel_on_mixer':
        return get_carousel_on_mixer_position()
    if pos == 'carousel_in_stock':
        return get_carousel_in_stock_position()
    if pos == 'carousel_used_stack':
        return get_carousel_used_stack_position()
    raise ValueError('unknown station %r' % (pos,))

def search_reference(pos):
    """Where locate_apriltag() drives to before hunting for `pos`'s AprilTag.

    The station's taught X/Y, at the configured ref pose's Z and orientation.
    Each part comes from whichever source actually knows it:

      * X/Y from waypoints.ini, so moving a station -- by re-teaching it or by
        editing the ini -- moves where the search looks for it. Taking this
        from a constant is what left the search starting 0.33 m away from the
        mixer station after it was physically repositioned.
      * Z and orientation from `search_refs` in json/palmixer_config.json,
        because those are a *camera* standoff and attitude, not station
        geometry: how far back to stand for the tag to be in frame at a
        workable size, and which way to face. The taught pose's own Z is down
        at the grab point and its orientation is the gripper's angle on a
        possibly tilted seat, so neither substitutes.

    Both are read live -- config.py re-reads its file on every access, as
    waypoints.ini is -- so neither needs a restart to take effect.

    Falls back to the ref pose unchanged for a station with no taught position:
    the first search for a station has nothing else to go on, which is the case
    these poses were originally written for.
    """
    ref = _ref(pos, ref_sampletable)
    try:
        taught = get_station_position(pos)
    except (RuntimeError, ValueError):
        return ref          # never taught, or not a station -- ref as written
    return [taught[0], taught[1]] + list(ref[2:])


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


POSITION_AXES = ('x', 'y', 'z')

#: Largest single position jog, in metres. A jog is an operator nudging the arm
#: by eye next to a seat; anything this big is a typo or a wrong unit, and the
#: arm is usually millimetres from something it must not hit. Enforced here
#: rather than only in the GUI spin box, because the command is reachable over
#: ZMQ without going through it.
MAX_POSITION_TWEAK = 0.05

def tweak_position(robot, axis, metres):
    """Move the arm `metres` along base-frame `axis`, leaving orientation alone.

    The position counterpart of tweak_orientation, and the jog behind the
    Configuration tab's position teaching. Deliberately in the **base** frame,
    not the tool frame the rotation jog uses: what this exists to fine-tune is
    the X/Y/Z that "Set Current Robot Position As" then writes to
    waypoints.ini, and those are base-frame numbers. Pressing "+X" moves the
    arm the same direction as the stored x going up, whatever angle the wrist
    happens to be at -- so a nudge watched on camera and the number that lands
    in the file agree.

    Implemented as read-pose / add / moveto rather than a relative move, for
    the same reason dropdown_at sends a full pose: where the arm ends up does
    not then depend on the controller's idea of the current frame. Runs at
    placement speed, half the default -- this is close-quarters work.
    """
    axis = str(axis).lower()
    if axis not in POSITION_AXES:
        raise ValueError('axis must be one of %s, got %r' % (POSITION_AXES, axis))
    metres = float(metres)
    if not abs(metres) <= MAX_POSITION_TWEAK:      # also catches NaN
        raise ValueError('jog of %.4f m exceeds the %.3f m limit for a single '
                         'step (MAX_POSITION_TWEAK)' % (metres, MAX_POSITION_TWEAK))
    # An aborted search can leave the camera TCP live; the jog has to move the
    # gripper tip, which is the frame the taught positions are expressed in.
    robot.set_tcp(robot.tcp)
    target = list(robot.get_pos())
    target[POSITION_AXES.index(axis)] += metres
    robot.moveto(target, acc=placement_accel, vel=placement_speed)
    return list(robot.get_pos())

def put_camera_to_tcp(robot):
    """Move the camera onto the pose the gripper tip occupies now.

    The arm translates by the camera offset; it does not reorient. Since the
    tool is normally face-down, the camera ends up face-down too, which is why
    this is also how a search makes the camera level. Leaves the gripper TCP
    active (robUR.put_camera2tcp), so a pose read afterwards is the tool's.

    Exposed as a button because "start from the current position" needs the
    camera already looking down before it will search without reorienting:
    this is how the operator gets it there.
    """
    robot.put_camera2tcp()
    return list(robot.get_pos())

def put_tcp_to_camera(robot):
    """Move the gripper tip onto the pose the camera occupies now.

    The inverse of put_camera_to_tcp, and the way back after one: having lined
    the camera up on something, this brings the tool to that same spot. Leaves
    the camera TCP active (robUR.put_tcp2camera), which is what the caller
    wants while still looking through it -- every PALmixer transport asserts
    robot.tcp before it moves, so a camera TCP left live here cannot leak into
    one.
    """
    robot.put_tcp2camera()
    return list(robot.get_pos())

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

    The pose is read live with get_transfer_point() -- the taught value if one
    has been recorded (record_current_transfer_point), else the module default.
    This is the manual "park it out of the way" / "start from somewhere known"
    -- from the corridor every station is one ordinary move away, which is the
    same property move_via_transferpoint relies on.

    Sent direct rather than through move_via_transferpoint, which would be
    circular. The gripper TCP is asserted first because the transfer point is
    expressed in that frame: an aborted AprilTag search can leave the camera
    TCP active, and that would put the camera on the pose with the gripper an
    offset away from it.
    """
    robot.set_tcp(robot.tcp)
    target = get_transfer_point()
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
    apriltag_size() -- the measurement is directly proportional to it.

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

def locate_apriltag(robot, pos = '', stop_event=None, skip_roll=False,
                    ref_pos=None, store=True, via_transferpoint=True,
                    from_current=False):
    # Record the taught position of a station. Returns the pose it found, and
    # also stores it in sample_table / cleaning_station. stop_event, if given,
    # is a threading.Event the caller can set (alongside stopping the robot)
    # to abort the search early; see camera_tools.search_apriltag_by_tilt.
    # skip_roll leaves the camera face-normal-down (level) instead of rolling
    # it face-down / squaring it to a tilted tag: position is still recorded,
    # only the orientation differs (teach a tilted seat's angle by hand).
    global sample_table, cleaning_station1, cleaning_station2, mixer_cleaning_station, mixer_station
    # The station's own taught X/Y at its configured camera standoff -- see
    # search_reference(). This used to be a per-station constant, which meant
    # a station that physically moved was still searched for where it used to
    # be, and no edit to waypoints.ini could say otherwise.
    # ref_pos lets a caller say where to search from instead. Protective-stop
    # recovery does: it stands over the seat it just failed at, not at the
    # tuned teaching standoff (see _recovery_search_pose).
    # from_current searches from wherever the arm is standing. For a station
    # that has moved far enough that its configured reference no longer looks
    # anywhere near the tag: drive or jog the camera over it by eye (the
    # Configuration tab shows the view), then search from there. Nothing is
    # read from waypoints.ini or search_refs, so it also works for a station
    # whose stored position is wrong in a way that would send the arm
    # somewhere unhelpful or unsafe.
    if from_current:
        robot.set_tcp(robot.tcp)
        ref_pos = list(robot.get_pos())
    elif ref_pos is None:
        ref_pos = search_reference(pos)
    print("Looking for %s .... (%s)" % (pos, "from where the arm is standing"
          if from_current else "from %s" % [round(v, 4) for v in ref_pos[:3]]))
    # Tell the camera how big the tag it is about to look at actually is,
    # before anything measures a distance from it. This is read by every
    # distance-based step downstream, not just the descent below: the tilt
    # search's own "descend until the tag is 0.2 m away" loop, the pivot point
    # roll_around_tag() sets, and the pixels-to-meters conversion in
    # center_camera2apriltag() all scale with it.
    robot.camera.AT_physical_size = apriltag_size(pos)
    # And which tag it is, when the station names one. Same scope as the size
    # above and set beside it for that reason: both describe the tag being
    # looked for, and both are read by every detection downstream rather than
    # being passed from step to step.
    #
    # Without this, a frame with two tags in it is resolved by "nearest the
    # image center" -- decided afresh each time, so the tilt grid, the roll
    # and the descent can each land on a different tag, and the size set above
    # then belongs to the wrong one. None restores that behaviour, which is
    # correct while only one tag can be in view.
    # The set of tag ids this seat accepts -- one tag for most, a range for the
    # carousel positions (31-40); selectAT matches any member. See apriltag_ids.
    robot.camera.AT_preferred_id = apriltag_ids(pos)
    # Fail closed for a station that names specific tag(s). "Nearest the image
    # centre" was safe only while one tag at a time could be in view; the Venton
    # fiducials now sit on the bench near these stations, so a search that fell
    # back to proximity could lock onto a neighbour. If no id is configured, stop
    # here with a clear message rather than search by nearest.
    if pos in APRILTAG_SUBJECT and not robot.camera.AT_preferred_id:
        raise RuntimeError(
            "no AprilTag id configured for %s (apriltags.%s.id / .id_range); set "
            "it so the search matches only that tag instead of whichever is "
            "nearest the image centre" % (pos, _apriltag_subject(pos)))
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
    # via_transferpoint=False for a recovery search: the arm is standing on the
    # seat it stopped at, so the corridor detour would drag it sideways out of
    # whatever it is touching before lifting off it.
    #
    # from_current skips the approach outright. Routing through the corridor to
    # reach a pose the arm is already on would be the whole point of the mode
    # thrown away, and already_at() cannot be relied on to notice: the operator
    # may well have jogged to within its radius of some *other* station.
    #
    # Fully shut the gripper before the search: open fingers can clip the
    # camera's view of the tag and are more likely to catch on the bench or a
    # fixture as the arm tilts and descends over the station.
    close_gripper_fully(robot)
    if from_current:
        pass
    elif via_transferpoint:
        move_via_transferpoint(robot, list(ref_pos))
    else:
        robot.moveto(list(ref_pos))
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
        stop_event=stop_event, skip_roll=skip_roll, from_current=from_current,
        rz_offset=apriltag_rz_offset(pos))
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
    # Which tag this actually measured. Worth a line even when it is the only
    # one that could have been: the position about to be recorded is derived
    # from this tag, and with more than one in the cell "the search succeeded"
    # does not by itself say it succeeded on the right thing. It is also how
    # an operator reads off the number to put in apriltag_ids.
    seen = list(getattr(robot.camera, 'AT_ids', []) or [])
    print("Found %s on AprilTag %s%s." % (
        pos, getattr(robot.camera, 'AT_id', None),
        " (tags in view: %s)" % seen if len(seen) > 1 else ""))
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
    # The search already squared the camera to the seat frame, applying the
    # tag's mounting offset (apriltag_rz_offset) itself -- see
    # camera_tools.search_apriltag_by_tilt -- so nothing to undo here.
    #
    # Remember the (calibrated) camera-to-tag distance now, while it is fresh from
    # centring; then put the gripper over the tag and come straight down to
    # APRILTAG_BUMP_STANDOFF (30 mm) above the tag surface before bumping, so the
    # bump travels a short, consistent distance instead of the whole standoff.
    tag_distance = robot.camera.QRdistance
    robot.put_tcp2camera()
    if tag_distance is not None:
        robot.mvr2z(-max(0.0, tag_distance - APRILTAG_BUMP_STANDOFF))

    # Bump to measure the distance down to the surface, then back off -- so the
    # recorded Z is anchored to the real surface, not the camera standoff. A
    # reference-only station (the carousel centre) is NOT grabbed (nothing to pick
    # up there) and backs off CAROUSEL_TEACH_BACKOFF (60 mm), the amount
    # _grab_carousel later descends from the taught pose; every other station
    # backs off distance_gripper_tag as before.
    if pos not in NO_GRAB_STATIONS:
        robot.grab()
    backoff = CAROUSEL_TEACH_BACKOFF if pos in NO_GRAB_STATIONS else distance_gripper_tag
    robot.bump(z=-1, backoff=backoff)
    robot.set_tcp(robot.tcp)
    p = robot.get_pose()
    v = p.get_pose_vector().tolist()
    # store=False for a recovery search: a position found under duress must not
    # overwrite a good teach in waypoints.ini. The pose is still returned, so
    # the caller can use it for the run in hand.
    if store:
        store_station_position(pos, v)
    return v
