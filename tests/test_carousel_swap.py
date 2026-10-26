"""Unload / place a whole carousel: the motion sequences and the plumbing.

No robot and no camera -- a FakeRobot records the ordered base-frame moves, and
the camera-guided acquire/grab primitives are monkeypatched so these pin the
*choreography* (look-lift, grab recipe, the 45 deg seat-clearance lift and its
reverse, the planar carry, bump-and-release) and the carousel_on_mixer state,
not the camera geometry.
"""
from pathlib import Path
import sys
import types

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
from palmixer import commands as cmd
from palmixer import state
PAL12idb = pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")


class FakeGripper:
    def __init__(self, log):
        self._log = log
    def gripper_action(self, count):
        kind = "close" if count >= 128 else "open"
        self._log.append((kind, count))


class FakeRobot:
    """Tracks an xyz position and logs every motion/gripper call in order."""
    def __init__(self, pos=(0.30, -0.20, 0.50)):
        self.tcp = [0, 0, 0.195, 0, 0, 0]
        self.camtcp = [0, 0.0493, 0.06, -0.5236, 0, 0]
        self.pos = list(pos)
        self.log = []
        self.tcp_sets = []
        self.camera = types.SimpleNamespace(QRdistance=0.20)
        self.gripper = FakeGripper(self.log)
    def set_tcp(self, tcp):
        which = ("gripper" if list(tcp) == list(self.tcp) else
                 "camera" if list(tcp) == list(self.camtcp) else "other")
        self.tcp_sets.append(which)
    def get_pos(self):
        return list(self.pos)
    def mvr2z(self, dz, **k):
        self.pos[2] += dz
        self.log.append(("mvr2z", round(dz, 6)))
    def translate(self, v, **k):
        self.pos = [self.pos[i] + v[i] for i in range(3)]
        self.log.append(("translate", [round(x, 6) for x in v]))
    def moveto(self, p, **k):
        self.pos = [p[0], p[1], p[2]]
        self.log.append(("moveto", [round(x, 6) for x in p]))
    def bump(self, z=-1, backoff=0.0, **k):
        self.log.append(("bump", z, backoff))
    def release(self):
        self.log.append(("release",))
    def grab(self):
        self.log.append(("grab",))
    def put_tcp2camera(self):
        self.log.append(("put_tcp2camera",))
    def put_camera2tcp(self):
        self.log.append(("put_camera2tcp",))


@pytest.fixture
def stations(monkeypatch):
    pos = {"carousel_on_mixer": [0.40, 0.10, 0.30, 2.2, -2.2, 0.10],
           "carousel_in_stock": [0.10, 0.44, 0.28, 2.2, -2.2, 0.70],
           "carousel_used_stack": [0.55, 0.10, 0.32, 2.2, -2.2, 1.57]}
    monkeypatch.setattr(PAL12idb, "get_station_position", lambda s: list(pos[s]))
    # No carousel safe Z unless a test sets one (never the bench's waypoints.ini).
    monkeypatch.setattr(PAL12idb, "get_carousel_safe_z", lambda: None)
    return pos


@pytest.fixture
def quiet_camera(monkeypatch):
    # The camera is put over the tag with put_camera2tcp (logged by the fake);
    # only the corridor routing is stubbed to a plain move.
    monkeypatch.setattr(PAL12idb, "move_via_transferpoint",
                        lambda robot, target: robot.moveto(target[:3]))


@pytest.fixture(autouse=True)
def table_parallel_to_base(monkeypatch):
    # The table axes parallel to base X/Y, as the carousel geometry was measured
    # (never the bench's json/venton_frame.json).
    monkeypatch.setattr(PAL12idb, "load_venton_frame", lambda *a, **k: None)


@pytest.fixture
def tmp_state(tmp_path, monkeypatch):
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))


def _kinds(log):
    return [e[0] for e in log]


# -- the grab recipe --------------------------------------------------------
def test_grab_descends_to_30mm_above_via_tag_distance_then_bumps(tmp_state):
    # Using the measured tag distance, come straight down to 30 mm above the tag
    # surface, close the gripper, bump to feel the surface, up 1 mm, open, down
    # under the rim, close. No taught-height move any more.
    rob = FakeRobot()
    tag_distance = 0.20
    PAL12idb._grab_carousel(rob, "carousel_on_mixer", tag_distance)
    kinds = _kinds(rob.log)
    assert kinds == ["put_tcp2camera", "mvr2z", "close", "bump", "mvr2z",
                     "open", "mvr2z", "grab"]
    assert "moveto" not in kinds                              # no taught-height move
    ups_downs = [e for e in rob.log if e[0] == "mvr2z"]
    # first mvr2z: down to 30 mm above the surface = -(tag_distance - standoff)
    assert ups_downs[0][1] == pytest.approx(
        -(tag_distance - PAL12idb.APRILTAG_BUMP_STANDOFF))
    assert kinds.index("close") < kinds.index("bump")        # fully shut before the bump
    pre_bump = rob.log[kinds.index("close")]
    assert pre_bump[1] == PAL12idb.GRIPPER_CLOSE_COUNT       # all the way shut, not a force stop
    assert ups_downs[1][1] == pytest.approx(PAL12idb.CAROUSEL_BUMP_LIFT)    # +1 mm
    assert ups_downs[2][1] == pytest.approx(-PAL12idb.CAROUSEL_GRAB_UNDER)  # -12 mm under the rim
    # grabbing off the mixer records the real seat the grab found (gripper X,Y and
    # the bumped-surface Z), for place to reuse so it lands exactly here.
    assert state.get_carousel_mixer_seat() is not None


def test_grab_switches_to_the_gripper_tcp(tmp_state):
    # put_tcp2camera leaves the camera TCP active; the grab must put the gripper
    # TCP back so the carry/drop are in the gripper frame (the drop-at-wrong-
    # position bug).
    rob = FakeRobot()
    PAL12idb._grab_carousel(rob, "carousel_on_mixer", 0.20)
    assert "gripper" in rob.tcp_sets


# -- the seat-clearance lift ------------------------------------------------
def test_lift_path_is_up_diagonal_up():
    rob = FakeRobot()
    PAL12idb._carousel_lift(rob)
    steps = PAL12idb.CAROUSEL_SLIDE_STEPS
    expected = [("mvr2z", round(PAL12idb.CAROUSEL_SNAP_LIFT, 6))]
    expected += [("translate", [round(-dx, 6), 0.0, round(dz, 6)]) for dx, dz in steps]
    expected += [("mvr2z", round(PAL12idb.CAROUSEL_CLEAR_LIFT, 6))]
    assert rob.log == expected


# -- unload -----------------------------------------------------------------
def test_unload_sequence(stations, quiet_camera, tmp_state, monkeypatch):
    monkeypatch.setattr(PAL12idb, "_look_for_used_stack_tag", lambda r: None)  # empty stack
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (33, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    rob = FakeRobot()
    PAL12idb.unload_current_carousel(rob)
    kinds = _kinds(rob.log)
    # look-lift, grab, lift(up/diag/up), carry over used stack, bump, release, back up.
    # The drop opens as soon as it bumps -- no forcing further down onto the stack.
    # The three leading moveto are _carousel_goto to the mixer carousel: lift
    # straight up, across at height, straight down -- DIRECT, not via the transfer
    # point (that route crosses the base and protective-stops).
    n = len(PAL12idb.CAROUSEL_SLIDE_STEPS)
    assert kinds == ["moveto", "moveto", "moveto", "mvr2z", "put_camera2tcp", "grab", "mvr2z"] +                     ["translate"] * n + ["mvr2z", "moveto", "bump", "release", "moveto"]
    assert kinds.index("release") == kinds.index("bump") + 1   # open right after the bump
    # the two carries are planar over the used stack at the clear height
    moves = [e for e in rob.log if e[0] == "moveto"]
    used = stations["carousel_used_stack"]
    assert moves[-2][1][:2] == [used[0], used[1]]
    assert moves[-2][1][3:] == used[3:]               # dropped at the stack's own heading
    assert moves[-1][1] == moves[-2][1]               # returned to the clear height
    assert state.get_carousel_on_mixer() == state.UNKNOWN   # mixer now empty


def test_unload_stacks_on_an_existing_used_carousel(stations, quiet_camera, tmp_state, monkeypatch):
    # When the used stack already holds a carousel (a tag is found), set the new
    # one down on top: over that tag, straight down to 100 mm above it, then bump.
    over_pose = [0.55, 0.10, 0.45, 2.2, -2.2, 1.57]       # gripper over the found tag
    dist = 0.18                                            # tag is 180 mm below `over_pose`
    monkeypatch.setattr(PAL12idb, "_look_for_used_stack_tag",
                        lambda r: (list(over_pose), dist))
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (33, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    rob = FakeRobot()
    PAL12idb.unload_current_carousel(rob)
    kinds = _kinds(rob.log)
    n = len(PAL12idb.CAROUSEL_SLIDE_STEPS)
    # Three leading moveto: _carousel_goto to the mixer carousel (direct, not via
    # the transfer point). _look_for_used_stack_tag is mocked, so its own direct
    # goto does not appear here.
    assert kinds == ["moveto", "moveto", "moveto", "mvr2z", "put_camera2tcp", "grab", "mvr2z"] +                     ["translate"] * n + ["mvr2z", "moveto", "moveto", "bump", "release", "moveto"]
    assert kinds.index("release") == kinds.index("bump") + 1   # open right after the bump
    moves = [e for e in rob.log if e[0] == "moveto"]
    # over the found tag's X,Y (the two seating moves and the back-up)
    assert moves[-3][1][:2] == [over_pose[0], over_pose[1]]    # carry above the tag
    assert moves[-2][1][:2] == [over_pose[0], over_pose[1]]    # straight down over the tag
    # straight down to 100 mm above the tag (tag is `dist` below over_pose)
    assert moves[-2][1][2] == pytest.approx(
        over_pose[2] - dist + PAL12idb.CAROUSEL_STACK_ABOVE)
    # dropped at the TAUGHT used-stack heading (not the found tag's), so the bump
    # travels straight down.
    used = stations["carousel_used_stack"]
    assert moves[-3][1][3:] == used[3:]                        # above pose uses taught heading
    assert moves[-2][1][3:] == used[3:]                        # straight-down pose uses taught heading
    assert moves[-1][1] == moves[-3][1]                        # back up to the clear height
    assert state.get_carousel_on_mixer() == state.UNKNOWN


def test_unload_errors_when_no_carousel(stations, quiet_camera, tmp_state, monkeypatch):
    monkeypatch.setattr(PAL12idb, "_look_for_used_stack_tag", lambda r: None)
    def boom(r, s, **k):
        raise RuntimeError("no carousel at %s" % s)
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", boom)
    with pytest.raises(RuntimeError, match="no carousel"):
        PAL12idb.unload_current_carousel(FakeRobot())
    assert state.get_carousel_on_mixer() == state.UNKNOWN   # left honestly unknown


# -- place ------------------------------------------------------------------
def test_place_sequence_and_reverse_diagonal(stations, quiet_camera, tmp_state, monkeypatch):
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (37, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    rob = FakeRobot()
    PAL12idb.place_new_carousel(rob)
    kinds = _kinds(rob.log)
    n = len(PAL12idb.CAROUSEL_SLIDE_STEPS)
    # The three leading moveto are _carousel_goto over the stock (up, across,
    # down). Trailing "moveto" is the park at the transport (transfer) position.
    assert kinds == ["moveto", "moveto", "moveto", "put_camera2tcp", "grab", "mvr2z",
                     "moveto", "moveto"] + ["translate"] * n + ["bump", "release", "mvr2z", "moveto"]
    # the landing retraces the unload lift's segments in reverse (reversed order,
    # each +X / -Z) -- so it mirrors the lift trajectory.
    steps = PAL12idb.CAROUSEL_SLIDE_STEPS
    slides = [e[1] for e in rob.log if e[0] == "translate"]
    assert slides == [[round(dx, 6), 0.0, round(-dz, 6)] for dx, dz in reversed(steps)]
    mixer = stations["carousel_on_mixer"]
    tdx = PAL12idb.CAROUSEL_SLIDE_TOTAL_DX
    tdz = PAL12idb.CAROUSEL_SLIDE_TOTAL_DZ
    # No unload measured the seat in this test, so place falls back to the taught
    # pose X,Y and the camera-based taught Z (taught_Z - teach_backoff). The
    # stand-off adds TOTAL_DZ + STANDOFF - net_under above the surface; the bump
    # seats the rest.
    surface = mixer[2] - PAL12idb.CAROUSEL_TEACH_BACKOFF
    net_under = PAL12idb.CAROUSEL_GRAB_UNDER - PAL12idb.CAROUSEL_BUMP_LIFT
    start_z = surface + tdz + PAL12idb.CAROUSEL_SNAP_LIFT - net_under
    # [-1] is now the transport-position park; the down-to-start is [-2].
    start = [e for e in rob.log if e[0] == "moveto"][-2]
    assert start[1][0] == pytest.approx(mixer[0] - tdx, abs=1e-6)  # back along -X for the slide
    assert start[1][1] == pytest.approx(mixer[1])
    assert start[1][2] == pytest.approx(start_z, abs=1e-6)  # the stand-off start before the landing
    assert start[1][3:] == mixer[3:]                       # seated at the mixer heading, not stock's
    assert start[1][0] + tdx == pytest.approx(mixer[0], abs=1e-6)  # reverse slide reaches the seat X
    assert state.get_carousel_on_mixer() == "37"       # the carousel it seated


def test_place_reuses_the_measured_mixer_seat(stations, quiet_camera, tmp_state, monkeypatch):
    # When an unload has measured the real mixer seat, place anchors X,Y AND Z on
    # it (not the taught pose), so the landing lands exactly where unload grabbed.
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (37, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    seat = [0.412, 0.097, 0.264]                            # measured seat x, y, z
    state.set_carousel_mixer_seat(seat)
    rob = FakeRobot()
    PAL12idb.place_new_carousel(rob)
    tdx = PAL12idb.CAROUSEL_SLIDE_TOTAL_DX
    tdz = PAL12idb.CAROUSEL_SLIDE_TOTAL_DZ
    net_under = PAL12idb.CAROUSEL_GRAB_UNDER - PAL12idb.CAROUSEL_BUMP_LIFT
    # [-1] is now the transport-position park; the down-to-start is [-2].
    start = [e for e in rob.log if e[0] == "moveto"][-2]
    assert start[1][0] == pytest.approx(seat[0] - tdx, abs=1e-6)  # start X back along -X from the MEASURED seat X
    assert start[1][1] == pytest.approx(seat[1])           # measured seat Y
    assert start[1][2] == pytest.approx(                   # anchored on the measured surface Z
        seat[2] + tdz + PAL12idb.CAROUSEL_SNAP_LIFT - net_under, abs=1e-6)
    assert start[1][0] + tdx == pytest.approx(seat[0], abs=1e-6)  # the slide landing reaches the measured seat X


def test_place_marks_mixer_unknown_until_seated(stations, quiet_camera, tmp_state, monkeypatch):
    # If it fails after the up-front clear, the mixer is left unknown, not stale.
    state.set_carousel_on_mixer(31)
    def boom(r, s, **k):
        raise RuntimeError("no new carousel in stock")
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", boom)
    with pytest.raises(RuntimeError, match="no new carousel"):
        PAL12idb.place_new_carousel(FakeRobot())
    assert state.get_carousel_on_mixer() == state.UNKNOWN


def test_place_acquires_with_the_load_approach_distance(stations, quiet_camera, tmp_state, monkeypatch):
    # Loading approaches to a set camera-to-tag distance before the grab.
    seen = {}
    def spy(r, s, approach_distance_m=None, **k):
        seen["station"] = s
        seen["approach"] = approach_distance_m
        return (37, 0.20)
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", spy)
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    PAL12idb.place_new_carousel(FakeRobot())
    assert seen["station"] == "carousel_in_stock"
    assert seen["approach"] == pytest.approx(PAL12idb.CAROUSEL_LOAD_APPROACH_M)


def test_unload_acquires_without_an_approach_distance(stations, quiet_camera, tmp_state, monkeypatch):
    # Unloading does not approach -- it grabs from the look height.
    monkeypatch.setattr(PAL12idb, "_look_for_used_stack_tag", lambda r: None)
    seen = {}
    def spy(r, s, approach_distance_m=None, **k):
        seen["approach"] = approach_distance_m
        return (33, 0.20)
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", spy)
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    PAL12idb.unload_current_carousel(FakeRobot())
    assert seen["approach"] is None


# -- carousel safe Z --------------------------------------------------------
SAFE_Z = 0.80          # above every FakeRobot / station height in these tests


def test_goto_crosses_at_the_safe_z(stations, monkeypatch):
    monkeypatch.setattr(PAL12idb, "get_carousel_safe_z", lambda: SAFE_Z)
    rob = FakeRobot()
    target = stations["carousel_used_stack"]
    PAL12idb._carousel_goto(rob, target)
    moves = [e[1] for e in rob.log if e[0] == "moveto"]
    assert moves[0][:2] == [0.30, -0.20] and moves[0][2] == SAFE_Z   # straight up
    assert moves[1][:2] == target[:2] and moves[1][2] == SAFE_Z     # across at the safe Z
    assert moves[2] == target                                        # straight down


def test_unload_carries_at_the_safe_z(stations, quiet_camera, tmp_state, monkeypatch):
    monkeypatch.setattr(PAL12idb, "get_carousel_safe_z", lambda: SAFE_Z)
    monkeypatch.setattr(PAL12idb, "_look_for_used_stack_tag", lambda r: None)
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (33, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    rob = FakeRobot()
    PAL12idb.unload_current_carousel(rob)
    moves = [e[1] for e in rob.log if e[0] == "moveto"]
    assert moves[-2][2] == SAFE_Z            # over the used stack at the safe Z
    assert moves[-1][2] == SAFE_Z            # and back up to it after the drop
    i = rob.log.index(("moveto", moves[-2]))
    assert rob.log[i - 1][0] == "mvr2z"      # straight up to it before crossing


def test_place_carries_at_the_safe_z(stations, quiet_camera, tmp_state, monkeypatch):
    monkeypatch.setattr(PAL12idb, "get_carousel_safe_z", lambda: SAFE_Z)
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (37, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    rob = FakeRobot()
    PAL12idb.place_new_carousel(rob)
    moves = [e[1] for e in rob.log if e[0] == "moveto"]
    assert moves[1][2] == SAFE_Z             # across to the stock at the safe Z
    assert moves[-3][2] == SAFE_Z            # carry to over the mixer at the safe Z


def test_safe_z_round_trips_through_the_ini(tmp_path, monkeypatch):
    monkeypatch.setattr(PAL12idb, "_INI_PATH", str(tmp_path / "waypoints.ini"))
    assert PAL12idb.get_carousel_safe_z() is None
    PAL12idb.store_carousel_safe_z(-0.0224)
    assert PAL12idb.get_carousel_safe_z() == pytest.approx(-0.0224)
    assert PAL12idb._carousel_travel_z(-0.10) == pytest.approx(-0.0224)
    assert PAL12idb._carousel_travel_z(0.05) == pytest.approx(0.05)  # never lowers a leg


# -- table rotation ---------------------------------------------------------
ROTATED_90 = {"origin_m": [0.3, -0.3, -0.44], "x_axis": [0.0, 1.0, 0.0],
              "y_axis": [-1.0, 0.0, 0.0], "normal": [0.0, 0.0, 1.0]}


def test_slide_follows_a_rotated_table(monkeypatch):
    # Table turned 90 deg (table +X = base +Y): the slide-out runs along base -Y.
    monkeypatch.setattr(PAL12idb, "load_venton_frame", lambda *a, **k: ROTATED_90)
    rob = FakeRobot()
    PAL12idb._carousel_lift(rob)
    slides = [e[1] for e in rob.log if e[0] == "translate"]
    for (dx, dz), v in zip(PAL12idb.CAROUSEL_SLIDE_STEPS, slides):
        assert v == pytest.approx([0.0, -dx, dz], abs=1e-6)


def test_place_slide_in_follows_a_rotated_table(stations, quiet_camera, tmp_state, monkeypatch):
    monkeypatch.setattr(PAL12idb, "load_venton_frame", lambda *a, **k: ROTATED_90)
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (37, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station, dist: r.log.append(("grab",)))
    seat = [0.412, 0.097, 0.264]
    state.set_carousel_mixer_seat(seat)
    rob = FakeRobot()
    PAL12idb.place_new_carousel(rob)
    tdx = PAL12idb.CAROUSEL_SLIDE_TOTAL_DX
    start = [e for e in rob.log if e[0] == "moveto"][-2]
    assert start[1][0] == pytest.approx(seat[0], abs=1e-6)        # no X offset now
    assert start[1][1] == pytest.approx(seat[1] - tdx, abs=1e-6)  # backed off along base -Y
    slides = [e[1] for e in rob.log if e[0] == "translate"]
    assert sum(v[1] for v in slides) == pytest.approx(tdx, abs=1e-6)  # slide-in reaches the seat
    assert sum(v[0] for v in slides) == pytest.approx(0.0, abs=1e-6)


def test_table_offset_is_identity_without_a_frame():
    assert PAL12idb._table_to_base_xy(0.01, 0.02) == (0.01, 0.02)


# -- plumbing ---------------------------------------------------------------
def test_command_builders():
    assert cmd.unload_carousel_command() == "unload_carousel"
    assert cmd.place_new_carousel_command() == "place_new_carousel"


def test_state_round_trips_and_defaults_unknown(tmp_state):
    assert state.get_carousel_on_mixer() == state.UNKNOWN
    state.set_carousel_on_mixer(34)
    assert state.get_carousel_on_mixer() == "34"
    state.set_carousel_on_mixer(None)
    assert state.get_carousel_on_mixer() == state.UNKNOWN


def test_mixer_seat_round_trips_and_defaults_none(tmp_state):
    assert state.get_carousel_mixer_seat() is None
    state.set_carousel_mixer_seat([0.41, 0.10, 0.2718])
    assert state.get_carousel_mixer_seat() == pytest.approx([0.41, 0.10, 0.2718])
    state.set_carousel_mixer_seat(None)
    assert state.get_carousel_mixer_seat() is None


def test_snapshot_carries_the_mixer_carousel(tmp_state):
    state.set_carousel_on_mixer(35)
    assert state.snapshot()["carousel_on_mixer"] == "35"


# -- locate the carousel on the mixer from a raster scan ---------------------
def _fake_scan(tags_xy_z):
    """Build a raster_tag_scan-shaped dict from {id: (x, y, z)}."""
    return {t: {"xyz": [xyz[0], xyz[1], xyz[2]], "n": 1, "seen_at": []}
            for t, xyz in tags_xy_z.items()}


def _patch_ids(monkeypatch):
    def ids(s):
        if s == "mixer_station":
            return {25, 26}
        if s == "carousel_on_mixer":
            return set(range(31, 41))
        return set()
    monkeypatch.setattr(PAL12idb, "apriltag_ids", ids)


def test_locate_mixer_carousel_identifies_and_stores(tmp_state, monkeypatch):
    _patch_ids(monkeypatch)
    head = (0.440, -0.270, 0.160)
    car = (head[0] + PAL12idb.MIXER_CAROUSEL_DX,          # exactly the expected point
           head[1] + PAL12idb.MIXER_CAROUSEL_DY, 0.120)
    far = (0.100, 0.440, 0.250)                            # another carousel, far away
    monkeypatch.setattr(PAL12idb, "raster_tag_scan",
                        lambda r, base_from, base_to: _fake_scan(
                            {26: head, 31: car, 32: far}))
    res = PAL12idb.locate_mixer_carousel(FakeRobot())
    assert res["mixer_carousel_id"] == 31
    assert res["carousel_xyz"] == pytest.approx(list(car))
    assert res["sample_xyz"] == pytest.approx(
        [car[0], car[1] + PAL12idb.SAMPLE_OFFSET_Y, car[2]])
    # stored as the carousel seat for later sample calcs
    assert state.get_carousel_mixer_seat() == pytest.approx(list(car))


def test_locate_mixer_carousel_no_match_leaves_seat(tmp_state, monkeypatch):
    _patch_ids(monkeypatch)
    state.set_carousel_mixer_seat([0.1, 0.2, 0.3])         # pre-existing seat
    head = (0.440, -0.270, 0.160)
    # carousel 50 mm off the expected offset point -- beyond the 10 mm tolerance
    car = (head[0] + PAL12idb.MIXER_CAROUSEL_DX + 0.050,
           head[1] + PAL12idb.MIXER_CAROUSEL_DY, 0.120)
    monkeypatch.setattr(PAL12idb, "raster_tag_scan",
                        lambda r, base_from, base_to: _fake_scan({26: head, 31: car}))
    res = PAL12idb.locate_mixer_carousel(FakeRobot())
    assert res["mixer_carousel_id"] is None
    assert state.get_carousel_mixer_seat() == pytest.approx([0.1, 0.2, 0.3])  # untouched


def test_locate_mixer_carousel_no_head(tmp_state, monkeypatch):
    _patch_ids(monkeypatch)
    monkeypatch.setattr(PAL12idb, "raster_tag_scan",
                        lambda r, base_from, base_to: _fake_scan({31: (0.3, -0.3, 0.1)}))
    res = PAL12idb.locate_mixer_carousel(FakeRobot())
    assert res["mixer_carousel_id"] is None
    assert "mixer-head" in res["summary"]


def test_locate_mixer_carousel_command_builder():
    assert cmd.locate_mixer_carousel_command() == "locate_mixer_carousel -100 20"
    assert cmd.locate_mixer_carousel_command(-80, 0) == "locate_mixer_carousel -80 0"


def test_locate_mixer_carousel_computes_venton_axes(tmp_state, monkeypatch):
    _patch_ids(monkeypatch)
    # Three Venton fiducials with known table coords; place them in base at yaw 0
    # about an origin so the fit should recover origin and identity axes.
    monkeypatch.setattr(PAL12idb, "venton_table_tags", lambda: {
        11: {"table_xy_mm": [0.0, 0.0]},
        12: {"table_xy_mm": [100.0, 0.0]},
        13: {"table_xy_mm": [0.0, 100.0]},
    })
    ox, oy, oz = 0.50, -0.30, 0.055
    scan = _fake_scan({
        11: (ox + 0.000, oy + 0.000, oz),
        12: (ox + 0.100, oy + 0.000, oz),
        13: (ox + 0.000, oy + 0.100, oz),
    })
    monkeypatch.setattr(PAL12idb, "raster_tag_scan",
                        lambda r, base_from, base_to: scan)
    res = PAL12idb.locate_mixer_carousel(FakeRobot())
    vf = res["venton_frame"]
    assert vf is not None
    assert vf["yaw_deg"] == pytest.approx(0.0, abs=1e-6)
    assert vf["origin_m"] == pytest.approx([ox, oy, oz])
    assert vf["x_axis"] == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)
    assert vf["y_axis"] == pytest.approx([0.0, 1.0, 0.0], abs=1e-6)
    assert vf["rmse_mm"] == pytest.approx(0.0, abs=1e-3)
