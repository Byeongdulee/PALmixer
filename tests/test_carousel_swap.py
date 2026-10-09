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
    return pos


@pytest.fixture
def quiet_camera(monkeypatch):
    # The camera is put over the tag with put_camera2tcp (logged by the fake);
    # only the corridor routing is stubbed to a plain move.
    monkeypatch.setattr(PAL12idb, "move_via_transferpoint",
                        lambda robot, target: robot.moveto(target[:3]))


@pytest.fixture
def tmp_state(tmp_path, monkeypatch):
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))


def _kinds(log):
    return [e[0] for e in log]


# -- the grab recipe --------------------------------------------------------
def test_grab_moves_to_taught_height_then_bumps_to_surface(monkeypatch, tmp_state):
    # Move down to the taught carousel height (Z_carousel), close the gripper and
    # bump to feel the real surface, up 1 mm, open wide, down 8 mm, close.
    monkeypatch.setattr(PAL12idb, "get_station_position",
                        lambda s: [0.40, 0.10, 0.30, 2.2, -2.2, 0.0])
    rob = FakeRobot()
    PAL12idb._grab_carousel(rob, "carousel_on_mixer")
    kinds = _kinds(rob.log)
    assert kinds == ["put_tcp2camera", "moveto", "close", "bump", "mvr2z",
                     "open", "mvr2z", "grab"]
    moveto = next(e for e in rob.log if e[0] == "moveto")
    assert moveto[1][2] == pytest.approx(0.30)               # Z_carousel (the taught Z)
    assert kinds.index("close") < kinds.index("bump")        # fully shut before the bump
    pre_bump = rob.log[kinds.index("close")]
    assert pre_bump[1] == PAL12idb.GRIPPER_CLOSE_COUNT       # all the way shut, not a force stop
    ups_downs = [e for e in rob.log if e[0] == "mvr2z"]
    assert ups_downs[0][1] == pytest.approx(PAL12idb.CAROUSEL_BUMP_LIFT)    # +1 mm
    assert ups_downs[1][1] == pytest.approx(-PAL12idb.CAROUSEL_GRAB_UNDER)  # -8 mm
    # grabbing off the mixer records the real seat the grab found: the gripper's
    # current X,Y (camera-centred in the real flow) with Z moved to the taught
    # height, for place to reuse so it lands exactly here. The grab keeps the
    # current X,Y (0.30, -0.20) and moves only Z to Z_carousel (0.30).
    assert state.get_carousel_mixer_seat() == pytest.approx([0.30, -0.20, 0.30])


def test_grab_switches_to_the_gripper_tcp(monkeypatch, tmp_state):
    # put_tcp2camera leaves the camera TCP active; the grab must put the gripper
    # TCP back so the carry/drop are in the gripper frame (the drop-at-wrong-
    # position bug).
    monkeypatch.setattr(PAL12idb, "get_station_position",
                        lambda s: [0.40, 0.10, 0.30, 2.2, -2.2, 0.0])
    rob = FakeRobot()
    PAL12idb._grab_carousel(rob, "carousel_on_mixer")
    assert "gripper" in rob.tcp_sets


def test_grab_without_a_taught_height_bumps_from_the_standoff(monkeypatch, tmp_state):
    def untaught(s):
        raise RuntimeError("not taught")
    monkeypatch.setattr(PAL12idb, "get_station_position", untaught)
    rob = FakeRobot()
    PAL12idb._grab_carousel(rob, "carousel_on_mixer")
    kinds = _kinds(rob.log)
    assert "moveto" not in kinds                  # no taught height to move to
    assert kinds == ["put_tcp2camera", "close", "bump", "mvr2z", "open", "mvr2z", "grab"]


# -- the seat-clearance lift ------------------------------------------------
def test_lift_path_is_up_diagonal_up():
    rob = FakeRobot()
    PAL12idb._carousel_lift(rob)
    d = PAL12idb.CAROUSEL_DIAG
    assert rob.log == [("mvr2z", round(PAL12idb.CAROUSEL_SNAP_LIFT, 6)),
                       ("translate", [round(-d, 6), 0.0, round(d, 6)]),
                       ("mvr2z", round(PAL12idb.CAROUSEL_CLEAR_LIFT, 6))]


# -- unload -----------------------------------------------------------------
def test_unload_sequence(stations, quiet_camera, tmp_state, monkeypatch):
    monkeypatch.setattr(PAL12idb, "_look_for_used_stack_tag", lambda r: None)  # empty stack
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (33, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station: r.log.append(("grab",)))
    rob = FakeRobot()
    PAL12idb.unload_current_carousel(rob)
    kinds = _kinds(rob.log)
    # look-lift, grab, lift(up/diag/up), carry over used stack, bump, release, back up.
    # The drop opens as soon as it bumps -- no forcing further down onto the stack.
    assert kinds == ["moveto", "mvr2z", "put_camera2tcp", "grab", "mvr2z",
                     "translate", "mvr2z", "moveto", "bump", "release", "moveto"]
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
                        lambda r, station: r.log.append(("grab",)))
    rob = FakeRobot()
    PAL12idb.unload_current_carousel(rob)
    kinds = _kinds(rob.log)
    assert kinds == ["moveto", "mvr2z", "put_camera2tcp", "grab", "mvr2z",
                     "translate", "mvr2z", "moveto", "moveto", "bump", "release", "moveto"]
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
                        lambda r, station: r.log.append(("grab",)))
    rob = FakeRobot()
    PAL12idb.place_new_carousel(rob)
    kinds = _kinds(rob.log)
    assert kinds == ["moveto", "moveto", "put_camera2tcp", "grab", "mvr2z",
                     "moveto", "moveto", "translate", "bump", "release", "mvr2z"]
    # the landing is exactly the reverse of the unload lift diagonal: +X / -Z by
    # DIAG (same 45 deg travel), no extra -- so it mirrors the lift.
    d = PAL12idb.CAROUSEL_DIAG
    slide = next(e for e in rob.log if e[0] == "translate")
    assert slide[1] == [round(d, 6), 0.0, round(-d, 6)]
    mixer = stations["carousel_on_mixer"]
    # No unload measured the seat in this test, so place falls back to the taught
    # pose X,Y and the camera-based taught Z (taught_Z - teach_backoff). The
    # stand-off adds DIAG + STANDOFF - net_under above the surface; the bump seats
    # the rest.
    surface = mixer[2] - PAL12idb.CAROUSEL_TEACH_BACKOFF
    net_under = PAL12idb.CAROUSEL_GRAB_UNDER - PAL12idb.CAROUSEL_BUMP_LIFT
    start_z = surface + d + PAL12idb.CAROUSEL_PLACE_STANDOFF - net_under
    start = [e for e in rob.log if e[0] == "moveto"][-1]
    assert start[1][0] == pytest.approx(mixer[0] - d)      # back along -X for the diagonal
    assert start[1][1] == pytest.approx(mixer[1])
    assert start[1][2] == pytest.approx(start_z)           # the stand-off start before the landing
    assert start[1][3:] == mixer[3:]                       # seated at the mixer heading, not stock's
    assert start[1][0] + d == pytest.approx(mixer[0])      # reverse diagonal reaches the seat X
    assert state.get_carousel_on_mixer() == "37"       # the carousel it seated


def test_place_reuses_the_measured_mixer_seat(stations, quiet_camera, tmp_state, monkeypatch):
    # When an unload has measured the real mixer seat, place anchors X,Y AND Z on
    # it (not the taught pose), so the landing lands exactly where unload grabbed.
    monkeypatch.setattr(PAL12idb, "_acquire_carousel", lambda r, s, **k: (37, 0.20))
    monkeypatch.setattr(PAL12idb, "_grab_carousel",
                        lambda r, station: r.log.append(("grab",)))
    seat = [0.412, 0.097, 0.264]                            # measured seat x, y, z
    state.set_carousel_mixer_seat(seat)
    rob = FakeRobot()
    PAL12idb.place_new_carousel(rob)
    d = PAL12idb.CAROUSEL_DIAG
    net_under = PAL12idb.CAROUSEL_GRAB_UNDER - PAL12idb.CAROUSEL_BUMP_LIFT
    start = [e for e in rob.log if e[0] == "moveto"][-1]
    assert start[1][0] == pytest.approx(seat[0] - d)       # start X back along -X from the MEASURED seat X
    assert start[1][1] == pytest.approx(seat[1])           # measured seat Y
    assert start[1][2] == pytest.approx(                   # anchored on the measured surface Z
        seat[2] + d + PAL12idb.CAROUSEL_PLACE_STANDOFF - net_under)
    assert start[1][0] + d == pytest.approx(seat[0])       # the DIAG landing reaches the measured seat X


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
                        lambda r, station: r.log.append(("grab",)))
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
                        lambda r, station: r.log.append(("grab",)))
    PAL12idb.unload_current_carousel(FakeRobot())
    assert seen["approach"] is None


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
