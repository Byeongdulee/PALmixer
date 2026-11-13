"""Carousel loads free every slot; coordinated carousel replacement.

Fakes only: temporary state and mixing-record directories, a fake PAL12idb whose
unload/place only update tracked state, and no robot, camera, pump or network.
"""
import ast
from datetime import datetime, timedelta
import functools
import json
from pathlib import Path
import threading
from types import SimpleNamespace
from uuid import uuid4

import pytest

from palmixer import commands as cmd, preparation_api, state
from palmixer.mixing_records import MixingJournal, new_record
from palmixer.preparation_api import NO_STOCK_MESSAGE, PreparationAPI
from palmixer.server import PALmixerServer
from palmixer.workflows import Workflows

RUN = "752e4fc0-fa3d-47ac-ab9e-68624b0763a4"
UID_ONE = "0b9f4a52-2f43-4c1e-9d55-3c7d3b1e6a01"
UID_TWO = "6a1d3a0e-8c84-4f0d-a1c4-55d4e3f2b902"


@pytest.fixture
def tracked(monkeypatch, tmp_path):
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "state.ini"))
    monkeypatch.setattr(state, "get_carousel_size", lambda: 8)
    monkeypatch.setattr(state, "get_carousel_step", lambda: 4.2)
    state.mount_carousel("tray-A", {1: "S-one", 2: "S-two", 5: "S-five"})
    state.set_sample_uid(1, "S-one", UID_ONE)
    state.set_sample_uid(2, "S-two", UID_TWO)
    state.teach_carousel_slot(2, 15.5)
    state.set_carousel_slot(3)
    state.set_mixer_head(state.MIXER_AT_CLEANING)
    state.set_flowcell_location(1, state.FC_AT_CLEANING)
    return tmp_path


@pytest.fixture
def journal(tmp_path):
    return MixingJournal(tmp_path / "records")


def save_mix(journal, sample_id, sample_uid="", status="completed"):
    record = new_record(sample_id, 1, "tray-A", 1, sample_uid=sample_uid)
    record["mixing_status"] = status
    journal.save(record)
    return record


# -- MixingJournal.was_mixed --------------------------------------------------
def test_was_mixed_matches_uid_when_given_else_sample_id(journal):
    save_mix(journal, "S-one", UID_ONE, "completed")
    save_mix(journal, "S-two", UID_TWO, "not_started")
    save_mix(journal, "S-three", "", "stopped")
    assert journal.was_mixed("S-one", UID_ONE)
    assert journal.was_mixed("relabelled", UID_ONE)          # the UUID decides
    assert not journal.was_mixed("S-one", UID_TWO)           # ... even against the ID
    assert not journal.was_mixed("S-two", UID_TWO)           # never started mixing
    assert journal.was_mixed("S-three", "")                  # a stopped mix still drew on the vial
    assert not journal.was_mixed("S-unknown", "")
    assert not journal.was_mixed("", "")


def test_was_mixed_skips_bad_files_and_tolerates_a_missing_directory(journal, tmp_path):
    assert not MixingJournal(tmp_path / "absent").was_mixed("S-one", "")
    journal.directory.mkdir(parents=True)
    (journal.directory / "corrupt.json").write_text("not json")
    (journal.directory / "bad-envelope.json").write_text('{"record": []}')
    (journal.directory / "no-status.json").write_text(
        json.dumps({"record": {"sample_id": "S-one"}, "pvapp_sync": {}}))
    assert not journal.was_mixed("S-one", "")
    save_mix(journal, "S-one")
    assert journal.was_mixed("S-one", "")


# -- the carousel-load record ---------------------------------------------------
def test_load_record_contents_and_one_atomic_transition(tracked, journal, monkeypatch):
    save_mix(journal, "S-one", UID_ONE, "completed")
    save_mix(journal, "S-two", UID_TWO, "not_started")
    save_mix(journal, "S-five", "", "unknown")
    state.set_last_unloaded_carousel(33)
    before = state.snapshot()
    seen, writes = [], []
    monkeypatch.setattr(state, "_listeners", [seen.append])
    original_write = state._write
    monkeypatch.setattr(state, "_write", lambda parser: writes.append(1) or original_write(parser))

    record = state.record_carousel_load("37", journal.was_mixed)

    assert len(writes) == 1 and len(seen) == 1       # one transition, one notification
    assert len(record["load_id"]) == 32 and int(record["load_id"], 16) >= 0
    assert record["sequence"] == 1
    assert datetime.fromisoformat(record["at"]).utcoffset() == timedelta(0)
    assert (record["old_tag"], record["new_tag"], record["replacement_id"]) == ("33", "37", None)
    assert record["slots"] == [
        {"slot": 1, "sample_id": "S-one", "sample_uid": UID_ONE, "mixed": True},
        {"slot": 2, "sample_id": "S-two", "sample_uid": UID_TWO, "mixed": False},
        {"slot": 5, "sample_id": "S-five", "sample_uid": "", "mixed": True},
    ]
    after = state.snapshot()
    assert seen == [after]                           # never a half-cleared inventory
    assert after == dict(before, carousel_samples={}, carousel_sample_uids={},
                         carousel_full=False, carousel_load={"sequence": 1, "record": record})
    # Label, taught reference and tracked slot survive; the old tag is consumed.
    assert after["carousel_id"] == "tray-A"
    assert after["carousel_reference"] == (2, 15.5) and after["carousel_slot"] == 3
    assert state.get_last_unloaded_carousel() == state.UNKNOWN
    raw = state._read().get("carousel_load", "record")
    assert json.loads(raw) == record and ": " not in raw and ", " not in raw


def test_sequence_increments_and_unknown_tags_are_recorded_as_unknown(tracked, journal):
    first = state.record_carousel_load(None, journal.was_mixed)
    state.set_sample_id(4, "S-four")
    replacement = str(uuid4())
    second = state.record_carousel_load("", journal.was_mixed, replacement)
    assert (first["sequence"], second["sequence"]) == (1, 2)
    assert first["load_id"] != second["load_id"]
    assert (first["old_tag"], first["new_tag"]) == ("unknown", "unknown")
    assert (second["old_tag"], second["new_tag"]) == ("unknown", "unknown")
    assert second["replacement_id"] == replacement
    assert second["slots"] == [{"slot": 4, "sample_id": "S-four", "sample_uid": "", "mixed": False}]
    assert state.carousel_load() == {"sequence": 2, "record": second}


def test_raw_slot_tables_are_recorded_beyond_the_configured_size(tracked, journal):
    orphan = str(uuid4())
    def hand_edit(parser):
        parser.set("carousel_samples", "slot_12", "S-twelve")
        parser.set("carousel_sample_uids", "slot_7", orphan)
    state._mutate(hand_edit)
    assert 12 not in state.carousel_samples()          # outside size 8
    slots = {s["slot"]: s for s in state.record_carousel_load(31, journal.was_mixed)["slots"]}
    assert sorted(slots) == [1, 2, 5, 7, 12]
    assert slots[7] == {"slot": 7, "sample_id": "", "sample_uid": orphan, "mixed": False}
    assert slots[12]["sample_id"] == "S-twelve"
    assert state._slot_keys(state._read(), "carousel_samples") == {}
    assert state._slot_keys(state._read(), "carousel_sample_uids") == {}


def test_no_load_is_reported_before_any_placement_and_reset_writes_none(tracked):
    assert state.carousel_load() == {"sequence": 0, "record": None}
    assert state.snapshot()["carousel_load"] == {"sequence": 0, "record": None}
    state.reset_carousel()
    assert state.carousel_samples() == {}
    assert state.carousel_load() == {"sequence": 0, "record": None}


# -- server: Place New Carousel and the mixing record ----------------------------
class FakePAL:
    """Stands in for PAL12idb: unload/place only update tracked state, and the
    workflow park (goto_transfer_point) is counted in `parks`, apart from the log."""

    class NoCarouselError(RuntimeError):
        pass

    class CarouselNotSeatedError(RuntimeError):
        pass

    def __init__(self):
        self.log = []
        self.missing = []
        self.unload_error = self.place_error = self.park_error = None
        self.place_notes = []
        self.parks = 0
        self.during_unload = self.during_place = self.during_park = lambda: None

    def check_positions_defined(self, keys):
        return [k for k in keys if k in self.missing]

    def goto_transfer_point(self, robot):
        self.parks += 1
        self.during_park()
        if self.park_error:
            raise self.park_error
        return [0.2, -0.19, 0.08]

    def unload_current_carousel(self, robot):
        self.log.append("unload")
        self.during_unload()
        if self.unload_error:
            raise self.unload_error
        state.set_last_unloaded_carousel(33)
        state.set_carousel_on_mixer(None)
        return [0.1, 0.2, 0.3]

    def place_new_carousel(self, robot, notes=None):
        self.log.append("place")
        if notes is not None:
            notes.extend(self.place_notes)
        state.set_carousel_on_mixer(None)
        self.during_place()
        if self.place_error:
            raise self.place_error
        state.set_carousel_on_mixer(37)
        return [0.4, 0.5, 0.6]



@pytest.fixture
def server(tracked, journal, monkeypatch):
    # No table turn unless a test configures one (never the bench's config file).
    monkeypatch.setattr(state, "get_carousel_place_alignment_slot", lambda: None)
    instance = object.__new__(PALmixerServer)
    instance.simulate = False
    instance.PAL12idb = FakePAL()
    instance.rob = object()
    instance._busy = False
    instance._busy_lock = threading.Lock()
    instance._current_action = instance._current_trace = instance._current_step = None
    instance._last_result = None
    instance._beamline = "test"
    instance._mqtt = SimpleNamespace(publish=lambda *a, **k: None)
    # Workflows._park drives the fake PAL's goto_transfer_point.
    instance.workflows = Workflows(instance.PAL12idb, instance.rob, None, None)
    instance._mixing_journal = journal
    return instance


def test_place_button_frees_every_slot_and_reports_the_load(server, journal):
    save_mix(journal, "S-one", UID_ONE)
    state.set_last_unloaded_carousel(32)
    reply = server._place_new_carousel()
    assert "tag 37" in reply and "every slot cleared (carousel load 1)" in reply
    assert state.carousel_samples() == {} and state.carousel_sample_uids() == {}
    record = state.carousel_load()["record"]
    assert (record["old_tag"], record["new_tag"], record["replacement_id"]) == ("32", "37", None)
    assert [s["mixed"] for s in record["slots"]] == [True, False, False]
    assert state.get_carousel_id() == "tray-A"


def test_place_button_needs_the_used_stack_for_a_take_back(server):
    server.PAL12idb.missing = ["carousel_used_stack"]
    with pytest.raises(ValueError, match="Used Carousel Stack"):
        server._resolve(cmd.PLACE_NEW_CAROUSEL, [])
    assert server.PAL12idb.log == []


def test_a_rejected_carousel_is_named_in_the_reply(server):
    server.PAL12idb.place_notes = ["carousel 35 did not land (its top is 6.0 mm above the "
                                   "seat) and was moved to the used stack, unused; carousel "
                                   "37 placed instead"]
    reply = server._place_new_carousel()
    assert "tag 37" in reply and "carousel 35 did not land" in reply
    assert "every slot cleared (carousel load 1)" in reply


@pytest.mark.parametrize("error", [RuntimeError("protective stop"),
                                   FakePAL.NoCarouselError("no carousel at carousel_in_stock")])
def test_failed_placement_clears_nothing(server, error):
    state.set_last_unloaded_carousel(32)
    server.PAL12idb.place_error = error
    before = state.snapshot()
    with pytest.raises(RuntimeError):
        server._place_new_carousel()
    after = state.snapshot()
    assert after["carousel_samples"] == before["carousel_samples"]
    assert after["carousel_sample_uids"] == before["carousel_sample_uids"]
    assert after["carousel_load"] == {"sequence": 0, "record": None}
    assert state.get_last_unloaded_carousel() == "32"   # still owed to the next load
    assert server.PAL12idb.log == ["place"]
    assert server.PAL12idb.parks == 0                   # and no park


def test_place_button_ends_with_the_standard_park(server):
    # An earlier workflow parked; this worker action clears that, and the park sets it again.
    server.workflows._step("park_at_transfer_point", lambda: "parked")
    server._run_action(server._place_new_carousel, cmd.PLACE_NEW_CAROUSEL, "trace-1")
    assert server._last_result["ok"] is True, server._last_result
    assert server._last_result["detail"].endswith(
        "; every slot cleared (carousel load 1); arm parked")
    assert server.PAL12idb.log == ["place"]
    assert server.PAL12idb.parks == 1
    assert server.workflows.cleanup_snapshot()["parked"] is True


TAKEN_BACK = ("carousel 35 did not land (its top is 6.0 mm above the seat) and was moved to the "
              "used stack, unused; carousel 37 placed instead")


@pytest.mark.parametrize("notes", [[], [TAKEN_BACK]], ids=["placed", "placed_after_a_take_back"])
def test_a_failed_park_after_the_button_placement_names_the_recorded_load(server, notes):
    server.PAL12idb.place_notes = notes
    server.PAL12idb.park_error = RuntimeError("protective stop")
    with pytest.raises(RuntimeError) as failure:
        server._place_new_carousel()
    # What the placement did, a carousel taken back included, then the load and the cause.
    assert str(failure.value) == "; ".join(
        ["new carousel placed on the mixer (tag 37, arm at [0.4, 0.5, 0.6])"] + notes
        + ["every slot cleared (carousel load 1)", "parking the arm failed: protective stop"])
    assert failure.value.__cause__ is server.PAL12idb.park_error
    # The placement stands: the load is recorded and every slot is free.
    assert state.carousel_load()["sequence"] == 1 and state.carousel_samples() == {}
    assert server.PAL12idb.log == ["place"]
    assert server.PAL12idb.parks == 1
    assert server.workflows.cleanup_snapshot()["parked"] is False


def test_simulated_unload_and_place_record_unknown_tags(server, monkeypatch):
    import palmixer.server as server_module
    monkeypatch.setattr(server_module.time, "sleep", lambda s: None)
    server.simulate = True
    state.set_last_unloaded_carousel(32)
    server._unload_carousel()
    assert state.get_last_unloaded_carousel() == state.UNKNOWN
    assert "carousel load 1" in server._place_new_carousel()
    record = state.carousel_load()["record"]
    assert (record["old_tag"], record["new_tag"]) == ("unknown", "unknown")
    assert state.carousel_samples() == {}


def test_mixing_record_carries_the_carousel_tag(server, journal):
    assert new_record("S", 1, "C", 1)["carousel_tag"] == ""
    state.set_carousel_on_mixer(37)
    server._active_mix_record = server._mix_record_thread = server._last_mixing_record = None
    server._mix_sync_wake = threading.Event()
    server.workflows = SimpleNamespace(make_sample=lambda slot, sample_id: "done")
    server._make_sample_recorded(1, None)
    record = json.loads(next(journal.directory.glob("*.json")).read_text())["record"]
    assert (record["carousel_id"], record["carousel_tag"]) == ("tray-A", "37")
    assert (record["sample_id"], record["sample_uid"]) == ("S-one", UID_ONE)


# -- placement turns the table to the alignment slot first --------------------------
class FakeMotor:
    """Records each absolute move in the PAL log, so the order against the arm shows."""

    def __init__(self, log):
        self.log = log
        self.error = None

    def move_to(self, position, timeout=120.0):
        if self.error:
            raise self.error
        self.log.append(("turn", position))
        return position


# The tracked fixture teaches slot 2 at 15.5 with a 4.2 step: slot 4 is at 23.9.
SLOT_4 = 15.5 + 2 * 4.2


@pytest.fixture
def aligned(server, monkeypatch):
    monkeypatch.setattr(state, "get_carousel_place_alignment_slot", lambda: 4)
    state.set_flowcell_location(2, state.FC_AT_CLEANING)
    server.motor = FakeMotor(server.PAL12idb.log)
    return server


def test_place_turns_the_table_to_the_alignment_slot_before_the_arm_moves(aligned):
    reply = aligned._place_new_carousel()
    assert aligned.PAL12idb.log == [("turn", pytest.approx(SLOT_4)), "place"]
    assert aligned.PAL12idb.parks == 1
    assert state.get_carousel_slot() == 4
    assert "table turned to slot 4" in reply and "every slot cleared (carousel load 1)" in reply


def test_without_an_alignment_slot_the_table_stays_put(server):
    server.motor = FakeMotor(server.PAL12idb.log)
    server._place_new_carousel()
    assert server.PAL12idb.log == ["place"]
    assert server.PAL12idb.parks == 1
    assert state.get_carousel_slot() == 3


@pytest.mark.parametrize("obstacle,match", [
    ("mixer_head", "mixer head must be at its cleaning station"),
    ("flowcell_in_gripper", "flowcell 1 must be at its cleaning station or the beam"),
    ("flowcell_unknown", "flowcell 1 must be at its cleaning station or the beam"),
    ("other_flowcell_in_gripper", "flowcell 2 must be at its cleaning station or the beam"),
    ("untaught", "has not been taught"),
    ("slot_off_the_carousel", "slot"),
])
def test_place_is_refused_before_anything_moves_when_the_table_cannot_turn(
        aligned, monkeypatch, obstacle, match):
    if obstacle == "mixer_head":
        state.set_mixer_head(state.MIXER_AT_MIXER)
    elif obstacle == "flowcell_in_gripper":
        state.set_flowcell_location(1, state.FC_IN_GRIPPER)
    elif obstacle == "flowcell_unknown":
        state.set_flowcell_location(1, state.UNKNOWN)
    elif obstacle == "other_flowcell_in_gripper":
        state.set_flowcell_location(2, state.FC_IN_GRIPPER)     # flowcell 1 is in use
    elif obstacle == "untaught":
        monkeypatch.setattr(state, "carousel_reference", lambda: None)
    else:
        monkeypatch.setattr(state, "get_carousel_place_alignment_slot", lambda: 9)  # 8 slots
    with pytest.raises(ValueError, match=match):
        aligned._resolve(cmd.PLACE_NEW_CAROUSEL, [])       # the synchronous ERROR reply
    with pytest.raises(ValueError, match=match):
        aligned._place_new_carousel()                      # re-checked when it starts
    assert aligned.PAL12idb.log == []
    assert state.get_carousel_slot() == 3
    assert state.carousel_load() == {"sequence": 0, "record": None}


def test_place_with_the_flowcell_at_the_beam_still_turns(aligned):
    state.set_flowcell_location(1, state.FC_AT_BEAM)
    aligned._resolve(cmd.PLACE_NEW_CAROUSEL, [])
    aligned._place_new_carousel()
    assert aligned.PAL12idb.log == [("turn", pytest.approx(SLOT_4)), "place"]
    assert aligned.PAL12idb.parks == 1


def test_a_failed_turn_leaves_the_new_carousel_in_stock(aligned):
    aligned.motor.error = RuntimeError("Galil did not answer")
    with pytest.raises(RuntimeError, match="Galil did not answer"):
        aligned._place_new_carousel()
    assert aligned.PAL12idb.log == []                       # the arm never moved
    assert state.get_carousel_slot() == 3                   # not recorded as turned
    assert state.carousel_load() == {"sequence": 0, "record": None}


def test_simulated_place_records_the_alignment_slot(aligned, monkeypatch):
    import palmixer.server as server_module
    monkeypatch.setattr(server_module.time, "sleep", lambda s: None)
    aligned.simulate = True
    aligned.workflows.simulate = True    # the real server gives its workflows the same flag
    aligned.motor.error = RuntimeError("a simulated place must not touch the motor")
    aligned._place_new_carousel()
    assert aligned.PAL12idb.log == []
    assert state.get_carousel_slot() == 4


@pytest.mark.parametrize("value,expected", [(None, None), (11, 11)])
def test_alignment_slot_comes_from_the_carousel_config(monkeypatch, value, expected):
    from palmixer import config
    monkeypatch.setattr(config, "get_section", lambda name: {"place_alignment_slot": value})
    assert state.get_carousel_place_alignment_slot() == expected


@pytest.mark.parametrize("value", ["11", 11.5, True])
def test_alignment_slot_must_be_a_whole_number(monkeypatch, value):
    from palmixer import config
    monkeypatch.setattr(config, "get_section", lambda name: {"place_alignment_slot": value})
    with pytest.raises(ValueError, match="place_alignment_slot"):
        state.get_carousel_place_alignment_slot()


# -- PAL12idb pieces, compiled from source (UR_12idb is not importable here) ------
PAL_SOURCE = Path(__file__).resolve().parents[1] / "palmixer" / "PAL12idb.py"


def pal_definitions(names, **env):
    tree = ast.parse(PAL_SOURCE.read_text(encoding="utf-8"))
    body = [node for node in tree.body
            if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    assert {node.name for node in body} == set(names)
    env.setdefault("functools", functools)
    exec(compile(ast.Module(body=body, type_ignores=[]), str(PAL_SOURCE), "exec"), env)
    return env


def test_acquire_raises_no_carousel_error_for_an_empty_seat():
    env = pal_definitions({"NoCarouselError", "_acquire_carousel"},
                          camera_tools=SimpleNamespace(_detect_apriltag=lambda robot, settle: None),
                          apriltag_size=lambda station: 0.03,
                          apriltag_ids=lambda station: {31, 32}, CAROUSEL_CENTER_SETTLE=0)
    robot = SimpleNamespace(tcp=[0] * 6, set_tcp=lambda tcp: None,
                            camera=SimpleNamespace(AT_ids=[7]))
    assert issubclass(env["NoCarouselError"], RuntimeError)
    with pytest.raises(env["NoCarouselError"], match=r"no carousel at carousel_in_stock: .*\[31, 32\]"):
        env["_acquire_carousel"](robot, "carousel_in_stock")


def test_unload_keeps_the_tag_it_reads(tracked):
    state.set_carousel_on_mixer(33)
    robot = SimpleNamespace(tcp=[0] * 6, set_tcp=lambda tcp: None, mvr2z=lambda dz: None,
                            put_camera2tcp=lambda: None, get_pos=lambda: [0.4, 0.1, 0.5],
                            moveto=lambda pose: None, bump=lambda **k: None, release=lambda: None)
    env = pal_definitions({"_tracks", "unload_current_carousel"}, state=state,
                          _look_for_used_stack_tag=lambda r: None,
                          _carousel_goto=lambda r, target: None,
                          get_station_position=lambda s: [0.4, 0.1, 0.3, 2.2, -2.2, 0.1],
                          CAROUSEL_LOOK_LIFT=0.05, CAROUSEL_STACK_ABOVE=0.1,
                          _acquire_carousel=lambda r, station: (33, 0.2),
                          _grab_carousel=lambda r, station, dist: None,
                          _carousel_lift=lambda r: None,
                          _carousel_travel_z=lambda *zs: max(zs), _pose_str=lambda r: "")
    env["unload_current_carousel"](robot)
    assert state.get_last_unloaded_carousel() == "33"
    assert state.get_carousel_on_mixer() == state.UNKNOWN


# -- coordinated replace_carousel ----------------------------------------------------
@pytest.fixture
def coordinated(server, tracked, monkeypatch):
    started = []

    class RecordingThread(threading.Thread):
        def start(self):
            started.append(self)
            super().start()

    monkeypatch.setattr(preparation_api, "threading",
                        SimpleNamespace(Thread=RecordingThread, RLock=threading.RLock))
    # These tests exercise the full robot swap; the blocked-placement test turns it off.
    monkeypatch.setattr(preparation_api, "auto_place_new_carousel", lambda: True)
    api = PreparationAPI(server, tracked / "session.json")
    server.preparation_api = api
    server.workflows.pal = server.PAL12idb          # a finished swap parks the arm
    server.workflows.robot = server.rob
    api.request({"version": 1, "run_id": RUN, "op": "begin"})

    def finish():
        for thread in started:
            thread.join(5)
            assert not thread.is_alive()
        return len(started)

    yield SimpleNamespace(api=api, server=server, pal=server.PAL12idb, finish=finish,
                          started=started, path=tracked / "session.json")
    api.coordinator.close()


def replace_message(replacement_id=None, **extra):
    replacement_id = replacement_id or str(uuid4())
    return dict(version=1, run_id=RUN, op="replace_carousel", replacement_id=replacement_id,
                action_key="replace:" + replacement_id, **extra)


def persisted(c, replacement_id):
    return json.loads(c.path.read_text())["replacements"].get(replacement_id)


def status(c):
    return c.api.request({"version": 1, "run_id": RUN, "op": "status"})


def test_replace_unloads_places_and_commits_the_load_while_washing(coordinated, journal):
    c = coordinated
    save_mix(journal, "S-one", UID_ONE)
    # The flowcell wash from the last unload is still running: the intended overlap.
    c.server.workflows._cleanup_operations["wash_flowcell"] = {"status": "running", "error": ""}
    message = replace_message()
    rid = message["replacement_id"]
    observed = {}
    c.pal.during_unload = lambda: observed.update(
        unload=persisted(c, rid)["state"], busy=c.server._busy,
        action=c.server._current_action)
    c.pal.during_place = lambda: observed.update(place=persisted(c, rid)["state"])

    reply = json.loads(c.server.fast_dispatch("preparation_control " + json.dumps(message)))
    assert reply["ok"], reply
    accepted = reply["result"]
    assert c.finish() == 1

    assert set(accepted) == {"replacement_id", "action_id", "request", "state", "error"}
    assert (accepted["replacement_id"], accepted["error"]) == (rid, "")
    assert accepted["state"] == "accepted"      # returned before the worker can save progress
    assert observed == {"unload": "unloading", "busy": True,
                        "action": "preparation_replace_carousel", "place": "placing"}
    assert c.pal.log == ["unload", "place"]
    assert c.pal.parks == 1
    final = status(c)["replacements"][rid]
    assert final["state"] == "committed" and final["error"] == ""
    assert final["action_id"] == accepted["action_id"] == c.server._last_result["action_id"]
    load = final["load"]
    assert (load["old_tag"], load["new_tag"], load["replacement_id"]) == ("33", "37", rid)
    assert [(s["slot"], s["mixed"]) for s in load["slots"]] == [(1, True), (2, False), (5, False)]
    assert state.carousel_load() == {"sequence": 1, "record": load}
    assert state.carousel_samples() == {} and state.get_carousel_id() == "tray-A"
    assert persisted(c, rid) == final
    assert c.server._busy is False and c.server._last_result["ok"] is True
    assert c.api.record["error"] == "" and c.api.coordinator.snapshot()["error"] == ""


def test_duplicate_replace_never_starts_a_second_swap(coordinated):
    c = coordinated
    message = replace_message()
    rid = message["replacement_id"]
    in_flight = []
    c.pal.during_unload = lambda: in_flight.append(c.api.request(dict(message)))
    first = c.api.request(message)
    c.finish()
    assert [r["state"] for r in in_flight] == ["unloading"]
    assert in_flight[0]["action_id"] == first["action_id"]
    again = c.api.request(dict(message))
    assert again["state"] == "committed" and again["action_id"] == first["action_id"]
    assert len(c.started) == 1 and c.pal.log == ["unload", "place"] and c.pal.parks == 1
    with pytest.raises(ValueError, match="different contents"):
        c.api.request(dict(message, note="changed"))
    assert state.carousel_load()["sequence"] == 1
    # A new identity is a new swap request, checked like any other.
    c.api.request(replace_message())
    c.finish()
    assert c.pal.log == ["unload", "place", "unload", "place"]
    assert c.pal.parks == 2
    assert state.carousel_load()["sequence"] == 2
    assert set(status(c)["replacements"]) >= {rid}


def test_waiting_for_stock_after_a_take_back_names_the_rejected_carousel(coordinated):
    c = coordinated
    c.pal.place_error = FakePAL.NoCarouselError(
        "no carousel at carousel_in_stock. Carousel 35, the first one, did not land (its "
        "top is 6.0 mm above the seat) and is on the used stack, unused.")
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    record = status(c)["replacements"][rid]
    assert record["state"] == "waiting_for_stock" and record["error"] == NO_STOCK_MESSAGE
    assert "Carousel 35, the first one, did not land" in record["detail"]
    assert "Carousel 35, the first one, did not land" in c.server._last_result["detail"]


def test_empty_stock_seat_waits_for_the_operator_without_failing_the_session(coordinated):
    c = coordinated
    c.pal.place_error = FakePAL.NoCarouselError("no carousel at carousel_in_stock: tags in view []")
    message = replace_message()
    rid = message["replacement_id"]
    c.api.request(message)
    c.finish()
    record = status(c)["replacements"][rid]
    assert record["state"] == "waiting_for_stock" and record["error"] == NO_STOCK_MESSAGE
    assert "load" not in record
    assert c.api.record["error"] == "" and c.api.coordinator.snapshot()["error"] == ""
    assert c.server._busy is False and c.server._last_result["ok"] is True
    # Nothing was cleared: the placement never seated anything.
    assert sorted(state.carousel_samples()) == [1, 2, 5]
    assert state.carousel_load()["sequence"] == 0
    assert c.api.request(dict(message))["state"] == "waiting_for_stock"   # still no replay
    assert c.pal.log == ["unload", "place"]
    assert c.pal.parks == 1
    # PALsystem can end the session; the operator's Place New Carousel finishes the job.
    assert c.api.request({"version": 1, "run_id": RUN, "op": "end"})["active"] is False
    c.pal.place_error = None
    assert "carousel load 1" in c.server._place_new_carousel()
    load = state.carousel_load()["record"]
    assert (load["old_tag"], load["new_tag"], load["replacement_id"]) == ("33", "37", None)
    assert state.carousel_samples() == {}


@pytest.mark.parametrize("where,error,phase", [
    ("unload", RuntimeError("protective stop during unload"), "unloading"),
    ("unload", FakePAL.NoCarouselError("no carousel at carousel_on_mixer"), "unloading"),
    ("place", RuntimeError("protective stop during place"), "placing"),
])
def test_other_failures_latch_the_session(coordinated, where, error, phase):
    c = coordinated
    setattr(c.pal, where + "_error", error)
    message = replace_message()
    rid = message["replacement_id"]
    c.api.request(message)
    c.finish()
    record = status(c)["replacements"][rid]
    assert (record["state"], record["failed_phase"], record["error"]) == ("failed", phase, str(error))
    assert c.api.record["error"] == str(error)
    assert c.api.coordinator.snapshot()["error"] == str(error)
    assert c.server._busy is False and c.server._last_result["ok"] is False
    assert sorted(state.carousel_samples()) == [1, 2, 5]
    assert state.carousel_load()["sequence"] == 0
    with pytest.raises(ValueError, match=str(error)):
        c.api.request(replace_message())               # no further motion until reconciled
    assert c.pal.log == (["unload"] if where == "unload" else ["unload", "place"])


@pytest.mark.parametrize("obstacle,match", [
    ("other_run", "does not own"),
    ("session_error", "inspect"),
    ("bad_id", "Canonical replacement UUID"),
    ("bad_key", "action_key"),
    ("coordinator", "owns the carousel"),
    ("busy", "busy"),
    ("cleanup_problem", "cleanup needs attention"),
    ("mixer_head", "mixer head"),
    ("flowcell", "Flowcell 1"),
    ("positions", "carousel_in_stock|Carousel in Stock|stock"),
])
def test_preconditions_refuse_before_accepting(coordinated, obstacle, match):
    c = coordinated
    message = replace_message()
    if obstacle == "other_run":
        message["run_id"] = str(uuid4())
    elif obstacle == "session_error":
        c.api.record["error"] = "Earlier failure; inspect before further motion."
    elif obstacle == "bad_id":
        message["replacement_id"] = message["replacement_id"].upper()
        message["action_key"] = "replace:" + message["replacement_id"]
    elif obstacle == "bad_key":
        message["action_key"] = "replace"
    elif obstacle == "coordinator":
        uid = str(uuid4())
        c.api.request({"version": 1, "run_id": RUN, "op": "queue", "binding": dict(
            placement_id=str(uuid4()), sample_uid=uid, order_id=uid, carousel_id="tray-A",
            slot=3, release_after_sample_uid=None)})
    elif obstacle == "busy":
        c.server._busy = True
    elif obstacle == "cleanup_problem":
        c.server.workflows._cleanup_operations["wash_flowcell"] = {
            "status": "failed", "error": "pump stalled"}
    elif obstacle == "mixer_head":
        state.set_mixer_head(state.MIXER_AT_MIXER)
    elif obstacle == "flowcell":
        state.set_flowcell_location(1, state.FC_AT_BEAM)
    else:
        c.pal.missing = ["carousel_in_stock"]
    with pytest.raises(Exception, match=match):
        c.api.request(message)
    assert c.started == [] and c.pal.log == []
    assert c.api.record.get("replacements", {}) == {}
    assert json.loads(c.path.read_text()).get("replacements", {}) == {}
    assert sorted(state.carousel_samples()) == [1, 2, 5]


def test_replace_turns_the_table_before_the_unload(coordinated, monkeypatch):
    c = coordinated
    monkeypatch.setattr(state, "get_carousel_place_alignment_slot", lambda: 4)
    state.set_flowcell_location(2, state.FC_AT_CLEANING)
    c.server.motor = FakeMotor(c.pal.log)
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    turn = ("turn", pytest.approx(SLOT_4))
    assert c.pal.log == [turn, "unload", turn, "place"]   # the second turn is already there
    assert c.pal.parks == 1
    assert status(c)["replacements"][rid]["state"] == "committed"
    assert state.get_carousel_slot() == 4


def test_replace_stops_before_the_unload_when_the_table_does_not_turn(coordinated, monkeypatch):
    c = coordinated
    monkeypatch.setattr(state, "get_carousel_place_alignment_slot", lambda: 4)
    state.set_flowcell_location(2, state.FC_AT_CLEANING)
    c.server.motor = FakeMotor(c.pal.log)
    c.server.motor.error = RuntimeError("Galil did not answer")
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    record = status(c)["replacements"][rid]
    assert (record["state"], record["failed_phase"]) == ("failed", "unloading")
    assert "did not turn" in record["error"] and "Galil did not answer" in record["error"]
    assert "nothing else moved" in record["error"]
    assert c.pal.log == []                                   # the old carousel was not touched
    assert sorted(state.carousel_samples()) == [1, 2, 5]
    assert c.pal.parks == 0


@pytest.mark.parametrize("ending", ["committed", "waiting_for_stock", "waiting_for_carousel"])
def test_a_swap_that_ends_without_a_failure_parks_the_arm(coordinated, monkeypatch, ending):
    c = coordinated
    if ending == "waiting_for_stock":
        c.pal.place_error = FakePAL.NoCarouselError("no carousel at carousel_in_stock")
    elif ending == "waiting_for_carousel":
        monkeypatch.setattr(preparation_api, "auto_place_new_carousel", lambda: False)
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    assert status(c)["replacements"][rid]["state"] == ending
    assert c.pal.parks == 1
    assert c.server.workflows.cleanup_snapshot()["parked"] is True


def test_a_failed_swap_does_not_claim_the_arm_parked(coordinated):
    c = coordinated
    c.pal.place_error = RuntimeError("protective stop")
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    assert status(c)["replacements"][rid]["state"] == "failed"
    assert c.pal.parks == 0
    assert c.server.workflows.cleanup_snapshot()["parked"] is False


def test_replace_is_refused_before_the_unload_when_the_table_cannot_turn(coordinated, monkeypatch):
    c = coordinated
    monkeypatch.setattr(state, "get_carousel_place_alignment_slot", lambda: 4)
    state.set_flowcell_location(2, state.FC_AT_CLEANING)
    monkeypatch.setattr(state, "carousel_reference", lambda: None)
    c.server.motor = FakeMotor(c.pal.log)
    with pytest.raises(ValueError, match="has not been taught"):
        c.api.request(replace_message())
    assert c.started == [] and c.pal.log == []
    assert c.api.record.get("replacements", {}) == {}


def test_replace_is_refused_while_a_flowcell_was_never_tracked(coordinated, monkeypatch):
    # A station that uses only flowcell 1 never tracks flowcell 2, which reads unknown:
    # every swap is refused until the operator declares where it is (the README says so).
    c = coordinated
    monkeypatch.setattr(state, "get_carousel_place_alignment_slot", lambda: 4)
    c.server.motor = FakeMotor(c.pal.log)
    assert state.get_flowcell_location(2) == state.UNKNOWN
    with pytest.raises(ValueError, match=r"flowcell 2 must be at its cleaning station or the "
                                         r"beam .*\(flowcell 2: unknown\)"):
        c.api.request(replace_message())
    assert c.started == [] and c.pal.log == []
    assert c.api.record.get("replacements", {}) == {}
    # PALmixer refuses set_location during a session, so it is declared before one opens.
    assert c.server.fast_dispatch("set_location flowcell_2 cleaning_station").startswith("ERROR")
    state.set_location(state.WHAT_FLOWCELL_2, state.FC_AT_CLEANING)   # what that command does
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    assert status(c)["replacements"][rid]["state"] == "committed"


# -- a swap ends with the standard park, which sets cleanup.parked again ---------------
def test_replace_parks_the_arm_before_reporting_committed(coordinated):
    c = coordinated
    # The last sample's unload parked; the swap's worker action clears that.
    c.server.workflows._step("park_at_transfer_point", lambda: "parked")
    message = replace_message()
    rid = message["replacement_id"]
    during = {}
    c.pal.during_park = lambda: during.update(
        record=persisted(c, rid), parked=c.server.workflows.cleanup_snapshot()["parked"])
    c.api.request(message)
    c.finish()
    assert c.pal.log == ["unload", "place"]
    assert c.pal.parks == 1
    # The park comes after the load is recorded, and before committed is reported.
    assert during["record"]["state"] == "parking" and during["record"]["load"]["sequence"] == 1
    assert during["parked"] is False
    final = status(c)["replacements"][rid]
    assert final["state"] == "committed" and final["load"] == during["record"]["load"]
    assert c.server.workflows.cleanup_snapshot()["parked"] is True
    assert c.server._last_result["ok"] is True
    assert c.server._last_result["detail"].endswith(
        "; every slot cleared (carousel load 1); arm parked")


def test_a_carousel_taken_back_during_the_swap_is_named_in_its_result(coordinated):
    c = coordinated
    note = ("carousel 35 did not land (its top is 6.0 mm above the seat) and was moved to the "
            "used stack, unused; carousel 37 placed instead")
    c.pal.place_notes = [note]
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    assert status(c)["replacements"][rid]["state"] == "committed"
    assert note in c.server._last_result["detail"]
    assert c.pal.log == ["unload", "place"]
    assert c.pal.parks == 1
    assert c.server.workflows.cleanup_snapshot()["parked"] is True


EMPTY_STOCK = "no carousel at carousel_in_stock: looked for tag(s) [32], tags in view []"
EMPTY_AFTER_A_TAKE_BACK = EMPTY_STOCK + (". Carousel 35 did not land and was moved to the used "
                                         "stack, unused; load a fresh carousel and press Place "
                                         "New Carousel.")


@pytest.mark.parametrize("text", [EMPTY_STOCK, EMPTY_AFTER_A_TAKE_BACK],
                         ids=["empty", "empty_after_a_take_back"])
def test_empty_stock_seat_parks_and_keeps_the_placement_error(coordinated, text):
    c = coordinated
    c.pal.place_error = FakePAL.NoCarouselError(text)
    message = replace_message()
    rid = message["replacement_id"]
    during = []
    c.pal.during_park = lambda: during.append(persisted(c, rid))
    c.api.request(message)
    c.finish()
    assert c.pal.log == ["unload", "place"]
    assert c.pal.parks == 1
    assert [(r["state"], r["detail"]) for r in during] == [("parking", text)]
    record = status(c)["replacements"][rid]
    assert (record["state"], record["error"], record["detail"]) == (
        "waiting_for_stock", NO_STOCK_MESSAGE, text)
    assert "load" not in record
    assert c.server.workflows.cleanup_snapshot()["parked"] is True
    result = c.server._last_result
    assert result["ok"] is True
    assert result["detail"].startswith("carousel unloaded to the used stack (tag 33")
    # Then what to do: the session error stays empty, so PALmixer's result line is
    # where an operator at PALmixer reads it.
    assert result["detail"].endswith("; %s; arm parked; %s" % (text, NO_STOCK_MESSAGE))
    # Nothing was cleared, and the session stays usable: PALsystem can end it.
    assert sorted(state.carousel_samples()) == [1, 2, 5] and state.carousel_load()["sequence"] == 0
    assert c.api.record["error"] == "" and c.api.coordinator.snapshot()["error"] == ""
    assert c.api.request({"version": 1, "run_id": RUN, "op": "end"})["active"] is False


def test_a_failed_seat_check_fails_in_placing_without_parking(coordinated):
    c = coordinated
    # The last sample's unload parked; a failed swap must not leave that standing.
    c.server.workflows._step("park_at_transfer_point", lambda: "parked")
    error = FakePAL.CarouselNotSeatedError(
        "the carousel did not land: nothing felt down to 20 mm below the seat")
    c.pal.place_error = error
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    record = status(c)["replacements"][rid]
    assert (record["state"], record["failed_phase"], record["error"]) == (
        "failed", "placing", str(error))
    # The arm stays where the check left it, for the operator to inspect the mixer.
    assert c.pal.log == ["unload", "place"]
    assert c.server.workflows.cleanup_snapshot()["parked"] is False
    assert c.api.record["error"] == str(error)
    assert c.api.coordinator.snapshot()["error"] == str(error)
    assert c.server._last_result["ok"] is False
    assert sorted(state.carousel_samples()) == [1, 2, 5] and state.carousel_load()["sequence"] == 0
    with pytest.raises(ValueError, match="did not land"):
        c.api.request(replace_message())               # no further motion until reconciled


@pytest.mark.parametrize("stock", ["fresh", "empty"])
def test_a_failed_park_fails_the_swap_in_parking(coordinated, stock):
    c = coordinated
    # The last sample's unload parked; a failed swap must not leave that standing.
    c.server.workflows._step("park_at_transfer_point", lambda: "parked")
    if stock == "empty":
        c.pal.place_error = FakePAL.NoCarouselError(EMPTY_STOCK)
    c.pal.park_error = RuntimeError("protective stop on the way to the transfer point")
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    # The error says what the swap had done before the park, then why the park failed.
    done = (EMPTY_STOCK if stock == "empty" else
            "new carousel placed on the mixer (tag 37, arm at [0.4, 0.5, 0.6]); "
            "every slot cleared (carousel load 1)")
    error = "%s; parking the arm failed: protective stop on the way to the transfer point" % done
    record = status(c)["replacements"][rid]
    assert (record["state"], record["failed_phase"], record["error"]) == (
        "failed", "parking", error)
    assert c.pal.log == ["unload", "place"]
    assert c.pal.parks == 1
    assert c.server.workflows.cleanup_snapshot()["parked"] is False
    assert c.api.record["error"] == error
    assert c.api.coordinator.snapshot()["error"] == error
    assert c.server._last_result["ok"] is False and c.server._last_result["detail"] == error
    with pytest.raises(ValueError, match="protective stop"):
        c.api.request(replace_message())               # no further motion until reconciled
    if stock == "fresh":
        # Seated and recorded before the park: the load stands and every slot is free.
        assert record["load"]["sequence"] == 1
        assert state.carousel_load()["record"] == record["load"]
        assert state.carousel_samples() == {}
    else:
        assert record["detail"] == EMPTY_STOCK and "load" not in record
        assert sorted(state.carousel_samples()) == [1, 2, 5]
        assert state.carousel_load()["sequence"] == 0


@pytest.mark.parametrize("stock", ["seated", "empty"])
def test_a_failed_park_after_a_take_back_still_names_the_rejected_carousel(coordinated, stock):
    c = coordinated
    if stock == "seated":
        placement = TAKEN_BACK
        c.pal.place_notes = [TAKEN_BACK]
    else:
        placement = EMPTY_AFTER_A_TAKE_BACK
        c.pal.place_error = FakePAL.NoCarouselError(EMPTY_AFTER_A_TAKE_BACK)
    c.pal.park_error = RuntimeError("protective stop")
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    record = status(c)["replacements"][rid]
    assert (record["state"], record["failed_phase"]) == ("failed", "parking")
    # An unused carousel is on the used stack now. The record, the latched session error
    # that PALsystem reports and the action's result line all still say so.
    for text in (record["error"], c.api.record["error"], c.server._last_result["detail"]):
        assert placement in text and text.endswith("; parking the arm failed: protective stop")


def test_placement_off_parks_before_reporting_waiting_for_carousel(coordinated, monkeypatch):
    c = coordinated
    monkeypatch.setattr(preparation_api, "auto_place_new_carousel", lambda: False)
    rid = replace_message()["replacement_id"]
    during = []
    c.pal.during_park = lambda: during.append(persisted(c, rid)["state"])
    c.api.request(replace_message(rid))
    c.finish()
    assert during == ["parking"]
    assert status(c)["replacements"][rid]["state"] == "waiting_for_carousel"
    assert c.pal.log == ["unload"] and c.pal.parks == 1
    assert c.server._last_result["detail"].endswith(
        "; arm parked; " + preparation_api.WAITING_FOR_CAROUSEL_MESSAGE)


def test_a_failed_park_with_placement_off_fails_in_parking(coordinated, monkeypatch):
    c = coordinated
    monkeypatch.setattr(preparation_api, "auto_place_new_carousel", lambda: False)
    c.pal.park_error = RuntimeError("protective stop")
    rid = replace_message()["replacement_id"]
    c.api.request(replace_message(rid))
    c.finish()
    record = status(c)["replacements"][rid]
    assert (record["state"], record["failed_phase"]) == ("failed", "parking")
    # The error says the old carousel is already off, then why the park failed.
    assert record["error"].startswith("carousel unloaded to the used stack (tag 33")
    assert record["error"].endswith("; parking the arm failed: protective stop")
    assert c.api.record["error"] == record["error"]
    assert c.server.workflows.cleanup_snapshot()["parked"] is False


def test_state_advertises_that_a_swap_ends_parked(coordinated):
    c = coordinated
    # The few attributes get_state reads that the bare test server lacks.
    c.server.pump = SimpleNamespace(speed=800.0, min_rpm=0.0, max_rpm=3000.0)
    c.server._pump_status = c.server._last_mixing_record = None
    snapshot = json.loads(c.server.fast_dispatch(cmd.GET_STATE))
    assert snapshot["preparation"]["replace_parks_version"] == 1


# -- placement blocked: the operator seats the carousel and presses Reset --------------
def test_blocked_placement_stops_after_unload_and_reset_finishes_it(coordinated, journal, monkeypatch):
    c = coordinated
    monkeypatch.setattr(preparation_api, "auto_place_new_carousel", lambda: False)
    save_mix(journal, "S-one", UID_ONE)
    # The last sample's unload parked; the swap's worker action clears that.
    c.server.workflows._step("park_at_transfer_point", lambda: "parked")
    message = replace_message()
    rid = message["replacement_id"]
    c.api.request(message)
    c.finish()
    record = status(c)["replacements"][rid]
    assert record["state"] == "waiting_for_carousel"
    assert record["error"] == preparation_api.WAITING_FOR_CAROUSEL_MESSAGE
    assert c.pal.log == ["unload"]                      # no robot placement
    # It still parks, clear of the mixer, before waiting for the person seating one.
    assert c.pal.parks == 1 and c.server.workflows.cleanup_snapshot()["parked"] is True
    assert sorted(state.carousel_samples()) == [1, 2, 5]  # nothing cleared yet
    assert c.api.record["error"] == "" and c.server._last_result["ok"] is True
    # The session is still open, yet Reset is accepted while the swap waits for it.
    assert c.api.active and c.server.fast_dispatch("reset_carousel") == "OK"
    record = status(c)["replacements"][rid]
    assert record["state"] == "committed" and record["placed_by"] == "operator"
    # Reset moves nothing: the arm is still parked where the swap left it.
    assert c.server.workflows.cleanup_snapshot()["parked"] is True
    load = record["load"]
    assert (load["old_tag"], load["new_tag"], load["replacement_id"]) == ("33", "unknown", rid)
    assert [(s["slot"], s["mixed"]) for s in load["slots"]] == [(1, True), (2, False), (5, False)]
    assert state.carousel_samples() == {} and state.get_carousel_on_mixer() == state.UNKNOWN
    assert state.carousel_load()["record"] == load
    # Anything else stays blocked by the session.
    assert c.server.fast_dispatch("clear_slots").startswith("ERROR: coordinated")


def test_reset_is_refused_in_a_session_unless_a_replacement_waits(coordinated):
    reply = coordinated.server.fast_dispatch("reset_carousel")
    assert reply.startswith("ERROR: coordinated preparation owns the equipment")
    assert sorted(state.carousel_samples()) == [1, 2, 5]


def test_reset_outside_a_session_records_a_hand_placed_load(server):
    state.set_carousel_on_mixer(32)
    assert server.fast_dispatch("reset_carousel") == "OK"
    load = state.carousel_load()["record"]
    assert (load["sequence"], load["old_tag"], load["new_tag"], load["replacement_id"]) == (1, "32", "unknown", None)
    assert state.carousel_samples() == {} and state.get_carousel_on_mixer() == state.UNKNOWN


def test_reset_is_refused_while_busy(server):
    server._busy, server._current_action = True, "make_sample"
    assert server.fast_dispatch("reset_carousel").startswith("ERROR: busy")
    assert sorted(state.carousel_samples()) == [1, 2, 5]
