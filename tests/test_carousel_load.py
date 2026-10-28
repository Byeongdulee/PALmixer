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

from palmixer import preparation_api, state
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
    """Stands in for PAL12idb: unload/place only update tracked state."""

    class NoCarouselError(RuntimeError):
        pass

    def __init__(self):
        self.log = []
        self.missing = []
        self.unload_error = self.place_error = None
        self.during_unload = self.during_place = lambda: None

    def check_positions_defined(self, keys):
        return [k for k in keys if k in self.missing]

    def unload_current_carousel(self, robot):
        self.log.append("unload")
        self.during_unload()
        if self.unload_error:
            raise self.unload_error
        state.set_last_unloaded_carousel(33)
        state.set_carousel_on_mixer(None)
        return [0.1, 0.2, 0.3]

    def place_new_carousel(self, robot):
        self.log.append("place")
        state.set_carousel_on_mixer(None)
        self.during_place()
        if self.place_error:
            raise self.place_error
        state.set_carousel_on_mixer(37)
        return [0.4, 0.5, 0.6]


@pytest.fixture
def server(tracked, journal):
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
    instance.workflows = Workflows(None, None, None, None)
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
    # Robot placement is blocked by default for now; these tests exercise the full swap.
    monkeypatch.setattr(preparation_api, "auto_place_new_carousel", lambda: True)
    api = PreparationAPI(server, tracked / "session.json")
    server.preparation_api = api
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
    assert len(c.started) == 1 and c.pal.log == ["unload", "place"]
    with pytest.raises(ValueError, match="different contents"):
        c.api.request(dict(message, note="changed"))
    assert state.carousel_load()["sequence"] == 1
    # A new identity is a new swap request, checked like any other.
    c.api.request(replace_message())
    c.finish()
    assert c.pal.log == ["unload", "place", "unload", "place"]
    assert state.carousel_load()["sequence"] == 2
    assert set(status(c)["replacements"]) >= {rid}


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


# -- placement blocked: the operator seats the carousel and presses Reset --------------
def test_blocked_placement_stops_after_unload_and_reset_finishes_it(coordinated, journal, monkeypatch):
    c = coordinated
    monkeypatch.setattr(preparation_api, "auto_place_new_carousel", lambda: False)
    save_mix(journal, "S-one", UID_ONE)
    message = replace_message()
    rid = message["replacement_id"]
    c.api.request(message)
    c.finish()
    record = status(c)["replacements"][rid]
    assert record["state"] == "waiting_for_carousel"
    assert record["error"] == preparation_api.WAITING_FOR_CAROUSEL_MESSAGE
    assert c.pal.log == ["unload"]                      # no robot placement
    assert sorted(state.carousel_samples()) == [1, 2, 5]  # nothing cleared yet
    assert c.api.record["error"] == "" and c.server._last_result["ok"] is True
    # The session is still open, yet Reset is accepted while the swap waits for it.
    assert c.api.active and c.server.fast_dispatch("reset_carousel") == "OK"
    record = status(c)["replacements"][rid]
    assert record["state"] == "committed" and record["placed_by"] == "operator"
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
