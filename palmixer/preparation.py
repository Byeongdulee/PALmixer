"""Opt-in preparation scheduler; no sockets, UR commands or automatic startup.

The later PALsystem/ChemLab adapter supplies validated placements and callbacks.
One worker owns positioning. The existing UR thread only opens an opportunity.
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import threading
from uuid import UUID


class PreparationError(RuntimeError):
    pass


def placement(value):
    """Internal identity contract, deliberately independent of the future wire JSON."""
    keys = {"placement_id", "order_id", "sample_uid", "carousel_id", "slot",
            "release_after_sample_uid"}
    if not isinstance(value, dict) or set(value) != keys:
        raise PreparationError("A complete preparation placement is required")
    result = dict(value)
    for key in ("placement_id", "carousel_id"):
        if not isinstance(result[key], str) or not result[key].strip():
            raise PreparationError("%s must be a nonempty identity" % key)
    try:
        uid = UUID(result["sample_uid"])
        if not uid.int or str(uid) != result["sample_uid"]:
            raise ValueError()
        if result["order_id"] != str(uid):
            raise ValueError()
        previous = result["release_after_sample_uid"]
        if previous is not None:
            previous_uid = UUID(previous)
            if not previous_uid.int or str(previous_uid) != previous or previous == str(uid):
                raise ValueError()
    except (ValueError, TypeError, AttributeError):
        raise PreparationError("Use canonical sample UUIDs; order_id must equal sample_uid") from None
    if type(result["slot"]) is not int or result["slot"] < 1:
        raise PreparationError("slot must be a positive integer")
    return result


class PreparationCoordinator:
    """A durable, single pending placement plus a one-use workflow opportunity.

    ``move(record)`` must wait for the existing motor completion. ``notify(record)``
    returns after acknowledging readiness, or raises; retry_ready retries only
    that notification. Neither callback may command the UR robot.
    """
    TERMINAL = {"released", "cancelled"}

    def __init__(self, path, move, notify):
        self.path = Path(path)
        self.move, self.notify = move, notify
        self._lock = threading.RLock()
        self._worker = None
        self._closing = False
        self._cycle = None
        self._window = None
        self._initial = None
        self._inspection = None
        self._records = {}
        self._error = ""
        if self.path.exists():
            saved = json.loads(self.path.read_text(encoding="utf-8"))
            if saved.get("version") != 1:
                raise PreparationError("Unsupported preparation state version")
            self._records = saved["placements"]
            self._error = saved.get("error", "")
            self._inspection = saved.get("inspection")
            if self._inspection:
                self._error = "Restart interrupted an empty-cell inspection; reconciliation required"
            if saved.get("cycle"):
                self._error = "Restart interrupted a sample workflow; reconciliation required"
            for record in self._records.values():
                if record["phase"] not in self.TERMINAL:
                    record.update(phase="uncertain", error="Restart invalidated physical readiness")
                    self._error = "Preparation requires reconciliation after restart"
            self._save()

    def _save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump({"version": 1, "placements": self._records, "error": self._error,
                       "cycle": self._cycle, "inspection": self._inspection},
                      handle, allow_nan=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self.path)

    def _active(self):
        return next((r for r in self._records.values() if r["phase"] not in self.TERMINAL), None)

    def snapshot(self):
        with self._lock:
            return deepcopy({"placements": self._records, "error": self._error,
                             "cycle": self._cycle, "window": self._window, "inspection": self._inspection})

    def get(self, placement_id):
        with self._lock:
            if placement_id not in self._records:
                raise PreparationError("Unknown placement")
            return deepcopy(self._records[placement_id])

    def queue(self, value):
        binding = placement(value)
        with self._lock:
            old = self._records.get(binding["placement_id"])
            if old is not None:
                if old["binding"] != binding:
                    raise PreparationError("Placement identity already has different contents")
                return deepcopy(old)
            if self._closing or self._error:
                raise PreparationError(self._error or "Preparation scheduler is closing")
            if self._active() is not None:
                raise PreparationError("A preparation already owns the carousel")
            if any(r["binding"]["sample_uid"] == binding["sample_uid"] for r in self._records.values()):
                raise PreparationError("This sample already has a placement; reuse its identity")
            if binding["release_after_sample_uid"] is None and any(
                    r["phase"] != "cancelled" for r in self._records.values()):
                raise PreparationError("Only the first placement may omit its predecessor")
            record = {"binding": binding, "phase": "pending", "error": "", "action_id": None}
            self._records[binding["placement_id"]] = record
            self._save()
            self._kick()
            return deepcopy(record)

    def open_initial(self, placement_id):
        """Called only after the initial existing clearance sequence returns."""
        with self._lock:
            record = self._records[placement_id]
            if self._cycle or self._window or self._error or self._closing or self._inspection:
                raise PreparationError("Initial preparation is not eligible")
            if record["phase"] != "pending" or record["binding"]["release_after_sample_uid"] is not None:
                raise PreparationError("Not an initial pending placement")
            self._initial = placement_id
            self._kick()

    def begin_sample(self, sample_uid, slot, action_id, *, carousel_id):
        with self._lock:
            if self._closing or self._error or self._cycle or self._inspection:
                raise PreparationError(self._error or "A sample workflow is already active")
            if not isinstance(action_id, str) or not action_id.strip():
                raise PreparationError("A workflow action identity is required")
            prepared = [r for r in self._records.values() if r["phase"] == "released"
                        and r["binding"]["sample_uid"] == sample_uid
                        and r["binding"]["carousel_id"] == carousel_id
                        and r["binding"]["slot"] == slot]
            if not prepared or prepared[0].get("workflow_started"):
                raise PreparationError("The sample needs one released preparation, not a replay")
            pending = self._active()
            if pending and (pending["phase"] != "pending" or
                            pending["binding"]["release_after_sample_uid"] != sample_uid):
                raise PreparationError("Another preparation owns the carousel")
            prepared[0]["workflow_started"] = action_id
            self._cycle = {"sample_uid": sample_uid, "action_id": action_id}
            self._window = None
            self._save()

    def lift_complete(self, sample_uid, action_id):
        """Nonblocking, nonthrowing hook after the unchanged mvr2z call."""
        try:
            with self._lock:
                expected = {"sample_uid": sample_uid, "action_id": action_id}
                if self._cycle != expected or self._error or self._closing:
                    raise PreparationError("Lift notification does not match the active workflow")
                if self._window is None:
                    self._window = dict(expected, consumed=False)
                self._kick()
        except Exception as error:
            # A scheduling failure must not interrupt the existing beam transfer.
            with self._lock:
                self._error = str(error)
                self._window = None
                # No disk/network work on the UR thread. finish_sample persists
                # this failure; the already-durable active cycle blocks replay
                # even if the process exits before that completion is recorded.

    def finish_sample(self, sample_uid, action_id, error=None):
        with self._lock:
            if self._cycle != {"sample_uid": sample_uid, "action_id": action_id}:
                raise PreparationError("Wrong sample workflow completion")
            self._cycle = None
            record = next(r for r in self._records.values()
                          if r["binding"]["sample_uid"] == sample_uid)
            record["workflow_outcome"] = {"action_id": action_id,
                "status": "failed" if error is not None else "completed",
                "error": str(error) if error is not None else ""}
            if error is not None:
                self._error = str(error) or "Sample workflow failed"
                self._window = None
            self._save()

    def _kick(self):
        record = self._active()
        if self._closing or self._error or self._inspection or not record or record["phase"] != "pending":
            return
        previous = record["binding"]["release_after_sample_uid"]
        if previous is None:
            if self._initial != record["binding"]["placement_id"]:
                return
            self._initial = None
        else:
            if not self._window or self._window["sample_uid"] != previous or self._window["consumed"]:
                return
            self._window["consumed"] = True
            record["action_id"] = self._window["action_id"]
        record["phase"] = "positioning"
        self._worker = threading.Thread(target=self._position, args=(record,),
                                        name="PAL preparation positioning", daemon=True)
        self._worker.start()

    def _position(self, record):
        try:
            with self._lock:
                if self._error or self._closing:
                    raise PreparationError(self._error or "Scheduler closed before positioning")
                self._save()  # Off the UR thread, and before any motor command.
            self.move(deepcopy(record["binding"]))
            with self._lock:
                if self._error or self._closing:
                    raise PreparationError(self._error or "Scheduler closed during positioning")
                record["phase"] = "ready_unacknowledged"
                self._save()
            self._notify(record)
        except Exception as error:
            with self._lock:
                record.update(phase="uncertain", error=str(error))
                self._error = str(error)
                self._save()

    def _notify(self, record):
        with self._lock:
            if self._error or self._closing:
                record.update(phase="uncertain", error=self._error or "Scheduler closed")
                self._save()
                return
        try:
            self.notify(deepcopy(record["binding"]))
        except Exception as error:
            with self._lock:
                record["error"] = str(error)
                self._save()
            return
        with self._lock:
            if self._error or self._closing:
                record.update(phase="uncertain", error=self._error or "Scheduler closed")
            else:
                record.update(phase="slot_ready", error="")
            self._save()

    def retry_ready(self, placement_id):
        with self._lock:
            record = self._records[placement_id]
            if self._error or self._closing or record["phase"] != "ready_unacknowledged":
                raise PreparationError("Readiness cannot be retried in this state")
            if self._worker and self._worker.is_alive():
                raise PreparationError("A notification is already running")
            self._worker = threading.Thread(target=self._notify, args=(record,), daemon=True)
            self._worker.start()

    def release(self, placement_id, sample_uid, *, completion_id, prepared, pal_tool_clear):
        with self._lock:
            record = self._records[placement_id]
            if sample_uid != record["binding"]["sample_uid"]:
                raise PreparationError("Wrong sample identity")
            if prepared is not True or pal_tool_clear is not True or not completion_id:
                raise PreparationError("Matching preparation completion and PAL tool clearance are required")
            if record["phase"] == "released" and record.get("completion_id") == completion_id:
                return deepcopy(record)
            if record["phase"] != "slot_ready" or self._error or self._closing:
                raise PreparationError("Preparation is not ready for release")
            record.update(phase="released", completion_id=completion_id)
            self._save()
            return deepcopy(record)

    def cancel(self, placement_id):
        with self._lock:
            record = self._records[placement_id]
            if record["phase"] != "pending":
                raise PreparationError("Only a placement that has not started can be cancelled")
            record["phase"] = "cancelled"
            self._window = None
            self._save()

    def require_idle(self):
        """Guard a conflicting manual carousel action or return-to-vial path."""
        with self._lock:
            if self._active() or self._cycle or self._error or self._inspection:
                raise PreparationError("Preparation coordination owns the carousel")
            self._window = None

    def begin_inspection(self, inspection_id):
        """Hold positioning and mixing while the empty cell is away from its station."""
        with self._lock:
            if self._inspection == inspection_id:
                return
            active = self._active()
            if (self._closing or self._error or self._cycle or self._inspection
                    or (active and active['phase'] != 'pending')):
                raise PreparationError("Wait for PAL clearance before the empty-cell inspection")
            self._inspection = inspection_id
            self._save()

    def end_inspection(self, inspection_id):
        """Called only after the matching cell return and arm parking complete."""
        with self._lock:
            if self._inspection != inspection_id or self._error or self._closing:
                raise PreparationError("The empty-cell inspection cannot be released")
            self._inspection = None
            self._save()
            self._kick()

    def invalidate(self, error):
        """Latch an uncertain clearance/motion failure without automatic replay."""
        with self._lock:
            self._error = str(error) or "Preparation coordination failed"
            self._window = self._initial = None
            record = self._active()
            if record:
                record.update(phase="uncertain", error=self._error)
            self._save()

    def wait(self, timeout=5):
        with self._lock:
            worker = self._worker
        if worker:
            worker.join(timeout)
            if worker.is_alive():
                raise TimeoutError("Preparation worker is still active")

    def close(self):
        with self._lock:
            self._closing = True
            self._window = None
        self.wait()
