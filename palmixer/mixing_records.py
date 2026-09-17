"""Durable mixing events and retryable PVapp delivery; no hardware operations."""
from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
from tempfile import NamedTemporaryFile
from threading import Lock
from uuid import uuid4


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def new_record(sample_id, slot, carousel_id, flowcell, simulated=False, sample_uid=""):
    return {"schema_version": 1, "run_id": uuid4().hex, "sample_id": sample_id,
            "sample_uid": sample_uid,
            "slot": slot, "carousel_id": carousel_id, "flowcell": flowcell,
            "simulated": bool(simulated), "started_at": utc_now(),
            "finished_at": None, "workflow_status": "running",
            "mixing_status": "not_started", "steps": [], "pump": None}


class MixingJournal:
    def __init__(self, directory=None):
        self.directory = Path(directory or os.environ.get("PALMIXER_MIXING_RECORDS_DIR")
                              or Path(__file__).resolve().parents[1] / "json" / "mixing_records")
        self._sync_lock = Lock()

    def _path(self, run_id):
        if not isinstance(run_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", run_id):
            raise ValueError("Invalid mixing run ID")
        return self.directory / (run_id + ".json")

    def _write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                    suffix=".tmp", delete=False) as handle:
                temporary = handle.name
                json.dump(value, handle, indent=2, allow_nan=False)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    def save(self, record):
        envelope = {"record": deepcopy(record), "pvapp_sync": {
            "status": "local_only" if record.get("simulated") or not record.get("sample_id") else "pending"}}
        self._write(self._path(record["run_id"]), envelope)

    @staticmethod
    def _read(path):
        envelope = json.loads(path.read_text(encoding="utf-8"))
        if (not isinstance(envelope, dict) or not isinstance(envelope.get("record"), dict)
                or not isinstance(envelope.get("pvapp_sync"), dict)):
            raise ValueError("Invalid mixing journal envelope")
        return envelope

    def recover_interrupted(self):
        """Called once at server startup; never invent a successful outcome."""
        for path in self.directory.glob("*.json"):
            try:
                envelope = self._read(path)
                record = envelope["record"]
                if record.get("workflow_status") == "running":
                    record.update(workflow_status="interrupted", recovered_at=utc_now())
                    if record.get("mixing_status") == "running":
                        record["mixing_status"] = "unknown"
                    self.save(record)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                print("WARNING: could not recover mixing record %s: %s" % (path.name, exc))

    def sync_pending(self, updater):
        """Retry by run ID. Replaying a successful PUT cannot append duplicates."""
        if not self._sync_lock.acquire(blocking=False):
            return
        try:
            for path in self.directory.glob("*.json"):
                try:
                    envelope = self._read(path)
                    record = envelope["record"]
                    if (envelope["pvapp_sync"]["status"] != "pending"
                            or record["workflow_status"] == "running"):
                        continue
                    try:
                        response = updater.confirm_mix(record["sample_id"], record["slot"], record["flowcell"],
                                                       mixing_record=record)
                        if isinstance(response, dict) and not record.get("sample_uid"):
                            uid = response.get("sample_uid") or (response.get("data") or {}).get("sample_uid")
                            if uid:
                                record["sample_uid"] = uid
                    except Exception as exc:
                        if envelope["pvapp_sync"].get("error") != str(exc):
                            print("WARNING: mixing run %s saved locally; PVapp sync pending: %s"
                                  % (record["run_id"], exc))
                        envelope["pvapp_sync"].update(last_attempt_at=utc_now(), error=str(exc))
                        self._write(path, envelope)
                        # A missing sample must not prevent other samples syncing.
                        continue
                    envelope["pvapp_sync"] = {"status": "synced", "synced_at": utc_now()}
                    self._write(path, envelope)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    print("WARNING: mixing record %s: %s" % (path.name, exc))
        finally:
            self._sync_lock.release()
