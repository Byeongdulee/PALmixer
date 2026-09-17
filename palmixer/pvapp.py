# -*- coding: utf-8 -*-
"""Appending mixing provenance to the existing PVapp sample.

PALsystem posts the "prepared" record the moment solutions go in a vial's two input
wells -- before anything is mixed. ``make_sample`` produces a durable local event
(see :mod:`palmixer.mixing_records`). A background worker merges it into that same
sample's ``data.mixing_records``, preserving preparation and other metadata. Runs
include failed/interrupted attempts; only confirmed pump completion updates the
legacy ``mixed_*`` summary. A run ID makes retries idempotent.

    GET  /api/samples/<id>      -> the record PALsystem already created
    PUT  /api/samples/<id>      -> replace it (``PVapp/app/api/routes.py``)

**This never creates a record, only updates one.** ``PUT`` replaces ``data``
wholesale rather than merging into it -- confirmed against
``aps12sdl/experiments/lnp_mesophase/pvapp.py:update_sample``, which carries the same
warning -- so :meth:`PVappUpdater.confirm_mix` reads the existing record first and
merges the mix fields into its ``data`` before writing it back. A ``GET`` that 404s
means PALsystem never recorded this sample at all; that is reported and nothing is
written, because a bare record created here would overwrite nothing useful with less.

The read-modify-write is not atomic. Concurrent writers require a PVapp event-append
API or revision check to prevent lost updates. Reading fresh data preserves fields
already present, but cannot protect a change made between GET and PUT.

Both need HTTP Basic (same as PALsystem): a **badge number** with that user's
password, or a **staff name** with the shared staff password plus ``owner_badge`` so
PVapp knows whose sample it is.

**The password comes from the environment only** (``PVAPP_PASSWORD``), never from the
config file -- config files live in git and get copied between beamlines. A campaign
may also hand it over live over ZMQ (``set_credentials``); see :mod:`palmixer.commands`.
"""

import time
from copy import deepcopy
from uuid import UUID

from . import config


class PVappError(RuntimeError):
    """Raised when PVapp cannot be reached, or refuses the update."""


class PVappUpdater:
    """Merges mixing events into the record PALsystem already created."""

    def __init__(self, base_url="", username="", password="", owner_badge="",
                 timeout_s=10.0, badge="", gup=""):
        section = config.get_section("pvapp")
        self.base_url = (base_url or section.get("base_url") or "").rstrip("/")
        self.username = username or section.get("username") or ""
        self.password = password or config.pvapp_password()
        self.owner_badge = str(owner_badge or section.get("owner_badge") or "").strip()
        self.timeout_s = float(timeout_s or section.get("timeout_s") or 10.0)
        #: The campaign's pair, set by ``set_credentials``. Wins: it names the run.
        self.badge = str(badge or "").strip()
        self.gup = str(gup or "").strip()
        #: Typed into the GUI's User tab -- the fallback for when no campaign is driving.
        self.user_badge = ""
        self.user_gup = ""
        #: Set when a campaign hands over a username and password: it chose the password
        #: route deliberately, and a configured pair must not silently outrank it.
        self.password_only = False
        # owner_badge as the fallback for the badge -- same number, and a config that
        # predates the pair then needs only its proposal adding. Matches PALsystem.
        self._config_badge = str(section.get("badge")
                                 or section.get("owner_badge") or "").strip()
        self._config_gup = str(section.get("gup") or "").strip()

    def effective_pair(self):
        """``(badge, gup, where_it_came_from)`` -- campaign, then User tab, then config."""
        if self.password_only:
            return "", "", ""
        for badge, gup, source in ((self.badge, self.gup, "campaign"),
                                   (self.user_badge, self.user_gup, "User tab"),
                                   (self._config_badge, self._config_gup, "config file")):
            if badge and gup:
                return badge, gup, source
        return "", "", ""

    @property
    def params(self):
        """``{"badge": ..., "gup": ...}`` for the no-password route, or ``{}``.

        Both or neither: the proposal is what stands in for the password, so a badge on
        its own is not a credential and PVapp would refuse it a round trip later.
        """
        badge, gup, _source = self.effective_pair()
        if not (badge and gup):
            return {}
        try:
            return {"badge": str(int(badge)), "gup": str(int(gup))}
        except (TypeError, ValueError):
            raise PVappError("pvapp badge=%r gup=%r must both be numbers" % (badge, gup))

    @property
    def auth(self):
        """HTTP Basic, unless the pair is doing the authenticating.

        Never both: PVapp tries Basic first, so a stale staff login would win and the
        confirmation would be attributed to the staff account instead of the badge.
        """
        if self.params:
            return None
        return (self.username, self.password) if self.username else None

    @property
    def identity(self):
        """One line for the log: which route, never the password."""
        params = self.params
        if params:
            return "badge %s on proposal %s" % (params["badge"], params["gup"])
        return "user %r" % self.username if self.username else "no credentials"

    def url(self, sample_id):
        return "%s/api/samples/%s" % (self.base_url, sample_id)

    def available(self):
        """Whether an update can even be attempted, and why not when it cannot."""
        if not self.base_url:
            return False, "no pvapp.base_url is configured"
        if self.params:
            return True, ""          # badge + proposal, no password needed
        if not self.username:
            return False, ("no PVapp credentials: set pvapp.badge and pvapp.gup (a badge "
                           "and a proposal it is on, no password, beamline network only), "
                           "export PVAPP_USERNAME and PVAPP_PASSWORD, or have the campaign "
                           "send set_credentials")
        if not self.username.isdigit() and not self.owner_badge:
            return False, ("logged in as staff (%r), so PVapp cannot tell whose "
                           "sample this is: set pvapp.owner_badge or "
                           "PALMIXER_PVAPP_BADGE" % self.username)
        return True, ""

    def confirm_mix(self, sample_id, slot, flowcell, mixing_record=None):
        """Merge a run into ``sample_id``'s existing PVapp record.

        Returns the updated record. Raises :class:`PVappError` on any failure,
        including "PALsystem never recorded this sample". The journal retains
        failed deliveries for retry without holding up hardware operations.
        Omitting ``mixing_record`` retains the legacy confirmation-only API.
        """
        import requests

        if mixing_record is not None and (not mixing_record.get("run_id")
                or mixing_record.get("sample_id") != sample_id):
            raise PVappError("Mixing record needs a run ID and the matching sample ID")

        ok, why = self.available()
        if not ok:
            raise PVappError(why)

        url = self.url(sample_id)
        try:
            got = requests.get(url, auth=self.auth, params=self.params,
                               timeout=self.timeout_s)
        except Exception as e:                          # noqa: BLE001
            raise PVappError("PVapp unreachable at %s: %s" % (url, e))
        if got.status_code == 404:
            raise PVappError(
                "PVapp has no record for %r -- PALsystem never posted one, so there is "
                "nothing here to confirm the mix of" % sample_id)
        if got.status_code >= 400:
            raise PVappError("PVapp returned %d for GET %s: %s"
                             % (got.status_code, url, got.text[:200]))
        try:
            record = got.json()
        except ValueError:
            raise PVappError("PVapp returned non-JSON for GET %s" % url)

        data = dict(record.get("data") or {})
        try:
            identities = {str(UUID(str(uid))) for uid in (
                record.get("sample_uid"), data.get("sample_uid"),
                (mixing_record or {}).get("sample_uid")) if uid}
        except ValueError:
            raise PVappError("Invalid sample_uid; refusing to update the sample") from None
        if len(identities) > 1:
            raise PVappError("sample_uid conflict; refusing to attach mixing to a different sample")
        uid = next(iter(identities), "")
        if uid and UUID(uid).int == 0:
            raise PVappError("sample_uid cannot be the nil UUID")
        if uid:
            data["sample_uid"] = uid
            if mixing_record is not None:
                mixing_record = dict(mixing_record, sample_uid=uid)
        if mixing_record is not None:
            runs = data.get("mixing_records", [])
            if not isinstance(runs, list) or any(not isinstance(run, dict) for run in runs):
                raise PVappError("PVapp mixing_records is not a list of events; refusing to replace it")
            # Idempotent delivery, preserving all other runs and all preparation data.
            updated = deepcopy(runs)
            existing = next((i for i, run in enumerate(updated)
                             if run.get("run_id") == mixing_record["run_id"]), None)
            if existing is None:
                updated.append(deepcopy(mixing_record))
            else:
                updated[existing] = deepcopy(mixing_record)
            data["mixing_records"] = updated
        if mixing_record is None or (mixing_record.get("mixing_status") == "completed"
                                     and not mixing_record.get("simulated")):
            summary = None
            if mixing_record is not None:
                # Retries can arrive out of order. Keep the summary on the
                # latest successful run, not whichever old file synced last.
                completed = [run for run in data["mixing_records"]
                             if run.get("mixing_status") == "completed" and not run.get("simulated")]
                summary = max(completed, key=_finished_at)
            data["mixed_by"] = "PALmixer"
            data["mixed_at"] = (_finished_at(summary) if summary else None) or time.strftime("%Y-%m-%dT%H:%M:%S")
            data["mixed_slot"] = summary.get("slot", slot) if summary else slot
            data["flowcell"] = summary.get("flowcell", flowcell) if summary else flowcell
        payload = {"data": data}

        try:
            put = requests.put(url, json=payload, auth=self.auth,
                               params=self.params, timeout=self.timeout_s)
        except Exception as e:                          # noqa: BLE001
            raise PVappError("PVapp unreachable at %s: %s" % (url, e))
        if put.status_code in (401, 403):
            raise PVappError(
                "PVapp refused the update (%d), authenticating as %s. Two routes: from a "
                "beamline address, pvapp.badge + pvapp.gup with no password; from "
                "anywhere else HTTP Basic, a BADGE NUMBER with that user's password or a "
                "STAFF NAME with the shared one."
                % (put.status_code, self.identity))
        if put.status_code >= 400:
            raise PVappError("PVapp returned %d for PUT %s: %s"
                             % (put.status_code, url, put.text[:200]))
        try:
            return put.json()
        except ValueError:
            raise PVappError("PVapp returned non-JSON for PUT %s" % url)


def _finished_at(record):
    """Journal timestamps are ISO 8601 UTC, so chronological sorting is stable."""
    pump = record.get("pump")
    return str((pump.get("finished_at") if isinstance(pump, dict) else None)
               or record.get("finished_at") or record.get("started_at") or "")
