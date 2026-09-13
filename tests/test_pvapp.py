# -*- coding: utf-8 -*-
"""Confirming a mix in PVapp, and the login that lets PALmixer do it.

PALsystem posts a sample's record when the solutions go into the vial -- before
anything is mixed. These cover the other half: PALmixer updating that record once
``make_sample`` has actually run, and the ``set_credentials`` command a campaign uses
to hand over the PVapp login rather than having it exported separately on this host.

Nothing here touches a real socket, a real robot, or a real PVapp. ``requests`` is
replaced with a fake whose responses the test states outright, so what is asserted
about the HTTP is what PVapp would actually receive.
"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from palmixer import commands as cmd                                  # noqa: E402
from palmixer import pvapp                                            # noqa: E402


class Response:
    def __init__(self, status_code=200, body=None, text=""):
        self.status_code = status_code
        self._body = body
        self.text = text or json.dumps(body or {})

    def json(self):
        if self._body is None:
            raise ValueError("not JSON")
        return self._body


class FakeRequests:
    """Records what was sent and replays what the test said to answer."""

    def __init__(self, get=None, put=None):
        self._get = get or Response(200, {"sample_id": "s1", "data": {}})
        self._put = put or Response(200, {"sample_id": "s1", "data": {}})
        self.gets, self.puts = [], []

    def get(self, url, **kwargs):
        self.gets.append((url, kwargs))
        if isinstance(self._get, Exception):
            raise self._get
        return self._get

    def put(self, url, **kwargs):
        self.puts.append((url, kwargs))
        if isinstance(self._put, Exception):
            raise self._put
        return self._put


@pytest.fixture
def fake_requests(monkeypatch):
    """Install a fake ``requests`` for the import *inside* confirm_mix."""
    fake = FakeRequests()
    monkeypatch.setitem(sys.modules, "requests", fake)
    return fake


def updater(**kwargs):
    kwargs.setdefault("base_url", "http://pvapp.test")
    kwargs.setdefault("username", "12345")       # a badge, so no owner_badge needed
    kwargs.setdefault("password", "pw")
    return pvapp.PVappUpdater(**kwargs)


# --- when an update can be attempted at all ---------------------------------------------
def test_no_credentials_is_reported_not_guessed():
    ok, why = updater(username="", password="").available()
    assert not ok and "PVAPP_USERNAME" in why


def test_staff_login_without_a_badge_is_refused():
    """PVapp cannot infer an owner from a staff name, so it would 400 several seconds
    later; saying so here names the setting to fix."""
    ok, why = updater(username="blee").available()
    assert not ok and "owner_badge" in why


def test_a_badge_login_needs_no_owner_badge():
    assert updater(username="12345").available() == (True, "")


def test_no_base_url_is_reported():
    # Cleared after construction, not passed in: an empty argument falls back to the
    # configured default, which is the whole point of that `or` chain.
    client = updater()
    client.base_url = ""
    ok, why = client.available()
    assert not ok and "base_url" in why


# --- the read-modify-write --------------------------------------------------------------
def test_the_existing_record_is_read_and_merged_not_replaced(fake_requests):
    """PUT replaces `data` wholesale, so everything PALsystem wrote has to be carried
    forward -- losing the recipe to record the mix would be a bad trade."""
    fake_requests._get = Response(200, {"sample_id": "s1", "data": {
        "prepared_by": "PALsystem", "design": {"DSPC": 1.0}, "carousel_slot": 2}})

    updater().confirm_mix("s1", slot=2, flowcell=1)

    sent = fake_requests.puts[0][1]["json"]["data"]
    assert sent["prepared_by"] == "PALsystem"        # PALsystem's fields survive
    assert sent["design"] == {"DSPC": 1.0}
    assert sent["mixed_by"] == "PALmixer"            # and the mix is recorded alongside
    assert sent["mixed_slot"] == 2 and sent["flowcell"] == 1
    assert sent["mixed_at"]


def test_a_sample_palsystem_never_recorded_is_not_created(fake_requests):
    """A bare record made here would replace nothing useful with less -- the recipe,
    the design point and the actuals only exist on PALsystem's side."""
    fake_requests._get = Response(404, text="not found")

    with pytest.raises(pvapp.PVappError, match="never posted one"):
        updater().confirm_mix("s1", slot=2, flowcell=1)
    assert fake_requests.puts == []


def test_the_record_is_addressed_by_sample_id(fake_requests):
    updater().confirm_mix("lnp-001", slot=3, flowcell=2)
    assert fake_requests.gets[0][0] == "http://pvapp.test/api/samples/lnp-001"
    assert fake_requests.puts[0][0] == "http://pvapp.test/api/samples/lnp-001"


def test_a_refused_update_names_both_login_routes(fake_requests):
    fake_requests._put = Response(403, text="forbidden")
    with pytest.raises(pvapp.PVappError, match="BADGE NUMBER"):
        updater().confirm_mix("s1", slot=1, flowcell=1)


def test_an_unreachable_pvapp_names_the_url(fake_requests):
    fake_requests._get = RuntimeError("connection refused")
    with pytest.raises(pvapp.PVappError, match="unreachable at http://pvapp.test"):
        updater().confirm_mix("s1", slot=1, flowcell=1)


def test_credentials_are_checked_before_any_request_goes_out(fake_requests):
    with pytest.raises(pvapp.PVappError):
        updater(username="", password="").confirm_mix("s1", slot=1, flowcell=1)
    assert fake_requests.gets == [] and fake_requests.puts == []


# --- the set_credentials command ---------------------------------------------------------
def test_a_login_survives_the_wire_intact():
    """Including the characters that are exactly why it is base64'd rather than sent
    as bare tokens on a whitespace-delimited protocol."""
    password = "p ss:w rd with spaces"
    token = cmd.set_credentials_command("12345", password).split()[1]
    assert cmd.decode_credentials(token) == {"username": "12345", "password": password}


def test_a_malformed_payload_says_nothing_about_its_contents():
    """A credential blob that fails to parse must not put any part of itself in the
    error, which is what the server prints."""
    with pytest.raises(ValueError) as caught:
        cmd.decode_credentials("bm90LWpzb24=")       # valid base64, not JSON
    assert "not base64'd JSON" in str(caught.value)
    assert "not-json" not in str(caught.value)


def test_a_non_object_payload_is_refused():
    import base64

    token = base64.b64encode(b'["12345", "pw"]').decode("ascii")
    with pytest.raises(ValueError, match="must be a JSON object"):
        cmd.decode_credentials(token)
