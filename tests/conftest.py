"""Shared test setup."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer import state


@pytest.fixture(autouse=True)
def _isolated_state(tmp_path, monkeypatch):
    # No test reads or writes the live palmixer_state.ini: the server on this
    # machine reads it, and a station write can change it -- a new
    # carousel_on_mixer position clears the measured mixer seat there.
    monkeypatch.setattr(state, "_INI_PATH", str(tmp_path / "palmixer_state.ini"))
