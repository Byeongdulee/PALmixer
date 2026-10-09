"""The Configuration tab's consolidated Position panel.

One position dropdown + one action dropdown + a Go button replaced the three
groups of a button per station per action. These pin the dispatch: Go sends the
right command for each (position, action) pair, the transfer point uses its own
commands, a search on a tagless position is refused, and the search-only options
grey out for the other actions.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))

QtWidgets = pytest.importorskip("PyQt5.QtWidgets", reason="PyQt5 not installed")
pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")

from palmixer import commands as cmd
import palmixer.gui.config_tab as ct
from palmixer.gui.config_tab import ConfigTab


@pytest.fixture(scope="module")
def qapp():
    yield QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def tab(qapp):
    sent = []
    w = ConfigTab("127.0.0.1", sent.append)
    w.sent = sent
    return w


def _select(combo, data):
    i = combo.findData(data)
    assert i >= 0, "no item with data %r" % (data,)
    combo.setCurrentIndex(i)


@pytest.fixture
def yes(monkeypatch):
    monkeypatch.setattr(ct.QMessageBox, "question", lambda *a, **k: ct.QMessageBox.Yes)


@pytest.fixture
def no(monkeypatch):
    monkeypatch.setattr(ct.QMessageBox, "question", lambda *a, **k: ct.QMessageBox.No)


def test_position_dropdown_lists_taught_stations_then_transfer_point(tab):
    datas = [tab.position_combo.itemData(i) for i in range(tab.position_combo.count())]
    assert datas == list(cmd.TAUGHT_STATIONS) + [ConfigTab._TRANSFER_POINT]


def test_action_dropdown_has_the_three_actions(tab):
    datas = [tab.action_combo.itemData(i) for i in range(tab.action_combo.count())]
    assert datas == [ConfigTab._SEARCH, ConfigTab._GOTO, ConfigTab._SET]


def test_the_three_old_groups_are_gone(tab):
    titles = [g.title() for g in tab.findChildren(QtWidgets.QGroupBox)]
    assert "Position" in titles
    for gone in ("Search AprilTag", "Go To Position", "Set Current Robot Position As"):
        assert gone not in titles


def test_go_searches_the_selected_station(tab):
    _select(tab.position_combo, "mixer_station")
    _select(tab.action_combo, ConfigTab._SEARCH)
    tab.go_btn.click()
    assert tab.sent == ["search_apriltag mixer_station"]


def test_search_carries_the_option_flags(tab):
    _select(tab.position_combo, "sample_table")
    _select(tab.action_combo, ConfigTab._SEARCH)
    tab.skip_roll_check.setChecked(True)
    tab.from_current_check.setChecked(True)
    tab.go_btn.click()
    assert tab.sent == ["search_apriltag sample_table skip_roll from_current"]


def test_go_drives_to_the_selected_station(tab):
    _select(tab.position_combo, "mixer_station")
    _select(tab.action_combo, ConfigTab._GOTO)
    tab.go_btn.click()
    assert tab.sent == ["goto_position mixer_station"]


def test_go_drives_to_the_transfer_point(tab):
    _select(tab.position_combo, ConfigTab._TRANSFER_POINT)
    _select(tab.action_combo, ConfigTab._GOTO)
    tab.go_btn.click()
    assert tab.sent == ["goto_transfer_point"]


def test_set_confirms_then_records_the_station(tab, yes):
    _select(tab.position_combo, "cleaning_station")
    _select(tab.action_combo, ConfigTab._SET)
    tab.go_btn.click()
    assert tab.sent == ["set_position_here cleaning_station"]


def test_set_transfer_point_uses_its_own_command(tab, yes):
    _select(tab.position_combo, ConfigTab._TRANSFER_POINT)
    _select(tab.action_combo, ConfigTab._SET)
    tab.go_btn.click()
    assert tab.sent == ["set_transfer_point_here"]


def test_set_cancelled_sends_nothing(tab, no):
    _select(tab.position_combo, "cleaning_station")
    _select(tab.action_combo, ConfigTab._SET)
    tab.go_btn.click()
    assert tab.sent == []


def test_search_on_a_tagless_position_is_refused(tab, monkeypatch):
    warned = []
    monkeypatch.setattr(ct.QMessageBox, "information", lambda *a, **k: warned.append(a))
    _select(tab.action_combo, ConfigTab._SEARCH)
    for pos in (ConfigTab._TRANSFER_POINT, "sample_on_mixer_station"):
        _select(tab.position_combo, pos)
        tab.go_btn.click()
    assert tab.sent == []
    assert len(warned) == 2


def test_carousels_are_selectable_and_searchable(tab):
    for pos in ("carousel_on_mixer", "carousel_in_stock", "carousel_used_stack"):
        tab.sent.clear()
        _select(tab.position_combo, pos)
        _select(tab.action_combo, ConfigTab._SEARCH)
        tab.go_btn.click()
        assert tab.sent == ["search_apriltag %s" % pos]


def test_search_options_enabled_only_for_search(tab):
    _select(tab.action_combo, ConfigTab._SEARCH)
    assert tab.skip_roll_check.isEnabled()
    assert tab.from_current_check.isEnabled()
    _select(tab.action_combo, ConfigTab._GOTO)
    assert not tab.skip_roll_check.isEnabled()
    assert not tab.from_current_check.isEnabled()


def test_go_is_gated_on_busy_but_stop_search_is_not(tab):
    assert tab.go_btn in tab.busy_widgets
    assert tab.position_combo in tab.busy_widgets
    assert tab.action_combo in tab.busy_widgets
    assert tab.stop_search_button not in tab.busy_widgets
