"""The Configuration tab's combined jog panel.

Position and orientation are jogged in one panel, because they are one job:
getting the arm onto a seat by eye. The save button records the *whole* pose --
saving the orientation alone would throw away the position just jogged.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))

QtWidgets = pytest.importorskip("PyQt5.QtWidgets", reason="PyQt5 not installed")
pytest.importorskip("palmixer.PAL12idb", reason="UR_12idb not importable")

from palmixer import commands as cmd
from palmixer.gui.config_tab import ConfigTab


@pytest.fixture(scope="module")
def qapp():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


@pytest.fixture
def tab(qapp):
    sent = []
    widget = ConfigTab("127.0.0.1", sent.append)
    widget.sent = sent
    return widget


def _titles(widget):
    return [g.title() for g in widget.findChildren(QtWidgets.QGroupBox)]


def test_there_is_one_jog_panel_not_two(tab):
    titles = _titles(tab)
    assert "Move Robot by Hand" in titles
    assert "Teach Orientation by Hand" not in titles, "the panels were not combined"
    assert sum(t == "Move Robot by Hand" for t in titles) == 1


def test_both_jogs_live_in_it_with_their_own_step_sizes(tab):
    """Millimetres and degrees are different quantities; one shared step box
    would make one of them useless."""
    assert [b.text() for b in tab.pos_buttons] == ["X -", "X +", "Y -", "Y +", "Z -", "Z +"]
    assert [b.text() for b in tab.rot_buttons] == ["RX -", "RX +", "RY -", "RY +", "RZ -", "RZ +"]
    assert tab.pos_step_box.suffix().strip() == "mm"
    assert tab.rot_step_box.suffix().strip() == "deg"
    assert tab.pos_step_box is not tab.rot_step_box


def test_each_jog_button_sends_its_own_step(tab):
    tab.pos_step_box.setValue(0.5)
    tab.rot_step_box.setValue(2.0)
    for btn in tab.pos_buttons + tab.rot_buttons:
        btn.click()
    assert tab.sent == [
        "tweak_position x -0.5", "tweak_position x 0.5",
        "tweak_position y -0.5", "tweak_position y 0.5",
        "tweak_position z -0.5", "tweak_position z 0.5",
        "tweak_orientation x -2.0", "tweak_orientation x 2.0",
        "tweak_orientation y -2.0", "tweak_orientation y 2.0",
        "tweak_orientation z -2.0", "tweak_orientation z 2.0",
    ]


def test_save_records_the_whole_pose_not_the_orientation_alone(tab, monkeypatch):
    """The regression this change exists to prevent: set_orientation_here keeps
    the taught X/Y/Z, so it would discard whatever the position jog just did."""
    monkeypatch.setattr(QtWidgets.QMessageBox, "question",
                        staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Yes))
    index = tab.jog_station.findData(cmd.STATION_SAMPLE_ON_MIXER)
    tab.jog_station.setCurrentIndex(index)
    tab.save_pose_btn.click()
    assert tab.sent == ["set_position_here sample_on_mixer_station"]
    assert not any("set_orientation_here" in s for s in tab.sent)


def test_saying_no_saves_nothing(tab, monkeypatch):
    """It overwrites a taught position every later move reads."""
    monkeypatch.setattr(QtWidgets.QMessageBox, "question",
                        staticmethod(lambda *a, **k: QtWidgets.QMessageBox.No))
    tab.save_pose_btn.click()
    assert tab.sent == []


def test_the_save_button_says_it_saves_both(tab):
    """Labelled "Save Orientation" it would be actively misleading now."""
    # Qt uses && for a literal ampersand in a button label.
    assert tab.save_pose_btn.text().replace("&&", "&") == "Save Position & Orientation"


def test_it_opens_on_the_station_it_exists_for(tab):
    """The flowcell cleaning station: the seat whose tilt the AprilTag search
    cannot resolve, which is why hand-teaching exists at all."""
    assert tab.jog_station.currentData() == cmd.STATION_CLEANING_STATION


def test_every_station_can_be_saved_to(tab):
    stations = [tab.jog_station.itemData(i) for i in range(tab.jog_station.count())]
    assert stations == list(cmd.TAUGHT_STATIONS)


def test_the_whole_panel_greys_out_while_busy(tab):
    """Every control here moves or writes; none is a stop button."""
    busy = tab.busy_widgets
    for widget in tab.pos_buttons + tab.rot_buttons + [tab.save_pose_btn]:
        assert widget in busy
