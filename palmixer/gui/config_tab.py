# -*- coding: utf-8 -*-
"""Configuration tab: live camera view, one Position panel, and a jog panel.

The Position panel is a single row -- pick a position from the dropdown, pick an
action (Search AprilTag / Go To Position / Set Current Position), and press Go --
rather than a button per position per action, which did not scale once there were
eight taught positions. Each action maps to the same ZMQ commands as before:

* Search AprilTag -> ``search_apriltag <station> [skip_roll] [from_current]``;
  the server drives the robot to find the station's tag (PAL12idb.locate_apriltag).
* Go To Position  -> ``goto_position <station>`` (or ``goto_transfer_point``).
* Set Current Position -> ``set_position_here <station>`` (or
  ``set_transfer_point_here``) -- records where the arm is standing now.

"Stop Search" sends ``stop_search``, a fast command the server answers even while
busy (see server.py's fast_dispatch) -- deliberately left out of busy_widgets so
it stays enabled for exactly the window where it is useful.
"""

from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel,
    QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from .. import commands as cmd
from .camera_widget import CameraWidget
from .flowcell_selector import FlowcellSelector


class ConfigTab(QWidget):
    # Dropdown data sentinel for the corridor waypoint, which is not a taught
    # station (no AprilTag, its own commands) but is a "position" for Go/Set.
    _TRANSFER_POINT = "__transfer_point__"

    # Action values carried as the action dropdown's itemData.
    _SEARCH, _GOTO, _SET = "search", "goto", "set"

    def __init__(self, robot_ip, send_command, query=None, parent=None):
        super().__init__(parent)
        self._send_command = send_command

        self.flowcell = FlowcellSelector(send_command)
        self.camera = CameraWidget(robot_ip, query=query)

        # Freeze the calibrated Venton coordinate frame onto the live image. A
        # checkable toggle, not a one-shot: on press it asks the server to
        # project the origin + X/Y/Z axes into the current view and holds that
        # overlay fixed until pressed again (it does not track the arm).
        self.venton_axes_btn = QPushButton("Show Venton Axes")
        self.venton_axes_btn.setCheckable(True)
        self.venton_axes_btn.setToolTip(
            "Overlay the calibrated Venton table origin and X/Y/Z axes on the "
            "live image, frozen at the pose you press it from. Press again to "
            "clear. Run a Venton calibration first.")
        self.venton_axes_btn.toggled.connect(self._on_toggle_venton_axes)

        overlay_row = QHBoxLayout()
        overlay_row.addWidget(self.venton_axes_btn)
        overlay_row.addStretch(1)

        layout = QVBoxLayout()
        layout.addWidget(self.flowcell)
        layout.addWidget(self.camera, stretch=1)
        layout.addLayout(overlay_row)
        layout.addWidget(self._build_position_group())
        layout.addWidget(self._build_jog_group())
        self.setLayout(layout)
        self._sync_search_options()

    # -- position: pick one, pick an action, press Go --------------------------
    def _build_position_group(self):
        box = QGroupBox("Position")
        rows = QVBoxLayout()

        row = QHBoxLayout()
        row.addWidget(QLabel("Position:"))
        self.position_combo = QComboBox()
        for station in cmd.TAUGHT_STATIONS:
            self.position_combo.addItem(cmd.STATION_LABELS[station], station)
        # The corridor waypoint is a position for Go/Set but not a station -- it
        # has no tag, falls back to a built-in default, and uses its own
        # commands. Listed last so the taught stations stay together.
        self.position_combo.addItem("Transfer Point", self._TRANSFER_POINT)
        row.addWidget(self.position_combo)

        row.addSpacing(8)
        row.addWidget(QLabel("Action:"))
        self.action_combo = QComboBox()
        self.action_combo.addItem("Search AprilTag", self._SEARCH)
        self.action_combo.addItem("Go To Position", self._GOTO)
        self.action_combo.addItem("Set Current Position", self._SET)
        self.action_combo.currentIndexChanged.connect(self._sync_search_options)
        row.addWidget(self.action_combo)

        self.go_btn = QPushButton("Go")
        self.go_btn.setToolTip(
            "Run the selected action on the selected position:\n"
            " - Search AprilTag: drive to the station and find its tag.\n"
            " - Go To Position: move to the taught pose and stop.\n"
            " - Set Current Position: record where the arm is standing now.")
        self.go_btn.clicked.connect(self._on_go)
        row.addWidget(self.go_btn)

        # Stays live while the server is BUSY -- that is when an in-progress
        # search needs stopping -- so it is kept out of busy_widgets.
        self.stop_search_button = QPushButton("Stop Search")
        self.stop_search_button.setStyleSheet("color: darkred; font-weight: bold;")
        self.stop_search_button.setToolTip(
            "Abort an in-progress AprilTag search and stop the robot immediately.")
        self.stop_search_button.clicked.connect(
            lambda: self._send_command(cmd.stop_search_command()))
        row.addWidget(self.stop_search_button)
        row.addStretch(1)
        rows.addLayout(row)

        # Search-only options, enabled only when the action is Search.
        opt_row = QHBoxLayout()
        self.skip_roll_check = QCheckBox("Keep camera face-down (skip roll)")
        self.skip_roll_check.setToolTip(
            "Search for the tag and record the position, but leave the camera "
            "level (facing straight down) instead of tipping it face-down or "
            "squaring it to a tilted tag. Use for a seat whose tilt you teach "
            "by hand afterward.")
        opt_row.addWidget(self.skip_roll_check)
        self.from_current_check = QCheckBox("Start from the current position")
        self.from_current_check.setToolTip(
            "Search from where the arm is standing now, instead of first "
            "driving to the station's configured search reference.\n\n"
            "The camera must be looking down. If it already is, nothing is "
            "reoriented and the view does not shift; if it is not, the camera "
            "is brought face-down first, which moves the arm. Use the Robot "
            "panel's Camera to TCP button on the Experiment tab to get it "
            "there yourself beforehand.")
        opt_row.addWidget(self.from_current_check)
        opt_row.addStretch(1)
        rows.addLayout(opt_row)

        box.setLayout(rows)
        return box

    def _sync_search_options(self, *_):
        """The skip-roll / from-current options only mean anything for a search,
        so grey them out for the other actions."""
        is_search = self.action_combo.currentData() == self._SEARCH
        self.skip_roll_check.setEnabled(is_search)
        self.from_current_check.setEnabled(is_search)

    def _on_go(self):
        pos = self.position_combo.currentData()
        action = self.action_combo.currentData()
        is_transfer = (pos == self._TRANSFER_POINT)
        label = "Transfer Point" if is_transfer else cmd.STATION_LABELS[pos]

        if action == self._SEARCH:
            if is_transfer or pos not in cmd.STATIONS:
                QMessageBox.information(
                    self, "Search AprilTag",
                    "%s has no AprilTag to search for.\n\nUse Go To Position or "
                    "Set Current Position for it instead." % label)
                return
            self._send_command(cmd.search_apriltag_command(
                pos, skip_roll=self.skip_roll_check.isChecked(),
                from_current=self.from_current_check.isChecked()))

        elif action == self._GOTO:
            self._send_command(cmd.goto_transfer_point_command() if is_transfer
                               else cmd.goto_position_command(pos))

        elif action == self._SET:
            # Overwrites a taught pose every later move reads, and the arm's
            # current pose is not something the dialog can show back, so confirm
            # against the position name before sending.
            if QMessageBox.question(
                    self, "Set Current Position",
                    "Record the robot's current pose as %s?\n\n"
                    "This replaces the stored position every move to %s uses.\n\n"
                    "The arm must be left where a search would leave it -- "
                    "transports descend from the stored pose, so setting this "
                    "with the gripper already down records a position too low."
                    % (label, label),
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
                return
            self._send_command(cmd.set_transfer_point_here_command() if is_transfer
                               else cmd.set_position_here_command(pos))

    def _on_toggle_venton_axes(self, checked):
        """Turn the frozen Venton axes overlay on or off. On failure to compute
        it (no calibration yet, robot busy, ...) leave the button off and say
        why, rather than showing a stale or empty overlay."""
        ok, message = self.camera.set_venton_axes_visible(checked)
        if checked and not ok:
            # Revert without re-firing this handler.
            self.venton_axes_btn.blockSignals(True)
            self.venton_axes_btn.setChecked(False)
            self.venton_axes_btn.blockSignals(False)
            QMessageBox.warning(self, "Show Venton Axes",
                                "Could not show the Venton axes:\n\n%s"
                                % (message or "unknown error"))

    # -- jog the pose by hand, then save it ------------------------------------
    def _build_jog_group(self):
        """Jog position and orientation together, and record the result.

        One panel rather than two, because they are one job: getting the arm
        onto a seat by eye. A tilted seat needs both -- rotating the wrist onto
        the seat angle usually needs the position nudged after it, and the
        operator was otherwise moving between two boxes to do one thing.

        The two halves keep their own step sizes (mm and degrees) and their own
        frames, which is the one thing to know about this panel:

        * **Position is base frame.** "+X" moves the arm the way the stored
          `x` increases, so what is watched on camera and what lands in
          waypoints.ini agree.
        * **Orientation is tool frame.** The gripper tip stays where it is and
          the wrist swings around it, so a rotation does not undo the position
          just set.

        Saving writes the *whole* pose (see _on_save_pose).
        """
        box = QGroupBox("Move Robot by Hand")
        rows = QVBoxLayout()

        # -- position row
        pos_row = QHBoxLayout()
        pos_row.addWidget(QLabel("Move:"))
        self.pos_step_box = QDoubleSpinBox()
        self.pos_step_box.setDecimals(2)
        # Upper end matches PAL12idb.MAX_POSITION_TWEAK (50 mm), which the
        # server enforces independently; the spin box is the convenient limit,
        # not the safety one.
        self.pos_step_box.setRange(0.1, 50.0)
        self.pos_step_box.setValue(1.0)
        self.pos_step_box.setSingleStep(0.5)
        self.pos_step_box.setSuffix(" mm")
        pos_row.addWidget(self.pos_step_box)

        self.pos_buttons = []
        for axis, sense in zip(cmd.POSITION_AXES,
                               ("out board / in board", "along / against the X-ray",
                                "up / down")):
            for sign, glyph in ((-1.0, "-"), (+1.0, "+")):
                btn = QPushButton("%s %s" % (axis.upper(), glyph))
                btn.setToolTip(
                    "Move the arm %s along the base %s axis (%s), keeping the "
                    "tool's orientation.\n\nBase frame, so the step shown is "
                    "exactly how much the saved X/Y/Z changes."
                    % ("+" if sign > 0 else "-", axis.upper(), sense))
                btn.clicked.connect(
                    lambda _checked, a=axis, s=sign: self._on_tweak_position(a, s))
                pos_row.addWidget(btn)
                self.pos_buttons.append(btn)
        pos_row.addStretch(1)
        rows.addLayout(pos_row)

        # -- orientation row
        rot_row = QHBoxLayout()
        rot_row.addWidget(QLabel("Rotate:"))
        self.rot_step_box = QDoubleSpinBox()
        self.rot_step_box.setDecimals(2)
        self.rot_step_box.setRange(0.05, 45.0)
        self.rot_step_box.setValue(1.0)
        self.rot_step_box.setSuffix(" deg")
        rot_row.addWidget(self.rot_step_box)

        self.rot_buttons = []
        for axis in cmd.ORIENTATION_AXES:
            for sign, glyph in ((-1.0, "-"), (+1.0, "+")):
                btn = QPushButton("R%s %s" % (axis.upper(), glyph))
                btn.setToolTip(
                    "Rotate the tool about its own %s axis. The gripper tip "
                    "stays where it is and the wrist swings around it, so a "
                    "rotation does not move the position you just set."
                    % axis.upper())
                btn.clicked.connect(
                    lambda _checked, a=axis, s=sign: self._on_tweak_orientation(a, s))
                rot_row.addWidget(btn)
                self.rot_buttons.append(btn)
        rot_row.addStretch(1)
        rows.addLayout(rot_row)

        # -- save row
        save_row = QHBoxLayout()
        save_row.addWidget(QLabel("Save as:"))
        self.jog_station = QComboBox()
        for station in cmd.TAUGHT_STATIONS:
            self.jog_station.addItem(cmd.STATION_LABELS[station], station)
        # Opens on the station this panel exists for: the one whose seat tilt
        # the AprilTag search cannot resolve.
        default = self.jog_station.findData(cmd.STATION_CLEANING_STATION)
        if default >= 0:
            self.jog_station.setCurrentIndex(default)
        save_row.addWidget(self.jog_station)

        self.save_pose_btn = QPushButton("Save Position && Orientation")
        self.save_pose_btn.setToolTip(
            "Record the arm's current pose -- X/Y/Z and RX/RY/RZ together -- "
            "as the selected station's taught position, replacing whatever the "
            "AprilTag search last found.\n\nThe arm must be left where a search "
            "would leave it: transports descend from the stored pose to reach "
            "the object, so saving with the gripper already down on the object "
            "teaches a position that is too low.")
        self.save_pose_btn.clicked.connect(self._on_save_pose)
        save_row.addWidget(self.save_pose_btn)
        save_row.addStretch(1)
        rows.addLayout(save_row)

        box.setLayout(rows)
        return box

    def _on_tweak_position(self, axis, sign):
        self._send_command(cmd.tweak_position_command(
            axis, sign * self.pos_step_box.value()))

    def _on_tweak_orientation(self, axis, sign):
        self._send_command(cmd.tweak_orientation_command(
            axis, sign * self.rot_step_box.value()))

    def _on_save_pose(self):
        """Save the whole pose, not the orientation alone.

        set_position_here records the full pose
        (PAL12idb.record_current_position), so both the jogged position and the
        jogged orientation survive.
        """
        station = self.jog_station.currentData()
        label = cmd.STATION_LABELS[station]
        if QMessageBox.question(
                self, "Save Position and Orientation",
                "Record the robot's current pose as %s?\n\n"
                "Both its position (X/Y/Z) and its orientation (RX/RY/RZ) are "
                "replaced, and every move to %s uses them." % (label, label),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        self._send_command(cmd.set_position_here_command(station))

    @property
    def busy_widgets(self):
        """Widgets to disable while the server reports BUSY. Stop Search and the
        search-option checkboxes stay live; everything that starts motion does
        not."""
        return ([self.position_combo, self.action_combo, self.go_btn]
                + list(self.pos_buttons) + list(self.rot_buttons)
                + [self.save_pose_btn, self.jog_station]
                + self.flowcell.busy_widgets)

    def showEvent(self, event):
        super().showEvent(event)
        self.camera.start()

    def hideEvent(self, event):
        super().hideEvent(event)
        self.camera.stop()
