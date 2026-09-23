# -*- coding: utf-8 -*-
"""Configuration tab: live camera view + AprilTag search buttons.

Each search button sends a ``search_apriltag <station>`` command over ZMQ to
the PALmixer server, which drives the robot to look for the station's
AprilTag (PAL12idb.locate_apriltag). The server replies "ACCEPTED"
immediately and the outcome shows up in the shared MQTT status log.

"Stop Search" sends ``stop_search``, a fast command the server answers even
while busy (see server.py's fast_dispatch) -- it is deliberately left out of
busy_widgets so it stays enabled for exactly the window where it is useful.
"""

from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel,
    QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from .. import commands as cmd
from .camera_widget import CameraWidget
from .flowcell_selector import FlowcellSelector


class ConfigTab(QWidget):
    def __init__(self, robot_ip, send_command, parent=None):
        super().__init__(parent)
        self._send_command = send_command

        self.flowcell = FlowcellSelector(send_command)
        self.camera = CameraWidget(robot_ip)

        button_box = QGroupBox("Search AprilTag")
        button_layout = QHBoxLayout()
        self.station_buttons = []
        for station in cmd.STATIONS:
            btn = QPushButton(cmd.STATION_LABELS[station])
            btn.clicked.connect(lambda _checked, s=station: self._on_search(s))
            button_layout.addWidget(btn)
            self.station_buttons.append(btn)
        self.stop_search_button = QPushButton("Stop Search")
        self.stop_search_button.setStyleSheet("color: darkred; font-weight: bold;")
        self.stop_search_button.setToolTip(
            "Abort an in-progress AprilTag search and stop the robot immediately.")
        self.stop_search_button.clicked.connect(
            lambda: self._send_command(cmd.stop_search_command()))
        button_layout.addWidget(self.stop_search_button)
        # Applies to whichever station is searched next: keep the camera level
        # (face straight down) instead of rolling it face-down / squaring it to
        # a tilted tag. Position is still recorded; teach a tilted seat's angle
        # by hand (below) afterward. Default off = the current search behavior.
        self.skip_roll_check = QCheckBox("Keep camera face-down (skip roll)")
        self.skip_roll_check.setToolTip(
            "Search for the tag and record the position, but leave the camera "
            "level (facing straight down) instead of tipping it face-down or "
            "squaring it to a tilted tag. Use for a seat whose tilt you teach "
            "by hand afterward.")
        button_layout.addWidget(self.skip_roll_check)
        button_box.setLayout(button_layout)

        layout = QVBoxLayout()
        layout.addWidget(self.flowcell)
        layout.addWidget(self.camera, stretch=1)
        layout.addWidget(button_box)
        layout.addWidget(self._build_goto_group())
        layout.addWidget(self._build_teach_group())
        layout.addWidget(self._build_jog_group())
        layout.addWidget(self._build_sync_group())
        self.setLayout(layout)

    def _on_search(self, station):
        self._send_command(cmd.search_apriltag_command(
            station, skip_roll=self.skip_roll_check.isChecked()))

    # -- go to a taught position -----------------------------------------------
    def _build_goto_group(self):
        """Drive to what a station's taught position actually is -- the way to
        check by eye what the search above, or the manual teach below, recorded.
        The server refuses a station nobody has taught yet, with the same
        message a transport gives."""
        box = QGroupBox("Go To Position")
        layout = QHBoxLayout()
        self.goto_buttons = []
        for station in cmd.TAUGHT_STATIONS:
            btn = QPushButton(cmd.STATION_LABELS[station])
            btn.setToolTip(
                "Move the robot to the taught %s position and stop there, going "
                "by way of the transfer point. Nothing is picked up or released, "
                "and the arm stops at the pose a transport would start from -- "
                "above the object, not down on it." % cmd.STATION_LABELS[station])
            btn.clicked.connect(lambda _checked, s=station: self._on_goto(s))
            layout.addWidget(btn)
            self.goto_buttons.append(btn)

        # The corridor waypoint, kept apart from the stations because it is not
        # one: nothing is taught for it and nothing can be unconfigured about
        # it, so this button works on a fresh install where the others refuse.
        layout.addSpacing(12)
        self.transfer_point_btn = QPushButton("Transfer Point")
        self.transfer_point_btn.setToolTip(
            "Move the robot to the transfer point -- the fixed corridor pose "
            "every move between the sample table and the mixer side routes "
            "through.\n\nNot a taught position, so this works before anything "
            "has been configured. Useful to park the arm clear, or to start "
            "from a known place: from here every station is one ordinary move "
            "away.")
        self.transfer_point_btn.clicked.connect(
            lambda: self._send_command(cmd.goto_transfer_point_command()))
        layout.addWidget(self.transfer_point_btn)
        self.goto_buttons.append(self.transfer_point_btn)

        layout.addStretch(1)
        box.setLayout(layout)
        return box

    def _on_goto(self, station):
        self._send_command(cmd.goto_position_command(station))

    # -- teach from the current pose -------------------------------------------
    def _build_teach_group(self):
        """The manual counterpart to the AprilTag search above: record where the
        arm is standing as a station's position. For a station whose tag is
        obscured, or one being nudged off a position that is already close."""
        box = QGroupBox("Set Current Robot Position As")
        layout = QHBoxLayout()
        self.teach_buttons = []
        for station in cmd.TAUGHT_STATIONS:
            btn = QPushButton(cmd.STATION_LABELS[station])
            btn.setToolTip(
                "Record the robot's current pose as the %s position, replacing "
                "whatever the AprilTag search last found.\n\nThe arm must be left "
                "where a search would leave it: transports descend from the "
                "stored pose to reach the object, so teaching this with the "
                "gripper already down on the object records a position that is "
                "too low." % cmd.STATION_LABELS[station])
            btn.clicked.connect(lambda _checked, s=station: self._on_teach_here(s))
            layout.addWidget(btn)
            self.teach_buttons.append(btn)
        layout.addStretch(1)
        box.setLayout(layout)
        return box

    def _on_teach_here(self, station):
        # Overwrites a taught position every later move reads, and the arm's
        # current pose is not something the dialog can show back, so confirm
        # against the station name before sending.
        if QMessageBox.question(
                self, "Set Position",
                "Record the robot's current position as %s?\n\n"
                "This replaces the taught position that every move to %s uses."
                % (cmd.STATION_LABELS[station], cmd.STATION_LABELS[station]),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        self._send_command(cmd.set_position_here_command(station))

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

        This button used to send set_orientation_here, which keeps the taught
        X/Y/Z and replaces only RX/RY/RZ -- it predates there being any way to
        jog the position from the GUI, so position was never the operator's to
        change here. Now that it is, saving half of what was just jogged would
        silently throw the other half away. set_position_here records the full
        pose (PAL12idb.record_current_position), so both survive.
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

    # -- EPICS waypoint sync ---------------------------------------------------
    def _build_sync_group(self):
        """Taught positions live in the server's waypoints.ini, which is what
        every move reads; the 12idUR:WaypointL:* PVs are a separate copy for
        the rest of the beamline. These two buttons are the only thing that
        moves values between them, in either direction."""
        box = QGroupBox("EPICS Waypoint PVs")
        layout = QHBoxLayout()
        self.sync_buttons = []
        for label, command, tip in (
            ("Push to EPICS PVs", cmd.push_positions_command(),
             "Copy every taught position out to its 12idUR:WaypointL:* PVs."),
            ("Pull from EPICS PVs", cmd.pull_positions_command(),
             "Import positions from the 12idUR:WaypointL:* PVs, replacing the "
             "taught positions this server uses."),
        ):
            btn = QPushButton(label)
            btn.setToolTip(tip)
            btn.clicked.connect(lambda _checked, c=command: self._send_command(c))
            layout.addWidget(btn)
            self.sync_buttons.append(btn)
        layout.addStretch(1)
        box.setLayout(layout)
        return box

    @property
    def busy_widgets(self):
        """Widgets to disable while the server reports BUSY."""
        return (list(self.station_buttons) + list(self.goto_buttons)
                + list(self.teach_buttons) + list(self.pos_buttons)
                + list(self.rot_buttons) + [self.save_pose_btn]
                + list(self.sync_buttons) + self.flowcell.busy_widgets)

    def showEvent(self, event):
        super().showEvent(event)
        self.camera.start()

    def hideEvent(self, event):
        super().hideEvent(event)
        self.camera.stop()
