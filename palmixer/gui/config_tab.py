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
    QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
    QPushButton, QVBoxLayout, QWidget,
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
        button_box.setLayout(button_layout)

        layout = QVBoxLayout()
        layout.addWidget(self.flowcell)
        layout.addWidget(self.camera, stretch=1)
        layout.addWidget(button_box)
        layout.addWidget(self._build_goto_group())
        layout.addWidget(self._build_teach_group())
        layout.addWidget(self._build_orientation_group())
        layout.addWidget(self._build_sync_group())
        self.setLayout(layout)

    def _on_search(self, station):
        self._send_command(cmd.search_apriltag_command(station))

    # -- go to a taught position -----------------------------------------------
    def _build_goto_group(self):
        """Drive to what a station's taught position actually is -- the way to
        check by eye what the search above, or the manual teach below, recorded.
        The server refuses a station nobody has taught yet, with the same
        message a transport gives."""
        box = QGroupBox("Go To Position")
        layout = QHBoxLayout()
        self.goto_buttons = []
        for station in cmd.STATIONS:
            btn = QPushButton(cmd.STATION_LABELS[station])
            btn.setToolTip(
                "Move the robot to the taught %s position and stop there, going "
                "by way of the transfer point. Nothing is picked up or released, "
                "and the arm stops at the pose a transport would start from -- "
                "above the object, not down on it." % cmd.STATION_LABELS[station])
            btn.clicked.connect(lambda _checked, s=station: self._on_goto(s))
            layout.addWidget(btn)
            self.goto_buttons.append(btn)
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
        for station in cmd.STATIONS:
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

    # -- teach an orientation by hand ------------------------------------------
    def _build_orientation_group(self):
        """Jog the wrist onto a seat angle and record just that angle.

        For a station the AprilTag search places well but cannot orient: the
        flowcell does not sit level in its cleaning station, and its 12 mm tag
        is too small at any workable standoff for the pose solver to resolve
        which way it is tilted. Centring on the tag is a 1-2 px measurement and
        stays reliable, so the taught X/Y/Z is kept and only RX/RY/RZ is
        replaced. The seat angle is fixed geometry -- this is a once-per-setup
        job, not part of a teach.
        """
        box = QGroupBox("Teach Orientation by Hand")
        layout = QHBoxLayout()

        layout.addWidget(QLabel("Step:"))
        self.rot_step_box = QDoubleSpinBox()
        self.rot_step_box.setDecimals(2)
        self.rot_step_box.setRange(0.05, 45.0)
        self.rot_step_box.setValue(1.0)
        self.rot_step_box.setSuffix(" deg")
        layout.addWidget(self.rot_step_box)

        self.rot_buttons = []
        for axis in cmd.ORIENTATION_AXES:
            for sign, glyph in ((-1.0, "-"), (+1.0, "+")):
                btn = QPushButton("R%s %s" % (axis.upper(), glyph))
                btn.setToolTip(
                    "Rotate the tool about its own %s axis. The gripper tip "
                    "stays where it is and the wrist swings around it, so the "
                    "position the AprilTag search found is not lost."
                    % axis.upper())
                btn.clicked.connect(
                    lambda _checked, a=axis, s=sign: self._on_tweak_orientation(a, s))
                layout.addWidget(btn)
                self.rot_buttons.append(btn)

        layout.addSpacing(12)
        self.orient_station = QComboBox()
        for station in cmd.STATIONS:
            self.orient_station.addItem(cmd.STATION_LABELS[station], station)
        # Opens on the station this exists for.
        default = self.orient_station.findData(cmd.STATION_CLEANING_STATION)
        if default >= 0:
            self.orient_station.setCurrentIndex(default)
        layout.addWidget(self.orient_station)

        self.save_orientation_btn = QPushButton("Save Orientation")
        self.save_orientation_btn.setToolTip(
            "Give the selected station the tool's current orientation, keeping "
            "its taught X/Y/Z. The station must already have a taught position.")
        self.save_orientation_btn.clicked.connect(self._on_save_orientation)
        layout.addWidget(self.save_orientation_btn)
        self.rot_buttons.append(self.save_orientation_btn)

        layout.addStretch(1)
        box.setLayout(layout)
        return box

    def _on_tweak_orientation(self, axis, sign):
        self._send_command(cmd.tweak_orientation_command(
            axis, sign * self.rot_step_box.value()))

    def _on_save_orientation(self):
        station = self.orient_station.currentData()
        label = cmd.STATION_LABELS[station]
        if QMessageBox.question(
                self, "Save Orientation",
                "Give %s the tool's current orientation?\n\n"
                "Its taught X/Y/Z is kept; only RX/RY/RZ is replaced."
                % label,
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        self._send_command(cmd.set_orientation_here_command(station))

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
                + list(self.teach_buttons) + list(self.rot_buttons)
                + list(self.sync_buttons) + self.flowcell.busy_widgets)

    def showEvent(self, event):
        super().showEvent(event)
        self.camera.start()

    def hideEvent(self, event):
        super().hideEvent(event)
        self.camera.stop()
