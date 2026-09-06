# -*- coding: utf-8 -*-
"""Configuration tab: live camera view + AprilTag search buttons.

Each button sends a ``search_apriltag <station>`` command over ZMQ to the
PALmixer server, which drives the robot to look for the station's AprilTag
(PAL12idb.locate_apriltag). The server replies "ACCEPTED" immediately and the
outcome shows up in the shared MQTT status log.
"""

from PyQt5.QtWidgets import QGroupBox, QHBoxLayout, QPushButton, QVBoxLayout, QWidget

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
        button_box.setLayout(button_layout)

        layout = QVBoxLayout()
        layout.addWidget(self.flowcell)
        layout.addWidget(self.camera, stretch=1)
        layout.addWidget(button_box)
        layout.addWidget(self._build_sync_group())
        self.setLayout(layout)

    def _on_search(self, station):
        self._send_command(cmd.search_apriltag_command(station))

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
        return (list(self.station_buttons) + list(self.sync_buttons)
                + self.flowcell.busy_widgets)

    def showEvent(self, event):
        super().showEvent(event)
        self.camera.start()

    def hideEvent(self, event):
        super().hideEvent(event)
        self.camera.stop()
