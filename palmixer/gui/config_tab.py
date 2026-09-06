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


class ConfigTab(QWidget):
    def __init__(self, robot_ip, send_command, parent=None):
        super().__init__(parent)
        self._send_command = send_command

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
        layout.addWidget(self.camera, stretch=1)
        layout.addWidget(button_box)
        self.setLayout(layout)

    def _on_search(self, station):
        self._send_command(cmd.search_apriltag_command(station))

    @property
    def busy_widgets(self):
        """Widgets to disable while the server reports BUSY."""
        return list(self.station_buttons)

    def showEvent(self, event):
        super().showEvent(event)
        self.camera.start()

    def hideEvent(self, event):
        super().hideEvent(event)
        self.camera.stop()
