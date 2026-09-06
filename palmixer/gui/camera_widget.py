# -*- coding: utf-8 -*-
"""Live wrist-camera view, pulled directly over HTTP.

The UR wrist camera exposes its own tiny HTTP server on port 4242
(``http://<robot-ip>:4242/current.jpg?type=color``), independent of the UR
controller and of PALmixer's ZMQ control plane (see
UR_12idb/common/urcamera.py:334, ``camera.capture()`` for the IP case). That
means this widget needs no robot object, no OpenCV, and works even while the
ZMQ server/robot is busy or stopped -- it just polls JPEG stills with
``requests`` and paints them into a QLabel.
"""

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QLabel, QSizePolicy

CAMERA_URL_TMPL = "http://{ip}:4242/current.jpg?type=color"
POLL_INTERVAL_MS = 200  # ~5 fps; the camera's own HTTP server is the real limit
REQUEST_TIMEOUT_S = 1.0


class CameraWidget(QLabel):
    """A QLabel that keeps repainting itself with the latest camera frame."""

    def __init__(self, robot_ip, parent=None):
        super().__init__(parent)
        self.robot_ip = robot_ip
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: black; color: gray;")
        self.setText("Camera feed not started")

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll_frame)

    def start(self):
        self._timer.start(POLL_INTERVAL_MS)

    def stop(self):
        self._timer.stop()

    def _poll_frame(self):
        import requests

        url = CAMERA_URL_TMPL.format(ip=self.robot_ip)
        try:
            resp = requests.get(url, timeout=REQUEST_TIMEOUT_S)
            resp.raise_for_status()
        except Exception as e:
            self.setText("Camera unavailable:\n%s" % e)
            return

        pixmap = QPixmap()
        if not pixmap.loadFromData(resp.content):
            self.setText("Camera: received unreadable image data")
            return
        scaled = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(scaled)
