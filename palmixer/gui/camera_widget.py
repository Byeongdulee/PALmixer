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
AT_MIN_MARGIN = 30.0


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
        import cv2
        import numpy as np
        import requests

        url = CAMERA_URL_TMPL.format(ip=self.robot_ip)
        try:
            resp = requests.get(url, timeout=REQUEST_TIMEOUT_S)
            resp.raise_for_status()
        except Exception as e:
            self.setText("Camera unavailable:\n%s" % e)
            return

        image = cv2.imdecode(np.frombuffer(resp.content, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.setText("Camera: received unreadable image data")
            return

        self._draw_apriltag_boxes(image)
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            self.setText("Camera: could not encode image data")
            return

        pixmap = QPixmap()
        if not pixmap.loadFromData(encoded.tobytes(), "PNG"):
            self.setText("Camera: could not display image data")
            return
        scaled = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(scaled)

    @staticmethod
    def _draw_apriltag_boxes(image):
        """Draw boxes using the same detector and filters as UR_12idb."""
        import os
        import sys
        import cv2
        import numpy as np

        ur12idb_path = os.environ.get("PALMIXER_UR12IDB_PATH")
        if ur12idb_path and ur12idb_path not in sys.path:
            sys.path.insert(0, ur12idb_path)
        try:
            from common.urcamera import AT_size, camera_f, detect_AT
            height, width = image.shape[:2]
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            detections = detect_AT(
                gray,
                [camera_f, camera_f, width / 2.0, height / 2.0],
                tag_size=AT_size,
            )
        except (ImportError, ModuleNotFoundError):
            return

        for detection in detections:
            if detection.hamming != 0 or detection.decision_margin < AT_MIN_MARGIN:
                continue
            corners = np.asarray(detection.corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=3)
