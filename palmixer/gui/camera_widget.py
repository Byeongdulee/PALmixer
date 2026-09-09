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

        # Resolved on the first frame and cached: () once the import has been
        # tried and failed, so a broken path costs one message rather than one
        # per frame at 5 fps.
        self._detector = None
        self._detector_error = None
        self._tags_seen = 0

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
        if self._detector_error:
            # On the image rather than in a log nobody is watching: a feed that
            # never boxes a tag is indistinguishable from one pointed at
            # nothing, which is what made this hard to spot.
            cv2.putText(image, "no AprilTag overlay -- see console", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
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

    def _load_detector(self):
        """Import UR_12idb's AprilTag detector once, and remember what happened.

        The path comes from config, which is where the rest of the app gets it
        (config.py merges PALMIXER_UR12IDB_PATH into robot.ur12idb_path, so the
        environment variable still works). Reading only the environment
        variable was the bug behind "no tag on the camera view": a normal
        launch configures the path in json/palmixer_config.json, the import
        then failed, and the except swallowed it -- leaving a live feed that
        simply never drew a box, with nothing said about why.
        """
        if self._detector is not None:
            return self._detector
        import sys

        from .. import config

        path = config.get_section("robot").get("ur12idb_path")
        if path and path not in sys.path:
            sys.path.insert(0, path)
        try:
            from common.urcamera import AT_size, camera_f, detect_AT
        except Exception as e:                  # ImportError, or cv2/apriltag missing
            self._detector_error = (
                "AprilTag overlay unavailable: cannot import UR_12idb from %r (%s)"
                % (path, e))
            print(self._detector_error)
            self._detector = ()                 # falsy, and not None: do not retry
            return self._detector
        self._detector = (detect_AT, camera_f, AT_size)
        return self._detector

    def _draw_apriltag_boxes(self, image):
        """Draw boxes using the same detector and filters as UR_12idb."""
        import cv2
        import numpy as np

        loaded = self._load_detector()
        if not loaded:
            return
        detect_AT, camera_f, AT_size = loaded
        height, width = image.shape[:2]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        try:
            detections = detect_AT(
                gray,
                [camera_f, camera_f, width / 2.0, height / 2.0],
                tag_size=AT_size,
            )
        except Exception as e:
            # One bad frame must not kill the feed, but say so rather than
            # looking like "no tag in view".
            print("AprilTag detection failed on this frame: %s" % e)
            return

        self._tags_seen = 0
        for detection in detections:
            if detection.hamming != 0 or detection.decision_margin < AT_MIN_MARGIN:
                continue
            self._tags_seen += 1
            corners = np.asarray(detection.corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=3)
            cX, cY = int(detection.center[0]), int(detection.center[1])
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            cv2.putText(image, "id %d" % detection.tag_id, (corners[0][0][0], corners[0][0][1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
