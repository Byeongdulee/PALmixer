# -*- coding: utf-8 -*-
"""PALmixer GUI: a thin ZMQ client with an MQTT-fed status log.

- Every button on the Configuration/Experiment tabs sends a command string
  over ZMQ (ZMQClient.send) to the headless PALmixer server (see server.py).
  ZMQ replies only confirm the request was accepted/rejected; they are shown
  as an immediate one-line log entry.
- The server reports the actual outcome of each action asynchronously over
  MQTT (see mqtt_status.py); this window subscribes to
  ``aps12/<beamline>/palmixer/#`` and renders those as the running status
  log + an IDLE/BUSY badge, which also disables the action buttons while busy.

Both the ZMQ reply and the MQTT callback arrive on background threads; both
are marshaled onto the Qt main thread via signals before touching any widget.
"""

import json
import sys
import threading

from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication, QGroupBox, QHBoxLayout, QLabel, QListWidget,
    QListWidgetItem, QMainWindow, QMessageBox, QTabWidget, QVBoxLayout, QWidget,
)

from .. import commands as cmd
from .. import config
from ..mqtt_status import (MQTTSubscriber, PHASE_FAILURE, motion_topic,
                           state_topic, tracking_topic)
from ..zmq_transport import ZMQClient
from .config_tab import ConfigTab
from .experiment_tab import ExperimentTab
from .workflow_tab import WorkflowTab

STATUS_POLL_INTERVAL_MS = 3000

PHASE_COLORS = {
    "started": QColor("gray"),
    "success": QColor("darkgreen"),
    "failure": QColor("darkred"),
}


class MainWindow(QMainWindow):
    _zmq_reply = pyqtSignal(str, str)       # (command, reply)
    _status_polled = pyqtSignal(str)        # "IDLE" / "BUSY" / "" (poll failed)
    _mqtt_state = pyqtSignal(dict)          # decoded state payload
    _mqtt_motion = pyqtSignal(dict)         # decoded motion payload
    _tracking_updated = pyqtSignal(dict)    # state.snapshot() dict, from poll or MQTT

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PALmixer Control")
        self.resize(900, 700)

        cfg = config.get_config()
        self._beamline = cfg["mqtt"].get("beamline", "12idb")
        self.client = ZMQClient(host=cfg["zmq"].get("host", "localhost"),
                                 port=cfg["zmq"].get("port", 9880))

        self._build_ui(cfg["robot"].get("ip", ""))
        self._wire_signals()
        self._start_mqtt(cfg)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._poll_status_async)
        self._status_timer.start(STATUS_POLL_INTERVAL_MS)
        self._poll_status_async()

    # -- UI ------------------------------------------------------------------
    def _build_ui(self, robot_ip):
        central = QWidget()
        layout = QVBoxLayout()

        # Built before the tabs (though added to the layout after them, so it
        # still renders underneath): it creates self.log_list, which
        # send_command() writes to. A tab that sends a command while being
        # constructed would otherwise raise inside a Qt slot, which PyQt turns
        # into a hard abort rather than a catchable exception.
        status_panel = self._build_status_panel()

        self.tabs = QTabWidget()
        self.config_tab = ConfigTab(robot_ip, self.send_command)
        self.experiment_tab = ExperimentTab(self.send_command)
        self.workflow_tab = WorkflowTab(self.send_command)
        self.tabs.addTab(self.config_tab, "Configuration")
        self.tabs.addTab(self.experiment_tab, "Experiment")
        self.tabs.addTab(self.workflow_tab, "Automation")
        layout.addWidget(self.tabs, stretch=1)

        layout.addWidget(status_panel)

        central.setLayout(layout)
        self.setCentralWidget(central)

        self._busy_widgets = (self.config_tab.busy_widgets + self.experiment_tab.busy_widgets
                               + self.workflow_tab.busy_widgets)

    def _build_status_panel(self):
        box = QGroupBox("Status")
        layout = QVBoxLayout()

        header = QHBoxLayout()
        header.addWidget(QLabel("Server state:"))
        self.state_badge = QLabel("UNKNOWN")
        self.state_badge.setStyleSheet("font-weight: bold;")
        header.addWidget(self.state_badge)
        header.addStretch(1)
        layout.addLayout(header)

        # How the last action ended, in colour and in words. Outcomes used to
        # be one line in the log below, which scrolls away and is easy to
        # miss -- an AprilTag search that gave up looked the same as one still
        # running. This stays put until the next command is sent.
        self.result_label = QLabel("")
        self.result_label.setWordWrap(True)
        self.result_label.hide()
        self._last_result_stamp = None
        layout.addWidget(self.result_label)

        self.log_list = QListWidget()
        self.log_list.setMaximumHeight(200)
        layout.addWidget(self.log_list)

        box.setLayout(layout)
        return box

    def _wire_signals(self):
        self._zmq_reply.connect(self._on_zmq_reply)
        self._status_polled.connect(self._on_status_polled)
        self._mqtt_state.connect(self._on_mqtt_state)
        self._mqtt_motion.connect(self._on_mqtt_motion)
        self._tracking_updated.connect(self._on_tracking_updated)

    # -- ZMQ (command sending) ------------------------------------------------
    def send_command(self, command):
        self._clear_result()    # the banner describes the previous command
        self._log("-> %s" % command, QColor("black"))
        threading.Thread(target=self._send_worker, args=(command,), daemon=True).start()

    def _send_worker(self, command):
        try:
            reply = self.client.send(command, timeout_ms=5000)
        except Exception as e:
            reply = "ERROR: %s" % e
        self._zmq_reply.emit(command, reply)

    def _on_zmq_reply(self, command, reply):
        color = QColor("darkred") if reply.startswith("ERROR") else QColor("gray")
        self._log("<- %s: %s" % (command, reply), color)
        if reply.startswith("ERROR"):
            # A command the server refused outright. Async actions that fail
            # after being ACCEPTED come back through _on_mqtt_motion instead.
            self._show_error("%s -- %s" % (command, reply[len("ERROR:"):].strip()))
        # This ERROR text only comes from the Experiment/Automation tabs'
        # transport, make_sample, and unload_sample commands (see server.py's
        # _require_positions) -- Configuration-tab commands are how a
        # position gets configured in the first place, so they never trigger
        # it. A dialog is worth the interruption here: the alternative is a
        # log line that is easy to miss before the operator walks away for a
        # multi-minute sequence that would otherwise never have started.
        if reply.startswith("ERROR: %s" % cmd.POSITION_NOT_CONFIGURED):
            QMessageBox.warning(self, "Position Not Configured", reply[len("ERROR: "):])

    def _poll_status_async(self):
        threading.Thread(target=self._poll_status_worker, daemon=True).start()

    def _poll_status_worker(self):
        self._status_polled.emit(self.client.status())
        try:
            reply = self.client.send(cmd.get_state_command(), timeout_ms=5000)
            snapshot = json.loads(reply)
        except Exception:
            return
        self._tracking_updated.emit(snapshot)

    def _on_status_polled(self, state):
        if state:
            self._set_busy(state.strip().upper() == "BUSY")

    def _on_tracking_updated(self, snapshot):
        self.workflow_tab.update_state(snapshot)
        self.experiment_tab.update_state(snapshot)
        # How the last action ended, carried on the ordinary 3 s get_state
        # poll. This is the path that works with no MQTT broker -- an async
        # failure otherwise reaches the GUI only over the motion topic, which
        # is silent when telemetry is disabled. Keyed on the timestamp so the
        # same failure is not re-shown on every poll, which would stop the
        # operator from dismissing it by sending the next command.
        last_result = snapshot.get("last_result")
        if isinstance(last_result, dict):
            stamp = last_result.get("time")
            if stamp != self._last_result_stamp:
                self._last_result_stamp = stamp
                detail = last_result.get("detail", "")
                self._show_result(bool(last_result.get("ok")),
                                  "%s%s" % (last_result.get("action", "?"),
                                            (" -- %s" % detail) if detail else ""))
        fc_in_use = snapshot.get("flowcell_in_use")
        if fc_in_use is not None:
            self.config_tab.flowcell.set_flowcell_in_use(fc_in_use)
            self.experiment_tab.flowcell.set_flowcell_in_use(fc_in_use)

    # -- MQTT (status feed) ----------------------------------------------------
    def _start_mqtt(self, cfg):
        self.mqtt_sub = MQTTSubscriber(host=cfg["mqtt"].get("host"),
                                        port=cfg["mqtt"].get("port"),
                                        client_id_prefix="palmixer-gui")
        self.mqtt_sub.subscribe(state_topic(self._beamline), self._on_mqtt_state_raw, qos=0)
        self.mqtt_sub.subscribe(motion_topic(self._beamline), self._on_mqtt_motion_raw, qos=1)
        self.mqtt_sub.subscribe(tracking_topic(self._beamline), self._on_mqtt_tracking_raw, qos=0)
        self.mqtt_sub.start()

    # These run on paho's background thread -- only emit signals here, no widgets.
    def _on_mqtt_state_raw(self, topic, payload):
        if isinstance(payload, dict):
            self._mqtt_state.emit(payload)

    def _on_mqtt_motion_raw(self, topic, payload):
        if isinstance(payload, dict):
            self._mqtt_motion.emit(payload)

    def _on_mqtt_tracking_raw(self, topic, payload):
        if isinstance(payload, dict):
            self._tracking_updated.emit(payload)

    def _on_mqtt_state(self, payload):
        self._set_busy(str(payload.get("state", "")).upper() == "BUSY")

    def _on_mqtt_motion(self, payload):
        action = payload.get("action", "?")
        phase = payload.get("phase", "?")
        detail = payload.get("detail", "")
        color = PHASE_COLORS.get(phase, QColor("black"))
        text = "[%s] %s" % (phase.upper(), action)
        if detail:
            text += " -- %s" % detail
        self._log(text, color)
        # An action that failed after it was ACCEPTED -- an AprilTag search
        # that could not square up to the tag, a transport that raised. The
        # ZMQ reply for these was "ACCEPTED" long ago, so this topic is the
        # only place the outcome arrives.
        if phase == PHASE_FAILURE:
            self._show_error("%s failed%s" % (action, (" -- %s" % detail) if detail else ""))

    # -- shared helpers ----------------------------------------------------------
    def _set_busy(self, busy):
        self.state_badge.setText("BUSY" if busy else "IDLE")
        self.state_badge.setStyleSheet(
            "font-weight: bold; color: %s;" % ("darkred" if busy else "darkgreen"))
        for w in self._busy_widgets:
            w.setEnabled(not busy)

    def _show_result(self, ok, text):
        self.result_label.setText(("OK: " if ok else "FAILED: ") + text)
        self.result_label.setStyleSheet(
            "color: white; background-color: %s; font-weight: bold; padding: 3px;"
            % ("darkgreen" if ok else "darkred"))
        self.result_label.show()

    def _show_error(self, text):
        self._show_result(False, text)

    def _clear_result(self):
        self.result_label.clear()
        self.result_label.hide()

    def _log(self, text, color=None):
        item = QListWidgetItem(text)
        if color is not None:
            item.setForeground(color)
        self.log_list.addItem(item)
        self.log_list.scrollToBottom()

    def closeEvent(self, event):
        try:
            self.mqtt_sub.close()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
