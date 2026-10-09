# -*- coding: utf-8 -*-
"""Experiment tab: transport buttons, EPICS motor tweak, and pump buttons.

All buttons send their command over the same ZMQ REQ/REP client used by the
Configuration tab -- this tab has no direct hardware access of its own.
"""

from PyQt5.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QMessageBox, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from .. import commands as cmd
from .flowcell_selector import FlowcellSelector
from .pump_status import PumpStatusPanel


class ExperimentTab(QWidget):
    def __init__(self, send_command, parent=None):
        super().__init__(parent)
        self._send_command = send_command
        self._all_buttons = []

        self.flowcell = FlowcellSelector(send_command)
        self.pump_status = PumpStatusPanel()

        # Whether the sample-table transports drive the DAQ sample-stage motors
        # (sth/sav/stv) to the recorded position. Checked = the long-standing
        # behaviour; unchecked leaves those EPICS motors untouched, so a beam
        # transfer runs even when the DAQ GUI is unreachable. A persisted server
        # setting, so update_state() drives the box from the snapshot; not a
        # busy_widget -- it is a fast setting, answerable any time, and it only
        # affects the *next* transport's decision.
        self.position_motors_check = QCheckBox("Position sample_table motors (sth/sav/stv)")
        self.position_motors_check.setToolTip(
            "When checked, moving the flowcell onto or off the sample table first "
            "drives the DAQ sample-stage motors to the position recorded when the "
            "sample_table was taught, and restores them afterwards.\n\n"
            "Uncheck to leave sth/sav/stv untouched -- the transport then runs "
            "without touching those EPICS motors at all (useful when the DAQ GUI "
            "is unreachable or you are positioning the stage by hand).")
        self.position_motors_check.toggled.connect(self._on_toggle_position_motors)

        layout = QVBoxLayout()
        layout.addWidget(self.flowcell)
        layout.addWidget(self.position_motors_check)
        layout.addWidget(self._build_transport_group())
        layout.addWidget(self._build_carousel_group())
        layout.addWidget(self._build_robot_group())
        layout.addWidget(self._build_motor_group())
        layout.addWidget(self._build_pump_group())
        layout.addWidget(self.pump_status)
        layout.addStretch(1)
        self.setLayout(layout)

    # -- transport ------------------------------------------------------------
    def _build_transport_group(self):
        box = QGroupBox("Transport")
        grid = QGridLayout()
        ncols = 2
        for i, name in enumerate(cmd.TRANSPORT_FUNCTIONS):
            btn = QPushButton(cmd.TRANSPORT_LABELS[name])
            btn.clicked.connect(lambda _checked, n=name: self._send_command(n))
            grid.addWidget(btn, i // ncols, i % ncols)
            self._all_buttons.append(btn)
        box.setLayout(grid)
        return box

    # -- carousel swap ----------------------------------------------------------
    def _build_carousel_group(self):
        """Swap the carousel on the mixer: unload the spent one onto the used
        stack, or place a fresh one from stock. Both drive the arm across the
        cell, so each asks first and is disabled while the server is BUSY."""
        box = QGroupBox("Carousel")
        layout = QHBoxLayout()

        self.unload_carousel_btn = QPushButton("Unload Current Carousel")
        self.unload_carousel_btn.setToolTip(
            "Take the carousel off the mixer and set it on the used-carousel "
            "stack.\n\nNeeds Carousel on Mixer and Used Carousel Stack taught. "
            "The arm looks for the carousel first and stops with an error if "
            "none is there.")
        self.unload_carousel_btn.clicked.connect(self._on_unload_carousel)
        layout.addWidget(self.unload_carousel_btn)

        self.place_carousel_btn = QPushButton("Place New Carousel")
        self.place_carousel_btn.setToolTip(
            "Bring a fresh carousel from the stock position onto the mixer.\n\n"
            "Needs Carousel in Stock and Carousel on Mixer taught. The arm "
            "looks for a carousel in stock first and stops with an error if "
            "none is there.")
        self.place_carousel_btn.clicked.connect(self._on_place_new_carousel)
        layout.addWidget(self.place_carousel_btn)

        layout.addStretch(1)
        self._all_buttons.extend([self.unload_carousel_btn, self.place_carousel_btn])
        box.setLayout(layout)
        return box

    def _on_unload_carousel(self):
        if QMessageBox.question(
                self, "Unload Current Carousel",
                "Take the carousel off the mixer and set it on the used stack?\n\n"
                "The arm will move across the cell.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        self._send_command(cmd.unload_carousel_command())

    def _on_place_new_carousel(self):
        if QMessageBox.question(
                self, "Place New Carousel",
                "Bring a fresh carousel from stock onto the mixer?\n\n"
                "The arm will move across the cell.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        self._send_command(cmd.place_new_carousel_command())

    # -- robot ------------------------------------------------------------------
    def _build_robot_group(self):
        box = QGroupBox("Robot")
        layout = QHBoxLayout()

        self.zalign_btn = QPushButton("Z Align")
        self.zalign_btn.setToolTip(
            "Point the tool Z axis straight down, keeping the current position "
            "and heading. Use it when the wrist has been left tilted -- by an "
            "aborted AprilTag search, or a jog from the pendant.")
        self.zalign_btn.clicked.connect(lambda: self._send_command(cmd.zalign_command()))
        layout.addWidget(self.zalign_btn)

        # Swap which tool frame stands on the current pose. Both translate the
        # arm by the camera offset without reorienting it, so Camera to TCP
        # also leaves the camera face-down whenever the tool was -- which is
        # what the Configuration tab's "Start from the current position"
        # search needs before it will run without reorienting anything.
        self.camera_to_tcp_btn = QPushButton("Camera to TCP")
        self.camera_to_tcp_btn.setToolTip(
            "Move the camera onto the pose the gripper tip is on now.\n\n"
            "The arm shifts by the camera offset; nothing rotates. With the "
            "tool face-down this leaves the camera face-down too, which is "
            "how you set it up for a \"Start from the current position\" "
            "AprilTag search.")
        self.camera_to_tcp_btn.clicked.connect(
            lambda: self._send_command(cmd.put_camera_to_tcp_command()))
        layout.addWidget(self.camera_to_tcp_btn)
        self._all_buttons.append(self.camera_to_tcp_btn)

        self.tcp_to_camera_btn = QPushButton("TCP to Camera")
        self.tcp_to_camera_btn.setToolTip(
            "Move the gripper tip onto the pose the camera is on now -- the "
            "way back from Camera to TCP.\n\n"
            "Having lined the camera up on something, this brings the tool to "
            "that same spot.")
        self.tcp_to_camera_btn.clicked.connect(
            lambda: self._send_command(cmd.put_tcp_to_camera_command()))
        layout.addWidget(self.tcp_to_camera_btn)
        self._all_buttons.append(self.tcp_to_camera_btn)

        # A checkable toggle, not a one-shot: checked pushes the arm into
        # freedrive, unchecked takes it out. Deliberately left out of
        # _all_buttons so it stays live while the server is BUSY -- the same
        # reasoning as the Unlock button below. Turning freedrive *off* is a
        # recovery that must always work; the server refuses turning it *on*
        # while busy on its own. `clicked(bool)` fires only on operator clicks
        # (not on any programmatic setChecked), carrying the new checked state.
        self.freedrive_btn = QPushButton("Free Drive")
        self.freedrive_btn.setCheckable(True)
        self.freedrive_btn.setToolTip(
            "Release the arm's brakes so you can push it to a pose by hand, "
            "then press again to stiffen it.\n\n"
            "Refused while the robot is busy. The controller re-stiffens the "
            "arm on its own after a while, so if this is left on the robot "
            "recovers by itself -- press again to sync the button.")
        self.freedrive_btn.clicked.connect(
            lambda checked: self._send_command(cmd.freedrive_command(checked)))
        layout.addWidget(self.freedrive_btn)

        self.release_btn = QPushButton("Release Gripper")
        self.release_btn.setToolTip(
            "Open the gripper where the arm is standing.\n\n"
            "If it is holding the flowcell, that drops it -- so this asks "
            "first, and afterwards the flowcell's tracked location is set to "
            "unknown for you to reconcile.")
        self.release_btn.clicked.connect(self._on_release_gripper)
        layout.addWidget(self.release_btn)
        self._all_buttons.append(self.release_btn)

        # Not in _all_buttons, so it stays live while the server is BUSY -- a
        # protective stop happens during a move, and the action it interrupted
        # may still be holding the server busy. Same reasoning as the
        # Configuration tab's Stop Search and the Automation tab's Stop Shake.
        self.unlock_btn = QPushButton("Unlock Protective Stop")
        self.unlock_btn.setStyleSheet("color: darkred; font-weight: bold;")
        self.unlock_btn.setToolTip(
            "Clear the robot's protective stop, so it will accept moves again.\n\n"
            "Works while the server is BUSY -- that is when a protective stop\n"
            "happens. Clear whatever caused it first; the reply says whether\n"
            "the stop actually went away.")
        self.unlock_btn.clicked.connect(
            lambda: self._send_command(cmd.unlock_stop_command()))
        layout.addWidget(self.unlock_btn)

        layout.addStretch(1)

        self._all_buttons.append(self.zalign_btn)
        box.setLayout(layout)
        return box

    def _on_release_gripper(self):
        # Opening the jaws is instant and unrecoverable if something is in
        # them, and the button gives no clue whether anything is -- so ask.
        if QMessageBox.question(
                self, "Release Gripper",
                "Open the gripper now?\n\n"
                "If the robot is holding the flowcell, this drops it from "
                "wherever the arm is standing.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        self._send_command(cmd.release_gripper_command())

    # -- motor ------------------------------------------------------------------
    def _build_motor_group(self):
        box = QGroupBox("Carousel Motor (Galil ch A)")
        layout = QHBoxLayout()

        layout.addWidget(QLabel("Step:"))
        self.step_box = QDoubleSpinBox()
        self.step_box.setDecimals(4)
        self.step_box.setRange(0.0001, 100000.0)
        self.step_box.setValue(1.0)
        layout.addWidget(self.step_box)

        self.reverse_btn = QPushButton("<< Reverse")
        self.reverse_btn.clicked.connect(lambda: self._on_motor_tweak(cmd.MOTOR_REVERSE))
        layout.addWidget(self.reverse_btn)

        self.forward_btn = QPushButton("Forward >>")
        self.forward_btn.clicked.connect(lambda: self._on_motor_tweak(cmd.MOTOR_FORWARD))
        layout.addWidget(self.forward_btn)

        # Teach a slot from right here, next to the tweak buttons that got the
        # motor onto it -- jog to the slot, then record it, without changing
        # tabs. Same teach_carousel_slot command the Automation tab sends: one
        # taught slot plus the configured step locates every other slot.
        layout.addSpacing(12)
        layout.addWidget(QLabel("Slot:"))
        self.slot_box = QSpinBox()
        # Opens at 1..1 and widens once a get_state snapshot reports the
        # configured carousel size, so the spinbox never offers a slot number
        # the server would reject (see update_state).
        self.slot_box.setRange(1, 1)
        self.slot_box.setEnabled(False)
        layout.addWidget(self.slot_box)

        self.teach_slot_btn = QPushButton("Set Current Position")
        self.teach_slot_btn.setToolTip(
            "Record the motor's current position as this carousel slot.")
        self.teach_slot_btn.setEnabled(False)
        self.teach_slot_btn.clicked.connect(self._on_teach_slot)
        layout.addWidget(self.teach_slot_btn)

        self._all_buttons.extend([self.reverse_btn, self.forward_btn])
        box.setLayout(layout)
        return box

    def _on_motor_tweak(self, direction):
        step = self.step_box.value()
        self._send_command(cmd.motor_tweak_command(direction, step))

    def _on_teach_slot(self):
        self._send_command(cmd.teach_carousel_slot_command(self.slot_box.value()))

    def _on_toggle_position_motors(self, checked):
        self._send_command(cmd.set_position_sample_table_motors_command(checked))

    def update_state(self, snapshot):
        """Track the configured carousel size from a state.snapshot()-shaped dict.

        Until the size is known nothing slot-shaped means anything, so the slot
        controls stay disabled rather than offering a range the server would
        reject. Deliberately not added to busy_widgets: teach_carousel_slot is
        a fast bookkeeping command answered synchronously, not an action, so it
        is enabled and disabled by this alone.
        """
        self.pump_status.update_state(snapshot)
        # Reflect the server's persisted setting without re-sending it: a
        # programmatic setChecked would otherwise fire _on_toggle_position_motors
        # and bounce the command straight back on every 3 s poll.
        want = snapshot.get("position_sample_table_motors")
        if isinstance(want, bool) and want != self.position_motors_check.isChecked():
            self.position_motors_check.blockSignals(True)
            self.position_motors_check.setChecked(want)
            self.position_motors_check.blockSignals(False)
        size = snapshot.get("carousel_size", cmd.UNKNOWN)
        known = isinstance(size, int) and not isinstance(size, bool) and size >= 1
        self.slot_box.setEnabled(known)
        self.teach_slot_btn.setEnabled(known)
        if known and self.slot_box.maximum() != size:
            self.slot_box.setRange(1, size)

    # -- pump ------------------------------------------------------------------
    def _build_pump_group(self):
        box = QGroupBox("Pump")
        layout = QHBoxLayout()
        for op in cmd.EXPERIMENT_PUMP_OPS:
            btn = QPushButton(cmd.PUMP_LABELS[op])
            btn.clicked.connect(lambda _checked, o=op: self._send_command(cmd.pump_command(o)))
            layout.addWidget(btn)
            self._all_buttons.append(btn)
        box.setLayout(layout)
        return box

    @property
    def busy_widgets(self):
        """Widgets to disable while the server reports BUSY."""
        return list(self._all_buttons) + self.flowcell.busy_widgets
