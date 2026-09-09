# -*- coding: utf-8 -*-
"""Experiment tab: transport buttons, EPICS motor tweak, and pump buttons.

All buttons send their command over the same ZMQ REQ/REP client used by the
Configuration tab -- this tab has no direct hardware access of its own.
"""

from PyQt5.QtWidgets import (
    QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from .. import commands as cmd
from .flowcell_selector import FlowcellSelector


class ExperimentTab(QWidget):
    def __init__(self, send_command, parent=None):
        super().__init__(parent)
        self._send_command = send_command
        self._all_buttons = []

        self.flowcell = FlowcellSelector(send_command)

        layout = QVBoxLayout()
        layout.addWidget(self.flowcell)
        layout.addWidget(self._build_transport_group())
        layout.addWidget(self._build_robot_group())
        layout.addWidget(self._build_motor_group())
        layout.addWidget(self._build_pump_group())
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
        layout.addStretch(1)

        self._all_buttons.append(self.zalign_btn)
        box.setLayout(layout)
        return box

    # -- motor ------------------------------------------------------------------
    def _build_motor_group(self):
        box = QGroupBox("EPICS Motor (12idb:m6)")
        layout = QHBoxLayout()

        layout.addWidget(QLabel("Step:"))
        self.step_box = QDoubleSpinBox()
        self.step_box.setDecimals(4)
        self.step_box.setRange(0.0001, 1000.0)
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

    def update_state(self, snapshot):
        """Track the configured carousel size from a state.snapshot()-shaped dict.

        Until the size is known nothing slot-shaped means anything, so the slot
        controls stay disabled rather than offering a range the server would
        reject. Deliberately not added to busy_widgets: teach_carousel_slot is
        a fast bookkeeping command answered synchronously, not an action, so it
        is enabled and disabled by this alone.
        """
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
        for op in cmd.PUMP_OPS:
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
