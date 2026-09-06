# -*- coding: utf-8 -*-
"""Experiment tab: transport buttons, EPICS motor tweak, and pump buttons.

All buttons send their command over the same ZMQ REQ/REP client used by the
Configuration tab -- this tab has no direct hardware access of its own.
"""

from PyQt5.QtWidgets import (
    QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QVBoxLayout, QWidget,
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

        self._all_buttons.extend([self.reverse_btn, self.forward_btn])
        box.setLayout(layout)
        return box

    def _on_motor_tweak(self, direction):
        step = self.step_box.value()
        self._send_command(cmd.motor_tweak_command(direction, step))

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
