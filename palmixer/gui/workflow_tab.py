# -*- coding: utf-8 -*-
"""Automation tab: composite make_sample / unload_sample workflows, plus the
tracking panel and carousel-slot teaching that back them.

Like the other tabs, every button here just sends a command over the shared
ZMQ client. The tracking panel does not read any state itself -- app.py
pushes it a state.snapshot()-shaped dict (from get_state polling or the
tracking MQTT topic) via update_state().
"""

from PyQt5.QtWidgets import (
    QButtonGroup, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QRadioButton, QSpinBox, QVBoxLayout, QWidget,
)

from .. import commands as cmd


class WorkflowTab(QWidget):
    def __init__(self, send_command, parent=None):
        super().__init__(parent)
        self._send_command = send_command
        self._all_buttons = []
        self._tracking_labels = {}

        layout = QVBoxLayout()
        layout.addWidget(self._build_flowcell_group())
        layout.addWidget(self._build_carousel_group())
        layout.addWidget(self._build_workflow_group())
        layout.addWidget(self._build_tracking_group())
        layout.addStretch(1)
        self.setLayout(layout)

    # -- flowcell in use --------------------------------------------------
    def _build_flowcell_group(self):
        box = QGroupBox("Flowcell in Use")
        layout = QHBoxLayout()
        self._flowcell_group = QButtonGroup(self)
        for fc in cmd.FLOWCELL_IDS:
            radio = QRadioButton("Flowcell %d" % fc)
            # `clicked` fires only on real user interaction, unlike `toggled`,
            # which also fires for the default setChecked() below and for the
            # programmatic sync in update_state() -- either of which would
            # otherwise send an unsolicited set_flowcell to the server.
            radio.clicked.connect(lambda _checked, n=fc: self._on_set_flowcell(n))
            self._flowcell_group.addButton(radio, fc)
            layout.addWidget(radio)
            if fc == cmd.FLOWCELL_IDS[0]:
                radio.setChecked(True)
        layout.addStretch(1)
        box.setLayout(layout)
        return box

    def _on_set_flowcell(self, flowcell_id):
        self._send_command(cmd.set_flowcell_command(flowcell_id))

    # -- carousel -----------------------------------------------------------
    def _build_carousel_group(self):
        box = QGroupBox("Carousel")
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Slot:"))
        self.slot_box = QSpinBox()
        self.slot_box.setRange(1, 99)
        layout.addWidget(self.slot_box)

        teach_btn = QPushButton("Teach Current Position as This Slot")
        teach_btn.clicked.connect(self._on_teach_slot)
        layout.addWidget(teach_btn)
        self._all_buttons.append(teach_btn)

        layout.addStretch(1)
        box.setLayout(layout)
        return box

    def _on_teach_slot(self):
        self._send_command(cmd.teach_carousel_slot_command(self.slot_box.value()))

    # -- workflows ------------------------------------------------------------
    def _build_workflow_group(self):
        box = QGroupBox("Automation")
        layout = QHBoxLayout()

        make_btn = QPushButton("Make a Sample")
        make_btn.clicked.connect(self._on_make_sample)
        layout.addWidget(make_btn)
        self._all_buttons.append(make_btn)

        unload_btn = QPushButton("Unload Sample")
        unload_btn.clicked.connect(lambda: self._send_command(cmd.unload_sample_command()))
        layout.addWidget(unload_btn)
        self._all_buttons.append(unload_btn)

        box.setLayout(layout)
        return box

    def _on_make_sample(self):
        self._send_command(cmd.make_sample_command(self.slot_box.value()))

    # -- tracking panel ---------------------------------------------------
    def _build_tracking_group(self):
        box = QGroupBox("Tracking")
        grid = QGridLayout()

        rows = [
            (cmd.WHAT_MIXER_HEAD, "Mixer Head", cmd.MIXER_LOCATIONS),
            (cmd.WHAT_FLOWCELL_1, "Flowcell 1", cmd.FC_LOCATIONS),
            (cmd.WHAT_FLOWCELL_2, "Flowcell 2", cmd.FC_LOCATIONS),
        ]
        for i, (what, title, allowed) in enumerate(rows):
            grid.addWidget(QLabel(title + ":"), i, 0)
            value_label = QLabel(cmd.UNKNOWN)
            self._tracking_labels[what] = value_label
            grid.addWidget(value_label, i, 1)

            combo = QComboBox()
            combo.addItems(list(allowed))
            reconcile_btn = QPushButton("Set")
            reconcile_btn.clicked.connect(
                lambda _checked, w=what, c=combo: self._on_reconcile(w, c.currentText()))
            grid.addWidget(combo, i, 2)
            grid.addWidget(reconcile_btn, i, 3)
            self._all_buttons.append(reconcile_btn)

        carousel_row = len(rows)
        grid.addWidget(QLabel("Carousel Slot:"), carousel_row, 0)
        self._carousel_label = QLabel(cmd.UNKNOWN)
        grid.addWidget(self._carousel_label, carousel_row, 1)

        in_use_row = carousel_row + 1
        grid.addWidget(QLabel("Flowcell in Use:"), in_use_row, 0)
        self._flowcell_in_use_label = QLabel(str(cmd.FLOWCELL_IDS[0]))
        grid.addWidget(self._flowcell_in_use_label, in_use_row, 1)

        box.setLayout(grid)
        return box

    def _on_reconcile(self, what, value):
        self._send_command(cmd.set_location_command(what, value))

    # -- external updates (from get_state polling / tracking MQTT) ----------
    def update_state(self, snapshot):
        """Refresh the tracking panel from a state.snapshot()-shaped dict."""
        for what, label in self._tracking_labels.items():
            value = str(snapshot.get(what, cmd.UNKNOWN))
            label.setText(value)
            label.setStyleSheet("color: red; font-weight: bold;" if value == cmd.UNKNOWN else "")

        self._carousel_label.setText(str(snapshot.get("carousel_slot", cmd.UNKNOWN)))

        fc_in_use = snapshot.get("flowcell_in_use")
        if fc_in_use is not None:
            self._flowcell_in_use_label.setText(str(fc_in_use))
            btn = self._flowcell_group.button(int(fc_in_use))
            if btn is not None and not btn.isChecked():
                btn.setChecked(True)  # `clicked` does not fire for this

    @property
    def busy_widgets(self):
        """Widgets to disable while the server reports BUSY."""
        return list(self._all_buttons)
