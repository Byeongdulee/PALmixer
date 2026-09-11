# -*- coding: utf-8 -*-
"""Automation tab: composite make_sample / unload_sample workflows, plus the
tracking panel and carousel-slot teaching that back them.

Like the other tabs, every button here just sends a command over the shared
ZMQ client. The tracking panel does not read any state itself -- app.py
pushes it a state.snapshot()-shaped dict (from get_state polling or the
tracking MQTT topic) via update_state().
"""

from PyQt5.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox, QPushButton,
    QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from .. import commands as cmd
from .flowcell_selector import FlowcellSelector
from .pump_status import PumpStatusPanel


class WorkflowTab(QWidget):
    def __init__(self, send_command, parent=None):
        super().__init__(parent)
        self._send_command = send_command
        self._all_buttons = []
        self._tracking_labels = {}
        # Last snapshot's slot -> sample ID table, so switching slots can show
        # that slot's ID without a round trip to the server.
        self._samples = {}

        self.flowcell = FlowcellSelector(send_command)
        self.pump_status = PumpStatusPanel()

        layout = QVBoxLayout()
        layout.addWidget(self.flowcell)
        layout.addWidget(self._build_carousel_group())
        layout.addWidget(self._build_workflow_group())
        layout.addWidget(self.pump_status)
        layout.addWidget(self._build_tracking_group())
        layout.addStretch(1)
        self.setLayout(layout)

    # -- carousel -----------------------------------------------------------
    def _build_carousel_group(self):
        """The carousel's slot inventory: how big it is, what each slot holds,
        and the controls for teaching, tagging, and replacing it."""
        box = QGroupBox("Carousel")
        outer = QVBoxLayout()

        # Row 1: the geometry (read-only -- it is configuration, see config.py),
        # how much of the carousel is spent, and replacing it wholesale.
        size_row = QHBoxLayout()
        self._geometry_label = QLabel("")
        size_row.addWidget(self._geometry_label)
        self._used_label = QLabel("")
        size_row.addWidget(self._used_label)
        size_row.addStretch(1)

        reset_btn = QPushButton("Replace Carousel (Reset)")
        reset_btn.clicked.connect(self._on_reset_carousel)
        size_row.addWidget(reset_btn)
        self._all_buttons.append(reset_btn)
        outer.addLayout(size_row)

        # Row 2: the per-slot controls.
        slot_row = QHBoxLayout()
        slot_row.addWidget(QLabel("Slot:"))
        self.slot_box = QSpinBox()
        self.slot_box.setRange(1, 1)
        self.slot_box.valueChanged.connect(self._on_slot_changed)
        slot_row.addWidget(self.slot_box)

        slot_row.addWidget(QLabel("Sample ID:"))
        self.sample_id_edit = QLineEdit()
        self.sample_id_edit.setMaxLength(cmd.MAX_SAMPLE_ID_LEN)
        self.sample_id_edit.setPlaceholderText("(blank -> timestamp ID assigned on mix)")
        slot_row.addWidget(self.sample_id_edit, 1)

        for text, handler in (("Set ID", self._on_set_sample_id),
                              ("Clear ID", self._on_clear_sample_id),
                              ("Teach Position", self._on_teach_slot)):
            btn = QPushButton(text)
            btn.clicked.connect(handler)
            slot_row.addWidget(btn)
            self._all_buttons.append(btn)
        outer.addLayout(slot_row)

        # Row 3: what is actually in the carousel right now.
        self.slot_table = QTableWidget(0, 3)
        self.slot_table.setHorizontalHeaderLabels(["Slot", "Position", "Sample ID"])
        self.slot_table.verticalHeader().setVisible(False)
        self.slot_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.slot_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.slot_table.setMaximumHeight(180)
        header = self.slot_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        outer.addWidget(self.slot_table)

        box.setLayout(outer)
        return box

    def _on_teach_slot(self):
        self._send_command(cmd.teach_carousel_slot_command(self.slot_box.value()))

    def _on_set_sample_id(self):
        sample_id = self.sample_id_edit.text().strip()
        if not sample_id:
            QMessageBox.warning(self, "No Sample ID",
                                "Type a sample ID to tag slot %d with." % self.slot_box.value())
            return
        self._send_command(cmd.set_sample_id_command(self.slot_box.value(), sample_id))

    def _on_clear_sample_id(self):
        self._send_command(cmd.clear_sample_id_command(self.slot_box.value()))

    def _on_slot_changed(self, slot):
        """Show the selected slot's sample ID. Deliberately only on an explicit
        slot change -- doing it from update_state() would wipe a half-typed ID
        every time the 3 s get_state poll came back."""
        self.sample_id_edit.setText(self._samples.get(slot, ""))

    def _show_assigned_sample_id(self, previous):
        """Fill the ID box with an ID the server assigned, e.g. the timestamp
        make_sample mints when its mix finishes and the operator named nothing.

        Only on a change, and only into a box holding nothing or the value that
        just changed. update_state() cannot simply write the selected slot's ID
        every time -- the 3 s get_state poll would wipe a half-typed one
        between keystrokes (see _on_slot_changed) -- but leaving the box empty
        after a run means the ID the sample actually got is only visible in the
        slot table, and pressing "Set ID" next would then tag the slot with
        whatever stale text was there.
        """
        slot = self.slot_box.value()
        was, now = previous.get(slot, ""), self._samples.get(slot, "")
        if now != was and self.sample_id_edit.text().strip() in ("", was):
            self.sample_id_edit.setText(now)

    def _on_reset_carousel(self):
        # Destructive and not obviously so: it discards the taught positions
        # too, so say that before doing it rather than after.
        used = len(self._samples)
        if QMessageBox.question(
                self, "Replace Carousel",
                "Reset the carousel inventory?\n\n"
                "This clears all %d sample ID(s) AND the taught reference position, "
                "so one slot must be taught again before the next sample.\n\n"
                "Do this after physically swapping in a fresh carousel." % used,
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        self._send_command(cmd.reset_carousel_command())

    # -- workflows ------------------------------------------------------------
    def _build_workflow_group(self):
        box = QGroupBox("Automation")
        layout = QHBoxLayout()

        make_btn = QPushButton("Make a Sample")
        make_btn.clicked.connect(self._on_make_sample)
        layout.addWidget(make_btn)
        self._all_buttons.append(make_btn)

        draw_btn = QPushButton("Draw and Load")
        draw_btn.setToolTip(
            "Draw from the vial the mixer is already over and put the\n"
            "flowcell in the beam -- make_sample without the mixing.\n"
            "Consumes no carousel slot and assigns no sample ID.")
        draw_btn.clicked.connect(lambda: self._send_command(cmd.draw_and_load_command()))
        layout.addWidget(draw_btn)
        self._all_buttons.append(draw_btn)

        unload_btn = QPushButton("Unload Sample")
        unload_btn.clicked.connect(self._on_unload_sample)
        layout.addWidget(unload_btn)
        self._all_buttons.append(unload_btn)

        # Off by default: the usual run washes the sample away with the
        # flowcell. Ticking this spends three extra legs and a pump operation
        # to put the sample back in the vial it was mixed in first.
        self.aspirate_box = QCheckBox("Aspirate back to mixer")
        self.aspirate_box.setToolTip(
            "Recover the sample into its carousel vial before washing.\n"
            "Unticked, the flowcell goes straight to the cleaning station\n"
            "and the sample is discarded with the wash.")
        layout.addWidget(self.aspirate_box)
        self._all_buttons.append(self.aspirate_box)

        layout.addSpacing(12)
        shake_btn = QPushButton("Shake Sample")
        shake_btn.setToolTip(
            "Cycle the sample back and forth inside the flowcell in use --\n"
            "flow2_sample for flowcell 1, flow3_sample for flowcell 2.\n"
            "Volume and cycle count come from the flowcell dashboard.")
        shake_btn.clicked.connect(
            lambda: self._send_command(cmd.shake_sample_command()))
        layout.addWidget(shake_btn)
        self._all_buttons.append(shake_btn)

        # Deliberately NOT in _all_buttons, so it stays enabled while the
        # server is BUSY -- the same reason the Configuration tab keeps "Stop
        # Search" out of its busy list. A stop button greyed out for exactly
        # the window in which it is useful would be worse than no button.
        self.stop_shake_btn = QPushButton("Stop Shake")
        self.stop_shake_btn.setStyleSheet("color: darkred; font-weight: bold;")
        self.stop_shake_btn.setToolTip(
            "End the active Shake Sample cycle. Answered even while a pump\n"
            "operation is running -- that is what it is for.\n\n"
            "Only interrupts an active shake; an unrelated draw, wash, or\n"
            "aspirate keeps running.")
        self.stop_shake_btn.clicked.connect(
            lambda: self._send_command(cmd.stop_pump_command()))
        layout.addWidget(self.stop_shake_btn)

        box.setLayout(layout)
        return box

    def _on_unload_sample(self):
        self._send_command(
            cmd.unload_sample_command(self.aspirate_box.isChecked()))

    def _on_make_sample(self):
        # A blank ID is fine: the server assigns a timestamp one, so the slot
        # this consumes is recorded either way.
        self._send_command(cmd.make_sample_command(self.slot_box.value(),
                                                   self.sample_id_edit.text().strip()))

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
    @staticmethod
    def _by_slot(table):
        """Re-key a snapshot slot table to ints. The server keys these by int,
        but the snapshot reaches us through JSON, which only has string keys."""
        out = {}
        for key, value in (table or {}).items():
            try:
                out[int(key)] = value
            except (TypeError, ValueError):
                continue
        return out

    def update_state(self, snapshot):
        """Refresh the tracking panel from a state.snapshot()-shaped dict."""
        for what, label in self._tracking_labels.items():
            value = str(snapshot.get(what, cmd.UNKNOWN))
            label.setText(value)
            label.setStyleSheet("color: red; font-weight: bold;" if value == cmd.UNKNOWN else "")

        self._carousel_label.setText(str(snapshot.get("carousel_slot", cmd.UNKNOWN)))
        self._update_carousel(snapshot)
        self.pump_status.update_state(snapshot)

        fc_in_use = snapshot.get("flowcell_in_use")
        if fc_in_use is not None:
            self._flowcell_in_use_label.setText(str(fc_in_use))
            self.flowcell.set_flowcell_in_use(fc_in_use)

    def _update_carousel(self, snapshot):
        previous = self._samples
        self._samples = self._by_slot(snapshot.get("carousel_samples"))
        self._show_assigned_sample_id(previous)
        positions = self._by_slot(snapshot.get("carousel_slots"))
        size = snapshot.get("carousel_size", cmd.UNKNOWN)
        step = snapshot.get("carousel_step", cmd.UNKNOWN)
        known = isinstance(size, int) and size >= 1

        # Nothing slot-shaped means anything until the carousel's geometry is
        # configured, so the controls stay off rather than offering a range the
        # server will reject.
        for widget in (self.slot_box, self.sample_id_edit):
            widget.setEnabled(known)

        if not known:
            self._geometry_label.setText(
                "carousel.size not configured (json/palmixer_config.json)")
            self._geometry_label.setStyleSheet("color: red; font-weight: bold;")
            self._used_label.setText("")
            self.slot_table.setRowCount(0)
            return

        step_known = isinstance(step, (int, float)) and not isinstance(step, bool)
        self._geometry_label.setText(
            "%d slots, step %s --" % (size, ("%g" % step) if step_known
                                      else "NOT CONFIGURED"))
        self._geometry_label.setStyleSheet(
            "" if step_known else "color: red; font-weight: bold;")

        if self.slot_box.maximum() != size:
            # Clamping the value would fire valueChanged and overwrite a
            # half-typed sample ID, so suppress it while adjusting the range.
            blocked = self.slot_box.blockSignals(True)
            self.slot_box.setRange(1, size)
            self.slot_box.blockSignals(blocked)

        used = len(self._samples)
        full = bool(snapshot.get("carousel_full"))
        self._used_label.setText("%d/%d slots used%s" % (used, size,
                                                         " -- FULL, replace it" if full else ""))
        self._used_label.setStyleSheet("color: red; font-weight: bold;" if full else "")

        self.slot_table.setRowCount(size)
        for row in range(size):
            slot = row + 1
            position = positions.get(slot)
            cells = (str(slot),
                     "not taught" if position is None else "%.4f" % position,
                     self._samples.get(slot, ""))
            for col, text in enumerate(cells):
                item = self.slot_table.item(row, col)
                if item is None:
                    item = QTableWidgetItem()
                    self.slot_table.setItem(row, col, item)
                if item.text() != text:
                    item.setText(text)

    @property
    def busy_widgets(self):
        """Widgets to disable while the server reports BUSY."""
        return list(self._all_buttons) + self.flowcell.busy_widgets
