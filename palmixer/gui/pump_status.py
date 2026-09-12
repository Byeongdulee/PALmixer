# -*- coding: utf-8 -*-
"""A read-only panel showing all four pumps, shared by the Experiment and
Automation tabs.

The data arrives on the ordinary get_state snapshot as ``pump_status`` (see
Pump.status_snapshot and the server's background poller), so this widget does
no I/O of its own -- like the tracking panel, it is handed a dict and draws it.
Both tabs get their own instance; app.py already calls update_state() on both.

Four rows, in the order an operator thinks about them: the two mixing pumps
(port 5555) then the two flowcell pumps (port 5556).
"""

from PyQt5.QtWidgets import (
    QAbstractItemView, QGroupBox, QHeaderView, QLabel, QTableWidget,
    QTableWidgetItem, QVBoxLayout,
)

#: Server key -> what to call it in the Pump column.
SERVER_LABELS = {"mixer": "Mixer", "flowcell": "Flowcell"}
#: The order rows are drawn in, regardless of dict ordering.
SERVER_ORDER = ("mixer", "flowcell")

_BUSY_STYLE = "color: darkred; font-weight: bold;"
_ERROR_STYLE = "color: darkred; font-weight: bold;"

#: Table rows: exactly the two mixer pumps plus the two flowcell pumps, never
#: more or fewer -- the row count itself must not vary once data is flowing,
#: or the table (and this panel) would change height with it.
ROW_COUNT = 4
#: Longest summary text shown before it is elided with "...", full text in
#: the tooltip. The summary can carry up to three "|"-joined notes (one per
#: server, plus a stop-requested one), which word-wrapping let grow this
#: panel by one or two lines depending on how many applied at the moment --
#: eliding to one line keeps the panel's height constant regardless.
SUMMARY_TEXT_LIMIT = 100


def _elide(text, limit):
    """(shown, tooltip) -- tooltip is None when nothing was cut."""
    text = str(text)
    if len(text) <= limit:
        return text, None
    return text[:limit - 3] + "...", text


class PumpStatusPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Pumps", parent)

        layout = QVBoxLayout()
        # Single line, never wrapped: this panel's height must not change as
        # status arrives, and a 1-3 line summary (depending how many "|"-
        # joined notes apply right now) did exactly that. Long text is
        # elided instead, with the full text in the tooltip.
        self._summary = QLabel("waiting for the server ...")
        self._summary.setWordWrap(False)
        layout.addWidget(self._summary)

        self.table = QTableWidget(ROW_COUNT, 4)
        self.table.setHorizontalHeaderLabels(["Pump", "Server", "Status", "Position"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        # Fixed at exactly ROW_COUNT rows (set once, never resized -- see
        # _draw) and a fixed height, not a maximum: a maximum still lets the
        # table shrink below it before the first snapshot arrives.
        self.table.setFixedHeight(140)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        layout.addWidget(self.table)

        self.setLayout(layout)

    # -- external updates ------------------------------------------------------
    def update_state(self, snapshot):
        """Redraw from a get_state snapshot. Tolerates a missing pump_status,
        which is what an older server (or one that has not polled yet) sends."""
        status = snapshot.get("pump_status")
        if not isinstance(status, dict):
            self._set_summary("no pump status from the server yet", warn=False)
            return
        if status.get("error"):
            self._set_summary("pump status failed: %s" % status["error"], warn=True)
            return

        servers = status.get("servers") or {}
        rows, notes = [], []
        for key in SERVER_ORDER:
            entry = servers.get(key)
            if not isinstance(entry, dict):
                continue
            label = SERVER_LABELS.get(key, key)
            port = entry.get("port")
            if entry.get("error"):
                notes.append("%s (%s): %s" % (label, port, entry["error"]))
            elif not entry.get("connected"):
                notes.append("%s (%s): dashboard up, pumps not connected"
                             % (label, port))
            if entry.get("stop_requested"):
                notes.append("%s (%s): EMERGENCY STOP requested" % (label, port))
            for pump in entry.get("pumps") or []:
                rows.append((str(pump.get("name", "")),
                             "%s %s" % (label, port if port else ""),
                             str(pump.get("status", "")),
                             str(pump.get("position", "")),
                             bool(entry.get("operation_active"))))

        self._draw(rows)
        if notes:
            self._set_summary(" | ".join(notes), warn=True)
        else:
            active = [SERVER_LABELS.get(k, k) for k in SERVER_ORDER
                      if isinstance(servers.get(k), dict)
                      and servers[k].get("operation_active")]
            self._set_summary("running: %s" % ", ".join(active) if active
                              else "both dashboards connected, idle",
                              warn=bool(active))

    def _set_summary(self, text, warn):
        shown, tooltip = _elide(text, SUMMARY_TEXT_LIMIT)
        self._summary.setText(shown)
        self._summary.setToolTip(tooltip or "")
        self._summary.setStyleSheet(_ERROR_STYLE if warn else "")

    def _draw(self, rows):
        # Always exactly ROW_COUNT rows -- never resized to len(rows), so a
        # partial snapshot (a server missing from `servers` entirely, say)
        # leaves the remaining rows blank rather than changing the table's,
        # and so this panel's, height.
        for r in range(ROW_COUNT):
            if r < len(rows):
                name, server, status, position, active = rows[r]
            else:
                name = server = status = position = ""
                active = False
            for c, text in enumerate((name, server, status, position)):
                item = self.table.item(r, c)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(r, c, item)
                if item.text() != text:
                    item.setText(text)
                # Only the status cell is coloured: colouring the whole row for
                # a pump that is simply busy would make a normal run look like
                # a fault.
                if c == 2:
                    item.setToolTip(text)
                    self.table.item(r, c).setForeground(
                        _busy_brush() if active else _plain_brush())


def _busy_brush():
    from PyQt5.QtGui import QBrush, QColor
    return QBrush(QColor("darkred"))


def _plain_brush():
    from PyQt5.QtGui import QBrush
    return QBrush()
