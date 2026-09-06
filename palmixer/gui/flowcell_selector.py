# -*- coding: utf-8 -*-
"""Shared "Flowcell in Use" radio group.

Every transport/pump/workflow action acts on whichever flowcell is currently
selected (PAL12idb's module-global ``flowcell_ID``), so the Configuration,
Experiment, and Automation tabs each embed one of these rather than only the
tab that happens to run a given action.
"""

from PyQt5.QtWidgets import QButtonGroup, QGroupBox, QHBoxLayout, QRadioButton

from .. import commands as cmd


class FlowcellSelector(QGroupBox):
    def __init__(self, send_command, parent=None):
        super().__init__("Flowcell in Use", parent)
        self._send_command = send_command

        layout = QHBoxLayout()
        self._group = QButtonGroup(self)
        for fc in cmd.FLOWCELL_IDS:
            radio = QRadioButton("Flowcell %d" % fc)
            # `clicked` fires only on real user interaction, unlike `toggled`,
            # which also fires for the default setChecked() below and for the
            # programmatic sync in set_flowcell_in_use() -- either of which
            # would otherwise send an unsolicited set_flowcell to the server.
            radio.clicked.connect(lambda _checked, n=fc: self._send_command(cmd.set_flowcell_command(n)))
            self._group.addButton(radio, fc)
            layout.addWidget(radio)
            if fc == cmd.FLOWCELL_IDS[0]:
                radio.setChecked(True)
        layout.addStretch(1)
        self.setLayout(layout)

    def set_flowcell_in_use(self, flowcell_id):
        """External sync, from get_state polling or the tracking MQTT topic.

        Works while the group is disabled: setChecked() is unaffected by
        enabled state, so the display stays truthful all the way through a
        run even though the operator cannot touch it.
        """
        btn = self._group.button(int(flowcell_id))
        if btn is not None and not btn.isChecked():
            btn.setChecked(True)  # `clicked` does not fire for this

    @property
    def busy_widgets(self):
        """Disable the whole group while the server is BUSY.

        `set_flowcell` is a fast command the server answers even mid-action,
        but acting on it then would be wrong: PAL12idb reads `flowcell_ID`
        afresh at each transport call, so a switch part-way through a
        workflow sends its remaining steps to the other flowcell's cleaning
        station and records their outcome against the wrong flowcell.
        Returning the group box rather than the individual radios greys out
        the title too, so it reads as unavailable rather than merely
        unresponsive.
        """
        return [self]
