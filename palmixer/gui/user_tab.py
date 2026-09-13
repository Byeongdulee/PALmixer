# -*- coding: utf-8 -*-
"""Who is at the bench: the badge and proposal mix confirmations are written under.

PVapp takes a **badge number** and a **GUP (proposal) number** as one credential. From
a beamline address it accepts the pair in place of a username and password, and the
record PALmixer updates after a mix stays under that proposal, where the rest of the
team can find it.

A campaign sends its own pair when it starts, and that always wins -- it is the thing
that knows which proposal asked for a sample. This tab is the answer to the other case:
somebody running ``make_sample`` by hand with no campaign driving, who is still working
under a proposal. What is typed here is held as a **fallback**, so setting it can never
take a running campaign's samples out from under the proposal that asked for them.

Nothing here is secret. A badge number is not a password, and the pair only works from
inside the beamline network -- which is why it can sit in a form rather than an
environment variable. It is sent to the server, not written to the config file, so it
lasts until the server restarts.
"""

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIntValidator
from PyQt5.QtWidgets import (
    QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QWidget,
)

from .. import commands as cmd
from .. import config


class UserTab(QWidget):
    """Badge + GUP, used when no campaign has said whose run this is."""

    def __init__(self, send_command, parent=None):
        super().__init__(parent)
        self.send_command = send_command

        layout = QVBoxLayout()
        layout.addWidget(QLabel(
            "Who samples are recorded as. PVapp takes a badge number and a proposal "
            "(GUP) number together, and records made with them stay under that "
            "proposal.\n\nA running campaign sends its own and that takes precedence; "
            "what you set here is used when none has."))

        box = QGroupBox("PVapp identity")
        form = QFormLayout()
        self.badge = QLineEdit()
        self.badge.setPlaceholderText("e.g. 313294")
        # Digits only: PVapp looks the badge up as an integer, and a typo here is a 401
        # a minute later on a sample that has already been made.
        self.badge.setValidator(QIntValidator(1, 99999999, self))
        self.gup = QLineEdit()
        self.gup.setPlaceholderText("e.g. 1010753")
        self.gup.setValidator(QIntValidator(1, 99999999, self))
        form.addRow("Badge number", self.badge)
        form.addRow("Proposal (GUP)", self.gup)
        box.setLayout(form)
        layout.addWidget(box)

        buttons = QHBoxLayout()
        self.apply_button = QPushButton("Use these")
        self.clear_button = QPushButton("Clear")
        buttons.addWidget(self.apply_button)
        buttons.addWidget(self.clear_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)

        self.status = QLabel("")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addStretch(1)
        self.setLayout(layout)

        self.apply_button.clicked.connect(self.apply)
        self.clear_button.clicked.connect(self.clear)
        self._show_configured()

    # -- the two buttons -------------------------------------------------------
    def apply(self):
        badge, gup = self.badge.text().strip(), self.gup.text().strip()
        # Both or neither. The proposal is what stands in for the password, so a badge
        # on its own is not a credential -- refusing here beats a 401 several seconds
        # later on a sample that has already been prepared.
        if not badge or not gup:
            self._warn("Both a badge number and a proposal are needed: PVapp checks that "
                       "the proposal belongs to the badge, and that check is what lets "
                       "the pair stand in for a password.")
            return
        self.send_command(cmd.set_credentials_command(badge=badge, gup=gup, source="user"))
        self._ok("Sent: badge %s on proposal %s. In use unless a campaign provides its "
                 "own." % (badge, gup))

    def clear(self):
        """Stop offering a fallback, without touching whatever a campaign has set."""
        self.badge.clear()
        self.gup.clear()
        self.send_command(cmd.set_credentials_command(badge="", gup="", source="user"))
        self._ok("Cleared. Confirmations fall back to this host's configured identity.")

    # -- helpers ---------------------------------------------------------------
    def _show_configured(self):
        """Prefill from the config file, so the common case is one click.

        Read here rather than asked of the server: the pair is a credential, so it is
        deliberately not in the server's status snapshot, which is broadcast.
        """
        try:
            section = config.get_section("pvapp")
        except Exception:                       # noqa: BLE001 - a GUI must still open
            return
        badge = str(section.get("badge") or section.get("owner_badge") or "").strip()
        gup = str(section.get("gup") or "").strip()
        self.badge.setText(badge)
        self.gup.setText(gup)
        if badge and gup:
            self.status.setText("This host is configured as badge %s on proposal %s. "
                                "Press 'Use these' to confirm, or type another."
                                % (badge, gup))
        else:
            self.status.setText("This host has no PVapp identity configured yet.")

    def _ok(self, text):
        self.status.setStyleSheet("")
        self.status.setText(text)

    def _warn(self, text):
        self.status.setStyleSheet("color: #b00;")
        self.status.setText(text)
