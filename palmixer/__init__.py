"""
PALmixer: control package for the APS 12-ID-B PALmixer flowcell workflow.

Ties together the UR3 robot (via UR_12idb / PAL12idb), a placeholder pump,
and an EPICS motor behind a ZMQ REQ/REP command server, with MQTT motion
status. See palmixer.server (headless server) and palmixer.gui.app (PyQt5
client GUI).
"""

__version__ = "0.1.0"
