"""
PALmixer: control package for the APS 12-ID-B PALmixer flowcell workflow.

Ties together the UR3 robot (via UR_12idb / PAL12idb), a placeholder pump,
and an EPICS motor behind a ZMQ REQ/REP command server, with MQTT motion
status. See palmixer.server (headless server) and palmixer.gui.app (PyQt5
client GUI).
"""

__version__ = "0.1.0"

# Before anything imports numpy. On Windows this repairs the DLL search path
# when the conda environment was started by interpreter path instead of being
# activated -- without it the first np.linalg.inv() (which urx's get_pose()
# reaches through math3d) kills the process with a delay-load failure rather
# than raising. See palmixer._winenv for the full story.
from ._winenv import ensure_conda_dll_path as _ensure_conda_dll_path

_ensure_conda_dll_path()
