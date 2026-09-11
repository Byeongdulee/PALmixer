## 3.1 — Connect to the GUI

import configparser
import os
import zmq
import time

# Importable either as part of the palmixer package or run directly as a
# script from within this directory (`python daq_client.py`), which is not a
# package context -- the relative import then fails and the bare one, which
# needs this directory on sys.path, is what actually resolves. Same fallback
# PAL12idb.py uses for its own sibling-module imports.
try:
    from . import config as _config
except ImportError:
    import config as _config

_daq_cfg = _config.get_section('daq')

GUI_HOST = _daq_cfg.get('host', 'purple.xray.aps.anl.gov')  # hostname of the machine running the GUI
GUI_PORT = int(_daq_cfg.get('port', 9876))                  # default port; change if GUI uses a different port
REQUEST_TIMEOUT_MS = int(float(_daq_cfg.get('request_timeout_s', 5.0)) * 1000)  # how long to wait for a reply

_ctx = zmq.Context()
_sock = None


def _connect():
    """(Re)open the REQ socket to the GUI.

    A REQ socket that misses a reply is stuck -- it cannot send again until it
    receives the one it is still waiting for. cmd() drops the socket and calls
    this again after any timeout or transport error, so one dropped reply
    cannot wedge every later call for the rest of the process.
    """
    global _sock
    sock = _ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, REQUEST_TIMEOUT_MS)
    sock.setsockopt(zmq.SNDTIMEO, REQUEST_TIMEOUT_MS)
    sock.connect(f'tcp://{GUI_HOST}:{GUI_PORT}')
    _sock = sock


_connect()


def cmd(command):
    """Send a command string to the GUI and return the reply string.

    Raises RuntimeError if the GUI does not reply within REQUEST_TIMEOUT_MS, or
    on any other transport error. The socket is closed and reopened first, so
    the *next* call starts clean instead of hanging on a reply this one never
    got.
    """
    try:
        _sock.send_string(command)
        reply = _sock.recv_string()
    except zmq.ZMQError as e:
        _sock.close(0)
        _connect()
        raise RuntimeError(f'daq_client: {command!r} failed: {e}') from e
    print(f'  >> {command!r:50s}  ->  {reply}')
    return reply


if __name__ == '__main__':
    # Verify the connection
    cmd('status')

## 3.2 — Status check and wait-for-idle helper

# Acquisition commands return ACCEPTED immediately and run in the background.
# Use status polling to detect when a scan or acquisition has finished.

def wait_idle(timeout=300, poll=1.0):
    """Block until the GUI reports IDLE (acquisition/scan finished)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cmd('status') == 'IDLE':
            return True
        time.sleep(poll)
    print('WARNING: timed out waiting for IDLE')
    return False


if __name__ == '__main__':
    # Quick check
    print('Current state:', cmd('status'))   # IDLE or RUNNING

## 3.3 — Motor read and move

MOTORS = ['sth', 'sav', 'stv']

def get_pos():
    """Read positions for MOTORS, in order."""
    return [cmd(f'getpos {motor}') for motor in MOTORS]

def set_pos(positions):
    """Move each motor in MOTORS to the matching entry in `positions`.

    `positions` must have one value per entry in MOTORS, in the same order
    get_pos() returns them -- not a single shared value.
    """
    for motor, pos in zip(MOTORS, positions):
        cmd(f'move {motor} {pos}')
        time.sleep(0.5)
    wait_idle()

## 3.4 — Sample table alignment

# The DAQ GUI's sample-stage motors are a separate control system from the
# robot: PALmixer's own waypoints.ini records where the robot's gripper found
# the sample table's AprilTag, but says nothing about where the beamline stage
# itself was standing at the time. record_positions() captures that the moment
# the robot's sample_table position is (re)taught (PAL12idb.store_station_
# position); align_sample_table() puts the stage back there before every
# transport that moves the flowcell onto or off of the sample table, so the
# two never drift apart just because something else moved the stage in
# between.
_POSITIONS_INI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'daq_positions.ini')
SAMPLE_TABLE_SECTION = 'sample_table'


def record_positions(section=SAMPLE_TABLE_SECTION):
    """Read MOTORS' current positions from the GUI and save them under `section`.

    Returns {motor: position}. Merges into daq_positions.ini rather than
    overwriting it, so recording one section leaves any others untouched.
    """
    positions = dict(zip(MOTORS, get_pos()))
    parser = configparser.ConfigParser()
    parser.read(_POSITIONS_INI_PATH)
    if not parser.has_section(section):
        parser.add_section(section)
    for motor, value in positions.items():
        parser.set(section, motor, str(value))
    with open(_POSITIONS_INI_PATH, 'w') as fp:
        parser.write(fp)
    return positions


def get_recorded_positions(section=SAMPLE_TABLE_SECTION):
    """The positions last recorded for `section`, as {motor: position}.

    None if `section` has never been recorded, or was recorded before MOTORS
    named one of its current entries -- either way, nothing to align to.
    """
    parser = configparser.ConfigParser()
    parser.read(_POSITIONS_INI_PATH)
    if not parser.has_section(section):
        return None
    try:
        return {motor: parser.get(section, motor) for motor in MOTORS}
    except configparser.NoOptionError:
        return None


def align_sample_table():
    """Move MOTORS to the positions recorded for the sample table.

    Raises RuntimeError if they have never been recorded, rather than leaving
    the flowcell at a sample table the beam is not actually looking at --
    search (or teach) the sample_table station once with this client able to
    reach the GUI, which records them (see record_positions()).
    """
    positions = get_recorded_positions()
    if positions is None:
        raise RuntimeError(
            f'sample_table has no recorded DAQ motor positions in '
            f'{_POSITIONS_INI_PATH}; search its AprilTag (or teach it by '
            f'hand) once to record them')
    set_pos([positions[motor] for motor in MOTORS])
    return positions
