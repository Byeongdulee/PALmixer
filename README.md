# PALmixer

Control package for the APS 12-ID-B PALmixer flowcell workflow: a UR3 robot
(via [UR_12idb](../UR_12idb)), an EPICS motor (`12idb:m6`), and a mixer pump
(placeholder driver, no hardware integration yet).

## Architecture

- **`palmixer.server`** -- a headless process that owns the robot, pump,
  motor, and the `PAL12idb` transport/AprilTag library. It exposes a small
  command vocabulary over a **ZMQ REQ/REP** socket and reports motion
  start/success/failure over **MQTT**.
- **`palmixer.gui.app`** -- a **PyQt5** GUI that is a thin ZMQ client: every
  button sends a command string to the server and shows the immediate
  accept/reject reply. The actual outcome of each action streams back over
  MQTT and is shown in a status log with an IDLE/BUSY badge.
- The camera view on the Configuration tab talks **directly** to the UR
  wrist camera's own HTTP server (`http://<robot-ip>:4242/current.jpg`),
  independent of the robot controller and the ZMQ control plane, so it keeps
  working even when the robot is stopped or busy.

```
GUI (PyQt5) --ZMQ REQ/REP--> server.py --controls--> robot / motor / pump
   |                                        |
   +-----------HTTP :4242 (camera)          +--MQTT status--> GUI + others
```

## Configuration

Edit `json/palmixer_config.json`, or override with environment variables:

| Env var | Overrides |
|---|---|
| `PALMIXER_ZMQ_HOST` / `PALMIXER_ZMQ_PORT` | ZMQ command server address |
| `PALMIXER_MQTT_HOST` / `PALMIXER_MQTT_PORT` | MQTT broker address |
| `PALMIXER_ROBOT_IP` | UR3 robot / camera IP or hostname |
| `PALMIXER_UR12IDB_PATH` | Path to the `UR_12idb` checkout (importable `robot12idb`) |
| `PALMIXER_MOTOR_PV` | EPICS motor PV base (default `12idb:m6`) |

## Running

On the Linux beamline host, running against real hardware (no `--simulate`)
needs `robot12idb`/`camera_tools` and their dependencies, which live in the
`aps12robot` environment -- activate it before starting the server:

```sh
conda activate aps12robot
```

```sh
# Terminal 1: the control server. Add --simulate to skip real hardware I/O
# (robot/motor calls just sleep and report success -- useful for GUI/dev work,
# and does not require the aps12robot environment).
python -m palmixer.server --simulate

# Terminal 2: the GUI
python -m palmixer.gui.app
```

An MQTT broker (e.g. `mosquitto`) reachable at the configured host/port is
needed for the status log to populate; ZMQ command/reply works without one
(MQTT publishing fails open -- it never blocks or breaks control).

## Command vocabulary (ZMQ)

| Command | Effect |
|---|---|
| `status` | `IDLE` or `BUSY` |
| `search_apriltag <station>` | Locate a station's AprilTag (`sample_table`, `cleaning_station`, `mixer_station`, `mixer_cleaning_station`) |
| `mixer2cleaningstation` etc. (8 names) | Run the matching `PAL12idb` transport function |
| `motor_tweak forward\|reverse <step>` | Tweak `12idb:m6` by `step` |
| `pump mix\|clean_mixer\|draw_to_flowcell\|aspirate_from_flowcell` | Run a pump operation (placeholder) |

Only one hardware action runs at a time; a command sent while busy gets
`ERROR: busy`.

## Pump driver

`palmixer/pump.py` is a **placeholder** -- there is no pump hardware
integration yet. It implements the same four operations the GUI calls
(`mix`, `clean_mixer`, `draw_to_flowcell`, `aspirate_from_flowcell`), each
just logging and sleeping briefly. Swap its internals for a real driver
(e.g. a Tricontinent-style pump, following the pattern in
`UR_12idb/common/tc_pipet.py`) without touching `server.py` or the GUI.
