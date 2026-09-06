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

| Command | Path | Effect |
|---|---|---|
| `status` | fast | `IDLE` or `BUSY` |
| `search_apriltag <station>` | worker | Locate a station's AprilTag (`sample_table`, `cleaning_station`, `mixer_station`, `mixer_cleaning_station`) |
| `mixer2cleaningstation` etc. (8 names) | worker | Run the matching `PAL12idb` transport function |
| `motor_tweak forward\|reverse <step>` | worker | Tweak `12idb:m6` by `step` |
| `pump mix\|clean_mixer\|draw_to_flowcell\|aspirate_from_flowcell\|wash_flowcell` | worker | Run a pump operation (placeholder) |
| `make_sample <slot>` | worker | Full mix-and-load sequence (see below) |
| `unload_sample` | worker | Full return-and-wash sequence (see below) |
| `set_flowcell <1\|2>` | fast | Change which flowcell the transport functions and workflows act on |
| `get_state` | fast | JSON tracking snapshot (see below) |
| `set_location <mixer_head\|flowcell_1\|flowcell_2> <value>` | fast | Reconcile a tracked location after manual intervention |
| `teach_carousel_slot <n>` | fast | Record the motor's current `.RBV` as carousel slot `n` |

"worker" commands run on the server's single background worker thread and
reply `ACCEPTED` / `ERROR: <reason>` immediately, with the actual
started/success/failure reported over MQTT as the action runs. "fast"
commands are read-only or pure bookkeeping and reply straight away --
including while a worker command is running, since they don't touch
hardware.

Only one worker action runs at a time; a worker command sent while busy gets
`ERROR: busy`.

## State tracking

The server keeps a small ini-backed model (`palmixer/state.py`,
`palmixer/palmixer_state.ini`) of where things physically are, since the
`make_sample`/`unload_sample` workflows below need to know before they start
moving anything:

| Tracked item | Values |
|---|---|
| Mixer head location | `mixer_station`, `mixer_cleaning_station`, or `unknown` |
| Flowcell 1 / flowcell 2 location (independent) | `sample_table`, `cleaning_station`, `gripper` (transient, mid-workflow), or `unknown` |
| Carousel position | a slot number, taught to an absolute motor position |
| Flowcell in use | `1` or `2` |

`unknown` is a first-class state, not an error -- a fresh install, or any
manual/hardware intervention, leaves the affected item `unknown` until the
operator reconciles it with `set_location` (or a workflow completes and sets
it itself). Each of the 8 `PAL12idb` transport functions records its own
target location on success and resets to `unknown` on a failed move, so
state stays correct whether a transport function is run directly (e.g. from
the Experiment tab) or as part of a workflow.

`get_state` returns this as JSON, e.g.:

```json
{"mixer_head": "mixer_cleaning_station", "flowcell_1": "cleaning_station",
 "flowcell_2": "unknown", "flowcell_in_use": 1, "carousel_slot": 3,
 "carousel_slots": {"1": 0.0, "2": 45.0, "3": 90.0}}
```

The same snapshot is published, retained, on MQTT topic
`aps12/<beamline>/palmixer/tracking` on every change, so a GUI connecting
mid-run sees the current picture immediately.

### Teaching carousel slots

There is no slot-to-position table shipped -- teach it once per install:
drive the carousel motor to a slot with the Experiment tab's forward/reverse
tweak buttons, then use the Automation tab's "Teach current position as this
slot" button (`teach_carousel_slot <n>`) to record the current position as
slot `n`. `make_sample <slot>` then drives there with an absolute move.

## Automation workflows

The Automation tab's two buttons replace the nine-click manual sequence with
one call each. Both refuse immediately (`ERROR: <reason>`, naming the
`set_location` command to fix it) if a required location is `unknown` or not
where it needs to be -- they never guess.

**`make_sample <slot>`**: mixer2cleaningstation (if needed) -> rotate
carousel to `<slot>` -> mixer2mixingstation -> pump mix ->
mixer2cleaningstation -> pump clean_mixer (fired on a background thread, not
awaited) -> ready_flowcell_to_draw -> pump draw_to_flowcell (blocks until
clean_mixer finishes, since pump ops serialize on one lock) ->
load_sample_to_beam. Requires the flowcell in use to be at its cleaning
station and the target slot to be taught.

**`unload_sample`**: mixer2cleaningstation (if needed) -> return_sample ->
pump aspirate_from_flowcell -> wash_flowcell_after_return -> pump
wash_flowcell -> raise the robot back to sample-table height. Requires the
flowcell in use to be at the sample table.

## Pump driver

`palmixer/pump.py` is a **placeholder** -- there is no pump hardware
integration yet. It implements the five operations the GUI and workflows
call (`mix`, `clean_mixer`, `draw_to_flowcell`, `aspirate_from_flowcell`,
`wash_flowcell`), each just logging and sleeping briefly, serialized on a
single lock so two pump ops never run concurrently. Swap its internals for a
real driver without touching `server.py`, `workflows.py`, or the GUI.
