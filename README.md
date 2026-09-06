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
| `search_apriltag <station>` | worker | Locate a station's AprilTag (`sample_table`, `cleaning_station`, `mixer_station`, `mixer_cleaning_station`) and record its position |
| `stop_search` | fast | Abort an in-progress `search_apriltag`: stops the robot immediately (`stopj`) and signals the search loop to give up rather than continue to the next tilt/step. `ERROR: no AprilTag search is running` if nothing is searching |
| `push_positions` / `pull_positions` | worker | Sync taught positions with the EPICS waypoint PVs (see below) |
| `mixer2cleaningstation` etc. (8 names) | worker | Run the matching `PAL12idb` transport function |
| `motor_tweak forward\|reverse <step>` | worker | Tweak `12idb:m6` by `step` |
| `pump mix\|clean_mixer\|draw_to_flowcell\|aspirate_from_flowcell\|wash_flowcell` | worker | Run a pump operation (placeholder) |
| `make_sample <slot>` | worker | Full mix-and-load sequence (see below) |
| `unload_sample` | worker | Full return-and-wash sequence (see below) |
| `set_flowcell <1\|2>` | fast | Change which flowcell the transport functions and workflows act on -- a "Flowcell in Use" selector sending this lives on all three GUI tabs, kept in sync with each other from `get_state`/the tracking topic |
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

The GUI greys its flowcell selectors out while the server is BUSY, even
though `set_flowcell` is one of the fast commands the server would answer
mid-action: `PAL12idb` re-reads `flowcell_ID` at every transport call, so
switching part-way through a workflow would send the remaining steps to the
other flowcell's cleaning station and record their outcome against the wrong
flowcell. The selectors keep following the server's value while disabled, so
the display stays truthful throughout a run.

Transport commands, `make_sample`, and `unload_sample` also fail fast --
before ever replying `ACCEPTED` -- if a station position they need has never
been taught: the reply is `ERROR: position not configured: <Station Labels>
-- use the Configuration tab to search its AprilTag first`. This only ever
originates from the Experiment and Automation tabs' actions (the
Configuration tab's `search_apriltag` is how a position gets taught in the
first place), so the GUI pops up a dialog with this specific error instead
of only logging it -- better than discovering it minutes into a workflow
that was never going to succeed.

Which stations a command needs is derived from the transport functions it
actually runs, not a hand-kept list, and it follows the current tracked
state: `unload_sample` only needs the mixer cleaning station when the mixer
head is sitting at the mixer station and therefore has to be parked out of
the way first.

## Taught positions and the EPICS waypoint PVs

Positions taught by `search_apriltag` are stored in `palmixer/waypoints.ini`
(git-ignored, created on first use). **That file is what every move reads.**
Nothing on the control path does Channel Access for a waypoint: a `caget` on
a disconnected PV blocks for seconds, and the position pre-check above runs
on the synchronous ZMQ reply path, where a stall would time the GUI out --
or worse, let the server start a robot sequence after the client had already
given up on it.

The `12idUR:WaypointL:<ID>:<X|Y|Z|RX|RY|RZ>` PVs are an interchange with the
rest of the beamline rather than a store this package reads. The
Configuration tab's **Push to EPICS PVs** / **Pull from EPICS PVs** buttons
(`push_positions` / `pull_positions`) are the only things that move values
between the two, in either direction:

- **Push** writes every taught position out to its PVs. Stations not yet
  taught are skipped, not an error.
- **Pull** reads the PVs back into `waypoints.ini`. A station is only written
  when all six of its fields read back, so a partial EPICS outage cannot
  overwrite a good position with junk.

Both report per-station results (`pushed 3/5 (...)`; `no value from PV: ...`)
and only fail outright when no station synced at all.

| Waypoint ID | Station | `waypoints.ini` section |
|---|---|---|
| 1 | Sample table | `sample_table` |
| 2 / 3 | Flowcell cleaning station 1 / 2 | `cleaning_station_1` / `_2` |
| 4 | Mixer station | `mixer_station` |
| 5 | Mixer cleaning station | `mixer_cleaning_station` |

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
where it needs to be -- they never guess. This state-based guard runs after
the position-not-configured guard described above (which checks whether the
stations involved have been taught at all), so an unconfigured install is
caught before an out-of-place flowcell is.

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
