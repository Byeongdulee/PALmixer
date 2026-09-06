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
| `PALMIXER_CAROUSEL_SIZE` / `PALMIXER_CAROUSEL_STEP` | Carousel geometry: slot count, and motor travel between adjacent slots (see below) |

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

### On Windows

`UR_12idb` is not installed in the environment, so point `PALmixer` at a
checkout of it:

```powershell
$env:PALMIXER_UR12IDB_PATH = 'C:\path\to\UR_12idb'
```

Activating the environment (rather than invoking `...\envs\aps12robot\python.exe`
by path) is still the right thing to do, but it is no longer load-bearing:
`palmixer/_winenv.py` puts `<env>\Library\bin` back on `PATH` at import if
activation did not. Without that, conda-forge's MKL-linked numpy dies at its
first `np.linalg.inv()` with a delay-load failure (`0xC06D007F`) that kills the
process with no traceback -- which in practice meant the server vanishing
part-way through an AprilTag search, since `urx`'s `get_pose()` is what reaches
that call. See the docstring in `_winenv.py`.

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
| `make_sample <slot> [sample id]` | worker | Full mix-and-load sequence (see below), tagging `<slot>` with the sample ID -- a timestamp ID is assigned if none is given |
| `unload_sample` | worker | Full return-and-wash sequence (see below) |
| `set_flowcell <1\|2>` | fast | Change which flowcell the transport functions and workflows act on -- a "Flowcell in Use" selector sending this lives on all three GUI tabs, kept in sync with each other from `get_state`/the tracking topic |
| `get_state` | fast | JSON tracking snapshot (see below) |
| `set_location <mixer_head\|flowcell_1\|flowcell_2> <value>` | fast | Reconcile a tracked location after manual intervention |
| `teach_carousel_slot <n>` | fast | Record the motor's current `.RBV` as carousel slot `n`, locating **every** slot (see below) |
| `set_sample_id <slot> <sample id>` | fast | Tag `slot` with a sample ID, marking it used. Overwrites any existing ID |
| `get_sample_id <slot>` | fast | That slot's sample ID, or `unknown` if it is unused |
| `clear_sample_id <slot>` | fast | Drop the slot's sample ID, marking it unused again |
| `reset_carousel` | fast | Replace the carousel: clear every sample ID **and** the taught reference position |

Every argument is a single whitespace-delimited token except a sample ID,
which is the rest of the line and may contain spaces (runs of whitespace in it
collapse to one).

"worker" commands run on the server's single background worker thread and
reply `ACCEPTED` / `ERROR: <reason>` immediately, with the actual
started/success/failure reported over MQTT as the action runs. "fast"
commands are read-only or pure bookkeeping and reply straight away --
including while a worker command is running, since they don't touch
hardware.

Only one worker action runs at a time; a worker command sent while busy gets
`ERROR: busy`.

### Transport functions

The eight `PAL12idb` transport functions above, each sent as its own bare
command:

| Command | Move |
|---|---|
| `mixer2cleaningstation` | Mixer -> Cleaning Station |
| `mixer2mixingstation` | Cleaning Station -> Mixer |
| `load_flowcell_from_cleaningstation_to_beam` | Cleaning Station -> Beam |
| `load_flowcell_from_beam_to_cleaningstation` | Beam -> Cleaning Station |
| `ready_flowcell_to_draw` | Ready Flowcell to Draw |
| `load_sample_to_beam` | Load Sample to Beam |
| `return_sample` | Return Sample from Beam |
| `wash_flowcell_after_return` | Wash Flowcell After Return |

Each acts on the flowcell currently in use (`set_flowcell`) and records its
own target location on success (see [State tracking](#state-tracking)).

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
| Carousel position | the slot last moved to |
| Carousel reference | one taught `(slot, motor position)`, from which every slot's position is derived |
| Carousel sample IDs | the sample ID held in each used slot |
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
 "carousel_slots": {"1": 0.0, "2": 30.0, "3": 60.0}, "carousel_size": 3,
 "carousel_step": 30.0, "carousel_reference": [1, 0.0],
 "carousel_samples": {"1": "BSA 5 mg/ml", "3": "S20260906-142530"},
 "carousel_full": false}
```

`carousel_size` and `carousel_step` are echoed from the config so a client does
not need to read it. `carousel_slots` is **derived** from `carousel_reference`
and the step, so it is either empty (nothing taught) or complete. It and
`carousel_samples` are keyed by slot number; JSON makes those keys strings.

The same snapshot is published, retained, on MQTT topic
`aps12/<beamline>/palmixer/tracking` on every change, so a GUI connecting
mid-run sees the current picture immediately.

### The carousel: slots, sample IDs, and replacing it

The carousel is a consumable. It holds a fixed number of slots, each slot is
spent once a sample has been mixed in it, and when they are all used the whole
carousel is swapped for a fresh one. The Automation tab's Carousel panel is
that inventory: the slot count, a table of what each slot holds, and the
controls below.

**Geometry is configuration.** How many slots the carousel has and how far the
motor travels between two adjacent ones are properties of the hardware, so they
live in `json/palmixer_config.json`, not in a command:

```json
"carousel": { "size": 12, "step": 30.0 }
```

Both default to `0`, meaning "not configured" -- a wrong `step` would drive the
carousel to the wrong slot, so an absent one refuses the move rather than
guessing. `step` is in the motor's engineering units and may be negative, for a
carousel whose slot numbering runs against the motor's positive direction.
`PALMIXER_CAROUSEL_SIZE` / `PALMIXER_CAROUSEL_STEP` override them. Slot numbers
are bounded to `1..size`.

**Teaching is one slot, not all of them.** Because the slots are evenly spaced,
only one has to be located: drive the motor to any slot with the Experiment
tab's forward/reverse tweak buttons, then use "Teach Position"
(`teach_carousel_slot <n>`) to record it. That one reference plus `step` gives
every other slot -- `slot n = reference + (n - reference_slot) * step` -- and
`make_sample <slot>` drives there with an absolute move. Teaching a second slot
does not extend a table; it *replaces* the reference, re-locating the whole
carousel. The Automation tab's slot table shows the resulting positions, which
is the quickest way to catch a wrong `step`.

A reference for a slot outside the configured size (a differently-sized
carousel was configured since) reads as untaught rather than being trusted.

**Sample IDs.** A slot is "used" exactly when it holds a sample ID -- the two
are the same fact, so they cannot disagree about whether a slot is spent.
`make_sample <slot> [sample id]` tags the slot the moment its `mix` step
succeeds, because mixing is what consumes the vial: a later step failing does
not un-consume it, and the record does not depend on the run finishing. Omit
the ID and a timestamp one (`S20260906-142530`) is assigned, so a used slot is
never anonymous. `set_sample_id` / `get_sample_id` / `clear_sample_id` read and
write the same tags outside a run. Mixing into an already-used slot is allowed
and overwrites its ID.

**Replacing it.** `carousel_full` goes true once every slot is used. It is
advisory, not a block -- re-mixing a used slot stays legal -- but it is
surfaced in the tracking snapshot, in red on the Automation tab, and in the
`make_sample` completion detail. Swap in a fresh carousel, then
`reset_carousel` (the tab's **Replace Carousel (Reset)** button, behind a
confirmation). That clears the sample IDs **and the taught reference**, since a
replacement carousel is not guaranteed to seat where the old one did -- so one
slot must be taught again before the next sample. The geometry is configuration
and is untouched; a physically different carousel means editing
`json/palmixer_config.json` and restarting the server.

## Automation workflows

The Automation tab's two buttons replace the nine-click manual sequence with
one call each. Both refuse immediately (`ERROR: <reason>`, naming the
`set_location` command to fix it) if a required location is `unknown` or not
where it needs to be -- they never guess. This state-based guard runs after
the position-not-configured guard described above (which checks whether the
stations involved have been taught at all), so an unconfigured install is
caught before an out-of-place flowcell is.

**`make_sample <slot> [sample id]`**: mixer2cleaningstation (if needed) ->
rotate carousel to `<slot>` -> mixer2mixingstation -> pump mix (**slot tagged
with the sample ID here**) -> mixer2cleaningstation -> pump clean_mixer (fired
on a background thread, not awaited) -> ready_flowcell_to_draw -> pump
draw_to_flowcell (blocks until clean_mixer finishes, since pump ops serialize
on one lock) -> load_sample_to_beam. Requires the flowcell in use to be at its
cleaning station, the slot to be within the configured carousel size, and the
carousel to have been located (one taught reference).

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
