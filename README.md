# PALmixer

Control package for the APS 12-ID-B PALmixer flowcell workflow: a UR3 robot
(via [UR_12idb](../UR_12idb)), an EPICS motor (`12idb:m6`), and a mixer pump
(a ZMQ client to [apssector12_pump_control](../apssector12_pump_control)).

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
| `PALMIXER_DAQ_HOST` / `PALMIXER_DAQ_PORT` | Beamline DAQ GUI address (`daq_client.py`; sample-stage motor alignment, see below) |

`carousel.step` is in the motor's own engineering units, **not** degrees:
`12idb:m6` reads in mm with 33 mm of travel, so one slot is `1.0`. A `step`
whose slot positions fall outside the motor's soft limits makes every slot but
the reference unreachable -- see [Soft limits](#the-carousel).
`carousel.draw_offset_steps` (4) is how many slots `make_sample` advances
between mixing a vial and drawing from it.

`robot.ur12idb_path` holds one path per operating system, so the same config
serves the beamline Linux host and the Windows control machine:

```json
"ur12idb_path": {
    "linux":   "/home/beams15/S12STAFF/python_codes/UR_12idb",
    "windows": "C:/Users/s12idb/Documents/GitHub/UR_12idb"
}
```

A list of candidate paths, or a single path, works too. The first entry that
exists is used, preferring this OS's; if none does, a `UR_12idb` checkout
sitting next to this repo is picked up, so a fresh clone elsewhere needs no
edit to the shared JSON.

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
| `release_gripper` | worker | Open the gripper where the arm stands. Refused while busy, so it cannot be pressed mid-carry; a flowcell the tracking says was in the gripper is set to `unknown`, since after this it is wherever it fell. The Experiment tab's **Release Gripper** button, behind a confirmation |
| `unlock_stop` | fast | Clear the robot's protective stop (`robUR.unlock_stop`). Fast and ungated by the busy flag: a protective stop happens *during* a move, and the action it interrupted may still be holding the server busy. It reaches the robot over the dashboard socket, not the motion one, so a wedged move does not block it. Reads the state back and says whether the stop actually cleared. The Experiment tab's **Unlock Protective Stop** button, which stays enabled while BUSY |
| `goto_transfer_point` | worker | Move to the fixed corridor pose every cross-cell leg routes through. Not a taught position, so it needs no configuration and is never refused as unconfigured -- the Configuration tab's **Transfer Point** button, next to the per-station Go To buttons |
| `push_positions` / `pull_positions` | worker | Sync taught positions with the EPICS waypoint PVs (see below) |
| `mixer2cleaningstation` etc. (8 names) | worker | Run the matching `PAL12idb` transport function |
| `motor_tweak forward\|reverse <step>` | worker | Tweak `12idb:m6` by `step` |
| `pump mix\|clean_mixer\|draw_to_flowcell\|aspirate_from_flowcell\|wash_flowcell\|shake_sample` | worker | Run a pump operation (ZMQ to apssector12_pump_control). `shake_sample` cycles the sample inside the flowcell in use -- `flow2_sample` for flowcell 1, `flow3_sample` for flowcell 2, since that server names the device in the command rather than as an argument. The Automation tab's **Shake Sample** button; the other five are the Experiment tab's Pump row |
| `stop_pump` | fast | End an active `shake_sample` cycle now (sends the flowcell server's `stop_shaking`, not its plainer `stop` -- see [Pump concurrency](#pump-concurrency)). Fast and ungated by the busy flag, because the point is to reach a pump operation that is running -- exactly when a worker command would be refused; `Pump` sends it on a socket the running op is not holding. Does nothing to an unrelated draw/wash/aspirate. The Automation tab's **Stop Shake** button, which stays enabled while the server is BUSY |
| `set_mixing_speed <rpm>` | fast | The speed the next `mix` runs at. Bounds-checked against `pump.min_rpm`/`max_rpm`; refused while an action is running |
| `get_mixing_speed` | fast | That speed, in rpm |
| `mount_carousel <id> [base64 {slot: sample_id}]` | fast | Declare which carousel is in the machine and everything on it, in **one** transition |
| `get_carousel_id` | fast | The mounted carousel's ID, or `unknown` |
| `make_sample <slot> [sample id]` | worker | Full mix-and-load sequence (see below), tagging `<slot>` with the sample ID -- a timestamp ID is assigned if none is given |
| `draw_and_load` | worker | Draw an already-mixed vial into the flowcell and put it in the beam (see below) -- `make_sample` without the mixing, so no slot is consumed. Does **not** rotate the carousel |
| `draw_load_sample <slot> [sample id]` | worker | Like `draw_and_load`, but rotates the carousel to `<slot>`'s drawing position first (see below) -- for a specific, already-prepared slot |
| `unload_sample [aspirate]` | worker | Take the flowcell off the beam and wash it (see below). `aspirate` recovers the sample into its vial first; without it the sample is discarded with the wash |
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

The `PAL12idb` transport functions above, each sent as its own bare command:

| Command | Move |
|---|---|
| `mixer2cleaningstation` | Mixer -> Cleaning Station |
| `mixer2mixingstation` | Cleaning Station -> Mixer |
| `load_flowcell_from_cleaningstation_to_beam` | Cleaning Station -> Beam |
| `load_flowcell_from_beam_to_cleaningstation` | Beam -> Cleaning Station |
| `ready_flowcell_to_draw` | Ready Flowcell to Draw (onto the sample seat) |
| `load_sample_to_beam` | Load Sample to Beam |
| `return_sample` | Return Sample from Beam |
| `wash_flowcell_after_return` | Wash Flowcell After Return |
| `flowcell_to_sample_on_mixer` | Flowcell -> Sample on Mixer (hold) |

Each acts on the flowcell currently in use (`set_flowcell`) and records its
own target location on success (see [State tracking](#state-tracking)).

`flowcell_to_sample_on_mixer` is the odd one out: it picks the flowcell up
from its cleaning station and **holds** it over the sample seat on the mixer
station instead of setting it down, so the seat can be taught (see
[Sample on Mixer Station](#sample-on-mixer-station)).

`ready_flowcell_to_draw` and `return_sample` both work at that **sample seat**,
not at the mixer station. The mixer head comes down where `rotate_carousel`
leaves the carousel; the flowcell reaches the vial `draw_offset_steps` round
from there, which is what the seat is. `make_sample` turns the vial it just
mixed round to the seat before the draw, and `unload_sample aspirate` pushes
the sample back into that same vial, so both have to aim at the same place.

Three seats are set down by feel rather than at a computed release height
(`PAL12idb.BUMP_RELEASE_STATIONS`): the mixer cleaning station, the sample seat
beside it, and the **flowcell cleaning station**. How deep the piece sits there
depends on how it is held, so the seat is found rather than computed.

The descent is in two parts (`PAL12idb.dropdown_by_bump`): an ordinary move
down to `bump_approach_clearance` (0.03 m) below the taught pose, then a bump
for the rest -- touch, back off 2 mm, open, lift clear. A bump has to creep in
order to read contact, so feeling out the whole carry clearance took most of a
minute over travel that is known empty air; this leaves it the last ~25 mm,
where the seat actually is. The fast part is open-loop but no more so than
`dropdown_at`, which drives blind all the way to the release height. It only
ever moves **down**, and skips itself entirely if the arm's pose cannot be read
-- the bump then does the whole descent, slow but correct.

The **sample table** keeps its computed release Z, deliberately. Out there a
bump reads contact off anything the gripper brushes on the way in, and
releasing high drops the flowcell onto the stage. What makes the difference is
the approach: the three bump seats are come down on vertically from directly
overhead, so there is nothing to brush.

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
actually runs, not a hand-kept list, and it follows both the flags it was
given and the current tracked state: `unload_sample` needs no mixer position
at all unless `aspirate` was asked for, and even then it only needs the mixer
cleaning station when the mixer head is sitting at the mixer station and
therefore has to be parked out of the way first.

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
| 6 | Sample on mixer station | `sample_on_mixer_station` |

### Sample table alignment with the DAQ GUI

The sample table is where the beam is, and the beam's actual position there is
set by a separate control system -- the beamline's own DAQ GUI, driving its
sample-stage motors (`sth`, `sav`, `stv`) -- not by anything the robot knows.
`palmixer/daq_client.py` is a small ZMQ client to that GUI
(`json/palmixer_config.json`'s `daq` section: `host`, `port`,
`request_timeout_s`; env overrides `PALMIXER_DAQ_HOST` / `PALMIXER_DAQ_PORT`),
and PAL12idb.py uses it to keep the two systems from drifting apart:

- **Recording.** Every time the robot's `sample_table` waypoint is (re)taught
  -- searching its AprilTag, teaching it by hand, or replacing just its
  orientation -- the DAQ motors' current positions are read and saved to
  `palmixer/daq_positions.ini` (git-ignored, like `waypoints.ini`). Best
  effort: a DAQ GUI that happens to be unreachable at that moment prints a
  warning but does not stop the robot position from being taught.
- **Alignment.** Every transport that moves the flowcell onto or off of the
  sample table -- `load_flowcell_from_cleaningstation_to_beam`,
  `load_flowcell_from_beam_to_cleaningstation`, `load_sample_to_beam`,
  `return_sample` -- calls `daq_client.align_sample_table()` as its very first
  step, before the robot moves at all. It sends the DAQ motors back to the
  recorded positions and waits for them to arrive, so a stage nudged by
  something else in between is put back before the robot approaches, and a
  slow DAQ move overlaps with nothing (this runs first, not concurrently).
- **Restoring.** `load_flowcell_from_beam_to_cleaningstation` -- the one that
  takes the flowcell *off* the sample table -- reads the DAQ motors' current
  positions before it aligns anything, and once the flowcell has actually
  reached its cleaning station, sends the stage back to that reading. The beam
  is not looking at the sample table once the flowcell has left it, so nothing
  is served by leaving the stage parked at the sample-table alignment
  indefinitely. Best-effort in the other direction from recording: the robot's
  own work is already done and successful by that point, so a DAQ failure here
  is printed rather than raised -- it must not turn an actually-successful
  transport into one that looks failed, or mark the flowcell's location
  `unknown` when it is not.

If `sample_table` has never been taught with the DAQ GUI reachable,
`align_sample_table()` raises before any robot motion -- the sample table
positions are recorded but the DAQ ones are not, so nothing has run to capture
them yet. Search (or teach) `sample_table` once with `daq_client` able to
reach the GUI to fix this.

### Sample on Mixer Station

The sample seat beside the mixer cleaning station. Unlike the other five it
carries **no AprilTag**, so `search_apriltag` cannot teach it and it is not
one of that command's stations. It is taught by hand instead:

1. **Experiment tab -> Flowcell -> Sample on Mixer (hold)**
   (`flowcell_to_sample_on_mixer`). The flowcell is picked up from its
   cleaning station and carried to the seat, and the arm stops holding it
   there -- at the seat's reference pose, so the flowcell hangs
   `distance_gripper_tag + grab_depth` (60 mm) clear of it.
2. Jog the arm until the flowcell sits over the seat the way you want it.
3. **Configuration tab -> Set Current Robot Position As -> Sample on Mixer
   Station** to record it. Use **Save Orientation** instead if you only mean
   to replace RX/RY/RZ and keep the taught X/Y/Z -- that button does not
   record position.

Until it has been taught, the target is the mixer cleaning station's taught
pose shifted by `PAL12idb.sample_on_mixer_offset` (-0.0395 m X, -0.0894 m Y,
+0.0202 m Z -- measured from a seat found by hand), which is what makes step 1
land on the seat rather than merely near it. That
fallback is also why the position pre-check accepts the station as configured
as long as the mixer cleaning station is: refusing the very move that exists
to get the arm close enough to teach it would be circular. With neither
taught, the error names the mixer cleaning station -- the one to go and
search for.

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
it itself). Each `PAL12idb` transport function records its own
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

**Soft limits.** Every slot position, and the post-mix advance, must be inside
the motor's `.LLM`/`.HLM`. The EPICS motor record does **not** report a refusal
in any way a client would otherwise notice -- a `.VAL` outside its limits is
simply not acted on, `.DMOV` stays 1, and the move appears to succeed instantly
with the motor exactly where it started. `Motor.check_in_range` therefore
refuses out-of-range targets itself, with `.LVIO` checked after the write as a
backstop, and `rotate_carousel` checks the *advance* target before the mix so an
unreachable advance costs no vial. If `make_sample` reports "outside the soft
limits", either widen `.LLM`/`.HLM` or fix `carousel.step`.

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
on a background thread, not awaited) -> advance carousel -> ready_flowcell_to_draw
-> pump draw_to_flowcell -> load_sample_to_beam -> shake_sample (auto, not
awaited) -> park at the transfer point. Requires the flowcell in use to be at
its cleaning station, the slot to be within the configured carousel size, and
the carousel to have been located (one taught reference). Also refused
outright while a previous run's mixer clean is still going -- see
[Mixer wash safety](#mixer-wash-safety).

The mixer clean (**5555**) and the flowcell draw (**5556**) are on independent
pumps and **run at the same time** -- the draw does not wait for the clean. See
[Pump concurrency](#pump-concurrency).

The **advance** is `carousel.draw_offset_steps` (4) steps forward of the
position the vial was mixed at -- `draw_offset_steps x carousel.step` in motor
units, so it follows the slot spacing rather than being fixed in code. The
mixer head comes down where `rotate_carousel` left the carousel, but the
flowcell draws from a slot further on, so the vial has to come round to it. It
runs once the head is
parked and washing -- the carousel cannot turn under the head -- and unlike the
wash it **is** awaited: the flowcell descends onto whatever slot is underneath
it, so the next step must not start until the motor has stopped. The move is
absolute (`mixed position + 4 x carousel.step`), like `rotate_carousel`, so
rounding cannot accumulate across runs. The tracked carousel slot follows the
motion and wraps around the ring, so it keeps meaning "the slot the carousel is
turned to" rather than "the slot last mixed".

**`draw_and_load`**: mixer2cleaningstation (if needed) ->
ready_flowcell_to_draw -> pump draw_to_flowcell -> load_sample_to_beam ->
shake_sample (auto, not awaited) -> park at the transfer point. The tail of
`make_sample` without the mixing, for a vial that already holds what is
wanted -- one mixed by an earlier run, or a sample just recovered by
`unload_sample aspirate`. Requires the flowcell in use to be at the cleaning
station and the mixer head's location to be known. Nothing here touches the
carousel: no slot is consumed and no sample ID is assigned, so the vial drawn
from is whichever one the last rotation left the mixer over. **This does not
rotate the carousel** -- if you want a specific slot, use `draw_load_sample`
instead; pressing this without having first positioned the carousel there
draws from whatever vial happens to already be at the draw point, silently.

**`draw_load_sample <slot> [sample id]`**: advance carousel (to `<slot>`'s
*drawing* position -- `draw_offset_steps` forward of its mixing position, same
as `make_sample`'s advance) -> mixer2cleaningstation (if needed) ->
ready_flowcell_to_draw -> pump draw_to_flowcell -> load_sample_to_beam ->
shake_sample (auto, not awaited) -> park at the transfer point. `draw_and_load`'s
slot-aware counterpart, for a named,
already-prepared slot rather than whatever the carousel already happens to be
sitting on. The rotation runs *before* `ready_flowcell_to_draw`, deliberately:
that step lowers the flowcell onto and bumps whatever is currently under the
seat, so the requested slot has to already be there. Requires the flowcell in
use to be at the cleaning station and the mixer head's location to be known,
same as `draw_and_load`. Nothing here touches the carousel's slot inventory
either: no slot is consumed, and `sample id` is a label on the completion
detail only, not recorded against the slot. The Automation tab's **Draw and
Load Slot** button, next to **Draw and Load**, using the same Slot/Sample ID
boxes as **Make a Sample**.

**`unload_sample`**: load_flowcell_from_beam_to_cleaningstation -> pump
wash_flowcell -> raise the robot back to sample-table height -> park at the
transfer point. Requires the flowcell in use to be at the sample table. The
wash is **not awaited**: it needs the flowcell sitting in the station rather
than the arm that put it there, so the lift runs alongside it and the sequence
returns without waiting. The next call to the **same** pump waits for it in
its turn -- the `draw_to_flowcell` of a following `make_sample`, for instance.
The `mix` of that run does not: that is the other pump.

**Parking.** Every workflow's last step, `park_at_transfer_point`, sends the
arm to the fixed corridor pose (`goto_transfer_point`) once everything before
it has finished cleanly -- see [Transport functions](#transport-functions) for
what that pose is. It leaves the arm somewhere known and out of the way for
the *next* command, from either side of the cell, instead of wherever the
workflow's last real step happened to leave it. It only runs on a clean
finish: an exception from any step above it propagates instead, since an arm
that stopped mid-sequence may still be holding the flowcell or mid-descent,
and driving it to the corridor is not obviously safe then -- the same
reasoning that already leaves every other mid-workflow failure for the
operator to look at rather than trying to recover from automatically.

### Auto-shake

`make_sample`, `draw_and_load`, and `draw_load_sample` all fire
`Workflows._start_auto_shake()` right after `load_sample_to_beam`, not
awaited -- parking, and whatever the operator does next, runs while the
sample is agitated at the beam. Unlike every other pump op used here, a shake
is not one bounded call: `flow2_sample`/`flow3_sample` runs a single fixed
recipe and then stops, so "keep shaking" means re-firing it in a loop, on its
own daemon thread, for as long as the sample sits at the beam.

The loop is **pinned** to the flowcell that was actually just loaded
(`Pump.shake_sample`'s `flowcell_id`), read once when the loop starts --
switching the "Flowcell in Use" selector afterward, to work on the other
flowcell, does not retarget an already-running shake.

`Workflows.stop_auto_shake()` ends it, called from two places:

- The start of `unload_sample`, either sequence -- before its own first real
  step, since aspirate's `return_sample` picks the flowcell straight up and a
  shake still running then would jostle it during exactly that.
- `server.py`'s `_run_transport`, specifically when
  `load_flowcell_from_beam_to_cleaningstation` runs on its own (the
  Experiment tab's transport button) rather than through `unload_sample`.

Either way, once the flowcell is leaving the sample table there is nothing
left to agitate. `stop_auto_shake()` also reaches into a shake that a plain
press of the Automation tab's **Shake Sample** button started, not only one
the loop itself began -- it calls `Pump.stop_shaking()` unconditionally,
which is what actually interrupts whatever cycle happens to be running right
now rather than waiting for it to finish on its own; the loop only checks
whether to start *another* cycle in between. Starting a second auto-shake
(loading a different sample while an old one is somehow still shaking) stops
the first rather than letting two loops fight over the same hardware -- only
one flowcell can physically be mid-shake at a time regardless, since the
flowcell dashboard serializes both its pumps through one recipe thread.

### Mixer wash safety

`clean_mixer` washes the mixer head by running liquid through it while it sits
docked at the cleaning station. It is fired without being awaited (`make_sample`
starts it on a background thread and moves straight on), so it can still be
running well after `make_sample` has returned and the server has gone idle
again -- nothing else waits for it to finish on its own the way a same-server
pump op does. Moving the head off the station while that is happening risks
spilling the liquid actively flowing through it, damaging the tubing, or
fouling the needle alignment.

`Workflows.mixer_head_busy()` answers whether a `clean_mixer` is still
running, checked fresh every time rather than cached, and two things refuse
while it is:

- **Moving the mixer head at all** -- `mixer2cleaningstation` and
  `mixer2mixingstation`, whichever way they are reached: the Experiment tab's
  own buttons for either (refused outright by the server, before ACCEPTED),
  or a step inside `make_sample`, `draw_and_load`, or `unload_sample aspirate`
  (refused with a `WorkflowError` at the point that step would have run --
  `Workflows._move_mixer_head` is the one place every internal call to either
  transport goes through).
- **Starting `make_sample`** -- refused outright by `_check_make_sample`, the
  same synchronous pre-flight path as its other guards. The realistic case is
  a second `make_sample` fired before the first one's wash has finished.

### Pump status panel

Both the Experiment and Automation tabs carry a **Pumps** panel listing all
four physical pumps -- `pump0` and `pump1` on the mixing server, `flowcell2`
and `flowcell3` on the flowcell server -- with each one's status and plunger
position, plus a summary line naming any dashboard that is unreachable or whose
pumps are not connected.

It rides on the ordinary `get_state` snapshot as `pump_status`, filled by a
background thread in the server that polls both dashboards' ZMQ `status` every
2 s (`PUMP_STATUS_INTERVAL_S`). Polled on a thread rather than fetched when
asked, because `get_state` is answered on the ZMQ reply thread: two round trips
to the dashboards there would stall every other command behind them, and a
dashboard that is down would stall them for its whole timeout. The poll uses
its own sockets inside `Pump`, separate from the operation ones, so it keeps
reporting *during* an operation -- which is when it is worth reading -- and
never waits on one. Status reads also get a shorter timeout than operations
(`pump.STATUS_TIMEOUT_S`, 2 s): a slow status read is a stale panel, not a
failed move.

The two servers report quite differently -- the flowcell one has a per-pump
list, the mixing one has parallel arrays plus a single multi-line `operation`
string shared by both its pumps -- so `Pump.status_snapshot()` flattens them
into one row per pump and the GUI only draws rows.

### Pump concurrency

There are two pump servers, and they are independent hardware:

| Port | Dashboard | Operations |
|---|---|---|
| 5555 | `pump_dashboard.py` | `mix`, `clean_mixer` |
| 5556 | `flowcell_dashboard.py` | `draw_to_flowcell`, `aspirate_from_flowcell`, `wash_flowcell` |

`Pump` holds **one lock per server**, on the endpoint itself. Within a server
everything serializes -- the ZMQ REQ socket cannot be shared and neither can the
pump behind it. Across the two, nothing does: cleaning the mixer and drawing
into the flowcell happen at the same time, which is the whole reason
`make_sample` fires `clean_mixer` on a background thread.

Where a workflow *does* have to wait for a not-awaited op, it waits as an
explicit `wait_for_<op>` step rather than stalling inside the next one, and only
when the two ops share a server (`pump.server_for`). A step that sits for
minutes with no command sent is otherwise indistinguishable from one aimed at
the wrong pump server.

**Stopping a shake.** The flowcell server has two different stop commands, and
`Pump.stop_shaking()` (the Automation tab's **Stop Shake** button, via
`stop_pump`) deliberately sends the narrower one: `stop_shaking`, which only
ends an active `flow2_sample`/`flow3_sample` cycle and reports
`accepted: false` if the running action is not one. Its plain `stop` finishes
*whatever* recipe happens to be active -- a draw, a wash, an aspirate -- which
is not what a button labelled "Stop Shake" should be able to reach into and
interrupt. (There is a third, `emergency_stop`, for an immediate hardware halt
of both pumps; nothing in PALmixer sends it.)

**`unload_sample aspirate`**: mixer2cleaningstation (if needed) ->
return_sample -> pump aspirate_from_flowcell -> wash_flowcell_after_return ->
pump wash_flowcell -> raise the robot back to sample-table height. The
sequence this workflow always ran before the flag existed: it recovers the
sample into the vial it was mixed in instead of washing it away, at the cost
of three extra legs and a pump operation. The Automation tab exposes it as an
"Aspirate back to mixer" checkbox next to the Unload Sample button, unticked
by default.

## Pump driver

`palmixer/pump.py` is a **ZMQ client** to
[apssector12_pump_control](../apssector12_pump_control), which runs the real
TriContinent syringe pumps behind two loopback-only ZMQ REP servers. It
implements the five operations the GUI and workflows call, routing each to the
right server:

| PALmixer op | Server | Endpoint | ZMQ command |
|---|---|---|---|
| `mix` | mixing | `tcp://127.0.0.1:5555` | `sample_make` |
| `clean_mixer` | mixing | `tcp://127.0.0.1:5555` | `clean_all` |
| `draw_to_flowcell` | flowcell | `tcp://127.0.0.1:5556` | `draw_to_flowcell` |
| `aspirate_from_flowcell` | flowcell | `tcp://127.0.0.1:5556` | `aspirate_from_flowcell` |
| `wash_flowcell` | flowcell | `tcp://127.0.0.1:5556` | `wash_flowcell` |

All five stay serialized on a single lock so two pump ops never run
concurrently, and each keeps the `(ok, detail)` contract, so `server.py`,
`workflows.py`, and the GUI are untouched.

**Both pump dashboards must be running first.** Launch `PumpControl.cmd` and
`FlowCellControl.cmd`, then in each one connect, confirm syringe sizes, and
initialize -- the dashboards own the COM ports and those safety confirmations,
and they are the only gate on readiness. PALmixer does not pre-check them: it
opens no socket until the first op, and an unready pump simply replies
`ok:false`, which surfaces as a failed op / `WorkflowError`. A ZMQ reply means
the op is **queued**, not done: each op then polls `status` until
`operation_active` goes false before returning.

**Flowcell selection is by device index.** The flowcell ops carry a 0-based
`device` derived from `state.get_flowcell_in_use()`: PALmixer flowcell **ID 1**
maps to flowcell2 / `device 0` (`flow2_sample`), and **ID 2** to flowcell3 /
`device 1` (`flow3_sample`). Use `set_flowcell` to switch which one the ops act
on.

**Mixing speed is advisory.** `set_mixing_speed`/`get_mixing_speed` still read
and write a value, the mix's completion detail names it, and out-of-range is
**refused, not clamped** -- but the *pump dashboard* owns the real mix speed
(`sample_make` uses the dashboard's volumes/speeds and rejects overrides), so
this value is a local record folded into the sample record, not a command sent
to the pump.

Endpoints, timeouts, and the advisory speed limits live in
`json/palmixer_config.json`:

```json
"pump": {
    "mixing_speed_rpm": 800.0, "min_rpm": 0.0, "max_rpm": 3000.0,
    "host": "127.0.0.1", "mixer_port": 5555, "flowcell_port": 5556,
    "request_timeout_s": 5.0, "poll_interval_s": 0.5, "operation_timeout_s": 600.0
}
```

They can be overridden with `PALMIXER_PUMP_HOST`, `PALMIXER_PUMP_MIXER_PORT`,
and `PALMIXER_PUMP_FLOWCELL_PORT`. These have **pump-specific names on purpose**:
the pump dashboards themselves read `PALMIXER_ZMQ_PORT` / `PALMIXER_MQTT_PORT`
(the same env names PALmixer uses for its own control plane), so the pump client
is given its own to avoid the collision.
