# Automatic search for the PULSE table: plan

Status: plan for review. Nothing is built yet. Written 2026-11-12.

The code calls this table the **Venton table** (`calibrate_venton_table`,
`json/venton_frame.json`). This plan calls it the PULSE table.

## Why

The PULSE table carries the mixer station, the mixer cleaning station, both
flowcell cleaning stations and the carousel on the mixer. They are fixed to the
table, so when the table moves they all move together, by the same amount. On
2026-11-12 the table moved about 8 cm and every station on it had to be
re-taught. A stored value that was missed sent the arm down onto the flowcell
washing station during **Place New Carousel**.

This adds one button on the Automation tab. It finds where the table is now,
works out where each station should be, searches each station's tag to confirm
it, and stores the new positions.

## Decisions so far

| Question | Decision |
|---|---|
| How the table is found | From its own reference tags (11, 12 and 13), not from the carousel |
| Close-and-bump at the end of each station search | OK at every station |
| Placement refusing on a mismatched seat | Never (already removed) |
| Mixer carousel's measured seat | Discarded whenever Carousel on Mixer is re-taught (already built) |

## What it covers

| Station | Tag the search uses | The tag is only there when |
|---|---|---|
| Carousel on Mixer | The seated carousel's tag (31–40) | A carousel is seated on the mixer |
| Mixer Station | Mixer head, tag 25 | The mixer head is at the mixer station |
| Mixer Cleaning Station | Mixer head, tag 25 | The mixer head is at its cleaning station |
| Flowcell cleaning station 1 | Flowcell 1, tag 1 | Flowcell 1 is parked there |
| Flowcell cleaning station 2 | Flowcell 2, tag 2 | Flowcell 2 is parked there |
| Sample on Mixer Station | None | Recomputed from the mixer station (step 5) |

The tags are on the parts, not on the table, so a station can only be searched
while its part is there. PALmixer already tracks where the mixer head, each
flowcell and the mixer carousel are, and the search reads that to decide which
stations it can search. A location tracked as `unknown` counts as not there.

Not touched, because they do not move with the table: Carousel in Stock, Used
Carousel Stack, Sample Table and the transfer point.

## Sequence

### 0. Checks before anything moves

- PALmixer is idle, the robot is in Remote mode, and there is no protective
  stop.
- A saved table frame exists (`json/venton_frame.json`) and every station above
  has a taught position.
- The gripper TCP is active.

### 1. Find the table from its reference tags

For each reference tag, 11 then 12 then 13:

1. Predict where the tag is: its hand-measured spot on the table
   (`apriltags.venton_table_*.table_xy_mm`), placed with the saved table frame.
   Once the first tag is found, the offset it showed is added to the predictions
   for the rest, so a large move still lands the camera over the next tag.
2. Travel at a safe height to above the prediction, then come down to **70 mm
   above it**.
3. Move the camera to where the gripper TCP is.
4. Measure the tag with the existing closed-loop `measure_venton_tag`: centre,
   descend, re-centre, and record its position in robot coordinates. That needs
   the tag near the image centre to start, which the prediction provides.
5. If the tag is not in view, go up 50 mm and look again. If it is still not
   seen, stop with an error that names the tag.

Then fit the new table frame from the three tags with the existing fit
(`palmixer/calibration.py`). The difference between the saved frame and the new
one is the table's move: a shift, and a rotation about vertical.

### 2. Predict every station

Each station's taught position is converted into table coordinates using the
saved frame, then back into robot coordinates using the new frame. Its heading
turns by the same angle as the table.

### 3. Search each station whose tag is there

For each such station, nearest first:

1. Travel at a safe height to above the predicted position, then come down to
   **70 mm above it**.
2. Move the camera to where the gripper TCP is.
3. Run the normal AprilTag search from there (`from_current`). Like teaching,
   it finishes by closing the gripper and bumping down onto the station to fix
   the height, then stores the position.
4. **Carousel on Mixer only:** store the position from the search, but take the
   heading from the table (the old heading turned by the table's rotation). The
   carousel's tag turns with the carousel, so its heading says nothing about the
   table.
5. Update the table's move from the stations measured so far, and use it to
   predict the stations still to come.

A station whose search fails is handled as decided in question 3.

### 4. Stations whose tag is not there

As decided in question 1: either store the predicted position, labelled
"predicted" in the report, or leave the station unchanged.

### 5. Update what depends on these positions

- **Sample on Mixer Station:** recomputed from the new mixer station by the
  existing rule when it is derived (the usual case). If it was tuned by hand,
  it is treated like a station with no tag.
- **Mixer carousel's measured seat:** discarded when Carousel on Mixer is stored
  (already built), so the next placement uses the new position.
- **Table frame:** the new frame is saved to `json/venton_frame.json`. It sets
  the direction of the carousel's slide in and out of its seat.

### 6. Park and report

The arm parks at the transfer point, using the existing swing-first route from
the carousel side. The reply and the server console list each station:

| Station | Old x, y, z (m) | New x, y, z (m) | How | Distance from prediction |
|---|---|---|---|---|
| Mixer Station | … | … | measured | 3 mm |
| Mixer Cleaning Station | … | … | predicted | — |

The report also gives the table's shift (mm), its rotation (degrees) and the
frame fit's quality. A measured station more than 10 mm from its prediction is
flagged: it may have moved on the table, or the search may have found the wrong
tag. It is reported, never refused.

## Safety

- Between stations the arm travels at or above the carousel safe Z (the height
  carousel moves already cross the cell at), and only then comes down to 70 mm
  above the next station. The mixer head, the flowcells and the carousel stand up
  between the stations.
- Moves around the base use the existing swing-first rules.
- The Stop button stops the run, wired the same way as for an AprilTag search.
  Stations already stored stay stored, the rest keep their old positions, and
  the report says which is which.
- It moves the robot only: no pumps and no carousel motor.

## One-time setup before first use

The saved table frame is from 2026-10-26, before today's move, but the stations
were re-taught today at the table's new position, so the two disagree. Before the
first search, measure the frame once with the table where it is now:

1. Jog the camera so all three reference tags are in view.
2. Run `calibrate_venton_table`. It saves the frame.

After that, every table search keeps the frame and the stations in step itself.

## Where it goes in the code

| Piece | File |
|---|---|
| The sequence (`search_pulse_table`) and the table-move math | `palmixer/PAL12idb.py` |
| New worker command `search_pulse_table` | `palmixer/commands.py`, `palmixer/server.py` |
| Automation tab button, behind a confirmation | `palmixer/gui/workflow_tab.py` |
| Settings: approach height (default 70 mm), travel height (default: carousel safe Z), flag distance (default 10 mm) | `json/palmixer_config.json`, defaults in `palmixer/config.py` |
| Tests with a fake robot and camera | `tests/test_pulse_table_search.py` |
| How it works and when to use it | `README.md` |

## Tests

- The table-move math: a known shift and rotation applied to the reference tags
  and the stations comes back exactly.
- Which stations get searched, from the tracked locations of the mixer head, the
  flowcells and the mixer carousel.
- The moves for one station, in order: safe height, down to 70 mm above it,
  camera to the TCP, search.
- Carousel on Mixer keeps a table-derived heading, not the tag's.
- A failed station search, and Stop part-way through, leave the right stations
  stored.
- The report lists old and new positions, how each was found and the distance
  from the prediction, and flags anything over 10 mm.

## Open questions

1. **Stations whose tag is not there.** Store the predicted position, or leave
   the station unchanged? Recommended: store it, labelled "predicted" in the
   report. It is computed from the stations measured in the same run (or, if
   none could be, from the reference tags).
2. **Carousel on Mixer heading, now.** Its heading changed from 0.2° this
   morning to 135.6° today, while every other table station kept its heading
   within about 1°. The new value came from the carousel's tag, which turns with
   the carousel. Placement lands at that heading plus 19°, so the next placement
   would come in turned about 135° from before. Restore this morning's heading
   and keep today's position?
3. **A station search that fails part-way.** Leave that station unchanged and
   carry on with the rest (recommended), or stop the whole run?
4. **Button name.** "Search PULSE Table"?
