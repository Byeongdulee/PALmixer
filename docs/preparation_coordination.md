# PAL robot preparation: internal coordination

## Update: carousel replacement (`replace_carousel`)

When the carousel is exhausted, the owning run sends:

```json
{"version": 1, "run_id": "<run UUID>", "op": "replace_carousel",
 "replacement_id": "<canonical UUID>", "action_key": "replace:<replacement_id>"}
```

It runs the **Unload Current Carousel** code path and then the
**Place New Carousel** code path as one worker action labelled
`preparation_replace_carousel`. With robot placement on, the table first turns to
`carousel.place_alignment_slot`, with the old carousel still on it. A swap that
ends without a failure parks the arm with the standard workflow park
(`park_at_transfer_point`) before it reports that state. Placement saves the carousel-load record, with
this `replacement_id`, and frees every slot, as the button does. The carousel ID
label and taught positions are unchanged, and `mount_carousel` is not called.

It needs the same gate as the other motion operations: the owning run, no
session error, and a coordinator. It is refused before acceptance unless all of
these hold:

- The coordinator is idle (`require_idle`).
- PALmixer is not busy.
- Cleanup reports no `problem`. A running flowcell wash is allowed: the swap is
  meant to overlap it.
- The mixer head and the flowcell in use are at their cleaning stations.
- With robot placement on (`carousel.auto_place_new_carousel`), the carousel
  table can be turned to `carousel.place_alignment_slot`: the slot positions
  are taught, and every flowcell, not only the one in use, is at its cleaning
  station or the beam. A flowcell PALmixer has never tracked reads `unknown` and
  is refused too; declare it with `set_location` (for example
  `set_location flowcell_2 cleaning_station`) while no session is open.
- Carousel on Mixer, Used Carousel Stack and Carousel in Stock are taught.

The record is kept in `replacements[replacement_id]` and returned by `status`:
`{replacement_id, action_id, request, state, error}`, plus `load` once placed,
`detail` (placement's own message) when the stock seat was empty, and
`failed_phase` on failure. It is persisted when the request is accepted and again
before each physical phase. A repeated identical request returns the record and
never starts a second swap. A different request under the same identity is refused.

The states run `accepted`, `unloading`, `old_removed`, `placing`, `parking`, then
`committed` or `waiting_for_stock`. With robot placement off they run `accepted`,
`unloading`, `old_removed`, `parking`, `waiting_for_carousel`.

| `state` | Meaning |
| --- | --- |
| `accepted` | Persisted; no motion yet |
| `unloading` | Unload running; with robot placement on, the table first turns to the alignment slot |
| `old_removed` | Old carousel is on the used stack |
| `placing` | Placement running: the arm places, and the height check confirms the carousel landed (a carousel felt too high is taken back to the used stack once and a fresh one placed) |
| `parking` | The standard park is moving the arm to the transfer point: after a placement (`load` is already recorded), after finding the stock seat empty (`detail`), or, with robot placement off, after the unload |
| `committed` | Placed; `load` holds the carousel-load record and every slot is free. The arm is parked, also after a hand placement that **Replace Carousel (Reset)** commits (`placed_by` `operator`): the swap parked before it waited, and Reset moves nothing |
| `waiting_for_stock` | Stock seat empty after the unload, or after a placed carousel that did not land was taken back to the used stack. The arm is parked. `error` tells the operator what to do; `detail` is the placement's own message, which names a carousel that was taken back. The session stays usable so PALsystem can end it. The operator loads a carousel and presses **Place New Carousel** on PALmixer, which records the load. |
| `waiting_for_carousel` | Robot placement is off (`carousel.auto_place_new_carousel` `false`). The arm is parked. The operator seats a new carousel by hand and presses **Replace Carousel (Reset)**, which records the load and marks the replacement `committed`. |
| `failed` | Any other error. `failed_phase` says whether unloading, placing or parking failed; a park that fails after a placement keeps `load`, because the carousel load is already recorded. A failed park's `error` first says what the swap had done, the placement (including any carousel taken back) or the empty stock seat, then gives the cause. A failure is never followed by a park. The session error is latched and the coordinator is invalidated, as for other motion operations, with no replay. |

Every worker action clears `cleanup.parked` when it starts, and only the standard
park sets it again. A swap that ends without a failure (`committed`,
`waiting_for_stock` or `waiting_for_carousel`) therefore parks before it reports
that state: by the time PALsystem sees the final state and PALmixer idle,
`cleanup.parked` is true, and the last sample's unload, which waits for
`wash_complete` and `parked`, can finish. The preparation snapshot in `get_state`
advertises this as `replace_parks_version: 1`. The action's `last_result.detail`
says what happened: the unload and placement details, including any carousel taken
back, then `arm parked`. On `waiting_for_stock` and `waiting_for_carousel` it ends
with the operator's instruction, because the session error stays empty.

## Update: version-1 live test endpoint

The server now creates `PreparationAPI`, inactive until PALsystem's real test sends
`preparation_control {"version":1,"run_id":"UUID","op":"begin"}`. Operations are
`queue`, `initial`, `status`, `release`, `tag`, `make`, `unload`, `replace_carousel`,
`end`, `inhibit`.
Motion operations have durable `action_key`/`action_id` records and reuse existing
workflows. During ownership, conflicting ordinary commands are refused. A restart
with an active session blocks further motion for inspection; there is no replay.

The JSON configuration now sets preparation offset -4; drawing stays +4. Readiness
is persisted after motor completion and relayed by PALsystem to ChemLab. Release
uses ChemLab's final aqueous-dispense milestone: existing tip disposal/withdrawal,
then two successful 0.5 s Beep activities, matched to UUID and placement. The
existing UR lift call, return-value handling and subsequent movements are unchanged.

This wiring is used by PALsystem's **Test 3 Sample Cycle** controller. Ordinary
campaign adoption and operator reconciliation remain separate follow-up work.
The earlier internal-only notes below describe the preceding implementation stage.

Implemented 22 October 2026. This is an opt-in Python component for the later
PALsystem/PAL_ChemLab adapter. The server does not instantiate it and no new
network commands or live configuration switch are enabled yet.

## Responsibilities and order

`PreparationCoordinator` owns at most one pending/positioning/dispensing placement.
PALsystem supplies the sample UUID, actual mounted carousel identity and reserved
holder. `order_id` equals `sample_uid`; `placement_id` identifies one positioning
request. A next placement names the previous sample in `release_after_sample_uid`.

1. Queue an initial placement. `Workflows.prepare_initial` runs the existing
   preflight and mixer-head removal when needed, then opens initial positioning.
2. The coordinator's worker invokes `position_for_preparation` and waits for the
   existing motor completion before invoking its readiness callback.
3. Release requires matching sample/placement IDs, a completion reference, and
   explicit preparation-complete and PAL-tool-clear results. The production
   source of qualified PAL clearance remains unresolved; no extra motion is added.
4. The opt-in `make_sample` binds the released sample, holder, mounted carousel
   and workflow action identity. It runs the existing sample sequence.
5. Immediately after `robot.mvr2z(needle_clear_height)` returns, the hook opens
   a one-use opportunity for the queued next sample. The return value is ignored,
   including `None` or `False`; an exception prevents notification and propagates.
6. The worker persists the dispatch before moving the next holder. UR continues
   its existing transfer, beam placement and parking. A coordination failure
   inhibits further dispatch but does not interrupt that UR route.

`FC_AT_BEAM` still means the whole beam-transfer function completed. Readiness is
acknowledged separately from movement, so `retry_ready` only retries delivery.
The preparation remains owned even after the preceding `make_sample` returns.
Next-sample mixing still needs the existing flowcell-at-cleaning precondition.

## Internal setup example (fakes only; not a server command)

```python
coordinator = PreparationCoordinator(journal_path, fake_move, fake_notify)
coordinator.queue({
    "placement_id": placement_uuid,
    "sample_uid": sample_uuid,
    "order_id": sample_uuid,
    "carousel_id": mounted_carousel_id,
    "slot": assigned_holder,
    "release_after_sample_uid": preceding_sample_uuid_or_none,
})
```

`move` must await the existing motor completion. `notify` must acknowledge the
same order and placement, or raise. Neither callback may control UR.

## Geometry and configuration

The motor callback requires an explicit integer `offset_steps`, with no default:

`target = slot_position(slot) + offset_steps * require_carousel_step()`

The proposed preparation offset is **−4**, opposite the existing drawing offset
**+4**. Confirm the installation's direction/reachability before wiring it into
production. There is no shortest-path/wrap guess. Existing motor range checks
apply. A drawn slot remains occupied; rotation never clears sample tags.

## Recovery and integration work still required

The version-1 JSON journal is replaced atomically. Restart marks nonterminal
placements uncertain and invalidates lift opportunities; an interrupted sample
cycle also blocks new dispatch. There is no automatic movement replay or implicit
recovery. Only a pending placement can be cancelled. Qualified completion releases
ownership; it does not clear a vial or replay mixing.

Keep the default `Workflows(..., preparation=None)` for legacy paths. Return-to-vial
unloading is refused in an opted-in session and remains available in the legacy
workflow. Before live enablement, the production adapter must connect queue/status/
release operations, route **all** conflicting server/GUI/manual carousel actions
through the coordinator, persist correlated lifecycle events, reconcile failures,
and qualify PAL completion. Existing command-string parsing and generic busy
handling have not been changed. Do not attach a coordinator to a live server until
those guards and the production JSON mapping are implemented.

Offline tests: `python -m pytest -q tests/test_preparation_coordination.py`.
The cross-stack fake cycle is in PALsystem's `tests/test_preparation_stack.py`.
