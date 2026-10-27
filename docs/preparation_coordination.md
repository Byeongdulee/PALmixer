# PAL robot preparation: internal coordination

## Update: carousel replacement (`replace_carousel`)

When the carousel is exhausted, the owning run sends:

```json
{"version": 1, "run_id": "<run UUID>", "op": "replace_carousel",
 "replacement_id": "<canonical UUID>", "action_key": "replace:<replacement_id>"}
```

It runs the **Unload Current Carousel** code path and then the
**Place New Carousel** code path as one worker action labelled
`preparation_replace_carousel`. Placement saves the carousel-load record, with
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
- Carousel on Mixer, Used Carousel Stack and Carousel in Stock are taught.

The record is kept in `replacements[replacement_id]` and returned by `status`:
`{replacement_id, action_id, request, state, error}`, plus `load` once
committed and `failed_phase` on failure. It is persisted when the
request is accepted and again before each physical phase. A repeated identical
request returns the record and never starts a second swap. A different request
under the same identity is refused.

| `state` | Meaning |
| --- | --- |
| `accepted` | Persisted; no motion yet |
| `unloading` | Unload running |
| `old_removed` | Old carousel is on the used stack |
| `placing` | Placement running |
| `committed` | Placed; `load` holds the carousel-load record and every slot is free |
| `waiting_for_stock` | Stock seat empty after the unload. The session stays usable so PALsystem can end it. The operator loads a carousel and presses **Place New Carousel** on PALmixer, which records the load. |
| `failed` | Any other error. `failed_phase` says whether unloading or placing failed. The session error is latched and the coordinator is invalidated, as for other motion operations, with no replay. |

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
