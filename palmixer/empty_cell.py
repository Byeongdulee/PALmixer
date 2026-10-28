"""Empty-cell transport within the existing preparation ownership and motion journal."""
from copy import deepcopy
import json
import threading
from uuid import UUID, uuid4

from . import state


def active_inspection(api):
    return next((r for r in api.record.get('inspections', {}).values()
                 if r['stage'] != 'completed'), None)


def inspection_action(api, message):
    # PreparationAPI holds its lock throughout acceptance. The scheduler's hold
    # additionally blocks its independent carousel-positioning worker.
    op, inspection_id = message['op'], message.get('inspection_id')
    if not isinstance(inspection_id, str) or str(UUID(inspection_id)) != inspection_id:
        raise ValueError('Canonical inspection UUID required')
    key = op + ':' + inspection_id
    if message.get('action_key') != key:
        raise ValueError('Inspection action identity does not match')
    cell = message.get('flowcell')
    if cell not in ('ID1', 'ID2'):
        raise ValueError('Choose ID1 or ID2 for the empty-cell inspection')
    signature = json.dumps(message, sort_keys=True)
    rows = api.record.setdefault('inspections', {})
    inspection = rows.get(inspection_id)
    if op == 'inspection_load' and inspection and inspection['stage'] == 'completed':
        raise ValueError('This inspection already returned its cell; use a new inspection identity')
    previous = api.record['actions'].get(key)
    if previous:
        if previous['request'] != signature:
            raise ValueError('Inspection identity already has different contents')
        return deepcopy(previous)
    server, workflow = api.server, api.server.workflows
    placement = api.coordinator.get(message['placement_id'])
    binding = placement['binding']
    if (binding['sample_uid'] != message.get('sample_uid')
            or binding['carousel_id'] != state.get_carousel_id()):
        raise ValueError('Inspection does not match the prepared sample and carousel')
    if cell != 'ID%s' % state.get_flowcell_in_use():
        raise ValueError('Inspection must use the coordinated session flow cell')
    cleanup = workflow.cleanup_snapshot()
    if cleanup.get('active') or cleanup.get('problem'):
        raise ValueError('Wait for cleaning to finish and resolve any cleaning failure')
    if state.get_mixer_head() != state.MIXER_AT_CLEANING:
        raise ValueError('The mixer head must be at its cleaning station')
    loading = op == 'inspection_load'
    if loading:
        if active_inspection(api):
            raise ValueError('Another empty-cell inspection holds the equipment')
        if placement['phase'] != 'released' or placement.get('workflow_started'):
            raise ValueError('Inspect before loading the matching prepared sample')
        expected = state.FC_AT_CLEANING
    else:
        if (not inspection or inspection['stage'] != 'at_beam'
                or inspection['flowcell'] != cell
                or inspection['sample_uid'] != binding['sample_uid']
                or inspection['placement_id'] != binding['placement_id']):
            raise ValueError('Return must match the inspection holding the cell at the beam')
        expected = state.FC_AT_BEAM
    if state.get_flowcell_location(state.get_flowcell_in_use()) != expected:
        raise ValueError('The flow cell is not at the expected inspection location')
    transport = ('load_flowcell_from_cleaningstation_to_beam' if loading
                 else 'load_flowcell_from_beam_to_cleaningstation')
    server._require_positions(workflow._stations(transport) + ('sample_table',))
    trace = uuid4().hex
    saved = dict(action_id=trace, request=signature, state='accepted')
    with server._busy_lock:
        if server._busy:
            raise ValueError('PALmixer is busy')
        if loading:
            api.coordinator.begin_inspection(inspection_id)
            inspection = rows[inspection_id] = dict(inspection_id=inspection_id, flowcell=cell,
                sample_uid=binding['sample_uid'], placement_id=binding['placement_id'], stage='loading')
        else:
            inspection['stage'] = 'returning'
        api.record['actions'][key] = saved
        api._save()  # A restart never replays an accepted move.
        server._busy = True
        server._current_action = 'preparation_' + op
        server._current_trace = trace

    def run():
        try:
            workflow._step(transport, lambda: server._run_transport(transport))
            if not loading:
                workflow._step('raise_to_sampletable_height',
                               lambda: workflow.pal.raise_to_sampletable_height(workflow.robot))
            workflow._park()
            target = state.FC_AT_BEAM if loading else state.FC_AT_CLEANING
            if state.get_flowcell_location(state.get_flowcell_in_use()) != target:
                raise ValueError('Inspection transport did not confirm the flow-cell location')
            with api.lock:
                if api.record.get('error'):
                    raise ValueError(api.record['error'])
                if not loading:
                    api.coordinator.end_inspection(inspection_id)
                inspection['stage'] = 'at_beam' if loading else 'completed'
                saved['state'] = 'completed'
                api._save()
            return 'empty-cell inspection %s complete' % ('load' if loading else 'return')
        except Exception as error:
            with api.lock:
                saved.update(state='failed', error=str(error))
                inspection.update(stage='failed', error=str(error))
                api.record['error'] = str(error)
                api._save()
                api.coordinator.invalidate(str(error))
            raise
    label = 'preparation_' + op
    threading.Thread(target=server._run_action, args=(run, label, trace, label), daemon=True).start()
    return deepcopy(saved)
