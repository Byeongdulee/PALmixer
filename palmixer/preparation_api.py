"""Versioned PALsystem coordination endpoint; existing workflows own all UR motion."""
from copy import deepcopy
import json
import os
from pathlib import Path
import threading
from uuid import UUID, uuid4

from . import config, state
from .preparation import PreparationCoordinator


class PreparationAPI:
    def __init__(self, server, path=None):
        self.server = server
        self.path = Path(path or str(state._INI_PATH) + '.preparation-session.json')
        self.lock = threading.RLock()
        self.record = json.loads(self.path.read_text()) if self.path.exists() else {}
        self.coordinator = None
        if self.record.get('active'):
            self.record['error'] = 'Server restarted during coordinated work; inspect and reconcile before further motion.'
            self._save()

    @property
    def active(self):
        return bool(self.record.get('active'))

    def _save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        with temporary.open('w') as handle:
            json.dump(self.record, handle, allow_nan=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self.path)

    def snapshot(self):
        with self.lock:
            return {'version': 1, 'active': self.active, 'run_id': self.record.get('run_id'),
                    'error': self.record.get('error', ''),
                    'offset_steps': config.get_section('carousel').get('preparation_offset_steps')}

    def request(self, message):
        with self.lock:
            if message.get('version') != 1:
                raise ValueError('Unsupported preparation protocol version')
            run_id = message.get('run_id')
            if str(UUID(run_id)) != run_id:
                raise ValueError('Canonical run UUID required')
            op = message.get('op')
            if op == 'begin':
                if self.active:
                    if run_id == self.record.get('run_id') and not self.record.get('error'):
                        return self.snapshot()
                    raise ValueError('Coordinated work already owns the equipment')
                offset = config.get_section('carousel').get('preparation_offset_steps')
                if type(offset) is not int:
                    raise ValueError('Configure carousel.preparation_offset_steps (agreed station: -4)')
                if self.server.simulate:
                    raise ValueError('The live test requires a real PALmixer server')
                cleanup = self.server.workflows.cleanup_snapshot()
                if self.server._busy or cleanup.get('active') or cleanup.get('problem'):
                    raise ValueError('PALmixer or its cleanup is busy or needs attention')
                journal = self.path.parent / ('preparation-' + run_id + '.json')
                if journal.exists():
                    raise ValueError('This run already has a motion journal; it cannot be replayed')
                self.coordinator = PreparationCoordinator(journal,
                    lambda binding: self.server.workflows.position_for_preparation(binding, offset_steps=offset),
                    lambda binding: None)  # PALsystem relays persisted readiness to ChemLab.
                self.record = dict(active=True, run_id=run_id, actions={}, error='')
                self._save()
                self.server.workflows.preparation = self.coordinator
                return self.snapshot()
            if run_id != self.record.get('run_id') or not self.active:
                raise ValueError('This run does not own coordinated preparation')
            if op == 'status':
                return dict(self.snapshot(), coordination=self.coordinator.snapshot() if self.coordinator else {},
                            actions=deepcopy(self.record.get('actions', {})))
            if op == 'inhibit':
                self.record['error'] = 'PALsystem inhibited further work; inspect the original run.'
                self._save()
                if self.coordinator:
                    self.coordinator.invalidate(self.record['error'])
                return self.snapshot()
            if self.record.get('error') or self.coordinator is None:
                raise ValueError(self.record.get('error') or 'No preparation coordinator')
            if op == 'queue':
                binding = message['binding']
                if binding['carousel_id'] != state.get_carousel_id():
                    raise ValueError('The mounted carousel changed')
                state.validate_slot(binding['slot'])
                if state.get_sample_id(binding['slot']):
                    raise ValueError('The requested holder is occupied')
                return self.coordinator.queue(binding)
            if op == 'release':
                return self.coordinator.release(message['placement_id'], message['sample_uid'],
                    completion_id=message['completion_id'], prepared=True, pal_tool_clear=True)
            if op == 'tag':
                record = self.coordinator.get(message['placement_id'])
                b = record['binding']
                if record['phase'] != 'released' or b['sample_uid'] != message['sample_uid']:
                    raise ValueError('Only the matching completed preparation can tag a holder')
                if b['carousel_id'] != state.get_carousel_id():
                    raise ValueError('The mounted carousel changed')
                old = state.carousel_sample_uids().get(b['slot'])
                if old and old != b['sample_uid']:
                    raise ValueError('The holder contains another UUID')
                state.set_sample_id(b['slot'], message['sample_id'])
                state.set_sample_uid(b['slot'], message['sample_id'], b['sample_uid'])
                return {'tagged': True}
            if op in ('initial', 'make', 'unload'):
                return self._action(message)
            if op == 'end':
                if self.server._busy or self.server.workflows.cleanup_snapshot().get('active'):
                    raise ValueError('Wait for motion and cleanup to finish')
                self.coordinator.require_idle()
                self.coordinator.close()
                self.server.workflows.preparation = None
                self.record['active'] = False
                self._save()
                return self.snapshot()
            raise ValueError('Unknown preparation operation')

    def _action(self, message):
        key = message['action_key']
        signature = json.dumps(message, sort_keys=True)
        previous = self.record['actions'].get(key)
        if previous:
            if previous['request'] != signature:
                raise ValueError('Action identity already has different contents')
            return deepcopy(previous)
        op = message['op']
        workflow = self.server.workflows
        record = self.coordinator.get(message['placement_id'])
        b = record['binding']
        if message['sample_uid'] != b['sample_uid'] or state.get_carousel_id() != b['carousel_id']:
            raise ValueError('Action does not match the sample or mounted carousel')
        if op == 'initial':
            self.server._require_positions(workflow.stations_for_make_sample())
            fn = lambda: workflow.prepare_initial(message['placement_id'])
        elif op == 'make':
            if record['phase'] != 'released' or state.carousel_sample_uids().get(b['slot']) != b['sample_uid']:
                raise ValueError('Matching preparation and holder UUID required before mixing')
            self.server._require_positions(workflow.stations_for_make_sample())
            workflow._check_make_sample(b['slot'], state.get_sample_id(b['slot']))
            fn = lambda: self.server._make_sample_recorded(b['slot'], state.get_sample_id(b['slot']), sample_uid=b['sample_uid'])
        else:
            if record.get('workflow_outcome', {}).get('status') != 'completed':
                raise ValueError('The matching sample has not completed loading')
            self.server._require_positions(workflow.stations_for_unload_sample(False))
            workflow._check_unload_sample(False)
            fn = lambda: workflow.unload_sample(False)
        trace = uuid4().hex
        saved = dict(action_id=trace, request=signature, state='accepted')
        def run():
            try:
                detail = fn()
                with self.lock:
                    saved.update(state='completed')
                    self._save()
                return detail
            except Exception as error:
                with self.lock:
                    saved.update(state='failed', error=str(error))
                    self.record['error'] = str(error)
                    self._save()
                    self.coordinator.invalidate(str(error))
                raise
        with self.server._busy_lock:
            if self.server._busy:
                raise ValueError('PALmixer is busy')
            self.record['actions'][key] = saved
            self._save()  # Persist acceptance before any physical action.
            self.server._busy = True
            self.server._current_action = 'preparation_' + op
            self.server._current_trace = trace
        threading.Thread(target=self.server._run_action,
            args=(run, 'preparation_' + op, trace, 'preparation_' + op), daemon=True).start()
        return deepcopy(saved)
