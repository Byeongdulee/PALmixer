"""Versioned PALsystem coordination endpoint; existing workflows own all UR motion."""
from copy import deepcopy
import json
import os
from pathlib import Path
import threading
from uuid import UUID, uuid4

from . import config, state
from .preparation import PreparationCoordinator

# Reported when placement finds the stock seat empty after the old carousel was
# already unloaded. Not a session failure: the operator's button press finishes it.
NO_STOCK_MESSAGE = ("No fresh carousel in stock. Put one on the stock seat and press "
                    "Place New Carousel on PALmixer.")


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
                self.record = dict(active=True, run_id=run_id, actions={}, replacements={}, error='')
                self._save()
                self.server.workflows.preparation = self.coordinator
                return self.snapshot()
            if run_id != self.record.get('run_id') or not self.active:
                raise ValueError('This run does not own coordinated preparation')
            if op == 'status':
                return dict(self.snapshot(), coordination=self.coordinator.snapshot() if self.coordinator else {},
                            actions=deepcopy(self.record.get('actions', {})),
                            replacements=deepcopy(self.record.get('replacements', {})))
            if op == 'inhibit':
                self.record['error'] = 'PALsystem inhibited further work; inspect the original run.'
                self._save()
                if self.coordinator:
                    self.coordinator.invalidate(self.record['error'])
                return self.snapshot()
            if op == 'discard':
                # Operator force-clear for a session an inhibited or restarted run left
                # open: `end` is blocked once there is an error or no coordinator, so this
                # is the only way out. Requires an explicit acknowledgement that PAL has
                # cleared the carousel and holders have been inspected.
                if message.get('acknowledged') is not True:
                    raise ValueError('Confirm PAL cleared the carousel and holders before discarding the session')
                if self.server._busy or self.server.workflows.cleanup_snapshot().get('active'):
                    raise ValueError('Wait for motion and cleanup to finish before discarding the session')
                if self.coordinator:
                    self.coordinator.invalidate('Preparation session discarded by operator')
                    self.coordinator.close()
                self.server.workflows.preparation = None
                self.record = dict(active=False, run_id=run_id, error='')
                self._save()
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
            if op == 'replace_carousel':
                return self._replace_carousel(message)
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

    def _replace_carousel(self, message):
        """Unload the spent carousel and place a fresh one, as one coordinated action.

        Reuses the Unload Current Carousel and Place New Carousel code paths; the
        placement records the carousel load and frees every slot. Allowed while
        the flowcell wash still runs -- that overlap is the point -- but not
        while anything else moves or a cleanup has failed. Progress is persisted
        before each physical phase, and a duplicate request returns the record
        instead of starting a second swap."""
        replacement_id = message.get('replacement_id')
        try:
            canonical = isinstance(replacement_id, str) and str(UUID(replacement_id)) == replacement_id
        except ValueError:
            canonical = False
        if not canonical:
            raise ValueError('Canonical replacement UUID required')
        if message.get('action_key') != 'replace:' + replacement_id:
            raise ValueError('action_key must be replace:<replacement_id>')
        signature = json.dumps(message, sort_keys=True)
        replacements = self.record.setdefault('replacements', {})
        previous = replacements.get(replacement_id)
        if previous:
            if previous['request'] != signature:
                raise ValueError('Replacement identity already has different contents')
            return deepcopy(previous)
        self.coordinator.require_idle()
        if self.server._busy:
            raise ValueError('PALmixer is busy')
        problem = self.server.workflows.cleanup_snapshot().get('problem')
        if problem:
            raise ValueError('PALmixer cleanup needs attention: %s' % problem)
        if state.get_mixer_head() != state.MIXER_AT_CLEANING:
            raise ValueError('The mixer head must be at its cleaning station to replace the carousel')
        flowcell = state.get_flowcell_in_use()
        if state.get_flowcell_location(flowcell) != state.FC_AT_CLEANING:
            raise ValueError('Flowcell %d must be at its cleaning station to replace the carousel' % flowcell)
        self.server._require_positions(('carousel_on_mixer', 'carousel_used_stack', 'carousel_in_stock'))
        no_carousel = getattr(self.server.PAL12idb, 'NoCarouselError', None)
        trace = uuid4().hex
        saved = dict(replacement_id=replacement_id, action_id=trace, request=signature,
                     state='accepted', error='')

        def progress(name, **extra):
            with self.lock:
                saved.update(state=name, **extra)
                self._save()

        def run():
            placing = False
            try:
                progress('unloading')
                unloaded = self.server._unload_carousel()
                progress('old_removed')
                progress('placing')
                placing = True
                placed, load = self.server._place_and_record_load(replacement_id)
                progress('committed', load=load)
                return '%s; %s; every slot cleared (carousel load %d)' % (
                    unloaded, placed, load['sequence'])
            except Exception as error:
                if placing and no_carousel is not None and isinstance(error, no_carousel):
                    # The old carousel is already on the used stack. Leave the
                    # session usable so PALsystem can end it; the operator loads
                    # stock and presses Place New Carousel, which records the load.
                    progress('waiting_for_stock', error=NO_STOCK_MESSAGE)
                    return NO_STOCK_MESSAGE
                with self.lock:
                    # failed_phase keeps "unload failed" apart from "old carousel
                    # removed, placement failed" -- they need different recovery.
                    saved.update(state='failed', failed_phase=saved['state'], error=str(error))
                    self.record['error'] = str(error)
                    self._save()
                    self.coordinator.invalidate(str(error))
                raise
        label = 'preparation_replace_carousel'
        with self.server._busy_lock:
            if self.server._busy:
                raise ValueError('PALmixer is busy')
            replacements[replacement_id] = saved
            self._save()  # Persist acceptance before any physical action.
            self.server._busy = True
            self.server._current_action = label
            self.server._current_trace = trace
        threading.Thread(target=self.server._run_action,
            args=(run, label, trace, label), daemon=True).start()
        return deepcopy(saved)
