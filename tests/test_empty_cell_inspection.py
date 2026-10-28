"""Inspection reserves both the arm and carousel scheduler across the exposure."""
from types import SimpleNamespace
import threading
import time
from uuid import uuid4

import pytest

from palmixer import state
from palmixer.preparation import PreparationCoordinator, PreparationError
from palmixer.preparation_api import PreparationAPI
from palmixer.server import PALmixerServer
from test_preparation_coordination import binding, released


def until(predicate):
    deadline = time.monotonic() + 2
    while not predicate():
        assert time.monotonic() < deadline, 'Fake inspection action timed out'
        time.sleep(.002)


@pytest.fixture
def rig(tmp_path, monkeypatch):
    log = []
    physical = dict(location=state.FC_AT_CLEANING, head=state.MIXER_AT_CLEANING, fail_return=False)
    cleanup = dict(version=1, active=False, problem='', parked=True, wash_complete=True)
    monkeypatch.setattr(state, 'get_carousel_id', lambda: 'carousel-A')
    monkeypatch.setattr(state, 'get_flowcell_in_use', lambda: 1)
    monkeypatch.setattr(state, 'get_flowcell_location', lambda _: physical['location'])
    monkeypatch.setattr(state, 'get_mixer_head', lambda: physical['head'])
    monkeypatch.setattr(state, 'get_sample_id', lambda _: '')
    monkeypatch.setattr(state, 'validate_slot', lambda slot: slot)
    def step(name, fn):
        cleanup['parked'] = False
        log.append(name)
        return fn()
    def park():
        log.append('park')
        cleanup['parked'] = True
    def transport(name):
        if 'beam_to' in name:
            if physical['fail_return']:
                raise ValueError('return motion failed')
            physical['location'] = state.FC_AT_CLEANING
        else:
            physical['location'] = state.FC_AT_BEAM
    wf = SimpleNamespace(cleanup_snapshot=lambda: dict(cleanup), _step=step, _park=park,
        _stations=lambda *names: (), robot=None,
        pal=SimpleNamespace(raise_to_sampletable_height=lambda _: log.append('raise')))
    server = SimpleNamespace(workflows=wf, _busy=False, _busy_lock=threading.Lock(),
                             _require_positions=lambda _: None, _run_transport=transport)
    def execute(fn, *args):
        try:
            fn()
        except Exception:
            pass  # The API has already persisted the failed action and inhibition.
        finally:
            server._busy = False
    server._run_action = execute
    api = PreparationAPI(server, tmp_path / 'session.json')
    server.preparation_api = api
    run_id = str(uuid4())
    api.record = dict(active=True, run_id=run_id, actions={}, error='')
    coordinator = api.coordinator = PreparationCoordinator(tmp_path / 'placements.json',
        lambda b: log.append(('position', b['slot'])), lambda b: None)
    b = binding()
    released(coordinator, b)
    api._save()
    inspection_id = str(uuid4())
    def action(op, **extra):
        message = dict(version=1, run_id=run_id, op=op, action_key=op + ':' + inspection_id,
            inspection_id=inspection_id, flowcell='ID1', placement_id=b['placement_id'], sample_uid=b['sample_uid'])
        message.update(extra)
        return api.request(message)
    value = SimpleNamespace(api=api, server=server, coordinator=coordinator, binding=b, run_id=run_id,
        inspection_id=inspection_id, action=action, physical=physical, cleanup=cleanup, log=log)
    yield value
    until(lambda: not server._busy)
    coordinator.close()


def test_round_trip_holds_motion_through_exposure_and_deduplicates(rig):
    first = rig.action('inspection_load')
    until(lambda: not rig.server._busy)
    assert rig.physical['location'] == state.FC_AT_BEAM
    assert rig.api.snapshot()['inspection']['stage'] == 'at_beam'
    assert rig.action('inspection_load')['action_id'] == first['action_id']
    with pytest.raises(ValueError, match='holds the equipment'):
        rig.api.request(dict(version=1, run_id=rig.run_id, op='make'))
    assert 'owns the equipment' in PALmixerServer.fast_dispatch(
        rig.server, 'load_flowcell_from_beam_to_cleaningstation')
    with pytest.raises(PreparationError):
        rig.coordinator.begin_sample(rig.binding['sample_uid'], 1, 'mix', carousel_id='carousel-A')
    next_binding = binding(2, rig.binding['sample_uid'])
    rig.coordinator.queue(next_binding)
    assert rig.coordinator.get(next_binding['placement_id'])['phase'] == 'pending'
    rig.log.append('exposure')
    returned = rig.action('inspection_unload')
    until(lambda: not rig.server._busy)
    assert rig.action('inspection_unload')['action_id'] == returned['action_id']
    assert rig.physical['location'] == state.FC_AT_CLEANING
    assert rig.cleanup['parked'] and rig.api.snapshot()['inspection'] is None
    assert rig.coordinator.snapshot()['inspection'] is None
    assert rig.log.count('load_flowcell_from_cleaningstation_to_beam') == 1
    assert rig.log.count('load_flowcell_from_beam_to_cleaningstation') == 1
    assert rig.log.index('exposure') < rig.log.index('load_flowcell_from_beam_to_cleaningstation')
    rig.coordinator.begin_sample(rig.binding['sample_uid'], 1, 'mix', carousel_id='carousel-A')
    with pytest.raises(ValueError, match='already returned'):
        rig.action('inspection_load')


@pytest.mark.parametrize('fault', ['washing', 'wash_failure', 'busy', 'pal_not_clear', 'wrong_cell', 'wrong_sample'])
def test_no_inspection_motion_until_preconditions_hold(rig, fault):
    args = {}
    if fault == 'washing': rig.cleanup['active'] = True
    if fault == 'wash_failure': rig.cleanup['problem'] = 'wash failed'
    if fault == 'busy': rig.server._busy = True
    if fault == 'pal_not_clear': rig.coordinator._records[rig.binding['placement_id']]['phase'] = 'slot_ready'
    if fault == 'wrong_cell': args['flowcell'] = 'ID2'
    if fault == 'wrong_sample': args['sample_uid'] = str(uuid4())
    try:
        with pytest.raises((ValueError, PreparationError)):
            rig.action('inspection_load', **args)
        assert not any(isinstance(x, str) and x.startswith('load_flowcell') for x in rig.log)
    finally:
        rig.server._busy = False


def test_failed_return_keeps_hold_and_rejects_further_motion(rig):
    rig.action('inspection_load')
    until(lambda: not rig.server._busy)
    rig.physical['fail_return'] = True
    rig.action('inspection_unload')
    until(lambda: not rig.server._busy)
    assert rig.api.snapshot()['inspection']['stage'] == 'failed'
    assert rig.coordinator.snapshot()['inspection'] == rig.inspection_id
    with pytest.raises(ValueError, match='return motion failed'):
        rig.api.request(dict(version=1, run_id=rig.run_id, op='make'))


def test_restart_with_cell_at_beam_requires_review_without_replay(rig):
    rig.action('inspection_load')
    until(lambda: not rig.server._busy)
    before = list(rig.log)
    restarted = PreparationAPI(rig.server, rig.api.path)
    assert restarted.snapshot()['inspection']['stage'] == 'at_beam'
    with pytest.raises(ValueError, match='restarted'):
        restarted.request(dict(version=1, run_id=rig.run_id, op='inspection_unload'))
    scheduler = PreparationCoordinator(rig.coordinator.path,
        lambda b: rig.log.append('unexpected move'), lambda b: None)
    assert scheduler.snapshot()['inspection'] == rig.inspection_id
    with pytest.raises(PreparationError):
        scheduler.end_inspection(rig.inspection_id)
    scheduler.close()
    assert rig.log == before


def test_another_inspection_cannot_return_or_steal_the_cell(rig):
    rig.action('inspection_load')
    until(lambda: not rig.server._busy)
    other = str(uuid4())
    for op in ('inspection_load', 'inspection_unload'):
        with pytest.raises(ValueError):
            rig.action(op, inspection_id=other, action_key=op + ':' + other)
    assert rig.physical['location'] == state.FC_AT_BEAM
