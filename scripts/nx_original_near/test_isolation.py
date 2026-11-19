"""Offline regression of generated NX control ownership, without ROS or hardware."""
import ast
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest


ROOT = Path(os.environ.get('NX_NEAR_PREPARED', '/tmp/nx-near-prepared'))


def methods(path, classname, names, globals_):
    source = (ROOT/path).read_text()
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == classname)
    lines = source.splitlines(True)
    selected = 'class '+classname+':\n'+''.join(
        ''.join(lines[n.lineno-1:n.end_lineno])+'\n' for n in cls.body
        if isinstance(n, ast.FunctionDef) and n.name in names)
    exec(compile(selected, str(path), 'exec'), globals_)
    return globals_[classname]


def mission():
    cls = methods(Path('camera_ws/src/go2_science_perception/go2_science_perception/mission_manager.py'),
                  'MissionManager', {'step', 'original_near_step', 'command_clear', 'tracks'},
                  dict(time=time, NEAR_STATES=('LOCK_TARGET', 'APPROACH_TO_GRASP_POSE'),
                       json=json, math=math))
    m = cls()
    m.enabled, m.error, m.state, m.finished = True, None, 'APPROACH', False
    m.run = SimpleNamespace(navigation_suspended=True, last_control_received=time.monotonic(),
                           control={'state': 'APPROACH_TO_GRASP_POSE', 'docking_phase': 'LATERAL',
                                    'target_id': 1, 'grasp_side': 'left'},
                           event=lambda *a, **kw: None)
    m.arbiter = SimpleNamespace(enabled=False)
    m.anchor, m.active_side, m.side_hint = {'id': 1}, 'left', 'left'
    m.stage, m.sides = None, {'left': object(), 'right': object()}
    m.change = lambda state, mode: setattr(m, 'state', state)
    m.fail = lambda reason: (_ for _ in ()).throw(RuntimeError(reason))
    return m


def test_approach_does_not_query_dead_navigation_or_terrain():
    m = mission()
    # node, pose, terrain and navigation endpoints intentionally absent.
    m.step()
    assert m.state == 'DOCK' and m.arbiter.enabled
    assert m.command_clear(None)


@pytest.mark.parametrize('target,side,reason', [(7, 'left', 'identity mismatch'),
                                             (1, 'right', 'side mismatch')])
def test_target_and_grasp_side_consistency_remain_required(target, side, reason):
    m = mission()
    m.state = 'STOP_CONFIRM'
    m.anchor = {'id': 1}
    m.deadline = time.monotonic()+2
    m.run.control = {'state': 'LOCK_TARGET', 'target_id': target, 'grasp_side': side}
    with pytest.raises(RuntimeError, match=reason):
        m.step()


def test_multi_target_selection_and_completed_filter_are_preserved():
    m = mission()
    m.anchor, m.pose = None, (0., 0., 0.)
    m.association_radius, m.completed = .15, [(3., 0.)]
    m.fresh, m.can_retry_target = lambda *a: True, lambda *a: True
    data = {'frame': 'map', 'tracks': [
        dict(id=i, position=[float(i), 0., 0.], visited=(i == 4),
             observed_ns=1, state='CONFIRMED') for i in range(1, 5)]}
    m.tracks(SimpleNamespace(data=json.dumps(data)))
    assert [t['id'] for t in m.eligible_tracks] == [1, 2]


@pytest.mark.parametrize('complete,expected', [(True, 'GRASP'), (False, 'IDLE')])
def test_only_original_docking_completion_authorizes_grasp(complete, expected):
    m = mission()
    m.run.control = {'state': 'STOP', 'grasp_docking_complete': complete,
                     'target_id': 1, 'grasp_side': 'left'}
    m.step()
    assert m.finished and m.state == expected


def test_missing_original_control_stops_handoff():
    m = mission()
    m.run.last_control_received = time.monotonic()-1.1
    with pytest.raises(RuntimeError, match='status reception stopped'):
        m.step()


def test_suspension_stops_owned_process_and_retains_unrelated_process():
    cls = methods(Path('camera_ws/scripts/run_map_anchor_experiment.py'), 'Experiment',
                  {'halt_planners', 'suspend_navigation'},
                  dict(time=time, os=os, signal=signal, subprocess=subprocess))
    owned = subprocess.Popen([sys.executable, '-c',
        'import signal,time,sys; '
        'signal.signal(signal.SIGINT, lambda *args: (time.sleep(.1), sys.exit(0))); '
        'print("READY",flush=True); time.sleep(60)'],
                             start_new_session=True, stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL)
    other = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                             start_new_session=True)
    try:
        assert owned.stdout.readline().strip() == b'READY'
        e = cls()
        events = []
        e.nav_process, e.navigation_suspended = owned, False
        e.mission = SimpleNamespace(arbiter=SimpleNamespace(flush=lambda: events.append('ZERO')))
        e.event = lambda name, **kw: events.append(name)
        e.node = SimpleNamespace(count_publishers=lambda topic: 0)
        def spin(seconds):
            events.append('HEARTBEAT')
            time.sleep(.005)
        e.spin = spin
        e.suspend_navigation()
        assert owned.poll() is not None and other.poll() is None
        assert e.navigation_suspended and events[0] == 'ZERO'
        assert events.count('HEARTBEAT') > 1
        assert events[-1] == 'NEAR_NAVIGATION_STOPPED'
    finally:
        for process in (owned, other):
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=2)
