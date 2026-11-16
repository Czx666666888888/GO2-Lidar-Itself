"""Offline session counting and round-isolated camera handoff checks."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from go2_three_grasps import run_three_grasps
from go2_full_test import Workflow


def test_three_successful_standups_stop_without_a_fourth_round():
    calls, progress = [], []

    def run(number, visited):
        calls.append((number, list(visited)))
        return dict(task_motion_complete=True, target_map_xy=[number, 0],
                    placement_confirmed=None)

    results = run_three_grasps(run, lambda done: progress.append(len(done)))
    assert len(results) == 3 and progress == [0, 1, 2, 3]
    assert calls == [(1, []), (2, [[1, 0]]), (3, [[1, 0], [2, 0]])]


@pytest.mark.parametrize('failed', [1, 2, 3])
def test_abort_keeps_existing_failure_behavior_without_counting_or_continuing(failed):
    calls, progress = [], []

    def run(number, visited):
        calls.append(number)
        if number == failed:
            raise RuntimeError('original arm fault')
        return dict(task_motion_complete=True, target_map_xy=[number, 0])

    with pytest.raises(RuntimeError, match='original arm fault'):
        run_three_grasps(run, lambda done: progress.append(len(done)))
    assert calls == list(range(1, failed+1))
    assert progress[-1] == failed-1


def test_acceptance_or_return_to_zero_alone_is_not_a_completed_round():
    with pytest.raises(RuntimeError, match='did not complete'):
        run_three_grasps(lambda *a: dict(accepted=True), lambda done: None)


def test_each_round_has_new_camera_ack_channel_and_passes_completed_positions(tmp_path):
    w = Workflow(SimpleNamespace(work_dir=ROOT, mode='full', grasp_side='auto'))
    w.folder = tmp_path
    channels = []
    for number in range(1, 4):
        w.current_round = number
        w.completed_targets = [[1., 2.]] if number > 1 else []
        command = w.task_command()
        assert '--continuous-search' in command
        channels.append(command[command.index('--camera-handoff-directory')+1])
        assert command[command.index('--completed-targets-json')+1] == (
            '[[1.0, 2.0]]' if number > 1 else '[]')
        assert 'round_%02d_result.json' % number in command[command.index('--round-result')+1]
    assert len(set(channels)) == 3


@pytest.mark.parametrize('detector,completed,reset', [
    ('yolo', [], False), ('yolo', [[1., 0.]], True), ('hsv', [[1., 0.]], False)])
def test_yolo_reset_ack_before_next_controller(tmp_path, monkeypatch, detector, completed, reset):
    import go2_full_test as entry
    import json
    w = Workflow(SimpleNamespace(work_dir=ROOT, mode='full', grasp_side='auto', detector=detector))
    w.folder = tmp_path
    monkeypatch.setattr(w, 'status', lambda: dict(state='ZERO_HOLD', busy=False, navigation_allowed=True))
    monkeypatch.setattr(w, 'shell', lambda command: command)
    calls = []
    monkeypatch.setattr(entry, 'run', lambda *a, **kw: calls.append('reset'))
    def task():
        calls.append('task')
        (tmp_path/'round_02_result.json').write_text(json.dumps({'task_motion_complete': True}))
    monkeypatch.setattr(w, 'run_task', task)
    w.run_full_round(2, completed)
    assert calls == (['reset', 'task'] if reset else ['task'])


def test_reset_failure_never_starts_next_search(tmp_path, monkeypatch):
    import go2_full_test as entry
    w = Workflow(SimpleNamespace(work_dir=ROOT, mode='full', grasp_side='auto', detector='yolo'))
    w.folder = tmp_path
    monkeypatch.setattr(w, 'status', lambda: dict(state='ZERO_HOLD', busy=False, navigation_allowed=True))
    monkeypatch.setattr(w, 'shell', lambda command: command)
    def fail(*a, **kw):
        raise RuntimeError('reset not acknowledged')
    monkeypatch.setattr(entry, 'run', fail)
    monkeypatch.setattr(w, 'run_task', lambda: pytest.fail('must not start controller'))
    with pytest.raises(RuntimeError, match='not acknowledged'):
        w.run_full_round(2, [[1., 0.]])
