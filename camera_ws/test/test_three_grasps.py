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


def test_missed_target_counts_as_processed_and_is_excluded_from_next_round():
    calls = []
    def run_round(number, processed):
        calls.append(list(processed))
        return dict(task_motion_complete=True, target_map_xy=[number, 0],
                    target_id=number, outcome='MISSED' if number == 1 else 'COMPLETED')
    results = run_three_grasps(run_round, lambda rows: None)
    assert calls == [[], [[1,0]], [[1,0],[2,0]]]
    assert [row['outcome'] for row in results] == ['MISSED', 'COMPLETED', 'COMPLETED']


def test_completed_standup_is_persisted_if_camera_restore_then_fails(tmp_path, monkeypatch):
    import json
    w = Workflow(SimpleNamespace(work_dir=ROOT, mode='full', grasp_side='auto', detector='yolo'))
    w.folder, w.multi_memory_session, w.processed_results = tmp_path, True, []
    monkeypatch.setattr(w, 'status', lambda: dict(state='ZERO_HOLD', busy=False, navigation_allowed=True))
    result = dict(task_motion_complete=True, target_map_xy=[1.,0.], target_id=1,
                  outcome='MISSED', handoff_ready=False)
    def failed_restore():
        (tmp_path/'round_01_result.json').write_text(json.dumps(result))
        raise RuntimeError('chain not ready')
    monkeypatch.setattr(w, 'run_task', failed_restore)
    with pytest.raises(RuntimeError, match='chain not ready'):
        w.run_full_round(1, [])
    progress = json.loads((tmp_path/'three_grasps.json').read_text())
    assert progress['processed_count'] == 1 and progress['completed_count'] == 0
    assert progress['missed_count'] == 1


def test_manual_return_interrupts_pending_round_without_counting_a_grasp():
    requested = []
    progress = []

    def round_task(number, visited):
        assert number == 1 and visited == []
        requested.append(True)
        return None

    assert run_three_grasps(round_task, lambda done: progress.append(len(done)),
                            stop_requested=lambda: bool(requested)) == []
    assert progress == [0]


def test_manual_return_during_grasp_counts_safe_standup_before_ending():
    requested = []
    rounds = []
    progress = []

    def round_task(number, visited):
        rounds.append(number)
        requested.append(True)
        return dict(task_motion_complete=True, target_map_xy=[1., 0.])

    results = run_three_grasps(round_task, lambda done: progress.append(len(done)),
                               stop_requested=lambda: bool(requested))
    assert len(results) == 1
    assert rounds == [1] and progress == [0, 1]


def test_shortcut_reads_f_without_enter_and_restores_terminal(tmp_path, monkeypatch):
    import os
    import pty
    import termios
    import time

    master, slave = pty.openpty()
    workflow = Workflow(SimpleNamespace(work_dir=ROOT, mode='full'))
    workflow.folder = tmp_path
    try:
        with os.fdopen(slave, 'r') as terminal:
            monkeypatch.setattr(sys, 'stdin', terminal)
            original = termios.tcgetattr(terminal.fileno())
            with workflow.return_hotkey():
                os.write(master, b'f')
                until = time.monotonic()+2.
                while not workflow.manual_return_request.exists() and time.monotonic() < until:
                    time.sleep(.01)
                assert workflow.manual_return_request.is_file()
            assert termios.tcgetattr(terminal.fileno()) == original
    finally:
        os.close(master)


def test_manual_return_immediately_disables_motion_before_leaving_coordinator(tmp_path, monkeypatch):
    sys.path.insert(0, str(ROOT/'camera_ws/scripts'))
    from run_map_anchor_experiment import Experiment

    request = tmp_path/'manual_return_request.json'
    request.touch()
    calls = []
    experiment = Experiment.__new__(Experiment)
    experiment.args = SimpleNamespace(search_timeout=60., return_only=False,
                                      manual_return_request=request)
    experiment.node = SimpleNamespace(count_publishers=lambda _: 1)
    experiment.nav_process = SimpleNamespace(poll=lambda: None)
    experiment.children = []
    experiment.control = {'state': 'IDLE'}
    experiment.spin = lambda _: None
    experiment.event = lambda event, **_: calls.append(event)
    experiment.mission = SimpleNamespace(state='EXPLORE',
        arbiter=SimpleNamespace(flush=lambda: calls.append('zero')),
        step=lambda: pytest.fail('planner must not step after the return request'))

    experiment.run_mission()
    assert calls == ['zero', 'MANUAL_RETURN_STOPPING']
    assert experiment.manual_return_requested
