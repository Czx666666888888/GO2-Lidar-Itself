"""Camera lifecycle checks with simulated process ownership; no ROS or hardware."""
import json
import ast
from pathlib import Path
import sys
import subprocess
import threading
import os
import pwd
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from go2_camera_handoff import CameraHandoff, service_requests, atomic_json, RELEASE_CHECK
from go2_full_test import Workflow


@pytest.mark.parametrize('failed', [False, True])
def test_arm_runs_only_after_pause_ack_and_restores_even_on_failure(tmp_path, failed):
    events = []
    done = threading.Event()

    def parent():
        while not done.wait(.005):
            service_requests(tmp_path, lambda: events.append('D435_OFF'),
                             lambda: events.append('D435_ON'))

    thread = threading.Thread(target=parent)
    thread.start()
    try:
        try:
            with CameraHandoff(tmp_path, timeout=1.):
                events.append('D455_OPEN')
                try:
                    if failed:
                        raise RuntimeError('camera capture failed')
                finally:
                    events.append('D455_CLOSED')
        except RuntimeError:
            assert failed
        assert events == ['D435_OFF', 'D455_OPEN', 'D455_CLOSED', 'D435_ON']
    finally:
        done.set()
        thread.join()


def test_failed_pause_never_authorizes_arm_and_ack_is_not_repeated(tmp_path):
    calls = []

    def pause():
        calls.append('pause')
        raise RuntimeError('D435 still owned')

    atomic_json(tmp_path/'pause.request.json', dict(action='pause'))
    for _ in range(2):
        service_requests(tmp_path, pause, lambda: None)
    acknowledgement = json.loads((tmp_path/'pause.ack.json').read_text())
    assert not acknowledgement['ok'] and 'still owned' in acknowledgement['error']
    assert calls == ['pause']


def test_partial_pause_failure_requests_restoration_without_entering_arm(tmp_path):
    events = []
    done = threading.Event()

    def pause():
        events.append('D435_OFF')
        raise RuntimeError('release verification failed')

    def parent():
        while not done.wait(.005):
            service_requests(tmp_path, pause, lambda: events.append('D435_ON'))

    thread = threading.Thread(target=parent)
    thread.start()
    try:
        with pytest.raises(RuntimeError, match='release verification'):
            with CameraHandoff(tmp_path, timeout=1.):
                pytest.fail('arm must not start')
        assert events == ['D435_OFF', 'D435_ON']
    finally:
        done.set()
        thread.join()


def test_missing_supervisor_times_out_without_authorizing_arm(tmp_path):
    with pytest.raises(TimeoutError, match='pause'):
        CameraHandoff(tmp_path, timeout=.001).request('pause')


def workflow(tmp_path):
    w = Workflow(SimpleNamespace(work_dir=ROOT, mode='full', grasp_side='right'))
    w.folder = tmp_path
    return w


@pytest.mark.parametrize('state', [dict(busy=True, state='GRASPING'), {},
                                  dict(busy=False, state='RETURNING_ZERO')])
def test_resume_refuses_active_or_unknown_arm_without_opening_camera(tmp_path, state):
    w = workflow(tmp_path)
    w.camera_paused = True
    w.status = lambda: state
    w.cameras_released = lambda p: pytest.fail('must check arm first')
    with pytest.raises(RuntimeError, match='active/unknown'):
        w.resume_front_camera()
    assert w.camera_paused


def test_resume_refuses_d455_ownership_before_start(tmp_path):
    w = workflow(tmp_path)
    w.camera_paused = True
    w.status = lambda: dict(busy=False, state='ZERO_HOLD')

    def occupied(products):
        assert products == {'0b5c'}
        raise RuntimeError('D455 still owned')

    w.cameras_released = occupied
    with pytest.raises(RuntimeError, match='still owned'):
        w.resume_front_camera()
    assert w.camera_paused


def test_restore_verifies_fresh_rgb_depth_and_retains_driver(tmp_path, monkeypatch):
    import go2_full_test
    w = workflow(tmp_path)
    w.camera_paused = True
    w.status = lambda: dict(busy=False, state='ZERO_HOLD')
    events = []
    w.cameras_released = lambda products: events.append(products)
    monkeypatch.setattr(go2_full_test, 'remote', lambda *a, **kw: events.append('launch'))
    w.probe = lambda *a, **kw: dict(camera_ok=True, ready=True)
    w.resume_front_camera()
    assert events == [{'0b5c'}, {'0b07'}, 'launch']
    assert not w.camera_paused and not w.children


def test_restored_camera_without_frames_is_not_reported_ready(tmp_path, monkeypatch):
    import go2_full_test
    w = workflow(tmp_path)
    w.camera_paused = True
    w.status = lambda: dict(busy=False, state='ZERO_HOLD')
    w.cameras_released = lambda products: None
    launches = []
    monkeypatch.setattr(go2_full_test, 'remote', lambda *a, **kw: launches.append('launch'))
    w.probe = lambda *a, **kw: dict(camera_ok=False)
    with pytest.raises(RuntimeError, match='full RGB-D/localization/TF chain not ready'):
        w.resume_front_camera()
    w.resume_front_camera()  # No duplicate launch after a successful process start.
    assert launches == ['launch']


def test_pause_closes_own_camera_before_release_check(tmp_path):
    w = workflow(tmp_path)
    events = []
    pipe = SimpleNamespace(close=lambda: events.append('EOF'))
    child = SimpleNamespace(poll=lambda: None, stdin=pipe,
                            wait=lambda **kw: events.append('EXIT'))
    stream = SimpleNamespace(close=lambda: events.append('LOG_CLOSE'))
    w.children = [(child, stream, 'camera')]
    w.stop_invalid_camera = lambda: events.append('REUSED_CHECK')
    w.cameras_released = lambda p: events.append('RELEASE_CHECK')
    w.pause_front_camera()
    assert events == ['EOF', 'EXIT', 'LOG_CLOSE', 'REUSED_CHECK', 'RELEASE_CHECK']
    assert not w.children and w.camera_paused


def test_full_entry_wires_switch_and_dock_does_not(tmp_path):
    w = workflow(tmp_path)
    command = w.task_command()
    assert command[command.index('--camera-handoff-directory')+1] == str(tmp_path/'camera_handoff')
    w.args.mode = 'dock'
    assert '--camera-handoff-directory' not in w.task_command()


def test_direct_full_without_supervisor_is_rejected_before_ros_or_hardware():
    result = subprocess.run([sys.executable, str(ROOT/'camera_ws/scripts/run_map_s1_task.py'),
                             '--execute', '--full'], capture_output=True, text=True)
    assert result.returncode == 2
    assert 'Full camera switching requires' in result.stderr


@pytest.mark.parametrize('name,parent_command,expected', [
    ('(sd-pam)', b'/lib/systemd/systemd\0--user\0', True),
    ('python3', b'/lib/systemd/systemd\0--user\0', False),
    ('(sd-pam)', b'/usr/bin/python3\0--user\0', False),
    ('(sd-pam)', b'/lib/systemd/systemd\0', False),
])
def test_only_known_systemd_session_helper_can_be_excluded(tmp_path, name, parent_command,
                                                         expected):
    helper = tmp_path/'2257'
    parent = tmp_path/'2256'
    helper.mkdir()
    parent.mkdir()
    (helper/'comm').write_text(name)
    (helper/'status').write_text('PPid:\t2256\n')
    (parent/'comm').write_text('systemd')
    (parent/'cmdline').write_bytes(parent_command)
    node = next(n for n in ast.parse(RELEASE_CHECK).body
                if isinstance(n, ast.FunctionDef) and n.name == 'systemd_session_helper')
    scope = dict(Path=Path)
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<release-check>', 'exec'), scope)
    assert scope['systemd_session_helper'](helper) is expected


def test_release_check_error_preserves_remote_stderr_without_embedded_script(tmp_path, monkeypatch):
    import go2_full_test
    w = workflow(tmp_path)

    def failure(*a, **kw):
        raise subprocess.CalledProcessError(1, ['ssh', 'long script'], stderr='Permission denied')

    monkeypatch.setattr(go2_full_test, 'remote', failure)
    with pytest.raises(RuntimeError, match='Camera release check failed: Permission denied') as exc:
        w.cameras_released({'0b5c'})
    assert 'long script' not in str(exc.value)


@pytest.mark.parametrize('parent_uid,parent_name,child_name,expected', [
    (0, 'sshd', 'sshd', True),
    (1000, 'sshd', 'sshd', False),
    (0, 'python3', 'sshd', False),
    (0, 'sshd', 'python3', False),
])
def test_unreadable_ssh_session_requires_privileged_sshd_parent(
        tmp_path, monkeypatch, parent_uid, parent_name, child_name, expected):
    helper, parent = tmp_path/'10', tmp_path/'9'
    helper.mkdir()
    parent.mkdir()
    (helper/'comm').write_text(child_name)
    (helper/'status').write_text('PPid:\t9\n')
    (helper/'cmdline').write_bytes(('sshd: '+pwd.getpwuid(os.getuid()).pw_name+'@notty\0').encode())
    (parent/'comm').write_text(parent_name)
    original_stat = Path.stat

    def stat(path, *a, **kw):
        return (SimpleNamespace(st_uid=parent_uid) if path == parent else
                original_stat(path, *a, **kw))

    monkeypatch.setattr(Path, 'stat', stat)
    node = next(n for n in ast.parse(RELEASE_CHECK).body
                if isinstance(n, ast.FunctionDef) and n.name == 'systemd_session_helper')
    scope = dict(Path=Path, os=os, pwd=pwd)
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<release-check>', 'exec'), scope)
    assert scope['systemd_session_helper'](helper) is expected


def test_camera_only_recovery_cannot_start_memory_round(tmp_path, monkeypatch):
    w = workflow(tmp_path)
    w.camera_paused = True
    monkeypatch.setattr(w, 'status', lambda: dict(state='ZERO_HOLD', busy=False))
    monkeypatch.setattr(w, 'cameras_released', lambda _: None)
    monkeypatch.setattr(w, 'probe', lambda *a, **kw: dict(camera_ok=True, ready=False))
    import go2_full_test as full
    monkeypatch.setattr(full, 'remote', lambda *a, **kw: None)
    with pytest.raises(RuntimeError, match='full RGB-D/localization/TF chain not ready'):
        w.resume_front_camera()
