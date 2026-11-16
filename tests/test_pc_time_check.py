"""Offline fault injection: no real systemctl, sudo, SSH or robot commands."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('pc_time_check', ROOT/'camera_ws/scripts/pc_time_check.py')
clock = importlib.util.module_from_spec(spec)
spec.loader.exec_module(clock)


@pytest.mark.parametrize('code,out,err,expected', [
    (0, 'active\n', '', True), (3, 'inactive\n', '', False),
    (1, '', "Failed to activate service 'org.freedesktop.systemd1'", None),
    (3, '', 'Failed to retrieve unit state', None),
    (4, 'unknown', '', None), (3, 'failed', '', None),
    (0, '', '', None), (3, 'inactive', 'D-Bus error', None),
])
def test_only_explicit_states_are_accepted(monkeypatch, code, out, err, expected):
    runner = Mock(return_value=SimpleNamespace(returncode=code, stdout=out, stderr=err))
    monkeypatch.setattr(clock.subprocess, 'run', runner)
    if expected is None:
        with pytest.raises(RuntimeError):
            clock.service_active()
    else:
        assert clock.service_active() is expected
    assert runner.call_args.kwargs['timeout'] == 3


def setup_main(monkeypatch, tmp_path):
    monkeypatch.setattr(clock, 'ROOT', tmp_path)
    monkeypatch.setattr(sys, 'argv', ['pc_time_check', '--execute', '--mode', 'full', '--view'])
    monkeypatch.setattr(clock.signal, 'signal', lambda *args: None)
    # Default deny all actual subprocess calls.
    runner = Mock(side_effect=AssertionError('unexpected subprocess'))
    monkeypatch.setattr(clock.subprocess, 'run', runner)
    launch = Mock(side_effect=AssertionError('must not launch hardware'))
    monkeypatch.setattr(clock.subprocess, 'Popen', launch)
    return runner, launch


def report(tmp_path):
    return json.loads(next(tmp_path.glob('camera_ws/records/pc_time_*/report.json')).read_text())


def test_initial_timeout_blocks_launch_and_persists_elapsed(monkeypatch, tmp_path):
    runner, launch = setup_main(monkeypatch, tmp_path)
    now = [10.]
    monkeypatch.setattr(clock.time, 'monotonic', lambda: now[0])
    def timeout(*args, **kwargs):
        now[0] += 3
        raise subprocess.TimeoutExpired(args[0], kwargs['timeout'])
    runner.side_effect = timeout
    with pytest.raises(RuntimeError, match='超过3秒'):
        clock.main()
    assert runner.call_count == 1
    launch.assert_not_called()
    state = report(tmp_path)
    assert 'was_active' not in state
    assert state['time_context_failed_sec'] == 3
    assert state['elapsed_sec'] == 3
    assert 'error' in state


def test_bus_error_never_becomes_inactive(monkeypatch, tmp_path, capsys):
    runner, launch = setup_main(monkeypatch, tmp_path)
    runner.side_effect = None
    runner.return_value = SimpleNamespace(returncode=1, stdout='', stderr='D-Bus timed out')
    with pytest.raises(RuntimeError, match='D-Bus timed out'):
        clock.main()
    launch.assert_not_called()
    assert 'was_active' not in report(tmp_path)
    assert '快速启动' not in capsys.readouterr().out


def test_known_inactive_keeps_original_command_and_counts_query_time(monkeypatch, tmp_path):
    runner, launch = setup_main(monkeypatch, tmp_path)
    now = [0.]
    monkeypatch.setattr(clock.time, 'monotonic', lambda: now[0])
    def inactive(*args, **kwargs):
        now[0] += .4
        return SimpleNamespace(returncode=3, stdout='inactive\n', stderr='')
    runner.side_effect = inactive
    launch.side_effect = None
    launch.return_value = SimpleNamespace(poll=lambda: 0, wait=lambda: 0)
    with pytest.raises(SystemExit) as exc:
        clock.main()
    assert exc.value.code == 0
    assert '--view' in launch.call_args.args[0]
    assert launch.call_args.kwargs['env']['GO2_PC_TIME_HELD'] == '1'
    state = report(tmp_path)
    assert state['time_context_ready_sec'] == pytest.approx(.8)
    assert state['full_clock_diagnostic'] is False


def test_monitor_failure_stops_child_before_restoration(monkeypatch, tmp_path):
    runner, launch = setup_main(monkeypatch, tmp_path)
    states = iter([True, False, RuntimeError('monitor D-Bus failure'), True])
    def status():
        value = next(states)
        if isinstance(value, Exception):
            raise value
        return value
    monkeypatch.setattr(clock, 'service_active', status)
    monkeypatch.setattr(clock, 'time_status', lambda: 'OK')
    events = []
    monkeypatch.setattr(clock, 'run', lambda args, **kwargs: events.append(args[-2]))
    child = SimpleNamespace(poll=lambda: None if 'interrupt' not in events else 0,
                            send_signal=lambda sig: events.append('interrupt'))
    launch.side_effect = None
    launch.return_value = child
    def restore(args, **kwargs):
        assert 'interrupt' in events
        assert kwargs['timeout'] == 10
        events.append('restore')
        return SimpleNamespace(returncode=0)
    runner.side_effect = restore
    with pytest.raises(RuntimeError, match='monitor D-Bus failure'):
        clock.main()
    assert events.index('interrupt') < events.index('restore')
    assert report(tmp_path)['service_active_after'] is True


def test_restore_failure_still_writes_report(monkeypatch, tmp_path):
    runner, launch = setup_main(monkeypatch, tmp_path)
    states = iter([True, False])
    monkeypatch.setattr(clock, 'service_active', lambda: next(states))
    monkeypatch.setattr(clock, 'time_status', lambda: 'OK')
    monkeypatch.setattr(clock, 'run', lambda *args, **kwargs: None)
    runner.side_effect = subprocess.TimeoutExpired('systemctl start', 10)
    launch.side_effect = None
    launch.return_value = SimpleNamespace(poll=lambda: 0, wait=lambda: 0)
    with pytest.raises(RuntimeError):
        clock.main()
    state = report(tmp_path)
    assert 'cleanup_error' in state
    assert state['service_active_after'] is None


def test_stop_timeout_attempts_restore_without_launch(monkeypatch, tmp_path):
    runner, launch = setup_main(monkeypatch, tmp_path)
    monkeypatch.setattr(clock, 'service_active', lambda: True)
    monkeypatch.setattr(clock, 'time_status', lambda: 'OK')
    def stop(args, **kwargs):
        if 'stop' in args:
            assert kwargs['timeout'] == 10
            raise subprocess.TimeoutExpired(args, 10)
    monkeypatch.setattr(clock, 'run', stop)
    runner.side_effect = None
    runner.return_value = SimpleNamespace(returncode=0)
    with pytest.raises(subprocess.TimeoutExpired):
        clock.main()
    launch.assert_not_called()
    assert runner.call_args.args[0][-2:] == ['start', 'systemd-timesyncd']
    assert report(tmp_path)['restore_exit'] == 0


def test_main_shell_uses_guard_before_integration_handoff(tmp_path):
    # Replace only the Python executable with a harmless argument recorder.
    shell = (ROOT/'go2_test.sh').read_text()
    recorder = tmp_path/'record'
    recorder.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    recorder.chmod(0o755)
    entry = tmp_path/'go2_test.sh'
    entry.write_text(shell.replace('/usr/bin/python3', str(recorder)))
    result = subprocess.run(['bash', str(entry), '--execute', '--mode', 'full', '--view'],
                            capture_output=True, text=True, check=True,
                            env={'PATH': '/usr/bin:/bin'})
    assert str(tmp_path/'camera_ws/scripts/pc_time_check.py') in result.stdout
    assert '--view' in result.stdout
