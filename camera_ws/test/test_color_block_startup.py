"""Mock-only entry/clock regression checks; no SSH, sudo, ROS nodes or motion."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
import go2_color_block_test as color
import go2_full_test as transport


@pytest.mark.parametrize('running', [False, True])
def test_healthy_clock_reuses_without_sync(monkeypatch, running):
    calls = []
    monkeypatch.setattr(color.subprocess, 'run', lambda *a, **k:
        calls.append(a[0]) or SimpleNamespace(returncode=0 if running else 1))
    monkeypatch.setattr(transport, 'verify_clock', lambda _: None)
    assert color.prepare_remote_camera_clock(['ssh'], 'host') is running
    assert len(calls) == 1


def test_skew_before_camera_syncs_once_then_rechecks(monkeypatch):
    calls, checks = [], []
    def run(argv, **kw):
        calls.append(argv)
        return SimpleNamespace(returncode=1 if argv[0] == 'ssh' else 0)
    def verify(argv):
        checks.append(argv)
        if len(checks) == 1:
            raise transport.ClockOffsetError('clock returned to 2022')
    monkeypatch.setattr(color.subprocess, 'run', run)
    monkeypatch.setattr(transport, 'verify_clock', verify)
    assert color.prepare_remote_camera_clock(['ssh'], 'host') is False
    assert len(checks) == 2 and checks[0] == checks[1]
    assert len(calls) == 2 and calls[1][-1].endswith('sync_go2_clock.py')


@pytest.mark.parametrize('error,running', [
    (transport.ClockOffsetError('skew'), True),
    (RuntimeError('SSH RTT too large'), False),
    (RuntimeError('handshake failed'), False),
])
def test_active_camera_or_transport_failure_never_changes_clock(monkeypatch, error, running):
    calls = []
    monkeypatch.setattr(color.subprocess, 'run', lambda *a, **k:
        calls.append(a[0]) or SimpleNamespace(returncode=0 if running else 1))
    def verify(_): raise error
    monkeypatch.setattr(transport, 'verify_clock', verify)
    with pytest.raises(RuntimeError): color.prepare_remote_camera_clock(['ssh'], 'host')
    assert len(calls) == 1


def test_sync_failure_never_proceeds(monkeypatch):
    calls = []
    def run(argv, **kw):
        calls.append(argv)
        if len(calls) > 1: raise subprocess.CalledProcessError(1, argv)
        return SimpleNamespace(returncode=1)
    def verify(_): raise transport.ClockOffsetError('skew')
    monkeypatch.setattr(color.subprocess, 'run', run)
    monkeypatch.setattr(transport, 'verify_clock', verify)
    with pytest.raises(subprocess.CalledProcessError):
        color.prepare_remote_camera_clock(['ssh'], 'host')
    assert len(calls) == 2


def test_failed_recheck_never_proceeds(monkeypatch):
    calls = []
    monkeypatch.setattr(color.subprocess, 'run', lambda *a, **k:
        calls.append(a[0]) or SimpleNamespace(returncode=1 if len(calls) == 1 else 0))
    def verify(_): raise transport.ClockOffsetError('persistent skew')
    monkeypatch.setattr(transport, 'verify_clock', verify)
    with pytest.raises(transport.ClockOffsetError):
        color.prepare_remote_camera_clock(['ssh'], 'host')
    assert len(calls) == 2


def test_approach_view_arguments_are_preserved():
    args = color.arguments(['--execute', '--mode', 'approach', '--view'])
    assert args.execute and args.view and args.mode == 'approach'
    assert not args.preview


@pytest.mark.parametrize('rtt,stamp,skew', [(.01,0.,True),(.30,100.15,False)])
def test_clock_skew_is_distinct_from_slow_transport(monkeypatch,rtt,stamp,skew):
    import io
    process=SimpleNamespace(stdin=io.StringIO(),
        stdout=io.StringIO('CLOCK_READY\n'+str(stamp)+'\n'),wait=lambda **kw:0)
    monkeypatch.setattr(transport.subprocess,'Popen',lambda *a,**kw:process)
    monkeypatch.setattr(transport.select,'select',lambda *a:([process.stdout],[],[]))
    clock=iter([100.,100.+rtt])
    monkeypatch.setattr(transport.time,'time',lambda:next(clock))
    with pytest.raises(RuntimeError) as caught: transport.verify_clock(['mock_clock'])
    assert isinstance(caught.value,transport.ClockOffsetError) is skew
