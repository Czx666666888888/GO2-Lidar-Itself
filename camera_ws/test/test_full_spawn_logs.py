"""Exercise repeated child startup without ROS, network, serial, or motion."""
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
import go2_full_test as launcher  # noqa: E402


def workflow(tmp_path):
    w = launcher.Workflow(SimpleNamespace(work_dir=ROOT, mode='full'))
    w.folder = tmp_path
    return w


def spawn_text(w, text, label='slam'):
    child = w.spawn([sys.executable, '-c', f'print({text!r}, flush=True)'], label)
    assert child.wait(timeout=5) == 0
    w.children[-1][1].flush()
    return child


def test_reinitialized_slam_gets_new_log_after_original_child_is_reaped(tmp_path, monkeypatch):
    w = workflow(tmp_path)
    spawn_text(w, 'first attempt')
    before = (tmp_path/'slam.log').read_bytes()
    original_glob = Path.glob
    monkeypatch.setattr(Path, 'glob', lambda p, pattern: iter(()) if str(p) == '/proc'
                        else original_glob(p, pattern))
    monkeypatch.setattr(launcher.subprocess, 'run',
                        lambda *a, **kw: SimpleNamespace(returncode=1, stdout=''))
    original_stream = w.children[0][1]
    w.stop_invalid_localization()
    assert not w.children and original_stream.closed
    spawn_text(w, 'reinitialized attempt')
    assert w.children[0][2] == 'slam'  # Lifecycle matching must still see the same label.
    assert (tmp_path/'slam.log').read_bytes() == before
    assert (tmp_path/'slam_2.log').read_text() == 'reinitialized attempt\n'
    w.children[0][1].close()


@pytest.mark.parametrize('label', ['slam', 'camera', 'viewer'])
def test_existing_numbered_logs_are_not_overwritten_or_appended(tmp_path, label):
    w = workflow(tmp_path)
    (tmp_path/(label+'.log')).write_bytes(b'original\x00evidence')
    (tmp_path/(label+'_2.log')).write_text('second evidence')
    spawn_text(w, 'third attempt', label)
    assert (tmp_path/(label+'.log')).read_bytes() == b'original\x00evidence'
    assert (tmp_path/(label+'_2.log')).read_text() == 'second evidence'
    assert (tmp_path/(label+'_3.log')).read_text() == 'third attempt\n'
    assert w.children[0][2] == label
    w.children[0][1].close()


def test_failed_popen_closes_stream_and_next_attempt_preserves_that_log(tmp_path, monkeypatch):
    w = workflow(tmp_path)
    streams = []
    popen = launcher.subprocess.Popen

    def fail(*args, **kwargs):
        streams.append(kwargs['stdout'])
        raise OSError('synthetic spawn failure')

    monkeypatch.setattr(launcher.subprocess, 'Popen', fail)
    with pytest.raises(OSError, match='synthetic spawn failure'):
        w.spawn(['unused'], 'slam')
    assert streams[0].closed and not w.children
    assert (tmp_path/'slam.log').exists()
    monkeypatch.setattr(launcher.subprocess, 'Popen', popen)
    spawn_text(w, 'recovered')
    assert (tmp_path/'slam.log').read_bytes() == b''
    assert (tmp_path/'slam_2.log').read_text() == 'recovered\n'
    w.children[0][1].close()


def test_live_child_with_same_label_is_rejected_before_creating_any_log(tmp_path):
    w = workflow(tmp_path)
    w.children.append((SimpleNamespace(poll=lambda: None), None, 'slam'))
    with pytest.raises(RuntimeError, match='仍在运行'):
        w.spawn(['must-not-launch'], 'slam')
    assert not list(tmp_path.iterdir())
