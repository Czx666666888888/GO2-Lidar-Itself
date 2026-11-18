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


def test_full_view_starts_two_independent_optional_children(tmp_path, monkeypatch):
    w = workflow(tmp_path)
    w.args.view = True
    monkeypatch.setenv('DISPLAY', ':99')
    children = []
    calls = []

    def popen(argv, **kwargs):
        calls.append(argv[-1])
        child = SimpleNamespace(pid=len(children) + 1, returncode=None)
        child.poll = lambda: child.returncode
        children.append(child)
        return child

    monkeypatch.setattr(launcher.subprocess, 'Popen', popen)
    try:
        w.start_viewers()
        assert [label for _, _, label in w.children] == ['image_viewer', 'rviz']
        assert 'rqt_image_view' in calls[0]
        assert 'rviz2' in calls[1]
        assert {p.name for p in tmp_path.iterdir()} == {'image_viewer.log', 'rviz.log'}
        for child in children:
            child.returncode = 1
        w.healthy()
    finally:
        for _, stream, _ in w.children:
            stream.close()


def test_show_status_waits_for_complete_record_and_skips_unrelated_output(tmp_path, capsys):
    from go2_terminal_status import show_status

    path = tmp_path/'round_01_coordinator.log'
    with path.open('w') as writer, path.open(encoding='utf-8') as reader:
        writer.write('unrelated child output\nFULL_STATUS {"event":"MISSION_PHASE",')
        writer.flush()
        show_status(reader, '第 1 轮')
        assert not capsys.readouterr().out

        writer.write('"state":"EXPLORE"}\n')
        writer.flush()
        show_status(reader, '第 1 轮')
        assert capsys.readouterr().out == '[第 1 轮] 导航阶段 | 状态=EXPLORE\n'
        show_status(reader, '第 1 轮')
        assert not capsys.readouterr().out


def test_live_status_is_shown_before_task_process_exits(tmp_path, monkeypatch):
    import builtins
    import subprocess
    import threading
    from go2_terminal_status import follow_status

    report_received = threading.Event()
    output = []
    def capture(message, **kwargs):
        output.append(message)
        report_received.set()

    monkeypatch.setattr(builtins, 'print', capture)
    path = tmp_path/'round_00_coordinator.log'
    stop = threading.Event()
    code = ('print(\'FULL_STATUS {"event":"MISSION_PHASE","state":"RETURN"}\',flush=True); '
            'import sys; sys.stdin.readline()')
    with path.open('w') as writer, path.open(encoding='utf-8') as reader:
        with subprocess.Popen([sys.executable, '-c', code], stdin=subprocess.PIPE,
                              stdout=writer) as process:
            thread = threading.Thread(target=follow_status, args=(reader, '返航', stop))
            thread.start()
            try:
                assert report_received.wait(3.)
                assert process.poll() is None
                assert output == ['[返航] 导航阶段 | 状态=RETURN']
            finally:
                stop.set()
                thread.join(timeout=3.)
                process.stdin.close()
                process.wait(timeout=3.)
            assert not thread.is_alive()
