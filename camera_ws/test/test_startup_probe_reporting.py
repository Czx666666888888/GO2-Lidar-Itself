"""Readiness reporting faults are tested without ROS, devices, network or motion."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'startup_reporting', ROOT/'camera_ws/scripts/go2_startup_probe.py')
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def arguments(monkeypatch, tmp_path):
    path = tmp_path/'pre_motion.json'
    monkeypatch.setattr(sys, 'argv', [
        'probe', '--scope', 'chain', '--detector', 'hsv', '--report', str(path)])
    return path


def test_normal_success_preserves_ready_result(monkeypatch, tmp_path):
    path = arguments(monkeypatch, tmp_path)

    def run(*args, progress):
        assert json.loads(path.read_text())['probe_state'] == 'initializing'
        value = dict(ready=True, camera_ok=True, slam_ok=True, body_tf=True, camera_tf=True,
                     use_sim_time_false=True, detector_ready=True, elapsed_sec=1.)
        progress(value)
        return value

    monkeypatch.setattr(probe, 'probe', run)
    assert probe.main() == 0
    result = json.loads(path.read_text())
    assert result['ready'] and result['probe_state'] == 'complete'
    assert result['blockers'] == []
    assert not path.with_name(path.name+'.tmp').exists()


@pytest.mark.parametrize('error', [KeyboardInterrupt(), RuntimeError('synthetic DDS failure')])
def test_interrupt_or_exception_keeps_last_sampling_evidence(monkeypatch, tmp_path, error):
    path = arguments(monkeypatch, tmp_path)

    def run(*args, progress):
        progress(dict(ready=False, elapsed_sec=12., sample_counts=dict(rgb=100, depth=1),
                      stream_status=dict(depth=dict(fresh=False))))
        assert json.loads(path.read_text())['sample_counts']['depth'] == 1
        raise error

    monkeypatch.setattr(probe, 'probe', run)
    with pytest.raises(type(error)):
        probe.main()
    result = json.loads(path.read_text())
    assert result['sample_counts'] == dict(rgb=100, depth=1)
    assert not result['ready'] and 'depth_missing_or_stale' in result['blockers']
    expected = 'interrupted' if isinstance(error, KeyboardInterrupt) else 'error'
    assert result['probe_state'] == expected
    assert type(error).__name__ in result['exception_traceback']


def test_timeout_reports_specific_blockers_and_remains_not_ready(monkeypatch, tmp_path):
    path = arguments(monkeypatch, tmp_path)
    monkeypatch.setattr(probe, 'probe', lambda *a, **kw: dict(
        ready=False, elapsed_sec=30., camera_ok=False, slam_ok=True, body_tf=True,
        camera_tf=False, use_sim_time_false=True, detector_ready=False,
        stream_status=dict(rgb=dict(fresh=True), depth=dict(fresh=False))))
    assert probe.main() == 2
    result = json.loads(path.read_text())
    assert result['probe_state'] == 'complete'
    assert set(result['blockers']) == {
        'depth_missing_or_stale', 'camera_ok', 'camera_tf', 'detector_not_ready'}
