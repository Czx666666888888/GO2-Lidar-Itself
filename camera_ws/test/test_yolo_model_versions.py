"""Offline model identity, rollback and readiness propagation checks."""
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
import go2_color_block_test as assets
from go2_full_test import Workflow


@pytest.mark.parametrize('version', ['v3', 'v2'])
def test_default_v3_and_explicit_v2_rollback(tmp_path, monkeypatch, version):
    monkeypatch.setenv('GO2_VISION_ROOT', str(tmp_path))
    model = tmp_path/f'vision_training/runs/science_targets_mixed_{version}/baseline/weights/best.pt'
    model.parent.mkdir(parents=True)
    model.write_bytes(version.encode())
    python = tmp_path/'vision_training/.venv/bin/python'
    python.parent.mkdir(parents=True)
    python.touch()
    digest = hashlib.sha256(model.read_bytes()).hexdigest()
    monkeypatch.setattr(assets, 'VERIFIED_MODELS', {digest: f'science_targets_mixed_{version}'})
    args = SimpleNamespace(detector='yolo', model_path=model if version == 'v2' else None)
    assert assets.detector_assets(args, ROOT) == (model, python)
    w = Workflow(SimpleNamespace(work_dir=ROOT, mode='full', detector='yolo',
                                 grasp_side='auto', view=False, model_path=args.model_path))
    w.folder = tmp_path
    (tmp_path/'manifest.json').write_text('{}')
    w.configure_vision()
    manifest = json.loads((tmp_path/'manifest.json').read_text())
    assert manifest['model_sha256'] == digest
    assert manifest['model_version'] == f'science_targets_mixed_{version}'
    calls = []
    def probe(command):
        calls.append(command)
        (tmp_path/'check.json').write_text('{"errors": []}')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(w, 'shell', lambda command: command)
    monkeypatch.setattr(assets.subprocess, 'run', probe)
    w.probe('check', scope='chain')
    import shlex
    argv = shlex.split(calls[0])
    assert argv[argv.index('--model-sha256')+1] == digest
    assert 'yolo_model:='+str(model) in w.perception_command()


def test_unknown_weights_rejected_before_startup(tmp_path):
    model = tmp_path/'unknown.pt'
    model.write_bytes(b'not a released checkpoint')
    with pytest.raises(RuntimeError, match='differs from verified'):
        assets.detector_assets(SimpleNamespace(detector='yolo', model_path=model,
                                               python_executable=Path(sys.executable)))


def test_missing_v3_does_not_silently_fall_back(tmp_path, monkeypatch):
    monkeypatch.setenv('GO2_VISION_ROOT', str(tmp_path))
    with pytest.raises(RuntimeError, match='Missing local V3'):
        assets.detector_assets(SimpleNamespace(detector='yolo'))


def test_hsv_does_not_require_weights(tmp_path, monkeypatch):
    monkeypatch.setenv('GO2_VISION_ROOT', str(tmp_path))
    assets.detector_assets(SimpleNamespace(detector='hsv'))


@pytest.mark.parametrize('explicit', [False, True])
def test_readonly_probe_before_configure_uses_selected_identity(tmp_path, monkeypatch, explicit):
    model = tmp_path/'v2.pt'
    model.write_bytes(b'v2')
    digest = hashlib.sha256(model.read_bytes()).hexdigest()
    monkeypatch.setattr(assets, 'VERIFIED_MODELS', {digest: 'science_targets_mixed_v2'})
    w = Workflow(SimpleNamespace(work_dir=ROOT, mode='full', detector='yolo',
                                 grasp_side='auto', view=False,
                                 model_path=model if explicit else None))
    w.folder = tmp_path
    calls = []
    def probe(command):
        calls.append(command)
        (tmp_path/'readonly_current.json').write_text('{"errors": []}')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(w, 'shell', lambda command: command)
    monkeypatch.setattr(assets.subprocess, 'run', probe)
    w.probe('readonly_current')
    import shlex
    argv = shlex.split(calls[0])
    assert argv[argv.index('--model-sha256')+1] == (digest if explicit else assets.V3_SHA256)
