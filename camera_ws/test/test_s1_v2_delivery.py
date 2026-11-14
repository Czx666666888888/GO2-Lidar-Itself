"""Focused v2 adapter tests; fake feedback only, no SDK/serial/camera imports."""
import ast
import base64
import json
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from s1_v2_compat import compatibility_payload, replace_once
from s1_v2_delivery import ensure_v2_compat
from go2_full_test import Workflow, zero_hold


@pytest.fixture
def payload():
    path = os.environ.get('GO2_S1_V2_ARCHIVE')
    if not path:
        pytest.skip('Set GO2_S1_V2_ARCHIVE to test the actual user delivery')
    with zipfile.ZipFile(path) as bundle:
        return {name: base64.b64decode(meta['content']).decode()
                for name, meta in compatibility_payload(ROOT, bundle).items()}


def test_unknown_upstream_layout_is_preserved():
    with pytest.raises(ValueError):
        replace_once('changed upstream', 'expected source', 'replacement')


@pytest.mark.parametrize('at_zero', [True, False])
def test_startup_uses_original_return_only_when_needed(payload, at_zero):
    source = payload['scripts/run_arm_integration.py']
    tree = ast.parse(source)
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    startup = next(n for n in main.body if isinstance(n, ast.Try))
    begin = next(i for i, n in enumerate(startup.body)
                 if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                 and n.targets[0].id == 'before')
    end = next(i for i, n in enumerate(startup.body)
               if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call)
               and isinstance(n.value.func, ast.Attribute) and n.value.func.attr == 'ready')
    block = ast.Module(body=startup.body[begin:end+1], type_ignores=[])
    calls = []
    values = iter([dict(at_zero=at_zero), dict(at_zero=True)])
    arm = SimpleNamespace(enable=lambda: calls.append('enable'),
        move=lambda q, target, name: calls.append((q, target, name)),
        limits=SimpleNamespace(max_temp_c=55))
    session = SimpleNamespace(_update=lambda **kw: None,
        start_hold=lambda: calls.append('hold'), ready=lambda: calls.append('ready'))
    q = [.1]*6
    scope = dict(arm=arm, session=session, args=SimpleNamespace(dev='/dev/ttyUSB1'),
        device_identity=(188, 1), feedback_status=lambda *a, **kw: next(values),
        json=json, q=q, ZERO_Q=[0.]*6,
        wait_for_valid_feedback=lambda *a: calls.append('valid_feedback'))
    exec(compile(block, '<v2 startup>', 'exec'), scope)
    assert calls == (['enable', 'hold', 'ready'] if at_zero else
        ['enable', (q, [0.]*6, 'startup_zero'), 'valid_feedback', 'hold', 'ready'])


def test_live_snapshot_fails_closed_for_serial_errors(payload, monkeypatch):
    package = ModuleType('v2_adapter_test');package.__path__ = []
    monkeypatch.setitem(sys.modules, package.__name__, package)
    helper = ModuleType(package.__name__+'.startup_status')
    exec(payload['s1_control/startup_status.py'], helper.__dict__)
    monkeypatch.setitem(sys.modules, helper.__name__, helper)
    module = ModuleType(package.__name__+'.integration');module.__package__ = package.__name__
    exec(payload['s1_control/integration.py'], module.__dict__)
    arm = SimpleNamespace(limits=SimpleNamespace(max_temp_c=55, max_track_err_rad=.08),
        get_pos=lambda: [0.]*7, get_vel=lambda: [0.]*7, get_temp=lambda: [25.]*7)
    session = module.ArmIntegrationSession(arm);session.ready()
    assert zero_hold(session.snapshot())
    arm.get_pos = lambda: [.12]*7
    assert not zero_hold(session.snapshot())
    def io_error():
        raise OSError('Input/output error')
    arm.get_pos = io_error
    state = session.snapshot()
    assert not zero_hold(state)
    assert state['readiness_state'] == 'COMMUNICATION_ERROR'


def test_running_verified_compat_reuses_without_archive(monkeypatch, tmp_path):
    import s1_v2_delivery as delivery
    calls = []
    def run(*args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout='READY\n')
    monkeypatch.setattr(delivery.subprocess, 'run', run)
    ensure_v2_compat(tmp_path, ['ssh'], 'host', running=True)
    assert len(calls) == 1


def test_running_unpatched_service_never_patched(monkeypatch, tmp_path):
    import s1_v2_delivery as delivery
    monkeypatch.setattr(delivery.subprocess, 'run',
                        lambda *a, **kw: SimpleNamespace(stdout='MISSING\n'))
    with pytest.raises(RuntimeError, match='running'):
        ensure_v2_compat(tmp_path, ['ssh'], 'host', running=True)


def test_field_light_v2_status_and_no_zero_rpc():
    assert zero_hold(dict(state='ZERO_HOLD', busy=False, navigation_allowed=True))
    workflow = Workflow(SimpleNamespace(work_dir=ROOT, mode='dock', grasp_side='right'))
    with pytest.raises(RuntimeError, match='only status/grasp'):
        workflow.arm_command('zero')
