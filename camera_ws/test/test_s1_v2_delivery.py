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


@pytest.mark.parametrize('side', ['right', 'left'])
def test_side_selection_reuses_service_parameters_and_original_side_resources(payload, side):
    import argparse
    source = ast.parse(payload['scripts/run_arm_integration.py'])
    function = next(n for n in source.body if isinstance(n, ast.FunctionDef)
                    and n.name == 'selected_grasp_arguments')
    namespace = dict(argparse=argparse)
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<side-dispatch>', 'exec'), namespace)
    args = argparse.Namespace(speed=1.5, descent_speed=.75, dev='/dev/ttyUSB1',
                              non_interactive=True, config='original', search_plan=[])
    selected = namespace['selected_grasp_arguments'](args, side)
    assert selected.speed == args.speed and selected.descent_speed == args.descent_speed
    assert selected.dev == args.dev and selected.non_interactive == args.non_interactive
    assert ('left' in selected.config) == (side == 'left')
    assert all(side in value for value in selected.search_plan)
    assert args.config == 'original'


def test_existing_default_side_retains_custom_resource_arguments(payload):
    import argparse
    function = next(n for n in ast.parse(payload['scripts/run_arm_integration.py']).body
                    if isinstance(n, ast.FunctionDef) and n.name == 'selected_grasp_arguments')
    namespace = dict(argparse=argparse)
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<side-default>', 'exec'), namespace)
    args = argparse.Namespace(config='custom_right.json', search_plan=['custom_a', 'custom_b'],
                              speed=1.5)
    chosen = namespace['selected_grasp_arguments'](args, 'right')
    assert vars(chosen) == vars(args) and chosen is not args


def test_grasp_side_is_latched_with_the_queued_job_and_cannot_change_while_busy(payload):
    import queue
    import threading
    source = ast.parse(payload['s1_control/integration.py'])
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef))
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'request_grasp')
    scope = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<side-request>', 'exec'), scope)
    session = SimpleNamespace(_lock=threading.Lock(), jobs=queue.Queue(),
                              _state=dict(state='ZERO_HOLD', grasp_side='right'))
    session.snapshot = lambda: dict(readiness_state='ZERO_HOLD')
    call = scope['request_grasp']
    assert not call(session, 'invalid')['accepted']
    assert call(session, 'left')['accepted'] and session.jobs.qsize() == 1
    assert session._state['grasp_side'] == 'left'
    assert not call(session, 'right')['accepted']
    assert session._state['grasp_side'] == 'left' and session.jobs.qsize() == 1


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
    arm.get_pos = lambda: [.026]*7
    assert zero_hold(session.snapshot())  # Original failure: permission stayed false forever.
    session._update(busy=True)
    assert not zero_hold(session.snapshot())
    session._update(busy=False)
    session.mark_fault('hold failure')
    assert not zero_hold(session.snapshot())
    session._update(state='FAULT_IO')
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


@pytest.mark.parametrize('state', ['MISSING', 'UPGRADE'])
def test_running_unpatched_service_never_patched(monkeypatch, tmp_path, state):
    import s1_v2_delivery as delivery
    monkeypatch.setattr(delivery.subprocess, 'run',
                        lambda *a, **kw: SimpleNamespace(stdout=state+'\n'))
    with pytest.raises(RuntimeError, match='running'):
        ensure_v2_compat(tmp_path, ['ssh'], 'host', running=True)


def test_field_light_v2_status_and_no_zero_rpc():
    assert zero_hold(dict(state='ZERO_HOLD', busy=False, navigation_allowed=True))
    workflow = Workflow(SimpleNamespace(work_dir=ROOT, mode='dock', grasp_side='right'))
    with pytest.raises(RuntimeError, match='only status/grasp'):
        workflow.arm_command('zero')


def test_upgrade_accepts_exact_deployed_previous_adapter():
    path = os.environ.get('GO2_S1_V2_ARCHIVE')
    if not path:
        pytest.skip('Set GO2_S1_V2_ARCHIVE to verify upstream version')
    with zipfile.ZipFile(path) as bundle:
        meta = compatibility_payload(ROOT, bundle)['s1_control/integration.py']
    assert meta['accepted_previous_sha256'][:2] == [
        'b98a9881893686af2e998d3cf60874c616c1de6290dd426a0a6822d2bce52d87',
        '384dea03626da4634821ab79fddf327fd0d476beb895284ae1a50e0d7d63d6d6']
    assert meta['sha256'] != meta['accepted_previous_sha256'][0]


def test_video_removal_restores_original_capture_and_direct_grasp_callback():
    path = os.environ.get('GO2_S1_V2_ARCHIVE')
    if not path:
        pytest.skip('Set GO2_S1_V2_ARCHIVE to verify upstream version')
    with zipfile.ZipFile(path) as bundle:
        payload = compatibility_payload(ROOT, bundle)
        upstream = bundle.read('s1-arm-control/demo_grasp/capture_rgbd.py').decode()
    capture = base64.b64decode(payload['demo_grasp/capture_rgbd.py']['content']).decode()
    assert capture == upstream
    assert 's1_control/grasp_video.py' not in payload
    entry = base64.b64decode(payload['scripts/run_arm_integration.py']['content']).decode()
    assert 'run_recorded_flow' not in entry and 'grasp_video' not in entry
    assert 'session.run_task(lambda active: flow.main(' in entry
    assert payload['scripts/run_arm_integration.py']['accepted_previous_sha256'][:2] == [
        '035fd4e6d73946b38e0c06ef0eaa6d3e571cb9b5b784206d7f4f0a79a06755b7',
        '7abc5219e43b944667e6a867e256a5dc0e640f1b926ee7769255bc9d209642e7']
    assert payload['demo_grasp/capture_rgbd.py']['accepted_previous_sha256'] == [
        '8c8150066467e09bd4ae5520b0409d9559be4f0ef55e8c09f4aef10fcaf5de81']


def test_upgrade_recovers_hash_verified_installed_originals(tmp_path, monkeypatch):
    import s1_v2_delivery as delivery
    import hashlib
    import shlex
    import subprocess
    path = os.environ.get('GO2_S1_V2_ARCHIVE')
    if not path:
        pytest.skip('Set GO2_S1_V2_ARCHIVE to verify upstream version')
    with zipfile.ZipFile(path) as archive:
        names = ['scripts/run_arm_integration.py','s1_control/integration.py',
                 'scripts/arm_task_client.py','scripts/verify_handoff.py','demo_grasp/capture_rgbd.py',
                 'scripts/run_go2_visual_alignment.py']
        hashes = []
        for name in names:
            content = archive.read('s1-arm-control/'+name)
            destination = tmp_path/'.v2-compat-upstream'/name
            destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(content)
            hashes.append(hashlib.sha256(content).hexdigest()+'  '+name)
    (tmp_path/'SHA256SUMS').write_text('\n'.join(hashes)+'\n')
    original_run = subprocess.run
    def run(argv, **kwargs):
        command=shlex.split(argv[-1]);command[-1]=str(tmp_path)
        return original_run(command,**kwargs)
    monkeypatch.setattr(delivery.subprocess,'run',run)
    with delivery.installed_source_bundle(['ssh'],'host') as recovered:
        compatibility_payload(ROOT,recovered)
    (tmp_path/'.v2-compat-upstream'/names[0]).write_text('modified')
    with pytest.raises(subprocess.CalledProcessError):
        delivery.installed_source_bundle(['ssh'],'host')


def test_video3_upgrade_restores_sources_and_retires_only_verified_helper(tmp_path, monkeypatch):
    import hashlib
    import shlex
    import subprocess
    import s1_v2_delivery as delivery
    path = os.environ.get('GO2_S1_V2_ARCHIVE')
    if not path:
        pytest.skip('Set GO2_S1_V2_ARCHIVE to verify upstream version')
    source_root = tmp_path/'old-source'
    for name in ('grasp_video.py', 'startup_status.py'):
        content = subprocess.check_output(['git','show',
            '65ff2b5:camera_ws/s1_service/s1_control/'+name],cwd=ROOT)
        target = source_root/'camera_ws/s1_service/s1_control'/name
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(content)
    old_module = ModuleType('old_video3_payload')
    exec(subprocess.check_output(['git','show','65ff2b5:scripts/s1_v2_compat.py'],cwd=ROOT),
         old_module.__dict__)
    installed = tmp_path/'installed'
    installed.mkdir()
    with zipfile.ZipFile(path) as bundle:
        previous = old_module.compatibility_payload(source_root,bundle)
        for name,meta in previous.items():
            target = installed/name
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(base64.b64decode(meta['content']))
        originals = ['scripts/run_arm_integration.py','s1_control/integration.py',
                     'scripts/arm_task_client.py','scripts/verify_handoff.py',
                     'demo_grasp/capture_rgbd.py']
        for name in originals:
            backup = installed/'.v2-compat-upstream'/name
            backup.parent.mkdir(parents=True,exist_ok=True)
            backup.write_bytes(bundle.read('s1-arm-control/'+name))
        (installed/'SHA256SUMS').write_bytes(bundle.read('s1-arm-control/SHA256SUMS'))
        original_capture = bundle.read('s1-arm-control/demo_grasp/capture_rgbd.py')
    manifest = dict(version='v2_usb01_feedback_video_3',files={
        name:{k:v for k,v in meta.items() if k!='content'} for name,meta in previous.items()})
    (installed/'S1_V2_COMPAT.json').write_text(json.dumps(manifest))
    # This fixture runs the remote installer locally; never inspect real host owners.
    bin_path = tmp_path/'mock_bin'
    bin_path.mkdir()
    pgrep = bin_path/'pgrep'
    pgrep.write_text('#!/bin/sh\nexit 1\n');pgrep.chmod(0o755)
    monkeypatch.setenv('PATH', str(bin_path)+os.pathsep+os.environ['PATH'])
    original_run = subprocess.run
    def run(argv, **kwargs):
        command = shlex.split(argv[-1]);command[3]=str(installed)
        return original_run(command,**kwargs)
    monkeypatch.setattr(delivery.subprocess,'run',run)
    ensure_v2_compat(ROOT,['ssh'],'host',running=False,archive=path)
    current = json.loads((installed/'S1_V2_COMPAT.json').read_text())
    assert current['version'] == delivery.COMPAT_VERSION
    assert (installed/'demo_grasp/capture_rgbd.py').read_bytes() == original_capture
    assert 'run_recorded_flow' not in (installed/'scripts/run_arm_integration.py').read_text()
    assert not (installed/'s1_control/grasp_video.py').exists()
    digest = delivery.RETIRED_VIDEO_FILES['s1_control/grasp_video.py']
    backup = installed/'.v2-compat-backups'/digest/'s1_control/grasp_video.py'
    assert hashlib.sha256(backup.read_bytes()).hexdigest() == digest


def test_no_video4_upgrade_enables_side_dispatch_without_changing_capture(tmp_path, monkeypatch):
    import shlex
    import subprocess
    import s1_v2_delivery as delivery
    path = os.environ.get('GO2_S1_V2_ARCHIVE')
    if not path:
        pytest.skip('Set GO2_S1_V2_ARCHIVE to verify upstream version')
    with zipfile.ZipFile(path) as archive:
        previous = compatibility_payload(ROOT, archive, dual_side=False)
        for name, meta in previous.items():
            target = tmp_path/name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(base64.b64decode(meta['content']))
        original_capture = archive.read('s1-arm-control/demo_grasp/capture_rgbd.py')
    metadata = dict(version='v2_usb01_feedback_no_video_4', files={
        name: {k: v for k, v in meta.items() if k != 'content'} for name, meta in previous.items()})
    (tmp_path/'S1_V2_COMPAT.json').write_text(json.dumps(metadata))
    bin_path = tmp_path/'mock_bin'
    bin_path.mkdir()
    pgrep = bin_path/'pgrep'
    pgrep.write_text('#!/bin/sh\nexit 1\n');pgrep.chmod(0o755)
    monkeypatch.setenv('PATH', str(bin_path)+os.pathsep+os.environ['PATH'])
    original_run = subprocess.run

    def run(argv, **kwargs):
        command = shlex.split(argv[-1])
        command[3] = str(tmp_path)
        return original_run(command, **kwargs)

    monkeypatch.setattr(delivery.subprocess, 'run', run)
    ensure_v2_compat(ROOT, ['ssh'], 'host', running=False, archive=path)
    current = json.loads((tmp_path/'S1_V2_COMPAT.json').read_text())
    assert current['version'] == delivery.COMPAT_VERSION
    assert 'selected_grasp_arguments' in (tmp_path/'scripts/run_arm_integration.py').read_text()
    assert (tmp_path/'demo_grasp/capture_rgbd.py').read_bytes() == original_capture


@pytest.mark.parametrize('allow,status,plane,miss', [
    (True,'NO_TARGET',True,True), (False,'NO_TARGET',True,False),
    (True,'NO_TARGET',False,False), (True,'DETECTED_CAMERA_FRAME_ONLY',True,False),
    (True,'CAMERA_ERROR',True,False)])
def test_safe_no_target_dispatch_is_explicit_and_requires_valid_plane(payload, allow, status, plane, miss):
    source = ast.parse(payload['scripts/run_go2_visual_alignment.py'])
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'MemoryTargetMiss')
    condition = next(n for n in ast.walk(source) if isinstance(n, ast.If)
                     and any(isinstance(child, ast.Raise) and isinstance(child.exc, ast.Call)
                             and isinstance(child.exc.func, ast.Name)
                             and child.exc.func.id == 'MemoryTargetMiss' for child in n.body))
    namespace = dict(arm_session=SimpleNamespace(allow_missed=allow),
                     initial_capture={'detection':{'status':status,'plane':{'quality_pass':plane}}})
    exec(compile(ast.Module(body=[cls], type_ignores=[]), '<miss-type>', 'exec'), namespace)
    code = compile(ast.Module(body=[condition], type_ignores=[]), '<miss-branch>', 'exec')
    if miss:
        with pytest.raises(namespace['MemoryTargetMiss']):
            exec(code, namespace)
    else:
        exec(code, namespace)
