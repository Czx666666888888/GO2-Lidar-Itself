"""Small interface checks; no SDK import, serial device or motion is used."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'camera_ws/s1_service'))
sys.path.insert(0,str(ROOT/'scripts'))
from s1_control.startup_status import (
    feedback_status, communication_error, resolve_serial_device,
)
from s1_control.integration import ArmIntegrationSession
from go2_full_test import Workflow, select_arm_device
from go2_build_fingerprint import fingerprint

class Arm:
    limits=SimpleNamespace(max_temp_c=55.,max_track_err_rad=.08)
    def __init__(self,q=0.):self.q=[q]*6
    def get_pos(self):return self.q+[0.]
    def get_vel(self):return [0.]*7
    def get_temp(self):return [25.]*7


def test_zero_vs_nonzero_and_real_bus_error():
    assert feedback_status(Arm())['position_state']=='AT_ZERO'
    state=feedback_status(Arm(.126))
    assert state['position_state']=='NOT_AT_ZERO' and state['communication_state']=='OK'
    assert not communication_error(RuntimeError('initial pose outside zero-start window'))
    assert communication_error(RuntimeError('bus init failed'))
    assert communication_error(OSError('Input/output error'))


def test_bad_feedback_is_not_zero():
    arm=Arm();arm.q=[float('nan')]*6
    with pytest.raises(OSError):feedback_status(arm)


def test_service_queues_zero_once_and_never_during_grasp():
    service=ArmIntegrationSession(Arm(.10))
    service._update(state='NOT_AT_ZERO',busy=False)
    assert service.request_zero()['accepted']
    assert service.jobs.get_nowait()=='zero'
    assert service.request_zero()==dict(accepted=False,reason='BUSY')
    service._update(state='GRASPING',busy=True)
    assert not service.request_zero()['accepted']


def test_service_reuses_verified_zero_hold_without_job():
    service=ArmIntegrationSession(Arm())
    service._update(state='ZERO_HOLD',busy=False,navigation_allowed=True)
    assert service.request_zero()['reused']
    assert service.jobs.empty()


def test_unverified_zero_path_is_rejected():
    service=ArmIntegrationSession(Arm(.12))
    service._update(state='NOT_AT_ZERO',busy=False,return_zero_allowed=False)
    assert not service.request_zero()['accepted']
    assert service.jobs.empty()


def test_workflow_reuses_s1_without_zero_or_deploy():
    workflow=Workflow(SimpleNamespace(work_dir=ROOT,mode='dock'))
    workflow.status=lambda:dict(state='ZERO_HOLD',busy=False,navigation_allowed=True,
        joint_positions_rad=[0.]*6,at_zero=True,communication_state='OK',readiness_state='ZERO_HOLD')
    workflow.deploy_s1=lambda:pytest.fail('must not deploy/reconnect a ready service')
    workflow.arm_command=lambda *args:pytest.fail('must not command zero again')
    workflow.ensure_s1(True)


def test_build_hash_changes_only_with_build_inputs(tmp_path):
    (tmp_path/'a.py').write_text('a=1')
    first=fingerprint(tmp_path)
    (tmp_path/'__pycache__').mkdir();(tmp_path/'__pycache__/a.pyc').write_bytes(b'x')
    assert fingerprint(tmp_path)==first
    (tmp_path/'a.py').write_text('a=2')
    assert fingerprint(tmp_path)!=first


@pytest.mark.parametrize('full',[False,True])
def test_clock_sampling_is_explicit_only(tmp_path,monkeypatch,full):
    path=ROOT/'camera_ws/scripts/pc_time_check.py'
    spec=importlib.util.spec_from_file_location('clock_context_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['pc_time_check','--execute','--mode','dock']+
                        (['--clock-diagnostic'] if full else []))
    durations=[]
    monkeypatch.setattr(module,'probe',lambda seconds:durations.append(seconds) or {})
    monkeypatch.setattr(module,'run',lambda *a,**k:None)
    active=iter([0,1,0])
    def run(argv,**kw):
        code=next(active) if argv[:2]==['systemctl','is-active'] else 0
        return SimpleNamespace(returncode=code,stdout='')
    monkeypatch.setattr(module.subprocess,'run',run)
    monkeypatch.setattr(module.subprocess,'Popen',lambda *a,**k:SimpleNamespace(poll=lambda:0,wait=lambda:0))
    with pytest.raises(SystemExit) as exc:module.main()
    assert exc.value.code==0
    assert durations==([20,20] if full else [])


def test_readonly_startup_never_enables_or_starts_devices(tmp_path,monkeypatch):
    import go2_full_test as launcher
    workflow=Workflow(SimpleNamespace(work_dir=ROOT,mode='dock',startup_check=True))
    workflow.folder=tmp_path/'readonly'
    workflow.prepare=lambda:None
    workflow.probe=lambda *a,**kw:dict(ready=False,reason='no hardware in test')
    workflow.ensure_s1=lambda *a:pytest.fail('read-only must not initialize arm')
    workflow.spawn=lambda *a,**kw:pytest.fail('read-only must not start devices')
    inventory=dict(arm=[],arm_devices=[],task=[],camera=[],serial_devices=['/dev/ttyUSB0'],
                   cameras=['213622070598','231622302209'],router=False)
    monkeypatch.setattr(launcher,'run',lambda *a,**kw:None)
    monkeypatch.setattr(launcher,'remote',lambda *a,**kw:SimpleNamespace(stdout=__import__('json').dumps(inventory)))
    monkeypatch.setattr(launcher.subprocess,'check_output',lambda *a,**kw:'dev enp12s0 src 192.168.123.99')
    monkeypatch.setattr(launcher.subprocess,'run',lambda *a,**kw:SimpleNamespace(returncode=1,stdout=''))
    workflow.execute()
    assert (workflow.folder/'startup_timing.json').exists()


@pytest.mark.parametrize('device', ['/dev/ttyUSB0', '/dev/ttyUSB1'])
def test_unique_serial_is_selected_and_same_service_can_be_reused(device):
    assert select_arm_device([device], []) == device
    assert select_arm_device([device], [device]) == device


@pytest.mark.parametrize('devices,running', [
    ([], []), (['/dev/ttyUSB0', '/dev/ttyUSB1'], []),
    (['/dev/ttyUSB1'], ['/dev/ttyUSB0']),
    (['/dev/ttyUSB0'], [None]),
    (['/dev/ttyUSB1'], ['/dev/ttyUSB1', '/dev/ttyUSB1']),
])
def test_missing_ambiguous_or_old_serial_owner_is_blocked(devices, running):
    with pytest.raises(RuntimeError):
        select_arm_device(devices, running)


def test_cold_start_uses_service_auto_selection(tmp_path, monkeypatch):
    import go2_full_test as launcher
    workflow=Workflow(SimpleNamespace(work_dir=ROOT,mode='dock'))
    workflow.folder=tmp_path/'cold'
    workflow.deploy_s1=lambda:None
    commands=[]
    state=dict(state='ZERO_HOLD',busy=False,navigation_allowed=True,joint_positions_rad=[0.]*6)
    def remote(command,**kwargs):
        commands.append(command)
        return SimpleNamespace(stdout=__import__('json').dumps(state))
    monkeypatch.setattr(launcher,'remote',remote)
    workflow.ensure_s1(False, '/dev/ttyUSB1')
    assert 'auto' in commands[0] and '/dev/ttyUSB0' not in commands[0]
    assert len(commands)==2  # One original owner, then original socket wait.


@pytest.mark.parametrize('device', ['/dev/ttyUSB0', '/dev/ttyUSB1'])
@pytest.mark.parametrize('requested', ['auto', '/dev/ttyUSB0', '/dev/ttyUSB1'])
def test_service_selects_zero_or_one_without_editing_config(monkeypatch, device, requested):
    import s1_control.startup_status as status
    monkeypatch.setattr(status.os.path, 'exists', lambda value: value==device)
    assert resolve_serial_device(requested)==device


@pytest.mark.parametrize('devices', [[], ['/dev/ttyUSB0','/dev/ttyUSB1']])
def test_service_rejects_missing_or_ambiguous_ports(monkeypatch, devices):
    import s1_control.startup_status as status
    monkeypatch.setattr(status.os.path, 'exists', lambda value: value in devices)
    with pytest.raises(OSError): resolve_serial_device()


def test_recreated_device_does_not_validate_stale_sdk_feedback(monkeypatch):
    import s1_control.startup_status as status
    monkeypatch.setattr(status.os.path,'exists',lambda _:True)
    monkeypatch.setattr(status,'serial_identity',lambda _:(188,222))
    with pytest.raises(OSError, match='re-enumerated'):
        feedback_status(Arm(), '/dev/ttyUSB0', device_identity=(188,111))


def test_service_upgrade_accepts_only_recorded_previous_hash_and_backs_it_up(tmp_path, monkeypatch):
    import base64
    import hashlib
    import io
    import json
    import shlex
    import go2_full_test as launcher
    captured={}
    def remote(command,**kwargs):
        captured['script']=shlex.split(command)[2]
    monkeypatch.setattr(launcher,'remote',remote)
    workflow=Workflow(SimpleNamespace(work_dir=ROOT,mode='dock'))
    workflow.deploy_s1()
    target=tmp_path/'scripts/service.py';target.parent.mkdir();target.write_bytes(b'old')
    old_hash=hashlib.sha256(b'old').hexdigest()
    payload={'scripts/service.py':dict(original=None,previous=[],
        content=base64.b64encode(b'new').decode())}
    monkeypatch.setattr(sys,'argv',['remote',str(tmp_path)])
    monkeypatch.setattr(launcher.subprocess,'run',lambda *a,**kw:SimpleNamespace(returncode=1))
    monkeypatch.setattr(sys,'stdin',io.StringIO(json.dumps(payload)))
    with pytest.raises(SystemExit):exec(captured['script'],{})
    assert target.read_bytes()==b'old'
    payload['scripts/service.py']['previous']=[old_hash]
    monkeypatch.setattr(sys,'stdin',io.StringIO(json.dumps(payload)))
    exec(captured['script'],{})
    assert target.read_bytes()==b'new'
    assert (tmp_path/'.go2-startup-backups'/old_hash/'scripts/service.py').read_bytes()==b'old'
