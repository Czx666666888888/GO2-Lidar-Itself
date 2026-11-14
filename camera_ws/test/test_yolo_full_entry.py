"""Full-chain wiring tests with no ROS nodes, SSH, S1 or motion."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/'camera_ws/scripts'))
from go2_full_test import Workflow
from go2_startup_probe import detector_kind


def workflow(tmp_path, **values):
    args = dict(work_dir=ROOT, mode='full', grasp_side='right', view=True, detector='yolo')
    args.update(values)
    w = Workflow(SimpleNamespace(**args));w.folder=tmp_path
    return w


def test_yolo_full_keeps_the_original_arm_handoff_and_preview(tmp_path):
    w=workflow(tmp_path);w.vision_model=Path('/local/best.pt');w.vision_python=Path('/local/python')
    perception=w.perception_command();task=w.task_command()
    assert 'detector:=yolo' in perception
    assert 'yolo_model:=/local/best.pt' in perception
    assert 'yolo_python:=/local/python' in perception
    assert '--full' in task and '--posture-profile' in task and '--preview-image' in task
    assert '--forward-response-check' not in task


def test_hsv_is_explicit_and_dock_does_not_trigger_arm(tmp_path):
    w=workflow(tmp_path,mode='dock',detector='hsv',view=False)
    assert 'detector:=hsv' in w.perception_command()
    assert not any('yolo_model' in x for x in w.perception_command())
    assert '--full' not in w.task_command()


def test_forward_response_does_not_start_yolo_or_grasp(tmp_path):
    w=workflow(tmp_path,mode='dock',forward_response_check=True,response_distance_limit=.35)
    assert w.detector=='hsv'
    assert '--forward-response-check' in w.task_command()
    assert '--full' not in w.task_command()


def test_shared_model_validation_is_used_before_full_start(tmp_path,monkeypatch):
    import go2_color_block_test as assets
    w=workflow(tmp_path,view=False);(tmp_path/'manifest.json').write_text('{}')
    monkeypatch.setattr(assets,'detector_assets',lambda args,root:(Path('/local/best.pt'),Path('/local/python')))
    w.configure_vision()
    state=json.loads((tmp_path/'manifest.json').read_text())
    assert state['detector']=='yolo' and state['model_sha256']==assets.V2_SHA256
    def fail(*a):raise RuntimeError('wrong model')
    monkeypatch.setattr(assets,'detector_assets',fail)
    with pytest.raises(RuntimeError,match='wrong model'):w.configure_vision()


def test_detector_identity_rejects_mixed_owners():
    assert detector_kind(['yolo_detector'])=='yolo'
    assert detector_kind(['coarse_target_locator'])=='hsv'
    assert detector_kind(['yolo_detector','coarse_target_locator'])=='mixed'
    assert detector_kind([]) is None


def test_outer_time_wrapper_forwards_full_vision_and_view(tmp_path,monkeypatch):
    import importlib.util
    path=ROOT/'camera_ws/scripts/pc_time_check.py'
    spec=importlib.util.spec_from_file_location('full_time_wiring_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['pc_time_check','--execute','--mode','full','--view',
        '--detector','yolo','--model-path','/local/best.pt','--python-executable','/local/python'])
    monkeypatch.setattr(module.subprocess,'run',lambda *a,**kw:SimpleNamespace(returncode=1,stdout=''))
    commands=[]
    def launch(argv,**kw):
        commands.append(argv)
        return SimpleNamespace(poll=lambda:0,wait=lambda:0)
    monkeypatch.setattr(module.subprocess,'Popen',launch)
    with pytest.raises(SystemExit) as result:module.main()
    assert result.value.code==0
    argv=commands[0]
    assert argv[argv.index('--mode')+1]=='full' and '--view' in argv
    assert argv[argv.index('--detector')+1]=='yolo'
    assert argv[argv.index('--model-path')+1]=='/local/best.pt'
    assert argv[argv.index('--python-executable')+1]=='/local/python'
