"""Offline camera ownership, video finalization and evidence collection checks."""
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'camera_ws/s1_service'))
sys.path.insert(0, str(ROOT/'camera_ws/scripts'))
from s1_control import grasp_video as video
from full_video_evidence import save_front_evidence, save_video_evidence


@pytest.fixture
def camera(monkeypatch):
    calls = []
    class Frame:
        def __init__(self, n): self.n = n
        def get_data(self): return np.full((48, 64, 3), self.n % 255, np.uint8)
        def get_frame_number(self): return self.n
        def get_timestamp(self): return self.n * 1000/30
        def get_frame_timestamp_domain(self): return 'synthetic'
    class Frames:
        def __init__(self, n): self.color = Frame(n)
        def get_color_frame(self): return self.color
        def get_depth_frame(self): return True
    class Pipeline:
        def __init__(self, *args): self.n = 0;calls.append('pipeline')
        def start(self, config):
            calls.append('start')
            return SimpleNamespace(get_device=lambda:SimpleNamespace(get_info=lambda key:'WRIST'))
        def wait_for_frames(self, timeout):
            time.sleep(.005)
            self.n += 1
            return Frames(self.n)
        def stop(self): calls.append('stop')
    class Config:
        def enable_device(self, serial): calls.append(('device', serial))
        def enable_stream(self, *args): pass
    rs = SimpleNamespace(pipeline=Pipeline, config=Config,
                         stream=SimpleNamespace(color=1, depth=2),
                         format=SimpleNamespace(rgb8=1, z16=2),
                         camera_info=SimpleNamespace(serial_number=1))
    monkeypatch.setitem(sys.modules, 'pyrealsense2', rs)
    monkeypatch.setattr(video, '_active', None)
    return rs, calls


def flow_and_session(tmp_path):
    flow = SimpleNamespace(PROJECT_ROOT=tmp_path,
        load_alignment_config=lambda _: (None, dict(wrist_servo_config='bound.json')),
        resolve_project_path=lambda value:value,
        load_wrist_servo_config=lambda _:SimpleNamespace(record=dict(
            camera_serial='WRIST', capture=dict(width=64,height=48,fps=30))))
    state = {}
    session = SimpleNamespace(_update=lambda **kw:state.update(kw))
    return flow, session, state


@pytest.mark.parametrize('fault', [False, True])
def test_one_camera_records_before_capture_through_return_and_finalizes(tmp_path,camera,fault):
    rs,calls=camera
    flow,session,state=flow_and_session(tmp_path)
    def grasp(owner):
        assert owner is session
        assert calls.count('start') == 1
        for _ in range(2):
            lease=video.capture_pipeline(rs,None,'WRIST',64,48,30)
            lease.start(None)
            a=lease.wait_for_frames().get_color_frame().get_frame_number()
            b=lease.wait_for_frames().get_color_frame().get_frame_number()
            assert b > a
            lease.stop()
            assert 'stop' not in calls
        # Motion/return intervals do not call the snapshot capture API.
        time.sleep(.03)
        if fault: raise ValueError('original grasp fault')
        return 0
    if fault:
        with pytest.raises(ValueError,match='original grasp fault'):
            video.run_recorded_flow(flow,SimpleNamespace(config='config.json'),session,grasp)
    else:
        assert video.run_recorded_flow(flow,SimpleNamespace(config='config.json'),session,grasp)==0
    record=state['last_recording']
    assert record['complete'] and record['finalized'] and record['frames'] > 4
    assert calls.count('pipeline')==calls.count('start')==calls.count('stop')==1
    assert video._active is None
    capture=cv2.VideoCapture(record['video']);decoded=0
    while True:
        ok,frame=capture.read()
        if not ok:break
        assert frame.shape==(48,64,3)
        decoded+=1
    capture.release()
    assert decoded==record['frames']
    rows=[json.loads(line) for line in (Path(record['directory'])/'frames.jsonl').read_text().splitlines()]
    assert len(rows)==decoded
    assert all(b['camera_frame_number']>a['camera_frame_number'] for a,b in zip(rows,rows[1:]))
    assert json.loads((Path(record['directory'])/'recording.json').read_text())==record


def test_writer_failure_prevents_grasp_and_releases_camera(tmp_path,camera,monkeypatch):
    flow,session,state=flow_and_session(tmp_path)
    invoked=[]
    monkeypatch.setattr(cv2,'VideoWriter',lambda *args:SimpleNamespace(
        isOpened=lambda:False,release=lambda:None))
    with pytest.raises(RuntimeError,match='MP4 writer'):
        video.run_recorded_flow(flow,SimpleNamespace(config='x'),session,lambda _:invoked.append(1))
    assert not invoked and not state['last_recording']['complete']
    assert camera[1].count('stop')==1


def test_capture_profile_cannot_switch_to_front_camera(camera):
    video._active=SimpleNamespace(settings=('WRIST',64,48,30))
    with pytest.raises(RuntimeError,match='profile differs'):
        video.capture_pipeline(camera[0],None,'FRONT',64,48,30)


def test_standalone_capture_keeps_original_pipeline(camera):
    assert video.capture_pipeline(camera[0],None,'WRIST',64,48,30) is not None
    assert camera[1]==['pipeline']


def test_front_index_exists_without_any_grasp(tmp_path):
    front=dict(video='overlay.mp4',raw_video='raw.mp4',frames=20,closed=True)
    save_front_evidence(tmp_path,front)
    report=json.loads((tmp_path/'recordings.json').read_text())
    assert report['front']==front and report['wrist']['status']=='NOT STARTED'


@pytest.mark.parametrize('new', [None, dict(directory='/old')])
def test_no_grasp_never_copies_previous_video(tmp_path,new,monkeypatch):
    import full_video_evidence as evidence
    monkeypatch.setattr(evidence.subprocess,'run',lambda *a,**kw:pytest.fail('No copy allowed'))
    save_video_evidence(SimpleNamespace(),tmp_path,{},dict(directory='/old'),
                        lambda *a:dict(last_recording=new))
    assert json.loads((tmp_path/'recordings.json').read_text())['wrist']['status']=='NOT STARTED'


@pytest.mark.parametrize('failed', [False, True])
def test_transfer_is_indexed_and_remote_is_retained(tmp_path,monkeypatch,failed):
    import full_video_evidence as evidence
    wrist=dict(directory='/home/unitree/s1-arm-control-v2/data/grasp_videos/grasp_123',
               finalized=True,frames=50,complete=True)
    calls=[]
    def copy(argv,**kw):
        calls.append(argv)
        if failed:raise OSError('link lost')
    monkeypatch.setattr(evidence.subprocess,'run',copy)
    save_video_evidence(SimpleNamespace(arm_host='unitree@host',ssh_key='/key'),tmp_path,{},None,
                        lambda *a:dict(last_recording=wrist))
    report=json.loads((tmp_path/'recordings.json').read_text())['wrist']
    assert report['status']==('REMOTE_ONLY' if failed else 'COPIED')
    assert report['recording']==wrist and len(calls)==1
    assert calls[0][0]=='scp' and '-r' in calls[0]


@pytest.mark.parametrize('directory,finalized', [('/unexpected/grasp_123',True),
    ('/home/unitree/s1-arm-control-v2/data/grasp_videos/grasp_123',False)])
def test_invalid_or_active_recording_never_copied(tmp_path,monkeypatch,directory,finalized):
    import full_video_evidence as evidence
    monkeypatch.setattr(evidence.subprocess,'run',lambda *a,**kw:pytest.fail('No copy allowed'))
    save_video_evidence(SimpleNamespace(),tmp_path,{},None,
        lambda *a:dict(last_recording=dict(directory=directory,finalized=finalized)))
    assert 'error' in json.loads((tmp_path/'recordings.json').read_text())['wrist']
