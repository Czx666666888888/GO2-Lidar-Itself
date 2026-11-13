"""Extraction fixtures only; never labels or alters the actual phone video pool."""
import hashlib
import json
from pathlib import Path
import zipfile

import cv2
import numpy as np
import pytest
import yaml

from vision_training.tools.extract_training_frames import difference, extract, package, verify
from vision_training.tools.import_cvat import import_cvat


CONFIG = Path(__file__).resolve().parents[1]/'configs/science_targets_video_20261113.yaml'


def fixture_config(tmp_path):
    source=tmp_path/'input';source.mkdir()
    path=source/'fixture.avi'
    writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'MJPG'),10,(240,160))
    assert writer.isOpened()
    for index in range(30):
        frame=np.full((160,240,3),30,dtype=np.uint8)
        # Include repeated views followed by local object movement in a dark scene.
        step=max(0,index-8)
        cv2.rectangle(frame,(10+step*4,50),(30+step*4,70),(60,80,120),-1)
        writer.write(frame)
    writer.release()
    config=yaml.safe_load(CONFIG.read_text())
    config.pop('source_type');config.pop('source_confirmation')
    config['input_dir']=str(source)
    config['videos']=[dict(id='video_A',filename=path.name,capture_group='unverified_scene')]
    config['sampling'].update(min_interval_sec=0.1,base_interval_sec=0.2,max_interval_sec=0.3)
    config['batch_size']=4
    target=tmp_path/'config.yaml';target.write_text(yaml.safe_dump(config))
    return target,path


def test_local_small_low_contrast_change_is_not_a_duplicate():
    config=yaml.safe_load(CONFIG.read_text())['sampling']
    first=np.full((544,960),50,np.uint8)
    second=first.copy();second[80:90,80:90]+=5
    result=difference(second,first,config)
    assert result['similarity']>0.9999
    assert not result['near_duplicate']
    assert difference(first,first,config)['near_duplicate']


def test_coverage_decode_zip_provenance_annotation_only_import(tmp_path):
    config,source=fixture_config(tmp_path)
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    root=tmp_path/'data'
    summary=extract(config,root)
    result=verify(root)
    assert result['original_decoded_frames']==30
    assert result['images']==summary['images']>2
    assert result['source_frame_content_checks']==summary['images']
    assert result['cross_zip_duplicates']==0
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    rows=json.loads((root/'pool/manifest.json').read_text())
    assert all(r['source_type']=='video_source_unverified' and not r['reviewed'] for r in rows)
    assert any(r['quality_flags'] for r in rows)
    counts=summary['videos'][0]['counts']
    assert counts.get('exact_decoded_duplicate',0)>0
    assert not list(root.rglob('*.txt'))
    assert all((root/r['image']).is_file() for r in rows)
    # A label-only CVAT test export must match the original by its stable filename.
    name=rows[0]['filename']
    archive=tmp_path/'annotation_only.zip'
    with zipfile.ZipFile(archive,'w') as z:
        z.writestr('data.yaml',yaml.safe_dump(dict(names=['color_block','rock_compact','rock_spiky'])))
        z.writestr('labels/train/'+Path(name).with_suffix('.txt').name,'0 .5 .5 .1 .1\n')
    review=tmp_path/'review.csv'
    review.write_text(f'image,status\n{name},positive\n{rows[1]["filename"]},uncertain\n')
    assert import_cvat(root,archive,review,'TEST_FIXTURE_ONLY')==1
    updated=json.loads((root/'pool/manifest.json').read_text())
    assert updated[0]['reviewed'] and not updated[1]['reviewed']
    assert updated[0]['source_video_sha256']==digest
    assert updated[0]['capture_group']=='unverified_scene'


def test_timeline_mode_is_complete_not_a_deduped_evaluation_sequence(tmp_path):
    config,_=fixture_config(tmp_path)
    root=tmp_path/'timeline'
    summary=extract(config,root,mode='timeline')
    assert summary['images']==0 and summary['zip_count']==0
    lines=[json.loads(s) for s in (root/'manifests/video_A_timeline.jsonl').read_text().splitlines()]
    assert [r['frame_index'] for r in lines]==list(range(30))
    assert all(r['action']=='timeline_only' for r in lines)
    assert verify(root)['original_decoded_frames']==30


def test_repackage_size_and_no_overwrite(tmp_path):
    config,_=fixture_config(tmp_path)
    root=tmp_path/'data'
    extract(config,root)
    before=(root/'manifests/images.json').read_bytes()
    packages=package(root,2,tmp_path/'small_packages',tmp_path/'small_contacts')
    assert all(batch['images']<=2 for batch in packages)
    assert (root/'manifests/images.json').read_bytes()==before
    names=[]
    for batch in packages:
        with zipfile.ZipFile(batch['archive']) as z:names.extend(z.namelist())
    assert len(names)==len(set(names))
    with pytest.raises(FileExistsError): extract(config,root)
    with pytest.raises(FileExistsError): package(root,2,tmp_path/'small_packages',tmp_path/'unused')


def test_confirmed_source_requires_explicit_evidence(tmp_path):
    config,_=fixture_config(tmp_path)
    data=yaml.safe_load(config.read_text());data['source_type']='d435_real'
    config.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match='source confirmation'):extract(config,tmp_path/'data')
    assert not (tmp_path/'data').exists()


def test_shared_capture_group_does_not_imply_independent_scenes(tmp_path):
    config,source=fixture_config(tmp_path)
    second=source.with_name('second.avi')
    cap=cv2.VideoCapture(str(source))
    writer=cv2.VideoWriter(str(second),cv2.VideoWriter_fourcc(*'MJPG'),10,(240,160))
    while True:
        ok,frame=cap.read()
        if not ok:break
        cv2.circle(frame,(180,100),10,(0,150,0),-1)
        writer.write(frame)
    writer.release();cap.release()
    data=yaml.safe_load(config.read_text())
    data['videos'].append(dict(id='video_B',filename='second.avi',capture_group='unverified_scene'))
    config.write_text(yaml.safe_dump(data))
    root=tmp_path/'shared'
    extract(config,root)
    rows=json.loads((root/'pool/manifest.json').read_text())
    assert {r['video_id'] for r in rows}=={'video_A','video_B'}
    assert {r['group'] for r in rows}=={'unverified_scene'}
    assert all((root/r['image']).is_file() for r in rows)
    assert verify(root)['images']==len(rows)
