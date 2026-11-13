"""Round2 selection cannot silently import reviewed frames or validation video B."""
import csv
import hashlib
import json
from pathlib import Path
import zipfile

import cv2
import numpy as np
import pytest
import yaml
from vision_training.tools.prepare_priority_round2 import prepare, choose


def test_temporal_strata_dont_prefer_only_sharp_frames():
    rows=[dict(frame_index=i,sharpness=1 if i==0 else 100) for i in range(5)]
    assert choose(rows,3)==[rows[0],rows[2],rows[4]]
    with pytest.raises(ValueError):choose(rows,6)


def test_b_excluded_reviewed_excluded_and_zip_exact(tmp_path):
    root=tmp_path/'pool_source';(root/'pool').mkdir(parents=True)
    rows=[]
    for v in ['video_A','video_B','video_C']:
        for i in range(3):
            name=f'{v}_f{i:07d}.jpg';folder=root/'images'/v;folder.mkdir(parents=True,exist_ok=True)
            im=np.full((80,128,3),20+len(rows)*10,np.uint8)
            cv2.circle(im,(20+i*10,30),8,(0,0,100+i*30),-1)
            path=folder/name;cv2.imwrite(str(path),im)
            rows.append(dict(filename=name,video_id=v,frame_index=i,source_seconds=float(i),
                image_file=str(path.relative_to(root)),image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                resolution=[128,80],sharpness=5.,mean_gray=30.,reviewed=(v=='video_A' and i==0),
                source_video=v+'.mp4',source_type='phone_real',capture_group=v))
    (root/'pool/manifest.json').write_text(json.dumps(rows))
    config=tmp_path/'config.yaml';config.write_text(yaml.safe_dump(dict(windows=[dict(video_id=v,start=0,end=3,count=1,reason='manual_candidate') for v in ['video_A','video_C']])))
    before=(root/'pool/manifest.json').read_bytes()
    out=tmp_path/'delivery';summary=prepare(root,config,out)
    assert summary['remaining']==8 and summary['selected']==2
    manifest=json.loads((out/'manifest.json').read_text())['rows']
    assert all(r['video_id']!='video_B' and not r['reviewed'] and r['suggested_class'] is None for r in manifest)
    assert 'video_A_f0000000.jpg' not in {r['filename'] for r in manifest}
    with zipfile.ZipFile(out/'round2_priority_180.zip') as z:
        assert z.testzip() is None
        assert set(z.namelist())=={r['filename'] for r in manifest}
    assert (root/'pool/manifest.json').read_bytes()==before
    assert {r['status'] for r in csv.DictReader((out/'round2_review.csv').open())}=={'pending'}
    config.write_text(yaml.safe_dump(dict(windows=[dict(video_id='video_B',start=0,end=3,count=1,reason='forbidden')])))
    with pytest.raises(ValueError,match='validation-only'):prepare(root,config,tmp_path/'forbidden')
    assert not (tmp_path/'forbidden').exists()
