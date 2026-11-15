"""Protect timestamp-based gap interpretation and reviewed V3 split invariants."""
import json
from pathlib import Path

import pytest

from vision_training.tools.analyze_go2_continuity_v3 import absence_runs
from vision_training.common import sha256
from vision_training.dataset import label_for, validate_label


def test_absence_uses_rgb_stamps_and_marks_unrecovered_tail():
    rows = [dict(frame_index=i, rgb_stamp_ns=t, predictions=p) for i,t,p in [
        (0, 0, [dict(class_id=0, confidence=.8)]),
        (1, 100_000_000, []),
        (2, 450_000_000, [dict(class_id=2, confidence=.9)]),
        (3, 900_000_000, [dict(class_id=0, confidence=.7)]),
        (4, 1_000_000_000, [dict(class_id=0, confidence=.2)])]]
    gaps = absence_runs(rows)
    assert gaps[0] == dict(first_frame=1,last_frame=2,frames=2,
        observed_absence_span_sec=.35,next_prediction_after_first_absent_sec=.8,recovered=True)
    assert gaps[1] == dict(first_frame=4,last_frame=4,frames=1,
        observed_absence_span_sec=0.,next_prediction_after_first_absent_sec=None,recovered=False)
    assert absence_runs([]) == []
    assert absence_runs(rows[:1]) == []


def test_local_v3_inherits_exact_val_and_authorized_negatives():
    base = Path(__file__).resolve().parents[1]/'datasets'
    root = base/'science_targets_mixed_v3'
    if not (root/'preparation.json').exists():
        pytest.skip('Local media not distributed with Git')
    d = json.loads((root/'preparation.json').read_text())
    old = json.loads((base/'science_targets_mixed_v2/preparation.json').read_text())
    assert d['imported']==305 and d['positive']==259 and d['user_confirmed_negative']==46
    assert {(r['image_sha256'],r['label_sha256']) for r in d['records'] if r['split']=='val'} == {
        (r['image_sha256'],r['label_sha256']) for r in old['records'] if r['split']=='val'}
    new = [r for r in d['records'] if r.get('annotation_round')==3]
    assert len(new)==305
    negatives=0
    for r in new:
        image=root/r['image'];label=label_for(image)
        assert sha256(image)==r['image_sha256'] and sha256(label)==r['label_sha256']
        assert r['split']=='train' and r['video_id'] not in ('video_B','r3_new3','r3_new4')
        boxes=validate_label(label,3)
        if not boxes:
            negatives+=1
            assert r['annotation_status']=='negative'
            assert r['reviewer']=='user_round3_CVAT_259_positive_46_negative_20261115'
    assert negatives==46
    assert {r['capture_group'] for r in new if r['source_type']=='d435_real'}=={'lab_floor_20261108_09'}
