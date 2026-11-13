#!/usr/bin/env python3
"""Select unreviewed A/C time strata and package images only; never infer labels."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import cv2
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json
from vision_training.tools.extract_training_frames import sheet, representative_rows, review_csv


def choose(rows, count):
    """Sample each temporal stratum; retain a mix of quality rather than reject blur."""
    rows=sorted(rows,key=lambda r:r['frame_index'])
    if len(rows)<count:raise ValueError('Insufficient unreviewed frames in window')
    return [rows[int(i)] for i in np.linspace(0,len(rows)-1,count).round().astype(int)]


def prepare(dataset, config, output):
    config_data=yaml.safe_load(config.read_text())
    pool=json.loads((dataset/'pool/manifest.json').read_text())
    reviewed={r['filename'] for r in pool if r['reviewed']}
    remaining=[r for r in pool if not r['reviewed']]
    # Audit every remaining image, including the held-out B inventory.
    for row in remaining:
        path=dataset/row['image_file'];image=cv2.imread(str(path))
        if image is None or list(image.shape[1::-1])!=row['resolution']:raise ValueError('Unreadable or resized image')
        if sha256(path)!=row['image_sha256']:raise ValueError('Image changed')
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        row['readback_sharpness']=float(cv2.Laplacian(gray,cv2.CV_64F).var())
        row['readback_mean_gray']=float(gray.mean())
    selected=[]
    for window in config_data['windows']:
        if window['video_id'] not in ('video_A','video_C'):raise ValueError('B is validation-only')
        candidates=[r for r in remaining if r['video_id']==window['video_id'] and window['start']<=r['source_seconds']<window['end']]
        for row in choose(candidates,window['count']):
            selected.append(dict(row,priority_reason=window['reason'],suggested_class=None,
                class_status='unconfirmed_manual_review_required',annotation_status='pending',reviewed=False))
    if len({r['filename'] for r in selected})!=len(selected):raise ValueError('Overlapping selection windows')
    assert not reviewed & {r['filename'] for r in selected}
    assert not any(r['video_id']=='video_B' for r in selected)
    assert len({r['image_sha256'] for r in selected})==len(selected)
    output.mkdir(parents=True,exist_ok=False)
    archive=output/'round2_priority_180.zip'
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_STORED) as z:
        for row in selected:z.write(dataset/row['image_file'],row['filename'])
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:raise ValueError('ZIP CRC failed')
        assert set(z.namelist())=={r['filename'] for r in selected}
        for row in selected:
            data=z.read(row['filename'])
            assert hashlib.sha256(data).hexdigest()==row['image_sha256']
            image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
            assert list(image.shape[1::-1])==row['resolution']
    write_json(output/'manifest.json',dict(rows=selected,zip_sha256=sha256(archive),
        note='Original pool references; no labels, no automatic negatives, B excluded'))
    review_csv(output/'round2_review.csv',selected)
    contacts=output/'contact_sheets'
    for start in range(0,len(selected),36):sheet(dataset,selected[start:start+36],contacts/f'page_{start//36+1:02d}.jpg',columns=6,tile_width=320)
    sheet(dataset,representative_rows(selected,32),contacts/'representatives.jpg')
    for video in ('video_A','video_B','video_C'):
        rows=[r for r in remaining if r['video_id']==video]
        for start in range(0,len(rows),64):sheet(dataset,rows[start:start+64],output/'inventory_sheets'/f'{video}_page_{start//64+1:02d}.jpg',columns=8,tile_width=240)
    fields=['filename','video_id','source_video','source_type','capture_group','frame_index','source_seconds','sharpness','mean_gray','readback_sharpness','readback_mean_gray','selected','disposition']
    picked={r['filename'] for r in selected}
    with (output/'remaining_inventory.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for row in remaining:
            writer.writerow(dict({k:row[k] for k in fields if k in row},selected=row['filename'] in picked,
                disposition='validation_only' if row['video_id']=='video_B' else ('priority_round2' if row['filename'] in picked else 'available_unreviewed')))
    def stats(rows):
        return dict(images=len(rows),sharpness_percentiles=np.percentile([r['readback_sharpness'] for r in rows],[0,25,50,75,100]).tolist(),
            mean_gray_percentiles=np.percentile([r['readback_mean_gray'] for r in rows],[0,25,50,75,100]).tolist(),
            low_sharpness_readback_metric=sum(r['readback_sharpness']<25 for r in rows))
    summary=dict(remaining=len(remaining),source_counts=dict(Counter(r['video_id'] for r in remaining)),
        selected=len(selected),selected_sources=dict(Counter(r['video_id'] for r in selected)),
        inventory_quality={v:stats([r for r in remaining if r['video_id']==v]) for v in ('video_A','video_B','video_C')},
        selected_quality=stats(selected),selection_reasons=dict(Counter(r['priority_reason'] for r in selected)),
        zip=str(archive),zip_sha256=sha256(archive),zip_integrity='PASS',images_readable=True,
        filename_content_collisions=0,previously_reviewed_selected=0,video_B_selected=0,
        new_labels=0,new_reviews=0,object_class_counts='Unknown: requires manual annotation',
        config_sha256=sha256(config),script_sha256=sha256(Path(__file__)))
    write_json(output/'summary.json',summary)
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();print(json.dumps(prepare(args.dataset,args.config,args.output),indent=2))
