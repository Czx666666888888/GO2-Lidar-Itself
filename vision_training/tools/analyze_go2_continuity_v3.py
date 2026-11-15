#!/usr/bin/env python3
"""Prepare raw consecutive RGB review clips or measure prediction presence, never infer GT."""
import argparse
import csv
import json
from pathlib import Path
import sys
import zipfile

import cv2
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json
from vision_training.tools.prepare_round3 import log_frames

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'vision_training'
SOURCES = BASE/'datasets/science_targets_round3_candidates/audit_go2/manifests/videos.json'


def absence_runs(rows, confidence=.25):
    """Prediction gaps only. Times are observed RGB stamp spans, not frame count / fps."""
    runs = []
    start = None
    for i in range(len(rows)+1):
        present = i == len(rows) or any(p['class_id']==0 and p['confidence']>=confidence for p in rows[i]['predictions'])
        if not present and start is None:
            start = i
        if present and start is not None:
            end = i-1
            runs.append(dict(first_frame=rows[start]['frame_index'], last_frame=rows[end]['frame_index'],
                frames=end-start+1, observed_absence_span_sec=(rows[end]['rgb_stamp_ns']-rows[start]['rgb_stamp_ns'])/1e9,
                next_prediction_after_first_absent_sec=(rows[i]['rgb_stamp_ns']-rows[start]['rgb_stamp_ns'])/1e9 if i<len(rows) else None,
                recovered=i<len(rows)))
            start = None
    return runs


def package(output):
    output.mkdir(parents=True, exist_ok=False)
    images = output/'images'; images.mkdir()
    sources = json.loads(SOURCES.read_text())
    train = json.loads((BASE/'datasets/science_targets_mixed_v3/preparation.json').read_text())['records']
    trained = {(r['source_video'], r.get('frame_index')) for r in train if r['split']=='train'}
    rows = []
    # Fixed source/frame ranges before V3 evaluation, with clear and moving frames.
    for video_id, first, last in [('r3_go2_07', 100, 144), ('r3_go2_08', 150, 194)]:
        spec = next(v for v in sources if v['video_id']==video_id)
        source = Path(spec['source_video_path']); assert sha256(source)==spec['source_video_sha256']
        logs = log_frames(source.with_name(source.stem.removesuffix('_raw')+'.jsonl'))
        cap = cv2.VideoCapture(str(source)); index = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if first <= index <= last:
                filename = f'continuity_{video_id}_f{index:07d}.jpg'
                assert cv2.imwrite(str(images/filename), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                stamp = logs[index]['rgb_stamp_ns']; assert stamp is not None
                rows.append(dict(image=filename, source_video=spec['source_video'], video_id=video_id,
                    source_video_sha256=spec['source_video_sha256'], frame_index=index, rgb_stamp_ns=stamp,
                    capture_group='lab_floor_20261108_09', image_sha256=sha256(images/filename),
                    status='pending', blue_target_identity='pending_manual_review',
                    exact_training_frame=(spec['source_video'],index) in trained,
                    scope='same training capture group; diagnostic only', control_log=logs[index]))
            index += 1
        cap.release(); assert index==spec['decoded_frames']
    assert len(rows)==90
    write_json(output/'manifest.json', rows)
    with zipfile.ZipFile(output/'go2_continuity_90_images.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for r in rows:
            z.write(images/r['image'], r['image'])
    with (output/'review.csv').open('w', newline='') as f:
        w=csv.DictWriter(f, fieldnames=['image','status','blue_target_identity','notes']);w.writeheader()
        w.writerows(dict(image=r['image'],status='pending',blue_target_identity='pending',notes='') for r in rows)
    for offset in range(0,90,30):
        sheet = Image.new('RGB',(1200,6*176),'white');draw=ImageDraw.Draw(sheet)
        for i,r in enumerate(rows[offset:offset+30]):
            with Image.open(images/r['image']) as im:
                im.thumbnail((236,142));x=i%5*240;y=i//5*176;sheet.paste(im,(x,y))
            draw.text((x+2,y+143),f"{r['video_id']} f{r['frame_index']}",fill='black')
            draw.text((x+2,y+158),'PENDING manual GT',fill='red')
        sheet.save(output/f'contact_{offset//30+1:02d}.jpg')
    (output/'README.md').write_text('''# GO2连续片段人工审核包

90张原始RGB连续帧，无YOLO/HSV伪标签。两段分别为r3_go2_07帧100–144、r3_go2_08帧150–194。
请将go2_continuity_90_images.zip导入CVAT，按color_block、rock_compact、rock_spiky三类标全；
review.csv逐图确认positive/negative/uncertain，并确认蓝方块身份和可辨认性（不可辨认不能计作漏检）。
缺少TXT不能自动视为负样本，需明确确认。导出Ultralytics YOLO Detection并交回审核表。
manifest保留源视频hash、源帧、原始rgb_stamp_ns、图像hash和训练帧重合状态。
所有片段同属lab_floor_20261108_09训练采集组，部分帧已参与训练，不能作独立验证。
它只支持同场景连续性诊断；真正泛化需另采不进入训练的独立GO2连续录像。
最长漏检和恢复速度须等人工逐帧真值后，以同类IoU匹配计算；无输出框不是漏检真值。
''')
    print(json.dumps(dict(images=90, exact_training_frames=sum(r['exact_training_frame'] for r in rows),output=str(output))))


def predict(output, weights):
    from ultralytics import YOLO
    output.mkdir(parents=True,exist_ok=False)
    model=YOLO(str(weights));assert model.names=={0:'color_block',1:'rock_compact',2:'rock_spiky'}
    summaries=[]
    for spec in json.loads(SOURCES.read_text()):
        source=Path(spec['source_video_path']);assert sha256(source)==spec['source_video_sha256']
        logs=log_frames(source.with_name(source.stem.removesuffix('_raw')+'.jsonl'))
        cap=cv2.VideoCapture(str(source));index=0;rows=[]
        while True:
            frames=[]
            for _ in range(8):
                ok,frame=cap.read()
                if not ok:break
                frames.append(frame)
            if not frames:break
            results=model.predict(frames,imgsz=640,conf=.10,iou=.45,rect=False,device=0,verbose=False)
            for frame,result in zip(frames,results):
                stamp=logs[index]['rgb_stamp_ns'];assert stamp is not None
                predictions=[dict(class_id=int(b.cls.item()),confidence=float(b.conf.item()),bbox_xyxy=b.xyxy[0].cpu().tolist()) for b in result.boxes]
                rows.append(dict(frame_index=index,rgb_stamp_ns=stamp,video_time_sec=index/spec['fps'],
                    predictions=predictions, max_color_conf_at_conf010=max([p['confidence'] for p in predictions if p['class_id']==0],default=None),
                    blur_metric=float(cv2.Laplacian(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var()),control_log=logs[index]))
                index+=1
        cap.release();assert index==spec['decoded_frames'] and len(logs)==index
        assert all(b['rgb_stamp_ns']>a['rgb_stamp_ns'] for a,b in zip(rows,rows[1:]))
        with (output/(spec['video_id']+'.jsonl')).open('x') as f:
            for r in rows:f.write(json.dumps(r)+'\n')
        gaps=absence_runs(rows)
        summaries.append(dict(video_id=spec['video_id'],source_video=spec['source_video'],source_video_sha256=spec['source_video_sha256'],
            frames=index,color_prediction_present_conf025=sum(any(p['class_id']==0 and p['confidence']>=.25 for p in r['predictions']) for r in rows),
            max_prediction_absence_frames=max([g['frames'] for g in gaps],default=0),
            max_prediction_absence_span_sec=max([g['observed_absence_span_sec'] for g in gaps],default=0),
            prediction_absence_runs=gaps))
    write_json(output/'summary.json',dict(weights_sha256=sha256(weights),imgsz=640,confidence_saved=.10,presence_threshold=.25,
        videos=summaries,interpretation='Prediction presence/absence only, not GT-confirmed detection/miss/recovery. Same laboratory training capture group.',
        gt_metrics='NOT VERIFIED: per-frame human boxes, blue identity and visibility pending'))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['package','predict'])
    p.add_argument('--output',type=Path,required=True);p.add_argument('--weights',type=Path)
    a=p.parse_args()
    if a.action=='package':package(a.output)
    else:
        if a.weights is None:p.error('--weights required')
        predict(a.output,a.weights)


if __name__=='__main__':main()
