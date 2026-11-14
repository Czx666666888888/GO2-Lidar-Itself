#!/usr/bin/env python3
"""Prepare unlabelled round-three candidates; never train or promote predictions to GT."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import zipfile

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json
from vision_training.tools.extract_training_frames import extract, sheet, representative_rows, fingerprint, difference

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT/'vision_training/datasets/science_targets_round3_candidates'
REPORT = ROOT/'vision_training/reports/science_targets_round3'
CONFIG = ROOT/'vision_training/configs/science_targets_round3.yaml'
ANN = ROOT/'vision_training/annotations/science_targets_round3'
MODEL = ROOT/'vision_training/runs/science_targets_mixed_v2/baseline/weights/best.pt'
MODEL_SHA = 'e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1'


def protection():
    bases = [ROOT/'vision_training/datasets'/n for n in (
        'science_targets_synthetic_v1','science_targets_video_v1','science_targets_v1',
        'science_targets_mixed_v1','science_targets_mixed_v2','science_targets_round2_reviewed_v2',
        'blue_cube','blue_cube_pilot_v1')]
    bases += [ROOT/'vision_training/runs'/n for n in ('science_targets_mixed_v1','science_targets_mixed_v2')]
    bases += [ROOT/'vision_training/annotations/science_targets_round2_priority']
    files = {p for base in bases for p in base.rglob('*') if p.is_file()}
    files.update(p for p in (ROOT/'camera_ws/records').glob('approach_2026110[6-9]*') if p.is_file())
    return {str(p.relative_to(ROOT)):sha256(p) for p in sorted(files) if p.suffix not in ('.cache','.pyc')}


def log_frames(path):
    result={}
    if not path.is_file():return result
    with path.open() as f:
        for line in f:
            row=json.loads(line)
            if row.get('kind')=='frame':
                i=row['frame_index']
                if i in result:raise ValueError('Duplicate frame log index')
                c=row.get('control',{})
                result[i]=dict(rgb_stamp_ns=row.get('rgb_stamp_ns'),state=c.get('state'),
                    vx=c.get('vx'),wz=c.get('wz'),actual_speed=c.get('actual_speed'),
                    sport_velocity=c.get('sport_velocity'),hsv_visible=row.get('visible'),
                    camera_distance_xy=c.get('camera_distance_xy'))
    return result


def audit():
    if OUTPUT.exists() or REPORT.exists():raise FileExistsError('Use a new output version')
    OUTPUT.mkdir(parents=True);REPORT.mkdir(parents=True)
    write_json(OUTPUT/'protected_before.json',protection())
    cfg=yaml.safe_load(CONFIG.read_text())
    incoming=ROOT/'vision_training/video_sources/science_targets_round3'
    incoming.mkdir(parents=True,exist_ok=True)
    located=[]
    search=Path('/home/czx666/文档/xwechat_files')
    for spec in cfg['new_videos']:
        target=incoming/spec['filename']
        if not target.exists():
            matches=list(search.rglob(spec['filename']))
            if len(matches)!=1:raise FileNotFoundError(f'Need exactly one {spec["filename"]}: {matches}')
            shutil.copy2(matches[0],target)
            assert sha256(target)==sha256(matches[0])
        located.append(dict(spec,path=str(target)))
    history=sorted((ROOT/'camera_ws/records').glob('approach_2026110[6-9]*_raw.mp4'))
    for i,p in enumerate(history,1):
        located.append(dict(id=f'r3_go2_{i:02d}',filename=p.name,path=str(p),
            capture_group=cfg['historical']['capture_group'],role='train_candidate',inference=True,
            priority=p.name in cfg['historical']['priority']))
    write_json(OUTPUT/'sources.json',located)
    sampling=yaml.safe_load((ROOT/'vision_training/configs/science_targets_video_20261113.yaml').read_text())['sampling']
    sampling.update(min_interval_sec=.12,base_interval_sec=.25,max_interval_sec=.60)
    for group,typ,confirmation in [('new','phone_real',cfg['source_confirmation']),('go2','d435_real',cfg['historical']['source_confirmation'])]:
        specs=[r for r in located if ('_new' in r['id'])==(group=='new')]
        config=dict(input_dir=str(ROOT),source_type=typ,source_confirmation=confirmation,
            videos=[dict(id=r['id'],filename=r['path'],capture_group=r['capture_group']) for r in specs],
            sampling=sampling,batch_size=160)
        config_path=OUTPUT/(group+'_extraction.yaml');config_path.write_text(yaml.safe_dump(config,allow_unicode=True))
        extract(config_path,OUTPUT/('audit_'+group),make_packages=False)
    videos=[]
    for group in ('new','go2'):
        folder=OUTPUT/('audit_'+group)
        for v in json.loads((folder/'manifests/videos.json').read_text()):
            logs=log_frames(ROOT/'camera_ws/records'/(Path(v['source_video']).stem.removesuffix('_raw')+'.jsonl')) if group=='go2' else {}
            v['frame_log_records']=len(logs)
            v['rgb_source_elapsed_sec']=((logs[max(logs)]['rgb_stamp_ns']-logs[min(logs)]['rgb_stamp_ns'])/1e9 if logs else None)
            videos.append(v)
            shutil.copy2(folder/'reports'/(v['video_id']+'_overview.jpg'),REPORT/(v['video_id']+'_overview.jpg'))
    write_json(REPORT/'video_audit.json',videos)
    assert sha256(MODEL)==MODEL_SHA
    write_json(REPORT/'model_status.json',dict(path=str(MODEL.relative_to(ROOT)),sha256=MODEL_SHA,
        training='Existing V2 only; no training performed',names=cfg['names']))
    print(json.dumps([dict(video=v['video_id'],frames=v['decoded_frames'],size=v['resolution'],
                           audit_candidates=v['counts']['kept'],rgb_elapsed=v['rgb_source_elapsed_sec']) for v in videos],indent=2))


def box_iou(a,b):
    x1,y1=max(a[0],b[0]),max(a[1],b[1]);x2,y2=min(a[2],b[2]),min(a[3],b[3])
    inter=max(0.,x2-x1)*max(0.,y2-y1)
    area=lambda x:max(0.,x[2]-x[0])*max(0.,x[3]-x[1])
    return inter/max(1e-9,area(a)+area(b)-inter)


def mine():
    from ultralytics import YOLO
    assert sha256(MODEL)==MODEL_SHA
    cfg=yaml.safe_load(CONFIG.read_text())
    model=YOLO(str(MODEL))
    assert [model.names[i] for i in range(3)]==cfg['names']
    shape={}
    def observed(_module,args):
        if args and hasattr(args[0],'shape'):shape['tensor']=list(args[0].shape)
    hook=model.model.register_forward_pre_hook(observed)
    folder=OUTPUT/'mining';folder.mkdir(exist_ok=False)
    summaries=[]
    for spec in json.loads((OUTPUT/'sources.json').read_text()):
        if not spec['inference']:
            summaries.append(dict(video_id=spec['id'],inference=False,reason='Reserved video: no model-based mining'))
            continue
        source=Path(spec['path']);cap=cv2.VideoCapture(str(source))
        fps=cap.get(cv2.CAP_PROP_FPS);declared=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        logs=log_frames(source.with_name(source.stem.removesuffix('_raw')+'.jsonl')) if 'go2' in spec['id'] else {}
        first_stamp=logs[min(logs)]['rgb_stamp_ns'] if logs else None
        index=0;previous=None;previous_thumb=None;rows=[]
        timeline=folder/(spec['id']+'_predictions.jsonl')
        with timeline.open('x') as stream:
            while True:
                frames=[]
                for _ in range(8):
                    ok,frame=cap.read()
                    if not ok:break
                    frames.append(frame)
                if not frames:break
                results=model.predict(frames,imgsz=cfg['imgsz'],conf=cfg['confidence'],iou=cfg['nms_iou'],
                                      rect=False,device=0,verbose=False)
                for frame,result in zip(frames,results):
                    h,w=frame.shape[:2];gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                    thumb=cv2.resize(gray,(160,90),interpolation=cv2.INTER_AREA)
                    motion=float(cv2.absdiff(thumb,previous_thumb).mean()) if previous_thumb is not None else 0.
                    previous_thumb=thumb
                    predictions=[]
                    for box in result.boxes:
                        cls=int(box.cls.item());b=[float(v) for v in box.xyxy[0].cpu().tolist()]
                        short=min(b[2]-b[0],b[3]-b[1])*640/max(w,h)
                        edge=min(b[0]/w,b[1]/h,1-b[2]/w,1-b[3]/h)<.05
                        x1,y1,x2,y2=[max(0,int(v)) for v in b];crop=gray[y1:min(h,y2),x1:min(w,x2)]
                        predictions.append(dict(class_id=cls,class_name=model.names[cls],confidence=float(box.conf.item()),
                            bbox_xyxy=b,short_side_at_640=short,size_band='small' if short<16 else 'medium_small' if short<=32 else 'large',
                            edge_candidate=edge,roi_laplacian=float(cv2.Laplacian(crop,cv2.CV_64F).var()) if crop.size else None))
                    class_change=bool(previous is not None and any(p['class_id']!=q['class_id'] and
                        box_iou(p['bbox_xyxy'],q['bbox_xyxy'])>.3 for p in predictions for q in previous))
                    class_counts=Counter(p['class_id'] for p in predictions)
                    logged=logs.get(index,{})
                    clock=(logged['rgb_stamp_ns']-first_stamp)/1e9 if logged.get('rgb_stamp_ns') and first_stamp else index/fps
                    row=dict(frame_index=index,video_time_sec=index/fps,source_seconds=clock,
                        time_basis='rgb_source_stamp' if logged.get('rgb_stamp_ns') else 'container_fps',
                        blur_metric=float(cv2.Laplacian(gray,cv2.CV_64F).var()),mean_gray=float(gray.mean()),motion_mae=motion,
                        predictions=predictions,classes=dict(class_counts),
                        max_color_block_confidence_at_conf010=max([p['confidence'] for p in predictions if p['class_id']==0] or [0.]),
                        predicted_class_change=class_change,
                        prediction_count_change=previous is not None and len(previous)!=len(predictions),
                        inference_imgsz=cfg['imgsz'],observed_tensor_shape=shape.get('tensor'),
                        model_sha256=MODEL_SHA,control_log=logged,
                        interpretation='Predictions are mining hints only; no boxes does not imply negative')
                    stream.write(json.dumps(row,allow_nan=False)+'\n');rows.append(row)
                    previous=predictions;index+=1
        cap.release()
        assert index==declared and sha256(source)==next(v['source_video_sha256'] for group in ('new','go2')
            for v in json.loads((OUTPUT/('audit_'+group)/'manifests/videos.json').read_text()) if v['video_id']==spec['id'])
        gaps=[]
        for cls in range(3):
            start=None
            for i,row in enumerate(rows+[{'predictions':[dict(class_id=cls)]}]):
                visible=any(p['class_id']==cls for p in row['predictions'])
                if not visible and start is None:start=i
                if visible and start is not None:
                    end=i-1
                    gaps.append(dict(class_id=cls,first_frame=start,last_frame=end,frames=end-start+1,
                        start_sec=rows[start]['source_seconds'],end_sec=rows[end]['source_seconds'],
                        duration_sec=max(0.,rows[end]['source_seconds']-rows[start]['source_seconds']),
                        bounded_by_predictions=start>0 and i<len(rows),
                        interpretation='No prediction above conf=.10, NOT GT-confirmed missed detection'))
                    start=None
        summary=dict(video_id=spec['id'],inference=True,frames=index,frames_with_predictions=sum(bool(r['predictions']) for r in rows),
            predicted_small_frames=sum(any(p['short_side_at_640']<16 for p in r['predictions']) for r in rows),
            predicted_edge_frames=sum(any(p['edge_candidate'] for p in r['predictions']) for r in rows),
            class_change_frames=sum(r['predicted_class_change'] for r in rows),
            prediction_absence_runs=gaps,full_timeline=True)
        summaries.append(summary)
        print(spec['id'],index,'full frames inferred',flush=True)
    hook.remove()
    write_json(folder/'summary.json',summaries)
    write_json(REPORT/'mining_summary.json',summaries)


def read_predictions(video_id):
    path=OUTPUT/'mining'/(video_id+'_predictions.jsonl')
    return [json.loads(s) for s in path.read_text().splitlines()] if path.exists() else []


def hints(row, previous=None):
    ps=row.get('predictions',[]);flags=[]
    if not ps:flags.append('v2_prediction_absent_not_negative')
    if any(p['short_side_at_640']<16 for p in ps):flags.append('predicted_small_lt16')
    if any(p['edge_candidate'] for p in ps):flags.append('predicted_edge')
    if len({p['class_id'] for p in ps})>1:flags.append('predicted_multi_class')
    if row.get('predicted_class_change'):flags.append('predicted_class_change')
    if row.get('prediction_count_change'):flags.append('prediction_transition')
    c=row.get('control_log',{})
    if c.get('state'):flags.append('control_'+c['state'])
    moving=abs(c.get('vx') or 0)>.01 or abs(c.get('wz') or 0)>.03 or (c.get('actual_speed') or 0)>.03
    if moving:flags.append('control_motion_hint')
    if c.get('hsv_visible'):flags.append('hsv_visibility_hint_NOT_GT')
    if row.get('blur_metric',math.inf)<25:flags.append('low_laplacian_review_only')
    if row.get('motion_mae',0)>3:flags.append('large_frame_change')
    return flags


def known_annotations():
    v2=ROOT/'vision_training/datasets/science_targets_mixed_v2'
    data=json.loads((v2/'preparation.json').read_text())['records']
    records=[]
    for r in data:
        if r['source_type']=='phone_real':
            records.append(dict(image=str(v2/r['image']),source_video=r['source_video'],
                source_video_sha256=r['source_video_sha256'],frame_index=r['frame_index'],
                image_sha256=r['image_sha256'],kind='reviewed_335'))
    legacy=ROOT/'vision_training/datasets/blue_cube_pilot_v1'
    old=json.loads((legacy/'version.json').read_text())['records']
    reused=[]
    for r in old:
        image=legacy/'images'/r['split']/r['filename'];label=legacy/'labels'/r['split']/(Path(r['filename']).stem+'.txt')
        assert sha256(image)==r['image_sha256'] and sha256(label)==r['label_sha256']
        source=ROOT/'camera_ws/records'/r['source_video']
        records.append(dict(image=str(image),source_video=r['source_video'],source_video_sha256=sha256(source),
            frame_index=r['source_frame_index'],image_sha256=r['image_sha256'],kind='legacy_20'))
        reused.append(dict(r,label_source=str(label.relative_to(ROOT)),image_source=str(image.relative_to(ROOT)),
            reuse_review='Original human blue_cube box maps to color_block0; other-class completeness NOT re-reviewed'))
    # Pending round-two images are protected, never re-sent or called negatives.
    pool=ROOT/'vision_training/datasets/science_targets_round2_reviewed_v2'
    for r in json.loads((pool/'pool/manifest.json').read_text()):
        if not r['reviewed']:
            records.append(dict(image=str(pool/r['image']),source_video=r['source_video'],
                source_video_sha256=r['source_video_sha256'],frame_index=r['frame_index'],
                image_sha256=r['image_sha256'],kind='pending_19'))
    return records,reused


def curate():
    cfg=yaml.safe_load(CONFIG.read_text());sources=json.loads((OUTPUT/'sources.json').read_text())
    known,reused=known_annotations()
    assert Counter(r['kind'] for r in known)=={'reviewed_335':335,'legacy_20':20,'pending_19':19}
    known_bits=[];known_thumbs=[]
    for r in known:
        gray=cv2.imread(r['image'],cv2.IMREAD_GRAYSCALE);small,bits=fingerprint(gray)
        known_bits.append(bits);known_thumbs.append(small)
    known_bits=np.array(known_bits)
    known_hashes={r['image_sha256'] for r in known}
    exclusion=Counter();excluded=[];available={};settings=yaml.safe_load((OUTPUT/'new_extraction.yaml').read_text())['sampling']
    for group in ('new','go2'):
        folder=OUTPUT/('audit_'+group)
        for r in json.loads((folder/'manifests/images.json').read_text()):
            match=next((k for k in known if k['source_video_sha256']==r['source_video_sha256'] and k['frame_index']==r['frame_index']),None)
            reason='already_annotated_source_frame' if match else None
            if not reason and r['image_sha256'] in known_hashes:reason='known_exact_image_hash'
            if not reason and any(k['kind']=='legacy_20' and k['source_video_sha256']==r['source_video_sha256'] and
                    abs(k['frame_index']-r['frame_index'])<=7 for k in known):reason='legacy_label_temporal_neighbour_not_exact_duplicate'
            image=folder/r['image_file'];gray=cv2.imread(str(image),cv2.IMREAD_GRAYSCALE);small,bits=fingerprint(gray)
            if not reason:
                for i in np.flatnonzero(np.count_nonzero(known_bits!=bits,axis=1)<=2):
                    if float(cv2.absdiff(small,known_thumbs[i]).mean())>.6:continue
                    old=cv2.imread(known[int(i)]['image'],cv2.IMREAD_GRAYSCALE)
                    if old.shape==gray.shape and difference(gray,old,settings)['near_duplicate']:
                        reason='known_strict_near_duplicate';break
            if reason:
                exclusion[reason]+=1;excluded.append(dict(filename=r['filename'],reason=reason));continue
            row=dict(r,audit_image=str(image),_small=small,_bits=bits)
            available.setdefault(r['video_id'],[]).append(row)
    selected=[];selected_hash=set();chosen_visual=[]
    def accept(row):
        if row['image_sha256'] in selected_hash:return False
        if any(row['video_id']==old['video_id'] and abs(row['source_seconds']-old['source_seconds'])<.5 for old in chosen_visual):
            exclusion['temporal_neighbour_selection_attempts']+=1
            return False
        for old in chosen_visual:
            if np.count_nonzero(row['_bits']!=old['_bits'])>2:continue
            if float(cv2.absdiff(row['_small'],old['_small']).mean())>.6:continue
            a=cv2.imread(row['audit_image'],cv2.IMREAD_GRAYSCALE);b=cv2.imread(old['audit_image'],cv2.IMREAD_GRAYSCALE)
            if a.shape==b.shape and difference(a,b,settings)['near_duplicate']:
                exclusion['selected_strict_near_duplicate']+=1;return False
        selected_hash.add(row['image_sha256']);chosen_visual.append(row);selected.append(row);return True
    def choose(spec,count):
        rows=available[spec['id']];pred=read_predictions(spec['id']);by_index={r['frame_index']:r for r in pred}
        if 'go2' in spec['id']:
            markers=[p['frame_index'] for p in pred if p.get('control_log',{}).get('hsv_visible')]
            if markers:
                rows=[r for r in rows if min(markers)-15<=r['frame_index']<=max(markers)+15]
        if not rows:return 0
        for row in rows:
            p=by_index.get(row['frame_index']);flags=hints(p) if p else ['held_out_temporal_coverage_no_inference']
            row.update(selection_reason=flags,model_predictions=p['predictions'] if p else None,
                mining_source_frame=row['frame_index'] if p else None,
                control_log=p.get('control_log',{}) if p else {},
                rgb_stamp_ns=p.get('control_log',{}).get('rgb_stamp_ns') if p else None,
                source_rgb_elapsed_sec=p.get('source_seconds') if p and 'go2' in spec['id'] else None,
                blur_metric=row['sharpness'],reviewed=False,annotation_status='pending',
                proposed_role=spec['role'],class_confirmation='Unknown; manual review required')
            if p:
                row['_score']=(4*('predicted_small_lt16' in flags)+3*('predicted_edge' in flags)+
                    3*('predicted_class_change' in flags)+2*('prediction_transition' in flags)+
                    3*('control_motion_hint' in flags)+2*('hsv_visibility_hint_NOT_GT' in flags)+
                    2*('low_laplacian_review_only' in flags))
            else:row['_score']=0
        rows=sorted(rows,key=lambda r:r['frame_index']);picked=0
        # One temporal stratum per target image; quality hints rank within it, not remove blur globally.
        for chunk in np.array_split(np.arange(len(rows)),min(count,len(rows))):
            choices=sorted((rows[int(i)] for i in chunk),key=lambda r:(-r['_score'],r['frame_index']))
            for row in choices:
                if accept(row):picked+=1;break
        return picked
    for spec in sources:
        if '_new' in spec['id']:choose(spec,spec['count'])
    hist=[s for s in sources if 'go2' in s['id']]
    # Priority recordings receive 30 each; other clips spread the remaining 70.
    for i,spec in enumerate(hist):
        count=30 if spec['priority'] else (9 if i<7 else 8)
        choose(spec,count)
    history_count=sum('go2' in r['video_id'] for r in selected)
    # Fill remaining temporal diversity only if distinct eligible images exist.
    if history_count<cfg['historical']['count']:
        for spec in sorted(hist,key=lambda s:not s['priority']):
            if sum('go2' in r['video_id'] for r in selected)>=cfg['historical']['count']:break
            remaining=cfg['historical']['count']-sum('go2' in r['video_id'] for r in selected)
            choose(spec,min(len(available[spec['id']]),remaining+8))
    # Cap history at requested quota; don't discard source diversity from early strata.
    hist_rows=[r for r in selected if 'go2' in r['video_id']]
    if len(hist_rows)>cfg['historical']['count']:
        remove={r['filename'] for r in hist_rows[cfg['historical']['count']: ]}
        selected=[r for r in selected if r['filename'] not in remove]
    image_dir=OUTPUT/'selected/images';image_dir.mkdir(parents=True,exist_ok=False)
    records=[]
    for row in selected:
        target=image_dir/row['filename'];shutil.copy2(row['audit_image'],target)
        clean={k:v for k,v in row.items() if not k.startswith('_') and k!='audit_image'}
        clean['image_file']=str(target.relative_to(OUTPUT));clean['image']=clean['image_file']
        records.append(clean)
    reused_dir=OUTPUT/'reused_legacy20';(reused_dir/'images').mkdir(parents=True);(reused_dir/'labels').mkdir()
    for r in reused:
        shutil.copy2(ROOT/r['image_source'],reused_dir/'images'/r['filename'])
        shutil.copy2(ROOT/r['label_source'],reused_dir/'labels'/(Path(r['filename']).stem+'.txt'))
    write_json(reused_dir/'manifest.json',reused)
    write_json(REPORT/'legacy20_reuse.json',reused)
    write_json(OUTPUT/'selected/manifest.json',records)
    write_json(OUTPUT/'duplicate_audit.json',dict(counts=dict(exclusion),excluded=excluded,
        known_reviewed_335=335,legacy20_reused=20,pending19_protected=19,known_images_selected=0))
    ANN.mkdir(parents=True,exist_ok=False)
    with (ANN/'review.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['image','status','notes'],lineterminator='\n');w.writeheader()
        for r in records:w.writerow(dict(image=r['filename'],status='pending',notes='Unlabelled candidate; no automatic negative'))
    for spec in sources:
        rows=[r for r in records if r['video_id']==spec['id']]
        if rows:sheet(OUTPUT,rows,ANN/'contact_sheets'/(spec['id']+'_selected.jpg'),columns=4,tile_width=400)
    write_json(REPORT/'selection_counts.json',dict(selected=len(records),sources=dict(Counter(r['video_id'] for r in records)),
        roles=dict(Counter(r['proposed_role'] for r in records)),duplicate_exclusions=dict(exclusion)))
    print((REPORT/'selection_counts.json').read_text())


def apply_legacy_attention(records):
    plan=yaml.safe_load((ROOT/'vision_training/datasets/blue_cube/annotation_plan.yaml').read_text())
    for kind,names in plan.get('attention',{}).items():
        for name in names:
            video,frame=Path(name).stem.rsplit('_',1)
            for r in records:
                if r['source_video']==video+'.mp4' and abs(r['frame_index']-int(frame))<=3:
                    tag='legacy_attention_'+kind
                    if tag not in r['selection_reason']:r['selection_reason'].append(tag)


def package_final():
    from vision_training.tools.extract_training_frames import package
    cfg=yaml.safe_load(CONFIG.read_text())
    quality=yaml.safe_load((ROOT/'vision_training/configs/science_targets_round3_quality.yaml').read_text())
    records=json.loads((OUTPUT/'selected/manifest.json').read_text())
    uncertain=set(quality['uncertain_boundary']);blur=set(quality['visually_blurred_recognizable_blocks'])
    for r in records:
        r['quality_review_basis']=quality['basis'] if 'go2' in r['video_id'] else 'Source overview inspected; not per-object annotation'
        r['visually_blurred_block']=r['filename'] in blur
        r['annotation_status']='uncertain' if r['filename'] in uncertain else 'pending'
        r['reviewed']=False
        r['eligible_for_priority_annotation']=r['filename'] not in uncertain and r['proposed_role']=='train_candidate'
        r['eligible_for_training']=False
    apply_legacy_attention(records)
    (OUTPUT/'pool').mkdir(exist_ok=True)
    write_json(OUTPUT/'pool/manifest.json',records)
    (OUTPUT/'data.yaml').write_text(yaml.safe_dump(dict(names=cfg['names'],nc=3,status='unreviewed_candidates_no_formal_split')))
    train=[r for r in records if r['proposed_role']=='train_candidate' and r['annotation_status']=='pending']
    (OUTPUT/'manifests').mkdir(exist_ok=False)
    write_json(OUTPUT/'manifests/images.json',train)
    batches=package(OUTPUT,cfg['batch_size'],ANN/'priority_packages',ANN/'priority_contact_sheets')
    for b in batches:
        source=Path(b['archive']);dest=ANN/('round3_priority_'+source.name)
        shutil.move(str(source),dest)
        b['archive']=dest.name;b['sha256']=sha256(dest)
        for row in b['records']:
            original=next(r for r in records if r['filename']==row['filename'])
            row.update({k:original[k] for k in ['source_type','source_video_sha256','source_seconds','capture_group','selection_reason','blur_metric','reviewed','annotation_status','proposed_role']})
        image=ANN/'priority_contact_sheets'/(b['batch']+'_representatives.jpg')
        shutil.copy2(image,REPORT/(b['batch']+'_representatives.jpg'))
    with (ANN/'review.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['image','status','notes'],lineterminator='\n');w.writeheader()
        for r in records:w.writerow(dict(image=r['filename'],status=r['annotation_status'],notes=(
            'Assistant quality triage: boundary ambiguous; excluded from ZIP, not a negative' if r['annotation_status']=='uncertain' else
            'All class identities/completeness and negatives require human confirmation')))
    (ANN/'classes.txt').write_text('\n'.join(cfg['names'])+'\n')
    (ANN/'label_schema.yaml').write_text(yaml.safe_dump(dict(names=cfg['names'],nc=3)))
    write_json(ANN/'manifest.json',dict(names=cfg['names'],records=records,batches=batches,
        classification='Prediction hints only, no generated object labels',formal_splits_created=False))
    write_json(OUTPUT/'selected/manifest.json',records)
    compact=[]
    fields=['filename','image_file','source_type','source_video','source_video_sha256','frame_index','source_seconds',
            'source_rgb_elapsed_sec','rgb_stamp_ns','capture_group','image_sha256','resolution','selection_reason','blur_metric',
            'reviewed','annotation_status','proposed_role','visually_blurred_block']
    for r in records:compact.append({k:r.get(k) for k in fields})
    write_json(REPORT/'selection_manifest.json',compact)
    shutil.copy2(ANN/'review.csv',REPORT/'review.csv')
    shutil.copy2(OUTPUT/'duplicate_audit.json',REPORT/'duplicate_audit.json')
    # Limited Git previews: retain audit overviews locally; publish only two priority batch sheets,
    # four source overviews and a clear/blur comparison rather than bulk images.
    local=REPORT/'local_audit_previews';local.mkdir(exist_ok=True)
    for p in list(REPORT.glob('r3_go2_*_overview.jpg')):shutil.move(str(p),local/p.name)
    example_names=[quality['visually_blurred_recognizable_blocks'][i] for i in [0,1,3,7,14,17]]
    examples=[r for r in records if r['filename'] in example_names]
    examples += [r for r in records if r['video_id']=='r3_go2_08' and r['frame_index'] in (239,485)]
    sheet(OUTPUT,examples,REPORT/'go2_blur_clear_examples.jpg',columns=4,tile_width=400)
    for b in batches:
        path=ANN/b['archive'];expected={r['filename']:r['image_sha256'] for r in b['records']}
        with zipfile.ZipFile(path) as z:
            assert not z.testzip() and set(z.namelist())==set(expected)
            for n in z.namelist():
                assert Path(n).suffix=='.jpg' and hashlib.sha256(z.read(n)).hexdigest()==expected[n]
    # Independently decode original videos again and reproduce every selected JPEG.
    checked=0
    for spec in json.loads((OUTPUT/'sources.json').read_text()):
        target={r['frame_index']:r for r in records if r['video_id']==spec['id']}
        cap=cv2.VideoCapture(spec['path']);index=0
        while True:
            ok,image=cap.read()
            if not ok:break
            if index in target:
                r=target[index];ok,data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,97]);assert ok
                assert hashlib.sha256(data.tobytes()).hexdigest()==r['image_sha256']==sha256(OUTPUT/r['image_file'])
                assert list(image.shape[1::-1])==r['resolution'];checked+=1
            index+=1
        cap.release()
        v=next(x for x in json.loads((REPORT/'video_audit.json').read_text()) if x['video_id']==spec['id'])
        assert index==v['decoded_frames'] and sha256(spec['path'])==v['source_video_sha256']
        if spec['inference']:
            timeline=read_predictions(spec['id'])
            assert [x['frame_index'] for x in timeline]==list(range(index))
    assert checked==len(records)
    # Check the WeChat originals against the untouched local copies as well.
    for spec in cfg['new_videos']:
        matches=list(Path('/home/czx666/文档/xwechat_files').rglob(spec['filename']))
        assert len(matches)==1 and sha256(matches[0])==sha256(ROOT/'vision_training/video_sources/science_targets_round3'/spec['filename'])
    before=json.loads((OUTPUT/'protected_before.json').read_text());after=protection();assert before==after
    write_json(REPORT/'preservation.json',dict(unchanged=True,files=len(before),sha256=before))
    ps=[r for r in records if r['model_predictions'] is not None]
    class_frames={n:sum(any(p['class_id']==i for p in r['model_predictions']) for r in ps) for i,n in enumerate(cfg['names'])}
    class_boxes={n:sum(p['class_id']==i for r in ps for p in r['model_predictions']) for i,n in enumerate(cfg['names'])}
    summary=dict(total_candidates=len(records),sources=dict(Counter(r['video_id'] for r in records)),
        roles=dict(Counter(r['proposed_role'] for r in records)),priority_zip_images=len(train),
        pending=sum(r['annotation_status']=='pending' for r in records),uncertain=len(uncertain),confirmed_negative=0,
        go2_candidates=sum('go2' in r['video_id'] for r in records),go2_visually_blurred_motion_supported=len(blur),
        legacy20_reused=20,old_reviewed_335_selected=0,pending19_selected=0,
        predicted_class_frames=class_frames,predicted_boxes=class_boxes,
        predicted_small_frames=sum(any(p['short_side_at_640']<16 for p in r['model_predictions']) for r in ps),
        predicted_edge_frames=sum(any(p['edge_candidate'] for p in r['model_predictions']) for r in ps),
        prediction_assessed_images=len(ps),heldout_unassessed_images=len(records)-len(ps),
        gt_class_counts='Unknown: not annotated; grey rocks require human identity confirmation',
        new_video_predicted_class_frames={n:sum(any(p['class_id']==i for p in (r['model_predictions'] or [])) for r in records if r['video_id'] in ('r3_new1','r3_new2')) for i,n in enumerate(cfg['names'])},
        legacy_attention_edge_candidates=sum('legacy_attention_edge_check' in r['selection_reason'] for r in records),
        prediction_size_counts_are_not_gt=True,model_imgsz=640,model_confidence=.10,
        batches=[{k:b[k] for k in ('batch','archive','images','sha256','source_counts')} for b in batches],
        verified_original_frame_matches=checked,full_input_frames=sum(v['decoded_frames'] for v in json.loads((REPORT/'video_audit.json').read_text())),
        full_inferred_frames=sum(v['frames'] for v in json.loads((OUTPUT/'mining/summary.json').read_text()) if v['inference']),
        preserved_files=len(before),training_performed=False,formal_splits_created=False)
    write_json(REPORT/'summary.json',summary)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['audit','mine','curate','package_final'])
    args=p.parse_args();globals()[args.stage]()
