#!/usr/bin/env python3
"""Source-separated validation, real training fit and class-aware error previews."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch
import yaml
from ultralytics import YOLO

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from vision_training.common import write_json, sha256
from vision_training.dataset import validate_label


def match(gt, pred, threshold=.5):
    # Class-aware greedy IoU matching; one GT/prediction can match only once.
    pairs=[]
    for gi,g in enumerate(gt):
        for pi,p in enumerate(pred):
            if g[0]!=p[0]:continue
            intersection=np.maximum(0,np.minimum(g[3:5],p[3:5])-np.maximum(g[1:3],p[1:3])).prod()
            union=np.prod(g[3:5]-g[1:3])+np.prod(p[3:5]-p[1:3])-intersection
            iou=intersection/union if union>0 else 0
            if iou>=threshold:pairs.append((iou,gi,pi))
    used_g,used_p=set(),set()
    for _,gi,pi in sorted(pairs,reverse=True):
        if gi not in used_g and pi not in used_p:used_g.add(gi);used_p.add(pi)
    return used_g,used_p


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--weights',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.dataset=args.dataset.resolve();args.output=args.output.resolve()
    args.output.mkdir(parents=True,exist_ok=False)
    records=json.loads((args.dataset/'preparation.json').read_text())['records']
    for row in records:
        image = args.dataset/row['image']
        label = args.dataset/'labels'/row['split']/(image.stem+'.txt')
        if sha256(image) != row['image_sha256'] or sha256(label) != row['label_sha256']:
            raise ValueError('Dataset changed since preparation')
    names=['color_block','rock_compact','rock_spiky']
    model=YOLO(str(args.weights))
    if [model.names[i] for i in range(3)] != names:
        raise ValueError('Model class names/order do not match dataset')
    report={'weights_sha256':sha256(args.weights),'groups':{}}
    groups=dict(synthetic_val=[r for r in records if r['split']=='val' and r['source_type']=='synthetic'],
                phone_val=[r for r in records if r['split']=='val' and r['source_type']=='phone_real'],
                phone_train_fit=[r for r in records if r['split']=='train' and r['source_type']=='phone_real'])
    for group, rows in groups.items():
        listing=args.output/(group+'.txt');listing.write_text(''.join(str(args.dataset/r['image'])+'\n' for r in rows))
        data=args.output/(group+'.yaml');data.write_text(yaml.safe_dump(dict(path=str(args.dataset),train=str(listing),val=str(listing),names=names)))
        val=model.val(data=str(data),split='val',imgsz=640,device=0,batch=8,workers=0,
                      plots=True,project=str(args.output),name=group,exist_ok=True)
        per_class={}
        for i,c in enumerate(val.box.ap_class_index):
            per_class[names[int(c)]]=dict(precision=float(val.box.p[i]),recall=float(val.box.r[i]),
                map50=float(val.box.ap50[i]),map50_95=float(val.box.ap[i]))
        metrics=dict(images=len(rows),precision=float(val.box.mp),recall=float(val.box.mr),
            map50=float(val.box.map50),map50_95=float(val.box.map),per_class=per_class,
            scope='training fit only' if group.endswith('fit') else 'held-out validation',
            confusion_matrix=val.confusion_matrix.matrix.tolist(),speed_ms=val.speed)
        cases=[];counts=Counter();tiles={i:[] for i in range(3)}
        for row in rows:
            path=args.dataset/row['image'];image=cv2.imread(str(path));h,w=image.shape[:2]
            label=Path(str(path).replace('/images/','/labels/')).with_suffix('.txt')
            boxes=validate_label(label,3)
            gt=np.array([[c,(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h] for c,x,y,bw,bh in boxes]).reshape(-1,5)
            result=model.predict(str(path),imgsz=640,conf=.25,device=0,verbose=False)[0]
            pred=np.concatenate([result.boxes.cls.cpu().numpy()[:,None],result.boxes.xyxy.cpu().numpy()],axis=1)
            used_g,used_p=match(gt,pred)
            error=dict(image=path.name,source_video=row['source_video'],gt=len(gt),pred=len(pred),
                tp=len(used_g),fn=len(gt)-len(used_g),fp=len(pred)-len(used_p),
                missed_classes=[int(gt[i,0]) for i in range(len(gt)) if i not in used_g],
                false_classes=[int(pred[i,0]) for i in range(len(pred)) if i not in used_p],
                missed_gt=[gt[i].tolist() for i in range(len(gt)) if i not in used_g],
                false_predictions=[pred[i].tolist() for i in range(len(pred)) if i not in used_p],
                gt_short_side_at_640=[float(min(g[3]-g[1],g[4]-g[2])*640/max(h,w)) for g in gt])
            counts.update({k:error[k] for k in ['tp','fp','fn']});cases.append(error)
            left=image.copy()
            for c,x1,y1,x2,y2 in gt:
                cv2.rectangle(left,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)
                cv2.putText(left,names[int(c)],(int(x1),max(16,int(y1))),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
            right=result.plot();full=np.concatenate([left,right],axis=1)
            tile=cv2.resize(full,(960,round(full.shape[0]*960/full.shape[1])))
            banner=np.zeros((38,960,3),np.uint8)
            cv2.putText(banner,f"GT | PRED conf=.25 {path.name} TP={error['tp']} FP={error['fp']} FN={error['fn']}",(4,24),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
            tile=np.concatenate([banner,tile])
            if group=='phone_val':
                for c in set(int(x) for x in list(gt[:,0])+list(pred[:,0])):
                    tiles[c].append((error['fn']+error['fp'],tile,path.name))
        metrics['fixed_conf_025_iou_05_counts']=dict(counts)
        metrics['cases']=cases
        report['groups'][group]=metrics
        for c,items in tiles.items():
            if not items:continue
            # Mix difficult cases and successes; deterministic and explicitly GT vs prediction.
            ordered=sorted(items,key=lambda t:-t[0]);chosen=ordered[:3]+list(reversed(ordered))[:3]
            seen={};[seen.setdefault(item[2],item[1]) for item in chosen]
            cv2.imwrite(str(args.output/f'{group}_{names[c]}_examples.jpg'),np.concatenate(list(seen.values())))
    first=str(args.dataset/groups['phone_val'][0]['image'])
    for _ in range(10):model.predict(first,imgsz=640,device=0,verbose=False)
    times=[];inference=[]
    for i in range(90):
        path=str(args.dataset/groups['phone_val'][i%len(groups['phone_val'])]['image'])
        torch.cuda.synchronize();started=time.perf_counter()
        result=model.predict(path,imgsz=640,device=0,verbose=False)[0]
        torch.cuda.synchronize();times.append((time.perf_counter()-started)*1000);inference.append(result.speed['inference'])
    report['timing']=dict(samples=90,warmup=10,batch=1,device=torch.cuda.get_device_name(0),
        wall_mean_ms=float(np.mean(times)),wall_p50_ms=float(np.median(times)),wall_p95_ms=float(np.percentile(times,95)),
        network_inference_mean_ms=float(np.mean(inference)),fp16=bool(model.predictor.model.fp16),scope='FP32 GPU, file decode + preprocessing + inference + NMS; excludes ROS/camera')
    write_json(args.output/'metrics.json',report)
    print(json.dumps({g:{k:v for k,v in r.items() if k!='cases'} for g,r in report['groups'].items()},indent=2))

if __name__=='__main__':main()
