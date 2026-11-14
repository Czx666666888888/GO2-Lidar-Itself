#!/usr/bin/env python3
"""Collect same-scene model/resolution evidence without treating confidence as accuracy."""
import argparse
from collections import Counter
from PIL import Image
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import write_json, sha256
from vision_training.dataset import label_for, validate_label


def enrich_legacy(metrics, dataset):
    """Derive fixed-threshold subgroup counts from saved exact missed boxes and GT."""
    records=json.loads((dataset/'preparation.json').read_text())['records']
    by_name={Path(r['image']).name:r for r in records}
    names=['color_block','rock_compact','rock_spiky']
    counts={n:Counter(tp=0,fp=0,fn=0) for n in names}
    small={n:Counter(tp=0,fn=0) for n in names}
    multi=Counter(images=0,tp=0,fp=0,fn=0)
    for case in metrics['cases']:
        row=by_name[case['image']];image=dataset/row['image'];label=label_for(image)
        if sha256(image)!=row['image_sha256'] or sha256(label)!=row['label_sha256']:
            raise ValueError('Legacy data changed')
        gt=validate_label(label,3)
        with Image.open(image) as im: w,h=im.size
        for c,x,y,bw,bh in gt:
            n=names[int(c)];counts[n]['tp']+=1
            if min(bw*w,bh*h)*640/max(h,w)<16: small[n]['tp']+=1
        for c in case['missed_classes']:
            counts[names[c]]['tp']-=1;counts[names[c]]['fn']+=1
        for c in case['false_classes']: counts[names[c]]['fp']+=1
        for c,x1,y1,x2,y2 in case['missed_gt']:
            if min(x2-x1,y2-y1)*640/max(h,w)<16:
                small[names[int(c)]]['tp']-=1;small[names[int(c)]]['fn']+=1
        if case['gt']>1: multi.update({k:case[k] for k in ('tp','fp','fn')});multi['images']+=int(case['gt']>1)
    metrics=dict(metrics)
    metrics['fixed_conf_per_class']={n:dict(c) for n,c in counts.items()}
    metrics['small_short_side_lt16_at_640']={n:dict(c) for n,c in small.items()}
    metrics['multi_object_counts']=dict(multi)
    return metrics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--v1',type=Path,required=True)
    p.add_argument('--v2',type=Path,required=True)
    p.add_argument('--v1-dataset', type=Path, required=True)
    p.add_argument('--v2-960',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    sources={'V1_640':args.v1,'V2_640':args.v2}
    if args.v2_960: sources['V2_960_inference']=args.v2_960
    report={}
    identity=None
    for name,path in sources.items():
        raw=json.loads(path.read_text())
        metrics=raw['groups']['phone_val']
        if 'fixed_conf_per_class' not in metrics:
            metrics=enrich_legacy(metrics,args.v1_dataset)
        names=sorted(c['image'] for c in metrics['cases'])
        if identity is None: identity=names
        if identity!=names or len(names)!=57:
            raise ValueError('Comparison must use exactly the same 57 video B images')
        report[name]=dict(weights_sha256=raw['weights_sha256'],imgsz=raw.get('imgsz',640),
            metrics={k:v for k,v in metrics.items() if k not in ('cases','confusion_matrix','speed_ms')})
    write_json(args.output,dict(validation_images=identity,results=report,
        protocol='Ultralytics P/R at curve-selected operating point; fixed counts separately at conf0.25/class-aware IoU0.5',
        limitation='B is development validation used in model selection, not untouched final test; 960 uses 640-trained weights unless explicitly documented otherwise'))
    for run,data in report.items():
        print(run)
        print(json.dumps(data['metrics'],indent=2))


if __name__=='__main__':main()
