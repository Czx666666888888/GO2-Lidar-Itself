#!/usr/bin/env python3
"""Compare exact same video-B GT, checked dataset/weight hashes, and saved V1/V2/V3 metrics."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json
from vision_training.dataset import label_for
from vision_training.tools.compare_science_v2 import enrich_legacy

BASE=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--v3',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    metrics_files=[BASE/'reports/science_targets_mixed_v1/evaluation_metrics.json',
                   BASE/'reports/science_targets_mixed_v2/evaluation_metrics_640.json',a.v3]
    identity=None;results={}
    for version,file in zip(('v1','v2','v3'),metrics_files):
        dataset=BASE/f'datasets/science_targets_mixed_{version}'
        weights=BASE/f'runs/science_targets_mixed_{version}/baseline/weights/best.pt'
        records=json.loads((dataset/'preparation.json').read_text())['records']
        rows=[r for r in records if r['split']=='val' and r['source_type']=='phone_real']
        gt={Path(r['image']).name:(sha256(dataset/r['image']),sha256(label_for(dataset/r['image']))) for r in rows}
        assert len(gt)==57
        assert all(gt[Path(r['image']).name]==(r['image_sha256'],r['label_sha256']) for r in rows)
        if identity is None:identity=gt
        assert gt==identity
        raw=json.loads(file.read_text());assert raw['weights_sha256']==sha256(weights)
        metrics=raw['groups']['phone_val']
        if 'fixed_conf_per_class' not in metrics:metrics=enrich_legacy(metrics,dataset)
        names=sorted(c['image'] for c in metrics['cases'])
        assert names==sorted(gt)
        results[version]=dict(weights_sha256=sha256(weights),imgsz=raw.get('imgsz',640),
            metrics={k:v for k,v in metrics.items() if k not in ('cases','confusion_matrix','speed_ms')})
    write_json(a.output,dict(validation_images=identity,results=results,
        protocol='All 57 video-B images and labels identical by SHA256. P/R from Ultralytics curve; TP/FP/FN at conf=.25/class-aware IoU=.5. Small=short side<16 after scaling long side to640.',
        limitation='B is development validation used during training selection; not untouched final test. No claim of motion continuity from still-frame B results.'))
    print(json.dumps({v:d['metrics']['per_class'] for v,d in results.items()},indent=2))


if __name__=='__main__':main()
