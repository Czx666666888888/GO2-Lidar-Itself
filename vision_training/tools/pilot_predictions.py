#!/usr/bin/env python3
"""Save all pilot validation predictions and preselected unlabelled qualitative cases."""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import new_output, sha256, write_json  # noqa: E402
from vision_training.dataset import validate_label  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights', required=True)
    p.add_argument('--dataset', required=True)
    p.add_argument('--source', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    root, source, out = Path(a.dataset), Path(a.source), new_output(a.output)
    version = json.loads((root / 'version.json').read_text())
    used_names = {r['filename'] for r in version['records']}
    used_videos = {r['source_video'] for r in version['records']}
    rows = json.loads((source / 'manifest.json').read_text())['images']
    candidates = [r for r in rows if r['action'] == 'include'
                  and r['filename'] not in used_names and r['source_video'] not in used_videos]
    selected, selected_names = [], set()
    for tag, limit in [('dim_appearance', 4), ('edge_check', 4), ('motion_blur_keep', 2)]:
        pool = [r for r in candidates if tag in r['attention']
                and r['filename'] not in selected_names]
        indices = np.linspace(0, len(pool)-1, min(limit, len(pool)), dtype=int)
        for i in indices:
            r = dict(pool[i], selection_reason=tag)
            selected.append(r)
            selected_names.add(r['filename'])
    for video in sorted({r['source_video'] for r in candidates}):
        pool = [r for r in candidates if r['source_video'] == video
                and r['filename'] not in selected_names]
        if pool and len(selected) < 12:
            r = dict(pool[len(pool)//2], selection_reason='additional source video')
            selected.append(r)
            selected_names.add(r['filename'])
    write_json(out / 'qualitative_selection.json', dict(
        note='Selected by existing visual tags BEFORE inference; no ground truth, no metrics. '
             'Videos excluded from train and val, but same lab scene.', images=selected))
    cases = [dict(name=r['filename'], source=str(root / 'images/val' / r['filename']),
                  kind='val', reason='held-out source video') for r in version['records']
             if r['split'] == 'val']
    cases += [dict(
        name=r['filename'], source=str(source / 'images' / r['filename']),
        kind='unlabelled', reason=r['selection_reason']) for r in selected]
    model = YOLO(a.weights)
    kw = dict(imgsz=640, device=0, conf=.25, verbose=False)
    for _ in range(5):
        model.predict(cases[0]['source'], **kw)
    timings, boards = [], {'val': [], 'unlabelled': []}
    for i, c in enumerate(cases):
        original = cv2.imread(c['source'])
        torch.cuda.synchronize()
        start = time.perf_counter()
        result = model.predict(c['source'], **kw)[0]
        torch.cuda.synchronize()
        wall = 1000*(time.perf_counter()-start)
        detections = [dict(xyxy=b.xyxy[0].tolist(), confidence=float(b.conf[0]),
                           class_id=int(b.cls[0])) for b in result.boxes]
        c.update(detections=detections,
                 image_sha256=sha256(c['source']), wall_ms=wall, speed_ms=result.speed)
        if c['kind'] == 'val':
            h, w = original.shape[:2]
            label = root / 'labels/val' / Path(c['name']).with_suffix('.txt')
            for _, x, y, bw, bh in validate_label(label, 1):
                cv2.rectangle(original, (round((x-bw/2)*w), round((y-bh/2)*h)),
                              (round((x+bw/2)*w), round((y+bh/2)*h)), (0, 255, 0), 2)
        prediction = result.plot()
        dest = out / c['kind']
        dest.mkdir(exist_ok=True)
        cv2.imwrite(str(dest / c['name']), prediction)
        tile = np.zeros((274, 640, 3), np.uint8)
        tile[:240] = cv2.hconcat([cv2.resize(original, (320, 240)),
                                 cv2.resize(prediction, (320, 240))])
        cv2.putText(tile, f'{i+1:02d} {c["kind"]} | {c["reason"]}', (4, 255),
                    cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
        cv2.putText(tile, 'Left: GT (val only) / original | Right: prediction conf>=0.25',
                    (4, 270), cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 255), 1)
        boards[c['kind']].append(tile)
        timings.append(dict(case=i+1, wall_ms=wall, **result.speed))
    for kind, tiles in boards.items():
        cv2.imwrite(str(out / f'{kind}_comparison.jpg'), cv2.vconcat(tiles))
    # Repeat preselected cases for latency only; no repeated observations count as new images.
    for repeat in range(4):
        for c in cases:
            torch.cuda.synchronize()
            start = time.perf_counter()
            result = model.predict(c['source'], **kw)[0]
            torch.cuda.synchronize()
            timings.append(dict(repeat=repeat+1, wall_ms=1000*(time.perf_counter()-start),
                                **result.speed))
    write_json(out / 'cases.json', cases)
    write_json(out / 'timings.json', timings)

    def summary(key):
        v = [t[key] for t in timings]
        return dict(mean=float(np.mean(v)), p50=float(np.median(v)),
                    p95=float(np.percentile(v, 95)))
    write_json(out / 'summary.json', dict(
        weights_sha256=sha256(a.weights), conf=.25, imgsz=640, device=0,
        validation_images=sum(c['kind'] == 'val' for c in cases),
        qualitative_images=len(selected), timing_samples=len(timings), warmup=5,
        gpu_inference_ms=summary('inference'), synchronized_predict_wall_ms=summary('wall_ms'),
        scope='batch1 FP32 .pt, file read/preprocessing/NMS in wall time; inference from '
              'Ultralytics synchronized GPU profile; same cases repeated 5 times; not ROS latency'))
    print((out / 'summary.json').read_text())


if __name__ == '__main__':
    main()
