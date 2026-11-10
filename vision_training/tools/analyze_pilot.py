#!/usr/bin/env python3
"""Read-only diagnosis of a single-positive pilot checkpoint; never train or relabel."""
import argparse
import csv
import itertools
import json
from pathlib import Path
import sys

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.data.augment import LetterBox
from ultralytics.utils import ops
import yaml

matplotlib.use('Agg')
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import environment, new_output, sha256, write_json  # noqa: E402
from vision_training.dataset import inspect_dataset, validate_label  # noqa: E402

THRESHOLDS = (.01, .05, .10, .25)


def ious(boxes, gt):
    boxes = np.asarray(boxes, dtype=float).reshape(-1, 4)
    intersection = np.maximum(0, np.minimum(boxes[:, 2:], gt[2:]) -
                              np.maximum(boxes[:, :2], gt[:2])).prod(axis=1)
    area = np.maximum(0, boxes[:, 2:]-boxes[:, :2]).prod(axis=1)
    return intersection / np.maximum(area + np.prod(gt[2:]-gt[:2]) - intersection, 1e-12)


def matched_counts(boxes, confidence, classes, gt, threshold):
    """One GT: one class-correct IoU>=.5 TP at most; duplicates remain FP."""
    overlap = ious(boxes, gt)
    keep = np.asarray(confidence) >= threshold
    correct = keep & (np.asarray(classes) == 0) & (overlap >= .5)
    tp = int(correct.any())
    return dict(predictions=int(keep.sum()), tp=tp, fp=int(keep.sum())-tp, fn=1-tp,
                wrong_class=int((keep & (np.asarray(classes) != 0)).sum()),
                background_iou_below_01=int((keep & (overlap < .1)).sum()),
                localization_iou_01_to_05=int((keep & (overlap >= .1) & (overlap < .5)).sum()),
                best_iou=float(overlap[keep].max()) if keep.any() else None,
                matched_confidence=float(np.asarray(confidence)[correct].max()) if tp else None)


def raw_predictions(model, image, square=False):
    """Inspect all decoded candidates BEFORE confidence filtering/NMS."""
    padded = LetterBox(new_shape=(640, 640), auto=False)(image=image) if square else image
    tensor = model.predictor.preprocess([padded])
    with torch.inference_mode():
        output = model.predictor.model(tensor)
    output = output[0] if isinstance(output, (tuple, list)) else output
    assert output.shape[1] == 5, 'This audit requires exactly one class'
    prediction = output[0].T
    boxes = ops.xywh2xyxy(prediction[:, :4])
    boxes = ops.scale_boxes(tensor.shape[2:], boxes, image.shape).cpu().numpy()
    return boxes, prediction[:, 4].cpu().numpy(), list(tensor.shape)


def draw_box(image, box, color, thickness=1):
    x1, y1, x2, y2 = np.round(box).astype(int)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)


def run(a):
    out = new_output(a.output)
    data, manifest = inspect_dataset(a.data, splits=('train', 'val'), require_review=True)
    version = json.loads((Path(a.data).parent / 'version.json').read_text())
    mapping = {r['filename']: r for r in version['records']}
    models = {'best': YOLO(a.weights)}
    if a.last:
        models['last'] = YOLO(a.last)
    for model in models.values():
        assert model.names == {0: 'blue_cube'}
    rows, reviews, prediction_tiles, fingerprints = [], [], [], []
    for index, entry in enumerate(manifest['entries'], 1):
        path = Path(entry['image'])
        image = cv2.imread(str(path))
        h, w = image.shape[:2]
        labels = validate_label(entry['label'], 1)
        assert len(labels) == 1
        _, x, y, bw, bh = labels[0]
        gt = np.array([(x-bw/2)*w, (y-bh/2)*h, (x+bw/2)*w, (y+bh/2)*h])
        gain = min(640/w, 640/h)
        row = dict(index=index, split=entry['split'], filename=path.name,
                   source_video=mapping[path.name]['source_video'],
                   source_seconds=mapping[path.name]['source_seconds'],
                   gt_xyxy=gt.tolist(), width_640=bw*w*gain, height_640=bh*h*gain,
                   area_640=bw*w*gain*bh*h*gain, center_normalized=[x, y],
                   top_clipped=bool(gt[1] <= 1), models={})
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        fingerprints.append((path.name, cv2.resize(gray, (64, 48)).astype(float)))
        for key, model in models.items():
            per_threshold = {}
            for threshold in THRESHOLDS:
                result = model.predict(image, imgsz=640, device=a.device, conf=threshold,
                                       iou=.7, max_det=300, rect=True, verbose=False)[0]
                b = result.boxes
                boxes, confidence, classes = (b.xyxy.cpu().numpy(), b.conf.cpu().numpy(),
                                              b.cls.cpu().numpy())
                counts = matched_counts(boxes, confidence, classes, gt, threshold)
                detections = zip(boxes, confidence, classes, ious(boxes, gt))
                counts['boxes'] = [dict(
                    xyxy=box.tolist(), confidence=float(c), class_id=int(cls), iou=float(iou))
                    for box, c, cls, iou in detections]
                per_threshold[str(threshold)] = counts
            boxes, confidence, shape = raw_predictions(model, image)
            overlap = ious(boxes, gt)
            top = int(confidence.argmax())
            best_iou = int(overlap.argmax())
            target = overlap >= .5
            diagnostic = dict(
                thresholds=per_threshold, input_shape=shape, candidates=len(boxes),
                max_confidence=float(confidence[top]), top_box_xyxy=boxes[top].tolist(),
                top_box_iou=float(overlap[top]), max_raw_iou=float(overlap[best_iou]),
                max_iou_box_confidence=float(confidence[best_iou]),
                max_confidence_iou_ge_05=float(confidence[target].max()) if target.any() else None)
            # Shape-only control, no augmentation/weight updates; square input matches training.
            square_boxes, square_conf, square_shape = raw_predictions(model, image, square=True)
            assert square_shape[-2:] == [640, 640]
            diagnostic['square_control'] = dict(
                input_shape=square_shape, max_confidence=float(square_conf.max()),
                raw_matching_candidate_at_001=matched_counts(
                    square_boxes, square_conf, np.zeros(len(square_conf)), gt, .01)['tp'])
            row['models'][key] = diagnostic
        annotated = image.copy()
        draw_box(annotated, gt, (0, 255, 0), 2)
        margin = max(gt[2]-gt[0], gt[3]-gt[1])*.6 + 8
        x1, y1 = np.maximum(0, np.floor(gt[:2]-margin)).astype(int)
        x2, y2 = np.minimum([w, h], np.ceil(gt[2:]+margin)).astype(int)
        zoom = cv2.resize(annotated[y1:y2, x1:x2], (320, 240))
        tile = np.zeros((278, 640, 3), np.uint8)
        tile[:240] = cv2.hconcat([cv2.resize(annotated, (320, 240)), zoom])
        cv2.putText(tile, f'{index:02d} {entry["split"]} {path.stem[-8:]} | '
                    f'GT {row["width_640"]:.1f}x{row["height_640"]:.1f}px',
                    (4, 257), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
        cv2.putText(tile, 'GT green; right is enlarged crop (resized for visibility)',
                    (4, 274), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
        reviews.append(tile)
        diagnostic = row['models']['best']
        predicted = annotated.copy()
        draw_box(predicted, diagnostic['top_box_xyxy'], (0, 0, 255), 2)
        tile = np.zeros((518, 640, 3), np.uint8)
        tile[:480] = cv2.resize(predicted, (640, 480))
        cv2.putText(tile, f'{index:02d} {entry["split"]} raw maxconf='
                    f'{diagnostic["max_confidence"]:.6f} IoU={diagnostic["top_box_iou"]:.3f}',
                    (4, 497), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
        cv2.putText(tile, 'Green GT / red highest-score RAW candidate, NOT accepted detection',
                    (4, 514), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
        prediction_tiles.append(tile)
        rows.append(row)
    for name, tiles in [('label_review', reviews), ('raw_top_candidate', prediction_tiles)]:
        for start in range(0, len(tiles), 5):
            cv2.imwrite(str(out / f'{name}_{start//5+1}.jpg'), cv2.vconcat(tiles[start:start+5]))
    pairs = []
    for (name1, im1), (name2, im2) in itertools.combinations(fingerprints, 2):
        pairs.append(dict(image_a=name1, image_b=name2,
                          gray64x48_mae=float(np.abs(im1-im2).mean())))
    pairs.sort(key=lambda p: p['gray64x48_mae'])
    train_rows = list(csv.DictReader(Path(a.results).open()))
    summary = dict(environment=environment(), dataset_fingerprint=manifest['fingerprint'],
                   checkpoint_sha256={k: sha256(v) for k, v in
                                      [('best', a.weights), ('last', a.last)] if v},
                   matching='One class-0 ground truth per image; class-correct IoU>=0.5; '
                            'one TP maximum, duplicate predictions are FP; NMS IoU=.7 max_det=300',
                   per_split={}, loss_rows=[{k: float(v) for k, v in r.items()}
                                            for r in train_rows],
                   closest_image_pairs=pairs[:15], images=rows)
    for split in ('train', 'val'):
        group = [r for r in rows if r['split'] == split]
        stats = dict(images=len(group), models={})
        for key in models:
            stats['models'][key] = {}
            for t in THRESHOLDS:
                counts = [r['models'][key]['thresholds'][str(t)] for r in group]
                stats['models'][key][str(t)] = {
                    k: sum(c[k] for c in counts) for k in
                    ('predictions', 'tp', 'fp', 'fn', 'wrong_class', 'background_iou_below_01',
                     'localization_iou_01_to_05')}
        for key in ('width_640', 'height_640', 'area_640'):
            values = [r[key] for r in group]
            stats[key] = dict(min=min(values), median=float(np.median(values)), max=max(values))
        stats['top_clipped'] = sum(r['top_clipped'] for r in group)
        stats['area_below_32_squared'] = sum(r['area_640'] < 32**2 for r in group)
        summary['per_split'][split] = stats
    write_json(out / 'analysis.json', summary)
    (out / 'analysis.yaml').write_text(yaml.safe_dump(summary, sort_keys=False))
    # Charts contain measurements only, no source images.
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    epoch = [r['epoch'] for r in summary['loss_rows']]
    for j, loss in enumerate(('box_loss', 'cls_loss', 'dfl_loss')):
        for split in ('train', 'val'):
            axes[0, j].plot(epoch, [r[f'{split}/{loss}'] for r in summary['loss_rows']],
                            label=split)
        axes[0, j].axvline(5, color='gray', linestyle=':')
        axes[0, j].set_title(loss)
        axes[0, j].legend()
    for key in ('metrics/mAP50(B)', 'metrics/mAP50-95(B)'):
        axes[1, 0].plot(epoch, [r[key] for r in summary['loss_rows']], label=key)
    axes[1, 0].legend(fontsize=7)
    axes[1, 0].set_title('Validation AP')
    for split, color in [('train', 'tab:blue'), ('val', 'tab:orange')]:
        group = [r for r in rows if r['split'] == split]
        axes[1, 1].scatter([r['width_640'] for r in group], [r['height_640'] for r in group],
                           color=color, label=split)
        axes[1, 2].scatter([r['center_normalized'][0] for r in group],
                           [r['center_normalized'][1] for r in group], color=color, label=split)
    axes[1, 1].set(xlabel='Width at imgsz640 (px)', ylabel='Height (px)', title='GT size')
    axes[1, 2].set(xlabel='Normalized x', ylabel='Normalized y', title='GT center position')
    axes[1, 2].invert_yaxis()
    axes[1, 1].legend()
    axes[1, 2].legend()
    fig.tight_layout()
    fig.savefig(out / 'loss_size_position.png', dpi=140)
    plt.close(fig)
    assert inspect_dataset(a.data, splits=('train', 'val'), require_review=True)[1][
        'fingerprint'] == manifest['fingerprint']
    print(yaml.safe_dump(summary['per_split'], sort_keys=False))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights', required=True)
    p.add_argument('--last', help='optional saved last.pt comparison; no training')
    p.add_argument('--data', required=True)
    p.add_argument('--results', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--device', default='0')
    run(p.parse_args())
