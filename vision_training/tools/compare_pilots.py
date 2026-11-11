#!/usr/bin/env python3
"""Compare existing single-positive pilot analyses; render actual GT-matched detections."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import new_output, write_json  # noqa: E402
from vision_training.tools.analyze_pilot import draw_box, plot_measurements  # noqa: E402


def compare(a):
    v1, v2 = [json.loads(Path(p).read_text()) for p in (a.v1_analysis, a.v2_analysis)]
    if v1['dataset_fingerprint'] != v2['dataset_fingerprint']:
        raise ValueError('Comparison requires identical data, labels and split fingerprint')
    out = new_output(a.output)
    run = Path(a.v2_run)
    progress = [json.loads(s) for s in (run / 'fit_progress.jsonl').read_text().splitlines()]
    batches = [json.loads(s) for s in (run / 'batch_audit.jsonl').read_text().splitlines()]
    snapshots = []
    for r in progress:
        snapshots.append({k: r[k] for k in ('epoch', 'counts', 'batches', 'optimizer_step_calls')})
    result = dict(
        dataset_fingerprint=v2['dataset_fingerprint'], checkpoint_sha256={
            'v1': v1['checkpoint_sha256'], 'v2': v2['checkpoint_sha256']},
        matching=v2['matching'], v1=v1['per_split'], v2=v2['per_split'], snapshots=snapshots,
        runtime_setup=json.loads((run / 'runtime_setup.json').read_text()),
        monitoring=json.loads((run / 'monitor_result.json').read_text()),
        train_result=json.loads((run / 'training_result.json').read_text()),
        best_checkpoint=json.loads((run / 'checkpoint_metadata.json').read_text()),
        nonfinite_csv_events=v2['nonfinite_csv_events'],
        runtime_accumulate_values=sorted({r['accumulate'] for r in batches}),
        amp_skipped_batch_indices=[r['batch_index'] for i, r in enumerate(batches)
                                   if r['optimizer_step_calls'] == (
                                       batches[i-1]['optimizer_step_calls'] if i else 0)],
        per_image=[])
    boards = {str(t): [] for t in (.05, .25)}
    previous = {r['filename']: r for r in v1['images']}
    for r in v2['images']:
        old = previous[r['filename']]
        if old['split'] != r['split'] or not np.allclose(old['gt_xyxy'], r['gt_xyxy']):
            raise ValueError('GT or split changed')
        image = cv2.imread(str(Path(a.dataset) / 'images' / r['split'] / r['filename']))
        details = dict(filename=r['filename'], split=r['split'], models={})
        for key, source, model in [('v1_best', old, 'best'), ('v2_best', r, 'best'),
                                   ('v2_last', r, 'last')]:
            details['models'][key] = {
                t: {k: z[k] for k in ('tp', 'fp', 'fn', 'best_iou', 'matched_confidence')}
                for t, z in source['models'][model]['thresholds'].items() if t in boards}
        result['per_image'].append(details)
        for threshold in boards:
            tiles = []
            for key, source, model in [('v1_best', old, 'best'), ('v2_best', r, 'best'),
                                       ('v2_last', r, 'last')]:
                annotated = image.copy()
                draw_box(annotated, r['gt_xyxy'], (0, 255, 0), 2)
                d = source['models'][model]['thresholds'][threshold]
                for box in d['boxes']:
                    color = (255, 255, 0) if box['iou'] >= .5 else (0, 0, 255)
                    draw_box(annotated, box['xyxy'], color, 1)
                    x, y = np.round(box['xyxy'][:2]).astype(int)
                    cv2.putText(annotated, f'{box["confidence"]:.2f}', (max(x, 0), max(y, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, .35, color, 1)
                folder = out / key / f'conf_{threshold}' / r['split']
                folder.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(folder / r['filename']), annotated)
                tile = np.zeros((266, 320, 3), np.uint8)
                tile[:240] = cv2.resize(annotated, (320, 240))
                cv2.putText(tile, f'{key} TP{d["tp"]} FP{d["fp"]} FN{d["fn"]}', (3, 258),
                            cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
                tiles.append(tile)
            boards[threshold].append(cv2.hconcat(tiles))
    for threshold, tiles in boards.items():
        for start in range(0, len(tiles), 5):
            cv2.imwrite(str(out / f'conf_{threshold}_page_{start//5+1}.jpg'),
                        cv2.vconcat(tiles[start:start+5]))
        chosen = [0, 5, 13, 14, 16, 18]
        cv2.imwrite(str(out / f'conf_{threshold}_overview.jpg'),
                    cv2.vconcat([tiles[i] for i in chosen]))
    write_json(out / 'comparison.json', result)
    (out / 'comparison.yaml').write_text(yaml.safe_dump(result, sort_keys=False))
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    for label, analysis in [('V1', v1), ('V2', v2)]:
        for split, style in [('train', '-'), ('val', '--')]:
            axes[0].plot([r['epoch'] for r in analysis['loss_rows']],
                         [r[f'{split}/cls_loss'] for r in analysis['loss_rows']],
                         style, label=f'{label} {split}')
    for split, ax in zip(('train', 'val'), axes[1:]):
        for threshold in (.05, .25):
            ax.plot([r['epoch'] for r in progress],
                    [r['counts'][split][str(threshold)]['tp'] for r in progress],
                    marker='o', label=f'V2 TP conf={threshold}')
        ax.axhline(14 if split == 'train' else 6, color='gray', linestyle=':')
        ax.set_title(f'{split}: EMA GT-matched TP, IoU>=0.5')
        ax.set(xlabel='Epoch', ylabel='Matched targets')
    axes[0].set(xlabel='Epoch', ylabel='Classification loss', title='Raw CSV; NaN stays a gap')
    axes[0].axvspan(24, 27, color='gray', alpha=.1)
    for ax in axes:
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'learning_comparison.png', dpi=140)
    plt.close(fig)
    # Re-render the standalone chart from saved measurements, without further GPU work.
    plot_measurements(v2, Path(a.v2_analysis).parent)
    print(f'Comparison saved: {out}')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--v1-analysis', required=True)
    p.add_argument('--v2-analysis', required=True)
    p.add_argument('--v2-run', required=True)
    p.add_argument('--dataset', required=True)
    p.add_argument('--output', required=True)
    compare(p.parse_args())
