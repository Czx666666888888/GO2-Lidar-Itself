#!/usr/bin/env python3
"""Fine-tune a configurable detector after explicit human dataset review."""

import argparse
import json
from pathlib import Path

try:
    from .common import new_output, read_yaml, resolve, sha256, snapshot, write_json
    from .dataset import inspect_dataset
except ImportError:
    from common import new_output, read_yaml, resolve, sha256, snapshot, write_json
    from dataset import inspect_dataset


def training_plan(config_path, pilot=False):
    path = Path(config_path).resolve()
    cfg = read_yaml(path)
    required = ('project', 'data', 'model', 'output', 'name', 'imgsz', 'epochs',
                'batch', 'device', 'patience')
    for key in required:
        if key not in cfg:
            raise ValueError(f'Missing training config: {key}')
    for key in ('project', 'name'):
        if not str(cfg[key]) or Path(str(cfg[key])).name != str(cfg[key]):
            raise ValueError(f'{key} must be a single directory name')
    if cfg['imgsz'] <= 0 or cfg['epochs'] <= 0 or cfg['batch'] <= 0:
        raise ValueError('imgsz, epochs, batch must be positive')
    splits = ('train', 'val') if pilot else ('train', 'val', 'test')
    data, manifest = inspect_dataset(resolve(path.parent, cfg['data']), splits=splits,
                                     require_review=True)
    if pilot and 'test' in data:
        raise ValueError('Pilot must not declare a test split')
    cfg['validation_protocol'] = 'pilot_train_val_only' if pilot else 'formal_train_val_test'
    output = resolve(path.parent, cfg['output']) / cfg['project'] / cfg['name']
    spec = str(cfg['model'])
    model_path = (Path(__file__).resolve().parent / 'weights' / spec
                  if Path(spec).name == spec else resolve(path.parent, spec))
    if model_path.suffix != '.pt':
        raise ValueError('Fine-tuning requires a pretrained detection .pt checkpoint')
    args = {k: cfg[k] for k in ('imgsz', 'epochs', 'batch', 'device', 'patience')}
    args.update(seed=cfg.get('seed', 42), workers=cfg.get('workers', 4),
                deterministic=True, pretrained=True, plots=True, save=True,
                amp=cfg.get('amp', True))
    # Flat standard Ultralytics training augmentation options, no custom framework.
    args.update(cfg.get('augmentation', {}))
    augmentation_keys = {'hsv_h', 'hsv_s', 'hsv_v', 'degrees', 'translate', 'scale',
                         'shear', 'perspective', 'flipud', 'fliplr', 'bgr', 'mosaic',
                         'mixup', 'copy_paste', 'close_mosaic'}
    if set(cfg.get('augmentation', {}))-augmentation_keys:
        raise ValueError('Unsupported augmentation key or attempted training-argument override')
    return cfg, data, manifest, output, model_path, args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--dry-run', action='store_true', help='validate without loading torch')
    parser.add_argument('--pilot', action='store_true', help='explicit small-sample train/val only')
    opts = parser.parse_args()
    cfg, data, manifest, output, model_path, args = training_plan(opts.config, pilot=opts.pilot)
    if opts.dry_run:
        print(json.dumps(dict(output=str(output), model=str(model_path), names=data['names'],
                              images=len(manifest['entries']), arguments=args), indent=2))
        return
    from ultralytics import YOLO
    output = new_output(output)
    model_path.parent.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(model_path))  # Official asset download if absent, or local checkpoint.
    if model.task != 'detect':
        raise ValueError('Only object detection models are supported in this workflow')
    snapshot(output, dict(cfg, resolved_model=str(model_path), training_args=args), data, manifest)
    write_json(output / 'pretrained.json', dict(path=str(model_path), sha256=sha256(model_path)))
    metrics = model.train(data=str(output / 'data.yaml'), project=str(output.parent),
                          name=output.name, exist_ok=True, **args)
    best = output / 'weights/best.pt'
    last = output / 'weights/last.pt'
    if not best.is_file() or not last.is_file():
        raise RuntimeError('Training did not produce both best.pt and last.pt')
    values = {k: float(v) for k, v in metrics.results_dict.items()}
    write_json(output / 'training_result.json', dict(
        metrics=values, dataset_fingerprint=manifest['fingerprint'], names=data['names'],
        weights={p.name: sha256(p) for p in (best, last)}))
    print(f'Training artifacts: {output}')


if __name__ == '__main__':
    main()
