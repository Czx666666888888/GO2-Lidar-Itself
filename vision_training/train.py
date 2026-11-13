#!/usr/bin/env python3
"""Fine-tune a configurable detector after explicit human dataset review."""

import argparse
import json
import math
from pathlib import Path
import time

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
    if (isinstance(cfg['patience'], bool) or not isinstance(cfg['patience'], int) or
            cfg['patience'] < 0):
        raise ValueError('patience must be a nonnegative integer')
    if 'nbs' in cfg and (isinstance(cfg['nbs'], bool) or
                         not isinstance(cfg['nbs'], int) or cfg['nbs'] <= 0):
        raise ValueError('nbs must be a positive integer')
    if 'warmup_epochs' in cfg and (isinstance(cfg['warmup_epochs'], bool) or
                                   not isinstance(cfg['warmup_epochs'], (int, float)) or
                                   not math.isfinite(cfg['warmup_epochs']) or
                                   cfg['warmup_epochs'] < 0):
        raise ValueError('warmup_epochs must be finite and nonnegative')
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
    args.update({k: cfg[k] for k in ('nbs', 'warmup_epochs') if k in cfg})
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
    parser.add_argument('--monitor-every', type=int, default=0,
                        help='opt-in pilot-only EMA inference/optimizer audit every N epochs')
    opts = parser.parse_args()
    if opts.monitor_every < 0 or (opts.monitor_every and not opts.pilot):
        parser.error('--monitor-every requires --pilot and a nonnegative interval')
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
    if opts.monitor_every:
        try:
            from .pilot_monitor import attach_monitor
        except ImportError:
            from pilot_monitor import attach_monitor
        attach_monitor(model, manifest, output, opts.monitor_every, args)
    if cfg.get('runtime_audit', False):
        try:
            from .runtime_audit import attach_runtime_audit
        except ImportError:
            from runtime_audit import attach_runtime_audit
        attach_runtime_audit(model, output)
    started = time.perf_counter()
    metrics = model.train(data=str(output / 'data.yaml'), project=str(output.parent),
                          name=output.name, exist_ok=True, **args)
    train_call_seconds = time.perf_counter() - started
    best = output / 'weights/best.pt'
    last = output / 'weights/last.pt'
    if not best.is_file() or not last.is_file():
        raise RuntimeError('Training did not produce both best.pt and last.pt')
    values = {k: float(v) for k, v in metrics.results_dict.items()}
    write_json(output / 'training_result.json', dict(
        metrics=values, dataset_fingerprint=manifest['fingerprint'], names=data['names'],
        train_call_seconds=train_call_seconds, monitor_every=opts.monitor_every,
        weights={p.name: sha256(p) for p in (best, last)}))
    print(f'Training artifacts: {output}')


if __name__ == '__main__':
    main()
