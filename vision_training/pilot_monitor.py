"""Optional single-positive pilot audit via public hooks; no optimizer/library changes."""
from copy import deepcopy
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import torch
from ultralytics.models.yolo.detect.predict import DetectionPredictor
import ultralytics

try:
    from .common import sha256, write_json
    from .dataset import validate_label
    from .tools.analyze_pilot import matched_counts
except ImportError:
    from common import sha256, write_json
    from dataset import validate_label
    from tools.analyze_pilot import matched_counts


def attach_monitor(model, manifest, output, interval, arguments):
    """Audit the saved-model EMA at fixed epochs, preserving training model and RNG."""
    output = Path(output)
    state = dict(optimizer_step_calls=0, batches=0, monitor_seconds=0.0)
    seen = set()
    entries = []
    for entry in manifest['entries']:
        labels = validate_label(entry['label'], 1)
        if len(labels) != 1:
            raise ValueError('Pilot monitor requires exactly one class-0 GT per image')
        image = cv2.imread(entry['image'])
        h, w = image.shape[:2]
        _, x, y, bw, bh = labels[0]
        gt = np.array([(x-bw/2)*w, (y-bh/2)*h, (x+bw/2)*w, (y+bh/2)*h])
        entries.append((entry, gt))

    def stepped(optimizer, args, kwargs):
        state['optimizer_step_calls'] += 1

    def setup(trainer):
        for key in ('nbs', 'warmup_epochs', 'patience'):
            if key in arguments and getattr(trainer.args, key) != arguments[key]:
                raise RuntimeError(f'Requested {key} was not applied')
        handle = trainer.optimizer.register_step_post_hook(stepped)
        state['hook'] = handle
        nb = len(trainer.train_loader)
        nw = (max(round(trainer.args.warmup_epochs * nb), 100)
              if trainer.args.warmup_epochs > 0 else -1)
        source = Path(ultralytics.__file__).parent / 'engine/trainer.py'
        write_json(output / 'runtime_setup.json', dict(
            ultralytics=ultralytics.__version__, trainer_source_sha256=sha256(source),
            nbs=trainer.args.nbs, batch=trainer.batch_size, patience=trainer.args.patience,
            warmup_epochs=trainer.args.warmup_epochs, warmup_batch_boundary=nw,
            batches_per_epoch=nb, initial_accumulate=trainer.accumulate,
            optimizer=type(trainer.optimizer).__name__,
            optimizer_groups=[dict(lr=g['lr'], weight_decay=g['weight_decay'])
                              for g in trainer.optimizer.param_groups],
            step_counter='torch optimizer post-step hook; AMP skipped steps do not call hook; '
                         'a called step with lr=0 is not an effective parameter update'))

    def batch_end(trainer):
        ni = state['batches']
        state['batches'] += 1
        if trainer.args.nbs == trainer.batch_size and trainer.accumulate != 1:
            raise RuntimeError('nbs=batch but runtime gradient accumulation is not one')
        record = dict(epoch=trainer.epoch+1, batch_index=ni,
                      accumulate=trainer.accumulate, nbs=trainer.args.nbs,
                      optimizer_step_calls=state['optimizer_step_calls'],
                      lr=[g['lr'] for g in trainer.optimizer.param_groups],
                      amp_scale=float(trainer.scaler.get_scale()))
        with (output / 'batch_audit.jsonl').open('a') as stream:
            stream.write(json.dumps(record)+'\n')

    def epoch_end(trainer):
        epoch = trainer.epoch+1
        if epoch in seen or (epoch != 1 and epoch % interval and not trainer.stop):
            return
        seen.add(epoch)  # final_eval invokes this callback again; don't count an epoch twice.
        started = time.perf_counter()
        numpy_state, python_state = np.random.get_state(), random.getstate()
        cuda_devices = [trainer.device.index] if trainer.device.type == 'cuda' else []
        try:
            with torch.random.fork_rng(devices=cuda_devices):
                predictor = DetectionPredictor(overrides=dict(
                    imgsz=arguments['imgsz'], device=arguments['device'], batch=1,
                    conf=.05, iou=.7, max_det=300, half=False, verbose=False,
                    save=False, project=str(output), name='monitor_inference', exist_ok=True))
                # AutoBackend fuses its model. Always give it an isolated EMA copy.
                predictor.setup_model(model=deepcopy(trainer.ema.ema).float(), verbose=False)
                by_split = {s: {str(t): dict(tp=0, fp=0, fn=0) for t in (.05, .25)}
                            for s in ('train', 'val')}
                cases = []
                for entry, gt in entries:
                    result = predictor(source=entry['image'])[0]
                    boxes = result.boxes
                    scores = {}
                    for threshold in (.05, .25):
                        count = matched_counts(boxes.xyxy.cpu().numpy(), boxes.conf.cpu().numpy(),
                                               boxes.cls.cpu().numpy(), gt, threshold)
                        scores[str(threshold)] = count
                        for k in ('tp', 'fp', 'fn'):
                            by_split[entry['split']][str(threshold)][k] += count[k]
                    cases.append(dict(image=Path(entry['image']).name, split=entry['split'],
                                      thresholds=scores))
                del predictor
        finally:
            np.random.set_state(numpy_state)
            random.setstate(python_state)
        seconds = time.perf_counter()-started
        state['monitor_seconds'] += seconds
        record = dict(epoch=epoch, model='isolated EMA copy, FP32 eval; no augmentation',
                      counts=by_split, cases=cases, batches=state['batches'],
                      optimizer_step_calls=state['optimizer_step_calls'],
                      monitor_seconds=seconds, cumulative_monitor_seconds=state['monitor_seconds'])
        with (output / 'fit_progress.jsonl').open('a') as stream:
            stream.write(json.dumps(record)+'\n')
        print(f'PILOT_MONITOR epoch={epoch} steps={state["optimizer_step_calls"]} '
              f'counts={json.dumps(by_split)}', flush=True)

    def end(trainer):
        state.pop('hook').remove()
        write_json(output / 'monitor_result.json', state)

    model.add_callback('on_pretrain_routine_end', setup)
    model.add_callback('on_train_batch_end', batch_end)
    model.add_callback('on_fit_epoch_end', epoch_end)
    model.add_callback('on_train_end', end)
