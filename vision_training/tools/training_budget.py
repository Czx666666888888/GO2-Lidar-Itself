#!/usr/bin/env python3
"""Inspect reviewed single-GPU data and the installed training schedule; never train."""
import argparse
from collections import Counter
from importlib.metadata import distribution
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.dataset import validate_label  # noqa: E402
from vision_training.train import training_plan  # noqa: E402
from vision_training.common import sha256  # noqa: E402


def budget(config):
    _, data, manifest, _, _, args = training_plan(config)
    dist = distribution('ultralytics')
    source = Path(dist.locate_file('ultralytics/engine/trainer.py'))
    text = source.read_text()
    expected = ('max(round(self.args.nbs / self.batch_size), 1)',
                'max(round(self.args.warmup_epochs * nb), 100)',
                'if self.args.warmup_epochs > 0 else -1')
    if dist.version != '8.3.40' or any(s not in text for s in expected):
        raise ValueError('Recheck schedule against this installed Ultralytics version')
    counts = Counter(e['split'] for e in manifest['entries'])
    classes = {s: Counter(int(b[0]) for e in manifest['entries'] if e['split'] == s
                          for b in validate_label(e['label'], len(data['names'])))
               for s in ('train', 'val', 'test')}
    batch, nbs = args['batch'], args.get('nbs', 64)
    nb = math.ceil(counts['train']/min(batch, counts['train']))
    warmup = args.get('warmup_epochs', 3.0)
    nw = max(round(warmup*nb), 100) if warmup > 0 else -1
    return dict(ultralytics=dist.version, trainer_sha256=sha256(source),
                images=counts, boxes_by_class=classes, configured_batch=batch, nbs=nbs,
                planned_batches_per_epoch=nb, initial_accumulate=max(round(nbs/batch), 1),
                warmup_batch_boundary=nw, maximum_batches=nb*args['epochs'],
                warmup_covers_maximum_training=nw >= nb*args['epochs'],
                missing_classes={s: sorted(set(data['names'])-set(c)) for s, c in classes.items()},
                runtime_batch_count='NOT VERIFIED: training not started; '
                                    'early stopping may shorten run',
                note='Warmup changes accumulation from 1 toward nbs/batch. AMP may skip steps. '
                     'Reassess nbs, actual loader length and memory before training.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    print(json.dumps(budget(args.config), indent=2))
