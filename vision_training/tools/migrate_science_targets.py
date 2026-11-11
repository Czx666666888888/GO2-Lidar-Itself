#!/usr/bin/env python3
"""Copy hash-verified historical blue_cube labels to a new three-class pool."""
import argparse
from pathlib import Path
import shutil
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import read_yaml, sha256  # noqa: E402
from vision_training.dataset import validate_label  # noqa: E402
from vision_training.tools.dataset import add_pool  # noqa: E402


def migrate(source, target):
    source, target = Path(source).resolve(), Path(target).resolve()
    version = read_yaml(source / 'version.yaml')
    if read_yaml(source / 'data.yaml')['names'] != {0: 'blue_cube'}:
        raise ValueError('Expected historical blue_cube ID 0')
    if read_yaml(target / 'data.yaml')['names'] != {
            0: 'color_block', 1: 'rock_compact', 2: 'rock_spiky'}:
        raise ValueError('Expected official three-class mapping')
    group = 'legacy_lab_session_20261108'
    if (target / 'pool/images' / group).exists():
        raise ValueError('Migration already exists; do not overwrite')
    records = version['records']
    if len(records) != 20:
        raise ValueError('Expected the 20 human-labelled historical images')
    for r in records:
        for kind, key, suffix in [('images', 'image_sha256', '.jpg'),
                                  ('labels', 'label_sha256', '.txt')]:
            p = source / kind / r['split'] / Path(r['filename']).with_suffix(suffix)
            if sha256(p) != r[key]:
                raise ValueError(f'Historical hash mismatch: {p}')
        validate_label(source / 'labels' / r['split'] / Path(r['filename']).with_suffix('.txt'), 1)
    rows = []
    for r in records:
        image = target / 'pool/images' / group / r['filename']
        label = target / 'pool/labels' / group / Path(r['filename']).with_suffix('.txt')
        image.parent.mkdir(parents=True, exist_ok=True)
        label.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / 'images' / r['split'] / r['filename'], image)
        shutil.copy2(source / 'labels' / r['split'] / label.name, label)
        rows.append(dict(image=str(image.relative_to(target)), group=group,
                         scene='historical_lab_floor', source=r['source_video'],
                         frame_index=r['source_frame_index'], source_seconds=r['source_seconds'],
                         tags=['legacy_blue_cube'], reviewed=False,
                         annotation_status='historical_positive_needs_three_class_review',
                         image_sha256=sha256(image), label_sha256=sha256(label),
                         source_dataset='blue_cube_pilot_v1', source_split=r['split'],
                         source_label_sha256=r['label_sha256'],
                         class_mapping={0: dict(source='blue_cube', target='color_block',
                                                target_id=0)}))
    add_pool(target, rows)
    report = dict(source_version_sha256=sha256(source / 'version.yaml'),
                  original_archive_sha256=version['archive_sha256'],
                  images=len(rows), class_mapping={0: 'blue_cube -> color_block (0)'},
                  group=group, reviewed=False, records=rows)
    (target / 'legacy_mapping.yaml').write_text(yaml.safe_dump(report, allow_unicode=True))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='vision_training/datasets/blue_cube_pilot_v1')
    parser.add_argument('--target', default='vision_training/datasets/science_targets_v1')
    args = parser.parse_args()
    print('Migrated historical images:', migrate(args.source, args.target)['images'])
