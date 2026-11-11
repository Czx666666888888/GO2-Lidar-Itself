#!/usr/bin/env python3
"""Import/extract, explicitly review, split by capture group, and check YOLO data."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import class_names, read_yaml, sha256, write_json  # noqa: E402
from vision_training.dataset import (  # noqa: E402
    images_in, inspect_dataset, label_for, validate_label,
)


def pool_manifest(root):
    path = root / 'pool/manifest.json'
    return json.loads(path.read_text()) if path.exists() else []


def safe_group(value):
    if not value or Path(value).name != value or value in ('.', '..'):
        raise ValueError('group/tag must be one nonempty directory name')
    return value


def add_pool(root, rows):
    existing = pool_manifest(root)
    if {r['image'] for r in rows}.intersection(r['image'] for r in existing):
        raise ValueError('Images already in pool; do not overwrite existing annotations')
    write_json(root / 'pool/manifest.json', existing + rows)


def extract(args):
    import cv2
    root, video = Path(args.dataset).resolve(), Path(args.video).resolve()
    safe_group(args.group)
    if args.interval <= 0:
        raise ValueError('interval must be positive seconds')
    target = root / 'pool/images' / args.group
    target.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(video))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not cap.isOpened() or fps <= 0:
        raise ValueError(f'Cannot decode video: {video}')
    rows, index, next_t = [], 0, 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t = index / fps
            if t + 1e-9 >= next_t:
                name = f'{video.stem}_{index:08d}.jpg'
                image = target / name
                if image.exists():
                    raise ValueError(f'Existing frame: {image}')
                if not cv2.imwrite(str(image), frame):
                    raise IOError(f'Cannot write {image}')
                rows.append(dict(image=str(image.relative_to(root)), group=args.group,
                                 tags=args.tags, source=str(video), frame_index=index,
                                 source_seconds=t, reviewed=False))
                next_t += args.interval
                if args.max_frames and len(rows) >= args.max_frames:
                    break
            index += 1
    finally:
        cap.release()
    if not rows:
        raise ValueError('No frames extracted')
    (root / 'pool/labels' / args.group).mkdir(parents=True, exist_ok=True)
    add_pool(root, rows)
    print(f'{len(rows)} UNREVIEWED frames extracted; annotation and review required')


def import_images(args):
    root = Path(args.dataset).resolve()
    source = Path(args.images).resolve()
    safe_group(args.group)
    dest = root / 'pool/images' / args.group
    dest.mkdir(parents=True, exist_ok=True)
    rows = []
    for image in images_in(source):
        relative = image.relative_to(source)
        target = dest / relative
        if target.exists():
            raise ValueError(f'Existing image: {target}')
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image, target)
        if args.labels:
            label = Path(args.labels) / relative.with_suffix('.txt')
            if not label.is_file():
                raise ValueError(f'Missing imported label: {label}')
            output_label = label_for(target)
            output_label.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(label, output_label)
        rows.append(dict(image=str(target.relative_to(root)), group=args.group,
                         source=str(image), tags=args.tags, reviewed=False))
    if not rows:
        raise ValueError('No images imported')
    add_pool(root, rows)
    print(f'{len(rows)} images imported; still UNREVIEWED')


def review_group(args):
    root = Path(args.dataset).resolve()
    names = class_names(read_yaml(root / 'data.yaml')['names'])
    rows = pool_manifest(root)
    selected = [r for r in rows if r['group'] == args.group]
    if not selected:
        raise ValueError('Unknown/empty capture group')
    for row in selected:
        image = root / row['image']
        label = label_for(image)
        validate_label(label, len(names))
        row.update(reviewed=True, reviewer=args.reviewer, image_sha256=sha256(image),
                   label_sha256=sha256(label), reviewed_names=names)
    write_json(root / 'pool/manifest.json', rows)
    print(f'Recorded explicit human review by {args.reviewer}: {len(selected)} images')


def review_dataset(args):
    _, manifest = inspect_dataset(args.data)
    receipt = dict(reviewer=args.reviewer, fingerprint=manifest['fingerprint'],
                   reviewed_utc=datetime.now(timezone.utc).isoformat())
    write_json(Path(args.data).with_suffix('.review.json'), receipt)
    print('Recorded explicit review; any image/label/class change invalidates receipt')


def split_groups(args):
    root = Path(args.dataset).resolve()
    assignments = read_yaml(args.assignments)
    mapping = {}
    if set(assignments) != {'train', 'val', 'test'}:
        raise ValueError('assignments must define train, val and test')
    for split, groups in assignments.items():
        if not isinstance(groups, list) or not groups:
            raise ValueError(f'{split} must contain independent capture groups')
        for group in groups:
            safe_group(group)
            if group in mapping:
                raise ValueError(f'Group leaks across splits: {group}')
            mapping[group] = split
    rows = pool_manifest(root)
    if set(mapping) != {r['group'] for r in rows}:
        raise ValueError('Assign all and only existing capture groups exactly once')
    # Prevalidate all review hashes and cross-split image hashes before creating output.
    digests = {}
    names = class_names(read_yaml(root / 'data.yaml')['names'])
    for row in rows:
        image = root / row['image']
        label = label_for(image)
        if (not row.get('reviewed') or not row.get('reviewer') or
                class_names(row.get('reviewed_names', ['UNREVIEWED'])) != names or
                row.get('image_sha256') != sha256(image) or
                row.get('label_sha256') != sha256(label)):
            raise ValueError(f'Unreviewed or modified image/label: {image}')
        split = mapping[row['group']]
        digest = row['image_sha256']
        if digest in digests and digests[digest] != split:
            raise ValueError('Identical image content crosses splits')
        digests[digest] = split
    if (root / 'images').exists() or (root / 'labels').exists():
        raise ValueError('Split output exists; use a new dataset version, do not overwrite')
    subsets, provenance = {}, []
    for row in rows:
        split, group = mapping[row['group']], row['group']
        image = root / row['image']
        relative = image.relative_to(root / 'pool/images' / group)
        output = root / 'images' / split / group / relative
        output.parent.mkdir(parents=True, exist_ok=True)
        label_for(output).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image, output)
        shutil.copy2(label_for(image), label_for(output))
        provenance.append(dict(row, split=split, prepared_image=str(output)))
        for tag in row.get('tags', []):
            safe_group(tag)
            subsets.setdefault((split, tag), []).append(str(output))
    (root / 'subsets').mkdir(exist_ok=True)
    for (split, tag), images in subsets.items():
        (root / 'subsets' / f'{split}_{tag}.txt').write_text('\n'.join(images)+'\n')
    write_json(root / 'split_manifest.json', dict(assignments=assignments, entries=provenance))
    data_path = root / 'data.yaml'
    data = read_yaml(data_path)
    data.update(path='.', train='images/train', val='images/val', test='images/test')
    data_path.write_text(yaml.safe_dump(data, allow_unicode=True))
    # This combines only unchanged labels already explicitly reviewed by a human.
    args.data = str(data_path)
    args.reviewer = '; '.join(sorted({r['reviewer'] for r in rows}))
    review_dataset(args)
    print('Prepared disjoint group splits and explicit tagged subsets')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    ext = sub.add_parser('extract')
    ext.add_argument('--video', required=True)
    ext.add_argument('--interval', type=float, default=1.0)
    ext.add_argument('--max-frames', type=int, default=0)
    imp = sub.add_parser('import')
    imp.add_argument('--images', required=True)
    imp.add_argument('--labels')
    for cmd in (ext, imp):
        cmd.add_argument('--dataset', required=True)
        cmd.add_argument('--group', required=True)
        cmd.add_argument('--tags', nargs='*', default=[])
    rev = sub.add_parser('review-group')
    rev.add_argument('--dataset', required=True)
    rev.add_argument('--group', required=True)
    direct = sub.add_parser('review-dataset')
    direct.add_argument('--data', required=True)
    for cmd in (rev, direct):
        cmd.add_argument('--reviewer', required=True)
        cmd.add_argument('--confirm-manual-review', action='store_true', required=True)
    split = sub.add_parser('split')
    split.add_argument('--dataset', required=True)
    split.add_argument('--assignments', required=True)
    check = sub.add_parser('check')
    check.add_argument('--data', required=True)
    check.add_argument('--require-review', action='store_true')
    args = parser.parse_args()
    if args.command == 'check':
        _, manifest = inspect_dataset(args.data, require_review=args.require_review)
        print(json.dumps(dict(images=len(manifest['entries']), names=manifest['names'],
                              fingerprint=manifest['fingerprint']), indent=2))
    else:
        {'extract': extract, 'import': import_images, 'review-group': review_group,
         'review-dataset': review_dataset, 'split': split_groups}[args.command](args)


if __name__ == '__main__':
    main()
