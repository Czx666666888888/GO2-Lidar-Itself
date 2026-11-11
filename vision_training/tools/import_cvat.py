#!/usr/bin/env python3
"""Import CVAT YOLO Detection labels into an existing pool after per-image review."""
import argparse
import csv
from pathlib import Path
import sys
import tempfile
import zipfile

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import class_names, read_yaml, sha256, write_json  # noqa: E402
from vision_training.dataset import label_for, validate_label  # noqa: E402
from vision_training.tools.dataset import pool_manifest  # noqa: E402


def import_cvat(dataset, archive, review_csv, reviewer, class_map=None):
    root = Path(dataset).resolve()
    rows = pool_manifest(root)
    names = class_names(read_yaml(root / 'data.yaml')['names'])
    aliases = read_yaml(class_map) if class_map else {}
    by_name = {}
    for row in rows:
        by_name.setdefault(Path(row['image']).name, []).append(row)
    with Path(review_csv).open(newline='') as stream:
        reviews = list(csv.DictReader(stream))
    if not reviewer.strip() or len({r['image'] for r in reviews}) != len(reviews):
        raise ValueError('Reviewer required; duplicate image review rows are not allowed')
    prepared = []
    with zipfile.ZipFile(archive) as z, tempfile.TemporaryDirectory() as temp:
        # CVAT supports Ultralytics Detection and classic YOLO 1.1 exports.
        if 'data.yaml' in z.namelist():
            original_names = class_names(yaml.safe_load(z.read('data.yaml'))['names'])
        elif 'obj.names' in z.namelist():
            original_names = class_names(z.read('obj.names').decode().splitlines())
        else:
            raise ValueError('Expected data.yaml or obj.names with ordered class names')
        mapped = {i: next((j for j, n in names.items() if n == aliases.get(nm, nm)), None)
                  for i, nm in original_names.items()}
        if None in mapped.values() or len(set(mapped.values())) != len(mapped):
            raise ValueError('Export classes do not map uniquely to dataset classes')
        labels = {}
        for n in z.namelist():
            p = Path(n)
            if p.is_absolute() or '..' in p.parts:
                raise ValueError('Unsafe archive member path')
            if p.suffix == '.txt' and (n.startswith('labels/') or
                                       n.startswith('obj_train_data/') or
                                       n.startswith('obj_valid_data/')):
                labels.setdefault(p.stem, []).append(n)
        for review in reviews:
            status = review['status'].strip().lower()
            if status in ('pending', 'uncertain'):
                continue
            if status not in ('positive', 'negative'):
                raise ValueError(f'Unknown review status: {status}')
            matches = by_name.get(review['image'], [])
            if len(matches) != 1:
                raise ValueError(f'Ambiguous/missing pool image: {review["image"]}')
            row = matches[0]
            members = labels.get(Path(row['image']).stem, [])
            if len(members) != 1:
                raise ValueError(f'Missing/ambiguous label, including negatives: {row["image"]}')
            original = z.read(members[0])
            tmp = Path(temp) / 'label.txt'
            tmp.write_bytes(original)
            boxes = validate_label(tmp, len(original_names))
            if bool(boxes) != (status == 'positive'):
                raise ValueError(f'Review status and label disagree: {row["image"]}')
            lines = [s for s in original.decode().splitlines() if s.strip()]
            text = ''.join(f'{mapped[int(b[0])]} ' + ' '.join(line.split()[1:]) + '\n'
                           for b, line in zip(boxes, lines))
            label = label_for(root / row['image'])
            if label.exists() and label.read_text() != text:
                raise ValueError(f'Existing label differs; use a new dataset version: {label}')
            prepared.append((row, label, text, members[0], original, status))
        # No pool changes until all selected records have passed validation.
        for row, label, text, member, original, status in prepared:
            label.parent.mkdir(parents=True, exist_ok=True)
            label.write_text(text)
            original_file = root / 'pool/original_cvat' / sha256(archive) / member
            original_file.parent.mkdir(parents=True, exist_ok=True)
            original_file.write_bytes(original)
            row.update(reviewed=True, reviewer=reviewer, annotation_status=status,
                       image_sha256=sha256(root / row['image']), label_sha256=sha256(label),
                       reviewed_names=names, cvat_archive_sha256=sha256(archive),
                       cvat_label=member, original_label_sha256=sha256(original_file),
                       class_mapping={i: dict(source=original_names[i], target_id=j,
                                              target=names[j]) for i, j in mapped.items()})
    write_json(root / 'pool/manifest.json', rows)
    return len(prepared)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--archive', required=True)
    parser.add_argument('--review-csv', required=True,
                        help='image,status columns; positive/negative/pending/uncertain')
    parser.add_argument('--reviewer', required=True)
    parser.add_argument('--class-map', help='Optional YAML source_name: target_name aliases')
    parser.add_argument('--confirm-manual-review', action='store_true', required=True)
    args = parser.parse_args()
    print('Imported reviewed images:', import_cvat(
        args.dataset, args.archive, args.review_csv, args.reviewer, args.class_map))


if __name__ == '__main__':
    main()
