#!/usr/bin/env python3
"""Import explicitly reviewed round-two positives into an isolated V2 pool and dataset."""
import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import shutil
from statistics import median
import sys
import tempfile
import zipfile

from PIL import Image, ImageDraw
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import class_names, sha256, write_json
from vision_training.dataset import inspect_dataset, label_for, validate_label
from vision_training.tools.import_cvat import import_cvat

ROOT = Path(__file__).resolve().parents[2]
NAMES = ['color_block', 'rock_compact', 'rock_spiky']


def inventory(archive, records):
    expected = {Path(r['filename']).stem: r for r in records}
    if len(expected) != len(records):
        raise ValueError('Duplicate source filenames')
    with zipfile.ZipFile(archive) as z, tempfile.TemporaryDirectory() as temp:
        if z.testzip():
            raise ValueError('ZIP CRC failure')
        names = class_names(yaml.safe_load(z.read('data.yaml'))['names'])
        mapping = {i: NAMES.index(n) for i, n in names.items()}
        members = z.namelist()
        if len(members) != len(set(members)):
            raise ValueError('Duplicate ZIP entries')
        for member in members:
            if Path(member).is_absolute() or '..' in Path(member).parts:
                raise ValueError('Unsafe ZIP path')
        images = [Path(n).stem for n in z.read('train.txt').decode().splitlines() if n.strip()]
        if len(images) != len(set(images)) or set(images) != set(expected):
            raise ValueError('CVAT image list differs from selected sources')
        labels = {Path(n).stem: n for n in members if n.startswith('labels/') and n.endswith('.txt')}
        if len(labels) != sum(n.startswith('labels/') and n.endswith('.txt') for n in members):
            raise ValueError('Duplicate label stems')
        if not set(labels) <= set(expected):
            raise ValueError('Unexpected labels')
        boxes = {}
        for stem, member in labels.items():
            path = Path(temp)/'label.txt'
            path.write_bytes(z.read(member))
            boxes[stem] = validate_label(path, len(names))
            if not boxes[stem]:
                raise ValueError('Empty labels need separate explicit negative review')
        return expected, labels, boxes, mapping


def box_size_report(dataset):
    records = json.loads((dataset/'preparation.json').read_text())['records']
    sizes = defaultdict(list)
    for row in records:
        if row['source_type'] != 'phone_real':
            continue
        group = ('B_validation' if row['split']=='val' else
                 'round2_AC' if row.get('annotation_round')==2 else 'round1_AC')
        image = dataset/row['image']
        with Image.open(image) as im:
            w, h = im.size
        for c, x, y, bw, bh in validate_label(label_for(image), 3):
            sizes[(group, int(c))].append(min(bw*w, bh*h)*640/max(w,h))
    result = {}
    for (group, c), values in sorted(sizes.items()):
        result.setdefault(group, {})[NAMES[c]] = dict(boxes=len(values),
            short_side_lt16=sum(v<16 for v in values), median_short_side_at_640=median(values),
            min_short_side_at_640=min(values), max_short_side_at_640=max(values))
    return result


def protected_snapshot():
    paths = [ROOT/'vision_training/datasets/science_targets_mixed_v1',
             ROOT/'vision_training/datasets/science_targets_synthetic_v1',
             ROOT/'vision_training/datasets/science_targets_video_v1',
             ROOT/'vision_training/runs/science_targets_mixed_v1']
    return {str(p.relative_to(ROOT)): sha256(p) for root in paths for p in root.rglob('*')
            if p.is_file() and p.suffix not in ('.cache', '.pyc')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--confirm-161-complete-positives', action='store_true', required=True)
    args = parser.parse_args()
    ann = ROOT/'vision_training/annotations/science_targets_round2_priority'
    report = ROOT/'vision_training/reports/science_targets_mixed_v2'
    dataset = ROOT/'vision_training/datasets/science_targets_mixed_v2'
    pool = ROOT/'vision_training/datasets/science_targets_round2_reviewed_v2'
    for path in (report, dataset, pool):
        if path.exists():
            raise FileExistsError(path)
    before = protected_snapshot()
    records = json.loads((ann/'manifest.json').read_text())['rows']
    expected, labels, boxes, mapping = inventory(args.archive, records)
    assert len(expected) == 180 and len(labels) == 161
    assert mapping == {0: 0, 1: 2, 2: 1}
    report.mkdir()
    original = ann/'round2_annotations.zip'
    if original.exists():
        if sha256(original) != sha256(args.archive):
            raise ValueError('Existing original archive differs')
    else:
        shutil.copy2(args.archive, original)
    pool.mkdir()
    (pool/'data.yaml').write_text(yaml.safe_dump(dict(names=NAMES)))
    pool_rows = []
    reviews = []
    for stem, row in expected.items():
        if row['video_id'] not in ('video_A', 'video_C'):
            raise ValueError('Held-out B cannot enter round-two training')
        source = ROOT/'vision_training/datasets/science_targets_video_v1'/row['image']
        assert sha256(source) == row['image_sha256']
        target = pool/row['image']
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        pool_rows.append(dict(row, reviewed=False, annotation_status='pending'))
        reviews.append(dict(image=row['filename'], status='positive' if stem in labels else 'pending',
                            boxes=len(boxes[stem]) if stem in boxes else '', source_video=row['source_video'],
                            source_seconds=row['source_seconds'], notes=(
                                'User confirmed all 161 positives complete and correct on 2026-11-14'
                                if stem in labels else 'Missing TXT; NOT a confirmed negative')))
    write_json(pool/'pool/manifest.json', pool_rows)
    def csv_write(path, rows):
        with path.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n'); w.writeheader(); w.writerows(rows)
    csv_write(report/'round2_review.csv', reviews)
    pending = [r for r in reviews if r['status'] == 'pending']
    csv_write(report/'pending_19.csv', pending)
    sheet = Image.new('RGB', (1200, 5*170), 'white')
    draw = ImageDraw.Draw(sheet)
    for i, row in enumerate(pending):
        rec = expected[Path(row['image']).stem]
        with Image.open(pool/rec['image']) as im:
            im.thumbnail((290, 136)); x=(i%4)*300; y=(i//4)*170
            sheet.paste(im, (x, y))
        draw.text((x+3, y+139), row['image'], fill='black')
        draw.text((x+3, y+153), f"PENDING t={row['source_seconds']:.2f}s", fill='red')
    sheet.save(report/'pending_19.jpg')
    imported = import_cvat(pool, original, report/'round2_review.csv',
                           'user_confirmed_161_complete_positives_20261114')
    assert imported == 161
    normalized = ann/'round2_reviewed_normalized_v2.zip'
    with zipfile.ZipFile(normalized, 'x', compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr('data.yaml', yaml.safe_dump(dict(names=NAMES, path='.', train='train.txt')))
        z.writestr('train.txt', ''.join(f'images/train/{r["image"]}\n' for r in reviews if r['status']=='positive'))
        z.writestr('pending.txt', ''.join(r['image']+'\n' for r in pending))
        for stem in labels:
            z.writestr(f'labels/train/{stem}.txt', label_for(pool/expected[stem]['image']).read_bytes())
    dataset.mkdir()
    v1 = ROOT/'vision_training/datasets/science_targets_mixed_v1'
    prep = json.loads((v1/'preparation.json').read_text())
    provenance = []
    def add(image, label, split, row):
        target = dataset/'images'/split/image.name
        target_label = dataset/'labels'/split/label.name
        target.parent.mkdir(parents=True, exist_ok=True)
        target_label.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            raise ValueError('Duplicate training source')
        shutil.copy2(image, target); shutil.copy2(label, target_label)
        provenance.append(dict(row, split=split, image=str(target.relative_to(dataset)),
                               image_sha256=sha256(target), label_sha256=sha256(target_label)))
    for row in prep['records']:
        image = v1/row['image']; label = label_for(image)
        assert sha256(image) == row['image_sha256'] and sha256(label) == row['label_sha256']
        add(image, label, row['split'], row)
    for row in json.loads((pool/'pool/manifest.json').read_text()):
        if not row['reviewed']:
            assert not label_for(pool/row['image']).exists()
            continue
        add(pool/row['image'], label_for(pool/row['image']), 'train',
            dict(row, annotation_round=2))
    assert not ({r['capture_group'] for r in provenance if r['split']=='train'} &
                {r['capture_group'] for r in provenance if r['split']=='val'})
    assert {r['image_sha256'] for r in provenance if r['split']=='val'} == {
        r['image_sha256'] for r in prep['records'] if r['split']=='val'}
    counts = {}
    for split in ('train', 'val'):
        selected = [r for r in provenance if r['split']==split]
        c = Counter(int(b[0]) for r in selected for b in validate_label(label_for(dataset/r['image']),3))
        counts[split] = dict(images=len(selected), boxes={n:c[i] for i,n in enumerate(NAMES)},
                            sources=dict(Counter(r['source_type'] for r in selected)))
    data = dataset/'data.yaml'
    data.write_text(yaml.safe_dump(dict(path=str(dataset), train='images/train', val='images/val', names=NAMES)))
    _, manifest = inspect_dataset(data, splits=('train','val'))
    write_json(data.with_suffix('.review.json'), dict(reviewer='user_round1_and_round2_confirmation_20261114',
        fingerprint=manifest['fingerprint'], real_review='Round1 157 positive/17 negative; round2 161 complete positive/19 pending excluded',
        synthetic_review='Original mask labels, not a claim of manual review of 600 images'))
    new_counts = Counter(mapping[int(b[0])] for bs in boxes.values() for b in bs)
    audit = dict(imported=imported, positive=161, confirmed_negative=0, pending=19,
        new_boxes={n:new_counts[i] for i,n in enumerate(NAMES)}, class_mapping=mapping,
        original_sha256=sha256(original), normalized_sha256=sha256(normalized), split_counts=counts,
        review_source='User explicitly confirmed all 161 labeled images complete and correct in this session',
        limitation='No new real compact boxes; B remains held out; 19 missing labels are pending, not negatives')
    write_json(dataset/'preparation.json', dict(audit, records=provenance))
    write_json(report/'annotation_audit.json', audit)
    write_json(report/'real_box_sizes.json', box_size_report(dataset))
    after = protected_snapshot()
    assert before == after
    write_json(report/'preservation.json', dict(unchanged=True, files=len(before), sha256=before))
    print(json.dumps(audit, indent=2))


if __name__ == '__main__':
    main()
