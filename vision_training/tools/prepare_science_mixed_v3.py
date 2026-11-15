#!/usr/bin/env python3
"""Import user-reviewed round-three CVAT exports; preserve V2 and held-out scenes."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import shutil
import sys
import tempfile
import zipfile

from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json
from vision_training.dataset import inspect_dataset, label_for, validate_label
from vision_training.tools.import_cvat import import_cvat
from vision_training.tools.prepare_science_mixed_v2 import inventory

ROOT = Path(__file__).resolve().parents[2]
NAMES = ['color_block', 'rock_compact', 'rock_spiky']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archives', nargs=2, type=Path, required=True)
    p.add_argument('--confirm-user-reviewed-259-positive-46-negative', action='store_true', required=True)
    args = p.parse_args()
    base = ROOT/'vision_training'
    ann = base/'annotations/science_targets_round3'
    source = base/'datasets/science_targets_round3_candidates'
    old = base/'datasets/science_targets_mixed_v2'
    dataset = base/'datasets/science_targets_mixed_v3'
    pool = base/'datasets/science_targets_round3_reviewed_v3'
    report = base/'reports/science_targets_mixed_v3'
    for path in (dataset, pool, report):
        if path.exists():
            raise FileExistsError(path)
    protected_roots = [old, base/'datasets/science_targets_mixed_v1',
                       base/'datasets/science_targets_synthetic_v1',
                       base/'datasets/blue_cube_pilot_v1', base/'runs/science_targets_mixed_v1',
                       base/'runs/science_targets_mixed_v2']
    before = {str(f.relative_to(ROOT)): sha256(f) for root in protected_roots
              for f in root.rglob('*') if f.is_file() and f.suffix not in ('.cache', '.pyc')}
    records = json.loads((source/'selected/manifest.json').read_text())
    selected = [r for r in records if r.get('eligible_for_priority_annotation')]
    assert len(selected) == 305
    old_records = json.loads((old/'preparation.json').read_text())['records']
    legacy = json.loads((source/'reused_legacy20/manifest.json').read_text())
    old_hashes = {r['image_sha256'] for r in old_records + legacy if r.get('source_video')}
    old_frames = {(Path(r['source_video']).name, r.get('frame_index', r.get('source_frame_index'))) for r in old_records + legacy if r.get('source_video')}
    assert len({r['image_sha256'] for r in selected}) == 305
    assert not old_hashes & {r['image_sha256'] for r in selected}
    for r in selected:
        assert r['proposed_role'] == 'train_candidate'
        assert r['video_id'] not in ('r3_new3', 'r3_new4', 'video_B')
        assert (Path(r['source_video']).name, r['frame_index']) not in old_frames
        image = source/r['image']
        assert sha256(image) == r['image_sha256']
        with Image.open(image) as im:
            assert list(im.size) == r['resolution']
            im.verify()
    # Validate BOTH archives before writing any output. Missing labels are authorized
    # negatives only for these exact two fully reviewed batches in this user request.
    jobs = []
    seen = set()
    for index, archive in enumerate(args.archives, 1):
        with zipfile.ZipFile(archive) as z:
            stems = {Path(n).stem for n in z.read('train.txt').decode().splitlines() if n.strip()}
        with zipfile.ZipFile(ann/f'round3_priority_batch_{index:03d}.zip') as package:
            package_stems = {Path(n).stem for n in package.namelist() if Path(n).suffix.lower() in ('.jpg', '.png')}
        assert stems == package_stems
        rows = [r for r in selected if Path(r['filename']).stem in stems]
        expected, labels, boxes, mapping = inventory(archive, rows)
        assert len(rows) == (153 if index == 1 else 152)
        assert len(labels) == (127 if index == 1 else 132)
        assert not seen & set(expected)
        seen.update(expected)
        jobs.append((archive, rows, labels, boxes, mapping))
    assert seen == {Path(r['filename']).stem for r in selected}
    counts = Counter()
    for _, _, _, boxes, mapping in jobs:
        counts.update(mapping[int(b[0])] for bs in boxes.values() for b in bs)
    assert [counts[i] for i in range(3)] == [365, 0, 140]
    pool.mkdir(); (pool/'pool').mkdir(); report.mkdir(); dataset.mkdir()
    (pool/'data.yaml').write_text(yaml.safe_dump(dict(names=NAMES)))
    pool_rows = []
    for r in selected:
        target = pool/r['image']; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source/r['image'], target)
        pool_rows.append(dict(r, reviewed=False, annotation_status='pending'))
    write_json(pool/'pool/manifest.json', pool_rows)
    audit = []
    all_reviews = []
    for index, (archive, rows, labels, boxes, mapping) in enumerate(jobs, 1):
        original = ann/f'round3_batch_{index:03d}_original_annotations.zip'
        if original.exists():
            assert sha256(original) == sha256(archive)
        else:
            shutil.copy2(archive, original)
        normalized = ann/f'round3_batch_{index:03d}_confirmed_negatives.zip'
        # Preserve source class order here; existing import_cvat maps by NAME.
        with zipfile.ZipFile(original) as z, zipfile.ZipFile(normalized, 'x', zipfile.ZIP_DEFLATED) as out:
            for name in z.namelist():
                out.writestr(name, z.read(name))
            for r in rows:
                stem = Path(r['filename']).stem
                if stem not in labels:
                    out.writestr(f'labels/train/{stem}.txt', '')
        reviews = [dict(image=r['filename'], status='positive' if Path(r['filename']).stem in labels else 'negative',
                        boxes=len(boxes.get(Path(r['filename']).stem, [])),
                        review_source='User CVAT annotation and explicit negative confirmation 2026-11-15',
                        batch=index) for r in rows]
        review_path = report/f'batch_{index:03d}_review.csv'
        with review_path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(reviews[0]), lineterminator='\n'); writer.writeheader(); writer.writerows(reviews)
        assert import_cvat(pool, normalized, review_path, 'user_round3_CVAT_259_positive_46_negative_20261115') == len(rows)
        all_reviews.extend(reviews)
        audit.append(dict(batch=index, images=len(rows), positive=len(labels), negative=len(rows)-len(labels),
                          original_sha256=sha256(original), source_sha256=sha256(archive),
                          normalized_sha256=sha256(normalized), class_mapping=mapping,
                          multi_object_images=sum(len(b)>1 for b in boxes.values()),
                          boxes=sum(map(len, boxes.values()))))
    provenance = []
    def add(root, row):
        image = root/row['image']; label = label_for(image)
        assert sha256(image) == row['image_sha256'] and sha256(label) == row['label_sha256']
        split = row.get('split', 'train')
        dst = dataset/'images'/split/image.name
        dst_label = dataset/'labels'/split/label.name
        assert not dst.exists()
        dst.parent.mkdir(parents=True, exist_ok=True); dst_label.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image, dst); shutil.copy2(label, dst_label)
        provenance.append(dict(row, split=split, image=str(dst.relative_to(dataset)),
                               image_sha256=sha256(dst), label_sha256=sha256(dst_label)))
    for r in old_records:
        add(old, r)
    for r in json.loads((pool/'pool/manifest.json').read_text()):
        add(pool, dict(r, split='train', annotation_round=3))
    train = [r for r in provenance if r['split']=='train']
    val = [r for r in provenance if r['split']=='val']
    assert not {r['capture_group'] for r in train} & {r['capture_group'] for r in val}
    assert len({r['image_sha256'] for r in provenance}) == len(provenance)
    assert {(r['image_sha256'], r['label_sha256']) for r in val} == {(r['image_sha256'], r['label_sha256']) for r in old_records if r['split']=='val'}
    summary = {}
    for split in ('train', 'val'):
        rows = [r for r in provenance if r['split']==split]
        c = Counter(int(b[0]) for r in rows for b in validate_label(label_for(dataset/r['image']), 3))
        summary[split] = dict(images=len(rows), boxes={n:c[i] for i,n in enumerate(NAMES)},
                              sources=dict(Counter(r['source_type'] for r in rows)))
    data = dataset/'data.yaml'
    data.write_text(yaml.safe_dump(dict(path=str(dataset), train='images/train', val='images/val', names=NAMES)))
    _, manifest = inspect_dataset(data, splits=('train','val'))
    write_json(data.with_suffix('.review.json'), dict(reviewer='user_confirmed_round1_round2_round3', fingerprint=manifest['fingerprint'],
        review_source='Round3 259 CVAT positives; 46 negatives explicitly confirmed by user; synthetic mask labels unchanged'))
    result = dict(batches=audit, imported=305, positive=259, user_confirmed_negative=46,
                  new_boxes={n:counts[i] for i,n in enumerate(NAMES)}, split_counts=summary,
                  legacy20_added=0, legacy20_reason='Not in V2; single-class labels lack full three-class review; retained separately, no duplicate import',
                  held_out_new3=50, held_out_new4=86, pending_round2=19,
                  continuity_scope='Historical GO2 laboratory group enters training. Same-group video evaluation is diagnostic, not independent generalization.')
    write_json(dataset/'preparation.json', dict(result, records=provenance))
    write_json(report/'annotation_audit.json', result)
    write_json(report/'dataset_version.json', dict(result, records=provenance))
    after = {name: sha256(ROOT/name) for name in before}
    assert before == after
    write_json(report/'preservation.json', dict(unchanged=True, files=len(before), sha256=before))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
