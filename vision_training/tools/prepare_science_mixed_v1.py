#!/usr/bin/env python3
"""One-cycle preparation with explicit batch001 human confirmation; no inferred negatives."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import shutil
import sys
import tempfile
import zipfile

import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json, class_names
from vision_training.dataset import validate_label, inspect_dataset
from vision_training.tools.import_cvat import import_cvat

ROOT = Path(__file__).resolve().parents[2]
NAMES = ['color_block', 'rock_compact', 'rock_spiky']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive', type=Path, required=True)
    p.add_argument('--confirm-157-positive-17-negative', action='store_true', required=True)
    args = p.parse_args()
    video = ROOT/'vision_training/datasets/science_targets_video_v1'
    synthetic = ROOT/'vision_training/datasets/science_targets_synthetic_v1'
    output = ROOT/'vision_training/datasets/science_targets_mixed_v1'
    if output.exists(): raise FileExistsError(output)
    batch = json.loads((video/'cvat_packages/manifest.json').read_text())['batches'][0]['records']
    expected = {Path(r['filename']).stem:r for r in batch}
    annotation_dir = ROOT/'vision_training/annotations'
    annotation_dir.mkdir(exist_ok=True)
    original = annotation_dir/'batch_001_annotations.zip'
    if original.exists():
        if sha256(original) != sha256(args.archive): raise ValueError('Existing annotation differs')
    else: shutil.copy2(args.archive, original)
    normalized = annotation_dir/'batch_001_reviewed_normalized.zip'
    stats, rows = Counter(), []
    with zipfile.ZipFile(original) as z, tempfile.TemporaryDirectory() as temp:
        if z.testzip(): raise ValueError('ZIP CRC failure')
        assert class_names(yaml.safe_load(z.read('data.yaml'))['names']) == dict(enumerate(NAMES))
        exported_images = z.read('train.txt').decode().splitlines()
        assert len(exported_images) == len(set(exported_images)) == 174
        assert {Path(n).stem for n in exported_images} == set(expected)
        members = [n for n in z.namelist() if n.startswith('labels/') and n.endswith('.txt')]
        labels = {Path(n).stem:n for n in members}
        assert len(labels) == len(members) == 157 and set(labels) <= set(expected)
        assert len(expected) == 174 and len(expected.keys()-labels.keys()) == 17
        with zipfile.ZipFile(normalized, 'x') as dest:
            for n in z.namelist(): dest.writestr(n, z.read(n))
            for stem, r in expected.items():
                source = video/r['image_file']
                assert sha256(source) == r['image_sha256']
                if stem in labels:
                    path = Path(temp)/'label.txt';path.write_bytes(z.read(labels[stem]))
                    boxes = validate_label(path, 3)
                    assert boxes
                    status = 'positive'
                    stats.update(str(int(b[0])) for b in boxes)
                else:
                    boxes = [];status = 'negative'
                    dest.writestr(f'labels/train/{stem}.txt', '')
                rows.append(dict(image=r['filename'], status=status, boxes=len(boxes),
                                 notes='User explicitly confirmed 157 complete positives and all 17 missing labels negative on 2026-11-14'))
    with (video/'cvat_packages/batch_001_review.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['image','status','notes']);writer.writeheader()
        writer.writerows({k:r[k] for k in writer.fieldnames} for r in rows)
    imported = import_cvat(video, normalized, video/'cvat_packages/batch_001_review.csv', 'user_confirmation_20261114')
    assert imported == 174
    output.mkdir()
    provenance = []
    syn_manifest = json.loads((synthetic/'manifest.json').read_text())
    before = {str(f.relative_to(synthetic)):sha256(f) for f in synthetic.rglob('*') if f.is_file()}
    pool = {Path(r['image']).name:r for r in json.loads((video/'pool/manifest.json').read_text())}
    for split in ['train','val']:
        (output/'images'/split).mkdir(parents=True);(output/'labels'/split).mkdir(parents=True)
    def add(image, label, split, record):
        target = output/'images'/split/image.name
        if target.exists(): raise ValueError('Duplicate destination')
        shutil.copy2(image, target)
        target_label = output/'labels'/split/(image.stem+'.txt');shutil.copy2(label,target_label)
        validate_label(target_label, 3)
        provenance.append(dict(record, split=split,image=str(target.relative_to(output)),
                               image_sha256=sha256(target), label_sha256=sha256(target_label)))
    for r in syn_manifest['records']:
        split=r['split'];name=r['id']
        add(synthetic/'images'/split/(name+'.png'), synthetic/'labels'/split/(name+'.txt'),split,
            dict(source_type='synthetic',capture_group=r['scene_id'],source_video=None))
    for review in rows:
        r=pool[review['image']];assert r['reviewed']
        split='val' if r['video_id']=='video_B' else 'train'
        image=video/r['image'];label=Path(str(image).replace('/images/','/labels/')).with_suffix('.txt')
        add(image,label,split,dict(source_type=r['source_type'],source_video=r['source_video'],
            source_video_sha256=r['source_video_sha256'],frame_index=r['frame_index'],
            capture_group=r['capture_group'],annotation_status=r['annotation_status']))
    train_groups={r['capture_group'] for r in provenance if r['split']=='train'}
    val_groups={r['capture_group'] for r in provenance if r['split']=='val'}
    assert not train_groups & val_groups
    data=output/'data.yaml'
    data.write_text(yaml.safe_dump(dict(path=str(output),train='images/train',val='images/val',names=NAMES)))
    _,manifest=inspect_dataset(data,splits=('train','val'))
    write_json(data.with_suffix('.review.json'),dict(reviewer='user_batch001_20261114_and_automatic_synthetic_labels',
        fingerprint=manifest['fingerprint'],synthetic_review='Original automatic mask labels; not 600-image human review',
        real_review='Explicit user confirmation 157 complete positives /17 negatives'))
    after={str(f.relative_to(synthetic)):sha256(f) for f in synthetic.rglob('*') if f.is_file()}
    assert before==after
    report=dict(imported=imported,positive=157,negative=17,missing_labels_normalized=17,
        archive_sha256=sha256(original),normalized_sha256=sha256(normalized),boxes=stats,
        multi_object_images=sum(r['boxes']>1 for r in rows),
        split_sources=dict(Counter(r['split']+':'+r['source_type'] for r in provenance)),
        synthetic_files_unchanged=len(before),records=provenance,review=rows,
        split_reason='Hold out full phone scene B for independent three-class real validation; A/C train; synthetic original 480/120 scene split',
        limitation='Real compact labels occur only in scene B validation; training compact relies on synthetic samples')
    write_json(output/'preparation.json', report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('records','review')},indent=2))

if __name__=='__main__':main()
