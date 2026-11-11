#!/usr/bin/env python3
"""Import only CVAT labels into a new pilot version; never use train.txt as labels."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import zipfile

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json  # noqa: E402
from vision_training.dataset import inspect_dataset, validate_label  # noqa: E402


def prepare(archive, source, output, val_video):
    archive, source, output = map(lambda p: Path(p).resolve(), (archive, source, output))
    if output.exists():
        raise ValueError('Choose a new version; output already exists')
    metadata = json.loads((source / 'manifest.json').read_text())['images']
    originals = {r['filename']: r for r in metadata if r['action'] == 'include'}
    before = {n: sha256(source / 'images' / n) for n in originals}
    with zipfile.ZipFile(archive) as z:
        if z.testzip():
            raise ValueError('Corrupt ZIP')
        classes = yaml.safe_load(z.read('data.yaml'))['names']
        if classes != {0: 'blue_cube'}:
            raise ValueError('Expected class 0 blue_cube')
        labels = [n for n in z.namelist() if n.startswith('labels/') and n.endswith('.txt')]
        if len(labels) != 20 or len({Path(n).stem for n in labels}) != 20:
            raise ValueError('Expected exactly 20 unique human label files')
        records = []
        for label in sorted(labels):
            matches = [n for n in originals if Path(n).stem == Path(label).stem]
            if len(matches) != 1:
                raise ValueError(f'Ambiguous/missing image: {label}')
            name = matches[0]
            r = originals[name]
            split = 'val' if r['source_video'] == val_video else 'train'
            records.append(dict(r, split=split, label_text=z.read(label).decode(),
                                zip_label=label))
    if {s: sum(r['split'] == s for r in records) for s in ('train', 'val')} != {
            'train': 14, 'val': 6}:
        raise ValueError('Selected source video must yield 14 train / 6 val')
    for split in ('train', 'val'):
        for kind in ('images', 'labels'):
            (output / kind / split).mkdir(parents=True)
    tiles = []
    for r in records:
        name, split = r['filename'], r['split']
        image_path = output / 'images' / split / name
        label_path = output / 'labels' / split / Path(name).with_suffix('.txt')
        shutil.copy2(source / 'images' / name, image_path)
        label_path.write_text(r.pop('label_text'))
        boxes = validate_label(label_path, 1)
        if len(boxes) != 1:
            raise ValueError('Expected one human blue_cube box per image')
        image = cv2.imread(str(image_path))
        h, w = image.shape[:2]
        _, x, y, bw, bh = boxes[0]
        cv2.rectangle(image, (round((x-bw/2)*w), round((y-bh/2)*h)),
                      (round((x+bw/2)*w), round((y+bh/2)*h)), (0, 255, 0), 2)
        tile = np.zeros((264, 320, 3), np.uint8)
        tile[:240] = cv2.resize(image, (320, 240))
        cv2.putText(tile, f'{len(tiles)+1:02d} {split} {r["source_seconds"]}s',
                    (5, 257), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
        tiles.append(tile)
        r['image_sha256'] = sha256(image_path)
        r['label_sha256'] = sha256(label_path)
    (output / 'data.yaml').write_text(yaml.safe_dump(dict(
        path='.', train='images/train', val='images/val', names={0: 'blue_cube'})))
    _, manifest = inspect_dataset(output / 'data.yaml', splits=('train', 'val'))
    write_json(output / 'version.json', dict(
        archive_sha256=sha256(archive), records=records,
        source_hashes=before, dataset_fingerprint=manifest['fingerprint'],
        grouping='source video, same lab scene; not independent capture sessions',
        unlabelled_excluded=len(originals)-len(records)))
    cv2.imwrite(str(output / 'label_review.jpg'), cv2.vconcat([
        cv2.hconcat(tiles[i:i+4]) for i in range(0, len(tiles), 4)]))
    assert before == {n: sha256(source / 'images' / n) for n in originals}
    print(json.dumps(dict(train=14, val=6, source_images_unchanged=len(before),
                          fingerprint=manifest['fingerprint']), indent=2))
    # Review receipt is deliberately separate: inspect the actual rendered boxes first.


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive', required=True)
    p.add_argument('--source', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--val-video', required=True)
    a = p.parse_args()
    prepare(a.archive, a.source, a.output, a.val_video)
