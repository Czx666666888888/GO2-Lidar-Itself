#!/usr/bin/env python3
"""Package visually curated pool images for annotation without creating labels."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import zipfile

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import class_names, read_yaml, sha256, write_json  # noqa: E402


def image_fingerprint(root, rows):
    """Bind the visual triage plan to the exact original image bytes."""
    hashes = {r['image']: sha256(root / r['image']) for r in rows}
    return hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(), hashes


def prepare(dataset, plan_path, output):
    """Copy included originals and hold cases; never mark annotations reviewed."""
    root, output = Path(dataset).resolve(), Path(output).resolve()
    manifest_path = root / 'pool/manifest.json'
    original = manifest_path.read_bytes()
    rows = json.loads(original)
    plan = read_yaml(plan_path)
    if plan.get('schema_version') != 1 or plan.get('default_action') != 'include':
        raise ValueError('Expected schema_version=1, default_action=include visual plan')
    if len(rows) != plan['expected_images'] or sha256(manifest_path) != plan[
            'dataset_manifest_sha256']:
        raise ValueError('Source manifest changed; visually recheck the plan')
    if output.exists():
        raise ValueError('Output exists; choose a new annotation version, do not overwrite')
    if output == root or output.is_relative_to(root / 'pool'):
        raise ValueError('Annotation package must not overwrite the original pool')
    names = class_names(read_yaml(root / 'data.yaml')['names'])
    by_name = {Path(r['image']).name: r for r in rows}
    if len(by_name) != len(rows):
        raise ValueError('Ambiguous duplicate basenames; cannot import into a flat CVAT task')
    duplicates = plan.get('excluded_near_duplicates', {})
    held = plan.get('hold_for_visual_review', {})
    attention = plan.get('attention', {})
    mentioned = set(duplicates) | set(duplicates.values()) | set(held)
    mentioned |= {name for values in attention.values() for name in values}
    if mentioned-set(by_name) or set(duplicates) & set(held):
        raise ValueError('Unknown or conflicting image names in curation plan')
    for name, representative in duplicates.items():
        if representative in duplicates or representative in held or name == representative:
            raise ValueError('Duplicate representative must remain included')
        if by_name[name]['source'] != by_name[representative]['source']:
            raise ValueError('Near-duplicate removal must stay within a source video')
    fingerprint, hashes = image_fingerprint(root, rows)
    if fingerprint != plan['input_images_fingerprint']:
        raise ValueError('Source image bytes changed; visually recheck the plan')
    records = []
    # Decode before mutating output. Low sharpness never automatically removes a frame.
    for index, row in enumerate(rows, 1):
        path = root / row['image']
        if not path.resolve().is_relative_to(root):
            raise ValueError('Source image must remain inside the dataset')
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f'Unreadable source image: {path}')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        name = path.name
        action = 'near_duplicate' if name in duplicates else 'hold' if name in held else 'include'
        records.append(dict(
            source_index=index, filename=name, source_image=row['image'],
            capture_group=row['group'], source_video=Path(row['source']).name,
            source_seconds=row['source_seconds'], source_frame_index=row['frame_index'],
            image_sha256=hashes[row['image']], action=action,
            representative=duplicates.get(name), hold_reason=held.get(name),
            attention=[tag for tag, values in attention.items() if name in values],
            width=int(image.shape[1]), height=int(image.shape[0]),
            mean_gray=float(gray.mean()),
            laplacian_variance=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
            annotation_reviewed=False))
    output.mkdir(parents=True)
    (output / 'images').mkdir()
    (output / 'hold').mkdir()
    for record in records:
        if record['action'] in ('include', 'hold'):
            folder = 'images' if record['action'] == 'include' else 'hold'
            shutil.copy2(root / record['source_image'], output / folder / record['filename'])
    archive = output / 'images_for_cvat.zip'
    included = [r for r in records if r['action'] == 'include']
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as stream:
        for record in included:
            stream.write(output / 'images' / record['filename'], arcname=record['filename'])
    # Verify both ZIP integrity and exact original image bytes after re-reading the archive.
    with zipfile.ZipFile(archive) as stream:
        if stream.testzip() is not None or len(stream.namelist()) != len(included):
            raise ValueError('Annotation ZIP failed integrity validation')
        for record in included:
            actual = hashlib.sha256(stream.read(record['filename'])).hexdigest()
            if actual != record['image_sha256']:
                raise ValueError('Annotation ZIP image mismatch')
    (output / 'classes.txt').write_text('\n'.join(names.values())+'\n')
    fields = ['filename', 'source_index', 'source_video', 'source_seconds', 'capture_group',
              'attention', 'human_status', 'object_count', 'reviewer', 'notes']
    with (output / 'human_review.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in included:
            writer.writerow(dict(
                **{k: row[k] for k in fields[:5]}, attention=';'.join(row['attention']),
                human_status='PENDING', object_count='', reviewer='', notes=''))
    write_json(output / 'manifest.json', dict(
        source_manifest_sha256=sha256(manifest_path), input_images_fingerprint=fingerprint,
        curation_plan_sha256=sha256(plan_path), names=names, images=records,
        annotation_reviewed=False, note='Quality triage only, not boxes/negative ground truth'))
    summary = dict(total=len(records), included=len(included),
                   excluded_near_duplicates=len(duplicates), held=len(held), unreadable=0,
                   label_files_created=0, human_reviewed=0, names=names,
                   groups=sorted({r['capture_group'] for r in records}),
                   retained_attention={tag: sum(tag in r['attention'] for r in included)
                                       for tag in attention},
                   zip_sha256=sha256(archive), original_pool_unchanged=True)
    if manifest_path.read_bytes() != original:
        raise RuntimeError('Original pool manifest unexpectedly changed')
    if image_fingerprint(root, rows)[0] != fingerprint:
        raise RuntimeError('Original source image bytes unexpectedly changed')
    write_json(output / 'summary.json', summary)
    contact_sheets(root, records, output / 'contact_sheets')
    return summary


def contact_sheets(root, records, output):
    """Write static overview pages; originals remain full resolution for CVAT."""
    output.mkdir()
    for start in range(0, len(records), 24):
        tiles = []
        for row in records[start:start+24]:
            tile = np.zeros((222, 256, 3), dtype=np.uint8)
            tile[:192] = cv2.resize(cv2.imread(str(root / row['source_image'])), (256, 192))
            color = (0, 255, 0) if row['action'] == 'include' else (0, 165, 255)
            cv2.putText(tile, f'{row["source_index"]:03d} {row["action"]}', (3, 207),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
            tiles.append(tile)
        while len(tiles) % 4:
            tiles.append(np.zeros((222, 256, 3), dtype=np.uint8))
        sheet = cv2.vconcat([cv2.hconcat(tiles[i:i+4]) for i in range(0, len(tiles), 4)])
        cv2.imwrite(str(output / f'page_{start//24+1:02d}.jpg'), sheet)


def main():
    """Create a local package; uploading and box annotation remain human actions."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--plan', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.dataset, args.plan, args.output), indent=2))


if __name__ == '__main__':
    main()
