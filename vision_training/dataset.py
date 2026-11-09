"""Validate standard YOLO detection datasets and explicit human-review receipts."""

import hashlib
import json
import math
from pathlib import Path

from PIL import Image

try:
    from .common import class_names, read_yaml, resolve, sha256
except ImportError:
    from common import class_names, read_yaml, resolve, sha256

IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff', '.webp'}


def images_in(path):
    path = Path(path)
    if path.is_dir():
        return sorted(p.resolve() for p in path.rglob('*') if p.suffix.lower() in IMAGE_SUFFIXES)
    if path.suffix == '.txt' and path.is_file():
        return [resolve(path.parent, s.strip()) for s in path.read_text().splitlines()
                if s.strip()]
    raise ValueError(f'Expected image directory or image-list .txt: {path}')


def label_for(image):
    parts = list(Path(image).parts)
    # Standard YOLO images/... -> labels/... convention.
    indices = [i for i, s in enumerate(parts) if s == 'images']
    if not indices:
        raise ValueError(f'Image path must contain an images directory: {image}')
    parts[indices[-1]] = 'labels'
    return Path(*parts).with_suffix('.txt')


def validate_label(path, count):
    if not Path(path).is_file():
        raise ValueError(f'Missing label (empty file required for reviewed negatives): {path}')
    rows = []
    for line_no, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            values = [float(x) for x in line.split()]
            if len(values) != 5 or not all(math.isfinite(x) for x in values):
                raise ValueError('expected five finite values')
            cls, x, y, w, h = values
            if cls != int(cls) or not 0 <= cls < count:
                raise ValueError('illegal class ID')
            if not (0 <= x <= 1 and 0 <= y <= 1 and 0 < w <= 1 and 0 < h <= 1):
                raise ValueError('normalized coordinates out of range')
            if min(x-w/2, y-h/2) < -1e-6 or max(x+w/2, y+h/2) > 1+1e-6:
                raise ValueError('box extends outside image')
            rows.append(values)
        except ValueError as exc:
            raise ValueError(f'{path}:{line_no}: {exc}') from exc
    return rows


def inspect_dataset(data_path, splits=('train', 'val', 'test'), require_review=False):
    data_path = Path(data_path).resolve()
    data = read_yaml(data_path)
    names = class_names(data.get('names'))
    if 'nc' in data and data['nc'] != len(names):
        raise ValueError('nc disagrees with names')
    root = resolve(data_path.parent, data.get('path', '.'))
    normalized = dict(data, path=str(root), names=names)
    entries, hashes, label_paths = [], {}, set()
    for split in splits:
        if split not in data:
            raise ValueError(f'Missing split: {split}')
        sources = data[split] if isinstance(data[split], list) else [data[split]]
        resolved_sources = [resolve(root, s) for s in sources]
        normalized[split] = [str(s) for s in resolved_sources]
        images = [p for source in resolved_sources for p in images_in(source)]
        if not images:
            raise ValueError(f'Empty split {split}')
        seen = set()
        for image in images:
            if image in seen:
                raise ValueError(f'Duplicate image path in {split}: {image}')
            seen.add(image)
            with Image.open(image) as im:
                im.verify()
            label = label_for(image)
            validate_label(label, len(names))
            if label in label_paths:
                raise ValueError(f'Multiple images share one label path: {label}')
            label_paths.add(label)
            digest = sha256(image)
            if digest in hashes and hashes[digest] != split:
                raise ValueError(f'Cross-split duplicate image: {image} ({hashes[digest]}/{split})')
            hashes[digest] = split
            entries.append(dict(split=split, image=str(image), label=str(label),
                                image_sha256=digest, label_sha256=sha256(label)))
        # Orphan labels in directories corresponding to the selected image directories.
        for source in resolved_sources:
            if source.is_dir():
                orphan_root = label_for(source / 'placeholder.jpg').parent
                if orphan_root.is_dir():
                    expected = {label_for(p) for p in images_in(source)}
                    orphan = set(orphan_root.rglob('*.txt')) - expected
                    if orphan:
                        raise ValueError(f'Orphan labels: {sorted(str(x) for x in orphan)[:5]}')
    fingerprint = hashlib.sha256(json.dumps(
        dict(names=names, entries=entries), sort_keys=True).encode()).hexdigest()
    manifest = dict(data_yaml=str(data_path), names=names, entries=entries,
                    fingerprint=fingerprint, data_yaml_sha256=sha256(data_path))
    if require_review:
        receipt = data_path.with_suffix('.review.json')
        if not receipt.is_file():
            raise ValueError(f'Human review required: create {receipt.name} after annotation')
        review = json.loads(receipt.read_text())
        if not review.get('reviewer') or review.get('fingerprint') != fingerprint:
            raise ValueError('Review receipt stale or missing reviewer; re-review changed data')
        manifest['human_review'] = review
    return normalized, manifest
