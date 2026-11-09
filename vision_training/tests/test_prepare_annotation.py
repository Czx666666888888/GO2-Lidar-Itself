"""Annotation packaging must preserve originals and never create training truth."""

import json
import zipfile

import cv2
import numpy as np
import pytest
import yaml

from vision_training.common import sha256
from vision_training.tools.prepare_annotation import image_fingerprint, prepare


def fixture(tmp_path):
    root = tmp_path / 'dataset'
    folder = root / 'pool/images/session_A'
    folder.mkdir(parents=True)
    rows = []
    for i in range(4):
        image = folder / f'image_{i}.jpg'
        cv2.imwrite(str(image), np.full((32, 32, 3), i*50, np.uint8))
        rows.append(dict(image=str(image.relative_to(root)), group='session_A',
                         source='/local/video.mp4', source_seconds=i, frame_index=i*15,
                         reviewed=False))
    manifest = root / 'pool/manifest.json'
    manifest.write_text(json.dumps(rows))
    (root / 'data.yaml').write_text(yaml.safe_dump(dict(names=['target'])))
    plan = dict(schema_version=1, default_action='include', expected_images=4,
                dataset_manifest_sha256=sha256(manifest),
                input_images_fingerprint=image_fingerprint(root, rows)[0],
                excluded_near_duplicates={'image_1.jpg': 'image_0.jpg'},
                hold_for_visual_review={'image_2.jpg': 'ambiguous extent'},
                attention={'motion_blur_keep': ['image_3.jpg']})
    path = root / 'plan.yaml'
    path.write_text(yaml.safe_dump(plan))
    return root, path, rows


def test_package_preserves_originals_and_creates_no_labels_or_review(tmp_path):
    root, plan, rows = fixture(tmp_path)
    original_manifest = (root / 'pool/manifest.json').read_bytes()
    original_fingerprint = image_fingerprint(root, rows)[0]
    out = root / 'annotation_v1'
    result = prepare(root, plan, out)
    assert result['included'] == 2 and result['held'] == 1
    assert result['excluded_near_duplicates'] == 1
    assert result['retained_attention']['motion_blur_keep'] == 1
    assert (root / 'pool/manifest.json').read_bytes() == original_manifest
    assert image_fingerprint(root, rows)[0] == original_fingerprint
    assert not list(root.rglob('*.review.json'))
    assert not (root / 'pool/labels').exists()
    with zipfile.ZipFile(out / 'images_for_cvat.zip') as stream:
        assert set(stream.namelist()) == {'image_0.jpg', 'image_3.jpg'}
        assert stream.testzip() is None
        assert 'classes.txt' not in stream.namelist()
    assert (out / 'hold/image_2.jpg').is_file()
    assert 'PENDING' in (out / 'human_review.csv').read_text()


def test_image_change_requires_new_visual_review(tmp_path):
    root, plan, rows = fixture(tmp_path)
    image = root / rows[0]['image']
    cv2.imwrite(str(image), np.ones((32, 32, 3), np.uint8))
    with pytest.raises(ValueError, match='bytes changed'):
        prepare(root, plan, root / 'annotation_v1')
    assert not (root / 'annotation_v1').exists()


def test_duplicate_cannot_reference_a_held_image(tmp_path):
    root, plan, rows = fixture(tmp_path)
    data = yaml.safe_load(plan.read_text())
    data['excluded_near_duplicates']['image_1.jpg'] = 'image_2.jpg'
    plan.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='remain included'):
        prepare(root, plan, root / 'annotation_v1')


def test_existing_output_not_overwritten(tmp_path):
    root, plan, rows = fixture(tmp_path)
    out = root / 'annotation_v1'
    out.mkdir()
    marker = out / 'human_work.txt'
    marker.write_text('keep existing annotation work')
    with pytest.raises(ValueError, match='Output exists'):
        prepare(root, plan, out)
    assert marker.read_text() == 'keep existing annotation work'
