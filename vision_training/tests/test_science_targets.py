"""Focused three-class migration, CVAT review and source-session checks."""
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile

from PIL import Image
import pytest
import yaml

from vision_training.common import sha256, write_json
from vision_training.dataset import validate_label
from vision_training.tools.dataset import import_rgbd, pool_manifest
from vision_training.tools.import_cvat import import_cvat
from vision_training.train import training_plan

ROOT = Path(__file__).resolve().parents[2]
NAMES = {0: 'color_block', 1: 'rock_compact', 2: 'rock_spiky'}


def pool(tmp):
    (tmp / 'data.yaml').write_text(yaml.safe_dump(dict(names=NAMES)))
    rows = []
    for name in ('mixed', 'negative', 'pending'):
        image = tmp / f'pool/images/session_A/{name}.png'
        image.parent.mkdir(parents=True, exist_ok=True)
        Image.new('RGB', (20, 20)).save(image)
        rows.append(dict(image=str(image.relative_to(tmp)), group='session_A',
                         source='camera', frame_index=len(rows), reviewed=False))
    write_json(tmp / 'pool/manifest.json', rows)
    return rows


@pytest.mark.parametrize('missing_negative', [False, True])
def test_cvat_mixed_boxes_explicit_negative_and_unlabelled(tmp_path, missing_negative):
    pool(tmp_path)
    archive = tmp_path / 'export.zip'
    with zipfile.ZipFile(archive, 'w') as z:
        z.writestr('data.yaml', yaml.safe_dump(dict(names=NAMES)))
        z.writestr('labels/train/mixed.txt', '0 .2 .2 .1 .1\n1 .5 .5 .2 .2\n2 .8 .8 .1 .1\n')
        if not missing_negative:
            z.writestr('labels/train/negative.txt', '')
        # The exported list is deliberately larger than the real label set.
        z.writestr('train.txt', 'mixed.png\nnegative.png\npending.png\n')
    reviews = tmp_path / 'review.csv'
    reviews.write_text('image,status\nmixed.png,positive\nnegative.png,negative\n'
                       'pending.png,pending\n')
    if missing_negative:
        with pytest.raises(ValueError, match='Missing/ambiguous label'):
            import_cvat(tmp_path, archive, reviews, 'TEST_ONLY')
        assert not (tmp_path / 'pool/labels').exists()
    else:
        assert import_cvat(tmp_path, archive, reviews, 'TEST_ONLY') == 2
        assert len(validate_label(tmp_path / 'pool/labels/session_A/mixed.txt', 3)) == 3
        assert (tmp_path / 'pool/labels/session_A/negative.txt').read_bytes() == b''
        assert not (tmp_path / 'pool/labels/session_A/pending.txt').exists()
        rows = pool_manifest(tmp_path)
        assert [r['reviewed'] for r in rows] == [True, True, False]
        assert rows[0]['frame_index'] == 0 and rows[0]['source'] == 'camera'


def test_real_legacy_mapping_and_formal_config_are_not_training_ready():
    target = ROOT / 'vision_training/datasets/science_targets_v1'
    source = ROOT / 'vision_training/datasets/blue_cube_pilot_v1'
    assert yaml.safe_load((target / 'data.yaml').read_text())['names'] == NAMES
    report = yaml.safe_load((target / 'legacy_mapping.yaml').read_text())
    assert report['images'] == 20
    # Portable report is tracked, actual media are required for this local audit.
    if not (source / 'images').exists():
        pytest.skip('Historical media are local and excluded from Git')
    for row in report['records']:
        original = (source / 'labels' / row['source_split'] /
                    Path(row['image']).with_suffix('.txt').name)
        assert sha256(original) == row['source_label_sha256']
        assert sha256(target / row['image']) == row['image_sha256']
        assert row['class_mapping'][0]['target'] == 'color_block'
        assert not row['reviewed']
    assert len({r['group'] for r in report['records']}) == 1
    with pytest.raises(ValueError, match='Expected image directory|Empty split'):
        training_plan(ROOT / 'vision_training/configs/science_targets_v1.yaml')


def test_capture_rgb_import_keeps_session_sidecar_and_no_labels(tmp_path):
    root, session = tmp_path / 'data', tmp_path / 'session_A'
    root.mkdir()
    sample = session / '000000_123'
    sample.mkdir(parents=True)
    Image.new('RGB', (20, 20)).save(sample / 'rgb.png')
    write_json(session / 'session.json', dict(session_id='session_A'))
    write_json(sample / 'sample.json', dict(
        rgb_file='rgb.png', frame_index=3, rgb_stamp_ns=123, scene='lit_cave',
        session_id='session_A', source_video=None, geometry_usable=False))
    import_rgbd(SimpleNamespace(dataset=str(root), session=str(session), group='A', tags=[]))
    row = json.loads((root / 'pool/manifest.json').read_text())[0]
    assert row['frame_index'] == 3 and row['rgb_stamp_ns'] == 123
    assert row['scene'] == 'lit_cave' and row['session_id'] == 'session_A'
    assert Path(row['rgbd_sidecar']) == sample / 'sample.json'
    assert not row['reviewed'] and not (root / 'pool/labels').exists()


def test_classic_cvat_blue_alias_preserves_original_bytes(tmp_path):
    pool(tmp_path)
    archive = tmp_path / 'classic.zip'
    original = b'0 .2 .2 .1 .1\n'
    with zipfile.ZipFile(archive, 'w') as z:
        z.writestr('obj.names', 'blue_cube\n')
        z.writestr('obj_train_data/mixed.txt', original)
    reviews = tmp_path / 'review.csv'
    reviews.write_text('image,status\nmixed.png,positive\n')
    aliases = tmp_path / 'alias.yaml'
    aliases.write_text('blue_cube: color_block\n')
    assert import_cvat(tmp_path, archive, reviews, 'TEST_ONLY', aliases) == 1
    row = pool_manifest(tmp_path)[0]
    assert row['class_mapping']['0']['target_id'] == 0
    assert row['class_mapping']['0']['source'] == 'blue_cube'
    assert next((tmp_path / 'pool/original_cvat').rglob('mixed.txt')).read_bytes() == original


def test_budget_uses_actual_reviewed_three_class_counts(tmp_path):
    from vision_training.tests.test_workflow import dataset, receipt
    from vision_training.tools.training_budget import budget
    data = dataset(tmp_path, list(NAMES.values()))
    receipt(data)
    config = yaml.safe_load((ROOT / 'vision_training/configs/science_targets_v1.yaml').read_text())
    config['data'] = str(data)
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(config))
    plan = budget(path)
    assert plan['planned_batches_per_epoch'] == 1
    assert plan['initial_accumulate'] == 1 and plan['warmup_batch_boundary'] == 100
    assert plan['images'] == {'train': 1, 'val': 1, 'test': 1}
    assert plan['missing_classes']['train'] == [0, 1]
