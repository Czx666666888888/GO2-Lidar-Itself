"""Dataset identity, leakage and reusable class/config tests; no network/GPU."""

from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml

from vision_training.common import check_model_names, write_json
from vision_training.dataset import inspect_dataset, validate_label
from vision_training.tools.dataset import import_images, review_group, split_groups
from vision_training.train import training_plan


def dataset(tmp_path, names):
    root = tmp_path / 'data'
    root.mkdir()
    for i, split in enumerate(('train', 'val', 'test')):
        (root / 'images' / split).mkdir(parents=True)
        (root / 'labels' / split).mkdir(parents=True)
        image = root / 'images' / split / 'image.png'
        Image.fromarray(np.full((32, 32, 3), i*50, dtype=np.uint8)).save(image)
        (root / 'labels' / split / 'image.txt').write_text(
            f'{len(names)-1} 0.5 0.5 0.5 0.5\n')
    path = root / 'data.yaml'
    path.write_text(yaml.safe_dump(dict(path='.', train='images/train', val='images/val',
                                        test='images/test', names=names)))
    return path


def receipt(path):
    _, manifest = inspect_dataset(path)
    write_json(path.with_suffix('.review.json'), dict(
        reviewer='UNIT_TEST_FIXTURE_ONLY', fingerprint=manifest['fingerprint']))


@pytest.mark.parametrize('names', [['object_a'], ['marker', 'tool']])
def test_same_training_code_accepts_one_or_multiple_classes(tmp_path, names):
    path = dataset(tmp_path, names)
    receipt(path)
    config = tmp_path / 'train.yaml'
    config.write_text(yaml.safe_dump(dict(
        project='competition_target', name='v1', data=str(path), model='yolo11n.pt',
        output='runs', imgsz=640, epochs=2, batch=2, device='cpu', patience=1)))
    cfg, data, manifest, output, weights, args = training_plan(config)
    assert list(data['names'].values()) == names
    assert len(manifest['entries']) == 3
    assert output == tmp_path / 'runs/competition_target/v1'
    assert weights.name == 'yolo11n.pt'


@pytest.mark.parametrize('text', ['1 0.5 0.5 0.1 0.1', '0 .1 .5 .5 .2',
                                  '0 nan .5 .2 .2', '0 .5 .5 0 .2', '0.5 .5 .5 .2 .2'])
def test_invalid_labels_rejected(tmp_path, text):
    p = tmp_path / 'label.txt'
    p.write_text(text)
    with pytest.raises(ValueError):
        validate_label(p, 1)


def test_empty_reviewed_negative_allowed_but_missing_label_rejected(tmp_path):
    p = tmp_path / 'negative.txt'
    with pytest.raises(ValueError):
        validate_label(p, 1)
    p.write_text('')
    assert validate_label(p, 1) == []


def test_cross_split_duplicate_and_orphan_label_rejected(tmp_path):
    path = dataset(tmp_path, ['object'])
    root = path.parent
    (root / 'labels/train/orphan.txt').write_text('')
    with pytest.raises(ValueError, match='Orphan'):
        inspect_dataset(path)
    (root / 'labels/train/orphan.txt').unlink()
    (root / 'images/test/image.png').write_bytes((root / 'images/train/image.png').read_bytes())
    with pytest.raises(ValueError, match='Cross-split'):
        inspect_dataset(path)


def test_review_receipt_required_and_invalidated_by_label_change(tmp_path):
    path = dataset(tmp_path, ['object'])
    with pytest.raises(ValueError, match='Human review'):
        inspect_dataset(path, require_review=True)
    receipt(path)
    inspect_dataset(path, require_review=True)
    (path.parent / 'labels/test/image.txt').write_text('0 .5 .5 .2 .2')
    with pytest.raises(ValueError, match='stale'):
        inspect_dataset(path, require_review=True)


def test_group_assignment_cannot_leak_or_accept_unreviewed_pool(tmp_path):
    config = tmp_path / 'split.yaml'
    config.write_text(yaml.safe_dump(dict(train=['A'], val=['A'], test=['B'])))
    args = SimpleNamespace(dataset=str(tmp_path), assignments=str(config))
    with pytest.raises(ValueError, match='leaks'):
        split_groups(args)
    config.write_text(yaml.safe_dump(dict(train=['A'], val=['B'], test=['C'])))
    (tmp_path / 'pool').mkdir()
    (tmp_path / 'data.yaml').write_text(yaml.safe_dump(dict(names=['object'])))
    write_json(tmp_path / 'pool/manifest.json', [
        dict(group=g, reviewed=False, image=f'pool/images/{g}/x.png')
        for g in ('A', 'B', 'C')])
    with pytest.raises(ValueError, match='Unreviewed'):
        split_groups(args)


def test_checkpoint_class_mapping_cannot_silently_change():
    model = SimpleNamespace(names={0: 'marker', 1: 'tool'})
    assert check_model_names(model, ['marker', 'tool']) == model.names
    with pytest.raises(ValueError, match='mismatch'):
        check_model_names(model, ['tool', 'marker'])


def test_import_review_split_preserves_provenance_and_tagged_subset(tmp_path):
    data = dataset(tmp_path, ['marker', 'tool'])
    root = tmp_path / 'prepared'
    root.mkdir()
    (root / 'data.yaml').write_text(data.read_text())
    for split, group in zip(('train', 'val', 'test'), ('A', 'B', 'C')):
        args = SimpleNamespace(dataset=str(root), images=str(data.parent / 'images' / split),
                               labels=str(data.parent / 'labels' / split), group=group,
                               tags=['low_light'] if group == 'C' else [])
        import_images(args)
        args.reviewer = 'UNIT_TEST_FIXTURE_ONLY'
        review_group(args)
    assignment = tmp_path / 'assignment.yaml'
    assignment.write_text(yaml.safe_dump(dict(train=['A'], val=['B'], test=['C'])))
    split_groups(SimpleNamespace(dataset=str(root), assignments=str(assignment)))
    _, manifest = inspect_dataset(root / 'data.yaml', require_review=True)
    assert len(manifest['entries']) == 3
    assert (root / 'subsets/test_low_light.txt').is_file()
    assert not (root / 'subsets/test_motion_blur.txt').exists()
    changed = yaml.safe_load((root / 'data.yaml').read_text())
    changed['names'] = ['tool', 'marker']
    (root / 'data.yaml').write_text(yaml.safe_dump(changed))
    with pytest.raises(ValueError, match='Unreviewed'):
        split_groups(SimpleNamespace(dataset=str(root), assignments=str(assignment)))


def test_pilot_is_explicit_and_formal_workflow_stays_strict(tmp_path):
    path = dataset(tmp_path, ['object'])
    data = yaml.safe_load(path.read_text())
    del data['test']
    path.write_text(yaml.safe_dump(data))
    _, manifest = inspect_dataset(path, splits=('train', 'val'))
    write_json(path.with_suffix('.review.json'), dict(
        reviewer='UNIT_TEST_FIXTURE_ONLY', fingerprint=manifest['fingerprint']))
    config = tmp_path / 'pilot.yaml'
    cfg = dict(project='pilot', name='v1', data=str(path), model='yolo11n.pt',
               output='runs', imgsz=640, epochs=50, batch=4, device=0, patience=15)
    config.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match='Missing split: test'):
        training_plan(config)
    plan = training_plan(config, pilot=True)
    assert plan[0]['validation_protocol'] == 'pilot_train_val_only'
    assert len(plan[2]['entries']) == 2
    data['test'] = 'images/test'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='must not declare'):
        training_plan(config, pilot=True)
