"""Round-two mapping and missing-label semantics are independent of class order."""
import csv
import json
import zipfile

import pytest
import yaml

from vision_training.tools.prepare_science_mixed_v2 import inventory
from vision_training.tools.import_cvat import import_cvat


def test_swapped_ids_and_pending_not_negative(tmp_path):
    records=[dict(filename='a.jpg'),dict(filename='b.jpg')]
    archive=tmp_path/'export.zip'
    with zipfile.ZipFile(archive,'w') as z:
        z.writestr('data.yaml',yaml.safe_dump(dict(names=['color_block','rock_spiky','rock_compact'])))
        z.writestr('train.txt','images/a.jpg\nimages/b.jpg\n')
        z.writestr('labels/train/a.txt','1 0.5 0.5 0.2 0.2\n2 0.2 0.2 0.1 0.1\n')
    expected,labels,boxes,mapping=inventory(archive,records)
    assert mapping=={0:0,1:2,2:1}
    assert set(expected)-set(labels)=={'b'}
    assert len(boxes['a'])==2
    root=tmp_path/'poolroot';(root/'pool/images/group').mkdir(parents=True)
    (root/'data.yaml').write_text(yaml.safe_dump(dict(names=['color_block','rock_compact','rock_spiky'])))
    rows=[]
    for name in ['a','b']:
        path=root/f'pool/images/group/{name}.jpg';path.write_bytes(b'unchanged')
        rows.append(dict(image=str(path.relative_to(root)),reviewed=False))
    (root/'pool/manifest.json').write_text(json.dumps(rows))
    review=tmp_path/'review.csv'
    with review.open('w') as f:
        w=csv.writer(f);w.writerow(['image','status']);w.writerow(['a.jpg','positive']);w.writerow(['b.jpg','pending'])
    assert import_cvat(root,archive,review,'test_manual_review')==1
    text=(root/'pool/labels/group/a.txt').read_text()
    assert text.splitlines()==['2 0.5 0.5 0.2 0.2','1 0.2 0.2 0.1 0.1']
    assert not (root/'pool/labels/group/b.txt').exists()
    assert json.loads((root/'pool/manifest.json').read_text())[1]['reviewed'] is False


def test_mismatched_export_rejected(tmp_path):
    archive=tmp_path/'export.zip'
    with zipfile.ZipFile(archive,'w') as z:
        z.writestr('data.yaml',yaml.safe_dump(dict(names=['color_block','rock_spiky','rock_compact'])))
        z.writestr('train.txt','images/other.jpg\n')
    with pytest.raises(ValueError,match='differs'):
        inventory(archive,[dict(filename='a.jpg')])


def test_legacy_subgroup_counts_preserve_false_positives(tmp_path):
    from PIL import Image
    from vision_training.common import sha256
    from vision_training.tools.compare_science_v2 import enrich_legacy
    (tmp_path/'images/val').mkdir(parents=True)
    (tmp_path/'labels/val').mkdir(parents=True)
    im=tmp_path/'images/val/a.jpg';Image.new('RGB',(640,640)).save(im)
    label=tmp_path/'labels/val/a.txt'
    label.write_text('0 0.5 0.5 0.01 0.01\n2 0.2 0.2 0.1 0.1\n')
    (tmp_path/'preparation.json').write_text(json.dumps(dict(records=[dict(
        image='images/val/a.jpg',image_sha256=sha256(im),label_sha256=sha256(label))])))
    metrics=dict(cases=[dict(image='a.jpg',gt=2,tp=1,fp=1,fn=1,
        missed_classes=[0],false_classes=[1],missed_gt=[[0,316.8,316.8,323.2,323.2]])])
    result=enrich_legacy(metrics,tmp_path)
    assert result['fixed_conf_per_class']['color_block']==dict(tp=0,fp=0,fn=1)
    assert result['fixed_conf_per_class']['rock_compact']==dict(tp=0,fp=1,fn=0)
    assert result['small_short_side_lt16_at_640']['color_block']==dict(tp=0,fn=1)
    assert result['multi_object_counts']==dict(images=1,tp=1,fp=1,fn=1)
