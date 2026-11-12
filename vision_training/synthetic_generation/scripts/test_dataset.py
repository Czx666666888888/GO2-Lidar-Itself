import json
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image
import yaml
from check_dataset import check


class DatasetTest(unittest.TestCase):
    def fixture(self, root):
        rows=[]
        for split, scene in [('train','sceneA'),('val','sceneB')]:
            for sub in ('images','labels','masks'):
                (root/sub/split).mkdir(parents=True)
            name=scene+'_v0'
            Image.new('RGB',(20,10)).save(root/'images'/split/(name+'.png'))
            mask=np.zeros((10,20),dtype=np.uint16)
            mask[2:6,15:20]=7
            np.save(root/'masks'/split/(name+'.npy'),mask)
            (root/'labels'/split/(name+'.txt')).write_text('0 0.875 0.4 0.25 0.4\n')
            ann=dict(class_id=0,instance_id=7,bbox_xyxy=[15,2,20,6],visible_pixels=20,visible_fraction=0.8)
            rows.append(dict(id=name,scene_id=scene,split=split,source_type='synthetic',primary_class=0,annotations=[ann]))
        manifest=dict(config=dict(render=dict(min_pixels=4,min_visible_fraction=0.3)),records=rows)
        (root/'manifest.json').write_text(json.dumps(manifest))
        (root/'COMPLETE').write_text('2\n')
        return manifest

    def test_masks_split_import_provenance_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'synthetic'
            root.mkdir()
            self.fixture(root)
            report=check(root)
            self.assertEqual(report['images'],2)
            pool=Path(temp)/'pool_dataset'
            pool.mkdir()
            (pool/'data.yaml').write_text(yaml.safe_dump(dict(names=['color_block','rock_compact','rock_spiky'])))
            command=[sys.executable,str(Path(__file__).with_name('import_pool.py')),
                     '--source',str(root),'--dataset',str(pool)]
            result=subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            imported=json.loads((pool/'pool/manifest.json').read_text())
            self.assertTrue(all(r['source_type']=='synthetic' and not r['reviewed'] for r in imported))
            self.assertEqual({r['synthetic_split'] for r in imported},{'train','val'})
            before=(pool/'pool/manifest.json').read_bytes()
            self.assertNotEqual(subprocess.run(command,capture_output=True).returncode,0)
            self.assertEqual((pool/'pool/manifest.json').read_bytes(),before)

    def test_sampling_metadata_and_config_hash_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            manifest=self.fixture(root)
            manifest['config']['camera']=dict(distance_bands=dict(far=dict(range_m=[1.5,2.5])))
            for row in manifest['records']:
                row.update(sampling_request=dict(distance_band='far'),distance_m=2.0,
                           image_placement=dict(visibility='partial',side='right',projected_bbox_unclipped=[15,2,22,6]),
                           camera=dict(depression_deg=[25,65]),depression_deg=45)
            manifest['run']=dict(config_sha256=hashlib.sha256(yaml.safe_dump(manifest['config'],sort_keys=True).encode()).hexdigest())
            (root/'manifest.json').write_text(json.dumps(manifest))
            check(root)
            manifest['records'][0]['distance_m']=0.5
            (root/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(AssertionError,'Distance outside requested band'):
                check(root)
            manifest['config']['camera']['distance_bands']['far']['range_m']=[0.4,2.5]
            (root/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(AssertionError,'Config hash mismatch'):
                check(root)

    def test_leakage_and_wrong_box_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            manifest=self.fixture(root)
            manifest['records'][1]['scene_id']='sceneA'
            (root/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(AssertionError,'Scene leakage'):
                check(root)
            manifest['records'][1]['scene_id']='sceneB'
            (root/'manifest.json').write_text(json.dumps(manifest))
            (root/'labels/train/sceneA_v0.txt').write_text('0 0.5 0.5 0.2 0.2\n')
            with self.assertRaises(AssertionError):
                check(root)


if __name__=='__main__':
    unittest.main()
