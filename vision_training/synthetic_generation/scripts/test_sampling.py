import copy
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml
from sampling import (camera_rotation, code_hashes, load_camera, placement_matches,
                      projected_bounds, sample_image_placement, stable_rest_poses,
                      support_stability, validate_config, weighted_choice)
from labels import visible_annotations

CONFIG = Path(__file__).resolve().parents[1]/'configs/default.yaml'


def cube():
    return np.array([[x,y,z] for x in (-0.025,0.025) for y in (-0.025,0.025) for z in (-0.025,0.025)])


class SamplingTest(unittest.TestCase):
    def setUp(self):
        self.c = yaml.safe_load(CONFIG.read_text())
        self.k = np.array(self.c['camera']['K']).reshape(3,3)
        self.rng = np.random.default_rng(7)

    def test_config_invariants_and_literal_bands(self):
        validate_config(self.c)
        self.assertEqual([self.c[x] for x in ['rock_compact_scale','rock_spiky_scale']], [1.0,1.0])
        self.assertFalse(self.c['physical_scale_confirmed'])
        self.assertIsNone(self.c['block']['colors']['color_6_pending'])
        for name, expected in [('near',[0.4,0.8]),('medium',[0.8,1.5]),('far',[1.5,2.5])]:
            self.assertEqual(self.c['camera']['distance_bands'][name]['range_m'],expected)
            entries=copy.deepcopy(self.c['camera']['distance_bands'])
            for key in entries: entries[key]['weight']=int(key==name)
            self.assertTrue(all(weighted_choice(self.rng,entries)==name for _ in range(20)))
        bad=copy.deepcopy(self.c)
        bad['camera']['distance_bands']['far']['range_m']=[2.5,1.5]
        with self.assertRaises(ValueError): validate_config(bad)

    def test_projection_exactly_hits_requested_pixel(self):
        position=np.array([0.,-0.5,0.4])
        for uv in ([0.5,0.5],[0.12,0.5],[0.88,0.5],[0.5,0.05],[0.5,0.95],[-0.01,0.5]):
            rotation=camera_rotation(-position,self.k,uv,640,480)
            bounds=projected_bounds(np.zeros((1,3)),position,rotation,self.k)
            np.testing.assert_allclose(bounds[:2],np.array(uv)*[640,480],atol=1e-9)
            np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(rotation),1)

    def test_all_regions_and_clipped_mask_boxes(self):
        position=np.array([0.,-0.5,0.4])
        for region,settings in self.c['camera']['image_regions'].items():
            accepted=0
            for _ in range(12):
                _,placement=sample_image_placement(self.rng,region,settings,cube(),np.zeros(3),position,self.k,640,480)
                bounds=np.array(placement['projected_bbox_unclipped'])
                mask=np.zeros((480,640),dtype=np.uint8)
                x0,y0,x1,y1=np.rint(bounds).astype(int)
                mask[max(0,y0):min(480,y1),max(0,x0):min(640,x1)]=7
                if not placement_matches(placement,mask,7,640,480):
                    self.assertEqual(placement['visibility'],'full')
                    self.assertTrue(bounds[0]<1 or bounds[1]<1 or bounds[2]>639 or bounds[3]>479)
                    continue
                accepted += 1
                if region=='near_edge_full':
                    gap={'left':bounds[0],'right':640-bounds[2],'top':bounds[1],'bottom':480-bounds[3]}[placement['side']]
                    self.assertAlmostEqual(gap,placement['requested_gap_px'],delta=0.06)
                annotations,rejected=visible_annotations(mask,[dict(idx=7,name='target')],
                    dict(target=dict(class_id=0)),dict(target=int((mask==7).sum())),self.c['render'])
                self.assertFalse(rejected)
                box=annotations[0]['bbox_xyxy']
                self.assertTrue(0<=box[0]<box[2]<=640 and 0<=box[1]<box[3]<=480)
            self.assertGreater(accepted,0,region)
        invisible=np.zeros((480,640),dtype=np.uint8)
        self.assertFalse(placement_matches(dict(visibility='partial',side='left',projected_bbox_unclipped=[-50,10,-10,30]),invisible,None,640,480))

    def test_far_tiny_and_thin_visible_targets_rejected(self):
        rotation=camera_rotation([0,1,0],self.k,[0.5,0.5],640,480)
        bounds=projected_bounds(cube()*0.3,np.array([0,-2.5,0]),rotation,self.k)
        self.assertLess(min(bounds[2:]-bounds[:2]),self.c['render']['min_bbox_side_px'])
        mask=np.zeros((100,100),dtype=np.uint8)
        mask[10:90,10:12]=4  # area 160 but only two pixels wide
        annotations,rejected=visible_annotations(mask,[dict(idx=4,name='rock')],
            dict(rock=dict(class_id=1)),dict(rock=160),self.c['render'])
        self.assertFalse(annotations)
        self.assertEqual(rejected[0]['reason'],'bbox_side_too_small')

    def test_camera_info_and_helper_hashes(self):
        self.assertEqual(load_camera(self.c['camera'])['source'],'synthetic_camera')
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'info.yaml'
            info=dict(width=640,height=480,k=self.c['camera']['K'],d=[0]*5)
            path.write_text(yaml.safe_dump(info))
            camera=load_camera(dict(self.c['camera'],camera_info=str(path)))
            self.assertEqual(camera['source'],'d435_real_camera_info')
            self.assertEqual(camera['K'],info['k'])
            self.assertTrue(camera['camera_info_sha256'])
            info['d'][0]=0.1
            path.write_text(yaml.safe_dump(info))
            with self.assertRaises(ValueError): load_camera(dict(self.c['camera'],camera_info=str(path)))
            script=Path(temp)/'sampling.py'
            script.write_text('before')
            old=code_hashes(temp)
            script.write_text('after')
            self.assertNotEqual(old,code_hashes(temp))

    def test_resting_poses_and_slope_support(self):
        import trimesh
        mesh=trimesh.creation.box(extents=[0.05]*3)
        poses,com=stable_rest_poses(mesh.vertices,mesh.faces,self.c['rock_poses']['rock_compact'])
        self.assertGreaterEqual(len(poses),2)
        for pose in poses:
            slope=np.deg2rad(6)
            cs,sn=np.cos(slope),np.sin(slope)
            rotate=np.array([[cs,0,sn],[0,1,0],[-sn,0,cs]])@pose['rotation']
            world=mesh.vertices@rotate.T
            z=-np.min(world[:,2]+world[:,0]*np.tan(slope))
            world[:,2]+=z
            world_com=rotate@com+[0,0,z]
            result=support_stability(world,world_com,slope,1e-6,0.00005)
            self.assertTrue(result['stable'],result)
        self.assertFalse(support_stability(np.array([[0,0,0],[0,0,1],[1,0,1]]),np.array([0,0,0.5]),0,1e-6,0)['stable'])
        world=cube()+[0,0,0.025]
        self.assertFalse(support_stability(world,np.array([0.2,0,0.025]),0,1e-6,0)['stable'])


if __name__=='__main__': unittest.main()
