import unittest
import numpy as np
from labels import visible_annotations


class LabelsTest(unittest.TestCase):
    def test_edges_background_and_class_zero(self):
        mask=np.zeros((10,20),dtype=int)
        mask[2:6,15:20]=7
        mask[0:2,0:2]=9  # ordinary rock, never labelled
        rows,rejected=visible_annotations(mask,[{'idx':7,'name':'block'},{'idx':9,'name':'rock'}],
            {'block':{'class_id':0}},{'block':25},{'min_pixels':4,'min_visible_fraction':0.3})
        self.assertEqual(rejected,[])
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['bbox_xyxy'],[15,2,20,6])
        self.assertEqual(rows[0]['class_id'],0)
        self.assertEqual(rows[0]['yolo'],[0.875,0.4,0.25,0.4])

    def test_occluded_and_missing_targets_rejected(self):
        mask=np.zeros((10,10),dtype=int)
        mask[1:3,1:3]=2
        rows,rejected=visible_annotations(mask,[{'idx':2,'name':'spiky'}],
            {'spiky':{'class_id':2},'compact':{'class_id':1}},
            {'spiky':20,'compact':0},{'min_pixels':3,'min_visible_fraction':0.3})
        self.assertEqual(rows,[])
        self.assertEqual(len(rejected),2)


if __name__=='__main__':
    unittest.main()
