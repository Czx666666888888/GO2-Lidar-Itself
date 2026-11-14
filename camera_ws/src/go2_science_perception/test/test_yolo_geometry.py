"""Depth reliability failures must suppress 3D without depending on class names."""

import numpy as np
import pytest

from go2_science_perception.yolo_geometry import back_project, reliable_box_depth


def test_depth_holes_and_mixed_layers_rejected():
    depth = np.zeros((40, 40), dtype=np.float32)
    assert not reliable_box_depth(depth, [0, 0, 40, 40])['valid']
    depth[:] = 1.0
    depth[:, 20:] = 3.0
    assert reliable_box_depth(depth, [0, 0, 40, 40])['reason'] == 'mixed_or_noisy_depth'


def test_valid_central_roi_and_projection():
    depth = np.ones((40, 40), dtype=np.float32)*2
    depth[:5] = np.nan
    out = reliable_box_depth(depth, [0, 0, 40, 40])
    assert out['valid'] and out['depth_m'] == 2
    k = [100, 0, 20, 0, 100, 20, 0, 0, 1]
    assert back_project(25, 30, 2, k) == pytest.approx([.1, .2, 2])
    with pytest.raises(ValueError):
        back_project(25, 30, 2, k, model='unsupported')


def test_edge_box_and_invalid_box_do_not_index_outside_image():
    depth = np.ones((40, 40), dtype=np.float32)
    out = reliable_box_depth(depth, [-5, -5, 15, 15])
    assert out['valid'] and out['roi_xyxy'][0] >= 0
    assert not reliable_box_depth(depth, [15, 15, 5, 5])['valid']


def test_depth_projection_uses_a_measured_pixel_when_box_center_is_a_hole():
    depth = np.ones((40,40),dtype=np.float32)
    depth[18:23,18:23] = 0
    result = reliable_box_depth(depth,[0,0,40,40])
    assert result['valid']
    assert depth[int(result['v']),int(result['u'])] > 0
