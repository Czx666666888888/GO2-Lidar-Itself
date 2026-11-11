"""Numeric RGB-D round-trip and real source-stamp association, without a camera."""
import json
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest

from go2_science_perception.rgbd_capture import nearest, sample_metadata, save_sample


def message(sec, ns=0, encoding='16UC1', frame='color_optical'):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=ns), frame_id=frame),
              height=3, width=4, encoding=encoding)


@pytest.mark.parametrize('encoding,dtype,scale', [
    ('16UC1', np.uint16, .001), ('32FC1', np.float32, 1.0)])
def test_timestamp_pair_and_lossless_round_trip(tmp_path, encoding, dtype, scale):
    rgb = message(10, 0, 'rgb8')
    depths = [message(11, encoding=encoding), message(9, 980_000_000, encoding)]
    depth_msg = nearest(depths, 10_000_000_000)
    info = message(10)
    info.k = [100., 0., 2., 0., 100., 1., 0., 0., 1.]
    info.d, info.r, info.p = [], [0.]*9, [0.]*12
    info.distortion_model = 'plumb_bob'
    info.binning_x = info.binning_y = 0
    info.roi = NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False)
    config = dict(scene='TEST_ONLY', color_topic='rgb', depth_topic='aligned',
                  max_rgb_depth_delta_sec=.03, max_info_delta_sec=.1)
    meta = sample_metadata(rgb, depth_msg, info, config, 'session_A', 4)
    assert meta['geometry_usable'] and meta['rgb_depth_delta_sec'] == pytest.approx(.02)
    assert meta['camera_info']['stamp_ns'] == meta['rgb_stamp_ns']
    assert meta['depth_scale_to_m'] == scale
    color = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)
    depth = np.arange(12, dtype=dtype).reshape(3, 4)
    if dtype == np.float32:
        depth[0, 0] = np.nan
    folder = tmp_path / 'sample'
    save_sample(folder, color, depth, meta)
    np.testing.assert_array_equal(cv2.imread(str(folder / 'rgb.png')), color)
    np.testing.assert_array_equal(np.load(folder / 'depth.npy', allow_pickle=False), depth)
    assert np.load(folder / 'depth.npy').dtype == dtype
    saved = json.loads((folder / 'sample.json').read_text())
    assert saved['session_id'] == 'session_A' and saved['frame_index'] == 4
    config['max_rgb_depth_delta_sec'] = .01
    assert 'rgb_depth_time_mismatch' in sample_metadata(
        rgb, depth_msg, info, config, 'A', 4)['flags']
    depth_msg.width = 8
    assert not sample_metadata(rgb, depth_msg, info, config, 'A', 4)['geometry_usable']
    missing = sample_metadata(rgb, None, None, config, 'A', 5)
    assert not missing['geometry_usable'] and missing['depth_stamp_ns'] is None
