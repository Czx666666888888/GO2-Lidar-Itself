"""Tests for mask-based coarse target localization."""

import numpy as np
import pytest

from go2_science_perception.coarse_target_locator import (
    back_project_pixel,
    erode_target_mask,
    largest_blue_mask,
    mask_centroid,
    median_mask_depth,
)


def test_largest_blue_mask_reuses_hsv_component_rule():
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    image[20:60, 30:80] = (255, 0, 0)
    raw, mask, contours, area = largest_blue_mask(
        image, (90, 80, 50), (135, 255, 255), 5, 1, 1, 100
    )
    assert np.count_nonzero(raw) == 2000
    assert area == 1992
    assert np.count_nonzero(mask) == 1992
    assert len(contours) == 1


def test_mask_centroid_uses_complete_target_mask():
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[4:14, 8:18] = 255
    assert mask_centroid(mask) == pytest.approx((12.5, 8.5))


def test_eroded_mask_depth_filters_invalid_values_and_uses_median():
    mask = np.zeros((7, 7), dtype=np.uint8)
    mask[1:6, 1:6] = 255
    eroded = erode_target_mask(mask, 3, 1)
    assert 0 < np.count_nonzero(eroded) < np.count_nonzero(mask)
    depth = np.zeros((7, 7), dtype=np.float32)
    depth[eroded > 0] = 1.5
    depth[3, 3] = np.nan
    depth_m, count = median_mask_depth(depth, eroded, 0.15, 5.0)
    assert depth_m == pytest.approx(1.5)
    assert count == np.count_nonzero(eroded) - 1


def test_back_project_pixel_uses_runtime_camera_info_intrinsics():
    point = back_project_pixel(12.0, 8.0, 2.0, (4.0, 8.0, 10.0, 4.0))
    np.testing.assert_allclose(point, [1.0, 1.0, 2.0])


def test_empty_mask_centroid_and_depth_are_rejected():
    mask = np.zeros((3, 3), dtype=np.uint8)
    with pytest.raises(ValueError):
        mask_centroid(mask)
    depth_m, count = median_mask_depth(
        np.zeros((3, 3), dtype=np.float32), mask, 0.15, 5.0
    )
    assert np.isnan(depth_m)
    assert count == 0
