"""Tests for multi-target coarse localization and blue-wall rejection."""

import numpy as np
import pytest

from go2_science_perception.coarse_target_locator import (
    back_project_pixel,
    blue_candidate_mask,
    detect_blue_wall,
    erode_target_mask,
    locate_targets,
    mask_centroid,
    median_mask_depth,
)


INTRINSICS = (100.0, 100.0, 80.0, 60.0)


def wall_result(depth, mask):
    """Run deterministic wall detection for a synthetic front-facing wall."""
    return detect_blue_wall(
        depth, mask, INTRINSICS, 0.15, 5.0,
        distance_threshold_m=0.01,
        ransac_iterations=60,
        min_inliers=200,
        min_support_ratio=0.55,
        min_image_fraction=0.40,
        mount_pitch_deg=0.0,
        max_vertical_error_deg=10.0,
        max_ransac_points=12000,
        rng=np.random.default_rng(7),
    )


def targets_after_wall(depth, mask, wall):
    """Locate synthetic targets after removing detected wall inliers."""
    remaining = mask.copy()
    remaining[wall.mask > 0] = 0
    return locate_targets(
        depth, remaining, INTRINSICS, 0.15, 5.0,
        min_component_area_px=40,
        min_valid_depth_points=20,
        cluster_depth_gap_m=0.08,
        min_depth_cluster_ratio=0.15,
        erosion_kernel_px=1,
        erosion_iterations=0,
    )


def test_hsv_mask_keeps_multiple_blue_components():
    """HSV extraction must preserve every blue component."""
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    image[10:25, 10:25] = (255, 0, 0)
    image[40:70, 60:90] = (255, 0, 0)
    raw, cleaned = blue_candidate_mask(
        image, (90, 80, 50), (135, 255, 255), 1, 0, 0
    )
    assert np.count_nonzero(raw) == 1125
    assert np.count_nonzero(cleaned) == 1125


def test_static_only_blue_wall_publishes_no_targets():
    """A supported vertical blue wall must leave zero targets."""
    mask = np.full((120, 160), 255, dtype=np.uint8)
    depth = np.full(mask.shape, 2.0, dtype=np.float32)
    wall = wall_result(depth, mask)
    assert wall.detected
    assert wall.support_ratio == pytest.approx(1.0)
    assert targets_after_wall(depth, mask, wall) == []


def test_static_blue_wall_plus_one_cube_returns_one_target():
    """One depth-separated blue cube must survive wall removal."""
    mask = np.full((120, 160), 255, dtype=np.uint8)
    depth = np.full(mask.shape, 2.0, dtype=np.float32)
    depth[45:70, 65:90] = 1.2
    wall = wall_result(depth, mask)
    targets = targets_after_wall(depth, mask, wall)
    assert wall.detected
    assert len(targets) == 1
    assert targets[0].centroid == pytest.approx((77.0, 57.0))
    assert targets[0].median_depth_m == pytest.approx(1.2)


def test_static_blue_wall_plus_different_size_cubes_returns_all_targets():
    """Differently sized targets must all survive without size priors."""
    mask = np.full((120, 160), 255, dtype=np.uint8)
    depth = np.full(mask.shape, 2.0, dtype=np.float32)
    depth[20:30, 20:30] = 1.0
    depth[65:85, 105:135] = 1.4
    wall = wall_result(depth, mask)
    targets = targets_after_wall(depth, mask, wall)
    assert wall.detected
    assert len(targets) == 2
    assert [target.valid_depth_points for target in targets] == [100, 600]
    assert [target.median_depth_m for target in targets] == pytest.approx([1.0, 1.4])


def test_connected_region_is_split_by_distinct_depth_layers():
    """A connected color region must split at a clear 3D depth gap."""
    mask = np.zeros((40, 50), dtype=np.uint8)
    mask[10:30, 10:40] = 255
    depth = np.zeros(mask.shape, dtype=np.float32)
    depth[10:30, 10:25] = 1.0
    depth[10:30, 25:40] = 1.5
    targets = locate_targets(
        depth, mask, (100.0, 100.0, 25.0, 20.0), 0.15, 5.0,
        40, 20, 0.08, 0.15, 1, 0
    )
    assert len(targets) == 2
    assert [target.median_depth_m for target in targets] == pytest.approx([1.0, 1.5])


def test_mask_centroid_depth_and_back_projection_helpers():
    """Centroid, median depth and pinhole projection must remain valid."""
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[4:14, 8:18] = 255
    assert mask_centroid(mask) == pytest.approx((12.5, 8.5))
    eroded = erode_target_mask(mask, 3, 1)
    depth = np.zeros(mask.shape, dtype=np.float32)
    depth[eroded > 0] = 1.5
    depth_m, count = median_mask_depth(depth, eroded, 0.15, 5.0)
    assert depth_m == pytest.approx(1.5)
    assert count == np.count_nonzero(eroded)
    point = back_project_pixel(12.0, 8.0, 2.0, (4.0, 8.0, 10.0, 4.0))
    np.testing.assert_allclose(point, [1.0, 1.0, 2.0])
