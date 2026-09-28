"""Tests for blue-surface 3D geometry and plane selection."""

import numpy as np
import pytest

from go2_science_perception.blue_surface_center import (
    PlaneCandidate,
    expected_horizontal_normal,
    plane_normal_error_deg,
    project_masked_depth,
    ransac_dominant_plane,
    robust_center,
    top_region_mask,
)


def test_project_masked_depth_uses_runtime_intrinsics_and_valid_depth():
    depth = np.array([[1.0, 0.0], [2.0, np.nan]], dtype=np.float32)
    mask = np.full((2, 2), 255, dtype=np.uint8)
    points, pixels = project_masked_depth(
        depth,
        mask,
        (2.0, 4.0, 0.0, 0.0),
        0.1,
        3.0,
    )
    assert pixels.tolist() == [[0, 0], [0, 1]]
    np.testing.assert_allclose(points, [[0.0, 0.0, 1.0], [0.0, 0.5, 2.0]])


def test_plane_normal_error_uses_mount_pitch():
    expected = expected_horizontal_normal(45.0)
    top = PlaneCandidate(
        normal=expected,
        offset=-1.0,
        indices=np.arange(100),
        median_residual_m=0.003,
    )
    assert plane_normal_error_deg(top, 45.0) == pytest.approx(0.0)


def test_plane_normal_error_reports_wrong_normal():
    candidate = PlaneCandidate(
        normal=np.array([1.0, 0.0, 0.0]),
        offset=0.0,
        indices=np.arange(100),
        median_residual_m=0.001,
    )
    assert plane_normal_error_deg(candidate, 45.0) == pytest.approx(90.0)


def test_ransac_fits_one_dominant_plane_in_prefiltered_roi():
    rng = np.random.default_rng(3)
    xy = rng.uniform(-0.2, 0.2, size=(180, 2))
    horizontal = np.column_stack((xy[:, 0], xy[:, 1], np.ones(180)))
    outliers = rng.uniform(-1.0, 1.0, size=(20, 3))
    candidate = ransac_dominant_plane(
        np.vstack((horizontal, outliers)),
        iterations=100,
        distance_threshold_m=0.002,
        min_inliers=80,
        rng=np.random.default_rng(8),
    )
    assert candidate is not None
    assert candidate.indices.size >= 180
    assert abs(candidate.normal[2]) == pytest.approx(1.0, abs=1e-4)


def test_top_region_mask_keeps_upper_fraction_then_erodes():
    mask = np.zeros((10, 8), dtype=np.uint8)
    mask[2:8, 1:7] = 255
    top = top_region_mask(mask, 0.5, 3, 1)
    assert np.count_nonzero(top) > 0
    assert not np.any(top[5:])
    assert not np.any(top[:, 0])


def test_robust_center_uses_component_median():
    points = np.array(
        [[0.0, 0.0, 1.0], [0.1, 0.2, 1.1], [50.0, 50.0, 50.0]]
    )
    assert robust_center(points) == pytest.approx([0.1, 0.2, 1.1])
