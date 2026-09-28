"""Tests for blue-surface 3D geometry and plane selection."""

import numpy as np
import pytest

from go2_science_perception.blue_surface_center import (
    PlaneCandidate,
    expected_horizontal_normal,
    project_masked_depth,
    ransac_multiple_planes,
    robust_center,
    select_horizontal_plane,
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


def test_plane_selection_uses_mount_prior_not_largest_plane():
    expected = expected_horizontal_normal(45.0)
    large_wrong = PlaneCandidate(
        normal=np.array([1.0, 0.0, 0.0]),
        offset=0.0,
        indices=np.arange(500),
        median_residual_m=0.001,
    )
    smaller_top = PlaneCandidate(
        normal=expected,
        offset=-1.0,
        indices=np.arange(100),
        median_residual_m=0.003,
    )
    selected, error = select_horizontal_plane(
        [large_wrong, smaller_top], 45.0, 15.0
    )
    assert selected is smaller_top
    assert error == pytest.approx(0.0)


def test_plane_selection_rejects_wrong_normal():
    candidate = PlaneCandidate(
        normal=np.array([1.0, 0.0, 0.0]),
        offset=0.0,
        indices=np.arange(100),
        median_residual_m=0.001,
    )
    selected, error = select_horizontal_plane([candidate], 45.0, 20.0)
    assert selected is None
    assert error == pytest.approx(90.0)


def test_ransac_extracts_multiple_planes():
    rng = np.random.default_rng(3)
    xy = rng.uniform(-0.2, 0.2, size=(180, 2))
    horizontal = np.column_stack((xy[:, 0], xy[:, 1], np.ones(180)))
    yz = rng.uniform(-0.2, 0.2, size=(140, 2))
    vertical = np.column_stack((np.full(140, 0.3), yz[:, 0], yz[:, 1] + 1.0))
    points = np.vstack((horizontal, vertical))
    candidates = ransac_multiple_planes(
        points,
        max_planes=3,
        iterations=100,
        distance_threshold_m=0.002,
        min_inliers=80,
        rng=np.random.default_rng(8),
    )
    assert len(candidates) >= 2
    sizes = sorted(candidate.indices.size for candidate in candidates)
    assert sizes[-2] >= 135
    assert sizes[-1] >= 175


def test_robust_center_uses_component_median():
    points = np.array(
        [[0.0, 0.0, 1.0], [0.1, 0.2, 1.1], [50.0, 50.0, 50.0]]
    )
    assert robust_center(points) == pytest.approx([0.1, 0.2, 1.1])
