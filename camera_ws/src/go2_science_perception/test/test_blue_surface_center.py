"""Tests for blue-surface 3D geometry and plane selection."""

import numpy as np
import pytest

from go2_science_perception.blue_surface_center import (
    INVALID_REASONS,
    PlaneCandidate,
    aggregate_diagnostics,
    expected_horizontal_normal,
    plane_normal_error_deg,
    project_masked_depth,
    ransac_multiple_planes,
    robust_center,
    select_horizontal_plane,
)


def test_aggregate_diagnostics_reports_all_invalid_reasons_and_statistics():
    counts = {reason: 0 for reason in INVALID_REASONS}
    counts["bad_normal"] = 2
    counts["no_blue"] = 1
    lines = aggregate_diagnostics(
        10, 7, counts, [10.0, 20.0, 30.0], [0.5, 0.75]
    )
    assert lines[0] == "frames=10 valid=7 invalid=3 valid_ratio=0.700"
    assert "invalid_reason=no_blue count=1 ratio=0.100" in lines
    assert "invalid_reason=bad_normal count=2 ratio=0.200" in lines
    assert "normal_error_deg count=3 mean=20.000 median=20.000 max=30.000" in lines
    assert "inlier_ratio count=2 mean=0.625 median=0.625" in lines


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


def test_ransac_extracts_multiple_planes_from_complete_mask_cloud():
    rng = np.random.default_rng(3)
    yz = rng.uniform(-0.2, 0.2, size=(220, 2))
    side = np.column_stack((np.full(220, 0.3), yz[:, 0], yz[:, 1] + 1.0))
    xy = rng.uniform(-0.2, 0.2, size=(140, 2))
    top = np.column_stack((xy[:, 0], xy[:, 1], np.ones(140)))
    candidates = ransac_multiple_planes(
        np.vstack((side, top)),
        max_planes=4,
        iterations=100,
        distance_threshold_m=0.002,
        min_inliers=60,
        rng=np.random.default_rng(8),
    )
    assert len(candidates) == 2
    assert sorted(candidate.indices.size for candidate in candidates) == [140, 220]


def test_selection_rejects_largest_side_and_uses_horizontal_candidate():
    horizontal = expected_horizontal_normal(45.0)
    side = PlaneCandidate(
        np.array([1.0, 0.0, 0.0]), 0.0, np.arange(700), 0.001
    )
    top = PlaneCandidate(horizontal, 0.0, np.arange(200), 0.003)
    selected = select_horizontal_plane([side, top], 900, 45.0, 35.0)
    assert selected is top


def test_selection_prefers_support_then_residual_after_normal_filter():
    horizontal = expected_horizontal_normal(45.0)
    large = PlaneCandidate(horizontal, 0.0, np.arange(300), 0.006)
    small = PlaneCandidate(horizontal, 0.0, np.arange(200), 0.001)
    assert select_horizontal_plane([small, large], 500, 45.0, 35.0) is large
    equal_clean = PlaneCandidate(horizontal, 0.0, np.arange(300), 0.002)
    assert (
        select_horizontal_plane([large, equal_clean], 600, 45.0, 35.0)
        is equal_clean
    )


def test_robust_center_uses_component_median():
    points = np.array(
        [[0.0, 0.0, 1.0], [0.1, 0.2, 1.1], [50.0, 50.0, 50.0]]
    )
    assert robust_center(points) == pytest.approx([0.1, 0.2, 1.1])
