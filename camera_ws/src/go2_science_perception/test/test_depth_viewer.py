"""Unit tests for depth ROI filtering and metre conversion."""

import numpy as np
import pytest

from go2_science_perception.depth_viewer import roi_depth_median


def test_uint16_roi_ignores_zero_and_converts_millimetres():
    depth = np.array(
        [[0, 1000, 1100], [900, 0, 1200], [800, 1300, 0]],
        dtype=np.uint16,
    )
    valid, total, median_m = roi_depth_median(depth, 1, 1, 3, "16UC1")
    assert valid == 6
    assert total == 9
    assert median_m == pytest.approx(1.05)


def test_float_roi_rejects_nonfinite_and_nonpositive_values():
    depth = np.array(
        [[np.nan, 1.0, 2.0], [np.inf, 0.0, -1.0], [3.0, 4.0, 5.0]],
        dtype=np.float32,
    )
    valid, total, median_m = roi_depth_median(depth, 1, 1, 3, "32FC1")
    assert valid == 5
    assert total == 9
    assert median_m == 3.0


def test_edge_roi_is_clipped_and_all_zero_is_invalid():
    depth = np.zeros((4, 4), dtype=np.uint16)
    valid, total, median_m = roi_depth_median(depth, 0, 0, 5, "16UC1")
    assert valid == 0
    assert total == 9
    assert median_m is None
