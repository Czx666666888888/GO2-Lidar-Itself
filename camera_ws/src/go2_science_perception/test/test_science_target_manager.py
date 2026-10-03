"""Unit tests for multi-target association and standoff geometry."""

import math

import pytest

from go2_science_perception.science_target_manager import (
    CONFIRMED_COLOR,
    LABEL_COLOR,
    SELECTED_COLOR,
    SELECTED_SCALE_FACTOR,
    STANDOFF_COLOR,
    TargetTracker,
    VISITED_COLOR,
    median_point,
    standoff_point,
)


def test_median_point_rejects_outlier():
    assert median_point([(1.0, 2.0, 3.0), (1.1, 2.1, 3.1),
                         (9.0, 9.0, 9.0)]) == pytest.approx((1.1, 2.1, 3.1))


def test_same_frame_multiple_points_update_distinct_tracks_once():
    tracker = TargetTracker(association_radius=0.20, confirmation_count=2,
                            recent_window=5)
    tracker.update_frame([(0.0, 0.0, 0.2), (0.10, 0.0, 0.3)], 1)
    assert len(tracker.tracks) == 2
    tracker.update_frame([(0.01, 0.0, 0.2), (0.11, 0.0, 0.3)], 2)
    assert [track.observation_count for track in tracker.tracks] == [2, 2]
    assert all(track.confirmed for track in tracker.tracks)


def test_confirmation_counts_frames_not_points():
    tracker = TargetTracker(association_radius=0.20, confirmation_count=3,
                            recent_window=5)
    tracker.update_frame([(0.0, 0.0, 0.0), (0.02, 0.0, 0.0)], 1)
    assert len(tracker.tracks) == 2
    assert not any(track.confirmed for track in tracker.tracks)


def test_stable_ids_and_nearest_unvisited_selection():
    tracker = TargetTracker(association_radius=0.20, confirmation_count=2,
                            recent_window=5)
    tracker.update_frame([(1.0, 0.0, 0.0), (3.0, 0.0, 0.0)], 1)
    tracker.update_frame([(1.02, 0.0, 0.0), (2.98, 0.0, 0.0)], 2)
    selected = tracker.nearest_unvisited((0.0, 0.0, 0.0))
    assert selected.target_id == 1
    assert tracker.mark_visited(1)
    assert tracker.nearest_unvisited((0.0, 0.0, 0.0)).target_id == 2


def test_recent_observation_window_controls_median():
    tracker = TargetTracker(association_radius=2.0, confirmation_count=1,
                            recent_window=3)
    for stamp, x in enumerate((0.0, 0.1, 0.2, 1.0), start=1):
        tracker.update_frame([(x, 0.0, 0.0)], stamp)
    assert tracker.tracks[0].position[0] == pytest.approx(0.2)


def test_standoff_is_point_four_tenths_from_target_toward_robot():
    goal = standoff_point((0.0, 0.0, 0.1), (1.0, 1.0, 0.5), 0.4)
    assert math.hypot(goal[0] - 1.0, goal[1] - 1.0) == pytest.approx(0.4)
    assert goal[2] == pytest.approx(0.1)


def test_rviz_marker_palette_and_selected_scale_are_distinct():
    assert CONFIRMED_COLOR == (0.0, 1.0, 1.0)
    assert SELECTED_COLOR == (1.0, 0.0, 0.0)
    assert STANDOFF_COLOR == (1.0, 1.0, 0.0)
    assert VISITED_COLOR == (0.65, 0.65, 0.65)
    assert LABEL_COLOR == (1.0, 1.0, 1.0)
    assert SELECTED_SCALE_FACTOR > 1.0
