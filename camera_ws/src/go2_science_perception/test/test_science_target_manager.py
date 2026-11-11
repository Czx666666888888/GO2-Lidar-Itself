"""Unit tests for multi-target association and standoff geometry."""

import math

import pytest

from go2_science_perception.science_target_manager import (
    CANDIDATE,
    CONFIRMED_COLOR,
    LABEL_COLOR,
    SELECTED_COLOR,
    SELECTED_SCALE_FACTOR,
    STALE,
    STANDOFF_COLOR,
    TargetTracker,
    VISITED_COLOR,
    deletion_markers,
    median_point,
    standoff_point,
)
from visualization_msgs.msg import Marker


NS = 1_000_000_000


def make_tracker(**overrides):
    settings = dict(
        association_radius=0.20,
        confirmation_count=2,
        recent_window=20,
        candidate_timeout_sec=1.0,
        confirmed_stale_timeout_sec=2.0,
        remove_timeout_sec=5.0,
        merge_radius=0.10,
        merge_confirmation_count=3,
    )
    settings.update(overrides)
    return TargetTracker(**settings)


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


def test_candidate_timeout_removes_track():
    tracker = make_tracker()
    tracker.update_frame([(0.0, 0.0, 0.0)], 0)
    assert tracker.tracks[0].state == CANDIDATE
    events = tracker.expire(NS)
    assert tracker.tracks == []
    assert events == ["REMOVED id=1 state=CANDIDATE"]


def test_confirmed_becomes_stale_and_stale_is_not_selected():
    tracker = make_tracker(confirmation_count=1)
    tracker.update_frame([(1.0, 0.0, 0.0)], 0)
    tracker.expire(2 * NS)
    assert tracker.tracks[0].state == STALE
    assert tracker.nearest_unvisited((0.0, 0.0, 0.0)) is None


def test_confirmed_track_is_removed_after_remove_timeout():
    tracker = make_tracker(confirmation_count=1)
    tracker.update_frame([(1.0, 0.0, 0.0)], 0)
    events = tracker.expire(5 * NS)
    assert tracker.tracks == []
    assert "STALE id=1" in events
    assert "REMOVED id=1 state=STALE" in events


def test_marked_target_is_excluded_by_spatial_visit():
    tracker = make_tracker(confirmation_count=1)
    tracker.update_frame([(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], 1)
    assert tracker.mark_visited(1)
    assert tracker.nearest_unvisited((0.0, 0.0, 0.0)).target_id == 2


def test_new_id_near_visited_position_is_still_excluded():
    tracker = make_tracker(confirmation_count=1, visited_radius=0.15)
    tracker.update_frame([(1.0, 0.0, 0.0)], 0)
    tracker.add_visited_position(tracker.tracks[0].position)
    tracker.expire(5 * NS)
    tracker.update_frame([(1.10, 0.0, 0.0), (2.0, 0.0, 0.0)], 6 * NS)
    assert [track.target_id for track in tracker.confirmed_unvisited()] == [3]


def test_target_outside_visited_radius_remains_eligible():
    tracker = make_tracker(confirmation_count=1, visited_radius=0.15)
    tracker.add_visited_position((1.0, 0.0, 0.0))
    tracker.update_frame([(1.15, 0.0, 0.0)], 1)
    assert tracker.nearest_unvisited((0.0, 0.0, 0.0)).target_id == 1


def test_multiple_visited_positions_filter_all_nearby_tracks():
    tracker = make_tracker(confirmation_count=1)
    tracker.add_visited_position((1.0, 0.0, 0.0))
    tracker.add_visited_position((3.0, 0.0, 0.0))
    tracker.update_frame([
        (1.02, 0.0, 0.0), (2.0, 0.0, 0.0), (3.03, 0.0, 0.0)], 1)
    assert [track.target_id for track in tracker.confirmed_unvisited()] == [2]


def test_visited_positions_are_not_duplicated_inside_radius():
    tracker = make_tracker(visited_radius=0.15)
    first = tracker.add_visited_position((1.0, 2.0, 0.1))
    duplicate = tracker.add_visited_position((1.10, 2.0, 9.0))
    assert first == (True, 0, 0.0)
    assert duplicate[0:2] == (False, 0)
    assert duplicate[2] == pytest.approx(0.10)
    assert tracker.visited_positions == [(1.0, 2.0, 0.1)]


def test_nearest_unvisited_target_is_selected():
    tracker = make_tracker(confirmation_count=1)
    tracker.add_visited_position((1.0, 0.0, 0.0))
    tracker.update_frame([
        (1.02, 0.0, 0.0), (4.0, 0.0, 0.0), (2.0, 0.0, 0.0)], 1)
    assert tracker.nearest_unvisited((0.0, 0.0, 0.0)).target_id == 3


def test_close_confirmed_tracks_merge_only_after_consecutive_frames():
    tracker = make_tracker(confirmation_count=1, merge_confirmation_count=3)
    for stamp in range(1, 4):
        events = tracker.update_frame(
            [(0.0, 0.0, 0.0), (0.08, 0.0, 0.0)], stamp)
    assert len(tracker.tracks) == 1
    assert any(event.startswith("MERGED ") for event in events)


def test_brief_close_approach_does_not_merge():
    tracker = make_tracker(confirmation_count=1, merge_confirmation_count=3)
    tracker.update_frame([(0.0, 0.0, 0.0), (0.08, 0.0, 0.0)], 1)
    tracker.update_frame([(0.0, 0.0, 0.0), (0.15, 0.0, 0.0)], 2)
    tracker.update_frame([(0.0, 0.0, 0.0), (0.08, 0.0, 0.0)], 3)
    assert len(tracker.tracks) == 2


def test_merge_keeps_more_observed_stable_id_and_recomputes_median():
    tracker = make_tracker(confirmation_count=1, merge_confirmation_count=2)
    tracker.update_frame([(0.00, 0.0, 0.0)], 1)
    tracker.update_frame([(0.00, 0.0, 0.0), (0.08, 0.0, 0.0)], 2)
    events = tracker.update_frame([(0.02, 0.0, 0.0), (0.08, 0.0, 0.0)], 3)
    assert len(tracker.tracks) == 1
    kept = tracker.tracks[0]
    assert kept.target_id == 1
    assert kept.observation_count == 5
    assert kept.position == pytest.approx((0.02, 0.0, 0.0))
    assert "MERGED 2 -> 1" in events


def test_removed_track_markers_clear_sphere_and_label_namespaces():
    markers = deletion_markers(
        "map", ["science_confirmed", "science_confirmed_labels"])
    assert [(item.ns, item.action) for item in markers.markers] == [
        ("science_confirmed", Marker.DELETEALL),
        ("science_confirmed_labels", Marker.DELETEALL),
    ]
