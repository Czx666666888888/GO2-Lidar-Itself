#!/usr/bin/env python3
"""Cluster map-frame science targets and publish non-motion guidance markers."""

import math
from collections import deque
from dataclasses import dataclass, field
from statistics import median
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, Int32
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray


Point3 = Tuple[float, float, float]

CONFIRMED_COLOR = (0.0, 1.0, 1.0)
SELECTED_COLOR = (1.0, 0.0, 0.0)
STANDOFF_COLOR = (1.0, 1.0, 0.0)
VISITED_COLOR = (0.65, 0.65, 0.65)
LABEL_COLOR = (1.0, 1.0, 1.0)
SELECTED_SCALE_FACTOR = 1.5
CANDIDATE = "CANDIDATE"
CONFIRMED = "CONFIRMED"
STALE = "STALE"


def xy_distance(left: Sequence[float], right: Sequence[float]) -> float:
    """Return planar Euclidean distance."""
    return math.hypot(float(left[0]) - float(right[0]),
                      float(left[1]) - float(right[1]))


def median_point(points: Iterable[Point3]) -> Point3:
    """Return the coordinate-wise median of at least one point."""
    values = list(points)
    if not values:
        raise ValueError("median_point requires at least one point")
    return tuple(float(median(axis)) for axis in zip(*values))


def standoff_point(robot: Point3, target: Point3, distance_m: float) -> Point3:
    """Place a goal between robot and target at the requested target distance."""
    dx = robot[0] - target[0]
    dy = robot[1] - target[1]
    norm = math.hypot(dx, dy)
    if norm <= 1e-9:
        return target[0], target[1], robot[2]
    scale = max(0.0, float(distance_m)) / norm
    return target[0] + dx * scale, target[1] + dy * scale, robot[2]


def deletion_markers(map_frame: str, namespaces: Iterable[str]) -> MarkerArray:
    """Create namespace clears so removed track spheres and labels cannot linger."""
    markers = []
    for namespace in namespaces:
        marker = Marker()
        marker.header.frame_id = map_frame
        marker.ns = namespace
        marker.action = Marker.DELETEALL
        markers.append(marker)
    return MarkerArray(markers=markers)


@dataclass
class TargetTrack:
    """Persistent target state, updated at most once per source frame."""

    target_id: int
    observations: deque
    observation_count: int = 1
    state: str = CANDIDATE
    visited: bool = False
    first_seen: int = 0
    last_seen: int = 0
    last_stamp_ns: int = 0
    position: Point3 = field(init=False)

    def __post_init__(self) -> None:
        self.position = median_point(self.observations)

    def update(self, point: Point3, stamp_ns: int) -> None:
        self.observations.append(point)
        self.observation_count += 1
        self.last_seen = stamp_ns
        self.last_stamp_ns = stamp_ns
        self.position = median_point(self.observations)

    @property
    def confirmed(self) -> bool:
        """Compatibility view used by existing callers and tests."""
        return self.state == CONFIRMED


class TargetTracker:
    """One-to-one, per-frame nearest-neighbour target association."""

    def __init__(self, association_radius: float, confirmation_count: int,
                 recent_window: int, candidate_timeout_sec: float = 1.0,
                 confirmed_stale_timeout_sec: float = 2.0,
                 remove_timeout_sec: float = 10.0,
                 merge_radius: float = 0.10,
                 merge_confirmation_count: int = 5,
                 visited_radius: float = 0.15) -> None:
        if association_radius <= 0.0:
            raise ValueError("association_radius must be positive")
        if confirmation_count <= 0 or recent_window <= 0:
            raise ValueError("counts must be positive")
        if candidate_timeout_sec <= 0.0 or confirmed_stale_timeout_sec <= 0.0:
            raise ValueError("lifecycle timeouts must be positive")
        if remove_timeout_sec <= confirmed_stale_timeout_sec:
            raise ValueError("remove_timeout_sec must exceed stale timeout")
        if merge_radius <= 0.0 or merge_confirmation_count <= 0:
            raise ValueError("merge settings must be positive")
        if visited_radius <= 0.0:
            raise ValueError("visited_radius must be positive")
        self.association_radius = float(association_radius)
        self.confirmation_count = int(confirmation_count)
        self.recent_window = int(recent_window)
        self.candidate_timeout_ns = int(candidate_timeout_sec * 1e9)
        self.confirmed_stale_timeout_ns = int(confirmed_stale_timeout_sec * 1e9)
        self.remove_timeout_ns = int(remove_timeout_sec * 1e9)
        self.merge_radius = float(merge_radius)
        self.merge_confirmation_count = int(merge_confirmation_count)
        self.visited_radius = float(visited_radius)
        self.visited_positions: List[Point3] = []
        self.tracks: List[TargetTrack] = []
        self.next_id = 1
        self.merge_counts: Dict[Tuple[int, int], int] = {}

    def update_frame(self, points: Iterable[Point3], stamp_ns: int) -> List[str]:
        """Associate all observations from one frame without track reuse."""
        events = self.expire(stamp_ns)
        observations = list(points)
        candidates = []
        for observation_index, point in enumerate(observations):
            for track_index, track in enumerate(self.tracks):
                distance = xy_distance(point, track.position)
                if distance <= self.association_radius:
                    candidates.append((distance, observation_index, track_index))

        used_observations = set()
        used_tracks = set()
        for _, observation_index, track_index in sorted(candidates):
            if observation_index in used_observations or track_index in used_tracks:
                continue
            track = self.tracks[track_index]
            track.update(observations[observation_index], stamp_ns)
            events.append(f"ASSOCIATED id={track.target_id}")
            if track.state == STALE:
                track.state = CONFIRMED
                events.append(f"CONFIRMED id={track.target_id} reactivated=true")
            if (track.state == CANDIDATE and
                    track.observation_count >= self.confirmation_count):
                track.state = CONFIRMED
                events.append(f"CONFIRMED id={track.target_id}")
            used_observations.add(observation_index)
            used_tracks.add(track_index)

        for observation_index, point in enumerate(observations):
            if observation_index in used_observations:
                continue
            track = TargetTrack(
                target_id=self.next_id,
                observations=deque([point], maxlen=self.recent_window),
                first_seen=stamp_ns,
                last_seen=stamp_ns,
                last_stamp_ns=stamp_ns,
                state=(CONFIRMED if self.confirmation_count <= 1 else CANDIDATE),
            )
            self.next_id += 1
            self.tracks.append(track)
            events.append(f"NEW_TRACK id={track.target_id}")
            if track.state == CONFIRMED:
                events.append(f"CONFIRMED id={track.target_id}")
        events.extend(self._merge_confirmed())
        return events

    def expire(self, now_ns: int) -> List[str]:
        """Apply lifecycle timeouts and return auditable transition events."""
        events = []
        kept = []
        for track in self.tracks:
            age_ns = max(0, now_ns - track.last_seen)
            if track.state == CANDIDATE and age_ns >= self.candidate_timeout_ns:
                events.append(f"REMOVED id={track.target_id} state={CANDIDATE}")
                continue
            if track.state == CONFIRMED and age_ns >= self.confirmed_stale_timeout_ns:
                track.state = STALE
                events.append(f"STALE id={track.target_id}")
            if track.state == STALE and age_ns >= self.remove_timeout_ns:
                events.append(f"REMOVED id={track.target_id} state={STALE}")
                continue
            kept.append(track)
        self.tracks = kept
        active_ids = {track.target_id for track in kept}
        self.merge_counts = {
            pair: count for pair, count in self.merge_counts.items()
            if pair[0] in active_ids and pair[1] in active_ids
        }
        return events

    @staticmethod
    def _keep_rank(track: TargetTrack) -> Tuple[int, int, int, int]:
        state_rank = {CANDIDATE: 0, STALE: 1, CONFIRMED: 2}[track.state]
        return state_rank, track.observation_count, -track.first_seen, -track.target_id

    def _merge_confirmed(self) -> List[str]:
        confirmed = [track for track in self.tracks if track.state == CONFIRMED]
        close_pairs = set()
        for left_index, left in enumerate(confirmed):
            for right in confirmed[left_index + 1:]:
                pair = tuple(sorted((left.target_id, right.target_id)))
                if xy_distance(left.position, right.position) < self.merge_radius:
                    close_pairs.add(pair)
                    self.merge_counts[pair] = self.merge_counts.get(pair, 0) + 1
        self.merge_counts = {
            pair: count for pair, count in self.merge_counts.items()
            if pair in close_pairs
        }
        events = []
        ready = sorted(pair for pair, count in self.merge_counts.items()
                       if count >= self.merge_confirmation_count)
        for pair in ready:
            by_id = {track.target_id: track for track in self.tracks}
            if pair[0] not in by_id or pair[1] not in by_id:
                continue
            left, right = by_id[pair[0]], by_id[pair[1]]
            kept = max((left, right), key=self._keep_rank)
            old = right if kept is left else left
            combined = list(kept.observations) + list(old.observations)
            kept.observations = deque(combined, maxlen=self.recent_window)
            kept.observation_count += old.observation_count
            kept.first_seen = min(kept.first_seen, old.first_seen)
            kept.last_seen = max(kept.last_seen, old.last_seen)
            kept.last_stamp_ns = max(kept.last_stamp_ns, old.last_stamp_ns)
            kept.visited = kept.visited or old.visited
            kept.position = median_point(kept.observations)
            self.tracks.remove(old)
            events.append(f"MERGED {old.target_id} -> {kept.target_id}")
            self.merge_counts = {
                key: value for key, value in self.merge_counts.items()
                if old.target_id not in key and kept.target_id not in key
            }
        return events

    def confirmed_unvisited(self) -> List[TargetTrack]:
        return [track for track in self.tracks
                if track.state == CONFIRMED and
                self.visited_match(track.position) is None]

    def visited_match(self, position: Point3) -> Optional[Tuple[int, float]]:
        """Return the closest visited position strictly inside the XY radius."""
        matches = [(index, xy_distance(position, visited))
                   for index, visited in enumerate(self.visited_positions)]
        if not matches:
            return None
        index, distance = min(matches, key=lambda item: item[1])
        inside = (distance < self.visited_radius and
                  not math.isclose(distance, self.visited_radius,
                                   rel_tol=0.0, abs_tol=1e-9))
        return (index, distance) if inside else None

    def add_visited_position(
            self, position: Point3) -> Tuple[bool, int, float]:
        """Add a spatial visit or merge it with an existing nearby visit."""
        match = self.visited_match(position)
        if match is not None:
            index, distance = match
            return False, index, distance
        self.visited_positions.append(tuple(float(value) for value in position))
        return True, len(self.visited_positions) - 1, 0.0

    def mark_visited(self, target_id: int) -> bool:
        for track in self.tracks:
            if track.target_id == target_id and track.state == CONFIRMED:
                track.visited = True
                self.add_visited_position(track.position)
                return True
        return False

    def nearest_unvisited(self, robot: Point3) -> Optional[TargetTrack]:
        candidates = self.confirmed_unvisited()
        return min(candidates, key=lambda item: xy_distance(robot, item.position),
                   default=None)


class ScienceTargetManager(Node):
    """Maintain stable target IDs and publish visualization-only outputs."""

    def __init__(self) -> None:
        super().__init__("science_target_manager")
        defaults = {
            "target_topic": "/science/target_coarse_point_map",
            "base_odom_topic": "/base_state_estimation",
            "visited_target_topic": "/science/visited_target_id",
            "mark_selected_visited_topic": "/science/mark_selected_visited",
            "confirmed_marker_topic": "/science/confirmed_targets",
            "selected_target_topic": "/science/selected_target",
            "selected_marker_topic": "/science/selected_target_marker",
            "standoff_goal_topic": "/science/standoff_goal",
            "standoff_marker_topic": "/science/standoff_goal_marker",
            "map_frame": "map",
            "association_radius": 0.20,
            "confirmation_count": 5,
            "recent_window": 15,
            "candidate_timeout_sec": 1.0,
            "confirmed_stale_timeout_sec": 2.0,
            "remove_timeout_sec": 10.0,
            "merge_radius": 0.10,
            "merge_confirmation_count": 5,
            "visited_radius": 0.15,
            "standoff_distance": 0.40,
            "frame_batch_timeout_sec": 0.05,
            "marker_scale_m": 0.16,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.tracker = TargetTracker(
            float(self.get_parameter("association_radius").value),
            int(self.get_parameter("confirmation_count").value),
            int(self.get_parameter("recent_window").value),
            float(self.get_parameter("candidate_timeout_sec").value),
            float(self.get_parameter("confirmed_stale_timeout_sec").value),
            float(self.get_parameter("remove_timeout_sec").value),
            float(self.get_parameter("merge_radius").value),
            int(self.get_parameter("merge_confirmation_count").value),
            float(self.get_parameter("visited_radius").value),
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_position: Optional[Point3] = None
        self.robot_frame: Optional[str] = None
        self.batch_stamp_ns: Optional[int] = None
        self.batch_points: List[Point3] = []
        self.batch_updated_ns = 0
        self.selected_id: Optional[int] = None
        self.logged_visited_matches = set()

        self.confirmed_markers = self.create_publisher(
            MarkerArray, self.get_parameter("confirmed_marker_topic").value, 10)
        self.selected_points = self.create_publisher(
            PointStamped, self.get_parameter("selected_target_topic").value, 10)
        self.selected_markers = self.create_publisher(
            MarkerArray, self.get_parameter("selected_marker_topic").value, 10)
        self.standoff_points = self.create_publisher(
            PointStamped, self.get_parameter("standoff_goal_topic").value, 10)
        self.standoff_markers = self.create_publisher(
            MarkerArray, self.get_parameter("standoff_marker_topic").value, 10)
        self.create_subscription(
            PointStamped, self.get_parameter("target_topic").value,
            self._target_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter("base_odom_topic").value,
            self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            Int32, self.get_parameter("visited_target_topic").value,
            self._visited_callback, 10)
        self.create_subscription(
            Empty, self.get_parameter("mark_selected_visited_topic").value,
            self._mark_selected_visited_callback, 10)
        self.create_timer(0.02, self._flush_expired_batch)
        self.get_logger().info(
            "Science target manager is visualization-only: it does not publish "
            "/goal_point or any motion command. Waiting for actual odometry frame.")

    @staticmethod
    def _stamp_ns(message) -> int:
        return int(message.sec) * 1_000_000_000 + int(message.nanosec)

    def _target_callback(self, message: PointStamped) -> None:
        if message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f"Rejected target frame={message.header.frame_id!r}; "
                f"expected {self.map_frame!r}")
            return
        stamp_ns = self._stamp_ns(message.header.stamp)
        if self.batch_stamp_ns is not None and stamp_ns != self.batch_stamp_ns:
            self._flush_batch()
        self.batch_stamp_ns = stamp_ns
        self.batch_points.append((message.point.x, message.point.y,
                                  message.point.z))
        self.batch_updated_ns = self.get_clock().now().nanoseconds

    def _flush_expired_batch(self) -> None:
        events = self.tracker.expire(self.get_clock().now().nanoseconds)
        if events:
            self._log_events(events)
            self._publish_outputs()
        if self.batch_stamp_ns is None:
            return
        timeout_ns = int(float(self.get_parameter(
            "frame_batch_timeout_sec").value) * 1e9)
        if self.get_clock().now().nanoseconds - self.batch_updated_ns >= timeout_ns:
            self._flush_batch()

    def _flush_batch(self) -> None:
        if self.batch_stamp_ns is None:
            return
        events = self.tracker.update_frame(self.batch_points, self.batch_stamp_ns)
        self._log_events(events)
        self.batch_stamp_ns = None
        self.batch_points = []
        self._publish_outputs()

    def _log_events(self, events: Iterable[str]) -> None:
        for event in events:
            self.get_logger().info(event)

    def _odom_callback(self, message: Odometry) -> None:
        source_frame = message.header.frame_id
        if not source_frame:
            self.get_logger().warning("Rejected /base_state_estimation with empty frame")
            return
        if source_frame != self.robot_frame:
            self.robot_frame = source_frame
            self.get_logger().info(
                f"Observed /base_state_estimation frame={source_frame!r}; "
                f"transforming it to {self.map_frame!r} before target selection")
        source = PointStamped()
        source.header = message.header
        source.point = message.pose.pose.position
        try:
            if source_frame == self.map_frame:
                mapped = source
            else:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, source_frame,
                    rclpy.time.Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=0.0))
                mapped = do_transform_point(source, transform)
        except TransformException as exc:
            self.get_logger().warning(
                f"Cannot transform GO2 center {source_frame}->{self.map_frame} "
                f"at odometry timestamp: {exc}", throttle_duration_sec=2.0)
            return
        self.robot_position = (mapped.point.x, mapped.point.y, mapped.point.z)
        self._publish_outputs()

    def _visited_callback(self, message: Int32) -> None:
        track = next((item for item in self.tracker.tracks
                      if item.target_id == int(message.data) and
                      item.state == CONFIRMED), None)
        if track is not None:
            self._store_visited(track.position)
            track.visited = True
            self._publish_outputs()
        else:
            self.get_logger().warning(
                f"Cannot mark unknown or unconfirmed target id={message.data} visited")

    def _mark_selected_visited_callback(self, _message: Empty) -> None:
        track = next((item for item in self.tracker.tracks
                      if item.target_id == self.selected_id and
                      item.state == CONFIRMED), None)
        if track is None:
            self.get_logger().warning(
                "Cannot mark selected visited: no ACTIVE CONFIRMED selection")
            return
        self._store_visited(track.position)
        track.visited = True
        self._publish_outputs()

    def _store_visited(self, position: Point3) -> None:
        added, index, distance = self.tracker.add_visited_position(position)
        if added:
            self.get_logger().info(
                f"VISITED_ADDED x={position[0]:.3f} y={position[1]:.3f} "
                f"visited_index={index}")
        else:
            self.get_logger().info(
                f"VISITED_DUPLICATE visited_index={index} distance={distance:.3f}")

    def _point_message(self, position: Point3) -> PointStamped:
        message = PointStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.point.x, message.point.y, message.point.z = position
        return message

    def _sphere_marker(self, marker_id: int, namespace: str, position: Point3,
                       color: Tuple[float, float, float], scale: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=position[0], y=position[1], z=position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.95
        return marker

    def _delete_all(self, namespace: str) -> MarkerArray:
        return deletion_markers(self.map_frame, [namespace])

    def _publish_outputs(self) -> None:
        scale = float(self.get_parameter("marker_scale_m").value)
        confirmed = [track for track in self.tracker.tracks
                     if track.state == CONFIRMED]
        markers = self._delete_all("science_confirmed").markers
        markers.extend(self._delete_all("science_confirmed_labels").markers)
        for track in confirmed:
            match = self.tracker.visited_match(track.position)
            color = VISITED_COLOR if match is not None else CONFIRMED_COLOR
            if match is not None:
                key = (track.target_id, match[0])
                if key not in self.logged_visited_matches:
                    self.get_logger().info(
                        f"VISITED_MATCH track={track.target_id} "
                        f"visited_index={match[0]} distance={match[1]:.3f}")
                    self.logged_visited_matches.add(key)
            markers.append(self._sphere_marker(
                track.target_id, "science_confirmed", track.position, color, scale))
            label = Marker()
            label.header = markers[-1].header
            label.ns = "science_confirmed_labels"
            label.id = track.target_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=track.position[0], y=track.position[1],
                z=track.position[2] + scale)
            label.pose.orientation.w = 1.0
            label.scale.z = scale
            label.color.r, label.color.g, label.color.b = LABEL_COLOR
            label.color.a = 1.0
            label.text = (f"T{track.target_id} visited/ignored"
                          if match is not None else f"T{track.target_id}")
            markers.append(label)
        for index, position in enumerate(self.tracker.visited_positions):
            markers.append(self._sphere_marker(
                index, "science_visited_positions", position,
                VISITED_COLOR, scale * 0.75))
        self.confirmed_markers.publish(MarkerArray(markers=markers))

        selected = (self.tracker.nearest_unvisited(self.robot_position)
                    if self.robot_position is not None else None)
        new_selected_id = selected.target_id if selected is not None else None
        if new_selected_id != self.selected_id:
            self.get_logger().info(
                f"SELECTED_CHANGED {self.selected_id} -> {new_selected_id}")
            self.selected_id = new_selected_id
        if selected is None:
            self.selected_markers.publish(self._delete_all("science_selected"))
            self.standoff_markers.publish(self._delete_all("science_standoff"))
            return
        selected_message = self._point_message(selected.position)
        self.selected_points.publish(selected_message)
        selected_markers = self._delete_all("science_selected").markers
        selected_markers.append(self._sphere_marker(
            selected.target_id, "science_selected", selected.position,
            SELECTED_COLOR, scale * SELECTED_SCALE_FACTOR))
        self.selected_markers.publish(MarkerArray(markers=selected_markers))

        goal = standoff_point(
            self.robot_position, selected.position,
            float(self.get_parameter("standoff_distance").value))
        self.standoff_points.publish(self._point_message(goal))
        standoff_markers = self._delete_all("science_standoff").markers
        standoff_markers.append(self._sphere_marker(
            selected.target_id, "science_standoff", goal,
            STANDOFF_COLOR, scale))
        self.standoff_markers.publish(MarkerArray(markers=standoff_markers))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScienceTargetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
