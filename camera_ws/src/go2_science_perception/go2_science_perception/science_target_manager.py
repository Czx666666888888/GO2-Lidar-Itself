#!/usr/bin/env python3
"""Cluster map-frame science targets and publish non-motion guidance markers."""

import math
from collections import deque
from dataclasses import dataclass, field
from statistics import median
from typing import Iterable, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray


Point3 = Tuple[float, float, float]


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


@dataclass
class TargetTrack:
    """Persistent target state, updated at most once per source frame."""

    target_id: int
    observations: deque
    observation_count: int = 1
    confirmed: bool = False
    visited: bool = False
    last_stamp_ns: int = 0
    position: Point3 = field(init=False)

    def __post_init__(self) -> None:
        self.position = median_point(self.observations)

    def update(self, point: Point3, stamp_ns: int) -> None:
        self.observations.append(point)
        self.observation_count += 1
        self.last_stamp_ns = stamp_ns
        self.position = median_point(self.observations)


class TargetTracker:
    """One-to-one, per-frame nearest-neighbour target association."""

    def __init__(self, association_radius: float, confirmation_count: int,
                 recent_window: int) -> None:
        if association_radius <= 0.0:
            raise ValueError("association_radius must be positive")
        if confirmation_count <= 0 or recent_window <= 0:
            raise ValueError("counts must be positive")
        self.association_radius = float(association_radius)
        self.confirmation_count = int(confirmation_count)
        self.recent_window = int(recent_window)
        self.tracks: List[TargetTrack] = []
        self.next_id = 1

    def update_frame(self, points: Iterable[Point3], stamp_ns: int) -> None:
        """Associate all observations from one frame without track reuse."""
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
            if track.observation_count >= self.confirmation_count:
                track.confirmed = True
            used_observations.add(observation_index)
            used_tracks.add(track_index)

        for observation_index, point in enumerate(observations):
            if observation_index in used_observations:
                continue
            track = TargetTrack(
                target_id=self.next_id,
                observations=deque([point], maxlen=self.recent_window),
                last_stamp_ns=stamp_ns,
                confirmed=self.confirmation_count <= 1,
            )
            self.next_id += 1
            self.tracks.append(track)

    def confirmed_unvisited(self) -> List[TargetTrack]:
        return [track for track in self.tracks
                if track.confirmed and not track.visited]

    def mark_visited(self, target_id: int) -> bool:
        for track in self.tracks:
            if track.target_id == target_id and track.confirmed:
                track.visited = True
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
            "confirmed_marker_topic": "/science/confirmed_targets",
            "selected_target_topic": "/science/selected_target",
            "selected_marker_topic": "/science/selected_target_marker",
            "standoff_goal_topic": "/science/standoff_goal",
            "standoff_marker_topic": "/science/standoff_goal_marker",
            "map_frame": "map",
            "association_radius": 0.20,
            "confirmation_count": 5,
            "recent_window": 15,
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
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_position: Optional[Point3] = None
        self.robot_frame: Optional[str] = None
        self.batch_stamp_ns: Optional[int] = None
        self.batch_points: List[Point3] = []
        self.batch_updated_ns = 0

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
        if self.batch_stamp_ns is None:
            return
        timeout_ns = int(float(self.get_parameter(
            "frame_batch_timeout_sec").value) * 1e9)
        if self.get_clock().now().nanoseconds - self.batch_updated_ns >= timeout_ns:
            self._flush_batch()

    def _flush_batch(self) -> None:
        if self.batch_stamp_ns is None:
            return
        self.tracker.update_frame(self.batch_points, self.batch_stamp_ns)
        self.batch_stamp_ns = None
        self.batch_points = []
        self._publish_outputs()

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
        if self.tracker.mark_visited(int(message.data)):
            self.get_logger().info(f"Marked target id={message.data} visited")
            self._publish_outputs()
        else:
            self.get_logger().warning(
                f"Cannot mark unknown or unconfirmed target id={message.data} visited")

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
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.ns = namespace
        marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])

    def _publish_outputs(self) -> None:
        scale = float(self.get_parameter("marker_scale_m").value)
        confirmed = [track for track in self.tracker.tracks if track.confirmed]
        markers = self._delete_all("science_confirmed").markers
        for track in confirmed:
            color = (0.15, 0.85, 0.25) if not track.visited else (0.4, 0.4, 0.4)
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
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"T{track.target_id}"
            markers.append(label)
        self.confirmed_markers.publish(MarkerArray(markers=markers))

        selected = (self.tracker.nearest_unvisited(self.robot_position)
                    if self.robot_position is not None else None)
        if selected is None:
            self.selected_markers.publish(self._delete_all("science_selected"))
            self.standoff_markers.publish(self._delete_all("science_standoff"))
            return
        selected_message = self._point_message(selected.position)
        self.selected_points.publish(selected_message)
        selected_markers = self._delete_all("science_selected").markers
        selected_markers.append(self._sphere_marker(
            selected.target_id, "science_selected", selected.position,
            (1.0, 0.75, 0.0), scale * 1.35))
        self.selected_markers.publish(MarkerArray(markers=selected_markers))

        goal = standoff_point(
            self.robot_position, selected.position,
            float(self.get_parameter("standoff_distance").value))
        self.standoff_points.publish(self._point_message(goal))
        standoff_markers = self._delete_all("science_standoff").markers
        standoff_markers.append(self._sphere_marker(
            selected.target_id, "science_standoff", goal,
            (0.75, 0.1, 1.0), scale))
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
