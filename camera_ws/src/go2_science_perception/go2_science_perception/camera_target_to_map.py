#!/usr/bin/env python3
"""Transform coarse camera targets into the SLAM map frame."""

import math
from collections import deque
from dataclasses import dataclass
from typing import Sequence, Tuple

from geometry_msgs.msg import PointStamped, TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a ROS builtin time message to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def point_minus_tf_seconds(point_stamp, tf_stamp) -> float:
    """Return positive seconds when a point is newer than available TF."""
    return (
        stamp_to_nanoseconds(point_stamp) - stamp_to_nanoseconds(tf_stamp)
    ) * 1e-9


@dataclass
class PendingPoint:
    """A point awaiting the transform at its original source timestamp."""

    point: PointStamped
    received_ns: int


def quaternion_from_rpy(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> Tuple[float, float, float, float]:
    """Return an xyzw quaternion for fixed-axis roll, pitch and yaw."""
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_static_transform(
    parent_frame: str,
    child_frame: str,
    translation: Sequence[float],
    rpy_deg: Sequence[float],
    stamp,
) -> TransformStamped:
    """Build the parameterized vehicle-to-camera-link transform."""
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    quaternion = quaternion_from_rpy(
        *(math.radians(float(angle)) for angle in rpy_deg)
    )
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    return transform


class CameraTargetToMap(Node):
    """Publish timestamp-correct map points and RViz markers."""

    def __init__(self) -> None:
        super().__init__("camera_target_to_map")
        defaults = {
            "input_topic": "/science/target_coarse_point",
            "output_topic": "/science/target_coarse_point_map",
            "marker_topic": "/science/target_coarse_point_map_marker",
            "map_frame": "map",
            "vehicle_frame": "vehicle",
            "camera_link_frame": "camera_link",
            "camera_x_m": 0.36,
            "camera_y_m": 0.0,
            "camera_z_m": 0.12,
            "camera_roll_deg": 0.0,
            "camera_pitch_deg": 30.0,
            "camera_yaw_deg": 0.0,
            "transform_timeout_sec": 0.0,
            "pending_timeout_sec": 8.0,
            "pending_queue_size": 128,
            "retry_period_sec": 0.02,
            "marker_scale_m": 0.12,
            "marker_lifetime_sec": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.point_publisher = self.create_publisher(
            PointStamped, self.get_parameter("output_topic").value, 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, self.get_parameter("marker_topic").value, 10
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("input_topic").value,
            self._point_callback,
            qos_profile_sensor_data,
        )
        self.transformed_count = 0
        self.failed_count = 0
        self.input_count = 0
        self.initial_miss_count = 0
        self.deferred_success_count = 0
        self.pending_points = deque()
        self.create_timer(
            float(self.get_parameter("retry_period_sec").value),
            self._retry_pending,
        )
        self._publish_camera_extrinsic()

    def _publish_camera_extrinsic(self) -> None:
        parent = str(self.get_parameter("vehicle_frame").value)
        child = str(self.get_parameter("camera_link_frame").value)
        translation = tuple(
            float(self.get_parameter(name).value)
            for name in ("camera_x_m", "camera_y_m", "camera_z_m")
        )
        rpy_deg = tuple(
            float(self.get_parameter(name).value)
            for name in (
                "camera_roll_deg", "camera_pitch_deg", "camera_yaw_deg"
            )
        )
        transform = make_static_transform(
            parent, child, translation, rpy_deg,
            self.get_clock().now().to_msg()
        )
        self.static_broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Static extrinsic {parent} -> {child}: "
            f"xyz={translation} m rpy={rpy_deg} deg; "
            "camera optical rotation remains owned by RealSense TF."
        )

    def _latest_tf_diagnostic(self, point: PointStamped) -> str:
        """Describe point time against the latest transform in the buffer."""
        point_text = (
            f"{point.header.stamp.sec}."
            f"{point.header.stamp.nanosec:09d}"
        )
        try:
            latest = self.tf_buffer.lookup_transform(
                self.map_frame,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            return (
                f"point_timestamp={point_text} "
                "latest_tf_timestamp=unavailable time_delta_sec=nan "
                f"latest_lookup_error={exc}"
            )
        latest_text = (
            f"{latest.header.stamp.sec}."
            f"{latest.header.stamp.nanosec:09d}"
        )
        delta = point_minus_tf_seconds(
            point.header.stamp, latest.header.stamp
        )
        return (
            f"point_timestamp={point_text} "
            f"latest_tf_timestamp={latest_text} "
            f"time_delta_sec={delta:+.9f}"
        )

    def _transform_and_publish(
        self, point: PointStamped, timeout_sec: float
    ) -> None:
        """Transform and publish only at the point's exact source time."""
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            point.header.frame_id,
            rclpy.time.Time.from_msg(point.header.stamp),
            timeout=Duration(seconds=max(0.0, timeout_sec)),
        )
        mapped = do_transform_point(point, transform)
        mapped.header.stamp = point.header.stamp
        mapped.header.frame_id = self.map_frame
        self.point_publisher.publish(mapped)
        self.marker_publisher.publish(self._make_marker(mapped))
        self.transformed_count += 1
        self.get_logger().info(
            f"MAP_TARGET source_frame={point.header.frame_id} "
            f"x={mapped.point.x:.3f} y={mapped.point.y:.3f} "
            f"z={mapped.point.z:.3f} transformed={self.transformed_count} "
            f"failed={self.failed_count} pending={len(self.pending_points)}"
        )

    def _drop_pending(self, pending: PendingPoint, reason: str) -> None:
        """Count and record a point that can no longer be transformed."""
        self.failed_count += 1
        self.get_logger().warning(
            f"TF_POINT_DROPPED reason={reason} "
            f"{self._latest_tf_diagnostic(pending.point)}"
        )

    def _retry_pending(self) -> None:
        """Retry queued points without replacing their source timestamps."""
        if not self.pending_points:
            return
        now_ns = self.get_clock().now().nanoseconds
        max_age_ns = int(
            float(self.get_parameter("pending_timeout_sec").value) * 1e9
        )
        retained = deque()
        while self.pending_points:
            pending = self.pending_points.popleft()
            if now_ns - pending.received_ns > max_age_ns:
                self._drop_pending(pending, "pending_timeout")
                continue
            try:
                self._transform_and_publish(pending.point, 0.0)
                self.deferred_success_count += 1
            except TransformException:
                retained.append(pending)
        self.pending_points = retained

    def _point_callback(self, point: PointStamped) -> None:
        self.input_count += 1
        if not point.header.frame_id:
            self.failed_count += 1
            self.get_logger().warning("Rejected point with empty source frame")
            return
        try:
            self._transform_and_publish(
                point,
                float(self.get_parameter("transform_timeout_sec").value),
            )
        except TransformException as exc:
            self.initial_miss_count += 1
            self.get_logger().warning(
                f"TF_LOOKUP_DEFERRED source={point.header.frame_id} "
                f"target={self.map_frame} "
                f"{self._latest_tf_diagnostic(point)} error={exc}"
            )
            self.pending_points.append(PendingPoint(
                point=point,
                received_ns=self.get_clock().now().nanoseconds,
            ))
            max_size = int(self.get_parameter("pending_queue_size").value)
            while len(self.pending_points) > max_size:
                self._drop_pending(
                    self.pending_points.popleft(), "queue_overflow"
                )
            return

    def close(self) -> None:
        """Account for pending points and emit final conversion counters."""
        while self.pending_points:
            self._drop_pending(self.pending_points.popleft(), "shutdown")
        ratio = (
            self.transformed_count / self.input_count
            if self.input_count else 0.0
        )
        self.get_logger().info(
            f"FINAL_MAP_TRANSFORM input={self.input_count} "
            f"transformed={self.transformed_count} failed={self.failed_count} "
            f"initial_miss={self.initial_miss_count} "
            f"deferred_success={self.deferred_success_count} "
            f"success_ratio={ratio:.6f}"
        )

    def _make_marker(self, point: PointStamped) -> Marker:
        marker = Marker()
        marker.header = point.header
        marker.ns = "science_target_coarse"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        scale = float(self.get_parameter("marker_scale_m").value)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.95
        lifetime = float(self.get_parameter("marker_lifetime_sec").value)
        marker.lifetime = Duration(seconds=max(0.0, lifetime)).to_msg()
        return marker


def main(args=None) -> None:
    """Run the camera-target to map transformer."""
    rclpy.init(args=args)
    node = CameraTargetToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
