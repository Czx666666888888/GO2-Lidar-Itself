#!/usr/bin/env python3
"""Transform coarse camera targets into the SLAM map frame."""

import math
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
            "transform_timeout_sec": 0.20,
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
        self.last_warning_ns = -10**18
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

    def _warn_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= 2_000_000_000:
            self.get_logger().warning(message)
            self.last_warning_ns = now_ns

    def _point_callback(self, point: PointStamped) -> None:
        if not point.header.frame_id:
            self.failed_count += 1
            self._warn_throttled("Rejected point with empty source frame")
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                point.header.frame_id,
                rclpy.time.Time.from_msg(point.header.stamp),
                timeout=Duration(
                    seconds=float(
                        self.get_parameter("transform_timeout_sec").value
                    )
                ),
            )
            mapped = do_transform_point(point, transform)
        except TransformException as exc:
            self.failed_count += 1
            self._warn_throttled(
                f"TF unavailable at source timestamp "
                f"{point.header.stamp.sec}.{point.header.stamp.nanosec:09d}: "
                f"{point.header.frame_id} -> {self.map_frame}: {exc}"
            )
            return

        mapped.header.stamp = point.header.stamp
        mapped.header.frame_id = self.map_frame
        self.point_publisher.publish(mapped)
        self.marker_publisher.publish(self._make_marker(mapped))
        self.transformed_count += 1
        self.get_logger().info(
            f"MAP_TARGET source_frame={point.header.frame_id} "
            f"x={mapped.point.x:.3f} y={mapped.point.y:.3f} "
            f"z={mapped.point.z:.3f} transformed={self.transformed_count} "
            f"failed={self.failed_count}"
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
