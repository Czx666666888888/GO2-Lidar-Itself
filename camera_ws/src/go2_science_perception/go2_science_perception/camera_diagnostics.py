#!/usr/bin/env python3
"""Read-only health and stream metadata checks for a RealSense D435."""

from functools import partial
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .statistics import StreamStats


class CameraDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("camera_diagnostics")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "aligned_depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "color_info_topic", "/camera/camera/color/camera_info"
        )
        self.declare_parameter(
            "depth_info_topic", "/camera/camera/depth/camera_info"
        )
        self.declare_parameter("report_period_sec", 5.0)
        self.declare_parameter("stale_after_sec", 2.0)

        self.topics = {
            "color": self.get_parameter("color_topic").value,
            "aligned_depth": self.get_parameter("aligned_depth_topic").value,
            "color_info": self.get_parameter("color_info_topic").value,
            "depth_info": self.get_parameter("depth_info_topic").value,
        }
        self.stats: Dict[str, StreamStats] = {
            name: StreamStats() for name in self.topics
        }

        self.create_subscription(
            Image,
            self.topics["color"],
            partial(self._image_callback, "color"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.topics["aligned_depth"],
            partial(self._image_callback, "aligned_depth"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.topics["color_info"],
            partial(self._camera_info_callback, "color_info"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.topics["depth_info"],
            partial(self._camera_info_callback, "depth_info"),
            qos_profile_sensor_data,
        )

        period = float(self.get_parameter("report_period_sec").value)
        self.create_timer(max(period, 0.1), self._report)
        self.get_logger().info(
            "Read-only D435 diagnostics started; no robot-control topics are used."
        )

    def _image_callback(self, name: str, msg: Image) -> None:
        stat = self.stats[name]
        stat.update(self.get_clock().now().nanoseconds / 1e9)
        stat.message_type = "sensor_msgs/msg/Image"
        stat.frame_id = msg.header.frame_id
        stat.width = msg.width
        stat.height = msg.height
        stat.encoding = msg.encoding

    def _camera_info_callback(self, name: str, msg: CameraInfo) -> None:
        stat = self.stats[name]
        stat.update(self.get_clock().now().nanoseconds / 1e9)
        stat.message_type = "sensor_msgs/msg/CameraInfo"
        stat.frame_id = msg.header.frame_id
        stat.width = msg.width
        stat.height = msg.height

    def _report(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        stale_after = float(self.get_parameter("stale_after_sec").value)
        lines = ["D435 stream report:"]
        all_fresh = True
        for name, topic in self.topics.items():
            stat = self.stats[name]
            fresh = stat.is_fresh(now, stale_after)
            all_fresh = all_fresh and fresh
            resolution = (
                f"{stat.width}x{stat.height}"
                if stat.width is not None and stat.height is not None
                else "unknown"
            )
            encoding = stat.encoding or "n/a"
            lines.append(
                f"  {name}: topic={topic} type={stat.message_type or 'unknown'} "
                f"count={stat.count} rate={stat.frequency_hz:.2f}Hz "
                f"frame_id={stat.frame_id or 'unknown'} resolution={resolution} "
                f"encoding={encoding} fresh={fresh}"
            )
        log = self.get_logger().info if all_fresh else self.get_logger().warn
        log("\n".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraDiagnostics()
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
