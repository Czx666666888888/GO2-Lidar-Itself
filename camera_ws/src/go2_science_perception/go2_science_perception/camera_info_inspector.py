#!/usr/bin/env python3
"""Print D435 intrinsics from CameraInfo; never hard-code calibration values."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class CameraInfoInspector(Node):
    def __init__(self) -> None:
        super().__init__("camera_info_inspector")
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )
        topic = self.get_parameter("camera_info_topic").value
        self._printed = False
        self.create_subscription(
            CameraInfo, topic, self._callback, qos_profile_sensor_data
        )
        self.get_logger().info(f"Waiting for CameraInfo on {topic}")

    def _callback(self, msg: CameraInfo) -> None:
        if self._printed:
            return
        self._printed = True
        self.get_logger().info(
            "CameraInfo:\n"
            f"  fx={msg.k[0]:.9g}\n"
            f"  fy={msg.k[4]:.9g}\n"
            f"  cx={msg.k[2]:.9g}\n"
            f"  cy={msg.k[5]:.9g}\n"
            f"  width={msg.width}\n"
            f"  height={msg.height}\n"
            f"  distortion_model={msg.distortion_model}\n"
            f"  frame_id={msg.header.frame_id}\n"
            "Future pixel-to-3D conversion must use these runtime intrinsics."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraInfoInspector()
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
