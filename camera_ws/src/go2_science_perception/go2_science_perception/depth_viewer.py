#!/usr/bin/env python3
"""Display aligned D435 images and report robust click-to-depth probes."""

from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def roi_depth_median(
    depth: np.ndarray,
    u: int,
    v: int,
    roi_size: int,
    encoding: str,
) -> Tuple[int, int, Optional[float]]:
    """Return valid count, total count and median depth in metres."""
    if depth.ndim != 2:
        raise ValueError("Depth image must have exactly two dimensions")
    if roi_size < 1 or roi_size % 2 == 0:
        raise ValueError("roi_size must be a positive odd integer")
    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
        return 0, 0, None

    radius = roi_size // 2
    x0 = max(0, u - radius)
    x1 = min(depth.shape[1], u + radius + 1)
    y0 = max(0, v - radius)
    y1 = min(depth.shape[0], v + radius + 1)
    roi = depth[y0:y1, x0:x1].astype(np.float64, copy=False)
    valid = roi[np.isfinite(roi) & (roi > 0.0)]
    total_count = int(roi.size)
    if valid.size == 0:
        return 0, total_count, None

    if encoding in ("16UC1", "mono16"):
        valid = valid * 0.001
    elif encoding != "32FC1":
        raise ValueError(f"Unsupported depth encoding: {encoding}")
    return int(valid.size), total_count, float(np.median(valid))


class DepthViewer(Node):
    """Read-only RGB/aligned-depth viewer with a mouse depth probe."""

    def __init__(self) -> None:
        super().__init__("depth_viewer")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "aligned_depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )
        self.declare_parameter("roi_size", 5)
        self.declare_parameter("display_max_depth_m", 5.0)

        self.roi_size = int(self.get_parameter("roi_size").value)
        if self.roi_size < 1 or self.roi_size % 2 == 0:
            raise ValueError("roi_size must be a positive odd integer")
        self.display_max_depth_m = float(
            self.get_parameter("display_max_depth_m").value
        )
        if self.display_max_depth_m <= 0.0:
            raise ValueError("display_max_depth_m must be positive")

        self.bridge = CvBridge()
        self.color_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None
        self.depth_encoding = ""
        self.last_probe: Optional[Tuple[int, int, Optional[float]]] = None
        self.camera_info_received = False

        self.create_subscription(
            Image,
            self.get_parameter("color_topic").value,
            self._color_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("aligned_depth_topic").value,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        cv2.namedWindow("D435 RGB", cv2.WINDOW_NORMAL)
        cv2.namedWindow("D435 Aligned Depth", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("D435 RGB", self._mouse_callback)
        self.create_timer(1.0 / 30.0, self._render)
        self.get_logger().info(
            "Depth viewer started. Left-click RGB for a 5x5 ROI median; "
            "press q or Esc to quit. No robot commands are published."
        )

    def _color_callback(self, msg: Image) -> None:
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            ).copy()
        except CvBridgeError as exc:
            self.get_logger().error(f"Color conversion failed: {exc}")

    def _depth_callback(self, msg: Image) -> None:
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            ).copy()
            self.depth_encoding = msg.encoding
        except CvBridgeError as exc:
            self.get_logger().error(f"Depth conversion failed: {exc}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_info_received:
            return
        self.camera_info_received = True
        self.get_logger().info(
            "Color CameraInfo: "
            f"frame_id={msg.header.frame_id} size={msg.width}x{msg.height} "
            f"fx={msg.k[0]:.6f} fy={msg.k[4]:.6f} "
            f"cx={msg.k[2]:.6f} cy={msg.k[5]:.6f} "
            f"distortion_model={msg.distortion_model}"
        )

    def _mouse_callback(self, event: int, u: int, v: int, _flags, _data) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.depth_image is None:
            self.get_logger().warning("No aligned-depth image received yet")
            return
        valid, total, median_m = roi_depth_median(
            self.depth_image, u, v, self.roi_size, self.depth_encoding
        )
        self.last_probe = (u, v, median_m)
        if median_m is None:
            self.get_logger().warning(
                f"Depth probe (u={u}, v={v}) ROI={self.roi_size}x"
                f"{self.roi_size}: valid={valid}/{total}, median=INVALID"
            )
        else:
            self.get_logger().info(
                f"Depth probe (u={u}, v={v}) ROI={self.roi_size}x"
                f"{self.roi_size}: valid={valid}/{total}, "
                f"median={median_m:.3f} m"
            )

    def _depth_metres(self) -> Optional[np.ndarray]:
        if self.depth_image is None:
            return None
        depth = self.depth_image.astype(np.float32)
        if self.depth_encoding in ("16UC1", "mono16"):
            return depth * 0.001
        if self.depth_encoding == "32FC1":
            return depth
        return None

    def _render(self) -> None:
        if self.color_image is not None:
            color = self.color_image.copy()
            if self.last_probe is not None:
                u, v, median_m = self.last_probe
                radius = self.roi_size // 2
                cv2.rectangle(
                    color,
                    (u - radius, v - radius),
                    (u + radius, v + radius),
                    (0, 255, 0),
                    1,
                )
                label = (
                    f"({u},{v}) {median_m:.3f} m"
                    if median_m is not None
                    else f"({u},{v}) invalid"
                )
                cv2.putText(
                    color,
                    label,
                    (max(0, u + 8), max(20, v - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
            cv2.imshow("D435 RGB", color)

        depth_m = self._depth_metres()
        if depth_m is not None:
            valid = np.isfinite(depth_m) & (depth_m > 0.0)
            scaled = np.zeros(depth_m.shape, dtype=np.uint8)
            scaled[valid] = np.clip(
                depth_m[valid] / self.display_max_depth_m * 255.0,
                0.0,
                255.0,
            ).astype(np.uint8)
            depth_color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            depth_color[~valid] = 0
            cv2.imshow("D435 Aligned Depth", depth_color)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            rclpy.shutdown()

    def close_windows(self) -> None:
        """Close OpenCV windows during normal or interrupted shutdown."""
        cv2.destroyAllWindows()


def main(args=None) -> None:
    """Run the D435 RGB/aligned-depth viewer."""
    rclpy.init(args=args)
    node = DepthViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_windows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
