#!/usr/bin/env python3
"""Publish a coarse blue-target point from mask centroid and median depth."""

import math
from typing import Optional, Sequence, Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def largest_blue_mask(
    image: np.ndarray,
    hsv_lower: Sequence[int],
    hsv_upper: Sequence[int],
    morphology_kernel_px: int,
    close_iterations: int,
    open_iterations: int,
    min_component_area_px: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Apply the precise detector's HSV/morphology/component mask rules."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    raw = cv2.inRange(
        hsv, np.asarray(hsv_lower, dtype=np.uint8),
        np.asarray(hsv_upper, dtype=np.uint8)
    )
    size = max(1, int(morphology_kernel_px) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    cleaned = cv2.morphologyEx(
        raw, cv2.MORPH_CLOSE, kernel, iterations=max(0, close_iterations)
    )
    cleaned = cv2.morphologyEx(
        cleaned, cv2.MORPH_OPEN, kernel, iterations=max(0, open_iterations)
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, 8)
    mask = np.zeros_like(raw)
    contours = np.empty((0, 1, 2), dtype=np.int32)
    if count <= 1:
        return raw, mask, contours, 0
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[component, cv2.CC_STAT_AREA])
    if area < min_component_area_px:
        return raw, mask, contours, area
    mask[labels == component] = 255
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return raw, mask, contours, area


def mask_centroid(mask: np.ndarray) -> Tuple[float, float]:
    """Return the image-moment centroid of a non-empty binary mask."""
    moments = cv2.moments(mask, binaryImage=True)
    if moments["m00"] <= 0.0:
        raise ValueError("Cannot calculate centroid of an empty mask")
    return moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]


def erode_target_mask(
    mask: np.ndarray, kernel_px: int, iterations: int
) -> np.ndarray:
    """Slightly erode a target mask before sampling aligned depth."""
    size = max(1, int(kernel_px) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return cv2.erode(mask, kernel, iterations=max(0, iterations))


def median_mask_depth(
    depth_m: np.ndarray,
    mask: np.ndarray,
    min_depth_m: float,
    max_depth_m: float,
) -> Tuple[float, int]:
    """Return median valid depth and sample count inside a binary mask."""
    if depth_m.shape != mask.shape:
        raise ValueError("Depth and mask dimensions must match")
    valid = (
        (mask > 0)
        & np.isfinite(depth_m)
        & (depth_m >= min_depth_m)
        & (depth_m <= max_depth_m)
    )
    values = depth_m[valid]
    if values.size == 0:
        return math.nan, 0
    return float(np.median(values)), int(values.size)


def back_project_pixel(
    u: float, v: float, depth_m: float, intrinsics: Sequence[float]
) -> np.ndarray:
    """Back-project one color pixel using runtime pinhole intrinsics."""
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if fx <= 0.0 or fy <= 0.0 or not math.isfinite(depth_m):
        raise ValueError("Valid focal lengths and depth are required")
    return np.array(
        [(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m],
        dtype=np.float64,
    )


class CoarseTargetLocator(Node):
    """Locate a blue target without plane fitting or semantic surface checks."""

    def __init__(self) -> None:
        super().__init__("coarse_target_locator")
        defaults = {
            "color_topic": "/camera/camera/color/image_raw",
            "aligned_depth_topic": (
                "/camera/camera/aligned_depth_to_color/image_raw"
            ),
            "camera_info_topic": "/camera/camera/color/camera_info",
            "output_topic": "/science/target_coarse_point",
            "h_min": 90,
            "h_max": 135,
            "s_min": 80,
            "v_min": 50,
            "min_component_area_px": 100,
            "morphology_kernel_px": 5,
            "morphology_close_iterations": 1,
            "morphology_open_iterations": 1,
            "depth_erosion_kernel_px": 3,
            "depth_erosion_iterations": 1,
            "min_depth_m": 0.15,
            "max_depth_m": 5.0,
            "min_valid_depth_points": 20,
            "max_rgb_depth_delta_sec": 0.20,
            "processing_rate_hz": 8.0,
            "validation_duration_sec": 0.0,
            "show_debug": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.bridge = CvBridge()
        self.color: Optional[np.ndarray] = None
        self.depth_m: Optional[np.ndarray] = None
        self.color_stamp = 0.0
        self.depth_stamp = 0.0
        self.last_processed_stamp = -1.0
        self.depth_header = None
        self.intrinsics: Optional[Tuple[float, float, float, float]] = None
        self.camera_frame = ""
        self.camera_size = (0, 0)
        self.started_at: Optional[float] = None
        self.processed_frames = 0
        self.published_frames = 0
        self.invalid_frames = 0
        self.last_warning_time = -math.inf
        self.publisher = self.create_publisher(
            PointStamped, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("color_topic").value,
            self._color_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.get_parameter("aligned_depth_topic").value,
            self._depth_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._camera_info_callback, qos_profile_sensor_data
        )
        rate = float(self.get_parameter("processing_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._process)
        if bool(self.get_parameter("show_debug").value):
            cv2.namedWindow("Coarse Target Locator", cv2.WINDOW_NORMAL)
        self.get_logger().info(
            "Coarse target locator started; no RANSAC or normal filtering."
        )

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _color_callback(self, msg: Image) -> None:
        try:
            self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            self.color_stamp = self._stamp_seconds(msg.header.stamp)
        except CvBridgeError as exc:
            self._warn(f"RGB conversion failed: {exc}")

    def _depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(
                np.float32
            )
            if msg.encoding in ("16UC1", "mono16"):
                depth *= 0.001
            elif msg.encoding != "32FC1":
                self._warn(f"unsupported depth encoding {msg.encoding}")
                return
            self.depth_m = depth
            self.depth_stamp = self._stamp_seconds(msg.header.stamp)
            self.depth_header = msg.header
        except CvBridgeError as exc:
            self._warn(f"depth conversion failed: {exc}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.camera_frame = msg.header.frame_id
        self.camera_size = (msg.width, msg.height)

    def _warn(self, reason: str) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_warning_time >= 2.0:
            self.get_logger().warning(f"INVALID: {reason}")
            self.last_warning_time = now

    def _record(self, valid: bool) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.started_at is None:
            self.started_at = now
        self.processed_frames += 1
        if valid:
            self.published_frames += 1
        else:
            self.invalid_frames += 1
        duration = float(self.get_parameter("validation_duration_sec").value)
        if duration > 0.0 and now - self.started_at >= duration:
            self._log_summary()
            rclpy.shutdown()

    def _log_summary(self) -> None:
        ratio = (
            self.published_frames / float(self.processed_frames)
            if self.processed_frames else 0.0
        )
        self.get_logger().info(
            f"FINAL_COARSE frames={self.processed_frames} "
            f"published={self.published_frames} invalid={self.invalid_frames} "
            f"publish_ratio={ratio:.3f}"
        )

    def _draw_debug(
        self,
        raw_mask: np.ndarray,
        target_mask: np.ndarray,
        depth_mask: np.ndarray,
        contours: np.ndarray,
        centroid: Optional[Tuple[float, float]] = None,
        depth_m: Optional[float] = None,
        point: Optional[np.ndarray] = None,
        valid_depth_points: int = 0,
    ) -> None:
        if not bool(self.get_parameter("show_debug").value) or self.color is None:
            return
        annotated = self.color.copy()
        if len(contours):
            cv2.drawContours(annotated, contours, -1, (255, 255, 0), 2)
        if centroid is not None:
            uv = tuple(np.rint(centroid).astype(int).tolist())
            cv2.drawMarker(
                annotated, uv, (0, 0, 255), cv2.MARKER_CROSS, 20, 2
            )
        lines = [
            f"centroid={centroid}" if centroid is not None else "centroid=n/a",
            f"median_depth={depth_m:.3f} m" if depth_m is not None
            else "median_depth=n/a",
            f"valid_depth_points={valid_depth_points}",
        ]
        if point is not None:
            lines.append(
                f"XYZ=({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) m"
            )
        for index, line in enumerate(lines):
            cv2.putText(
                annotated, line, (10, 28 + 25 * index),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                cv2.LINE_AA
            )
        panels = [
            annotated,
            cv2.cvtColor(raw_mask, cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(target_mask, cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(depth_mask, cv2.COLOR_GRAY2BGR),
        ]
        debug = np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))
        cv2.imshow("Coarse Target Locator", debug)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            rclpy.shutdown()

    def _process(self) -> None:
        if self.color is None or self.depth_m is None or self.intrinsics is None:
            self._warn("waiting for RGB, aligned depth and CameraInfo")
            return
        if self.depth_stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = self.depth_stamp
        empty = np.zeros(self.color.shape[:2], dtype=np.uint8)
        no_contours = np.empty((0, 1, 2), dtype=np.int32)
        if (
            self.color.shape[:2] != self.depth_m.shape
            or self.camera_size != (self.color.shape[1], self.color.shape[0])
            or abs(self.color_stamp - self.depth_stamp) > float(
                self.get_parameter("max_rgb_depth_delta_sec").value
            )
        ):
            self._warn("RGB, aligned depth and CameraInfo are not compatible")
            self._record(False)
            self._draw_debug(empty, empty, empty, no_contours)
            return

        raw, target, contours, largest_area = largest_blue_mask(
            self.color,
            (
                int(self.get_parameter("h_min").value),
                int(self.get_parameter("s_min").value),
                int(self.get_parameter("v_min").value),
            ),
            (int(self.get_parameter("h_max").value), 255, 255),
            int(self.get_parameter("morphology_kernel_px").value),
            int(self.get_parameter("morphology_close_iterations").value),
            int(self.get_parameter("morphology_open_iterations").value),
            int(self.get_parameter("min_component_area_px").value),
        )
        if not np.any(target):
            self._warn(
                "no blue target" if largest_area == 0 else "blue target too small"
            )
            self._record(False)
            self._draw_debug(raw, target, empty, contours)
            return
        centroid = mask_centroid(target)
        depth_mask = erode_target_mask(
            target,
            int(self.get_parameter("depth_erosion_kernel_px").value),
            int(self.get_parameter("depth_erosion_iterations").value),
        )
        depth_m, valid_count = median_mask_depth(
            self.depth_m, depth_mask,
            float(self.get_parameter("min_depth_m").value),
            float(self.get_parameter("max_depth_m").value),
        )
        min_points = int(self.get_parameter("min_valid_depth_points").value)
        if valid_count < min_points:
            self._warn(f"only {valid_count} valid eroded-mask depth points")
            self._record(False)
            self._draw_debug(
                raw, target, depth_mask, contours, centroid,
                valid_depth_points=valid_count
            )
            return
        xyz = back_project_pixel(*centroid, depth_m, self.intrinsics)
        point = PointStamped()
        point.header = self.depth_header
        point.header.frame_id = self.camera_frame
        point.point.x, point.point.y, point.point.z = (float(x) for x in xyz)
        self.publisher.publish(point)
        self.get_logger().info(
            f"COARSE_METRICS mask_area={int(np.count_nonzero(target))} "
            f"centroid_u={centroid[0]:.2f} centroid_v={centroid[1]:.2f} "
            f"valid_depth_points={valid_count} median_depth_m={depth_m:.3f} "
            f"X={xyz[0]:.3f} Y={xyz[1]:.3f} Z={xyz[2]:.3f} "
            f"frame_id={self.camera_frame}"
        )
        self._record(True)
        self._draw_debug(
            raw, target, depth_mask, contours, centroid, depth_m, xyz, valid_count
        )

    def close_windows(self) -> None:
        """Log final counters and close this node's debug window."""
        if self.processed_frames:
            self._log_summary()
        if bool(self.get_parameter("show_debug").value):
            cv2.destroyWindow("Coarse Target Locator")


def main(args=None) -> None:
    """Run coarse target localization."""
    rclpy.init(args=args)
    node = CoarseTargetLocator()
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
