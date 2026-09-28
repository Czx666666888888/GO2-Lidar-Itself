#!/usr/bin/env python3
"""Estimate the 3D center of a blue object's horizontal top surface."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


@dataclass
class PlaneCandidate:
    """One fitted plane and its indices in the original point cloud."""

    normal: np.ndarray
    offset: float
    indices: np.ndarray
    median_residual_m: float


def expected_horizontal_normal(mount_pitch_deg: float) -> np.ndarray:
    """Return world-up in optical coordinates for a downward-pitched camera."""
    pitch = math.radians(mount_pitch_deg)
    normal = np.array([0.0, -math.cos(pitch), -math.sin(pitch)])
    return normal / np.linalg.norm(normal)


def project_masked_depth(
    depth_m: np.ndarray,
    mask: np.ndarray,
    intrinsics: Sequence[float],
    min_depth_m: float,
    max_depth_m: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Back-project valid masked pixels and return points plus their (u, v)."""
    if depth_m.shape != mask.shape:
        raise ValueError("Depth and mask dimensions must match")
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("Camera focal lengths must be positive")

    valid = (
        (mask > 0)
        & np.isfinite(depth_m)
        & (depth_m >= min_depth_m)
        & (depth_m <= max_depth_m)
    )
    v, u = np.nonzero(valid)
    if u.size == 0:
        return np.empty((0, 3)), np.empty((0, 2), dtype=np.int32)
    z = depth_m[v, u].astype(np.float64)
    x = (u.astype(np.float64) - cx) * z / fx
    y = (v.astype(np.float64) - cy) * z / fy
    points = np.column_stack((x, y, z))
    pixels = np.column_stack((u, v)).astype(np.int32)
    return points, pixels


def fit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """Fit a plane to points using SVD and return unit normal and offset."""
    if points.shape[0] < 3:
        raise ValueError("At least three points are required")
    center = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = vh[-1]
    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        raise ValueError("Degenerate plane points")
    normal = normal / norm
    return normal, -float(np.dot(normal, center))


def ransac_multiple_planes(
    points: np.ndarray,
    max_planes: int,
    iterations: int,
    distance_threshold_m: float,
    min_inliers: int,
    rng: np.random.Generator,
) -> List[PlaneCandidate]:
    """Sequentially extract several RANSAC planes from a local point cloud."""
    remaining = np.arange(points.shape[0])
    candidates: List[PlaneCandidate] = []
    for _ in range(max_planes):
        if remaining.size < min_inliers:
            break
        local = points[remaining]
        best_local = np.empty(0, dtype=np.int64)
        for _ in range(iterations):
            sample_ids = rng.choice(local.shape[0], size=3, replace=False)
            p0, p1, p2 = local[sample_ids]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            normal /= norm
            distance = np.abs((local - p0) @ normal)
            inliers = np.flatnonzero(distance <= distance_threshold_m)
            if inliers.size > best_local.size:
                best_local = inliers
        if best_local.size < min_inliers:
            break

        original_indices = remaining[best_local]
        normal, offset = fit_plane_svd(points[original_indices])
        residual = np.abs(points[original_indices] @ normal + offset)
        refined_local = np.flatnonzero(
            np.abs(local @ normal + offset) <= distance_threshold_m
        )
        if refined_local.size >= min_inliers:
            original_indices = remaining[refined_local]
            normal, offset = fit_plane_svd(points[original_indices])
            residual = np.abs(points[original_indices] @ normal + offset)
            best_local = refined_local
        candidates.append(
            PlaneCandidate(
                normal=normal,
                offset=offset,
                indices=original_indices,
                median_residual_m=float(np.median(residual)),
            )
        )
        keep = np.ones(remaining.size, dtype=bool)
        keep[best_local] = False
        remaining = remaining[keep]
    return candidates


def select_horizontal_plane(
    candidates: Sequence[PlaneCandidate],
    mount_pitch_deg: float,
    max_normal_error_deg: float,
) -> Tuple[Optional[PlaneCandidate], Optional[float]]:
    """Select the plane whose unsigned normal best matches world-up prior."""
    expected = expected_horizontal_normal(mount_pitch_deg)
    ranked = []
    for candidate in candidates:
        cosine = float(np.clip(abs(np.dot(candidate.normal, expected)), 0.0, 1.0))
        angle_deg = math.degrees(math.acos(cosine))
        ranked.append((angle_deg, candidate.median_residual_m, candidate))
    if not ranked:
        return None, None
    angle_deg, _, candidate = min(ranked, key=lambda item: (item[0], item[1]))
    if angle_deg > max_normal_error_deg:
        return None, angle_deg
    return candidate, angle_deg


def robust_center(points: np.ndarray) -> np.ndarray:
    """Return the component-wise median of a non-empty point set."""
    if points.shape[0] == 0:
        raise ValueError("Cannot calculate the center of an empty point set")
    return np.median(points, axis=0)


class BlueSurfaceCenter(Node):
    """Segment blue pixels, fit planes and publish a valid top center."""

    def __init__(self) -> None:
        super().__init__("blue_surface_center")
        defaults = {
            "color_topic": "/camera/camera/color/image_raw",
            "aligned_depth_topic": (
                "/camera/camera/aligned_depth_to_color/image_raw"
            ),
            "camera_info_topic": "/camera/camera/color/camera_info",
            "output_topic": "/science/blue_surface_center",
            "h_min": 90,
            "h_max": 135,
            "s_min": 80,
            "v_min": 50,
            "min_component_area_px": 200,
            "erosion_kernel_px": 3,
            "erosion_iterations": 2,
            "min_depth_m": 0.15,
            "max_depth_m": 5.0,
            "min_depth_points": 100,
            "max_cloud_points": 12000,
            "ransac_max_planes": 4,
            "ransac_iterations": 120,
            "ransac_distance_threshold_m": 0.008,
            "ransac_min_inliers": 60,
            "min_surface_inlier_ratio": 0.08,
            "mount_pitch_deg": 45.0,
            "max_normal_error_deg": 25.0,
            "max_rgb_depth_delta_sec": 0.20,
            "processing_rate_hz": 8.0,
            "show_debug": True,
            "random_seed": 7,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.bridge = CvBridge()
        self.color: Optional[np.ndarray] = None
        self.depth_m: Optional[np.ndarray] = None
        self.color_stamp = 0.0
        self.depth_stamp = 0.0
        self.depth_header = None
        self.last_processed_stamp = -1.0
        self.intrinsics: Optional[Tuple[float, float, float, float]] = None
        self.camera_frame = ""
        self.camera_size = (0, 0)
        self.last_status = "waiting for RGB, aligned depth and CameraInfo"
        self.last_invalid_log_time = -math.inf
        self.last_valid_log_time = -math.inf
        self.rng = np.random.default_rng(
            int(self.get_parameter("random_seed").value)
        )
        self.publisher = self.create_publisher(
            PointStamped, self.get_parameter("output_topic").value, 10
        )
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
        rate = float(self.get_parameter("processing_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._process)
        if bool(self.get_parameter("show_debug").value):
            cv2.namedWindow("Blue Surface Center", cv2.WINDOW_NORMAL)
        self.get_logger().info(
            "Blue surface center started; output remains in CameraInfo frame."
        )

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _color_callback(self, msg: Image) -> None:
        try:
            self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            self.color_stamp = self._stamp_seconds(msg.header.stamp)
        except CvBridgeError as exc:
            self._invalid(f"RGB conversion failed: {exc}")

    def _depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(
                np.float32
            )
            if msg.encoding in ("16UC1", "mono16"):
                depth *= 0.001
            elif msg.encoding != "32FC1":
                self._invalid(f"unsupported depth encoding {msg.encoding}")
                return
            self.depth_m = depth
            self.depth_stamp = self._stamp_seconds(msg.header.stamp)
            self.depth_header = msg.header
        except CvBridgeError as exc:
            self._invalid(f"depth conversion failed: {exc}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.camera_frame = msg.header.frame_id
        self.camera_size = (msg.width, msg.height)

    def _largest_blue_mask(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array(
            [
                int(self.get_parameter("h_min").value),
                int(self.get_parameter("s_min").value),
                int(self.get_parameter("v_min").value),
            ],
            dtype=np.uint8,
        )
        upper = np.array(
            [int(self.get_parameter("h_max").value), 255, 255], dtype=np.uint8
        )
        raw = cv2.inRange(hsv, lower, upper)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(raw, 8)
        mask = np.zeros_like(raw)
        contours = np.empty((0, 1, 2), dtype=np.int32)
        if count <= 1:
            return mask, contours
        component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = int(stats[component, cv2.CC_STAT_AREA])
        if area < int(self.get_parameter("min_component_area_px").value):
            return mask, contours
        mask[labels == component] = 255
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        kernel_size = int(self.get_parameter("erosion_kernel_px").value)
        kernel_size = max(1, kernel_size | 1)
        iterations = max(
            0, int(self.get_parameter("erosion_iterations").value)
        )
        if iterations:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
            )
            mask = cv2.erode(mask, kernel, iterations=iterations)
        return mask, contours

    def _invalid(self, reason: str) -> None:
        status = f"INVALID: {reason}"
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_invalid_log_time >= 2.0:
            self.get_logger().warning(status)
            self.last_invalid_log_time = now
        self.last_status = status

    def _draw_debug(
        self,
        contours: np.ndarray,
        selected_pixels: Optional[np.ndarray] = None,
        center_pixel: Optional[Tuple[int, int]] = None,
        center: Optional[np.ndarray] = None,
        valid_count: int = 0,
    ) -> None:
        if not bool(self.get_parameter("show_debug").value) or self.color is None:
            return
        debug = self.color.copy()
        if len(contours):
            cv2.drawContours(debug, contours, -1, (255, 255, 0), 2)
        if selected_pixels is not None and selected_pixels.size:
            overlay = debug.copy()
            overlay[selected_pixels[:, 1], selected_pixels[:, 0]] = (0, 255, 255)
            debug = cv2.addWeighted(overlay, 0.45, debug, 0.55, 0.0)
        if center_pixel is not None:
            cv2.drawMarker(
                debug,
                center_pixel,
                (0, 0, 255),
                cv2.MARKER_CROSS,
                18,
                2,
            )
        lines = [self.last_status, f"valid depth points: {valid_count}"]
        if center is not None:
            lines.append(
                f"X={center[0]:.3f} Y={center[1]:.3f} Z={center[2]:.3f} m"
            )
        for index, line in enumerate(lines):
            cv2.putText(
                debug,
                line,
                (10, 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 0) if center is not None else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        cv2.imshow("Blue Surface Center", debug)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            rclpy.shutdown()

    def _process(self) -> None:
        if self.color is None or self.depth_m is None or self.intrinsics is None:
            self._invalid("waiting for RGB, aligned depth and CameraInfo")
            self._draw_debug(np.empty((0, 1, 2), dtype=np.int32))
            return
        if self.depth_stamp == self.last_processed_stamp:
            self._draw_debug(np.empty((0, 1, 2), dtype=np.int32))
            return
        self.last_processed_stamp = self.depth_stamp
        if self.color.shape[:2] != self.depth_m.shape:
            self._invalid("RGB and aligned-depth dimensions differ")
            self._draw_debug(np.empty((0, 1, 2), dtype=np.int32))
            return
        width, height = self.camera_size
        if (width, height) != (self.color.shape[1], self.color.shape[0]):
            self._invalid("CameraInfo dimensions do not match images")
            self._draw_debug(np.empty((0, 1, 2), dtype=np.int32))
            return
        max_delta = float(
            self.get_parameter("max_rgb_depth_delta_sec").value
        )
        if abs(self.color_stamp - self.depth_stamp) > max_delta:
            self._invalid("RGB/aligned-depth timestamps are too far apart")
            self._draw_debug(np.empty((0, 1, 2), dtype=np.int32))
            return

        mask, contours = self._largest_blue_mask(self.color)
        if not np.any(mask):
            self._invalid("no sufficiently large blue component")
            self._draw_debug(contours)
            return
        points, pixels = project_masked_depth(
            self.depth_m,
            mask,
            self.intrinsics,
            float(self.get_parameter("min_depth_m").value),
            float(self.get_parameter("max_depth_m").value),
        )
        valid_count = points.shape[0]
        min_points = int(self.get_parameter("min_depth_points").value)
        if valid_count < min_points:
            self._invalid(f"only {valid_count} valid masked depth points")
            self._draw_debug(contours, valid_count=valid_count)
            return

        max_points = int(self.get_parameter("max_cloud_points").value)
        if points.shape[0] > max_points:
            chosen = self.rng.choice(points.shape[0], max_points, replace=False)
            points = points[chosen]
            pixels = pixels[chosen]
        candidates = ransac_multiple_planes(
            points,
            int(self.get_parameter("ransac_max_planes").value),
            int(self.get_parameter("ransac_iterations").value),
            float(self.get_parameter("ransac_distance_threshold_m").value),
            int(self.get_parameter("ransac_min_inliers").value),
            self.rng,
        )
        selected, normal_error = select_horizontal_plane(
            candidates,
            float(self.get_parameter("mount_pitch_deg").value),
            float(self.get_parameter("max_normal_error_deg").value),
        )
        if selected is None:
            detail = (
                "no plane candidate"
                if normal_error is None
                else f"best plane normal error {normal_error:.1f} deg"
            )
            self._invalid(detail)
            self._draw_debug(contours, valid_count=valid_count)
            return

        surface_points = points[selected.indices]
        surface_pixels = pixels[selected.indices]
        inlier_ratio = surface_points.shape[0] / float(points.shape[0])
        min_inlier_ratio = float(
            self.get_parameter("min_surface_inlier_ratio").value
        )
        if inlier_ratio < min_inlier_ratio:
            self._invalid(
                f"top plane inlier ratio {inlier_ratio:.3f} below "
                f"{min_inlier_ratio:.3f}"
            )
            self._draw_debug(contours, valid_count=valid_count)
            return
        center = robust_center(surface_points)
        center_uv = tuple(
            np.rint(np.median(surface_pixels, axis=0)).astype(int).tolist()
        )
        point = PointStamped()
        point.header = self.depth_header
        point.header.frame_id = self.camera_frame
        point.point.x = float(center[0])
        point.point.y = float(center[1])
        point.point.z = float(center[2])
        self.publisher.publish(point)
        self.last_status = (
            f"VALID: top plane inliers={surface_points.shape[0]} "
            f"normal_error={normal_error:.1f} deg"
        )
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_valid_log_time >= 2.0:
            self.get_logger().info(
                f"{self.last_status}; center=({center[0]:.3f}, "
                f"{center[1]:.3f}, {center[2]:.3f}) m; "
                f"frame_id={self.camera_frame}"
            )
            self.last_valid_log_time = now
        self._draw_debug(
            contours,
            surface_pixels,
            center_uv,
            center,
            valid_count,
        )

    def close_windows(self) -> None:
        """Close this node's OpenCV window."""
        if bool(self.get_parameter("show_debug").value):
            cv2.destroyWindow("Blue Surface Center")


def main(args=None) -> None:
    """Run blue surface center estimation."""
    rclpy.init(args=args)
    node = BlueSurfaceCenter()
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
