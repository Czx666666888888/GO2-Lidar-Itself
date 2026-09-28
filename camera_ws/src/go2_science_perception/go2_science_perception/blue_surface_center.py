#!/usr/bin/env python3
"""Estimate the 3D center of a blue object's horizontal top surface."""

from dataclasses import dataclass
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


def ransac_dominant_plane(
    points: np.ndarray,
    iterations: int,
    distance_threshold_m: float,
    min_inliers: int,
    rng: np.random.Generator,
) -> Optional[PlaneCandidate]:
    """Fit one dominant plane to a 2D-prior-filtered local point cloud."""
    if points.shape[0] < min_inliers:
        return None
    best = np.empty(0, dtype=np.int64)
    for _ in range(iterations):
        sample_ids = rng.choice(points.shape[0], size=3, replace=False)
        p0, p1, p2 = points[sample_ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        inliers = np.flatnonzero(
            np.abs((points - p0) @ normal) <= distance_threshold_m
        )
        if inliers.size > best.size:
            best = inliers
    if best.size < min_inliers:
        return None
    normal, offset = fit_plane_svd(points[best])
    refined = np.flatnonzero(
        np.abs(points @ normal + offset) <= distance_threshold_m
    )
    if refined.size >= min_inliers:
        best = refined
        normal, offset = fit_plane_svd(points[best])
    residual = np.abs(points[best] @ normal + offset)
    return PlaneCandidate(normal, offset, best, float(np.median(residual)))


def plane_normal_error_deg(
    candidate: PlaneCandidate,
    mount_pitch_deg: float,
) -> float:
    """Return unsigned angle between a plane normal and horizontal prior."""
    expected = expected_horizontal_normal(mount_pitch_deg)
    cosine = float(np.clip(abs(np.dot(candidate.normal, expected)), 0.0, 1.0))
    return math.degrees(math.acos(cosine))


def top_region_mask(
    target_mask: np.ndarray,
    ratio: float,
    erosion_kernel_px: int,
    erosion_iterations: int,
) -> np.ndarray:
    """Keep the upper fraction of a target bounding box and erode its edge."""
    if not 0.0 < ratio <= 1.0:
        raise ValueError("top region ratio must be in (0, 1]")
    rows, _ = np.nonzero(target_mask)
    result = np.zeros_like(target_mask)
    if rows.size == 0:
        return result
    y_min = int(rows.min())
    y_max = int(rows.max())
    cutoff = y_min + int(math.ceil((y_max - y_min + 1) * ratio))
    result[y_min:min(cutoff, y_max + 1)] = target_mask[
        y_min:min(cutoff, y_max + 1)
    ]
    if erosion_iterations > 0:
        size = max(1, erosion_kernel_px | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        result = cv2.erode(result, kernel, iterations=erosion_iterations)
    return result


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
            "morphology_kernel_px": 5,
            "morphology_close_iterations": 1,
            "morphology_open_iterations": 1,
            "top_region_ratio": 0.55,
            "top_erosion_kernel_px": 3,
            "top_erosion_iterations": 1,
            "min_depth_m": 0.15,
            "max_depth_m": 5.0,
            "min_depth_points": 100,
            "max_cloud_points": 12000,
            "ransac_iterations": 120,
            "ransac_distance_threshold_m": 0.008,
            "ransac_min_inliers": 60,
            "min_surface_inlier_ratio": 0.08,
            "mount_pitch_deg": 45.0,
            "max_normal_error_deg": 35.0,
            "max_rgb_depth_delta_sec": 0.20,
            "processing_rate_hz": 8.0,
            "validation_duration_sec": 0.0,
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
        self.metrics_start_time: Optional[float] = None
        self.last_metrics_log_time = -math.inf
        self.processed_frames = 0
        self.valid_frames = 0
        self.invalid_frames = 0
        self.loss_events = 0
        self.previous_valid: Optional[bool] = None
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

    def _largest_blue_mask(
        self, image: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
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
        size = max(
            1, int(self.get_parameter("morphology_kernel_px").value) | 1
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        cleaned = cv2.morphologyEx(
            raw,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=max(
                0,
                int(
                    self.get_parameter("morphology_close_iterations").value
                ),
            ),
        )
        cleaned = cv2.morphologyEx(
            cleaned,
            cv2.MORPH_OPEN,
            kernel,
            iterations=max(
                0,
                int(self.get_parameter("morphology_open_iterations").value),
            ),
        )
        count, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, 8)
        mask = np.zeros_like(raw)
        contours = np.empty((0, 1, 2), dtype=np.int32)
        if count <= 1:
            return raw, mask, contours
        component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = int(stats[component, cv2.CC_STAT_AREA])
        if area < int(self.get_parameter("min_component_area_px").value):
            return raw, mask, contours
        mask[labels == component] = 255
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return raw, mask, contours

    def _record_outcome(self, valid: bool) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.metrics_start_time is None:
            self.metrics_start_time = now
        self.processed_frames += 1
        if valid:
            self.valid_frames += 1
        else:
            self.invalid_frames += 1
            if self.previous_valid is True:
                self.loss_events += 1
        self.previous_valid = valid
        if now - self.last_metrics_log_time >= 5.0:
            ratio = self.valid_frames / float(self.processed_frames)
            self.get_logger().info(
                f"Detection summary: frames={self.processed_frames} "
                f"valid={self.valid_frames} invalid={self.invalid_frames} "
                f"loss_events={self.loss_events} valid_ratio={ratio:.3f}"
            )
            self.last_metrics_log_time = now
        duration = float(self.get_parameter("validation_duration_sec").value)
        if duration > 0.0 and now - self.metrics_start_time >= duration:
            self.get_logger().info(f"Final {self._summary_text()}")
            rclpy.shutdown()

    def _summary_text(self) -> str:
        ratio = (
            self.valid_frames / float(self.processed_frames)
            if self.processed_frames
            else 0.0
        )
        return (
            f"detection summary: frames={self.processed_frames} "
            f"valid={self.valid_frames} invalid={self.invalid_frames} "
            f"loss_events={self.loss_events} valid_ratio={ratio:.3f}"
        )

    def _invalid(self, reason: str) -> None:
        status = f"INVALID: {reason}"
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_invalid_log_time >= 2.0:
            self.get_logger().warning(status)
            self.last_invalid_log_time = now
        self.last_status = status

    def _draw_debug(
        self,
        raw_mask: np.ndarray,
        target_mask: np.ndarray,
        top_mask: np.ndarray,
        contours: np.ndarray,
        inlier_pixels: Optional[np.ndarray] = None,
        center_pixel: Optional[Tuple[int, int]] = None,
        center: Optional[np.ndarray] = None,
        mask_area: int = 0,
        valid_depth_ratio: float = 0.0,
        inlier_ratio: float = 0.0,
        normal_error: Optional[float] = None,
    ) -> None:
        if not bool(self.get_parameter("show_debug").value) or self.color is None:
            return
        annotated = self.color.copy()
        if len(contours):
            cv2.drawContours(annotated, contours, -1, (255, 255, 0), 2)
        if inlier_pixels is not None and inlier_pixels.size:
            annotated[inlier_pixels[:, 1], inlier_pixels[:, 0]] = (0, 255, 255)
        if center_pixel is not None:
            cv2.drawMarker(
                annotated,
                center_pixel,
                (0, 0, 255),
                cv2.MARKER_CROSS,
                18,
                2,
            )
        error_text = "n/a" if normal_error is None else f"{normal_error:.1f} deg"
        lines = [
            self.last_status,
            f"mask_area={mask_area} valid_depth_ratio={valid_depth_ratio:.3f}",
            f"inlier_ratio={inlier_ratio:.3f} normal_error={error_text}",
        ]
        if center is not None:
            lines.append(
                f"X={center[0]:.3f} Y={center[1]:.3f} Z={center[2]:.3f} m"
            )
        for index, line in enumerate(lines):
            cv2.putText(
                annotated,
                line,
                (10, 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 0) if center is not None else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        def mask_panel(mask: np.ndarray, label: str) -> np.ndarray:
            panel = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(
                panel, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 255, 0), 2, cv2.LINE_AA
            )
            return panel

        top_panel = mask_panel(top_mask, "top candidate ROI / RANSAC inliers")
        if inlier_pixels is not None and inlier_pixels.size:
            top_panel[inlier_pixels[:, 1], inlier_pixels[:, 0]] = (0, 255, 255)
        debug = np.vstack(
            (
                np.hstack((annotated, mask_panel(raw_mask, "raw HSV mask"))),
                np.hstack(
                    (mask_panel(target_mask, "final target mask"), top_panel)
                ),
            )
        )
        cv2.imshow("Blue Surface Center", debug)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            rclpy.shutdown()

    def _process(self) -> None:
        if self.color is None or self.depth_m is None or self.intrinsics is None:
            self._invalid("waiting for RGB, aligned depth and CameraInfo")
            return
        if self.depth_stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = self.depth_stamp
        empty = np.zeros(self.color.shape[:2], dtype=np.uint8)
        no_contours = np.empty((0, 1, 2), dtype=np.int32)
        if self.color.shape[:2] != self.depth_m.shape:
            self._invalid("RGB and aligned-depth dimensions differ")
            self._record_outcome(False)
            self._draw_debug(empty, empty, empty, no_contours)
            return
        width, height = self.camera_size
        if (width, height) != (self.color.shape[1], self.color.shape[0]):
            self._invalid("CameraInfo dimensions do not match images")
            self._record_outcome(False)
            self._draw_debug(empty, empty, empty, no_contours)
            return
        max_delta = float(
            self.get_parameter("max_rgb_depth_delta_sec").value
        )
        if abs(self.color_stamp - self.depth_stamp) > max_delta:
            self._invalid("RGB/aligned-depth timestamps are too far apart")
            self._record_outcome(False)
            self._draw_debug(empty, empty, empty, no_contours)
            return

        raw_mask, target_mask, contours = self._largest_blue_mask(self.color)
        mask_area = int(np.count_nonzero(target_mask))
        if mask_area == 0:
            self._invalid("no sufficiently large blue component")
            self._record_outcome(False)
            self._draw_debug(raw_mask, target_mask, empty, contours)
            return
        top_mask = top_region_mask(
            target_mask,
            float(self.get_parameter("top_region_ratio").value),
            int(self.get_parameter("top_erosion_kernel_px").value),
            int(self.get_parameter("top_erosion_iterations").value),
        )
        top_area = int(np.count_nonzero(top_mask))
        if top_area == 0:
            self._invalid("top candidate ROI is empty after erosion")
            self._record_outcome(False)
            self._draw_debug(
                raw_mask,
                target_mask,
                top_mask,
                contours,
                mask_area=mask_area,
            )
            return
        points, pixels = project_masked_depth(
            self.depth_m,
            top_mask,
            self.intrinsics,
            float(self.get_parameter("min_depth_m").value),
            float(self.get_parameter("max_depth_m").value),
        )
        valid_count = points.shape[0]
        valid_depth_ratio = valid_count / float(top_area)
        min_points = int(self.get_parameter("min_depth_points").value)
        if valid_count < min_points:
            self._invalid(f"only {valid_count} valid top-ROI depth points")
            self._record_outcome(False)
            self._draw_debug(
                raw_mask,
                target_mask,
                top_mask,
                contours,
                mask_area=mask_area,
                valid_depth_ratio=valid_depth_ratio,
            )
            return

        max_points = int(self.get_parameter("max_cloud_points").value)
        if points.shape[0] > max_points:
            chosen = self.rng.choice(points.shape[0], max_points, replace=False)
            points = points[chosen]
            pixels = pixels[chosen]
        selected = ransac_dominant_plane(
            points,
            int(self.get_parameter("ransac_iterations").value),
            float(self.get_parameter("ransac_distance_threshold_m").value),
            int(self.get_parameter("ransac_min_inliers").value),
            self.rng,
        )
        if selected is None:
            self._invalid("no reliable dominant plane in top ROI")
            self._record_outcome(False)
            self._draw_debug(
                raw_mask,
                target_mask,
                top_mask,
                contours,
                mask_area=mask_area,
                valid_depth_ratio=valid_depth_ratio,
            )
            return

        surface_points = points[selected.indices]
        surface_pixels = pixels[selected.indices]
        inlier_ratio = surface_points.shape[0] / float(points.shape[0])
        normal_error = plane_normal_error_deg(
            selected, float(self.get_parameter("mount_pitch_deg").value)
        )
        max_normal_error = float(
            self.get_parameter("max_normal_error_deg").value
        )
        if normal_error > max_normal_error:
            self._invalid(f"plane normal error {normal_error:.1f} deg")
            self._record_outcome(False)
            self._draw_debug(
                raw_mask,
                target_mask,
                top_mask,
                contours,
                surface_pixels,
                mask_area=mask_area,
                valid_depth_ratio=valid_depth_ratio,
                inlier_ratio=inlier_ratio,
                normal_error=normal_error,
            )
            return
        min_inlier_ratio = float(
            self.get_parameter("min_surface_inlier_ratio").value
        )
        if inlier_ratio < min_inlier_ratio:
            self._invalid(
                f"dominant plane inlier ratio {inlier_ratio:.3f} below "
                f"{min_inlier_ratio:.3f}"
            )
            self._record_outcome(False)
            self._draw_debug(
                raw_mask,
                target_mask,
                top_mask,
                contours,
                surface_pixels,
                mask_area=mask_area,
                valid_depth_ratio=valid_depth_ratio,
                inlier_ratio=inlier_ratio,
                normal_error=normal_error,
            )
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
        self._record_outcome(True)
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
            raw_mask,
            target_mask,
            top_mask,
            contours,
            surface_pixels,
            center_uv,
            center,
            mask_area,
            valid_depth_ratio,
            inlier_ratio,
            normal_error,
        )

    def close_windows(self) -> None:
        """Close this node's OpenCV window."""
        if self.processed_frames:
            self.get_logger().info(f"Final {self._summary_text()}")
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
