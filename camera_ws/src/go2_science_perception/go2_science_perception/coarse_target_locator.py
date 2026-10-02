#!/usr/bin/env python3
"""Publish coarse points for multiple blue targets after blue-wall removal."""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


@dataclass
class WallResult:
    """Dominant vertical blue plane and its rejected image mask."""

    mask: np.ndarray
    detected: bool = False
    support_points: int = 0
    support_ratio: float = 0.0
    image_fraction: float = 0.0
    vertical_error_deg: float = math.nan


@dataclass
class TargetResult:
    """One remaining blue target component."""

    mask: np.ndarray
    centroid: Tuple[float, float]
    median_depth_m: float
    valid_depth_points: int
    xyz: np.ndarray


def blue_candidate_mask(image, hsv_lower, hsv_upper, kernel_px, close_n, open_n):
    """Return raw and cleaned HSV blue masks without selecting one component."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    raw = cv2.inRange(
        hsv, np.asarray(hsv_lower, dtype=np.uint8),
        np.asarray(hsv_upper, dtype=np.uint8)
    )
    size = max(1, int(kernel_px) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    cleaned = cv2.morphologyEx(
        raw, cv2.MORPH_CLOSE, kernel, iterations=max(0, close_n)
    )
    cleaned = cv2.morphologyEx(
        cleaned, cv2.MORPH_OPEN, kernel, iterations=max(0, open_n)
    )
    return raw, cleaned


def mask_centroid(mask: np.ndarray) -> Tuple[float, float]:
    """Return the image-moment centroid of a non-empty mask."""
    moments = cv2.moments(mask, binaryImage=True)
    if moments["m00"] <= 0.0:
        raise ValueError("Cannot calculate centroid of an empty mask")
    return moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]


def erode_target_mask(mask, kernel_px, iterations):
    """Erode a target mask before reporting depth."""
    size = max(1, int(kernel_px) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return cv2.erode(mask, kernel, iterations=max(0, iterations))


def median_mask_depth(depth_m, mask, min_depth_m, max_depth_m):
    """Return median valid depth and sample count inside a mask."""
    if depth_m.shape != mask.shape:
        raise ValueError("Depth and mask dimensions must match")
    valid = (
        (mask > 0) & np.isfinite(depth_m)
        & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    )
    values = depth_m[valid]
    if values.size == 0:
        return math.nan, 0
    return float(np.median(values)), int(values.size)


def back_project_pixel(u, v, depth_m, intrinsics):
    """Back-project one pixel using runtime color intrinsics."""
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if fx <= 0.0 or fy <= 0.0 or not math.isfinite(depth_m):
        raise ValueError("Valid focal lengths and depth are required")
    return np.array(
        [(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m],
        dtype=np.float64,
    )


def project_masked_depth(depth_m, mask, intrinsics, min_depth_m, max_depth_m):
    """Back-project all valid masked pixels and return points plus (u, v)."""
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if depth_m.shape != mask.shape or fx <= 0.0 or fy <= 0.0:
        raise ValueError("Compatible depth/mask and positive focal lengths required")
    valid = (
        (mask > 0) & np.isfinite(depth_m)
        & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    )
    v, u = np.nonzero(valid)
    if u.size == 0:
        return np.empty((0, 3)), np.empty((0, 2), dtype=np.int32)
    z = depth_m[v, u].astype(np.float64)
    points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
    return points, np.column_stack((u, v)).astype(np.int32)


def fit_plane(points):
    """Fit a plane by SVD and return its unit normal and offset."""
    if points.shape[0] < 3:
        raise ValueError("At least three plane points are required")
    center = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = vh[-1]
    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        raise ValueError("Degenerate plane points")
    normal /= norm
    return normal, -float(np.dot(normal, center))


def camera_up_vector(mount_pitch_deg):
    """Return gravity-up expressed in the camera optical frame."""
    pitch = math.radians(mount_pitch_deg)
    value = np.array([0.0, -math.cos(pitch), -math.sin(pitch)])
    return value / np.linalg.norm(value)


def detect_blue_wall(
    depth_m, blue_mask, intrinsics, min_depth_m, max_depth_m,
    distance_threshold_m, ransac_iterations, min_inliers,
    min_support_ratio, min_image_fraction, mount_pitch_deg,
    max_vertical_error_deg, max_ransac_points, rng,
):
    """Detect a dominant, broad, well-supported vertical blue plane."""
    empty = np.zeros_like(blue_mask)
    points, pixels = project_masked_depth(
        depth_m, blue_mask, intrinsics, min_depth_m, max_depth_m
    )
    if points.shape[0] < max(3, min_inliers):
        return WallResult(empty)
    fit_points = points
    if points.shape[0] > max_ransac_points:
        fit_points = points[rng.choice(
            points.shape[0], max_ransac_points, replace=False
        )]
    best = np.empty(0, dtype=np.int64)
    for _ in range(max(1, ransac_iterations)):
        ids = rng.choice(fit_points.shape[0], 3, replace=False)
        p0, p1, p2 = fit_points[ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        inliers = np.flatnonzero(
            np.abs((fit_points - p0) @ normal) <= distance_threshold_m
        )
        if inliers.size > best.size:
            best = inliers
    if best.size < min_inliers:
        return WallResult(empty)
    normal, offset = fit_plane(fit_points[best])
    best = np.flatnonzero(
        np.abs(points @ normal + offset) <= distance_threshold_m
    )
    normal, offset = fit_plane(points[best])
    best = np.flatnonzero(
        np.abs(points @ normal + offset) <= distance_threshold_m
    )
    support_ratio = best.size / float(points.shape[0])
    support_pixels = pixels[best]
    image_fraction = (
        (int(np.ptp(support_pixels[:, 0])) + 1)
        * (int(np.ptp(support_pixels[:, 1])) + 1)
        / float(blue_mask.size)
    )
    angle_from_up = math.degrees(math.acos(float(np.clip(
        abs(np.dot(normal, camera_up_vector(mount_pitch_deg))), 0.0, 1.0
    ))))
    vertical_error = abs(90.0 - angle_from_up)
    detected = (
        best.size >= min_inliers and support_ratio >= min_support_ratio
        and image_fraction >= min_image_fraction
        and vertical_error <= max_vertical_error_deg
    )
    wall_mask = empty.copy()
    if detected:
        wall_mask[support_pixels[:, 1], support_pixels[:, 0]] = 255
    return WallResult(
        wall_mask, detected, int(best.size), support_ratio,
        image_fraction, vertical_error
    )


def cluster_target_masks(
    depth_m, remaining_mask, min_depth_m, max_depth_m,
    min_component_area_px, min_valid_depth_points, depth_gap_m,
    min_depth_cluster_ratio,
):
    """Cluster by 2D connectivity, then split distinct 3D depth layers."""
    valid = (
        (remaining_mask > 0) & np.isfinite(depth_m)
        & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    ).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(valid, 8)
    clusters = []
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < min_component_area_px:
            continue
        component_pixels = np.argwhere(labels == label)
        values = depth_m[component_pixels[:, 0], component_pixels[:, 1]]
        order = np.argsort(values)
        splits = np.flatnonzero(np.diff(values[order]) > depth_gap_m) + 1
        for group in np.split(order, splits):
            minimum = max(
                min_component_area_px,
                min_valid_depth_points,
                math.ceil(component_pixels.shape[0] * min_depth_cluster_ratio),
            )
            if group.size < minimum:
                continue
            chosen = component_pixels[group]
            mask = np.zeros_like(remaining_mask)
            mask[chosen[:, 0], chosen[:, 1]] = 255
            clusters.append(mask)
    return clusters


def locate_targets(
    depth_m, remaining_mask, intrinsics, min_depth_m, max_depth_m,
    min_component_area_px, min_valid_depth_points, cluster_depth_gap_m,
    min_depth_cluster_ratio, erosion_kernel_px, erosion_iterations,
):
    """Return one coarse result for every valid remaining cluster."""
    results: List[TargetResult] = []
    masks = cluster_target_masks(
        depth_m, remaining_mask, min_depth_m, max_depth_m,
        min_component_area_px, min_valid_depth_points, cluster_depth_gap_m,
        min_depth_cluster_ratio,
    )
    for mask in masks:
        centroid = mask_centroid(mask)
        depth_mask = erode_target_mask(mask, erosion_kernel_px, erosion_iterations)
        median_depth, valid_count = median_mask_depth(
            depth_m, depth_mask, min_depth_m, max_depth_m
        )
        if valid_count < min_valid_depth_points:
            continue
        results.append(TargetResult(
            mask, centroid, median_depth, valid_count,
            back_project_pixel(*centroid, median_depth, intrinsics)
        ))
    return sorted(results, key=lambda result: result.centroid[0])


class CoarseTargetLocator(Node):
    """Locate multiple blue targets while suppressing a dominant blue wall."""

    def __init__(self):
        """Configure subscriptions, wall filtering and multi-target output."""
        super().__init__("coarse_target_locator")
        defaults = {
            "color_topic": "/camera/camera/color/image_raw",
            "aligned_depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "output_topic": "/science/target_coarse_point",
            "h_min": 90, "h_max": 135, "s_min": 80, "v_min": 50,
            "min_component_area_px": 100,
            "morphology_kernel_px": 5,
            "morphology_close_iterations": 1,
            "morphology_open_iterations": 1,
            "depth_erosion_kernel_px": 3,
            "depth_erosion_iterations": 1,
            "min_depth_m": 0.15, "max_depth_m": 5.0,
            "min_valid_depth_points": 20,
            "cluster_depth_gap_m": 0.08,
            "min_depth_cluster_ratio": 0.15,
            "wall_distance_threshold_m": 0.025,
            "wall_ransac_iterations": 100,
            "wall_max_ransac_points": 12000,
            "wall_min_inliers": 500,
            "wall_min_support_ratio": 0.55,
            "wall_min_image_fraction": 0.08,
            "wall_mount_pitch_deg": 30.0,
            "wall_max_vertical_error_deg": 20.0,
            "random_seed": 7,
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
        self.color_stamp = self.depth_stamp = 0.0
        self.last_processed_stamp = -1.0
        self.depth_header = None
        self.intrinsics = None
        self.camera_frame = ""
        self.camera_size = (0, 0)
        self.started_at = None
        self.processed_frames = self.frames_with_targets = 0
        self.published_targets = self.invalid_frames = 0
        self.last_warning_time = -math.inf
        self.rng = np.random.default_rng(int(self.get_parameter("random_seed").value))
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
            "Multi-target coarse locator started with blue-wall rejection."
        )

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _color_callback(self, msg):
        try:
            self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            self.color_stamp = self._stamp_seconds(msg.header.stamp)
        except CvBridgeError as exc:
            self._warn(f"RGB conversion failed: {exc}")

    def _depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32)
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

    def _camera_info_callback(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.camera_frame = msg.header.frame_id
        self.camera_size = (msg.width, msg.height)

    def _warn(self, reason):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_warning_time >= 2.0:
            self.get_logger().warning(f"INVALID: {reason}")
            self.last_warning_time = now

    def _record(self, target_count):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.started_at is None:
            self.started_at = now
        self.processed_frames += 1
        if target_count:
            self.frames_with_targets += 1
            self.published_targets += target_count
        else:
            self.invalid_frames += 1
        duration = float(self.get_parameter("validation_duration_sec").value)
        if duration > 0.0 and now - self.started_at >= duration:
            self._log_summary()
            rclpy.shutdown()

    def _log_summary(self):
        ratio = self.frames_with_targets / float(max(self.processed_frames, 1))
        self.get_logger().info(
            f"FINAL_COARSE frames={self.processed_frames} "
            f"frames_with_targets={self.frames_with_targets} "
            f"published_targets={self.published_targets} "
            f"invalid={self.invalid_frames} frame_publish_ratio={ratio:.3f}"
        )

    def _draw_debug(self, raw, wall, remaining, targets):
        if not bool(self.get_parameter("show_debug").value) or self.color is None:
            return
        annotated = self.color.copy()
        annotated[wall.mask > 0] = (0, 0, 255)
        target_panel = np.zeros_like(annotated)
        colors = [(0, 255, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0)]
        for index, target in enumerate(targets):
            color = colors[index % len(colors)]
            contours, _ = cv2.findContours(
                target.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(annotated, contours, -1, color, 2)
            target_panel[target.mask > 0] = color
            uv = tuple(np.rint(target.centroid).astype(int))
            cv2.drawMarker(annotated, uv, color, cv2.MARKER_CROSS, 18, 2)
        cv2.putText(
            annotated,
            f"wall={'YES' if wall.detected else 'NO'} "
            f"support={wall.support_ratio:.2f} targets={len(targets)}",
            (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (255, 255, 255), 2, cv2.LINE_AA
        )
        wall_panel = np.zeros_like(annotated)
        wall_panel[wall.mask > 0] = (0, 0, 255)
        debug = np.vstack((
            np.hstack((annotated, cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR))),
            np.hstack((wall_panel, target_panel + cv2.cvtColor(
                remaining // 4, cv2.COLOR_GRAY2BGR
            ))),
        ))
        cv2.imshow("Coarse Target Locator", debug)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            rclpy.shutdown()

    def _process(self):
        if self.color is None or self.depth_m is None or self.intrinsics is None:
            self._warn("waiting for RGB, aligned depth and CameraInfo")
            return
        if self.depth_stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = self.depth_stamp
        empty = np.zeros(self.color.shape[:2], dtype=np.uint8)
        if (
            self.color.shape[:2] != self.depth_m.shape
            or self.camera_size != (self.color.shape[1], self.color.shape[0])
            or abs(self.color_stamp - self.depth_stamp)
            > float(self.get_parameter("max_rgb_depth_delta_sec").value)
        ):
            self._warn("RGB, aligned depth and CameraInfo are not compatible")
            self._record(0)
            self._draw_debug(empty, WallResult(empty), empty, [])
            return
        raw, blue = blue_candidate_mask(
            self.color,
            (int(self.get_parameter("h_min").value),
             int(self.get_parameter("s_min").value),
             int(self.get_parameter("v_min").value)),
            (int(self.get_parameter("h_max").value), 255, 255),
            int(self.get_parameter("morphology_kernel_px").value),
            int(self.get_parameter("morphology_close_iterations").value),
            int(self.get_parameter("morphology_open_iterations").value),
        )
        if not np.any(blue):
            self._warn("no blue candidates")
            self._record(0)
            self._draw_debug(raw, WallResult(empty), empty, [])
            return
        get = self.get_parameter
        wall = detect_blue_wall(
            self.depth_m, blue, self.intrinsics,
            float(get("min_depth_m").value), float(get("max_depth_m").value),
            float(get("wall_distance_threshold_m").value),
            int(get("wall_ransac_iterations").value),
            int(get("wall_min_inliers").value),
            float(get("wall_min_support_ratio").value),
            float(get("wall_min_image_fraction").value),
            float(get("wall_mount_pitch_deg").value),
            float(get("wall_max_vertical_error_deg").value),
            int(get("wall_max_ransac_points").value),
            self.rng,
        )
        remaining = blue.copy()
        remaining[wall.mask > 0] = 0
        targets = locate_targets(
            self.depth_m, remaining, self.intrinsics,
            float(get("min_depth_m").value), float(get("max_depth_m").value),
            int(get("min_component_area_px").value),
            int(get("min_valid_depth_points").value),
            float(get("cluster_depth_gap_m").value),
            float(get("min_depth_cluster_ratio").value),
            int(get("depth_erosion_kernel_px").value),
            int(get("depth_erosion_iterations").value),
        )
        for index, target in enumerate(targets):
            point = PointStamped()
            point.header = self.depth_header
            point.header.frame_id = self.camera_frame
            point.point.x, point.point.y, point.point.z = map(float, target.xyz)
            self.publisher.publish(point)
            self.get_logger().info(
                f"COARSE_TARGET index={index} targets_in_frame={len(targets)} "
                f"mask_area={np.count_nonzero(target.mask)} "
                f"centroid_u={target.centroid[0]:.2f} "
                f"centroid_v={target.centroid[1]:.2f} "
                f"valid_depth_points={target.valid_depth_points} "
                f"median_depth_m={target.median_depth_m:.3f} "
                f"X={target.xyz[0]:.3f} Y={target.xyz[1]:.3f} "
                f"Z={target.xyz[2]:.3f} frame_id={self.camera_frame}"
            )
        self.get_logger().info(
            f"COARSE_FRAME blue_pixels={np.count_nonzero(blue)} "
            f"wall_detected={int(wall.detected)} "
            f"wall_points={wall.support_points} "
            f"wall_support={wall.support_ratio:.3f} "
            f"wall_image_fraction={wall.image_fraction:.3f} "
            f"wall_vertical_error_deg={wall.vertical_error_deg:.2f} "
            f"targets={len(targets)}"
        )
        if not targets:
            self._warn("no valid targets after wall rejection and clustering")
        self._record(len(targets))
        self._draw_debug(raw, wall, remaining, targets)

    def close_windows(self):
        """Log counters and close the debug window."""
        if self.processed_frames:
            self._log_summary()
        if bool(self.get_parameter("show_debug").value):
            cv2.destroyWindow("Coarse Target Locator")


def main(args=None):
    """Run multi-target coarse localization."""
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
