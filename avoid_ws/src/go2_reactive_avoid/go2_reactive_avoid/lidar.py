"""Body-relative distances from a single native UTLiDAR cloud on level ground."""
import math

import numpy as np

from .controller import Direction, FRONT_SLOW_GAP, FRONT_STOP_GAP, Scan

# Same standing envelope as navigation_test/body_footprint.yaml.
HALF_LENGTH = 0.35
HALF_WIDTH = 0.20
HARD_GAP = 0.08
SWEEP_RADIUS = math.hypot(HALF_LENGTH, HALF_WIDTH) + HARD_GAP
# Native mount pitch from transform_sensors; x offset from base_odom_node.
MOUNT_PITCH = 2.8782025850555556
LIDAR_X = 0.30
FRONT_WIDTH = HALF_WIDTH + HARD_GAP + 0.05
SIDE_NEAR_GAP = 0.42
MIN_OBSTACLE_HEIGHT = 0.10
DEFAULT_LIDAR_HEIGHT = 0.30
FRAMES = ("utlidar_lidar", "utlidar_lidar_1")


def body_points(raw, roll, pitch, lidar_height):
    x, y, z = raw.T
    c, s = math.cos(MOUNT_PITCH), math.sin(MOUNT_PITCH)
    forward, up = c*x + s*z + LIDAR_X, -s*x + c*z
    cr, sr = math.cos(roll), math.sin(roll)
    left, up = cr*y - sr*up, sr*y + cr*up
    cp, sp = math.cos(pitch), math.sin(pitch)
    return np.column_stack((cp*forward + sp*up, left, -sp*forward + cp*up + lidar_height))


def measure(points):
    x, y, height = points[np.all(np.isfinite(points), axis=1)].T
    radius = np.hypot(x, y)
    outside = (np.abs(x) > HALF_LENGTH) | (np.abs(y) > HALF_WIDTH)
    floor = (np.abs(height) <= 0.06) & outside
    if np.count_nonzero(floor & (radius < 2.0)) < 12:
        return None
    obstacles = (height >= MIN_OBSTACLE_HEIGHT) & (height <= 1.2) & outside
    coverage = (floor | obstacles) & (radius >= SWEEP_RADIUS) & (radius <= 2.0)
    # Three 30-degree bins per quadrant. Missing returns remain unknown.
    angle = (np.arctan2(y, x) + math.pi / 4) % (2 * math.pi)
    bins = np.floor(angle / (math.pi / 6)).astype(int) % 12
    seen = (np.bincount(bins[coverage], minlength=12) >= 2).reshape((4, 3)).all(axis=1)
    quadrant = bins // 3

    corridor = obstacles & (np.abs(y) <= FRONT_WIDTH)
    front_points = corridor & (x > 0) & (x <= HALF_LENGTH + FRONT_SLOW_GAP)
    front_gap = max(0.0, float(np.min(x[front_points], initial=np.inf)) - HALF_LENGTH)
    sector_blocks = obstacles & (radius <= 0.78) & ((quadrant == 1) | (quadrant == 3))
    left = np.any(sector_blocks & (quadrant == 1))
    right = np.any(sector_blocks & (quadrant == 3))
    front = front_gap <= FRONT_STOP_GAP
    rear = np.any(corridor & (x < 0) & (-x <= HALF_LENGTH + FRONT_STOP_GAP))
    edge = np.hypot(np.maximum(np.abs(x)-HALF_LENGTH, 0), np.maximum(np.abs(y)-HALF_WIDTH, 0))
    body_clear = not np.any(obstacles & (edge <= HARD_GAP))
    can_rotate = bool(seen.all()) and not np.any(obstacles & (radius <= SWEEP_RADIUS))
    near = obstacles & (np.abs(x) <= HALF_LENGTH + 0.20) & (np.abs(y) <= HALF_WIDTH + SIDE_NEAR_GAP)
    states = [
        Direction.BLOCKED if blocked else Direction.CLEAR if observed else Direction.UNKNOWN
        for blocked, observed in zip((front, left, rear, right), seen)
    ]
    return Scan(states[0], states[3], states[1], states[2], can_rotate, body_clear,
                bool(np.any(near & (y < -FRONT_WIDTH))), bool(np.any(near & (y > FRONT_WIDTH))),
                front_gap)
