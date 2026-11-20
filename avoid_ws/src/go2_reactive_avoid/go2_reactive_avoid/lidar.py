"""Body-relative distances from a single native UTLiDAR cloud on level ground."""
import math

import numpy as np

from .controller import Direction, FRONT_STOP_GAP, Scan

# Same standing envelope as navigation_test/body_footprint.yaml.
HALF_LENGTH = 0.35
HALF_WIDTH = 0.20
# Native mount pitch from transform_sensors; x offset from base_odom_node.
MOUNT_PITCH = 2.8782025850555556
LIDAR_X = 0.30
FRONT_WIDTH = 0.33
TIMED_FRONT_WIDTH = 0.50
TIMED_RANGE_EDGES = (0.20, 0.60, 1.00, 1.40, 1.80, 2.20, 2.60)
REAR_BLOCK_GAP = 0.55
SIDE_NEAR_GAP = 0.15
MIN_OBSTACLE_HEIGHT = 0.10
DEFAULT_LIDAR_HEIGHT = 0.30
FRAMES = ("utlidar_lidar", "utlidar_lidar_1")


def body_points(raw, roll, pitch, lidar_height, *, filter_self_returns=False):
    x, y, z = raw.T
    c, s = math.cos(MOUNT_PITCH), math.sin(MOUNT_PITCH)
    forward, up = c*x + s*z + LIDAR_X, -s*x + c*z
    if filter_self_returns:
        # Stationary nose returns confirmed on the test GO2. Bounds are fixed
        # to the lidar mount and stay independent of pitch, roll and floor height.
        nose = ((forward >= 0.34) & (forward <= 0.43)
                & (y >= -0.19) & (y <= 0.13)
                & (up >= -0.225) & (up <= -0.12)
                & (x*x + y*y + z*z <= 0.32**2))
        forward, y, up = forward[~nose], y[~nose], up[~nose]
    cr, sr = math.cos(roll), math.sin(roll)
    left, up = cr*y - sr*up, sr*y + cr*up
    cp, sp = math.cos(pitch), math.sin(pitch)
    return np.column_stack((cp*forward + sp*up, left, -sp*forward + cp*up + lidar_height))


def timed_clearance(x, y, floor, obstacles):
    """Limit clearance to contiguous observed ground across the timed corridor."""
    front_gap = x - HALF_LENGTH
    corridor = (front_gap >= 0) & (np.abs(y) <= TIMED_FRONT_WIDTH)
    closest = float(np.min(front_gap[corridor & obstacles], initial=np.inf))
    lane = np.minimum(2, ((y[corridor & floor] + TIMED_FRONT_WIDTH)
                          / (2 * TIMED_FRONT_WIDTH / 3)).astype(int))
    ground_gap = front_gap[corridor & floor]
    covered = 0.0
    for start, end in zip(TIMED_RANGE_EDGES, TIMED_RANGE_EDGES[1:]):
        in_bin = (ground_gap >= start) & (ground_gap < end)
        if (np.bincount(lane[in_bin], minlength=3) < 2).any():
            break
        covered = min(float(np.max(ground_gap[in_bin & (lane == index)]))
                      for index in range(3))
    return closest, covered


def measure(points, *, timed=False):
    """A return inside a fixed box sets a bit; no return sets it to zero.

    A nonempty finite cloud is required. Absence of returns is not proof of
    free space; this reactive test deliberately makes no coverage estimate.
    """
    points = points[np.all(np.isfinite(points), axis=1)]
    if not len(points):
        return None
    x, y, height = points.T
    outside = (np.abs(x) > HALF_LENGTH) | (np.abs(y) > HALF_WIDTH)
    obstacles = (height >= MIN_OBSTACLE_HEIGHT) & (height <= 1.2) & outside
    corridor = obstacles & (np.abs(y) <= FRONT_WIDTH)
    front_gap = float(np.min(x[corridor & (x >= HALF_LENGTH)], initial=np.inf)) - HALF_LENGTH
    front = front_gap <= FRONT_STOP_GAP + 1e-6
    side = obstacles & (np.abs(x) <= HALF_LENGTH + 0.20)
    left = bool(np.any(side & (y > HALF_WIDTH) & (y <= HALF_WIDTH + SIDE_NEAR_GAP + 1e-6)))
    right = bool(np.any(side & (y < -HALF_WIDTH) & (-y <= HALF_WIDTH + SIDE_NEAR_GAP + 1e-6)))
    rear = bool(np.any(corridor & (x <= -HALF_LENGTH)
                       & (-x <= HALF_LENGTH + REAR_BLOCK_GAP)))
    states = [Direction.BLOCKED if bit else Direction.CLEAR for bit in (front, right, left, rear)]
    floor = (np.abs(height) <= 0.06) & outside
    timed_ranges = timed_clearance(x, y, floor, obstacles) if timed else (math.inf, 0.0)
    # Keep legacy Scan fields for entrance/timed callers; no sweep/body gate.
    return Scan(*states, True, True, right, left, front_gap, *timed_ranges)
