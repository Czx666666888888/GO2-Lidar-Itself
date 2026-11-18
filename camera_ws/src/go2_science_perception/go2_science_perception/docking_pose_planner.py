"""Select geometrically valid staging poses for the existing FAR planner."""
from dataclasses import dataclass
import math

from .grasp_pose_geometry import docking_goal, rotate_xy, target_in_s1, wrap


@dataclass(frozen=True)
class StagingPose:
    pose: tuple
    dock: tuple
    side: str


class TerrainFootprint:
    """Known-free 2-D terrain with the standing rectangle and directional sweep."""
    def __init__(self, footprint, resolution=.1):
        self.half_length = footprint['length'] / 2
        self.half_width = footprint['width'] / 2
        self.hard_clearance = footprint['hard_clearance']
        self.parking_clearance = footprint['parking_clearance']
        self.resolution = resolution
        self.cells = {}

    def cell(self, x, y):
        return round(x / self.resolution), round(y / self.resolution)

    def update(self, xs, ys, heights):
        cells = {}
        for x, y, height in zip(xs, ys, heights):
            if not all(math.isfinite(v) for v in (x, y, height)):
                continue
            index = self.cell(x, y)
            state = 0 if height >= 1.45 else (2 if height >= .20 else 1)
            cells[index] = max(state, cells.get(index, 0))
        self.cells = cells

    def overlaps(self, pose, x, y, clearance):
        body_x, body_y = rotate_xy(-pose[2], (x - pose[0], y - pose[1]))
        return math.hypot(max(abs(body_x) - self.half_length, 0.),
                          max(abs(body_y) - self.half_width, 0.)) <= clearance

    def clear(self, pose, margin, *, starting_pose=None):
        res = self.resolution
        margin += res / math.sqrt(2)
        radius = math.ceil((math.hypot(self.half_length, self.half_width) + margin) / res)
        cx, cy = self.cell(*pose[:2])
        for ix in range(cx - radius, cx + radius + 1):
            for iy in range(cy - radius, cy + radius + 1):
                x, y = ix * res, iy * res
                state = self.cells.get((ix, iy), 0)
                if state == 1 or not self.overlaps(pose, x, y, margin):
                    continue
                # The ground under the current body may be occluded. This
                # exception never admits an occupied cell.
                if state == 0 and starting_pose and self.overlaps(starting_pose, x, y, 0.):
                    continue
                return False
        return True

    def sweep_clear(self, start, end):
        distance = math.dist(start[:2], end[:2])
        turn = wrap(end[2] - start[2])
        steps = max(1, math.ceil(distance / (.5 * self.resolution)), math.ceil(abs(turn) / .1))
        for i in range(steps + 1):
            t = i / steps
            pose = (start[0] + t * (end[0] - start[0]),
                    start[1] + t * (end[1] - start[1]), start[2] + t * turn)
            if not self.clear(pose, self.hard_clearance, starting_pose=start):
                return False
        return True


def staging_candidates(target, robot, sides, terrain):
    """Check the final S1 geometry and the full near-approach sweep before FAR trials."""
    bearing = math.atan2(target[1] - robot[1], target[0] - robot[0])
    options = []
    for delta in (0, -20, 20, -40, 40):
        yaw = wrap(bearing + math.radians(delta))
        offset = rotate_xy(yaw, (.70, 0.))
        for side, settings in sides.items():
            dock = docking_goal(target, yaw, settings)
            x, y = target_in_s1(target, dock, settings)
            if not (settings.region_x_min - 1e-6 <= x <= settings.region_x_max + 1e-6
                    and settings.region_y_min - 1e-6 <= y <= settings.region_y_max + 1e-6):
                continue
            stage = (dock[0] - offset[0], dock[1] - offset[1], yaw)
            if (terrain.clear(stage, terrain.parking_clearance)
                    and terrain.sweep_clear(stage, dock)):
                options.append(StagingPose(stage, dock, side))
    return sorted(options, key=lambda candidate: math.dist(robot[:2], candidate.pose[:2]))
