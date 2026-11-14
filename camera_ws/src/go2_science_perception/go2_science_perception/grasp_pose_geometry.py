"""Planar S1 docking geometry, metres/radians; never invent a base height."""
from dataclasses import dataclass
import math


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def rotate_xy(theta, point):
    c, s = math.cos(theta), math.sin(theta)
    x, y = point
    return c*x-s*y, s*x+c*y


@dataclass(frozen=True)
class DockSettings:
    """New-mode-only tolerances and user-measured planar mounting geometry."""

    s1_base_x: float = -0.06
    s1_base_y: float = 0.0
    target_s1_x: float = 0.0
    target_s1_y: float = 0.31
    position_tolerance: float = 0.04
    yaw_tolerance: float = math.radians(10)
    settled_sec: float = 0.5
    stationary_reset_sec: float = 0.0  # Zero preserves immediate-reset compatibility.
    min_linear_speed: float = 0.0  # Opt-in through the docking profile.
    fine_distance: float = 0.15
    fine_forward_correction_enabled: bool = True
    fine_min_linear_speed: float = 0.0
    fine_max_linear_speed: float = 0.08
    settle_exit_tolerance: float = 0.06
    settle_wait_sec: float = 2.0
    motion_window_sec: float = 0.0  # Dock overlay only; ordinary MAP/search unchanged.
    motion_noise_m: float = 0.02
    stable_window_sec: float = 0.0
    settle_exit_confirm_sec: float = 0.0
    visual_loss_stop_sec: float = 0.0
    allow_estimated_forward_profile: bool = False
    forward_deadzone_guard: bool = False
    forward_effective_speed: float = 0.20  # Positive vx only, never a global floor.
    forward_step_enabled: bool = False  # Needs a reviewed response measurement first.
    forward_response_max_distance: float = 0.20  # Diagnostic only; explicit override required.
    forward_response_require_speed: bool = True
    forward_response_check: bool = False  # Explicit diagnostic, never a grasp goal.
    progress_timeout: float = 10.0  # Accepted for old configs; no longer stops docking.
    region_x_min: float = -0.05
    region_x_max: float = 0.05
    region_y_min: float = 0.29
    region_y_max: float = 0.33

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Dock geometry must be finite')
        if min(self.position_tolerance, self.yaw_tolerance,
               self.settled_sec, self.progress_timeout) <= 0:
            raise ValueError('Dock tolerances and settling time must be positive')
        if self.min_linear_speed < 0:
            raise ValueError('Dock minimum speed must be nonnegative')
        if not (self.position_tolerance < self.settle_exit_tolerance < self.fine_distance
                and 0 <= self.fine_min_linear_speed <= self.fine_max_linear_speed
                and self.fine_max_linear_speed > 0 and self.settle_wait_sec > 0):
            raise ValueError('Invalid fine-control or stop-wait settings')
        if (min(self.motion_window_sec, self.stable_window_sec,
                self.settle_exit_confirm_sec, self.visual_loss_stop_sec,
                self.stationary_reset_sec) < 0 or
                self.motion_noise_m <= 0 or not .14 <= self.forward_effective_speed <= .30):
            raise ValueError('Invalid dock motion evidence settings')
        if not .10 <= self.forward_response_max_distance <= .50:
            raise ValueError('Response-check distance limit must be 0.10 to 0.50m')
        # Dock clearance can exceed the original grasp reference region.
        # Keep that region unchanged so reachability diagnostics remain honest.
        if not (self.region_x_min <= self.region_x_max and
                self.region_y_min < self.region_y_max and self.target_s1_y != 0 and
                self.region_y_min * self.target_s1_y > 0 and
                self.region_y_max * self.target_s1_y > 0):
            raise ValueError('Dock target and S1 grasp region must use the same side')


def target_in_s1(target_map, vehicle_pose, settings=None):
    """vehicle +X forward/+Y left; S1 +X rear/+Y right (planar yaw pi)."""
    p = settings or DockSettings()
    x, y, theta = vehicle_pose
    local = rotate_xy(-theta, (target_map[0]-x, target_map[1]-y))
    return p.s1_base_x-local[0], p.s1_base_y-local[1]


def docking_goal(target_map, theta, settings=None):
    """vehicle_goal = target_map - R(theta) @ desired_target_vehicle."""
    p = settings or DockSettings()
    if not all(math.isfinite(v) for v in (*target_map, theta)):
        raise ValueError('Nonfinite target or heading')
    target_vehicle = (p.s1_base_x-p.target_s1_x, p.s1_base_y-p.target_s1_y)
    offset = rotate_xy(theta, target_vehicle)
    return target_map[0]-offset[0], target_map[1]-offset[1], wrap(theta)


def geometry_report(target_map, vehicle_pose, theta, settings=None):
    p = settings or DockSettings()
    goal = docking_goal(target_map, theta, p)
    predicted = target_in_s1(target_map, vehicle_pose, p)
    return dict(target_map_xy=list(target_map), vehicle_map_pose=list(vehicle_pose),
                vehicle_goal_map_pose=list(goal), desired_yaw=goal[2],
                predicted_target_s1_xy=list(predicted),
                position_error_m=math.dist(vehicle_pose[:2], goal[:2]),
                yaw_error_rad=wrap(goal[2]-vehicle_pose[2]),
                predicted_in_grasp_region=(p.region_x_min <= predicted[0] <= p.region_x_max
                                           and p.region_y_min <= predicted[1] <= p.region_y_max),
                physical_placement_verified=False,
                mounting_status='USER_MEASURED_XY_PROVISIONAL_SENSOR_ORIGIN',
                s1_base_z=None)
