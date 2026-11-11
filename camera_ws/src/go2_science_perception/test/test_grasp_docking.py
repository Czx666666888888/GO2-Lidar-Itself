"""Three mounting cases and focused optional docking guard checks; no hardware."""
import math

import pytest

from go2_science_perception.approach_control import ApproachControl, Settings
from go2_science_perception.grasp_docking import GraspDocking
from go2_science_perception.grasp_pose_geometry import (
    DockSettings, docking_goal, geometry_report, target_in_s1,
)


@pytest.mark.parametrize('theta,goal', [
    (0, (1.06, 2.31, 0)), (math.pi/2, (.69, 2.06, math.pi/2))])
def test_goal_and_heading_rotate_the_actual_offset(theta, goal):
    assert docking_goal((1, 2), theta) == pytest.approx(goal)
    assert target_in_s1((1, 2), goal) == pytest.approx((0, .31))


def test_s1_positive_y_is_vehicle_right_and_unknown_height_is_retained():
    assert target_in_s1((-.06, -.31), (0, 0, 0)) == pytest.approx((0, .31))
    report = geometry_report((-.06, -.31), (0, 0, 0), 0)
    assert report['s1_base_z'] is None
    assert report['predicted_in_grasp_region'] and not report['physical_placement_verified']


def dock_core(position=None, yaw=0.0):
    core = ApproachControl(
        Settings(stop_reference='camera_origin', allow_odom=True,
                 odom_max_time=25, odom_max_distance=5), GraspDocking())
    core.target = core.anchor_initial = (1., 0.)
    core.lock_id = 1
    core.search_origin = (0., 0., 0.)
    core.state = 'LOCK_TARGET'
    core.started = 10.
    core.visual_time = 10.
    for i in range(6):
        tick(core, 10+i*.1, position or (0., 0.), yaw)
    return core


def tick(core, t, position, yaw=0.0):
    x, y = position
    core.camera_pose(x+.36*math.cos(yaw), y+.36*math.sin(yaw), t)
    core.pose(x, y, yaw, t)
    core.feedback_time, core.feedback_speed, core.feedback_yaw_rate = t, 0., 0.


def test_position_alone_cannot_stop_and_settled_correct_pose_can():
    goal = docking_goal((1, 0), 0)
    core = dock_core(goal[:2], yaw=.1)
    out = core.step(10.5)
    assert out['docking_phase'] == 'FINAL_YAW' and out['state'] != 'STOP'
    for i in range(1, 17):
        t = 10.5+i*.1
        core.visual_time = t
        tick(core, t, goal[:2])
        out = core.step(t)
    assert out['state'] == 'STOP' and out['grasp_docking_complete']
    assert out['vx'] == out['wz'] == 0
    assert out['grasp_docking']['predicted_target_s1_xy'] == pytest.approx([0, .31])
    # The terminal marker must survive subsequent STOP reports used by the coordinator.
    assert core.step(t+.01)['grasp_docking_complete']


def test_docking_uses_existing_odom_budget_and_no_lateral_command():
    core = dock_core()
    out = core.step(10.5)
    assert out['state'] == 'APPROACH_TO_GRASP_POSE'
    assert out['control_reference'] == 'MAP_GRASP_DOCK'
    assert abs(out['wz']) <= .3 and 0 <= out['vx'] <= .3
    core.lost_at, core.odom_travel = 10., 5.
    out = core.step(10.6)
    assert out['state'] == 'FAULT' and out['reason'] == 'odometry continuation limit'
    assert out['vx'] == out['wz'] == 0


def test_final_heading_survives_position_jitter_without_claiming_completion():
    goal = docking_goal((1, 0), 0)
    core = dock_core(goal[:2], yaw=.2)
    assert core.step(10.5)['docking_phase'] == 'FINAL_YAW'
    # Crossing the 2cm boundary previously switched back to bearing-to-point,
    # abandoning final yaw repeatedly in the first physical trial.
    tick(core, 10.6, (goal[0]-.025, goal[1]), yaw=.2)
    out = core.step(10.6)
    assert out['docking_phase'] == 'FINAL_YAW'
    assert out['vx'] == 0 and out['wz'] < 0
    assert not out['grasp_docking_complete']
    # Completing yaw does not waive the original position requirement.
    tick(core, 10.7, (goal[0]-.025, goal[1]), yaw=0)
    out = core.step(10.7)
    assert out['state'] != 'STOP' and not out['grasp_docking_complete']


def test_new_tolerance_does_not_change_plain_map_stopping():
    core = dock_core((.9, 0))
    core.docking = None
    core.camera = (.91, 0)
    core.visual_time = 10.5
    out = core.step(10.5)
    assert out['state'] == 'STOP' and out['reason'] == 'camera XY aligned'
    assert 'grasp_docking' not in out
    assert DockSettings().position_tolerance == .02
