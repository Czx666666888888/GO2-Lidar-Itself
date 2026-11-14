"""Focused docking geometry, three-axis commands and independent guards; no hardware."""
import math

import pytest

from go2_science_perception.approach_control import ApproachControl, Settings
from go2_science_perception.grasp_docking import GraspDocking
from go2_science_perception.grasp_pose_geometry import (
    DockSettings, docking_goal, geometry_report, rotate_xy, target_in_s1,
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


def tick(core, t, position, yaw=0.0):
    x, y = position
    core.camera_pose(x+.36*math.cos(yaw), y+.36*math.sin(yaw), t)
    core.pose(x, y, yaw, t)
    core.feedback_time, core.feedback_speed, core.feedback_yaw_rate = t, 0., 0.


def dock_core(error=(.5, .1), yaw=0.0):
    core = ApproachControl(
        Settings(stop_reference='camera_origin', allow_odom=True,
                 max_run_time=25, odom_max_time=25, odom_max_distance=5), GraspDocking())
    core.target = core.anchor_initial = (1., 0.)
    core.lock_id = 1
    core.search_origin = (0., 0., 0.)
    core.state = 'LOCK_TARGET'
    core.started = 10.
    core.visual_time = 10.5
    goal = docking_goal(core.target, 0)
    for i in range(6):
        tick(core, 10+i*.1, (goal[0]-error[0], goal[1]-error[1]), yaw)
    return core


@pytest.mark.parametrize('error,yaw,sign', [
    ((0, .3), 0, 1), ((0, -.3), 0, -1),
    ((.3, 0), math.pi/2, -1), ((-.3, 0), math.pi/2, 1)])
def test_lateral_direction_and_rotated_map_error(error, yaw, sign):
    c = dock_core(error, yaw)
    out = c.step(10.5)
    assert out['vy']*sign > 0
    assert out['docking_body_error_xy'] == pytest.approx(rotate_xy(-yaw, error))


@pytest.mark.parametrize('error,yaw,sign', [
    ((.5, .3), .25, 1), ((-.5, .3), -.25, -1)])
def test_forward_backward_lateral_and_yaw_commands_can_be_simultaneous(error, yaw, sign):
    c = dock_core(error, yaw=yaw)
    out = c.step(10.5)
    assert out['vx']*sign > 0 and out['vy'] > 0 and out['wz']*sign < 0
    assert out['docking_phase'] == 'CONTROL_POSE'


def test_planar_acceleration_speed_cap_and_braking_include_lateral_axis():
    c = dock_core((1., 1.))
    previous = (0., 0.)
    for i in range(26):
        t = 10.5+i*.1
        c.visual_time = t
        tick(c, t+.001, c.robot[:2])  # Strictly increasing input timestamps.
        out = c.step(t+.001)
        current = (out['vx'], out['vy'])
        assert math.hypot(*current) <= c.p.max_linear_speed+1e-12
        assert math.dist(current, previous) <= c.p.max_acceleration*.101+1e-9
        previous = current
    assert math.hypot(*previous) == pytest.approx(.3)
    # Exercise vector braking without simulating robot physics.
    c.target = (c.robot[0]-.06, c.robot[1]-.31)
    c.visual_time = 13.2
    tick(c, 13.2, c.robot[:2])
    out = c.step(13.2)
    assert out['vy'] > 0  # Slew-limited braking, not instantaneous lateral reversal.
    assert math.dist((out['vx'], out['vy']), previous) <= c.p.max_deceleration*.2+1e-9
    assert math.hypot(out['vx'], out['vy']) < math.hypot(*previous)


@pytest.mark.parametrize('error,yaw', [
    ((.04, 0), math.radians(10)), ((.039, 0), math.radians(9.9)), ((0, -.039), 0)])
def test_four_cm_ten_degrees_and_half_second_settle_without_hard_s1_rectangle(error, yaw):
    c = dock_core(error, yaw)
    out = c.step(10.5)
    assert out['state'] != 'STOP'
    for i in range(1, 8):
        t = 10.5+i*.1
        c.visual_time = t
        tick(c, t, c.robot[:2], yaw)
        out = c.step(t)
        if i <= 4:
            assert out['state'] != 'STOP'
    assert out['state'] == 'STOP' and out['grasp_docking_complete']
    assert out['vx'] == out['vy'] == out['wz'] == 0
    if error[1] < 0:
        assert not out['grasp_docking']['predicted_in_grasp_region']
    assert c.step(t+.01)['grasp_docking_complete']


@pytest.mark.parametrize('error,yaw', [((.041, 0), 0), ((0, 0), math.radians(10.1))])
def test_outside_position_or_heading_tolerance_cannot_complete(error, yaw):
    c = dock_core(error, yaw)
    for i in range(8):
        t = 10.5+i*.1
        c.visual_time = t
        tick(c, t+.001, c.robot[:2], yaw)
        out = c.step(t+.001)
    assert out['state'] != 'STOP' and not out['grasp_docking_complete']


@pytest.mark.parametrize('terminal', ['STOP', 'FAULT'])
def test_terminal_states_zero_all_three_commands(terminal):
    c = dock_core()
    c.step(10.5)
    if terminal == 'FAULT':
        c.fault('manual emergency stop')
    else:
        c.state = 'STOP'
    out = c.step(10.6)
    assert out['vx'] == out['vy'] == out['wz'] == 0


def test_progressing_docking_can_exceed_fixed_25_seconds():
    c = dock_core((2., 0))
    c.step(10.5)
    x, y = c.robot[:2]
    for i in range(1, 301):
        t = 10.5+i*.1
        c.visual_time = t
        tick(c, t, (x+i*.002, y))
        out = c.step(t)
    assert out['elapsed'] > 25 and out['state'] == 'APPROACH_TO_GRASP_POSE'


@pytest.mark.parametrize('step_m', [0., .0001])
def test_no_progress_or_small_corrections_keep_docking_with_fresh_inputs(step_m):
    c = dock_core()
    x, y = c.robot[:2]
    for i in range(310):
        t = 10.5+i*.1
        c.visual_time = t
        tick(c, t+.001, (x+i*step_m, y))
        out = c.step(t+.001)
        assert out['state'] == 'APPROACH_TO_GRASP_POSE'
    assert out['elapsed'] > 25
    assert out['docking_progress_timeout_enabled'] is False
    assert math.hypot(out['vx'], out['vy']) > 0


@pytest.mark.parametrize('budget', ['time', 'distance'])
def test_docking_removes_odom_time_limit_but_keeps_distance(budget):
    c = dock_core()
    c.step(10.5)
    c.lost_at = 10.
    c.visual_time = 9.9
    now = 35.1 if budget == 'time' else 10.6
    c.odom_travel = 5. if budget == 'distance' else 0.
    c.last_tick = now-.1
    tick(c, now, c.robot[:2])
    out = c.step(now)
    if budget == 'time':
        assert out['state'] == 'APPROACH_TO_GRASP_POSE'
        assert out['docking_odom_time_limit_enabled'] is False
        assert out['odom_remaining_time_s'] is None
    else:
        assert out['state'] == 'FAULT' and out['reason'] == 'odometry continuation limit'
        assert out['vx'] == out['vy'] == out['wz'] == 0


def test_localization_expiry_still_faults():
    c = dock_core()
    c.last_tick = 11.
    out = c.step(11.1)
    assert out['state'] == 'FAULT' and out['reason'] == 'localization timeout'
    assert out['vx'] == out['vy'] == out['wz'] == 0


def test_plain_map_keeps_zero_lateral_speed_and_original_runtime_limit():
    c = dock_core()
    c.docking = None
    out = c.step(10.5)
    assert out['vy'] == 0 and 'grasp_docking' not in out
    c.last_tick = 35.
    c.visual_time = 35.1
    tick(c, 35.1, c.robot[:2])
    out = c.step(35.1)
    assert out['reason'] == 'run timeout' and out['vy'] == 0


def test_new_tolerance_does_not_change_plain_map_stopping():
    c = dock_core()
    c.docking = None
    c.camera = (.91, 0)
    out = c.step(10.5)
    assert out['state'] == 'STOP' and out['reason'] == 'camera XY aligned'
    assert out['vy'] == 0 and DockSettings().position_tolerance == .04


def test_additional_right_clearance_preserves_grasp_reference():
    old = DockSettings()
    new = DockSettings(target_s1_y=0.41)
    before = docking_goal((1.0, -0.5), 0.0, old)
    after = docking_goal((1.0, -0.5), 0.0, new)
    assert after[0] == pytest.approx(before[0])
    assert after[1] - before[1] == pytest.approx(0.10)
    report = geometry_report((1.0, -0.5), after, 0.0, new)
    assert report['position_error_m'] == pytest.approx(0.0)
    assert not report['predicted_in_grasp_region']


@pytest.mark.parametrize('minimum,error', [
    (.10, (.08, 0)), (.10, (0, -.08)), (.10, (.048, .064)),
    (.20, (.15, 0)), (.20, (0, -.15)), (.20, (.09, .12)),
])
def test_minimum_is_planar_and_preserves_braking(minimum, error):
    c = dock_core(error)
    c.docking.p = DockSettings(min_linear_speed=minimum)
    c.p.braking_latency = .30  # Actual deployed profile.
    for i in range(20):
        t = 10.6+i*.1
        c.visual_time = t
        tick(c, t, c.robot[:2])
        out = c.step(t)
    assert math.hypot(out['vx'], out['vy']) == pytest.approx(minimum)
    # Near the tolerance boundary the braking cap can go below the minimum.
    t += .1
    c.visual_time = t
    goal = docking_goal(c.target, 0)
    tick(c, t, (goal[0]-.041, goal[1]))
    c.actual_speed = 0.0
    for _ in range(10):
        c.visual_time = t
        tick(c, t+.001, c.robot[:2])
        out = c.step(t+.001)
        t += .1
    assert math.hypot(out['vx'], out['vy']) < minimum
