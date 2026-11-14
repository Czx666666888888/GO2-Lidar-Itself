"""Small targeted regressions for temporal evidence and fine-control guards."""

import pytest

from go2_science_perception.docking_motion import WindowedTravel, window_stats
from go2_science_perception.grasp_docking import GraspDocking
from go2_science_perception.grasp_pose_geometry import DockSettings
from test_grasp_docking import dock_core, tick


def test_stationary_jitter_is_not_a_many_metre_path():
    m = WindowedTravel(.5, .02)
    for i in range(601):
        m.add(i*.1, .005*(-1)**i, .003*(-1)**(i//2), 0.)
    assert m.raw > 5
    assert m.distance < .03
    assert not m.conflict


def test_slow_motion_and_round_trip_remain_counted():
    m = WindowedTravel(.5, .02)
    for i in range(401):
        t = i*.1
        x = min(t*.05, 1.) if t <= 20 else max(0., 1-(t-20)*.05)
        m.add(t, x, 0., .05)
    for i in range(10):
        m.add(40.1+i*.1, 0., 0., 0.)
    assert m.distance == pytest.approx(2., abs=.06)
    assert not m.conflict


def test_pose_motion_is_not_discarded_when_sport_says_zero():
    m = WindowedTravel(.5, .02)
    for i in range(50):
        m.add(i*.1, i*.02, 0., 0.)
    assert m.distance > .8
    assert m.conflict


def test_motion_window_distinguishes_trend_from_one_endpoint_jitter():
    stopped = [(i*.1, .005*(-1)**i, 0.) for i in range(6)]
    moving = [(i*.1, i*.01, 0.) for i in range(6)]
    assert window_stats(stopped, .5, .02)['speed_mps'] < .05
    assert window_stats(moving, .5, .02)['speed_mps'] > .05


def test_enabled_but_uncalibrated_step_stops_instead_of_repeating_deadzone_vx():
    c = dock_core((.10, 0.))
    c.docking = GraspDocking(DockSettings(forward_step_enabled=True,
        forward_deadzone_guard=True, forward_effective_speed=.20))
    out = c.step(10.5)
    assert out['state'] == 'FAULT'
    assert 'calibration required' in out['reason']
    assert out['vx'] == out['vy'] == out['wz'] == 0.


def test_visual_loss_is_bounded_even_with_large_odometry_budget():
    c = dock_core((.1, 0.))
    c.docking = GraspDocking(DockSettings(visual_loss_stop_sec=3.))
    c.visual_time = 7.
    out = c.step(10.5)
    assert out['state'] == 'FAULT'
    assert 'reconfirm target' in out['reason']
    assert c.odom_travel < c.p.odom_max_distance


def test_user_approved_six_cm_still_requires_half_second_quiet_and_ten_degrees():
    c = dock_core((.05, 0.))
    c.docking = GraspDocking(DockSettings(position_tolerance=.06,
        settle_exit_tolerance=.08, settled_sec=.5, stable_window_sec=.5))
    for i in range(13):
        t = 10.5+i*.1
        c.visual_time = t
        tick(c, t+.001, c.robot[:2])
        out = c.step(t+.001)
    assert out['state'] == 'STOP'
    assert out['grasp_docking_complete']


def test_isolated_pose_outlier_does_not_restart_stop_wait_or_claim_success():
    c = dock_core((.05, 0.))
    c.docking = GraspDocking(DockSettings(position_tolerance=.06,
        settle_exit_tolerance=.08, stable_window_sec=.5, settle_exit_confirm_sec=.3))
    c.step(10.5)
    start = c.docking.stop_wait_started
    x, y, yaw = c.robot
    tick(c, 10.6, (x-.05, y))
    out = c.step(10.6)
    assert out['docking_phase'] == 'STOP_WAIT'
    assert c.docking.stop_wait_started == start
    assert out['state'] != 'STOP'
