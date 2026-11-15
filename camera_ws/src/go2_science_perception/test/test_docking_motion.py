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


@pytest.mark.parametrize('enabled', [True, False])
def test_disagreement_stop_can_be_disabled_without_hiding_evidence(enabled):
    c = dock_core((1., 0.))
    c.docking = GraspDocking(DockSettings(motion_window_sec=.5,
        motion_disagreement_stop_enabled=enabled))
    c.loss_motion = WindowedTravel(.5, .02)
    c.lost_at = 10.5
    x, y = c.robot[:2]
    for i in range(1, 51):
        tick(c, 10.5+i*.1, (x+i*.02, y))
    assert c.loss_motion.conflict
    out = c.step(15.5)
    evidence = out['odometry_evidence']
    assert evidence['localization_untrusted'] is True
    assert evidence['disagreement_stop_enabled'] is enabled
    assert evidence['disagreement_distance_m'] > .08
    assert (out['state'] == 'FAULT') is enabled
    if enabled:
        assert out['reason'] == 'localization untrusted: sustained LIO/Sport motion disagreement'
    else:
        c.pose(float('nan'), y, 0., 15.6)
        assert c.state == 'FAULT' and c.reason == 'nonfinite localization'


@pytest.mark.parametrize('name', ['science_grasp_docking.yaml', 'science_grasp_docking_left.yaml'])
def test_actual_dock_profiles_disable_only_disagreement_stop(name):
    from pathlib import Path
    import yaml
    from dataclasses import fields
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config'/name).read_text())
    params = config['science_target_approach']['ros__parameters']
    settings = DockSettings(**{field.name: params['dock_'+field.name]
        for field in fields(DockSettings) if 'dock_'+field.name in params})
    assert settings.motion_disagreement_stop_enabled is False
    assert settings.motion_window_sec == settings.stable_window_sec == .5
    assert settings.position_tolerance == .10
    assert settings.settle_exit_tolerance == .12
    assert settings.settled_sec == .5
    assert DockSettings().motion_disagreement_stop_enabled is True


@pytest.mark.parametrize('name', ['science_grasp_docking.yaml', 'science_grasp_docking_left.yaml'])
@pytest.mark.parametrize('distance,completed', [(.099, True), (.10, True), (.101, False)])
def test_ten_cm_actual_profiles_preserve_stopped_confirmation(name, distance, completed):
    from pathlib import Path
    from dataclasses import fields
    import yaml
    from go2_science_perception.approach_control import ApproachControl, Settings
    from go2_science_perception.grasp_pose_geometry import docking_goal
    params = yaml.safe_load((Path(__file__).resolve().parents[1]/'config'/name).read_text())[
        'science_target_approach']['ros__parameters']
    settings = DockSettings(**{f.name: params['dock_'+f.name]
        for f in fields(DockSettings) if 'dock_'+f.name in params})
    c = ApproachControl(Settings(stop_reference='camera_origin', allow_odom=True),
                        GraspDocking(settings))
    c.target = c.anchor_initial = (1., 0.)
    c.lock_id, c.search_origin, c.state, c.started = 1, (0., 0., 0.), 'LOCK_TARGET', 10.
    goal = docking_goal(c.target, 0., settings)
    # Both components are nonzero: the criterion is the combined planar norm.
    point = (goal[0]-.6*distance, goal[1]-.8*distance)
    for i in range(6):
        tick(c, 10.+i*.1, point)
    c.visual_time = 10.5
    first = c.step(10.5)
    assert first['state'] != 'STOP'
    for i in range(1, 8):
        now = 10.5+i*.1
        c.visual_time = now
        tick(c, now, point)
        out = c.step(now)
    assert (out['state'] == 'STOP') is completed
    assert out['grasp_docking_complete'] is completed


def remembered_arrival_core(distance=.09):
    c = dock_core((distance, 0.))
    c.docking = GraspDocking(DockSettings(position_tolerance=.10,
        settle_exit_tolerance=.12, arrival_latch_enabled=True,
        stable_window_sec=.5, stationary_reset_sec=1.))
    return c


def test_enter_once_then_drift_outside_still_completes_only_after_stop():
    c = remembered_arrival_core()
    first = c.step(10.5)
    assert first['docking_arrival_latched'] and first['state'] != 'STOP'
    original_sample = first['docking_arrival_sample']
    x, y, _ = c.robot
    for i in range(1, 20):
        now = 10.5+i*.1
        c.visual_time = now
        tick(c, now, (x-.07, y))  # Live error is 16cm after one 9cm sample.
        out = c.step(now)
        assert out['vx'] == out['vy'] == 0.
        if i < 5:
            assert out['state'] != 'STOP'  # Drift is not called stopped motion.
        if out['state'] == 'STOP':
            break
    assert out['state'] == 'STOP' and out['grasp_docking_complete']
    assert out['distance'] == pytest.approx(.16)
    assert out['docking_arrival_sample'] == original_sample
    assert not out['docking_pose_in_tolerance']
    assert out['docking_stationary_good_sec'] >= .5-1e-9


def test_stopped_without_ever_entering_range_does_not_complete():
    c = remembered_arrival_core(.101)
    for i in range(20):
        now = 10.5+i*.1
        c.visual_time = now
        tick(c, now+.001, c.robot[:2])
        out = c.step(now+.001)
    assert out['state'] != 'STOP' and not out['docking_arrival_latched']


def test_arrival_does_not_replace_measured_stop():
    c = remembered_arrival_core()
    for i in range(20):
        now = 10.5+i*.1
        c.visual_time = now
        tick(c, now+.001, c.robot[:2])
        c.feedback_speed = .12
        out = c.step(now+.001)
    assert out['docking_arrival_latched'] and out['state'] != 'STOP'
    assert 'sport_still_moving' in out['docking_stationary_blockers']


def test_arrival_memory_keeps_current_heading_requirement():
    c = remembered_arrival_core()
    assert c.step(10.5)['docking_arrival_latched']
    point = c.robot[:2]
    for i in range(1, 25):
        now = 10.5+i*.1
        c.visual_time = now
        tick(c, now, point, yaw=min(.24, i*.04))
        out = c.step(now)
        assert out['state'] not in ('STOP', 'FAULT')
        assert out['vx'] == out['vy'] == 0.
    assert out['docking_arrival_latched']
    assert abs(out['grasp_docking']['yaw_error_rad']) > c.docking.p.yaw_tolerance


@pytest.mark.parametrize('guard', ['stale', 'manual', 'budget', 'reset'])
def test_arrival_memory_cannot_bypass_faults_or_survive_task_reset(guard):
    from dataclasses import replace
    c = remembered_arrival_core()
    assert c.step(10.5)['docking_arrival_latched']
    if guard == 'reset':
        c.reset()
        assert c.docking.arrival_sample is None
        return
    if guard == 'stale':
        out = c.step(11.1)
    elif guard == 'manual':
        c.fault('manual emergency stop')
        out = c.step(10.6)
    else:
        c.p = replace(c.p, odom_max_distance=.1)
        c.visual_time = 9.
        c.lost_at, c.odom_travel = 10., .2
        out = c.step(10.6)
    assert out['state'] == 'FAULT' and not out['grasp_docking_complete']
    assert not out['docking_arrival_latched']
    assert c.docking.arrival_sample is None


@pytest.mark.parametrize('guard', ['stale', 'budget'])
def test_disabling_disagreement_stop_keeps_other_guards(guard):
    from dataclasses import replace
    c = dock_core((.5, 0.))
    c.docking = GraspDocking(DockSettings(motion_window_sec=.5,
        motion_disagreement_stop_enabled=False))
    if guard == 'stale':
        out = c.step(11.1)
        assert out['state'] == 'FAULT' and 'timeout' in out['reason']
    else:
        c.step(10.5)
        c.p = replace(c.p, odom_max_distance=.1)
        c.visual_time = 9.
        c.lost_at = 10.
        c.odom_travel = .02
        out = c.step(10.6)
        assert out['state'] == 'FAULT' and 'budget' in out['reason']


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


def test_response_start_has_no_extra_pose_baseline_gate():
    c = dock_core()
    c.docking = GraspDocking(DockSettings(forward_response_check=True,
        stable_window_sec=.5, forward_effective_speed=.20))
    # Fresh original inputs remain; an incomplete extra stability window must
    # not create a new one-shot startup fault before this measurement test.
    c.pose_history.clear()
    c.camera_history.clear()
    c.docking.begin_response_check(c, 10.5)
    out = c.step(10.5)
    assert out['state'] == 'APPROACH_TO_GRASP_POSE'
    assert 0 < out['vx'] <= .20
    assert out['vy'] == out['wz'] == 0.


def arrival_core():
    c = dock_core((.058, 0.))
    c.docking = GraspDocking(DockSettings(position_tolerance=.06,
        settle_exit_tolerance=.08, settled_sec=.5, stable_window_sec=.5,
        visual_loss_stop_sec=3.))
    c.visual_time = 7.
    return c


def test_expired_visual_at_arrival_allows_only_zero_then_confirmed_stop():
    c = arrival_core()
    for i in range(8):
        t = 10.501+i*.1
        tick(c, t, c.robot[:2])
        out = c.step(t)
        assert out['vx'] == out['vy'] == out['wz'] == 0.
        assert out['state'] != 'FAULT'
    assert out['state'] == 'STOP'
    assert out['grasp_docking_complete']


def test_expired_arrival_still_requires_actual_stop():
    c = arrival_core()
    for i in range(32):
        t = 10.501+i*.1
        tick(c, t, c.robot[:2])
        c.feedback_speed = .1
        out = c.step(t)
        assert out['state'] != 'STOP'
        assert out['vx'] == out['vy'] == out['wz'] == 0.
    assert out['state'] == 'FAULT'
    assert 'settling timed out' in out['reason']


def test_expired_arrival_drift_cannot_restart_correction():
    c = arrival_core()
    c.step(10.5)
    x, y, _ = c.robot
    for i in range(7):
        t = 10.6+i*.1
        tick(c, t, (x-.05, y))
        out = c.step(t)
    assert out['state'] == 'FAULT'
    assert 'lost pose' in out['reason']
    assert out['vx'] == out['vy'] == out['wz'] == 0.


def test_expired_visual_wrong_heading_does_not_enter_arrival_settle():
    c = arrival_core()
    tick(c, 10.5, c.robot[:2], yaw=.3)
    out = c.step(10.5)
    assert out['state'] == 'FAULT'
    assert c.docking.visual_settle_started is None


def test_arrival_exception_does_not_bypass_odometry_budget():
    c = arrival_core()
    c.lost_at = 9.
    c.odom_travel = 5.
    out = c.step(10.5)
    assert out['state'] == 'FAULT'
    assert 'odometry continuation limit' in out['reason']


@pytest.mark.parametrize('profile', ['science_grasp_docking.yaml',
                                     'science_grasp_docking_left.yaml'])
def test_deployed_profiles_continue_map_docking_without_visual_age_fault(profile):
    from pathlib import Path
    import yaml
    p = Path(__file__).parents[1]/'config'/profile
    settings = yaml.safe_load(p.read_text())['science_target_approach']['ros__parameters']
    c = dock_core((.42, 0.))
    c.docking = GraspDocking(DockSettings(
        visual_loss_stop_sec=settings['dock_visual_loss_stop_sec']))
    c.visual_time = 0.
    out = c.step(10.5)
    assert out['state'] == 'APPROACH_TO_GRASP_POSE'
    assert out['vx'] > 0.
    assert c.docking.visual_settle_started is None
    # The retained budget still stops a blind continuation at 5m.
    c.odom_travel = 5.
    tick(c, 10.6, c.robot[:2])
    out = c.step(10.6)
    assert out['state'] == 'FAULT'
    assert out['reason'] == 'odometry continuation limit'


def test_map_docking_can_finish_with_old_visual_only_after_actual_settling():
    c = arrival_core()
    c.docking = GraspDocking(DockSettings(position_tolerance=.06,
        settle_exit_tolerance=.08, stable_window_sec=.5, visual_loss_stop_sec=0.))
    c.visual_time = 0.
    for i in range(8):
        t = 10.501+i*.1
        tick(c, t, c.robot[:2])
        out = c.step(t)
        assert out['vx'] == out['vy'] == out['wz'] == 0.
    assert out['state'] == 'STOP'
    assert out['grasp_docking_complete']


def lateral_only_core(error):
    c = dock_core(error)
    c.docking = GraspDocking(DockSettings(position_tolerance=.08,
        settle_exit_tolerance=.10, fine_forward_correction_enabled=False))
    return c


@pytest.mark.parametrize('error', [(.12, .03), (-.12, .03)])
def test_lateral_only_disables_forward_and_reverse_but_keeps_lateral(error):
    c = lateral_only_core(error)
    out = c.step(10.5)
    assert out['vx'] == pytest.approx(0., abs=1e-12)
    assert out['vy'] > 0.
    assert out['forward_step']['count'] == 0


def test_lateral_only_brakes_existing_speed_and_never_restarts_on_drift():
    c = lateral_only_core((.12, 0.))
    c.docking.velocity_map = (.2, 0.)
    old = .2
    x, y, _ = c.robot
    for i in range(12):
        t = 10.501+i*.1
        # First enter 12cm; later drift back beyond 15cm must not restart vx.
        pos = (x, y) if i == 0 else (x-.04, y)
        tick(c, t, pos)
        out = c.step(t)
        assert out['state'] != 'FAULT'
        assert -1e-12 <= out['vx'] <= old+1e-12
        assert old-out['vx'] <= c.p.max_deceleration*.101+1e-9
        old = out['vx']
    assert out['vx'] == pytest.approx(0., abs=1e-12)
    assert c.docking.lateral_only


def test_eight_cm_acceptance_still_requires_quiet_and_rejects_outside():
    c = lateral_only_core((.075, 0.))
    for i in range(8):
        t = 10.501+i*.1
        tick(c, t, c.robot[:2])
        out = c.step(t)
    assert out['state'] == 'STOP'
    c = lateral_only_core((.085, 0.))
    out = c.step(10.5)
    assert out['state'] != 'STOP'
    assert out['vx'] == pytest.approx(0., abs=1e-12)


def test_one_second_reset_pauses_invalid_time_and_resumes_only_qualified_time():
    d = GraspDocking(DockSettings(stationary_reset_sec=1.))
    assert not d.settle_clock(10., True)
    assert not d.settle_clock(10.3, True)
    assert not d.settle_clock(10.4, False)
    assert not d.settle_clock(11.2, False)
    assert d.stationary_since == 10.
    assert d.stationary_good_sec == pytest.approx(.3)
    assert not d.settle_clock(11.3, True)  # Gap is never added to good time.
    assert not d.settle_clock(11.4, True)
    assert d.settle_clock(11.5, True)


def test_persistent_invalid_feedback_resets_after_one_second():
    d = GraspDocking(DockSettings(stationary_reset_sec=1.))
    d.settle_clock(10., True)
    d.settle_clock(10.3, True)
    d.settle_clock(10.4, False)
    d.settle_clock(11.3, False)
    assert d.stationary_since == 10.
    assert not d.settle_clock(11.401, False)
    assert d.stationary_since is None
    assert d.stationary_good_sec == 0.
    assert not d.settle_clock(11.5, True)
    assert not d.settle_clock(11.9, True)
    assert d.settle_clock(12., True)


def test_eight_cm_outlier_pauses_without_claiming_success_or_resetting_immediately():
    c = dock_core((.075, 0.))
    c.docking = GraspDocking(DockSettings(position_tolerance=.08,
        settle_exit_tolerance=.10, stationary_reset_sec=1.))
    for t in [10.501, 10.601, 10.701]:
        tick(c, t, c.robot[:2])
        c.step(t)
    initial = c.docking.stationary_since
    x, y, _ = c.robot
    tick(c, 10.801, (x-.01, y))
    out = c.step(10.801)
    assert out['state'] != 'STOP'
    assert out['docking_stationary_paused']
    assert out['docking_stationary_since'] == initial
    assert out['docking_stationary_good_sec'] == pytest.approx(.2)
    assert out['vx'] == out['vy'] == out['wz'] == 0.
