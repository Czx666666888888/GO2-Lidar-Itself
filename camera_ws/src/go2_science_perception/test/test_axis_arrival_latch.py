"""Independent six/one-centimetre arrival memory; no hardware or ROS init."""
from dataclasses import fields, replace
from pathlib import Path
import math

import pytest
import yaml

from go2_science_perception.approach_control import ApproachControl, Settings
from go2_science_perception.grasp_docking import GraspDocking
from go2_science_perception.grasp_pose_geometry import DockSettings
from test_grasp_docking import tick


def core(side='right'):
    name = 'science_grasp_docking'+('_left' if side == 'left' else '')+'.yaml'
    params = yaml.safe_load((Path(__file__).parents[1]/'config'/name).read_text())[
        'science_target_approach']['ros__parameters']
    values = {f.name: params['dock_'+f.name] for f in fields(DockSettings)
              if 'dock_'+f.name in params}
    settings = DockSettings(**values)
    c = ApproachControl(Settings(stop_reference='camera_origin', allow_odom=True),
                        GraspDocking(settings))
    c.target = c.anchor_initial = (1., 0.)
    c.lock_id, c.search_origin, c.state, c.started = 1, (0., 0., 0.), 'LOCK_TARGET', 10.
    c.docking.heading = 0.
    return c


def step(c, now, ex, ey, speed=0., yaw=0.):
    # Position constructed from the actual S1 target, including nonzero heading.
    p = c.docking.p
    target_body = (p.s1_base_x-p.target_s1_x+ex, p.s1_base_y-p.target_s1_y+ey)
    x = c.target[0]-(math.cos(yaw)*target_body[0]-math.sin(yaw)*target_body[1])
    y = c.target[1]-(math.sin(yaw)*target_body[0]+math.cos(yaw)*target_body[1])
    tick(c, now, (x, y), yaw)
    c.visual_time = now
    c.feedback_speed = speed
    return c.step(now)


@pytest.mark.parametrize('side', ['right', 'left'])
@pytest.mark.parametrize('first_axis', ['longitudinal', 'lateral'])
def test_one_axis_cannot_stop_and_remaining_axis_keeps_correcting(side, first_axis):
    c = core(side)
    ex, ey = (.05, .0793) if first_axis == 'longitudinal' else (.09, .005)
    for i in range(10):
        out = step(c, 10.+i*.1, ex, ey)
        assert out['state'] != 'STOP' and not out['grasp_docking_complete']
        assert out['docking_phase'] != 'STOP_WAIT'
        assert out['docking_'+first_axis+'_arrival_latched']
        assert not out['docking_arrival_latched']
    assert (out['vx'] == pytest.approx(0.)) if first_axis == 'longitudinal' else out['vx'] > 0
    assert (out['vy'] > 0) if first_axis == 'longitudinal' else out['vy'] == pytest.approx(0.)


@pytest.mark.parametrize('side', ['right', 'left'])
def test_lateral_correction_does_not_stall_in_old_eight_cm_braking_deadband(side):
    c = core(side)
    for i in range(10):
        out = step(c, 10.+i*.1, 0., .025)
    assert out['vy'] > 0 and not out['docking_lateral_arrival_latched']
    assert not out['grasp_docking_complete']


@pytest.mark.parametrize('side', ['right', 'left'])
@pytest.mark.parametrize('first_axis', ['longitudinal', 'lateral'])
def test_axes_latch_at_different_times_and_still_require_real_stop(side, first_axis):
    c = core(side)
    first = step(c, 10., .05 if first_axis == 'longitudinal' else .09,
                 .08 if first_axis == 'longitudinal' else .005, speed=.2)
    original = first['docking_'+first_axis+'_arrival_sample']
    # Preserve the first arrival despite drift; the other axis reaches its band.
    for i in range(1, 9):
        out = step(c, 10.+i*.1, .11 if first_axis == 'longitudinal' else .05,
                   .005 if first_axis == 'longitudinal' else .08, speed=.2)
        assert out['docking_arrival_latched']
        assert not out['grasp_docking_complete']
    assert out['docking_'+first_axis+'_arrival_sample'] == original
    for i in range(9, 25):
        out = step(c, 10.+i*.1, .11 if first_axis == 'longitudinal' else .05,
                   .005 if first_axis == 'longitudinal' else .08)
        if out['grasp_docking_complete']:
            break
    assert out['state'] == 'STOP' and out['grasp_docking_complete']


@pytest.mark.parametrize('side', ['right', 'left'])
@pytest.mark.parametrize('lateral_error,accepted', [
    (.01, True), (.0101, False), (-.01, True), (-.0101, False)])
def test_lateral_band_is_actual_s1_distance_even_with_heading_error(side, lateral_error, accepted):
    c = core(side)
    out = step(c, 10., .05, lateral_error, yaw=math.radians(9))
    assert out['docking_lateral_arrival_latched'] is accepted
    assert out['docking_body_error_xy'] == pytest.approx([.05, lateral_error])


@pytest.mark.parametrize('guard', ['fault', 'reset', 'timeout'])
def test_partial_arrival_cannot_survive_fault_reset_or_stale_pose(guard):
    c = core()
    assert step(c, 10., .05, .08)['docking_longitudinal_arrival_latched']
    if guard == 'reset':
        c.reset()
    else:
        if guard == 'fault':
            c.fault('manual emergency stop')
        c.step(11. if guard == 'timeout' else 10.1)
        assert c.state == 'FAULT'
    assert c.docking.arrival_sample is None
    assert c.docking.longitudinal_arrival_sample is None
    assert c.docking.lateral_arrival_sample is None


def test_both_axis_latches_do_not_replace_current_heading_requirement():
    c = core()
    for i in range(12):
        out = step(c, 10.+i*.1, .05, .005, yaw=math.radians(15))
        assert out['docking_arrival_latched']
        assert not out['grasp_docking_complete'] and out['state'] != 'STOP'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_full_reopens_the_early_lateral_latch_and_completes_only_on_both_live_axes(side):
    c = core(side)
    c.docking.p = replace(c.docking.p, require_current_arrival=True)
    out = step(c, 10., .15, .005)
    assert out['docking_lateral_arrival_latched']
    # The red block's early lateral sample was in range. By longitudinal
    # arrival, the logged lateral error had drifted to about 21 cm.
    for i in range(1, 13):
        out = step(c, 10.+i*.1, max(.05, .15-.01*i), min(.2116, .005+.02*i))
    assert out['docking_longitudinal_arrival_latched']
    assert not out['docking_lateral_arrival_latched']
    assert not out['grasp_docking_complete'] and out['docking_phase'] != 'STOP_WAIT'
    assert out['vy'] > 0.

    for i in range(13, 40):
        out = step(c, 10.+i*.1, .05, max(.005, .2116-.02*(i-12)))
        if out['grasp_docking_complete']:
            break
    assert out['state'] == 'STOP'
    assert out['docking_pose_in_tolerance']
    assert out['docking_longitudinal_in_tolerance'] and out['docking_lateral_in_tolerance']


def test_full_drift_during_stop_wait_restarts_correction_and_the_settle_window():
    c = core('left')
    c.docking.p = replace(c.docking.p, require_current_arrival=True)
    for i in range(4):
        out = step(c, 10.+i*.1, .02, .005)
    assert out['docking_phase'] == 'STOP_WAIT'
    drift = step(c, 10.4, .02, .033)
    assert drift['docking_stop_wait_exit_reason'] == 'current_pose_outside_tolerance'
    assert not drift['docking_lateral_arrival_latched']
    assert not drift['grasp_docking_complete']
    assert drift['vy'] > 0.
    for i in range(5, 10):
        out = step(c, 10.+i*.1, .02, .005)
        assert not out['grasp_docking_complete']
    for i in range(10, 23):
        out = step(c, 10.+i*.1, .02, .005)
        if out['grasp_docking_complete']:
            break
    assert out['grasp_docking_complete']
    drift_after_stop = step(c, 12.4, .02, .033)
    assert not drift_after_stop['grasp_docking_complete']
    assert drift_after_stop['vx'] == drift_after_stop['vy'] == drift_after_stop['wz'] == 0.
