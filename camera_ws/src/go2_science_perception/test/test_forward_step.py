"""Synthetic response fixtures only; no hardware calibration claims."""
import hashlib
import json
import math

import pytest

from go2_science_perception.forward_step import ForwardStep, load_forward_profile
from go2_science_perception.grasp_docking import GraspDocking
from go2_science_perception.grasp_pose_geometry import DockSettings
from test_grasp_docking import dock_core, tick


def profile():
    return dict(speed_mps=.15, stop_distance_m=.01, min_step_m=.01, max_step_m=.03,
                max_on_sec=2., settle_timeout_sec=2., max_acceleration=.15,
                max_deceleration=.30, braking_latency=.30, uncertainty_m=.005)


def test_profile_requires_physical_review_and_exact_evidence(tmp_path):
    source = tmp_path/'synthetic.jsonl'
    source.write_text('synthetic test fixture, not physical evidence')
    data = dict(profile(), source_log=str(source),
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                reviewed=False, physical_motion_confirmed=False)
    target = tmp_path/'profile.json'
    target.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_forward_profile(target)
    data.update(reviewed=True, physical_motion_confirmed=True)
    target.write_text(json.dumps(data))
    assert load_forward_profile(target)['speed_mps'] == .15
    source.write_text('changed fixture')
    with pytest.raises(ValueError):
        load_forward_profile(target)


@pytest.mark.parametrize('vx,ex', [(-.08, -.06), (0., .06), (.18, .06), (.08, -.06)])
def test_no_reverse_or_global_floor(vx, ex):
    c = dock_core()
    c.p.braking_latency = .30
    f = ForwardStep(profile())
    assert f.request(c, 10.5, ex, 0., vx, .01, True) is None
    assert f.phase == 'IDLE'


def test_step_uses_measured_displacement_then_waits_for_measured_stop():
    c = dock_core()
    c.p.braking_latency = .30
    f = ForwardStep(profile())
    assert f.request(c, 10.5, .06, 0., .08, .01, True) == .15
    origin = c.robot
    # Time passing alone is not treated as displacement.
    assert f.request(c, 11., .06, 0., .08, .01, True) == .15
    assert f.travel(c) == 0.
    tick(c, 11.1, (origin[0]+.03, origin[1]))
    assert f.request(c, 11.1, .03, 0., .08, .01, True) == 0.
    assert f.phase == 'SETTLE'
    c.actual_speed = .1
    f.observe(c, 11.1)
    assert f.phase == 'SETTLE'
    tick(c, 11.2, c.robot[:2])
    c.actual_speed = c.camera_speed = 0.
    f.observe(c, 11.2)
    assert f.phase == 'IDLE'
    assert f.last_result['measured_travel_m'] == pytest.approx(.03)


def test_mismatched_braking_and_no_progress_fail_closed():
    c = dock_core()
    f = ForwardStep(profile())
    assert f.request(c, 10.5, .06, 0., .08, 0., True) == 0.
    assert c.state == 'FAULT'  # Fixture defaults have a different braking latency.
    c = dock_core()
    c.p.braking_latency = .30
    f = ForwardStep(profile())
    f.request(c, 10.5, .06, 0., .08, 0., True)
    f.request(c, 12.6, .06, 0., .08, 0., True)
    tick(c, 12.7, c.robot[:2])
    c.actual_speed = c.camera_speed = 0.
    f.observe(c, 12.7)
    assert c.state == 'FAULT' and 'no verified displacement' in c.reason


def test_response_check_ramps_then_stops_on_actual_pose_without_grasp_success():
    c = dock_core()
    c.docking = GraspDocking(DockSettings(forward_response_check=True,
                                          forward_response_require_speed=False))
    c.docking.begin_response_check(c, 10.5)
    origin = c.robot
    previous = 0.
    for i in range(12):
        t = 10.6+i*.1
        tick(c, t, origin[:2])
        out = c.step(t)
        assert 0 <= out['vx'] <= .20
        assert out['vx']-previous <= .01501
        assert out['vy'] == out['wz'] == 0.
        previous = out['vx']
    for i in range(12, 25):
        t = 10.6+i*.1
        tick(c, t, (origin[0]+.02, origin[1]))
        out = c.step(t)
        assert out['vx'] == 0.
    assert out['state'] == 'STOP'
    assert not out['grasp_docking_complete']


@pytest.mark.parametrize('kind', ['timeout', 'cross', 'yaw', 'stale'])
def test_response_check_keeps_bounds_and_freshness(kind):
    c = dock_core()
    c.docking = GraspDocking(DockSettings(forward_response_check=True,
                                          forward_response_require_speed=False))
    c.docking.begin_response_check(c, 10.5)
    t = 18.6 if kind == 'timeout' else 10.6
    c.last_tick = t-.1
    tick(c, t, (c.robot[0], c.robot[1]+(.04 if kind == 'cross' else 0.)),
         math.radians(4) if kind == 'yaw' else 0.)
    if kind == 'stale':
        c.odom_time = t-1.
    out = c.step(t)
    assert out['state'] == 'FAULT'
    assert out['vx'] == out['vy'] == out['wz'] == 0.


def test_response_separates_reference_reached_request_zero_and_outlet_zero():
    c = dock_core()
    c.docking = GraspDocking(DockSettings(forward_response_check=True,
        forward_response_max_distance=.35, forward_effective_speed=.20))
    c.docking.begin_response_check(c, 10.5)
    origin = c.robot
    # An apparent early 2cm displacement cannot label a .20m/s test complete.
    tick(c, 10.6, (origin[0]+.025, origin[1]))
    out = c.step(10.6)
    assert out['response_check']['zero_requested'] is None
    for i in range(1, 15):
        t = 10.6+i*.1
        tick(c, t, (origin[0]+.025, origin[1]))
        c.sport_output = dict(received_monotonic=t, vx=min(.20,i*.015), vy=0., wz=0.)
        out = c.step(t)
    assert out['response_check']['outlet_speed_reached'] is not None
    tick(c, 12.1, (origin[0]+.046, origin[1]))
    out = c.step(12.1)
    zero = out['response_check']['zero_requested']
    assert zero == pytest.approx(12.1)
    assert out['vx'] == 0.
    assert out['response_check']['outlet_zero'] is None
    tick(c, 12.2, (origin[0]+.055, origin[1]))
    c.sport_output = dict(received_monotonic=12.2, vx=.10, vy=0., wz=0.)
    assert c.step(12.2)['response_check']['outlet_zero'] is None
    tick(c, 12.3, (origin[0]+.06, origin[1]))
    c.sport_output = dict(received_monotonic=12.3, vx=0., vy=0., wz=0.)
    result = c.step(12.3)['response_check']
    assert result['outlet_zero']['monotonic'] > zero
    assert result['stop_extra_m'] == pytest.approx(.014)
    assert not result['calibration_accepted']


def test_response_final_output_decoder_is_readonly_and_rejects_bad_values():
    from go2_science_perception.forward_step import sport_output_sample
    assert sport_output_sample(1008, '{"x":0.2,"y":0,"z":0}', 1.)['vx'] == .2
    assert sport_output_sample(1003, '', 2.)['vx'] == 0.
    assert sport_output_sample(101, '', 2.) is None
    with pytest.raises(ValueError):
        sport_output_sample(1008, '{"x":NaN,"y":0,"z":0}', 1.)


def test_estimated_trial_requires_opt_in_and_cannot_claim_calibration(tmp_path):
    from pathlib import Path
    source = Path(__file__).parents[1]/'config/forward_step_estimated_trial.json'
    with pytest.raises(ValueError):
        load_forward_profile(source)
    data = load_forward_profile(source, allow_estimated=True)
    assert not data['reviewed'] and not data['physical_motion_confirmed']
    c = dock_core()
    c.p.braking_latency = .30
    f = ForwardStep(data)
    state = f.report(c)
    assert state['profile_available'] and not state['calibration_ready']
    data['stop_distance_m'] = .01
    path = tmp_path/'unsafe_estimate.json';path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_forward_profile(path, allow_estimated=True)


def test_estimated_trial_can_request_step_without_weakening_braking_envelope():
    from pathlib import Path
    source = Path(__file__).parents[1]/'config/forward_step_estimated_trial.json'
    data = load_forward_profile(source, allow_estimated=True)
    c = dock_core();c.p.braking_latency=.30
    c.docking = GraspDocking(DockSettings(position_tolerance=.06, settle_exit_tolerance=.08,
        forward_step_enabled=True, forward_deadzone_guard=True, forward_effective_speed=.20),
        forward_profile=data)
    assert c.docking.forward.request(c,10.5,.13,0.,.10,0.,True)==.20
    assert .01 <= c.docking.forward.planned <= .02
    c = dock_core();c.p.braking_latency=.30
    c.docking = GraspDocking(DockSettings(position_tolerance=.06, settle_exit_tolerance=.08,
        forward_step_enabled=True, forward_deadzone_guard=True, forward_effective_speed=.20),
        forward_profile=data)
    assert c.docking.forward.request(c,10.5,.07,0.,.10,0.,True)==0.
    assert c.state=='FAULT' and 'outside measured step envelope' in c.reason
