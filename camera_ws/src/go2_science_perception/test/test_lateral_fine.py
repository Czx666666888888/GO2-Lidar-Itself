"""Dock-only lateral request, braking, slew and gate audit; no ROS initialization."""
import ast
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

from go2_science_perception.grasp_pose_geometry import DockSettings
from test_axis_arrival_latch import core, step


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('sign', [-1, 1])
def test_pure_lateral_can_reach_point_four_with_original_acceleration(side, sign):
    c = core(side)
    previous = 0.
    for i in range(35):
        out = step(c, 10.+i*.1, .05, sign*.55)
        assert out['docking_lateral_fine_active']
        assert out['docking_lateral_fine_nominal_mps'] == .4
        assert abs(out['vy']-previous) <= c.p.max_acceleration*.1+1e-10
        assert out['vx'] == pytest.approx(0.)
        assert math.hypot(out['vx'], out['vy']) <= .4+1e-10
        previous = out['vy']
    assert out['vy'] == pytest.approx(sign*.4)
    assert not out['grasp_docking_complete']


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('ey', [-.025, .025])
def test_lateral_near_target_is_braked_below_nominal_and_keeps_correcting(side, ey):
    c = core(side)
    for i in range(15):
        out = step(c, 10.+i*.1, .05, ey)
        assert out['docking_lateral_fine_active']
        assert 0 < abs(out['vy']) <= out['docking_lateral_fine_braking_limit_mps']+1e-10
        assert out['vy']*ey > 0
        assert abs(out['vy']) < .4
    assert not out['docking_lateral_arrival_latched']


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('offset', [-.01, .01])
def test_twenty_eight_to_thirty_cm_latches_and_requests_zero(side, offset):
    c = core(side)
    assert abs(c.docking.p.target_s1_y) == .29
    out = step(c, 10., .05, offset)
    assert .28-1e-12 <= abs(out['grasp_docking']['predicted_target_s1_xy'][1]) <= .30+1e-12
    assert out['docking_arrival_latched']
    assert out['vx'] == out['vy'] == 0.
    assert not out['grasp_docking_complete']  # Still needs the original stop confirmation.


@pytest.mark.parametrize('ex,yaw', [(.09, 0.), (.05, math.radians(15))])
def test_no_point_four_before_forward_arrival_or_with_wrong_heading(ex, yaw):
    c = core()
    for i in range(25):
        out = step(c, 10.+i*.1, ex, .55, yaw=yaw)
        assert not out['docking_lateral_fine_active']
        assert math.hypot(out['vx'], out['vy']) <= .3+1e-10


@pytest.mark.parametrize('guard', ['fault', 'stale'])
def test_fast_lateral_request_never_survives_fault_or_stale_feedback(guard):
    c = core()
    for i in range(35):
        step(c, 10.+i*.1, .05, .55)
    if guard == 'fault':
        c.fault('manual emergency stop')
    out = c.step(14.5 if guard == 'stale' else 13.5)
    assert out['state'] == 'FAULT'
    assert out['vx'] == out['vy'] == out['wz'] == 0.


@pytest.mark.parametrize('speed', [-.01, .4001, float('nan')])
def test_invalid_lateral_fine_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        DockSettings(lateral_fine_speed=speed)


@pytest.mark.parametrize('mode,speed,valid', [
    ('dock', .4, True), ('dock', .401, False), ('plain', .4, False),
    ('disabled', .4, False), ('response', .4, False), ('plain', .3, True)])
def test_gate_audit_only_accepts_raised_cap_for_opted_in_docking(mode, speed, valid):
    # Exercise the production callback without importing ROS native extensions.
    path = Path(__file__).parents[1]/'go2_science_perception/science_target_approach.py'
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_gate_result')
    scope = dict(math=math, time=time, parameter_value_to_python=lambda value: value)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
    c = core()
    if mode == 'plain':
        c.docking = None
    elif mode == 'disabled':
        c.docking.p = replace(c.docking.p, lateral_fine_speed=0.)
    elif mode == 'response':
        c.docking.p = replace(c.docking.p, forward_response_check=True)
    node = SimpleNamespace(
        core=c, gate_verified_at=-math.inf,
        get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name='go2_safety_gate')])
    future = SimpleNamespace(result=lambda: SimpleNamespace(values=[False, 0., speed, .3, .3, .3, True]))
    scope['_gate_result'](node, future)
    assert (node.gate_verified_at > 0) is valid


@pytest.mark.parametrize('docking,response,raised', [
    (True, False, True), (False, False, False), (True, True, False)])
def test_gate_launch_override_is_scoped_to_docking_not_response_check(docking, response, raised):
    path = Path(__file__).resolve().parents[3]/'scripts/run_map_anchor_experiment.py'
    tree = ast.parse(path.read_text())
    assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                      and any(isinstance(target, ast.Name) and target.id == 'gate_log'
                              for target in n.targets))
    called = []
    fake = SimpleNamespace(args=SimpleNamespace(grasp_pose=docking, forward_response_check=response),
                           launch=lambda *values: called.append(values))
    exec(compile(ast.Module(body=[assignment], type_ignores=[]), str(path), 'exec'),
         dict(self=fake, share=Path('/tmp/example_share')))
    assert ('max_speed:=0.4' in called[0][2]) is raised
