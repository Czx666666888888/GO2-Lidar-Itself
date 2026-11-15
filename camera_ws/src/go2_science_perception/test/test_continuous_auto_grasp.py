"""Continuous exploration and nearest mirrored docking candidate selection."""
from dataclasses import fields, replace
import math

import pytest

from go2_science_perception.approach_control import ApproachControl
from go2_science_perception.grasp_docking import GraspDocking
from go2_science_perception.grasp_pose_geometry import DockSettings, shared_side_settings
from test_search_approach import search


def test_continuous_search_goes_beyond_old_distance_and_time_without_braking_to_endpoint():
    c = search()
    c.p.search_continuous = True
    # Continuous, fresh samples avoid inventing a localization jump/time gap.
    for i in range(1, 1201):
        t, x = 10.+i*.1, i*.02
        c.pose(x, 0., 0., t)
        c.camera_pose(x+.36, 0., t)
        c.feedback_time, c.feedback_speed, c.feedback_yaw_rate = t, .2, 0.
        c.observations([], t)
        out = c.step(t)
    assert out['state'] == 'SEARCH' and out['vx'] > 0
    assert out['search_progress'] > 5 and t-c.search_started > 60
    c.feedback_time = t-1
    assert c.step(t+.1)['state'] == 'FAULT'  # Existing feedback protection retained.


@pytest.mark.parametrize('y,expected', [(0.5, 'left'), (-.5, 'right'), (0., 'right')])
@pytest.mark.parametrize('yaw', [0., math.pi/2])
def test_side_uses_travel_to_docking_pose_in_current_heading(y, expected, yaw):
    settings = DockSettings(target_s1_y=.37)
    options = shared_side_settings(settings)
    c = ApproachControl(docking=GraspDocking(settings), docking_options=options)
    c.robot = (0., 0., yaw)
    c.search_origin = c.robot
    point = (math.cos(yaw)*1.-math.sin(yaw)*y, math.sin(yaw)*1.+math.cos(yaw)*y, 0.)
    chosen = c._search_candidates([dict(id=1, position=point)])
    assert chosen[0]['id'] == 1 and c.grasp_side == expected
    assert c.side_candidates[0]['distance_m'] <= c.side_candidates[1]['distance_m']


def test_multiple_candidates_choose_nearest_target_side_pair():
    p = DockSettings(target_s1_y=.37)
    c = ApproachControl(docking=GraspDocking(p), docking_options=shared_side_settings(p))
    c.robot = c.search_origin = (0., 0., 0.)
    chosen = c._search_candidates([
        dict(id=1, position=(1.4, -.5, 0.)), dict(id=2, position=(.7, .5, 0.))])
    assert chosen[0]['id'] == 2 and c.grasp_side == 'left'


def test_auto_left_side_remains_selected_through_stop_confirmation():
    c = search()
    p = DockSettings(target_s1_y=.37)
    c.docking = GraspDocking(p)
    c.docking_options = shared_side_settings(p)
    for i in range(1, 25):
        now = 10.+i*.1
        c.pose(0., 0., 0., now)
        c.camera_pose(.36, 0., now)
        c.feedback_time, c.feedback_speed, c.feedback_yaw_rate = now, 0., 0.
        point = [.8, .5, 0.]
        c.observations([dict(id=4, position=point, latest_observation=point,
                            state='CONFIRMED', visited=False, age=0.,
                            observed_ns=int(now*1e9), camera_range_m=.7)], now)
        c.step(now)
        if c.state == 'LOCK_TARGET':
            break
    assert c.state == 'LOCK_TARGET'
    assert c.grasp_side == 'left' and c.docking.p.target_s1_y == -.37
    assert c.stop_candidate['grasp_side'] == 'left'


@pytest.mark.parametrize('source_side', ['right', 'left'])
def test_left_and_right_share_every_tuning_value_except_mirrored_y(source_side):
    base = DockSettings(target_s1_y=.37, position_tolerance=.08,
                        settle_exit_tolerance=.10, arrival_latch_enabled=True)
    if source_side == 'left':
        base = replace(base, target_s1_y=-.37, region_y_min=-.33, region_y_max=-.29)
    options = shared_side_settings(base)
    for f in fields(DockSettings):
        if f.name not in ('target_s1_y', 'region_y_min', 'region_y_max'):
            assert getattr(options['left'], f.name) == getattr(options['right'], f.name)
    assert options['left'].target_s1_y == -.37 and options['right'].target_s1_y == .37
