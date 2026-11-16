"""Focused memory lifecycle, direct docking, reacquisition and safe miss checks."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from go2_science_perception.approach_control import ApproachControl, Settings
from go2_science_perception.grasp_docking import GraspDocking
from go2_science_perception.grasp_pose_geometry import DockSettings, shared_side_settings
from go2_science_perception.science_target_manager import TargetTracker
from go2_science_perception.s1_task_sequence import finish_docked_task, SAFE_MISSES


def track(identity, x, y=0., visited=False):
    return dict(id=identity, state='STALE', position=[x,y,0.], latest_observation=[x,y,0.],
                age=60., observed_ns=40_000_000_000, first_seen_ns=39_000_000_000,
                visited=visited, memory_confirmed=True, observation_count=5, memory_spread_m=.01)


def ready(core, now):
    core.pose(0., 0., 0., now)
    core.camera_pose(.36, 0., now)
    core.feedback_time, core.feedback_speed, core.feedback_yaw_rate = now, 0., 0.


def memory_core(tracks, resume=True):
    docking = GraspDocking(DockSettings())
    c = ApproachControl(Settings(multi_target_memory=True, resume_memory=resume,
        search_enabled=True, anchor_window_sec=1., stop_reference='camera_origin', allow_odom=True,
        odom_max_distance=5.), docking, shared_side_settings(docking.p))
    ready(c, 100.)
    c.observations(tracks, 100.)
    assert c.start(100.)
    return c


def test_distinct_tracks_survive_arm_pause_without_reusing_source_stamps():
    t = TargetTracker(.2, 3, 15, remember_confirmed=True)
    for stamp in (1,2,3):
        t.update_frame([(1.,0.,0.), (2.,0.,0.)], stamp*100_000_000)
    assert [p.target_id for p in t.tracks] == [1,2]
    counts = [p.observation_count for p in t.tracks]
    t.update_frame([(1.,0.,0.), (2.,0.,0.)], 300_000_000)
    assert [p.observation_count for p in t.tracks] == counts
    t.expire(80_000_000_000)
    assert len(t.tracks) == 2 and all(p.state == 'STALE' for p in t.tracks)
    t.update_frame([(2.01,0.,0.)], 81_000_000_000)
    assert t.tracks[1].target_id == 2 and t.tracks[1].state == 'CONFIRMED'


def test_next_round_selects_nearest_unhandled_dock_and_moves_without_new_rgb():
    c = memory_core([track(1,.5,visited=True), track(2,1.), track(3,3.)])
    assert c.state == 'LOCK_TARGET' and c.lock_id == 2 and c.memory_goal
    assert c.visual_time == 40.  # Never pretend memory is a fresh visual measurement.
    ready(c, 100.1)
    report = c.step(100.1)
    assert report['state'] != 'FAULT' and report['memory_goal']
    assert c.target == (1.,0.) and c.stop_candidate is None
    report = c.step(101.)
    assert report['state'] == 'FAULT' and report['vx'] == 0.  # Localization is mandatory.


def test_first_round_or_no_eligible_memory_keeps_original_search():
    assert memory_core([track(1,1.)], resume=False).state == 'SEARCH'
    assert memory_core([track(1,1.,visited=True)]).state == 'SEARCH'
    assert memory_core([track(1,1.), track(2,1.1)]).state == 'SEARCH'
    bad = track(1,1.); bad['memory_spread_m'] = .5
    assert memory_core([bad]).state == 'SEARCH'


def test_memory_recheck_requires_three_distinct_unique_observations():
    c = memory_core([track(2,1.)])
    for i in range(3):
        t = track(2,1.01);t.update(state='CONFIRMED', age=0., observed_ns=101_000_000_000+i*100_000_000)
        c.observations([t],101.+i*.1)
        if i < 2:assert c.anchor_updates == 0
    assert c.anchor_updates == 1 and c.memory_reconfirmed == 1
    c.observations([t],101.3)
    assert c.anchor_updates == 1


@pytest.mark.parametrize('outcome', sorted(SAFE_MISSES))
def test_only_known_safe_miss_with_new_log_and_zero_feedback_can_stand_up(outcome):
    before = dict(state='ZERO_HOLD', busy=False, navigation_allowed=True, last_log='old')
    after = dict(before, last_log='new', last_result=outcome, last_exit_code=3,
                 at_zero=True, communication_state='OK', readiness_state='ZERO_HOLD')
    states = iter([before,before,after,after])
    def request(command):
        return {'accepted':True} if command == 'grasp' else next(states)
    posture = Mock(return_value={'confirmed':True})
    result = finish_docked_task(request, posture, Mock(), allow_missed=True, sleep=lambda _:None)
    assert result['outcome'] == 'MISSED' and result['task_motion_complete']
    assert posture.call_args.args == ('up',)


def test_communication_fault_is_not_a_miss_and_never_stands_up():
    before = dict(state='ZERO_HOLD', busy=False, navigation_allowed=True, last_log='old')
    states = iter([before,before,dict(state='FAULT_IO',busy=False)])
    request = lambda cmd: {'accepted':True} if cmd=='grasp' else next(states)
    posture = Mock(return_value={'confirmed':True})
    with pytest.raises(RuntimeError):
        finish_docked_task(request, posture, Mock(), allow_missed=True, sleep=lambda _:None)
    assert posture.call_args_list[-1].args == ('down',)
