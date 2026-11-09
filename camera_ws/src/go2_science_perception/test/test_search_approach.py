"""Focused stop-before-lock, source freshness, candidate identity and route checks."""

from go2_science_perception.approach_control import ApproachControl, Settings


def search():
    c = ApproachControl(Settings(search_enabled=True, stop_reference='camera_origin',
                                 anchor_window_sec=.8, anchor_max_spread=.08,
                                 allow_odom=True, stop_confirm_timeout=2.0))
    c.pose(0, 0, 0, 10)
    c.camera_pose(.36, 0, 10)
    c.feedback_time = 10
    assert c.start(10)
    return c


def sample(c, t, x=1.4, position=.03, distance=1.1, feedback=0.0, identity=1, age=0.0,
           stamp=None):
    c.pose(position, 0, 0, t)
    c.camera_pose(position+.36, 0, t)
    c.feedback_time, c.feedback_speed, c.feedback_yaw_rate = t, feedback, 0.0
    c.observations([dict(id=identity, state='CONFIRMED', visited=False, age=age,
                         lifetime=.1, observed_ns=int((t-age)*1e9) if stamp is None else stamp,
                         camera_range_m=distance, position=[1.4, 0, 0],
                         latest_observation=[x, 0, 0])], t)
    return c.step(t)


def trigger(c):
    for i in range(1, 4):
        r = sample(c, 10+i*.1, position=i*.01, feedback=.15)
    assert r['state'] == 'STOP_CONFIRM'
    assert r['transition'] == 'SEARCH->STOP_CONFIRM'
    assert r['vx'] == r['wz'] == 0
    assert c.target is None
    return r


def test_far_detection_does_not_lock_and_trigger_is_consecutive():
    c = search()
    for i in range(1, 8):
        assert sample(c, 10+i*.1, distance=2.1)['state'] == 'SEARCH'
    assert c.target is None
    for i in range(8, 11):
        r = sample(c, 10+i*.1)
    assert r['state'] == 'STOP_CONFIRM' and r['vx'] == 0
    assert c.stop_request_range_m <= 1.8


def test_moving_feedback_never_confirms_then_new_stationary_samples_handoff():
    c = search()
    trigger(c)
    for i in range(4, 9):
        r = sample(c, 10+i*.1, feedback=.2)
        assert r['state'] == 'STOP_CONFIRM' and r['vx'] == r['wz'] == 0
        assert c.settled_at is None and not c.stop_samples
    for i in range(9, 24):
        r = sample(c, 10+i*.1)
        if r.get('transition') == 'STOP_CONFIRM->APPROACH':
            break
    assert r['transition'] == 'STOP_CONFIRM->APPROACH', r
    assert r['vx'] == r['wz'] == 0
    assert len(c.stop_samples) >= 5 and c.confirmation_spread < .08
    assert all(s[0] > c.settled_at for s in c.stop_samples)
    assert (c.stop_samples[-1][1]-c.stop_samples[0][1])*1e-9 >= .6
    assert c.confirmed_at-c.stop_confirm_started < 2
    assert c.step(10+i*.1+.01)['state'] == 'APPROACH_VISUAL'


def test_duplicate_and_pre_settle_observations_cannot_supply_confirmation():
    c = search()
    trigger(c)
    for i in range(4, 10):
        sample(c, 10+i*.1)
    assert c.settled_at is not None
    c.stop_samples.clear()
    c.stop_sample_stamps.clear()
    old = c.settled_at-.01
    t = 11.0
    sample(c, t, age=t-old)
    assert not c.stop_samples
    for i in range(1, 5):
        t = 11+i*.1
        r = sample(c, t, age=t-11.05)
    assert r['state'] == 'STOP_CONFIRM' and len(c.stop_samples) == 1
    assert c.target is None


def test_different_spatial_candidate_times_out_and_cooldown_keeps_route_origin():
    c = search()
    trigger(c)
    origin = c.search_origin
    for i in range(4, 25):
        r = sample(c, 10+i*.1, x=2.0, identity=99)
        if r.get('transition') == 'STOP_CONFIRM->SEARCH':
            break
    assert r['state'] == 'SEARCH' and c.target is None
    assert c.stop_confirm_attempts[-1]['result'] == 'TIMEOUT'
    assert c.search_origin == origin and c.search_started == 10
    timeout = 10+i*.1
    for j in range(1, 6):
        assert sample(c, timeout+j*.1)['state'] == 'SEARCH'


def test_spread_and_restarted_motion_do_not_pass_static_confirmation():
    c = search()
    trigger(c)
    for i in range(4, 11):
        sample(c, 10+i*.1)
    assert c.settled_at is not None and c.stop_samples
    sample(c, 11.1, feedback=.15)
    assert c.settled_at is None and not c.stop_samples
    for i in range(12, 25):
        r = sample(c, 10+i*.1, x=1.4+(.05 if i % 2 else -.05))
        if r['state'] == 'SEARCH':
            break
    assert c.target is None
    assert c.stop_confirm_attempts[-1]['result'] == 'TIMEOUT'


def test_search_route_end_and_localization_loss_stop():
    c = search()
    for i in range(1, 102):
        t = 10+i*.1
        c.pose(i*.01, 0, 0, t)
        c.camera_pose(i*.01+.36, 0, t)
        c.feedback_time = t
        r = c.step(t)
        if r['state'] == 'STOP':
            break
    assert r['reason'] == 'search route ended without target' and r['vx'] == 0
    c = search()
    assert c.step(10.6)['state'] == 'FAULT'
    assert c.step(10.7)['vx'] == 0
    c = search()
    trigger(c)
    assert c.step(11)['state'] == 'FAULT'
    assert c.step(11.1)['vx'] == 0


def test_five_metre_search_passes_old_limits_and_stops_at_route_end():
    c = search()
    c.p.search_distance, c.p.search_timeout = 5.0, 60.0
    for i in range(1, 502):
        t = 10+i*.1
        c.pose(i*.01, 0, 0, t)
        c.camera_pose(i*.01+.36, 0, t)
        c.feedback_time = t
        r = c.step(t)
        if i < 500:
            assert r['state'] == 'SEARCH'
        else:
            break
    assert r['reason'] == 'search route ended without target'
    assert r['search_progress'] == 5.0
    assert r['vx'] == r['wz'] == 0


def test_camera_range_matches_both_source_timestamp_and_raw_map_point():
    from types import SimpleNamespace
    from go2_science_perception.science_target_approach import ScienceTargetApproach

    holder = SimpleNamespace(map_measurements=[
        (200, (1.4, 0, 0), 2.1), (200, (2.0, 0, 0), .4), (199, (1.4, 0, 0), .3)])
    track = dict(observed_ns=200, latest_observation=[1.4, 0, 0])
    assert ScienceTargetApproach._matched_camera_range(holder, track) == 2.1
    track['observed_ns'] = 201
    assert ScienceTargetApproach._matched_camera_range(holder, track) is None
    track.update(observed_ns=200, latest_observation=[1.5, 0, 0])
    assert ScienceTargetApproach._matched_camera_range(holder, track) is None


def test_extended_confirmation_budget_keeps_zero_until_six_seconds():
    from dataclasses import replace

    c = search()
    c.p = replace(c.p, stop_confirm_timeout=6.0)
    trigger(c)
    for i in range(4, 65):
        r = sample(c, 10+i*.1, feedback=.15)
        if 10+i*.1-c.stop_confirm_started < 6:
            assert r['state'] == 'STOP_CONFIRM' and r['vx'] == r['wz'] == 0
        else:
            break
    assert r['state'] == 'SEARCH' and c.target is None
