"""Small boundary suite for measured-stop fast rechecks and bounded retries."""

from go2_science_perception.approach_control import Settings
from go2_science_perception.search_confirmation import FastConfirmation


def machine(confirmed=True):
    p = Settings(search_enabled=True, stop_reference='camera_origin', anchor_window_sec=1.,
                 stop_confirm_fast_enabled=True)
    seed = dict(id=1, map=(1.4, 0), trigger_stamp_ns=10300000000)
    history = [(10.1, 10100000000, (1.4, 0), 1.), (10.3, 10300000000, (1.4, 0), 1.)]
    return FastConfirmation(p, seed, history, confirmed, 10.3)


def track(t, identity=1, x=1.4, state='CONFIRMED', source=None):
    source = t if source is None else source
    return dict(id=identity, observed_ns=round(source*1e9), age=t-source,
                latest_observation=(x, 0), camera_range_m=1., state=state)


def settle(m):
    m.step(10.4, True, [track(10.4)])
    r = m.step(10.61, True, [track(10.61)])
    assert r['settled_at'] == 10.61


def test_three_new_frames_after_actual_stop_lock_without_point_six_span():
    m = machine()
    for t in (10.4, 10.5, 10.6):
        assert m.step(t, False, [track(t)])['action'] == 'wait'
    m.step(10.7, True, [track(10.7)])
    m.step(10.91, True, [track(10.91)])
    for t in (11.0, 11.1, 11.2):
        r = m.step(t, True, [track(t, state='CANDIDATE')])
    assert r['action'] == 'lock' and r['confirmation_path'] == 'fast'
    assert r['confirmation_sample_count'] == 3
    assert r['confirmation_sample_span'] < .6  # Earlier Manager confirmation reused.


def test_motion_jitter_retains_evidence_but_demands_three_frames_after_new_stop():
    m = machine()
    settle(m)
    m.step(10.7, True, [track(10.7)])
    m.step(10.8, True, [track(10.8)])
    r = m.step(10.9, False, [track(10.9)])
    assert r['action'] == 'wait' and r['settled_at'] is None
    assert len(m.samples) == 2 and len(m.history) == 2
    m.step(11.0, True, [track(11.0)])
    m.step(11.21, True, [track(11.21)])
    for t in (11.3, 11.4):
        assert m.step(t, True, [track(t)])['action'] == 'wait'
    assert m.step(11.5, True, [track(11.5)])['action'] == 'lock'


def test_short_dropout_over_point_three_keeps_samples_and_duplicate_cannot_count():
    m = machine()
    settle(m)
    m.step(10.7, True, [track(10.7)])
    for t in (10.8, 10.9, 11.0):
        m.step(t, True, [track(t, source=10.7)])
    assert len(m.samples) == 1
    m.step(11.05, True, [])
    m.step(11.1, True, [track(11.1)])
    r = m.step(11.2, True, [track(11.2)])
    assert r['action'] == 'lock' and len(r['samples']) == 3


def test_ambiguity_and_changed_identity_never_fast_lock():
    m = machine()
    settle(m)
    for t in (10.7, 10.8, 10.9):
        r = m.step(t, True, [track(t), track(t, identity=2, x=1.42)])
    assert r['action'] == 'wait' and 'target_ambiguous' in r['confirmation_blockers']
    for t in (11.0, 11.1, 11.2):
        r = m.step(t, True, [track(t, identity=2)])
    assert r['action'] == 'wait' and r['confirmation_path'] == 'full_fallback'


def test_unreliable_history_falls_back_to_five_frames_and_point_six_span():
    m = machine(confirmed=False)
    settle(m)
    for t in (10.7, 10.85, 11., 11.15):
        assert m.step(t, True, [track(t)])['action'] == 'wait'
    r = m.step(11.31, True, [track(11.31)])
    assert r['action'] == 'lock' and r['confirmation_path'] == 'full_fallback'
    assert r['confirmation_sample_span'] >= .6


def test_visible_target_gets_one_bounded_in_place_recheck_then_fault():
    m = machine()
    r = m.step(16.31, False, [track(16.31)])
    assert r['timeout_result'] == 'IN_PLACE_RECHECK' and r['action'] == 'wait'
    r = m.step(18.31, False, [track(18.31)])
    assert r['timeout_result'] == 'TIMEOUT' and r['action'] == 'fault'
    assert r['stop_confirm_reason'] == 'parking_unstable'
    m = machine()
    r = m.step(16.31, True, [])
    assert r['action'] == 'search' and 'target_lost' in r['confirmation_blockers']


def test_spread_and_wrong_candidate_position_are_explicit_blockers():
    m = machine()
    settle(m)
    for i, t in enumerate((10.7, 10.8, 10.9)):
        r = m.step(t, True, [track(t, x=1.35 if i % 2 else 1.45)])
    assert r['action'] == 'wait' and 'position_spread_excess' in r['confirmation_blockers']
    r = m.step(11., True, [track(11., x=2.)])
    assert 'candidate_position_inconsistent' in r['confirmation_blockers']


def test_controller_integration_uses_actual_stop_and_switch_off_preserves_original_path():
    from test_search_approach import search, sample

    c = search()
    c.p.stop_confirm_fast_enabled = True
    for i in (1, 2):
        r = sample(c, 10+i*.1, feedback=.2)
    assert r['state'] == 'STOP_CONFIRM'  # Manager already confirmed; only two SEARCH frames.
    assert c.stop_candidate['manager_confirmed'] and len(c.stop_candidate['pre_stop_history']) == 2
    for i in range(3, 7):
        assert sample(c, 10+i*.1, feedback=.2)['state'] == 'STOP_CONFIRM'
    for i in range(7, 21):
        r = sample(c, 10+i*.1)
        if r['state'] == 'LOCK_TARGET':
            break
    assert r['state'] == 'LOCK_TARGET' and r['vx'] == r['vy'] == r['wz'] == 0
    assert c.stop_confirm_attempts[-1]['confirmation_path'] == 'fast'
    c = search()
    assert not c.p.stop_confirm_fast_enabled
    assert sample(c, 10.1)['state'] == 'SEARCH'
    assert sample(c, 10.2)['state'] == 'SEARCH'
    assert sample(c, 10.3)['state'] == 'STOP_CONFIRM'


def test_search_history_survives_finite_missing_snapshot_but_expires():
    from test_search_approach import search, sample

    c = search()
    c.p.stop_confirm_fast_enabled = True
    sample(c, 10.1)
    assert len(c.trigger_history[1]) == 1
    c.observations([], 10.65)
    assert len(c.trigger_history[1]) == 1
    # Refresh state rather than advance the control loop by an invalid long tick.
    c.observations([], 10.9)
    assert not c.trigger_history[1]


def test_late_third_frame_cannot_lock_after_the_bounded_recheck_deadline():
    m = machine()
    m.step(16.31, False, [track(16.31)])
    m.step(17.6, True, [track(17.6)])
    m.step(17.81, True, [track(17.81)])
    m.step(18.1, True, [track(18.1)])
    m.step(18.2, True, [track(18.2)])
    r = m.step(18.31, True, [track(18.31)])
    assert r['action'] == 'fault' and r['stop_confirm_reason'] == 'confirmation_deadline_exceeded'
