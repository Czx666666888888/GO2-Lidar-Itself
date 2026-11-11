"""Offline closed-loop and fault-injection acceptance of the approach core."""

import math
import pytest

from go2_science_perception.approach_control import ApproachControl, Settings


def track(identity=1, x=2.0, y=0.0, age=0.0, **kwargs):
    return dict(id=identity, position=[x, y, 0.0], age=age, lifetime=5.0,
                state=kwargs.get('state', 'CONFIRMED'), visited=kwargs.get('visited', False))


def ready(**kwargs):
    c = ApproachControl(Settings(**kwargs))
    c.pose(0, 0, 0, 10)
    c.feedback_time = 10
    c.observations([track(), track(2, 3)], 10)
    assert c.confirm(10)
    return c


def test_manual_confirmation_nearest_and_lock():
    c = ready()
    c.observations([track(), track(2, 0.9)], 10.1)
    assert c.lock_id == 1
    assert c.target == (2.0, 0.0)
    assert not c.confirm(10.1)


def test_alignment_and_acceleration_caps():
    c = ready()
    c.robot = (0, 0, math.pi / 2)
    r = c.step(10)
    assert r['state'] == 'ALIGN' and r['vx'] == 0
    assert r['wz'] == -0.3
    c.robot = (0, 0, 0)
    assert c.step(10.1)['vx'] <= 0.015 + 1e-9


def test_closed_loop_stops_without_restart():
    c = ready(max_run_time=60)
    x, speed, rows = 0.0, 0.0, []
    for i in range(1, 550):
        now = 10 + i * 0.1
        x += speed * 0.1
        c.pose(x, 0, 0, now)
        c.feedback_time = now
        c.observations([track()], now)
        r = c.step(now)
        rows.append(r)
        speed = r['vx']
        assert 0 <= speed <= 0.3
        if r['state'] == 'STOP':
            break
    assert r['state'] == 'STOP'
    assert abs(r['distance'] - 0.8) <= 0.031
    assert max(row['vx'] for row in rows) == pytest.approx(0.3)
    assert rows[-2]['vx'] < 0.05
    c.pose(x - 0.1, 0, 0, now + 0.2)
    assert c.step(now + 0.2)['vx'] == 0
    assert not c.confirm(now + 0.2)


@pytest.mark.parametrize('failure',
                         ['vision', 'odom', 'feedback', 'jump', 'removed', 'visited', 'reset'])
def test_faults_latch(failure):
    c = ready()
    c.step(10)
    if failure == 'vision':
        c.pose(0, 0, 0, 10.6)
        c.feedback_time = 10.6
        now = 10.6
        c.last_tick = 10.5
    elif failure == 'odom':
        c.feedback_time = 10.5
        now = 10.5
        c.last_tick = 10.4
    elif failure == 'feedback':
        c.pose(0, 0, 0, 10.6)
        now = 10.6
        c.last_tick = 10.5
    elif failure == 'jump':
        c.observations([track(x=2.4)], 10.1)
        now = 10.1
    elif failure == 'removed':
        c.observations([track(2)], 10.1)
        now = 10.1
    elif failure == 'visited':
        c.observations([track(visited=True)], 10.1)
        now = 10.1
    else:
        c.pose(1, 0, 0, 10.1)
        now = 10.1
    r = c.step(now)
    assert r['state'] == 'FAULT' and r['vx'] == r['wz'] == 0
    c.feedback_time = now
    c.observations([track()], now)
    assert c.step(now + 0.01)['state'] == 'FAULT'
    c.reset()
    assert not c.confirm(now)


def test_odom_mode_uses_pose_and_time_limit():
    c = ready(allow_odom=True)
    for i in range(1, 17):
        now = 10 + i * 0.1
        c.pose(0.01 * i, 0, 0, now)
        c.feedback_time = now
        r = c.step(now)
        if i == 7:
            assert r['state'] == 'APPROACH_ODOM'
        if r['state'] == 'FAULT':
            break
    assert r['state'] == 'FAULT'
    assert 'continuation limit' in r['reason']
    assert r['distance'] == pytest.approx(2 - 0.01 * i)


def test_odom_travel_limit_and_reacquisition():
    c = ready(allow_odom=True, odom_max_distance=0.02, odom_max_time=4)
    for i in range(1, 10):
        now = 10 + i * 0.1
        c.pose(0.01 * i, 0, 0, now)
        c.feedback_time = now
        r = c.step(now)
        if r['state'] == 'FAULT':
            break
    assert r['state'] == 'FAULT'
    assert c.odom_travel >= 0.02


def test_eta_uses_actual_closing_speed_not_command():
    c = ready()
    c.pose(0.02, 0, 0, 10.1)
    c.feedback_time = 10.1
    r = c.step(10.1)
    assert r['eta'] == pytest.approx(r['remaining'] / 0.2)
    assert r['actual_speed'] == pytest.approx(0.2)


@pytest.mark.parametrize('kwargs', [dict(max_linear_speed=0.31), dict(stop_distance=0.4),
                                    dict(kp_linear=float('nan')), dict(max_deceleration=0)])
def test_invalid_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        Settings(**kwargs)


def test_confirmation_refuses_transient_and_wrong_id():
    c = ApproachControl()
    c.pose(0, 0, 0, 10)
    c.feedback_time = 10
    transient = track()
    transient['lifetime'] = 0.8
    c.observations([transient], 10)
    assert not c.confirm(10, 1)
    c.observations([track()], 10)
    assert not c.confirm(10, 9)
    assert c.confirm(10, 1)


def test_cached_snapshot_cannot_extend_visibility():
    c = ApproachControl()
    c.pose(0, 0, 0, 10)
    c.feedback_time = 10
    c.observations([track(age=0.3)], 10)
    c.pose(0, 0, 0, 10.3)
    c.feedback_time = 10.3
    assert not c.confirm(10.3)
