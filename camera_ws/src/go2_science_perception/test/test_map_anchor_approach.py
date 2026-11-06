"""Focused camera-reference, spatial reacquisition and mixed-loop checks."""

import math

from go2_science_perception.approach_control import ApproachControl, Settings


def observation(x=1.4, identity=1):
    return dict(id=identity, position=[x, 0.0, 0.0], state='CONFIRMED',
                visited=False, age=0.0, lifetime=5.0)


def locked():
    c = ApproachControl(Settings(stop_reference='camera_origin', allow_odom=True,
                                 odom_max_time=25.0, odom_max_distance=2.0))
    for i in range(6):
        now = 10+i*.1
        c.camera_pose(.36, 0, now)
        c.pose(0, 0, 0, now)
        c.feedback_time = now
        c.observations([observation()], now)
    assert c.confirm(now, 1)
    return c, now


def test_manual_lock_start_and_camera_reference():
    c, now = locked()
    assert c.step(now)['vx'] == 0
    assert math.isclose(c.initial_distance, 1.04)
    assert c.start(now)
    assert c.step(now+.1)['state'] == 'APPROACH_VISUAL'
    assert c.anchor_initial == (1.4, 0)


def test_anchor_survives_track_removal_reacquires_spatially_and_rejects_jump():
    c, now = locked()
    c.observations([], now+.1)
    assert c.target == (1.4, 0) and c.state != 'FAULT'
    c.observations([observation(1.41, 99)], now+.2)
    assert 1.4 < c.target[0] < 1.41 and c.lock_id == 1
    anchor = c.target
    c.observations([observation(4.0)], now+.3)
    assert c.target == anchor and c.state != 'FAULT'


def test_mixed_loop_stops_on_camera_xy_with_no_visual_refresh():
    c, now = locked()
    c.start(now)
    c.step(now)
    x, speed, odom_seen = 0.0, 0.0, False
    for i in range(1, 260):
        t = now+i*.1
        x += speed*.1
        c.camera_pose(x+.36, 0, t)
        c.pose(x, 0, 0, t)
        c.feedback_time = t
        c.observations([], t)
        r = c.step(t)
        speed = r['vx']
        assert 0 <= speed <= .3
        odom_seen |= r['state'] == 'APPROACH_ODOM'
        if r['state'] in ('STOP', 'FAULT'):
            break
    assert odom_seen and r['state'] == 'STOP', r
    assert r['camera_distance_xy'] <= .1
    assert c.target[0]-c.robot[0] > .36  # body center is deliberately not the endpoint
    assert r['vx'] == r['wz'] == 0
    assert not c.start(t)
