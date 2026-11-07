"""Focused SEARCH ownership, raw MAP scatter and route-end checks."""

from go2_science_perception.approach_control import ApproachControl, Settings


def search():
    c = ApproachControl(Settings(search_enabled=True, stop_reference='camera_origin',
                                 anchor_window_sec=1.0, anchor_max_spread=.10,
                                 allow_odom=True))
    c.pose(0, 0, 0, 10)
    c.camera_pose(.36, 0, 10)
    c.feedback_time = 10
    assert c.start(10)
    return c


def sample(c, t, x, position=0.0):
    c.pose(position, 0, 0, t)
    c.camera_pose(position+.36, 0, t)
    c.feedback_time = t
    c.observations([dict(id=1, state='CONFIRMED', visited=False, age=0,
                         lifetime=.1, position=[1.4, 0], latest_observation=[x, 0])], t)
    return c.step(t)


def test_moving_search_auto_handoff_and_raw_scatter():
    c = search()
    for i in range(1, 7):
        r = sample(c, 10+i*.1, 1.4+(.08 if i % 2 else -.08), i*.01)
        assert r['state'] == 'SEARCH'  # smoothed track is stable, raw points are not
        assert 0 < r['vx'] <= .20
    for i in range(7, 19):
        r = sample(c, 10+i*.1, 1.4, i*.01)
        if r.get('transition'):
            break
    assert r['transition'] == 'SEARCH->APPROACH'
    assert r['vx'] == r['wz'] == 0
    assert c.confirmation_spread < .1 and c.confirmation_count >= 5
    assert c.step(10+i*.1+.01)['state'] == 'APPROACH_VISUAL'


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
