import json
import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped

from go2_science_perception.docking_pose_planner import TerrainFootprint, staging_candidates
from go2_science_perception.grasp_pose_geometry import DockSettings
from go2_science_perception.mission_manager import MissionManager
from go2_science_perception.motion_arbiter import MotionArbiter


def floor():
    terrain = TerrainFootprint(dict(length=.70, width=.40, hard_clearance=.08,
                                    parking_clearance=.15))
    terrain.cells = {(i, j): 1 for i in range(-20, 25) for j in range(-20, 21)}
    return terrain


def test_staging_keeps_body_and_final_corridor_in_known_space():
    terrain = floor()
    stages = staging_candidates((1.5, 0), (0, 0, 0), {'right': DockSettings()}, terrain)
    assert stages
    stage = stages[0]
    assert stage.pose[:2] != (1.5, 0)
    assert terrain.sweep_clear(stage.pose, stage.dock)
    center = terrain.cell((stage.pose[0] + stage.dock[0])/2,
                          (stage.pose[1] + stage.dock[1])/2)
    terrain.cells[center] = 2
    assert not terrain.sweep_clear(stage.pose, stage.dock)
    assert stage not in staging_candidates((1.5, 0), (0, 0, 0), {'right': DockSettings()}, terrain)
    del terrain.cells[terrain.cell(*stage.pose[:2])]
    assert not terrain.clear(stage.pose, terrain.parking_clearance)


def test_only_selected_source_with_new_stamp_can_publish():
    now = [1_000_000_000]
    sent = []
    def clock():
        return SimpleNamespace(nanoseconds=now[0],
                               to_msg=lambda: Time(sec=now[0]//10**9,
                                                              nanosec=now[0]%10**9))
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=clock),
                           create_publisher=lambda *args: SimpleNamespace(publish=sent.append),
                           create_subscription=lambda *args: None, create_timer=lambda *args: None)
    arbiter = MotionArbiter(node)
    arbiter.select('NAV')
    msg = TwistStamped()
    msg.header.stamp = Time(sec=1, nanosec=100)
    msg.twist.linear.x = .3
    now[0] += 150
    arbiter.receive('science', msg)
    assert arbiter.latest is None
    arbiter.receive('nav', msg)
    arbiter.enabled = True
    arbiter.tick()
    assert sent[-1].twist.linear.x == .3
    arbiter.select('APPROACH')
    arbiter.receive('science', msg)
    arbiter.enabled = True
    arbiter.tick()
    assert sent[-1].twist.linear.x == 0
    arbiter.select('REACQUIRE')
    msg.header.stamp = Time(sec=1, nanosec=200)
    now[0] += 100
    arbiter.receive('nav', msg)
    arbiter.receive('science', msg)
    assert arbiter.latest is None
    arbiter.receive('reacquire', msg)
    assert arbiter.latest is not None
    arbiter.select('IDLE')
    assert arbiter.latest is None


def test_staging_does_not_start_approach_with_blocked_last_segment():
    mission = MissionManager.__new__(MissionManager)
    mission.state = 'NAV_TO_TARGET'
    stage, = staging_candidates((1.5, 0), (0, 0, 0), {'right': DockSettings()},
                                floor())[:1]
    mission.stage = stage
    mission.pose = stage.pose
    mission.had_path = True
    mission.sides = {'right': DockSettings()}
    mission.anchor = mission.track = {'id': 1, 'position': (1.5, 0, 0), 'observed_ns': 1}
    mission.fresh = lambda *_: True
    mission.stage_clear = lambda _: False
    mission.arbiter = SimpleNamespace(enabled=True)
    reasons = []
    mission.replan = reasons.append
    mission.stage_step()
    assert reasons and not mission.arbiter.enabled


def test_return_grouping_preserves_a_recorded_corner():
    mission = MissionManager.__new__(MissionManager)
    mission.terrain = floor()
    mission.pose = (0., .60, -1.57)
    mission.trail = [(-.3, 0.), (0., 0.), (0., .30), (0., .58)]
    assert mission.return_segment(2) == 1


def test_far_marker_from_another_goal_does_not_admit_path():
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker

    mission = MissionManager.__new__(MissionManager)
    mission.state = 'PROBE'
    mission.goal = (1., 0., 0.)
    mission.goal_ns = 1_000_000_000
    mission.far_cost = None
    mission.forward_waypoint = lambda: None
    marker = Marker(ns='global_path', points=[Point(), Point(x=1.)])
    marker.header.frame_id = 'map'
    marker.header.stamp = Time(sec=1)
    mission.far_path(marker)
    assert mission.far_cost is None
    marker.header.stamp.nanosec = 200
    marker.points[-1].x = 2.
    mission.far_path(marker)
    assert mission.far_cost is None
    marker.points[-1].x = 1.
    mission.far_path(marker)
    assert mission.far_cost == 1.


def test_full_nav_does_not_use_point_lio_path_topic():
    from pathlib import Path
    from xml.etree import ElementTree

    from go2_science_perception.mission_manager import LOCAL_PATH_TOPIC, NAV_TOPICS

    config = Path(__file__).resolve().parents[1]/'config/full_local_planner.xml'
    launch = ElementTree.parse(config)
    for name in ('localPlanner', 'pathFollower'):
        remap = launch.find(f"./node[@name='{name}']/remap[@from='/path']")
        assert remap is not None and remap.attrib['to'] == LOCAL_PATH_TOPIC
    assert '/path' not in NAV_TOPICS


def test_no_feasible_staging_defers_only_the_target_and_resumes_explore():
    mission = MissionManager.__new__(MissionManager)
    mission.state = 'PROBE'
    mission.pose = (0., 0., 0.)
    mission.terrain = floor()
    mission.anchor = mission.track = {'id': 1, 'position': (1.5, 0., 0.)}
    mission.deferred_targets = {}
    mission.stage = SimpleNamespace()
    mission.pending = mission.ranked = []
    mission.goal = (1., 0., 0.)
    mission.goal_ns = 1
    events = []
    mission.run = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)))
    mission.arbiter = SimpleNamespace(flush=lambda: events.append(('zero', {})))
    mission.change = lambda state, mode: setattr(mission, 'state', state)

    mission.next_probe()

    assert mission.state == 'EXPLORE'
    assert mission.anchor is None and mission.goal is None and mission.goal_ns == 0
    assert events[0][1]['status'] == 'UNREACHABLE'
    assert events[-1][0] == 'zero'
    assert not mission.can_retry_target(1)
    assert mission.can_retry_target(2)
    mission.pose = (.36, 0., 0.)
    assert mission.can_retry_target(1)


def test_parking_rejection_reports_each_kind_of_blocking_cell():
    from collections import Counter

    terrain = floor()
    target, robot = (1.5, 0.), (0., 0., 0.)
    sides = {'right': DockSettings()}
    stage = staging_candidates(target, robot, sides, terrain)[0]
    i, j = terrain.cell(*stage.pose[:2])
    terrain.cells[(i, j)] = 2
    terrain.cells[(i, j + 1)] = 0
    del terrain.cells[(i, j - 1)]
    counts = Counter()
    records = []
    accepted = staging_candidates(target, robot, sides, terrain, rejected=counts,
                                  on_reject=lambda **row: records.append(row))
    assert accepted == staging_candidates(target, robot, sides, terrain)
    row = next(r for r in records if r['stage_pose_map_xy_yaw'] == stage.pose)
    assert row['reason'] == 'parking_clearance'
    assert row['side'] == 'right'
    assert row['clearance_m'] == .15
    assert set(row['blockers']) == {'occupied', 'terrain_unknown', 'unobserved'}
    assert terrain.cell(*row['blockers']['occupied']['cell_map_xy_m']) == (i, j)
    assert counts['parking_clearance'] == len(records)


def test_corridor_reports_first_blocked_pose_and_map_cell():
    terrain = floor()
    start, end = (0., 0., 0.), (1., 0., 0.)
    terrain.cells[(10, 0)] = 2
    evidence = {}
    assert not terrain.sweep_clear(start, end, evidence=evidence)
    assert 0 < evidence['check_pose_map_xy_yaw'][0] < 1
    assert evidence['clearance_m'] == .08
    assert evidence['blockers']['occupied']['cell_map_xy_m'] == (1., 0.)


def test_probe_timeout_reports_local_path_failure_for_this_target():
    mission = MissionManager.__new__(MissionManager)
    mission.state = 'PROBE'
    mission.track = None
    mission.anchor = {'id': 7}
    mission.stage = SimpleNamespace(pose=(1., 2., .5), side='right')
    mission.deadline = 0.
    mission.far_cost = 1.
    mission.local_ns = 0
    mission.far_ns = mission.way_ns = 1
    mission.local_path_issue = 'local_path_map_tf_unavailable'
    mission.fresh = lambda ns, _: bool(ns)
    events = []
    mission.run = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)))
    mission.next_probe = lambda: None

    mission.stage_step()

    name, row = events[0]
    assert name == 'STAGING_PROBE_TIMEOUT'
    assert row['target_id'] == 7
    assert row['reason'] == 'local_path_map_tf_unavailable'


def test_grasp_clearance_admits_candidate_without_relaxing_return_sweep():
    terrain = floor()
    terrain.parking_clearance = terrain.grasp_clearance = .05
    target, robot = (1.5, -.25), (0., -.25, 0.)
    sides = {'right': DockSettings()}
    candidate = next(p for p in staging_candidates(target, robot, sides, terrain) if p.pose[2] == 0.)
    terrain.cells[(9, 4)] = 2

    assert candidate in staging_candidates(target, robot, sides, terrain)
    assert not terrain.sweep_clear(candidate.pose, candidate.dock)
    assert terrain.sweep_clear(candidate.pose, candidate.dock, clearance=terrain.grasp_clearance)
    terrain.cells[terrain.cell(*candidate.pose[:2])] = 2
    assert candidate not in staging_candidates(target, robot, sides, terrain)


def test_precise_cell_check_keeps_only_real_clearance_intersections():
    terrain = floor()
    terrain.cells[(0, 4)] = 2
    pose = (0., .09, 0.)
    assert not terrain.clear(pose, .05, exact_cells=False)
    assert terrain.clear(pose, .05)
    assert not terrain.clear((0., .11, 0.), .05)
    assert terrain.clear(pose, .05, evidence={})


def test_rotated_cell_overlap_checks_corners_and_crossing_edges():
    import math

    terrain = TerrainFootprint(dict(length=.70, width=.04, hard_clearance=.08,
                                    parking_clearance=.05))
    pose = (0., 0., math.pi/4)
    assert terrain.cell_overlaps(pose, .1, .1, 0.)
    assert not terrain.cell_overlaps(pose, .1, -.1, .05)

    terrain = floor()
    pose = (0., 0., math.pi/4)
    assert terrain.cell_overlaps(pose, .48, .11, .05)
    assert not terrain.cell_overlaps(pose, .49, .11, .05)


def test_full_prefers_valid_left_and_probes_right_only_after_left_fails():
    left = SimpleNamespace(pose=(1., 2., .5), side='left')
    right = SimpleNamespace(pose=(.5, .5, .5), side='right')
    mission = MissionManager.__new__(MissionManager)
    mission.pose = (0., 0., 0., 0.)
    mission.anchor = {'id': 7}
    events = []
    mission.run = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)))
    mission.change = lambda state, _='IDLE': setattr(mission, 'state', state)
    mission.send_goal = lambda point: None
    mission.publish_stage = lambda: None

    mission.probe([right, left])
    assert mission.stage is left
    mission.ranked.append((3., left))
    mission.next_probe()
    assert mission.stage is left
    assert mission.state == 'NAV_TO_TARGET'

    mission.probe([right, left])
    assert mission.stage is left
    mission.next_probe()
    assert mission.stage is right
    assert any(event == 'STAGING_SIDE_FALLBACK' for event, _ in events)


def test_direct_grasp_uses_nearest_supported_wall_and_limits():
    import numpy as np
    from go2_science_perception.wall_distance import wall_segments

    samples = np.arange(0., 1.01, .025)
    corner = [(v, 0.) for v in samples] + [(0., v) for v in samples]
    distance, ends = wall_segments((.20, .10), corner)[0]
    assert abs(distance-.10) < .01
    assert max(abs(p[1]) for p in ends) < .01
    distance, _ = wall_segments((.01, .24), corner)[0]
    assert abs(distance-.01) < .007
    walls = wall_segments((.005, .24), corner)
    assert len(walls) == 2
    assert walls[0][0] < .01
    assert abs(walls[1][0]-.24) < .01
    assert max(abs(p[1]) for p in walls[1][1]) < .01
    distances = [distance for distance, _ in wall_segments((.426, .24), corner)]
    assert len(distances) == 2
    assert abs(distances[0]-.24) < .01 and abs(distances[1]-.426) < .01
    assert not wall_segments((0., .25), [(x, 0.) for x in samples[:10]])
    assert not wall_segments((0., .2), [(-1., 0.), (-.9, 0.), (0., 0.), (.1, 0.),
                                        (.8, 0.), (.9, 0.)])


def test_wall_qualified_target_claims_navigation_without_a_staging_candidate(monkeypatch):
    from go2_science_perception import mission_manager

    mission = MissionManager.__new__(MissionManager)
    mission.stage = SimpleNamespace(side='right')
    mission.args = SimpleNamespace(grasp_side='auto')
    mission.pose = (0., 0., 0., 0.)
    mission.direct_blockers = {}
    mission.cloud_ns, mission.last_direct_scan = 2, 1
    mission.direct_range = 1.8
    mission.wall_points = []
    mission.eligible_tracks = [{'id': 1, 'position': (.1, .1, 0.), 'observed_ns': 1},
                               {'id': 2, 'position': (.2, .1, 0.), 'observed_ns': 1}]
    mission.fresh = mission.can_retry_target = lambda *args: True
    mission.camera_range_for = lambda _: 1.
    mission.change = lambda state: setattr(mission, 'state', state)
    events = []
    mission.run = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)))
    mission.hold_navigation = lambda: events.append(('hold', {}))
    # The first target has no usable wall. For the second target, the closest
    # wall is too close, and the wall on the opposite side qualifies.
    too_close = (.005, ((0., .105), (1., .105)))
    in_range = (.35, ((0., -.25), (1., -.25)))
    far = (.426, ((.626, 0.), (.626, 1.)))
    monkeypatch.setattr(mission_manager, 'wall_segments',
                        lambda target, _: [too_close, far] if target[0] < .2
                        else [too_close, in_range, far])

    assert mission.prioritize_grasp()
    assert mission.state == 'TARGET_STOP'
    assert mission.stage is None
    assert mission.anchor['id'] == 2
    assert events[0][1]['reason'] == 'outside_wall_range'
    assert [wall['eligible'] for wall in events[0][1]['wall_candidates']] == [False, False]
    assert events[1][0] == 'hold'
    assert events[2][1]['distance_m'] == .35
    assert [wall['eligible'] for wall in events[2][1]['wall_candidates']] == [False, True, False]
    assert events[2][1]['side'] == mission.side_hint == 'right'
    assert events[2][1]['corner_turn_deg'] is None
    assert mission.dock_heading is None


def test_direct_handoff_waits_for_measured_stop_and_keeps_selected_identity():
    mission = MissionManager.__new__(MissionManager)
    mission.state = 'TARGET_STOP'
    mission.deadline = float('inf')
    mission.stage = None
    mission.anchor = mission.track = {'id': 1, 'position': (.2, .1, 0.), 'observed_ns': 1}
    mission.fresh = lambda *args: True
    stopped = [False]
    mission.stopped_for_handoff = lambda: stopped[0]
    mission.begin_stop_confirm = lambda _: setattr(mission, 'state', 'STOP_CONFIRM')
    mission.stage_step()
    assert mission.state == 'TARGET_STOP'
    stopped[0] = True
    mission.stage_step()
    assert mission.state == 'STOP_CONFIRM'
    assert mission.anchor['id'] == 1


def test_direct_grasp_skips_footprint_sweep_but_checks_telemetry():
    mission = MissionManager.__new__(MissionManager)
    mission.arbiter = SimpleNamespace(mode='DOCK')
    mission.pose = (0., 0., 0.)
    mission.pose_ns = mission.cloud_ns = 1
    mission.fresh = lambda ns, _: bool(ns)
    mission.stage = None
    mission.motion_blocked_at = None
    mission.terrain = floor()
    mission.terrain.cells[(4, 0)] = 2
    cmd = TwistStamped()
    cmd.twist.linear.x = .30

    assert mission.command_clear(cmd)
    mission.cloud_ns = 0
    assert not mission.command_clear(cmd)
    mission.cloud_ns = 1
    mission.stage = SimpleNamespace(side='right')
    assert not mission.command_clear(cmd)
    assert mission.motion_blocked_at is not None


def test_wall_side_hint_overrides_shortest_dock_and_keeps_body_away_from_wall():
    from go2_science_perception.approach_control import ApproachControl, Settings
    from go2_science_perception.grasp_docking import GraspDocking
    from go2_science_perception.grasp_pose_geometry import shared_side_settings
    from go2_science_perception.wall_distance import wall_grasp_side

    docking = GraspDocking(DockSettings())
    options = shared_side_settings(docking.p)
    core = ApproachControl(Settings(), docking, options)
    core.robot = (0., 0., 0.)
    target = dict(id=2, position=(1., .2, 0.))
    for wall_y, expected, direction in ((.4, 'left', -1), (0., 'right', 1)):
        segment = ((.5, wall_y), (1.5, wall_y))
        side = wall_grasp_side(target['position'][:2], segment, 0.)
        assert side == expected
        assert wall_grasp_side(target['position'][:2], segment, math.pi) != side
        core._search_candidates([dict(target, grasp_side_hint=side)])
        assert core.grasp_side == expected
        assert len(core.side_candidates) == 1
        assert direction * (core.side_candidates[0]['goal'][1] - target['position'][1]) > 0
    assert wall_grasp_side((1., .2), ((1.2, 0.), (1.2, .4)), 0.) is None


def test_memory_revisit_uses_an_unvisited_map_waypoint_before_confirming():
    def target(identity, x, state='STALE'):
        return dict(id=identity, position=(x, 0., 0.), state=state, visited=False,
                    observed_ns=1, memory_confirmed=True, memory_spread_m=.01)

    def snapshot(*targets):
        return SimpleNamespace(data=json.dumps(dict(frame='map', tracks=targets)))

    mission = MissionManager.__new__(MissionManager)
    mission.pose = (0., 0., 0., 0.)
    mission.completed = [(1., 0.)]
    mission.anchor = None
    mission.association_radius = .12
    mission.memory_spread_limit, mission.memory_range = .08, 5.
    mission.fresh = mission.can_retry_target = lambda *args: True
    mission.memory_visit = None
    mission.memory_visited = set()
    mission.terrain = floor()
    mission.send_goal = lambda point: setattr(mission, 'goal', point)
    mission.run = SimpleNamespace(event=lambda *_args, **_kwargs: None)
    published = []
    mission.tracks_pub = SimpleNamespace(publish=published.append)
    mission.tracks(snapshot(target(1, 1.), target(2, 2.), target(3, 3.)))
    assert not mission.eligible_tracks
    assert [t['id'] for t in mission.memory_tracks] == [2, 3]
    assert mission.revisit_memory()
    assert mission.goal == (1.15, 0., 0.)
    assert mission.memory_visit[0] == 2
    assert mission.anchor is None
    assert not published

    mission.tracks(snapshot(target(2, 2., 'CONFIRMED'), target(3, 3.)))
    assert [t['id'] for t in mission.eligible_tracks] == [2]
    mission.anchor = mission.eligible_tracks[0]
    mission.side_hint = 'left'
    mission.dock_heading = math.radians(-40.)
    mission.tracks(snapshot(target(2, 2., 'CONFIRMED'), target(3, 3., 'CONFIRMED')))
    forwarded = json.loads(published[-1].data)['tracks']
    assert [(t['id'], t['grasp_side_hint']) for t in forwarded] == [(2, 'left')]
    assert forwarded[0]['dock_heading_hint'] == mission.dock_heading


def test_navigation_recovers_only_after_stable_fresh_localization():
    mission = MissionManager.__new__(MissionManager)
    issue = {'reason': 'map_vehicle_tf_at_odom_stamp_unavailable'}
    current = [issue]
    cached_allowed = []
    events = []
    flushes = []
    mission.state = 'EXPLORE'
    mission.localization_paused_at = mission.localization_recovered_at = None
    mission.run = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)))
    mission.arbiter = SimpleNamespace(flush=lambda: flushes.append(True))
    mission.last_progress = 18.
    mission.pose_ns = 100

    def check(*, allow_cached_tf):
        cached_allowed.append(allow_cached_tf)
        return current[0]

    mission.check_localization = check
    assert not mission.localization_ready(20.)
    assert not mission.localization_ready(20.1)
    assert len(flushes) == 1
    current[0] = None
    assert not mission.localization_ready(21.)
    assert not mission.localization_ready(21.4)  # The same pose does not prove recovery.
    current[0] = issue
    assert not mission.localization_ready(21.5)
    current[0] = None
    mission.pose_ns += 1
    assert not mission.localization_ready(21.6)
    mission.pose_ns += 1
    assert not mission.localization_ready(21.8)
    assert mission.localization_ready(22.)
    assert cached_allowed == [True] + [False]*7
    assert len(flushes) == 2
    assert [e[0] for e in events] == ['LOCALIZATION_PAUSED', 'LOCALIZATION_RECOVERED']
    assert mission.localization_paused_at is None and mission.last_progress == 20.


@pytest.mark.parametrize('state, fail_at, error', [
    ('RETURN', 25., 'did not recover'),
    ('STOP_CONFIRM', 20., 'unavailable'),
])
def test_localization_timeout_keeps_specific_cause_and_near_handoff_policy(state, fail_at, error):
    mission = MissionManager.__new__(MissionManager)
    mission.state = state
    mission.startup_deadline = 15.
    mission.localization_paused_at = mission.localization_recovered_at = None
    issue = {'reason': 'body_odometry_timestamp_invalid', 'odom_age_sec': .65}
    mission.check_localization = lambda **kw: issue
    events = []
    mission.run = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)))
    mission.arbiter = SimpleNamespace(enabled=True, flush=lambda: None)
    mission.change = lambda state: setattr(mission, 'state', state)
    if state == 'RETURN':
        assert not mission.localization_ready(20.)
        assert not mission.localization_ready(24.9)
    with pytest.raises(RuntimeError, match=error):
        mission.localization_ready(fail_at)
    assert mission.state == 'UNREACHABLE'
    assert events[-1][1]['localization'] == issue


def test_localization_reports_timestamp_tf_and_pose_mismatch():
    from tf2_ros import TransformException

    mission = MissionManager.__new__(MissionManager)
    mission.pose = (1., 0., 0., 0.)
    mission.now_ns = lambda: 2_000_000_000
    mission.pose_ns = 1_900_000_000
    mission.verified_pose_ns = mission.pose_ns
    mission.rejected_odom = None

    def transform(_target, source, _stamp):
        if source == 'vehicle':
            raise TransformException('requested stamp outside TF buffer')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0., y=0., z=0.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))

    mission.tf = SimpleNamespace(lookup_transform=transform)
    assert mission.check_localization(allow_cached_tf=True) is None
    issue = mission.check_localization(allow_cached_tf=False)
    assert issue['reason'] == 'map_vehicle_tf_at_odom_stamp_unavailable'
    assert issue['odom_stamp_ns'] == mission.pose_ns
    mission.tf.lookup_transform = lambda target, source, stamp: transform(target, 'map', stamp)
    issue = mission.check_localization(allow_cached_tf=False)
    assert issue['reason'] == 'body_odometry_tf_mismatch'
    assert issue['xy_error_m'] == 1.
    mission.pose_ns = 2_010_000_000
    issue = mission.check_localization(allow_cached_tf=True)
    assert issue['reason'] == 'body_odometry_timestamp_invalid'
    assert issue['odom_age_sec'] == -.01


@pytest.mark.parametrize('opening_y, expected_deg', [(1., 40.), (-1., -40.)])
def test_direct_grasp_turns_once_at_observed_corner_without_changing_side(opening_y, expected_deg):
    import numpy as np

    mission = MissionManager.__new__(MissionManager)
    mission.args = SimpleNamespace(grasp_side='auto')
    mission.pose = (0., 0., 0., 0.)
    mission.direct_blockers = {}
    mission.cloud_ns, mission.last_direct_scan = 2, 1
    mission.direct_range = 1.8
    samples = np.arange(0., 1.01, .025)
    mission.wall_points = [(0., opening_y*v) for v in samples] + [(v, 0.) for v in samples]
    mission.eligible_tracks = [{'id': 2, 'position': (.28, opening_y*.20, 0.), 'observed_ns': 1}]
    mission.fresh = mission.can_retry_target = lambda *args: True
    mission.camera_range_for = lambda _: 1.
    mission.change = lambda state: setattr(mission, 'state', state)
    mission.hold_navigation = lambda: None
    events = []
    mission.run = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)))

    assert mission.prioritize_grasp()
    assert mission.state == 'TARGET_STOP'
    assert mission.side_hint == ('right' if opening_y > 0 else 'left')
    assert events[0][1]['corner_turn_deg'] == expected_deg
    assert events[0][1]['corner_map'] == pytest.approx((0., 0.), abs=.02)
    assert mission.dock_heading == pytest.approx(math.radians(expected_deg))
    assert not mission.prioritize_grasp()


def test_corner_heading_rejects_unconnected_walls_and_target_outside_opening():
    from go2_science_perception.wall_distance import corner_grasp_heading

    wall = ((0., 0.), (1., 0.))
    assert corner_grasp_heading((.2, .2), wall, [((0., .5), (0., 1.))], 0.) is None
    assert corner_grasp_heading((.2, -.1), wall, [((0., 0.), (0., 1.))], 0.) is None


def test_corner_heading_reaches_original_docking_after_stationary_target_lock():
    from go2_science_perception.approach_control import ApproachControl, Settings
    from go2_science_perception.grasp_docking import GraspDocking
    from go2_science_perception.grasp_pose_geometry import shared_side_settings

    docking = GraspDocking(DockSettings())
    settings = Settings(search_enabled=True, stop_reference='camera_origin',
                        anchor_window_sec=1., stop_confirm_fast_enabled=True)
    core = ApproachControl(settings, docking, shared_side_settings(docking.p))
    heading = math.radians(-40.)

    def observe(now, rate):
        core.pose(0., 0., 0., now)
        core.camera_pose(.36, 0., now)
        core.feedback_time, core.feedback_speed, core.feedback_yaw_rate = now, 0., rate
        core.observations([dict(id=2, state='CONFIRMED', age=0., lifetime=1.,
                                visited=False, observed_ns=round(now*1e9), camera_range_m=1.,
                                position=(1., .1, 0.), latest_observation=(1., .1, 0.),
                                grasp_side_hint='left', dock_heading_hint=heading)], now)

    observe(10., .15)
    assert core.start(10.)
    for i in (1, 2):
        now = 10.+i*.1
        observe(now, .15)
        report = core.step(now)
    assert report['state'] == 'STOP_CONFIRM'
    assert report['stop_candidate']['dock_heading_rad'] == heading
    assert docking.heading is None
    for i in range(3, 20):
        now = 10.+i*.1
        observe(now, 0.)
        report = core.step(now)
        if report['state'] == 'LOCK_TARGET':
            break
    assert report['state'] == 'LOCK_TARGET'
    assert core.lock_id == 2 and core.grasp_side == 'left'
    assert docking.heading == heading
    observe(now+.1, 0.)
    report = core.step(now+.1)
    assert report['grasp_docking']['desired_yaw'] == pytest.approx(heading)
    assert report['wz'] < 0.
    assert not report['grasp_docking_complete']


def test_red_block_reacquires_from_recorded_near_pose_without_switching_target(monkeypatch):
    from go2_science_perception import mission_manager

    now = [10.]
    monkeypatch.setattr(mission_manager.time, 'monotonic', lambda: now[0])
    mission = MissionManager.__new__(MissionManager)
    mission.state = 'TARGET_STOP'
    mission.deadline = 15.
    mission.anchor = dict(id=3, position=(2.34709, .33934, 0.), observed_ns=8_900_000_000)
    mission.track = mission.anchor
    mission.pose = (1.82293, -.12260, -.21191, 0.)
    mission.side_hint = 'left'
    mission.now_ns = lambda: int(now[0]*1e9)
    stationary = [False]
    mission.stopped_for_handoff = lambda: stationary[0]
    mission.direct_range = 1.8
    mission.camera_range_for = lambda _: .70
    mission.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: Time(sec=int(now[0])))))
    commands, events = [], []
    mission.arbiter = SimpleNamespace(enabled=False, receive=lambda *args: commands.append(args))
    mission.run = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)))
    mission.change = lambda state, mode='IDLE': setattr(mission, 'state', state)
    mission.begin_stop_confirm = lambda _: mission.change('STOP_CONFIRM')

    mission.stage_step()
    assert mission.state == 'TARGET_STOP'
    stationary[0] = True
    now[0] = 10.2
    mission.stage_step()
    assert mission.state == 'TARGET_REACQUIRE'
    assert events[-1][1]['target_id'] == 3
    assert events[-1][1]['bearing_deg'] == pytest.approx(53.5, abs=.1)
    mission.stage_step()
    source, command = commands[-1]
    assert source == 'reacquire'
    assert command.twist.linear.x == command.twist.linear.y == 0.
    assert command.twist.angular.z == .25

    distance, error = mission.target_bearing()
    assert distance == pytest.approx(.70, abs=.01)
    mission.pose = (*mission.pose[:2], mission.pose[2]+error, 0.)
    now[0] = 14.5
    mission.track = dict(mission.anchor, id=7, observed_ns=14_400_000_000)
    mission.stage_step()
    assert mission.state == 'TARGET_REACQUIRE'
    assert not mission.arbiter.enabled
    mission.track = dict(mission.anchor, observed_ns=14_400_000_000)
    stationary[0] = False
    mission.stage_step()
    assert mission.state == 'TARGET_REACQUIRE'
    stationary[0] = True
    mission.stage_step()
    assert mission.state == 'STOP_CONFIRM'
    assert mission.anchor['id'] == 3 and mission.side_hint == 'left'


def test_reacquisition_turn_uses_body_sweep_and_defers_when_blocked(monkeypatch):
    from go2_science_perception import mission_manager

    monkeypatch.setattr(mission_manager.time, 'monotonic', lambda: 10.)
    mission = MissionManager.__new__(MissionManager)
    mission.arbiter = SimpleNamespace(mode='REACQUIRE')
    mission.pose = (0., 0., 0., 0.)
    mission.pose_ns = mission.cloud_ns = 1
    mission.fresh = lambda ns, _: bool(ns)
    mission.stage = None
    mission.terrain = floor()
    mission.motion_blocked_at = None
    mission.deadline = 18.
    cmd = TwistStamped()
    cmd.twist.angular.z = .25
    assert mission.command_clear(cmd)
    mission.terrain.cells[(4, 0)] = 2
    assert not mission.command_clear(cmd)

    deferred = []
    mission.defer_target = deferred.append
    mission.target_reacquire_step(11., False)
    assert 'prevents turning' in deferred[-1]
    mission.target_reacquire_step(18., False)
    assert 'not confirmed' in deferred[-1]
