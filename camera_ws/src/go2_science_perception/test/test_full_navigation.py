from types import SimpleNamespace

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


def test_staging_does_not_start_approach_with_blocked_last_segment():
    mission = MissionManager.__new__(MissionManager)
    mission.state = 'NAV_TO_TARGET'
    stage, = staging_candidates((1.5, 0), (0, 0, 0), {'right': DockSettings()},
                                floor())[:1]
    mission.stage = stage
    mission.pose = stage.pose
    mission.had_path = True
    mission.sides = {'right': DockSettings()}
    mission.anchor = mission.track = {'position': (1.5, 0, 0), 'observed_ns': 1}
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
