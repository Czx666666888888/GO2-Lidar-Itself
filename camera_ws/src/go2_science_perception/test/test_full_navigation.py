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
