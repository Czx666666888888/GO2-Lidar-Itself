"""Offline regressions on the actual node methods, without creating DDS participants."""
from pathlib import Path
import math
import sys
import time
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
from nbv_explore_node import ExploreNode as Explore
from safety_gate import NavigationGate as Gate
from builtin_interfaces.msg import Time

class NavigationRegressionTests(unittest.TestCase):
    def explorer(self):
        n=object.__new__(Explore);n.pos=(3.,0.);n.home=(0.,0.);n.mode='RETURN';n.return_goal=None;n.path_hist=[]
        n.return_arrive_dis=.15;n.return_stuck_sec=4.;n.return_t=time.monotonic();n.return_best=99.;n.odom_frame='camera_init';n.outbound_reached=True
        n.yaw = 0.
        n.grid = {(x, y): 1 for x in range(-32, 33) for y in range(-16, 17)}
        n.sent=[];n.way_pub=NS(publish=n.sent.append)
        n.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time()));n.get_logger=lambda:NS(info=lambda _:None,warn=lambda _:None)
        return n
    def test_empty_history_far_from_home_never_means_done(self):
        n=self.explorer();n._return_step()
        self.assertEqual(n.mode,'RETURN');self.assertEqual(n.return_goal,n.home)
        self.assertEqual(n.sent[-1].header.frame_id,'camera_init')
    def test_stuck_return_retains_goal_instead_of_skipping(self):
        n=self.explorer();n.return_goal=(1.,0.);n.return_best=2.;n.return_t=time.monotonic()-10
        n._return_step();self.assertEqual(n.mode,'RETURN_BLOCKED');self.assertEqual(n.return_goal,(1.,0.))
    def test_actual_home_position_completes_return(self):
        n=self.explorer();n.pos=(.1,0.);n._return_step();self.assertEqual(n.mode,'DONE')
    def test_straight_breadcrumbs_are_not_deleted_as_loops(self):
        n=self.explorer();n.last_rec=None;n.path_hist=[];n.path_record_step=.3;n.loop_close_radius=.5;n.pos=(0.,0.);n._record_path()
        for i in range(1,8):n.pos=(i*.31,0);n._record_path()
        self.assertEqual(len(n.path_hist),8);self.assertEqual(n.path_hist[0],n.home)
    def corridor(self):
        n = self.explorer()
        n.pos = n.home
        n.home_yaw = 0.
        n.grid = {(x, y): 1 for x in range(-30, 31) for y in range(-15, 16)}
        return n

    def test_forward_goal_is_deterministic_and_uses_initial_heading(self):
        n = self.corridor()
        n.yaw = math.pi
        for _ in range(2):
            self.assertEqual(n._select_goal(), (2., 0.))
        n.home_yaw = math.pi
        self.assertAlmostEqual(n._select_goal()[0], -2.)

    def test_clear_space_behind_wall_is_not_a_goal(self):
        n = self.corridor()
        for y in range(-15, 16):
            n.grid[4, y] = 2
        self.assertIsNone(n._select_goal())

    def test_no_goal_across_unknown_strip(self):
        n = self.corridor()
        for y in range(-15, 16):
            del n.grid[4, y]
        goal = n._select_goal()
        if goal is not None:
            self.assertLess(goal[0], .4)

    def test_front_obstacle_leaves_a_forward_side_route(self):
        n = self.corridor()
        n.grid[10, 0] = 2
        x, y = n._select_goal()
        self.assertGreater(x, 1.)
        self.assertGreater(abs(y), .4)

    def test_return_lookahead_keeps_corner_and_path_order(self):
        n = self.corridor()
        n.path_hist = [(0.6, 0.3), (0.6, 0.), (0.3, 0.)]
        self.assertEqual(n._next_return_goal(), (0.6, 0.))
        self.assertEqual(n.path_hist, [(0.6, 0.3)])

    def test_new_obstacle_prevents_merging_return_points(self):
        n = self.corridor()
        n.path_hist = [(0.7, 0.), (0.3, 0.)]
        n.grid[7, 0] = 2
        self.assertEqual(n._next_return_goal(), (0.3, 0.))
        self.assertEqual(n.path_hist, [(0.7, 0.)])

    def test_return_turn_is_stable_across_pi_and_threshold(self):
        n = self.corridor()
        n.return_goal = (-1., 0.)
        n.turn_direction = 0.
        for yaw in (1., -1., 139., 142.):
            n.yaw = math.radians(yaw)
            self.assertEqual(n._turn_to_return_goal(), 1.)
        n.yaw = math.radians(160.)
        self.assertEqual(n._turn_to_return_goal(), 0.)

    def test_outbound_keeps_goal_then_returns_on_stall(self):
        n = self.corridor()
        n.current_goal = (2., 0.)
        n.start_time = n.startup_delay = n.progress_time = 0.
        n.best_remaining, n.next_plan = 2., 20.
        n.no_progress_timeout, n.return_after = 8., 30.
        n.arrive_dis = .25
        n._record_path = lambda: None
        reasons = []
        n._begin_return = reasons.append
        n._outbound_step(7.)
        self.assertEqual(n.current_goal, (2., 0.))
        self.assertFalse(reasons)
        n._outbound_step(8.)
        self.assertEqual(len(reasons), 1)

    def test_simulated_outbound_and_return_complete_at_home(self):
        n = self.corridor()
        n.mode, n.outbound_reached = 'EXPLORE', False
        n.current_goal = n.start_time = n.last_rec = n.progress_time = None
        n.startup_delay = n.next_plan = 0.
        n.footprint_radius, n.path_record_step, n.arrive_dis = .4, .3, .25
        n.return_trim_dis, n.return_after, n.no_progress_timeout = .45, 30., 8.
        n.goal_pub = n.status_pub = NS(publish=lambda _: None)
        n._outbound_step(0.)
        self.assertEqual(n.current_goal, (2., 0.))
        for i, x in enumerate((.32, .65, 1., 1.35, 1.79), start=1):
            n.pos = (x, 0.)
            n._outbound_step(float(i))
        self.assertEqual(n.mode, 'RETURN')
        for _ in range(10):
            n.pos = n.return_goal
            n._return_step()
            if n.mode == 'DONE':
                break
        self.assertEqual(n.mode, 'DONE')
        self.assertLessEqual(math.dist(n.pos, n.home), n.return_arrive_dis)

    def gate(self):
        n=object.__new__(Gate);n.state=1;n.last_cmd_time=time.time()-.2;n.last_cmd=(0.,0.,0.)
        n.max_speed=.3;n.min_speed=.3;n.max_yaw_rate=.7;n.max_accel=10.;n.phase='EXPLORE';n.rotate=0.;n.front_blocked=False
        n.body_gap = math.inf
        n.sent=[];n._send=lambda *cmd:n.sent.append(cmd);return n
    def cmd(self,vx=0,vy=0,yaw=0):return NS(twist=NS(linear=NS(x=vx,y=vy),angular=NS(z=yaw)))
    def test_early_return_never_claims_roundtrip(self):
        n=self.explorer();n.outbound_reached=False;n.pos=(.05,0.)
        n._return_step()
        self.assertEqual(n.mode,'RETURNED_EARLY')

    def test_late_far_waypoint_during_return_is_ignored(self):
        n=self.explorer()
        n.cb_waypoint(NS(point=NS(x=20.,y=0.)))
        self.assertFalse(n.sent)

    def test_minimum_speed_never_exceeds_configured_maximum(self):
        n=self.gate();n.max_speed=.1;n.min_speed=.2;n.cb(self.cmd(.01));self.assertLessEqual(abs(n.sent[-1][0]),.1)
    def test_front_stop_is_immediate_even_with_previous_forward_velocity(self):
        n=self.gate();n.front_blocked=True;n.max_accel=.5;n.last_cmd=(.3,0,0);n.cb(self.cmd(.3));self.assertEqual(n.sent[-1][0],0)

    def test_rectangle_distance_uses_the_surface_and_rounded_corners(self):
        from body_footprint import edge_distance
        for x, y, expected in ((0., 0., 0.), (-.43, 0., .08), (0., .35, .15),
                               (.38, -.24, .05)):
            self.assertAlmostEqual(edge_distance(x, y), expected)

    def test_candidate_parking_checks_heading_and_unknown_cells(self):
        n = self.corridor()
        n.grid[4, 0] = 2
        self.assertFalse(n._parking_space(0., 0., 0.))
        n.grid[4, 0] = 1
        n.grid[5, 0] = 0
        self.assertFalse(n._parking_space(0., 0., 0.))
        self.assertTrue(n._parking_space(0., 0., math.pi/2))

    def test_rotated_body_hard_stop_and_unconfirmed_parking(self):
        from sensor_msgs.msg import PointField
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header, String
        fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(("x", "y", "z", "intensity"))]
        for obstacle_y, result in ((1.57, "BODY_CLEARANCE_STOP"),
                                   (1.54, "PARKING_CLEARANCE_UNCONFIRMED"),
                                   (1.48, "DONE")):
            n = self.gate()
            n._pose = (1., 2., math.pi/2)
            n.front_stop_min, n.front_stop_max = .15, .80
            n.front_stop_deg, n.front_stop_pts = 30., 6
            n.seen = {"odom": time.monotonic()}
            n.finished = None
            n.exit_requested = False
            n.hard_stop_pub = NS(publish=lambda _: None)
            faults = []
            n._fault = faults.append
            cloud = point_cloud2.create_cloud(Header(frame_id="map"), fields,
                                              [(1., obstacle_y, .25, .25)])
            n.cb_terrain(cloud)
            n.cb_phase(String(data="DONE"))
            self.assertEqual(n.finished, result)
            self.assertEqual(len(faults), 1)
            self.assertEqual(n.should_exit(), result != "BODY_CLEARANCE_STOP")
            if result == "BODY_CLEARANCE_STOP":
                n.cb_stop(NS(data=1))
                self.assertTrue(n.should_exit())
                self.assertEqual(n.finished, result)

    def test_exploration_stays_paused_after_body_hard_stop(self):
        n = self.corridor()
        n.mode = "EXPLORE"
        n.current_goal = (2., 0.)
        n.cb_hard_stop(NS(data=True))
        n.cb_return_home(NS(data=True))
        n.cb_waypoint(NS())
        self.assertEqual(n.mode, "FAULT_HOLD")
        self.assertFalse(n.sent)

    def test_unknown_cloud_cannot_authorize_parking(self):
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        n = self.gate()
        n._pose = (0., 0., 0.)
        n.finished = None
        n.seen = {"odom": time.monotonic(), "terrain": time.monotonic()}
        n._fault = lambda reason: None
        n.cb_terrain(PointCloud2())  # no recognized terrain frame
        n.cb_phase(String(data="DONE"))
        self.assertEqual(n.finished, "PARKING_CLEARANCE_UNCONFIRMED")

    def test_side_obstacle_blocks_forced_turn_until_clear(self):
        from sensor_msgs.msg import PointField
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(("x", "y", "z", "intensity"))]
        n = self.gate()
        n._pose = (0., 0., 0.)
        n.front_stop_enable = True
        n.front_stop_min, n.front_stop_max = .15, .80
        n.front_stop_deg, n.front_stop_pts = 30, 6
        n.phase, n.rotate = 'RETURN', .45
        n.seen = {'rotate': time.monotonic()}
        n.last_cmd = (.2, .1, .3)
        for y, blocked in ((.36, True), (.80, False)):
            cloud = point_cloud2.create_cloud(Header(frame_id="map"), fields, [(0., y, .5, .5)])
            n.cb_terrain(cloud)
            self.assertEqual(n.turn_blocked, blocked)
            n.cb(self.cmd(.2))
            if blocked:
                self.assertEqual(n.sent[-1], (0., 0., 0.))
            else:
                self.assertGreater(n.sent[-1][2], 0.)
    def test_cloud_transform_preserves_fractional_point_time(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "autonomy_stack/transform"))
        from transform_everything import Repuber
        from sensor_msgs_py import point_cloud2 as pc
        from sensor_msgs.msg import PointField
        from std_msgs.msg import Header
        from geometry_msgs.msg import TransformStamped
        fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i,name in enumerate(("x","y","z","intensity","time"))]
        cloud = pc.create_cloud(Header(), fields, [(1.,2.,3.,4.,.125)])
        n = object.__new__(Repuber)
        n.time_stamp_offset_set = True
        n.time_stamp_offset = 0
        n.body2cloud_trans = TransformStamped()
        n.body2cloud_trans.transform.rotation.w = 1.
        n.cam_offset = .25
        n.use_body_filter = False
        out = []
        n.cloud_pub = NS(publish=out.append)
        n.cloud_callback(cloud)
        result = pc.read_points(out[0])[0]
        self.assertEqual((result["x"], result["z"], result["time"]), (1.,2.75,.125))

if __name__=='__main__':unittest.main()
