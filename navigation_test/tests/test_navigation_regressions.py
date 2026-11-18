"""Offline regressions on the actual node methods, without creating DDS participants."""
from pathlib import Path
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
    def test_single_bfs_field_matches_candidate_bfs(self):
        n=self.explorer();n.pos=(0.,0.);n.snap_radius=.6
        n.grid={(x,y):1 for x in range(9) for y in range(5)}
        n._blocked={(4,y) for y in range(4)}
        distances=n._reachable_distances()
        for cell in n.grid:
            expected=n._bfs(*cell)
            actual=distances.get(cell)
            if expected is None:self.assertIsNone(actual)
            else:self.assertAlmostEqual(actual,expected)

    def gate(self):
        n=object.__new__(Gate);n.state=1;n.last_cmd_time=time.time()-.2;n.last_cmd=(0.,0.,0.)
        n.max_speed=.3;n.min_speed=.3;n.max_yaw_rate=.7;n.max_accel=10.;n.phase='EXPLORE';n.rotate=0.;n.front_blocked=False
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
