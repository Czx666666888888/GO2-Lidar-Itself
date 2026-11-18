import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

from go2_reactive_avoid.controller import Direction, Explorer, Scan, STOP, yaw_delta
from go2_reactive_avoid.lidar import LIDAR_X, MOUNT_PITCH, body_points, measure
from go2_reactive_avoid.node import ARMED, AvoidGate, Inputs

CLEAR, BLOCKED, UNKNOWN = Direction.CLEAR, Direction.BLOCKED, Direction.UNKNOWN


def scan(front=CLEAR, right=CLEAR, left=CLEAR, rear=CLEAR, rotate=True, body=True,
         near_right=False, near_left=False, front_gap=math.inf):
    return Scan(front, right, left, rear, rotate, body, near_right, near_left, front_gap)


class ExplorerTest(unittest.TestCase):
    def setUp(self):
        self.ctrl = Explorer()

    def step(self, time, view, degrees=0, still=True, observed=None):
        return self.ctrl.update(time, time if observed is None else observed, view,
                                math.radians(degrees), still)

    def test_front_stop_precedes_right_turn_and_checks_actual_settle(self):
        self.step(0, scan())
        self.assertEqual(self.step(.3, scan()), (.40, 0))
        blocked = scan(front=BLOCKED)
        self.assertEqual(self.step(.4, blocked, still=False), STOP)
        self.assertEqual(self.step(.7, blocked, still=False), STOP)
        self.step(.8, blocked)
        self.assertEqual(self.step(1.1, blocked, observed=.9), STOP)
        self.assertEqual(self.ctrl.phase, "BRAKE")
        self.assertEqual(self.step(1.2, blocked), STOP)
        self.assertEqual(self.ctrl.target, -math.pi / 2)
        self.assertEqual(self.step(1.3, blocked), (0, -.60))
        self.assertEqual(self.step(2, blocked, -40), (0, -.60))
        self.assertEqual(self.step(4, scan(), -88, still=False), STOP)
        self.assertEqual(self.ctrl.phase, "TURN")
        self.step(4.1, scan(), -92)
        self.step(4.5, scan(), -92)
        self.assertEqual(self.ctrl.phase, "BRAKE")
        self.step(4.6, scan(), -92)
        self.assertEqual(self.step(4.9, scan(), -92), (.40, 0))

    def test_left_turn_when_right_blocked(self):
        view = scan(front=BLOCKED, right=BLOCKED)
        self.step(0, view)
        self.step(.3, view)
        self.assertEqual(self.ctrl.target, math.pi / 2)
        self.assertEqual(self.step(.4, view), (0, .60))

    def test_four_blocked_two_90_degree_turns_then_hold(self):
        view = scan(BLOCKED, BLOCKED, BLOCKED, BLOCKED)
        self.step(0, view, 170)
        self.step(.3, view, 170)
        self.assertAlmostEqual(math.degrees(self.ctrl.target), 80)
        self.step(4.5, view, 78)
        self.step(4.8, view, 78)
        self.step(4.9, view, 78)
        self.step(5.2, view, 78)
        self.assertAlmostEqual(math.degrees(self.ctrl.target), -10)
        self.assertEqual(self.step(5.3, view, 77), (0, -.60))
        self.step(10, view, -10)
        self.step(10.3, view, -10)
        self.step(10.4, view, -10)
        self.assertEqual(self.step(10.7, view, -10), STOP)
        self.assertEqual(self.ctrl.phase, "HALTED")

    def test_dead_end_with_clear_rear_resumes_forward_after_uturn(self):
        view = scan(BLOCKED, BLOCKED, BLOCKED, CLEAR)
        self.step(0, view)
        self.step(.3, view)
        self.step(4, view, -90)
        self.step(4.3, view, -90)
        self.step(4.4, view, -90)
        self.step(4.7, view, -90)
        self.step(9, scan(), -180)
        self.step(9.3, scan(), -180)
        self.step(9.4, scan(), -180)
        self.assertEqual(self.step(9.7, scan(), -180), (.40, 0))

    def test_unknown_rear_prevents_uturn(self):
        view = scan(BLOCKED, BLOCKED, BLOCKED, UNKNOWN)
        self.step(0, view)
        self.assertEqual(self.step(.3, view), STOP)
        self.assertEqual(self.ctrl.phase, "BRAKE")

    def test_missing_front_coverage_does_not_drive(self):
        self.step(0, scan(front=UNKNOWN))
        self.assertEqual(self.step(.3, scan(front=UNKNOWN)), STOP)

    def test_microturn_moves_forward_and_does_not_repeat_in_same_encounter(self):
        view = scan(near_left=True)
        self.step(0, view)
        self.assertEqual(self.step(.3, view), (.25, -.50))
        self.assertEqual(self.ctrl.target, math.radians(-20))
        self.assertEqual(self.step(.5, view, -5)[0], .25)
        self.assertEqual(self.step(1.5, view, -20, still=False), (.40, 0))
        self.assertEqual(self.ctrl.phase, "DRIVE")
        self.assertEqual(self.step(2.2, view, -20), (.40, 0))
        self.step(2.3, scan(), -20)
        self.assertEqual(self.step(2.4, scan(near_right=True), -20), (.25, .50))
        self.assertAlmostEqual(self.ctrl.target, 0)

    def test_clear_microturn_has_no_stop_command_at_any_control_tick(self):
        view = scan(near_left=True)
        self.step(0, view)
        yaw = 0.0
        vx, wz = self.ctrl.update(.3, .3, view, yaw, True)
        self.assertGreaterEqual(vx, .20)
        self.assertLessEqual(vx, .40)
        for tick in range(1, 201):
            yaw += wz * .02
            vx, wz = self.ctrl.update(.3 + tick*.02, .3 + tick*.02, view, yaw, False)
            self.assertGreaterEqual(vx, .20)
            self.assertLessEqual(vx, .40)
        self.assertEqual(self.ctrl.phase, "DRIVE")
        self.assertLessEqual(abs(math.degrees(yaw) + 20), 3)

    def test_microturn_gives_way_to_front_stop(self):
        self.step(0, scan(near_right=True))
        self.step(.3, scan(near_right=True))
        self.assertEqual(self.step(.5, scan(front=BLOCKED), 3, still=False), STOP)
        self.assertEqual(self.ctrl.phase, "BRAKE")

    def test_turn_halts_on_sweep_loss(self):
        self.step(0, scan(front=BLOCKED))
        self.step(.3, scan(front=BLOCKED))
        self.assertEqual(self.step(.4, scan(rotate=False)), STOP)
        self.assertEqual(self.ctrl.phase, "HALTED")

    def test_turn_halts_on_timeout_and_stays_latched(self):
        self.step(0, scan(front=BLOCKED))
        self.step(.3, scan(front=BLOCKED))
        self.assertEqual(self.step(11, scan(), 0), STOP)
        self.assertIn("时限", self.ctrl.reason)
        self.assertEqual(self.step(12, scan(), -90), STOP)

    def test_yaw_unwrap_in_both_directions(self):
        self.assertAlmostEqual(math.degrees(yaw_delta(math.radians(-179), math.radians(179))), 2)
        self.assertAlmostEqual(math.degrees(yaw_delta(math.radians(179), math.radians(-179))), -2)


def floor():
    angle = np.radians(np.arange(0, 360, 10))
    return np.column_stack((np.cos(angle), np.sin(angle), np.zeros_like(angle)))


class LidarTest(unittest.TestCase):
    def test_mount_maps_axes(self):
        c, s = math.cos(MOUNT_PITCH), math.sin(MOUNT_PITCH)
        np.testing.assert_allclose(body_points(np.array([[c, 0, s], [0, 1, 0]]), 0, 0, .35),
                                   [[1 + LIDAR_X, 0, .35], [LIDAR_X, 1, .35]])

    def test_four_obstacles_with_sweep_room(self):
        view = measure(np.vstack((floor(), [[.7, 0, .15], [0, .55, .15],
                                            [-.65, 0, .15], [0, -.55, .15]])))
        self.assertEqual(view, scan(BLOCKED, BLOCKED, BLOCKED, BLOCKED,
                                    near_left=True, near_right=True, front_gap=.35))

    def test_front_cloud_triggers_cruise_slowdown_and_then_stop(self):
        ctrl = Explorer()
        view = lambda x: measure(np.vstack((floor(), [[x, 0, .15]])))
        ctrl.update(0., 0., view(1.4), 0., True)
        vx, wz = ctrl.update(.3, .3, view(1.4), 0., True)
        self.assertAlmostEqual(vx, .40)
        self.assertEqual(wz, 0.)
        vx, wz = ctrl.update(.4, .4, view(1.1), 0., False)
        self.assertAlmostEqual(vx, .30)
        self.assertEqual(wz, 0.)
        vx, _ = ctrl.update(.45, .45, view(.91), 0., False)
        self.assertAlmostEqual(vx, .205)
        self.assertEqual(ctrl.update(.5, .5, view(.90), 0., False), STOP)
        self.assertEqual(ctrl.phase, "BRAKE")

    def test_hard_clearance_and_sweep_are_independent(self):
        for obstacle, body_clear in (([.45, .0, .12], True), ([.41, 0, .12], False)):
            with self.subTest(obstacle=obstacle):
                view = measure(np.vstack((floor(), obstacle)))
                self.assertFalse(view.can_rotate)
                self.assertEqual(view.body_clear, body_clear)
                self.assertEqual(view.front, BLOCKED)

    def test_missing_rear_coverage_is_unknown_even_if_front_is_clear(self):
        points = floor()
        view = measure(points[points[:, 0] > -.70])
        self.assertEqual(view.front, CLEAR)
        self.assertEqual(view.rear, UNKNOWN)
        self.assertFalse(view.can_rotate)

    def test_no_floor_does_not_produce_clearances(self):
        points = floor()
        points[:, 2] = .3
        self.assertIsNone(measure(points))


class GateTest(unittest.TestCase):
    def setUp(self):
        with patch.object(AvoidGate, "__init__", return_value=None):
            self.gate = AvoidGate(.30)

    def test_first_forward_command_reaches_gait_floor_and_turn_can_start_at_zero(self):
        gate = self.gate
        gate.state = ARMED
        gate.last_cmd = (0.0, 0.0, 0.0)
        gate.last_cmd_time = 10.0
        gate.max_speed = .40
        gate.min_speed = .20
        gate.max_yaw_rate = .60
        gate.max_accel = .50
        gate._send = Mock()

        with patch("go2_reactive_avoid.node.time.time", return_value=10.02):
            gate.send_motion((.40, 0.0))
        vx, vy, wz = gate._send.call_args.args
        self.assertAlmostEqual(vx, .21)
        self.assertEqual((vy, wz), (0.0, 0.0))

        gate.last_cmd = (0.0, 0.0, 0.0)
        with patch("go2_reactive_avoid.node.time.time", return_value=10.04):
            gate.send_motion((0.0, .60))
        vx, vy, wz = gate._send.call_args.args
        self.assertEqual((vx, vy), (0.0, 0.0))
        self.assertGreater(wz, 0.0)

    def setup_writers(self):
        gate = self.gate
        gid = b"\x01" * 24
        onboard_gid = b"\x02" * 24
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        own = SimpleNamespace(node_name="avoid", node_namespace="/", endpoint_gid=gid, qos_profile=qos)
        onboard = SimpleNamespace(node_name="bare", node_namespace="bare", endpoint_gid=onboard_gid,
                                  qos_profile=qos)
        gate.get_name = Mock(return_value="avoid")
        gate.get_namespace = Mock(return_value="/")
        gate.count_publishers = Mock(return_value=1)
        gate.get_publishers_info_by_topic = Mock(return_value=[own, onboard])
        gate.verified_writers = {onboard_gid.hex()}
        gate.own_sport_gid = None
        gate.writers_since = None
        gate.armed_at = None
        gate.finish = Mock()
        return own, onboard

    def test_verified_idle_writer_and_missing_local_discovery_wait(self):
        own, onboard = self.setup_writers()
        gate = self.gate
        gate.get_publishers_info_by_topic.return_value = [onboard]
        self.assertFalse(gate.check_writers(0.0))
        gate.finish.assert_not_called()
        gate.get_publishers_info_by_topic.return_value = [own, onboard]
        self.assertFalse(gate.check_writers(1.0))
        self.assertTrue(gate.check_writers(3.0))
        gate.finish.assert_not_called()

    def test_unknown_writer_blocks_startup_and_duplicate_lease_stays_blocked(self):
        _, onboard = self.setup_writers()
        gate = self.gate
        gate.verified_writers = set()
        self.assertFalse(gate.check_writers(0.0))
        self.assertIn("未核实", gate.finish.call_args.args[0])
        gate.verified_writers = {onboard.endpoint_gid.hex()}
        gate.count_publishers.return_value = 2
        self.assertFalse(gate.check_writers(1.0))
        self.assertIn("lease=2", gate.finish.call_args.args[0])

    def test_only_own_sport_request_can_pass_even_with_verified_foreign_writer(self):
        self.setup_writers()
        gate = self.gate
        gate.check_writers(0.0)
        gate.first_request_id = 100
        gate._req_id = 101
        gate.lease_id = 55
        msg = Request()
        msg.header.identity.api_id = 1008
        msg.header.identity.id = 100
        msg.header.lease.id = 55
        gate.on_sport_request(msg)
        gate.finish.assert_not_called()
        msg.header.identity.id = 99
        gate.on_sport_request(msg)
        self.assertIn("api=1008", gate.finish.call_args.args[0])

    def test_missing_lease_writer_after_arming_stops(self):
        self.setup_writers()
        gate = self.gate
        gate.check_writers(0.0)
        gate.armed_at = 2.0
        gate.count_publishers.return_value = 0
        self.assertFalse(gate.check_writers(3.0))
        self.assertIn("丢失", gate.finish.call_args.args[0])


class SensorAdapterTest(unittest.TestCase):
    class Node:
        def create_subscription(self, *_args):
            pass

    def test_ros_messages_raw_cloud_axes_and_stale_or_invalid_inputs(self):
        inputs = Inputs(self.Node(), .35)
        state = SportModeState()
        state.stamp.sec = 100
        state.error_code = 100
        state.imu_state.rpy[2] = 3.12
        angles = math.cos(MOUNT_PITCH), math.sin(MOUNT_PITCH)
        c, s = angles
        points = np.vstack((floor(), [[.7, 0, .2]]))
        x, y, z = (points - [LIDAR_X, 0., .35]).T
        raw = np.column_stack((c*x - s*z, y, s*x + c*z)).astype(np.float32)
        header = Header(frame_id="utlidar_lidar_1")
        header.stamp.sec = 300
        fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate("xyz")]
        cloud = point_cloud2.create_cloud(header, fields, raw)
        with patch("go2_reactive_avoid.node.time.monotonic", return_value=10.0):
            inputs.sport(state)
            inputs.cloud(cloud)
        self.assertTrue(inputs.ready(10.1))
        self.assertEqual(inputs.scan.front, BLOCKED)
        self.assertTrue(inputs.still)
        self.assertEqual(inputs.sport_error_code, 100)
        self.assertFalse(inputs.ready(10.5))
        with patch("go2_reactive_avoid.node.time.monotonic", return_value=10.15):
            state.stamp.nanosec = 150_000_000
            state.imu_state.rpy[2] = -3.12
            inputs.sport(state)
            self.assertGreater(inputs.yaw, math.pi)
            cloud.header.frame_id = "map"
            cloud.header.stamp.nanosec = 150_000_000
            inputs.cloud(cloud)
        self.assertFalse(inputs.ready(10.15))
        self.assertIn("map", inputs.cloud_message)


if __name__ == "__main__":
    unittest.main()
