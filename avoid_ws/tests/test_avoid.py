import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

from go2_reactive_avoid.controller import Direction, Explorer, Scan, STOP, yaw_delta
from go2_reactive_avoid.entrance import Entrance
from go2_reactive_avoid.lidar import LIDAR_X, MOUNT_PITCH, body_points, measure
from go2_reactive_avoid.node import API_QOS, ARMED, DISARMED, AvoidGate, Inputs
from go2_reactive_avoid.writer_audit import verify_snapshot

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
        self.assertEqual(self.step(.3, scan()), (.20, 0))
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
        self.assertEqual(self.step(4.9, scan(), -92), (.20, 0))

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
        self.assertEqual(self.step(9.7, scan(), -180), (.20, 0))

    def test_rear_coverage_is_not_required_for_uturn(self):
        view = scan(BLOCKED, BLOCKED, BLOCKED, UNKNOWN)
        self.step(0, view)
        self.assertEqual(self.step(.3, view), STOP)
        self.assertEqual(self.ctrl.phase, "TURN")

    def test_missing_front_coverage_does_not_drive(self):
        self.step(0, scan(front=UNKNOWN))
        self.assertEqual(self.step(.3, scan(front=UNKNOWN)), STOP)

    def test_microturn_moves_forward_and_does_not_repeat_in_same_encounter(self):
        view = scan(near_left=True)
        self.step(0, view)
        self.assertEqual(self.step(.3, view), (.20, -.50))
        self.assertEqual(self.ctrl.target, math.radians(-20))
        self.assertEqual(self.step(.5, view, -5)[0], .20)
        self.assertEqual(self.step(1.5, view, -20, still=False), (.20, 0))
        self.assertEqual(self.ctrl.phase, "DRIVE")
        self.assertEqual(self.step(2.2, view, -20), (.20, 0))
        self.step(2.3, scan(), -20)
        self.assertEqual(self.step(2.4, scan(near_right=True), -20), (.20, .50))
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

    def test_turn_does_not_use_legacy_sweep_flag(self):
        self.step(0, scan(front=BLOCKED))
        self.step(.3, scan(front=BLOCKED))
        self.assertEqual(self.step(.4, scan(rotate=False)), (0, -.60))
        self.assertEqual(self.ctrl.phase, "TURN")

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

    def test_confirmed_nose_returns_are_filtered_in_mount_frame(self):
        c, s = math.cos(MOUNT_PITCH), math.sin(MOUNT_PITCH)
        points = np.array([[.37, 0., -.16], [.40, .22, -.16], [.37, 0., -.10],
                           [.46, 0., -.16], [.35, -.03, -.31], [.1, -.25, -.16]])
        x, y, z = (points - [LIDAR_X, 0., 0.]).T
        raw = np.column_stack((c*x - s*z, y, s*x + c*z))
        for roll, pitch, height in ((0., 0., .30), (.08, -.05, .33)):
            original = body_points(raw, roll, pitch, height)
            np.testing.assert_allclose(
                body_points(raw, roll, pitch, height, filter_self_returns=True), original[1:])

    def test_filter_preserves_front_obstacle_outside_nose(self):
        c, s = math.cos(MOUNT_PITCH), math.sin(MOUNT_PITCH)
        for obstacle in ([.38, 0., .22], [.59, 0., .16]):
            with self.subTest(obstacle=obstacle):
                points = np.vstack((floor(), [[.37, 0., .14], obstacle]))
                x, y, z = (points - [LIDAR_X, 0., .30]).T
                raw = np.column_stack((c*x - s*z, y, s*x + c*z))
                view = measure(body_points(raw, 0., 0., .30, filter_self_returns=True))
                self.assertTrue(view.body_clear)
                self.assertEqual(view.front, BLOCKED)

    def test_four_detection_bits_are_independent(self):
        view = measure(np.vstack((floor(), [[.59, 0, .15], [0, .32, .15],
                                            [-.65, 0, .15], [0, -.32, .15]])))
        self.assertEqual(view, scan(BLOCKED, BLOCKED, BLOCKED, BLOCKED,
                                    near_left=True, near_right=True, front_gap=.24))

    def test_side_micro_threshold_is_fifteen_cm_from_body_edge(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                for y, expect_near in ((.32, True), (.35, True), (.351, False)):
                    points = np.vstack((floor(), [[-.50, sign*y, .16]]))
                    view = measure(points)
                    self.assertEqual(view.left_near if sign > 0 else view.right_near, expect_near)
                    self.assertFalse(view.right_near if sign > 0 else view.left_near)
                    controller = Explorer()
                    controller.update(0., 0., view, 0., True)
                    _, yaw_rate = controller.update(.3, .3, view, 0., True)
                    if expect_near:
                        self.assertLess(sign * yaw_rate, 0.)
                    else:
                        self.assertEqual(yaw_rate, 0.)

    def test_fixed_speed_until_front_bit_then_stop(self):
        ctrl = Explorer()

        def view(x):
            return measure(np.array([[x, 0, .15]]))
        ctrl.update(0., 0., view(1.4), 0., True)
        for now, x in ((.3, 1.4), (.4, .95), (.45, .61)):
            self.assertEqual(ctrl.update(now, now, view(x), 0., False if now > .3 else True),
                             (.20, 0.))
        self.assertEqual(ctrl.update(.5, .5, view(np.float32(.60)), 0., False), STOP)
        self.assertEqual(ctrl.phase, "BRAKE")

    def test_point_beside_body_is_not_in_front_corridor(self):
        view = measure(np.vstack((floor(), [[.20, -.24, .16]])))
        self.assertEqual(view.front, CLEAR)
        self.assertTrue(view.right_near)
        self.assertTrue(view.body_clear)
        self.assertTrue(math.isinf(view.front_gap))

    def test_side_bit_triggers_arc_without_sweep_or_floor(self):
        view = measure(np.array([[0., .25, .16]]))
        self.assertTrue(view.left_near)
        controller = Explorer()
        self.assertEqual(controller.update(0., 0., view, 0., True), STOP)
        self.assertEqual(controller.update(.3, .3, view, 0., True), (.20, -.50))
        self.assertEqual(controller.phase, "MICRO")

    def test_partial_cloud_has_binary_directions(self):
        points = floor()
        view = measure(points[points[:, 0] > -.70])
        self.assertEqual(view.front, CLEAR)
        self.assertEqual(view.rear, CLEAR)

    def test_empty_or_nonfinite_cloud_is_invalid(self):
        for points in (np.empty((0, 3)), np.array([[math.nan, 0, 0]])):
            self.assertIsNone(measure(points))

    def test_all_eight_binary_inputs_select_expected_action(self):
        for front in (0, 1):
            for left in (0, 1):
                for right in (0, 1):
                    with self.subTest(front=front, left=left, right=right):
                        points = [[1., 0., 0.]]
                        if front:
                            points.append([.59, 0., .2])
                        if left:
                            points.append([0., .3, .2])
                        if right:
                            points.append([0., -.3, .2])
                        view = measure(np.array(points))
                        ctrl = Explorer()
                        ctrl.update(0., 0., view, 0., True)
                        command = ctrl.update(.3, .3, view, 0., True)
                        if front:
                            self.assertEqual(command, STOP)
                            self.assertEqual(ctrl.phase, "TURN")
                            self.assertAlmostEqual(ctrl.target, math.pi / 2 if right and not left
                                                   else -math.pi / 2)
                            self.assertEqual(ctrl.uturn_remaining, bool(left and right))
                        else:
                            self.assertEqual(command[0], .20)
                            self.assertEqual(command[1], -.50 if left and not right else
                                             .50 if right and not left else 0.)

    def test_height_and_body_filters_do_not_set_obstacle_bits(self):
        view = measure(np.array([[.59, 0, .09], [.59, 0, 1.21], [0, 0, .2]]))
        self.assertEqual(view.front, CLEAR)
        self.assertFalse(view.left_near or view.right_near)

    def test_both_sides_blocked_and_front_clear_keeps_straight(self):
        view = measure(np.array([[0, .3, .2], [0, -.3, .2]]))
        controller = Explorer()
        controller.update(0., 0., view, 0., True)
        self.assertEqual(controller.update(.3, .3, view, 0., True), (.20, 0.))


class WriterAuditTest(unittest.TestCase):
    def test_restarted_onboard_writer_can_refresh_only_while_idle(self):
        old_gid = "0110abcd000000000000000100000803" + "0" * 16
        new_gid = "0110abcd000000000000000200000803" + "0" * 16
        trace = (
            "1.0 [0] dq.builtin: SPDP ST0 110abcd:0:2:1c1 bes 3000fc3f NEW "
            "(0x00000000 Unitree/0.10.2/Linux/Linux) "
            "(data udp/239.255.0.1:7401@2 udp/192.168.123.161:2000@2 meta udp/192.168.123.161:2000@2)\n"
            "1.1 [0] dq.builtin: SEDP ST0 110abcd:0:2:803 reliable volatile writer unnamed: "
            "(default).rt/api/sport/request/unitree_api::msg::dds_::Request_ p(open) "
            "NEW (as udp/239.255.0.1:7401@2 udp/192.168.123.161:2000@2 ssm=0)\n"
        )
        verify_snapshot({old_gid}, {new_gid}, trace, 0, 0)
        with self.assertRaisesRegex(ValueError, "无法核实"):
            verify_snapshot({old_gid}, {new_gid}, trace.replace("192.168.123.161", "192.168.123.99"),
                            0, 0)
        for counts in ((1, 0), (0, 1)):
            with self.assertRaisesRegex(ValueError, "控制活动"):
                verify_snapshot({old_gid}, {new_gid}, trace, *counts)
        with self.assertRaisesRegex(ValueError, "拓扑"):
            verify_snapshot({old_gid}, {old_gid, new_gid}, trace, 0, 0)


class GateTest(unittest.TestCase):
    def setUp(self):
        with patch.object(AvoidGate, "__init__", return_value=None):
            self.gate = AvoidGate(.30)
        self.gate.entrance = None

    def test_proximity_reference_does_not_block_arming(self):
        gate = self.gate
        gate.started, gate.last_status = 0., 3.
        gate.state = DISARMED
        gate.result = None
        gate.lease_id = 55
        gate.check_writers = Mock(return_value=True)
        gate.command_receivers = Mock(return_value=(1, 2, 1))
        gate.inputs = SimpleNamespace(scan=scan(body=False), ready=Mock(return_value=True),
                                      cloud_count=5, state_count=5, still=True, sport_error_code=0)
        with (patch("go2_reactive_avoid.node.time.monotonic", return_value=3.),
              patch("go2_reactive_avoid.node.SafetyGate.check"),
              patch("go2_reactive_avoid.node.SafetyGate.cb_arm",
                    side_effect=lambda _: setattr(gate, "state", ARMED)) as arm):
            gate.check()
        arm.assert_called_once()
        self.assertEqual(gate.armed_at, 3.)

    def test_entrance_arms_and_drives_with_only_fresh_sport_feedback(self):
        gate = self.gate
        gate.entrance = Entrance(50, 30)
        gate.phase = ""
        gate.started, gate.last_status = 0., 3.
        gate.state, gate.result, gate.lease_id = DISARMED, None, 55
        gate.check_writers = Mock(return_value=True)
        gate.command_receivers = Mock(return_value=(1, 2, 1))
        gate.send_motion = Mock()
        gate.finish = Mock()
        gate.get_logger = Mock()
        gate.inputs = SimpleNamespace(scan=None, ready=Mock(return_value=True), status=Mock(),
                                      cloud_count=0, state_count=5, still=True, sport_error_code=0,
                                      state_at=3., yaw=0., position=(0., 0.))
        with (patch("go2_reactive_avoid.node.time.monotonic", return_value=3.),
              patch("go2_reactive_avoid.node.SafetyGate.check"),
              patch("go2_reactive_avoid.node.SafetyGate.cb_arm",
                    side_effect=lambda _: setattr(gate, "state", ARMED)) as arm):
            gate.check()
            arm.assert_called_once()
            gate.check()
            self.assertGreater(gate.send_motion.call_args.args[0][0], 0.0)
            gate.inputs.ready.return_value = False
            gate.send_motion.reset_mock()
            gate.check()
        gate.finish.assert_called_once_with("Sport 反馈或命令接收端丢失")
        gate.send_motion.assert_not_called()

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
        gate.lease_request_writers = Mock(return_value=([gid.hex()], []))
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
        gate.lease_request_writers.return_value = ([], ["foreign gid=99"])
        self.assertFalse(gate.check_writers(1.0))
        self.assertIn("lease外部=1", gate.finish.call_args.args[0])

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
        gate.lease_request_writers.return_value = ([], [])
        self.assertFalse(gate.check_writers(3.0))
        self.assertIn("丢失", gate.finish.call_args.args[0])

    def test_command_receiver_excludes_the_local_monitor_and_requires_a_match(self):
        self.setup_writers()
        gate = self.gate
        monitor = SimpleNamespace(node_name="avoid", node_namespace="/",
                                  topic_type="unitree_api/msg/Request", qos_profile=API_QOS)
        external = SimpleNamespace(node_name="sport_receiver", node_namespace="/",
                                   topic_type=monitor.topic_type, qos_profile=API_QOS)
        gate.pub = Mock()
        gate.get_subscriptions_info_by_topic = Mock(return_value=[monitor])
        gate.pub.get_subscription_count.return_value = 1
        self.assertEqual(gate.command_receivers(), (0, 1, 1))
        gate.get_subscriptions_info_by_topic.return_value = [monitor, external]
        self.assertEqual(gate.command_receivers(), (0, 1, 1))
        gate.get_subscriptions_info_by_topic.return_value = [external]
        self.assertEqual(gate.command_receivers(), (0, 1, 0))
        gate.get_subscriptions_info_by_topic.return_value = [monitor, external]
        gate.pub.get_subscription_count.return_value = 2
        self.assertEqual(gate.command_receivers(), (1, 2, 1))

    def test_unsolicited_lease_response_does_not_mark_lease_obtained(self):
        self.gate._lease_pending_api = 0
        response = Response()
        response.header.identity.api_id = 101
        with patch("go2_reactive_avoid.node.SafetyGate.cb_lease_response") as shared_callback:
            self.gate.cb_lease_response(response)
        shared_callback.assert_not_called()


class SensorAdapterTest(unittest.TestCase):
    class Node:
        def create_subscription(self, *_args):
            pass

    def test_sport_only_inputs_do_not_subscribe_to_cloud(self):
        node = Mock()
        inputs = Inputs(node, .35, track_position=True, use_lidar=False)
        node.create_subscription.assert_not_called()
        state = SportModeState()
        state.stamp.sec = 100
        state.body_height = .30
        with patch("go2_reactive_avoid.node.time.monotonic", return_value=10.0):
            inputs.sport(state)
        self.assertTrue(inputs.ready(10.1))
        self.assertIsNone(inputs.scan)
        self.assertIn("Sport XY", inputs.status(10.1))
        self.assertFalse(inputs.ready(10.5))

    def test_ros_messages_raw_cloud_axes_and_stale_or_invalid_inputs(self):
        inputs = Inputs(self.Node(), .35)
        state = SportModeState()
        state.stamp.sec = 100
        state.error_code = 100
        state.imu_state.rpy[2] = 3.12
        angles = math.cos(MOUNT_PITCH), math.sin(MOUNT_PITCH)
        c, s = angles
        points = np.vstack((floor(), [[.59, 0, .2]]))
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
        self.assertIn("前缘距离=0.240m", inputs.preflight(10.1))
        self.assertTrue(inputs.still)
        self.assertEqual(inputs.sport_error_code, 100)
        self.assertFalse(inputs.ready(10.5))
        stale = inputs.preflight(10.5)
        self.assertIn("雷达=超时", stale)
        self.assertIn("Sport=超时", stale)
        self.assertIn("静止=未知 前向=未知", stale)
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
