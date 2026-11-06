"""Opt-in real ROS transport tests using synthetic data in an isolated domain."""

import json
import math
import os
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('SCIENCE_ROS_TEST') != '1',
                                reason='opt-in DDS test; use isolated ROS_DOMAIN_ID=86')


def test_ros_closed_loop_and_terminal_latch():
    """Drive synthetic feedback through actual subscriptions without a raw publisher."""
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Empty, String
    from unitree_go.msg import SportModeState
    from go2_science_perception.science_target_approach import ScienceTargetApproach

    assert os.environ.get('ROS_DOMAIN_ID') == '86'
    rclpy.init()
    control = ScienceTargetApproach()
    source = rclpy.create_node('synthetic_approach_source')
    odom = source.create_publisher(Odometry, '/base_state_estimation', 10)
    targets = source.create_publisher(String, '/science/target_tracks', 10)
    feedback = source.create_publisher(SportModeState, '/lf/sportmodestate', 10)
    confirm = source.create_publisher(Empty, '/science/approach/confirm', 1)
    reports, commands = [], []
    source.create_subscription(String, '/science/approach/status',
                               lambda msg: reports.append(json.loads(msg.data)), 10)
    source.create_subscription(TwistStamped, '/science/approach/cmd_vel_preview',
                               lambda msg: commands.append(msg.twist.linear.x), 10)
    x, vx, confirmed = 0.0, 0.0, False
    started, previous = time.monotonic(), time.monotonic()
    try:
        while time.monotonic() - started < 15:
            now = time.monotonic()
            x += vx * (now - previous)
            previous = now
            stamp = source.get_clock().now().to_msg()
            msg = Odometry()
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp
            msg.pose.pose.orientation.w = 1.0
            msg.pose.pose.position.x = x
            odom.publish(msg)
            targets.publish(String(data=json.dumps({'frame': 'map', 'tracks': [
                {'id': 7, 'state': 'CONFIRMED', 'visited': False,
                 'position': [1.1, 0, 0],
                 'observed_ns': stamp.sec * 10**9 + stamp.nanosec,
                 'first_seen_ns': stamp.sec * 10**9 + stamp.nanosec - 5 * 10**9}]})))
            feedback.publish(SportModeState())
            if now - started > 1 and not confirmed:
                confirm.publish(Empty())
                confirmed = True
            for _ in range(5):
                rclpy.spin_once(control, timeout_sec=0.005)
                rclpy.spin_once(source, timeout_sec=0.005)
            vx = commands[-1] if commands else 0.0
            if reports and reports[-1]['state'] == 'STOP':
                break
            time.sleep(0.05)
        assert reports[-1]['state'] == 'STOP', reports[-1]
        assert reports[-1]['distance'] == pytest.approx(0.8, abs=0.031)
        assert 0 < max(commands) <= 0.3
        assert source.count_publishers('/cmd_vel_raw') == 0
        assert source.count_publishers('/api/sport/request') == 0
        assert reports[-1]['vx'] == reports[-1]['wz'] == 0
        assert math.isfinite(reports[-1]['actual_speed'])
        assert not control.core.confirm(time.monotonic())
    finally:
        control.stop()
        control.log_file.close()
        control.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_timestamped_tf_and_input_faults():
    """Rotate and translate body pose; reject stale/future/time-regressing data."""
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int8
    from go2_science_perception.science_target_approach import ScienceTargetApproach

    assert os.environ.get('ROS_DOMAIN_ID') == '86'
    rclpy.init()
    node = ScienceTargetApproach()
    try:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'camera_init'
        tf.transform.translation.x = 1.0
        tf.transform.rotation.z = math.sin(math.pi / 4)
        tf.transform.rotation.w = math.cos(math.pi / 4)
        node.tf_buffer.set_transform_static(tf, 'test')
        msg = Odometry()
        msg.header.frame_id = 'camera_init'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.orientation.w = 1.0
        from types import SimpleNamespace
        from rclpy.parameter import Parameter
        node.get_publishers_info_by_topic = lambda _: [
            SimpleNamespace(node_name='go2_safety_gate')]
        values = [False, 0.0, 0.3, 0.3, 0.3, 0.3, True]

        def profile_future():
            return SimpleNamespace(result=lambda: SimpleNamespace(values=[
                Parameter(str(i), value=v).get_parameter_value() for i, v in enumerate(values)]))
        node._gate_result(profile_future())
        assert node.gate_verified_at > 0
        values[1] = 0.2
        node._gate_result(profile_future())
        assert node.gate_verified_at == -math.inf
        node._odom(msg)
        assert node.core.robot == pytest.approx((1.0, 1.0, math.pi / 2))
        node._odom(msg)
        assert node.core.state == 'FAULT'
        node._reset(None)
        msg.header.stamp.sec += 10
        node._odom(msg)
        assert node.core.state == 'FAULT'
        node._reset(None)
        node._estop(Int8(data=1))
        assert node.core.state == 'FAULT'
        node._reset(None)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'nonexistent_frame'
        node._odom(msg)
        assert node.core.state == 'FAULT'
    finally:
        node.stop()
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()


def test_gate_profile_allows_continuous_slowing():
    """Exercise existing gate callback with dedicated no-floor approach settings."""
    import yaml
    from pathlib import Path
    from geometry_msgs.msg import TwistStamped
    from go2_keyboard_teleop.safety_gate import ARMED, SafetyGate

    path = Path(__file__).parents[1] / 'config/science_approach_safety_gate.yaml'
    params = yaml.safe_load(path.read_text())['go2_safety_gate']['ros__parameters']
    gate = SafetyGate.__new__(SafetyGate)
    gate.state = ARMED
    gate.max_speed, gate.min_speed = params['max_speed'], params['min_speed']
    gate.max_yaw_rate, gate.max_accel = params['max_yaw_rate'], params['max_accel']
    sent = []
    gate._send = lambda *args: sent.append(args)
    for speed in (0.1, 0.05, 0.01, 0.0):
        gate.last_cmd_time = time.time() - 1
        gate.last_cmd = (speed, 0.0, 0.0)
        msg = TwistStamped()
        msg.twist.linear.x = speed
        gate.cb(msg)
        assert sent[-1][0] == speed

    gate.check_lease = lambda _: None
    gate.cmd_timeout, gate.state_timeout = 0.3, 0.5
    gate.last_cmd_time, gate.last_state_time = time.time() - 1, time.time()
    gate.last_cmd = (0.3, 0.0, 0.0)
    gate._tilt, gate.enable_foot_check = (0.0, 0.0), False
    gate.max_tilt = 0.6
    gate.check()
    assert gate.last_cmd == (0.0, 0.0, 0.0)
    assert sent[-1][:3] == (0.0, 0.0, 0.0)
