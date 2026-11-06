"""ROS adapter for manually confirmed approach; defaults to isolated dry-run."""

from dataclasses import fields
import json
import math
import os
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, Int8, Int32, String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from unitree_go.msg import SportModeState

from .approach_control import ApproachControl, Settings


class ScienceTargetApproach(Node):
    """Publish only TwistStamped; delegate all Sport output to safety gate."""

    def __init__(self):
        """Create guarded inputs, explicit confirmation/reset and diagnostics."""
        super().__init__('science_target_approach')
        defaults = Settings()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        for name, value in {'dry_run': True, 'map_frame': 'map',
                            'future_tolerance': 0.10,
                            'log_directory': '/tmp'}.items():
            self.declare_parameter(name, value)
        self.core = ApproachControl(Settings(**{
            f.name: self.get_parameter(f.name).value for f in fields(defaults)}))
        self.dry_run = self.get_parameter('dry_run').value
        self.map_frame = self.get_parameter('map_frame').value
        self.future_tolerance = self.get_parameter('future_tolerance').value
        if not self.dry_run and self.core.p.allow_odom:
            raise ValueError('live APPROACH_ODOM is gated pending stage 2 acceptance')
        log_dir = self.get_parameter('log_directory').value
        self.log_path = os.path.join(log_dir, f'science_approach_{time.time_ns()}.jsonl')
        self.log_file = open(self.log_path, 'x', buffering=1)
        self.log_file.write(json.dumps({'parameters': vars(self.core.p),
                                        'dry_run': self.dry_run}) + '\n')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_stamp = None
        self.last_ros_time = self.get_clock().now().nanoseconds
        self.track_received = -math.inf
        self.last_reported_state = None
        self.gate_verified_at = -math.inf
        self.gate_pending = None
        self.gate_client = None if self.dry_run else self.create_client(
            GetParameters, '/go2_safety_gate/get_parameters')
        self.create_timer(0.5, self._audit_gate)
        self.velocity = self.create_publisher(
            TwistStamped, '/science/approach/cmd_vel_preview' if self.dry_run
            else '/cmd_vel_raw', 1)
        self.telemetry = self.create_publisher(String, '/science/approach/status', 10)
        self.stop_request = None if self.dry_run else self.create_publisher(Int8, '/stop', 1)
        self.create_subscription(String, '/science/target_tracks', self._tracks, 10)
        self.create_subscription(Odometry, '/base_state_estimation', self._odom,
                                 qos_profile_sensor_data)
        self.create_subscription(SportModeState, '/lf/sportmodestate', self._feedback,
                                 qos_profile_sensor_data)
        self.create_subscription(Empty, '/science/approach/confirm', self._confirm, 1)
        self.create_subscription(
            Int32, '/science/approach/confirm_target_id', self._confirm_id, 1)
        self.create_subscription(Empty, '/science/approach/reset', self._reset, 1)
        self.create_subscription(Int8, '/stop', self._estop, 1)
        self.create_timer(1 / self.core.p.control_rate_hz, self._tick)
        self.get_logger().info(
            f'Approach dry_run={self.dry_run}; log={self.log_path}; '
            'waiting for confirmation')

    def _audit_gate(self):
        if self.dry_run or (self.gate_pending is not None and not self.gate_pending.done()):
            return
        if not self.gate_client.service_is_ready():
            return
        request = GetParameters.Request(names=[
            'dry_run', 'min_speed', 'max_speed', 'max_yaw_rate',
            'max_accel', 'cmd_timeout', 'enable_lease'])
        self.gate_pending = self.gate_client.call_async(request)
        self.gate_pending.add_done_callback(self._gate_result)

    def _gate_result(self, future):
        try:
            dry, floor, speed, yaw, accel, timeout, lease = [
                parameter_value_to_python(v) for v in future.result().values]
            values = (floor, speed, yaw, accel, timeout)
            valid = (all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                     and dry is False and lease is True and floor == 0.0
                     and 0 < speed <= self.core.p.max_linear_speed
                     and 0 < yaw <= self.core.p.max_angular_speed
                     and accel >= self.core.p.max_deceleration
                     and 0 < timeout <= self.core.p.braking_latency)
            local = [e.node_name for e in self.get_publishers_info_by_topic(
                '/api/sport/request') if e.node_name != '_CREATED_BY_BARE_DDS_APP_']
            if valid and local == ['go2_safety_gate']:
                self.gate_verified_at = time.monotonic()
            else:
                self.gate_verified_at = -math.inf
        except (TypeError, ValueError, AttributeError, RuntimeError):
            self.gate_verified_at = -math.inf

    def _fresh(self, stamp, limit):
        value = stamp.sec + stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - value
        if value <= 0 or not -self.future_tolerance <= age <= limit:
            raise ValueError(f'stale/future/zero input timestamp: age={age:.6f}s')
        return value, age

    def _tracks(self, msg):
        try:
            data = json.loads(msg.data)
            if data['frame'] != self.map_frame:
                raise ValueError('target map frame mismatch')
            now_ros = self.get_clock().now().nanoseconds
            tracks = data['tracks']
            ids = set()
            for t in tracks:
                if t['id'] in ids or not all(math.isfinite(v) for v in t['position']):
                    raise ValueError('invalid target snapshot')
                ids.add(t['id'])
                t['age'] = (now_ros - t['observed_ns']) / 1e9
                if t['observed_ns'] <= 0 or t['age'] < -self.future_tolerance:
                    raise ValueError('invalid target timestamp')
                t['age'] = max(0.0, t['age'])
                t['lifetime'] = (now_ros - t['first_seen_ns']) / 1e9
            now = time.monotonic()
            self.core.observations(tracks, now)
            self.track_received = now
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            self.core.fault(str(exc))

    def _odom(self, msg):
        try:
            stamp, age = self._fresh(msg.header.stamp, self.core.p.odom_timeout)
            if self.last_stamp is not None and stamp <= self.last_stamp:
                raise ValueError('odometry stamp did not advance')
            if not msg.header.frame_id:
                raise ValueError('empty odometry frame')
            pose = msg.pose.pose
            if msg.header.frame_id != self.map_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp))
                pose = do_transform_pose(pose, transform)
            q, p = pose.orientation, pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                raise ValueError('nonfinite pose')
            if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
                raise ValueError('invalid orientation quaternion')
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            # Sample time, not callback frequency, determines measured velocity.
            self.core.pose(p.x, p.y, yaw, time.monotonic() - age)
            self.last_stamp = stamp
        except (ValueError, TransformException) as exc:
            self.core.fault(f'localization/TF: {exc}')

    def _feedback(self, _msg):
        self.core.feedback_time = time.monotonic()

    def _confirm(self, _msg):
        if not self.dry_run:
            self.get_logger().warning('live confirmation requires explicit target ID')
            return
        self._confirm_id(None)

    def _confirm_id(self, msg):
        now = time.monotonic()
        if not self.dry_run and now - self.gate_verified_at > 1.0:
            self.get_logger().warning('confirmation rejected: gate profile not verified')
            return
        if self.count_publishers('/cmd_vel_raw') > (0 if self.dry_run else 1):
            self.core.fault('competing cmd_vel_raw publishers')
        elif now - self.track_received > self.core.p.visual_timeout:
            self.get_logger().warning('confirmation rejected: target snapshot stale')
        else:
            if self.core.confirm(now, msg.data if msg is not None else None):
                self.get_logger().info(
                    f"LOCK_TARGET id={self.core.lock_id} target={self.core.target}")
            else:
                self.get_logger().warning('confirmation rejected: inputs/state not ready')

    def _reset(self, _msg):
        if self.core.state in ('STOP', 'FAULT'):
            self.core.reset()
            self.last_stamp = None
            self.track_received = -math.inf

    def _estop(self, msg):
        if msg.data and self.core.state not in ('STOP', 'FAULT'):
            self.core.fault('manual emergency stop')

    def _tick(self):
        now_ros = self.get_clock().now().nanoseconds
        if now_ros < self.last_ros_time:
            self.core.fault('ROS clock regressed/map epoch invalid')
        self.last_ros_time = now_ros
        if self.count_publishers('/cmd_vel_raw') > (0 if self.dry_run else 1):
            self.core.fault('competing cmd_vel_raw publishers')
        now = time.monotonic()
        if (not self.dry_run and self.core.state not in ('IDLE', 'STOP', 'FAULT')
                and now - self.gate_verified_at > 1.0):
            self.core.fault('gate profile or sole Sport publisher verification expired')
        report = self.core.step(now)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vehicle'
        msg.twist.linear.x, msg.twist.angular.z = report['vx'], report['wz']
        self.velocity.publish(msg)
        if self.core.state in ('STOP', 'FAULT') and self.stop_request is not None:
            self.stop_request.publish(Int8(data=1))
        report['ros_stamp_ns'] = now_ros
        report['odom_travel_after_loss'] = self.core.odom_travel
        encoded = json.dumps(report, allow_nan=False)
        self.log_file.write(encoded + '\n')
        self.telemetry.publish(String(data=encoded))
        if report['state'] != self.last_reported_state:
            self.get_logger().info(json.dumps(report))
            self.last_reported_state = report['state']

    def stop(self):
        """Best-effort shutdown stop; gate command watchdog covers process death."""
        self.log_file.flush()
        self.velocity.publish(TwistStamped())
        if self.stop_request is not None:
            self.stop_request.publish(Int8(data=1))


def main(args=None):
    """Run until stopped; exceptions and graceful exit request a stop."""
    rclpy.init(args=args)
    node = ScienceTargetApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.log_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
