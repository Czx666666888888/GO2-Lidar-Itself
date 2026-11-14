"""ROS adapter for manually confirmed approach; defaults to isolated dry-run."""

from collections import deque
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
from .grasp_pose_geometry import DockSettings
from .grasp_docking import GraspDocking


class ScienceTargetApproach(Node):
    """Publish only TwistStamped; delegate all Sport output to safety gate."""

    def __init__(self):
        """Create guarded inputs, explicit confirmation/reset and diagnostics."""
        super().__init__('science_target_approach')
        defaults = Settings()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        for name, value in {'dry_run': True, 'map_frame': 'map',
                            'future_tolerance': 0.15, 'camera_frame': 'camera_link',
                            'require_recording': False, 'recording_timeout': 2.0,
                            'log_directory': '/tmp'}.items():
            self.declare_parameter(name, value)
        self.declare_parameter('grasp_docking_enabled', False)
        self.declare_parameter('forward_step_profile_file', '')
        for field in fields(DockSettings()):
            self.declare_parameter('dock_' + field.name, getattr(DockSettings(), field.name))
        self.grasp_docking_enabled = self.get_parameter('grasp_docking_enabled').value
        from .forward_step import load_forward_profile
        profile_path = self.get_parameter('forward_step_profile_file').value
        forward_profile = load_forward_profile(profile_path) if profile_path else None
        docking = GraspDocking(DockSettings(**{
            f.name: self.get_parameter('dock_' + f.name).value for f in fields(DockSettings())
        }), forward_profile=forward_profile) if self.grasp_docking_enabled else None
        if docking and docking.p.forward_step_enabled and forward_profile is None:
            raise ValueError('Forward short steps require a reviewed response profile')
        self.core = ApproachControl(Settings(**{
            f.name: self.get_parameter(f.name).value for f in fields(defaults)}), docking=docking)
        self.dry_run = self.get_parameter('dry_run').value
        self.map_frame = self.get_parameter('map_frame').value
        self.future_tolerance = self.get_parameter('future_tolerance').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.recording_at = -math.inf
        self.recording_info = {}
        self.sport_velocity = None
        self.map_measurements = deque(maxlen=128)
        self.sport_state_info = {}
        log_dir = self.get_parameter('log_directory').value
        self.log_path = os.path.join(log_dir, f'science_approach_{time.time_ns()}.jsonl')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(self.log_path, 'x', buffering=1)
        self.log_file.write(json.dumps({'parameters': vars(self.core.p),
                                        'dry_run': self.dry_run}) + '\n')
        if docking is not None:
            self.log_file.write(json.dumps({
                'grasp_docking_enabled': True, 'docking_parameters': vars(docking.p),
                'vehicle_pose_source': 'exact_stamp_map_to_vehicle_TF',
                's1_base_z': None}) + '\n')
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
        self.create_subscription(String, '/science/map_measurement_debug',
                                 self._map_measurement, 10)
        self.create_subscription(String, '/science/target_tracks', self._tracks, 10)
        self.create_subscription(Odometry, '/base_state_estimation', self._odom,
                                 qos_profile_sensor_data)
        self.create_subscription(SportModeState, '/lf/sportmodestate', self._feedback,
                                 qos_profile_sensor_data)
        self.create_subscription(Empty, '/science/approach/start', self._start, 1)
        self.create_subscription(String, '/science/approach/recording', self._recording, 10)
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
            speed_cap = max(self.core.p.max_linear_speed,
                            self.core.p.search_speed if self.core.p.search_enabled else 0.0)
            valid = (all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                     and dry is False and lease is True and floor == 0.0
                     and 0 < speed <= speed_cap
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

    def _map_measurement(self, msg):
        """Retain exact source-stamp camera ranges; no extra detector or TF shortcut."""
        try:
            data = json.loads(msg.data)
            point = data['map_xyz']
            camera = data['camera_xyz']
            if (data['target_frame'] != self.map_frame or data['stamp_ns'] <= 0 or
                    len(point) != 3 or len(camera) != 3 or camera[2] <= 0 or
                    not all(math.isfinite(v) for v in point+camera)):
                return
            self.map_measurements.append((data['stamp_ns'], tuple(point), math.sqrt(
                sum(v*v for v in camera))))
        except (ValueError, TypeError, KeyError):
            return

    def _matched_camera_range(self, track):
        matches = [distance for stamp, point, distance in self.map_measurements
                   if stamp == track['observed_ns'] and
                   math.dist(point, track['latest_observation']) <= 1e-6]
        return matches[-1] if matches else None

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
                if self.core.p.search_enabled:
                    raw = t.get('latest_observation')
                    if raw is None or len(raw) != 3 or not all(math.isfinite(v) for v in raw):
                        raise ValueError('missing/nonfinite raw MAP observation')
                    distance = self._matched_camera_range(t)
                    if distance is not None:
                        t['camera_range_m'] = distance
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
            if self.grasp_docking_enabled:
                # map->vehicle already includes the sensor offset: never subtract it twice.
                vehicle_tf = self.tf_buffer.lookup_transform(
                    self.map_frame, 'vehicle', rclpy.time.Time.from_msg(msg.header.stamp))
                v = vehicle_tf.transform.translation
                q = vehicle_tf.transform.rotation
                if not all(math.isfinite(n) for n in (v.x, v.y, v.z, q.x, q.y, q.z, q.w)):
                    raise ValueError('nonfinite vehicle TF')
                if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 0.01:
                    raise ValueError('invalid vehicle TF quaternion')
                p.x, p.y, p.z = v.x, v.y, v.z
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            # Sample time, not callback frequency, determines measured velocity.
            sample_time = time.monotonic()-age
            if self.core.p.stop_reference == 'camera_origin':
                camera_tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.camera_frame,
                    rclpy.time.Time.from_msg(msg.header.stamp))
                c = camera_tf.transform.translation
                self.core.camera_pose(c.x, c.y, sample_time)
            self.core.pose(p.x, p.y, yaw, sample_time)
            self.last_stamp = stamp
        except (ValueError, TransformException) as exc:
            self.core.fault(f'localization/TF: {exc}')

    def _feedback(self, msg):
        self.core.feedback_time = time.monotonic()
        self.sport_state_info = {
            'mode': int(msg.mode), 'gait_type': int(msg.gait_type),
            'error_code': int(msg.error_code), 'yaw_speed': float(msg.yaw_speed)}
        velocity = [float(v) for v in msg.velocity]
        self.sport_velocity = velocity if all(math.isfinite(v) for v in velocity) else None
        self.core.feedback_speed = (math.hypot(*velocity[:2])
                                    if self.sport_velocity is not None else math.inf)
        self.core.feedback_yaw_rate = (float(msg.yaw_speed) if math.isfinite(msg.yaw_speed)
                                       else math.inf)
        if self.grasp_docking_enabled and (
                self.sport_velocity is None or not math.isfinite(msg.yaw_speed)):
            self.core.fault('invalid GO2 velocity feedback')

    def _recording(self, msg):
        try:
            data = json.loads(msg.data)
            self.recording_info = data
            if data.get('active') and data.get('frames', 0) >= 10:
                self.recording_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _recording_ready(self):
        return (not self.get_parameter('require_recording').value or
                time.monotonic()-self.recording_at <=
                self.get_parameter('recording_timeout').value)

    def _start(self, _msg):
        now = time.monotonic()
        if not self._recording_ready():
            self.get_logger().warning('START_REJECTED: recording not healthy')
            return
        if not self.dry_run and now-self.gate_verified_at > 1.0:
            self.get_logger().warning('START_REJECTED: gate not verified')
            return
        response_check = (self.grasp_docking_enabled and
                          self.core.docking.p.forward_response_check)
        if response_check and not self.core._is_stopped(now):
            self.get_logger().warning('START_REJECTED: response check requires measured stop')
            return
        if self.core.start(now):
            if response_check:
                self.core.docking.begin_response_check(self.core, now)
            self.get_logger().info('MANUALLY_STARTED locked MAP anchor')
        else:
            self.get_logger().warning('START_REJECTED: lock/pose/TF/feedback not ready')

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
        if self.core.started is not None and self.core.state not in ('STOP', 'FAULT'):
            if not self._recording_ready():
                self.core.fault('recording heartbeat lost')
        report = self.core.step(now)
        report['control_monotonic'] = now
        report['stop_inputs'] = dict(
            body_speed=self.core.actual_speed, camera_speed=self.core.camera_speed,
            sport_speed=self.core.feedback_speed, pose_yaw_rate=self.core.pose_yaw_rate,
            sport_yaw_rate=self.core.feedback_yaw_rate,
            odom_age=now-self.core.odom_time, camera_age=now-self.core.camera_time,
            sport_age=now-self.core.feedback_time)
        report['stop_inputs'] = {
            key: value if math.isfinite(value) else None
            for key, value in report['stop_inputs'].items()}
        report['sport_velocity'] = self.sport_velocity
        report['sport_state'] = self.sport_state_info
        report['camera_frame'] = self.camera_frame
        report['recording'] = self.recording_info
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vehicle'
        msg.twist.linear.x, msg.twist.angular.z = report['vx'], report['wz']
        msg.twist.linear.y = report['vy'] if self.grasp_docking_enabled else 0.0
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
