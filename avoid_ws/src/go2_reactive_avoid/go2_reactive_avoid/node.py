"""Native ROS inputs, read-only observation and the existing Sport safety gate."""
import argparse
import math
from pathlib import Path
import re
import secrets
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

from go2_keyboard_teleop.safety_gate import ARMED, DISARMED, FAULT, SafetyGate

from .controller import CRUISE_SPEED, MAX_YAW_RATE, SLOW_SPEED, Direction, Explorer, STOP, yaw_delta
from .lidar import DEFAULT_LIDAR_HEIGHT, FRAMES, body_points, measure

CLOUD = "/utlidar/cloud"
SPORT_STATE = "/lf/sportmodestate"
SPORT_REQUEST = "/api/sport/request"
LEASE_REQUEST = "/api/sport_lease/request"
STOP_TOPIC = "/go2_reactive_avoid/stop"
MAX_AGE = 0.40
START_TIMEOUT = 15.0
RUN_TIMEOUT = 90.0
WRITER_SETTLE = 2.0
SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
SPORT_MONITOR_QOS = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)


def load_verified_writers(path):
    if path is None or not path.exists():
        return frozenset()
    gids = frozenset(line.partition("#")[0].strip() for line in path.read_text().splitlines()) - {""}
    if any(re.fullmatch(r"[0-9a-f]{48}", gid) is None for gid in gids):
        raise ValueError("机载 Sport 端点文件必须每行填写一个已核实的 24 字节 DDS GID")
    return gids


class Inputs:
    def __init__(self, node, lidar_height):
        self.height = lidar_height
        self.yaw = self.previous_yaw = None
        self.scan = None
        self.attitude = None
        self.still = False
        self.scan_at = self.state_at = -math.inf
        self.cloud_stamp = self.state_stamp = 0
        self.cloud_clock = self.state_clock = None
        self.cloud_count = self.state_count = 0
        self.sport_error_code = None
        self.cloud_message = "等待雷达"
        self.state_message = "等待 Sport 状态"
        node.create_subscription(PointCloud2, CLOUD, self.cloud, SENSOR_QOS)

    def sport(self, msg):
        now = time.monotonic()
        roll, pitch, yaw = msg.imu_state.rpy
        vx, vy, _ = msg.velocity
        wz = msg.yaw_speed
        stamp = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        if stamp <= 0 or stamp < self.state_stamp:
            self.attitude = None
            self.state_message = "Sport 时间戳无效或回退"
            return
        if stamp == self.state_stamp:
            return
        self.state_stamp = stamp
        source_time = stamp / 1e9
        offset = now - source_time
        self.state_clock = offset if self.state_clock is None else min(offset, self.state_clock)
        if (offset - self.state_clock >= MAX_AGE
                or not all(math.isfinite(value) for value in (roll, pitch, yaw, vx, vy, wz))
                or max(abs(roll), abs(pitch)) > 0.25):
            self.attitude = None
            self.state_message = "Sport 姿态、速度或时间异常"
            return
        delta = 0.0 if self.previous_yaw is None else yaw_delta(yaw, self.previous_yaw)
        if abs(delta) > 0.50:
            self.attitude = None
            self.state_message = "Sport 航向跳变"
            return
        self.yaw = yaw if self.yaw is None else self.yaw + delta
        self.previous_yaw = yaw
        self.attitude = (roll, pitch)
        self.sport_error_code = msg.error_code
        self.still = math.hypot(vx, vy) < 0.035 and abs(wz) < 0.08
        self.state_at = source_time + self.state_clock
        self.state_count += 1
        self.state_message = "正常"

    def cloud(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp == self.cloud_stamp and stamp > 0:
            return
        self.scan = None
        if stamp <= 0 or stamp < self.cloud_stamp:
            self.cloud_message = "雷达时间戳无效或回退"
            return
        self.cloud_stamp = stamp
        offset = now - stamp / 1e9
        self.cloud_clock = offset if self.cloud_clock is None else min(offset, self.cloud_clock)
        if offset - self.cloud_clock >= MAX_AGE:
            self.cloud_message = "雷达时间延迟超限"
            return
        if msg.header.frame_id.lstrip("/") not in FRAMES or msg.row_step != msg.point_step * msg.width:
            self.cloud_message = f"未知雷达坐标系或点云布局 {msg.header.frame_id!r}"
            return
        if self.attitude is None or now - self.state_at >= MAX_AGE:
            self.cloud_message = "缺少新鲜姿态"
            return
        try:
            data = point_cloud2.read_points(msg, field_names=["x", "y", "z"])
            points = np.column_stack((data["x"], data["y"], data["z"]))
            self.scan = measure(body_points(points, *self.attitude, self.height))
        except (AssertionError, ValueError, KeyError, TypeError, IndexError) as error:
            self.cloud_message = f"点云解析失败 {error}"
            return
        if self.scan is None:
            self.cloud_message = "地面回波不足，检查安装高度与平整地面"
            return
        self.scan_at = stamp / 1e9 + self.cloud_clock
        self.cloud_count += 1
        self.cloud_message = "正常"

    def ready(self, now):
        return (self.scan is not None and self.attitude is not None
                and now - min(self.scan_at, self.state_at) < MAX_AGE)

    def status(self, now):
        if not self.ready(now):
            cloud_status = "超时" if self.cloud_message == "正常" and now - self.scan_at >= MAX_AGE else self.cloud_message
            state_status = "超时" if self.state_message == "正常" and now - self.state_at >= MAX_AGE else self.state_message
            return f"数据未就绪 雷达={cloud_status} Sport={state_status}"
        scan = self.scan
        sectors = " ".join(
            f"{label}={getattr(scan, key).value}"
            for label, key in (("前", "front"), ("右", "right"), ("左", "left"), ("后", "rear"))
        )
        gap = f"{scan.front_gap:.2f}m" if math.isfinite(scan.front_gap) else "超出量测窗"
        return (f"{sectors} 前缘间距={gap} 机身净空={scan.body_clear} 可旋转={scan.can_rotate} "
                f"航向={math.degrees(self.yaw):+.1f}° Sport.error_code={self.sport_error_code}")


class AvoidGate(SafetyGate):
    def __init__(self, lidar_height, verified_writers=frozenset()):
        super().__init__()
        self.inputs = Inputs(self, lidar_height)
        self.destroy_subscription(self.sub_state)
        self.sub_state = self.create_subscription(SportModeState, SPORT_STATE, self.cb_state, SENSOR_QOS)
        self.explorer = Explorer()
        self.started = time.monotonic()
        self.armed_at = None
        self.armed_error_code = None
        self.result = None
        self.last_status = -math.inf
        self.phase = ""
        self.last_stop = -math.inf
        self.verified_writers = verified_writers
        self.own_sport_gid = None
        self.writers_since = None
        # Humble's Python subscription callback exposes the message only.
        # Give locally generated Sport requests a session-specific identity range.
        self.first_request_id = secrets.randbits(62) + 1
        self._req_id = self.first_request_id
        self.sport_monitor = self.create_subscription(
            Request, SPORT_REQUEST, self.on_sport_request, SPORT_MONITOR_QOS)

    def cb_state(self, msg):
        self.inputs.sport(msg)
        if self.inputs.attitude is not None:
            super().cb_state(msg)

    def cb_arm(self, msg):
        if not msg.data:
            self.finish("OPERATOR_STOP")

    def cb_stop(self, msg):
        if msg.data:
            self.finish("OPERATOR_STOP")

    def cb(self, msg):
        # The local state machine owns the gate's motion input.
        pass

    def send_motion(self, command):
        vx, wz = command
        if vx > 0.0 and self.last_cmd[0] < SLOW_SPEED:
            # Start at the gait's usable speed, then ramp from there to the target.
            self.last_cmd = (SLOW_SPEED, *self.last_cmd[1:])
        msg = TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        super().cb(msg)

    def finish(self, reason):
        if self.result is None:
            self.result = reason
            if self.armed_at is not None:
                self._fault(reason)
            self.get_logger().warn(reason)

    def on_sport_request(self, msg):
        request_id = msg.header.identity.id
        own_request = (self.first_request_id <= request_id < self._req_id
                       and msg.header.lease.id == self.lease_id)
        if not own_request:
            self.finish(f"收到其他 Sport 请求 api={msg.header.identity.api_id} id={request_id}")

    def check_writers(self, now):
        own = []
        for writer in self.get_publishers_info_by_topic(SPORT_REQUEST):
            gid = bytes(writer.endpoint_gid).hex()
            if writer.node_name == self.get_name() and writer.node_namespace == self.get_namespace():
                own.append(gid)
            elif gid not in self.verified_writers or writer.qos_profile.reliability != ReliabilityPolicy.RELIABLE:
                self.finish(f"发现未核实的 Sport 发布者 {writer.node_name} gid={gid}")
                return False
        lease_writers = self.count_publishers(LEASE_REQUEST)
        if len(own) > 1 or lease_writers > 1:
            self.finish(f"检测到重复避障或 lease 发布者，避障={len(own)} lease={lease_writers}")
            return False
        gid = own[0] if own else None
        if self.armed_at is not None and (gid is None or gid != self.own_sport_gid or lease_writers != 1):
            self.finish("本机 Sport 或 lease 发布端点丢失")
            return False
        if gid is None or lease_writers == 0:
            self.writers_since = None
            self.own_sport_gid = None
        elif self.writers_since is None or gid != self.own_sport_gid:
            self.writers_since = now
            self.own_sport_gid = gid
        return self.writers_since is not None and now - self.writers_since >= WRITER_SETTLE

    def check_lease(self, now):
        monotonic_now = time.monotonic()
        if self.result is None and self.inputs.ready(monotonic_now):
            super().check_lease(now)

    def check(self):
        if self.result is not None:
            return
        now = time.monotonic()
        if not self.check_writers(now):
            if self.result is None and now - self.started >= START_TIMEOUT:
                self.finish("启动预检超时，Sport 或 lease 发布端点未就绪")
            return
        super().check()
        if self.state == FAULT:
            self.finish(self._fault_reason)
            return
        ready = self.inputs.ready(now)
        scan = self.inputs.scan
        if now - self.last_status >= 1.0:
            self.get_logger().info(self.inputs.status(now))
            self.last_status = now
        if self.state == DISARMED:
            if now - self.started >= START_TIMEOUT:
                self.finish(f"启动预检超时 lease={bool(self.lease_id)} 静止={self.inputs.still} "
                            f"指令接收端={self.pub.get_subscription_count()} {self.inputs.status(now)}")
            elif (ready and self.inputs.cloud_count >= 5
                  and self.inputs.state_count >= 5 and self.inputs.still and scan.body_clear
                  and scan.front is not Direction.UNKNOWN and self.lease_id
                  and self.pub.get_subscription_count() > 1):
                super().cb_arm(Bool(data=True))
                self.armed_at = now
                self.armed_error_code = self.inputs.sport_error_code
            return
        if not ready or self.pub.get_subscription_count() <= 1:
            self.finish("雷达、Sport 反馈或命令接收端丢失")
            return
        if self.inputs.sport_error_code != self.armed_error_code:
            self.finish(f"Sport.error_code 已变化 {self.armed_error_code} -> {self.inputs.sport_error_code}")
            return
        if now - self.armed_at >= RUN_TIMEOUT:
            self.finish("TIME_LIMIT")
            return
        observation = min(self.inputs.scan_at, self.inputs.state_at)
        command = self.explorer.update(now, observation, scan, self.inputs.yaw, self.inputs.still)
        if self.explorer.completed_turn:
            self.get_logger().info(self.explorer.completed_turn)
            self.explorer.completed_turn = ""
        if self.explorer.phase != self.phase:
            self.phase = self.explorer.phase
            detail = self.explorer.turn_label if self.phase in ("TURN", "MICRO") else ""
            self.get_logger().info(f"{self.phase} {detail} | {self.inputs.status(now)}")
        if self.phase == "HALTED":
            self.finish(self.explorer.reason)
        elif command == STOP:
            self.last_cmd_time = time.time()
            if self.last_cmd != (0.0, 0.0, 0.0) or now - self.last_stop >= 0.15:
                self.last_cmd = (0.0, 0.0, 0.0)
                self._send(*self.last_cmd)
                self.last_stop = now
        else:
            self.send_motion(command)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("observe", "execute"))
    parser.add_argument("--lidar-height", type=float, default=DEFAULT_LIDAR_HEIGHT)
    parser.add_argument("--verified-sport-writers", type=Path)
    args = parser.parse_args()
    if not 0.20 <= args.lidar_height <= 0.55:
        parser.error("雷达站立离地高度须为 0.20–0.55 m")
    execute = args.mode == "execute"
    try:
        verified_writers = load_verified_writers(args.verified_sport_writers)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    ros_args = ["--ros-args", "-r", "__node:=go2_reactive_avoid",
                "-p", "dry_run:=false", "-p", f"max_speed:={CRUISE_SPEED}",
                "-p", f"min_speed:={SLOW_SPEED}", "-p", f"max_yaw_rate:={MAX_YAW_RATE}",
                "-p", "max_tilt:=0.25",
                "-p", "cmd_timeout:=0.25", "-p", "state_timeout:=0.40",
                "-p", f"stop_topic:={STOP_TOPIC}", "-p", "arm_topic:=/go2_reactive_avoid/arm",
                "-p", "cmd_vel_topic:=/go2_reactive_avoid/cmd_vel_raw"] if execute else []
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    interrupted = False

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        if execute:
            node = AvoidGate(args.lidar_height, verified_writers)
            while not interrupted and node.result is None:
                rclpy.spin_once(node, timeout_sec=0.05)
            if interrupted:
                node.finish("OPERATOR_STOP")
            return 0 if node.result in ("OPERATOR_STOP", "TIME_LIMIT") else 1

        node = Node("go2_reactive_avoid_observe")
        inputs = Inputs(node, args.lidar_height)
        node.create_subscription(SportModeState, SPORT_STATE, inputs.sport, SENSOR_QOS)
        start = last_status = time.monotonic()
        while not interrupted and time.monotonic() - start < 10.0:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_status >= 1.0:
                node.get_logger().info(inputs.status(now))
                last_status = now
        node.get_logger().info(f"有效雷达 {inputs.cloud_count} 帧，Sport {inputs.state_count} 帧")
        return 0 if inputs.ready(time.monotonic()) and inputs.cloud_count >= 5 else 1
    finally:
        if execute and node is not None and node.armed_at is not None:
            for _ in range(3):
                node._send(0.0, 0.0, 0.0, "退出避障测试")
                time.sleep(0.05)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
