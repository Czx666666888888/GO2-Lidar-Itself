#!/usr/bin/env python3
"""WP4.0 命令桥安全门 (串联拦截, 唯一授权发布者)

核心整改:
  1. 订阅 TwistStamped (/cmd_vel_raw, 与 pathFollower 输出一致)
  2. 状态机: DISARMED -> ARMED -> FAULT, 默认 DISARMED(不发命令)
  3. 只有本节点发布 /api/sport/request (pathFollower 禁止直接发)
  4. 心跳超时(sportmodestate 丢失 -> FAULT)
  5. 显式复位(/arm=true 从 FAULT 回 ARMED)
  6. 速度回调首查状态, FAULT 时不发命令

用法:
  ros2 run go2_keyboard_teleop go2_safety_gate                      # 默认 DISARMED
  ros2 run go2_keyboard_teleop go2_safety_gate --ros-args -p dry_run:=false
  # 武装: ros2 topic pub /arm std_msgs/Bool "{data: true}" --once
"""
import json
import os
import socket
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Int8
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

API_MOVE = 1008
API_STOP_MOVE = 1003
API_LEASE_APPLY = 101
API_LEASE_RENEW = 102
ERR_LEASE_NOT_EXIST = 3206

# 状态机
DISARMED = 0   # 未武装, 不发任何命令
ARMED = 1      # 武装, 允许发命令
FAULT = 2      # 故障/急停, 锁存, 需显式复位


class SafetyGate(Node):
    def __init__(self):
        super().__init__("go2_safety_gate")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")
        self.declare_parameter("sport_topic", "/api/sport/request")
        self.declare_parameter("sportmode_topic", "/lf/sportmodestate")
        self.declare_parameter("arm_topic", "/arm")
        self.declare_parameter("stop_topic", "/stop")
        self.declare_parameter("max_speed", 0.1)       # 默认低速(首次闭环)
        self.declare_parameter("min_speed", 0.2)       # 最小线速(过小狗不动, 非零则抬到±min)
        self.declare_parameter("max_yaw_rate", 0.3)
        self.declare_parameter("max_accel", 0.5)
        self.declare_parameter("cmd_timeout", 0.5)     # 无 cmd_vel 超时
        self.declare_parameter("state_timeout", 1.0)   # 无 sportmodestate 超时(心跳)
        self.declare_parameter("max_tilt", 0.6)
        self.declare_parameter("foot_force_thre", 1.0)
        self.declare_parameter("enable_foot_check", False)  # 足端互锁默认关(Go2 foot_force 不可靠)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("enable_lease", True)
        self.declare_parameter("lease_retry_interval", 0.5)
        self.declare_parameter("log_interval", 0.5)   # 真实模式速度指令日志限流间隔(s)

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        sport_topic = self.get_parameter("sport_topic").value
        sportmode_topic = self.get_parameter("sportmode_topic").value
        arm_topic = self.get_parameter("arm_topic").value
        stop_topic = self.get_parameter("stop_topic").value
        self.max_speed = self.get_parameter("max_speed").value
        self.min_speed = self.get_parameter("min_speed").value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate").value
        self.max_accel = self.get_parameter("max_accel").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.state_timeout = self.get_parameter("state_timeout").value
        self.max_tilt = self.get_parameter("max_tilt").value
        self.foot_force_thre = self.get_parameter("foot_force_thre").value
        self.enable_foot_check = self.get_parameter("enable_foot_check").value
        self.dry_run = self.get_parameter("dry_run").value
        self.enable_lease = self.get_parameter("enable_lease").value
        self.lease_retry_interval = self.get_parameter("lease_retry_interval").value
        self.log_interval = self.get_parameter("log_interval").value

        self.sub = self.create_subscription(TwistStamped, cmd_topic, self.cb, 10)
        self.sub_state = self.create_subscription(
            SportModeState, sportmode_topic, self.cb_state, 10)
        self.sub_arm = self.create_subscription(Bool, arm_topic, self.cb_arm, 10)
        self.sub_stop = self.create_subscription(Int8, stop_topic, self.cb_stop, 10)
        self.pub = self.create_publisher(Request, sport_topic, 10)
        self.lease_pub = self.create_publisher(
            Request, "/api/sport_lease/request", 10)
        self.lease_sub = self.create_subscription(
            Response, "/api/sport_lease/response", self.cb_lease_response, 10)

        self.state = DISARMED            # 默认不运动
        self.state_name = {DISARMED: "DISARMED", ARMED: "ARMED", FAULT: "FAULT"}
        self._fault_reason = ""
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = time.time()
        self.last_state_time = 0.0       # 上次收到 sportmodestate 时间
        self._req_id = 0
        self.lease_id = 0
        self.lease_term = 1.0
        self._lease_last_ok = 0.0
        self._lease_next_action = 0.0
        self._lease_pending_api = 0
        self._lease_pending_since = 0.0
        self._lease_context = (
            f"{socket.gethostname()}/sport/{os.getpid()}")
        self._tilt = (0.0, 0.0)
        self._foot_force = (0, 0, 0, 0)
        self._last_log_time = 0.0

        self.timer = self.create_timer(0.01, self.check)

        mode = "DRY-RUN" if self.dry_run else "真机"
        self.get_logger().info(
            f"WP4.0 安全门 [{mode}] 状态={self.state_name[self.state]} "
            f"| 限速 {self.max_speed}m/s/{self.max_yaw_rate}rad/s "
            f"| cmd超时{self.cmd_timeout}s 心跳超时{self.state_timeout}s")
        self.get_logger().info(
            "默认 DISARMED(不发命令). 武装: ros2 topic pub /arm std_msgs/Bool '{data: true}'")
        if not self.dry_run and self.enable_lease:
            self.get_logger().info(
                f"Sport lease 已启用，正在申请: {self._lease_context}")

    # ---- 状态机 ----
    def _set_state(self, s, reason=""):
        if self.state != s:
            self.get_logger().warn(
                f"状态 {self.state_name[self.state]} -> {self.state_name[s]} {reason}")
            self.state = s
        self._fault_reason = reason

    def _fault(self, reason):
        self._set_state(FAULT, reason)
        self.last_cmd = (0.0, 0.0, 0.0)
        self._send(0.0, 0.0, 0.0, reason)

    def cb_arm(self, msg: Bool):
        """显式武装/复位: true -> ARMED(需先有状态), false -> DISARMED"""
        if msg.data:
            if self.last_state_time == 0.0:
                self.get_logger().warn("无状态(sportmodestate), 拒绝武装")
                return
            if not self.dry_run and self.enable_lease and self.lease_id == 0:
                self.get_logger().warn("无有效 Sport lease, 拒绝武装")
                return
            self._set_state(ARMED, "(武装/复位)")
        else:
            self._set_state(DISARMED, "(解除武装)")
            self.last_cmd = (0.0, 0.0, 0.0)
            self._send(0.0, 0.0, 0.0, "解除武装")

    def cb_stop(self, msg: Int8):
        if msg.data != 0:
            self._fault("独立停止")

    # ---- 数据回调 ----
    def _clamp(self, v, vmax):
        return max(-vmax, min(vmax, v))

    def _accel_limit(self, cur, target, dt):
        step = self.max_accel * dt
        if target > cur + step:
            return cur + step
        if target < cur - step:
            return cur - step
        return target

    def _send(self, vx, vy, vyaw, reason=""):
        if self.dry_run:
            tag = "StopMove" if (vx == 0 and vy == 0 and vyaw == 0) else "Move"
            self.get_logger().info(
                f">>> [dry-run] {tag} vx={vx:+.3f} vy={vy:+.3f} vyaw={vyaw:+.3f}"
                + (f" [{reason}]" if reason else ""))
            return
        if self.enable_lease and self.lease_id == 0:
            # 没有控制权时绝不发送 Move。StopMove 仍以 lease=0 尝试发送，
            # 作为控制权异常时的最后安全兜底。
            if vx != 0 or vy != 0 or vyaw != 0:
                self.get_logger().error("无有效 Sport lease，拦截 Move")
                return
        # 真实模式也记录速度指令(限流, 用于定位"指令层 vs 执行层"速度差)
        now = time.time()
        if now - self._last_log_time >= self.log_interval:
            self._last_log_time = now
            tag = "StopMove" if (vx == 0 and vy == 0 and vyaw == 0) else "Move"
            self.get_logger().info(
                f"{tag} vx={vx:+.3f} vy={vy:+.3f} vyaw={vyaw:+.3f}"
                + (f" [{reason}]" if reason else ""))
        req = Request()
        req.header.identity.id = self._req_id
        self._req_id += 1
        req.header.lease.id = self.lease_id
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        if vx == 0 and vy == 0 and vyaw == 0:
            req.header.identity.api_id = API_STOP_MOVE
        else:
            req.header.identity.api_id = API_MOVE
            req.parameter = json.dumps({"x": vx, "y": vy, "z": vyaw})
        self.pub.publish(req)

    # ---- Unitree Sport lease ----
    def _publish_lease_request(self, api_id):
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        req.header.identity.api_id = api_id
        req.header.lease.id = self.lease_id if api_id == API_LEASE_RENEW else 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = (json.dumps({"name": self._lease_context})
                         if api_id == API_LEASE_APPLY else "{}")
        self.lease_pub.publish(req)
        self._lease_pending_api = api_id
        self._lease_pending_since = time.time()

    def cb_lease_response(self, msg: Response):
        api_id = msg.header.identity.api_id
        if api_id not in (API_LEASE_APPLY, API_LEASE_RENEW):
            return
        if msg.header.status.code != 0:
            self.get_logger().error(
                f"Sport lease API {api_id} 失败 code={msg.header.status.code}")
            self._lease_pending_api = 0
            self._lease_next_action = time.time() + self.lease_retry_interval
            if msg.header.status.code == ERR_LEASE_NOT_EXIST:
                self._lose_lease("Sport lease 不存在")
            return

        now = time.time()
        if api_id == API_LEASE_APPLY:
            try:
                data = json.loads(msg.data)
                lease_id = int(data["id"])
                term = float(data["term"]) / 1_000_000.0
                if lease_id == 0:
                    raise ValueError("lease id 为0")
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                self.get_logger().error(f"Sport lease 响应无效: {exc}; data={msg.data!r}")
                self._lease_pending_api = 0
                self._lease_next_action = now + self.lease_retry_interval
                return
            self.lease_id = lease_id
            self.lease_term = term if term > 0 else 1.0
            self.get_logger().info(
                f"Sport lease 已取得 id={self.lease_id} term={self.lease_term:.3f}s")

        self._lease_last_ok = now
        self._lease_pending_api = 0
        self._lease_next_action = now + max(self.lease_term * 0.3, 0.05)

    def _lose_lease(self, reason):
        old_id = self.lease_id
        if self.state == ARMED:
            # 使用最后一个lease先发StopMove，再清除控制权并锁存FAULT。
            self._send(0.0, 0.0, 0.0, reason)
            self._set_state(FAULT, reason)
        self.lease_id = 0
        self._lease_last_ok = 0.0
        self._lease_pending_api = 0
        self._lease_next_action = time.time() + self.lease_retry_interval
        if old_id:
            self.get_logger().error(f"Sport lease 已丢失 id={old_id}: {reason}")

    def check_lease(self, now):
        if self.dry_run or not self.enable_lease:
            return
        if self.lease_id and self._lease_last_ok and \
                now - self._lease_last_ok > max(self.lease_term, 0.3):
            self._lose_lease("Sport lease 续租超时")
            return
        if self._lease_pending_api:
            if now - self._lease_pending_since <= 1.0:
                return
            self.get_logger().error(
                f"Sport lease API {self._lease_pending_api} 响应超时")
            self._lease_pending_api = 0
            self._lease_next_action = now + self.lease_retry_interval
            return
        if now < self._lease_next_action:
            return
        self._publish_lease_request(
            API_LEASE_RENEW if self.lease_id else API_LEASE_APPLY)

    def cb_state(self, msg: SportModeState):
        self.last_state_time = time.time()
        self._tilt = (msg.imu_state.rpy[0], msg.imu_state.rpy[1])
        self._foot_force = tuple(msg.foot_force)

    def cb(self, msg: TwistStamped):
        now = time.time()
        # 首查状态: 非 ARMED 不发命令(修复"急停被后续速度覆盖")
        if self.state != ARMED:
            return
        dt = max(now - self.last_cmd_time, 0.001)
        self.last_cmd_time = now

        tvx = self._clamp(msg.twist.linear.x, self.max_speed)
        # 最小线速: 非零但过小 → 抬到 ±min_speed(狗对过小速度不响应)
        if 0.0 < abs(tvx) < self.min_speed:
            floor_speed = min(self.min_speed, self.max_speed)
            tvx = floor_speed if tvx > 0 else -floor_speed
        tvy = self._clamp(msg.twist.linear.y, self.max_speed)
        tvyaw = self._clamp(msg.twist.angular.z, self.max_yaw_rate)

        cvx, cvy, cvyaw = self.last_cmd
        nvx = self._accel_limit(cvx, tvx, dt)
        nvy = self._accel_limit(cvy, tvy, dt)
        nvyaw = self._accel_limit(cvyaw, tvyaw, dt)

        if abs(nvx) < 0.005: nvx = 0.0
        if abs(nvy) < 0.005: nvy = 0.0
        if abs(nvyaw) < 0.005: nvyaw = 0.0

        self.last_cmd = (nvx, nvy, nvyaw)
        self._send(nvx, nvy, nvyaw)

    def check(self):
        """定时检查: 心跳超时 / cmd超时 / 倾角 / 足端"""
        # 控制权必须在武装前取得；DISARMED 期间也要持续申请/续租。
        now = time.time()
        self.check_lease(now)
        if self.state == DISARMED:
            return
        # 心跳超时: sportmodestate 丢失 -> FAULT
        if self.state == ARMED and now - self.last_state_time > self.state_timeout:
            self._fault(f"状态心跳超时({self.state_timeout}s 无 sportmodestate)")
            return
        # cmd 超时: 无新速度指令 -> 归零
        if self.state == ARMED and now - self.last_cmd_time > self.cmd_timeout:
            if self.last_cmd != (0.0, 0.0, 0.0):
                self.last_cmd = (0.0, 0.0, 0.0)
                self._send(0.0, 0.0, 0.0, "cmd超时归零")
        # 倾角互锁
        roll, pitch = self._tilt
        if self.state == ARMED and (abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt):
            self._fault(f"倾角超限 roll={roll:.2f} pitch={pitch:.2f}")
            return
        # 足端互锁(默认关: Go2 foot_force 常为0, 数据不可靠)
        if self.state == ARMED and self.enable_foot_check and \
                all(abs(f) < self.foot_force_thre for f in self._foot_force):
            self._fault(f"足端失稳 {self._foot_force}")
            return


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
