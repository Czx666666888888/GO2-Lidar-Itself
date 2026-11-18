#!/usr/bin/env python3
"""往返测试的运动许可、近障停车和返航转身。Sport lease 由项目安全门管理。"""
import math
import os
import signal
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String
from go2_keyboard_teleop.safety_gate import SafetyGate, ARMED, DISARMED, FAULT

from body_footprint import CORNER_RADIUS, HARD_CLEARANCE, PARKING_CLEARANCE, edge_distance


class NavigationGate(SafetyGate):
    def __init__(self):
        super().__init__()
        self.started = time.monotonic()
        self.seen = {}
        self.phase = "WAIT"
        self.ready_to_track = False
        self.finished = None
        self.front_stop_enable = True
        self.front_stop_min, self.front_stop_max = .15, .80
        self.front_stop_deg, self.front_stop_pts = 30, 6
        self.front_blocked = False
        self.body_gap = None
        self.turn_blocked = True
        self._pose = None
        self.rotate = 0.
        self.create_subscription(Odometry, "/base_state_estimation", self.cb_odom, 1)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terrain, 1)
        self.create_subscription(NavPath, "/path", self.cb_path, 1)
        self.create_subscription(PointStamped, "/way_point", self.cb_waypoint, 1)
        self.create_subscription(String, "/navigation_state", self.cb_phase, 1)
        self.create_subscription(Float32, "/escape_rotate", self.cb_rotate, 1)

    def cb_arm(self, msg):
        if not msg.data:
            self._fault("操作员解除武装")

    def cb_waypoint(self, msg):
        self.ready_to_track = True

    def cb_path(self, msg):
        if self.ready_to_track:
            self.seen["path"] = time.monotonic()

    def cb_rotate(self, msg):
        self.rotate = max(-1., min(1., msg.data)) * .45
        self.seen["rotate"] = time.monotonic()

    def cb_phase(self, msg):
        self.phase = msg.data
        self.seen["phase"] = time.monotonic()
        if self.phase in ("DONE", "RETURN_BLOCKED", "RETURNED_EARLY", "PARKING_CLEARANCE_UNCONFIRMED") and self.finished is None:
            self.finished = self.phase
            if self.phase in ("DONE", "RETURNED_EARLY") and (
                self.body_gap is None or self.body_gap < PARKING_CLEARANCE
                or any(time.monotonic() - self.seen.get(key, -math.inf) >= 1. for key in ("odom", "terrain"))
            ):
                self.finished = "PARKING_CLEARANCE_UNCONFIRMED"
            self._fault(self.finished)

    def cb_odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        if (msg.header.frame_id.lstrip("/") not in ("map", "camera_init")
            or not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w))
            or not .99 <= q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w <= 1.01):
            self._pose = self.body_gap = None
            self.seen.pop("odom", None)
            return
        self.seen["odom"] = time.monotonic()
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                      math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z)))

    def cb_terrain(self, msg):
        if self._pose is None or msg.header.frame_id.lstrip("/") != "map":
            self.body_gap = None
            return
        px, py, yaw = self._pose
        rmin = self.front_stop_min; rmax = self.front_stop_max
        half = math.radians(self.front_stop_deg)
        try:
            points = point_cloud2.read_points(msg, field_names=["x", "y", "intensity"], skip_nans=True)
            x, y, height = points["x"], points["y"], points["intensity"]
            known = np.isfinite(x) & np.isfinite(y) & np.isfinite(height) & (height < 1.45)
            observed = bool(known.any())
            obstacles = known & (height >= 0.2)
            dx, dy = x[obstacles] - px, y[obstacles] - py
            distance = np.hypot(dx, dy)
            turns = distance <= CORNER_RADIUS + HARD_CLEARANCE
            turn_blocked = bool(turns.any())
            nearby = distance <= CORNER_RADIUS + PARKING_CLEARANCE + 1e-6
            c, s = math.cos(yaw), math.sin(yaw)
            nearest = min((edge_distance(c*ax+s*ay, -s*ax+c*ay)
                           for ax, ay in zip(dx[nearby], dy[nearby])), default=math.inf)
            front = (height[obstacles] > 0.3) & (distance >= rmin) & (distance <= rmax)
            angle = (np.arctan2(dy[front], dx[front]) - yaw + np.pi) % (2*np.pi) - np.pi
            cnt = np.count_nonzero(np.abs(angle) < half)
        except (AssertionError, ValueError, KeyError, TypeError, IndexError):
            self.body_gap = None
            self.turn_blocked = True
            return
        self.body_gap = nearest if observed else None
        self.turn_blocked = turn_blocked or not observed
        if not observed:
            return
        if nearest <= HARD_CLEARANCE + 1e-6 and self.state == ARMED:
            self.finished = "BODY_CLEARANCE_STOP"
            self._fault(f"机身外沿净空 {nearest:.3f} m，触发硬停")
            return
        self.seen["terrain"] = time.monotonic()
        if not self.front_blocked and cnt > self.front_stop_pts:
            self.front_blocked = True
            self.get_logger().warn(f"前方 {rmax:.2f}m 内有 {cnt} 个障碍点 -> 禁止前进")
        elif self.front_blocked and cnt <= max(1, self.front_stop_pts//2):
            self.front_blocked = False
            self.get_logger().info("前方已清空 -> 恢复前进")

    def cb(self, msg):
        if self.state != ARMED:
            return
        forced_turn = (self.phase == "RETURN" and self.rotate != 0.
                       and time.monotonic() - self.seen["rotate"] < 1.5)
        if self.body_gap is None or self.phase == "HOLD" or (forced_turn and self.turn_blocked):
            self.last_cmd_time = time.time()
            self.last_cmd = (0., 0., 0.)
            self._send(*self.last_cmd)
            return
        if forced_turn:
            msg.twist.linear.x = msg.twist.linear.y = 0.
            msg.twist.angular.z = self.rotate
            self.last_cmd = (0., 0., self.last_cmd[2])
        if self.front_blocked:
            msg.twist.linear.x = min(0., msg.twist.linear.x)
            self.last_cmd = (min(0., self.last_cmd[0]), *self.last_cmd[1:])
        super().cb(msg)

    def check(self):
        super().check()
        now = time.monotonic()
        fresh = all(now - self.seen.get(key, -math.inf) < limit
                    for key, limit in (("terrain", 1.), ("odom", 1.), ("path", 1.), ("phase", 2.5)))
        if self.state == DISARMED:
            if now - self.started > 60.:
                self.finished = "START_TIMEOUT"
            elif (fresh and self.body_gap is not None and self.body_gap > HARD_CLEARANCE + 1e-6
                  and self.phase in ("EXPLORE", "RETURN")
                  and time.time() - self.last_state_time < self.state_timeout
                  and (self.dry_run or self.lease_id)):
                super().cb_arm(Bool(data=True))
        elif self.state == ARMED and (not fresh or now - self.started > 180.):
            self._fault("导航数据超时或任务总时限")
        if self.state == FAULT and self.finished is None:
            self.finished = self._fault_reason


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationGate()
    try:
        while node.finished is None:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        node.finished = "INTERRUPTED"
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.state == ARMED:
            node._fault("导航测试结束")
        node.get_logger().info(f"导航测试结果 {node.finished}")
        if node.finished:
            Path(os.environ["NAV_RESULT"]).write_text(node.finished + "\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
