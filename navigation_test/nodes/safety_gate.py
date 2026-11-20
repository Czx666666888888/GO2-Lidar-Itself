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
from std_msgs.msg import Bool, Float32, Int8, String
from go2_keyboard_teleop.safety_gate import SafetyGate, ARMED, DISARMED, FAULT

from body_footprint import CORNER_RADIUS, PARKING_CLEARANCE, edge_distance


FRONT_MIN, FRONT_MAX = .15, .80
FRONT_HALF_ANGLE = math.radians(30)
FRONT_TRIGGER_POINTS = 10
SIDE_OBSERVATION_RANGE = .50
SIDE_TRIGGER_DISTANCE = .30
SIDE_SLIDE_SPEED = .10
TURN_ANGLE = math.pi / 2
TURN_TOLERANCE = math.radians(3)
TURN_TIMEOUT = 10.
STILL_TIME = .25


class NavigationGate(SafetyGate):
    def __init__(self):
        super().__init__()
        self.started = time.monotonic()
        self.seen = {}
        self.phase = "WAIT"
        self.ready_to_track = False
        self.finished = None
        self.exit_requested = False
        self._pose = None
        self.rotate = 0.
        self._sectors = None
        self._parking_points = None
        self._turn_target = None
        self._turn_deadline = 0.
        self._still_since = None
        self._sport_still = None
        self._planner_stop = False
        self.create_subscription(Odometry, "/base_state_estimation", self.cb_odom, 1)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terrain, 1)
        self.create_subscription(NavPath, "/path", self.cb_path, 1)
        self.create_subscription(PointStamped, "/way_point", self.cb_waypoint, 1)
        self.create_subscription(String, "/navigation_state", self.cb_phase, 1)
        self.create_subscription(Float32, "/escape_rotate", self.cb_rotate, 1)
        self.create_subscription(Int8, "/stop", self.cb_planner_stop, 5)

    def should_exit(self):
        return self.exit_requested or self.finished is not None

    def cb_stop(self, msg):
        if msg.data != 0:
            super().cb_stop(msg)
            self.finished = self.finished or "OPERATOR_STOP"
            self.exit_requested = True

    def cb_arm(self, msg):
        if not msg.data:
            self._fault("操作员解除武装")

    def cb_planner_stop(self, msg):
        self._planner_stop = bool(msg.data)
        if self._planner_stop and self.state == ARMED:
            self._stop_motion()

    def cb_state(self, msg):
        vx, vy, _ = msg.velocity
        wz = msg.yaw_speed
        roll, pitch, _ = msg.imu_state.rpy
        if not all(math.isfinite(value) for value in (vx, vy, wz, roll, pitch)):
            self._sport_still = None
            return
        super().cb_state(msg)
        self._sport_still = math.hypot(vx, vy) < .035 and abs(wz) < .08

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
            if self.phase in ("DONE", "RETURNED_EARLY") and not self._parking_confirmed():
                self.finished = "PARKING_CLEARANCE_UNCONFIRMED"
            self._fault(self.finished)

    def cb_odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        if (msg.header.frame_id.lstrip("/") not in ("map", "camera_init")
            or not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w))
            or not .99 <= q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w <= 1.01):
            self._pose = self._sectors = self._parking_points = None
            self.seen.pop("odom", None)
            return
        self.seen["odom"] = time.monotonic()
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                      math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z)))

    def _parking_confirmed(self):
        now = time.monotonic()
        if (self._pose is None or self._parking_points is None
                or any(now - self.seen.get(key, -math.inf) >= 1. for key in ("odom", "terrain"))):
            return False
        px, py, yaw = self._pose
        dx, dy = self._parking_points - [[px], [py]]
        nearby = np.hypot(dx, dy) <= CORNER_RADIUS + PARKING_CLEARANCE
        c, s = math.cos(yaw), math.sin(yaw)
        return all(edge_distance(c*x + s*y, -s*x + c*y) + 1e-6 >= PARKING_CLEARANCE
                   for x, y in zip(dx[nearby], dy[nearby]))

    def cb_terrain(self, msg):
        was_front_blocked = self._sectors is not None and self._sectors["front_blocked"]
        self._sectors = self._parking_points = None
        self.seen.pop("terrain", None)
        if self._pose is None or msg.header.frame_id.lstrip("/") != "map":
            return
        try:
            points = point_cloud2.read_points(msg, field_names=["x", "y", "intensity"], skip_nans=True)
            x, y, height = points["x"], points["y"], points["intensity"]
            known = np.isfinite(x) & np.isfinite(y) & np.isfinite(height) & (height < 1.45)
            if not known.any():
                return
            x, y, height = x[known], y[known], height[known]
        except (AssertionError, ValueError, KeyError, TypeError, IndexError):
            return

        px, py, yaw = self._pose
        dx, dy = x - px, y - py
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c*dx + s*dy, -s*dx + c*dy
        distance = np.hypot(bx, by)
        angle = np.arctan2(by, bx)
        obstacles = height >= .20
        front = (bx > FRONT_MIN) & (distance <= FRONT_MAX) & (abs(angle) <= FRONT_HALF_ANGLE)
        side = (distance <= SIDE_OBSERVATION_RANGE) & (abs(angle) > FRONT_HALF_ANGLE) & (
            abs(angle) < math.pi - FRONT_HALF_ANGLE)
        left, right = side & (by > 0), side & (by < 0)
        left_dist = float(np.min(distance[left & obstacles], initial=math.inf))
        right_dist = float(np.min(distance[right & obstacles], initial=math.inf))
        front_count = int(np.count_nonzero(front & obstacles))
        self._sectors = {
            "front_seen": bool(front.any()),
            "front_blocked": front_count >= FRONT_TRIGGER_POINTS,
            "left_seen": bool(left.any()),
            "right_seen": bool(right.any()),
            "left_close": left_dist <= SIDE_TRIGGER_DISTANCE + 1e-6,
            "right_close": right_dist <= SIDE_TRIGGER_DISTANCE + 1e-6,
        }
        self._parking_points = np.array([x[obstacles], y[obstacles]])
        self.seen["terrain"] = time.monotonic()
        if self._sectors["front_blocked"]:
            if self.state == ARMED and not was_front_blocked and self._turn_target is None:
                self._stop_motion()
            self.get_logger().warn(
                f"前方障碍 {front_count} 点 | 左 {left_dist:.2f}m 右 {right_dist:.2f}m",
                throttle_duration_sec=1.0)

    def _fresh(self, now):
        return all(now - self.seen.get(key, -math.inf) < limit
                   for key, limit in (("terrain", 1.), ("odom", 1.), ("path", 1.), ("phase", 2.5)))

    def _stop_motion(self):
        self.last_cmd = (0., 0., 0.)
        self.last_cmd_time = time.time()
        self._send(*self.last_cmd)

    def _settled(self, now):
        if not self._sport_still:
            self._still_since = None
            return False
        if self._still_since is None:
            self._still_since = now
        return now - self._still_since >= STILL_TIME

    def _side_clear(self, direction):
        side = "left" if direction > 0 else "right"
        return self._sectors[f"{side}_seen"] and not self._sectors[f"{side}_close"]

    def _rotate_only(self, msg, rate):
        msg.twist.linear.x = msg.twist.linear.y = 0.
        msg.twist.angular.z = rate
        self.last_cmd = (0., 0., self.last_cmd[2])
        super().cb(msg)

    def cb(self, msg):
        if self.state != ARMED:
            return
        now = time.monotonic()
        sec = self._sectors
        if (sec is None or self._pose is None or not self._fresh(now) or self._planner_stop
                or self.phase not in ("EXPLORE", "RETURN") or self._sport_still is None
                or time.time() - self.last_state_time >= self.state_timeout):
            self._still_since = None
            self._stop_motion()
            return

        yaw = self._pose[2]
        if self._turn_target is not None:
            error = math.remainder(self._turn_target - yaw, math.tau)
            if now >= self._turn_deadline:
                self._fault("前障 90° 转向超时")
            elif abs(error) <= TURN_TOLERANCE:
                self._stop_motion()
                if self._settled(now):
                    self._turn_target = self._still_since = None
            elif not self._side_clear(error):
                self._still_since = None
                self._stop_motion()
            else:
                self._still_since = None
                self._rotate_only(msg, math.copysign(min(.45, max(.12, 1.5*abs(error))), error))
            return

        if sec["front_blocked"]:
            self._stop_motion()
            if not self._settled(now):
                return
            for direction in (-1, 1):
                if self._side_clear(direction):
                    self._turn_target = yaw + direction*TURN_ANGLE
                    self._turn_deadline = now + TURN_TIMEOUT
                    self._still_since = None
                    self.get_logger().info(f"前障停车后转向 {direction*90:+d}°")
                    break
            return

        self._still_since = None
        if not sec["front_seen"]:
            self._stop_motion()
            return
        if self.phase == "RETURN" and self.rotate != 0. and now - self.seen.get("rotate", -math.inf) < 1.5:
            if self._side_clear(self.rotate):
                self._rotate_only(msg, self.rotate)
            else:
                self._stop_motion()
            return
        if sec["left_close"] or sec["right_close"]:
            direction = -1 if sec["left_close"] else 1
            if not self._side_clear(direction):
                self._stop_motion()
                return
            msg.twist.linear.y = direction*SIDE_SLIDE_SPEED
            msg.twist.angular.z = 0.
            vx, vy, _ = self.last_cmd
            self.last_cmd = (vx, vy if direction*vy >= 0 else 0., 0.)
        super().cb(msg)

    def check(self):
        super().check()
        now = time.monotonic()
        fresh = self._fresh(now)
        if self.state == DISARMED:
            if now - self.started > 60.:
                self.finished = "START_TIMEOUT"
            elif (fresh and self._sectors is not None and self._sectors["front_seen"]
                  and self.phase in ("EXPLORE", "RETURN") and not self._planner_stop
                  and self._sport_still is not None
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
    result_path = Path(os.environ["NAV_RESULT"])
    try:
        while not node.should_exit():
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        node.finished = node.finished or "INTERRUPTED"
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.state == ARMED:
            node._fault("导航测试结束")
        node.get_logger().info(f"导航测试结果 {node.finished}")
        if node.finished:
            result_path.write_text(node.finished + "\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
