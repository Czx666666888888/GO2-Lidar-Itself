#!/usr/bin/env python3
"""独立导航测试：锁定起步方向的一个自由区目标，沿记录路径返航。"""
import time
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32, String
from sensor_msgs_py import point_cloud2

from body_footprint import CORNER_RADIUS, PARKING_CLEARANCE, edge_distance

RES = 0.1
OBSTACLE_THRE = 0.2
UNKNOWN_INT = 1.45


class ExploreNode(Node):
    def __init__(self):
        super().__init__("nbv_explore_node")
        self.arrive_dis = self.declare_parameter("arrive_dis", 0.25).value
        self.path_record_step = self.declare_parameter("path_record_step", 0.30).value
        self.startup_delay = self.declare_parameter("startup_delay", 2.0).value
        self.return_after = self.declare_parameter("return_after", 30.0).value
        self.return_stuck_sec = self.declare_parameter("return_stuck_sec", 15.0).value
        self.return_arrive_dis = self.declare_parameter("return_arrive_dis", 0.15).value
        self.return_trim_dis = self.declare_parameter("return_trim_dis", 0.45).value
        self.record_push_obs = self.declare_parameter("record_push_obs", True).value
        self.record_push_margin = self.declare_parameter("record_push_margin", 0.40).value
        self.no_progress_timeout = self.declare_parameter("no_progress_timeout", 8.0).value
        self.footprint_radius = 0.4
        self.terrain_sub = self.create_subscription(
            PointCloud2, "/terrain_map", self.cb_terrain, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/base_state_estimation", self.cb_odom, 10)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self.status_pub = self.create_publisher(String, "/navigation_state", 10)
        self.ret_sub = self.create_subscription(Bool, "/navigation_test/return_home", self.cb_return_home, 10)
        self.hard_stop_sub = self.create_subscription(Bool, "/navigation_test/hard_stop", self.cb_hard_stop, 1)
        self.way_pub = self.create_publisher(PointStamped, "/way_point", 10)
        self.rot_pub = self.create_publisher(Float32, "/escape_rotate", 10)   # 返航时转身面向目标
        self.waypoint_sub = self.create_subscription(
            PointStamped, "/navigation_test/far_waypoint", self.cb_waypoint, 10)

        self.grid = {}          # (ix, iy) -> 0/1/2 (unknown/free/occupied)
        self.home = None
        self.home_yaw = 0.
        self.pos = (0., 0.)
        self.yaw = 0.
        self.odom_frame = "camera_init"
        self.start_time = None
        self.mode = "EXPLORE"
        self.current_goal = None
        self.outbound_reached = False
        self.path_hist = []
        self.last_rec = None
        self.return_goal = None
        self.turn_direction = 0.
        self.progress_time = None
        self.next_plan = 0.
        self.timer = self.create_timer(0.2, self.loop)

    # ---- 数据回调 ----
    def cb_terrain(self, msg: PointCloud2):
        try:
            pts = point_cloud2.read_points(
                msg, field_names=["x", "y", "intensity"], skip_nans=True)
        except Exception:
            return
        if pts is None or len(pts) == 0:
            return
        finite = np.isfinite(pts["x"]) & np.isfinite(pts["y"]) & np.isfinite(pts["intensity"])
        x = pts["x"][finite].astype(np.float64)
        y = pts["y"][finite].astype(np.float64)
        it = pts["intensity"][finite]
        ix = np.round(x / RES).astype(np.int32)
        iy = np.round(y / RES).astype(np.int32)
        # 0=unknown, 1=free, 2=obstacle；向量化分类
        s = np.where(it >= UNKNOWN_INT, 0,
                     np.where(it >= OBSTACLE_THRE, 2, 1)).astype(np.int8)
        # 按 s 降序排列，同一格只写首次（即最大值）
        order = np.argsort(-s, kind="stable")
        seen: set = set()
        for idx in order:
            key = (int(ix[idx]), int(iy[idx]))
            if key in seen:
                continue
            seen.add(key)
            sv = int(s[idx])
            if sv > self.grid.get(key, 0):
                self.grid[key] = sv

    def cb_odom(self, msg: Odometry):
        self.odom_frame = msg.header.frame_id or self.odom_frame
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.home is None:
            self.home = self.pos
            self.home_yaw = self.yaw
            self.get_logger().info(
                f"home 锁存 = ({self.pos[0]:.2f}, {self.pos[1]:.2f})")

    def cb_waypoint(self, msg: PointStamped):
        if self.mode == "EXPLORE" and self.current_goal is not None:
            if self.progress_time is None:
                self.progress_time = time.monotonic()
            self.way_pub.publish(msg)

    def _mark_free_footprint(self):
        si = int(round(self.pos[0] / RES)); sj = int(round(self.pos[1] / RES))
        r = int(round(self.footprint_radius / RES))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    key = (si + di, sj + dj)
                    if self.grid.get(key, 0) == 0:
                        self.grid[key] = 1

    def _select_goal(self):
        """Prefer the longest forward projection within a known clear corridor."""
        best = None
        best_forward = 0.
        for angle in (0, 20, -20, 40, -40, 60, -60, 80, -80):
            heading = self.home_yaw + math.radians(angle)
            dx, dy = math.cos(heading), math.sin(heading)
            for step in range(1, 41):  # sample every half cell, up to 2 m
                distance = step * RES / 2
                gx = self.pos[0] + distance * dx
                gy = self.pos[1] + distance * dy
                ix, iy = round(gx / RES), round(gy / RES)
                if self.grid.get((ix, iy)) != 1 or self._near_obstacle(ix, iy, 2):
                    break
                forward = distance * math.cos(math.radians(angle))
                if distance < 0.8 or forward <= best_forward:
                    continue
                if self._near_obstacle(ix, iy, 4):
                    continue
                free = sum(self.grid.get((ix+i, iy+j)) == 1
                           for i in (-1, 0, 1) for j in (-1, 0, 1))
                if free >= 6 and self._parking_space(gx, gy, heading):
                    best, best_forward = (gx, gy), forward
        return best

    def _parking_space(self, x, y, yaw):
        # Account for rounding map points to cell centers (up to half a diagonal).
        margin = PARKING_CLEARANCE + RES / math.sqrt(2.)
        radius = math.ceil((CORNER_RADIUS + margin) / RES)
        cx, cy = round(x / RES), round(y / RES)
        c, s = math.cos(yaw), math.sin(yaw)
        for ix in range(cx-radius, cx+radius+1):
            for iy in range(cy-radius, cy+radius+1):
                dx, dy = ix*RES-x, iy*RES-y
                if edge_distance(c*dx+s*dy, -s*dx+c*dy) <= margin:
                    if self.grid.get((ix, iy)) != 1:
                        return False
        return True

    def _near_obstacle(self, ix, iy, radius):
        return any(self.grid.get((ix+i, iy+j)) == 2
                   for i in range(-radius, radius+1) for j in range(-radius, radius+1))

    def cb_hard_stop(self, msg):
        if msg.data:
            self.mode = "FAULT_HOLD"
            self.get_logger().warn("机身净空硬停，任务暂停")

    def cb_return_home(self, msg):
        if msg.data and self.mode == "EXPLORE" and self.home is not None:
            self._begin_return("手动返航")

    def _begin_return(self, reason):
        self.current_goal = None
        self.return_goal = None
        self.mode = "RETURN"
        self._trim_return_tail()
        self.get_logger().info(f"{reason}，沿 {len(self.path_hist)} 个记录点返航")
        self._return_step()
        self.status_pub.publish(String(data=self.mode))

    def _trim_return_tail(self):
        """裁掉路径尾部那些"已经站在上面"的点。
        最后一个记录点就是狗当前位置(距 0.2m), 而 return_arrive_dis=0.15,
        差几厘米判不到达 -> localPlanner 出不了路径 -> 永远卡在第一个点。"""
        n0 = len(self.path_hist)
        while self.path_hist:
            gx, gy = self.path_hist[-1]
            if np.hypot(self.pos[0]-gx, self.pos[1]-gy) <= self.return_trim_dis:
                self.path_hist.pop()
            else:
                break
        n1 = len(self.path_hist)
        if n1 != n0:
            self.get_logger().info(
                f"返航前裁掉 {n0-n1} 个已到达的尾部点 (阈值 {self.return_trim_dis:.2f}m), 剩 {n1} 个")

    def _push_away_from_obstacles(self, x, y):
        """把记录点从障碍处推开: 返航目标贴墙时 localPlanner 一定规划不进去。
        只在已建图区域内推, 找不到安全位置就保留原点。"""
        if not getattr(self, "record_push_obs", False):
            return (x, y)
        m = int(round(self.record_push_margin / RES))
        def clear(cx, cy):
            for di in range(-m, m+1):
                for dj in range(-m, m+1):
                    if self.grid.get((cx+di, cy+dj), 0) == 2:   # 2 = 障碍
                        return False
            return True
        ci, cj = int(round(x/RES)), int(round(y/RES))
        if clear(ci, cj):
            return (x, y)
        # 由近到远螺旋搜索最近的"无邻近障碍"自由格
        for rad in range(1, m+4):
            best = None; bestd = 1e9
            for di in range(-rad, rad+1):
                for dj in range(-rad, rad+1):
                    if max(abs(di), abs(dj)) != rad:
                        continue
                    ni, nj = ci+di, cj+dj
                    if self.grid.get((ni, nj), 0) != 1:
                        continue
                    if not clear(ni, nj):
                        continue
                    dd = np.hypot(di, dj)
                    if dd < bestd:
                        bestd = dd; best = (ni*RES, nj*RES)
            if best is not None:
                return best
        return (x, y)

    def _record_path(self):
        # Retain the actual traversed route; proximity alone does not prove a loop.
        if self.last_rec is None:
            self.path_hist.append(tuple(self.home))
            self.last_rec = tuple(self.pos)
        elif np.hypot(self.pos[0]-self.last_rec[0], self.pos[1]-self.last_rec[1]) >= self.path_record_step:
            self.path_hist.append(self._push_away_from_obstacles(*self.pos))
            self.last_rec = tuple(self.pos)

    def _next_return_goal(self):
        if not self.path_hist:
            return tuple(self.home)
        goal = self.path_hist.pop()
        # Extend only along the same straight run of recorded points. Preserve bends.
        while self.path_hist:
            candidate = self.path_hist[-1]
            vx, vy = goal[0]-self.pos[0], goal[1]-self.pos[1]
            wx, wy = candidate[0]-goal[0], candidate[1]-goal[1]
            dot = vx*wx + vy*wy
            cross = abs(vx*wy - vy*wx)
            if (dot <= 0 or cross > dot * math.tan(math.radians(8))
                    or math.dist(self.pos, candidate) > 0.8):
                break
            # A straight shortcut must still be clear in the current grid.
            count = math.ceil(math.dist(self.pos, candidate) / (RES / 2))
            for step in range(1, count + 1):
                ratio = step / count
                ix = round((self.pos[0] + ratio * (candidate[0] - self.pos[0])) / RES)
                iy = round((self.pos[1] + ratio * (candidate[1] - self.pos[1])) / RES)
                if self.grid.get((ix, iy)) != 1 or self._near_obstacle(ix, iy, 2):
                    return goal
            goal = self.path_hist.pop()
        return goal

    def _return_step(self):
        d_home = np.hypot(self.pos[0]-self.home[0], self.pos[1]-self.home[1])
        if d_home <= self.return_arrive_dis:
            if self.mode != "DONE":
                self.get_logger().info("返航到位：实际位置进入起点范围")
            self.mode = "DONE" if self.outbound_reached else "RETURNED_EARLY"
            if not self._parking_space(*self.pos, self.yaw):
                self.mode = "PARKING_CLEARANCE_UNCONFIRMED"
            self.return_goal = tuple(self.home)
        else:
            if self.return_goal is not None:
                d = np.hypot(self.pos[0]-self.return_goal[0], self.pos[1]-self.return_goal[1])
                if d <= self.return_arrive_dis:
                    self.return_goal = None
                    self.mode = "RETURN"
                elif d < self.return_best - 0.10:
                    self.return_best = d
                    self.return_t = time.monotonic()
                    self.mode = "RETURN"
                elif time.monotonic() - self.return_t > self.return_stuck_sec:
                    self.mode = "RETURN_BLOCKED"
                    self.get_logger().warn("返航点无进展，停车并保留该点")
                    return
            if self.return_goal is None:
                self.return_goal = self._next_return_goal()
                self.return_best = np.hypot(self.pos[0]-self.return_goal[0],self.pos[1]-self.return_goal[1])
                self.return_t = time.monotonic()
        if self.mode == "RETURN":
            self._publish_point(self.way_pub, self.return_goal)

    def _publish_point(self, publisher, goal):
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.odom_frame
        m.point.x, m.point.y, m.point.z = *goal, 0.0
        publisher.publish(m)

    def _turn_to_return_goal(self):
        gx, gy = self.return_goal
        error = math.atan2(gy-self.pos[1], gx-self.pos[0]) - self.yaw
        error = math.atan2(math.sin(error), math.cos(error))
        angle = abs(math.degrees(error))
        if angle <= 25:
            self.turn_direction = 0.
        elif (angle > 40 or self.turn_direction) and (not self.turn_direction or angle < 150):
            self.turn_direction = math.copysign(1., error)
        return self.turn_direction

    def loop(self):
        now = time.monotonic()
        if self.home is not None and self.grid:
            if self.mode == "EXPLORE":
                self._outbound_step(now)
            elif self.mode == "RETURN":
                self._return_step()

        turn = self._turn_to_return_goal() if self.mode == "RETURN" else 0.
        self.rot_pub.publish(Float32(data=turn))
        phase = "HOLD" if self.mode == "EXPLORE" and self.current_goal is None else self.mode
        self.status_pub.publish(String(data=phase))

    def _outbound_step(self, now):
        self._record_path()
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time - self.startup_delay
        if elapsed < 0:
            return

        if self.current_goal is not None:
            remaining = math.dist(self.pos, self.current_goal)
            if remaining <= self.arrive_dis:
                self.outbound_reached = self._parking_space(*self.pos, self.yaw)
                self._begin_return("到达进场目标" if self.outbound_reached else "进场停靠净空不足或未知")
                return
            if self.progress_time is not None and remaining <= self.best_remaining - 0.1:
                self.best_remaining, self.progress_time = remaining, now
            if self.progress_time is not None and now - self.progress_time >= self.no_progress_timeout:
                self._begin_return(f"进场无进展，距目标 {remaining:.2f} m")
                return
        if self.return_after > 0 and elapsed >= self.return_after:
            self._begin_return("进场时间到")
            return
        if now < self.next_plan:
            return
        self.next_plan = now + 1.
        if self.current_goal is None:
            self._mark_free_footprint()
            self.current_goal = self._select_goal()
            if self.current_goal is None:
                self.get_logger().info("等待前方可达自由区", throttle_duration_sec=5.)
                return
            self.best_remaining = math.dist(self.pos, self.current_goal)
            self.get_logger().info(f"锁定进场目标 {self.current_goal}，距离 {self.best_remaining:.2f} m")
        self._publish_point(self.goal_pub, self.current_goal)


def main(args=None):
    rclpy.init(args=args)
    node = ExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
