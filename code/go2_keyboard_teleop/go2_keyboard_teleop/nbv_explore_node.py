#!/usr/bin/env python3
"""nbv_explore_node: NBV(Next-Best-View) 在线探索节点 (WP5 的替代版, 只换选点逻辑)

与 wp5_explore_node 的区别只有 _select_goal():
  WP5: 前沿簇质心(frontier-based)
  NBV: 自由空间采样候选 + ray-cast 信息增益 + 代价, 选效用最大

其余完全复用 WP5: 订阅/发布/网格累积/BFS/超时/冷却/降权/兜底发布。

订阅:
  /terrain_map            (PointCloud2)   局部地形, 累积成全局网格
  /state_estimation       (Odometry)      狗位置
  /far_reach_goal_status  (Bool)          下游 far_planner 到达确认
发布:
  /goal_point             (PointStamped)  探索目标(给 far_planner)

用法:
  ros2 run go2_keyboard_teleop nbv_explore_node
"""
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from sensor_msgs_py import point_cloud2

RES = 0.1
OBSTACLE_THRE = 0.2
UNKNOWN_INT = 1.45


class ExploreNode(Node):
    def __init__(self):
        super().__init__("nbv_explore_node")
        self.declare_parameter("arrive_dis", 0.35)
        self.declare_parameter("goal_timeout", 60.0)
        self.declare_parameter("no_progress_timeout", 8.0)
        self.declare_parameter("progress_thre", 0.1)
        self.declare_parameter("info_radius", 1.5)
        self.declare_parameter("risk_radius", 1.0)
        self.declare_parameter("min_obs_dis", 0.2)   # 观察点到障碍的硬最小距离(含膨胀)
        self.declare_parameter("min_goal_dis", 0.8)   # 底盘→目标最小欧氏(直线)距离
        self.declare_parameter("snap_radius", 0.6)
        self.declare_parameter("footprint_radius", 0.4)   # 脚下盲区标 free 半径
        self.declare_parameter("penalty_radius", 0.5)     # 降权半径
        self.declare_parameter("penalty_factor", 0.3)      # 超时降权因子(score × 0.3)
        self.declare_parameter("penalty_duration", 60.0)   # 降权持续时长(s)
        self.declare_parameter("arrive_cooldown", 30.0)    # 到达后冷却时长(s)
        self.declare_parameter("arrive_cooldown_radius", 0.3)  # 冷却半径
        self.declare_parameter("path_weight", 0.3)
        self.declare_parameter("return_weight", 0.3)
        self.declare_parameter("risk_weight", 0.3)
        self.declare_parameter("turn_weight", 0.5)
        self.declare_parameter("reach_vote_thre", 3)
        # NBV 专属参数
        self.declare_parameter("nbv_sample_num", 150)   # 采样候选点上限
        self.declare_parameter("nbv_ray_num", 36)       # ray-cast 方向数
        self.declare_parameter("nbv_ray_range", 5.0)    # ray-cast 量程(m)

        self.arrive_dis = self.get_parameter("arrive_dis").value
        self.goal_timeout = self.get_parameter("goal_timeout").value
        self.no_progress_timeout = self.get_parameter("no_progress_timeout").value
        self.progress_thre = self.get_parameter("progress_thre").value
        self.info_radius = self.get_parameter("info_radius").value
        self.risk_radius = self.get_parameter("risk_radius").value
        self.min_obs_dis = self.get_parameter("min_obs_dis").value
        self.min_goal_dis = self.get_parameter("min_goal_dis").value
        self.snap_radius = self.get_parameter("snap_radius").value
        self.footprint_radius = self.get_parameter("footprint_radius").value
        self.penalty_radius = self.get_parameter("penalty_radius").value
        self.penalty_factor = self.get_parameter("penalty_factor").value
        self.penalty_duration = self.get_parameter("penalty_duration").value
        self.arrive_cooldown = self.get_parameter("arrive_cooldown").value
        self.arrive_cooldown_radius = self.get_parameter("arrive_cooldown_radius").value
        self.path_weight = self.get_parameter("path_weight").value
        self.return_weight = self.get_parameter("return_weight").value
        self.risk_weight = self.get_parameter("risk_weight").value
        self.turn_weight = self.get_parameter("turn_weight").value
        self.reach_vote_thre = self.get_parameter("reach_vote_thre").value
        self.nbv_sample_num = self.get_parameter("nbv_sample_num").value
        self.nbv_ray_num = self.get_parameter("nbv_ray_num").value
        self.nbv_ray_range = self.get_parameter("nbv_ray_range").value

        self.terrain_sub = self.create_subscription(
            PointCloud2, "/terrain_map", self.cb_terrain, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/state_estimation", self.cb_odom, 10)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self.reach_sub = self.create_subscription(
            Bool, "/far_reach_goal_status", self.cb_reach_goal, 10)

        self.grid = {}          # (ix, iy) -> 0/1/2 (unknown/free/occupied)
        self.pos = (0.0, 0.0)
        self.yaw = 0.0
        self.home = None
        self.got_odom = False
        self.current_goal = None
        self.current_goal_cell = None
        self.goal_time = 0.0
        self.explore_count = 0
        self.penalty = {}
        self.arrive_cd = {}
        self.last_pub_key = None
        self.reach_votes = 0
        self.progress_ref = (0.0, 0.0)
        self.progress_ref_time = 0.0
        self.last_yaw = 0.0
        self.cum_turn = 0.0

        self.timer = self.create_timer(1.0, self.loop)
        self.get_logger().info(
            f"NBV 在线探索节点就绪 | 采样 {self.nbv_sample_num} 点 "
            f"| ray {self.nbv_ray_num} 方向/{self.nbv_ray_range}m "
            f"| 到达阈值 {self.arrive_dis}m | 最小目标距 {self.min_goal_dis}m")

    # ---- 数据回调 ----
    def cb_terrain(self, msg: PointCloud2):
        pts = point_cloud2.read_points(
            msg, field_names=["x", "y", "z", "intensity"], skip_nans=True)
        for p in pts:
            x, y, it = p[0], p[1], p[3]
            ix = int(round(x / RES)); iy = int(round(y / RES))
            s = 0 if it >= UNKNOWN_INT else (2 if it >= OBSTACLE_THRE else 1)
            key = (ix, iy)
            self.grid[key] = max(self.grid.get(key, 0), s)

    def cb_odom(self, msg: Odometry):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self.got_odom:
            self.home = self.pos
            self.got_odom = True
            self.get_logger().info(
                f"home 锁存 = ({self.pos[0]:.2f}, {self.pos[1]:.2f})")

    def cb_reach_goal(self, msg: Bool):
        if msg.data:
            self.reach_votes += 1
        else:
            self.reach_votes = 0

    def _nearest_free(self, si, sj):
        r = int(round(self.snap_radius / RES))
        for rad in range(0, r + 1):
            for di in range(-rad, rad + 1):
                for dj in range(-rad, rad + 1):
                    if max(abs(di), abs(dj)) != rad:
                        continue
                    if self.grid.get((si + di, sj + dj), 0) == 1:
                        return (si + di, sj + dj)
        return None

    def _mark_free_footprint(self):
        si = int(round(self.pos[0] / RES)); sj = int(round(self.pos[1] / RES))
        r = int(round(self.footprint_radius / RES))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    key = (si + di, sj + dj)
                    if self.grid.get(key, 0) == 0:
                        self.grid[key] = 1

    def _bfs(self, gi, gj):
        si = int(round(self.pos[0]/RES)); sj = int(round(self.pos[1]/RES))
        start = (si, sj) if self.grid.get((si, sj), 0) == 1 else self._nearest_free(si, sj)
        if start is None:
            return None
        if self.grid.get((gi, gj), 0) != 1:
            return None
        seen = {start}; q = deque([(start[0], start[1], 0)])
        while q:
            cx, cy, d = q.popleft()
            if (cx, cy) == (gi, gj):
                return d * RES
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                nb = (cx+dx, cy+dy)
                if nb not in seen and self.grid.get(nb, 0) == 1:
                    seen.add(nb); q.append((nb[0], nb[1], d+1))
        return None

    def _penalize_goal(self, gix, giy):
        self.penalty[(gix, giy)] = time.time() + self.penalty_duration
        self.arrive_cd[(gix, giy)] = time.time() + self.penalty_duration
        self.get_logger().warn(
            f"降权+硬排除目标栅格 ({gix},{giy}) {self.penalty_duration:.0f}s")

    def _penalty_factor(self, gix, giy):
        now = time.time()
        r = int(round(self.penalty_radius / RES))
        factor = 1.0
        for key in list(self.penalty.keys()):
            if self.penalty[key] <= now:
                del self.penalty[key]
                continue
            if abs(gix - key[0]) <= r and abs(giy - key[1]) <= r:
                factor = min(factor, self.penalty_factor)
        return factor

    def _cooldown_goal(self, gix, giy):
        self.arrive_cd[(gix, giy)] = time.time() + self.arrive_cooldown
        self.get_logger().info(
            f"到达冷却栅格 ({gix},{giy}) {self.arrive_cooldown:.0f}s")

    def _is_cooldown(self, gix, giy):
        now = time.time()
        r = int(round(self.arrive_cooldown_radius / RES))
        for key in list(self.arrive_cd.keys()):
            if self.arrive_cd[key] <= now:
                del self.arrive_cd[key]
                continue
            if abs(gix - key[0]) <= r and abs(giy - key[1]) <= r:
                return True
        return False

    # ---- NBV 选点 ----
    def _raycast_ig(self, gix, giy, max_r, ray_angles):
        """从 (gix,giy) 做 ray-cast, 统计可见 unknown 格数(遮挡感知: 遇障碍停)"""
        seen = set()
        for ang in ray_angles:
            dx = np.cos(ang); dy = np.sin(ang)
            for r in range(1, max_r + 1):
                ix = int(round(gix + r * dx))
                iy = int(round(giy + r * dy))
                s = self.grid.get((ix, iy), 0)
                if s == 2:          # 障碍: 射线被挡
                    break
                if s == 0:          # unknown: 可见
                    seen.add((ix, iy))
                # s==1 (free): 穿透继续
        return len(seen)

    def _select_goal(self):
        """NBV 选点: 自由空间采样候选 + ray-cast 信息增益 + 代价, 选效用最大.
        返回 dict(score,path,gx,gy,gix,giy,info,ret,risk,turn,eu) 或 None.
          info  = ray-cast 可见 unknown 格数(遮挡感知)
          path  = 狗 -> 候选点 BFS 路径距离
          ret   = 候选点 -> home 距离
          turn  = |目标方向 - 狗朝向|
          score = info / (1 + path_weight*path + return_weight*ret + turn_weight*turn)
        """
        # 1. 采样自由空间候选点
        free_cells = [(ix, iy) for (ix, iy), s in self.grid.items() if s == 1]
        if not free_cells:
            return None
        if len(free_cells) > self.nbv_sample_num:
            idx = np.random.choice(len(free_cells), self.nbv_sample_num, replace=False)
            free_cells = [free_cells[i] for i in idx]

        hx, hy = self.home if self.home is not None else (0.0, 0.0)
        max_r = int(round(self.nbv_ray_range / RES))
        ray_angles = np.linspace(0.0, 2.0 * np.pi, self.nbv_ray_num, endpoint=False)

        candidates = []
        for (gix, giy) in free_cells:
            gx, gy = gix * RES, giy * RES
            if self._is_cooldown(gix, giy):
                continue
            # 硬门槛: 欧氏距离
            eu = np.hypot(gx - self.pos[0], gy - self.pos[1])
            if eu < self.min_goal_dis:
                continue
            # 硬限制: min_obs_dis 内不得有障碍
            m = int(round(self.min_obs_dis / RES))
            near_obs = False
            for di in range(-m, m + 1):
                for dj in range(-m, m + 1):
                    if self.grid.get((gix + di, giy + dj), 0) == 2:
                        near_obs = True
                        break
                if near_obs:
                    break
            if near_obs:
                continue
            # BFS 可达性 + 代价
            bfs = self._bfs(gix, giy)
            if bfs is None:
                continue
            # ray-cast 信息增益
            info = self._raycast_ig(gix, giy, max_r, ray_angles)
            if info <= 0:
                continue
            ret = np.hypot(gx - hx, gy - hy)
            goal_ang = np.arctan2(gy - self.pos[1], gx - self.pos[0])
            turn = abs((goal_ang - self.yaw + np.pi) % (2.0 * np.pi) - np.pi)
            risk = 0
            score = info / (1.0 + self.path_weight * bfs
                            + self.return_weight * ret
                            + self.turn_weight * turn)
            score *= self._penalty_factor(gix, giy)
            candidates.append((score, bfs, gx, gy, gix, giy, info, ret, risk, turn, eu))

        if not candidates:
            return None
        candidates.sort(key=lambda c: -c[0])
        best = candidates[0]
        return dict(score=best[0], path=best[1], gx=best[2], gy=best[3],
                    gix=best[4], giy=best[5], info=best[6], ret=best[7],
                    risk=best[8], turn=best[9], eu=best[10])

    # ---- 主循环 ----
    def loop(self):
        if not self.got_odom or not self.grid:
            return
        now = time.time()
        self._mark_free_footprint()

        if self.current_goal is not None:
            far_reach = self.reach_votes >= self.reach_vote_thre
            d = self._bfs(*self.current_goal_cell) if self.current_goal_cell is not None else None
            if far_reach or (d is not None and d < self.arrive_dis):
                dlog = f"{d:.2f}m" if d is not None else "不可达"
                self.get_logger().info(
                    f"到达观察点 ({self.current_goal[0]:.1f},{self.current_goal[1]:.1f}) "
                    f"(far确认={far_reach}, BFS距离={dlog}), 重新选点")
                if self.current_goal_cell is not None:
                    self._cooldown_goal(*self.current_goal_cell)
                self.current_goal = None
                self.current_goal_cell = None
                self.reach_votes = 0
            elif d is None:
                if self.current_goal_cell is not None:
                    self._penalize_goal(*self.current_goal_cell)
                self.get_logger().warn(
                    f"目标 ({self.current_goal[0]:.1f},{self.current_goal[1]:.1f}) "
                    f"BFS 不可达, 放弃并降权")
                self.current_goal = None
                self.current_goal_cell = None
                self.reach_votes = 0
            else:
                dp = np.hypot(self.pos[0] - self.progress_ref[0],
                              self.pos[1] - self.progress_ref[1])
                if dp >= self.progress_thre:
                    self.progress_ref = (self.pos[0], self.pos[1])
                    self.progress_ref_time = now
                    self.cum_turn = 0.0
                else:
                    dyaw = abs((self.yaw - self.last_yaw + np.pi) % (2.0 * np.pi) - np.pi)
                    if dyaw > 0.01:
                        self.cum_turn += dyaw
                        if self.cum_turn < 2.0 * np.pi:
                            self.progress_ref_time = now
                self.last_yaw = self.yaw
                no_progress = (now - self.progress_ref_time) > self.no_progress_timeout
                wall_timeout = (now - self.goal_time) > self.goal_timeout
                if no_progress or wall_timeout:
                    if self.current_goal_cell is not None:
                        self._penalize_goal(*self.current_goal_cell)
                    self.get_logger().warn(
                        f"目标超时(无进展={no_progress}, 墙钟={wall_timeout}), 换下一个")
                    self.current_goal = None
                    self.current_goal_cell = None
                    self.reach_votes = 0

        if self.current_goal is None:
            res = self._select_goal()
            if res is not None:
                gx, gy = res["gx"], res["gy"]
                gix, giy = res["gix"], res["giy"]
                self.current_goal_cell = (gix, giy)
                self.explore_count += 1
                si = int(round(self.pos[0] / RES)); sj = int(round(self.pos[1] / RES))
                start_state = self.grid.get((si, sj), 0)
                self.get_logger().info(
                    f"[诊断] robot=({self.pos[0]:.2f},{self.pos[1]:.2f}) "
                    f"goal_cell=({gix},{giy}) goal_xy=({gx:.2f},{gy:.2f}) "
                    f"eu={res['eu']:.2f}m info={res['info']} path={res['path']:.2f}m "
                    f"ret={res['ret']:.2f}m turn={res['turn']:.2f} score={res['score']:.2f}")
                self.current_goal = (gx, gy)
                self.goal_time = now
                self.reach_votes = 0
                self.progress_ref = (self.pos[0], self.pos[1])
                self.progress_ref_time = now
                self.last_yaw = self.yaw
                self.cum_turn = 0.0
                self.get_logger().info(
                    f"[NBV] 第{self.explore_count}个目标 -> ({gx:.2f},{gy:.2f}) info={res['info']}")

        if self.current_goal is not None:
            pub_x, pub_y = self.current_goal[0], self.current_goal[1]
        else:
            pub_x, pub_y = self.pos[0], self.pos[1]
        key = (round(pub_x, 2), round(pub_y, 2))
        if key != self.last_pub_key:
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.point.x = pub_x
            msg.point.y = pub_y
            msg.point.z = 0.0
            self.goal_pub.publish(msg)
            self.last_pub_key = key


def main(args=None):
    rclpy.init(args=args)
    node = ExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
