#!/usr/bin/env python3
"""wp5_explore_node: WP5 在线循环探索节点

持续: 累积地形图 → 提取前沿 → 选最优观察点 → 发 /goal_point → 监视到达 → 重选

循环逻辑:
  EXPLORE: 从最新地图选前沿观察点, 发目标
    到达(far_planner 连续确认 或 BFS路径长度<arrive_dis 兜底) → 重新选点
    目标超时(无进展/墙钟) → 降权该目标 + 换下一个
    无有效目标 → 停在原地(不返航), 每循环重试选点

订阅:
  /terrain_map            (PointCloud2)   局部地形, 累积成全局网格
  /state_estimation       (Odometry)      狗位置
  /far_reach_goal_status  (Bool)          下游 far_planner 到达确认
发布:
  /goal_point             (PointStamped)  探索目标(给 far_planner); 无目标时发当前位置

用法:
  ros2 run go2_keyboard_teleop wp5_explore_node
  ros2 run go2_keyboard_teleop wp5_explore_node --ros-args \
    -p arrive_dis:=0.3 -p goal_timeout:=60.0
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
        super().__init__("wp5_explore_node")
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
        self.declare_parameter("arrive_cooldown", 30.0)    # 到达后冷却时长(s) [粘性前沿: 5→30]
        self.declare_parameter("arrive_cooldown_radius", 0.3)  # 冷却半径(只盖刚到达的前沿, 不能盖住放宽选点)
        self.declare_parameter("path_weight", 0.3)
        self.declare_parameter("return_weight", 0.3)
        self.declare_parameter("risk_weight", 0.3)
        self.declare_parameter("turn_weight", 0.5)
        self.declare_parameter("reach_vote_thre", 3)

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

        self.terrain_sub = self.create_subscription(
            PointCloud2, "/terrain_map", self.cb_terrain, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/state_estimation", self.cb_odom, 10)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self.reach_sub = self.create_subscription(
            Bool, "/far_reach_goal_status", self.cb_reach_goal, 10)

        self.grid = {}          # (ix, iy) -> 0/1/2 (unknown/free/occupied)
        self.pos = (0.0, 0.0)
        self.yaw = 0.0          # 狗朝向(rad, map系)
        self.home = None        # 首次 odom 锁存, 不写死 (0,0)
        self.got_odom = False
        self.current_goal = None
        self.current_goal_cell = None   # 当前目标栅格 (gix, giy), 超时/到达用黑名单
        self.goal_time = 0.0
        self.explore_count = 0
        self.penalty = {}       # (ix, iy) -> 过期时间戳 (超时降权: 软降分)
        self.arrive_cd = {}     # (ix, iy) -> 过期时间戳 (到达冷却: 短时硬排除)
        self.last_pub_key = None   # 上次发布的目标(去重用, 防每秒重置 FAR)
        self.reach_votes = 0    # /far_reach_goal_status 连续 True 计数
        self.progress_ref = (0.0, 0.0)        # (x,y) 进展参考位置
        self.progress_ref_time = 0.0          # 上次有进展的时刻
        self.last_yaw = 0.0                   # 上一帧朝向(累计转角用)
        self.cum_turn = 0.0                   # 位置无进展期间的累计转角(rad)

        self.timer = self.create_timer(1.0, self.loop)
        self.get_logger().info(
            f"WP5 在线探索节点就绪 | 到达阈值 {self.arrive_dis}m "
            f"| 目标超时 {self.goal_timeout}s | 吸附半径 {self.snap_radius}m "
            f"| 最小目标距 {self.min_goal_dis}m")

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
        # yaw(四元数→欧拉绕z), 用于朝向惩罚
        self.yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self.got_odom:
            self.home = self.pos
            self.got_odom = True
            self.get_logger().info(
                f"home 锁存 = ({self.pos[0]:.2f}, {self.pos[1]:.2f})")

    def cb_reach_goal(self, msg: Bool):
        """下游 far_planner 判定已到达(连续 True 达阈值才算)"""
        if msg.data:
            self.reach_votes += 1
        else:
            self.reach_votes = 0

    # ---- 前沿提取 + 选点 ----
    def _extract_frontiers(self):
        """提取前沿(free 邻接 unknown 的格子), 返回 list[(ix,iy)]"""
        frontier = set()
        for (ix, iy), s in self.grid.items():
            if s != 1:   # 只找 free 格
                continue
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                if self.grid.get((ix+dx, iy+dy), 0) == 0:   # 邻接 unknown
                    frontier.add((ix, iy))
                    break
        return list(frontier)

    def _cluster(self, frontier):
        """BFS 聚类, 返回 list[list[(ix,iy)]]"""
        visited = set(); clusters = []
        for f in frontier:
            if f in visited:
                continue
            q = deque([f]); visited.add(f); cl = []
            while q:
                c = q.popleft(); cl.append(c)
                for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                    nb = (c[0]+dx, c[1]+dy)
                    if nb in frontier and nb not in visited:
                        visited.add(nb); q.append(nb)
            if len(cl) >= 2:
                clusters.append(cl)
        return clusters

    def _nearest_free(self, si, sj):
        """起点吸附: 狗脚下是雷达盲区, 在 snap_radius 内找最近 free 格作 BFS 起点(不篡改地图)"""
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
        """把狗脚下盲区(雷达看不到)标为 free, 消除'脚下假前沿'导致原地打转.
        只升级 unknown→free, 绝不碰 occupied(不擦除真实障碍).
        """
        si = int(round(self.pos[0] / RES)); sj = int(round(self.pos[1] / RES))
        r = int(round(self.footprint_radius / RES))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    key = (si + di, sj + dj)
                    if self.grid.get(key, 0) == 0:   # 只 unknown→free
                        self.grid[key] = 1

    def _bfs(self, gi, gj):
        """BFS 从狗位置(吸附起点)沿 free 到目标, 返回路径长度(米); 不可达返回 None"""
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

    def _reachable(self, gi, gj):
        return self._bfs(gi, gj) is not None

    # ---- 超时降权(软降分) + 到达冷却(短时硬排除) ----
    def _penalize_goal(self, gix, giy):
        """超时后降权(软) + 硬排除: 防止降权后仍重选同一点导致锁死"""
        self.penalty[(gix, giy)] = time.time() + self.penalty_duration
        self.arrive_cd[(gix, giy)] = time.time() + self.penalty_duration   # 硬排除(复用到达冷却检查)
        self.get_logger().warn(
            f"降权+硬排除目标栅格 ({gix},{giy}) {self.penalty_duration:.0f}s")

    def _penalty_factor(self, gix, giy):
        """返回降权因子(1.0=不降, penalty_factor=降权); 顺带清理过期项"""
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
        """到达后短时冷却该栅格附近(硬排除), 防止 7cm 微振荡反复选同一片"""
        self.arrive_cd[(gix, giy)] = time.time() + self.arrive_cooldown
        self.get_logger().info(
            f"到达冷却栅格 ({gix},{giy}) {self.arrive_cooldown:.0f}s")

    def _is_cooldown(self, gix, giy):
        """检查栅格是否在到达冷却半径内(顺带清理过期项)"""
        now = time.time()
        r = int(round(self.arrive_cooldown_radius / RES))
        for key in list(self.arrive_cd.keys()):
            if self.arrive_cd[key] <= now:
                del self.arrive_cd[key]
                continue
            if abs(gix - key[0]) <= r and abs(giy - key[1]) <= r:
                return True
        return False

    def _select_goal(self):
        """选最优前沿观察点. 返回 dict(score,info,path,ret,risk,turn,gx,gy,gix,giy) 或 None.
        五项评分(离线四项 + 朝向惩罚):
          info  = 观察点 1.5m 窗口内 unknown 格数
          path  = 狗 -> 观察点 BFS 路径距离
          ret   = 观察点 -> home 距离
          risk  = 观察点 1m 窗口内 occupied 格数
          turn  = |目标方向 - 狗朝向| (0=正前方, pi=正后方)
          score = info / (1 + 0.5*path + 0.3*ret + 0.1*risk + 0.5*turn)
        栅格键 (ix, iy) = (x栅格, y栅格); 世界坐标 = 栅格索引 × RES(勿交换, 勿加 0.5).
        """
        frontier = self._extract_frontiers()
        if not frontier:
            return None
        clusters = self._cluster(frontier)
        if not clusters:
            return None

        r = int(self.info_radius / RES)
        rr = int(self.risk_radius / RES)   # 风险用更小的 1m 窗口
        hx, hy = self.home if self.home is not None else (0.0, 0.0)
        candidates = []   # (score, dist, gx, gy, gix, giy, info, ret, risk)
        for cl in clusters:
            c = np.array(cl)               # (N,2), 每行 [ix, iy] = [x栅格, y栅格]
            mi, mj = c.mean(axis=0)        # mi=x栅格质心, mj=y栅格质心
            # 前沿簇成员本身就是 free 格; 取离质心最近的成员(质心可能落 unknown/occupied)
            k = int(np.argmin((c[:, 0] - mi) ** 2 + (c[:, 1] - mj) ** 2))
            gix, giy = int(c[k, 0]), int(c[k, 1])
            gx, gy = gix * RES, giy * RES    # 栅格中心世界坐标(勿交换)
            if self._is_cooldown(gix, giy):   # 到达冷却区, 跳过(防微振荡)
                continue
            # 硬限制: 观察点 min_obs_dis 内不得有障碍(含膨胀), 有则跳过
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
            # 信息增益(1.5m窗口 unknown数) / 地形风险(1m窗口 occupied数)
            info = 0
            risk = 0
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    s = self.grid.get((gix + di, giy + dj), 0)
                    if s == 0:
                        info += 1
                    elif s == 2 and max(abs(di), abs(dj)) <= rr:
                        risk += 1
            bfs = self._bfs(gix, giy)          # BFS 路径长度(米), 不可达为 None
            if bfs is None:
                continue
            path = bfs                         # 到达代价用 BFS 路径长(比直线更准)
            ret = np.hypot(gx - hx, gy - hy)   # 返航代价(直线近似)
            eu = np.hypot(gx - self.pos[0], gy - self.pos[1])   # 底盘→目标欧氏距离(选点门槛用)
            # 朝向/转向代价: 目标方向 vs 狗朝向, 0=正前方, pi=正后方
            goal_ang = np.arctan2(gy - self.pos[1], gx - self.pos[0])
            turn = abs((goal_ang - self.yaw + np.pi) % (2.0 * np.pi) - np.pi)
            score = info / (1.0 + self.path_weight * path
                            + self.return_weight * ret
                            + self.risk_weight * risk
                            + self.turn_weight * turn)
            score *= self._penalty_factor(gix, giy)   # 超时目标软降权(不硬排除)
            candidates.append((score, path, gx, gy, gix, giy, info, ret, risk, turn, eu))
        if not candidates:
            return None

        # 硬门槛(欧氏距离): 底盘→目标直线距离 >= min_goal_dis, 不设近目标回退
        # (审查意见: 近目标会造成 pathFollower/FAR 到达门槛死区 → 近目标循环, 必须拒绝)
        far = [c for c in candidates if c[10] >= self.min_goal_dis]
        if not far:
            return None
        far.sort(key=lambda c: -c[0])
        best = far[0]
        return dict(score=best[0], path=best[1], gx=best[2], gy=best[3],
                    gix=best[4], giy=best[5], info=best[6], ret=best[7],
                    risk=best[8], turn=best[9], eu=best[10])

    # ---- 主循环 ----
    def loop(self):
        if not self.got_odom or not self.grid:
            return
        now = time.time()
        self._mark_free_footprint()   # 脚下盲区标 free, 消除假前沿

        # 检查当前目标是否到达/超时
        if self.current_goal is not None:
            far_reach = self.reach_votes >= self.reach_vote_thre
            # 到达兜底统一用 BFS 路径长度(与 min_goal_dis 选点口径一致):
            # 隔墙目标直线<阈值但 BFS 绕行很长, 不能误判"已到达"(cornerA 微振荡根因)
            d = self._bfs(*self.current_goal_cell) if self.current_goal_cell is not None else None
            if far_reach or (d is not None and d < self.arrive_dis):
                dlog = f"{d:.2f}m" if d is not None else "不可达"
                self.get_logger().info(
                    f"到达观察点 ({self.current_goal[0]:.1f},{self.current_goal[1]:.1f}) "
                    f"(far确认={far_reach}, BFS距离={dlog}), 重新选点")
                if self.current_goal_cell is not None:
                    self._cooldown_goal(*self.current_goal_cell)   # 到达冷却(短时)
                self.current_goal = None
                self.current_goal_cell = None
                self.reach_votes = 0
            elif d is None:
                # 目标不可达(栅格被占/路径被隔断): 降权放弃, 换下一个
                if self.current_goal_cell is not None:
                    self._penalize_goal(*self.current_goal_cell)
                self.get_logger().warn(
                    f"目标 ({self.current_goal[0]:.1f},{self.current_goal[1]:.1f}) "
                    f"BFS 不可达, 放弃并降权")
                self.current_goal = None
                self.current_goal_cell = None
                self.reach_votes = 0
            else:
                # 进展跟踪: 位置移动 或 实际转身(累计<2π) 才算进展
                dp = np.hypot(self.pos[0] - self.progress_ref[0],
                              self.pos[1] - self.progress_ref[1])
                if dp >= self.progress_thre:
                    self.progress_ref = (self.pos[0], self.pos[1])
                    self.progress_ref_time = now
                    self.cum_turn = 0.0
                else:
                    dyaw = abs((self.yaw - self.last_yaw + np.pi) % (2.0 * np.pi) - np.pi)
                    if dyaw > 0.01:          # 实际在转才累计
                        self.cum_turn += dyaw
                        if self.cum_turn < 2.0 * np.pi:
                            self.progress_ref_time = now   # 未转满一圈, 算进展
                    # dyaw≈0(停住) → 不刷新 → no_progress 正常超时
                self.last_yaw = self.yaw
                no_progress = (now - self.progress_ref_time) > self.no_progress_timeout
                wall_timeout = (now - self.goal_time) > self.goal_timeout
                if no_progress or wall_timeout:
                    # 卡住: 降权(软, 不排除)
                    if self.current_goal_cell is not None:
                        self._penalize_goal(*self.current_goal_cell)
                    self.get_logger().warn(
                        f"目标超时(无进展={no_progress}, 墙钟={wall_timeout}), 换下一个")
                    self.current_goal = None
                    self.current_goal_cell = None
                    self.reach_votes = 0

        # 没有当前目标 -> 选新目标(选不到就停原地, 不返航)
        if self.current_goal is None:
            res = self._select_goal()
            if res is not None:
                gx, gy = res["gx"], res["gy"]
                gix, giy = res["gix"], res["giy"]
                self.current_goal_cell = (gix, giy)
                self.explore_count += 1
                # 诊断输出(起点吸附 + 五项评分分解)
                si = int(round(self.pos[0] / RES)); sj = int(round(self.pos[1] / RES))
                start_state = self.grid.get((si, sj), 0)
                nf = self._nearest_free(si, sj)
                nfd = None if nf is None else round(
                    np.hypot((nf[0] - si) * RES, (nf[1] - sj) * RES), 2)
                self.get_logger().info(
                    f"[诊断] robot=({self.pos[0]:.2f},{self.pos[1]:.2f}) "
                    f"start_cell=({si},{sj}) state={start_state} nearest_free={nfd}m "
                    f"goal_cell=({gix},{giy}) goal_xy=({gx:.2f},{gy:.2f}) "
                    f"eu={res['eu']:.2f}m info={res['info']} path={res['path']:.2f}m ret={res['ret']:.2f}m "
                    f"risk={res['risk']} turn={res['turn']:.2f} score={res['score']:.2f}")
                self.current_goal = (gx, gy)
                self.goal_time = now
                self.reach_votes = 0
                # 重置进展参考(新目标从当前位置起算无进展超时)
                self.progress_ref = (self.pos[0], self.pos[1])
                self.progress_ref_time = now
                self.last_yaw = self.yaw
                self.cum_turn = 0.0
                self.get_logger().info(
                    f"[EXPLORE] 第{self.explore_count}个目标 -> ({gx:.2f},{gy:.2f}) [探索点]")
            # res is None → 停在原地, 不设目标

        # 发布目标: 只在目标变化时发(避免同目标每秒重发重置 FAR)
        if self.current_goal is not None:
            pub_x, pub_y = self.current_goal[0], self.current_goal[1]
        else:
            pub_x, pub_y = self.pos[0], self.pos[1]   # 无目标: 发当前位置让狗停
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
