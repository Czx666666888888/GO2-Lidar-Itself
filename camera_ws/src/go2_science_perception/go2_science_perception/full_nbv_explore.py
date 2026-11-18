#!/usr/bin/env python3
"""nbv_explore_node: NBV(Next-Best-View) 在线探索节点 (WP5 的替代版, 只换选点逻辑)

与 wp5_explore_node 的区别只有 _select_goal():
  WP5: 前沿簇质心(frontier-based)
  NBV: 自由空间采样候选 + ray-cast 信息增益 + 代价, 选效用最大

其余完全复用 WP5: 订阅/发布/网格累积/BFS/超时/冷却/降权/兜底发布。

订阅:
  /terrain_map            (PointCloud2)   局部地形, 累积成全局网格
  /base_state_estimation       (Odometry)      狗位置
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
from std_msgs.msg import Bool, Float32, Int8, String
from sensor_msgs_py import point_cloud2

RES = 0.1
OBSTACLE_THRE = 0.2
UNKNOWN_INT = 1.45


def return_tail_index(history, position, radius):
    """Index of the last recorded point outside the arrival radius."""
    index = len(history) - 1
    while index >= 0 and np.hypot(position[0]-history[index][0],
                                  position[1]-history[index][1]) <= radius:
        index -= 1
    return index


class ExploreNode(Node):
    return_face_deg = 60.0
    stop_on_done = True
    _done_stopped = False
    home_arrive_dis = 0.35
    # 类级默认: 队友的回归测试用 object.__new__ 构造替身, 不跑 __init__
    blocked_count = 0
    return_blocked_reverse = True
    return_blocked_after = 2
    return_blocked_dist = 0.30
    return_reverse_until = 0.0
    def __init__(self):
        super().__init__("nbv_explore_node")
        self.declare_parameter("arrive_dis", 0.25)   # 0.35->0.25: 少切一次目标, 每次多走一点
        self.declare_parameter("path_record_step", 0.30)   # 路径记录间隔(m)
        self.declare_parameter("goal_front_deg", 100.0)   # 只选狗朝向前方±100°内的目标(不选身后)
        self.declare_parameter("require_interior", True)   # 只选已知自由空间内的点(far才能规划)
        self.declare_parameter("interior_min_free", 6)     # 3x3 邻域至少几个 free (9=最严, 6=放宽)
        self.declare_parameter("return_face_deg", 60.0)          # 返航时目标偏离车头超过此角度先纯旋转
        self.declare_parameter("stop_on_done", True)             # 到家后发 /stop 让下游(far/local/pathFollower)真正停下
        self.declare_parameter("home_arrive_dis", 0.35)          # 到家区域半径(m): 家本身常贴墙, 不能用 return_arrive_dis 的 0.15
        self.declare_parameter("return_goal_margin", 0.40)       # 返航目标距障碍的最小距离(m)
        self.declare_parameter("record_push_obs", True)          # 记录路径点时把点推离障碍
        self.declare_parameter("return_blocked_reverse", True)   # 返航 BLOCKED 时后退一小段换位置
        self.declare_parameter("return_blocked_after", 2)        # 连续几次 BLOCKED 后触发后退
        self.declare_parameter("return_blocked_dist", 0.30)      # 后退距离(m)
        self.declare_parameter("escape_enable", True)      # 卡住时主动后退脱困
        self.declare_parameter("escape_after", 5.0)        # 无进展多少秒后触发
        self.declare_parameter("escape_dist", 0.25)   # 0.40->0.25: 后退距离减小, 避免退过头        # 后退距离(m)
        self.declare_parameter("startup_delay", 12.0)      # 启动后先等地图铺开(秒), 期间不选点
        self.declare_parameter("return_after", 0.0)        # 默认关闭测试用定时返航
        self.declare_parameter("return_stuck_sec", 8.0)   # 15->8: 每轮等待缩短, 600s 预算更紧
        self.declare_parameter("return_trim_dis", 0.45)     # 进入返航时, 距狗这么近的尾部点直接裁掉
        self.declare_parameter("return_arrive_dis", 0.15)  # 返航到点判定(m): 必须 < path_record_step(0.30), 否则狗没走就判到达
        self.declare_parameter("goal_timeout", 60.0)
        self.declare_parameter("no_progress_timeout", 8.0)
        self.declare_parameter("progress_thre", 0.1)
        self.declare_parameter("info_radius", 1.5)
        self.declare_parameter("risk_radius", 1.0)
        self.declare_parameter("min_obs_dis", 0.35)
        self.declare_parameter("back_penalty", 0.3)       # 目标比狗更靠近起点时的降权系数(1.0=关闭)   # 0.2->0.35: 目标点离障碍更远(1m通道中心最多0.5m, 不能再大)
        self.declare_parameter("min_goal_dis", 0.8)   # 底盘→目标最小欧氏距离(1.5->0.8回退: 实测0.8时目标近、到达快、path退化仅1%)
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
        self.declare_parameter("reach_inflate", 0.2)    # 可达性判据的障碍膨胀(m)=狗半宽, 防选到窄缝不可达点
        self.declare_parameter("far_stuck_timeout", 2.0)  # far 卡住(waypoint≈狗位)多久就立刻换目标(s)
        self.declare_parameter("far_stuck_dis", 0.15)     # waypoint 距【机身中心】多近才算"卡住"
        self.declare_parameter("spin_scan", False)         # 默认关: 原地旋转会让 Point-LIO 严重发散(实测漂17.8m)
        self.declare_parameter("spin_radius", 1.0)         # 侦察时"右侧目标"的距离(m)
        self.declare_parameter("spin_timeout", 45.0)       # 侦察最长时长(s): 实测转速~12°/s, 转一圈需~30s

        self.arrive_dis = self.get_parameter("arrive_dis").value
        self.path_record_step = self.get_parameter("path_record_step").value
        self.loop_close_radius = 0.5   # 回到已走路径 0.5m 内则删掉中间绕路(消环)
        self.goal_front_deg = self.get_parameter("goal_front_deg").value
        self.require_interior = self.get_parameter("require_interior").value
        self.interior_min_free = self.get_parameter("interior_min_free").value
        self.escape_enable = self.get_parameter("escape_enable").value
        self.return_goal_margin = self.get_parameter("return_goal_margin").value
        self.home_arrive_dis = self.get_parameter("home_arrive_dis").value
        self.stop_on_done = self.get_parameter("stop_on_done").value
        self.return_face_deg = self.get_parameter("return_face_deg").value
        self._done_stopped = False
        self.record_push_obs = self.get_parameter("record_push_obs").value
        self.return_blocked_reverse = self.get_parameter("return_blocked_reverse").value
        self.return_blocked_after = self.get_parameter("return_blocked_after").value
        self.return_blocked_dist = self.get_parameter("return_blocked_dist").value
        self.blocked_count = 0
        self.return_reverse_until = 0.0
        self.escape_after = self.get_parameter("escape_after").value
        self.escape_dist = self.get_parameter("escape_dist").value
        self.escape_goal = None
        self.escape_until = 0.0
        self.startup_delay = self.get_parameter("startup_delay").value
        self.return_after = self.get_parameter("return_after").value
        self.start_time = None
        self.return_arrive_dis = self.get_parameter("return_arrive_dis").value
        self.return_stuck_sec = self.get_parameter("return_stuck_sec").value
        self.return_trim_dis = self.get_parameter("return_trim_dis").value
        self.return_t = 0.0
        self.return_best = 1e9
        self.return_recovery_goal = None
        self.path_hist = []
        self.last_rec = None
        self.mode = "EXPLORE"
        self.return_goal = None
        self.goal_timeout = self.get_parameter("goal_timeout").value
        self.no_progress_timeout = self.get_parameter("no_progress_timeout").value
        self.progress_thre = self.get_parameter("progress_thre").value
        self.info_radius = self.get_parameter("info_radius").value
        self.risk_radius = self.get_parameter("risk_radius").value
        self.min_obs_dis = self.get_parameter("min_obs_dis").value
        self.back_penalty = self.get_parameter("back_penalty").value
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
        self.reach_inflate = self.get_parameter("reach_inflate").value
        self.far_stuck_timeout = self.get_parameter("far_stuck_timeout").value
        self.far_stuck_dis = self.get_parameter("far_stuck_dis").value
        self.spin_scan = self.get_parameter("spin_scan").value
        self.spin_radius = self.get_parameter("spin_radius").value
        self.spin_timeout = self.get_parameter("spin_timeout").value

        self.terrain_sub = self.create_subscription(
            PointCloud2, "/terrain_map", self.cb_terrain, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/state_estimation", self.cb_odom, 10)
        self.base_pos = None
        self.base_home = None
        self.base_sub = self.create_subscription(
            Odometry, "/base_state_estimation", self.cb_base_odom, 10)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self.status_pub = self.create_publisher(String, "/navigation_state", 10)
        self.ret_sub = self.create_subscription(Bool, "/return_home", self.cb_return_home, 10)
        self.esc_pub = self.create_publisher(Bool, "/escape_reverse", 10)
        self.stop_pub = self.create_publisher(Int8, "/stop", 10)
        self.rot_pub = self.create_publisher(Float32, "/escape_rotate", 10)   # 到家后让安全门进入停止
        self.reach_sub = self.create_subscription(
            Bool, "/far_reach_goal_status", self.cb_reach_goal, 10)
        self.waypoint_sub = self.create_subscription(
            PointStamped, "/way_point", self.cb_waypoint, 10)

        self.grid = {}          # (ix, iy) -> 0/1/2 (unknown/free/occupied)
        self._blocked = set()   # 障碍膨胀后的格子(机器人净空内), BFS 不可穿
        self.waypoint = None    # far 最新 waypoint
        self.far_stuck_since = 0.0   # far 卡住起始时刻(0=未卡)
        self.spin_done = not self.spin_scan   # 是否已完成启动旋转侦察
        self.spin_t0 = 0.0
        self.spin_last_yaw = None
        self.spin_total = 0.0
        self.pos = (0.0, 0.0)
        self.yaw = 0.0
        self.home = None
        self.odom_frame = "camera_init"
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
        if self.mode != "EXPLORE":
            return
        self.odom_frame = msg.header.frame_id or self.odom_frame
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self.got_odom:
            self.home = self.pos
            self.got_odom = True
            self.get_logger().info(
                f"home 锁存 = ({self.pos[0]:.2f}, {self.pos[1]:.2f})")

    def cb_base_odom(self, msg: Odometry):
        self.base_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.base_home is None:
            self.base_home = self.base_pos
        if self.mode != "EXPLORE":
            self.pos = self.base_pos
            q = msg.pose.pose.orientation
            self.yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


    def cb_waypoint(self, msg: PointStamped):
        self.waypoint = (msg.point.x, msg.point.y)

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

    def _inflate_obstacles(self):
        """把障碍按 reach_inflate 膨胀, 得到 BFS 不可穿过的格子集合(对齐 far 的净空)"""
        r = int(round(self.reach_inflate / RES))
        blocked = set()
        for (ix, iy), s in self.grid.items():
            if s == 2:
                for di in range(-r, r + 1):
                    for dj in range(-r, r + 1):
                        blocked.add((ix + di, iy + dj))
        return blocked

    def _spin_scan_step(self, now):
        """启动旋转侦察: 发一个"始终位于狗右侧90°"的目标, 让 pathFollower 原地右转
        (vx=cos90°≈0, wz=-gain·90° → 只转不走), 累计转满一圈后进入探索。"""
        if self.spin_t0 == 0.0:
            self.spin_t0 = now
            self.get_logger().info(
                f"开始旋转侦察 (目标一直保持在右侧 {self.spin_radius:.1f}m, 累计转满一圈后开始选点)")
        if self.spin_last_yaw is None:
            self.spin_last_yaw = self.yaw
        dyaw = abs((self.yaw - self.spin_last_yaw + np.pi) % (2.0 * np.pi) - np.pi)
        self.spin_total += dyaw
        self.spin_last_yaw = self.yaw
        overshoot = self.spin_total > (2.0 * np.pi + 0.2)
        timeout = (now - self.spin_t0) > self.spin_timeout
        if overshoot or timeout:
            why = "转满一圈" if overshoot else f"超时{self.spin_timeout:.0f}s"
            self.get_logger().info(
                f"旋转侦察完成({why}, 累计转角 {np.degrees(self.spin_total):.0f}°), 开始探索选点")
            self.spin_done = True
            self.current_goal = None
            self.current_goal_cell = None
            return
        # 目标固定在右侧 90°, 让狗持续右转
        ang = self.yaw - np.pi / 2.0
        gx = self.pos[0] + self.spin_radius * np.cos(ang)
        gy = self.pos[1] + self.spin_radius * np.sin(ang)
        self.current_goal = (gx, gy)
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.point.x = gx
        msg.point.y = gy
        msg.point.z = 0.0
        self.goal_pub.publish(msg)
        self.last_pub_key = (round(gx, 2), round(gy, 2))

    def _far_stuck(self, now):
        """far 是否规划卡住: waypoint 一直停在狗位附近(规划失败信号), 而目标还很远.
        真卡住时立刻换目标, 不再等 no_progress_timeout(8s)/goal_timeout(60s)."""
        if self.waypoint is None or self.current_goal is None:
            self.far_stuck_since = 0.0
            return False
        # far 规划失败时发的是 robot_pos_(机身中心), 与 /state_estimation(雷达位) 差 0.3m
        # 所以要和"机身中心"比, 否则相近的正常航点会被误判
        bx = self.pos[0] - 0.3 * np.cos(self.yaw)
        by = self.pos[1] - 0.3 * np.sin(self.yaw)
        dwp = np.hypot(self.waypoint[0] - bx, self.waypoint[1] - by)
        dgoal = np.hypot(self.current_goal[0] - self.pos[0], self.current_goal[1] - self.pos[1])
        if dwp < self.far_stuck_dis and dgoal > 0.5:
            if self.far_stuck_since == 0.0:
                self.far_stuck_since = now
            elif now - self.far_stuck_since > self.far_stuck_timeout:
                self.far_stuck_since = 0.0
                return True
        else:
            self.far_stuck_since = 0.0
        return False

    def _bfs(self, gi, gj, check_blocked=True):
        """check_blocked=True: 选点用(要求净空); False: 到达检查用(不再卡膨胀).
        注意: 到达检查必须用 False, 否则"回退无膨胀选出的目标"会被立刻判不可达 → 死循环。"""
        blocked = self._blocked if check_blocked else set()
        si = int(round(self.pos[0]/RES)); sj = int(round(self.pos[1]/RES))
        start = (si, sj) if self.grid.get((si, sj), 0) == 1 else self._nearest_free(si, sj)
        if start is None:
            return None
        if self.grid.get((gi, gj), 0) != 1 or (gi, gj) in blocked:
            return None
        seen = {start}; q = deque([(start[0], start[1], 0)])
        while q:
            cx, cy, d = q.popleft()
            if (cx, cy) == (gi, gj):
                return d * RES
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                nb = (cx+dx, cy+dy)
                if nb not in seen and self.grid.get(nb, 0) == 1 and nb not in blocked:
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

    def cb_return_home(self, msg):
        if msg.data and self.mode == "EXPLORE" and self.got_odom:
            self._begin_return("收到返航指令")

    def _begin_return(self, reason):
        if self.base_pos is None:
            self.get_logger().warn("尚未收到机身中心里程计，等待返航")
            return
        self.pos = self.base_pos
        self.home = self.base_home
        self.esc_pub.publish(Bool(data=False))
        self.escape_goal = None
        self.mode = "RETURN"
        self.current_goal = None
        self.current_goal_cell = None
        self.return_goal = None
        self.return_recovery_goal = None
        # 记录时的地图还不完整(墙可能稍后才进图), 这里用"现在的"地图再推一次
        if getattr(self, "record_push_obs", False):
            before = [tuple(p) for p in self.path_hist]
            pushed = [self._push_away_from_obstacles(x, y) for x, y in self.path_hist]
            moved = sum(1 for a, b in zip(before, pushed) if np.hypot(a[0]-b[0], a[1]-b[1]) > 0.05)
            self.path_hist = pushed
            if moved:
                self.get_logger().info(
                    f"返航前把 {moved}/{len(pushed)} 个路径点推离障碍 (margin {self.return_goal_margin:.2f}m)")
        self._trim_return_tail()
        self._log_return_state(reason)

    def _log_return_state(self, reason):
        """返航触发时打印完整状态, 便于排查"""
        n = len(self.path_hist)
        d_home = np.hypot(self.pos[0]-self.home[0], self.pos[1]-self.home[1]) if self.home else -1
        # 统计路径总长和首末点
        if n >= 2:
            pts = np.array(self.path_hist)
            plen = np.sum(np.hypot(*np.diff(pts, axis=0).T))
            first, last = self.path_hist[0], self.path_hist[-1]
        else:
            plen, first, last = 0.0, None, None
        self.get_logger().warn(
            f"==== {reason} → 进入返航 ====\n"
            f"  当前位姿: ({self.pos[0]:.2f},{self.pos[1]:.2f}) yaw={np.degrees(self.yaw):.0f}°\n"
            f"  起点(家): ({self.home[0]:.2f},{self.home[1]:.2f}) 距家 {d_home:.2f}m\n"
            f"  记录路径: {n} 个点, 总长 {plen:.2f}m\n"
            f"    首点 {first} / 末点 {last}\n"
            f"  参数: step={self.path_record_step:.2f}m arrive={self.return_arrive_dis:.2f}m "
            f"stuck={self.return_stuck_sec:.0f}s 消环半径={self.loop_close_radius:.1f}m\n"
            f"  预计返航 {plen/max(0.08,0.001):.0f}s (按 0.08m/s 估算)")

    def _trim_return_tail(self):
        """裁掉路径尾部那些"已经站在上面"的点。
        最后一个记录点就是狗当前位置(距 0.2m), 而 return_arrive_dis=0.15,
        差几厘米判不到达 -> localPlanner 出不了路径 -> 永远卡在第一个点。"""
        n0 = len(self.path_hist)
        last = return_tail_index(self.path_hist, self.pos, self.return_trim_dis)
        del self.path_hist[last+1:]
        n1 = len(self.path_hist)
        if n1 != n0:
            self.get_logger().info(
                f"返航前裁掉 {n0-n1} 个已到达的尾部点 (阈值 {self.return_trim_dis:.2f}m), 剩 {n1} 个")

    def _cell_clear(self, cx, cy, margin):
        """该栅格周围 margin(m) 内是否没有障碍(网格 2=障碍)。"""
        m = max(1, int(round(margin / RES)))
        for di in range(-m, m+1):
            for dj in range(-m, m+1):
                if self.grid.get((cx+di, cy+dj), 0) == 2:
                    return False
        return True

    def _push_away_from_obstacles(self, x, y):
        """把点推到"周围 return_goal_margin 内无障碍"的最近自由格。
        返航目标贴墙 -> localPlanner 一定规划不进去, 所以记录时就要避开。"""
        if not getattr(self, "record_push_obs", False):
            return (x, y)
        margin = getattr(self, "return_goal_margin", 0.40)
        ci, cj = int(round(x/RES)), int(round(y/RES))
        if self._cell_clear(ci, cj, margin):
            return (x, y)
        lim = max(2, int(round(margin/RES)) + 4)
        for rad in range(1, lim+1):
            best, bestd = None, 1e9
            for di in range(-rad, rad+1):
                for dj in range(-rad, rad+1):
                    if max(abs(di), abs(dj)) != rad:
                        continue
                    ni, nj = ci+di, cj+dj
                    if self.grid.get((ni, nj), 0) != 1:      # 必须是自由格
                        continue
                    if not self._cell_clear(ni, nj, margin):
                        continue
                    dd = np.hypot(di, dj)
                    if dd < bestd:
                        bestd, best = dd, (ni*RES, nj*RES)
            if best is not None:
                return best
        return (x, y)

    def _record_path(self):
        if self.base_pos is None:
            return
        if self.last_rec is None:
            self.path_hist.append(tuple(self.base_home))
            self.last_rec = tuple(self.base_pos)
        elif np.hypot(self.base_pos[0]-self.last_rec[0], self.base_pos[1]-self.last_rec[1]) >= self.path_record_step:
            self.path_hist.append(self._push_away_from_obstacles(*self.base_pos))
            self.last_rec = tuple(self.base_pos)

    def _select_return_recovery(self):
        """从当前机身前方的连通自由空间选择临时移动点。"""
        self._blocked = self._inflate_obstacles()
        distances = self._reachable_distances()
        previous = self.return_recovery_goal
        best_score = -float("inf")
        best_goal = None
        for (ix, iy), path_distance in distances.items():
            gx, gy = ix * RES, iy * RES
            dx, dy = gx - self.pos[0], gy - self.pos[1]
            forward = dx * np.cos(self.yaw) + dy * np.sin(self.yaw)
            lateral = -dx * np.sin(self.yaw) + dy * np.cos(self.yaw)
            distance = np.hypot(dx, dy)
            # 复用现有选点距离，前方扇区内优先选择直达的短移动。
            if not (2 * self.return_arrive_dis < distance <= self.min_goal_dis):
                continue
            if forward <= 0 or abs(lateral) > forward:
                continue
            if not self._cell_clear(ix, iy, getattr(self, "return_goal_margin", 0.40)):
                continue            # 临时点也必须离墙够远, 否则 localPlanner 同样进不去
            if previous is not None and np.hypot(gx-previous[0], gy-previous[1]) <= self.return_arrive_dis:
                continue
            score = 2 * forward - abs(lateral) - path_distance
            if score > best_score:
                best_score = score
                best_goal = (gx, gy)
        return best_goal

    def _return_step(self):
        if self.mode == "DONE":
            return
        now = time.time()
        d_home = np.hypot(self.pos[0]-self.home[0], self.pos[1]-self.home[1])
        # 到家用独立阈值(家常贴在墙边, 0.15m 太严, 实测曾到达 0.16m 判失败)
        if d_home <= getattr(self, "home_arrive_dis", 0.35):
            self.mode = "DONE"
            self.return_goal = tuple(self.home)
            self.return_recovery_goal = None
            self.get_logger().info(
                f"返航到位：实际位置距家 {d_home:.2f}m (阈值 {getattr(self,'home_arrive_dis',0.35):.2f}m)")
            # 关键: DONE 之后 far/localPlanner/pathFollower 仍在跑, 会继续按最后一个航点驱动
            #       必须主动让安全门停下, 否则狗到家后还会继续走
            if getattr(self, "stop_on_done", True) and not getattr(self, "_done_stopped", False):
                self._done_stopped = True
                try:
                    self.esc_pub.publish(Bool(data=False))
                    self.stop_pub.publish(Int8(data=1))
                    self.get_logger().warn("已发送 /stop, 安全门进入停止(到家后不再移动)")
                except AttributeError:
                    pass      # 回归测试替身没有发布器
        else:
            target = self.return_recovery_goal or self.return_goal
            if target is not None:
                distance = np.hypot(self.pos[0]-target[0], self.pos[1]-target[1])
                if distance <= self.return_arrive_dis:
                    if self.return_recovery_goal is not None:
                        self.return_recovery_goal = None
                        self.return_best = np.hypot(
                            self.pos[0]-self.return_goal[0], self.pos[1]-self.return_goal[1])
                        self.return_t = now
                        self.get_logger().info("已换到新位置，根据当前地图重新规划到起点")
                    else:
                        self.return_goal = None
                    self.mode = "RETURN"
                elif distance < self.return_best - 0.10:
                    self.return_best = distance
                    self.return_t = now
                    self.mode = "RETURN_REPOSITION" if self.return_recovery_goal else "RETURN"
                elif now - self.return_t >= self.return_stuck_sec:
                    # 受阻后以起点为全局目标，由 FAR 根据当前地图连接回去。
                    self.return_goal = tuple(self.home)
                    recovery = self._select_return_recovery()
                    self.return_recovery_goal = recovery
                    self.return_t = now
                    if recovery is not None:
                        self.blocked_count = 0
                        self.return_best = np.hypot(
                            self.pos[0]-recovery[0], self.pos[1]-recovery[1])
                        self.mode = "RETURN_REPOSITION"
                        self.get_logger().warn(
                            f"返航超时，移动至前方 {recovery} 后按当前地图规划回起点 {self.home}")
                    else:
                        self.mode = "RETURN_BLOCKED"
                        self.return_best = np.hypot(
                            self.pos[0]-self.return_goal[0], self.pos[1]-self.return_goal[1])
                        self.blocked_count += 1
                        self.get_logger().warn(
                            f"返航受阻，前方没有已知可通行的临时目标 (连续 {self.blocked_count} 次)")
                        # 连续受阻: 后退一小段, 让车头相对环境改变后再选点
                        # (不能放宽方向约束 —— 队友的 test_forward_recovery_does_not_cross_wall 锁死了"只选前方")
                        if (self.return_blocked_reverse and
                                self.blocked_count >= self.return_blocked_after):
                            self.return_reverse_until = now + 2.0
                            self.blocked_count = 0
                            self.get_logger().warn(
                                f"连续受阻 -> 后退 {self.return_blocked_dist:.2f}m 换位置")
            if self.return_goal is None:
                self.return_goal = self.path_hist.pop() if self.path_hist else tuple(self.home)
                self.return_best = np.hypot(
                    self.pos[0]-self.return_goal[0], self.pos[1]-self.return_goal[1])
                self.return_t = now
        target = self.return_recovery_goal or self.return_goal
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.odom_frame
        m.point.x, m.point.y, m.point.z = *target, 0.0
        self.goal_pub.publish(m)

    def _reachable_distances(self):
        """Compute one BFS distance field for all NBV candidates."""
        si = int(round(self.pos[0]/RES)); sj = int(round(self.pos[1]/RES))
        start = (si,sj) if self.grid.get((si,sj),0)==1 else self._nearest_free(si,sj)
        if start is None:return {}
        distances={start:0.0}; pending=deque([start])
        while pending:
            cell=pending.popleft()
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nb=(cell[0]+dx,cell[1]+dy)
                if nb not in distances and self.grid.get(nb,0)==1 and nb not in self._blocked:
                    distances[nb]=distances[cell]+RES
                    pending.append(nb)
        return distances

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

        distances = self._reachable_distances()
        candidates = []
        for (gix, giy) in free_cells:
            gx, gy = gix * RES, giy * RES
            if self._is_cooldown(gix, giy):
                continue
            # 硬门槛: 欧氏距离
            eu = np.hypot(gx - self.pos[0], gy - self.pos[1])
            if eu < self.min_goal_dis:
                continue
            # 硬限制: 3x3 邻域必须全是已知 free(不能是前沿/未知)
            # 否则 far_planner 的可见图连不到该点, 会 0ms 返回空路径 -> 狗停住
            if self.require_interior:
                # 3x3 内至少 interior_min_free 个已知 free 即可
                # (原来要求全 9 个, 可选点被限制在已探明区域最内部, 无法向外扩)
                nfree = 0
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        if self.grid.get((gix + di, giy + dj), 0) == 1:
                            nfree += 1
                if nfree < self.interior_min_free:
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
            bfs = distances.get((gix,giy))
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
            # 深度优先: 目标比狗更靠近起点 -> 降权(避免探索途中往回走)
            if self.home is not None and self.back_penalty < 1.0:
                d_goal_home = np.hypot(gx - self.home[0], gy - self.home[1])
                d_robot_home = np.hypot(self.pos[0] - self.home[0], self.pos[1] - self.home[1])
                if d_goal_home < d_robot_home - 0.30:
                    score *= self.back_penalty
            candidates.append((score, bfs, gx, gy, gix, giy, info, ret, risk, turn, eu))

        if not candidates:
            return None
        # 朝向硬约束: 优先只在"身体前方 ±goal_front_deg"内选点(避免选身后→原地掉头)
        # 前方完全无候选时退回全量, 以便脱困
        # 三档兜底: ±goal_front_deg -> ±135° -> 全部
        # (避免启动时地图太空 -> 直接选身后目标 -> 掉头148°)
        _front = np.radians(self.goal_front_deg)
        front = [c for c in candidates if c[9] <= _front]
        if not front:
            front = [c for c in candidates if c[9] <= np.radians(135.0)]
        pool = front if front else candidates
        pool.sort(key=lambda c: -c[0])
        best = pool[0]
        return dict(score=best[0], path=best[1], gx=best[2], gy=best[3],
                    gix=best[4], giy=best[5], info=best[6], ret=best[7],
                    risk=best[8], turn=best[9], eu=best[10])

    # ---- 主循环 ----
    def loop(self):
        state = String(); state.data = self.mode; self.status_pub.publish(state)
        if not self.got_odom or not self.grid:
            return
        now = time.time()          # 逃生/计时统一用这个
        if getattr(self, "mode", "EXPLORE") in ("RETURN", "RETURN_BLOCKED", "RETURN_REPOSITION", "DONE"):
            # 返航转身: 目标偏车头 >return_face_deg 时先纯旋转
            # (否则 pathFollower 会"一边前进一边满舵转向" -> 画弧撞墙)
            if self.mode in ("RETURN", "RETURN_REPOSITION") and self.mode != "DONE":
                _t = getattr(self, "return_recovery_goal", None) or getattr(self, "return_goal", None)
                if _t is not None:
                    _err = float(np.arctan2(_t[1]-self.pos[1], _t[0]-self.pos[0]) - self.yaw)
                    _err = (_err + np.pi) % (2*np.pi) - np.pi
                    _need = abs(_err) * 180.0 / np.pi > getattr(self, "return_face_deg", 60.0)
                    try:
                        self.rot_pub.publish(Float32(data=(1.0 if _err > 0 else -1.0) if _need else 0.0))
                    except AttributeError:
                        pass
                    if _need:
                        return          # 转身期间不发返航目标, 交给安全门纯旋转
            try:
                self.rot_pub.publish(Float32(data=0.0))
            except AttributeError:
                pass
            # 返航受阻后的小幅后退: 直接走安全门脱困通道, 绕过 FAR
            if time.time() < getattr(self, "return_reverse_until", 0.0) and self.mode != "DONE":
                self.esc_pub.publish(Bool(data=True))
                return
            if getattr(self, "return_reverse_until", 0.0) > 0.0:
                self.esc_pub.publish(Bool(data=False))
                self.return_reverse_until = 0.0
            self._return_step()
            return
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time - self.startup_delay
        if self.return_after > 0 and elapsed > self.return_after:
            self._begin_return(f"自动返航(已探索 {elapsed:.0f}s)")
            self._return_step()
            return
        self._record_path()

        # 卡住逃生: 主动发一个身后的目标, 让狗后退一段
        if self.escape_enable:
            if self.escape_goal is not None:
                if now < self.escape_until and \
                   np.hypot(self.pos[0]-self.escape_goal[0], self.pos[1]-self.escape_goal[1]) > 0.15:
                    self.esc_pub.publish(Bool(data=True))   # 让安全门直接倒车
                    # 同时发当前位置当目标, 避免 far 继续追旧目标
                    m = PointStamped()
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.header.frame_id = "map"
                    m.point.x, m.point.y, m.point.z = self.pos[0], self.pos[1], 0.0
                    self.goal_pub.publish(m)
                    return
                self.esc_pub.publish(Bool(data=False))
                self.get_logger().warn("后退脱困结束, 恢复探索选点")
                self.escape_goal = None
                self.current_goal = None
                self.progress_ref = (self.pos[0], self.pos[1])
                self.progress_ref_time = now
            elif self.current_goal is not None:
                dp = np.hypot(self.pos[0]-self.progress_ref[0], self.pos[1]-self.progress_ref[1])
                if dp < 0.05 and (now - self.progress_ref_time) > self.escape_after:
                    # 狗在后退方向 0.4m 处设临时目标(twoWayDrive已关, 目标在身后 -> pathFollower会转身;
                    # 所以这里直接用于 far -> local -> 反向路径不可用, 改为发"身后目标"让 far 规划转向)
                    ang = self.yaw + np.pi
                    self.escape_goal = (self.pos[0] + self.escape_dist*np.cos(ang),
                                        self.pos[1] + self.escape_dist*np.sin(ang))
                    self.escape_until = now + max(1.5, self.escape_dist / 0.20)   # 按距离/速度估时, 不再固定 6s(会退 1.5m)
                    self.get_logger().warn(
                        f"卡住 {(now-self.progress_ref_time):.0f}s, 触发后退脱困 -> "
                        f"({self.escape_goal[0]:.2f},{self.escape_goal[1]:.2f})")
                    return

        if now - self.start_time < self.startup_delay:
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.odom_frame
            msg.point.x, msg.point.y, msg.point.z = self.pos[0], self.pos[1], 0.0
            self.goal_pub.publish(msg)   # 发当前位置 -> 保持原地
            return
        now = time.time()
        self._mark_free_footprint()
        self._blocked = self._inflate_obstacles()   # 每轮重建膨胀障碍(对齐 far 净空)

        # ---- 启动阶段: 先原地转一圈看清环境, 再开始选点 ----
        if not self.spin_done:
            self._spin_scan_step(now)
            return

        if self.current_goal is not None:
            far_reach = self.reach_votes >= self.reach_vote_thre
            d = self._bfs(*self.current_goal_cell, check_blocked=False) if self.current_goal_cell is not None else None
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
            elif self._far_stuck(now):
                if self.current_goal_cell is not None:
                    self._penalize_goal(*self.current_goal_cell)
                self.get_logger().warn(
                    f"far 卡住 {self.far_stuck_timeout:.1f}s (waypoint≈狗位), 立刻换目标")
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
            if res is None and self._blocked:
                # 膨胀后无可达点 → 回退无膨胀(避免趴窝), 但记录警告
                self.get_logger().warn(
                    f"膨胀 {self.reach_inflate:.2f}m 后无可达点, 回退无膨胀选点")
                self._blocked = set()
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
                self.far_stuck_since = 0.0
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
            msg.header.frame_id = self.odom_frame
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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
