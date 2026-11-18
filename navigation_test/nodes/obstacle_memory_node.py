#!/usr/bin/env python3
"""obstacle_memory_node: 累计障碍记忆 → 注入 localPlanner 的 /added_obstacles

解决什么问题:
  localPlanner 只用"当前帧" terrain_map 做避障。狗靠近墙时 LiDAR 掠射角点数变稀疏,
  terrain_analysis 的 minBlockPointNum 会把整块 voxel 丢掉 → 墙在最后一米从地图"消失"
  → localPlanner 把墙当成可通行 → 狗撞墙(拐角擦墙)。

本节点做法:
  累积历史上看到过的障碍(按体素取最高), 持续以 /added_obstacles 发布给 localPlanner。
  localPlanner 现成的 addedObstaclesHandler 会把这些点 intensity 强制设为 200(硬障碍),
  所以即使当前帧看不到, 历史障碍记忆依然生效。

订阅:
  /terrain_map       (PointCloud2, map 系, intensity=相对高度)
  /state_estimation  (Odometry, 用于按距离裁剪)
发布:
  /added_obstacles   (PointCloud2, map 系)

参数:
  obstacle_thre : 相对高度阈值(默认 0.3, 与 localPlanner obstacleHeightThre 一致)
  voxel         : 体素尺寸 m(默认 0.15)
  keep_radius   : 只保留距机器人此半径内的格子(默认 10.0)
  decay_time    : 超过此时长未重扫则删除 s(默认 0=永不删, 适合静态墙)
  publish_rate  : 发布频率 Hz(默认 5.0)

用法:
  ros2 run go2_keyboard_teleop obstacle_memory_node
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


class ObstacleMemoryNode(Node):
    def __init__(self):
        super().__init__('obstacle_memory_node')
        self.declare_parameter('obstacle_thre', 0.3)
        self.declare_parameter('voxel', 0.15)
        self.declare_parameter('keep_radius', 10.0)
        self.declare_parameter('decay_time', 0.0)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('max_cells', 20000)
        self.declare_parameter('unknown_int', 1.45)   # >=该值是 unknown 哨兵, 不能当障碍累计

        self.obstacle_thre = float(self.get_parameter('obstacle_thre').value)
        self.voxel = float(self.get_parameter('voxel').value)
        self.keep_radius = float(self.get_parameter('keep_radius').value)
        self.decay_time = float(self.get_parameter('decay_time').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_cells = int(self.get_parameter('max_cells').value)
        self.unknown_int = float(self.get_parameter('unknown_int').value)

        self.cells = {}          # (ix,iy) -> [height, last_seen]
        self.robot_xy = None

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub_terrain = self.create_subscription(
            PointCloud2, '/terrain_map', self.cb_terrain, qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/base_state_estimation', self.cb_odom, 10)
        self.pub = self.create_publisher(PointCloud2, '/added_obstacles', 5)

        self.timer = self.create_timer(1.0 / max(self.publish_rate, 0.5), self.publish)
        self.get_logger().info(
            f'障碍记忆节点就绪 | 阈值>{self.obstacle_thre}m 体素{self.voxel}m '
            f'保留半径{self.keep_radius}m 衰减{self.decay_time}s')

    # ---------------- 回调 ----------------
    def cb_odom(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def cb_terrain(self, msg: PointCloud2):
        xs, ys, hs = [], [], []
        try:
            for p in point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'intensity'), skip_nans=True):
                if self.obstacle_thre < p[2] < self.unknown_int:   # 只收真障碍: 排地面(<=0.3)和unknown(>=1.45)
                    xs.append(p[0]); ys.append(p[1]); hs.append(p[2])
        except Exception as e:
            self.get_logger().warn(f'读取 terrain_map 失败: {e}', throttle_duration_sec=5.0)
            return
        if not xs:
            return
        x = np.asarray(xs, dtype=np.float32)
        y = np.asarray(ys, dtype=np.float32)
        h = np.asarray(hs, dtype=np.float32)
        ix = np.round(x / self.voxel).astype(np.int64)
        iy = np.round(y / self.voxel).astype(np.int64)
        now = time.time()
        # 同帧同格取最高
        order = np.argsort(-h)
        seen = set()
        for k in order:
            cell = (int(ix[k]), int(iy[k]))
            if cell in seen:
                continue
            seen.add(cell)
            prev = self.cells.get(cell)
            if prev is None or h[k] > prev[0]:
                self.cells[cell] = [float(h[k]), now]
            else:
                prev[1] = now

    # ---------------- 发布 ----------------
    def _prune(self):
        now = time.time()
        rx, ry = self.robot_xy if self.robot_xy else (None, None)
        dead = []
        for cell, (h, ts) in self.cells.items():
            if self.decay_time > 0 and now - ts > self.decay_time:
                dead.append(cell); continue
            if rx is not None:
                dx = cell[0] * self.voxel - rx
                dy = cell[1] * self.voxel - ry
                if dx * dx + dy * dy > self.keep_radius * self.keep_radius:
                    dead.append(cell)
        for c in dead:
            self.cells.pop(c, None)
        if len(self.cells) > self.max_cells:
            items = sorted(self.cells.items(), key=lambda kv: kv[1][1])
            for c, _ in items[: len(self.cells) - self.max_cells]:
                self.cells.pop(c, None)

    def publish(self):
        if not self.cells:
            return
        self._prune()
        pts = [(c[0] * self.voxel, c[1] * self.voxel, h, h)
               for c, (h, _ts) in self.cells.items()]
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = 'map'
        cloud = point_cloud2.create_cloud(hdr, FIELDS, pts)
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMemoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
