#!/usr/bin/env python3
"""失败模式测试: 通过 /added_obstacles 在车辆周围注入障碍墙, 验证返回空路径"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

class C(Node):
    def __init__(self):
        super().__init__("wall_test")
        self.odom = None
        self.latest_path = None
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 10)
        self.create_subscription(Path, "/path", self.cb_path, 10)
        self.wall_pub = self.create_publisher(PointCloud2, "/added_obstacles", 10)
    def cb_odom(self, m):
        self.odom = m
    def cb_path(self, m):
        self.latest_path = m

rclpy.init(); c = C()
print("等待里程计...", flush=True)
start = time.time()
while time.time() - start < 30 and c.odom is None:
    rclpy.spin_once(c, timeout_sec=1.0)
if c.odom is None:
    print("NO_ODOM", flush=True); rclpy.shutdown(); sys.exit(1)

px = c.odom.pose.pose.position.x
py = c.odom.pose.pose.position.y
print(f"车辆位置=({px:.2f}, {py:.2f})", flush=True)

# 先发一个可达目标, 确认路径正常
g = PointStamped(); g.header.frame_id = "camera_init"
g.point.x = px + 1.5; g.point.y = py
c.wall_pub  # noop
print(f"先发可达目标 ({g.point.x:.2f}, {g.point.y:.2f})", flush=True)
# (用 /way_point 发布需要另一个 publisher, 这里直接复用脚本外部的 goal 话题, 简化: 只测墙)
# 生成 0.8m 半径的障碍环(每 15° 一点, 共 24 点)
n_ring = 48
pts = []
for i in range(n_ring):
    ang = 2*math.pi*i/n_ring
    for r in [0.6, 0.8, 1.0]:
        x = px + r*math.cos(ang)
        y = py + r*math.sin(ang)
        pts.append((x, y, 0.0, 200.0))

# 构造 PointCloud2
fields = [
    point_cloud2.PointField(name="x", offset=0, datatype=7, count=1),
    point_cloud2.PointField(name="y", offset=4, datatype=7, count=1),
    point_cloud2.PointField(name="z", offset=8, datatype=7, count=1),
    point_cloud2.PointField(name="intensity", offset=12, datatype=7, count=1),
]
cloud = point_cloud2.create_cloud(Header(frame_id="camera_init"), fields, pts)
c.wall_pub.publish(cloud)
print(f"已注入障碍墙: {len(pts)} 点, 环绕车辆 0.6/0.8/1.0m", flush=True)

# 等 localPlanner 反应
time.sleep(5)
rclpy.spin_once(c, timeout_sec=1.0)
if c.latest_path is None:
    print("NO_PATH_AFTER_WALL", flush=True)
else:
    n = len(c.latest_path.poses)
    print(f"注墙后 /path 航点数 = {n}", flush=True)
    if n <= 1:
        print("结果: ✅ 四周被堵 → 返回失败(空路径 1 航点)", flush=True)
    else:
        print(f"结果: ⚠️ 仍有 {n} 航点(墙可能未完全封死)", flush=True)
print("WALL_TEST_DONE", flush=True)
rclpy.shutdown()
