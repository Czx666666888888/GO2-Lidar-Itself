#!/usr/bin/env python3
"""方案B: 合成障碍场景验证路径规划
场景1: 车辆前方 1m 处注入一堵墙, 目标在墙后 2m, 验证路径绕行
场景2: 三面墙(死胡同), 验证返回失败
"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

SCENARIO = int(sys.argv[1]) if len(sys.argv) > 1 else 1

class C(Node):
    def __init__(self):
        super().__init__("synth_obstacle_test")
        self.odom = None; self.path = None
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 10)
        self.create_subscription(Path, "/path", self.cb_path, 10)
        self.obs_pub = self.create_publisher(PointCloud2, "/added_obstacles", 10)
        self.goal_pub = self.create_publisher(PointStamped, "/way_point", 10)
    def cb_odom(self, m): self.odom = m
    def cb_path(self, m): self.path = m

def make_cloud(frame, pts):
    fields = [point_cloud2.PointField(name=n, offset=i*4, datatype=7, count=1)
              for i, n in enumerate(("x","y","z","intensity"))]
    return point_cloud2.create_cloud(Header(frame_id=frame), fields, pts)

rclpy.init(); c = C()
print(f"场景 {SCENARIO} 启动, 等待里程计...", flush=True)
start = time.time()
while time.time()-start < 30 and c.odom is None:
    rclpy.spin_once(c, timeout_sec=1.0)
if c.odom is None:
    print("NO_ODOM", flush=True); sys.exit(1)

px, py = c.odom.pose.pose.position.x, c.odom.pose.pose.position.y
yaw = math.atan2(2*(c.odom.pose.pose.orientation.w*c.odom.pose.pose.orientation.z+
                    c.odom.pose.pose.orientation.x*c.odom.pose.pose.orientation.y),
                 1-2*(c.odom.pose.pose.orientation.y**2+c.odom.pose.pose.orientation.z**2))
print(f"车辆=({px:.2f},{py:.2f}) yaw={math.degrees(yaw):.0f}°", flush=True)

# 车辆前方方向
fx, fy = math.cos(yaw), math.sin(yaw)
rx, ry = -fy, fx   # 右侧方向

obs_pts = []
goal_x, goal_y = 0, 0

if SCENARIO == 1:
    # 挡墙: 前方 1.5m, 横向 ±2.5m, 高 0.3m
    for s in np.arange(-1.0, 1.01, 0.1):
        for z in [0.0, 0.3]:
            obs_pts.append((px + 1.5*fx + s*rx, py + 1.5*fy + s*ry, z, 200.0))
    goal_x, goal_y = px + 3.0*fx, py + 3.0*fy   # 目标在墙后
elif SCENARIO == 2:
    # 死胡同: 前方1.5m墙 + 左右各1.5m墙(围成U形)
    for s in np.arange(-1.5, 1.51, 0.1):
        obs_pts.append((px + 1.5*fx + s*rx, py + 1.5*fy + s*ry, 0.0, 200.0))  # 前墙
    for s in np.arange(0, 1.51, 0.1):
        obs_pts.append((px + 1.5*rx + s*fx, py + 1.5*ry + s*fy, 0.0, 200.0))  # 右墙
        obs_pts.append((px - 1.5*rx + s*fx, py - 1.5*ry + s*fy, 0.0, 200.0))  # 左墙
    goal_x, goal_y = px + 3.0*fx, py + 3.0*fy

# 发布障碍
cloud = make_cloud("camera_init", obs_pts)
c.obs_pub.publish(cloud)
print(f"已注入 {len(obs_pts)} 个障碍点", flush=True)
time.sleep(1)

# 发布目标
g = PointStamped(); g.header.frame_id = "camera_init"
g.point.x, g.point.y, g.point.z = goal_x, goal_y, 0.0
c.goal_pub.publish(g)
print(f"已发布目标 ({goal_x:.2f},{goal_y:.2f})", flush=True)

start2 = time.time()
while time.time()-start2 < 8:
    rclpy.spin_once(c, timeout_sec=0.5)
if c.path is None:
    print("结果: 无路径", flush=True)
else:
    n = len(c.path.poses)
    print(f"结果: /path {n} 航点", flush=True)
    if n <= 1:
        print(f"场景{SCENARIO}: ✅ 无路可走 → 返回失败(空路径)", flush=True)
    else:
        print(f"场景{SCENARIO}: ✅ 生成 {n} 航点路径", flush=True)
print("SYNTH_DONE", flush=True)
rclpy.shutdown()
