#!/usr/bin/env python3
"""碰撞规避验证: 路径 vs 地形障碍最小距离"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class C(Node):
    def __init__(self):
        super().__init__("collision_check")
        self.path = None
        self.odom = None
        self.obstacles = []   # (x,y,z) 障碍点(intensity>0.3)
        self.create_subscription(Path, "/path", self.cb_path, 10)
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 10)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terrain, 10)
    def cb_path(self, m):
        self.path = m
    def cb_odom(self, m):
        self.odom = m
    def cb_terrain(self, m):
        for p in point_cloud2.read_points(m, field_names=("x","y","z","intensity"), skip_nans=True):
            if p[3] > 0.3:   # obstacleHeightThre
                self.obstacles.append((p[0], p[1], p[2]))

def yaw_from_quat(q):
    siny = 2*(q.w*q.z + q.x*q.y)
    cosy = 1 - 2*(q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)

rclpy.init(); c = C()
print("累积障碍并等待路径...", flush=True)
start = time.time()
while time.time() - start < 45:
    rclpy.spin_once(c, timeout_sec=1.0)
    if c.path is not None and c.odom is not None and len(c.obstacles) > 100:
        break
rclpy.shutdown()

if c.path is None or c.odom is None:
    print("NO_PATH_OR_ODOM", flush=True); sys.exit(1)

# 车辆位姿(map 系) 与航向
px = c.odom.pose.pose.position.x
py = c.odom.pose.pose.position.y
yaw = yaw_from_quat(c.odom.pose.pose.orientation)

# 路径航点(vehicle 系) 变换到 map 系
path_pts = []
for pose in c.path.poses:
    lx = pose.pose.position.x
    ly = pose.pose.position.y
    # 旋转到 map 系
    mx = px + lx*math.cos(yaw) - ly*math.sin(yaw)
    my = py + lx*math.sin(yaw) + ly*math.cos(yaw)
    path_pts.append((mx, my))

obstacles = np.array(c.obstacles)  # (N,3)
print(f"障碍点数 = {len(obstacles)}", flush=True)
print(f"路径航点数 = {len(path_pts)}", flush=True)
print(f"车辆位置(map系) = ({px:.2f}, {py:.2f}) yaw={math.degrees(yaw):.1f}°", flush=True)
print(f"路径起点(map系) = ({path_pts[0][0]:.2f}, {path_pts[0][1]:.2f})", flush=True)
print(f"路径终点(map系) = ({path_pts[-1][0]:.2f}, {path_pts[-1][1]:.2f})", flush=True)

# 每个路径航点到最近障碍的距离(只看 XY 平面)
min_dists = []
for mx, my in path_pts:
    if len(obstacles) == 0:
        break
    d = np.sqrt((obstacles[:,0]-mx)**2 + (obstacles[:,1]-my)**2)
    min_dists.append(d.min())

if min_dists:
    md = np.array(min_dists)
    print(f"路径-障碍最小距离: 最小={md.min():.3f}m 中位={np.median(md):.3f}m", flush=True)
    # vehicleWidth=0.7 → 半宽 0.35m, 留 0.1m 余量 → 阈值 0.45m
    collide = (md.min() < 0.45)
    print(f"碰撞判定: {'⚠️ 有碰撞风险(最小距离<0.45m)' if collide else '✅ 无碰撞(最小距离≥0.45m)'}", flush=True)
else:
    print("无障碍点或路径空", flush=True)
print("COLLISION_CHECK_DONE", flush=True)
