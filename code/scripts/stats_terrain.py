#!/usr/bin/env python3
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2

class C(Node):
    def __init__(self):
        super().__init__("ts")
        self.pts = []; self.traj = []
        self.create_subscription(PointCloud2, "/terrain_map", self.ct, 10)
        self.create_subscription(Odometry, "/state_estimation", self.co, 10)
    def ct(self, m):
        for p in point_cloud2.read_points(m, field_names=("x","y","z","intensity"), skip_nans=True):
            self.pts.append((p[0],p[1],p[2],p[3]))
    def co(self, m):
        self.traj.append((m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))

rclpy.init(); n = C()
start = time.time()
while time.time()-start < 50:
    rclpy.spin_once(n, timeout_sec=1.0)
rclpy.shutdown()
a = np.array(n.pts)
t = np.array(n.traj)
print(f"terrain 点数: {len(a)}")
print(f"轨迹点数: {len(t)}")
if len(a):
    print(f"X 范围: [{a[:,0].min():.2f}, {a[:,0].max():.2f}] m")
    print(f"Y 范围: [{a[:,1].min():.2f}, {a[:,1].max():.2f}] m")
    print(f"Z 范围: [{a[:,2].min():.2f}, {a[:,2].max():.2f}] m")
    it = a[:,3]
    print(f"intensity(相对地面高度): min={it.min():.3f} P50={np.median(it):.3f} P90={np.percentile(it,90):.3f} P99={np.percentile(it,99):.3f} max={it.max():.3f}")
    # 高度分层
    low=(it<0.05).sum(); mid=((it>=0.05)&(it<0.5)).sum(); high=(it>=0.5).sum()
    print(f"高度分布: <5cm={low/len(a)*100:.1f}%  5-50cm={mid/len(a)*100:.1f}%  >50cm={high/len(a)*100:.1f}%")
if len(t):
    print(f"轨迹范围 X:[{t[:,0].min():.2f},{t[:,0].max():.2f}] Y:[{t[:,1].min():.2f},{t[:,1].max():.2f}] 起终点距离={np.linalg.norm(t[-1]-t[0]):.3f} m")
