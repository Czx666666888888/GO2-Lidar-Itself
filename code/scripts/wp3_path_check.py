#!/usr/bin/env python3
"""订阅 /path 抓取完整路径, 统计航点数与坐标"""
import sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class C(Node):
    def __init__(self):
        super().__init__("path_check")
        self.paths = []
        self.create_subscription(Path, "/path", self.cb, 10)
    def cb(self, m):
        self.paths.append(m)

rclpy.init(); c = C()
print("等待 /path ...", flush=True)
start = time.time()
while time.time() - start < 40:
    rclpy.spin_once(c, timeout_sec=1.0)
    if c.paths:
        break
rclpy.shutdown()
if not c.paths:
    print("NO_PATH", flush=True); sys.exit(1)
p = c.paths[-1]
print(f"frame_id = {p.header.frame_id}", flush=True)
print(f"航点数 = {len(p.poses)}", flush=True)
if p.poses:
    first = p.poses[0].pose.position
    last = p.poses[-1].pose.position
    print(f"起点 = ({first.x:.2f}, {first.y:.2f})", flush=True)
    print(f"终点 = ({last.x:.2f}, {last.y:.2f})", flush=True)
    print("前5个航点:", flush=True)
    for i in range(min(5, len(p.poses))):
        q = p.poses[i].pose.position
        print(f"  [{i}] ({q.x:.2f}, {q.y:.2f})", flush=True)
