#!/usr/bin/env python3
"""验证 base_odom_node: /state_estimation 与 /base_state_estimation 的偏移应约 0.3m
用法: python3 verify_base_odom.py <时长秒>
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40

class C(Node):
    def __init__(self):
        super().__init__("verify_base_odom")
        self.raw = []
        self.base = []
        self.create_subscription(Odometry, "/state_estimation", self.cb_raw, 10)
        self.create_subscription(Odometry, "/base_state_estimation", self.cb_base, 10)
    def cb_raw(self, m):
        self.raw.append((m.pose.pose.position.x, m.pose.pose.position.y))
    def cb_base(self, m):
        self.base.append((m.pose.pose.position.x, m.pose.pose.position.y))

rclpy.init(); c = C()
print(f"采集 {DUR}s ...", flush=True)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(c, timeout_sec=0.5)
rclpy.shutdown()

if not c.raw or not c.base:
    print(f"数据不足: raw={len(c.raw)} base={len(c.base)}", flush=True)
    sys.exit(1)

# 按数量取交集(前 N 个对齐)
n = min(len(c.raw), len(c.base))
raw = np.array(c.raw[:n]); base = np.array(c.base[:n])
d = np.linalg.norm(base - raw, axis=1)
print(f"样本 {n} 帧", flush=True)
print(f"|base - raw| 偏移: 均值 {d.mean():.3f}m 中位 {np.median(d):.3f}m 最大 {d.max():.3f}m", flush=True)
print(f"期望 ≈0.30m (传感器前向偏置)", flush=True)
print("结论: " + ("✓ 偏移约0.3m, 转换正确" if 0.25 < d.mean() < 0.35 else
                "⚠️ 偏移异常, 需检查"), flush=True)
