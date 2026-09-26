#!/usr/bin/env python3
"""实时订阅 /terrain_map 计算量化指标(与 analyze_terrain_bag.py 同口径)"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class A(Node):
    def __init__(self):
        super().__init__("terrain_analyzer")
        self.msgs = 0
        self.t0 = None
        self.per_frame = []      # 每帧点数
        self.per_frame_measured = []
        self.per_frame_unknown = []
        self.intensities = []    # measured 高度
        self.unknown_pts = 0
        self.total_pts = 0
        self.frames = set()
        self.create_subscription(PointCloud2, "/terrain_map", self.cb, 10)

    def cb(self, m):
        if self.t0 is None:
            self.t0 = time.time()
        self.msgs += 1
        self.frames.add(m.header.frame_id)
        pts = list(point_cloud2.read_points(m, field_names=("x","y","z","intensity"), skip_nans=True))
        n = len(pts)
        self.per_frame.append(n)
        unk = 0; meas_n = 0
        for p in pts:
            it = p[3]
            self.total_pts += 1
            if abs(it - 1.5) < 1e-4:
                unk += 1; self.unknown_pts += 1
            else:
                meas_n += 1
                self.intensities.append(it)
        self.per_frame_measured.append(meas_n)
        self.per_frame_unknown.append(unk)

def pct(a, q):
    return np.percentile(a, q)

rclpy.init()
a = A()
print("analyzing /terrain_map ...", flush=True)
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60
start = time.time()
while time.time() - start < DUR:
    rclpy.spin_once(a, timeout_sec=1.0)
rclpy.shutdown()

if a.msgs == 0:
    print("NO_DATA", flush=True); sys.exit(1)

dur = time.time() - a.t0
it = np.array(a.intensities)
pf = np.array(a.per_frame)
print("=== TERRAIN MAP SUMMARY ===", flush=True)
print(f"message_count = {a.msgs}", flush=True)
print(f"duration_s = {dur:.3f}", flush=True)
print(f"average_hz = {a.msgs/dur:.3f}", flush=True)
print(f"frame_ids = {sorted(a.frames)}", flush=True)
print(f"=== POINTS PER FRAME ===", flush=True)
print(f"all: min={pf.min()} median={np.median(pf):.0f} p95={pct(pf,95):.0f} max={pf.max()}", flush=True)
pm = np.array(a.per_frame_measured); pu = np.array(a.per_frame_unknown)
print(f"measured: min={pm.min()} median={np.median(pm):.0f} p95={pct(pm,95):.0f} max={pm.max()}", flush=True)
print(f"unknown_injected: min={pu.min()} median={np.median(pu):.0f} p95={pct(pu,95):.0f} max={pu.max()}", flush=True)
print(f"=== MEASURED HEIGHT INTENSITY ===", flush=True)
print(f"measured_sample_count = {len(it)}", flush=True)
for q in [50, 75, 90, 95, 99, 99.9]:
    print(f"p{q}_m = {pct(it,q):.5f}", flush=True)
print(f"max_m = {it.max():.5f}", flush=True)
for th in [0.05, 0.10, 0.20, 0.30]:
    print(f"ratio_above_{th:.2f}m = {(it>th).sum()/len(it)*100:.3f}%", flush=True)
print(f"=== UNKNOWN AREA ===", flush=True)
print(f"total_points = {a.total_pts}", flush=True)
print(f"unknown_injected_points = {a.unknown_pts}", flush=True)
print(f"unknown_ratio = {a.unknown_pts/a.total_pts*100:.3f}%", flush=True)
print("LIVE_ANALYSIS_DONE", flush=True)
