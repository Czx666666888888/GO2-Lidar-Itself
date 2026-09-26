#!/usr/bin/env python3
"""订阅 /terrain_map 与 /state_estimation, 累积后渲染俯视高度图 + 轨迹"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2

OUT = "/home/lch/dog/tmp/terrain_effect.png"
DURATION = 55  # 采集时长(秒), 静止包 36s 足够
MAX_PTS = 120000  # 渲染前下采样点数

class Collector(Node):
    def __init__(self):
        super().__init__("terrain_collector")
        self.pts = []
        self.traj = []
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terrain, 10)
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 10)

    def cb_terrain(self, msg):
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            self.pts.append((p[0], p[1], p[2], p[3]))

    def cb_odom(self, msg):
        self.traj.append((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))


def main():
    rclpy.init()
    node = Collector()
    print("collecting terrain_map ...", flush=True)
    start = time.time()
    while time.time() - start < DURATION:
        rclpy.spin_once(node, timeout_sec=1.0)
    n = len(node.pts)
    print(f"collected {n} terrain points, {len(node.traj)} traj points", flush=True)
    rclpy.shutdown()
    if n == 0:
        print("no data", flush=True); sys.exit(1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    arr = np.array(node.pts)
    # 下采样
    if len(arr) > MAX_PTS:
        idx = np.random.choice(len(arr), MAX_PTS, replace=False)
        arr = arr[idx]
    x, y, z, it = arr[:,0], arr[:,1], arr[:,2], arr[:,3]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    ax = axes[0]
    sc = ax.scatter(x, y, c=it, s=2, cmap="turbo", vmin=0, vmax=1.0)
    plt.colorbar(sc, ax=ax, label="relative height (intensity)")
    if node.traj:
        tr = np.array(node.traj)
        ax.plot(tr[:,0], tr[:,1], "k-", lw=1.0, alpha=0.6, label="trajectory")
        ax.scatter(tr[0,0], tr[0,1], c="green", s=90, marker="o", label="start")
        ax.scatter(tr[-1,0], tr[-1,1], c="red", s=90, marker="o", label="end")
    ax.set_title(f"terrain_map top view (N={len(arr)} sampled)")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="best", fontsize=8)

    ax2 = axes[1]
    sc2 = ax2.scatter(x, z, c=it, s=2, cmap="turbo", vmin=0, vmax=1.0)
    plt.colorbar(sc2, ax=ax2, label="relative height")
    ax2.set_title("side view X-Z")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Z (m)")
    ax2.set_aspect("equal")

    fig.tight_layout()
    fig.savefig(OUT, dpi=110)
    print(f"saved {OUT}", flush=True)


if __name__ == "__main__":
    main()
