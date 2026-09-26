#!/usr/bin/env python3
"""
离线回放期间累积三套点云地图:
  1) transform_sensors  -> /utlidar/transformed_cloud  (body 帧, 用 /utlidar/robot_odom 变换到 odom 帧)
  2) point_lio_unilidar -> /registered_scan            (世界帧, 直接累积)
  3) terrain_analysis   -> /terrain_map                (map 帧, 直接累积)

同时记录 /state_estimation 与 /utlidar/robot_odom 轨迹。

用法:
  python3 collect_pointcloud_maps.py <输出目录> <采集秒数>
退出时把累积结果写成 .npy (Nx4: x,y,z,intensity)。
"""
import sys, time, os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from transforms3d.quaternions import quat2mat

CAP = 700_000          # 每张地图累积点数上限(随机下采样控制内存)


def downsample_if_needed(lst, cap=CAP):
    if len(lst) > cap:
        idx = np.random.choice(len(lst), cap, replace=False)
        lst = [lst[i] for i in idx]
    return lst


class Collector(Node):
    def __init__(self):
        super().__init__("pc_map_collector")
        self.raw = []        # transform_sensors 累积 (odom 帧)
        self.pio = []        # point-lio registered_scan (世界帧)
        self.terrain = []    # terrain_map (map 帧)
        self.traj_pio = []   # /state_estimation
        self.traj_odom = []  # /utlidar/robot_odom
        self.latest_odom = None  # (pos(3,), R(3,3))

        self.create_subscription(PointCloud2, "/utlidar/transformed_cloud", self.cb_raw, 20)
        self.create_subscription(PointCloud2, "/registered_scan", self.cb_pio, 20)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terrain, 20)
        self.create_subscription(Odometry, "/state_estimation", self.cb_pio_odom, 20)
        self.create_subscription(Odometry, "/utlidar/robot_odom", self.cb_robot_odom, 20)

    # ---- odometry ----
    def cb_robot_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        R = quat2mat([q.w, q.x, q.y, q.z])
        self.latest_odom = (np.array([p.x, p.y, p.z]), R)
        self.traj_odom.append((p.x, p.y, p.z))

    def cb_pio_odom(self, m):
        p = m.pose.pose.position
        self.traj_pio.append((p.x, p.y, p.z))

    # ---- point clouds ----
    def _read(self, m):
        """返回 (N,4) float64 数组 (x,y,z,intensity), intensity 缺失时补 0"""
        try:
            pts = point_cloud2.read_points(m, field_names=("x", "y", "z", "intensity"), skip_nans=True)
            arr = np.array([(p[0], p[1], p[2], p[3]) for p in pts], dtype=np.float64)
        except Exception:
            pts = point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
            a3 = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float64)
            arr = np.hstack([a3, np.zeros((len(a3), 1))])
        return arr

    def cb_raw(self, m):
        if self.latest_odom is None:
            return
        pos, R = self.latest_odom
        arr = self._read(m)
        if len(arr) == 0:
            return
        xyz = arr[:, :3] @ R.T + pos
        out = np.hstack([xyz, arr[:, 3:4]])
        self.raw.extend(out.tolist())
        self.raw = downsample_if_needed(self.raw)

    def cb_pio(self, m):
        arr = self._read(m)
        if len(arr):
            self.pio.extend(arr.tolist())
            self.pio = downsample_if_needed(self.pio)

    def cb_terrain(self, m):
        arr = self._read(m)
        if len(arr):
            self.terrain.extend(arr.tolist())
            self.terrain = downsample_if_needed(self.terrain)


def main():
    rclpy.init()
    outdir = sys.argv[1]
    duration = float(sys.argv[2])
    os.makedirs(outdir, exist_ok=True)
    node = Collector()
    print(f"[collect] start {duration:.0f}s -> {outdir}", flush=True)
    t0 = time.time()
    while time.time() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    def save(name, lst):
        arr = np.asarray(lst, dtype=np.float64).reshape(-1, 4)
        np.save(os.path.join(outdir, name), arr)
        return len(arr)

    n_raw = save("raw_cloud.npy", node.raw)
    n_pio = save("registered_scan.npy", node.pio)
    n_ter = save("terrain_map.npy", node.terrain)
    np.save(os.path.join(outdir, "traj_pio.npy"), np.asarray(node.traj_pio).reshape(-1, 3))
    np.save(os.path.join(outdir, "traj_odom.npy"), np.asarray(node.traj_odom).reshape(-1, 3))

    print(f"[collect] DONE raw={n_raw} pio={n_pio} terrain={n_ter}", flush=True)
    print(f"[collect] traj_pio={len(node.traj_pio)} traj_odom={len(node.traj_odom)}", flush=True)


if __name__ == "__main__":
    main()
