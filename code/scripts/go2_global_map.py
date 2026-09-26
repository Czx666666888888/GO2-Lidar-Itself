#!/usr/bin/env python3
"""WP2: 全局地图构建 + home_anchor + 轨迹记录
- 订阅 /terrain_map(局部2.5D, map系) 累积成持久全局地图(体素下采样)
- 订阅 /state_estimation 记录轨迹 + 锁存 home_anchor(起点)
- 发布 /global_map, /home_anchor, /trajectory
- 结束时保存 global_map.npz + home + traj
"""
import os, time, math
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

OUTDIR = "/home/lch/dog/tmp/wp2"

class GlobalMapBuilder(Node):
    def __init__(self):
        super().__init__("global_map_builder")
        self.voxel_size = 0.1
        self.grid = {}            # (ix,iy,iz) -> (x,y,z,intensity)
        self.home = None          # 起点位姿 (x,y,z,yaw)
        self.traj = []            # 轨迹点 (x,y,z)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terrain, 10)
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 10)
        self.pub_global = self.create_publisher(PointCloud2, "/global_map", 10)
        self.pub_home = self.create_publisher(PointStamped, "/home_anchor", 10)
        self.pub_traj = self.create_publisher(Path, "/trajectory", 10)
        self.timer = self.create_timer(2.0, self.publish_all)

    def cb_terrain(self, msg):
        for p in point_cloud2.read_points(msg, field_names=("x","y","z","intensity"), skip_nans=True):
            ix = int(round(p[0]/self.voxel_size)); iy = int(round(p[1]/self.voxel_size)); iz = int(round(p[2]/self.voxel_size))
            self.grid[(ix,iy,iz)] = (p[0], p[1], p[2], p[3])

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        self.traj.append((p.x, p.y, p.z))
        if self.home is None:
            q = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.home = (p.x, p.y, p.z, yaw)
            self.get_logger().info(f"home_anchor 锁存: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

    def publish_all(self):
        if not self.grid:
            return
        # 发布全局地图
        pts = list(self.grid.values())
        fields = [point_cloud2.PointField(name=n, offset=i*4, datatype=7, count=1)
                  for i, n in enumerate(("x","y","z","intensity"))]
        cloud = point_cloud2.create_cloud(Header(frame_id="map"), fields, pts)
        self.pub_global.publish(cloud)
        # 发布 home_anchor
        if self.home:
            h = PointStamped(); h.header.frame_id = "map"
            h.point.x, h.point.y, h.point.z = self.home[0], self.home[1], self.home[2]
            self.pub_home.publish(h)

    def report(self):
        pts = np.array(list(self.grid.values()))
        n = len(pts)
        print(f"=== WP2 全局地图结果 ===", flush=True)
        print(f"体素点数 = {n} (voxel {self.voxel_size}m)", flush=True)
        if n:
            print(f"覆盖范围 X[{pts[:,0].min():.2f},{pts[:,0].max():.2f}] Y[{pts[:,1].min():.2f},{pts[:,1].max():.2f}]", flush=True)
            it = pts[:,3]
            known = (it < 1.45).sum(); unknown = (it >= 1.45).sum()
            print(f"已知点 = {known}, 未知点 = {unknown}", flush=True)
        if self.home:
            print(f"home_anchor = ({self.home[0]:.2f}, {self.home[1]:.2f}, {self.home[2]:.2f})", flush=True)
        if self.traj:
            tr = np.array(self.traj)
            print(f"轨迹点数 = {len(tr)}, 终点距起点 = {np.linalg.norm(tr[-1]-tr[0]):.3f} m", flush=True)
        # 保存
        os.makedirs(OUTDIR, exist_ok=True)
        np.savez(f"{OUTDIR}/global_map.npz", points=pts)
        if self.traj:
            np.save(f"{OUTDIR}/trajectory.npy", np.array(self.traj))
        if self.home:
            np.save(f"{OUTDIR}/home_anchor.npy", np.array(self.home))
        print(f"已保存到 {OUTDIR}", flush=True)

def main():
    global OUTDIR
    rclpy.init()
    DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60
    OUTDIR = sys.argv[2] if len(sys.argv) > 2 else "/home/lch/dog/tmp/wp2"
    node = GlobalMapBuilder()
    print(f"WP2 全局地图构建运行中... 时长 {DUR}s, 输出 {OUTDIR}", flush=True)
    start = time.time()
    try:
        while time.time() - start < DUR:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    node.report()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
