#!/usr/bin/env python3
"""离线导航链采集: 订阅 /goal_point /way_point /path /cmd_vel_raw /state_estimation
保存到 npz, 并打印摘要
用法: python3 collect_nav_chain.py <时长秒> <输出目录>
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/lch/dog/tmp/nav_chain_out"

class Collect(Node):
    def __init__(self):
        super().__init__("nav_chain_collect")
        self.goals = []
        self.waypoints = []
        self.path_lens = []
        self.cmd_vel = []
        self.traj = []
        self.create_subscription(PointStamped, "/goal_point", self.cb_goal, 10)
        self.create_subscription(PointStamped, "/way_point", self.cb_way, 10)
        self.create_subscription(Path, "/path", self.cb_path, 10)
        self.create_subscription(TwistStamped, "/cmd_vel_raw", self.cb_cmd, 10)
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 10)

    def cb_goal(self, m):
        self.goals.append((m.point.x, m.point.y))
    def cb_way(self, m):
        self.waypoints.append((m.point.x, m.point.y))
    def cb_path(self, m):
        self.path_lens.append(len(m.poses))
    def cb_cmd(self, m):
        self.cmd_vel.append((m.twist.linear.x, m.twist.angular.z))
    def cb_odom(self, m):
        p = m.pose.pose.position
        self.traj.append((p.x, p.y))

def main():
    rclpy.init()
    node = Collect()
    print(f"采集导航链输出 {DUR}s ...", flush=True)
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()

    g = np.array(node.goals) if node.goals else np.zeros((0, 2))
    w = np.array(node.waypoints) if node.waypoints else np.zeros((0, 2))
    pl = np.array(node.path_lens) if node.path_lens else np.zeros(0)
    cv = np.array(node.cmd_vel) if node.cmd_vel else np.zeros((0, 2))
    tr = np.array(node.traj) if node.traj else np.zeros((0, 2))

    # 去重目标
    distinct = 0; prev = None
    for gg in node.goals:
        g2 = (round(gg[0], 2), round(gg[1], 2))
        if g2 != prev:
            distinct += 1; prev = g2

    print("=== 导航链输出摘要 ===", flush=True)
    print(f"goal_point: 总{len(g)} 去重{distinct}", flush=True)
    print(f"way_point : {len(w)} 帧", flush=True)
    if len(pl):
        print(f"/path     : 平均点数 {pl.mean():.0f}, 空路径 {(pl<2).sum()} 帧", flush=True)
    if len(cv):
        lx = cv[:, 0]; az = cv[:, 1]
        nz = (np.abs(lx) > 0.01).sum()
        print(f"cmd_vel_raw: 帧{len(cv)}, 非零线速{nz}帧({100*nz/len(cv):.0f}%), "
              f"|vx|max={np.abs(lx).max():.2f} |wz|max={np.abs(az).max():.2f}", flush=True)
    if len(tr):
        d = np.diff(tr, axis=0)
        path = np.linalg.norm(d, axis=1).sum()
        loop = np.linalg.norm(tr[-1] - tr[0])
        print(f"轨迹: {len(tr)}点, 路径{path:.1f}m, 闭环{loop:.2f}m", flush=True)

    np.savez(f"{OUT}/nav_chain.npz", goals=g, waypoints=w,
             path_lens=pl, cmd_vel=cv, traj=tr)
    print(f"已保存 {OUT}/nav_chain.npz", flush=True)

if __name__ == "__main__":
    main()
