#!/usr/bin/env python3
"""从 rosbag 直接分析导航链数据流: goal_point → way_point → path → cmd_vel
(不重跑链, 直接读 bag 里已录的导航输出)
用法: python3 analyze_nav_flow.py <bag目录>
"""
import sys, json
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Odometry, Path

bag = sys.argv[1]
r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id="sqlite3"),
       ConverterOptions(input_serialization_format="cdr",
                        output_serialization_format="cdr"))

goals = []      # (t, x, y)
ways = []       # (t, x, y)
path_n = []     # 每帧 path 点数
cmd = []        # (t, vx, wz)
traj = []       # (x, y)
t0 = None
while r.has_next():
    topic, data, t = r.read_next()
    if t0 is None: t0 = t
    if topic == "/goal_point":
        m = deserialize_message(data, PointStamped)
        goals.append((t, m.point.x, m.point.y))
    elif topic == "/way_point":
        m = deserialize_message(data, PointStamped)
        ways.append((t, m.point.x, m.point.y))
    elif topic == "/path":
        m = deserialize_message(data, Path)
        path_n.append((t, len(m.poses)))
    elif topic == "/cmd_vel_raw":
        m = deserialize_message(data, TwistStamped)
        cmd.append((t, m.twist.linear.x, m.twist.angular.z))
    elif topic == "/state_estimation":
        m = deserialize_message(data, Odometry)
        traj.append((m.pose.pose.position.x, m.pose.pose.position.y))

name = bag.split("/")[-1]
print(f"===== {name} =====")
print(f"时长: {(max(t for _,_,_ in goals) if goals else 0) and ( (goals[-1][0]-t0)/1e9 ):.0f}s")

# WP5 目标(去重)
dg = 0; prev = None
for _, x, y in goals:
    k = (round(x,2), round(y,2))
    if k != prev: dg += 1; prev = k
print(f"WP5目标: {len(goals)}条消息 / 去重{dg}个")

# way_point
print(f"way_point: {len(ways)}条")
if ways and goals:
    # waypoint 与目标的空间距离(评估 far_planner 是否朝目标规划)
    # 用最后一个 goal 做参考
    gx, gy = goals[-1][1], goals[-1][2]
    wd = [np.hypot(w[1]-gx, w[2]-gy) for w in ways]
    print(f"  waypoint 距最后目标: 均值{np.mean(wd):.2f}m 最小{np.min(wd):.2f}m")

# path
if path_n:
    pn = np.array([p[1] for p in path_n])
    print(f"path: {len(pn)}帧, 点数 均值{pn.mean():.0f} 最大{pn.max():.0f}, 空路径(<2点){(pn<2).sum()}帧")

# cmd_vel
if cmd:
    c = np.array([(c[1], c[2]) for c in cmd])
    vx, wz = c[:,0], c[:,1]
    nz = (np.abs(vx) > 0.01)
    wz_nz = (np.abs(wz) > 0.01)
    print(f"cmd_vel: {len(c)}帧, 非零线速{nz.sum()}({100*nz.sum()/len(c):.0f}%), "
          f"非零角速{wz_nz.sum()}({100*wz_nz.sum()/len(c):.0f}%)")
    print(f"  |vx|: max{vx.max():.2f} 均值{np.abs(vx).mean():.2f} | "
          f"|wz|: max{wz.max():.2f} 均值{np.abs(wz).mean():.2f}")

# 轨迹
if traj:
    tr = np.array(traj)
    d = np.diff(tr, axis=0)
    print(f"轨迹: {len(tr)}点, 路径{np.linalg.norm(d,axis=1).sum():.1f}m, "
          f"闭环{np.linalg.norm(tr[-1]-tr[0]):.2f}m")
print()
