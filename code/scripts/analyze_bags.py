#!/usr/bin/env python3
"""批量分析 rosbag: 指令速度 vs 实际速度 vs 轨迹 vs 目标
用法: python3 analyze_bags.py <bag目录1> [bag目录2 ...]
"""
import sys, json
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Odometry
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

def read_bag(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"),
           ConverterOptions(input_serialization_format="cdr",
                            output_serialization_format="cdr"))
    cmd_lin = []      # pathFollower 指令 linear.x
    cmd_ang = []      # pathFollower 指令 angular.z
    sport_x = []      # 安全门下发 Move x (参数)
    vel_actual = []   # 狗实际速度 |velocity|
    traj = []         # 狗轨迹 (x,y)
    goals = []        # WP5 目标 (x,y)
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/cmd_vel_raw":
            m = deserialize_message(data, TwistStamped)
            cmd_lin.append(m.twist.linear.x)
            cmd_ang.append(m.twist.angular.z)
        elif topic == "/api/sport/request":
            m = deserialize_message(data, Request)
            if m.header.identity.api_id == 1008 and m.parameter:
                try:
                    p = json.loads(m.parameter)
                    sport_x.append(p.get("x", 0.0))
                except Exception:
                    pass
        elif topic == "/lf/sportmodestate":
            m = deserialize_message(data, SportModeState)
            v = m.velocity
            vel_actual.append(float(np.hypot(v[0], v[1])))
        elif topic == "/state_estimation":
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            traj.append((p.x, p.y))
        elif topic == "/goal_point":
            m = deserialize_message(data, PointStamped)
            goals.append((round(m.point.x, 2), round(m.point.y, 2)))
    return cmd_lin, cmd_ang, sport_x, vel_actual, traj, goals

def stats(a):
    a = np.array(a, dtype=float)
    if len(a) == 0:
        return 0.0, 0.0, 0.0
    return float(a.max()), float(a.mean()), float(a.std())

print(f"{'bag':<28} {'时长s':>5} {'指令vx':>8} {'安全门vx':>8} {'实际v':>7} {'路径m':>7} {'闭环m':>7} {'目标':>4}")
print("-" * 100)
for bag in sys.argv[1:]:
    name = bag.split("/")[-1]
    cmd_lin, cmd_ang, sport_x, vel_actual, traj, goals = read_bag(bag)
    # 路径长度(累加位移)
    if traj:
        tr = np.array(traj)
        d = np.diff(tr, axis=0)
        path = float(np.linalg.norm(d, axis=1).sum())
        loop = float(np.linalg.norm(tr[-1] - tr[0]))
    else:
        path = loop = 0.0
    # 去重目标数
    distinct = 0; prev = None
    for g in goals:
        if g != prev:
            distinct += 1; prev = g
    cm = stats(cmd_lin); sm = stats(sport_x); vm = stats(vel_actual)
    print(f"{name:<28} {path/14.6:>5.0f} {cm[0]:>7.2f}/{cm[1]:.2f} "
          f"{sm[0]:>7.2f}/{sm[1]:.2f} {vm[0]:>6.2f}/{vm[1]:.2f} "
          f"{path:>7.1f} {loop:>7.2f} {distinct:>4}")
