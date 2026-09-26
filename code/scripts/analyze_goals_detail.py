#!/usr/bin/env python3
"""定量分析多个 bag 的 WP5 目标时空分布, 判断微振荡是否消失
用法: python3 analyze_goals_detail.py <bag1> [bag2 ...]
输出每个 bag: 目标数/唯一数/相邻间距分布/狗在目标间移动距离
"""
import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

def analyze(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"),
           ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    goals = []   # (t_ns, x, y)
    traj = []    # (t_ns, x, y)
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/goal_point":
            m = deserialize_message(data, PointStamped)
            goals.append((t, m.point.x, m.point.y))
        elif topic == "/state_estimation":
            m = deserialize_message(data, Odometry)
            traj.append((t, m.pose.pose.position.x, m.pose.pose.position.y))

    name = bag.split("/")[-1]
    print(f"\n===== {name} =====")
    print(f"目标消息数 = {len(goals)}, 轨迹点数 = {len(traj)}")

    if not goals:
        return
    # 相邻目标间距(按时间)
    g = np.array([(x, y) for _, x, y in goals])
    d = np.array([g[i+1]-g[i] for i in range(len(g)-1)])
    dn = np.linalg.norm(d, axis=1) if len(d) else np.array([])
    dt = np.diff(np.array([t for t,_,_ in goals])) / 1e9

    # 全局唯一目标(0.05m 聚类)
    uniq = []
    for x, y in g:
        if not uniq or min(np.hypot(x-u[0], y-u[1]) for u in uniq) > 0.05:
            uniq.append((x, y))
    print(f"全局唯一目标(0.05m) = {len(uniq)}")

    # 相邻间距分布
    if len(dn):
        print(f"相邻目标间距: min={dn.min():.3f}m 均值={dn.mean():.2f}m max={dn.max():.2f}m")
        print(f"  微振荡(<0.15m): {(dn<0.15).sum()}次 / {len(dn)}次 ({(dn<0.15).mean()*100:.0f}%)")
        print(f"  正常推进(>0.5m): {(dn>0.5).sum()}次 ({(dn>0.5).mean()*100:.0f}%)")
    if len(dt):
        print(f"相邻目标时间间隔: min={dt.min():.1f}s 均值={dt.mean():.1f}s max={dt.max():.1f}s")

    # 每个目标发布时, 狗离该目标的直线距离
    if traj:
        tr = np.array([(x, y) for _, x, y in traj])
        tt = np.array([t for t,_,_ in traj])
        dists = []
        for tg, gx, gy in goals:
            j = np.searchsorted(tt, tg)
            j = min(j, len(tt)-1)
            dists.append(np.hypot(tr[j][0]-gx, tr[j][1]-gy))
        dists = np.array(dists)
        print(f"目标发布时狗距目标: 均值{dists.mean():.2f}m 最小{dists.min():.2f}m 最大{dists.max():.2f}m")

    # 狗实际移动总距离
    if len(traj) > 1:
        dtr = np.diff(np.array([(x, y) for _, x, y in traj]), axis=0)
        print(f"狗轨迹: 累计{np.linalg.norm(dtr,axis=1).sum():.1f}m, 净位移{np.linalg.norm(g[-1]-g[0] if False else np.array([(x,y) for _,x,y in traj])[-1]-np.array([(x,y) for _,x,y in traj])[0]):.2f}m")

for b in sys.argv[1:]:
    analyze(b)
