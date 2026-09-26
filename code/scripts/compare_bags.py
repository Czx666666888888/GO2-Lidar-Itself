#!/usr/bin/env python3
"""多组 rosbag 对比: 轨迹叠加图 + 速度对比图
用法: python3 compare_bags.py <输出前缀> <bag1> [bag2 ...]
"""
import sys, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Odometry
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

OUT = sys.argv[1]
BAGS = sys.argv[2:]

for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
           '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
           '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def read_bag(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"),
           ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    traj, goals, cmd, vel = [], [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/state_estimation":
            m = deserialize_message(data, Odometry)
            traj.append((m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == "/goal_point":
            m = deserialize_message(data, PointStamped)
            goals.append((m.point.x, m.point.y))
        elif topic == "/cmd_vel_raw":
            m = deserialize_message(data, TwistStamped)
            cmd.append(m.twist.linear.x)
        elif topic == "/lf/sportmodestate":
            m = deserialize_message(data, SportModeState)
            v = m.velocity
            vel.append(float(np.hypot(v[0], v[1])))
    return np.array(traj) if traj else None, np.array(goals) if goals else None, \
           np.array(cmd) if cmd else None, np.array(vel) if vel else None

data = {}
for b in BAGS:
    name = b.split("/")[-1].replace("full_chain_test1_20260830_", "")
    data[name] = read_bag(b)

# ---- 图1: 轨迹叠加 ----
fig, ax = plt.subplots(figsize=(10, 8))
colors = plt.cm.tab10(np.linspace(0, 1, len(data)))
for (name, (traj, goals, cmd, vel)), c in zip(data.items(), colors):
    if traj is not None and len(traj):
        ax.plot(traj[:, 0], traj[:, 1], lw=1.5, color=c, alpha=0.8, label=name)
        ax.scatter(traj[0, 0], traj[0, 1], color=c, s=70, marker='o', edgecolors='k', zorder=5)
ax.set_aspect('equal')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title('各组轨迹对比 (圆点=起点)')
ax.legend(loc='best', fontsize=8)
fig.tight_layout(); fig.savefig(f"{OUT}_traj.png", dpi=120)

# ---- 图2: 速度对比(指令 vs 实际) ----
names = list(data.keys())
cmd_max = [data[n][2].max() if data[n][2] is not None and len(data[n][2]) else 0 for n in names]
cmd_mean = [abs(data[n][2]).mean() if data[n][2] is not None and len(data[n][2]) else 0 for n in names]
vel_max = [data[n][3].max() if data[n][3] is not None and len(data[n][3]) else 0 for n in names]
vel_mean = [data[n][3].mean() if data[n][3] is not None and len(data[n][3]) else 0 for n in names]

x = np.arange(len(names)); w = 0.35
fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
ax1.bar(x - w/2, cmd_max, w, label='指令vx max', color='steelblue')
ax1.bar(x + w/2, vel_max, w, label='实际v max', color='coral')
ax1.set_xticks(x); ax1.set_xticklabels(names, rotation=40, ha='right', fontsize=8)
ax1.set_ylabel('速度 (m/s)'); ax1.set_title('最大速度对比'); ax1.legend()

ax2.bar(x - w/2, cmd_mean, w, label='指令vx 均值', color='steelblue')
ax2.bar(x + w/2, vel_mean, w, label='实际v 均值', color='coral')
ax2.set_xticks(x); ax2.set_xticklabels(names, rotation=40, ha='right', fontsize=8)
ax2.set_ylabel('速度 (m/s)'); ax2.set_title('平均速度对比'); ax2.legend()
fig2.tight_layout(); fig2.savefig(f"{OUT}_speed.png", dpi=120)

print(f"已保存 {OUT}_traj.png 和 {OUT}_speed.png", flush=True)
