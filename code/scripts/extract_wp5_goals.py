#!/usr/bin/env python3
"""从 rosbag 提取 WP5 规划点(/goal_point) 与狗实际轨迹(/state_estimation), 画轨迹图
用法: python3 extract_wp5_goals.py <bag目录> <输出png>
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

BAG = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/lch/dog/tmp/wp5_goals.png"

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id="sqlite3"),
            ConverterOptions(input_serialization_format="cdr",
                             output_serialization_format="cdr"))

goals = []   # (x, y, t_ns)
traj = []    # (x, y)
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == "/goal_point":
        m = deserialize_message(data, PointStamped)
        goals.append((m.point.x, m.point.y, t))
    elif topic == "/state_estimation":
        m = deserialize_message(data, Odometry)
        traj.append((m.pose.pose.position.x, m.pose.pose.position.y))

print(f"goal_point 数 = {len(goals)}, 轨迹点数 = {len(traj)}", flush=True)

for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
           '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
           '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(10, 8))
if traj:
    tr = np.array(traj)
    ax.plot(tr[:, 0], tr[:, 1], 'b-', lw=1.2, alpha=0.8, label="狗实际轨迹")
    ax.scatter(tr[0, 0], tr[0, 1], c='green', s=120, marker='o', label='起点', zorder=5)
if goals:
    g = np.array([(x, y) for x, y, _ in goals])
    # 按时间上色
    ts = np.array([t for _, _, t in goals], dtype=float)
    ts = (ts - ts.min()) / max(ts.max() - ts.min(), 1e-9)
    sc = ax.scatter(g[:, 0], g[:, 1], c=ts, cmap='plasma', s=60,
                    marker='x', label='WP5规划点', zorder=4)
    # 编号
    for i, (x, y, _) in enumerate(goals):
        if i % max(1, len(goals)//15) == 0:
            ax.annotate(str(i), (x, y), fontsize=7, color='darkred', zorder=6)
    plt.colorbar(sc, ax=ax, label='时间(归一化, 深=早 亮=晚)')

ax.set_aspect('equal')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title(f'WP5 规划点轨迹 (共 {len(goals)} 个目标)')
ax.legend(loc='best')
fig.tight_layout()
fig.savefig(OUT, dpi=110)
print(f"已保存 {OUT}", flush=True)
