#!/usr/bin/env python3
"""把 WP5 规划点 + 狗轨迹 叠加到三态栅格地图上
用法: python3 plot_goals_on_grid.py <栅格目录> <bag目录> <输出png>
  栅格目录: 含 grid3state.npy + grid_meta.npz (wp2_grid.py 产出)
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.colors import ListedColormap

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

GRIDDIR = sys.argv[1]
BAG = sys.argv[2]
OUT = sys.argv[3] if len(sys.argv) > 3 else "/home/lch/dog/tmp/wp5_on_grid.png"

# ---- 读栅格 ----
grid = np.load(f"{GRIDDIR}/grid3state.npy")
meta = np.load(f"{GRIDDIR}/grid_meta.npz")
xmin, ymin, res = float(meta["xmin"]), float(meta["ymin"]), float(meta["res"])
ny, nx = grid.shape
xmax = xmin + nx * res
ymax = ymin + ny * res

# ---- 读 bag: goal_point + state_estimation ----
r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id="sqlite3"),
       ConverterOptions(input_serialization_format="cdr",
                        output_serialization_format="cdr"))
goals = []; traj = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic == "/goal_point":
        m = deserialize_message(data, PointStamped)
        goals.append((m.point.x, m.point.y, t))
    elif topic == "/state_estimation":
        m = deserialize_message(data, Odometry)
        traj.append((m.pose.pose.position.x, m.pose.pose.position.y))

print(f"goal={len(goals)} traj={len(traj)} grid={nx}x{ny}", flush=True)

# ---- 字体 ----
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
           '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
           '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ---- 画 ----
fig, ax = plt.subplots(figsize=(11, 9))
cmap = ListedColormap(["#888888", "#ffffff", "#d62728"])  # 灰=unknown 白=free 红=occupied
ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2,
          extent=[xmin, xmax, ymin, ymax], aspect="equal", alpha=0.85)

if traj:
    tr = np.array(traj)
    ax.plot(tr[:, 0], tr[:, 1], "b-", lw=1.3, alpha=0.85, label="狗轨迹")
    ax.scatter(tr[0, 0], tr[0, 1], c="green", s=140, marker="o",
               edgecolors="k", label="起点", zorder=6)
if goals:
    g = np.array([(x, y) for x, y, _ in goals])
    ts = np.array([t for _, _, t in goals], dtype=float)
    ts = (ts - ts.min()) / max(ts.max() - ts.min(), 1e-9)
    sc = ax.scatter(g[:, 0], g[:, 1], c=ts, cmap="plasma", s=90, marker="x",
                    linewidths=2, label="WP5规划点", zorder=5)
    for i, (x, y, _) in enumerate(goals):
        if i % max(1, len(goals)//12) == 0:
            ax.annotate(str(i), (x, y), fontsize=8, color="darkred", zorder=7)
    plt.colorbar(sc, ax=ax, label="时间(深=早 亮=晚)")

ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_title("WP5 规划点 + 三态栅格地图\n(灰=unknown 白=free 红=障碍)")
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(OUT, dpi=120)
print(f"已保存 {OUT}", flush=True)
