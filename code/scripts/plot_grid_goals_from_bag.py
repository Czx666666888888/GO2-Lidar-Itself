#!/usr/bin/env python3
"""从 rosbag 直接读 /terrain_map + /goal_point + /state_estimation,
建三态栅格并叠加 WP5 选点(100% 同源同帧, 无需重跑链)
用法: python3 plot_grid_goals_from_bag.py <bag目录> <输出png>
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

BAG = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/lch/dog/tmp/grid_goals.png"

RES = 0.1
OBSTACLE_THRE = 0.2
UNKNOWN_INT = 1.45

r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id="sqlite3"),
       ConverterOptions(input_serialization_format="cdr",
                        output_serialization_format="cdr"))

grid = {}
goals = []
traj = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic == "/terrain_map":
        m = deserialize_message(data, PointCloud2)
        for p in point_cloud2.read_points(m, field_names=["x", "y", "intensity"], skip_nans=True):
            x, y, it = p[0], p[1], p[2]
            ix = int(round(x / RES)); iy = int(round(y / RES))
            s = 0 if it >= UNKNOWN_INT else (2 if it >= OBSTACLE_THRE else 1)
            grid[(ix, iy)] = max(grid.get((ix, iy), 0), s)
    elif topic == "/goal_point":
        m = deserialize_message(data, PointStamped)
        goals.append((m.point.x, m.point.y, t))
    elif topic == "/state_estimation":
        m = deserialize_message(data, Odometry)
        traj.append((m.pose.pose.position.x, m.pose.pose.position.y))

print(f"terrain点={len(grid)} goal={len(goals)} traj={len(traj)}", flush=True)

# 栅格边界
if grid:
    xs = [k[0] for k in grid]; ys = [k[1] for k in grid]
    ixmin, ixmax = min(xs), max(xs); iymin, iymax = min(ys), max(ys)
else:
    ixmin = ixmax = iymin = iymax = 0
nx = ixmax - ixmin + 1; ny = iymax - iymin + 1
gmap = np.zeros((ny, nx), dtype=np.uint8)
for (ix, iy), s in grid.items():
    gmap[iy - iymin, ix - ixmin] = s
xmin = ixmin * RES; ymin = iymin * RES
xmax = (ixmax + 1) * RES; ymax = (iymax + 1) * RES

# 字体
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
           '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
           '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(11, 9))
cmap = ListedColormap(["#888888", "#ffffff", "#d62728"])
ax.imshow(gmap, origin="lower", cmap=cmap, vmin=0, vmax=2,
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
                    linewidths=2, label="WP5选点", zorder=5)
    # 去重目标编号
    prev = None; idx = 0
    for x, y, _ in goals:
        k = (round(x, 2), round(y, 2))
        if k != prev:
            ax.annotate(str(idx), (x, y), fontsize=8, color="darkred", zorder=7)
            idx += 1; prev = k
    plt.colorbar(sc, ax=ax, label="时间(深=早 亮=晚)")

ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_title(f"{BAG.split('/')[-1]}\n栅格地图 + WP5选点 (灰=unknown 白=free 红=障碍)")
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(OUT, dpi=120)
print(f"已保存 {OUT}", flush=True)
