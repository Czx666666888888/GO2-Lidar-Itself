#!/usr/bin/env python3
"""WP2: 全局地图 → 三态栅格(free/occupied/unknown) + 可视化
离线处理已保存的 global_map.npz / trajectory.npy / home_anchor.npy
"""
import numpy as np
import os
import sys

OUTDIR = sys.argv[1] if len(sys.argv) > 1 else "/home/lch/dog/tmp/wp2"
RES = 0.1              # 栅格分辨率
GROUND_THRE = 0.1      # groundHeightThre
OBSTACLE_THRE = 0.2    # obstacleHeightThre
UNKNOWN_INT = 1.45     # 未知注入阈值

# 加载
data = np.load(f"{OUTDIR}/global_map.npz")
pts = data["points"]          # (N,4) x,y,z,intensity
traj = np.load(f"{OUTDIR}/trajectory.npy") if os.path.exists(f"{OUTDIR}/trajectory.npy") else None
home = np.load(f"{OUTDIR}/home_anchor.npy") if os.path.exists(f"{OUTDIR}/home_anchor.npy") else None

x, y, z, it = pts[:,0], pts[:,1], pts[:,2], pts[:,3]

# 栅格范围
xmin, xmax = np.floor(x.min()/RES)*RES, np.ceil(x.max()/RES)*RES
ymin, ymax = np.floor(y.min()/RES)*RES, np.ceil(y.max()/RES)*RES
nx = int(round((xmax-xmin)/RES)) + 1
ny = int(round((ymax-ymin)/RES)) + 1
print(f"栅格尺寸: {nx} x {ny} cells (范围 {xmax-xmin:.1f}m x {ymax-ymin:.1f}m)", flush=True)

# 三态: 0=unknown, 1=free, 2=occupied
grid = np.zeros((ny, nx), dtype=np.uint8)

ix = ((x - xmin)/RES).astype(int)
iy = ((y - ymin)/RES).astype(int)
# 裁剪
valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
ix, iy, it = ix[valid], iy[valid], it[valid]

# 分类
is_unknown = it >= UNKNOWN_INT
is_occupied = (it >= OBSTACLE_THRE) & (it < UNKNOWN_INT)
is_free = it < OBSTACLE_THRE  # 包括地面和低矮(可通行)

# 填充(占优: occupied > free > unknown)
# 先标 unknown(有数据但未知)
# 再标 free(覆盖 unknown)
# 最后标 occupied(覆盖 free)
for mask, val in [(is_unknown, 0), (is_free, 1), (is_occupied, 2)]:
    grid[iy[mask], ix[mask]] = val

# 注意: 纯 unknown(1.5注入) 和"无数据"都应是 unknown(0), 上面顺序已经保证
# 重新统计: 无数据格保持 0=unknown; 有障碍格=2; 有free无obstacle格=1
# 但上述循环会把 unknown 点覆盖成 free, 需要修正:
# 正确做法: 每个格取"最危险"状态
from collections import defaultdict
cell_state = defaultdict(int)  # 0 unknown, 1 free, 2 occupied
for i in range(len(ix)):
    k = (iy[i], ix[i])
    if it[i] >= UNKNOWN_INT:
        s = 0   # unknown(注入点, 保持未知)
    elif it[i] >= OBSTACLE_THRE:
        s = 2   # occupied
    else:
        s = 1   # free
    cell_state[k] = max(cell_state[k], s)  # occupied(2) 优先于 free(1) 优先于 unknown(0)
grid[:] = 0
for (gy, gx), s in cell_state.items():
    grid[gy, gx] = s

n_free = (grid == 1).sum()
n_occ = (grid == 2).sum()
n_unk = (grid == 0).sum()
print(f"三态栅格: free={n_free}, occupied={n_occ}, unknown={n_unk}", flush=True)
print(f"occupancy 比例: free {n_free/(nx*ny)*100:.1f}%, occ {n_occ/(nx*ny)*100:.1f}%, unknown {n_unk/(nx*ny)*100:.1f}%", flush=True)

np.save(f"{OUTDIR}/grid3state.npy", grid)
np.savez(f"{OUTDIR}/grid_meta.npz", xmin=xmin, ymin=ymin, res=RES, nx=nx, ny=ny)

# 可视化
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.family'] = 'Noto Sans CJK SC'
plt.rcParams['axes.unicode_minus'] = False
fig, ax = plt.subplots(figsize=(10, 8))
cmap = plt.matplotlib.colors.ListedColormap(["#888888", "#ffffff", "#d62728"])  # unknown灰, free白, occupied红
ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2,
          extent=[xmin, xmax, ymin, ymax], aspect="equal")
if traj is not None and len(traj):
    ax.plot(traj[:,0], traj[:,1], "b-", lw=1.2, alpha=0.8, label="trajectory")
if home is not None:
    ax.scatter(home[0], home[1], c="green", s=120, marker="*", label="home", zorder=5)
ax.set_title(f"3-state grid: free(white) occupied(red) unknown(gray)\n"
             f"free={n_free} occ={n_occ} unknown={n_unk}")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(f"{OUTDIR}/grid3state.png", dpi=110)
print(f"已保存 {OUTDIR}/grid3state.png", flush=True)
