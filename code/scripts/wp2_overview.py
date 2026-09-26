#!/usr/bin/env python3
"""成果总览: 2x2 面板 = 全局地图高度图 / 三态栅格 / 轨迹 / 轨迹+home 叠加"""
import numpy as np, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.family'] = 'Noto Sans CJK SC'
plt.rcParams['axes.unicode_minus'] = False

OUTDIR = "/home/lch/dog/tmp/wp2"
data = np.load(f"{OUTDIR}/global_map.npz")
pts = data["points"]
traj = np.load(f"{OUTDIR}/trajectory.npy")
home = np.load(f"{OUTDIR}/home_anchor.npy") if os.path.exists(f"{OUTDIR}/home_anchor.npy") else None
grid = np.load(f"{OUTDIR}/grid3state.npy")
meta = np.load(f"{OUTDIR}/grid_meta.npz")
xmin, ymin, res, nx, ny = meta["xmin"], meta["ymin"], meta["res"], meta["nx"], meta["ny"]

x, y, it = pts[:,0], pts[:,1], pts[:,3]
# 下采样渲染
if len(pts) > 20000:
    idx = np.random.choice(len(pts), 20000, replace=False)
    x, y, it = x[idx], y[idx], it[idx]

fig, axes = plt.subplots(2, 2, figsize=(16, 14))

# 1. 全局地图高度图
ax = axes[0][0]
sc = ax.scatter(x, y, c=it, s=2, cmap="turbo", vmin=0, vmax=1.0)
plt.colorbar(sc, ax=ax, label="relative height (m)")
ax.set_title(f"1. 全局地图 (accumulated terrain, N={len(pts)} sampled)")
ax.set_aspect("equal"); ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)")

# 2. 三态栅格
ax = axes[0][1]
cmap = matplotlib.colors.ListedColormap(["#888888", "#ffffff", "#d62728"])
ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2,
          extent=[xmin, xmin+nx*res, ymin, ymin+ny*res], aspect="equal")
ax.set_title(f"2. 三态栅格 (灰=未知 白=可通行 红=障碍)")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)")

# 3. 轨迹(正方形闭环)
ax = axes[1][0]
if len(traj):
    ax.plot(traj[:,0], traj[:,1], "b-", lw=1.5)
    ax.scatter(traj[0,0], traj[0,1], c="green", s=150, marker="o", label="start", zorder=5)
    ax.scatter(traj[-1,0], traj[-1,1], c="red", s=150, marker="o", label="end", zorder=5)
    d = np.linalg.norm(traj[-1]-traj[0])
    ax.set_title(f"3. 轨迹(正方形) 闭环误差={d:.3f}m")
ax.set_aspect("equal"); ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.legend()

# 4. 轨迹 + home 叠加在全局地图上
ax = axes[1][1]
ax.scatter(x, y, c=it, s=2, cmap="turbo", vmin=0, vmax=1.0, alpha=0.5)
if len(traj):
    ax.plot(traj[:,0], traj[:,1], "k-", lw=1.5, alpha=0.9)
if home is not None:
    ax.scatter(home[0], home[1], c="lime", s=200, marker="*", edgecolors="black", label="home", zorder=6)
ax.set_title("4. 全局地图 + 轨迹 + home")
ax.set_aspect("equal"); ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.legend()

fig.suptitle("Go2 离线建图成果总览 (WP0-WP2, 正方形包)", fontsize=16, fontweight="bold")
fig.tight_layout()
fig.savefig("/home/lch/dog/tmp/wp2/成果总览.png", dpi=110)
print("已保存 /home/lch/dog/tmp/wp2/成果总览.png")
