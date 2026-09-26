#!/usr/bin/env python3
"""离线渲染: 全局地图 → 俯视高度图 + 轨迹图
输入: global_map.npz (points: Nx4 x,y,z,intensity) + trajectory.npy + home_anchor.npy
输出: height_map.png (相对高度俯视), trajectory.png (轨迹+起终点)
用法: python3 render_height_map.py <数据目录>
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

OUTDIR = sys.argv[1] if len(sys.argv) > 1 else "/home/lch/dog/tmp/wp2"

# 中文字体
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
           '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
           '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
    try:
        fm.fontManager.addfont(_f)
    except Exception:
        pass
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

data = np.load(f"{OUTDIR}/global_map.npz")
pts = data["points"]
traj = np.load(f"{OUTDIR}/trajectory.npy") if os.path.exists(f"{OUTDIR}/trajectory.npy") else None
home = np.load(f"{OUTDIR}/home_anchor.npy") if os.path.exists(f"{OUTDIR}/home_anchor.npy") else None

x, y, z, it = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]

# 去掉发散离群点(初始化瞬态/静止漂移)
clip = 30.0
m = (np.abs(x) <= clip) & (np.abs(y) <= clip) & (np.abs(z) <= clip)
x, y, z, it = x[m], y[m], z[m], it[m]

# 下采样
if len(x) > 150000:
    idx = np.random.choice(len(x), 150000, replace=False)
    x, y, z, it = x[idx], y[idx], z[idx], it[idx]

# 1) 俯视高度图
fig, ax = plt.subplots(figsize=(11, 8))
sc = ax.scatter(x, y, c=it, s=2, cmap="turbo", vmin=0, vmax=1.0, alpha=0.7)
cb = plt.colorbar(sc, ax=ax, label="相对地面高度 intensity (0=地面, >=1.45=未知)")
if traj is not None and len(traj):
    ax.plot(traj[:, 0], traj[:, 1], "k-", lw=1.2, alpha=0.85, label="轨迹")
if home is not None:
    ax.scatter(home[0], home[1], c="lime", s=140, marker="*", edgecolors="k", label="起点", zorder=6)
ax.set_aspect("equal")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_title(f"地形图俯视(相对高度)  N={len(x)}")
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(f"{OUTDIR}/height_map.png", dpi=110)
print(f"已保存 {OUTDIR}/height_map.png", flush=True)

# 2) 轨迹图
if traj is not None and len(traj):
    tr = np.array(traj)
    fig2, ax2 = plt.subplots(figsize=(8, 7))
    ax2.plot(tr[:, 0], tr[:, 1], "b-", lw=1.5, label="轨迹")
    ax2.scatter(tr[0, 0], tr[0, 1], c="green", s=120, marker="o", label="起点", zorder=5)
    ax2.scatter(tr[-1, 0], tr[-1, 1], c="red", s=120, marker="x", label="终点", zorder=5)
    d = np.linalg.norm(tr[-1] - tr[0])
    ax2.set_aspect("equal")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
    ax2.set_title(f"Point-LIO 位姿轨迹  终点距起点 {d:.3f} m")
    ax2.legend(loc="best")
    fig2.tight_layout()
    fig2.savefig(f"{OUTDIR}/trajectory.png", dpi=110)
    print(f"已保存 {OUTDIR}/trajectory.png (闭环残差 {d:.3f} m)", flush=True)
