#!/usr/bin/env python3
"""WP5: 前沿探索(离线) — 从三态栅格提取前沿、聚类、风险评分、选最优观察点
输入: wp2/grid3state.npy, grid_meta.npz, trajectory.npy, home_anchor.npy
"""
import numpy as np, os
from collections import deque

OUTDIR = "/home/lch/dog/tmp/wp2"
grid = np.load(f"{OUTDIR}/grid3state.npy")
meta = np.load(f"{OUTDIR}/grid_meta.npz")
xmin, ymin, res, nx, ny = meta["xmin"], meta["ymin"], meta["res"], meta["nx"], meta["ny"]
traj = np.load(f"{OUTDIR}/trajectory.npy")
home = np.load(f"{OUTDIR}/home_anchor.npy")

ny_, nx_ = grid.shape
# 车辆当前位置 = 轨迹终点
vx, vy = traj[-1,0], traj[-1,1]
hx, hy = home[0], home[1]
print(f"车辆当前位置=({vx:.2f},{vy:.2f}), home=({hx:.2f},{hy:.2f})", flush=True)

# 1. 提取前沿: free(1) 且相邻有 unknown(0)
frontier = np.zeros_like(grid, dtype=bool)
for i in range(ny_):
    for j in range(nx_):
        if grid[i,j] != 1:   # 必须是 free
            continue
        # 检查 4 邻域是否有 unknown
        for di, dj in [(0,1),(0,-1),(1,0),(-1,0)]:
            ni, nj = i+di, j+dj
            if 0 <= ni < ny_ and 0 <= nj < nx_ and grid[ni,nj] == 0:
                frontier[i,j] = True
                break
n_frontier = frontier.sum()
print(f"前沿单元数 = {n_frontier}", flush=True)

# 2. 聚类(连通域 BFS)
visited = np.zeros_like(frontier, dtype=bool)
clusters = []
for i in range(ny_):
    for j in range(nx_):
        if frontier[i,j] and not visited[i,j]:
            # BFS
            q = deque([(i,j)]); visited[i,j] = True
            cells = []
            while q:
                ci, cj = q.popleft(); cells.append((ci,cj))
                for di, dj in [(0,1),(0,-1),(1,0),(-1,0)]:
                    ni, nj = ci+di, cj+dj
                    if 0 <= ni < ny_ and 0 <= nj < nx_ and frontier[ni,nj] and not visited[ni,nj]:
                        visited[ni,nj] = True; q.append((ni,nj))
            if len(cells) >= 3:   # 过滤过小前沿
                clusters.append(cells)
print(f"前沿聚类数(≥3单元) = {len(clusters)}", flush=True)

# 3. 每个聚类: 质心(观察点) + 评分
def to_world(i, j):
    return xmin + (j+0.5)*res, ymin + (i+0.5)*res

candidates = []
for cl in clusters:
    arr = np.array(cl)
    ci, cj = arr.mean(axis=0)
    cx, cy = to_world(ci, cj)
    info_gain = len(cl)                      # 前沿大小(信息增益)
    path_cost = np.hypot(cx-vx, cy-vy)        # 到达代价
    return_cost = np.hypot(cx-hx, cy-hy)      # 返航代价
    # 地形风险: 前沿附近 occupied(2) 单元数
    risk = 0
    for (ii,jj) in cl:
        for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]:
            ni,nj = ii+di, jj+dj
            if 0<=ni<ny_ and 0<=nj<nx_ and grid[ni,nj]==2:
                risk += 1
    score = info_gain / (1.0 + 0.5*path_cost + 0.3*return_cost + 0.1*risk)
    candidates.append(dict(cx=cx, cy=cy, info=info_gain, path=path_cost,
                           ret=return_cost, risk=risk, score=score))

# 4. 排序取最优
candidates.sort(key=lambda c: -c["score"])
print(f"\n=== 前沿候选(按评分排序, 前10) ===", flush=True)
print(f"{'排名':<4}{'观察点(x,y)':<20}{'信息增益':<10}{'到达代价':<10}{'返航代价':<10}{'地形风险':<10}{'评分':<8}", flush=True)
for k, c in enumerate(candidates[:10], 1):
    print(f"{k:<4}({c['cx']:.2f},{c['cy']:.2f}){'':<6}{c['info']:<10}{c['path']:.2f}{'m':<6}{c['ret']:.2f}{'m':<6}{c['risk']:<10}{c['score']:.2f}", flush=True)

best = candidates[0] if candidates else None
if best:
    print(f"\n>>> 最优观察点: ({best['cx']:.2f}, {best['cy']:.2f}) 评分={best['score']:.2f}", flush=True)

# 5. 可视化
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.family']='Noto Sans CJK SC'; plt.rcParams['axes.unicode_minus']=False

fig, ax = plt.subplots(figsize=(12,10))
cmap = matplotlib.colors.ListedColormap(["#888888","#ffffff","#d62728"])
ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2,
          extent=[xmin, xmin+nx*res, ymin, ymin+ny*res], aspect="equal")
# 前沿单元(绿色)
for cl in clusters:
    xs = [to_world(i,j)[0] for i,j in cl]
    ys = [to_world(i,j)[1] for i,j in cl]
    ax.scatter(xs, ys, c="lime", s=3, alpha=0.7)
# 候选观察点(蓝色圆)
for c in candidates[:15]:
    ax.scatter(c["cx"], c["cy"], c="blue", s=40, alpha=0.6, edgecolors="black", linewidths=0.5)
# 最优(金色星)
if best:
    ax.scatter(best["cx"], best["cy"], c="gold", s=400, marker="*", edgecolors="black", label="最优观察点", zorder=6)
ax.plot(traj[:,0], traj[:,1], "b-", lw=1.5, label="轨迹")
ax.scatter(vx, vy, c="cyan", s=150, marker="o", label="当前位置", zorder=5)
ax.scatter(hx, hy, c="green", s=150, marker="*", label="home", zorder=5)
ax.set_title(f"WP5 前沿探索: {len(clusters)} 个前沿, 最优观察点({best['cx']:.2f},{best['cy']:.2f})" if best else "无前沿")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.legend(loc="best", fontsize=8)
fig.tight_layout()
fig.savefig(f"{OUTDIR}/wp5_frontier.png", dpi=110)
print(f"\n已保存 {OUTDIR}/wp5_frontier.png", flush=True)
