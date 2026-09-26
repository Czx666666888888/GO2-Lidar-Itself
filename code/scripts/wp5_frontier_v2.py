#!/usr/bin/env python3
"""WP5 v2: 前沿探索增强
改进: 1) 邻近簇合并(<1m)  2) 信息增益=局部未知面积  3) 可达性过滤(BFS沿free)
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
vx, vy = traj[-1,0], traj[-1,1]
hx, hy = home[0], home[1]

def to_grid(x, y):
    return int(round((y-ymin)/res)), int(round((x-xmin)/res))
def to_world(i, j):
    return xmin + (j+0.5)*res, ymin + (i+0.5)*res

# ---- 1. 前沿提取 ----
frontier = np.zeros_like(grid, dtype=bool)
for i in range(ny_):
    for j in range(nx_):
        if grid[i,j] != 1: continue
        if any(0 <= i+di < ny_ and 0 <= j+dj < nx_ and grid[i+di,j+dj]==0
               for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]):
            frontier[i,j] = True

# ---- 2. 初始聚类(BFS) ----
visited = np.zeros_like(frontier, dtype=bool)
clusters = []
for i in range(ny_):
    for j in range(nx_):
        if frontier[i,j] and not visited[i,j]:
            q = deque([(i,j)]); visited[i,j]=True; cells=[]
            while q:
                ci,cj = q.popleft(); cells.append((ci,cj))
                for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]:
                    ni,nj = ci+di,cj+dj
                    if 0<=ni<ny_ and 0<=nj<nx_ and frontier[ni,nj] and not visited[ni,nj]:
                        visited[ni,nj]=True; q.append((ni,nj))
            if len(cells) >= 3:
                clusters.append(cells)
print(f"初始前沿簇 = {len(clusters)}", flush=True)

# ---- 3. 邻近簇合并(<1m) ----
def centroid(cl):
    a = np.array(cl); return a.mean(axis=0)
merged = []
for cl in clusters:
    c = centroid(cl)
    # 找已合并簇中质心最近的
    best = None
    for m in merged:
        if np.hypot(c[0]-m[0], c[1]-m[1]) < 1.0/res:  # 1m
            best = m; break
    if best is None:
        merged.append([c[0], c[1], list(cl)])
    else:
        best[0] = (best[0]*len(best[2]) + c[0]*len(cl)) / (len(best[2])+len(cl))
        best[1] = (best[1]*len(best[2]) + c[1]*len(cl)) / (len(best[2])+len(cl))
        best[2].extend(cl)
print(f"合并后前沿簇 = {len(merged)}", flush=True)

# ---- 4. 可达性 BFS(从车辆沿 free) ----
def reachable(gi, gj):
    if not (0<=gi<ny_ and 0<=gj<nx_): return False
    if grid[gi,gj] != 1: return False
    si, sj = to_grid(vx, vy)
    if not (0<=si<ny_ and 0<=sj<nx_) or grid[si,sj] != 1:
        si, sj = 0, 0  # fallback
    seen = np.zeros_like(grid, dtype=bool)
    q = deque([(si,sj)]); seen[si,sj]=True
    while q:
        ci,cj = q.popleft()
        if (ci,cj)==(gi,gj): return True
        for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]:
            ni,nj = ci+di,cj+dj
            if 0<=ni<ny_ and 0<=nj<nx_ and not seen[ni,nj] and grid[ni,nj]==1:
                seen[ni,nj]=True; q.append((ni,nj))
    return False

# ---- 5. 评分 ----
candidates = []
for mi, mj, cl in merged:
    cx, cy = to_world(mi, mj)
    # 信息增益 = 局部未知面积(质心 1.5m 内 unknown 单元数)
    r = int(1.5/res)
    info = 0
    for di in range(-r, r+1):
        for dj in range(-r, r+1):
            ni, nj = int(mi)+di, int(mj)+dj
            if 0<=ni<ny_ and 0<=nj<nx_ and grid[ni,nj]==0:
                info += 1
    path = np.hypot(cx-vx, cy-vy)
    ret = np.hypot(cx-hx, cy-hy)
    # 风险(新口径): 质心 1.5m 半径内 occupied 格数 —— 与 info 同口径,
    # 直接衡量观察点所处环境的拥挤度。旧口径只数簇内格子贴墙的对数,
    # 会把"贴着墙的小簇"误判为安全(见 (2.78,-1.43) 旧risk=2 实则离墙0.1m)。
    risk = 0
    occ_pts = []
    for di in range(-r, r+1):
        for dj in range(-r, r+1):
            ni, nj = int(mi)+di, int(mj)+dj
            if 0<=ni<ny_ and 0<=nj<nx_ and grid[ni,nj]==2:
                risk += 1
                occ_pts.append((ni,nj))
    # 观察点离最近障碍的余量(m) —— 注意用当前簇质心 (mi,mj), 不能用残留变量 m
    clearance = (min(np.hypot((oi-mi)*res, (oj-mj)*res) for oi,oj in occ_pts)
                 if occ_pts else float("inf"))
    reach = reachable(int(mi), int(mj))
    candidates.append(dict(cx=cx, cy=cy, info=info, path=path, ret=ret,
                           risk=risk, clearance=clearance, reach=reach, cells=len(cl)))

# 过滤不可达
reachable_c = [c for c in candidates if c["reach"]]
print(f"可达候选 = {len(reachable_c)} / {len(candidates)}", flush=True)

for c in reachable_c:
    c["score"] = c["info"] / (1.0 + 0.5*c["path"] + 0.3*c["ret"] + 0.1*c["risk"])
reachable_c.sort(key=lambda c: -c["score"])

print(f"\n=== 可达前沿候选(前8) ===", flush=True)
print(f"{'观察点':<18}{'信息增益':<10}{'到达':<8}{'返航':<8}{'风险':<8}{'余量':<8}{'评分':<8}", flush=True)
for c in reachable_c[:8]:
    clr = f"{c['clearance']:.2f}m" if c['clearance'] != float("inf") else "∞"
    print(f"({c['cx']:.2f},{c['cy']:.2f}){'':<5}{c['info']:<10}{c['path']:.2f}{'m':<4}{c['ret']:.2f}{'m':<4}{c['risk']:<8}{clr:<8}{c['score']:.1f}", flush=True)
best = reachable_c[0] if reachable_c else None
if best:
    print(f"\n>>> 最优观察点: ({best['cx']:.2f},{best['cy']:.2f}) 信息增益={best['info']} 评分={best['score']:.1f}", flush=True)

# ---- 6. 可视化 ----
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
# 合并后的前沿簇(绿色)
for mi, mj, cl in merged:
    xs = [to_world(i,j)[0] for i,j in cl]
    ys = [to_world(i,j)[1] for i,j in cl]
    ax.scatter(xs, ys, c="lime", s=3, alpha=0.6)
for c in reachable_c[:15]:
    ax.scatter(c["cx"], c["cy"], c="blue", s=50, alpha=0.7, edgecolors="black", linewidths=0.5)
if best:
    ax.scatter(best["cx"], best["cy"], c="gold", s=400, marker="*", edgecolors="black", label="最优观察点", zorder=6)
ax.plot(traj[:,0], traj[:,1], "b-", lw=1.5, label="轨迹")
ax.scatter(vx, vy, c="cyan", s=150, marker="o", label="当前位置", zorder=5)
ax.scatter(hx, hy, c="green", s=150, marker="*", label="home", zorder=5)
ax.set_title(f"WP5 v2: 合并后{len(merged)}簇, 可达{len(reachable_c)}个, 最优({best['cx']:.2f},{best['cy']:.2f})" if best else "无")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.legend(loc="best", fontsize=8)
fig.tight_layout()
fig.savefig(f"{OUTDIR}/wp5_frontier_v2.png", dpi=110)
print(f"\n已保存 {OUTDIR}/wp5_frontier_v2.png", flush=True)
