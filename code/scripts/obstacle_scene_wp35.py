#!/usr/bin/env python3
"""WP3+WP5 障碍场景离线验证:
用新采集的障碍场景地形图(terrain_map, 固件odom建图)做:
  1. 三态栅格(0=unknown, 1=free, 2=occupied)
  2. WP5 前沿探索: 前沿提取+聚类+可达性+评分 -> 最优观察点
  3. WP3 路径规划: 当前位置 -> 最优观察点 的局部路径
"""
import numpy as np
from collections import deque

RES = 0.1
OBSTACLE_THRE = 0.3
UNKNOWN_INT = 1.45

# 合并两个障碍场景
d1 = np.load("/home/lch/dog/tmp/mapdata_obstacle_01/terrain_map.npy")
d2 = np.load("/home/lch/dog/tmp/mapdata_obstacle_02/terrain_map.npy")
pts = np.vstack([d1, d2])
x, y, z, it = pts[:,0], pts[:,1], pts[:,2], pts[:,3]
print(f"合并点云: {len(pts)} 点 (场景1 {len(d1)} + 场景2 {len(d2)})")

# 栅格范围
xmin, xmax = np.floor(x.min()/RES)*RES, np.ceil(x.max()/RES)*RES
ymin, ymax = np.floor(y.min()/RES)*RES, np.ceil(y.max()/RES)*RES
nx = int(round((xmax-xmin)/RES)) + 1
ny = int(round((ymax-ymin)/RES)) + 1
print(f"栅格: {nx} x {ny} ({xmax-xmin:.1f}m x {ymax-ymin:.1f}m)")

# 三态栅格
grid = np.zeros((ny, nx), dtype=np.uint8)
ix = ((x - xmin)/RES).astype(int)
iy = ((y - ymin)/RES).astype(int)
valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
ix, iy, it = ix[valid], iy[valid], it[valid]

cell = {}
for k in range(len(ix)):
    key = (iy[k], ix[k])
    if it[k] >= UNKNOWN_INT:
        s = 0
    elif it[k] >= OBSTACLE_THRE:
        s = 2
    else:
        s = 1
    cell[key] = max(cell.get(key, 0), s)
for (gy, gx), s in cell.items():
    grid[gy, gx] = s

n_free = (grid == 1).sum(); n_occ = (grid == 2).sum(); n_unk = (grid == 0).sum()
print(f"三态: free={n_free} occ={n_occ} unknown={n_unk} "
      f"({n_free/(nx*ny)*100:.1f}%/{n_occ/(nx*ny)*100:.1f}%/{n_unk/(nx*ny)*100:.1f}%)")

# ===== WP5 前沿探索 =====
frontier = np.zeros_like(grid, dtype=bool)
for i in range(1, ny-1):
    for j in range(1, nx-1):
        if grid[i,j] != 1: continue
        if any(grid[i+di, j+dj] == 0 for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]):
            frontier[i,j] = True

visited = np.zeros_like(frontier, dtype=bool)
clusters = []
for i in range(ny):
    for j in range(nx):
        if frontier[i,j] and not visited[i,j]:
            q = deque([(i,j)]); visited[i,j] = True; cl = []
            while q:
                ci,cj = q.popleft(); cl.append((ci,cj))
                for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]:
                    ni,nj = ci+di, cj+dj
                    if 0<=ni<ny and 0<=nj<nx and frontier[ni,nj] and not visited[ni,nj]:
                        visited[ni,nj] = True; q.append((ni,nj))
            if len(cl) >= 5:
                clusters.append(cl)
print(f"前沿簇: {len(clusters)}")

def centroid(cl):
    a = np.array(cl); return a.mean(axis=0)

# 合并邻近簇(<1m)
merged = []
for cl in clusters:
    c = centroid(cl); best = None
    for m in merged:
        if np.hypot(c[0]-m[0], c[1]-m[1]) < 1.0/RES: best = m; break
    if best is None: merged.append([c[0], c[1], list(cl)])
    else:
        best[0] = (best[0]*len(best[2]) + c[0]*len(cl)) / (len(best[2])+len(cl))
        best[1] = (best[1]*len(best[2]) + c[1]*len(cl)) / (len(best[2])+len(cl))
        best[2].extend(cl)
print(f"合并后簇: {len(merged)}")

# 起点: 用点云最密集区域的中心(近似狗起点), 或地图中心
# 用 odom 轨迹起点附近
odom = np.load("/home/lch/dog/tmp/mapdata_obstacle_01/traj_odom.npy")
sx, sy = odom[0,0], odom[0,1]
si, sj = int(round((sy-ymin)/RES)), int(round((sx-xmin)/RES))
print(f"起点: ({sx:.2f},{sy:.2f}) 栅格({si},{sj})")

def to_world(i, j):
    return xmin + (j+0.5)*RES, ymin + (i+0.5)*RES

# 起点修正: 若不在 free 格, 找最近 free 格
if not (0 <= si < ny and 0 <= sj < nx and grid[si, sj] == 1):
    freepos = np.argwhere(grid == 1)
    if len(freepos):
        dist = np.hypot(freepos[:, 0] - si, freepos[:, 1] - sj)
        si, sj = map(int, freepos[dist.argmin()])
        sx, sy = to_world(si, sj)
        print(f"起点修正到最近 free: ({sx:.2f},{sy:.2f}) 栅格({si},{sj})")

# 可达性 BFS
def reachable(gi, gj):
    if not (0<=gi<ny and 0<=gj<nx) or grid[gi,gj] != 1: return False
    if not (0<=si<ny and 0<=sj<nx) or grid[si,sj] != 1: return False
    seen = np.zeros_like(grid, dtype=bool)
    q = deque([(si,sj)]); seen[si,sj] = True
    while q:
        ci,cj = q.popleft()
        if (ci,cj) == (gi,gj): return True
        for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]:
            ni,nj = ci+di,cj+dj
            if 0<=ni<ny and 0<=nj<nx and not seen[ni,nj] and grid[ni,nj]==1:
                seen[ni,nj] = True; q.append((ni,nj))
    return False

# 评分
r = int(1.5/RES)
cands = []
for mi, mj, cl in merged:
    cx, cy = to_world(mi, mj)
    info = sum(1 for di in range(-r,r+1) for dj in range(-r,r+1)
               if 0<=int(mi)+di<ny and 0<=int(mj)+dj<nx and grid[int(mi)+di,int(mj)+dj]==0)
    risk = sum(1 for di in range(-r,r+1) for dj in range(-r,r+1)
               if 0<=int(mi)+di<ny and 0<=int(mj)+dj<nx and grid[int(mi)+di,int(mj)+dj]==2)
    if reachable(int(mi), int(mj)):
        cands.append(dict(cx=cx, cy=cy, info=info, risk=risk,
                          path=np.hypot(cx-sx, cy-sy), mi=mi, mj=mj))
print(f"可达前沿: {len(cands)}/{len(merged)}")
for c in cands:
    c["score"] = c["info"] / (1 + 0.5*c["path"] + 0.1*c["risk"])
cands.sort(key=lambda c: -c["score"])
best = cands[0] if cands else None
if best:
    print(f"最优观察点: ({best['cx']:.2f},{best['cy']:.2f}) info={best['info']} risk={best['risk']} score={best['score']:.1f}")

# ===== WP3 路径规划(简化为 A* 到最优观察点) =====
found = False
path = []
if best:
    gi, gj = int(best["mi"]), int(best["mj"])
    # A* 在 free 栅格上找路径
    import heapq
    def h(a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    start, goal = (si, sj), (gi, gj)
    if grid[start] == 1 and grid[goal] == 1:
        openq = [(0, start)]; gscore = {start: 0}; came = {}
        found = False
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == goal: found = True; break
            for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]:
                nb = (cur[0]+di, cur[1]+dj)
                if 0<=nb[0]<ny and 0<=nb[1]<nx and grid[nb]==1:
                    tg = gscore[cur] + 1
                    if tg < gscore.get(nb, 1e9):
                        gscore[nb] = tg; came[nb] = cur
                        heapq.heappush(openq, (tg+h(nb,goal), nb))
        if found:
            path = [goal]
            while path[-1] != start: path.append(came[path[-1]])
            path = path[::-1]
            print(f"WP3 路径规划: A* 找到路径 {len(path)} 步 ({len(path)*RES:.1f}m)")
        else:
            print("WP3 路径规划: 不可达(无路径)")
    else:
        print("WP3: 起点或目标不可达")

# 可视化
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try: fm.fontManager.addfont(f)
    except: pass
plt.rcParams['font.family']='Noto Sans CJK SC'; plt.rcParams['axes.unicode_minus']=False
fig, ax = plt.subplots(figsize=(11, 8))
cmap = matplotlib.colors.ListedColormap(["#888","#fff","#d62728"])
ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2,
          extent=[xmin, xmax, ymin, ymax], aspect="equal")
for c in cands[:15]:
    ax.scatter(c["cx"], c["cy"], c="blue", s=50, alpha=0.7, edgecolors="k", linewidths=0.5)
if best:
    ax.scatter(best["cx"], best["cy"], c="gold", s=400, marker="*", edgecolors="k", label="最优观察点", zorder=6)
if found:
    pw = [to_world(p[0], p[1]) for p in path]
    ax.plot([p[0] for p in pw], [p[1] for p in pw], "cyan", lw=2.5, label="WP3路径")
ax.scatter(sx, sy, c="cyan", s=150, marker="o", label="起点", zorder=5)
ax.set_title(f"障碍场景 WP3+WP5: {len(merged)}簇 {len(cands)}可达 → 最优({best['cx']:.1f},{best['cy']:.1f})" if best else "无")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.legend(loc="best", fontsize=8)
fig.tight_layout()
fig.savefig("/home/lch/dog/tmp/障碍场景_WP35.png", dpi=110)
print("\n已保存 /home/lch/dog/tmp/障碍场景_WP35.png")
