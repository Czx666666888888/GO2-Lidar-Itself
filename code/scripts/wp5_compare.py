#!/usr/bin/env python3
"""WP5 v1 vs v2 对比: 显示簇中心点数量差异(194→29→18可达→1最优)"""
import numpy as np
from collections import deque
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.family']='Noto Sans CJK SC'; plt.rcParams['axes.unicode_minus']=False

OUTDIR = "/home/lch/dog/tmp/wp2"
grid = np.load(f"{OUTDIR}/grid3state.npy")
meta = np.load(f"{OUTDIR}/grid_meta.npz")
xmin, ymin, res, nx, ny = meta["xmin"], meta["ymin"], meta["res"], meta["nx"], meta["ny"]
traj = np.load(f"{OUTDIR}/trajectory.npy")
home = np.load(f"{OUTDIR}/home_anchor.npy")
ny_, nx_ = grid.shape
cmap = matplotlib.colors.ListedColormap(["#888888","#ffffff","#d62728"])

def to_world(i,j): return xmin+(j+0.5)*res, ymin+(i+0.5)*res
def to_grid(x,y): return int(round((y-ymin)/res)), int(round((x-xmin)/res))

frontier = np.zeros_like(grid, dtype=bool)
for i in range(ny_):
    for j in range(nx_):
        if grid[i,j]==1 and any(0<=i+di<ny_ and 0<=j+dj<nx_ and grid[i+di,j+dj]==0 for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]):
            frontier[i,j]=True

visited = np.zeros_like(frontier, dtype=bool); raw=[]
for i in range(ny_):
    for j in range(nx_):
        if frontier[i,j] and not visited[i,j]:
            q=deque([(i,j)]); visited[i,j]=True; c=[]
            while q:
                a,b=q.popleft(); c.append((a,b))
                for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]:
                    na,nb=a+di,b+dj
                    if 0<=na<ny_ and 0<=nb<nx_ and frontier[na,nb] and not visited[na,nb]:
                        visited[na,nb]=True; q.append((na,nb))
            if len(c)>=3: raw.append(c)

def centroid(cl): a=np.array(cl); return a.mean(axis=0)
merged=[]
for cl in raw:
    c=centroid(cl); done=False
    for m in merged:
        if np.hypot(c[0]-m[0],c[1]-m[1])<1.0/res:
            m[0]=(m[0]*len(m[2])+c[0]*len(cl))/(len(m[2])+len(cl))
            m[1]=(m[1]*len(m[2])+c[1]*len(cl))/(len(m[2])+len(cl))
            m[2].extend(cl); done=True; break
    if not done: merged.append([c[0],c[1],list(cl)])

vx,vy = traj[-1,0], traj[-1,1]
def reachable(gi,gj):
    if not(0<=gi<ny_ and 0<=gj<nx_) or grid[gi,gj]!=1: return False
    si,sj=to_grid(vx,vy)
    if not(0<=si<ny_ and 0<=sj<nx_) or grid[si,sj]!=1: return False
    seen=np.zeros_like(grid,dtype=bool); q=deque([(si,sj)]); seen[si,sj]=True
    while q:
        a,b=q.popleft()
        if (a,b)==(gi,gj): return True
        for di,dj in [(0,1),(0,-1),(1,0),(-1,0)]:
            na,nb=a+di,b+dj
            if 0<=na<ny_ and 0<=nb<nx_ and not seen[na,nb] and grid[na,nb]==1:
                seen[na,nb]=True; q.append((na,nb))
    return False

cands=[]
for mi,mj,cl in merged:
    cx,cy=to_world(mi,mj)
    r=int(1.5/res); info=0
    for di in range(-r,r+1):
        for dj in range(-r,r+1):
            a,b=int(mi)+di,int(mj)+dj
            if 0<=a<ny_ and 0<=b<nx_ and grid[a,b]==0: info+=1
    path=np.hypot(cx-vx,cy-vy); ret=np.hypot(cx-home[0],cy-home[1])
    reach=reachable(int(mi),int(mj))
    score=info/(1+0.5*path+0.3*ret)
    cands.append([cx,cy,info,path,ret,reach,score])
cands.sort(key=lambda c:-c[6])
best=cands[0] if cands else None

fig, axes = plt.subplots(1, 2, figsize=(18, 8))
ax=axes[0]
ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2, extent=[xmin,xmin+nx*res,ymin,ymin+ny*res], aspect="equal")
for cl in raw:
    c=centroid(cl); cx,cy=to_world(c[0],c[1]); ax.scatter(cx,cy,c="red",s=15,alpha=0.8)
ax.plot(traj[:,0],traj[:,1],"b-",lw=1.5); ax.scatter(home[0],home[1],c="green",s=120,marker="*",zorder=5)
ax.set_title(f"v1: {len(raw)} 个前沿簇(碎, 无合并无过滤)")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)")

ax=axes[1]
ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2, extent=[xmin,xmin+nx*res,ymin,ymin+ny*res], aspect="equal")
reach_n=0
for mi,mj,cl in merged:
    cx,cy=to_world(mi,mj); r=reachable(int(mi),int(mj))
    if r: reach_n+=1; ax.scatter(cx,cy,c="blue",s=60,edgecolors="black",linewidths=0.5)
    else: ax.scatter(cx,cy,c="gray",s=40,alpha=0.5)
if best:
    ax.scatter(best[0],best[1],c="gold",s=400,marker="*",edgecolors="black",zorder=6,label="最优观察点")
ax.plot(traj[:,0],traj[:,1],"b-",lw=1.5,label="轨迹")
ax.scatter(vx,vy,c="cyan",s=150,marker="o",label="当前位置",zorder=5)
ax.scatter(home[0],home[1],c="green",s=150,marker="*",label="home",zorder=5)
ax.set_title(f"v2: {len(merged)} 簇 → {reach_n} 可达 → 1 最优")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.legend(loc="best",fontsize=8)

fig.suptitle("WP5 改进: 红点=194原始簇中心 → 蓝点=29合并簇(灰=不可达) → 金星=最优", fontsize=13, fontweight="bold")
fig.tight_layout()
fig.savefig(f"{OUTDIR}/wp5_v1_v2对比.png", dpi=110)
print(f"已保存 {OUTDIR}/wp5_v1_v2对比.png")
