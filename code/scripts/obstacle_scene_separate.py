#!/usr/bin/env python3
"""两组障碍场景分别验证(不合并): 每组独立生成地图俯视图 + 三态栅格"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try: fm.fontManager.addfont(f)
    except: pass
plt.rcParams['font.family']='Noto Sans CJK SC'; plt.rcParams['axes.unicode_minus']=False

RES = 0.1; OBSTACLE_THRE = 0.3; UNKNOWN_INT = 1.45

scenes = [
    ("场景1", "/home/lch/dog/tmp/mapdata_obstacle_01"),
    ("场景2", "/home/lch/dog/tmp/mapdata_obstacle_02"),
]

for tag, d in scenes:
    ter = np.load(f"{d}/terrain_map.npy")
    x, y, z, it = ter[:,0], ter[:,1], ter[:,2], ter[:,3]

    fig, axes = plt.subplots(1, 2, figsize=(15, 6.5))

    # 左: 地形俯视图(高度着色)
    ax = axes[0]
    step = max(1, len(ter)//50000)
    sub = ter[::step]
    sc = ax.scatter(sub[:,0], sub[:,1], c=sub[:,3], s=2, cmap='jet', alpha=0.7, vmin=0, vmax=1.5)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_title(f"{tag} 地形图(高度着色)")
    ax.set_aspect('equal'); ax.set_facecolor('#111')
    plt.colorbar(sc, ax=ax, label="高度(m)")

    # 右: 三态栅格
    xmin, xmax = np.floor(x.min()/RES)*RES, np.ceil(x.max()/RES)*RES
    ymin, ymax = np.floor(y.min()/RES)*RES, np.ceil(y.max()/RES)*RES
    nx = int(round((xmax-xmin)/RES)) + 1; ny = int(round((ymax-ymin)/RES)) + 1
    grid = np.zeros((ny, nx), dtype=np.uint8)
    ix = ((x-xmin)/RES).astype(int); iy = ((y-ymin)/RES).astype(int)
    v = (ix>=0)&(ix<nx)&(iy>=0)&(iy<ny); ix,iy,it = ix[v],iy[v],it[v]
    cell = {}
    for k in range(len(ix)):
        key = (iy[k], ix[k])
        s = 0 if it[k]>=UNKNOWN_INT else (2 if it[k]>=OBSTACLE_THRE else 1)
        cell[key] = max(cell.get(key,0), s)
    for (gy,gx),s in cell.items(): grid[gy,gx]=s
    nf=(grid==1).sum(); no=(grid==2).sum(); nu=(grid==0).sum()
    ax = axes[1]
    cmap = matplotlib.colors.ListedColormap(["#888","#fff","#d62728"])
    ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2,
              extent=[xmin,xmax,ymin,ymax], aspect="equal")
    ax.set_title(f"{tag} 三态栅格\nfree={nf} occ={no} unknown={nu}")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")

    fig.suptitle(f"障碍场景 {tag}: 地形图 + 三态栅格", fontsize=13)
    fig.tight_layout()
    out = f"/home/lch/dog/tmp/{tag}_验证图.png"
    fig.savefig(out, dpi=110)
    print(f"{tag}: 覆盖 {xmax-xmin:.1f}m x {ymax-ymin:.1f}m, "
          f"free {nf/(nx*ny)*100:.1f}% occ {no/(nx*ny)*100:.1f}% unknown {nu/(nx*ny)*100:.1f}%")
    print(f"  已保存 {out}")
