#!/usr/bin/env python3
"""离线对比 WP5(前沿簇质心) vs NBV(ray-cast 信息增益) 的选点
用法: python3 nbv_offline_compare.py <bag目录> <输出png前缀>
用 bag 里已录的 /terrain_map + /state_estimation, 不重跑链。
"""
import sys
from collections import deque

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

RES = 0.1
OBSTACLE_THRE = 0.2
UNKNOWN_INT = 1.45

# 与节点一致的参数
MIN_GOAL_DIS = 0.8
MIN_OBS_DIS = 0.2
PATH_W = 0.3
RET_W = 0.3
TURN_W = 0.5
NBV_SAMPLE = 150
NBV_RAY_NUM = 36
NBV_RAY_RANGE = 5.0
SNAP_RADIUS = 0.6


def nearest_free(grid, si, sj):
    r = int(round(SNAP_RADIUS / RES))
    for rad in range(0, r + 1):
        for di in range(-rad, rad + 1):
            for dj in range(-rad, rad + 1):
                if max(abs(di), abs(dj)) != rad:
                    continue
                if grid.get((si + di, sj + dj), 0) == 1:
                    return (si + di, sj + dj)
    return None


def bfs_len(grid, pos, gi, gj):
    si = int(round(pos[0] / RES)); sj = int(round(pos[1] / RES))
    start = (si, sj) if grid.get((si, sj), 0) == 1 else nearest_free(grid, si, sj)
    if start is None or grid.get((gi, gj), 0) != 1:
        return None
    seen = {start}; q = deque([(start[0], start[1], 0)])
    while q:
        cx, cy, d = q.popleft()
        if (cx, cy) == (gi, gj):
            return d * RES
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nb = (cx+dx, cy+dy)
            if nb not in seen and grid.get(nb, 0) == 1:
                seen.add(nb); q.append((nb[0], nb[1], d+1))
    return None


def near_obs(grid, gix, giy):
    m = int(round(MIN_OBS_DIS / RES))
    for di in range(-m, m + 1):
        for dj in range(-m, m + 1):
            if grid.get((gix + di, giy + dj), 0) == 2:
                return True
    return False


def raycast_ig(grid, gix, giy):
    max_r = int(round(NBV_RAY_RANGE / RES))
    angles = np.linspace(0.0, 2.0 * np.pi, NBV_RAY_NUM, endpoint=False)
    seen = set()
    for ang in angles:
        dx = np.cos(ang); dy = np.sin(ang)
        for r in range(1, max_r + 1):
            ix = int(round(gix + r * dx)); iy = int(round(giy + r * dy))
            s = grid.get((ix, iy), 0)
            if s == 2:
                break
            if s == 0:
                seen.add((ix, iy))
    return len(seen)


def extract_frontiers(grid):
    frontier = set()
    for (ix, iy), s in grid.items():
        if s != 1:
            continue
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            if grid.get((ix+dx, iy+dy), 0) == 0:
                frontier.add((ix, iy))
                break
    return list(frontier)


def cluster(frontier):
    visited = set(); clusters = []
    for f in frontier:
        if f in visited:
            continue
        q = deque([f]); visited.add(f); cl = []
        while q:
            c = q.popleft(); cl.append(c)
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                nb = (c[0]+dx, c[1]+dy)
                if nb in frontier and nb not in visited:
                    visited.add(nb); q.append(nb)
        if len(cl) >= 2:
            clusters.append(cl)
    return clusters


def wp5_select(grid, pos, yaw, home):
    """WP5: 前沿簇质心选点, 返回 (gx,gy,gix,giy,info,path,turn,score) 或 None"""
    frontier = extract_frontiers(grid)
    if not frontier:
        return None
    clusters = cluster(frontier)
    if not clusters:
        return None
    cands = []
    for cl in clusters:
        c = np.array(cl)
        mi, mj = c.mean(axis=0)
        k = int(np.argmin((c[:, 0]-mi)**2 + (c[:, 1]-mj)**2))
        gix, giy = int(c[k, 0]), int(c[k, 1])
        gx, gy = gix*RES, giy*RES
        eu = np.hypot(gx-pos[0], gy-pos[1])
        if eu < MIN_GOAL_DIS or near_obs(grid, gix, giy):
            continue
        bfs = bfs_len(grid, pos, gix, giy)
        if bfs is None:
            continue
        info = 0
        r = int(1.5 / RES)
        for di in range(-r, r+1):
            for dj in range(-r, r+1):
                if grid.get((gix+di, giy+dj), 0) == 0:
                    info += 1
        ret = np.hypot(gx-home[0], gy-home[1])
        ga = np.arctan2(gy-pos[1], gx-pos[0])
        turn = abs((ga-yaw+np.pi) % (2*np.pi)-np.pi)
        score = info/(1.0+PATH_W*bfs+RET_W*ret+TURN_W*turn)
        cands.append((score, gx, gy, gix, giy, info, bfs, turn))
    if not cands:
        return None
    cands.sort(key=lambda c: -c[0])
    b = cands[0]
    return (b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[0])


def nbv_select(grid, pos, yaw, home):
    """NBV: 自由空间采样 + ray-cast IG, 返回 (gx,gy,gix,giy,ig,path,turn,score) 或 None"""
    free_cells = [(ix, iy) for (ix, iy), s in grid.items() if s == 1]
    if not free_cells:
        return None
    if len(free_cells) > NBV_SAMPLE:
        idx = np.random.choice(len(free_cells), NBV_SAMPLE, replace=False)
        free_cells = [free_cells[i] for i in idx]
    cands = []
    for (gix, giy) in free_cells:
        gx, gy = gix*RES, giy*RES
        eu = np.hypot(gx-pos[0], gy-pos[1])
        if eu < MIN_GOAL_DIS or near_obs(grid, gix, giy):
            continue
        bfs = bfs_len(grid, pos, gix, giy)
        if bfs is None:
            continue
        ig = raycast_ig(grid, gix, giy)
        if ig <= 0:
            continue
        ret = np.hypot(gx-home[0], gy-home[1])
        ga = np.arctan2(gy-pos[1], gx-pos[0])
        turn = abs((ga-yaw+np.pi) % (2*np.pi)-np.pi)
        score = ig/(1.0+PATH_W*bfs+RET_W*ret+TURN_W*turn)
        cands.append((score, gx, gy, gix, giy, ig, bfs, turn))
    if not cands:
        return None
    cands.sort(key=lambda c: -c[0])
    b = cands[0]
    return (b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[0])


def main():
    bag = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else "/home/lch/dog/tmp/nbv_cmp"

    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"),
           ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    grid = {}; traj = []; goals = []
    t0 = None
    while r.has_next():
        topic, data, t = r.read_next()
        ts = t/1e9
        if t0 is None:
            t0 = ts
        if topic == "/terrain_map":
            for p in point_cloud2.read_points(deserialize_message(data, PointCloud2),
                                              field_names=["x", "y", "intensity"], skip_nans=True):
                x, y, it = p[0], p[1], p[2]
                ix, iy = int(round(x/RES)), int(round(y/RES))
                s = 0 if it >= UNKNOWN_INT else (2 if it >= OBSTACLE_THRE else 1)
                grid[(ix, iy)] = max(grid.get((ix, iy), 0), s)
        elif topic == "/state_estimation":
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            yaw = np.arctan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
            traj.append((ts, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        elif topic == "/goal_point":
            m = deserialize_message(data, PointStamped)
            goals.append((ts, m.point.x, m.point.y))

    name = bag.split("/")[-1]
    print(f"===== {name} =====  terrain={len(grid)}  traj={len(traj)}  goals={len(goals)}")

    # 选几个采样时刻(避开开头/结尾)
    ts_arr = [t for t, _, _, _ in traj]
    t_span = ts_arr[-1] - ts_arr[0]
    sample_ts = [ts_arr[0] + t_span * f for f in (0.2, 0.4, 0.6, 0.8)]

    print(f"{'t':>5s} | {'WP5目标':>14s} {'info':>5s} {'score':>6s} | {'NBV目标':>14s} {'IG':>5s} {'score':>6s} | 相同?")
    wp5_pts = []; nbv_pts = []
    for st in sample_ts:
        j = min(np.searchsorted(ts_arr, st), len(traj)-1)
        _, px, py, pyaw = traj[j]
        home = (traj[0][1], traj[0][2])
        pos = (px, py)
        w = wp5_select(grid, pos, pyaw, home)
        n = nbv_select(grid, pos, pyaw, home)
        wstr = f"({w[0]:.2f},{w[1]:.2f})" if w else "None"
        nstr = f"({n[0]:.2f},{n[1]:.2f})" if n else "None"
        print(f"{st-t0:5.0f}s | {wstr:>14s} {w[4]:5d} {w[7]:6.2f} | {nstr:>14s} {n[4]:5d} {n[7]:6.2f} | {wstr==nstr}")
        if w:
            wp5_pts.append((w[0], w[1]))
        if n:
            nbv_pts.append((n[0], n[1]))

    # 画图: 栅格 + 轨迹 + WP5/NBV 选点
    for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
               '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        try: fm.fontManager.addfont(_f)
        except Exception: pass
    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    if grid:
        xs = [k[0] for k in grid]; ys = [k[1] for k in grid]
        ixmin, ixmax = min(xs), max(xs); iymin, iymax = min(ys), max(ys)
        gmap = np.zeros((iymax-iymin+1, ixmax-ixmin+1), dtype=np.uint8)
        for (ix, iy), s in grid.items():
            gmap[iy-iymin, ix-ixmin] = s
        xmin, ymin = ixmin*RES, iymin*RES
        xmax, ymax = (ixmax+1)*RES, (iymax+1)*RES
    else:
        gmap = np.zeros((1, 1)); xmin = ymin = 0; xmax = ymax = 1

    fig, ax = plt.subplots(figsize=(11, 9))
    cmap = ListedColormap(["#888888", "#ffffff", "#d62728"])
    ax.imshow(gmap, origin="lower", cmap=cmap, vmin=0, vmax=2,
              extent=[xmin, xmax, ymin, ymax], aspect="equal", alpha=0.85)
    if traj:
        tr = np.array([(x, y) for _, x, y, _ in traj])
        ax.plot(tr[:, 0], tr[:, 1], "b-", lw=1.2, alpha=0.7, label="狗轨迹")
        ax.scatter(tr[0, 0], tr[0, 1], c="green", s=120, marker="o", label="起点", zorder=6)
    if wp5_pts:
        w = np.array(wp5_pts)
        ax.scatter(w[:, 0], w[:, 1], c="red", s=100, marker="x", linewidths=2.5, label="WP5(前沿)", zorder=5)
    if nbv_pts:
        n = np.array(nbv_pts)
        ax.scatter(n[:, 0], n[:, 1], c="blue", s=140, marker="*", label="NBV(ray-cast)", zorder=5)
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"{name}\nWP5(红x) vs NBV(蓝*) 选点对比 (灰=unknown 白=free 红=障碍)")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out + ".png", dpi=110)
    print(f"已保存 {out}.png")


if __name__ == "__main__":
    main()
