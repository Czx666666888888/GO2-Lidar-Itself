#!/usr/bin/env python3
"""路径规划演示: 模拟 local_planner 的核心算法
- 36 条候选路径 (每 10° 一条, 覆盖 360°, 每条为一条转向曲线模板)
- 栅格碰撞检测 (occupied 阻塞 / 未知按比例扣分)
- 目标方向加权评分 (dirWeight), 选出最优路径
- 可视化: 灰=碰撞, 绿=可行, 红粗线=选中路径
"""
import numpy as np
from math import atan2, pi, cos, sin, hypot

OUTDIR = "/home/lch/dog/tmp/wp2"
grid = np.load(f"{OUTDIR}/grid3state.npy")
meta = np.load(f"{OUTDIR}/grid_meta.npz")
xmin, ymin, res, nx, ny = meta["xmin"], meta["ymin"], meta["res"], meta["nx"], meta["ny"]
traj = np.load(f"{OUTDIR}/trajectory.npy")
ny_, nx_ = grid.shape

# ---- 起点 = 当前位置, 终点 = WP5 最优观察点 ----
sx, sy = traj[-1, 0], traj[-1, 1]
gx, gy = -3.23, 2.02   # WP5 最优观察点(新口径)
goal_dir = atan2(gy - sy, gx - sx) * 180 / pi   # 目标方向角

def to_grid(x, y):
    return int(round((y - ymin) / res)), int(round((x - xmin) / res))

# 起点所在格
si, sj = to_grid(sx, sy)

# ---- 1. 生成 36 条候选路径 (rotDir 0..35, 方向 = 10*rotDir-180) ----
# 每条路径 = 沿自身方向的"转向曲线模板"(简化: 曲率随 group 变化)
# group: 0=直行, 1..3=左转半径递减, 4..6=右转半径递减 (模拟路径模板)
def sample_path(rot_dir_deg, group, length=2.2, n=20):
    """生成一条曲线路径: 弧线曲率由 group 决定, 再旋转到 rot_dir_deg"""
    ang0 = rot_dir_deg * pi / 180
    if group == 0:
        curve = [(t / n * length, 0.0) for t in range(n + 1)]          # 直线
    else:
        sign = 1 if group <= 3 else -1
        radius = 0.4 + 0.25 * abs(group - (1 if sign > 0 else 4))       # 半径 0.4~1.15m
        arc = length / radius
        curve = []
        for t in range(n + 1):
            a = t / n * arc
            curve.append((radius * sin(a) * sign, radius * (1 - cos(a)) * sign))
    # 旋转
    pts = []
    for x, y in curve:
        pts.append((x * cos(ang0) - y * sin(ang0), x * sin(ang0) + y * cos(ang0)))
    return pts

candidates = []   # (rotDir, group, pts, blocked, penalty)
for rotDir in range(36):
    for group in range(7):
        pts = sample_path(10.0 * rotDir - 180.0, group)
        pts_w = [(sx + x, sy + y) for x, y in pts]     # vehicle 系 → world 系
        # ---- 2. 碰撞检测: 路径点所在格 ----
        blocked = 0
        for x, y in pts_w:
            gi_, gj_ = to_grid(x, y)
            if 0 <= gi_ < ny_ and 0 <= gj_ < nx_:
                if grid[gi_, gj_] == 2:
                    blocked += 1                       # occupied 硬阻塞
        # 未知格按比例计入 penalty(0.5)
        unk = 0
        for x, y in pts_w:
            gi_, gj_ = to_grid(x, y)
            if 0 <= gi_ < ny_ and 0 <= gj_ < nx_ and grid[gi_, gj_] == 0:
                unk += 1
        penalty = 0.5 * unk
        candidates.append(dict(rotDir=rotDir, group=group, pts=pts_w,
                               blocked=blocked, penalty=penalty))

# ---- 3. 评分: 方向接近目标 + 无碰撞 ----
feasible = [c for c in candidates if c["blocked"] == 0]
for c in feasible:
    end = c["pts"][-1]
    end_dir = atan2(end[1] - sy, end[0] - sx) * 180 / pi
    dir_diff = min(abs(goal_dir - end_dir), 360 - abs(goal_dir - end_dir))
    c["score"] = 100.0 * (1 - dir_diff / 180.0) - c["penalty"]
feasible.sort(key=lambda c: -c["score"])
best = feasible[0] if feasible else None

# ---- 3.5 滚动时域: 每帧沿最优路径前进 pathRange, 重新规划, 直到到达 ----
PATH_RANGE = 2.2          # 每帧局部规划长度(模拟 pathRange)
ARRIVE = 0.3              # 到达判定阈值(goalCloseDis)
pos = np.array([sx, sy])
frame_tracks = []         # 每帧: (帧起点, 该帧最优路径点集, 是否到达)
frame = 0
while frame < 15:
    dx, dy = gx - pos[0], gy - pos[1]
    if hypot(dx, dy) < ARRIVE:
        frame_tracks.append((pos.copy(), None, True))
        break
    g_dir = atan2(dy, dx) * 180 / pi
    f_cands = []          # 从新位置重新生成候选 + 评分
    for rotDir in range(36):
        for group in range(7):
            pts_w = [(pos[0] + x, pos[1] + y)
                     for x, y in sample_path(10.0 * rotDir - 180.0, group)]
            blocked = 0
            for x, y in pts_w:
                gi_, gj_ = to_grid(x, y)
                if 0 <= gi_ < ny_ and 0 <= gj_ < nx_ and grid[gi_, gj_] == 2:
                    blocked += 1
            if blocked > 0:
                continue
            unk = 0
            for x, y in pts_w:
                gi_, gj_ = to_grid(x, y)
                if 0 <= gi_ < ny_ and 0 <= gj_ < nx_ and grid[gi_, gj_] == 0:
                    unk += 1
            end_dir = atan2(pts_w[-1][1] - pos[1], pts_w[-1][0] - pos[0]) * 180 / pi
            dd = min(abs(g_dir - end_dir), 360 - abs(g_dir - end_dir))
            f_cands.append(dict(pts=pts_w, score=100.0 * (1 - dd / 180.0) - 0.5 * unk))
    if not f_cands:
        frame_tracks.append((pos.copy(), None, False))
        break
    f_cands.sort(key=lambda c: -c["score"])
    bpath = f_cands[0]["pts"]
    step_len = min(PATH_RANGE, hypot(dx, dy))   # 走 PATH_RANGE 或到目标距离(取小)
    walked = 0.0
    npos = pos.copy()
    for i in range(len(bpath) - 1):
        seg = hypot(bpath[i+1][0] - bpath[i][0], bpath[i+1][1] - bpath[i][1])
        if walked + seg >= step_len:
            t = (step_len - walked) / seg
            npos = np.array([bpath[i][0] + t * (bpath[i+1][0] - bpath[i][0]),
                             bpath[i][1] + t * (bpath[i+1][1] - bpath[i][1])])
            break
        walked += seg
        npos = np.array(bpath[i+1])
    frame_tracks.append((pos.copy(), bpath, False))
    pos = npos
    frame += 1
print(f"\n=== 滚动时域接力: 每帧走 {PATH_RANGE}m, 重新规划, 直至到达 ===")
for i, (p0, bp, arrived) in enumerate(frame_tracks):
    d = hypot(gx - p0[0], gy - p0[1])
    if arrived:
        print(f"  到达 ✓ (第{i}帧, 距目标 {d:.2f}m)")
    else:
        print(f"  帧{i}: 位置({p0[0]:.2f},{p0[1]:.2f}) 距目标 {d:.2f}m")
total_len = sum(hypot(frame_tracks[i+1][0][0] - frame_tracks[i][0][0],
                      frame_tracks[i+1][0][1] - frame_tracks[i][0][1])
                for i in range(len(frame_tracks) - 1))
print(f"实际行驶总长 ≈ {total_len:.2f}m (直线距离 {hypot(gx-sx, gy-sy):.2f}m)")

print(f"=== 路径规划演示 (local_planner 算法模拟) ===")
print(f"起点 = ({sx:.2f},{sy:.2f})  目标 = ({gx:.2f},{gy:.2f})  目标方向 = {goal_dir:.1f}°")
print(f"候选路径 = 36方向 × 7模板 = {len(candidates)} 条, 无碰撞可行 = {len(feasible)} 条")
if best:
    bdir = atan2(best['pts'][-1][1] - sy, best['pts'][-1][0] - sx) * 180 / pi
    bd = min(abs(goal_dir - bdir), 360 - abs(goal_dir - bdir))
    path_len = sum(hypot(best['pts'][i+1][0]-best['pts'][i][0],
                         best['pts'][i+1][1]-best['pts'][i][1]) for i in range(len(best['pts'])-1))
    print(f"\n>>> 最优路径: rotDir={best['rotDir']}(方向{10*best['rotDir']-180}°), group={best['group']}")
    print(f"    末端方向偏差 {bd:.1f}°, 长度 {path_len:.2f}m, 评分 {best['score']:.1f}")
    # 路径到最近障碍的距离(碰撞余量)
    occ = np.argwhere(grid == 2)
    if len(occ):
        dmin = min(min(hypot(px - (xmin + (oj+0.5)*res), py - (ymin + (oi+0.5)*res))
                       for oi, oj in occ)
                   for px, py in best['pts'])
        print(f"    路径-最近障碍距离 = {dmin:.2f}m")
else:
    print(">>> 无可行路径(全部被阻塞)")

# ---- 4. 可视化 ----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
           '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try:
        fm.fontManager.addfont(_f)
    except Exception:
        pass
plt.rcParams['font.family'] = 'Noto Sans CJK SC'
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(12, 10))
cmap = matplotlib.colors.ListedColormap(["#888888", "#ffffff", "#d62728"])
ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2,
          extent=[xmin, xmin + nx * res, ymin, ymin + ny * res], aspect="equal")
# 碰撞路径(灰)
for c in candidates:
    if c["blocked"] > 0:
        xs = [p[0] for p in c["pts"]]; ys = [p[1] for p in c["pts"]]
        ax.plot(xs, ys, color="#bbbbbb", lw=0.6, alpha=0.5)
# 可行路径(绿)
for c in feasible:
    if c is best:
        continue
    xs = [p[0] for p in c["pts"]]; ys = [p[1] for p in c["pts"]]
    ax.plot(xs, ys, color="lime", lw=0.8, alpha=0.55)
# 最优路径(红粗)
if best:
    xs = [p[0] for p in best["pts"]]; ys = [p[1] for p in best["pts"]]
    ax.plot(xs, ys, color="red", lw=4.0, label="最优路径", zorder=5)
    ax.annotate("", xy=(gx, gy), xytext=(sx, sy),
                arrowprops=dict(arrowstyle="->", color="purple", lw=1.5, ls="--"),
                zorder=4)
# 滚动时域各帧最优路径(橙色, 序号)
for i, (p0, bp, arrived) in enumerate(frame_tracks):
    if bp is None:
        continue
    xs = [p[0] for p in bp]; ys = [p[1] for p in bp]
    ax.plot(xs, ys, color="orange", lw=2.0, alpha=0.85, zorder=5)
    ax.scatter(p0[0], p0[1], c="orange", s=40, marker="x", zorder=6)
    ax.text(p0[0] + 0.1, p0[1] + 0.1, f"{i}", fontsize=9, color="darkorange", zorder=7)
# 起终点
ax.scatter(sx, sy, c="cyan", s=150, marker="o", label="当前位置", zorder=6)
ax.scatter(gx, gy, c="gold", s=300, marker="*", edgecolors="black", label="目标(最优观察点)", zorder=6)
ax.plot(traj[:, 0], traj[:, 1], "b-", lw=1.2, alpha=0.5, label="探索轨迹")
ax.set_title(f"路径规划: {len(candidates)}候选({len(feasible)}可行) → 最优方向{10*best['rotDir']-180}° 长度{path_len:.2f}m" if best else "无可行路径")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.legend(loc="best", fontsize=8)
# 显示整个地图范围
ax.set_xlim(xmin, xmin + nx * res)
ax.set_ylim(ymin, ymin + ny * res)
fig.tight_layout()
fig.savefig(f"{OUTDIR}/path_planning_demo.png", dpi=110)
print(f"\n已保存 {OUTDIR}/path_planning_demo.png")
