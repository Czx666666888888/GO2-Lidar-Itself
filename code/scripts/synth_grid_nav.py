#!/usr/bin/env python3
"""
自拟栅格地图(道路宽约 1m) + localPlanner 算法忠实复刻 —— 导航效果验证

背景:
  - localPlanner 是 autonomy_stack_go2 的局部规划器(前向, twoWayDrive=false 不倒车)
  - 本脚本不启动 ROS, 直接读入其真实路径模板, 1:1 复刻 localPlanner.cpp 的
    候选路径碰撞检测 + 方向加权评分 + 滚动时域, 在"手绘"的 ~1m 宽道路栅格上跑导航。

忠实复刻的关键点(来自 localPlanner.cpp / local_planner_dry_run.yaml / path_generator.m):
  - 路径模板: startPaths.ply(7 组, 每组 101 点, 长 1m, 末端方向 -27°~+27°)
              pathList.ply(343 条, 每条长 3m, 末端方向 2*atan2 用于评分)
              correspondences.txt(0.02m 体素, 由 rangesearch(searchRadius=0.55) 生成,
              即"距该体素 0.55m 内的所有路径点所属路径")
  - 参数: adjacentRange=3.0, pathScale=0.75(可缩到0.5), pathRange=3.0(可缩到1.0),
          dirThre=90, dirWeight=0.02, pointPerPathThre=2, goalClearRange=0.5,
          twoWayDrive=false, dirToVehicle=false, goalCloseDis=1.0
  - 评分: score = (1-sqrt(sqrt(dirWeight*dirDiff))) * rotDirW^4 * penaltyScore
          到达(<goalCloseDis)时改为 groupDirW^2
  - 无解时: pathScale 0.75 -> 0.5, pathRange 3.0 -> 1.0; 最终无解发布单点(0,0)空路径

关键结论(见文末汇总):
  - 有效净空 = pathScale * searchRadius ≈ 0.75*0.55=0.41m (缩到 0.5 后≈0.28m)
  - 因此 1m 宽道路仅支持直行/缓弯/小绕障; 直角弯需全局 waypoint(far_planner);
    <0.6m 的窄缺口无法通过。

验证场景(道路宽均约 1m):
  S1 直道      —— 1m 直道 8m, 直线到达            (应 PASS)
  S2 斜向循迹  —— 1m 宽斜向走廊(~28°), 循迹对齐    (应 PASS)
  S3 障碍绕行  —— 1m 直道横墙留 0.7m 缺口, 绕障通过 (应 PASS)
  S4 直角弯    —— 1m 宽 L 形走廊 90° 左转          (应 FAIL: 需全局 waypoint)
  S5 死胡同    —— 1m 直道尽头封死, 目标在墙后       (应 FAIL: 正确判死胡同)

用法:
  /home/lch/dog/ros2/humble/bin/python scripts/synth_grid_nav.py [输出目录]
"""
import os, math, sys, time
import numpy as np

BASE = "/home/lch/dog"
PATHS_DIR = BASE + "/thirdparty/autonomy_stack_go2/install/local_planner/share/local_planner/paths"
OUTDIR = sys.argv[1] if len(sys.argv) > 1 else BASE + "/tmp/synth_nav"

# ---------- localPlanner 参数(与 local_planner_dry_run.yaml / cpp 默认一致) ----------
P = dict(
    adjacentRange=3.0, pathScale=0.75, pathRange=3.0, dirThre=90.0, dirWeight=0.02,
    pointPerPathThre=2, goalClearRange=0.5, twoWayDrive=False, dirToVehicle=False,
    goalCloseDis=1.0, minPathScale=0.5, pathScaleStep=0.25, minPathRange=1.0,
    pathRangeStep=0.5, pathCropByGoal=True, checkObstacle=True, useTerrainAnalysis=False,
    obstacleHeightThre=0.3, autonomySpeed=1.0, maxSpeed=1.0, pathScaleBySpeed=True,
    pathRangeBySpeed=True,
    gridVoxelSize=0.02, searchRadius=0.55, gridVoxelOffsetX=3.2, gridVoxelOffsetY=4.5,
    gridVoxelNumX=161, gridVoxelNumY=451,
    pathNum=343, groupNum=7,
)

RES = 0.05          # 自拟栅格渲染分辨率(m)
OBS_SPACING = 0.05  # 障碍点采样间距(m), 对应 laserVoxelSize=0.05
ARRIVE = 0.25       # 到达判定(m)
MAX_FRAMES = 200

PERCEPTION_RANGE = 4.0   # Go2 感知半径(m): 感知范围外=未知(灰)
PERCEPTION_RAYS = 720    # 感知射线数(0.5° 步进)
FOV_DEG = 220.0          # 感知视场: 车头前方 220° 扇形(±110°), 模拟相机 FOV


# ====================================================================
# 1. 读取路径模板
# ====================================================================
def read_after_header(path):
    with open(path) as f:
        lines = [l for l in f.read().splitlines() if l.strip()]
    i = lines.index("end_header") + 1
    return [l.split() for l in lines[i:]]


def load_templates():
    sp = read_after_header(f"{PATHS_DIR}/startPaths.ply")
    start_paths = [[] for _ in range(P["groupNum"])]
    for toks in sp:
        start_paths[int(toks[3])].append([float(toks[0]), float(toks[1]), float(toks[2])])
    for g in range(P["groupNum"]):
        start_paths[g] = np.array(start_paths[g], dtype=np.float64)

    pl = read_after_header(f"{PATHS_DIR}/pathList.ply")
    group_of_path = np.zeros(P["pathNum"], dtype=np.int32)
    end_dir_of_path = np.zeros(P["pathNum"], dtype=np.float64)
    for toks in pl:
        pid, gid = int(toks[3]), int(toks[4])
        group_of_path[pid] = gid
        end_dir_of_path[pid] = 2.0 * math.atan2(float(toks[1]), float(toks[0])) * 180.0 / math.pi

    nvox = P["gridVoxelNumX"] * P["gridVoxelNumY"]
    maxdeg = 343  # 原点附近所有 343 条路径都经过
    corr_lookup = np.full((nvox, maxdeg), -1, dtype=np.int32)
    with open(f"{PATHS_DIR}/correspondences.txt") as f:
        for line in f:
            toks = line.split()
            if not toks:
                continue
            vid = int(toks[0])
            arr = np.array([int(t) for t in toks[1:-1]], dtype=np.int32)
            if arr.size:
                corr_lookup[vid, :arr.size] = arr
    return dict(start_paths=start_paths, group_of_path=group_of_path,
                end_dir_of_path=end_dir_of_path, corr_lookup=corr_lookup)


# ====================================================================
# 2. 自拟栅格地图场景
# ====================================================================
def perimeter_points(rect, spacing):
    """矩形边界按 spacing 采样 -> 障碍点(即激光看到的墙面)"""
    x0, y0, x1, y1 = rect
    pts = []
    n = max(2, int(round((x1 - x0) / spacing)) + 1)
    xs = np.linspace(x0, x1, n)
    for x in xs:
        pts.append((x, y0)); pts.append((x, y1))
    m = max(2, int(round((y1 - y0) / spacing)) + 1)
    ys = np.linspace(y0, y1, m)
    for y in ys:
        pts.append((x0, y)); pts.append((x1, y))
    return np.array(pts)


def build_grid(bounds, wall_rects):
    xmin, xmax, ymin, ymax = bounds
    nx = int(round((xmax - xmin) / RES))
    ny = int(round((ymax - ymin) / RES))
    grid = np.ones((ny, nx), dtype=np.uint8)  # 1 free, 2 occupied (0 留给"未知")
    for (x0, y0, x1, y1) in wall_rects:
        ix0 = int(round((x0 - xmin) / RES)); ix1 = int(round((x1 - xmin) / RES))
        iy0 = int(round((y0 - ymin) / RES)); iy1 = int(round((y1 - ymin) / RES))
        ix0, ix1 = max(0, ix0), min(nx, ix1)
        iy0, iy1 = max(0, iy0), min(ny, iy1)
        grid[iy0:iy1, ix0:ix1] = 2
    occ_parts = [perimeter_points(r, OBS_SPACING) for r in wall_rects]
    occ = np.vstack(occ_parts) if occ_parts else np.zeros((0, 2))
    return occ, dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, nx=nx, ny=ny, grid=grid)


def diagonal_corridor(p0, p1, half, bounds):
    """1m(2*half) 宽斜向走廊: 网格按到中线的距离栅格化, 障碍点取两条边线"""
    xmin, xmax, ymin, ymax = bounds
    nx = int(round((xmax - xmin) / RES)); ny = int(round((ymax - ymin) / RES))
    xs = xmin + (np.arange(nx) + 0.5) * RES
    ys = ymin + (np.arange(ny) + 0.5) * RES
    X, Y = np.meshgrid(xs, ys)
    p0 = np.array(p0, float); p1 = np.array(p1, float)
    v = p1 - p0; vv = np.dot(v, v)
    wx = X - p0[0]; wy = Y - p0[1]
    t = (wx * v[0] + wy * v[1]) / vv
    t = np.clip(t, 0, 1)
    projx = p0[0] + t * v[0]; projy = p0[1] + t * v[1]
    dist = np.hypot(X - projx, Y - projy)
    grid = np.where(dist < half, 1, 2).astype(np.uint8)
    u = v / np.hypot(*v); n = np.array([-u[1], u[0]])
    occ = []
    for s in (-half, half):
        a = p0 + s * n; b = p1 + s * n
        L = max(2, int(round(np.hypot(*(b - a)) / OBS_SPACING)))
        for i in range(L + 1):
            q = a + (b - a) * i / L
            occ.append(q)
    occ = np.array(occ)
    return occ, dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, nx=nx, ny=ny, grid=grid)


def build_from_corridors(bounds, corridor_rects):
    """复杂地图: 从"全墙"开始, 按 corridor_rects 挖出 1m 宽走廊(free)。
    障碍由感知射线实时生成, 故无需预生成 perimeter 点。"""
    xmin, xmax, ymin, ymax = bounds
    nx = int(round((xmax - xmin) / RES))
    ny = int(round((ymax - ymin) / RES))
    grid = np.full((ny, nx), 2, dtype=np.uint8)  # 全墙(occupied)
    for (x0, y0, x1, y1) in corridor_rects:
        ix0 = int(round((x0 - xmin) / RES)); ix1 = int(round((x1 - xmin) / RES))
        iy0 = int(round((y0 - ymin) / RES)); iy1 = int(round((y1 - ymin) / RES))
        ix0, ix1 = max(0, ix0), min(nx, ix1)
        iy0, iy1 = max(0, iy0), min(ny, iy1)
        grid[iy0:iy1, ix0:ix1] = 1  # 走廊=free
    occ = np.zeros((0, 2))
    return occ, dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, nx=nx, ny=ny, grid=grid)


def scenarios():
    sc = {}
    # S1 直道: 道路 y∈[1,2] (宽 1m), x∈[0,9]
    sc["S1_straight"] = dict(
        bounds=(0, 9, 0, 3),
        walls=[(0, 0, 9, 1), (0, 2, 9, 3)],
        start=(0.5, 1.5, 0.0), goal=(8.5, 1.5),
        desc="1m 直道, 直线到达")

    # S2 斜向循迹: 1m 宽斜向走廊(约 28°)
    sc["S2_diagonal"] = dict(
        builder="diagonal", args=((0.5, 0.5), (8.0, 4.5), 0.5, (0, 9, 0, 5.5)),
        start=(0.5, 0.5, 0.0), goal=(8.0, 4.5),
        desc="1m 宽斜向走廊(~28°), 循迹对齐")

    # S3 障碍绕行: 1m 直道, 横墙自顶墙下探 0.3m, 留 0.7m 缺口
    sc["S3_detour"] = dict(
        bounds=(0, 9, 0, 3),
        walls=[(0, 0, 9, 1), (0, 2, 9, 3), (5, 1.7, 5.3, 2)],
        start=(0.5, 1.85, 0.0), goal=(8.5, 1.35),
        desc="1m 直道横墙(留 0.7m 缺口), 绕障通过")

    # S4 直角弯(单目标): 1m 宽 L 形走廊, 90° 左转(应失败: 贪心切角)
    sc["S4_turn90"] = dict(
        bounds=(0, 8, 0, 7),
        walls=[(0, 0, 5.5, 1), (0, 2, 4.5, 7), (5.5, 0, 8, 7)],
        start=(1.0, 1.5, 0.0), goal=(5.0, 6.5),
        desc="1m 宽 L 形走廊, 90° 左转(单目标, 贪心切角)")

    # S4b 直角弯(waypoint 接力): 同一张地图, 沿走廊给中间点
    sc["S4b_turn90_wp"] = dict(
        bounds=(0, 8, 0, 7),
        walls=[(0, 0, 5.5, 1), (0, 2, 4.5, 7), (5.5, 0, 8, 7)],
        start=(1.0, 1.5, 0.0), goal=(5.0, 6.5),
        waypoints=[(5.0, 1.5), (5.0, 3.0)],
        desc="1m 宽 L 形走廊, 90° 左转(waypoint 接力)")

    # S5 死胡同: 1m 直道尽头封死, 目标在墙后
    sc["S5_deadend"] = dict(
        bounds=(0, 7, 0, 3),
        walls=[(0, 0, 4, 1), (0, 2, 4, 3), (4, 0, 4.3, 3)],
        start=(0.5, 1.5, 0.0), goal=(6.0, 1.5),
        desc="1m 直道尽头封死, 目标在墙后(应判死胡同)")

    # S6 复杂地图: 蛇形 3 个 90° 弯 + 3 条支路(2 死胡同 + 1 环路), 目标隐藏
    # 走廊宽 1.6m(1m 走廊的 90° 弯会因 localPlanner 的 rotDirW 怪癖过冲撞墙, 见 FINDINGS)
    sc["S6_complex"] = dict(
        builder="corridors",
        bounds=(0, 16, 0, 10),
        corridors=[(0, 1, 16, 2.6),     # A 底部横道
                   (2, 1, 3.6, 6),       # B 左侧竖道
                   (2, 5, 14, 6.6),      # C 中部横道
                   (12, 5, 13.6, 10),    # D 右侧竖道
                   (5, 3, 6.6, 5),       # 支路1: 下探死胡同
                   (8, 6.6, 9.6, 8.5),   # 支路2: 上探死胡同
                   (10, 1, 11.6, 5)],    # 支路3: 下探环路(连回 A)
        start=(0.5, 1.8, 0.0), goal=(12.8, 9.5),
        waypoints=[(2.8, 1.8), (2.8, 5.8), (12.8, 5.8)],
        desc="复杂地图(1.6m): 蛇形 3×90°弯 + 3 支路(2 死胡同+1 环路)")

    # S7 无 waypoint 前沿探索: 街区式地图(2横3竖, 多环路无死胡同), 目标隐藏
    sc["S7_explore"] = dict(
        builder="corridors",
        bounds=(0, 12, 0, 9),
        corridors=[(0, 1, 12, 2.6),     # H1 下横道
                   (0, 6, 12, 7.6),     # H2 上横道
                   (1, 1, 2.6, 7.6),    # V1 左竖道
                   (5, 1, 6.6, 7.6),    # V2 中竖道
                   (9, 1, 10.6, 7.6)],  # V3 右竖道
        start=(0.5, 1.8, 0.0), goal=(9.8, 6.8),
        explore=True,
        desc="前沿探索: 街区式 2×3 网格(无 waypoint), 隐藏目标")

    # S8 20×20 大场景: 树状(无环路)主干+鱼骨支路+岔路+死胡同, 开倒车(twoWayDrive)探索
    sc["S8_20x20"] = dict(
        builder="corridors",
        bounds=(0, 20, 0, 20),
        corridors=[(9, 1, 10.6, 19),       # 主干 V0(竖直)
                   (1, 3, 10.6, 4.6),      # R1 左支(死胡同)
                   (9, 6, 19, 7.6),        # R2 右支(带岔路 F1)
                   (1, 9, 10.6, 10.6),     # R3 左支(死胡同)
                   (9, 12, 19, 13.6),      # R4 右支(带岔路 F2)
                   (1, 15, 10.6, 16.6),    # R5 左支(死胡同)
                   (13, 7.4, 14.6, 11),    # F1 岔路(自 R2 上探, 死胡同)
                   (12, 9, 13.6, 12.2)],   # F2 岔路(自 R4 下探, 死胡同)
        start=(9.8, 1.5, 1.5708), goal=(1.5, 3.8),    # 起点朝北(主干底), 目标在第一个树杈 R1 死胡同末端
        explore=True, two_way=True,
        desc="20×20 树状(无环路)主干+鱼骨支路+岔路+死胡同")

    # S9 far_planner 全局规划: 房间内多个立柱障碍, 可见性图+Dijkstra 绕柱
    sc["S9_far"] = dict(
        bounds=(0, 12, 0, 8),
        walls=[(3, 0, 3.6, 3), (3, 5, 3.6, 8),      # 左柱(上下, 中间留 2m 缺口)
               (6, 2, 6.6, 6),                       # 中柱
               (9, 0, 9.6, 2.5), (9, 5.5, 9.6, 8)],  # 右柱(上下, 中间留 3m 缺口)
        start=(0.5, 4.0, 0.0), goal=(11.5, 4.0),
        planner="far",
        desc="far_planner 可见性图+Dijkstra 绕柱全局规划")

    for name, d in sc.items():
        if d.get("builder") == "diagonal":
            occ, meta = diagonal_corridor(*d["args"])
        elif d.get("builder") == "corridors":
            occ, meta = build_from_corridors(d["bounds"], d["corridors"])
        else:
            occ, meta = build_grid(d["bounds"], d["walls"])
        d["occ"] = occ
        d["meta"] = meta
    return sc


# ====================================================================
# 2.5 Go2 感知模型: 车头前方 220° 扇形(半径 R), 射线遇障碍即停
# ====================================================================
def perceive(vx, vy, vyaw, meta, R):
    """从 (vx,vy) 做前方 220° 扇形射线感知(半径 R), 射线打到 occupied 就停。
    返回 (可见障碍点(N,2), 已见掩膜 bool[grid.shape])。
    掩膜为 True 的格 = 本位置"已见"(扇形内); 掩膜外 = 未知(灰)。"""
    grid = meta["grid"]
    ny, nx = grid.shape
    xmin, ymin = meta["xmin"], meta["ymin"]
    seen = np.zeros_like(grid, dtype=bool)
    fov_half = math.radians(FOV_DEG) / 2.0
    angs = np.linspace(vyaw - fov_half, vyaw + fov_half, PERCEPTION_RAYS, endpoint=False)
    cos_a, sin_a = np.cos(angs), np.sin(angs)
    t = np.arange(0.0, R + RES, RES)
    occ_pts = []
    for ca, sa in zip(cos_a, sin_a):
        xs = vx + t * ca
        ys = vy + t * sa
        ix = np.floor((xs - xmin) / RES).astype(np.int64)
        iy = np.floor((ys - ymin) / RES).astype(np.int64)
        valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        ix, iy = ix[valid], iy[valid]
        if ix.size == 0:
            continue
        vals = grid[iy, ix]
        occ = np.nonzero(vals == 2)[0]
        end = occ[0] if occ.size else ix.size - 1
        seen[iy[:end + 1], ix[:end + 1]] = True
        if occ.size:
            k = occ[0]
            occ_pts.append((xmin + (ix[k] + 0.5) * RES, ymin + (iy[k] + 0.5) * RES))
    if occ_pts:
        occ_pts = np.unique(np.array(occ_pts), axis=0)
    else:
        occ_pts = np.zeros((0, 2))
    return occ_pts, seen


def in_target_fov(gx, gy, x, y, yaw):
    """目标是否落在车头前方 220° 扇形视场内(±110°)"""
    ang = math.atan2(gy - y, gx - x)
    diff = abs((ang - yaw + math.pi) % (2 * math.pi) - math.pi)   # [0, π]
    return diff <= math.radians(FOV_DEG) / 2.0


# ====================================================================
# 3. localPlanner 算法忠实复刻(向量化)
# ====================================================================
class LocalPlannerSim:
    def __init__(self, T):
        self.T = T
        self.pathNum = P["pathNum"]
        self.groupNum = P["groupNum"]
        self.rot_deg = np.array([10.0 * r - 180.0 for r in range(36)])
        self.rot_c = np.cos(np.radians(self.rot_deg))
        self.rot_s = np.sin(np.radians(self.rot_deg))

    def plan(self, vx, vy, vyaw, gx, gy, occ_world, two_way=None):
        T, Pc = self.T, P
        cy, sy = math.cos(vyaw), math.sin(vyaw)

        dxg, dyg = gx - vx, gy - vy
        relX = dxg * cy + dyg * sy
        relY = -dxg * sy + dyg * cy
        joy_dir = math.atan2(relY, relX) * 180.0 / math.pi
        tw = Pc["twoWayDrive"] if two_way is None else two_way
        if not tw:
            joy_dir = max(-90.0, min(90.0, joy_dir))
        rel_goal_dis = math.hypot(dxg, dyg)

        dxo = occ_world[:, 0] - vx
        dyo = occ_world[:, 1] - vy
        xv = dxo * cy + dyo * sy
        yv = -dxo * sy + dyo * cy
        dis = np.hypot(xv, yv)
        keep = (dis < Pc["adjacentRange"]) & (xv > -0.2)
        xv, yv = xv[keep], yv[keep]

        def_scale = Pc["pathScale"]
        path_scale = def_scale
        path_range = Pc["pathRange"]

        ad = np.abs(joy_dir - self.rot_deg)
        ad = np.where(ad > 180.0, 360.0 - ad, ad)
        gate = ad <= Pc["dirThre"]
        rot_dirs = np.nonzero(gate)[0]

        selected = None
        while path_scale >= Pc["minPathScale"] and path_range >= Pc["minPathRange"]:
            clear_list = np.zeros((36, self.pathNum), dtype=np.int32)
            xs = xv / path_scale
            ys = yv / path_scale
            d = np.hypot(xs, ys)
            mask = d < path_range / path_scale
            if Pc["pathCropByGoal"]:
                mask &= d <= (rel_goal_dis + Pc["goalClearRange"]) / path_scale
            xs, ys = xs[mask], ys[mask]

            for rotDir in rot_dirs:
                c, s = self.rot_c[rotDir], self.rot_s[rotDir]
                x2 = c * xs + s * ys
                y2 = -s * xs + c * ys
                scaleY = x2 / Pc["gridVoxelOffsetX"] + Pc["searchRadius"] / Pc["gridVoxelOffsetY"] \
                    * (Pc["gridVoxelOffsetX"] - x2) / Pc["gridVoxelOffsetX"]
                indX = ((Pc["gridVoxelOffsetX"] + Pc["gridVoxelSize"] / 2 - x2) / Pc["gridVoxelSize"]).astype(np.int64)
                indY = ((Pc["gridVoxelOffsetY"] + Pc["gridVoxelSize"] / 2 - y2 / scaleY) / Pc["gridVoxelSize"]).astype(np.int64)
                valid = (indX >= 0) & (indX < Pc["gridVoxelNumX"]) & (indY >= 0) & (indY < Pc["gridVoxelNumY"])
                ind = Pc["gridVoxelNumY"] * indX[valid] + indY[valid]
                if ind.size == 0:
                    continue
                pids = T["corr_lookup"][ind]
                pids = pids[pids >= 0]
                if pids.size:
                    clear_list[rotDir] += np.bincount(pids, minlength=self.pathNum)

            group_score = np.zeros((36, self.groupNum), dtype=np.float64)
            for rotDir in rot_dirs:
                rot_deg = self.rot_deg[rotDir]
                rot_dir_w = (abs(rotDir - 9) + 1) if rotDir < 18 else (abs(rotDir - 27) + 1)
                clear = clear_list[rotDir] < Pc["pointPerPathThre"]
                if not clear.any():
                    continue
                pids = np.nonzero(clear)[0]
                gids = T["group_of_path"][pids]
                dir_diff = np.abs(joy_dir - T["end_dir_of_path"][pids] - rot_deg)
                dir_diff = np.where(dir_diff > 360.0, dir_diff - 360.0, dir_diff)
                dir_diff = np.where(dir_diff > 180.0, 360.0 - dir_diff, dir_diff)
                group_dir_w = 4.0 - np.abs(gids - 3)
                if rel_goal_dis < Pc["goalCloseDis"]:
                    score = (1.0 - np.sqrt(np.sqrt(Pc["dirWeight"] * dir_diff))) * group_dir_w ** 2
                else:
                    score = (1.0 - np.sqrt(np.sqrt(Pc["dirWeight"] * dir_diff))) * rot_dir_w ** 4
                score = np.where(score > 0, score, 0.0)
                np.add.at(group_score[rotDir], gids, score)

            sel = int(np.argmax(group_score))
            if group_score[sel // self.groupNum, sel % self.groupNum] > 0:
                rot_dir = sel // self.groupNum
                grp = sel % self.groupNum
                rot_deg = self.rot_deg[rot_dir]
                selected = (rot_dir, grp, rot_deg, path_scale)
                break

            if path_scale >= Pc["minPathScale"] + Pc["pathScaleStep"]:
                path_scale -= Pc["pathScaleStep"]
                path_range = Pc["adjacentRange"] * path_scale / def_scale
            else:
                path_range -= Pc["pathRangeStep"]

        if selected is None:
            return None, dict(joy_dir=joy_dir, rel_dis=rel_goal_dis, feasible=0, sel=None)

        rot_dir, grp, rot_deg, path_scale = selected
        ra = math.radians(rot_deg)
        pts = self.T["start_paths"][grp]
        dd = np.hypot(pts[:, 0], pts[:, 1])
        keep = (dd <= path_range / path_scale) & (dd <= rel_goal_dis / path_scale)
        pts = pts[keep]
        pvx = path_scale * (math.cos(ra) * pts[:, 0] - math.sin(ra) * pts[:, 1])
        pvy = path_scale * (math.sin(ra) * pts[:, 0] + math.cos(ra) * pts[:, 1])
        wx = vx + cy * pvx - sy * pvy
        wy = vy + sy * pvx + cy * pvy
        return np.stack([wx, wy], axis=1), dict(joy_dir=joy_dir, rel_dis=rel_goal_dis,
                                                feasible=int(len(rot_dirs)), sel=selected)


# ====================================================================
# 4. 滚动时域导航
# ====================================================================
def navigate(name, d, sim):
    meta = d["meta"]
    two_way = d.get("two_way")
    sx, sy, syaw = d["start"]
    # 目标序列 = [waypoint...] + [goal]; 依次接力到达
    targets = []
    if d.get("waypoints"):
        targets += [tuple(w) for w in d["waypoints"]]
    targets.append(tuple(d["goal"]))
    gx, gy = targets[0]
    ti = 0
    x, y, yaw = sx, sy, syaw
    frames = [(x, y)]
    best_paths = []
    seen_cum = np.zeros(meta["grid"].shape, dtype=bool)
    anim = []          # 每帧: {pos, seen(累计), path}
    # 隐藏目标: 只有当感知射线真正看到目标格时才"揭示"
    gix = int(math.floor((d["goal"][0] - meta["xmin"]) / RES))
    giy = int(math.floor((d["goal"][1] - meta["ymin"]) / RES))
    goal_seen_frame = None
    reason = "max_frames"
    for f in range(MAX_FRAMES * len(targets)):
        if math.hypot(gx - x, gy - y) < ARRIVE:
            ti += 1
            if ti >= len(targets):
                reason = "reached"
                break
            gx, gy = targets[ti]          # 切到下一个 waypoint/goal
            continue
        # 每次移动后更新感知: 从当前位置做 4m 前方扇形, 累计"已见"
        occ_vis, seen = perceive(x, y, yaw, meta, PERCEPTION_RANGE)
        seen_cum |= seen
        if goal_seen_frame is None and 0 <= giy < meta["ny"] and 0 <= gix < meta["nx"] \
                and seen_cum[giy, gix] \
                and in_target_fov(d["goal"][0], d["goal"][1], x, y, yaw):
            goal_seen_frame = len(anim)   # 当前帧的 anim 索引
        anim.append(dict(pos=(x, y), yaw=yaw, seen=seen_cum.copy(), path=None))
        path, inf = sim.plan(x, y, yaw, gx, gy, occ_vis, two_way)
        anim[-1]["path"] = path
        if path is None:
            reason = "no_path"
            break
        best_paths.append(path)
        px, py = path[-1]
        step = math.hypot(px - x, py - y)
        if step < 1e-6:
            reason = "stuck"
            break
        dg = math.hypot(gx - x, gy - y)
        if step > dg:
            step = dg
            seg = math.hypot(path[-1][0] - x, path[-1][1] - y)
            px = x + (path[-1][0] - x) / seg * step
            py = y + (path[-1][1] - y) / seg * step
        if len(path) >= 2:
            yaw = math.atan2(path[-1][1] - path[-2][1], path[-1][0] - path[-2][0])
        x, y = px, py
        frames.append((x, y))
    anim.append(dict(pos=(x, y), yaw=yaw, seen=seen_cum.copy(), path=None))  # 结束帧(到达/失败位置)
    final_dis = math.hypot(d["goal"][0] - x, d["goal"][1] - y)
    fr = np.array(frames)
    travel = sum(math.hypot(fr[i+1][0]-fr[i][0], fr[i+1][1]-fr[i][1]) for i in range(len(fr)-1))
    return dict(name=name, reason=reason, final_dis=final_dis, frames=fr,
                best_paths=best_paths, travel=travel, seen=seen_cum, anim=anim,
                waypoints=d.get("waypoints"), goal_seen_frame=goal_seen_frame)


# ====================================================================
# 4.5 无 waypoint 前沿探索找隐藏目标
# ====================================================================
def frontier_mask(seen, grid):
    """前沿 = 未知格(~seen) 且 8 邻域内有已知可通行格(seen & free)"""
    known_free = seen & (grid == 1)
    pad = np.pad(known_free, 1, mode="constant")
    dil = np.zeros_like(known_free, dtype=bool)
    ny, nx = known_free.shape
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            if di == 0 and dj == 0:
                continue
            dil |= pad[1 + di:1 + di + ny, 1 + dj:1 + dj + nx]
    return dil & (~seen)


def wp5_frontiers(seen, grid, meta, vx, vy, vyaw, hx, hy):
    """WP5 前沿探索决策(对齐 wp5_frontier_v2.py):
    前沿提取 -> BFS 聚类(≥3格) -> 邻近簇合并(<1m) -> 可达性 BFS -> 评分选最优观察点。
    评分 = info / (1 + 0.5*path + 0.3*ret + 0.1*risk + 0.5*turn),
    info=1.5m 内未知面积, risk=1.5m 内障碍数, turn=朝该观察点需转向角(弧度[0,π])。
    返回按评分降序的候选列表。"""
    from collections import deque
    xmin, ymin = meta["xmin"], meta["ymin"]
    ny, nx = grid.shape
    g3 = np.where(seen, grid, 0).astype(np.uint8)  # 0未知 1free 2occupied

    # 1. 前沿: free 且 4 邻域有 unknown
    unknown = g3 == 0
    free = g3 == 1
    frontier = free & (np.roll(unknown, 1, 0) | np.roll(unknown, -1, 0)
                      | np.roll(unknown, 1, 1) | np.roll(unknown, -1, 1))
    frontier[0, :] = frontier[-1, :] = frontier[:, 0] = frontier[:, -1] = False

    # 2. BFS 聚类(≥3 格)
    clusters = []
    visited = np.zeros_like(frontier)
    for (i, j) in np.argwhere(frontier):
        if visited[i, j]:
            continue
        q = deque([(i, j)]); visited[i, j] = True; cells = []
        while q:
            ci, cj = q.popleft(); cells.append((ci, cj))
            for di, dj in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                ni, nj = ci + di, cj + dj
                if 0 <= ni < ny and 0 <= nj < nx and frontier[ni, nj] and not visited[ni, nj]:
                    visited[ni, nj] = True; q.append((ni, nj))
        if len(cells) >= 3:
            clusters.append(cells)

    # 3. 邻近簇合并(<1m)
    merged = []  # [ci, cj, cells]
    for cl in clusters:
        arr = np.array(cl); ci, cj = arr.mean(axis=0)
        hit = None
        for m in merged:
            if math.hypot(ci - m[0], cj - m[1]) < 1.0 / RES:
                hit = m; break
        if hit is None:
            merged.append([ci, cj, cl])
        else:
            n0, n1 = len(hit[2]), len(cl)
            hit[0] = (hit[0] * n0 + ci * n1) / (n0 + n1)
            hit[1] = (hit[1] * n0 + cj * n1) / (n0 + n1)
            hit[2].extend(cl)

    # 4. 可达性: 一次 BFS 从车辆沿已知 free
    si = int(math.floor((vy - ymin) / RES)); sj = int(math.floor((vx - xmin) / RES))
    reach = np.zeros((ny, nx), dtype=bool)
    if 0 <= si < ny and 0 <= sj < nx and g3[si, sj] == 1:
        q = deque([(si, sj)]); reach[si, sj] = True
        while q:
            ci, cj = q.popleft()
            for di, dj in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                ni, nj = ci + di, cj + dj
                if 0 <= ni < ny and 0 <= nj < nx and g3[ni, nj] == 1 and not reach[ni, nj]:
                    reach[ni, nj] = True; q.append((ni, nj))

    # 5. 评分
    r = int(1.5 / RES)
    candidates = []
    for mi, mj, cl in merged:
        cx = xmin + (mj + 0.5) * RES
        cy = ymin + (mi + 0.5) * RES
        ri = min(ny - 1, max(0, int(round(mi)))); rj = min(nx - 1, max(0, int(round(mj))))
        i0 = max(0, ri - r); i1 = min(ny, ri + r + 1)
        j0 = max(0, rj - r); j1 = min(nx, rj + r + 1)
        sub = g3[i0:i1, j0:j1]
        info = int((sub == 0).sum())
        risk = int((sub == 2).sum())
        path = math.hypot(cx - vx, cy - vy)
        ret = math.hypot(cx - hx, cy - hy)
        turn = abs((math.atan2(cy - vy, cx - vx) - vyaw + math.pi) % (2 * math.pi) - math.pi)
        score = info / (1.0 + 0.5 * path + 0.3 * ret + 0.1 * risk + 0.5 * turn)
        candidates.append(dict(cx=cx, cy=cy, info=info, path=path, ret=ret,
                               risk=risk, turn=turn, score=score,
                               reach=bool(reach[ri, rj]), ncluster=len(merged)))
    candidates.sort(key=lambda c: -c["score"])
    return candidates


def explore(name, d, sim, max_steps=400):
    """无 waypoint 前沿探索: 机器人不知道目标位置, 逐前沿探索, 感知到目标即"找到"。
    找到后局部逼近目标。成功 = 感知到目标(或到达)。"""
    two_way = d.get("two_way")
    meta = d["meta"]
    grid = meta["grid"]
    xmin, ymin = meta["xmin"], meta["ymin"]
    sx, sy, syaw = d["start"]
    gx, gy = d["goal"]
    hx, hy = sx, sy   # home = 起点(用于 WP5 返航代价)
    gix = int(math.floor((gx - xmin) / RES))
    giy = int(math.floor((gy - ymin) / RES))
    x, y, yaw = sx, sy, syaw
    frames = [(x, y)]
    best_paths = []
    seen_cum = np.zeros(grid.shape, dtype=bool)
    anim = []
    target_seen_frame = None
    committed = None       # 当前承诺的前沿目标, 到达后再重选(避免两前沿间振荡)
    reason = "max_steps"

    for f in range(max_steps):
        occ_vis, seen = perceive(x, y, yaw, meta, PERCEPTION_RANGE)
        seen_cum |= seen
        anim.append(dict(pos=(x, y), yaw=yaw, seen=seen_cum.copy(), path=None))

        # 目标是否被感知到(隐藏目标揭示)
        if target_seen_frame is None and 0 <= giy < meta["ny"] and 0 <= gix < meta["nx"] \
                and seen_cum[giy, gix] \
                and in_target_fov(d["goal"][0], d["goal"][1], x, y, yaw):
            target_seen_frame = len(anim) - 1

        # 已找到目标 -> 逼近目标并停
        if target_seen_frame is not None:
            if math.hypot(gx - x, gy - y) < ARRIVE:
                reason = "target_reached"
                break
            path, inf = sim.plan(x, y, yaw, gx, gy, occ_vis, two_way)
            if path is not None:
                anim[-1]["path"] = path
                best_paths.append(path)
                if len(path) >= 2:
                    yaw = math.atan2(path[-1][1] - path[-2][1], path[-1][0] - path[-2][0])
                x, y = path[-1]
                frames.append((x, y))
                continue

        # 前沿探索: WP5 评分选最优观察点(簇合并+信息增益+可达性+风险)
        # 承诺制: 到达当前前沿前不重选, 避免两个前沿之间来回振荡
        if committed is None or math.hypot(committed[0] - x, committed[1] - y) < ARRIVE:
            cands = wp5_frontiers(seen_cum, grid, meta, x, y, yaw, hx, hy)
            if not cands:
                reason = "no_frontier"
                break
            committed = None
            chosen = None
            for c in cands:
                if not c["reach"]:
                    continue
                path, inf = sim.plan(x, y, yaw, c["cx"], c["cy"], occ_vis, two_way)
                if path is not None:
                    committed = (c["cx"], c["cy"])
                    chosen = path
                    break
            if committed is None:  # 回退: 忽略可达性过滤, 直接按评分试
                for c in cands:
                    path, inf = sim.plan(x, y, yaw, c["cx"], c["cy"], occ_vis, two_way)
                    if path is not None:
                        committed = (c["cx"], c["cy"])
                        chosen = path
                        break
            if committed is None:
                reason = "stuck"
                break
        else:
            # 继续导航到已承诺的前沿
            path, inf = sim.plan(x, y, yaw, committed[0], committed[1], occ_vis, two_way)
            if path is None:
                committed = None      # 放弃该前沿, 下帧重选
                continue
            chosen = path
        anim[-1]["path"] = chosen
        best_paths.append(chosen)
        if len(chosen) >= 2:
            yaw = math.atan2(chosen[-1][1] - chosen[-2][1], chosen[-1][0] - chosen[-2][0])
        x, y = chosen[-1]
        frames.append((x, y))

    anim.append(dict(pos=(x, y), yaw=yaw, seen=seen_cum.copy(), path=None))
    final_dis = math.hypot(gx - x, gy - y)
    fr = np.array(frames)
    travel = sum(math.hypot(fr[i+1][0]-fr[i][0], fr[i+1][1]-fr[i][1]) for i in range(len(fr)-1))
    found = target_seen_frame is not None
    return dict(name=name, reason=reason, final_dis=final_dis, frames=fr,
                best_paths=best_paths, travel=travel, seen=seen_cum, anim=anim,
                waypoints=None, goal_seen_frame=target_seen_frame, found=found)


# ====================================================================
# 4.6 far_planner 复刻: 可见性图 + Dijkstra (全局路径规划)
# ====================================================================
FAR_CLEARANCE = 0.5   # kNavClearDist = robot_dim/2 + leaf = 0.4 + 0.1 = 0.5m
FAR_CONVERGE = 0.25   # g_planner/converge_distance
FAR_LEAF = 0.1        # voxel_dim


def obstacle_corners(seen, grid, meta):
    """从已见栅格提取障碍多边形顶点(凸角+凹角)。
    2×2 块: 3占1空=凹角(自由空间角点), 1占3空=凸角(障碍尖端)。返回 (N,2) 世界坐标。"""
    occ = seen & (grid == 2)
    cnt = (occ[:-1, :-1].astype(np.uint8) + occ[:-1, 1:].astype(np.uint8)
           + occ[1:, :-1].astype(np.uint8) + occ[1:, 1:].astype(np.uint8))
    ci, cj = np.where((cnt == 1) | (cnt == 3))
    xmin, ymin = meta["xmin"], meta["ymin"]
    wx = xmin + (cj + 1) * RES
    wy = ymin + (ci + 1) * RES
    return np.stack([wx, wy], axis=1)


def dilate_mask(mask, r):
    """4 邻域方形膨胀 r 格(近似圆盘), 用于障碍净空。"""
    out = mask.copy()
    for _ in range(int(r)):
        up = np.roll(out, 1, 0); up[0, :] = False
        dn = np.roll(out, -1, 0); dn[-1, :] = False
        lf = np.roll(out, 1, 1); lf[:, 0] = False
        rt = np.roll(out, -1, 1); rt[:, -1] = False
        out = out | up | dn | lf | rt
    return out


def line_of_sight_clear(p1, p2, blocked, meta):
    """两点连线是否不穿过障碍(含净空)。采样 FAR_LEAF 步长。"""
    xmin, ymin = meta["xmin"], meta["ymin"]
    d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    n = max(2, int(d / FAR_LEAF) + 1)
    h, w = blocked.shape
    for k in range(1, n):       # 跳过端点(角点在障碍边界上)
        t = k / n
        x = p1[0] + (p2[0] - p1[0]) * t
        y = p1[1] + (p2[1] - p1[1]) * t
        i = int((y - ymin) / RES); j = int((x - xmin) / RES)
        if 0 <= i < h and 0 <= j < w and blocked[i, j]:
            return False
    return True


def far_planner_path(seen, grid, meta, odom_xy, goal_xy):
    """far_planner 全局规划: 障碍角点 → 可见性图 → Dijkstra → 返回全局路径 (M,2) 或 None。"""
    import heapq
    corners = obstacle_corners(seen, grid, meta)
    nodes = [list(odom_xy), list(goal_xy)]
    if len(corners):
        nodes = [list(p) for p in corners] + nodes
    nodes = np.array(nodes, dtype=np.float64)
    n = len(nodes)
    odom_idx, goal_idx = n - 2, n - 1
    # 不可通行 = 未感知(未知) 或 已感知障碍; 即只允许穿过"已感知可通行"区(自由导航语义)
    blocked = ~seen | (grid == 2)
    # 建图(可见边)
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if line_of_sight_clear(nodes[i], nodes[j], blocked, meta):
                d = float(np.hypot(nodes[i, 0] - nodes[j, 0], nodes[i, 1] - nodes[j, 1]))
                adj[i].append((j, d)); adj[j].append((i, d))
    # Dijkstra
    dist = [float("inf")] * n; parent = [-1] * n
    dist[odom_idx] = 0.0
    pq = [(0.0, odom_idx)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        if u == goal_idx:
            break
        for v, w in adj[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd; parent[v] = u; heapq.heappush(pq, (nd, v))
    if dist[goal_idx] == float("inf"):
        return None
    path = [goal_idx]
    while path[-1] != odom_idx:
        path.append(parent[path[-1]])
    return nodes[path[::-1]]


def next_waypoint_from_path(global_path, odom_xy):
    """取全局路径中第一个超出 FAR_CONVERGE 的节点作为下一航点(对齐 NextNavWaypointFromPath)。"""
    if len(global_path) < 2:
        return global_path[-1]
    for p in global_path[1:]:
        if math.hypot(p[0] - odom_xy[0], p[1] - odom_xy[1]) >= FAR_CONVERGE:
            return p
    return global_path[-1]


def navigate_far(name, d, sim, max_steps=800):
    """far_planner(全局可见性图+Dijkstra) → localPlanner(局部) 完整链路。"""
    two_way = d.get("two_way")
    meta = d["meta"]
    grid = meta["grid"]
    sx, sy, syaw = d["start"]
    gx, gy = d["goal"]
    gix = int(math.floor((gx - meta["xmin"]) / RES))
    giy = int(math.floor((gy - meta["ymin"]) / RES))
    hx, hy = sx, sy   # home(用于 WP5 返航代价)
    x, y, yaw = sx, sy, syaw
    frames = [(x, y)]
    best_paths = []
    seen_cum = np.zeros(grid.shape, dtype=bool)
    anim = []
    goal_seen_frame = None
    committed = None
    reason = "max_steps"
    for f in range(max_steps):
        occ_vis, seen = perceive(x, y, yaw, meta, PERCEPTION_RANGE)
        seen_cum |= seen
        anim.append(dict(pos=(x, y), yaw=yaw, seen=seen_cum.copy(), path=None))
        if goal_seen_frame is None and 0 <= giy < meta["ny"] and 0 <= gix < meta["nx"] \
                and seen_cum[giy, gix] and in_target_fov(gx, gy, x, y, yaw):
            goal_seen_frame = len(anim) - 1
        if math.hypot(gx - x, gy - y) < ARRIVE:
            reason = "reached"; break
        # 全局层: far_planner(目标可见→直达) 否则 WP5 前沿探索(目标未感知→探索)
        gpath = far_planner_path(seen_cum, grid, meta, (x, y), (gx, gy))
        if gpath is not None and len(gpath) >= 2:
            committed = None
            wp = next_waypoint_from_path(gpath, (x, y))
            path, inf = sim.plan(x, y, yaw, float(wp[0]), float(wp[1]), occ_vis, two_way)
        else:
            # 前沿探索(承诺制)
            if committed is None or math.hypot(committed[0] - x, committed[1] - y) < ARRIVE:
                cands = wp5_frontiers(seen_cum, grid, meta, x, y, yaw, hx, hy)
                committed = None
                path = None
                if cands:
                    for c in cands:
                        if not c["reach"]:
                            continue
                        p, inf = sim.plan(x, y, yaw, c["cx"], c["cy"], occ_vis, two_way)
                        if p is not None:
                            committed = (c["cx"], c["cy"]); path = p; break
                if path is None:
                    reason = "no_frontier" if not cands else "stuck"; break
            else:
                path, inf = sim.plan(x, y, yaw, committed[0], committed[1], occ_vis, two_way)
                if path is None:
                    committed = None; continue
        if path is None:
            reason = "no_path"; break
        anim[-1]["path"] = path
        best_paths.append(path)
        if len(path) >= 2:
            yaw = math.atan2(path[-1][1] - path[-2][1], path[-1][0] - path[-2][0])
        x, y = path[-1]
        frames.append((x, y))
    anim.append(dict(pos=(x, y), yaw=yaw, seen=seen_cum.copy(), path=None))
    final_dis = math.hypot(gx - x, gy - y)
    fr = np.array(frames)
    travel = sum(math.hypot(fr[i+1][0]-fr[i][0], fr[i+1][1]-fr[i][1]) for i in range(len(fr)-1))
    found = goal_seen_frame is not None
    return dict(name=name, reason=reason, final_dis=final_dis, frames=fr,
                best_paths=best_paths, travel=travel, seen=seen_cum, anim=anim,
                waypoints=None, goal_seen_frame=goal_seen_frame, found=found)


# ====================================================================
# 5. 可视化
# ====================================================================
def setup_font():
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
    return plt


def render(plt, sc, results):
    os.makedirs(OUTDIR, exist_ok=True)
    cmap = plt.matplotlib.colors.ListedColormap(["#888888", "#ffffff", "#d62728"])  # 0未知灰 1可通行白 2障碍红
    for name, r in results.items():
        d = sc[name]; meta = d["meta"]
        rgrid = np.where(r["seen"], meta["grid"], 0)
        fig, ax = plt.subplots(figsize=(9, 8))
        ax.imshow(rgrid, origin="lower", cmap=cmap, vmin=0, vmax=2,
                  extent=[meta["xmin"], meta["xmax"], meta["ymin"], meta["ymax"]], aspect="equal")
        fr = r["frames"]
        ax.plot(fr[:, 0], fr[:, 1], "b-o", lw=1.6, ms=3, label="行驶轨迹", zorder=6)
        for bp in r["best_paths"]:
            ax.plot(bp[:, 0], bp[:, 1], "orange", lw=1.0, alpha=0.6, zorder=5)
        ax.scatter(d["start"][0], d["start"][1], c="cyan", s=120, marker="o", edgecolors="k", label="起点", zorder=7)
        if d.get("waypoints"):
            wp = np.array(d["waypoints"])
            ax.scatter(wp[:, 0], wp[:, 1], c="lime", s=110, marker="D", edgecolors="k", label="waypoint", zorder=7)
        gsf = r.get("goal_seen_frame")
        if gsf is not None:
            ax.scatter(d["goal"][0], d["goal"][1], c="gold", s=200, marker="*", edgecolors="k", label="目标(已感知)", zorder=7)
        else:
            ax.scatter(d["goal"][0], d["goal"][1], facecolors="none", edgecolors="gray", s=200, marker="o", label="目标(未感知)", zorder=7)
        if r.get("found") is not None:
            ok = bool(r["found"])
            status = "FOUND 找到目标" if ok else "FAIL 未找到"
        else:
            ok = r["reason"] == "reached"
            status = "PASS 到达" if ok else "FAIL 无路/死胡同"
        gs_str = f"  目标第{gsf}帧被感知" if gsf is not None else "  目标未被感知"
        ax.set_title(f"{name}  {d['desc']}\n{status}  终点距目标 {r['final_dis']:.2f}m  行程 {r['travel']:.2f}m{gs_str}")
        ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)")
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        fig.savefig(f"{OUTDIR}/{name}.png", dpi=110)
        plt.close(fig)

    # 汇总图(自适应布局)
    n = len(results)
    cols = 3 if n >= 5 else 2
    rows = int(math.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.5 * cols, 5 * rows))
    axes = np.atleast_1d(axes).ravel()
    for ax, (name, r) in zip(axes, results.items()):
        d = sc[name]; meta = d["meta"]
        rgrid = np.where(r["seen"], meta["grid"], 0)
        ax.imshow(rgrid, origin="lower", cmap=cmap, vmin=0, vmax=2,
                  extent=[meta["xmin"], meta["xmax"], meta["ymin"], meta["ymax"]], aspect="equal")
        fr = r["frames"]
        ax.plot(fr[:, 0], fr[:, 1], "b-o", lw=1.5, ms=2.5, zorder=6)
        for bp in r["best_paths"]:
            ax.plot(bp[:, 0], bp[:, 1], "orange", lw=0.9, alpha=0.5, zorder=5)
        ax.scatter(d["start"][0], d["start"][1], c="cyan", s=80, marker="o", edgecolors="k", zorder=7)
        if d.get("waypoints"):
            wp = np.array(d["waypoints"])
            ax.scatter(wp[:, 0], wp[:, 1], c="lime", s=90, marker="D", edgecolors="k", zorder=7)
        gsf = r.get("goal_seen_frame")
        if gsf is not None:
            ax.scatter(d["goal"][0], d["goal"][1], c="gold", s=140, marker="*", edgecolors="k", zorder=7)
        else:
            ax.scatter(d["goal"][0], d["goal"][1], facecolors="none", edgecolors="gray", s=140, marker="o", zorder=7)
        if r.get("found") is not None:
            ok = bool(r["found"])
            tag = "FOUND" if ok else "FAIL"
        else:
            ok = r["reason"] == "reached"
            tag = "PASS" if ok else "FAIL"
        gs_str = f" 目标第{gsf}帧感知" if gsf is not None else " 目标未感知"
        ax.set_title(f"{name}  {d['desc']}\n{tag}  终点距目标 {r['final_dis']:.2f}m{gs_str}")
        ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)")
    for ax in axes[n:]:
        ax.axis("off")
    fig.suptitle("自拟 ~1m 宽道路  localPlanner 导航效果验证 (忠实复刻)", fontsize=15, fontweight="bold")
    fig.tight_layout()
    fig.savefig(f"{OUTDIR}/overview.png", dpi=110)
    plt.close(fig)


def render_animation(plt, sc, results):
    """逐帧感知生长动画: 累计"已见"地图 + 4m 感知扇形 + 当前路径 + 机器人, 存 GIF"""
    import matplotlib.patches as mpatches
    from matplotlib.animation import FuncAnimation, PillowWriter
    os.makedirs(OUTDIR, exist_ok=True)
    cmap = plt.matplotlib.colors.ListedColormap(["#888888", "#ffffff", "#d62728"])
    for name, r in results.items():
        d = sc[name]; meta = d["meta"]
        anim = r["anim"]
        fig, ax = plt.subplots(figsize=(8, 7))

        def update(i):
            ax.clear()
            fr = anim[i]
            rgrid = np.where(fr["seen"], meta["grid"], 0)
            ax.imshow(rgrid, origin="lower", cmap=cmap, vmin=0, vmax=2,
                      extent=[meta["xmin"], meta["xmax"], meta["ymin"], meta["ymax"]], aspect="equal")
            x, y = fr["pos"]
            ax.add_patch(mpatches.Circle((x, y), PERCEPTION_RANGE, fill=False,
                                         ec="green", lw=1.2, ls="--", alpha=0.9))
            # 感知视场: 车头前方 220° 扇形(半径 4m)
            yaw_deg = math.degrees(fr["yaw"])
            fov_half = FOV_DEG / 2.0
            ax.add_patch(mpatches.Wedge((x, y), PERCEPTION_RANGE, yaw_deg - fov_half,
                                        yaw_deg + fov_half, fill=True, fc="gold",
                                        alpha=0.12, ec="gold", lw=0.8))
            pts = np.array([a["pos"] for a in anim[:i + 1]])
            ax.plot(pts[:, 0], pts[:, 1], "b-o", lw=1.5, ms=3, zorder=6)
            if fr["path"] is not None:
                ax.plot(fr["path"][:, 0], fr["path"][:, 1], "orange", lw=2.0, zorder=5)
            ax.scatter(d["start"][0], d["start"][1], c="cyan", s=100, marker="o", edgecolors="k", zorder=7)
            if d.get("waypoints"):
                wp = np.array(d["waypoints"])
                ax.scatter(wp[:, 0], wp[:, 1], c="lime", s=100, marker="D", edgecolors="k", zorder=7)
            gsf = r.get("goal_seen_frame")
            if gsf is not None and i >= gsf:
                ax.scatter(d["goal"][0], d["goal"][1], c="gold", s=180, marker="*", edgecolors="k", zorder=7)
            ax.scatter(x, y, c="red", s=130, marker="s", edgecolors="k", zorder=8)
            status = ""
            if gsf is not None:
                status = f"  目标第{gsf}帧被感知" if i >= gsf else f"  目标隐藏(第{gsf}帧揭示)"
            ax.set_title(f"{name}  感知生长  step {i}/{len(anim)-1}{status}")
            ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)")
            ax.set_xlim(meta["xmin"], meta["xmax"]); ax.set_ylim(meta["ymin"], meta["ymax"])

        step = max(1, len(anim) // 200)          # 长动画抽帧, GIF 帧数 ≤ ~200
        frame_idx = list(range(0, len(anim), step))
        ani = FuncAnimation(fig, update, frames=frame_idx, interval=400, repeat=True)
        ani.save(f"{OUTDIR}/{name}_anim.gif", writer=PillowWriter(fps=3), dpi=90)
        plt.close(fig)
        print(f"  动画 {name}_anim.gif 已生成 (抽样 {len(frame_idx)}/{len(anim)} 帧)", flush=True)


FINDINGS = """
关键结论(忠实复刻 localPlanner.cpp + path_generator.m + Go2 感知模型):
0. 感知: 车头前方 220° 扇形(±110°)、半径 4m, 射线打到障碍即停; 每次移动后更新并累计"已见"。
   已见格 = 白(可通行)/红(障碍), 未见格 = 灰(未知)。
   隐藏目标仅当落在该 4m 前方扇形内且被射线扫到才"揭示"。
1. 有效净空 = pathScale * searchRadius ≈ 0.75*0.55 = 0.41m; 无解时 pathScale 缩到 0.5,
   净空降到 ≈0.28m。故 1m 宽道路只能容纳"直行/缓弯/小绕障", 路径中轴需距墙 >~0.41m。
2. 直道/斜向循迹/0.7m 缺口绕障: 局部规划器可独立完成(见 PASS 场景)。
3. 90° 直角弯(对角单目标): 2m/360° 感知下贪心切角贴死内墙 => FAIL; 改用 4m 前方 220° 扇形
   感知后 S4 可单目标转过去 => 说明更长的感知前瞻(4m→planner 3m 视距)让局部规划器能
   提前看到拐角、避免过冲撞墙。1m 走廊的 90° 弯仍不稳(依赖感知前瞻与 arrival 角), 走廊
   放宽到 ≥1.4m 最稳。
4. <0.6m 窄缺口: 净空不够, 正确判"无路"(死胡同同款失败机制)。
5. 感知限制: 局部规划器只看 4m 前方扇形内可见障碍(未知按可通行处理), planner 路径
   超出感知的部分按"无碍"处理——这正是真实系统里 localPlanner 把 unknown 当 free 的行为。
6. 复杂地图(S6, 1.6m 走廊): 3 个 90° 弯 + 3 条支路(2 死胡同+1 环路), waypoint 接力全程走通。
   目标隐藏: 只有当 4m 前方扇形感知射线真正扫到目标格时目标才"揭示"(否则图上空心圈),
   模拟了"边探索边找目标"的语义。
7. 前沿探索(S7, 街区 2×3 网格, 无 waypoint): WP5 逻辑——前沿提取 → BFS 聚类(≥3格) →
   邻近簇合并(<1m) → 可达性 BFS → 评分 info/(1+0.5·path+0.3·ret+0.1·risk+0.5·turn)
   选最优观察点, turn=转向角(弧度), 感知到隐藏目标即 FOUND。
8. 20×20 大场景(S8): 树状(无环路)主干+鱼骨支路+岔路+死胡同, 开 twoWayDrive(允许倒车)后
   WP5 探索能进死胡同再倒车出来, 找到藏在死胡同末端的目标。说明: 带死胡同的探索必须开倒车,
   否则 localPlanner(前向)进死胡同后出不来。
9. far_planner 复刻(S9, 立柱房间): 忠实复刻全局层——障碍角点(2×2块凸/凹角)→可见性图
   (节点=角点+odom+goal, 边=视线穿越已感知自由空间)→Dijkstra→回溯取下一航点, 再交 localPlanner。
   全图规划路径绕柱正确, 增量模式下目标未感知时回退 WP5 前沿探索。
"""


def main():
    t0 = time.time()
    T = load_templates()
    print(f"路径模板加载完成 ({time.time()-t0:.1f}s)", flush=True)
    sim = LocalPlannerSim(T)
    sc = scenarios()
    results = {}
    print("=" * 72)
    print(f"自拟 ~1m 宽道路 + localPlanner 忠实复刻 (感知 {PERCEPTION_RANGE:.0f}m 前方{FOV_DEG:.0f}°扇形)")
    print("=" * 72)
    for name, d in sc.items():
        if d.get("planner") == "far":
            r = navigate_far(name, d, sim)
        elif d.get("explore"):
            r = explore(name, d, sim)
        else:
            r = navigate(name, d, sim)
        r["unknown_frac"] = float((~r["seen"]).mean())
        results[name] = r
        is_exp = d.get("explore", False)
        ok = r.get("found", False) if is_exp else (r["reason"] == "reached")
        print(f"\n[{name}] {d['desc']}")
        if is_exp:
            print(f"  起点 ({d['start'][0]:.1f},{d['start'][1]:.1f})  隐藏目标 ({d['goal'][0]:.1f},{d['goal'][1]:.1f})  (无 waypoint)")
            print(f"  结果: {'FOUND 找到目标' if ok else 'FAIL 未找到'}  (reason={r['reason']})")
        else:
            wp_str = " -> ".join(f"({w[0]:.1f},{w[1]:.1f})" for w in d["waypoints"]) if d.get("waypoints") else ""
            if wp_str:
                print(f"  起点 ({d['start'][0]:.1f},{d['start'][1]:.1f}) -> [waypoint {wp_str}] -> 目标 ({d['goal'][0]:.1f},{d['goal'][1]:.1f})")
            else:
                print(f"  起点 ({d['start'][0]:.1f},{d['start'][1]:.1f}) -> 目标 ({d['goal'][0]:.1f},{d['goal'][1]:.1f})")
            print(f"  结果: {'PASS 到达' if ok else 'FAIL'}  (reason={r['reason']})")
        print(f"  终点 ({r['frames'][-1][0]:.2f},{r['frames'][-1][1]:.2f})  距目标 {r['final_dis']:.2f}m  行程 {r['travel']:.2f}m  帧数 {len(r['frames'])-1}")
        gsf = r.get("goal_seen_frame")
        gs_str = f"目标第{gsf}帧被感知" if gsf is not None else "目标未被感知(仍隐藏)"
        print(f"  感知: 已见 {100*(1-r['unknown_frac']):.1f}%  未知(灰) {100*r['unknown_frac']:.1f}%  {gs_str}")

    print("\n" + "=" * 72)
    print("汇总")
    print("=" * 72)
    for name, r in results.items():
        is_exp = sc[name].get("explore", False)
        ok = r.get("found", False) if is_exp else (r["reason"] == "reached")
        tag = "FOUND" if (is_exp and ok) else ("PASS" if ok else "FAIL")
        print(f"  {name:16s} {tag:6s}  终点距目标 {r['final_dis']:.2f}m  未知 {100*r['unknown_frac']:.1f}%")
    print(FINDINGS)
    plt = setup_font()
    render(plt, sc, results)
    print("\n生成感知生长动画 ...", flush=True)
    render_animation(plt, sc, results)
    print(f"图与动画已保存到 {OUTDIR}/  总耗时 {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
