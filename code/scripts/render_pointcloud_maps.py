#!/usr/bin/env python3
"""
渲染累积好的三套点云地图, 每套输出俯视图 + 斜视图 (共 6 张 PNG).

输入: 由 collect_pointcloud_maps.py 生成的 .npy (Nx4: x,y,z,intensity)
用法:
  python3 render_pointcloud_maps.py <数据目录> <输出目录> [max_points]

颜色规则:
  raw_cloud(transform_sensors)   -> 按高度 z 着色
  registered_scan(point_lio)     -> 按高度 z 着色
  terrain_map(terrain_analysis)  -> 按 intensity(相对地面高度) 着色
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 中文字体
matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

MAXPTS = 250_000
CMAP = "turbo"
CLIP = 40.0   # 去掉发散帧(静止漂移/初始化瞬态)的离群点: |x|,|y|,|z| > CLIP 丢弃


def robust_lims(v, lo=2, hi=98):
    v = v[np.isfinite(v)]
    if len(v) == 0:
        return 0.0, 1.0
    return float(np.percentile(v, lo)), float(np.percentile(v, hi))


def sample(arr, n=MAXPTS):
    if len(arr) > n:
        idx = np.random.choice(len(arr), n, replace=False)
        arr = arr[idx]
    return arr


def clip_outliers(arr, clip=CLIP):
    """按绝对坐标裁剪, 丢弃发散/瞬态帧产生的远点"""
    m = (np.abs(arr[:, 0]) <= clip) & (np.abs(arr[:, 1]) <= clip) & (np.abs(arr[:, 2]) <= clip)
    return arr[m]


def oblique_projection(xyz, azim_deg=45.0, elev_deg=35.0):
    """正交投影到斜视相机平面, 返回 (sx, sy, depth)"""
    az = np.deg2rad(azim_deg)
    el = np.deg2rad(elev_deg)
    eye = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(eye, up)
    right = right / np.linalg.norm(right)
    true_up = np.cross(right, eye)
    true_up = true_up / np.linalg.norm(true_up)
    sx = xyz @ right
    sy = xyz @ true_up
    depth = xyz @ eye
    return sx, sy, depth


def render_map(xyz, color, out_prefix, title, color_label, vmin, vmax, traj=None):
    xyz = sample(xyz)
    color = sample(color) if color is not None else None
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    c = color if color is not None else z

    # ---- 俯视图 (X-Y) ----
    fig, ax = plt.subplots(figsize=(9, 9))
    sc = ax.scatter(x, y, c=c, s=1.2, cmap=CMAP, vmin=vmin, vmax=vmax,
                    linewidths=0, rasterized=True)
    if traj is not None and len(traj) > 1:
        ax.plot(traj[:, 0], traj[:, 1], "k-", lw=1.2, alpha=0.55)
        ax.scatter(traj[0, 0], traj[0, 1], c="lime", s=80, marker="o", zorder=5)
        ax.scatter(traj[-1, 0], traj[-1, 1], c="red", s=80, marker="o", zorder=5)
    ax.set_aspect("equal")
    ax.grid(True, color="0.85", lw=0.5)
    ax.set_title(f"{title} — 俯视图 (Top View)", fontsize=12)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label(color_label)
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_top.png", dpi=130)
    plt.close(fig)

    # ---- 斜视图 (正交投影, 相机方位 45° 俯仰 35°) ----
    sx, sy, depth = oblique_projection(xyz)
    # 远处点画在下面: 按 depth 排序让近处覆盖远处
    order = np.argsort(depth)[::-1]
    sx, sy, c = sx[order], sy[order], c[order]
    fig, ax = plt.subplots(figsize=(11, 8))
    sc = ax.scatter(sx, sy, c=c, s=1.0, cmap=CMAP, vmin=vmin, vmax=vmax,
                    linewidths=0, rasterized=True)
    if traj is not None and len(traj) > 1:
        tsx, tsy, _ = oblique_projection(traj)
        ax.plot(tsx, tsy, "k-", lw=1.1, alpha=0.5)
    ax.set_aspect("equal")
    ax.grid(True, color="0.85", lw=0.5)
    ax.set_title(f"{title} — 斜视图 (Oblique View, az=45°, el=35°)", fontsize=12)
    ax.set_xlabel("camera x (m)"); ax.set_ylabel("camera y (m)")
    cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label(color_label)
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_oblique.png", dpi=130)
    plt.close(fig)


def main():
    indir = sys.argv[1]
    outdir = sys.argv[2]
    if len(sys.argv) > 3:
        globals()["MAXPTS"] = int(sys.argv[3])
    os.makedirs(outdir, exist_ok=True)

    def load(name):
        p = os.path.join(indir, name)
        if os.path.exists(p):
            return np.load(p)
        return None

    raw = load("raw_cloud.npy")
    pio = load("registered_scan.npy")
    ter = load("terrain_map.npy")
    traj_pio = load("traj_pio.npy")
    traj_odom = load("traj_odom.npy")

    # 裁剪轨迹离群点(用于叠加显示)
    if traj_pio is not None and len(traj_pio):
        traj_pio = clip_outliers(traj_pio, CLIP)
    if traj_odom is not None and len(traj_odom):
        traj_odom = clip_outliers(traj_odom, CLIP)

    # 高度着色的公共色标 (取三张地图的 z 合并分位)
    zs = []
    for a in (raw, pio, ter):
        if a is not None and len(a):
            zs.append(clip_outliers(a, CLIP)[:, 2])
    if zs:
        z_all = np.concatenate(zs)
        zmin, zmax = robust_lims(z_all)
    else:
        zmin, zmax = 0, 1

    # terrain 用 intensity(相对地面高度) 着色
    tmin, tmax = 0.0, 1.0

    if raw is not None and len(raw):
        raw = clip_outliers(raw, CLIP)
        render_map(raw[:, :3], raw[:, 2], os.path.join(outdir, "transform_sensors"),
                   "transform_sensors /utlidar/transformed_cloud (raw+odom)",
                   "height z (m)", zmin, zmax, traj=traj_odom)
        print("rendered transform_sensors", flush=True)

    if pio is not None and len(pio):
        pio = clip_outliers(pio, CLIP)
        render_map(pio[:, :3], pio[:, 2], os.path.join(outdir, "point_lio"),
                   "Point-LIO /registered_scan",
                   "height z (m)", zmin, zmax, traj=traj_pio)
        print("rendered point_lio", flush=True)

    if ter is not None and len(ter):
        ter = clip_outliers(ter, CLIP)
        it = np.clip(ter[:, 3], tmin, tmax)
        render_map(ter[:, :3], it, os.path.join(outdir, "terrain_analysis"),
                   "terrain_analysis /terrain_map",
                   "relative ground height (m)", tmin, tmax, traj=traj_pio)
        print("rendered terrain_analysis", flush=True)

    print(f"ALL DONE -> {outdir}", flush=True)


if __name__ == "__main__":
    main()
