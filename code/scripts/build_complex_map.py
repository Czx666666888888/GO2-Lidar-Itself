#!/usr/bin/env python3
"""
从 go2_data 原始导出构建点云地图并渲染俯视/斜视图。

输入目录 (如 go2_data/complex_20260826_173134):
  - lidar.npz    : points(N,3) / timestamps(N,) / intensity(N,) —— body 帧点云
  - odom.csv     : sec,nanosec,px,py,pz,vx,vy,vz,yaw_speed —— 位置(里程计)
  - imu.csv      : sec,nanosec,qx,qy,qz,qw,gyro_*,accel_* —— 姿态四元数

方法(里程计建图, 无回环):
  相对位置(odom) + 相对偏航(imu 四元数 yaw) 把 body 帧点云变换到以起点为原点的地图系并累积。

用法:
  python3 scripts/build_complex_map.py <数据目录> <输出目录>
"""
import sys, os, csv
import numpy as np

MAXPTS = 250_000
CLIP = 50.0


def load_csv(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    return np.array([[float(x) for x in r] for r in rows[1:]])


def quat_yaw(qx, qy, qz, qw):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def build_map(data_dir):
    d = np.load(os.path.join(data_dir, "lidar.npz"))
    pts = d["points"].astype(np.float64)
    ts = d["timestamps"]
    it = d["intensity"].astype(np.float64)

    od = load_csv(os.path.join(data_dir, "odom.csv"))
    od_t = od[:, 0] + od[:, 1] * 1e-9
    pos = od[:, 2:5]

    imu = load_csv(os.path.join(data_dir, "imu.csv"))
    imu_t = imu[:, 0] + imu[:, 1] * 1e-9
    yaw = np.unwrap(quat_yaw(imu[:, 2], imu[:, 3], imu[:, 4], imu[:, 5]))

    pos_rel = pos - pos[0]
    yaw_rel = yaw - yaw[0]

    yaw_i = np.interp(ts, imu_t, yaw_rel)
    px = np.interp(ts, od_t, pos_rel[:, 0])
    py = np.interp(ts, od_t, pos_rel[:, 1])
    pz = np.interp(ts, od_t, pos_rel[:, 2])

    cy, sy = np.cos(yaw_i), np.sin(yaw_i)
    wx = cy * pts[:, 0] - sy * pts[:, 1] + px
    wy = sy * pts[:, 0] + cy * pts[:, 1] + py
    wz = pts[:, 2] + pz

    world = np.stack([wx, wy, wz, it], axis=1)
    return world, pos_rel


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else "go2_data/complex_20260826_173134"
    outdir = sys.argv[2] if len(sys.argv) > 2 else "outputs/pointcloud_maps/complex_20260826_173134"
    os.makedirs(outdir, exist_ok=True)

    print("building map ...", flush=True)
    world, traj = build_map(data_dir)
    print(f"world points: {len(world)}", flush=True)

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
    import render_pointcloud_maps as R

    xyz = world[:, :3]
    xyz = R.clip_outliers(xyz, CLIP)
    z = xyz[:, 2]
    zmin, zmax = R.robust_lims(z)
    if len(xyz) > MAXPTS:
        idx = np.random.choice(len(xyz), MAXPTS, replace=False)
        xyz = xyz[idx]
    traj = R.clip_outliers(traj, CLIP)

    R.render_map(xyz, xyz[:, 2], os.path.join(outdir, "complex_map"),
                 "complex_20260826_173134 点云地图 (里程计建图)",
                 "height z (m)", zmin, zmax, traj=traj)
    print(f"ALL DONE -> {outdir}", flush=True)


if __name__ == "__main__":
    main()
