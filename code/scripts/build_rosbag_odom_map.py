#!/usr/bin/env python3
"""
对新增 rosbag 离线构建里程计点云地图 (transform_sensors 变换 + /utlidar/robot_odom 累积),
并渲染 RViz 风格的俯视图 + 斜视图。

用法:
  python3 scripts/build_rosbag_odom_map.py <bag目录> <输出目录>
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from reconstruct_maps import read_bag_clouds, read_bag_odom, build_raw_odom_map
import render_pointcloud_maps as R

MAXPTS = 250_000
CLIP = 60.0


def main():
    bag_dir = sys.argv[1]
    outdir = sys.argv[2]
    os.makedirs(outdir, exist_ok=True)

    print("reading bag clouds ...", flush=True)
    frames = read_bag_clouds(bag_dir)
    print(f"cloud frames: {len(frames)}", flush=True)

    print("reading odom ...", flush=True)
    odom = read_bag_odom(bag_dir)
    print(f"odom msgs: {len(odom)}", flush=True)

    print("building odom map ...", flush=True)
    world = build_raw_odom_map(frames, odom)
    traj = np.array([o[1] for o in odom]) - np.array([odom[0][1]] * len(odom))
    print(f"world points: {len(world)}", flush=True)

    # 渲染
    xyz = world[:, :3]
    xyz = R.clip_outliers(xyz, CLIP)
    if len(xyz) > MAXPTS:
        idx = np.random.choice(len(xyz), MAXPTS, replace=False)
        xyz = xyz[idx]
    traj = R.clip_outliers(traj, CLIP)
    z = xyz[:, 2]
    zmin, zmax = R.robust_lims(z)

    R.render_map(xyz, z, os.path.join(outdir, "odom_map"),
                 "odometry map (transform_sensors + robot_odom)",
                 "height z (m)", zmin, zmax, traj=traj)
    print(f"ALL DONE -> {outdir}", flush=True)


if __name__ == "__main__":
    main()
