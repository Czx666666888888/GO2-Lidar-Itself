#!/usr/bin/env python3
"""用固件 odom(/utlidar/robot_odom)直接建图, 绕开本机 Point-LIO 发散

复用 reconstruct_maps.py 的固件 odom 累积逻辑:
  /utlidar/cloud + /utlidar/robot_odom -> raw_cloud(odom帧地图) -> terrain_map

用法: python3 build_odom_map.py <bag目录> <输出目录>
"""
import os
import sys

import numpy as np

sys.path.insert(0, "/home/lch/dog/scripts")
from reconstruct_maps import (read_bag_clouds, read_bag_odom,
                              build_raw_odom_map, terrain_from_registered)


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else sys.exit("用法: build_odom_map.py <bag> <outdir>")
    outdir = sys.argv[2] if len(sys.argv) > 2 else "/home/lch/dog/tmp/odom_map"
    os.makedirs(outdir, exist_ok=True)

    print("reading bag clouds ...", flush=True)
    frames = read_bag_clouds(bag)
    print(f"cloud frames: {len(frames)}", flush=True)

    print("reading bag odom ...", flush=True)
    odom = read_bag_odom(bag)
    print(f"odom: {len(odom)}", flush=True)

    print("building raw odom map ...", flush=True)
    raw = build_raw_odom_map(frames, odom)
    np.save(os.path.join(outdir, "raw_cloud.npy"), raw)
    np.save(os.path.join(outdir, "traj_odom.npy"), np.array([o[1] for o in odom]))
    print(f"raw_cloud points: {len(raw)}", flush=True)

    print("building terrain map ...", flush=True)
    ter = terrain_from_registered(raw)
    np.save(os.path.join(outdir, "terrain_map.npy"), ter)
    print(f"terrain points: {len(ter)}", flush=True)

    # 统计
    x, y = raw[:, 0], raw[:, 1]
    print(f"\n=== 固件 odom 建图结果 ===")
    print(f"覆盖范围 X[{x.min():.2f},{x.max():.2f}] Y[{y.min():.2f},{y.max():.2f}]")
    print(f"SAVED -> {outdir}")


if __name__ == "__main__":
    main()
