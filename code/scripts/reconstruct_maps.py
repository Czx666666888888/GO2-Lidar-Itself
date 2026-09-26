#!/usr/bin/env python3
"""
用团队已验证的 Point-LIO 位姿轨迹 + 原始 lidar 点云, 离线重建:
  1) Point-LIO 注册地图  (/registered_scan 等价)
  2) terrain_analysis 地形图 (/terrain_map 等价, intensity=相对地面高度)

因为本机 RoboStack(GCC13/C++17/PCL1.15) 构建的 Point-LIO 会发散,
而团队在 /opt/ros/humble(GCC11/C++14) 上跑出的位姿轨迹是已验证的
(正方形闭环残差 0.265m), 故用该轨迹重建地图。

用法:
  python3 reconstruct_maps.py <bag目录> <pose_csv> <输出目录> [offset]

步骤:
  1. 按序读取 bag 的 /utlidar/cloud 原始点云
  2. 复刻 transform_sensors 的 body2cloud 变换(pitch 2.878rad + z 偏移 0.046825 + 过滤盒)
  3. 复刻 Point-LIO 外参(extrinsic_T, 外参R=单位阵)后, 用团队位姿变换到世界系
  4. 累积得到注册地图; 再按体素算相对地面高度得到地形图
"""
import sys, os
import numpy as np

# transform_sensors 的 body2cloud 变换参数
PITCH = 2.87820258505555555556          # 绕 Y 轴 pitch (rad)
CAM_OFFSET = 0.046825
# 过滤盒 (transform_sensors 会剔除机器人本体的点)
BOX = dict(x=(-0.7, -0.1), y=(-0.3, 0.3), z=(-0.6 - CAM_OFFSET, 0 - CAM_OFFSET))
# Point-LIO 外参 (utlidar.yaml: extrinsic_R=单位阵, extrinsic_T)
EXTRINSIC_T = np.array([0.007698, 0.014655, -0.00667])


def read_bag_clouds(bag_dir):
    """按序读取 /utlidar/cloud -> [(stamp_ns, Nx4 float64)]"""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    frames = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != "/utlidar/cloud":
            continue
        msg = deserialize_message(data, PointCloud2)
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        arr = np.array([(p[0], p[1], p[2], p[3]) for p in pts], dtype=np.float64)
        frames.append((t, arr))
    return frames


def transform_sensors_apply(pts):
    """复刻 transform_everything.cloud_callback: pitch 旋转 + z 偏移 + 过滤盒
    mat = quat2mat(pitch about Y); 应用 points @ mat.T
    => x2 = cy*x + sy*z, z2 = -sy*x + cy*z
    """
    cy, sy = np.cos(PITCH), np.sin(PITCH)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    x2 = cy * x + sy * z
    z2 = -sy * x + cy * z
    y2 = y
    z2 = z2 - CAM_OFFSET
    keep = ~(
        (x2 > BOX["x"][0]) & (x2 < BOX["x"][1]) &
        (y2 > BOX["y"][0]) & (y2 < BOX["y"][1]) &
        (z2 > BOX["z"][0]) & (z2 < BOX["z"][1])
    )
    out = np.hstack([x2[:, None], y2[:, None], z2[:, None], pts[:, 3:4]])
    return out[keep]


def pose_to_mat(p):
    """p = [x,y,z,qx,qy,qz,qw] -> (R(3x3), t(3))"""
    from transforms3d.quaternions import quat2mat
    t = p[:3]
    R = quat2mat([p[6], p[3], p[4], p[5]])  # w,x,y,z
    return R, t


def reconstruct_registered(frames, poses, offset):
    """把每帧点云用对应位姿变换到世界系并累积"""
    acc = []
    n = min(len(poses), len(frames) - offset)
    for i in range(n):
        _, cloud = frames[i + offset]
        body = transform_sensors_apply(cloud)
        R, t = pose_to_mat(poses[i])
        # 外参 lidar->imu (R=单位阵): imu = body + extrinsic_T; 再 pose: world = R@imu + t
        imu = body[:, :3] + EXTRINSIC_T
        world = imu @ R.T + t
        acc.append(np.hstack([world, body[:, 3:4]]))
    return np.vstack(acc)


def terrain_from_registered(reg, voxel=0.05, quantile=0.25):
    """按体素算相对地面高度: intensity = z - 该体素列 ground(z 分位数)"""
    from collections import defaultdict
    vx = np.floor(reg[:, 0] / voxel).astype(np.int64)
    vy = np.floor(reg[:, 1] / voxel).astype(np.int64)
    cols = defaultdict(list)
    for i in range(len(reg)):
        cols[(vx[i], vy[i])].append(reg[i, 2])
    ground = {k: float(np.percentile(np.array(zs), quantile * 100)) for k, zs in cols.items()}
    disz = np.array([reg[i, 2] - ground[(vx[i], vy[i])] for i in range(len(reg))])
    disz = np.clip(disz, 0.0, 1.5)
    return np.hstack([reg[:, :3], disz[:, None]])


def read_bag_odom(bag_dir):
    """按序读取 /utlidar/robot_odom -> [(stamp_ns, [x,y,z], R(3x3))]"""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from transforms3d.quaternions import quat2mat

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    odom = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != "/utlidar/robot_odom":
            continue
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        R = quat2mat([q.w, q.x, q.y, q.z])
        odom.append((t, np.array([p.x, p.y, p.z]), R))
    return odom


def build_raw_odom_map(frames, odom):
    """transform_sensors 输出(body帧) + robot_odom 累积 -> 原始里程计地图(odom帧)"""
    ts = np.array([o[0] for o in odom])
    acc = []
    for stamp, cloud in frames:
        # 最近邻 odom
        i = int(np.argmin(np.abs(ts - stamp)))
        _, pos, R = odom[i]
        body = transform_sensors_apply(cloud)
        world = body[:, :3] @ R.T + pos
        acc.append(np.hstack([world, body[:, 3:4]]))
    return np.vstack(acc)


def main():
    bag_dir = sys.argv[1]
    pose_csv = sys.argv[2]
    outdir = sys.argv[3]
    offset = int(sys.argv[4]) if len(sys.argv) > 4 else 25
    os.makedirs(outdir, exist_ok=True)

    print("reading bag clouds ...", flush=True)
    frames = read_bag_clouds(bag_dir)
    print(f"cloud frames: {len(frames)}", flush=True)

    # 原始里程计地图 (transform_sensors + robot_odom)
    try:
        odom = read_bag_odom(bag_dir)
        raw = build_raw_odom_map(frames, odom)
        np.save(os.path.join(outdir, "raw_cloud.npy"), raw)
        np.save(os.path.join(outdir, "traj_odom.npy"), np.array([o[1] for o in odom]))
        print(f"raw_odom points: {len(raw)}", flush=True)
    except Exception as e:
        print(f"WARN: raw_odom build failed: {e}", flush=True)

    poses = np.loadtxt(pose_csv, delimiter=",")
    if poses.ndim == 1:
        poses = poses.reshape(1, -1)
    print(f"poses: {poses.shape}", flush=True)

    reg = reconstruct_registered(frames, poses, offset)
    print(f"registered points: {len(reg)}", flush=True)
    np.save(os.path.join(outdir, "registered_scan.npy"), reg)

    ter = terrain_from_registered(reg)
    print(f"terrain points: {len(ter)}", flush=True)
    np.save(os.path.join(outdir, "terrain_map.npy"), ter)

    # 位姿轨迹也存一份(用于渲染叠加)
    np.save(os.path.join(outdir, "traj_pio.npy"), poses[:, :3])
    print(f"SAVED -> {outdir}", flush=True)


if __name__ == "__main__":
    main()
