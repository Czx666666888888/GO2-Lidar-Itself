#!/usr/bin/env python3
"""从静止 rosbag 标定 IMU bias, 生成 imu_calib_data.yaml

读取静止 bag 的 /utlidar/imu, 按 transform_everything 的坐标变换逻辑,
算出 body 坐标系的 gyro/accel 零偏, 写入 transform_everything 期望的
imu_calib_data.yaml (home_path/Desktop/imu_calib_data.yaml)。

用法:
  python3 calibrate_imu_from_bag.py <静止bag路径> [输出yaml路径]
"""
import math
import os
import sys

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

THETA = 15.1 * math.pi / 180.0  # transform_everything 里 imu 绕 y 轴旋转角


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else \
        "/home/lch/dog/rosbags/go2_controlled_20260824/controlled_stationary_20260824_01"
    out_yaml = sys.argv[2] if len(sys.argv) > 2 else \
        "/home/lch/dog/tools/home/Desktop/imu_calib_data.yaml"

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id="sqlite3"),
        ConverterOptions("", ""))

    gx, gy, gz = [], [], []   # 原始角速度
    ax, ay, az = [], [], []   # 原始线加速度
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/utlidar/imu":
            m = deserialize_message(data, Imu)
            gx.append(m.angular_velocity.x)
            gy.append(m.angular_velocity.y)
            gz.append(m.angular_velocity.z)
            ax.append(m.linear_acceleration.x)
            ay.append(m.linear_acceleration.y)
            az.append(m.linear_acceleration.z)

    gx, gy, gz = np.array(gx), np.array(gy), np.array(gz)
    ax, ay, az = np.array(ax), np.array(ay), np.array(az)
    print(f"读取 IMU {len(gx)} 条")

    # 原始均值
    gxm, gym, gzm = gx.mean(), gy.mean(), gz.mean()
    axm, aym, azm = ax.mean(), ay.mean(), az.mean()
    print(f"原始角速度均值: ({gxm:.5f}, {gym:.5f}, {gzm:.5f}) rad/s")
    print(f"原始加速度均值: ({axm:.4f}, {aym:.4f}, {azm:.4f}) m/s²")

    # 应用 transform_everything 的变换:
    # 角速度: x→x, y→-y, z→-z, 绕y轴转15.1°
    # 加速度: 同角速度
    c, s = math.cos(THETA), math.sin(THETA)

    def rot(x, y, z):
        # y 取反, z 取反, 然后绕 y 轴旋转 theta
        y_ = -y
        z_ = -z
        x2 = c * x - s * z_
        y2 = y_
        z2 = s * x + c * z_
        return x2, y2, z2

    gx2, gy2, gz2 = rot(gxm, gym, gzm)
    ax2, ay2, az2 = rot(axm, aym, azm)

    # bias = 变换后静止均值 (角速度真值0, 加速度水平真值0)
    ang_bias_x, ang_bias_y, ang_bias_z = gx2, gy2, gz2
    acc_bias_x, acc_bias_y = ax2, ay2
    acc_bias_z = az2 - 9.8  # 去掉重力(z 向下为 -9.8, 这里 acc_z 是 z 方向分量)

    print(f"\n标定结果 (body 坐标系):")
    print(f"  ang_bias: ({ang_bias_x:.5f}, {ang_bias_y:.5f}, {ang_bias_z:.5f}) rad/s")
    print(f"  acc_bias: ({acc_bias_x:.4f}, {acc_bias_y:.4f}, {acc_bias_z:.4f}) m/s²")

    # 生成 yaml (保留默认投影角参数)
    os.makedirs(os.path.dirname(out_yaml), exist_ok=True)
    with open(out_yaml, "w") as f:
        f.write(f"acc_bias_x: {acc_bias_x:.8f}\n")
        f.write(f"acc_bias_y: {acc_bias_y:.8f}\n")
        f.write(f"acc_bias_z: {acc_bias_z:.8f}\n")
        f.write(f"ang_bias_x: {ang_bias_x:.8f}\n")
        f.write(f"ang_bias_y: {ang_bias_y:.8f}\n")
        f.write(f"ang_bias_z: {ang_bias_z:.8f}\n")
        f.write("ang_z2x_proj: 0.15\n")
        f.write("ang_z2y_proj: -0.28\n")
    print(f"\n已生成 {out_yaml}")


if __name__ == "__main__":
    main()
