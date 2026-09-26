#!/usr/bin/env python3
"""从方形 bag 标定投影角 ang_z2x_proj / ang_z2y_proj

原理: 方形 bag 有 4 次 90° 转弯(绕 z 轴纯 yaw 旋转)。
纯 yaw 旋转时, body 坐标系的 x/y 轴真实角速度应为 0, 只有 z 轴(yaw)非零。
若 IMU 安装有倾角, z 轴角速度会"投影"到 x/y 轴 -> 这就是投影角要补偿的。

方法: 在转弯段(|yaw_rate|大), 变换后的角速度 x2/y2 应正比于 z2:
  ang_z2x_proj = -mean(x2) / mean(z2)
  ang_z2y_proj = -mean(y2) / mean(z2)

用法: python3 calibrate_proj_from_bag.py [方形bag路径]
"""
import math
import sys

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

THETA = 15.1 * math.pi / 180.0
# 之前标定的 bias
ANG_BIAS = (-0.00070102, 0.00504276, 0.00366538)


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else \
        "/home/lch/dog/rosbags/go2_controlled_20260824/controlled_square_2mx2m_ccw_20260824_01"

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="sqlite3"),
                ConverterOptions("", ""))

    imu_t, imu_ang = [], []
    odom_t, odom_yaw = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/utlidar/imu":
            m = deserialize_message(data, Imu)
            imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            imu_ang.append([m.angular_velocity.x, m.angular_velocity.y,
                            m.angular_velocity.z])
        elif topic == "/utlidar/robot_odom":
            m = deserialize_message(data, Odometry)
            odom_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            odom_yaw.append(yaw_from_quat(m.pose.pose.orientation))

    imu_t = np.array(imu_t); imu_ang = np.array(imu_ang)
    odom_t = np.array(odom_t); odom_yaw = np.unwrap(np.array(odom_yaw))
    print(f"IMU {len(imu_t)} 条, odom {len(odom_t)} 条")

    # 对每个 IMU 时间戳, 插值 odom yaw, 算 yaw_rate
    yaw_at_imu = np.interp(imu_t, odom_t, odom_yaw)
    yaw_rate = np.gradient(yaw_at_imu, imu_t)

    # 变换 IMU 角速度(theta + bias)
    c, s = math.cos(THETA), math.sin(THETA)
    def transform(gx, gy, gz):
        y_ = -gy; z_ = -gz
        x2 = c * gx - s * z_ - ANG_BIAS[0]
        y2 = y_ - ANG_BIAS[1]
        z2 = s * gx + c * z_ - ANG_BIAS[2]
        return x2, y2, z2

    x2, y2, z2 = transform(imu_ang[:,0], imu_ang[:,1], imu_ang[:,2])

    # 转弯段: |yaw_rate| > 0.5 rad/s
    turn_mask = np.abs(yaw_rate) > 0.5
    print(f"转弯段样本: {turn_mask.sum()} / {len(turn_mask)} "
          f"({turn_mask.sum()/len(turn_mask)*100:.1f}%)")
    print(f"转弯段 yaw 总变化: {np.abs(np.diff(yaw_at_imu[turn_mask])).sum():.1f} rad "
          f"= {np.abs(np.diff(yaw_at_imu[turn_mask])).sum()*180/math.pi:.1f}°")

    # 投影角 = -mean(x2)/mean(z2)
    zx = np.polyfit(z2[turn_mask], x2[turn_mask], 1)[0]
    zy = np.polyfit(z2[turn_mask], y2[turn_mask], 1)[0]
    print(f"\n转弯段拟合: x2 vs z2 斜率 = {zx:.4f}, y2 vs z2 斜率 = {zy:.4f}")

    # 投影角是补偿项, transform 里 x2 += proj * z2, 所以 proj = -斜率
    ang_z2x_proj = -zx
    ang_z2y_proj = -zy
    print(f"\n标定结果:")
    print(f"  ang_z2x_proj = {ang_z2x_proj:.4f}  (原默认 0.15)")
    print(f"  ang_z2y_proj = {ang_z2y_proj:.4f}  (原默认 -0.28)")

    # 验证: 纯 yaw 旋转段, x2/y2 残余
    print(f"\n验证(转弯段残余角速度):")
    print(f"  x2 均值: {x2[turn_mask].mean():.5f} rad/s (应为0)")
    print(f"  y2 均值: {y2[turn_mask].mean():.5f} rad/s (应为0)")
    print(f"  z2 均值: {z2[turn_mask].mean():.5f} rad/s (yaw 变化率)")


if __name__ == "__main__":
    main()
