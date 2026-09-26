#!/usr/bin/env python3
"""go2_sensor_recorder_dds 离线冒烟测试
发布模拟 SportModeState(含 IMU+odom) + PointCloud2, 起 DDS 记录节点, 验证保存。
"""
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from unitree_go.msg import SportModeState

OUTDIR = "/home/lch/dog/tmp/recorder_dds_test"


class FakePublisher(Node):
    def __init__(self):
        super().__init__("fake_dds_pub")
        self.sport_pub = self.create_publisher(SportModeState, "lf/sportmodestate", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "/utlidar/cloud", 10)
        self.imu_pub = self.create_publisher(Imu, "/utlidar/imu", 10)

    def run(self, n=5):
        t0 = time.time()
        for i in range(n):
            # 标准 IMU
            imu = Imu()
            imu.header = Header()
            imu.header.frame_id = "utlidar_imu"
            imu.header.stamp.sec = int(t0 + i)
            imu.header.stamp.nanosec = i * 1000
            imu.orientation.w = 1.0
            imu.angular_velocity.x = 0.01 * i
            imu.angular_velocity.y = 0.02 * i
            imu.angular_velocity.z = 0.03 * i
            imu.linear_acceleration.z = 9.8
            self.imu_pub.publish(imu)

            s = SportModeState()
            s.stamp.sec = int(t0 + i)
            s.stamp.nanosec = i * 1000
            s.imu_state.quaternion = [0.0, 0.0, 0.0, 1.0]
            s.imu_state.gyroscope = [0.01 * i, 0.02 * i, 0.03 * i]
            s.imu_state.accelerometer = [0.0, 0.0, 9.8]
            s.imu_state.rpy = [0.1 * i, 0.2 * i, 0.3 * i]
            s.position = [0.1 * i, 0.2 * i, 0.3 * i]
            s.velocity = [0.01 * i, 0.02 * i, 0.0]
            s.yaw_speed = 0.05 * i
            self.sport_pub.publish(s)

            hdr = Header()
            hdr.frame_id = "lidar"
            hdr.stamp.sec = int(t0 + i)
            hdr.stamp.nanosec = i * 1000
            pts = np.array([
                [1.0 + i, 2.0, 3.0, 0.5],
                [4.0, 5.0 + i, 6.0, 0.8],
                [7.0, 8.0, 9.0 + i, 0.2],
            ], dtype=np.float32)
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            self.lidar_pub.publish(point_cloud2.create_cloud(hdr, fields, pts))
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.15)


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    for f in os.listdir(OUTDIR):
        os.remove(os.path.join(OUTDIR, f))

    env = dict(os.environ)
    rec = subprocess.Popen(
        ["ros2", "run", "go2_keyboard_teleop", "go2_sensor_recorder_dds",
         "--ros-args", "-p", f"outdir:={OUTDIR}", "-p", "duration:=8.0"],
        stdout=open(os.path.join(OUTDIR, "recorder.log"), "w"),
        stderr=subprocess.STDOUT, env=env)

    rclpy.init()
    pub = FakePublisher()
    time.sleep(3.0)
    pub.run(n=5)
    time.sleep(2.0)

    try:
        rec.wait(timeout=15)
    except subprocess.TimeoutExpired:
        rec.kill()

    pub.destroy_node()
    rclpy.shutdown()

    print("=== 输出文件 ===")
    for f in sorted(os.listdir(OUTDIR)):
        print(f"  {f}  ({os.path.getsize(os.path.join(OUTDIR, f))}B)")

    ok = True
    imu_path = os.path.join(OUTDIR, "imu.csv")
    odom_path = os.path.join(OUTDIR, "odom.csv")
    lidar_path = os.path.join(OUTDIR, "lidar.npz")

    if os.path.exists(imu_path):
        lines = open(imu_path).read().strip().split("\n")
        print(f"\n=== imu.csv: {len(lines)-1} 条 ===")
        print("  表头:", lines[0][:60], "...")
        print("  首行:", lines[1])
        ok = ok and len(lines) - 1 == 5
    else:
        print("imu.csv 缺失 ✗"); ok = False

    if os.path.exists(odom_path):
        lines = open(odom_path).read().strip().split("\n")
        print(f"\n=== odom.csv: {len(lines)-1} 条 ===")
        print("  首行:", lines[1])
        ok = ok and len(lines) - 1 == 5
    else:
        print("odom.csv 缺失 ✗"); ok = False

    if os.path.exists(lidar_path):
        d = np.load(lidar_path)
        print(f"\n=== lidar.npz: points {d['points'].shape}, intensity {d['intensity'].shape}")
        ok = ok and d["points"].shape == (15, 3)
    else:
        print("lidar.npz 缺失 ✗"); ok = False

    print("\n验证结果:", "通过 ✓" if ok else "失败 ✗")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
