#!/usr/bin/env python3
"""go2_sensor_recorder 离线冒烟测试
发布模拟 IMU + PointCloud2, 起 recorder 记录, 验证保存文件正确。
"""
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

OUTDIR = "/home/lch/dog/tmp/recorder_test"


class FakePublisher(Node):
    def __init__(self):
        super().__init__("fake_publisher")
        self.imu_pub = self.create_publisher(Imu, "imu", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "point_cloud2", 10)

    def run(self, n=5):
        t0 = time.time()
        for i in range(n):
            # IMU
            imu = Imu()
            imu.header = Header()
            imu.header.stamp.sec = int(t0 + i)
            imu.header.stamp.nanosec = i * 1000
            imu.linear_acceleration.x = 0.1 * i
            imu.linear_acceleration.y = 0.2 * i
            imu.linear_acceleration.z = 9.8
            imu.angular_velocity.x = 0.01 * i
            imu.angular_velocity.y = 0.02 * i
            imu.angular_velocity.z = 0.03 * i
            imu.orientation.w = 1.0
            self.imu_pub.publish(imu)

            # PointCloud2 (xyz + intensity)
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
            cloud = point_cloud2.create_cloud(hdr, fields, pts)
            self.lidar_pub.publish(cloud)
            # 驱动 DDS 可靠传输握手/发送
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.15)


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    for f in os.listdir(OUTDIR):
        os.remove(os.path.join(OUTDIR, f))

    env = dict(os.environ)
    rec = subprocess.Popen(
        ["ros2", "run", "go2_keyboard_teleop", "go2_sensor_recorder",
         "--ros-args", "-p", f"outdir:={OUTDIR}", "-p", "duration:=8.0"],
        stdout=open(os.path.join(OUTDIR, "recorder.log"), "w"),
        stderr=subprocess.STDOUT, env=env)

    rclpy.init()
    pub = FakePublisher()
    time.sleep(3.0)          # 等 recorder 就绪 + DDS 可靠传输握手完成
    pub.run(n=5)             # 发布 5 帧 IMU + 点云
    time.sleep(2.0)          # 让订阅收到全部数据

    # recorder 由 duration=8s 定时自动停止并保存
    try:
        rec.wait(timeout=15)
    except subprocess.TimeoutExpired:
        rec.kill()

    pub.destroy_node()
    rclpy.shutdown()

    # 检查输出
    imu_path = os.path.join(OUTDIR, "imu.csv")
    lidar_path = os.path.join(OUTDIR, "lidar.npz")
    print("=== 输出文件 ===")
    for f in sorted(os.listdir(OUTDIR)):
        print(f"  {f}  ({os.path.getsize(os.path.join(OUTDIR, f))}B)")

    ok = True
    if os.path.exists(imu_path):
        lines = open(imu_path).read().strip().split("\n")
        print(f"\n=== imu.csv: {len(lines)-1} 条数据 + 表头 ===")
        print("  表头:", lines[0])
        print("  首行:", lines[1])
        ok = ok and len(lines) - 1 == 5
    else:
        print("imu.csv 缺失 ✗"); ok = False

    if os.path.exists(lidar_path):
        d = np.load(lidar_path)
        print(f"\n=== lidar.npz ===")
        print(f"  points: {d['points'].shape} (期望 15x3 = 5帧×3点)")
        print(f"  timestamps: {d['timestamps'].shape}")
        print(f"  intensity: {d['intensity'].shape}" if "intensity" in d else "  intensity: 无")
        print(f"  首点: {d['points'][0]}")
        ok = ok and d["points"].shape == (15, 3) and "intensity" in d
    else:
        print("lidar.npz 缺失 ✗"); ok = False

    print("\n验证结果:", "通过 ✓" if ok else "失败 ✗")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
