#!/usr/bin/env python3
"""go2_sensor_recorder: 控制狗的同时记录回传的 IMU + 激光雷达数据

订阅(话题名与 go2_ros2_sdk 的 go2_driver_node 单机模式一致):
  imu           (sensor_msgs/Imu)          -> 保存 imu.csv
  point_cloud2  (sensor_msgs/PointCloud2)  -> 保存 lidar.npz (xyz/时间戳/intensity)

激光雷达为 best_effort QoS(与 go2_driver_node 匹配), IMU 为默认 reliable。
Ctrl-C 停止并落盘保存。数据实时累积在内存, 停止时统一写入, 适合数分钟遥控记录。

用法:
  ros2 run go2_keyboard_teleop go2_sensor_recorder
  ros2 run go2_keyboard_teleop go2_sensor_recorder --ros-args \
    -p outdir:=/home/lch/dog/go2_data -p imu_topic:=imu -p lidar_topic:=point_cloud2
"""
import os
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

IMU_HEADER = ("sec,nanosec,lin_acc_x,lin_acc_y,lin_acc_z,"
              "ang_vel_x,ang_vel_y,ang_vel_z,quat_x,quat_y,quat_z,quat_w")


class SensorRecorder(Node):
    def __init__(self):
        super().__init__("go2_sensor_recorder")
        self.declare_parameter("imu_topic", "imu")
        self.declare_parameter("lidar_topic", "point_cloud2")
        self.declare_parameter("outdir", os.path.expanduser("~/go2_data"))
        self.declare_parameter("duration", 0.0)  # >0: 自动记录 duration 秒后保存退出; 0: 手动 Ctrl-C

        imu_topic = self.get_parameter("imu_topic").value
        lidar_topic = self.get_parameter("lidar_topic").value
        outdir = self.get_parameter("outdir").value
        duration = self.get_parameter("duration").value
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self._stop_requested = False

        if duration > 0:
            self.create_timer(duration, self.request_stop)

        self.imu_rows = []
        self.imu_count = 0

        self.pts = []           # list of (N,3) float32
        self.intensities = []   # list of (N,) float32 (若有 intensity 字段)
        self.pts_ts = []        # list of (N,) 每点所属帧的时间戳(秒)
        self.lidar_frames = 0
        self.lidar_points = 0

        self.create_subscription(Imu, imu_topic, self.imu_cb, 1000)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PointCloud2, lidar_topic, self.lidar_cb, lidar_qos)

        self.get_logger().info(
            f"记录 IMU '{imu_topic}' + 激光雷达 '{lidar_topic}' -> '{outdir}'"
        )
        self.get_logger().info("Ctrl-C 停止并保存")

    def request_stop(self):
        self._stop_requested = True

    def imu_cb(self, msg: Imu):
        self.imu_rows.append([
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w,
        ])
        self.imu_count += 1

    def lidar_cb(self, msg: PointCloud2):
        # numpy 2.x 下 read_points_numpy 返回 (N, F) float32 二维数组(列=字段顺序)
        arr = point_cloud2.read_points_numpy(msg)
        names = [f.name for f in msg.fields]
        col = {n: i for i, n in enumerate(names)}
        xyz = arr[:, [col["x"], col["y"], col["z"]]].astype(np.float32)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pts.append(xyz)
        if "intensity" in col:
            self.intensities.append(arr[:, col["intensity"]].astype(np.float32))
        self.pts_ts.append(np.full(len(xyz), ts, dtype=np.float64))
        self.lidar_frames += 1
        self.lidar_points += len(xyz)

    def save(self):
        out = self.outdir
        if self.imu_rows:
            path = os.path.join(out, "imu.csv")
            with open(path, "w") as f:
                f.write(IMU_HEADER + "\n")
                for r in self.imu_rows:
                    f.write(",".join(f"{v:.9g}" for v in r) + "\n")
            self.get_logger().info(f"IMU 保存 {self.imu_count} 条 -> {path}")

        if self.pts:
            path = os.path.join(out, "lidar.npz")
            all_pts = np.concatenate(self.pts, axis=0)
            all_ts = np.concatenate(self.pts_ts, axis=0)
            if self.intensities:
                all_int = np.concatenate(self.intensities, axis=0)
                np.savez_compressed(path, points=all_pts, timestamps=all_ts,
                                    intensity=all_int)
            else:
                np.savez_compressed(path, points=all_pts, timestamps=all_ts)
            self.get_logger().info(
                f"激光雷达 {self.lidar_frames} 帧 / {self.lidar_points} 点 -> {path}"
            )

        if not self.imu_rows and not self.pts:
            self.get_logger().warn(
                "未收到任何数据! 请确认 go2_driver_node 已启动、话题名正确"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorRecorder()

    # SIGINT/SIGTERM -> 设置停止标志(不依赖异常传播, conda 环境下最可靠)
    # 注意: 在 signal handler 里 raise KeyboardInterrupt 会被 rclpy 的 C 层 spin 吞掉,
    # 改用标志位让主循环自行退出。
    signal.signal(signal.SIGINT, lambda s, f: node.request_stop())
    signal.signal(signal.SIGTERM, lambda s, f: node.request_stop())

    try:
        # 手动 spin 循环: 支持 duration 定时停止 + Ctrl-C
        while rclpy.ok() and not node._stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
