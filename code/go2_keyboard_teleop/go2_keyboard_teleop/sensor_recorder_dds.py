#!/usr/bin/env python3
"""go2_sensor_recorder_dds: DDS 链路传感器数据记录

记录官方 unitree_ros2 (DDS) 链路的传感器回传数据:
  /utlidar/imu        (sensor_msgs/Imu) -> imu.csv (标准 IMU)
  lf/sportmodestate   (SportModeState)  -> odom.csv (位置/速度)
  /utlidar/cloud      (PointCloud2)     -> lidar.npz (点云)

Ctrl-C 或 duration 定时停止并保存。
激光雷达为 best_effort QoS(与官方一致), 状态为默认 reliable。
"""
import json
import os
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from unitree_go.msg import SportModeState

IMU_HEADER = ("sec,nanosec,qx,qy,qz,qw,"
              "gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z")
ODOM_HEADER = "sec,nanosec,px,py,pz,vx,vy,vz,yaw_speed"


class SensorRecorderDDS(Node):
    def __init__(self):
        super().__init__("go2_sensor_recorder_dds")
        self.declare_parameter("imu_topic", "/utlidar/imu")
        self.declare_parameter("sportmode_topic", "lf/sportmodestate")
        self.declare_parameter("lidar_topic", "/utlidar/cloud")
        self.declare_parameter("outdir", os.path.expanduser("~/go2_data"))
        self.declare_parameter("duration", 0.0)

        imu_topic = self.get_parameter("imu_topic").value
        sport_topic = self.get_parameter("sportmode_topic").value
        lidar_topic = self.get_parameter("lidar_topic").value
        outdir = self.get_parameter("outdir").value
        duration = self.get_parameter("duration").value
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self._stop_requested = False

        if duration > 0:
            self.create_timer(duration, self.request_stop)

        self.imu_rows = []
        self.odom_rows = []
        self.pts = []
        self.intensities = []
        self.pts_ts = []
        self.lidar_frames = 0
        self.lidar_points = 0

        self.create_subscription(Imu, imu_topic, self.imu_cb, 100)
        self.create_subscription(SportModeState, sport_topic, self.sport_cb, 100)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PointCloud2, lidar_topic, self.lidar_cb, lidar_qos)

        self.get_logger().info(
            f"记录 DDS 传感器 '{imu_topic}' + '{sport_topic}' + '{lidar_topic}' -> '{outdir}'"
        )
        self.get_logger().info("Ctrl-C 停止并保存")

    def request_stop(self):
        self._stop_requested = True

    def imu_cb(self, msg: Imu):
        self.imu_rows.append([
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

    def sport_cb(self, msg: SportModeState):
        t_sec = msg.stamp.sec
        t_ns = msg.stamp.nanosec
        self.odom_rows.append([
            t_sec, t_ns,
            *msg.position,        # px py pz
            *msg.velocity,        # vx vy vz
            msg.yaw_speed,
        ])

    def lidar_cb(self, msg: PointCloud2):
        # read_points 返回结构化数组(字段名访问), 支持不同 datatype 字段
        arr = point_cloud2.read_points(
            msg, field_names=["x", "y", "z", "intensity"], skip_nans=False)
        if len(arr) == 0:
            return
        xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32)
        intensity = arr["intensity"].astype(np.float32)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pts.append(xyz)
        self.intensities.append(intensity)
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
            self.get_logger().info(f"IMU 保存 {len(self.imu_rows)} 条 -> {path}")
        if self.odom_rows:
            path = os.path.join(out, "odom.csv")
            with open(path, "w") as f:
                f.write(ODOM_HEADER + "\n")
                for r in self.odom_rows:
                    f.write(",".join(f"{v:.9g}" for v in r) + "\n")
            self.get_logger().info(f"里程计保存 {len(self.odom_rows)} 条 -> {path}")
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
                "未收到任何数据! 确认狗已连接、话题名正确(默认 lf/sportmodestate)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorRecorderDDS()
    signal.signal(signal.SIGINT, lambda s, f: node.request_stop())
    signal.signal(signal.SIGTERM, lambda s, f: node.request_stop())
    try:
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
