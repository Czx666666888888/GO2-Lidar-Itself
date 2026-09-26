#!/usr/bin/env python3
"""replay_dds_data: 把采集的 imu.csv + lidar.npz 回放成 ROS2 话题, 喂给建图链

发布:
  /utlidar/imu   (sensor_msgs/Imu)      <- imu.csv
  /utlidar/cloud (PointCloud2, x/y/z/intensity/ring) <- lidar.npz

按原始时间戳顺序实时回放(可加速)。配合 autonomy_stack_go2 建图链:
  point_lio_unilidar (mapping_utlidar.launch) + terrain_analysis

用法:
  python3 replay_dds_data.py
  python3 replay_dds_data.py --rate 2.0 --max_time 30.0   # 2倍速回放前30秒
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class ReplayDDS(Node):
    def __init__(self, imu_csv, lidar_npz, rate, max_time):
        super().__init__("replay_dds_data")
        self.imu_pub = self.create_publisher(Imu, "/utlidar/imu", 50)
        self.cloud_pub = self.create_publisher(PointCloud2, "/utlidar/cloud", 50)

        # 加载 IMU
        self.imu_events = self._load_imu(imu_csv)
        # 加载点云(按帧分组)
        self.cloud_events = self._load_cloud(lidar_npz)

        # 合并事件按时间戳排序
        self.events = sorted(self.imu_events + self.cloud_events, key=lambda e: e[0])
        self.get_logger().info(
            f"加载 IMU {len(self.imu_events)} 条 + 点云 {len(self.cloud_events)} 帧, "
            f"共 {len(self.events)} 事件"
        )

        if max_time > 0:
            self.events = [e for e in self.events if e[0] - self.events[0][0] <= max_time]
            self.get_logger().info(f"截取前 {max_time}s -> {len(self.events)} 事件")

        self.t0 = self.events[0][0]
        self.rate = rate
        self.idx = 0
        self.start_wall = None

        # 10ms 定时回放
        self.timer = self.create_timer(0.01, self.replay)
        self.get_logger().info(f"开始回放(rate={rate}x), 时长 {(self.events[-1][0]-self.t0):.1f}s")

    def _load_imu(self, path):
        data = np.loadtxt(path, delimiter=",", skiprows=1)
        events = []
        for i in range(len(data)):
            sec = int(data[i, 0]); ns = int(data[i, 1])
            t = sec + ns * 1e-9
            m = Imu()
            m.header = Header()
            m.header.stamp.sec = sec
            m.header.stamp.nanosec = ns
            m.header.frame_id = "utlidar_imu"
            m.orientation.x = data[i, 2]; m.orientation.y = data[i, 3]
            m.orientation.z = data[i, 4]; m.orientation.w = data[i, 5]
            m.angular_velocity.x = data[i, 6]
            m.angular_velocity.y = data[i, 7]
            m.angular_velocity.z = data[i, 8]
            m.linear_acceleration.x = data[i, 9]
            m.linear_acceleration.y = data[i, 10]
            m.linear_acceleration.z = data[i, 11]
            events.append((t, "imu", m))
        return events

    def _load_cloud(self, path):
        d = np.load(path)
        pts = d["points"]; ts = d["timestamps"]; it = d["intensity"]
        # 按时间戳排序后切分帧
        order = np.argsort(ts)
        pts, ts, it = pts[order], ts[order], it[order]
        bounds = np.where(np.diff(ts) > 0)[0] + 1
        starts = np.concatenate([[0], bounds])
        ends = np.concatenate([bounds, [len(ts)]])

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        events = []
        for s, e in zip(starts, ends):
            t = ts[s]
            sec = int(t); ns = int((t - sec) * 1e9)
            hdr = Header()
            hdr.stamp.sec = sec
            hdr.stamp.nanosec = ns
            hdr.frame_id = "utlidar_lidar"
            # x,y,z,intensity,ring(补0)
            arr = np.column_stack([
                pts[s:e, 0], pts[s:e, 1], pts[s:e, 2],
                it[s:e], np.zeros(e - s, dtype=np.float32),
            ]).astype(np.float32)
            cloud = point_cloud2.create_cloud(hdr, fields, arr)
            events.append((t, "cloud", cloud))
        return events

    def replay(self):
        if self.start_wall is None:
            self.start_wall = time.time()
        # 当前回放到的数据时间
        now_data_t = self.t0 + (time.time() - self.start_wall) * self.rate
        while self.idx < len(self.events) and self.events[self.idx][0] <= now_data_t:
            _, kind, msg = self.events[self.idx]
            if kind == "imu":
                self.imu_pub.publish(msg)
            else:
                self.cloud_pub.publish(msg)
            self.idx += 1
        if self.idx >= len(self.events):
            self.get_logger().info("回放完成")
            self.destroy_timer(self.timer)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--imu_csv", default="/home/lch/dog/go2_data/imu.csv")
    parser.add_argument("--lidar_npz", default="/home/lch/dog/go2_data/lidar.npz")
    parser.add_argument("--rate", type=float, default=1.0, help="回放倍速")
    parser.add_argument("--max_time", type=float, default=0.0, help="只回放前N秒(0=全部)")
    args = parser.parse_args()

    rclpy.init()
    node = ReplayDDS(args.imu_csv, args.lidar_npz, args.rate, args.max_time)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
