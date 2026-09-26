#!/usr/bin/env python3
"""base_odom_node: 把原始 Point-LIO 里程计(雷达位姿)转换成机身中心位姿

/state_estimation (原始, 雷达中心)
      ↓ 减去传感器前向偏置
/base_state_estimation (机身中心)

用途: 统一 FAR / localPlanner / pathFollower 的机器人位置参考点,
      消除 0.3m 传感器偏置不一致导致的"近航点死锁"。

用法:
  ros2 run go2_keyboard_teleop base_odom_node
  ros2 run go2_keyboard_teleop base_odom_node --ros-args \
    -p sensor_offset_x:=0.3 -p sensor_offset_y:=0.0
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class BaseOdomNode(Node):
    def __init__(self):
        super().__init__("base_odom_node")
        self.declare_parameter("sensor_offset_x", 0.3)
        self.declare_parameter("sensor_offset_y", 0.0)
        self.offset_x = self.get_parameter("sensor_offset_x").value
        self.offset_y = self.get_parameter("sensor_offset_y").value

        self.sub = self.create_subscription(
            Odometry, "/state_estimation", self.cb, 10)
        self.pub = self.create_publisher(
            Odometry, "/base_state_estimation", 10)
        self.get_logger().info(
            f"机身中心里程计转换就绪 | 偏置 x={self.offset_x}m y={self.offset_y}m")

    def cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # 减前向偏置: 机身中心 = 雷达 - R(yaw) * offset
        x = msg.pose.pose.position.x - np.cos(yaw) * self.offset_x \
            + np.sin(yaw) * self.offset_y
        y = msg.pose.pose.position.y - np.sin(yaw) * self.offset_x \
            - np.cos(yaw) * self.offset_y

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation = msg.pose.pose.orientation   # 朝向不变
        out.twist = msg.twist                                   # 线/角速度不变
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BaseOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
