#!/usr/bin/env python3
"""go2_cmd_monitor: 实时显示下发给狗的 cmd_vel_out 指令(去重, 指令变化时才打印)

用法:
  ros2 run go2_keyboard_teleop go2_cmd_monitor                 # 监听 cmd_vel_out
  ros2 run go2_keyboard_teleop go2_cmd_monitor --ros-args -p topic:=cmd_vel

每次键盘指令变化(按下/松手/换向)即打印一行下发给狗的速度指令。
持续按住期间指令不变, 不重复打印; 松手停止时打印"停止"。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdMonitor(Node):
    def __init__(self):
        super().__init__("go2_cmd_monitor")
        self.declare_parameter("topic", "cmd_vel_out")
        topic = self.get_parameter("topic").value
        self._last = None
        self.create_subscription(Twist, topic, self.cb, 10)
        self.get_logger().info(f"监听下发指令话题 '{topic}' (指令变化时打印)")

    def cb(self, msg: Twist):
        v = (round(msg.linear.x, 2), round(msg.linear.y, 2), round(msg.angular.z, 2))
        if v == self._last:
            return
        self._last = v
        if v == (0.0, 0.0, 0.0):
            self.get_logger().info(">>> 下发: 停止 (vx=0 vy=0 vyaw=0)")
        else:
            self.get_logger().info(
                f">>> 下发: vx={v[0]:+.2f} vy={v[1]:+.2f} vyaw={v[2]:+.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
