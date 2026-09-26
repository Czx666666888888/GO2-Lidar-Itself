#!/usr/bin/env python3
"""go2_keyboard_teleop: 键盘遥控 Unitree Go2 (长按运行, 松手停止)

按键行为: 按住 WASDQE 即朝对应方向满速运行, 松开即停止。
只发布 Twist 速度指令, 不含任何路径规划/建图逻辑。

按键映射:
  w / s   前进 / 后退   (linear.x,  +max_vx / -max_vx)
  a / d   左移 / 右移   (linear.y,  +max_vy / -max_vy)
  q / e   左转 / 右转   (angular.z, +max_vyaw / -max_vyaw)
  空格    急停(立即归零)
  x       退出(先清零)

发布话题:
  cmd_vel_out  -> go2_ros2_sdk 的 go2_driver_node 订阅(WebRTC 下发到狗)
  cmd_vel      -> 标准话题, 便于 ros2 topic echo /cmd_vel 调试

松手检测: termios 只能收到"按下"字节, 收不到"松开"。利用键盘自动重复
(auto-repeat) 特性: 按住时内核持续送来字节, 松手后停止。用 select 轮询,
超过 HOLD_TIMEOUT 没有新字节即判定松手并归零。
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HOLD_TIMEOUT = 0.3  # 秒: 该时间内无新按键, 判定松手 -> 停止


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("go2_keyboard_teleop")

        self.declare_parameter("topic_cmd_vel_out", "cmd_vel_out")
        self.declare_parameter("topic_cmd_vel", "cmd_vel")
        self.declare_parameter("max_vx", 0.6)
        self.declare_parameter("max_vy", 0.4)
        self.declare_parameter("max_vyaw", 0.8)
        self.declare_parameter("publish_rate", 20.0)

        topic_out = self.get_parameter("topic_cmd_vel_out").value
        topic_std = self.get_parameter("topic_cmd_vel").value
        self.max_vx = self.get_parameter("max_vx").value
        self.max_vy = self.get_parameter("max_vy").value
        self.max_vyaw = self.get_parameter("max_vyaw").value
        rate = self.get_parameter("publish_rate").value

        self.pub_out = self.create_publisher(Twist, topic_out, 10)
        self.pub_std = self.create_publisher(Twist, topic_std, 10)

        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self._last_state = None
        self.last_key_time = time.time()

        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.get_logger().info(
            f"键盘遥操就绪(长按运行/松手停止) -> '{topic_out}' + '{topic_std}' | "
            f"限幅 vx={self.max_vx} vy={self.max_vy} vyaw={self.max_vyaw}"
        )
        self.get_logger().info(
            "按住 w/s 前后 | a/d 平移 | q/e 转向 | 松开即停 | 空格急停 | x 退出"
        )

    def publish(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.vyaw
        self.pub_out.publish(msg)
        self.pub_std.publish(msg)

    def _log_if_changed(self):
        state = (self.vx, self.vy, self.vyaw)
        if state != self._last_state:
            self._last_state = state
            if self.vx == 0 and self.vy == 0 and self.vyaw == 0:
                self.get_logger().info(">>> 下发到狗: 停止 (vx=0 vy=0 vyaw=0)")
            else:
                self.get_logger().info(
                    f">>> 下发到狗: vx={self.vx:+.2f} vy={self.vy:+.2f} vyaw={self.vyaw:+.2f}"
                )

    def press(self, key: str):
        """按下键 -> 设置该方向满速"""
        if key == "w":
            self.vx = self.max_vx
        elif key == "s":
            self.vx = -self.max_vx
        elif key == "a":
            self.vy = self.max_vy
        elif key == "d":
            self.vy = -self.max_vy
        elif key == "q":
            self.vyaw = self.max_vyaw
        elif key == "e":
            self.vyaw = -self.max_vyaw
        elif key == " ":
            self.vx = self.vy = self.vyaw = 0.0
        else:
            # 读到了按键但未映射: 打印诊断, 便于排查大小写/方向键/输入法等问题
            self.get_logger().info(f"未映射按键: {key!r} (请用小写 w/s/a/d/q/e)")
            return
        self._log_if_changed()

    def release_all(self):
        """松手: 全部速度归零"""
        if self.vx != 0.0 or self.vy != 0.0 or self.vyaw != 0.0:
            self.vx = self.vy = self.vyaw = 0.0
            self._log_if_changed()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    # 非交互(无 TTY)环境: 只持续发布零速度, 不阻塞
    if not sys.stdin.isatty():
        node.get_logger().warn("stdin 不是 TTY, 进入静默模式(仅发布零速度)")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return 0

    # 交互式: 原始模式 + select 轮询(检测按下/松手)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    node.get_logger().info(
        f"已进入键盘读取模式 (fd={fd}, isatty={sys.stdin.isatty()}), 请按键 w/s/a/d/q/e"
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            r, _, _ = select.select([fd], [], [], 0.02)
            if r:
                ch = os.read(fd, 1)
                try:
                    key = ch.decode("utf-8")
                except UnicodeDecodeError:
                    key = ""
                node.last_key_time = time.time()
                if key in ("x", "X", "\x03"):  # x 或 Ctrl-C 退出
                    break
                node.press(key)
            else:
                # 超时无新按键 -> 判定松手 -> 停止
                if time.time() - node.last_key_time > HOLD_TIMEOUT:
                    node.release_all()
    except KeyboardInterrupt:
        pass
    finally:
        node.vx = node.vy = node.vyaw = 0.0
        node.publish()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
