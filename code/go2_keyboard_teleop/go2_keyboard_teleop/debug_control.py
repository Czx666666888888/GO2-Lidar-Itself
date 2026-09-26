#!/usr/bin/env python3
"""go2_debug_control: 真机调试键盘控制(控制导航程序启停, 不是控制狗运动)

按键:
  s / 空格 触发探索(发 /update_visibility_graph=true, far_planner 规划新目标)
  e        急停(发 StopMove 到 /api/sport/request + 停止探索)
  r        触发录包(提示, 实际由 go2_debug.sh 管理)
  q        退出

用途: 真机调试时, 用键盘控制导航程序的启动/触发/急停, 全程记录数据。
"""
import json
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from unitree_api.msg import Request

API_STOP_MOVE = 1003


class DebugControl(Node):
    def __init__(self):
        super().__init__("go2_debug_control")
        self.pub_update = self.create_publisher(Bool, "/update_visibility_graph", 5)
        self.pub_sport = self.create_publisher(Request, "/api/sport/request", 10)
        self._req_id = 0
        self.get_logger().info(
            "真机调试控制就绪: s/空格=触发探索, e=急停, q=退出")

    def trigger_explore(self):
        msg = Bool()
        msg.data = True
        self.pub_update.publish(msg)
        self.get_logger().info(">>> 触发探索(/update_visibility_graph=true)")

    def emergency_stop(self):
        req = Request()
        req.header.identity.id = self._req_id
        self._req_id += 1
        req.header.identity.api_id = API_STOP_MOVE
        self.pub_sport.publish(req)
        self.get_logger().warn(">>> 急停(StopMove)!")

    def handle(self, key):
        if key in ("s", " "):
            self.trigger_explore()
        elif key == "e":
            self.emergency_stop()
        elif key in ("q", "x", "\x03"):
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = DebugControl()

    if not sys.stdin.isatty():
        node.get_logger().warn("stdin 非 TTY, 仅订阅(不读键盘)")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return 0

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
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
                if not node.handle(key):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
