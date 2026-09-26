#!/usr/bin/env python3
"""go2_keyboard_teleop_dds: 键盘遥控 Go2 (官方 DDS 链路)

走官方 unitree_ros2 的 DDS 通道: 发布 unitree_api::msg::Request 到
/api/sport/request, 用 api_id 区分命令(与官方 SportClient 一致), 不经 WebRTC。

按键(长按运行/松手停止):
  w / s      前进/后退   (Move api_id=1008, vx=±max_vx)
  a / d      左移/右移   (vy=±max_vy)
  q / e      左转/右转   (vyaw=±max_vyaw)
  空格       急停        (速度归零)
  b          平衡站立    (BalanceStand, 1002)
  c          坐下        (Sit, 1009)
  v          起立        (StandUp, 1004)
  x          退出

依赖: unitree_api (官方消息包, go2_ws 已安装)。
狗侧需运行官方 unitree_ros2 节点(EDU/PRO 版机载计算单元)。
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
from unitree_api.msg import Request

# 官方 sport api_id (与 ros2_sport_client.h 一致)
API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_MOVE = 1008
API_SIT = 1009
API_SWITCH_GAIT = 1011

HOLD_TIMEOUT = 0.3  # 松手判定(秒)


class KeyboardTeleopDDS(Node):
    def __init__(self):
        super().__init__("go2_keyboard_teleop_dds")
        self.declare_parameter("topic", "/api/sport/request")
        self.declare_parameter("max_vx", 0.6)
        self.declare_parameter("max_vy", 0.4)
        self.declare_parameter("max_vyaw", 0.8)
        self.declare_parameter("publish_rate", 20.0)

        topic = self.get_parameter("topic").value
        self.max_vx = self.get_parameter("max_vx").value
        self.max_vy = self.get_parameter("max_vy").value
        self.max_vyaw = self.get_parameter("max_vyaw").value
        rate = self.get_parameter("publish_rate").value

        self.pub = self.create_publisher(Request, topic, 10)
        self.vx = self.vy = self.vyaw = 0.0
        self._req_id = 0
        self._last_state = None
        self._stop_locked = False   # 空格急停锁存: 按移动键才解除
        self.last_key_time = time.time()

        # 持续发布 Move(长按维持速度, 松手发 Move(0,0,0) 停止)
        self.timer = self.create_timer(1.0 / rate, self.publish_move)

        self.get_logger().info(
            f"DDS 键盘遥操就绪 -> '{topic}' | vx={self.max_vx} vy={self.max_vy} "
            f"vyaw={self.max_vyaw}"
        )
        self.get_logger().info(
            "按住 w/s 前后 | a/d 平移 | q/e 转向 | b站 c坐 v起 | 空格急停 | x退出"
        )

    def _make_request(self, api_id, parameter=""):
        req = Request()
        req.header.identity.id = self._req_id
        req.header.identity.api_id = api_id
        req.parameter = parameter
        self._req_id += 1
        return req

    def publish_move(self):
        """持续发布 Move 速度指令(急停锁存时发 Move(0,0,0) 保持停)"""
        if self._stop_locked:
            vx = vy = vyaw = 0.0
        else:
            vx, vy, vyaw = self.vx, self.vy, self.vyaw
        req = self._make_request(
            API_MOVE, json.dumps({"x": vx, "y": vy, "z": vyaw}))
        self.pub.publish(req)

    def send_api(self, api_id, parameter="", label=""):
        """发送单次命令(站立/坐下等)"""
        req = self._make_request(api_id, parameter)
        self.pub.publish(req)
        self.get_logger().info(f">>> 下发: api_id={api_id} ({label}) {parameter}")

    def _log_if_changed(self):
        state = (self.vx, self.vy, self.vyaw)
        if state != self._last_state:
            self._last_state = state
            if self.vx == 0 and self.vy == 0 and self.vyaw == 0:
                self.get_logger().info(">>> 下发到狗: 停止 (Move 0,0,0)")
            else:
                self.get_logger().info(
                    f">>> 下发到狗: Move vx={self.vx:+.2f} vy={self.vy:+.2f} "
                    f"vyaw={self.vyaw:+.2f}"
                )

    def press(self, key):
        if key in ("w", "s", "a", "d", "q", "e"):
            self._stop_locked = False   # 移动键解除急停锁存
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
            # 强制停止: 发 StopMove(1003) 真正的急停命令 + 速度归零 + 锁存
            self.vx = self.vy = self.vyaw = 0.0
            self._stop_locked = True
            self.send_api(API_STOP_MOVE, label="StopMove 急停")
            return
        elif key == "b":
            self.send_api(API_BALANCE_STAND, label="BalanceStand")
            return
        elif key == "c":
            self.send_api(API_SIT, label="Sit")
            return
        elif key == "v":
            self.send_api(API_STAND_UP, label="StandUp")
            return
        elif key == "g":
            self.send_api(API_SWITCH_GAIT, '{"data":1}', label="SwitchGait")
            return
        else:
            self.get_logger().info(f"未映射按键: {key!r}")
            return
        self._log_if_changed()

    def release_all(self):
        if self.vx != 0.0 or self.vy != 0.0 or self.vyaw != 0.0:
            self.vx = self.vy = self.vyaw = 0.0
            self._log_if_changed()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopDDS()

    if not sys.stdin.isatty():
        node.get_logger().warn("stdin 不是 TTY, 进入静默模式(仅发 Move 0)")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return 0

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    node.get_logger().info(
        f"已进入键盘读取模式 (isatty={sys.stdin.isatty()}), 请按键"
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
                if key in ("x", "X", "\x03"):
                    break
                node.press(key)
            else:
                if time.time() - node.last_key_time > HOLD_TIMEOUT:
                    node.release_all()
    except KeyboardInterrupt:
        pass
    finally:
        node.vx = node.vy = node.vyaw = 0.0
        node.publish_move()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
