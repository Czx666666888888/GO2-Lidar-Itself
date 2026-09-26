#!/usr/bin/env python3
"""go2_keyboard_teleop 离线速度指令完整验证
用 pty 模拟真实 TTY 键盘, 覆盖全部 6 个按键方向 + 急停 + 组合键 + 松手归零。
离线(无狗)环境: 只验证节点发布的 Twist 速度指令正确, 不涉及真机。
"""
import os
import sys
import time
import pty
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Recorder(Node):
    def __init__(self):
        super().__init__("e2e_recorder")
        self.msgs = []
        self.create_subscription(Twist, "cmd_vel", self.cb, 10)

    def cb(self, msg):
        self.msgs.append((time.time(), msg.linear.x, msg.linear.y, msg.angular.z))


def spin_for(node, t):
    end = time.time() + t
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def last_speed(rec):
    if not rec.msgs:
        return None
    _, vx, vy, vyaw = rec.msgs[-1]
    return (round(vx, 2), round(vy, 2), round(vyaw, 2))


def main():
    master, slave = pty.openpty()
    env = dict(os.environ)
    proc = subprocess.Popen(
        ["ros2", "run", "go2_keyboard_teleop", "go2_keyboard_teleop_node"],
        stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
    os.close(slave)

    rclpy.init()
    rec = Recorder()
    spin_for(rec, 1.5)

    checks = []

    def hold_then_release():
        spin_for(rec, 0.6)  # > HOLD_TIMEOUT(0.3s), 松手归零
        return last_speed(rec)

    print("=== 离线速度指令完整验证 (无狗) ===")
    print("参数: max_vx=0.6, max_vy=0.4, max_vyaw=0.8\n")

    # 1. 六个方向按键 (长按满速)
    keymap = [
        ("w", (0.6, 0.0, 0.0), "前进"),
        ("s", (-0.6, 0.0, 0.0), "后退"),
        ("a", (0.0, 0.4, 0.0), "左移"),
        ("d", (0.0, -0.4, 0.0), "右移"),
        ("q", (0.0, 0.0, 0.8), "左转"),
        ("e", (0.0, 0.0, -0.8), "右转"),
    ]
    for key, (ex_vx, ex_vy, ex_vyaw), desc in keymap:
        os.write(master, key.encode())
        spin_for(rec, 0.25)
        v = last_speed(rec)
        expect = (ex_vx, ex_vy, ex_vyaw)
        ok_press = v == expect
        print(f"  按住[{key}]({desc}) -> vx={v[0]:+.2f} vy={v[1]:+.2f} vyaw={v[2]:+.2f}  "
              f"期望({ex_vx:+.2f},{ex_vy:+.2f},{ex_vyaw:+.2f}) {'✓' if ok_press else '✗'}")
        checks.append(ok_press)
        v = hold_then_release()
        ok_rel = v == (0.0, 0.0, 0.0)
        print(f"  松开[{key}] -> {'全零 ✓' if ok_rel else f'✗ 实际{v}'}")
        checks.append(ok_rel)

    # 2. 空格急停
    os.write(master, b"w")
    spin_for(rec, 0.2)
    os.write(master, b" ")
    spin_for(rec, 0.2)
    v = last_speed(rec)
    ok_stop = v == (0.0, 0.0, 0.0)
    print(f"\n  按住[w]后按[空格]急停 -> {'全零 ✓' if ok_stop else f'✗ 实际{v}'}")
    checks.append(ok_stop)

    # 3. 组合键 w+e (前进+右转)
    os.write(master, b"w")
    spin_for(rec, 0.1)
    os.write(master, b"e")
    spin_for(rec, 0.2)
    v = last_speed(rec)
    ok_combo = v == (0.6, 0.0, -0.8)
    print(f"  按住[w+e]组合 -> vx={v[0]:+.2f} vy={v[1]:+.2f} vyaw={v[2]:+.2f}  "
          f"期望(+0.60,+0.00,-0.80) {'✓' if ok_combo else '✗'}")
    checks.append(ok_combo)
    v = hold_then_release()
    ok_rel = v == (0.0, 0.0, 0.0)
    print(f"  松开[w+e] -> {'全零 ✓' if ok_rel else f'✗ 实际{v}'}")
    checks.append(ok_rel)

    # 4. 退出
    os.write(master, b"x")
    try:
        proc.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()

    ok = all(checks)
    print(f"\n断言: {sum(checks)}/{len(checks)} 项通过")
    print("验证结果:", "全部通过 ✓" if ok else "存在失败 ✗")

    rec.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
