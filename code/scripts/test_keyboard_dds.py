#!/usr/bin/env python3
"""go2_keyboard_teleop_dds 离线验证
pty 喂按键, 订阅 /api/sport/request 验证 Request 的 api_id 和 parameter 正确。
"""
import os
import sys
import time
import pty
import subprocess
import json

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class Recorder(Node):
    def __init__(self):
        super().__init__("dds_recorder")
        self.reqs = []  # (api_id, parameter)
        self.create_subscription(Request, "/api/sport/request", self.cb, 10)

    def cb(self, msg):
        self.reqs.append((msg.header.identity.api_id, msg.parameter))


def spin_for(node, t):
    end = time.time() + t
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def last(rec):
    if not rec.reqs:
        return None
    return rec.reqs[-1]


def main():
    master, slave = pty.openpty()
    env = dict(os.environ)
    proc = subprocess.Popen(
        ["ros2", "run", "go2_keyboard_teleop", "go2_keyboard_teleop_dds"],
        stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
    os.close(slave)

    rclpy.init()
    rec = Recorder()
    spin_for(rec, 1.5)

    checks = []
    print("=== DDS 键盘链路离线验证 (/api/sport/request) ===\n")

    def press(key, label):
        os.write(master, key)
        spin_for(rec, 0.25)
        return last(rec)

    # 1. 按住 w -> Move api_id=1008, {"x":0.6,"y":0,"z":0}
    api, param = press(b"w", "w")
    expect_param = json.dumps({"x": 0.6, "y": 0.0, "z": 0.0})
    ok = api == 1008 and json.loads(param) == {"x": 0.6, "y": 0.0, "z": 0.0}
    print(f"  按住[w] -> api_id={api} param={param}  (期望 1008 + {expect_param}) {'✓' if ok else '✗'}")
    checks.append(ok)

    # 2. 按住 a -> Move vy=0.2
    api, param = press(b"a", "a")
    ok = api == 1008 and json.loads(param) == {"x": 0.6, "y": 0.4, "z": 0.0}
    print(f"  按住[a] -> api_id={api} param={param}  (期望 Move vy=+0.4) {'✓' if ok else '✗'}")
    checks.append(ok)

    # 3. 松手 -> Move 0,0,0
    spin_for(rec, 0.6)
    api, param = last(rec)
    ok = api == 1008 and json.loads(param) == {"x": 0.0, "y": 0.0, "z": 0.0}
    print(f"  松开    -> api_id={api} param={param}  (期望 Move 0,0,0) {'✓' if ok else '✗'}")
    checks.append(ok)

    # 4. 姿态键 b -> BalanceStand 1002 (单次命令, 检查窗口内是否出现)
    before = len(rec.reqs)
    os.write(master, b"b")
    spin_for(rec, 0.25)
    appeared = any(api == 1002 for api, _ in rec.reqs[before:])
    print(f"  按[b]   -> 窗口内出现 BalanceStand(1002): {appeared}  {'✓' if appeared else '✗'}")
    checks.append(appeared)

    # 5. 姿态键 c -> Sit 1009
    before = len(rec.reqs)
    os.write(master, b"c")
    spin_for(rec, 0.25)
    appeared = any(api == 1009 for api, _ in rec.reqs[before:])
    print(f"  按[c]   -> 窗口内出现 Sit(1009): {appeared}  {'✓' if appeared else '✗'}")
    checks.append(appeared)

    # 6. 姿态键 v -> StandUp 1004
    before = len(rec.reqs)
    os.write(master, b"v")
    spin_for(rec, 0.25)
    appeared = any(api == 1004 for api, _ in rec.reqs[before:])
    print(f"  按[v]   -> 窗口内出现 StandUp(1004): {appeared}  {'✓' if appeared else '✗'}")
    checks.append(appeared)

    # 退出
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
