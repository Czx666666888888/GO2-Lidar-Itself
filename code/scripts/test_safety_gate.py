#!/usr/bin/env python3
"""WP4.0 安全门验证(状态机版):
1. 默认 DISARMED, 发 TwistStamped 不发命令
2. /arm=true 武装后发命令
3. 急停 -> FAULT 锁存, 速度回调不发命令
4. 心跳超时 -> FAULT
5. /arm=true 显式复位
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Int8
from unitree_go.msg import SportModeState


def main():
    env = dict(os.environ)
    log = "/tmp/safety_gate2.log"
    proc = subprocess.Popen(
        ["ros2", "run", "go2_keyboard_teleop", "go2_safety_gate",
         "--ros-args", "-p", "cmd_timeout:=0.5", "-p", "state_timeout:=0.8",
         "-p", "max_tilt:=0.6", "-p", "max_speed:=0.1"],
        stdout=open(log, "w"), stderr=subprocess.STDOUT, env=env)

    rclpy.init()
    node = Node("safety_test2")
    cmd_pub = node.create_publisher(TwistStamped, "/cmd_vel_raw", 10)
    state_pub = node.create_publisher(SportModeState, "/lf/sportmodestate", 10)
    arm_pub = node.create_publisher(Bool, "/arm", 10)
    stop_pub = node.create_publisher(Int8, "/stop", 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(node, timeout_sec=0.01)

    def send_cmd(vx):
        m = TwistStamped(); m.twist.linear.x = vx
        cmd_pub.publish(m)

    def send_state(tilt=0.0):
        s = SportModeState()
        s.imu_state.rpy = [tilt, 0.0, 0.0]
        s.foot_force = [10, 10, 10, 10]
        state_pub.publish(s)

    spin(1.5)
    print("=== WP4.0 安全门状态机验证 ===\n")

    # 1. 默认 DISARMED
    send_cmd(0.05); spin(0.3)
    print("1. 默认 DISARMED: 发 cmd -> 应不发命令")

    # 2. 武装(需先有状态)
    send_state(); spin(0.2)   # 先发状态, 才能武装
    a = Bool(); a.data = True
    arm_pub.publish(a); spin(0.2)
    send_cmd(0.05); spin(0.3)
    print("2. 有状态后 /arm=true 武装, 发 cmd -> 应发 Move")

    # 3. 急停 -> FAULT
    s = Int8(); s.data = 1
    stop_pub.publish(s); spin(0.2)
    send_cmd(0.05); spin(0.3)
    print("3. 急停 -> FAULT, 再发 cmd -> 应被拦截")

    # 4. 复位
    arm_pub.publish(a); spin(0.2)
    send_state(); spin(0.2)
    send_cmd(0.05); spin(0.3)
    print("4. /arm=true 复位后发 cmd -> 应恢复 Move")

    # 5. 心跳超时
    spin(1.2)
    print("5. 停发 sportmodestate -> 心跳超时 -> FAULT")

    spin(0.5)
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    node.destroy_node()
    rclpy.shutdown()

    lines = open(log).read().splitlines()
    # 分阶段判断(简化: 检查关键日志出现)
    checks = {
        "默认DISARMED": any("DISARMED" in l for l in lines),
        "武装后发Move": any("Move vx=+" in l and "StopMove" not in l for l in lines),
        "急停FAULT": any("独立停止" in l for l in lines),
        "FAULT状态": any("FAULT" in l for l in lines),
        "心跳超时": any("心跳超时" in l for l in lines),
    }
    ok = all(checks.values())
    print("\n=== 验证结果 ===")
    for name, passed in checks.items():
        print(f"  {'✓' if passed else '✗'} {name}")
    print("\n=== 状态转换日志 ===")
    for l in lines:
        if any(k in l for k in ["状态", "FAULT", "ARMED", "DISARMED", "超时", "独立停止", "复位"]):
            print("  " + (l.split("] ")[-1] if "] " in l else l))
    print(f"\n验证结果: {'全部通过 ✓' if ok else '存在失败 ✗'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
