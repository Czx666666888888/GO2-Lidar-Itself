#!/usr/bin/env python3
"""WP4 真机低速闭环测试脚本

测试项(按评审验收):
  1. 低速前进 0.5m: 发 cmd_vel_raw=0.05m/s, 狗前进 0.5m 后停
  2. 停止响应: 停止发命令 -> 0.5s 内速度归零
  3. 急停: 中途触发 /stop -> 立即停(FAULT)
  4. 循环 10 次短目标(可配置)

前置(另开终端):
  - 安全门: ros2 run go2_keyboard_teleop go2_safety_gate --ros-args -p dry_run:=false
  - 本脚本自动武装(/arm=true) + 发低速命令

用法:
  python3 wp4_low_speed_test.py            # 单次前进 0.5m 测试
  python3 wp4_low_speed_test.py --loop 10  # 循环 10 次短目标
  python3 wp4_low_speed_test.py --estop    # 测试急停
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Int8
from unitree_go.msg import SportModeState


class LowSpeedTest(Node):
    def __init__(self):
        super().__init__("wp4_low_speed_test")
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel_raw", 10)
        self.arm_pub = self.create_publisher(Bool, "/arm", 10)
        self.stop_pub = self.create_publisher(Int8, "/stop", 10)
        self.state_sub = self.create_subscription(
            SportModeState, "/lf/sportmodestate", self.cb_state, 10)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.got_state = False

    def cb_state(self, msg: SportModeState):
        self.pos = np.array(msg.position)
        self.vel = np.array(msg.velocity)
        self.yaw = msg.imu_state.rpy[2]   # yaw(弧度)
        self.got_state = True

    def send_cmd(self, vx, vyaw=0.0):
        m = TwistStamped()
        m.header.frame_id = "vehicle"
        m.twist.linear.x = vx
        m.twist.angular.z = vyaw
        self.cmd_pub.publish(m)

    def spin(self, t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_state(self, timeout=5.0):
        e = time.time() + timeout
        while time.time() < e and not self.got_state:
            rclpy.spin_once(self, timeout_sec=0.01)
        return self.got_state

    def arm(self):
        b = Bool(); b.data = True
        self.arm_pub.publish(b)
        self.spin(0.3)

    def _move(self, speed, dist):
        """持续发命令走 dist 米, 停止后等速度归零. 返回 (moved, stop_time, stopped)"""
        duration = abs(dist / speed)
        end = time.time() + duration
        while time.time() < end:
            self.send_cmd(speed)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.send_cmd(0.0)
        t_stop = time.time()
        stop_time = None
        while time.time() - t_stop < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if np.linalg.norm(self.vel) < 0.02:
                stop_time = time.time() - t_stop
                break
        moved = np.linalg.norm(self.pos - self._move_start)
        self._move_start = self.pos.copy()
        stopped = np.linalg.norm(self.vel) < 0.02
        return moved, stop_time, stopped

    def test_forward(self, speed=0.3, dist=1.0):
        """低速前进 dist 米"""
        self.get_logger().info(f"=== 低速前进测试: {speed}m/s × {dist}m ===")
        if not self.wait_state():
            self.get_logger().error("无 sportmodestate, 无法测试")
            return False
        self._move_start = self.pos.copy()
        self.arm()
        moved, stop_time, stopped = self._move(speed, dist)
        self.get_logger().info(
            f"停止响应 {stop_time:.2f}s (验收<0.5s) | "
            f"最终移动 {moved:.3f}m (期望 {dist}m) | "
            f"{'已停' if stopped else '未停!'}")
        return (moved > dist * 0.5 and stopped
                and stop_time is not None and stop_time < 0.5)

    def test_roundtrip(self, speed=0.3, dist=1.0):
        """往返: 前进 dist 米 -> 后退 dist 米回到起点"""
        self.get_logger().info(f"=== 往返测试: ±{speed}m/s × {dist}m ===")
        if not self.wait_state():
            self.get_logger().error("无 sportmodestate, 无法测试")
            return False
        start = self.pos.copy()
        self._move_start = self.pos.copy()
        self.arm()
        # 前进
        m1, t1, s1 = self._move(speed, dist)
        self.get_logger().info(f"  前进 {m1:.3f}m (停止 {t1:.2f}s)")
        self.spin(0.5)   # 换向间隔
        # 后退
        m2, t2, s2 = self._move(-speed, dist)
        self.get_logger().info(f"  后退 {m2:.3f}m (停止 {t2:.2f}s)")
        # 回到起点误差
        ret_err = np.linalg.norm(self.pos - start)
        self.get_logger().info(
            f"回到起点误差 {ret_err:.3f}m (验收 <{dist*0.3:.1f}m)")
        return (m1 > dist * 0.5 and m2 > dist * 0.5 and ret_err < dist * 0.3
                and s1 and s2)

    def test_turn(self, vyaw=0.3, angle_deg=90.0):
        """原地转向: 发 vyaw 转 angle_deg 度"""
        angle = angle_deg * np.pi / 180.0   # 度 -> 弧度
        self.get_logger().info(
            f"=== 转向测试: {vyaw}rad/s × {angle_deg:.0f}° ===")
        if not self.wait_state():
            self.get_logger().error("无 sportmodestate, 无法测试")
            return False
        start_yaw = self.yaw
        self.arm()
        duration = abs(angle / vyaw)
        end = time.time() + duration
        while time.time() < end:
            self.send_cmd(0.0, vyaw)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.send_cmd(0.0, 0.0)
        # 等停止
        t_stop = time.time()
        while time.time() - t_stop < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if np.linalg.norm(self.vel) < 0.02:
                break
        dyaw = self.yaw - start_yaw
        dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi   # wrap 到 [-pi, pi]
        self.get_logger().info(
            f"实际转过 {dyaw*180/np.pi:.0f}° (期望 {angle_deg:.0f}°)")
        return abs(dyaw) > angle * 0.5

    def test_estop(self, speed=0.3):
        """急停测试: 狗持续前进时触发 /stop, 验证立即停"""
        self.get_logger().info(f"=== 急停测试: 前进中触发 /stop ===")
        if not self.wait_state():
            self.get_logger().error("无 sportmodestate, 无法测试")
            return False
        self.arm()
        # 持续前进, 1.5 秒后触发急停
        t0 = time.time()
        estop_sent = False
        while time.time() - t0 < 3.0:
            self.send_cmd(speed)
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 1.5 and not estop_sent:
                s = Int8(); s.data = 1
                self.stop_pub.publish(s)
                estop_sent = True
                self.get_logger().warn("触发急停 /stop=1")
                t_estop = time.time()
                break
        # 等速度归零, 记录停止响应
        stop_time = None
        while time.time() - t_estop < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if np.linalg.norm(self.vel) < 0.02:
                stop_time = time.time() - t_estop
                break
        stopped = np.linalg.norm(self.vel) < 0.02
        self.get_logger().info(
            f"急停后停止响应 {stop_time:.2f}s (验收<0.5s) -> "
            f"{'已停' if stopped else '未停!'}")
        return stopped and stop_time is not None and stop_time < 0.5


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--loop", type=int, default=1)
    parser.add_argument("--estop", action="store_true")
    parser.add_argument("--roundtrip", action="store_true")
    parser.add_argument("--turn", action="store_true")
    parser.add_argument("--speed", type=float, default=0.3)
    parser.add_argument("--dist", type=float, default=1.0)
    parser.add_argument("--vyaw", type=float, default=0.3, help="转向角速度 rad/s")
    parser.add_argument("--angle", type=float, default=90.0, help="转向角度(度)")
    args = parser.parse_args()

    rclpy.init()
    node = LowSpeedTest()

    if args.estop:
        ok = node.test_estop()
    elif args.roundtrip:
        ok = node.test_roundtrip(args.speed, args.dist)
    elif args.turn:
        ok = node.test_turn(args.vyaw, args.angle)
    else:
        ok_all = True
        for i in range(args.loop):
            node.get_logger().info(f"\n########## 第 {i+1}/{args.loop} 次 ##########")
            ok = node.test_forward(args.speed, args.dist)
            ok_all = ok_all and ok
            node.spin(1.0)   # 间隔
        ok = ok_all

    node.get_logger().info(f"\n测试结果: {'通过 ✓' if ok else '失败 ✗'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
