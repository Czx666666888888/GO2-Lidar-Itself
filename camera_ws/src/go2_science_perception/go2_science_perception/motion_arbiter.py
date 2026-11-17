"""Single cmd_vel_raw publisher; gates NAV vs APPROACH velocity sources."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


# Modes published by MissionManager on /mission/motion_mode
NAV = 'NAV'
APPROACH = 'APPROACH'
IDLE = 'IDLE'

_CMD_TIMEOUT = 0.4  # seconds; stale command treated as zero


class MotionArbiter(Node):
    """Pass nav or approach velocity to /cmd_vel_raw; never both simultaneously."""

    def __init__(self):
        super().__init__('motion_arbiter')
        self.declare_parameter('cmd_timeout', _CMD_TIMEOUT)
        self._timeout = float(self.get_parameter('cmd_timeout').value)

        self._mode = IDLE
        self._nav_vel: TwistStamped | None = None
        self._nav_stamp = -math.inf
        self._sci_vel: TwistStamped | None = None
        self._sci_stamp = -math.inf

        self._pub = self.create_publisher(TwistStamped, '/cmd_vel_raw', 1)

        self.create_subscription(
            TwistStamped, '/nav/cmd_vel_raw', self._nav_cb, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/science/cmd_vel_raw', self._sci_cb, qos_profile_sensor_data)
        self.create_subscription(
            String, '/mission/motion_mode', self._mode_cb, 10)

        self.create_timer(0.05, self._tick)   # 20 Hz pass-through
        self.get_logger().info('MotionArbiter ready; mode=IDLE')

    def _nav_cb(self, msg: TwistStamped):
        self._nav_vel = msg
        self._nav_stamp = time.monotonic()

    def _sci_cb(self, msg: TwistStamped):
        self._sci_vel = msg
        self._sci_stamp = time.monotonic()

    def _mode_cb(self, msg: String):
        new = msg.data.strip().upper()
        if new not in (NAV, APPROACH, IDLE):
            self.get_logger().warning(f'Unknown mode {new!r}; ignoring')
            return
        if new != self._mode:
            self.get_logger().info(f'mode {self._mode} -> {new}')
            self._mode = new

    def _fresh(self, stamp: float) -> bool:
        return time.monotonic() - stamp < self._timeout

    def _zero(self) -> TwistStamped:
        return TwistStamped()

    def _tick(self):
        if self._mode == NAV:
            out = self._nav_vel if (self._nav_vel and self._fresh(self._nav_stamp)) else self._zero()
        elif self._mode == APPROACH:
            out = self._sci_vel if (self._sci_vel and self._fresh(self._sci_stamp)) else self._zero()
        else:
            out = self._zero()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
