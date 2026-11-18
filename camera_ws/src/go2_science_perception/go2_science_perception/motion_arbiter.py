"""FULL has one velocity publisher and one selected controller at a time."""
import math
import time

from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data


class MotionArbiter:
    sources = {'NAV': 'nav', 'RETURN': 'nav', 'APPROACH': 'science', 'DOCK': 'science',
               'REACQUIRE': 'reacquire'}

    def __init__(self, node):
        self.node = node
        self.mode = 'IDLE'
        self.epoch = node.get_clock().now().nanoseconds
        self.latest = None
        self.enabled = False
        self.guard = None
        self.output = node.create_publisher(TwistStamped, '/cmd_vel_raw', 1)
        for source in ('nav', 'science'):
            node.create_subscription(
                TwistStamped, f'/{source}/cmd_vel_raw',
                lambda msg, src=source: self.receive(src, msg), qos_profile_sensor_data)
        self.timer = node.create_timer(.05, self.tick)

    def select(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.flush()

    def flush(self):
        self.epoch = self.node.get_clock().now().nanoseconds
        self.latest = None
        self.enabled = False
        self.send_zero()

    def receive(self, source, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age_ns = self.node.get_clock().now().nanoseconds - stamp
        twist = msg.twist
        if (self.sources.get(self.mode) == source and stamp > self.epoch
                and 0 <= age_ns < 400_000_000
                and all(math.isfinite(v) for v in (twist.linear.x, twist.linear.y, twist.angular.z))):
            self.latest = (time.monotonic(), msg)

    def send_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'vehicle'
        self.output.publish(msg)

    def tick(self):
        if (self.enabled and self.latest and time.monotonic() - self.latest[0] < .25
                and (self.guard is None or self.guard(self.latest[1]))):
            self.output.publish(self.latest[1])
        else:
            self.send_zero()

    def close(self):
        self.enabled = False
        self.send_zero()
        self.timer.cancel()
        self.node.destroy_publisher(self.output)
