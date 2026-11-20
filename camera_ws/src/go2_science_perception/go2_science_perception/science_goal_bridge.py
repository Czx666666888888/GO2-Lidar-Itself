"""Manually confirm a fresh science standoff goal before forwarding to FAR."""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener


def stamp_ns(stamp):
    """Convert a ROS timestamp message to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def point_is_finite(point):
    """Return whether every point coordinate is finite."""
    return all(math.isfinite(value) for value in (point.x, point.y, point.z))


def xyz_distance(first, second):
    """Return the Euclidean distance between two geometry points."""
    return math.sqrt(
        (first.x - second.x) ** 2
        + (first.y - second.y) ** 2
        + (first.z - second.z) ** 2
    )


class ScienceGoalBridge(Node):
    """Forward one manually confirmed, fresh map goal and support cancellation."""

    def __init__(self):
        """Create guarded subscriptions, publishers and localization state."""
        super().__init__("science_goal_bridge")
        defaults = {
            "input_topic": "/science/standoff_goal",
            "odom_topic": "/base_state_estimation",
            "confirm_topic": "/science/confirm_selected_goal",
            "cancel_topic": "/science/cancel_goal",
            "output_topic": "/goal_point",
            "far_reset_topic": "/reset_visibility_graph",
            "required_frame": "map",
            "max_goal_age_sec": 0.75,
            "max_odom_age_sec": 0.75,
            "future_tolerance_sec": 0.20,
            "odom_regression_tolerance_sec": 0.10,
            "localization_jump_m": 0.50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.required_frame = str(self.get_parameter("required_frame").value)
        self.max_goal_age_ns = int(
            float(self.get_parameter("max_goal_age_sec").value) * 1e9)
        self.max_odom_age_ns = int(
            float(self.get_parameter("max_odom_age_sec").value) * 1e9)
        self.future_tolerance_ns = int(
            float(self.get_parameter("future_tolerance_sec").value) * 1e9)
        self.odom_regression_tolerance_ns = int(
            float(self.get_parameter("odom_regression_tolerance_sec").value) * 1e9)
        self.localization_jump_m = float(
            self.get_parameter("localization_jump_m").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_publisher = self.create_publisher(
            PointStamped, self.get_parameter("output_topic").value, 1)
        self.far_reset_publisher = self.create_publisher(
            Empty, self.get_parameter("far_reset_topic").value, 1)
        self.create_subscription(
            PointStamped, self.get_parameter("input_topic").value,
            self._goal_callback, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._odom_callback, 10)
        self.create_subscription(
            Empty, self.get_parameter("confirm_topic").value,
            self._confirm_callback, 10)
        self.create_subscription(
            Empty, self.get_parameter("cancel_topic").value,
            self._cancel_callback, 10)
        self.create_timer(0.10, self._health_check)

        self.latest_goal = None
        self.latest_goal_received_ns = 0
        self.latest_odom = None
        self.latest_odom_received_ns = 0
        self.last_odom_stamp_ns = None
        self.last_odom_position = None
        self.localization_epoch = 0
        self.locked_goal = None
        self.locked_epoch = None
        self.get_logger().info(
            "WAITING_FOR_MANUAL_CONFIRMATION: no goal is published automatically")

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _goal_callback(self, message):
        if self.locked_goal is not None:
            return
        self.latest_goal = message
        self.latest_goal_received_ns = self._now_ns()

    def _odom_callback(self, message):
        now_ns = self._now_ns()
        current_stamp_ns = stamp_ns(message.header.stamp)
        current_position = message.pose.pose.position
        invalid_reason = None
        if self.last_odom_stamp_ns is not None:
            if current_stamp_ns < (
                    self.last_odom_stamp_ns
                    - self.odom_regression_tolerance_ns):
                invalid_reason = "odometry timestamp regressed"
            elif (self.last_odom_position is not None
                  and xyz_distance(current_position, self.last_odom_position)
                  > self.localization_jump_m):
                invalid_reason = "odometry position jumped"
        if invalid_reason is not None:
            self.localization_epoch += 1
            self.latest_goal = None
            self.latest_goal_received_ns = 0
            self._invalidate_locked_goal(invalid_reason)
            self.get_logger().error(
                f"LOCALIZATION_RESET epoch={self.localization_epoch}: "
                f"{invalid_reason}; waiting for a fresh standoff goal")

        self.latest_odom = message
        self.latest_odom_received_ns = now_ns
        self.last_odom_stamp_ns = current_stamp_ns
        self.last_odom_position = current_position

    def _message_is_fresh(self, message, received_ns, max_receive_age_ns, name):
        now_ns = self._now_ns()
        message_stamp_ns = stamp_ns(message.header.stamp)
        if message_stamp_ns <= 0:
            return False, f"{name} timestamp is zero"
        receive_age_ns = now_ns - received_ns
        if receive_age_ns < 0 or receive_age_ns > max_receive_age_ns:
            return False, f"{name} receive age is stale"
        stamp_age_ns = now_ns - message_stamp_ns
        if stamp_age_ns < -self.future_tolerance_ns:
            return False, f"{name} timestamp is in the future"
        if stamp_age_ns > max_receive_age_ns:
            return False, f"{name} timestamp is stale"
        return True, ""

    def _validate_confirmation(self):
        if self.locked_goal is not None:
            return False, "a goal is already locked; cancel it before reselecting"
        if self.latest_goal is None:
            return False, "no fresh selected standoff goal"
        if self.latest_odom is None:
            return False, "no localization sample"
        if self.latest_goal.header.frame_id != self.required_frame:
            return False, (
                f"goal frame {self.latest_goal.header.frame_id!r} is not "
                f"{self.required_frame!r}")
        if not point_is_finite(self.latest_goal.point):
            return False, "goal coordinates are non-finite"
        goal_fresh, reason = self._message_is_fresh(
            self.latest_goal, self.latest_goal_received_ns,
            self.max_goal_age_ns, "goal")
        if not goal_fresh:
            return False, reason
        odom_fresh, reason = self._message_is_fresh(
            self.latest_odom, self.latest_odom_received_ns,
            self.max_odom_age_ns, "odometry")
        if not odom_fresh:
            return False, reason
        odom_frame = self.latest_odom.header.frame_id
        if not odom_frame:
            return False, "odometry frame is empty"
        if odom_frame != self.required_frame:
            try:
                self.tf_buffer.lookup_transform(
                    self.required_frame, odom_frame,
                    rclpy.time.Time.from_msg(self.latest_odom.header.stamp),
                    timeout=Duration(seconds=0.0))
            except TransformException as exc:
                return False, f"localization TF unavailable: {exc}"
        return True, ""

    def _confirm_callback(self, _message):
        valid, reason = self._validate_confirmation()
        if not valid:
            self.get_logger().warning(f"CONFIRM_REJECTED: {reason}")
            return
        self.locked_goal = PointStamped()
        self.locked_goal.header = self.latest_goal.header
        self.locked_goal.point = self.latest_goal.point
        self.locked_epoch = self.localization_epoch
        self.goal_publisher.publish(self.locked_goal)
        self.get_logger().info(
            "CONFIRMED_LOCKED_AND_PUBLISHED "
            f"epoch={self.locked_epoch} frame={self.locked_goal.header.frame_id} "
            f"x={self.locked_goal.point.x:.3f} "
            f"y={self.locked_goal.point.y:.3f} "
            f"z={self.locked_goal.point.z:.3f}")

    def _cancel_callback(self, _message):
        self._invalidate_locked_goal("manual cancel", reset_far=True)
        self.latest_goal = None
        self.latest_goal_received_ns = 0
        self.get_logger().info(
            "GOAL_CANCELLED: waiting for a new standoff before re-confirmation")

    def _invalidate_locked_goal(self, reason, reset_far=True):
        if self.locked_goal is None:
            return
        self.locked_goal = None
        self.locked_epoch = None
        if reset_far:
            self.far_reset_publisher.publish(Empty())
        self.get_logger().error(f"LOCK_INVALIDATED: {reason}")

    def _health_check(self):
        if self.locked_goal is None:
            return
        now_ns = self._now_ns()
        if (self.latest_odom is None
                or now_ns - self.latest_odom_received_ns > self.max_odom_age_ns):
            self._invalidate_locked_goal("localization stream is stale")
            self.latest_goal = None
            self.latest_goal_received_ns = 0


def main(args=None):
    """Run the manually gated science goal bridge."""
    rclpy.init(args=args)
    node = ScienceGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
