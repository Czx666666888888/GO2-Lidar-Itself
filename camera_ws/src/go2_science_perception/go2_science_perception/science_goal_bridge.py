"""Lock the first science standoff goal and forward it to FAR once."""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class ScienceGoalBridge(Node):
    """Publish exactly one finite map-frame science goal to FAR."""

    def __init__(self):
        super().__init__("science_goal_bridge")
        self.declare_parameter("input_topic", "/science/standoff_goal")
        self.declare_parameter("output_topic", "/goal_point")
        self.declare_parameter("required_frame", "map")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.required_frame = self.get_parameter("required_frame").value
        self.goal_publisher = self.create_publisher(PointStamped, output_topic, 1)
        self.create_subscription(PointStamped, input_topic, self._goal_callback, 10)
        self.locked_goal = None
        self.get_logger().info(
            f"Waiting to lock first goal: {input_topic} -> {output_topic}; "
            "publishes once and ignores later target changes"
        )

    def _goal_callback(self, message):
        if self.locked_goal is not None:
            return
        if message.header.frame_id != self.required_frame:
            self.get_logger().warning(
                f"Rejecting goal in frame '{message.header.frame_id}'; "
                f"required '{self.required_frame}'"
            )
            return
        values = (message.point.x, message.point.y, message.point.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning("Rejecting non-finite science goal")
            return

        self.locked_goal = PointStamped()
        self.locked_goal.header = message.header
        self.locked_goal.point = message.point
        self.goal_publisher.publish(self.locked_goal)
        self.get_logger().info(
            "LOCKED_AND_PUBLISHED "
            f"frame={message.header.frame_id} "
            f"x={message.point.x:.3f} y={message.point.y:.3f} "
            f"z={message.point.z:.3f}"
        )


def main(args=None):
    """Run the science goal bridge."""
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
