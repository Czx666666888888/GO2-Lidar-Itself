"""Unit tests for the science-to-FAR goal bridge."""

from geometry_msgs.msg import PointStamped

from go2_science_perception.science_goal_bridge import ScienceGoalBridge


class FakePublisher:
    """Record messages published by the callback."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    """Provide the logger methods used by the callback."""

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class FakeBridge:
    """Minimal callback context without creating a ROS node."""

    required_frame = "map"

    def __init__(self):
        self.locked_goal = None
        self.goal_publisher = FakePublisher()

    def get_logger(self):
        return FakeLogger()


def goal(frame="map", x=1.0):
    """Build a standoff goal for callback tests."""
    message = PointStamped()
    message.header.frame_id = frame
    message.point.x = x
    message.point.y = 2.0
    message.point.z = 0.0
    return message


def test_locks_and_publishes_only_first_valid_goal():
    """Later selected-target updates must not reset FAR's goal."""
    bridge = FakeBridge()

    ScienceGoalBridge._goal_callback(bridge, goal(x=1.0))
    ScienceGoalBridge._goal_callback(bridge, goal(x=9.0))

    assert len(bridge.goal_publisher.messages) == 1
    assert bridge.goal_publisher.messages[0].point.x == 1.0
    assert bridge.locked_goal.point.x == 1.0


def test_rejects_wrong_frame_and_non_finite_goal():
    """Only finite map-frame points are allowed onto /goal_point."""
    bridge = FakeBridge()

    ScienceGoalBridge._goal_callback(bridge, goal(frame="camera_link"))
    ScienceGoalBridge._goal_callback(bridge, goal(x=float("nan")))

    assert bridge.locked_goal is None
    assert bridge.goal_publisher.messages == []
