"""Unit tests for the manually gated science-to-FAR goal bridge."""

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from go2_science_perception.science_goal_bridge import ScienceGoalBridge
from go2_science_perception.science_goal_bridge import point_is_finite
from go2_science_perception.science_goal_bridge import stamp_ns


class FakePublisher:
    """Record messages published by a callback."""

    def __init__(self):
        """Initialize an empty publication record."""
        self.messages = []

    def publish(self, message):
        """Record one published message."""
        self.messages.append(message)


class FakeLogger:
    """Record bridge log messages."""

    def __init__(self):
        """Initialize an empty log record."""
        self.messages = []

    def info(self, message):
        """Record an informational message."""
        self.messages.append(("info", message))

    def warning(self, message):
        """Record a warning message."""
        self.messages.append(("warning", message))

    def error(self, message):
        """Record an error message."""
        self.messages.append(("error", message))


class FakeBridge:
    """Minimal callback context without creating a ROS node."""

    _goal_callback = ScienceGoalBridge._goal_callback
    _odom_callback = ScienceGoalBridge._odom_callback
    _message_is_fresh = ScienceGoalBridge._message_is_fresh
    _validate_confirmation = ScienceGoalBridge._validate_confirmation
    _confirm_callback = ScienceGoalBridge._confirm_callback
    _cancel_callback = ScienceGoalBridge._cancel_callback
    _invalidate_locked_goal = ScienceGoalBridge._invalidate_locked_goal
    _health_check = ScienceGoalBridge._health_check

    def __init__(self):
        """Initialize bridge state with deterministic time and thresholds."""
        self.now_ns = 10_000_000_000
        self.required_frame = "map"
        self.max_goal_age_ns = 750_000_000
        self.max_odom_age_ns = 750_000_000
        self.future_tolerance_ns = 200_000_000
        self.odom_regression_tolerance_ns = 100_000_000
        self.localization_jump_m = 0.5
        self.latest_goal = None
        self.latest_goal_received_ns = 0
        self.latest_odom = None
        self.latest_odom_received_ns = 0
        self.last_odom_stamp_ns = None
        self.last_odom_position = None
        self.localization_epoch = 0
        self.locked_goal = None
        self.locked_epoch = None
        self.goal_publisher = FakePublisher()
        self.far_reset_publisher = FakePublisher()
        self.logger = FakeLogger()

    def _now_ns(self):
        return self.now_ns

    def get_logger(self):
        """Return the fake logger."""
        return self.logger


def set_stamp(stamp, nanoseconds):
    """Set a builtin_interfaces timestamp from integer nanoseconds."""
    stamp.sec = nanoseconds // 1_000_000_000
    stamp.nanosec = nanoseconds % 1_000_000_000


def goal(now_ns=10_000_000_000, frame="map", x=1.0):
    """Build a standoff goal for callback tests."""
    message = PointStamped()
    message.header.frame_id = frame
    set_stamp(message.header.stamp, now_ns)
    message.point.x = x
    message.point.y = 2.0
    message.point.z = 0.0
    return message


def odom(now_ns=10_000_000_000, frame="map", x=0.0):
    """Build a localization sample for callback tests."""
    message = Odometry()
    message.header.frame_id = frame
    set_stamp(message.header.stamp, now_ns)
    message.pose.pose.position.x = x
    return message


def ready_bridge():
    """Create a bridge with fresh goal and localization samples."""
    bridge = FakeBridge()
    bridge._odom_callback(odom())
    bridge._goal_callback(goal())
    return bridge


def test_receiving_goal_never_publishes_without_manual_confirmation():
    """Startup or transient detections cannot automatically trigger FAR."""
    bridge = FakeBridge()
    bridge._odom_callback(odom())
    bridge._goal_callback(goal())

    assert bridge.goal_publisher.messages == []
    assert bridge.locked_goal is None


def test_manual_confirmation_locks_and_publishes_once():
    """A fresh, finite map goal is published only after confirmation."""
    bridge = ready_bridge()

    bridge._confirm_callback(Empty())
    bridge._goal_callback(goal(x=9.0))
    bridge._confirm_callback(Empty())

    assert len(bridge.goal_publisher.messages) == 1
    assert bridge.goal_publisher.messages[0].point.x == 1.0
    assert bridge.locked_goal.point.x == 1.0


def test_confirmation_rejects_stale_wrong_frame_and_non_finite_goal():
    """Confirmation rechecks timestamp, frame and coordinate validity."""
    bridge = ready_bridge()
    bridge.latest_goal.header.frame_id = "camera_link"
    bridge._confirm_callback(Empty())
    bridge.latest_goal = goal(x=float("nan"))
    bridge._confirm_callback(Empty())
    bridge.latest_goal = goal(now_ns=8_000_000_000)
    bridge.latest_goal_received_ns = bridge.now_ns
    bridge._confirm_callback(Empty())

    assert bridge.goal_publisher.messages == []
    assert bridge.locked_goal is None
    assert point_is_finite(goal().point)
    assert stamp_ns(goal().header.stamp) == bridge.now_ns


def test_cancel_resets_far_and_requires_a_new_goal_before_reconfirming():
    """Manual cancellation clears both bridge and FAR state."""
    bridge = ready_bridge()
    bridge._confirm_callback(Empty())

    bridge._cancel_callback(Empty())
    bridge._confirm_callback(Empty())

    assert bridge.locked_goal is None
    assert bridge.latest_goal is None
    assert len(bridge.far_reset_publisher.messages) == 1
    assert len(bridge.goal_publisher.messages) == 1


def test_localization_jump_invalidates_lock_and_discards_old_goal():
    """A likely map reset prevents reuse of a previously published target."""
    bridge = ready_bridge()
    bridge._confirm_callback(Empty())
    bridge.now_ns += 100_000_000

    bridge._odom_callback(odom(now_ns=bridge.now_ns, x=2.0))

    assert bridge.localization_epoch == 1
    assert bridge.locked_goal is None
    assert bridge.latest_goal is None
    assert len(bridge.far_reset_publisher.messages) == 1


def test_stale_localization_invalidates_locked_goal():
    """Point-LIO loss automatically resets FAR and the cached target."""
    bridge = ready_bridge()
    bridge._confirm_callback(Empty())
    bridge.now_ns += bridge.max_odom_age_ns + 1

    bridge._health_check()

    assert bridge.locked_goal is None
    assert bridge.latest_goal is None
    assert len(bridge.far_reset_publisher.messages) == 1
