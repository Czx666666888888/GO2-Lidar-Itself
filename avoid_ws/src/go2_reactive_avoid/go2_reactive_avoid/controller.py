"""Lidar-only decisions and feedback turns, independent of ROS and the Sport API."""
from dataclasses import dataclass
from enum import Enum
from math import degrees, pi, radians


class Direction(Enum):
    UNKNOWN = "?"
    CLEAR = "clear"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class Scan:
    front: Direction
    right: Direction
    left: Direction
    rear: Direction
    can_rotate: bool
    body_clear: bool
    right_near: bool = False
    left_near: bool = False
    front_gap: float = float("inf")
    timed_front_obstacle: float = float("inf")
    timed_ground_coverage: float = 0.0

    @property
    def timed_front_gap(self):
        return min(self.timed_front_obstacle, self.timed_ground_coverage)


STOP = (0.0, 0.0)
ANGLE_TOLERANCE = radians(3)
SETTLE_TIME = 0.25
FRONT_STOP_GAP = 0.25
CRUISE_SPEED = 0.20
MICRO_SPEED = CRUISE_SPEED
SLOW_SPEED = 0.20
MAX_YAW_RATE = 0.60
MICRO_YAW_RATE = 0.50
MIN_YAW_RATE = 0.22


class Explorer:
    def __init__(self):
        self.phase = "BRAKE"
        self.reason = ""
        self.still_since = None
        self.target = 0.0
        self.deadline = 0.0
        self.turn_direction = 0
        self.turn_label = ""
        self.completed_turn = ""
        self.uturn_remaining = False
        self.uturn_done = False
        self.micro_used = False

    def halt(self, reason):
        self.phase = "HALTED"
        self.reason = reason
        return STOP

    def brake(self):
        self.phase = "BRAKE"
        self.still_since = None
        return STOP

    def settled(self, now, observed_at, still):
        if not still:
            self.still_since = None
            return False
        if self.still_since is None:
            self.still_since = now
        return observed_at >= self.still_since + SETTLE_TIME

    def begin_turn(self, now, yaw, direction, kind, continue_target=False):
        angle = 20 if kind == "MICRO" else 90
        origin = self.target if continue_target else yaw
        self.phase = kind
        self.turn_direction = direction
        self.target = origin + direction * radians(angle)
        self.deadline = now + (5.0 if kind == "MICRO" else 10.0)
        self.still_since = None
        self.turn_label = f"{angle}° {'左' if direction > 0 else '右'}"
        return STOP

    def turn_command(self, error):
        micro = self.phase == "MICRO"
        limit = MICRO_YAW_RATE if micro else MAX_YAW_RATE
        rate = (1 if error > 0 else -1) * min(limit, max(MIN_YAW_RATE, 1.6 * abs(error)))
        vx = MICRO_SPEED if micro else 0.0
        return (vx, rate)

    def update(self, now, observed_at, scan, yaw, still):
        if self.phase == "HALTED":
            return STOP
        if self.phase in ("TURN", "MICRO"):
            if now >= self.deadline:
                return self.halt(f"{self.turn_label}转角未在时限内收敛，剩余 {degrees(self.target - yaw):+.1f}°")
            if self.phase == "MICRO" and scan.front is not Direction.CLEAR:
                return self.brake()
            error = self.target - yaw
            if abs(error) <= ANGLE_TOLERANCE:
                if self.phase == "MICRO":
                    self.completed_turn = f"{self.turn_label}接续直行，切换时反馈误差 {degrees(error):+.1f}°"
                    self.phase = "DRIVE"
                    return (CRUISE_SPEED, 0.0)
                if self.settled(now, observed_at, still):
                    self.completed_turn = f"{self.turn_label}完成，反馈误差 {degrees(error):+.1f}°"
                    self.brake()
                return STOP
            self.still_since = None
            return self.turn_command(error)

        if self.phase == "BRAKE":
            if not self.settled(now, observed_at, still):
                return STOP
            if self.uturn_remaining:
                self.uturn_remaining = False
                self.uturn_done = True
                return self.begin_turn(now, yaw, self.turn_direction, "TURN", continue_target=True)
            if self.uturn_done:
                if scan.front is not Direction.CLEAR:
                    return self.halt("掉头完成，返向仍有障碍或视野不足")
                self.uturn_done = False
            if scan.front is Direction.UNKNOWN:
                return STOP
            if scan.front is Direction.BLOCKED:
                if scan.right is Direction.CLEAR:
                    return self.begin_turn(now, yaw, -1, "TURN")
                if scan.right is Direction.UNKNOWN:
                    return STOP
                if scan.left is Direction.CLEAR:
                    return self.begin_turn(now, yaw, 1, "TURN")
                if scan.left is Direction.BLOCKED:
                    self.uturn_remaining = True
                    return self.begin_turn(now, yaw, -1, "TURN")
                return STOP
            self.phase = "DRIVE"

        if scan.front is not Direction.CLEAR:
            return self.brake()
        if not scan.left_near and not scan.right_near:
            self.micro_used = False
        if not self.micro_used:
            if scan.left_near and not scan.right_near and scan.right is Direction.CLEAR:
                self.micro_used = True
                self.begin_turn(now, yaw, -1, "MICRO")
                return self.turn_command(self.target - yaw)
            if scan.right_near and not scan.left_near and scan.left is Direction.CLEAR:
                self.micro_used = True
                self.begin_turn(now, yaw, 1, "MICRO")
                return self.turn_command(self.target - yaw)
        return (CRUISE_SPEED, 0.0)


def yaw_delta(current, previous):
    return (current - previous + pi) % (2 * pi) - pi
