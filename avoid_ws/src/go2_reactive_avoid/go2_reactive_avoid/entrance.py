"""Relative entrance motion using native Sport odometry."""
import math

from .controller import ANGLE_TOLERANCE, STOP

DISTANCE_TOLERANCE = 0.03
PATH_TOLERANCE = 0.08
HEADING_TOLERANCE = math.radians(15)
SETTLE_SECONDS = 0.50
MIN_FORWARD_SPEED = 0.10
MAX_FORWARD_SPEED = 0.20


class Entrance:
    def __init__(self, distance_cm, clockwise_deg):
        self.distance = distance_cm / 100.0
        self.turn = -math.radians(clockwise_deg)
        self.phase = "READY"
        self.reason = ""
        self.still_since = None

    def halt(self, reason):
        self.phase = "HALTED"
        self.reason = reason
        return STOP

    def settled(self, observed_at, still):
        if not still:
            self.still_since = None
            return False
        if self.still_since is None:
            self.still_since = observed_at
        return observed_at - self.still_since >= SETTLE_SECONDS

    def update(self, now, observed_at, yaw, still, position):
        if self.phase == "READY":
            if not still:
                return self.halt("洞口段起点未停稳")
            self.origin = position
            self.initial_yaw = yaw
            self.forward = (math.cos(yaw), math.sin(yaw))
            self.deadline = now + self.distance / MIN_FORWARD_SPEED + 6.0
            self.phase = "DRIVE"
        dx = position[0] - self.origin[0]
        dy = position[1] - self.origin[1]
        along = dx * self.forward[0] + dy * self.forward[1]
        cross = -dx * self.forward[1] + dy * self.forward[0]
        remaining = self.distance - along
        if abs(cross) > PATH_TOLERANCE:
            return self.halt(f"洞口段横向偏差过大 {cross:+.3f} m")
        if self.phase == "DRIVE":
            yaw_error = self.initial_yaw - yaw
            if now >= self.deadline or abs(yaw_error) > HEADING_TOLERANCE:
                return self.halt(f"洞口直行超时或航向偏差过大，剩余 {remaining:+.3f} m")
            if remaining < -DISTANCE_TOLERANCE or (self.distance == 0 and remaining > DISTANCE_TOLERANCE):
                return self.halt(f"洞口段位置超出到位容差，剩余 {remaining:+.3f} m")
            if remaining <= DISTANCE_TOLERANCE:
                if self.settled(observed_at, still):
                    self.reason = f"直行 {along:.3f} m，剩余 {remaining:+.3f} m，横向 {cross:+.3f} m"
                    if self.turn == 0.0:
                        self.phase = "DONE"
                    else:
                        self.phase = "TURN"
                        self.turn_origin = yaw
                        self.deadline = now + abs(self.turn) / 0.18 + 6.0
                        self.still_since = None
                return STOP
            self.still_since = None
            speed = max(MIN_FORWARD_SPEED, min(MAX_FORWARD_SPEED, 0.9 * remaining))
            return speed, max(-0.20, min(0.20, 1.5 * yaw_error))
        if self.phase == "TURN":
            error = self.turn_origin + self.turn - yaw
            if abs(remaining) > PATH_TOLERANCE or now >= self.deadline:
                return self.halt(f"洞口转向位置漂移或超时，剩余 {math.degrees(error):+.1f}°")
            if abs(error) <= ANGLE_TOLERANCE:
                if self.settled(observed_at, still):
                    self.phase = "DONE"
                    self.reason += (f"，转向 {math.degrees(yaw - self.turn_origin):+.1f}°"
                                    f"，剩余角误差 {math.degrees(error):+.1f}°")
                return STOP
            self.still_since = None
            return 0.0, math.copysign(max(0.18, min(0.40, 1.5 * abs(error))), error)
        return STOP
