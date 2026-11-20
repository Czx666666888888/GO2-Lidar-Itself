"""Time-bounded initial-heading motion using IMU heading and stop feedback."""
import math

from .controller import STOP

SPEED_CAP = 1.0
MIN_CRUISE_SPEED = 0.80
START_SPEED = 0.10
COMMAND_ACCEL = 1.0
START_DELAY = 1.0
MAX_DURATION = 10.0
SETTLE_SECONDS = 0.50


class TimedStraight:
    def __init__(self, seconds, ready):
        if not math.isfinite(seconds) or not 0 < seconds <= MAX_DURATION:
            raise ValueError("直行时长必须大于 0 且不超过 10 秒")
        self.seconds = seconds
        self.ready = ready
        self.phase = "WAIT_CONFIRM"
        self.reason = ""
        self.confirmed_at = None
        self.still_since = None

    def halt(self, reason):
        self.phase = "HALTED"
        self.reason = reason
        return STOP

    def preflight_issue(self, scan, still, standing):
        if not standing:
            return "计时直行需要站立"
        if not still:
            return "计时直行起点未停稳"
        return ""

    def update(self, now, observed_at, scan, yaw, still, standing, speed_mps=0.0):
        if not standing:
            return self.halt("机身站立反馈丢失")
        if self.phase == "DRIVE" and now >= self.deadline:
            self.phase = "SETTLE"
            self.stop_at = now
            return STOP
        if self.phase == "SETTLE":
            if now - self.stop_at >= 3.0:
                return self.halt("计时直行结束后未确认停稳")
            if not still or observed_at <= self.stop_at:
                self.still_since = None
            elif self.still_since is None:
                self.still_since = observed_at
            elif observed_at - self.still_since >= SETTLE_SECONDS:
                self.phase = "DONE"
            return STOP
        speed = SPEED_CAP
        if self.phase == "WAIT_CONFIRM":
            if not still:
                return self.halt("计时直行起点未停稳")
            if self.confirmed_at is None or now < self.confirmed_at + START_DELAY:
                return STOP
            self.initial_yaw = yaw
            self.deadline = now + self.seconds
            self.phase = "DRIVE"
        error = self.initial_yaw - yaw
        if abs(error) > math.radians(12):
            return self.halt("计时直行航向偏差过大")
        return speed, max(-0.20, min(0.20, 1.5 * error))
