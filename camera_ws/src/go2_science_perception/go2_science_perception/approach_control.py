"""Pure, fail-closed single-target approach controller (no hardware I/O)."""

from collections import deque
from dataclasses import dataclass
import math


@dataclass
class Settings:
    """Experiment limits; the adapter exposes every field as a ROS parameter."""

    stop_distance: float = 0.8
    distance_tolerance: float = 0.03
    kp_linear: float = 0.5
    kp_angular: float = 0.8
    align_yaw_threshold: float = 0.15
    max_acceleration: float = 0.15
    max_deceleration: float = 0.3
    max_linear_speed: float = 0.3
    max_angular_speed: float = 0.3
    control_rate_hz: float = 10.0
    lock_stability_sec: float = 3.0
    visual_timeout: float = 0.5
    odom_timeout: float = 0.4
    feedback_timeout: float = 0.5
    target_jump_m: float = 0.15
    localization_jump_m: float = 0.3
    localization_max_speed: float = 0.8
    localization_max_yaw_rate: float = 1.0
    max_target_range: float = 5.0
    speed_window_sec: float = 0.5
    braking_latency: float = 0.5
    max_run_time: float = 45.0
    allow_odom: bool = False
    odom_max_time: float = 1.0
    odom_max_distance: float = 0.15

    def __post_init__(self):
        """Reject invalid limits rather than silently weakening safety bounds."""
        for name, value in vars(self).items():
            if name != 'allow_odom' and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be finite and positive')
        if self.max_linear_speed > 0.3 or self.max_angular_speed > 0.3:
            raise ValueError('experiment speed cap is 0.30')
        if self.stop_distance < 0.6 or self.distance_tolerance >= self.stop_distance:
            raise ValueError('unsafe standoff/tolerance')


def angle(value):
    """Normalize radians."""
    return math.atan2(math.sin(value), math.cos(value))


class ApproachControl:
    """Lock one ID on confirmation; STOP/FAULT require reset and confirmation."""

    def __init__(self, settings=None):
        """Initialize without a target or motion permission."""
        self.p = settings or Settings()
        self.reset()

    def reset(self):
        """Clear all previous observations as well as the terminal latch."""
        self.state, self.reason = 'IDLE', ''
        self.robot = self.target = self.lock_id = None
        self.tracks = []
        self.snapshot_time = -math.inf
        self.odom_time = self.feedback_time = self.visual_time = -math.inf
        self.last_tick = self.started = self.lost_at = None
        self.odom_travel = 0.0
        self.actual_speed = self.closing_speed = self.vx = 0.0
        self.pose_history = deque()
        self.initial_distance = None

    def fault(self, reason):
        """Latch a fault; never restart on data recovery."""
        if self.state not in ('STOP', 'FAULT'):
            self.state, self.reason = 'FAULT', reason
        self.vx = 0.0

    def pose(self, x, y, yaw, now):
        """Use transformed body-center positions for velocity and loss travel."""
        if not all(math.isfinite(v) for v in (x, y, yaw, now)):
            self.fault('nonfinite localization')
            return
        if self.robot is not None:
            dt = now - self.odom_time
            delta = math.hypot(x - self.robot[0], y - self.robot[1])
            if (dt <= 0 or delta > self.p.localization_jump_m or
                    delta > self.p.localization_max_speed * dt + 0.02 or
                    abs(angle(yaw - self.robot[2])) >
                    self.p.localization_max_yaw_rate * dt + 0.05):
                self.fault('localization jump or time regression')
                return
            if self.lost_at is not None:
                self.odom_travel += delta
        self.pose_history.append((now, x, y))
        while (len(self.pose_history) > 2 and
               self.pose_history[1][0] <= now - self.p.speed_window_sec):
            self.pose_history.popleft()
        t0, x0, y0 = self.pose_history[0]
        if now > t0:
            self.actual_speed = math.hypot(x - x0, y - y0) / (now - t0)
            if self.target is not None:
                dx, dy = self.target[0] - x, self.target[1] - y
                distance = math.hypot(dx, dy)
                self.closing_speed = ((x - x0) * dx + (y - y0) * dy) / max(
                    distance * (now - t0), 1e-9)
        self.robot, self.odom_time = (x, y, yaw), now

    def observations(self, tracks, now):
        """Consume ID, position, state, visited and original observation age."""
        self.tracks = tracks
        self.snapshot_time = now
        if self.lock_id is None or self.state in ('STOP', 'FAULT'):
            return
        track = next((t for t in tracks if t['id'] == self.lock_id), None)
        if track is None:
            # Removal/merge cannot silently reassign the locked identity.
            self.fault('locked target removed or merged')
            return
        if track['visited']:
            self.fault('locked target marked visited')
            return
        if track['state'] == 'CONFIRMED' and 0 <= track['age'] <= self.p.visual_timeout:
            point = track['position']
            jump = math.hypot(point[0] - self.target[0], point[1] - self.target[1])
            if jump > self.p.target_jump_m:
                self.fault('target position jump')
                return
            self.target = tuple(point[:2])
            self.visual_time = now - track['age']
            self.lost_at = None
            self.odom_travel = 0.0

    def candidate(self, now):
        """Preview the nearest eligible target; refuse an unstable nearest ID."""
        if self.robot is None:
            return None
        candidates = [t for t in self.tracks if t['state'] == 'CONFIRMED'
                      and not t['visited']
                      and 0 <= t['age'] + now - self.snapshot_time <= self.p.visual_timeout]
        if not candidates:
            return None
        return min(candidates, key=lambda t: math.hypot(
            t['position'][0] - self.robot[0], t['position'][1] - self.robot[1]))

    def confirm(self, now, target_id=None):
        """Choose nearest fresh active unvisited ID using the current body pose."""
        if self.state != 'IDLE':
            return False
        if (now - self.odom_time > self.p.odom_timeout or
                now - self.feedback_time > self.p.feedback_timeout):
            return False
        t = self.candidate(now)
        if (t is None or t.get('lifetime', 0.0) < self.p.lock_stability_sec or
                (target_id is not None and target_id != t['id'])):
            return False
        self.lock_id, self.target = t['id'], tuple(t['position'][:2])
        self.visual_time = self.snapshot_time - t['age']
        self.state, self.started = 'LOCK_TARGET', now
        self.initial_distance = math.hypot(self.target[0] - self.robot[0],
                                           self.target[1] - self.robot[1])
        return True

    def step(self, now):
        """Return telemetry plus bounded desired velocity; terminal states stay zero."""
        dt = 1 / self.p.control_rate_hz if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        active = self.state not in ('IDLE', 'STOP', 'FAULT')
        if active:
            if dt <= 0 or dt > self.p.odom_timeout:
                self.fault('control loop deadline')
            elif now - self.odom_time > self.p.odom_timeout:
                self.fault('localization timeout')
            elif now - self.feedback_time > self.p.feedback_timeout:
                self.fault('GO2 feedback timeout')
            elif now - self.started > self.p.max_run_time:
                self.fault('run timeout')
        out = {'state': self.state, 'reason': self.reason, 'vx': 0.0, 'wz': 0.0,
               'target_id': self.lock_id, 'target': self.target, 'robot': self.robot,
               'actual_speed': self.actual_speed, 'eta': None,
               'initial_distance': self.initial_distance,
               'elapsed': now - self.started if self.started is not None else 0.0,
               'visual_lost_at': self.lost_at,
               'candidate': self.candidate(now) if self.state == 'IDLE' else None}
        if self.robot is None or self.target is None:
            return out
        dx, dy = self.target[0] - self.robot[0], self.target[1] - self.robot[1]
        distance = math.hypot(dx, dy)
        remaining = max(0.0, distance - self.p.stop_distance)
        error = angle(math.atan2(dy, dx) - self.robot[2])
        brake = (self.actual_speed ** 2 / (2 * self.p.max_deceleration)
                 + self.actual_speed * self.p.braking_latency)
        out.update(distance=distance, remaining=remaining, yaw_error=error,
                   distance_error=distance - self.p.stop_distance,
                   braking_distance=brake,
                   eta=remaining / self.closing_speed if self.closing_speed > 0.01 else None)
        if self.state not in ('IDLE', 'STOP', 'FAULT'):
            if distance > self.p.max_target_range:
                self.fault('target out of range')
            elif now - self.visual_time > self.p.visual_timeout:
                if self.lost_at is None:
                    self.lost_at = now
                    self.odom_travel = 0.0
                if not self.p.allow_odom:
                    self.fault('visual target lost')
                elif (now - self.lost_at >= self.p.odom_max_time or
                      self.odom_travel >= self.p.odom_max_distance):
                    self.fault('odometry continuation limit')
                else:
                    self.state = 'APPROACH_ODOM'
            if self.state != 'FAULT':
                if remaining <= self.p.distance_tolerance:
                    self.state, self.reason = 'STOP', 'standoff reached'
                else:
                    visual = now - self.visual_time <= self.p.visual_timeout
                    self.state = ('ALIGN' if abs(error) > self.p.align_yaw_threshold
                                  else 'APPROACH_VISUAL' if visual else 'APPROACH_ODOM')
                    # Solve v^2/(2a)+latency*v <= remaining minus tolerance.
                    clearance = max(0.0, remaining - self.p.distance_tolerance / 2)
                    a, latency = self.p.max_deceleration, self.p.braking_latency
                    safe = math.sqrt((a * latency) ** 2 + 2 * a * clearance) - a * latency
                    desired = min(self.p.max_linear_speed, self.p.kp_linear * remaining, safe)
                    if abs(error) > self.p.align_yaw_threshold:
                        desired = 0.0
                    if brake >= clearance:
                        desired = min(desired, max(0.0, self.actual_speed - a * dt))
                    self.vx = min(desired, self.vx + self.p.max_acceleration * dt)
                    out.update(vx=self.vx, wz=max(-self.p.max_angular_speed,
                               min(self.p.max_angular_speed, self.p.kp_angular * error)))
        if self.state in ('STOP', 'FAULT', 'IDLE'):
            self.vx = 0.0
            out.update(vx=0.0, wz=0.0)
        out.update(state=self.state, reason=self.reason, visual_lost_at=self.lost_at)
        return out
