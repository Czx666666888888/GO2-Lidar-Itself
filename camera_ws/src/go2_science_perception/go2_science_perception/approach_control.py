"""Pure, fail-closed single-target approach controller (no hardware I/O)."""

from collections import deque
from dataclasses import dataclass
import math
from statistics import median


@dataclass
class Settings:
    """Experiment limits; the adapter exposes every field as a ROS parameter."""

    stop_reference: str = 'body_center'
    stop_xy_tolerance: float = 0.10
    anchor_window_sec: float = 2.0
    anchor_min_observations: int = 5
    anchor_max_spread: float = 0.08
    anchor_association_radius: float = 0.20
    anchor_alpha: float = 0.15
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
    search_enabled: bool = False
    search_speed: float = 0.20
    search_distance: float = 1.0
    search_timeout: float = 20.0
    search_cross_track_limit: float = 0.20
    search_trigger_range_m: float = 1.8
    search_trigger_observations: int = 3
    stop_confirm_timeout: float = 6.0
    stop_confirm_stable_sec: float = 0.20
    stop_confirm_min_span_sec: float = 0.60
    stop_confirm_speed_mps: float = 0.05
    stop_confirm_yaw_rate: float = 0.10
    search_cooldown_sec: float = 0.75
    confirmation_max_gap_sec: float = 0.30

    def __post_init__(self):
        """Reject invalid limits rather than silently weakening safety bounds."""
        for name, value in vars(self).items():
            if (name not in ('allow_odom', 'stop_reference', 'search_enabled') and
                    (not math.isfinite(value) or value <= 0)):
                raise ValueError(f'{name} must be finite and positive')
        if self.max_linear_speed > 0.3 or self.max_angular_speed > 0.3:
            raise ValueError('experiment speed cap is 0.30')
        if self.stop_reference not in ('body_center', 'camera_origin'):
            raise ValueError('unknown stop reference')
        if self.anchor_alpha > 1:
            raise ValueError('anchor_alpha must be <= 1')
        if self.search_enabled and self.search_speed > min(0.30, self.max_linear_speed):
            raise ValueError('search speed exceeds motion cap')
        if self.search_enabled and self.stop_reference != 'camera_origin':
            raise ValueError('SEARCH requires camera_origin')
        if self.search_enabled and (not isinstance(self.search_trigger_observations, int) or
                                    self.search_trigger_observations < 2 or
                                    self.anchor_min_observations < 5 or
                                    self.anchor_window_sec > 1.0 or
                                    self.stop_confirm_timeout > 6.0 or
                                    self.anchor_max_spread > 0.08 or
                                    self.search_trigger_range_m > 1.8 or
                                    self.stop_confirm_min_span_sec > self.anchor_window_sec):
            raise ValueError('SEARCH stop-confirm limits must not be weakened')
        if self.stop_reference == 'body_center' and (
                self.stop_distance < 0.6 or self.distance_tolerance >= self.stop_distance):
            raise ValueError('unsafe standoff/tolerance')


def angle(value):
    """Normalize radians."""
    return math.atan2(math.sin(value), math.cos(value))


class ApproachControl:
    """Lock one ID on confirmation; STOP/FAULT require reset and confirmation."""

    def __init__(self, settings=None, docking=None):
        """Initialize without a target or motion permission."""
        self.p = settings or Settings()
        self.docking = docking
        self.reset()

    def reset(self):
        """Clear all previous observations as well as the terminal latch."""
        if self.docking is not None:
            self.docking.reset()
        self.state, self.reason = 'IDLE', ''
        self.robot = self.target = self.lock_id = None
        self.tracks = []
        self.snapshot_time = -math.inf
        self.odom_time = self.feedback_time = self.visual_time = -math.inf
        self.last_tick = self.started = self.lost_at = None
        self.odom_travel = 0.0
        self.actual_speed = self.closing_speed = self.vx = 0.0
        self.vy = 0.0
        self.pose_history = deque()
        self.initial_distance = None
        self.camera = None
        self.camera_time = -math.inf
        self.camera_history = deque()
        self.camera_speed = self.camera_closing_speed = 0.0
        self.anchor_initial = self.anchor_lock_camera = None
        self.histories = {}
        self.last_observed_ns = {}
        self.anchor_stamp = -math.inf
        self.anchor_updates = self.anchor_rejections = 0
        self.association_diagnostics = []
        self.search_origin = None
        self.search_started = self.first_seen = self.confirmed_at = None
        self.confirmation_spread = self.confirmation_count = None
        self.first_seen_stamp_ns = None
        self.feedback_speed = self.feedback_yaw_rate = math.inf
        self.pose_yaw_rate = 0.0
        self.yaw_history = deque()
        self.trigger_history = {}
        self.search_cooldown_until = -math.inf
        self.stop_candidate = None
        self.stop_confirm_started = self.stationary_since = self.settled_at = None
        self.stop_samples = deque()
        self.stop_sample_stamps = set()
        self.stop_confirm_reason = ''
        self.stop_confirm_attempts = []
        self.search_first_detection_range_m = None
        self.stop_request_range_m = self.stop_stable_range_m = None
        self.stopped_after_sec = self.stationary_confirmation_sec = None
        self.visual_loss_count = 0

    def fault(self, reason):
        """Latch a fault; never restart on data recovery."""
        if self.state not in ('STOP', 'FAULT'):
            self.state, self.reason = 'FAULT', reason
        self.vx = 0.0
        self.vy = 0.0

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
        self.yaw_history.append((now, yaw))
        while len(self.yaw_history) > 2 and self.yaw_history[1][0] <= now-self.p.speed_window_sec:
            self.yaw_history.popleft()
        if now > self.yaw_history[0][0]:
            self.pose_yaw_rate = abs(angle(yaw-self.yaw_history[0][1])) / (
                now-self.yaw_history[0][0])
        self.robot, self.odom_time = (x, y, yaw), now

    def camera_pose(self, x, y, now):
        """Consume the actual timestamped map-to-camera TF translation."""
        if not all(math.isfinite(v) for v in (x, y, now)):
            self.fault('invalid camera TF')
            return
        self.camera, self.camera_time = (x, y), now
        self.camera_history.append((now, x, y))
        while (len(self.camera_history) > 2 and
               self.camera_history[1][0] <= now - self.p.speed_window_sec):
            self.camera_history.popleft()
        t0, x0, y0 = self.camera_history[0]
        if now > t0:
            self.camera_speed = math.hypot(x-x0, y-y0) / (now-t0)
            if self.target is not None:
                dx, dy = self.target[0]-x, self.target[1]-y
                self.camera_closing_speed = ((x-x0)*dx + (y-y0)*dy) / max(
                    math.hypot(dx, dy)*(now-t0), 1e-9)

    def reference(self):
        """Return the selected stopping reference in map XY."""
        return self.camera if self.p.stop_reference == 'camera_origin' else (
            self.robot[:2] if self.robot is not None else None)

    def observations(self, tracks, now):
        """Keep a spatial anchor across missing, merged or changed track IDs."""
        self.association_diagnostics = []
        self.tracks, self.snapshot_time = tracks, now
        if self.p.search_enabled and self.state in ('SEARCH', 'STOP_CONFIRM'):
            self._search_observations(tracks, now)
            return
        for t in tracks:
            if (t['state'] not in ('CANDIDATE', 'CONFIRMED') or t['visited'] or
                    not 0 <= t['age'] <= self.p.visual_timeout):
                continue
            observed = now - t['age']
            if (self.state == 'SEARCH' and self.search_started is not None and
                    observed < self.search_started):
                continue
            history = self.histories.setdefault(t['id'], deque(maxlen=40))
            stamp = t.get('observed_ns')
            new_stamp = stamp is None or stamp != self.last_observed_ns.get(t['id'])
            if new_stamp and (not history or observed > history[-1][0] + 0.001):
                self.last_observed_ns[t['id']] = stamp
                point = (t.get('latest_observation', t['position'])
                         if self.p.search_enabled else t['position'])
                history.append((observed, tuple(point[:2])))
                if self.state == 'SEARCH' and self.first_seen is None:
                    self.first_seen = observed
                    self.first_seen_stamp_ns = t.get('observed_ns')
            while history and now-history[0][0] > self.p.anchor_window_sec:
                history.popleft()
        if self.lock_id is None or self.state in ('STOP', 'FAULT'):
            return
        if self.p.stop_reference == 'body_center':
            old = next((t for t in tracks if t['id'] == self.lock_id), None)
            if old is None:
                self.fault('locked target removed or merged')
                return
            if math.dist(tuple(old['position'][:2]), self.target) > self.p.target_jump_m:
                self.fault('target position jump')
                return
        near = []

        def diagnostic(t, decision, reason):
            point = tuple(t['position'][:2])
            self.association_diagnostics.append({
                'id': t['id'], 'observed_ns': t.get('observed_ns'),
                'decision': decision, 'reason': reason, 'map_xy': point,
                'anchor_initial_xy': self.anchor_initial or self.target,
                'anchor_current_xy': self.target,
                'distance_initial_xy': math.dist(point, self.anchor_initial or self.target),
                'distance_current_xy': math.dist(point, self.target)})

        for t in tracks:
            if t['state'] != 'CONFIRMED' or not 0 <= t['age'] <= self.p.visual_timeout:
                diagnostic(t, 'REJECT', 'not_confirmed' if t['state'] != 'CONFIRMED'
                           else 'observation_age')
                continue
            point = tuple(t['position'][:2])
            if t['visited']:
                diagnostic(t, 'REJECT', 'visited')
                if t['id'] == self.lock_id:
                    self.fault('locked target marked visited')
                continue
            initial = self.anchor_initial or self.target
            if (math.dist(point, initial) > self.p.anchor_association_radius or
                    math.dist(point, self.target) > self.p.target_jump_m):
                diagnostic(t, 'REJECT', 'initial_anchor_radius'
                           if math.dist(point, initial) > self.p.anchor_association_radius
                           else 'current_anchor_jump')
                if t['id'] == self.lock_id:
                    self.anchor_rejections += 1
                continue
            near.append(t)
        # Ambiguous reacquisition never chooses a different physical target.
        if len(near) != 1:
            for item in near:
                diagnostic(item, 'REJECT', 'ambiguous_spatial_matches')
            return
        t = near[0]
        observed = now-t['age']
        if observed <= self.anchor_stamp + 0.001:
            diagnostic(t, 'REJECT', 'not_new_observation')
            return
        diagnostic(t, 'ACCEPT', 'unique_fresh_spatial_match')
        alpha = self.p.anchor_alpha
        self.target = tuple((1-alpha)*old + alpha*new
                            for old, new in zip(self.target, t['position'][:2]))
        self.anchor_stamp = observed
        self.visual_time = observed
        self.anchor_updates += 1
        self.lost_at = None
        self.odom_travel = 0.0

    def _near_detections(self, now):
        """Use source-matched camera ranges, never infer range from a smoothed MAP track."""
        return [t for t in self.tracks if t['state'] in ('CANDIDATE', 'CONFIRMED')
                and not t['visited'] and t.get('observed_ns', 0) > 0
                and 0 <= t['age'] + now-self.snapshot_time <= self.p.visual_timeout
                and 0 < t.get('camera_range_m', math.inf) <= self.p.search_trigger_range_m]

    def _is_stopped(self, now):
        return (now-self.odom_time <= self.p.odom_timeout and
                now-self.camera_time <= self.p.odom_timeout and
                now-self.feedback_time <= self.p.feedback_timeout and
                max(self.actual_speed, self.camera_speed, self.feedback_speed) <=
                self.p.stop_confirm_speed_mps and
                max(self.pose_yaw_rate, abs(self.feedback_yaw_rate)) <=
                self.p.stop_confirm_yaw_rate)

    def _clear_stationary(self):
        self.stationary_since = self.settled_at = None
        self.stop_samples.clear()
        self.stop_sample_stamps.clear()

    def _search_observations(self, tracks, now):
        """Keep moving hints separate from new, post-settle confirmation measurements."""
        valid = self._near_detections(now)
        if self.state == 'SEARCH':
            if self.first_seen is None:
                hints = [t for t in tracks if t['state'] in ('CANDIDATE', 'CONFIRMED')
                         and not t['visited'] and 0 <= t['age'] <= self.p.visual_timeout
                         and math.isfinite(t.get('camera_range_m', math.inf))
                         and now-t['age'] >= self.search_started]
                if hints:
                    first = min(hints, key=lambda t: t['camera_range_m'])
                    self.first_seen = now-first['age']
                    self.first_seen_stamp_ns = first['observed_ns']
                    self.search_first_detection_range_m = first['camera_range_m']
            # A single spatial candidate must persist through distinct source frames.
            if len(valid) != 1:
                self.trigger_history.clear()
                return
            t = valid[0]
            observed = now-t['age']
            if observed < max(self.search_started, self.search_cooldown_until):
                return
            if self.first_seen is None:
                self.first_seen, self.first_seen_stamp_ns = observed, t['observed_ns']
                self.search_first_detection_range_m = t['camera_range_m']
            history = self.trigger_history.setdefault(t['id'], deque(
                maxlen=self.p.search_trigger_observations))
            for other in list(self.trigger_history):
                if other != t['id']:
                    del self.trigger_history[other]
            if history and t['observed_ns'] <= history[-1][1]:
                return
            point = tuple(t['latest_observation'][:2])
            if history and ((t['observed_ns']-history[-1][1])*1e-9 >
                            self.p.confirmation_max_gap_sec or
                            math.dist(point, history[0][2]) > self.p.target_jump_m):
                history.clear()
            history.append((observed, t['observed_ns'], point, t['camera_range_m']))
            return
        if not self._is_stopped(now):
            self._clear_stationary()
            self.stop_confirm_reason = 'waiting for measured stop'
            return
        if self.settled_at is None:
            return
        seed = self.stop_candidate['map']
        matches = [t for t in valid if math.dist(tuple(t['latest_observation'][:2]), seed)
                   <= min(self.p.anchor_association_radius, self.p.target_jump_m)]
        if len(matches) != 1:
            self.stop_samples.clear()
            self.stop_confirm_reason = 'candidate missing or spatially ambiguous'
            return
        t = matches[0]
        observed = now-t['age']
        stamp = t['observed_ns']
        if observed <= self.settled_at or stamp in self.stop_sample_stamps:
            return
        if self.stop_samples and stamp <= self.stop_samples[-1][1]:
            return
        point = tuple(t['latest_observation'][:2])
        if self.stop_samples and (stamp-self.stop_samples[-1][1])*1e-9 > (
                self.p.confirmation_max_gap_sec):
            self.stop_samples.clear()
        self.stop_sample_stamps.add(stamp)
        self.stop_samples.append((observed, stamp, point, t['id'], t['camera_range_m']))
        while self.stop_samples and now-self.stop_samples[0][0] > self.p.anchor_window_sec:
            self.stop_samples.popleft()
        self.stop_confirm_reason = 'collecting new stationary observations'

    def _begin_stop_confirm(self, now):
        valid = self._near_detections(now)
        if len(valid) != 1:
            return False
        t = valid[0]
        history = self.trigger_history.get(t['id'], [])
        if len(history) < self.p.search_trigger_observations:
            return False
        if now-history[-1][0] > self.p.visual_timeout:
            return False
        self.stop_candidate = dict(
            id=t['id'], map=tuple(median(s[2][i] for s in history) for i in (0, 1)),
            trigger_stamp_ns=history[-1][1])
        self.stop_request_range_m = t['camera_range_m']
        self.stop_stable_range_m = None
        self.stopped_after_sec = self.stationary_confirmation_sec = None
        self.stop_confirm_started = now
        self._clear_stationary()
        self.trigger_history.clear()
        self.state, self.reason, self.vx = 'STOP_CONFIRM', '', 0.0
        self.stop_confirm_reason = 'waiting for measured stop'
        return True

    def _stop_confirmation(self, now, out):
        """Always zero; timeout resumes the original bounded search after cooldown."""
        self.vx = 0.0
        if now-self.stop_confirm_started >= self.p.stop_confirm_timeout:
            self.stop_confirm_attempts.append(dict(
                candidate=self.stop_candidate, result='TIMEOUT',
                stop_request_range_m=self.stop_request_range_m,
                stop_stable_range_m=self.stop_stable_range_m,
                stopped_after_sec=self.stopped_after_sec, reason=self.stop_confirm_reason,
                elapsed=now-self.stop_confirm_started))
            self.state, self.reason = 'SEARCH', 'confirmation timeout; search cooldown'
            self.search_cooldown_until = now+self.p.search_cooldown_sec
            self.trigger_history.clear()
            self._clear_stationary()
            out.update(state=self.state, reason=self.reason,
                       transition='STOP_CONFIRM->SEARCH', stop_confirm_reason='timeout')
            return out
        if not self._is_stopped(now):
            self._clear_stationary()
            self.stop_confirm_reason = 'waiting for measured stop'
        elif self.stationary_since is None:
            self.stationary_since = now
        elif (self.settled_at is None and
              now-self.stationary_since >= self.p.stop_confirm_stable_sec):
            self.settled_at = now
            self.stopped_after_sec = now-self.stop_confirm_started
            self.stop_samples.clear()
            self.stop_sample_stamps.clear()
            near = self._near_detections(now)
            match = [t for t in near
                     if math.dist(t['latest_observation'][:2], self.stop_candidate['map'])
                     <= self.p.target_jump_m]
            self.stop_stable_range_m = match[0]['camera_range_m'] if len(match) == 1 else None
        samples = list(self.stop_samples)
        spread = max((math.dist(a[2], b[2]) for a in samples for b in samples), default=0.0)
        span = (samples[-1][1]-samples[0][1])*1e-9 if samples else 0.0
        out.update(stop_confirm_reason=self.stop_confirm_reason,
                   stationary_since=self.stationary_since,
                   settled_at=self.settled_at, stopped_after_sec=self.stopped_after_sec,
                   stop_stable_range_m=self.stop_stable_range_m,
                   stationary_sample_count=len(samples), stationary_sample_span=span,
                   stationary_spread=spread,
                   stationary_observations=[dict(observed_ns=s[1], map=s[2], id=s[3],
                                                 camera_range_m=s[4]) for s in samples])
        current = self._near_detections(now)
        matches = [t for t in current
                   if math.dist(t['latest_observation'][:2], self.stop_candidate['map'])
                   <= self.p.target_jump_m]
        if (self.settled_at is None or len(samples) < self.p.anchor_min_observations or
                span < self.p.stop_confirm_min_span_sec or spread >= self.p.anchor_max_spread or
                len(matches) != 1 or matches[0]['state'] != 'CONFIRMED' or
                matches[0]['observed_ns'] != samples[-1][1]):
            return out
        self.target = tuple(median(s[2][i] for s in samples) for i in (0, 1))
        self.lock_id = samples[-1][3]
        self.anchor_initial, self.anchor_lock_camera = self.target, self.camera
        self.anchor_stamp = self.visual_time = samples[-1][0]
        self.confirmed_at = self.started = now
        self.initial_distance = math.dist(self.target, self.camera)
        self.confirmation_spread, self.confirmation_count = spread, len(samples)
        self.stationary_confirmation_sec = now-self.settled_at
        self.state, self.reason = 'LOCK_TARGET', ''
        self.stop_confirm_attempts.append(dict(
            candidate=self.stop_candidate, result='CONFIRMED',
            stop_request_range_m=self.stop_request_range_m,
            stop_stable_range_m=self.stop_stable_range_m,
            stopped_after_sec=self.stopped_after_sec,
            stationary_confirmation_sec=self.stationary_confirmation_sec, spread=spread,
            count=len(samples), elapsed=now-self.stop_confirm_started))
        out.update(state=self.state, target_id=self.lock_id, target=self.target,
                   target_map_anchor=self.target, anchor_initial=self.anchor_initial,
                   confirmed_at=now, confirmation_count=len(samples), confirmation_spread=spread,
                   stationary_confirmation_sec=self.stationary_confirmation_sec,
                   initial_distance=self.initial_distance, transition='STOP_CONFIRM->APPROACH')
        return out

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
        if self.state != 'IDLE' or self.p.search_enabled:
            return False
        if (now - self.odom_time > self.p.odom_timeout or
                now - self.feedback_time > self.p.feedback_timeout):
            return False
        t = self.candidate(now)
        if (t is None or (not self.p.search_enabled and
                          t.get('lifetime', 0.0) < self.p.lock_stability_sec) or
                (target_id is not None and target_id != t['id'])):
            return False
        reference = self.reference()
        if reference is None:
            return False
        if self.p.stop_reference == 'camera_origin':
            history = [point for stamp, point in self.histories.get(t['id'], [])
                       if now-stamp <= self.p.anchor_window_sec]
            if len(history) < self.p.anchor_min_observations:
                return False
            point = tuple(median(p[i] for p in history) for i in (0, 1))
            spread = (max(math.dist(a, b) for a in history for b in history)
                      if self.p.search_enabled else
                      max(math.dist(p, point) for p in history))
            if spread >= self.p.anchor_max_spread:
                return False
            self.confirmation_spread, self.confirmation_count = spread, len(history)
        else:
            point = tuple(t['position'][:2])
        self.lock_id, self.target = t['id'], point
        self.confirmed_at = now
        self.anchor_initial = point
        self.anchor_lock_camera = reference
        self.anchor_stamp = self.snapshot_time-t['age']
        self.visual_time = self.snapshot_time - t['age']
        self.state = 'LOCK_TARGET'
        self.started = now if self.p.stop_reference == 'body_center' else None
        self.initial_distance = math.hypot(self.target[0] - reference[0],
                                           self.target[1] - reference[1])
        return True

    def start(self, now):
        """Start a previously reviewed camera anchor only on a second manual command."""
        if self.p.search_enabled and self.state == 'IDLE':
            if (self.robot is None or self.camera is None or
                    now-self.odom_time > self.p.odom_timeout or
                    now-self.camera_time > self.p.odom_timeout or
                    now-self.feedback_time > self.p.feedback_timeout):
                return False
            self.histories.clear()
            self.search_origin = self.robot
            self.search_started = self.started = now
            self.state = 'SEARCH'
            return True
        if (self.state != 'LOCK_TARGET' or self.started is not None or
                now-self.odom_time > self.p.odom_timeout or
                now-self.camera_time > self.p.odom_timeout or
                now-self.feedback_time > self.p.feedback_timeout):
            return False
        self.started = now
        return True

    def step(self, now):
        """Return telemetry plus bounded desired velocity; terminal states stay zero."""
        dt = 1 / self.p.control_rate_hz if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        active = self.state not in ('IDLE', 'STOP', 'FAULT') and self.started is not None
        if active:
            if dt <= 0 or dt > self.p.odom_timeout:
                self.fault('control loop deadline')
            elif now - self.odom_time > self.p.odom_timeout:
                self.fault('localization timeout')
            elif (self.p.stop_reference == 'camera_origin' and
                  now-self.camera_time > self.p.odom_timeout):
                self.fault('camera TF timeout')
            elif now - self.feedback_time > self.p.feedback_timeout:
                self.fault('GO2 feedback timeout')
            elif (self.docking is None and self.state not in ('SEARCH', 'STOP_CONFIRM') and
                  now-self.started > self.p.max_run_time):
                self.fault('run timeout')
        out = {'state': self.state, 'reason': self.reason, 'vx': 0.0, 'vy': 0.0, 'wz': 0.0,
               'target_id': self.lock_id, 'target': self.target, 'robot': self.robot,
               'actual_speed': self.actual_speed, 'camera_speed': self.camera_speed,
               'camera': self.camera, 'target_map_anchor': self.target,
               'anchor_initial': self.anchor_initial, 'anchor_updates': self.anchor_updates,
               'anchor_rejections': self.anchor_rejections,
               'association_diagnostics': self.association_diagnostics,
               'visual_age': now-self.visual_time if math.isfinite(self.visual_time) else None,
               'visual_visible': now-self.visual_time <= self.p.visual_timeout, 'eta': None,
               'initial_distance': self.initial_distance,
               'elapsed': now - self.started if self.started is not None else 0.0,
               'visual_lost_at': self.lost_at,
               'candidate': self.candidate(now) if self.state in ('IDLE', 'SEARCH') else None}
        out.update(search_origin=self.search_origin, search_started=self.search_started,
                   search_observations=[{
                       'id': t['id'], 'map': t.get('latest_observation', t['position']),
                       'observed_ns': t.get('observed_ns'), 'age': t['age']}
                       for t in self.tracks] if self.state == 'SEARCH' else [],
                   first_seen=self.first_seen, first_seen_stamp_ns=self.first_seen_stamp_ns,
                   confirmed_at=self.confirmed_at, confirmation_spread=self.confirmation_spread,
                   confirmation_count=self.confirmation_count,
                   search_first_detection_range_m=self.search_first_detection_range_m,
                   stop_candidate=self.stop_candidate,
                   stop_request_range_m=self.stop_request_range_m,
                   stop_stable_range_m=self.stop_stable_range_m,
                   stop_confirm_started=self.stop_confirm_started,
                   stop_confirm_attempts=list(self.stop_confirm_attempts),
                   stopped_after_sec=self.stopped_after_sec,
                   stationary_confirmation_sec=self.stationary_confirmation_sec,
                   search_cooldown_until=(self.search_cooldown_until
                                          if math.isfinite(self.search_cooldown_until) else None),
                   visual_loss_count=self.visual_loss_count,
                   confirmation_seconds=(self.confirmed_at-self.first_seen
                                         if self.confirmed_at is not None and
                                         self.first_seen is not None else None))
        if self.state in ('SEARCH', 'STOP_CONFIRM'):
            x0, y0, heading = self.search_origin
            dx, dy = self.robot[0]-x0, self.robot[1]-y0
            progress = dx*math.cos(heading)+dy*math.sin(heading)
            cross = -dx*math.sin(heading)+dy*math.cos(heading)
            out.update(search_progress=progress, search_cross_track=cross)
            if abs(cross) > self.p.search_cross_track_limit:
                self.fault('search cross-track limit')
            elif now-self.search_started >= self.p.search_timeout:
                self.state, self.reason = 'STOP', 'search timeout without target'
            elif progress >= self.p.search_distance:
                self.state, self.reason = 'STOP', 'search route ended without target'
            else:
                if self.state == 'STOP_CONFIRM':
                    return self._stop_confirmation(now, out)
                if self._begin_stop_confirm(now):
                    out.update(state='STOP_CONFIRM', stop_candidate=self.stop_candidate,
                               stop_confirm_started=now,
                               stop_request_range_m=self.stop_request_range_m,
                               transition='SEARCH->STOP_CONFIRM')
                    return out
                error = angle(heading-self.robot[2])
                remaining = max(0.0, self.p.search_distance-progress)
                desired = min(self.p.search_speed, math.sqrt(
                    2*self.p.max_deceleration*remaining))
                self.vx = min(desired, self.vx+self.p.max_acceleration*dt)
                out.update(vx=self.vx, wz=max(-self.p.max_angular_speed, min(
                    self.p.max_angular_speed, self.p.kp_angular*error-0.5*cross)))
            if self.state in ('STOP', 'FAULT'):
                self.vx = 0.0
                out.update(vx=0.0, wz=0.0)
            out.update(state=self.state, reason=self.reason)
            return out
        if self.docking is not None:
            return self.docking.step(self, now, out, dt, active)
        reference = self.reference()
        if self.robot is None or self.target is None or reference is None:
            return out
        dx, dy = self.target[0] - reference[0], self.target[1] - reference[1]
        distance = math.hypot(dx, dy)
        camera_mode = self.p.stop_reference == 'camera_origin'
        stop_distance = 0.0 if camera_mode else self.p.stop_distance
        tolerance = self.p.stop_xy_tolerance if camera_mode else self.p.distance_tolerance
        remaining = max(0.0, distance - stop_distance)
        speed = self.camera_speed if camera_mode else self.actual_speed
        closing = self.camera_closing_speed if camera_mode else self.closing_speed
        error = angle(math.atan2(dy, dx) - self.robot[2])
        brake = (speed ** 2 / (2 * self.p.max_deceleration)
                 + speed * self.p.braking_latency)
        out.update(distance=distance, remaining=remaining, yaw_error=error,
                   distance_error=distance - stop_distance,
                   camera_distance_xy=distance if camera_mode else None,
                   braking_distance=brake,
                   eta=remaining / closing if closing > 0.01 else None)
        if active and self.state not in ('STOP', 'FAULT'):
            if distance > self.p.max_target_range:
                self.fault('target out of range')
            elif now - self.visual_time > self.p.visual_timeout:
                if self.lost_at is None:
                    self.lost_at = now
                    self.odom_travel = 0.0
                    self.visual_loss_count += 1
                if not self.p.allow_odom:
                    self.fault('visual target lost')
                elif (now - self.lost_at >= self.p.odom_max_time or
                      self.odom_travel >= self.p.odom_max_distance):
                    self.fault('odometry continuation limit')
                else:
                    self.state = 'APPROACH_ODOM'
            if self.state != 'FAULT':
                if remaining <= tolerance:
                    self.state = 'STOP'
                    self.reason = 'camera XY aligned' if camera_mode else 'standoff reached'
                elif (camera_mode and self.anchor_lock_camera is not None and
                      sum((self.target[i]-reference[i]) *
                          (self.anchor_initial[i]-self.anchor_lock_camera[i])
                          for i in (0, 1)) < 0):
                    self.fault('passed target plane outside XY tolerance')
                else:
                    visual = now - self.visual_time <= self.p.visual_timeout
                    self.state = ('ALIGN' if abs(error) > self.p.align_yaw_threshold
                                  else 'APPROACH_VISUAL' if visual else 'APPROACH_ODOM')
                    # Solve v^2/(2a)+latency*v <= remaining minus tolerance.
                    clearance = max(0.0, remaining - tolerance / 2)
                    a, latency = self.p.max_deceleration, self.p.braking_latency
                    safe = math.sqrt((a * latency) ** 2 + 2 * a * clearance) - a * latency
                    desired = min(self.p.max_linear_speed, self.p.kp_linear * remaining, safe)
                    if abs(error) > self.p.align_yaw_threshold:
                        desired = 0.0
                    if brake >= clearance:
                        desired = min(desired, max(0.0, speed - a * dt))
                    self.vx = min(desired, self.vx + self.p.max_acceleration * dt)
                    out.update(vx=self.vx, wz=max(-self.p.max_angular_speed,
                               min(self.p.max_angular_speed, self.p.kp_angular * error)))
        if self.state in ('STOP', 'FAULT', 'IDLE'):
            self.vx = 0.0
            out.update(vx=0.0, wz=0.0)
        out.update(state=self.state, reason=self.reason, visual_lost_at=self.lost_at)
        return out
