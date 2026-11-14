"""Measured positive-vx steps; no device access or assumed speed*time displacement."""
import hashlib
import json
import math
from pathlib import Path


def load_forward_profile(path):
    """Only accept an explicitly reviewed response measurement, never a guess."""
    data = json.loads(Path(path).read_text())
    required = ('speed_mps', 'stop_distance_m', 'min_step_m', 'max_step_m',
                'max_on_sec', 'settle_timeout_sec', 'max_acceleration',
                'max_deceleration', 'braking_latency', 'uncertainty_m')
    if data.get('reviewed') is not True or data.get('physical_motion_confirmed') is not True:
        raise ValueError('Forward response profile needs physical review')
    if not all(math.isfinite(data[k]) and data[k] > 0 for k in required):
        raise ValueError('Forward response measurements must be finite and positive')
    if not (.14 <= data['speed_mps'] <= .30 and
            data['min_step_m'] <= data['max_step_m'] < .15):
        raise ValueError('Invalid measured forward step envelope')
    source = Path(data['source_log'])
    if hashlib.sha256(source.read_bytes()).hexdigest() != data['source_sha256']:
        raise ValueError('Forward response evidence hash mismatch')
    return data


class ForwardStep:
    """One measured MOVE / SETTLE cycle inside the existing docking controller."""

    def __init__(self, profile=None):
        self.profile = profile
        self.reset()

    def reset(self):
        self.phase = 'IDLE'
        self.origin = None
        self.started = self.stopping = None
        self.planned = 0.
        self.count = 0
        self.events = []
        self.last_result = None

    def travel(self, core):
        if self.origin is None:
            return 0.
        x, y, yaw = self.origin
        return ((core.robot[0]-x)*math.cos(yaw) +
                (core.robot[1]-y)*math.sin(yaw))

    def event(self, name, now, **data):
        self.events.append(dict(event=name, monotonic=now, **data))

    def stop(self, core, now, reason):
        if self.phase == 'MOVE':
            self.phase, self.stopping = 'SETTLE', now
            self.event('FORWARD_STEP_ZERO_REQUEST', now, reason=reason,
                       measured_travel_m=self.travel(core))

    def observe(self, core, now):
        if self.phase != 'SETTLE':
            return
        if core.docking_stopped(now) and math.hypot(core.vx, core.vy) < 1e-6:
            moved = self.travel(core)
            self.last_result = dict(measured_travel_m=moved,
                                    moving_sec=self.stopping-self.started,
                                    settling_sec=now-self.stopping)
            self.event('FORWARD_STEP_STOPPED', now, **self.last_result)
            self.phase = 'IDLE'
            if moved < self.profile['min_step_m']/2:
                core.fault('forward step produced no verified displacement')
        elif now-self.stopping > self.profile['settle_timeout_sec']:
            core.fault('forward step did not stop within measured envelope')

    def request(self, core, now, ex, ey, vx, vy, yaw_ok):
        """Return vx reference or None. Lateral/turn control has no compensation."""
        p = self.profile
        if p is None:
            if (core.docking.p.forward_deadzone_guard and yaw_ok and
                    ex > core.docking.p.position_tolerance and
                    abs(ey) < core.docking.p.position_tolerance and
                    0 < vx < core.docking.p.forward_effective_speed):
                core.fault('forward step enabled but response/braking calibration required')
                return 0.
            return None
        if (core.docking.p.forward_deadzone_guard and
                abs(p['speed_mps']-core.docking.p.forward_effective_speed) > 1e-9):
            core.fault('forward profile does not match confirmed effective positive-vx speed')
            return 0.
        if any(abs(getattr(core.p, k)-p[k]) > 1e-9 for k in
               ('max_acceleration', 'max_deceleration', 'braking_latency')):
            core.fault('forward profile does not match current braking/slew limits')
            return 0.
        if self.phase == 'SETTLE':
            return 0.
        if self.phase == 'MOVE':
            if (not yaw_ok or ex <= 0 or abs(ey) >= core.docking.p.position_tolerance or
                    math.hypot(p['speed_mps'], vy) > core.p.max_linear_speed or
                    self.travel(core) >= self.planned or now-self.started >= p['max_on_sec']):
                self.stop(core, now, 'measured travel/geometry/watchdog boundary')
                return 0.
            return p['speed_mps']
        # Only strictly positive, required forward correction can use this path.
        tolerance = core.docking.p.position_tolerance
        if not (ex > tolerance and abs(ey) < tolerance and yaw_ok and
                0 < vx < p['speed_mps']):
            return None
        if not core.docking_stopped(now) or math.hypot(core.vx, core.vy) >= 1e-6:
            # First settle the previous segment before planning another step.
            return 0.
        # Include measured braking travel and uncertainty in the geometric envelope.
        allowance = ex + math.sqrt(tolerance**2-ey**2) - p['uncertainty_m']
        model_stop = (p['speed_mps']**2/(2*core.p.max_deceleration) +
                      p['speed_mps']*core.p.braking_latency)
        stopping = max(p['stop_distance_m'], model_stop)
        planned = min(p['max_step_m'], max(p['min_step_m'], ex-stopping))
        if (planned+stopping > allowance or
                math.hypot(p['speed_mps'], vy) > core.p.max_linear_speed):
            core.fault('remaining forward correction outside measured step envelope')
            return 0.
        self.origin, self.started = core.robot, now
        self.phase, self.planned = 'MOVE', planned
        self.count += 1
        self.event('FORWARD_STEP_START', now, planned_travel_m=planned,
                   remaining_forward_m=ex, reference_vx=p['speed_mps'])
        return p['speed_mps']

    def report(self, core):
        result = dict(phase=self.phase, count=self.count, planned_travel_m=self.planned,
                      measured_travel_m=self.travel(core), last_result=self.last_result,
                      calibration_ready=self.profile is not None, events=self.events)
        self.events = []
        return result
