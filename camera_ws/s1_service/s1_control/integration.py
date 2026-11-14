"""Single-owner arm session: zero hold <-> existing full grasp flow."""
from __future__ import annotations
import copy
import queue
import threading
import time
from .startup_status import feedback_status, communication_error

ZERO_Q = [0.0] * 6

class ArmIntegrationSession:
    def __init__(self, arm, *, hold_hz=50.0):
        self.arm = arm
        self.hold_hz = hold_hz
        self.idle_limits = copy.copy(arm.limits)
        self._lock = threading.Lock()
        self._stop_hold = threading.Event()
        self._hold_thread = None
        self.jobs = queue.Queue()
        self._state = {"state": "STARTING", "busy": True, "last_result": None,
                       "last_log": None, "zero_target_rad": list(ZERO_Q),
                       "navigation_allowed": False}
        self._fault = None
        self._hold_q = list(ZERO_Q)
        self._grip = None

    def snapshot(self):
        try:
            values = feedback_status(self.arm, getattr(self, 'device', None),
                                     getattr(self, 'zero_windows', None),
                                     getattr(self, 'device_identity', None))
            with self._lock:
                state = self._state['state']
                values['readiness_state'] = ('BUSY' if self._state['busy'] else
                    'ZERO_HOLD' if state == 'ZERO_HOLD' and values['at_zero'] else
                    'FAULT' if state.startswith('FAULT') else values['position_state'])
                if state == 'ZERO_HOLD' and not values['at_zero']:
                    values['navigation_allowed'] = False
                self._state.update(values)
        except Exception as exc:
            self._update(readiness_state='COMMUNICATION_ERROR' if communication_error(exc)
                         else 'FAULT', error=str(exc), navigation_allowed=False)
        with self._lock:
            return copy.deepcopy(self._state)

    def _update(self, **values):
        with self._lock:
            self._state.update(values)

    def request_grasp(self):
        status = self.snapshot()
        if status.get('readiness_state') != 'ZERO_HOLD':
            return dict(accepted=False, reason=status.get('readiness_state', 'FAULT'))
        with self._lock:
            if self._state["state"] != "ZERO_HOLD":
                return {"accepted": False, "reason": self._state["state"]}
            self._state.update(state="QUEUED", busy=True, navigation_allowed=False)
        self.jobs.put("grasp")
        return {"accepted": True, "state": "QUEUED"}

    def request_zero(self):
        state = self.snapshot()
        with self._lock:
            if self._state['busy'] or self._state['state'] in ('GRASPING','RETURNING_ZERO','QUEUED'):
                return dict(accepted=False, reason='BUSY')
            if state.get('readiness_state') in ('COMMUNICATION_ERROR', 'FAULT'):
                return dict(accepted=False, reason=state['readiness_state'], error=state.get('error'))
            if state.get('return_zero_allowed') is False:
                return dict(accepted=False, reason='NOT_AT_ZERO', error=state.get('error'))
            if state.get('readiness_state') == 'ZERO_HOLD' and state.get('navigation_allowed'):
                return dict(accepted=True, state='ZERO_HOLD', reused=True)
            self._state.update(state='RETURNING_ZERO', busy=True, navigation_allowed=False)
        self.jobs.put('zero')
        return dict(accepted=True, state='RETURNING_ZERO')

    def mark_fault(self, reason):
        self._fault = str(reason)

    def start_hold(self, target=None, *, gripper=None, gap=None, force=None):
        self.stop_hold()
        self._hold_q = list(ZERO_Q if target is None else target)
        self._update(hold_target_rad=list(self._hold_q))
        self._grip = (gripper, gap, force) if gripper is not None else None
        self.arm.set_position_commander(None)
        self.arm.limits = copy.copy(self.idle_limits)
        self.arm.dt = 1.0 / self.arm.limits.hz
        self._stop_hold.clear()
        self._hold_thread = threading.Thread(target=self._hold_loop,
                                              name="s1-position-hold", daemon=True)
        self._hold_thread.start()

    def _hold_loop(self):
        next_temp = time.monotonic()
        try:
            while not self._stop_hold.is_set():
                started = time.monotonic()
                self.arm.command_position(self._hold_q)
                if self._grip is not None:
                    gripper, gap, force = self._grip
                    gripper.command_mm(gap, force=force)
                if started >= next_temp:
                    if max(self.arm.get_temp()) > self.arm.limits.max_temp_c:
                        raise RuntimeError("hold temperature exceeds existing temperature limit")
                    next_temp = started + 2.0
                self._stop_hold.wait(max(0.0, 1.0/self.hold_hz - (time.monotonic()-started)))
        except Exception as exc:
            self.mark_fault(exc)
            self._update(state="FAULT_IO", busy=False, navigation_allowed=False,
                         error=str(exc))
            # A lost command channel or overheating cannot provide a reliable hold.
            self.arm.disable()

    def stop_hold(self):
        self._stop_hold.set()
        if self._hold_thread is not None:
            self._hold_thread.join()
            self._hold_thread = None

    def ready(self):
        if self._fault is None:
            self._update(state="ZERO_HOLD", busy=False, navigation_allowed=True)

    def acquire(self, limits):
        self.stop_hold()
        self.arm.limits = limits
        self.arm.dt = 1.0 / limits.hz
        self.arm._speed_strikes = 0
        self.arm._speed_strike_samples.clear()
        self.arm._guard_position_samples.clear()
        return self.arm

    def finish_task(self, *, at_zero, result, log_path, gripper=None,
                    grip_gap_mm=None, grip_force=None):
        self.stop_hold()
        if self._fault is None and not at_zero:
            self.mark_fault(result)
        target = ZERO_Q if self._fault is None else self.arm.get_pos()[:6]
        self._update(state="RETURNING_ZERO" if self._fault is None else "FAULT_HOLD",
                     navigation_allowed=False, busy=True, last_result=result,
                     last_log=log_path, error=self._fault)
        self.start_hold(target, gripper=gripper, gap=grip_gap_mm, force=grip_force)

    def run_task(self, callback):
        self._update(state="GRASPING", busy=True, navigation_allowed=False)
        try:
            code = callback(self)
        except Exception as exc:
            self.mark_fault(exc)
            self.finish_task(at_zero=False, result=str(exc), log_path=None)
            code = 1
        self._update(last_exit_code=code, busy=False)
        if self._fault is None:
            self.ready()
        elif self.snapshot()["state"] != "FAULT_IO":
            self._update(state="FAULT_HOLD", navigation_allowed=False)
        return code

    def close(self):
        self._update(state="STOPPING", busy=True, navigation_allowed=False)
        self.stop_hold()
        self.arm.disable()
        self.arm.close()
        self._update(state="STOPPED", busy=False)
