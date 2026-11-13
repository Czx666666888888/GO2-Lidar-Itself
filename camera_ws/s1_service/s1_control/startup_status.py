"""Read-only classification inside the original single-owner S1 service.

Source: S1_arm_integration_20261111_auto_zero_hold_v1.zip. No serial constructor here.
"""
import math
import os
import time


def communication_error(exc):
    text = str(exc)
    # Preserve the actual SDK / OS error, never classify pose-window errors as I/O.
    return any(word in text.lower() for word in (
        'input/output', 'bus init failed', 'motor out line', 'failed to open',
        'no such file', 'serial', 'device disconnected')) or isinstance(exc, OSError)


def feedback_status(arm, device=None, zero_windows=None):
    if device and not os.path.exists(device):
        raise OSError('serial device missing: '+device)
    q = [float(x) for x in arm.get_pos()[:6]]
    velocity = [float(x) for x in arm.get_vel()[:6]]
    temperature = [float(x) for x in arm.get_temp()]
    if (len(q) != 6 or len(velocity) != 6 or len(temperature) < 7 or
            not all(math.isfinite(x) for x in q+velocity+temperature) or
            min(temperature) < 5.0):
        raise OSError('invalid/uninitialized joint feedback')
    if max(temperature) > arm.limits.max_temp_c:
        raise RuntimeError('temperature exceeds existing limit')
    # Original service move/hold guard tolerance; do not invent a new zero definition.
    tolerance = arm.limits.max_track_err_rad
    at_zero = all(abs(x) <= tolerance for x in q)
    if zero_windows is not None:
        at_zero = at_zero and all(lo <= x <= hi for x, (lo, hi) in zip(q, zero_windows))
    return dict(joint_positions_rad=q, joint_velocities_rad_s=velocity,
                feedback_read_monotonic=time.monotonic(), zero_tolerance_rad=tolerance,
                at_zero=at_zero, position_state='AT_ZERO' if at_zero else 'NOT_AT_ZERO',
                communication_state='OK', feedback_source='existing Arm.get_pos/get_vel/get_temp',
                feedback_limitation='SDK feedback read; no independent motor packet timestamp exposed')
