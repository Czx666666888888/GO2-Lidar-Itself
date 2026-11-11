"""Sequential post-docking handoff to existing GO2 SDK and S1 service; no serial I/O."""
import time

ARM_MOTION_DONE = 'VISUAL_ADAPTIVE_GRASP_BOX_MOTION_DONE_PLACEMENT_UNVERIFIED'


def require_zero_hold(state):
    if (state.get('state') != 'ZERO_HOLD' or state.get('busy') is not False or
            state.get('navigation_allowed') is not True):
        raise RuntimeError('S1 is not idle ZERO_HOLD: ' + str(state))


def finish_docked_task(arm_request, posture, log, timeout=300.0, poll=0.25,
                       monotonic=time.monotonic, sleep=time.sleep):
    """RPC acceptance is never completion; faults never trigger an automatic stand-up."""
    before = arm_request('status')
    require_zero_hold(before)
    log('STAND_DOWN_REQUEST', {})
    down = posture('down')  # Adapter must await feedback-confirmed posture.
    if down.get('confirmed') is not True:
        raise RuntimeError('StandDown feedback not confirmed')
    log('STAND_DOWN_CONFIRMED', down)
    require_zero_hold(arm_request('status'))
    accepted = arm_request('grasp')
    log('GRASP_REQUEST', accepted)
    if accepted.get('accepted') is not True:
        raise RuntimeError('S1 rejected grasp; keep GO2 down')
    deadline = monotonic()+timeout
    while monotonic() < deadline:
        state = arm_request('status')
        log('ARM_STATUS', state)
        if state.get('state') in ('FAULT_HOLD', 'FAULT_IO', 'STOPPING', 'STOPPED'):
            raise RuntimeError('S1 fault/stopped; no StandUp: ' + str(state))
        if state.get('busy') is False:
            require_zero_hold(state)
            if (state.get('last_exit_code') != 0 or state.get('last_result') != ARM_MOTION_DONE or
                    not state.get('last_log') or state['last_log'] == before.get('last_log')):
                raise RuntimeError('No new successful complete grasp result; keep GO2 down')
            log('GRASP_RETURNED_ZERO_HOLD', state)
            # Check again immediately before initiating StandUp.
            require_zero_hold(arm_request('status'))
            up = posture('up')
            if up.get('confirmed') is not True:
                raise RuntimeError('StandUp feedback not confirmed')
            result = dict(task_motion_complete=True, placement_confirmed=None,
                          arm_result=state, stand_down=down, stand_up=up)
            log('TASK_COMPLETE', result)
            return result
        sleep(poll)
    raise RuntimeError('S1 grasp timeout; keep GO2 down and ZERO/FAULT hold service alive')
