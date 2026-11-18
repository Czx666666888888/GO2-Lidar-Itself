"""Sequential post-docking handoff to existing GO2 SDK and S1 service; no serial I/O."""
import time

ARM_MOTION_DONE = 'VISUAL_ADAPTIVE_GRASP_BOX_MOTION_DONE_PLACEMENT_UNVERIFIED'


def require_zero_hold(state):
    if (state.get('state') != 'ZERO_HOLD' or state.get('busy') is not False or
            state.get('navigation_allowed') is not True or
            state.get('at_zero') is not True or
            state.get('communication_state') != 'OK' or
            state.get('readiness_state') != 'ZERO_HOLD'):
        raise RuntimeError('S1 is not idle ZERO_HOLD: ' + str(state))


def finish_docked_task(arm_request, posture, log, timeout=300.0, poll=0.25,
                       monotonic=time.monotonic, sleep=time.sleep):
    """Stand up after a new terminal result and verified arm zero hold."""
    before = arm_request('status')
    require_zero_hold(before)
    log('STAND_DOWN_REQUEST', {})
    down = posture('down')  # Adapter must await feedback-confirmed posture.
    if down.get('confirmed') is not True:
        raise RuntimeError('StandDown feedback not confirmed')
    log('STAND_DOWN_CONFIRMED', down)
    before = arm_request('status')
    require_zero_hold(before)
    log('GRASP_START_REQUEST', {})
    accepted = arm_request('grasp')
    log('GRASP_REQUEST', accepted)
    if accepted.get('accepted') is not True:
        raise RuntimeError('S1 rejected grasp; keep GO2 down')
    deadline = monotonic()+timeout
    running_observed = False
    while monotonic() < deadline:
        state = arm_request('status')
        log('ARM_STATUS', state)
        if state.get('state') == 'GRASPING' and not running_observed:
            log('GRASP_RUNNING_OBSERVED', state)
            running_observed = True
        if state.get('state') in ('FAULT_HOLD', 'FAULT_IO', 'STOPPING', 'STOPPED'):
            raise RuntimeError('S1 fault/stopped; no StandUp: ' + str(state))
        if state.get('state') == 'ZERO_HOLD' and state.get('busy') is False:
            require_zero_hold(state)
            task, path = state.get('last_task'), state.get('last_log')
            exit_code, grasp_result = state.get('last_exit_code'), state.get('last_result')
            if (task != 'grasp' or not isinstance(path, str) or not path or path == before.get('last_log')
                    or type(exit_code) is not int or not isinstance(grasp_result, str) or not grasp_result):
                raise RuntimeError('No new terminal grasp result; keep GO2 down')
            completed = exit_code == 0 and grasp_result == ARM_MOTION_DONE
            log('GRASP_RETURNED_ZERO_HOLD', state)
            if not completed:
                log('GRASP_FAILED_SAFE_RETURN', dict(exit_code=exit_code, result=grasp_result, log=path))
            confirmed = arm_request('status')
            require_zero_hold(confirmed)
            if any(confirmed.get(key) != state[key]
                   for key in ('last_task', 'last_log', 'last_exit_code', 'last_result')):
                raise RuntimeError('S1 result changed before StandUp; keep GO2 down')
            up = posture('up')
            if up.get('confirmed') is not True:
                raise RuntimeError('StandUp feedback not confirmed')
            result = dict(task_motion_complete=True, outcome='COMPLETED' if completed else 'MISSED',
                          placement_confirmed=None,
                          arm_result=state, stand_down=down, stand_up=up)
            log('TASK_COMPLETE', result)
            return result
        sleep(poll)
    raise RuntimeError('S1 grasp timeout; keep GO2 down and ZERO/FAULT hold service alive')
