"""Accepted request, stale success and arm fault are not completion evidence."""
import pytest

from go2_science_perception.s1_task_sequence import ARM_MOTION_DONE, finish_docked_task


def zero(log='old'):
    return dict(state='ZERO_HOLD', busy=False, navigation_allowed=True,
                last_exit_code=0, last_result=ARM_MOTION_DONE, last_log=log)


def run(states):
    calls = []
    iterator = iter(states)

    def request(command):
        calls.append(command)
        if command == 'grasp':
            return dict(accepted=True, state='QUEUED')
        return next(iterator)

    def posture(direction):
        calls.append(direction)
        return dict(confirmed=True)

    try:
        result = finish_docked_task(request, posture, lambda *_: None, sleep=lambda _: None)
    except RuntimeError as exc:
        return calls, exc
    return calls, result


def test_waits_for_busy_to_end_new_result_and_zero_before_standup():
    calls, result = run([zero(), zero(),
                         dict(state='GRASPING', busy=True, navigation_allowed=False),
                         zero('new'), zero('new')])
    assert calls == ['status', 'down', 'status', 'grasp', 'status', 'status', 'status', 'up']
    assert result['task_motion_complete'] and result['placement_confirmed'] is None


@pytest.mark.parametrize('terminal', [
    dict(state='FAULT_HOLD', busy=False, navigation_allowed=False),
    dict(state='FAULT_IO', busy=False, navigation_allowed=False),
    zero('old'), dict(zero('new'), last_exit_code=3),
    dict(zero('new'), last_result='VISUAL_ADAPTIVE_CONTACT_MISS_SAFE_RETURN')])
def test_fault_old_task_or_nonzero_result_never_stands_up(terminal):
    calls, result = run([zero(), zero(), terminal])
    assert isinstance(result, RuntimeError) and 'up' not in calls
