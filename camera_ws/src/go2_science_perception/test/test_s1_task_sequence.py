"""Accepted request, stale success and arm fault are not completion evidence."""
import pytest

from go2_science_perception.s1_task_sequence import ARM_MOTION_DONE, finish_docked_task


def zero(log='old'):
    return dict(state='ZERO_HOLD', busy=False, navigation_allowed=True,
                at_zero=True, communication_state='OK', readiness_state='ZERO_HOLD',
                last_task='grasp', last_exit_code=0, last_result=ARM_MOTION_DONE, last_log=log)


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
    zero('old'), dict(zero('new'), last_task='dump'),
    dict(zero('new'), at_zero=False),
    dict(zero('new'), communication_state='ERROR'),
    dict(zero('new'), last_result=None),
    dict(zero('new'), last_exit_code=None)])
def test_fault_old_or_incomplete_result_never_stands_up(terminal):
    calls, result = run([zero(), zero(), terminal])
    assert isinstance(result, RuntimeError) and 'up' not in calls


@pytest.mark.parametrize(('exit_code', 'grasp_result'), [
    (1, 'ABORTED: combined pregrasp tool direction exceeds the existing orientation tolerance'),
    (1, 'ABORTED: high-view search exhausted without a grasp handoff'),
    (3, 'VISUAL_NO_TARGET_SAFE_RETURN'),
])
def test_finished_failure_stands_up_after_confirmed_zero_hold(exit_code, grasp_result):
    terminal = dict(zero('new'), last_exit_code=exit_code, last_result=grasp_result)
    calls, result = run([zero(), zero(), dict(terminal, state='RETURNING_ZERO'), terminal, terminal])
    assert calls[-1] == 'up'
    assert result['outcome'] == 'MISSED' and result['task_motion_complete']
    assert result['arm_result']['last_result'] == grasp_result


def test_verifies_result_and_zero_again_before_standup():
    terminal = dict(zero('new'), last_exit_code=1, last_result='ABORTED')
    for changed in (dict(terminal, at_zero=False), dict(terminal, last_log='other')):
        calls, result = run([zero(), zero(), terminal, changed])
        assert isinstance(result, RuntimeError) and 'up' not in calls
