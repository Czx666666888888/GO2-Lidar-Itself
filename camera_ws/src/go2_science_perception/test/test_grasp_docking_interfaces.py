"""Exercise velocity message wiring and runner deadlines without ROS initialization."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from go2_science_perception.science_target_approach import ScienceTargetApproach
from test_grasp_docking import dock_core


class Publisher:
    def __init__(self, action=lambda _: None):
        self.messages = []
        self.action = action

    def publish(self, msg):
        self.messages.append(msg)
        self.action(msg)


@pytest.mark.parametrize('enabled', [True, False])
def test_ros_velocity_message_carries_lateral_speed_and_fault_zeros(monkeypatch, enabled):
    import go2_science_perception.science_target_approach as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.5)
    core = dock_core((.5, .3), yaw=.25)
    if not enabled:
        core.docking = None
    fake = SimpleNamespace(
        core=core, grasp_docking_enabled=enabled, dry_run=True,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100, to_msg=lambda: Time())),
        last_ros_time=0, count_publishers=lambda _: 0,
        _recording_ready=lambda: True, sport_velocity=[0., 0., 0.], sport_state_info={},
        camera_frame='camera_link', recording_info={}, velocity=Publisher(),
        telemetry=Publisher(), stop_request=None,
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        last_reported_state='APPROACH_TO_GRASP_POSE',
        log_file=SimpleNamespace(write=lambda _: None))
    ScienceTargetApproach._tick(fake)
    msg = fake.velocity.messages[-1]
    assert msg.header.frame_id == 'vehicle'
    if enabled:
        assert msg.twist.linear.x > 0 and msg.twist.linear.y > 0 and msg.twist.angular.z < 0
    else:
        assert msg.twist.linear.y == 0
    core.fault('manual emergency stop')
    fake.last_reported_state = 'FAULT'
    ScienceTargetApproach._tick(fake)
    msg = fake.velocity.messages[-1]
    assert msg.twist.linear.x == msg.twist.linear.y == msg.twist.angular.z == 0


def test_invalid_feedback_faults_docking_without_changing_plain_map():
    for enabled in (True, False):
        c = dock_core()
        fake = SimpleNamespace(core=c, grasp_docking_enabled=enabled)
        msg = SimpleNamespace(mode=0, gait_type=0, error_code=0, yaw_speed=0.,
                              velocity=[0., float('nan'), 0.])
        ScienceTargetApproach._feedback(fake, msg)
        assert (c.state == 'FAULT') is enabled


def runner_module():
    path = Path(__file__).resolve().parents[3]/'scripts/run_map_anchor_experiment.py'
    spec = importlib.util.spec_from_file_location('offline_docking_runner', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('docking', [True, False])
def test_runner_docking_exceeds_former_combined_deadline_plain_map_remains_bounded(
        monkeypatch, tmp_path, docking):
    m = runner_module()
    clock, active = [0.], [False]
    timers = []
    monkeypatch.setattr(m.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(m, 'get_package_share_directory', lambda _: str(tmp_path))

    class Timer:
        def __init__(self, seconds, action):
            timers.append(seconds)

        def start(self):
            pass

    monkeypatch.setattr(m.threading, 'Timer', Timer)
    experiment = m.Experiment.__new__(m.Experiment)
    experiment.args = SimpleNamespace(no_log_terminal=True, search=True, search_distance=5.,
                                      search_timeout=60., timeout=25., grasp_pose=docking,
                                      target_xy=None)
    experiment.folder, experiment.root = tmp_path, tmp_path
    experiment.children = []
    experiment.timer = None
    experiment.control = {'state': 'IDLE'}
    experiment.recording = dict(active=True, frames=100)
    launched = []

    def launch(package, executable, arguments, label):
        launched.append((label, arguments))
        path = tmp_path/label
        path.write_text('Sport lease 已取得 id=1\n状态 DISARMED -> ARMED\n')
        return path

    experiment.launch = launch
    experiment.node = SimpleNamespace(
        count_publishers=lambda _: int(bool(launched)),
        get_publishers_info_by_topic=lambda _: [])
    experiment.arm, experiment.stop = Publisher(), Publisher()
    experiment.start = Publisher(lambda _: active.__setitem__(0, True))

    def spin(seconds):
        clock[0] += seconds
        experiment.last_control_received = clock[0]
        if active[0]:
            state = 'STOP' if clock[0] > 110. else 'APPROACH_TO_GRASP_POSE'
            experiment.control = dict(state=state)

    experiment.spin = spin
    experiment.run()
    if docking:
        assert clock[0] > 110 and not timers
        assert all('max_run_time:=' not in a for label, args in launched for a in args)
    else:
        assert clock[0] < 100 and timers == [85.5]
        assert any('max_run_time:=25.0' in args for _, args in launched)
    assert experiment.stop.messages


def test_successful_dock_close_releases_owners_without_fixed_sleeps(tmp_path):
    m = runner_module()
    e = m.Experiment.__new__(m.Experiment)
    e.args = SimpleNamespace(grasp_pose=True)
    e.folder, e.tag = tmp_path, 'test'
    e.control = dict(state='STOP', grasp_docking_complete=True)
    e.recording, e.reports, e.children = dict(closed=True), [], []
    e.timer, e.navigation_released = None, False
    e.stop, e.arm, e.finish = Publisher(), Publisher(), Publisher()
    halted = []
    e.halt = lambda index: halted.append(index)
    e.spin = lambda _: pytest.fail('no fixed wait when release/finalization already confirmed')
    e.node = SimpleNamespace(count_publishers=lambda _: 0,
                             get_publishers_info_by_topic=lambda _: [],
                             destroy_node=lambda: None)
    e.close()
    assert halted == [1, 0, 2] and e.navigation_released
    assert (tmp_path/'handoff_test.jsonl').exists()


def test_publisher_still_present_prevents_handoff(tmp_path, monkeypatch):
    m = runner_module()
    e = m.Experiment.__new__(m.Experiment)
    e.folder, e.tag, e.navigation_released = tmp_path, 'busy', False
    e.halt, e.arm = lambda _: None, Publisher()
    e.node = SimpleNamespace(count_publishers=lambda _: 1,
                             get_publishers_info_by_topic=lambda _: [])
    clock = [0.]
    monkeypatch.setattr(m.time, 'monotonic', lambda: clock[0])
    e.spin = lambda dt: clock.__setitem__(0, clock[0]+dt)
    e.release_navigation()
    assert not e.navigation_released


def test_phase_durations_use_control_timestamps_and_separate_step_settle():
    m = runner_module()
    reports = [dict(control_monotonic=0., state='SEARCH'),
               dict(control_monotonic=.1, state='STOP_CONFIRM'),
               dict(control_monotonic=.2, state='APPROACH_TO_GRASP_POSE',
                    docking_phase='FINE_CONTROL', forward_step=dict(phase='MOVE')),
               dict(control_monotonic=.4, state='APPROACH_TO_GRASP_POSE',
                    docking_phase='FINE_CONTROL', forward_step=dict(phase='SETTLE')),
               dict(control_monotonic=.5, state='STOP')]
    totals = m.phase_seconds(reports)
    assert totals == pytest.approx(dict(SEARCH=.1, STOP_CONFIRM=.1,
                                       FORWARD_MOVE=.2, FORWARD_SETTLE=.1))


def test_initial_missing_feedback_is_logged_as_null_not_invalid_json():
    import json
    c = dock_core()
    c.reset()
    encoded = []
    fake = SimpleNamespace(
        core=c, grasp_docking_enabled=True, dry_run=True,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100, to_msg=lambda: Time())), last_ros_time=0,
        count_publishers=lambda _: 0, sport_velocity=None, sport_state_info={},
        camera_frame='camera_link', recording_info={}, velocity=Publisher(),
        telemetry=Publisher(), stop_request=None, last_reported_state='IDLE',
        log_file=SimpleNamespace(write=encoded.append))
    ScienceTargetApproach._tick(fake)
    data = json.loads(encoded[-1])
    assert data['stop_inputs']['sport_speed'] is None
    assert data['stop_inputs']['odom_age'] is None
