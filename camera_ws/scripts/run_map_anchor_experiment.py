#!/usr/bin/env python3
"""Explicitly start one recorded MAP-anchor experiment through the existing gate."""

import argparse
import json
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from std_msgs.msg import Bool, Empty, Int8, Int32, String


def flow_metrics(reports):
    """Keep requested outcome evidence separate from the user's physical distance."""
    final = reports[-1] if reports else {}
    return dict(
        search_first_detection_range_m=final.get('search_first_detection_range_m'),
        stop_request_range_m=final.get('stop_request_range_m'),
        stop_stable_range_m=final.get('stop_stable_range_m'),
        stopped_after_sec=final.get('stopped_after_sec'),
        stationary_confirmation_sec=final.get('stationary_confirmation_sec'),
        confirmation_spread_m=final.get('confirmation_spread'),
        confirmation_count=final.get('confirmation_count'),
        confirmation_attempts=final.get('stop_confirm_attempts', []),
        approach_visual_loss_count=final.get('visual_loss_count'),
        final_map_xy_error_m=final.get('camera_distance_xy'),
        terminal_state=final.get('state'), terminal_reason=final.get('reason'),
        physical_distance_m=None, physical_result='PENDING_USER_OBSERVATION')


class Experiment:
    """Own only this run's processes; always stop, disarm and finalize video."""

    def __init__(self, args):
        """Create monitoring and existing manual-control topic clients."""
        self.args = args
        self.node = rclpy.create_node('science_map_anchor_experiment')
        self.root = Path(__file__).resolve().parents[2]
        self.folder = self.root / 'camera_ws/records'
        self.folder.mkdir(exist_ok=True)
        self.tag = str(time.time_ns())
        self.control, self.recording = {}, {}
        self.reports, self.children = [], []
        self.timer = None
        self.node.create_subscription(String, '/science/approach/status', self._status, 20)
        self.node.create_subscription(String, '/science/approach/recording', self._recording, 10)
        self.arm = self.node.create_publisher(Bool, '/arm', 1)
        self.stop = self.node.create_publisher(Int8, '/stop', 1)
        self.confirm = self.node.create_publisher(Int32, '/science/approach/confirm_target_id', 1)
        self.start = self.node.create_publisher(Empty, '/science/approach/start', 1)
        self.reset = self.node.create_publisher(Empty, '/science/approach/reset', 1)
        self.finish = self.node.create_publisher(Empty, '/science/approach/finish_recording', 1)

    def _status(self, msg):
        self.control = json.loads(msg.data)
        self.reports.append(self.control)

    def _recording(self, msg):
        self.recording = json.loads(msg.data)

    def spin(self, seconds):
        """Continue monitoring callbacks while waiting for an expected event."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def launch(self, package, executable, arguments, label):
        """Start one explicitly owned node and retain its diagnostic log."""
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        log = self.folder / f'{self.tag}_{label}.log'
        stream = log.open('w')
        process = subprocess.Popen([str(path), '--ros-args', *arguments],
                                   stdout=stream, stderr=subprocess.STDOUT)
        self.children.append((process, stream))
        return log

    def halt(self, index):
        """Stop only a child created by this experiment."""
        if len(self.children) <= index:
            return
        process = self.children[index][0]
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3)

    def run(self):
        """Record, spatially lock and execute once; no extra interactive prompt."""
        self.spin(0.5)
        if self.node.count_publishers('/cmd_vel_raw'):
            raise RuntimeError('another velocity controller is present')
        local = [e for e in self.node.get_publishers_info_by_topic('/api/sport/request')
                 if e.node_name != '_CREATED_BY_BARE_DDS_APP_']
        if local:
            raise RuntimeError('another local Sport publisher is present')
        share = Path(get_package_share_directory('go2_science_perception'))
        if not self.args.no_log_terminal:
            subprocess.Popen(['gnome-terminal', '--title=GO2 MAP approach live log',
                              '--geometry=160x38', '--', 'bash', '-c',
                              f'bash "{self.root}/camera_ws/scripts/watch_approach_logs.sh"; '
                              'exec bash'])
        gate_log = self.launch('go2_keyboard_teleop', 'go2_safety_gate', [
            '--params-file', str(share / 'config/science_approach_safety_gate.yaml'),
            '-p', 'dry_run:=false'], 'gate')
        search_params = (['--params-file', str(share / 'config/science_target_search.yaml'),
                          '-p', f'search_distance:={self.args.search_distance}',
                          '-p', f'search_timeout:={self.args.search_timeout}']
                         if self.args.search else [])
        self.launch('go2_science_perception', 'science_target_approach', [
            '--params-file', str(share / 'config/science_target_approach.yaml'),
            '-p', 'dry_run:=false', '-p', f'max_run_time:={self.args.timeout}',
            '-p', f'log_directory:={self.folder}', *search_params,
            *(['-p', 'visual_servo_enabled:=true'] if self.args.visual_servo else [])],
            'control')
        self.launch('go2_science_perception', 'approach_video_recorder', [
            '-p', f'output_directory:={self.folder}'], 'recorder')
        self.spin(2)
        if self.control.get('state') == 'FAULT':
            # Only startup TF discovery can be retried, never a motion fault.
            reason = self.control.get('reason', '')
            if self.control.get('target_id') is not None or 'localization/TF' not in reason:
                raise RuntimeError(f'startup fault: {reason}')
            self.reset.publish(Empty())
            self.spin(1.5)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            if (self.recording.get('active') and self.recording.get('frames', 0) >= 10
                    and (self.args.search or self.control.get('candidate'))):
                break
            self.spin(0.1)
        if (self.control.get('state') != 'IDLE' or not self.recording.get('active') or
                self.recording.get('frames', 0) < 10):
            raise RuntimeError('camera, recording or localization not ready')
        candidate = self.control.get('candidate')
        if self.args.target_xy and not self.args.search:
            xy = candidate['position']
            if sum((xy[i]-self.args.target_xy[i])**2 for i in (0, 1))**0.5 > 0.08:
                raise RuntimeError('candidate differs from the approved MAP position')
        if not re.search(r'已取得 id=\d+', gate_log.read_text()):
            raise RuntimeError('no Sport lease')
        if not self.args.search:
            self.confirm.publish(Int32(data=candidate['id']))
            self.spin(0.5)
            if self.control.get('state') != 'LOCK_TARGET' or self.control.get('vx') != 0:
                raise RuntimeError('stable anchor lock rejected')
            print('LOCKED', json.dumps(self.control), flush=True)
        self.arm.publish(Bool(data=True))
        self.spin(0.2)
        transitions = [s for s in gate_log.read_text().splitlines() if '状态 ' in s and '->' in s]
        if not transitions or '-> ARMED' not in transitions[-1]:
            raise RuntimeError('gate refused arming')
        started = time.monotonic()
        total_timeout = self.args.timeout + (self.args.search_timeout if self.args.search else 0)
        self.timer = threading.Timer(total_timeout+0.5,
                                     lambda: self.stop.publish(Int8(data=1)))
        self.timer.start()
        self.start.publish(Empty())
        seen = False
        while time.monotonic()-started < total_timeout+1:
            self.spin(0.05)
            state = self.control.get('state')
            seen |= state in ('SEARCH', 'STOP_CONFIRM', 'LOCK_TARGET', 'ALIGN',
                              'APPROACH_VISUAL', 'APPROACH_ODOM')
            if state in ('STOP', 'FAULT'):
                break
            if (self.node.count_publishers('/cmd_vel_raw') != 1 or
                    any(p.poll() is not None for p, _ in self.children)):
                raise RuntimeError('controller competition or child process exit')
            if not seen and time.monotonic()-started > 2:
                raise RuntimeError('start rejected; no automatic retry')
        self.stop.publish(Int8(data=1))
        self.spin(2)

    def close(self):
        """Finalize evidence even when setup or execution fails."""
        self.stop.publish(Int8(data=1))
        self.spin(0.2)
        if self.timer:
            self.timer.cancel()
        self.halt(1)
        self.spin(0.2)
        self.arm.publish(Bool(data=False))
        self.spin(0.2)
        self.halt(0)
        self.finish.publish(Empty())
        self.spin(0.8)
        self.halt(2)
        for _, stream in self.children:
            stream.close()
        summary = {'control_reports': self.reports, 'final': self.control,
                   'recording': self.recording, 'physical_result': 'PENDING_USER_OBSERVATION',
                   'flow_metrics': flow_metrics(self.reports)}
        (self.folder / f'run_{self.tag}.json').write_text(json.dumps(summary, indent=2))
        print('FINISHED', json.dumps(summary['final']),
              json.dumps(self.recording), flush=True)
        self.node.destroy_node()


def main():
    """Require an explicit execution command as the operator's start action."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='authorize one actual recorded run')
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--target-xy', nargs=2, type=float,
                        help='optional approved MAP XY position')
    parser.add_argument('--no-log-terminal', action='store_true')
    parser.add_argument('--search', action='store_true', help='search then approach automatically')
    parser.add_argument('--visual-servo', action='store_true',
                        help='opt into experimental camera-relative approach')
    parser.add_argument('--search-distance', type=float, default=5.0)
    parser.add_argument('--search-timeout', type=float, default=60.0)
    args = parser.parse_args()
    if not args.execute or not 0 < args.timeout <= 60:
        parser.error('explicit --execute and a timeout in (0,60] seconds are required')
    if not 0 < args.search_distance <= 5 or not 0 < args.search_timeout <= 60:
        parser.error('search distance must be (0,5] m and search timeout (0,60] s')
    if args.search and args.target_xy:
        parser.error('--target-xy is only supported for manually selected approach')
    rclpy.init()
    experiment = Experiment(args)
    try:
        experiment.run()
    finally:
        experiment.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
