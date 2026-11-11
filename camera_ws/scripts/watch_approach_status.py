#!/usr/bin/env python3
"""Print historical and live approach telemetry for the operator terminal."""

import json
from pathlib import Path
import time

import rclpy
from std_msgs.msg import String


def row(data):
    """Format the evidence needed to compare requested and actual motion."""
    native = data.get('sport_velocity') or [0, 0, 0]
    distance = data.get('camera_distance_xy')
    d = 'n/a' if distance is None else f'{distance:.3f}'
    return (f"t={data.get('elapsed', 0):6.2f}s {data.get('state', '?'):16} "
            f"XY={d:>6}m vx={data.get('vx', 0):+.3f} vy={data.get('vy', 0):+.3f} "
            f"wz={data.get('wz', 0):+.3f} sport_vx={native[0]:+.3f} "
            f"sport_vy={native[1]:+.3f} odom_v={data.get('actual_speed', 0):.3f} "
            f"id={data.get('target_id')} samples={data.get('stationary_sample_count', '-')} "
            f"confirm={data.get('stop_confirm_reason', '')} {data.get('reason', '')}")


def main():
    """Show the last recorded run, then subscribe without publishing commands."""
    folder = Path(__file__).resolve().parents[1] / 'records'
    runs = list(folder.glob('run_*.json'))
    if runs:
        last = max(runs, key=lambda p: p.stat().st_mtime)
        print(f'=== HISTORICAL RUN (not current motion): {last} ===', flush=True)
        last_second = -1
        for data in json.loads(last.read_text()).get('control_reports', []):
            second = int(data.get('elapsed', 0))
            if data.get('elapsed', 0) > 0 and second != last_second:
                print(row(data), flush=True)
                last_second = second
    print('\n=== LIVE: waiting for /science/approach/status; '
          'this terminal never commands motion ===',
          flush=True)
    rclpy.init()
    node = rclpy.create_node('science_operator_live_log')
    previous = [0.0, None]

    def callback(msg):
        data = json.loads(msg.data)
        now = time.monotonic()
        if now-previous[0] > 0.2 or data.get('state') != previous[1]:
            print(row(data), flush=True)
            previous[:] = [now, data.get('state')]

    node.create_subscription(String, '/science/approach/status', callback, 20)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
