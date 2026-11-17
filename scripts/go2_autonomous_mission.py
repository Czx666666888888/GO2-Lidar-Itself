#!/usr/bin/env python3
"""Launch the full autonomous mission graph for --nav-mode autonomous.

This orchestrator:
  1. sources ROS + deploy_ws + science perception overlay
  2. launches nav_stack (terrain + FAR + localPlanner with autonomyMode=true)
  3. launches autonomous_full (SLAM + camera + perception + arbiter + mission)
  4. waits for Ctrl+C, then tears everything down in reverse order

It deliberately does NOT start the S1 arm service or send real sport requests.
S1 grasp is delegated to the existing run_map_s1_task.py path triggered by the
MissionManager via /science/approach/* topics; real motion requires --execute
which is gated by go2_test.sh upstream safety checks.
"""
import argparse
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path


def shell(prefix: str, command: str) -> list:
    return ['bash', '-lc', prefix + '; ' + command]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--detector', default='yolo', choices=['yolo', 'hsv'])
    p.add_argument('--vision-root', type=Path, required=True)
    p.add_argument('--overlay', type=Path, required=True)
    p.add_argument('--deploy-ws', type=Path, required=True)
    p.add_argument('--nav-speed', type=float, default=0.4)
    p.add_argument('--max-targets', type=int, default=6)
    p.add_argument('--dry-run', action='store_true',
                   help='publish to preview topics only; no /cmd_vel_raw to sport')
    p.add_argument('--execute', action='store_true',
                   help='real motion; requires upstream safety gate already running')
    p.add_argument('--remote-camera', action='store_true')
    args, _ = p.parse_known_args()

    root = args.vision_root.resolve()
    env_prefix = '; '.join([
        f'source {shlex.quote(str(p))} || exit' for p in [
            '/opt/ros/humble/setup.bash',
            str(args.deploy_ws),
            str(args.overlay),
        ]
    ]) + '; '
    env_prefix += f'export SCIENCE_APPROACH_OVERLAY={shlex.quote(str(args.overlay))}; '

    dry_run = 'true' if (args.dry_run or not args.execute) else 'false'

    children = []

    def spawn(label: str, argv: list):
        print(f'+ [{label}] {shlex.join(map(str, argv))}', flush=True)
        proc = subprocess.Popen(
            shell(env_prefix, shlex.join(map(str, argv))),
            start_new_session=True)
        children.append((label, proc))

    def shutdown(*_):
        print('\n收到停止信号，按启动逆序清理子进程...', flush=True)
        for label, proc in reversed(children):
            if proc.poll() is None:
                print(f'  停止 {label}', flush=True)
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
        sys.exit(130)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ── 1. nav stack (terrain + FAR + localPlanner) ───────────────────────────
    spawn('nav_stack', [
        'ros2', 'launch', 'go2_science_perception', 'nav_stack.launch.py',
        'nav_speed:=', str(args.nav_speed),
    ])

    time.sleep(3.0)

    # ── 2. autonomous full (SLAM + perception + arbiter + mission) ───────────
    full_argv = [
        'ros2', 'launch', 'go2_science_perception', 'autonomous_full.launch.py',
        f'detector:={args.detector}',
        f'dry_run:={dry_run}',
        f'max_targets:={args.max_targets}',
    ]
    if args.remote_camera:
        full_argv.append('remote_camera:=true')
    spawn('autonomous_full', full_argv)

    print('\n=== 自主导航任务已启动 ===', flush=True)
    print(f'  detector={args.detector}  dry_run={dry_run}  '
          f'max_targets={args.max_targets}  nav_speed={args.nav_speed}', flush=True)
    print('  Ctrl+C 停止所有进程；物理急停保持可用。', flush=True)
    print('  实机运动结果需现场确认：NOT VERIFIED until observed on hardware.',
          flush=True)

    # ── monitor ─────────────────────────────────────────────────────────────
    while True:
        for label, proc in children:
            if proc.poll() is not None:
                print(f'[{label}] 进程退出 code={proc.returncode}', flush=True)
        time.sleep(2.0)


if __name__ == '__main__':
    main()
