#!/usr/bin/env python3
"""PC single-target coordinator. Default is an offline plan, never motion."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shlex
import subprocess
import time


def arm_command(args, command):
    argv = [args.host_python, args.arm_client, command, '--socket', args.arm_socket]
    if args.arm_host != 'local':
        argv = ['ssh', '-i', args.ssh_key, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
                args.arm_host, shlex.join(argv)]
    result = subprocess.run(argv, capture_output=True, text=True, timeout=4, check=True)
    return json.loads(result.stdout)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--full', action='store_true', help='after docking: StandDown/grasp/StandUp')
    p.add_argument('--docking-geometry-verified', action='store_true',
                   help='operator has completed separate physical XY docking validation')
    p.add_argument('--search-distance', type=float, default=5.0)
    p.add_argument('--search-timeout', type=float, default=60.0)
    p.add_argument('--timeout', type=float,
                   help='legacy argument accepted but ignored for grasp docking')
    p.add_argument('--no-log-terminal', action='store_true')
    p.add_argument('--arm-host', default='unitree@192.168.123.18')
    p.add_argument('--ssh-key', default='/home/czx666/.ssh/id_ed25519_go2')
    p.add_argument('--host-python', default='/usr/bin/python3')
    p.add_argument('--arm-client',
                   default='/home/unitree/s1-arm-control/scripts/arm_task_client.py')
    p.add_argument('--arm-socket', default='/tmp/s1-arm-integration.sock')
    p.add_argument('--task-host-script', default='/home/unitree/go2_map_s1/s1_task_host.py')
    p.add_argument('--go2-interface', default='eth0')
    p.add_argument('--sdk-cyclonedds-home',
                   default='/home/unitree/cyclonedds_ws/install/cyclonedds',
                   help='GO2 SDK Python build-matched DDS; affects posture process only')
    p.add_argument('--posture-profile', type=Path,
                   help='measured GO2 posture JSON; explicit CLI values take precedence')
    p.add_argument('--down-max-body-height-m', type=float)
    p.add_argument('--up-min-body-height-m', type=float)
    p.add_argument('--down-mode', type=int)
    p.add_argument('--up-mode', type=int)
    p.add_argument('--arm-timeout', type=float, default=300.0)
    return p


def main():
    p = parser()
    args = p.parse_args()
    if args.posture_profile is not None:
        profile = json.loads(args.posture_profile.read_text())
        for name in ('down_max_body_height_m', 'up_min_body_height_m', 'down_mode', 'up_mode'):
            if getattr(args, name) is None:
                setattr(args, name, profile[name])
    if not (0 < args.search_distance <= 5 and 0 < args.search_timeout <= 60 and
            (args.timeout is None or 0 < args.timeout <= 60) and args.arm_timeout > 0):
        p.error('Use original bounded search limits and positive arm timeout')
    if args.full and args.execute and (
            args.down_max_body_height_m is None or args.up_min_body_height_m is None or
            args.down_mode is None or args.up_mode is None or
            not 0 < args.down_max_body_height_m < args.up_min_body_height_m):
        p.error('Full execution requires measured down/up height boundaries and '
                'feedback modes; do not guess them')
    plan = dict(base_commit='e15c66ce2beb998ef9c73464b0bff72cbd4e2266',
                mode='full' if args.full else 'dock_only',
                stages=['SEARCH', 'STOP_CONFIRM', 'APPROACH_TO_GRASP_POSE', 'STOP'] +
                (['STAND_DOWN', 'S1_GRASP', 'ZERO_HOLD', 'STAND_UP'] if args.full else []),
                s1_base_vehicle_xy_m=[-.06, 0], desired_target_s1_xy_m=[0, .585],
                s1_base_z=None, operator_docking_validation_acknowledged=bool(
                    args.docking_geometry_verified),
                posture_profile=str(args.posture_profile) if args.posture_profile else None,
                posture_boundaries=dict(down_max_body_height_m=args.down_max_body_height_m,
                                        up_min_body_height_m=args.up_min_body_height_m,
                                        down_mode=args.down_mode, up_mode=args.up_mode),
                sdk_cyclonedds_home=args.sdk_cyclonedds_home,
                docking_fixed_timeout_sec=None, legacy_timeout_argument_ignored=args.timeout,
                execute=args.execute, arm_service_validation='NOT VERIFIED')
    print(json.dumps(plan, indent=2), flush=True)
    if not args.execute:
        return
    from go2_science_perception.s1_task_sequence import require_zero_hold
    from run_map_anchor_experiment import Experiment
    import rclpy
    args.search, args.grasp_pose, args.target_xy = True, True, None

    def navigation_guard():
        require_zero_hold(arm_command(args, 'status'))

    navigation_guard()
    rclpy.init()
    experiment = Experiment(args, navigation_guard=navigation_guard)
    try:
        try:
            experiment.run()
            final = dict(experiment.control)
        finally:
            experiment.close()
    except BaseException:
        if rclpy.ok():
            rclpy.shutdown()
        raise
    # The original controller AND gate must be gone before SDK posture control.
    if any(proc.poll() is None for proc, _ in experiment.children):
        raise RuntimeError('Navigation children still alive; no posture handoff')
    audit = rclpy.create_node('s1_post_dock_handoff_audit')
    try:
        for _ in range(20):
            rclpy.spin_once(audit, timeout_sec=0.05)
        local = [e for e in audit.get_publishers_info_by_topic('/api/sport/request')
                 if e.node_name != '_CREATED_BY_BARE_DDS_APP_']
        if audit.count_publishers('/cmd_vel_raw') or local:
            raise RuntimeError('Another movement publisher prevents posture handoff')
    finally:
        audit.destroy_node()
    if final.get('grasp_docking_complete') is not True or final.get('state') != 'STOP':
        rclpy.shutdown()
        raise RuntimeError('Docking was not completed; no StandDown or grasp')
    navigation_guard()
    if not args.full:
        print('DOCK_ONLY_COMPLETE: measure actual target XY in S1 before full execution',
              flush=True)
        rclpy.shutdown()
        return
    command = ['env', 'CYCLONEDDS_HOME='+args.sdk_cyclonedds_home,
               'LD_LIBRARY_PATH='+str(Path(args.sdk_cyclonedds_home)/'lib'),
               args.host_python, args.task_host_script, '--execute', '--require-heartbeat',
               '--arm-client', args.arm_client, '--socket', args.arm_socket,
               '--interface', args.go2_interface,
               '--down-max-body-height-m', str(args.down_max_body_height_m),
               '--up-min-body-height-m', str(args.up_min_body_height_m),
               '--down-mode', str(args.down_mode), '--up-mode', str(args.up_mode),
               '--arm-timeout', str(args.arm_timeout)]
    if args.arm_host != 'local':
        command = ['ssh', '-i', args.ssh_key, '-o', 'BatchMode=yes',
                   '-o', 'ConnectTimeout=3', args.arm_host, shlex.join(command)]
    log = experiment.folder / ('s1_task_' +
                               datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '.jsonl')
    from std_msgs.msg import Int8
    supervisor = rclpy.create_node('s1_posture_task_supervisor')
    stopped = [False]
    supervisor.create_subscription(Int8, '/stop',
                                   lambda m: stopped.__setitem__(0, stopped[0] or m.data != 0), 1)
    process = None
    try:
        with log.open('x') as stream:
            process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                       stdout=stream, stderr=subprocess.STDOUT, text=True)
            deadline = time.monotonic()+args.arm_timeout+90
            next_heartbeat = 0.0
            while process.poll() is None:
                rclpy.spin_once(supervisor, timeout_sec=0.05)
                if stopped[0] or time.monotonic() > deadline:
                    process.stdin.write('{"stop":true}\n')
                    process.stdin.flush()
                    raise RuntimeError('Posture task stopped/timed out; no automatic StandUp')
                if time.monotonic() >= next_heartbeat:
                    process.stdin.write('{"heartbeat":true}\n')
                    process.stdin.flush()
                    next_heartbeat = time.monotonic()+0.5
            completed = process
    finally:
        if process is not None:
            if process.stdin:
                process.stdin.close()
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        supervisor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if completed.returncode:
        raise RuntimeError('Posture/arm task failed; no automatic recovery. See '+str(log))
    print('FULL_TASK_MOTION_COMPLETE; placement remains unverified. Log: '+str(log))


if __name__ == '__main__':
    main()
