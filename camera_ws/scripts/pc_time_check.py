#!/usr/bin/env python3
"""PC timing context: read-only check or explicitly requested dock/full motion."""
import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
ENTRY = ROOT/'camera_ws/scripts/run_go2_time_diagnostic.sh'

def run(args, **kw):
    return subprocess.run([str(x) for x in args], check=True, **kw)

def probe(seconds):
    points = []
    start = time.monotonic()
    while time.monotonic()-start < seconds:
        points.append([time.time_ns(), time.monotonic_ns(),
                       time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)])
        time.sleep(.1)
    delta = [(b-a)/1e9 for a,b in zip(points[0], points[-1])]
    return dict(samples=points, elapsed_wall_mono_raw_s=delta,
                wall_minus_raw_elapsed_s=delta[0]-delta[2])

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='run real motion; full also performs posture/grasp')
    parser.add_argument('--mode', choices=('dock', 'full', 'approach'), default='full')
    parser.add_argument('--grasp-side', choices=('right','left'), default='right')
    parser.add_argument('--left-geometry-verified', action='store_true')
    parser.add_argument('--clock-diagnostic', action='store_true',
                        help='explicit full 20s online + 20s paused clock sampling')
    parser.add_argument('--forward-response-check', action='store_true')
    parser.add_argument('--response-distance-limit', type=float, default=.20)
    parser.add_argument('--work-dir', type=Path, help='forward existing integration checkout option')
    parser.add_argument('--detector', choices=('yolo','hsv'), default='yolo')
    parser.add_argument('--view', action='store_true')
    parser.add_argument('--local-camera', action='store_true')
    parser.add_argument('--timeout', type=float, default=45.)
    parser.add_argument('--model-path', type=Path)
    parser.add_argument('--python-executable', type=Path)
    args = parser.parse_args()
    if args.mode == 'approach' and (args.work_dir or args.left_geometry_verified):
        parser.error('approach mode does not use S1 geometry or work-dir')
    if not .10 <= args.response_distance_limit <= .50:
        parser.error('response distance limit must be 0.10 to 0.50m')
    if args.response_distance_limit != .20 and not args.forward_response_check:
        parser.error('response distance override is for response check only')
    if args.forward_response_check and args.mode != 'dock':
        parser.error('Forward response check requires --mode dock')
    folder = ROOT/'camera_ws/records'/('pc_time_'+str(time.time_ns()))
    folder.mkdir(parents=True)
    state = {'service':'systemd-timesyncd', 'execute_requested':args.execute}
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    # Preflight and build before stopping time service; no device enable here.
    if not args.execute:
        run(['bash', ENTRY, '--lidar-check', '--duration', '10', '--label', 'pc_controlled_native'])
        run(['bash', ENTRY, '--prepare-only'])
    state['was_active'] = subprocess.run(['systemctl','is-active','--quiet','systemd-timesyncd']).returncode == 0
    state['before_status'] = (subprocess.run(['timedatectl','timesync-status'],capture_output=True,text=True).stdout
                              if state['was_active'] else 'inactive; no D-Bus query')
    state['full_clock_diagnostic'] = args.clock_diagnostic or not args.execute
    started = time.monotonic()
    if state['full_clock_diagnostic']:
        print('显式完整时钟诊断：在线20秒 + 暂停20秒。', flush=True)
        state['online'] = probe(20)
    else:
        print('快速启动：跳过40秒完整时钟采样；保留服务监测和实时定位检查。', flush=True)
    changed = False
    try:
        if state['was_active']:
            run(['sudo','-v'])
            # Mark before mutation so interruption during systemctl still restores.
            changed = True
            run(['sudo','-n','systemctl','stop','systemd-timesyncd'])
        if state['full_clock_diagnostic']:
            state['paused'] = probe(20)
        state['time_context_ready_sec'] = time.monotonic()-started
        print('STAGE time_context %.3fs' % state['time_context_ready_sec'], flush=True)
        state['paused_service_active'] = subprocess.run(['systemctl','is-active','--quiet','systemd-timesyncd']).returncode == 0
        if changed and state['paused_service_active']:
            raise RuntimeError('PC校时服务意外重启，阻止启动任务')
        env = dict(os.environ, GO2_TIME_MAPPING_MODE='imu', GO2_PC_TIME_HELD='1')
        if args.execute:
            command = ['bash',str(ROOT/'go2_test.sh'),'--execute','--mode',args.mode,
                       '--grasp-side',args.grasp_side]
            if args.left_geometry_verified:
                command.append('--left-geometry-verified')
            if args.forward_response_check:
                command += ['--forward-response-check', '--response-distance-limit',
                            str(args.response_distance_limit)]
            if args.work_dir is not None:
                command += ['--work-dir',str(args.work_dir)]
            if args.mode != 'approach':
                command += ['--detector', args.detector]
                if args.view: command.append('--view')
                if args.model_path: command += ['--model-path',str(args.model_path)]
                if args.python_executable: command += ['--python-executable',str(args.python_executable)]
                if args.local_camera:
                    raise RuntimeError('--local-camera只用于approach；full/dock沿用GO2伴随主机相机')
            if args.mode == 'approach':
                command = ['bash',str(ROOT/'go2_test.sh'),'--execute','--mode','approach',
                           '--detector',args.detector,'--timeout',str(args.timeout)]
                if args.view: command.append('--view')
                if args.local_camera: command.append('--local-camera')
                if args.model_path: command += ['--model-path',str(args.model_path)]
                if args.python_executable: command += ['--python-executable',str(args.python_executable)]
        else:
            run(['/usr/bin/python3', ROOT/'camera_ws/scripts/sync_go2_clock.py'])
            command = ['bash',str(ENTRY),'--time-mode','imu','--restart-localization',
                       '--require-target','--duration','60','--label','pc_paused_imu']
        job = subprocess.Popen(command,env=env)
        try:
            while job.poll() is None:
                if changed and subprocess.run(['systemctl','is-active','--quiet','systemd-timesyncd']).returncode == 0:
                    state['unexpected_time_service_restart'] = True
                    print('PC校时服务意外重启，请求原任务停止。',flush=True)
                    job.send_signal(signal.SIGINT)
                    break
                time.sleep(.5)
            state['localization_exit'] = job.wait()
        except BaseException:
            job.send_signal(signal.SIGINT)
            while job.poll() is None:
                try:
                    job.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print('等待原停止流程退出，再恢复PC校时；必要时使用物理急停。', flush=True)
            raise
    finally:
        if changed:
            restoration = subprocess.run(['sudo','-n','systemctl','start','systemd-timesyncd'])
            state['restore_exit'] = restoration.returncode
        state['service_active_after'] = subprocess.run(['systemctl','is-active','--quiet','systemd-timesyncd']).returncode == 0
        state['after_status'] = (subprocess.run(['timedatectl','timesync-status'],capture_output=True,text=True).stdout
                                 if state['service_active_after'] else 'inactive; no D-Bus query')
        (folder/'report.json').write_text(json.dumps(state,indent=2)+'\n')
        print('PC校时恢复核对:', state['service_active_after'], '记录:',folder,flush=True)
        if changed and state.get('restore_exit') != 0:
            raise RuntimeError('恢复systemd-timesyncd失败，需立即人工恢复')
    raise SystemExit(state.get('localization_exit', 2))

if __name__ == '__main__':
    main()
