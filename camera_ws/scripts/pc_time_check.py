#!/usr/bin/env python3
"""Controlled read-only hardware run; temporarily pause and restore PC time service."""
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
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--mode', choices=('dock',), default='dock')
    args = parser.parse_args()
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
    state['before_status'] = subprocess.run(['timedatectl','timesync-status'],capture_output=True,text=True).stdout
    state['was_active'] = subprocess.run(['systemctl','is-active','--quiet','systemd-timesyncd']).returncode == 0
    print('记录PC在线校时时钟20秒；随后临时暂停校时，不改变频率参数。', flush=True)
    state['online'] = probe(20)
    changed = False
    try:
        if state['was_active']:
            run(['sudo','-v'])
            # Mark before mutation so interruption during systemctl still restores.
            changed = True
            run(['sudo','-n','systemctl','stop','systemd-timesyncd'])
        print('记录暂停校时后的时钟20秒，然后启动IMU映射只读定位。', flush=True)
        state['paused'] = probe(20)
        state['paused_service_active'] = subprocess.run(['systemctl','is-active','--quiet','systemd-timesyncd']).returncode == 0
        env = dict(os.environ, GO2_TIME_MAPPING_MODE='imu', GO2_PC_TIME_HELD='1')
        if args.execute:
            command = ['bash',str(ROOT/'go2_test.sh'),'--execute','--mode','dock']
        else:
            run(['/usr/bin/python3', ROOT/'camera_ws/scripts/sync_go2_clock.py'])
            command = ['bash',str(ENTRY),'--time-mode','imu','--restart-localization',
                       '--require-target','--duration','60','--label','pc_paused_imu']
        job = subprocess.Popen(command,env=env)
        try:
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
        state['after_status'] = subprocess.run(['timedatectl','timesync-status'],capture_output=True,text=True).stdout
        (folder/'report.json').write_text(json.dumps(state,indent=2)+'\n')
        print('PC校时恢复核对:', state['service_active_after'], '记录:',folder,flush=True)
        if changed and state.get('restore_exit') != 0:
            raise RuntimeError('恢复systemd-timesyncd失败，需立即人工恢复')
    raise SystemExit(state.get('localization_exit', 2))

if __name__ == '__main__':
    main()
