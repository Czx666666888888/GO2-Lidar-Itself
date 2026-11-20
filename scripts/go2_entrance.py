#!/usr/bin/env python3
"""Run the entrance stage under the PC clock wrapper before handing off."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys

from go2_full_test import Workflow, remote, zero_hold

ROOT = Path(__file__).resolve().parents[1]


def verify_ready():
    route = subprocess.check_output(['ip', 'route', 'get', '192.168.123.18'],
                                    text=True, timeout=3).split()
    if ('dev' not in route or route[route.index('dev') + 1] != os.environ.get('GO2_IFACE', 'enp12s0')
            or 'via' in route or 'src' not in route or route[route.index('src') + 1] != '192.168.123.99'):
        raise RuntimeError('洞口段需要 PC 的 GO2 有线直连路由和 192.168.123.99 地址')
    router = remote('systemctl show go2-remote-router.service -p ActiveState --value',
                    capture_output=True, text=True)
    if router.stdout.strip() != 'inactive':
        raise RuntimeError('洞口段要求旧自定义遥控服务已处于 inactive')
    if not zero_hold(Workflow.status()):
        raise RuntimeError('洞口段要求已有健康的 S1 ZERO_HOLD 服务')


def run_entrance(distance_cm, clockwise_deg, report_dir):
    command = ['bash', str(ROOT/'avoid_ws/run.sh'), '--execute',
               '--entrance', distance_cm, clockwise_deg]
    print('洞口段使用 Sport 高层接口和里程计闭环。', flush=True)
    with (report_dir/'entrance.log').open('x') as log:
        print(f'洞口段日志 {log.name}', flush=True)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            code = child.wait(timeout=180)
            if code:
                raise subprocess.CalledProcessError(code, command)
        finally:
            if child.poll() is None:
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    print('洞口段未退出，请立即使用物理急停；等待控制进程退出。', flush=True)
                    child.wait()


def main():
    distance_cm, clockwise_deg, report_dir, *handoff = sys.argv[1:]

    def interrupted(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    with open('/tmp/go2_full_test.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        verify_ready()
        run_entrance(distance_cm, clockwise_deg, Path(report_dir))
        verify_ready()
    print('洞口段闭环到位、停稳，开始原流程。', flush=True)
    os.execvp(handoff[0], handoff)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print(f'洞口段停止：{error}', file=sys.stderr)
        sys.exit(1)
