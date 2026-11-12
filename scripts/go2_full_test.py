#!/usr/bin/env python3
"""从 PC 启动现有 MAP + S1 单目标流程；默认离线计划。"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
BRANCH = 'feature/map-approach-s1-grasp-integration'
HOST = 'unitree@192.168.123.18'
KEY = '/home/czx666/.ssh/id_ed25519_go2'
REMOTE = '/home/unitree/go2_map_s1'
ARM = '/home/unitree/s1-arm-control'
ARM_PY = '/home/unitree/s1-arm-py310/bin/python'
SSH = ['ssh', '-i', KEY, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
       '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=3']

# Camera process group belongs to this connection. EOF tears it down without
# matching/killing another operator's processes. S1 hold deliberately persists.
CAMERA_OWNER = r'''
import os, select, signal, subprocess, sys
p = subprocess.Popen(['bash', sys.argv[1]], start_new_session=True,
                     env=dict(os.environ, GO2_D435_SERIAL='213622070598'))
try:
    while p.poll() is None:
        if select.select([sys.stdin], [], [], 0.2)[0]:
            if not os.read(sys.stdin.fileno(), 1):
                break
    if p.poll() is not None:
        raise SystemExit(p.returncode or 1)
finally:
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try: p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            p.wait(timeout=5)
'''


def run(argv, **kwargs):
    print('+', shlex.join(map(str, argv)), flush=True)
    return subprocess.run(list(map(str, argv)), check=True, **kwargs)


def remote(command, **kwargs):
    return run([*SSH, HOST, command], timeout=20, **kwargs)


def zero_hold(state):
    return (state.get('state') == 'ZERO_HOLD' and state.get('busy') is False
            and state.get('navigation_allowed') is True)


def ros_probe():
    """Require multiple recent samples and an actual TF, without publishing."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node('go2_full_test_readiness')
    specs = [('/camera/camera/color/image_raw', Image),
             ('/camera/camera/aligned_depth_to_color/image_raw', Image),
             ('/utlidar/cloud', PointCloud2), ('/utlidar/imu', Imu),
             ('/state_estimation', Odometry)]
    samples = {name: [] for name, _ in specs}

    def receive(msg, name):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if abs(time.time() - stamp) < 1.0:
            samples[name].append(time.monotonic())
    node._readiness_subscriptions = [node.create_subscription(
        typ, name, lambda msg, name=name: receive(msg, name), qos_profile_sensor_data)
        for name, typ in specs]
    buffer = Buffer()
    node._readiness_listener = TransformListener(buffer, node)
    end = time.monotonic() + 60
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            ready = all(len(ts) >= 5 and ts[-1]-ts[0] >= 0.3 and now-ts[-1] < 1
                        for ts in samples.values())
            if ready and buffer.can_transform('map', 'vehicle', rclpy.time.Time()):
                tf = buffer.lookup_transform('map', 'vehicle', rclpy.time.Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                if abs(time.time()-stamp) < 1:
                    if node.count_publishers('/cmd_vel_raw'):
                        raise RuntimeError('已有速度控制器，拒绝启动')
                    print('READINESS_OK: 连续新鲜传感器/里程计 + map->vehicle TF', flush=True)
                    return
        raise RuntimeError('连续数据/新鲜 TF 未就绪: ' + str({k: len(v) for k, v in samples.items()}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


class Workflow:
    def __init__(self, args):
        self.args = args
        self.work = args.work_dir.resolve()
        self.children = []
        self.folder = ROOT / 'camera_ws/records' / ('full_' + str(time.time_ns()))
        self.overlay = self.work / '.onekey/install/setup.bash'
        self.env_prefix = ''

    def status(self):
        result = remote(shlex.join(['/usr/bin/python3', ARM+'/scripts/arm_task_client.py',
                                    'status']), capture_output=True, text=True)
        return json.loads(result.stdout)

    def spawn(self, argv, label, **kwargs):
        stream = (self.folder / (label+'.log')).open('x')
        try:
            p = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True, **kwargs)
        except BaseException:
            stream.close()
            raise
        self.children.append((p, stream, label))
        print(f'{label}: PID={p.pid}, 日志={stream.name}', flush=True)
        return p

    def healthy(self):
        for p, _, name in self.children:
            if p.poll() is not None:
                raise RuntimeError(f'{name} 已退出({p.returncode})，查看 {self.folder}')

    def wait_job(self, p):
        while p.poll() is None:
            self.healthy()
            time.sleep(0.2)
        if p.returncode:
            raise RuntimeError(f'子步骤失败 exit={p.returncode}，查看 {self.folder}')

    def shell(self, command):
        return ['bash', '-c', self.env_prefix + command]

    def prepare(self):
        for path in [ROOT/'deploy_ws/install/setup.bash', Path('/opt/ros/humble/setup.bash'),
                     Path(KEY)]:
            if not path.exists():
                raise RuntimeError(f'缺少依赖: {path}')
        # A separate clone has its own Git metadata; never switch the user's branch.
        if not self.work.exists():
            run(['git', 'clone', '--no-hardlinks', '--branch', BRANCH, ROOT, self.work])
        if subprocess.check_output(['git', '-C', str(self.work), 'status', '--porcelain'],
                                   text=True).strip():
            raise RuntimeError('集成目录有未提交改动，保留现场并停止: '+str(self.work))
        branch = subprocess.check_output(['git', '-C', str(self.work), 'branch',
                                          '--show-current'], text=True).strip()
        if branch != BRANCH:
            raise RuntimeError('集成目录分支错误: '+branch)
        # Fetch latest without silently falling back to an old cached revision.
        url = subprocess.check_output(['git', '-C', str(ROOT), 'remote', 'get-url',
                                       'origin'], text=True).strip()
        run(['git', '-C', self.work, 'fetch', url, BRANCH])
        run(['git', '-C', self.work, 'merge', '--ff-only', 'FETCH_HEAD'])
        head = subprocess.check_output(['git', '-C', str(self.work), 'rev-parse', 'HEAD'],
                                       text=True).strip()
        fetched = subprocess.check_output(['git', '-C', str(self.work), 'rev-parse',
                                          'FETCH_HEAD'], text=True).strip()
        if head != fetched:
            raise RuntimeError('集成目录存在本地额外提交，拒绝当作远端最新版本')
        for name, target in [('deploy_ws', ROOT/'deploy_ws'),
                             ('camera_ws/records', ROOT/'camera_ws/records')]:
            link = self.work/name
            if not link.exists() and not link.is_symlink():
                link.symlink_to(target, target_is_directory=True)
            if link.resolve() != target.resolve():
                raise RuntimeError('集成目录链接指向不符: '+str(link))
        self.folder.mkdir(parents=True)
        (self.folder/'manifest.json').write_text(json.dumps(dict(
            commit=head, branch=BRANCH, mode=self.args.mode,
            work_dir=str(self.work), physical_result='NOT VERIFIED'), indent=2))
        self.env_prefix = '; '.join('source '+shlex.quote(str(p))+' || exit' for p in [
            Path('/opt/ros/humble/setup.bash'), ROOT/'deploy_ws/install/setup.bash'])+'; '
        run(self.shell('cd '+shlex.quote(str(self.work))+' && '+shlex.join([
            'colcon', '--log-base', str(self.work/'.onekey/log'), 'build',
            '--base-paths', 'camera_ws/src', '--build-base', str(self.work/'.onekey/build'),
            '--install-base', str(self.overlay.parent),
            '--packages-select', 'go2_science_perception'])))
        self.env_prefix += '; '.join('source '+shlex.quote(str(p))+' || exit' for p in [
            self.overlay, self.work/'camera_ws/scripts/science_approach_pc_env.bash'])+'; '
        self.env_prefix += 'export SCIENCE_APPROACH_OVERLAY='+shlex.quote(str(self.overlay))+'; '

    def execute(self):
        self.prepare()
        run(['ip', '-br', 'addr', 'show', 'enp12s0'])
        route = subprocess.check_output(['ip', 'route', 'get', '192.168.123.18'], text=True)
        if 'dev enp12s0' not in route or 'src 192.168.123.99' not in route:
            raise RuntimeError('有线路由/IP不符合现有DDS配置: '+route)
        # Do not change time underneath an old camera/SLAM session.
        remote("command -v pgrep >/dev/null || exit 1; "
               "if pgrep -f '[r]ealsense2_camera_node|[s]1_task_host.py'; then exit 1; fi")
        result = subprocess.run(['pgrep', '-af',
                                 'd435_slam_target_validation.launch.py|[l]aserMapping|'
                                 '[s]cience_target_approach|[g]o2_safety_gate'],
                                capture_output=True, text=True)
        if result.returncode != 1:
            raise RuntimeError('旧定位/控制进程或进程检查失败，先按手册停止: '+result.stdout)
        device_probe = (
            "import pyrealsense2 as r; from pathlib import Path; "
            "s={d.get_info(r.camera_info.serial_number) for d in r.context().query_devices()}; "
            "assert {'213622070598','231622302209'} <= s, s; "
            "assert Path('/dev/ttyUSB0').exists(); print(s)")
        remote(shlex.join([ARM_PY, '-c', device_probe]))
        # If a service exists it must be idle before clock sync or adapter upload.
        existing = remote("pgrep -f '[r]un_arm_integration.py' || true",
                          capture_output=True, text=True).stdout.strip()
        if existing and not zero_hold(self.status()):
            raise RuntimeError('已有S1服务未处于ZERO_HOLD，保留现场并停止')
        run(['/usr/bin/python3', self.work/'camera_ws/scripts/sync_go2_clock.py'])
        # Bounded RTT check, no passwords in logs or command arguments.
        for _ in range(3):
            start = time.time()
            stamp = float(remote('date +%s.%N', capture_output=True, text=True).stdout)
            end = time.time()
            if end-start > 0.2 or abs(stamp-(start+end)/2) > 0.05:
                raise RuntimeError('对时偏差/SSH往返过大，停止；重新检查网络和时钟')
        remote('mkdir -p '+REMOTE)
        run(['scp', '-i', KEY, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
             self.work/'camera_ws/scripts/launch_go2_d435_foxy.sh',
             self.work/'camera_ws/scripts/s1_task_host.py',
             self.work/'camera_ws/src/go2_science_perception' /
             'go2_science_perception/s1_task_sequence.py',
             HOST+':'+REMOTE+'/'])
        # sudo may prompt on the terminal; never capture or save the password.
        run(['ssh', '-t', '-i', KEY, '-o', 'ConnectTimeout=5', HOST,
             'sudo systemctl stop go2-remote-router.service'])
        remote('if systemctl is-active --quiet go2-remote-router.service; then exit 1; fi')
        if not existing:
            remote('command -v fuser >/dev/null || exit 1; '
                   'if fuser /dev/ttyUSB0; then exit 1; fi')
            remote('cd '+ARM+' && OPENBLAS_NUM_THREADS=1 '+ARM_PY +
                   ' -B scripts/run_arm_integration.py')
            arm_log = REMOTE+'/s1_hold_'+self.folder.name+'.log'
            remote('cd '+ARM+' && nohup env OPENBLAS_NUM_THREADS=1 '+ARM_PY +
                   ' -uB scripts/run_arm_integration.py --dev /dev/ttyUSB0 --execute >' +
                   shlex.quote(arm_log)+' 2>&1 < /dev/null &',
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print('S1保持服务日志: '+HOST+':'+arm_log, flush=True)
        deadline = time.monotonic()+45
        while True:
            try:
                state = self.status()
            except (subprocess.SubprocessError, ValueError):
                state = {}
            if zero_hold(state):
                break
            if state.get('state', '').startswith('FAULT') or time.monotonic() > deadline:
                raise RuntimeError('S1未就绪，保留保持服务: '+str(state))
            time.sleep(1)
        self.spawn([*SSH, HOST, shlex.join(['/usr/bin/python3', '-c', CAMERA_OWNER,
                    REMOTE+'/launch_go2_d435_foxy.sh'])], 'camera', stdin=subprocess.PIPE)
        slam_command = ['ros2', 'launch', 'go2_science_perception',
                        'd435_slam_target_validation.launch.py',
                        'remote_camera:=true', 'use_rviz:=false']
        self.spawn(self.shell(shlex.join(slam_command)), 'slam')
        probe = subprocess.Popen(self.shell(shlex.join(['/usr/bin/python3', __file__, '--probe'])))
        try:
            self.wait_job(probe)
        finally:
            if probe.poll() is None:
                probe.terminate()
                probe.wait(timeout=5)
        if not zero_hold(self.status()):
            raise RuntimeError('运动前S1状态变化，停止')
        command = ['bash', str(self.work/'camera_ws/scripts/run_map_s1_task.sh'), '--execute',
                   '--search-distance', '5', '--search-timeout', '60']
        if not (os.environ.get('DISPLAY') and shutil.which('gnome-terminal')):
            command.append('--no-log-terminal')
        if self.args.mode == 'full':
            profile = self.work/'camera_ws/src/go2_science_perception/config'/(
                'science_s1_posture_go2_20261112.json')
            command += ['--full', '--posture-profile', str(profile)]
        print('开始真实运动；Ctrl+C请求停止。物理遥控急停保持可用。', flush=True)
        task = subprocess.Popen(self.shell(shlex.join(command)))
        try:
            self.wait_job(task)
        finally:
            if task.poll() is None:
                task.send_signal(signal.SIGINT)
                try:
                    task.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    # Keep support nodes alive rather than tearing out localization
                    # under a controller which has not acknowledged shutdown.
                    print('任务未退出：立即物理急停；保留定位和S1，等待原任务退出。', flush=True)
                    task.wait()

    def close(self):
        for p, stream, label in reversed(self.children):
            try:
                if p.poll() is None:
                    if label == 'camera':
                        p.stdin.close()
                    else:
                        os.killpg(p.pid, signal.SIGINT)
                    try:
                        p.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGTERM)
                        p.wait(timeout=5)
            finally:
                stream.close()
        print('本轮相机/定位清理结束；S1服务不会自动关闭，停机请按手册人工处理。', flush=True)
        if self.folder.exists():
            print('日志: '+str(self.folder)+'；物理停止/抓取结果需现场确认。', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='确认路线空旷、急停在手、S1初始位和支撑已核对；执行真实运动')
    parser.add_argument('--mode', choices=['full', 'dock'], default='full')
    parser.add_argument('--work-dir', type=Path, default=Path('/tmp/go2_onekey_s1'))
    parser.add_argument('--probe', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.probe:
        ros_probe()
        return
    if not args.execute:
        print('离线计划（未连接硬件、未写文件）：\n'
              '恢复独立最新S1分支 → 单包构建 → 有线/设备/旧进程检查 → 对时 → '
              '同步适配器 → 停旧自定义路由 → S1回零保持 → D435 → SLAM/感知/TF → '
              '连续数据检查 → SEARCH 5m/0.25m/s → STOP_CONFIRM 6s → 三自由度停靠 → STOP' +
              (' → 趴下 → S1抓取 → ZERO_HOLD → 起立' if args.mode == 'full' else '') +
              '\n真实执行：bash go2_test.sh --execute\n'
              '无自动避障；退出保留S1保持服务。完整流程真机验收：NOT VERIFIED。')
        return
    with open('/tmp/go2_full_test.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        workflow = Workflow(args)

        def interrupted(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupted)
        try:
            workflow.execute()
        finally:
            # A second Ctrl+C must not interrupt ordered controller cleanup.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            workflow.close()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError, ValueError) as exc:
        print('停止：'+str(exc), file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print('收到中止，请现场确认已停止；S1保持服务保留。', file=sys.stderr)
        sys.exit(130)
