#!/usr/bin/env python3
"""从 PC 启动现有 MAP + S1 单目标流程；默认离线计划。"""
import argparse
import base64
from contextlib import contextmanager
from concurrent.futures import ThreadPoolExecutor
import threading
from go2_build_fingerprint import fingerprint
from s1_fusion_delivery import (ensure_fusion_package, REMOTE_PACKAGE, PACKAGE_VERSION,
                            ARCHIVE_SHA256)
from go2_camera_handoff import RELEASE_CHECK, service_requests, atomic_json
from go2_three_grasps import run_three_grasps, progress_snapshot
import fcntl
import json
import os
from pathlib import Path
import select
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
ARM = '/home/unitree/s1-arm-control'  # legacy
ARM_V2 = REMOTE_PACKAGE
ARM_PY = '/home/unitree/s1-arm-py310/bin/python'
ARM_DEVICES = ('/dev/ttyUSB0', '/dev/ttyUSB1')
SSH = ['ssh', '-i', KEY, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
       '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=3',
       '-o', 'ControlMaster=auto', '-o', 'ControlPersist=60',
       '-o', 'ControlPath=/tmp/go2-startup-%C']
CLOCK_RESPONDER = (
    "import sys,time; print('CLOCK_READY',flush=True); "
    "exec('for line in sys.stdin:\\n print(time.time(),flush=True)')")

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
    return run([*SSH, HOST, command], timeout=kwargs.pop('timeout',20), **kwargs)


INVENTORY_SCRIPT = r"""
import json, os, subprocess
from pathlib import Path
import pyrealsense2 as r
def pids(pattern):
    out=subprocess.run(['pgrep','-f',pattern],capture_output=True,text=True)
    return [int(x) for x in out.stdout.split()]
def arm_entry(argv, cwd):
    # Match the actual Python script operand, never text embedded in a shell/-c command.
    if not argv or not Path(argv[0]).name.startswith('python'):
        return None
    for value in argv[1:]:
        if value in ('-c', '-m'):
            return None
        if value.startswith('-'):
            continue
        if Path(value).name not in ('run_arm_integration.py', 'run_go2_left_grasp.py'):
            return None
        path=Path(value)
        return (path if path.is_absolute() else Path(cwd)/path).resolve()
    return None
arm=[]
entries={}
arguments={}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        argv=proc.joinpath('cmdline').read_bytes().decode().split('\0')
        entry=arm_entry(argv,proc.joinpath('cwd').resolve())
    except (OSError,UnicodeError):
        continue  # Process exited during read-only enumeration.
    if entry is not None:
        pid=int(proc.name);arm.append(pid);entries[pid]=entry;arguments[pid]=argv
arm_devices=[]
arm_modes=[]
arm_roots=[]
for pid in arm:
    argv=arguments[pid]
    device='/dev/ttyUSB0'  # Original service parser default.
    for index, value in enumerate(argv):
        if value=='--dev': device=argv[index+1]
        elif value.startswith('--dev='): device=value.split('=',1)[1]
    opened=set()
    for fd in (Path('/proc')/str(pid)/'fd').iterdir():
        try:
            target=os.readlink(fd).removesuffix(' (deleted)')
            if target in ('/dev/ttyUSB0','/dev/ttyUSB1'): opened.add(target)
        except OSError: pass
    if len(opened)==1: device=next(iter(opened))
    arm_devices.append(device)
    arm_modes.append('left' if entries[pid].name=='run_go2_left_grasp.py' else 'right')
    arm_roots.append(str(entries[pid].parent.parent))
print(json.dumps(dict(arm=arm, arm_devices=arm_devices, arm_modes=arm_modes, arm_roots=arm_roots, task=pids('[s]1_task_host.py'),
    camera=pids('[r]ealsense2_camera_node'),
    serial_devices=[device for device in ('/dev/ttyUSB0','/dev/ttyUSB1') if Path(device).exists()],
    cameras=[d.get_info(r.camera_info.serial_number) for d in r.context().query_devices()],
    router=subprocess.run(['systemctl','is-active','--quiet','go2-remote-router.service']).returncode==0)))
"""


def select_arm_device(devices, running_devices):
    candidates = [device for device in ARM_DEVICES if device in devices]
    if len(candidates) != 1:
        raise RuntimeError('S1串口必须唯一（ttyUSB0/ttyUSB1），保留现场: '+str(candidates))
    device = candidates[0]
    if running_devices and running_devices != [device]:
        raise RuntimeError('已有S1服务仍绑定其他/未知串口，保留服务，禁止重复连接: '+
                           str(running_devices)+'; 当前设备='+device)
    return device


def zero_hold(state):
    return (state.get('state') == 'ZERO_HOLD' and state.get('busy') is False
            and state.get('navigation_allowed') is True
            and state.get('at_zero', True) is True
            and state.get('communication_state', 'OK') == 'OK'
            and state.get('readiness_state', 'ZERO_HOLD') == 'ZERO_HOLD')


def require_clean_checkout(work):
    """Allow only our verified workspace links; preserve all source changes."""
    links = {'deploy_ws': ROOT/'deploy_ws', 'camera_ws/records': ROOT/'camera_ws/records'}
    status = subprocess.check_output(
        ['git', '-C', str(work), 'status', '--porcelain', '--untracked-files=all', '-z'],
        text=True)
    for entry in filter(None, status.split('\0')):
        name = entry[3:]
        path = work/name
        if (entry.startswith('?? ') and name in links and path.is_symlink()
                and path.resolve() == links[name].resolve()):
            continue
        raise RuntimeError('集成目录有未提交改动，保留现场并停止: '+str(work)+' '+entry)


class ClockOffsetError(RuntimeError):
    """Clock skew with a usable transport; eligible for the existing sync path."""


def verify_clock(argv):
    """Measure live exchanges after SSH authentication, using the same connection."""
    process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               text=True, bufsize=1)

    def receive():
        if not select.select([process.stdout], [], [], 5)[0]:
            raise RuntimeError('时钟测量响应超时')
        line = process.stdout.readline()
        if not line:
            raise RuntimeError('时钟测量连接退出')
        return line.strip()

    try:
        if receive() != 'CLOCK_READY':
            raise RuntimeError('时钟测量握手失败')
        for _ in range(5):
            start = time.time()
            process.stdin.write('ping\n')
            process.stdin.flush()
            stamp = float(receive())
            end = time.time()
            rtt, offset = end-start, stamp-(start+end)/2
            print(f'CLOCK_CHECK rtt={rtt:.6f}s offset={offset:+.6f}s', flush=True)
            if rtt > 0.2:
                raise RuntimeError('SSH往返过大，停止；重新检查网络和时钟')
            if abs(offset) > 0.05:
                raise ClockOffsetError('对时偏差超过50ms，停止；需要恢复伴随主机时钟')
    finally:
        process.stdin.close()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)
        process.stdout.close()


def detached_launcher(argv, cwd, log):
    """Close SSH descriptors before leaving the persistent S1 hold process."""
    return (
        "import os,subprocess; "
        f"log=open({str(log)!r},'x'); "
        f"p=subprocess.Popen({argv!r}, cwd={str(cwd)!r}, "
        "env=dict(os.environ,OPENBLAS_NUM_THREADS='1'), "
        "stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT, "
        "close_fds=True,start_new_session=True); print('S1_PID',p.pid)")


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
             ('/utlidar/transformed_cloud', PointCloud2),
             ('/utlidar/transformed_imu', Imu),
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
        self.detector = ('hsv' if getattr(args, 'forward_response_check', False)
                         else getattr(args, 'detector', 'yolo'))
        self.vision_model = self.vision_python = None
        self.vision_version = self.vision_sha256 = None
        self.folder = ROOT / 'camera_ws/records' / ('full_' + str(time.time_ns()))
        self.overlay = self.work / '.onekey/install/setup.bash'
        self.env_prefix = ''
        self.stages = []
        self.timing_lock = threading.Lock()
        self.camera_paused = False
        self.current_round = 0
        self.completed_targets = []

    @contextmanager
    def stage(self, name):
        started = time.monotonic()
        outcome = 'OK'
        try:
            yield
        except BaseException:
            outcome = 'FAILED'
            raise
        finally:
            row = dict(stage=name, elapsed_sec=time.monotonic()-started, outcome=outcome)
            with self.timing_lock:
                self.stages.append(row)
                print('STAGE '+json.dumps(row), flush=True)
                if self.folder.exists():
                    (self.folder/'startup_timing.json').write_text(json.dumps(self.stages, indent=2)+'\n')

    def arm_command(self, command='status'):
        if command not in ('status','grasp'):
            raise RuntimeError('S1 launcher supports only status/grasp; startup handles zero')
        result = remote(shlex.join(['/usr/bin/python3', ARM_V2+'/scripts/arm_task_client.py',
                                    command]), capture_output=True, text=True)
        return json.loads(result.stdout)

    def status(self):
        return self.arm_command()

    def probe(self, label, timeout=8, scope='all', require_target=False):
        target = self.folder/(label+'.json')
        command = ['/usr/bin/python3', str(self.work/'camera_ws/scripts/go2_startup_probe.py'),
                   '--timeout', str(timeout), '--scope', scope, '--report', str(target)]
        if require_target:
            command.append('--require-target')
        if scope != 'native' and not getattr(self.args, 'forward_response_check', False):
            command += ['--detector', self.detector]
            if self.detector == 'yolo':
                from go2_color_block_test import model_identity, V3_SHA256
                explicit_model = getattr(self.args, 'model_path', None)
                expected = self.vision_sha256 or (
                    model_identity(explicit_model)[1] if explicit_model else V3_SHA256)
                command += ['--model-sha256', expected]
        with self.stage('probe_'+label):
            result = subprocess.run(self.shell(shlex.join(command)))
        if result.returncode not in (0, 2) or not target.exists():
            raise RuntimeError('启动只读探针未完成，查看日志: '+label)
        report=json.loads(target.read_text())
        if any('existing velocity controller' in error for error in report.get('errors',[])):
            raise RuntimeError('已有其他速度发布者，保留定位并拒绝新任务')
        return report

    def configure_vision(self):
        """Validate model identity before S1 initialization or any navigation."""
        if getattr(self.args, 'view', False) and not os.environ.get('DISPLAY'):
            raise RuntimeError('--view需要桌面DISPLAY；无桌面时去掉--view')
        from go2_color_block_test import detector_assets, model_identity
        from types import SimpleNamespace
        assets = SimpleNamespace(detector=self.detector,
            model_path=getattr(self.args, 'model_path', None),
            python_executable=getattr(self.args, 'python_executable', None))
        self.vision_model, self.vision_python = detector_assets(assets, self.work)
        if self.detector == 'yolo':
            self.vision_version, self.vision_sha256 = model_identity(self.vision_model)
        path = self.folder/'manifest.json'
        data = json.loads(path.read_text())
        data.update(detector=self.detector, view=bool(getattr(self.args, 'view', False)),
                    model=str(self.vision_model) if self.detector == 'yolo' else None,
                    model_version=self.vision_version if self.detector == 'yolo' else None,
                    model_sha256=self.vision_sha256 if self.detector == 'yolo' else None)
        path.write_text(json.dumps(data, indent=2)+'\n')
        if self.detector == 'yolo':
            for key, value in dict(OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                    YOLO_CONFIG_DIR=str(self.folder/'ultralytics'),
                    MPLCONFIGDIR=str(self.folder/'matplotlib')).items():
                self.env_prefix += 'export '+key+'='+shlex.quote(value)+'; '

    def perception_command(self):
        command = ['ros2', 'launch', 'go2_science_perception',
                   'd435_slam_target_validation.launch.py', 'remote_camera:=true',
                   'use_rviz:=false', 'detector:='+self.detector]
        if self.detector == 'yolo':
            if self.args.mode == 'full':
                command += ['multi_target_memory:=true',
                            'target_memory_path:='+str(self.folder/'target_memory.json')]
            command += ['yolo_model:='+str(self.vision_model),
                        'yolo_python:='+str(self.vision_python)]
        return command

    @property
    def camera_handoff_directory(self):
        base = self.folder/'camera_handoff'
        return base/('round_%02d' % self.current_round) if self.current_round else base

    def task_command(self):
        command = ['bash', str(self.work/'camera_ws/scripts/run_map_s1_task.sh'), '--execute',
                   '--grasp-side', self.args.grasp_side, '--search-distance', '5', '--search-timeout', '60']
        if getattr(self.args, 'forward_response_check', False):
            command += ['--forward-response-check', '--response-distance-limit',
                        str(getattr(self.args, 'response_distance_limit', .20))]
        if getattr(self.args, 'view', False):
            command.append('--preview-image')
        if not (os.environ.get('DISPLAY') and shutil.which('gnome-terminal')):
            command.append('--no-log-terminal')
        if self.args.mode == 'full':
            profile = self.work/'camera_ws/src/go2_science_perception/config/science_s1_posture_go2_20261112.json'
            command += ['--full', '--posture-profile', str(profile)]
            if self.detector == 'yolo':
                command.append('--multi-target-memory')
            command += ['--evidence-directory', str(self.folder), '--continuous-search',
                        '--navigation-trail', str(self.folder/'navigation_trail.json'),
                        '--completed-targets-json', json.dumps(self.completed_targets),
                        '--round-result', str(self.folder/('round_%02d_result.json' % self.current_round))]
            command += ['--camera-handoff-directory', str(self.camera_handoff_directory)]
            if getattr(self.args, 'left_geometry_verified', False):
                command.append('--left-geometry-verified')
        return command

    def deploy_s1(self):
        # Only called while the single-owner service is absent. Never hot-patch it.
        source = self.work/'camera_ws/s1_service'
        manifest = json.loads((source/'source_manifest.json').read_text())
        files = {name: dict(original=meta['original_sha256'],
                           previous=meta.get('accepted_previous_sha256', []),
                           content=base64.b64encode((source/name).read_bytes()).decode())
                 for name, meta in manifest['files'].items()}
        script = r"""
import base64, hashlib, json, pathlib, shutil, sys, subprocess
root=pathlib.Path(sys.argv[1]); files=json.loads(sys.stdin.read())
if subprocess.run(['pgrep','-f','[r]un_arm_integration.py'],stdout=subprocess.DEVNULL).returncode==0:
    raise SystemExit('S1 service running: preserve it, no installation')
for name, item in files.items():
    p=root/name; new=base64.b64decode(item['content'])
    if p.exists() and hashlib.sha256(p.read_bytes()).hexdigest() not in (
            item['original'], *item.get('previous', []), hashlib.sha256(new).hexdigest()):
        raise SystemExit('S1 source differs from delivery; preserve and stop: '+name)
for name,item in files.items():
    p=root/name; new=base64.b64decode(item['content'])
    if p.exists() and p.read_bytes()==new: continue
    if p.exists():
        versioned=root/'.go2-startup-backups'/hashlib.sha256(p.read_bytes()).hexdigest()/name
        versioned.parent.mkdir(parents=True,exist_ok=True)
        if not versioned.exists(): shutil.copy2(p,versioned)
        backup=root/'.go2-startup-original'/name
        backup.parent.mkdir(parents=True,exist_ok=True)
        if not backup.exists(): shutil.copy2(p,backup)
    p.parent.mkdir(parents=True,exist_ok=True)
    temp=p.with_suffix(p.suffix+'.startup-tmp'); temp.write_bytes(new); temp.replace(p)
print('S1_EXTENSION_INSTALLED: original serial owner retained')
"""
        remote(shlex.join(['/usr/bin/python3','-c',script,ARM]),
               input=json.dumps(files),text=True)

    def ensure_s1(self, running, device='/dev/ttyUSB0', running_modes=None):
        """S1 fusion service reads feedback and performs controlled startup zero return."""
        side=self.args.grasp_side
        running_modes=running_modes or []
        entry='run_go2_left_grasp.py' if side=='left' else 'run_arm_integration.py'
        if running:
            if len(running_modes)!=1 or (side != 'auto' and running_modes[0]!=side):
                raise RuntimeError('S1抓取侧别与请求不一致；空闲ZERO_HOLD后手动切换服务: '+
                                   str(running_modes)+' requested='+side)
            state=self.status()
            if zero_hold(state):
                if self.args.mode == 'full':
                    ensure_fusion_package(ROOT, SSH, HOST, running=True)
                print('REUSE S1 fusion '+side+' ZERO_HOLD',flush=True)
                return
            if state.get('busy') or state.get('state') in ('STARTING','QUEUED','GRASPING','DUMPING','RETURNING_ZERO'):
                raise RuntimeError('S1正在初始化、回零或执行抓取，保留唯一串口所有者: '+str(state))
            raise RuntimeError('S1真实状态异常，禁止创建第二串口所有者: '+str(state))
        package=ensure_fusion_package(ROOT,SSH,HOST,running=False)
        remote('mkdir -p '+REMOTE)
        arm_log=REMOTE+'/s1_fusion_'+side+'_'+self.folder.name+'.log'
        argv=[ARM_PY,'-uB','scripts/'+entry,'--dev','auto','--execute']
        started=remote(shlex.join(['/usr/bin/python3','-c',detached_launcher(argv,package,arm_log)]),
                       capture_output=True,text=True)
        marker=next((line.split()[1] for line in started.stdout.splitlines()
                     if line.startswith('S1_PID ')),None)
        if not marker or not marker.isdigit():
            raise RuntimeError('S1融合服务启动未报告进程PID: '+started.stdout)
        print('S1_V4_AUTO_ZERO_START '+side+' pid='+marker+' log='+HOST+':'+arm_log,flush=True)
        deadline=time.monotonic()+75
        last=None
        while time.monotonic()<deadline:
            try:
                last=self.status()
                if zero_hold(last):
                    print('S1_V4_ZERO_HOLD_READY '+side,flush=True)
                    return
                if last.get('state') in ('FAULT_HOLD','FAULT_IO','STOPPING','STOPPED'):
                    raise RuntimeError('S1融合服务回零失败: '+str(last))
            except (OSError,subprocess.CalledProcessError,ValueError):
                # Initialization has not yet opened the socket. If the process died,
                # report its log immediately rather than waiting the full timeout.
                try:
                    remote('kill -0 '+marker,timeout=5)
                except subprocess.CalledProcessError:
                    details=remote('tail -n 18 '+shlex.quote(arm_log),
                                   capture_output=True,text=True,timeout=8).stdout
                    raise RuntimeError('S1融合服务启动已退出: '+details)
            time.sleep(.5)
        raise RuntimeError('S1融合服务初始化或回零超时: '+str(last)+' log='+arm_log)

    def spawn(self, argv, label, **kwargs):
        if any(name == label and child.poll() is None for child, _, name in self.children):
            raise RuntimeError(f'{label} 仍在运行，拒绝启动重复进程')
        # A stopped child keeps its evidence; retries retain the logical label
        # used by lifecycle checks, but reserve a distinct log atomically.
        attempt = 1
        while True:
            name = label+'.log' if attempt == 1 else f'{label}_{attempt}.log'
            try:
                stream = (self.folder / name).open('x')
                break
            except FileExistsError:
                attempt += 1
        try:
            p = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True, **kwargs)
        except BaseException:
            stream.close()
            raise
        self.children.append((p, stream, label))
        print(f'{label}: PID={p.pid}, 日志={stream.name}', flush=True)
        return p

    def start_viewers(self):
        if not getattr(self.args, 'view', False):
            return
        self.spawn(self.shell('ros2 run rqt_image_view rqt_image_view '
                              '/science/approach/debug_image'), 'image_viewer')
        if self.args.mode == 'full' and os.environ.get('DISPLAY'):
            config = self.work/'camera_ws/src/go2_science_perception/rviz/d435_slam_target_validation.rviz'
            self.spawn(self.shell(shlex.join(['ros2', 'run', 'rviz2', 'rviz2', '-d', str(config)])),
                       'rviz')

    def healthy(self):
        for p, _, name in self.children:
            if p.poll() is not None and name not in ('image_viewer', 'rviz'):
                raise RuntimeError(f'{name} 已退出({p.returncode})，查看 {self.folder}')

    def wait_job(self, p):
        while p.poll() is None:
            if self.args.mode == 'full':
                service_requests(self.camera_handoff_directory,
                                 self.pause_front_camera, self.resume_front_camera)
            self.healthy()
            time.sleep(0.2)
        if p.returncode:
            raise RuntimeError(f'子步骤失败 exit={p.returncode}，查看 {self.folder}')

    def cameras_released(self, products):
        script = 'PRODUCTS='+repr(products)+'\n'+RELEASE_CHECK
        try:
            remote(shlex.join(['/usr/bin/python3', '-c', script]),
                   capture_output=True, text=True, timeout=8)
        except subprocess.CalledProcessError as exc:
            detail = (exc.stderr or exc.stdout or str(exc)).strip()
            raise RuntimeError('Camera release check failed: '+detail) from exc

    def pause_front_camera(self):
        if self.camera_paused:
            return
        self.camera_paused = True  # Even partial teardown needs a restoration attempt.
        for child, stream, label in list(self.children):
            if label != 'camera':
                continue
            if child.poll() is None:
                child.stdin.close()  # Original owner shuts down its own process group.
                child.wait(timeout=15)
            stream.close()
            self.children.remove((child, stream, label))
        # Reused drivers are also paused, but only when explicitly bound to D435.
        self.stop_invalid_camera()
        self.cameras_released({'0b07', '0b5c'})
        print('D435_PAUSED_AND_RELEASED: D455 may now capture', flush=True)

    def resume_front_camera(self):
        if not self.camera_paused:
            return
        state = self.status()
        if state.get('busy') is not False or state.get('state') in (
                'STARTING', 'QUEUED', 'GRASPING', 'DUMPING', 'RETURNING_ZERO'):
            raise RuntimeError('S1 still active/unknown; keep D435 paused: '+str(state))
        self.cameras_released({'0b5c'})
        # Fail closed if any existing D435 owner survived an incomplete pause.
        self.cameras_released({'0b07'})
        stamp = str(time.time_ns())
        argv = ['env', 'GO2_D435_SERIAL=213622070598',
                'bash', REMOTE+'/launch_go2_d435_foxy.sh']
        script = detached_launcher(argv, REMOTE, REMOTE+'/d435_restored_'+stamp+'.log')
        remote(shlex.join(['/usr/bin/python3', '-c', script]),
               capture_output=True, text=True)
        # Preserve the restored driver for the next run, as for a reused session.
        self.camera_paused = False
        report = self.probe('post_arm_camera_'+stamp, timeout=12, scope='chain')
        if not report.get('ready'):
            raise RuntimeError('D435 restored but full RGB-D/localization/TF chain not ready; see camera probe')
        print('D455_RELEASED_D435_RESTORED: fresh RGB/depth verified; driver retained', flush=True)

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
        require_clean_checkout(self.work)
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
        self.folder.mkdir(parents=True, exist_ok=True)
        (self.folder/'manifest.json').write_text(json.dumps(dict(
            commit=head, branch=BRANCH, mode=self.args.mode,
            arm_package=PACKAGE_VERSION, arm_archive_sha256=ARCHIVE_SHA256,
            time_mapping_mode=os.environ.get('GO2_TIME_MAPPING_MODE', 'legacy'),
            work_dir=str(self.work), physical_result='NOT VERIFIED'), indent=2))
        self.env_prefix = '; '.join('source '+shlex.quote(str(p))+' || exit' for p in [
            Path('/opt/ros/humble/setup.bash'), ROOT/'deploy_ws/install/setup.bash'])+'; '
        stamp = self.work/'.git/go2_onekey_built_inputs'
        build_hash = fingerprint(self.work/'camera_ws/src/go2_science_perception')
        legacy_stamp=self.work/'.git/go2_onekey_built_commit'
        if self.overlay.exists() and not stamp.exists() and legacy_stamp.exists():
            previous=legacy_stamp.read_text().strip()
            unchanged=subprocess.run(['git','-C',str(self.work),'diff','--quiet',previous,'HEAD','--',
                                      'camera_ws/src/go2_science_perception']).returncode==0
            if unchanged:
                stamp.write_text(build_hash+'\n')

        if self.overlay.exists() and stamp.exists() and stamp.read_text().strip() == build_hash:
            print('REUSE build inputs unchanged: '+build_hash, flush=True)
        else:
            if stamp.exists():
                stamp.unlink()
            run(self.shell('cd '+shlex.quote(str(self.work))+' && '+shlex.join([
                '/usr/bin/python3', '-X', 'faulthandler', '/usr/bin/colcon',
                '--log-base', str(self.work/'.onekey/log'), 'build',
                '--base-paths', 'camera_ws/src', '--build-base', str(self.work/'.onekey/build'),
                '--install-base', str(self.overlay.parent),
                '--packages-select', 'go2_science_perception'])))
            if not self.overlay.exists():
                raise RuntimeError('构建未产生overlay，停止')
            stamp.write_text(build_hash+'\n')
        self.env_prefix += '; '.join('source '+shlex.quote(str(p))+' || exit' for p in [
            self.overlay, self.work/'camera_ws/scripts/science_approach_pc_env.bash'])+'; '
        self.env_prefix += 'export SCIENCE_APPROACH_OVERLAY='+shlex.quote(str(self.overlay))+'; '
        self.env_prefix += 'export GO2_TIME_MAPPING_MODE='+shlex.quote(
            os.environ.get('GO2_TIME_MAPPING_MODE', 'legacy'))+'; '

    def stop_invalid_camera(self):
        script = r"""
from pathlib import Path
import os, signal, time
matches=[]
for proc in Path('/proc').glob('[0-9]*'):
    try:
        args=(proc/'cmdline').read_bytes().split(b'\0')
        driver=any(a==b'realsense2_camera_node' or a.endswith(b'/realsense2_camera_node') for a in args)
        if driver:
            if not any(b'213622070598' in a for a in args):
                raise RuntimeError('camera ownership/serial unknown; preserve it')
            matches.append(int(proc.name))
    except (FileNotFoundError,PermissionError): pass
for pid in matches:
    try: os.kill(pid,signal.SIGINT)
    except ProcessLookupError: pass
deadline=time.monotonic()+10
while matches and time.monotonic()<deadline:
    matches=[pid for pid in matches if Path('/proc/'+str(pid)).exists()]
    time.sleep(.1)
if matches: raise RuntimeError('invalid camera failed to exit: '+str(matches))
"""
        remote(shlex.join(['/usr/bin/python3','-c',script]))

    def stop_invalid_localization(self):
        matches=[]
        for proc in Path('/proc').glob('[0-9]*'):
            try:
                argv=(proc/'cmdline').read_bytes().split(b'\0')
                text=b' '.join(argv).decode(errors='replace')
                if b'd435_slam_target_validation.launch.py' in argv:
                    pid=int(proc.name)
                    group=os.getpgid(pid)
                    leader=(Path('/proc')/str(group)/'cmdline').read_bytes().decode(errors='replace')
                    if 'd435_slam_target_validation.launch.py' not in leader or 'ros2' not in leader:
                        raise RuntimeError('无效定位由未知外部组管理，保留现场: '+str(pid))
                    if group not in matches:
                        matches.append(group)
            except (FileNotFoundError,ProcessLookupError,PermissionError):
                continue
        for pid in matches:
            os.killpg(pid,signal.SIGINT)
        deadline=time.monotonic()+10
        while matches and time.monotonic()<deadline:
            for child, _, label in self.children:
                if label=='slam':
                    child.poll()  # Reap our exited launch before checking /proc.
            matches=[pid for pid in matches if Path('/proc/'+str(pid)).exists()]
            time.sleep(.1)
        if matches:
            raise RuntimeError('旧定位未退出，不启动重复链: '+str(matches))
        retained=[]
        for child, stream, label in self.children:
            if label=='slam' and child.poll() is not None:
                stream.close()
            else:
                retained.append((child,stream,label))
        self.children=retained
        extra=subprocess.run(['pgrep','-af','[l]aserMapping|[t]ransform_everything|[b]ase_odom_node'],
                             capture_output=True,text=True)
        if extra.returncode!=1:
            raise RuntimeError('仍有外部定位节点，保留现场: '+extra.stdout)

    def execute(self):
        self.folder.mkdir(parents=True, exist_ok=True)
        with self.stage('prepare_code_and_build'):
            self.prepare()
            run(['bash',self.work/'camera_ws/scripts/run_go2_time_diagnostic.sh','--prepare-only'])
            self.env_prefix += 'source '+shlex.quote(str(self.work/'.time_sync/install/setup.bash'))+' || exit; '
            if not self.args.startup_check and not getattr(self.args, 'forward_response_check', False):
                self.configure_vision()
        with self.stage('connection_and_inventory'):
            route=subprocess.check_output(['ip','route','get','192.168.123.18'],text=True)
            if 'dev enp12s0' not in route or 'src 192.168.123.99' not in route:
                raise RuntimeError('有线路由/IP不符: '+route)
            active=subprocess.run(['pgrep','-af','[s]cience_target_approach|[g]o2_safety_gate|[s]1_task_host.py'],
                                  capture_output=True,text=True)
            if active.returncode != 1:
                raise RuntimeError('已有控制任务，拒绝并行启动: '+active.stdout)
            inventory=json.loads(remote(shlex.join([ARM_PY,'-c',INVENTORY_SCRIPT]),
                                        capture_output=True,text=True).stdout)
            if inventory['task']:
                raise RuntimeError('远端抓取任务仍在运行')
            if inventory['arm'] and any(root != ARM_V2 for root in inventory['arm_roots']):
                raise RuntimeError('检测到历史S1服务，与当前交付版本不同；需先停止旧串口进程: '+str(inventory['arm_roots']))
            arm_device=select_arm_device(inventory['serial_devices'], inventory['arm_devices'])
            print('S1_SERIAL '+arm_device, flush=True)
            if not {'213622070598','231622302209'} <= set(inventory['cameras']):
                raise RuntimeError('相机设备不匹配: '+str(inventory['cameras']))
        # --startup-check is deliberately read-only: no hold enable, zero, camera or SLAM start.
        if self.args.startup_check:
            with self.stage('readonly_s1_status'):
                print('S1_STATUS '+json.dumps(self.status() if inventory['arm'] else
                      dict(state='NOT VERIFIED',reason='service absent; serial visible; no second owner opened')),
                      flush=True)
            with self.stage('readonly_current_chain'):
                result=self.probe('readonly_current',timeout=3)
            print('READ_ONLY_CHECK '+json.dumps(result),flush=True)
            return
        with ThreadPoolExecutor(max_workers=1) as pool:
            require_target = (self.args.mode == 'dock' and
                              not getattr(self.args, 'forward_response_check', False))
            # Native/ROS observations are independent of the existing S1 socket.
            pending=pool.submit(self.probe,'existing',3,'all',require_target)
            with self.stage('s1_reuse_or_controlled_zero'):
                if inventory['router']:
                    run(['ssh','-t','-i',KEY,HOST,'sudo systemctl stop go2-remote-router.service'])
                if getattr(self.args, 'forward_response_check', False):
                    if not inventory['arm'] or not zero_hold(self.status()):
                        raise RuntimeError('Response check requires an existing S1 ZERO_HOLD')
                else:
                    self.ensure_s1(bool(inventory['arm']), arm_device, inventory['arm_modes'])
            existing=pending.result()
        if (existing.get('slam_ok') and not getattr(self.args, 'forward_response_check', False)
                and existing.get('detector_mode') != self.detector):
            raise RuntimeError('现有检测器与请求不一致，保留会话；先退出旧会话再切换: '+
                               str(existing.get('detector_mode'))+' requested='+self.detector)
        if not existing['native_ok']:
            raise RuntimeError('原生雷达不健康，不启动定位/运动: '+str(existing))
        with self.stage('camera_clock_and_chain'):
            # Healthy camera and localization are reused without touching either clock.
            if inventory['camera'] and not existing['camera_ok']:
                self.stop_invalid_localization()
                self.stop_invalid_camera()
                inventory['camera']=[]
                existing['slam_ok']=False
            if not inventory['camera']:
                try:
                    verify_clock([*SSH,HOST,shlex.join(['/usr/bin/python3','-u','-c',CLOCK_RESPONDER])])
                except ClockOffsetError:
                    run(['/usr/bin/python3',self.work/'camera_ws/scripts/sync_go2_clock.py'])
                    verify_clock([*SSH,HOST,shlex.join(['/usr/bin/python3','-u','-c',CLOCK_RESPONDER])])
                remote('mkdir -p '+REMOTE)
                run(['scp','-i',KEY,'-o','BatchMode=yes',
                     self.work/'camera_ws/scripts/launch_go2_d435_foxy.sh',HOST+':'+REMOTE+'/'])
                self.spawn([*SSH,HOST,shlex.join(['/usr/bin/python3','-c',CAMERA_OWNER,
                           REMOTE+'/launch_go2_d435_foxy.sh'])],'camera',stdin=subprocess.PIPE)
            if not existing['slam_ok']:
                self.stop_invalid_localization()
                command=self.perception_command()
                self.spawn(self.shell(shlex.join(command)),'slam')
            else:
                print('REUSE Point-LIO/base/IMU mapping/TF',flush=True)
        with self.stage('final_live_readiness'):
            fresh_existing=(existing.get('ready') and existing.get('base_stamp_ns') and
                existing.get('camera_stamp_ns') and
                -.15 <= time.time()-existing['base_stamp_ns']/1e9 < .4 and
                -.05 <= time.time()-existing['camera_stamp_ns']/1e9 < .4)
            if fresh_existing and not self.children:
                print('REUSE live readiness: no duplicate LiDAR/TF check',flush=True)
                result=existing
            else:
                result=self.probe('pre_motion',timeout=30,scope='chain',require_target=require_target)
            if not result['ready'] and not result['slam_ok']:
                print('INVALID_TIME_OR_LOCALIZATION: 重建本次定位会话一次，不复用offset',flush=True)
                self.stop_invalid_localization()
                command=self.perception_command()
                self.spawn(self.shell(shlex.join(command)),'slam')
                result=self.probe('pre_motion_reinitialized',timeout=30,scope='chain',require_target=require_target)
            if not result['ready']:
                raise RuntimeError('实时定位/目标检查失败；禁止运动: '+str(result))
        # Existing full-task adapters are uploaded only for the full mode, not dock.
        if self.args.mode=='full':
            with self.stage('full_task_adapters'):
                remote('mkdir -p '+REMOTE)
                run(['scp','-i',KEY,'-o','BatchMode=yes',
                     self.work/'camera_ws/scripts/s1_task_host.py',
                     self.work/'camera_ws/scripts/launch_go2_d435_foxy.sh',
                     self.work/'camera_ws/src/go2_science_perception/go2_science_perception/s1_task_sequence.py',
                     HOST+':'+REMOTE+'/'])
        if not zero_hold(self.status()):
            raise RuntimeError('运动前S1状态变化，停止')
        self.start_viewers()
        if self.args.mode == 'full':
            self.multi_memory_session = self.detector == 'yolo'
            self.processed_results = []
            run_three_grasps(self.run_full_round, lambda completed: atomic_json(
                self.folder/'three_grasps.json', progress_snapshot(completed)))
            self.current_round = 0
            self.run_task(return_only=True)
            print('THREE_TARGETS_PROCESSED: third StandUp and map return confirmed', flush=True)
        else:
            self.run_task()

    def run_full_round(self, number, completed_targets):
        self.current_round = number
        self.completed_targets = completed_targets
        if not zero_hold(self.status()):
            raise RuntimeError('S1 not ready before next exploration round')
        if self.detector == 'yolo' and completed_targets and not getattr(self, 'multi_memory_session', False):
            # Only run_three_grasps supplies prior grasp+StandUp successes here.
            # An acknowledgement is required before creating the next controller.
            with self.stage('advance_yolo_round_'+str(number)):
                run(self.shell(shlex.join(['/usr/bin/python3',
                    str(self.work/'camera_ws/scripts/advance_yolo_round.py'),
                    json.dumps(completed_targets)])), timeout=12)
        print('GRASP_ROUND_START '+str(number)+'/3', flush=True)
        result_path = self.folder/('round_%02d_result.json' % number)
        try:
            self.run_task()
        except BaseException:
            # A confirmed StandUp remains a processed target even if restoration fails.
            if result_path.exists():
                result = json.loads(result_path.read_text())
                if result.get('task_motion_complete') is True:
                    atomic_json(self.folder/'three_grasps.json', progress_snapshot(
                        getattr(self, 'processed_results', [])+[result]))
            raise
        result = json.loads(result_path.read_text())
        self.processed_results = getattr(self, 'processed_results', [])+[result]
        return result

    def run_task(self, return_only=False):
        command = self.task_command()
        if return_only:
            command.append('--return-only')
        print('开始真实运动；Ctrl+C请求停止。物理遥控急停保持可用。', flush=True)
        task_log = (self.folder/('round_%02d_coordinator.log' % self.current_round)).open('x')
        task = subprocess.Popen(self.shell(shlex.join(command)), stdout=task_log, stderr=subprocess.STDOUT)
        try:
            self.wait_job(task)
        finally:
            if task.poll() is None:
                task.send_signal(signal.SIGINT)
                deadline = time.monotonic()+20
                warned = False

                def reject_new_pause():
                    raise RuntimeError('Full task cancelled; no new arm handoff')

                while task.poll() is None:
                    # The cancelled coordinator still needs its finally/resume ack.
                    if self.args.mode == 'full':
                        service_requests(self.camera_handoff_directory,
                                         reject_new_pause, self.resume_front_camera)
                    if not warned and time.monotonic() > deadline:
                        print('任务未退出：立即物理急停；保留定位和S1，等待原任务退出。', flush=True)
                        warned = True
                    time.sleep(.1)
            task_log.close()

    def close(self):
        if self.camera_paused:
            try:
                self.resume_front_camera()
            except Exception as exc:
                print('D435_RESTORE_BLOCKED: '+str(exc), flush=True)
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
        if self.stages:
            slowest=max(self.stages,key=lambda row:row['elapsed_sec'])
            print('SLOWEST_STAGE '+json.dumps(slowest),flush=True)
        print('本轮自建相机/定位清理结束；复用会话保持运行；S1服务不会自动关闭，停机请按手册人工处理。', flush=True)
        if self.folder.exists():
            print('日志: '+str(self.folder)+'；物理停止/抓取结果需现场确认。', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='确认路线空旷、急停在手、S1初始位和支撑已核对；执行真实运动')
    parser.add_argument('--mode', choices=['full', 'dock'], default='full')
    parser.add_argument('--grasp-side', choices=['right','left','auto'], default=None)
    parser.add_argument('--detector', choices=['yolo','hsv'], default='yolo')
    parser.add_argument('--view', action='store_true')
    parser.add_argument('--model-path', type=Path)
    parser.add_argument('--python-executable', type=Path)
    parser.add_argument('--forward-response-check', action='store_true')
    parser.add_argument('--response-distance-limit', type=float, default=.20)
    parser.add_argument('--work-dir', type=Path, default=Path('/tmp/go2_onekey_s1'))
    parser.add_argument('--probe', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--left-geometry-verified', action='store_true',
                        help='operator has physically checked left docking offsets')
    parser.add_argument('--startup-check', action='store_true',
                        help='read-only stage timing/status; never zero/enable/move')
    args = parser.parse_args()
    args.grasp_side = args.grasp_side or ('auto' if args.mode == 'full' else 'right')
    if args.forward_response_check and args.mode != 'dock':
        parser.error('--forward-response-check requires --mode dock, never full')
    if not .10 <= args.response_distance_limit <= .50:
        parser.error('response distance limit must be 0.10 to 0.50m')
    if args.response_distance_limit != .20 and not args.forward_response_check:
        parser.error('response distance override is for --forward-response-check only')
    if args.probe:
        ros_probe()
        return
    if not args.execute and not args.startup_check:
        if args.forward_response_check:
            print('离线计划：复用现有S1保持和传感器预检；同一控制器/安全门正向缓升至配置参考速度（当前0.20m/s），'
                  '确认最终出口达到参考速度后，原始位姿再前移2cm即请求零速，记录出口归零与停稳。'
                  f'本次路径保护{args.response_distance_limit:.2f}m/8秒，不是预设脉冲时间。无SEARCH/趴下/抓取。')
            return
        print('离线计划（未连接硬件、未写文件）：\n'
              '最新代码/按内容缓存构建 → 连接/设备检查 → 复用S1或原路径受控回零 → '
              '复用健康D435/Point-LIO/TF，失效会话重建 → 实时数据就绪即继续 → '
              '检测器='+args.detector +
              (' → NBV探索 → FAR导航到MAP停靠区 → STOP_CONFIRM → 原MAP接近/停靠 → STOP'
               if args.mode == 'full' else ' → 搜索（最多5m）→ STOP_CONFIRM → 三自由度停靠 → STOP') +
              (' → 停止并释放D435 → 趴下 → D455采图/S1抓取 → ZERO_HOLD → 起立'
               ' → 确认D455释放 → 恢复D435 → 继续探索；第3轮起立后沿MAP历史路径返航' if args.mode == 'full' else '') +
              '\n真实执行：bash go2_test.sh --execute --mode '+args.mode+'\n' +
              ('远距和返航启用 localPlanner 避障；' if args.mode == 'full' else '无自动避障；') +
              '退出保留S1保持服务。完整流程真机验收：NOT VERIFIED。')
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
