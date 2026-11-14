#!/usr/bin/env python3
"""Integrated color-block approach: offline plan by default, preview never arms GO2."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
V2_SHA256 = 'e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1'


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['approach'], default='approach')
    p.add_argument('--detector', choices=['yolo', 'hsv'], default='yolo')
    action = p.add_mutually_exclusive_group()
    action.add_argument('--preview', action='store_true', help='RGB-D/MAP only, no gate or arm')
    action.add_argument('--execute', action='store_true', help='one authorized approach, no grasp')
    p.add_argument('--view', action='store_true')
    p.add_argument('--duration', type=float, default=60.)
    p.add_argument('--timeout', type=float, default=45.)
    p.add_argument('--local-camera', action='store_true')
    p.add_argument('--model-path', type=Path)
    p.add_argument('--python-executable', type=Path)
    args = p.parse_args(argv)
    if not 0 < args.duration <= 3600 or not 0 < args.timeout <= 60:
        p.error('duration must be (0,3600], timeout (0,60]')
    return args


def detector_assets(args, root=ROOT):
    """Resolve the same local V2 assets for approach, dock and full."""
    data_root = Path(os.environ.get('GO2_VISION_ROOT',
        str((root/'camera_ws/records').resolve().parents[1])))
    model = getattr(args, 'model_path', None) or data_root/'vision_training/runs/science_targets_mixed_v2/baseline/weights/best.pt'
    python = getattr(args, 'python_executable', None) or data_root/'vision_training/.venv/bin/python'
    if args.detector == 'yolo':
        if not python.is_file() or not model.is_file():
            raise RuntimeError('Missing local V2 weights/venv; use GO2_VISION_ROOT or explicit paths')
        if hashlib.sha256(model.read_bytes()).hexdigest() != V2_SHA256:
            raise RuntimeError('Model differs from verified science_targets_mixed_v2 best.pt')
    return model, python


def main(argv=None):
    args = arguments(argv)
    if not args.preview and not args.execute:
        print('离线计划：方块接近 detector='+args.detector+
              ('；仅class0/有效ROI深度→精确时间MAP→最近单目标锁定→原Manager/Approach；'
               if args.detector=='yolo' else '；原蓝色HSV粗定位/MAP/Manager/Approach；')+
              'camera_link原点到目标MAP XY≤0.10m；无抓取/S1启动。\n'
              '静态预览：bash go2_test.sh --mode approach --preview --view\n'
              '真实运动：bash go2_test.sh --mode approach --execute --view\n'
              'HSV回退：同命令追加 --detector hsv（先退出原会话）；不自动切换检测器。')
        return
    model, python = detector_assets(args)
    # The original full/dock runner uses this same lock; never run both owners.
    with open('/tmp/go2_full_test.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run(args, model, python)


def prepare_remote_camera_clock(ssh, host):
    """Repair a rebooted companion clock only before a camera owner exists."""
    from go2_full_test import CLOCK_RESPONDER, ClockOffsetError, verify_clock
    present = subprocess.run([*ssh, host, "pgrep -f '[r]ealsense2_camera_node'"],
                             capture_output=True, timeout=10)
    if present.returncode not in (0, 1):
        raise RuntimeError('Remote camera inventory failed; no clock change or camera restart')
    camera_running = present.returncode == 0
    command = [*ssh, host, shlex.join(['/usr/bin/python3', '-u', '-c', CLOCK_RESPONDER])]
    try:
        verify_clock(command)
    except ClockOffsetError:
        if camera_running:
            raise RuntimeError('远端相机正在运行且时钟失配；保留会话，先按原流程停止相机/定位再对时')
        print('伴随主机时钟失配且没有相机所有者；调用原对时脚本。'
              '请在本终端输入GO2 sudo密码，对时后重新检查50ms阈值。', flush=True)
        subprocess.run(['/usr/bin/python3', str(ROOT/'camera_ws/scripts/sync_go2_clock.py')],
                       check=True)
        verify_clock(command)  # Never start perception or motion on sync exit alone.
    return camera_running


def run(args, model, python):
    # Import only transport helpers. Never construct the S1/full Workflow.
    from go2_full_test import SSH, HOST, KEY, CAMERA_OWNER, require_clean_checkout
    require_clean_checkout(ROOT)
    folder = ROOT/'camera_ws/records'/('color_block_'+str(time.time_ns()))
    folder.mkdir(parents=True)
    (folder/'manifest.json').write_text(json.dumps(dict(
        mode='approach', detector=args.detector, preview=args.preview, execute=args.execute,
        commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        model=str(model), model_sha256=V2_SHA256 if args.detector=='yolo' else None,
        stop_reference='camera_origin', stop_xy_tolerance=.10, physical_result='NOT VERIFIED'), indent=2))
    overlay = ROOT/'.color_block/install/setup.bash'
    sources = [Path('/opt/ros/humble/setup.bash'), ROOT/'deploy_ws/install/setup.bash']
    if not all(p.is_file() for p in sources):
        raise RuntimeError('Missing ROS/Point-LIO underlay; preserve existing deployment')
    prefix = ''.join('source '+shlex.quote(str(p))+' || exit; ' for p in sources)
    def shell(command):
        return ['bash', '-c', prefix+shlex.join(list(map(str, command)))]
    build = ['/usr/bin/python3', '/usr/bin/colcon', '--log-base', ROOT/'.color_block/log',
             'build', '--base-paths', ROOT/'camera_ws/src', '--packages-select', 'go2_science_perception',
             '--build-base', ROOT/'.color_block/build', '--install-base', overlay.parent]
    subprocess.run(shell(build), check=True, cwd=ROOT)
    # Reuse the actual integration time-mapping overlay without changing clocks or mappings.
    time_overlay = ROOT/'.time_sync/install/setup.bash'
    if time_overlay.is_file():
        prefix += 'source '+shlex.quote(str(time_overlay))+' || exit; '
    prefix += 'source '+shlex.quote(str(overlay))+' || exit; '
    prefix += 'source '+shlex.quote(str(ROOT/'camera_ws/scripts/science_approach_pc_env.bash'))+' || exit; '
    prefix += 'export SCIENCE_APPROACH_OVERLAY='+shlex.quote(str(overlay))+'; '
    os.environ['OMP_NUM_THREADS'] = '1'
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    os.environ['MKL_NUM_THREADS'] = '1'
    os.environ['ROS_LOG_DIR'] = str(folder/'ros_logs')
    os.environ['YOLO_CONFIG_DIR'] = str(folder/'ultralytics')
    os.environ['MPLCONFIGDIR'] = str(folder/'matplotlib')
    children = []
    task = None
    def spawn(command, label, direct=False, **kw):
        stream = (folder/(label+'.log')).open('x')
        try:
            process = subprocess.Popen(command if direct else shell(command), stdout=stream,
                stderr=subprocess.STDOUT, start_new_session=True, **kw)
        except BaseException:
            stream.close()
            raise
        children.append((process, stream, label))
        return process
    def healthy():
        for process, _, label in children:
            if process.poll() is not None:
                raise RuntimeError(label+' exited; inspect '+str(folder))
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        # Read-only graph check, before any detector or controller is created.
        probe = """import rclpy,time
rclpy.init(); n=rclpy.create_node('color_block_startup')
end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
for t in ['/cmd_vel_raw','/science/target_coarse_point','/science/target_coarse_point_map','/science/yolo/detections']:
 if n.count_publishers(t):raise RuntimeError('Existing owner; stop its session first: '+t)
if any(name in ['laserMapping','base_odom_node','science_target_manager','science_target_approach','go2_safety_gate'] for name,ns in n.get_node_names_and_namespaces()):
 raise RuntimeError('Existing localization/control chain; preserve it and stop duplicate startup')
n.destroy_node();rclpy.shutdown()
"""
        subprocess.run(shell(['/usr/bin/python3','-c',probe]), check=True)
        if not args.local_camera:
            camera_running = prepare_remote_camera_clock(SSH, HOST)
            if not camera_running:
                remote_dir = '/home/unitree/go2_color_block_'+folder.name
                subprocess.run([*SSH,HOST,'mkdir -p '+shlex.quote(remote_dir)],check=True)
                script = ROOT/'camera_ws/scripts/launch_go2_d435_foxy.sh'
                subprocess.run(['scp','-i',KEY,str(script),HOST+':'+remote_dir+'/'],check=True)
                spawn([*SSH,HOST,shlex.join(['/usr/bin/python3','-c',CAMERA_OWNER,
                      remote_dir+'/'+script.name])], 'camera', direct=True, stdin=subprocess.PIPE)
        spawn(['ros2','launch','go2_science_perception','d435_slam_target_validation.launch.py',
               'remote_camera:='+str(not args.local_camera).lower(),'use_rviz:=false',
               'detector:='+args.detector,'yolo_model:='+str(model),'yolo_python:='+str(python)], 'perception_map')
        for topic,label in [('/science/map_measurement_debug','map_positions'),
                            ('/science/approach/status','approach_status')]:
            spawn(['ros2','topic','echo',topic,'std_msgs/msg/String'],label)
        if args.preview:
            spawn(['ros2','run','go2_science_perception','science_target_approach','--ros-args',
                   '--params-file',ROOT/'camera_ws/src/go2_science_perception/config/science_target_approach.yaml',
                   '-p','dry_run:=true','-p','stop_reference:=camera_origin','-p','stop_xy_tolerance:=0.10',
                   '-p','log_directory:='+str(folder)],'dry_run_approach')
            spawn(['ros2','run','go2_science_perception','approach_video_recorder','--ros-args',
                   '-p','publish_debug_image:=true','-p','output_directory:='+str(folder)],'preview_recorder')
        if args.view:
            spawn(['ros2','run','rqt_image_view','rqt_image_view','/science/approach/debug_image'],'viewer')
        print('记录目录：'+str(folder),flush=True)
        if args.preview:
            end=time.monotonic()+args.duration
            while time.monotonic()<end:
                healthy();time.sleep(.2)
        else:
            # Same existing 10cm approach experiment; no --grasp-pose, no SEARCH/S1.
            time.sleep(3);healthy()
            command=['bash',ROOT/'camera_ws/scripts/run_map_anchor_experiment.sh','--execute',
                     '--timeout',str(args.timeout),'--preview-image']
            if not (os.environ.get('DISPLAY') and __import__('shutil').which('gnome-terminal')):
                command.append('--no-log-terminal')
            task=subprocess.Popen(shell(command))
            while task.poll() is None:
                healthy();time.sleep(.2)
            if task.returncode:
                raise RuntimeError('Approach exited with '+str(task.returncode))
    finally:
        # A running controller must finish its original stop/disarm path before sensors exit.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        if task is not None and task.poll() is None:
            task.send_signal(signal.SIGINT)
            task.wait()
        for process,stream,label in reversed(children):
            if process.poll() is None:
                if label=='camera':process.stdin.close()
                else:os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            stream.close()
        print('本轮自建进程已退出；没有启动/关闭S1。实机结果以原控制日志和现场观察为准。',flush=True)


if __name__ == '__main__':
    try:main()
    except KeyboardInterrupt:raise SystemExit(130)
    except (OSError,RuntimeError,subprocess.SubprocessError,ValueError) as exc:
        print('停止：'+str(exc),flush=True);raise SystemExit(1)
