#!/usr/bin/env python3
"""Collect real GO2 sensor timestamps; reuse live SLAM, start read-only SLAM only if absent."""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import select
import shlex
import signal
import socket
import struct
import subprocess
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException

REPO = Path(__file__).resolve().parents[2]
SSH = ['ssh', '-i', '/home/czx666/.ssh/id_ed25519_go2', '-o', 'BatchMode=yes',
       '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=3',
       '-o', 'ServerAliveCountMax=3', 'unitree@192.168.123.18']


class TimestampExecutor(SingleThreadedExecutor):
    """This installed Humble executor discards MessageInfo; retain it for probes only."""
    def _take_subscription(self, sub):
        with sub.handle:
            result = sub.handle.take_message(sub.msg_type, sub.raw)
        if result is None:
            return None
        return result if getattr(sub, 'time_probe', False) else (result[0],)

    async def _execute_subscription(self, sub, values):
        if values:
            await await_or_execute(sub.callback, *values)


def command(args, timeout=10):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return dict(code=p.returncode, stdout=p.stdout, stderr=p.stderr)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(error=str(exc))


def ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


def ntp_probe(host):
    """Read-only NTP query; never set either clock or treat timeout as synchronization."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
        stream.settimeout(2)
        request = bytearray(48)
        request[0] = 0x23
        start = time.time()
        value = start+2208988800
        struct.pack_into('!II', request, 40, int(value), int((value % 1)*2**32))
        try:
            stream.sendto(request, (host, 123))
            reply, _ = stream.recvfrom(512)
            end = time.time()
            if len(reply) < 48 or not 1 <= reply[1] <= 15:
                return dict(host=host, error='invalid/unsynchronized NTP response')
            recv_sec, recv_frac, tx_sec, tx_frac = struct.unpack_from('!IIII', reply, 32)
            received = recv_sec-2208988800+recv_frac/2**32
            sent = tx_sec-2208988800+tx_frac/2**32
            return dict(host=host, offset_s=((received-start)+(sent-end))/2,
                        rtt_s=(end-start)-(sent-received), stratum=reply[1])
        except (OSError, ValueError) as exc:
            return dict(host=host, error=str(exc))


def cloud_times(msg):
    """Native 'time' is seconds; Point-LIO stores time*1000 in curvature (ms)."""
    field = next((f for f in msg.fields if f.name in ('time', 't', 'curvature')), None)
    if field is None:
        return dict(error='no per-point time field')
    formats = {7: 'f4', 8: 'f8', 6: 'u4', 5: 'i4'}
    if field.datatype not in formats:
        return dict(error='unsupported point time type')
    dtype = np.dtype(('>' if msg.is_bigendian else '<')+formats[field.datatype])
    values = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        offset=field.offset, strides=(msg.row_step, msg.point_step)).ravel()
    values = values[np.isfinite(values)]
    if not len(values):
        return dict(error='empty/nonfinite point times')
    scale = .001 if field.name == 'curvature' else 1.0
    return dict(point_time_field=field.name, point_time_min_s=float(values.min())*scale,
                point_time_max_s=float(values.max())*scale,
                curvature_max_ms=float(values.max())*scale*1000,
                width=msg.width, height=msg.height, point_step=msg.point_step,
                fields=[dict(name=f.name, offset=f.offset, datatype=f.datatype)
                        for f in msg.fields])


def stats(values):
    a = np.array([v for _, v in values], dtype=float)
    if not len(a):
        return dict(count=0)
    t = np.array([t for t, _ in values], dtype=float)
    return dict(count=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()), minimum=float(a.min()),
                slope_per_second=float(np.polyfit(t-t[0], a, 1)[0]) if np.ptp(t) else 0,
                first=float(a[0]), last=float(a[-1]))


def analyze(folder):
    all_rows = [json.loads(x) for x in (folder/'samples.jsonl').read_text().splitlines()]
    metadata = json.loads((folder/'metadata.json').read_text())
    start_mono = metadata.get('record_start_mono_ns')
    if start_mono is None:
        # Compatibility with the first real capture: derive the matching sample
        # boundary from its recorded wall+monotonic pair, not an invented stamp.
        start_mono = next((r['mono_ns'] for r in all_rows if r.get('wall_ns', 0) >=
                           metadata.get('record_start_wall_ns', 0)), 0)
    rows = [r for r in all_rows if r['mono_ns'] >= start_mono]
    groups = {k: [r for r in rows if r['kind'] == k] for k in
              ['raw_cloud', 'raw_imu', 'cloud', 'imu', 'state', 'base', 'tf', 'go2_clock']}
    metrics = {}
    for kind, key, sign in [('raw_cloud', 'receive_delay_s', -1),
                            ('raw_imu', 'imu_receive_delay_s', -1),
                            ('cloud', 'cloud_future_s', 1), ('imu', 'imu_future_s', 1),
                            ('state', 'state_future_s', 1), ('base', 'base_future_s', 1)]:
        metrics[key] = stats([(r['mono_ns']/1e9, sign*(r['stamp_ns']-r['ros_ns'])/1e9)
                              for r in groups[kind]])
    metrics['derived_lidar_end_future_s'] = stats([
        (r['mono_ns']/1e9, (r['stamp_ns']-r['ros_ns'])/1e9+r['point_time_max_s'])
        for r in groups['cloud'] if 'point_time_max_s' in r])
    metrics['scan_span_s'] = stats([
        (r['mono_ns']/1e9, r['point_time_max_s'])
        for r in groups['raw_cloud'] if 'point_time_max_s' in r])
    metrics['go2_minus_pc_s'] = stats([
        (r['mono_ns']/1e9, r['offset_s']) for r in groups['go2_clock'] if 'offset_s' in r])
    for kind in ('raw_cloud', 'raw_imu'):
        metrics[kind+'_dds_source_minus_header_s'] = stats([
            (r['mono_ns']/1e9, (r['dds_source_ns']-r['stamp_ns'])/1e9)
            for r in groups[kind] if r.get('dds_source_ns')])
    internal = []
    log = folder/'readonly_slam.log'
    if log.exists():
        pattern = (r'TIME_DIAGNOSTIC now=([\d.]+) beg=([\d.]+) end=([\d.]+) '
                   r'current=([\d.]+) imu=([\d.]+) odom=([\d.]+)')
        for line in log.read_text(errors='replace').splitlines():
            match = re.search(pattern, line)
            if match:
                value = dict(zip(('now', 'beg', 'end', 'current', 'imu', 'odom'),
                                 map(float, match.groups())))
                if value['now']*1e9 >= metadata.get('record_start_wall_ns', 0):
                    internal.append(value)
    metrics['internal_lidar_end_future_s'] = stats([(r['now'], r['end']-r['now']) for r in internal])
    (folder/'point_lio_internal.json').write_text(json.dumps(internal, indent=2)+'\n')
    # Exact nanosecond mode finds the fixed epoch offset despite callback jitter.
    offsets = Counter()
    for r in groups['cloud']:
        for raw in groups['raw_cloud']:
            if abs(r['mono_ns']-raw['mono_ns']) < 200_000_000:
                offsets[r['stamp_ns']-raw['stamp_ns']] += 1
    offset = offsets.most_common(3)
    raw_imu_stamps = {r['stamp_ns'] for r in groups['raw_imu']}
    state_stamps = {r['stamp_ns'] for r in groups['state']}
    end_stamps = [r['stamp_ns']+round(r.get('point_time_max_s', 0)*1e9)
                  for r in groups['cloud']]
    matches = sum(any(abs(r['stamp_ns']-s) < 2000 for s in end_stamps)
                  for r in groups['state'])
    result = dict(metrics=metrics, fixed_cloud_offsets_ns=offset,
                  analyzed_samples_start_mono_ns=start_mono,
                  imu_matching_same_offset=sum(r['stamp_ns']-offset[0][0] in raw_imu_stamps
                                               for r in groups['imu']) if offset else 0,
                  imu_matching_zero_offset=sum(r['stamp_ns'] in raw_imu_stamps
                                               for r in groups['imu']),
                  base_exact_state_stamp_matches=sum(r['stamp_ns'] in state_stamps
                                                     for r in groups['base']),
                  state_matches_derived_scan_end_within_2us=matches,
                  internal_time_evidence=('observed diagnostic log' if internal else
                                          'NOT VERIFIED: no internal log; derived values separate'),
                  physical_motion_commanded=False)
    all_raw_imu = {r['stamp_ns'] for r in all_rows if r['kind'] == 'raw_imu'}
    result['startup_imu_published_without_mapping'] = sum(
        r['stamp_ns'] in all_raw_imu for r in all_rows if r['kind'] == 'imu')
    result['tf_exact_query_success'] = sum(
        r.get('tf_stamp_ns') == r.get('query_ns') for r in groups['tf'])
    result['tf_queries'] = len(groups['tf'])
    result['receive_delay_warning'] = 'Raw stamp uses a different epoch; receive age is NOT network latency.'
    source_blocked = log.exists() and 'SOURCE_CLOCK_UNSYNCHRONIZED' in log.read_text()
    if source_blocked:
        result['status'] = 'BLOCKED_NATIVE_CLOCK: corrected converter refuses uncalibrated source epoch'
        result['earliest_unresolved_stage'] = (
            'native sensor/producer clock versus PC; no trusted mapping available')
    elif groups['state']:
        result['status'] = 'OBSERVED: actual timestamps collected; no motion validation'
        result['earliest_bias_stage'] = (
            'Compare transformed IMU and full-scan header/relative time before Point-LIO')
    else:
        result['status'] = 'NOT VERIFIED: missing odometry; inspect saved node logs'
    if offset:
        delay = metrics['receive_delay_s'].get('median')
        imu_delay = metrics['imu_receive_delay_s'].get('median')
        if delay is not None and imu_delay is not None:
            result['cloud_minus_imu_receive_age_s'] = delay-imu_delay
    baselines = [p for p in folder.parent.glob('time_*before_internal/report.json') if p.parent != folder]
    if baselines:
        baseline = max(baselines, key=lambda p: p.stat().st_mtime)
        before = json.loads(baseline.read_text())
        result['comparison'] = dict(baseline=str(baseline),
                                    before_base_future=before['metrics']['base_future_s'],
                                    after_base_future=metrics['base_future_s'],
                                    after_verified=bool(groups['base']))
    (folder/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    lines = [result['status'], result['receive_delay_warning'],
             'All metric units: seconds; slope: seconds/second.']
    for name, value in metrics.items():
        lines.append(name+': '+json.dumps(value))
    if result.get('comparison'):
        lines.append('Before/after: '+json.dumps(result['comparison']))
    (folder/'report.txt').write_text('\n'.join(lines)+'\n')
    # One-second median series is retained alongside the complete raw stream.
    series = []
    for second in range(int((rows[-1]['mono_ns']-rows[0]['mono_ns'])/1e9)+1 if rows else 0):
        selected = [r for r in rows if second <= (r['mono_ns']-rows[0]['mono_ns'])/1e9 < second+1]
        row = {'second': second}
        for kind in ('raw_cloud', 'raw_imu', 'cloud', 'imu', 'state', 'base'):
            values = [(r['stamp_ns']-r['ros_ns'])/1e9 for r in selected if r['kind'] == kind]
            row[kind+'_stamp_minus_pc_median_s'] = float(np.median(values)) if values else None
        series.append(row)
    (folder/'trend.json').write_text(json.dumps(series, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--label', default='diagnostic')
    args = parser.parse_args()
    if not 0 < args.duration <= 120:
        parser.error('duration must be in (0,120] seconds')
    folder = REPO/'camera_ws/records'/('time_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.label)
    folder.mkdir(parents=True)
    go2_status_command = 'date -Is; timedatectl status; systemctl is-active chrony systemd-timesyncd'
    metadata = dict(commit=command(['git', '-C', str(REPO), 'rev-parse', 'HEAD']),
                    pc_time_status=command(['timedatectl', 'status']),
                    go2_status=command([*SSH, go2_status_command]),
                    route=command(['ip', 'route', 'get', '192.168.123.18']),
                    pc_processes=command(['ps', '-eo', 'pid,etime,args']))
    metadata['native_ntp_probe'] = ntp_probe('192.168.123.161')
    metadata['pc_timesync_status'] = command(['timedatectl', 'timesync-status'])
    transform_spec = importlib.util.find_spec('transform_sensors.transform_everything')
    if transform_spec and transform_spec.origin:
        source_path = Path(transform_spec.origin)
        metadata['loaded_transform'] = {
            'path': str(source_path),
            'sha256': hashlib.sha256(source_path.read_bytes()).hexdigest()}
    if metadata['pc_processes'].get('stdout'):
        metadata['pc_processes']['stdout'] = '\n'.join(
            line for line in metadata['pc_processes']['stdout'].splitlines()
            if any(name in line for name in ('pointlio_mapping', 'transform_everything',
                                             'base_odom_node', 'science_target_approach',
                                             'go2_safety_gate')))
    if metadata['go2_status'].get('code') == 255:
        (folder/'metadata.json').write_text(json.dumps(metadata, indent=2))
        raise RuntimeError('GO2 SSH unavailable; no fabricated samples. '+str(folder))
    rclpy.init()
    node = rclpy.create_node('go2_readonly_time_diagnostic')
    executor = TimestampExecutor()
    executor.add_node(node)
    buffer = Buffer()
    node._time_diagnostic_tf_listener = TransformListener(buffer, node)
    children = []
    streams = []
    output = (folder/'samples.jsonl').open('x', buffering=1)
    output_lock = threading.Lock()
    stop = threading.Event()

    def write(row):
        with output_lock:
            output.write(json.dumps(row)+'\n')

    def receive(msg, kind, info):
        row = dict(kind=kind, ros_ns=node.get_clock().now().nanoseconds,
                   wall_ns=time.time_ns(), mono_ns=time.monotonic_ns(), stamp_ns=ns(msg.header.stamp),
                   mono_raw_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                   dds_source_ns=info['source_timestamp'], dds_received_ns=info['received_timestamp'],
                   frame=msg.header.frame_id)
        if isinstance(msg, PointCloud2):
            row.update(cloud_times(msg))
            if kind == 'cloud':
                row['lidar_beg_time_ns_derived'] = row['stamp_ns']
                if 'point_time_max_s' in row:
                    row['lidar_end_time_ns_derived'] = row['stamp_ns']+round(row['point_time_max_s']*1e9)
        write(row)
        if kind == 'base':
            query = dict(kind='tf', ros_ns=node.get_clock().now().nanoseconds,
                         mono_ns=time.monotonic_ns(), query_ns=row['stamp_ns'])
            try:
                tf = buffer.lookup_transform('map', 'vehicle', rclpy.time.Time.from_msg(msg.header.stamp))
                query['tf_stamp_ns'] = ns(tf.header.stamp)
            except TransformException as exc:
                query['error'] = str(exc)
            write(query)

    def callback(kind):
        def cb(msg, info):
            receive(msg, kind, info)
        return cb

    for kind, topic, typ in [('raw_cloud', '/utlidar/cloud', PointCloud2),
                             ('raw_imu', '/utlidar/imu', Imu),
                             ('cloud', '/utlidar/transformed_cloud', PointCloud2),
                             ('imu', '/utlidar/transformed_imu', Imu),
                             ('state', '/state_estimation', Odometry),
                             ('base', '/base_state_estimation', Odometry)]:
        sub = node.create_subscription(typ, topic, callback(kind), qos_profile_sensor_data)
        sub.time_probe = True

    def spawn(argv, name):
        stream = (folder/(name+'.log')).open('x')
        p = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(p)
        streams.append(stream)
        print('只读启动', name, p.pid, flush=True)

    def clock_worker():
        code = ("import time,sys; print('READY',flush=True); "
                "exec('for line in sys.stdin:\\n print(time.time_ns(),flush=True)')")
        p = subprocess.Popen([*SSH, shlex.join(['/usr/bin/python3', '-u', '-c', code])],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        try:
            if not select.select([p.stdout], [], [], 5)[0]:
                return
            if p.stdout.readline().strip() != 'READY':
                return
            while not stop.is_set():
                start = time.time_ns()
                p.stdin.write('ping\n')
                p.stdin.flush()
                if not select.select([p.stdout], [], [], 3)[0]:
                    break
                value = p.stdout.readline()
                end = time.time_ns()
                if not value:
                    break
                remote_ns = int(value)
                write(dict(kind='go2_clock', mono_ns=time.monotonic_ns(),
                           pc_send_ns=start, pc_receive_ns=end, go2_ns=remote_ns,
                           rtt_s=(end-start)/1e9, offset_s=(remote_ns-(start+end)/2)/1e9))
                stop.wait(.5)
        finally:
            if p.poll() is None:
                p.terminate()
            p.wait(timeout=5)
            p.stdin.close()
            p.stdout.close()

    clock_thread = threading.Thread(target=clock_worker, daemon=True)
    try:
        until = time.monotonic()+2
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.1)
        nodes = node.get_node_names()
        metadata['initial_nodes'] = nodes
        if node.count_publishers('/cmd_vel_raw'):
            raise RuntimeError('已有运动控制器；只读诊断拒绝与运动测试并行')
        if 'laserMapping' not in nodes:
            if 'transform_everything' in nodes or 'sensor_transformer' in nodes:
                raise RuntimeError('已有转换节点但无SLAM；保留现场，不启动重复链')
            spawn(['ros2', 'launch', 'point_lio_unilidar', 'mapping_utlidar.launch',
                   'rviz:=false', 'use_sim_time:=false', 'time_diagnostic_log:=true'], 'readonly_slam')
            transforms = [('map_to_camera_init', 'map', 'camera_init', '0'),
                          ('aft_mapped_to_sensor', 'aft_mapped', 'sensor', '0'),
                          ('sensor_to_vehicle', 'sensor', 'vehicle', '-0.3')]
            for name, parent, child, x in transforms:
                if name not in nodes:
                    spawn(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', x,
                           '--frame-id', parent, '--child-frame-id', child], name)
            if 'base_odom_node' not in nodes:
                spawn(['ros2', 'run', 'go2_keyboard_teleop', 'base_odom_node'], 'base_odom')
            until = time.monotonic()+5
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.1)
        metadata['clock_publishers'] = node.count_publishers('/clock')
        metadata['node_parameters'] = {}
        parameter_queries = [
            ('laserMapping', ['use_sim_time', 'common.time_lag_imu_to_lidar',
                              'odometry.publish_odometry_without_downsample',
                              'common.cut_frame', 'common.con_frame', 'preprocess.timestamp_unit']),
            ('transform_everything', ['use_sim_time', 'sensor_time_offset_sec'])]
        for target, names in parameter_queries:
            client = node.create_client(GetParameters, '/'+target+'/get_parameters')
            if client.wait_for_service(timeout_sec=3):
                future = client.call_async(GetParameters.Request(names=names))
                executor.spin_until_future_complete(future, timeout_sec=3)
                if future.result():
                    metadata['node_parameters'][target] = {
                        name: str(value) for name, value in zip(names, future.result().values)}
        metadata['raw_cloud_publishers'] = [
            str(x) for x in node.get_publishers_info_by_topic('/utlidar/cloud')]
        metadata['record_start_wall_ns'] = time.time_ns()
        metadata['record_start_mono_ns'] = time.monotonic_ns()
        clock_thread.start()
        print('采集30秒，目录:', folder, flush=True)
        until = time.monotonic()+args.duration
        while time.monotonic() < until:
            if any(p.poll() is not None for p in children):
                raise RuntimeError('本轮只读节点退出，保留日志')
            executor.spin_once(timeout_sec=.02)
    finally:
        stop.set()
        if clock_thread.is_alive():
            clock_thread.join(timeout=7)
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=5)
        for stream in streams:
            stream.close()
        output.close()
        metadata['record_end_wall_ns'] = time.time_ns()
        (folder/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    result = analyze(folder)
    if result['status'].startswith(('BLOCKED', 'NOT VERIFIED')):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
