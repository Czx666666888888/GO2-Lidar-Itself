#!/usr/bin/env python3
"""PC-only DDS timestamp/LidarState observation; no SSH, commands or SLAM changes."""
import argparse
from collections import Counter
from datetime import datetime
import json
import os
from pathlib import Path
import re
import subprocess
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

REPO = Path(__file__).resolve().parents[2]


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
              ['raw_cloud', 'raw_imu', 'cloud', 'imu', 'state', 'base', 'tf', 'go2_clock', 'lidar_state']}
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
    for kind in ('raw_cloud', 'raw_imu', 'lidar_state'):
        metrics[kind+'_header_minus_pc_wall_s'] = stats([
            (r['mono_ns']/1e9, (r['stamp_ns']-r['wall_ns'])/1e9)
            for r in groups[kind] if 'wall_ns' in r])
        metrics[kind+'_dds_source_minus_pc_wall_s'] = stats([
            (r['mono_ns']/1e9, (r['dds_source_ns']-r['wall_ns'])/1e9)
            for r in groups[kind] if r.get('dds_source_ns') and 'wall_ns' in r])
    for field, reference in [('serial_recv_stamp_ns', 'wall_ns'),
                             ('serial_recv_stamp_ns', 'stamp_ns'),
                             ('dds_source_ns', 'serial_recv_stamp_ns'),
                             ('dds_source_ns', 'stamp_ns')]:
        metrics['lidar_state_'+field+'_minus_'+reference+'_s'] = stats([
            (r['mono_ns']/1e9, (r[field]-r[reference])/1e9) for r in groups['lidar_state']])
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
    if metadata.get('transport', '').startswith('PC ROS2 DDS'):
        result['lidar_state_discovery'] = metadata['lidar_state_topics']
        result['lidar_state_samples'] = len(groups['lidar_state'])
        result['lidar_state_result'] = 'READABLE' if groups['lidar_state'] else 'NOT VERIFIED'
        result['lidar_versions'] = sorted({
            (r['fields']['firmware_version'], r['fields']['software_version'], r['fields']['sdk_version'])
            for r in groups['lidar_state']})
        result['hardware_model'] = metadata['hardware_model']
        result['model_evidence'] = metadata['model_evidence']
        result['no_hardware_writes'] = True
        result['no_ssh'] = True
        result['slam_processes_untouched'] = True
        result['existing_node_set_preserved'] = (
            set(metadata.get('initial_nodes', [])) == set(metadata.get('final_nodes', [])))
        prefixes = {p['participant_prefix'] for sub in metadata['subscriptions']
                    for p in sub.get('publishers', [])}
        result['same_dds_participant'] = len(prefixes) == 1
        if groups['raw_cloud'] and groups['raw_imu']:
            value = metrics['raw_imu_dds_source_minus_pc_wall_s'].get('median', 0)
            result['status'] = ('BLOCKED_NATIVE_CLOCK: DDS publisher source time differs from PC wall'
                                if abs(value) > 1 else 'OBSERVED: native DDS timestamps near PC wall')
            result['root_cause_assessment'] = (
                'Publisher-side time domain is also offset; A is supported, not merely PC conversion. '
                'DDS source timestamps alone cannot exclude explicit writer timestamps or prove OS date.')
        else:
            result['status'] = 'NOT VERIFIED: missing raw LiDAR/IMU data'
        if not groups['lidar_state']:
            result['lidar_state_result'] = 'NOT VERIFIED: no received LidarState; see actual discovery'
        for key in ('cloud_future_s', 'imu_future_s', 'state_future_s', 'base_future_s',
                    'derived_lidar_end_future_s', 'go2_minus_pc_s', 'internal_lidar_end_future_s'):
            metrics.pop(key, None)
        for key in ('fixed_cloud_offsets_ns', 'imu_matching_same_offset', 'imu_matching_zero_offset',
                    'base_exact_state_stamp_matches', 'state_matches_derived_scan_end_within_2us',
                    'internal_time_evidence', 'startup_imu_published_without_mapping',
                    'tf_exact_query_success', 'tf_queries'):
            result.pop(key, None)
    if offset:
        delay = metrics['receive_delay_s'].get('median')
        imu_delay = metrics['imu_receive_delay_s'].get('median')
        if delay is not None and imu_delay is not None:
            result['cloud_minus_imu_receive_age_s'] = delay-imu_delay
    baselines = [p for p in folder.parent.glob('time_*before_internal/report.json') if p.parent != folder]
    if baselines and not metadata.get('transport', '').startswith('PC ROS2 DDS'):
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
    parser.add_argument('--label', default='pc_dds')
    args = parser.parse_args()
    if not 0 < args.duration <= 120:
        parser.error('duration must be in (0,120] seconds')
    folder = REPO/'camera_ws/records'/('time_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.label)
    folder.mkdir(parents=True)
    metadata = dict(commit=command(['git', '-C', str(REPO), 'rev-parse', 'HEAD']),
                    pc_time_status=command(['timedatectl', 'status']),
                    pc_timesync_status=command(['timedatectl', 'timesync-status']),
                    route=command(['ip', 'route', 'get', '192.168.123.161']),
                    interfaces=command(['ip', '-br', 'addr', 'show', 'enp12s0']),
                    domain=os.environ.get('ROS_DOMAIN_ID'),
                    rmw=os.environ.get('RMW_IMPLEMENTATION'),
                    cyclone_configuration=os.environ.get('CYCLONEDDS_URI'),
                    transport='PC ROS2 DDS; official unitree_go ROS messages; no SSH/RPC/config writes',
                    hardware_model='L1', model_evidence='User confirmed L1 in this session',
                    existing_slam_policy='observe only; never start/stop/rebuild SLAM')
    rclpy.init()
    node = rclpy.create_node('go2_readonly_time_diagnostic')
    executor = TimestampExecutor()
    executor.add_node(node)
    output = (folder/'samples.jsonl').open('x', buffering=1)

    def receive(msg, kind, info, topic):
        row = dict(kind=kind, topic=topic, ros_ns=node.get_clock().now().nanoseconds,
                   wall_ns=time.time_ns(), mono_ns=time.monotonic_ns(),
                   mono_raw_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                   dds_source_ns=info['source_timestamp'], dds_received_ns=info['received_timestamp'])
        if kind == 'lidar_state':
            row['stamp_ns'] = round(msg.stamp*1e9)
            row['fields'] = {name: [float(x) for x in getattr(msg, name)] if name == 'imu_rpy'
                             else getattr(msg, name) for name in msg.get_fields_and_field_types()}
            row['serial_recv_stamp_ns'] = round(msg.serial_recv_stamp*1e9)
        else:
            row['stamp_ns'] = ns(msg.header.stamp)
            row['frame'] = msg.header.frame_id
            if isinstance(msg, PointCloud2):
                row.update(cloud_times(msg))
        output.write(json.dumps(row)+'\n')

    def callback(kind, topic):
        def cb(msg, info):
            receive(msg, kind, info, topic)
        return cb
    try:
        until = time.monotonic()+5
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.1)
        metadata['initial_nodes'] = node.get_node_names()
        metadata['clock_publishers'] = node.count_publishers('/clock')
        topics = node.get_topic_names_and_types()
        metadata['discovered_topics'] = topics
        metadata['lidar_state_topics'] = [(name, types) for name, types in topics
                                          if any(t.endswith('/LidarState') for t in types)]
        metadata['lidar_state_discovery'] = (
            'DISCOVERED' if metadata['lidar_state_topics'] else 'NOT DISCOVERED: no guessed subscription')
        metadata['subscriptions'] = []
        specs = [('raw_cloud', '/utlidar/cloud', 'sensor_msgs/msg/PointCloud2'),
                 ('raw_imu', '/utlidar/imu', 'sensor_msgs/msg/Imu')]
        specs += [('lidar_state', name, types[0]) for name, types in metadata['lidar_state_topics']]
        # No publisher, client, service call, hardware configuration or process restart.
        from rosidl_runtime_py.utilities import get_message
        for kind, topic, typename in specs:
            try:
                msg_type = get_message(typename)
                sub = node.create_subscription(
                    msg_type, topic, callback(kind, topic), qos_profile_sensor_data)
                sub.time_probe = True
                writers = node.get_publishers_info_by_topic(topic)
                metadata['subscriptions'].append(dict(
                    topic=topic, type=typename,
                    publishers=[dict(node=w.node_name, gid=bytes(w.endpoint_gid).hex(),
                                     participant_prefix=bytes(w.endpoint_gid[:12]).hex()) for w in writers]))
            except (ImportError, AttributeError, ValueError) as exc:
                metadata['subscriptions'].append(dict(topic=topic, type=typename, error=str(exc)))
        metadata['record_start_wall_ns'] = time.time_ns()
        metadata['record_start_mono_ns'] = time.monotonic_ns()
        print('PC-only DDS采集，保持现有SLAM不动。目录:', folder, flush=True)
        print('实际LidarState发现:', metadata['lidar_state_topics'], flush=True)
        until = time.monotonic()+args.duration
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.02)
        metadata['final_nodes'] = node.get_node_names()
    finally:
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
