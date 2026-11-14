#!/usr/bin/env python3
"""Read-only clock mapping/localization diagnostic; never publishes motion commands."""
import argparse
from collections import Counter
from datetime import datetime
import json
import importlib.util
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time
import traceback

import numpy as np
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException

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


def native_lidar_summary(groups):
    """Compare actual PC reception with producer-reported health; never configure it."""
    clouds = groups['raw_cloud']
    elapsed = ((clouds[-1]['mono_ns']-clouds[0]['mono_ns'])/1e9
               if len(clouds) > 1 else 0)
    received_hz = (len(clouds)-1)/elapsed if elapsed > 0 else None
    fields = {}
    for name in ('cloud_frequency', 'cloud_packet_loss_rate', 'cloud_scan_num',
                 'cloud_size', 'imu_frequency', 'imu_packet_loss_rate', 'error_state'):
        fields[name] = stats([(r['mono_ns']/1e9, r['fields'][name])
                              for r in groups['lidar_state'] if name in r['fields']])
    span = stats([(r['mono_ns']/1e9, r['point_time_max_s'])
                  for r in clouds if 'point_time_max_s' in r])
    reported_hz = fields['cloud_frequency'].get('median')
    if received_hz is None or reported_hz is None:
        assessment = 'NOT VERIFIED: missing cloud or LidarState samples'
    elif received_hz < 5 and reported_hz < 5:
        assessment = 'LOW_AT_PRODUCER: producer and PC both observe low frequency'
    elif received_hz < 5 and reported_hz >= 10:
        assessment = 'PC_PATH_SUSPECT: producer reports normal frequency; inspect DDS reception'
    else:
        assessment = 'OBSERVED: compare PC frequency, producer frequency and scan span together'
    # Native-feed checks use the measured ~15 Hz / ~64 ms reference. These
    # checks do not alter the converter or any controller timestamp tolerance.
    checks = dict(pc_cloud_rate_at_least_10hz=received_hz is not None and received_hz >= 10,
                  producer_cloud_rate_at_least_10hz=reported_hz is not None and reported_hz >= 10,
                  producer_loss_at_most_10pct=fields['cloud_packet_loss_rate'].get('median', 101) <= 10,
                  median_scan_at_most_100ms=span.get('median', 1) <= .1)
    return dict(pc_received_hz=received_hz, received_clouds=len(clouds),
                producer_fields=fields, scan_span_s=span, assessment=assessment,
                checks=checks, passed=all(checks.values()),
                limitation='LidarState reports producer health; exact pre-DDS loss mechanism is unknown. '
                           'cloud_scan_num has no proven equality to the standalone SDK initialization argument.')


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
              ['raw_cloud', 'raw_imu', 'cloud', 'imu', 'state', 'base', 'tf', 'go2_clock', 'lidar_state', 'mapping', 'map_debug', 'camera', 'tracks', 'coarse', 'map_point', 'selected']}
    native = native_lidar_summary(groups)
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
        r.get('success', True) and r.get('tf_stamp_ns') == r.get('query_ns') for r in groups['tf'])
    result['tf_queries'] = len(groups['tf'])
    result['receive_delay_warning'] = 'Raw stamp uses a different epoch; receive age is NOT network latency.'
    result['native_lidar'] = native
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
    if metadata.get('transport') in ('IMU mapping read-only localization',
                                      'Read-only time conversion/localization'):
        statuses = [r['fields'] for r in groups['mapping']]
        result['time_mapping'] = statuses[-1] if statuses else None
        result['time_conversion_mode'] = statuses[-1].get('mode', 'imu') if statuses else None
        result['mapping_offset_stability_s'] = stats([
            (r['mono_ns']/1e9, r['fields']['residual_offset_sec'])
            for r in groups['mapping'] if r['fields'].get('residual_offset_sec') is not None])
        result['stamp_nonadvancing'] = {}
        result['maximum_receive_gap_s'] = {}
        result['maximum_position_step_m'] = {}
        for kind in ('cloud', 'imu', 'state', 'base'):
            ordered = groups[kind]
            result['stamp_nonadvancing'][kind] = sum(
                b['stamp_ns'] <= a['stamp_ns'] for a, b in zip(ordered, ordered[1:]))
            result['maximum_receive_gap_s'][kind] = max([
                (b['mono_ns']-a['mono_ns'])/1e9 for a, b in zip(ordered, ordered[1:])], default=None)
            if kind in ('state', 'base'):
                result['maximum_position_step_m'][kind] = max([
                    float(np.linalg.norm(np.array(b['position'])-np.array(a['position'])))
                    for a, b in zip(ordered, ordered[1:])], default=None)
        exact = {}
        for stream in ('base', 'camera'):
            queries = [x for x in groups['tf'] if x.get('stream') == stream]
            successes = sum(x.get('success', False) and x.get('tf_stamp_ns') == x['query_ns']
                            for x in queries)
            exact[stream] = dict(queries=len(queries), exact_success=successes,
                                success_rate=successes/len(queries) if queries else None)
        result['exact_source_time_tf'] = exact
        debug = groups['map_debug']
        result['target_map_exact_stamp_matches'] = sum(
            x['fields'].get('query_ns') == x['fields'].get('stamp_ns') ==
            x['fields'].get('tf_stamp_ns') for x in debug)
        result['target_map_samples'] = len(debug)
        result['target_map_validation'] = ('OBSERVED exact source-time target transforms' if debug and
            result['target_map_exact_stamp_matches'] == len(debug)
                                         else 'NOT VERIFIED: missing or non-exact target map measurements')
        confirmed = [t for r in groups['tracks'][-1:] for t in r['fields'].get('tracks', [])
                     if t.get('state') == 'CONFIRMED' and not t.get('visited')]
        result['target_manager'] = dict(track_snapshots=len(groups['tracks']),
            confirmed_unvisited_ids=sorted({t['id'] for t in confirmed}),
            selected_samples=len(groups['selected']), coarse_samples=len(groups['coarse']),
            map_point_samples=len(groups['map_point']))
        params = yaml.safe_load((REPO/'camera_ws/src/go2_science_perception/config/'
                                 'science_target_approach.yaml').read_text())['science_target_approach']['ros__parameters']
        future_limit, odom_age_limit = params['future_tolerance'], params['odom_timeout']
        time_keys = ('state_future_s', 'base_future_s', 'derived_lidar_end_future_s')
        result['diagnostic_goals'] = dict(median_future_at_most_20ms=all(
            metrics[k].get('median', 1) <= .020 for k in time_keys))
        result['controller_timestamp_limits_s'] = dict(future_tolerance=future_limit,
                                                       odom_timeout=odom_age_limit)
        checks = dict(
            native_lidar_healthy=native['passed'],
            clock_publishers_absent=metadata.get('clock_publishers') == 0,
            slam_uses_wall_clock=(metadata.get('laser_mapping_use_sim_time', {}).get('code') == 0
                and 'False' in metadata['laser_mapping_use_sim_time'].get('stdout', '')),
            raw_inputs_received=all(len(groups[k]) >= 10 for k in ('raw_cloud', 'raw_imu')),
            mapping_ready=bool(statuses and statuses[-1].get('ready')),
            mapping_without_fault=bool(statuses and all(not x.get('fault') for x in statuses)),
            continuous_inputs=all(len(groups[k]) >= 10 and
                (result['maximum_receive_gap_s'][k] or 0) <= 1 for k in ('cloud', 'imu', 'state', 'base')),
            stamps_increasing=not any(result['stamp_nonadvancing'].values()),
            finite_positions=all(all(np.isfinite(x) for x in row['position'])
                for k in ('state', 'base') for row in groups[k]),
            controller_timestamp_limits=all(
                metrics[k].get('maximum', 1) <= future_limit
                and metrics[k].get('minimum', -2) >= -odom_age_limit for k in time_keys),
            controller_odom_continuity=all(
                result['maximum_receive_gap_s'][k] is not None and
                result['maximum_receive_gap_s'][k] <= odom_age_limit for k in ('state', 'base')),
            exact_base_tf=bool(exact['base']['queries'] and exact['base']['success_rate'] >= .95),
            exact_camera_tf=bool(exact['camera']['queries'] and exact['camera']['success_rate'] >= .95),
            capture_complete=metadata.get('record_complete', False),
            setup_ok=not metadata.get('setup_or_capture_error'))
        if result['time_conversion_mode'] == 'legacy':
            checks['legacy_offset_initialized'] = checks.pop('mapping_ready')
        if metadata.get('arguments', {}).get('require_target'):
            checks['exact_target_map'] = bool(len(debug) >= 5 and
                result['target_map_exact_stamp_matches'] == len(debug))
            checks['confirmed_selected_target'] = bool(confirmed and len(groups['selected']) >= 5)
            checks['fresh_confirmed_target'] = bool(confirmed and any(
                -future_limit <= (groups['tracks'][-1]['ros_ns']-t['observed_ns'])/1e9
                <= params['visual_timeout'] for t in confirmed))
        result['acceptance_checks'] = checks
        result['status'] = ('PASS: read-only mapped localization and exact camera TF observed'
                            if all(checks.values()) else 'NOT VERIFIED: '+', '.join(
                                k for k, passed in checks.items() if not passed))
        result['earliest_bias_stage'] = (
            'Historical first-cloud receipt minus header, fixed and shared by cloud/IMU; '
            'includes scan/transport delay, historically about 57 ms odometry future bias'
            if result['time_conversion_mode'] == 'legacy' else
            'IMU receipt P5 approximates native-to-local offset before shared cloud/IMU conversion; '
            'remaining one-way network/dispatch delay is not independently identifiable')
        result['position_jump_assessment'] = (
            'Observed position-step statistics only; physical trajectory/reference required to judge jumps')
        result['native_clock_repaired'] = False
        result['setup_or_capture_error'] = metadata.get('setup_or_capture_error')
    if metadata.get('arguments', {}).get('lidar_check'):
        result['status'] = ('PASS: native LiDAR feed only; localization/time mapping NOT VERIFIED'
                            if native['passed'] and metadata.get('record_complete')
                            and not metadata.get('setup_or_capture_error')
                            else 'NOT VERIFIED: native LiDAR feed; inspect native_lidar checks')
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
                                    after_verified=result['status'].startswith('PASS'))
    (folder/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    lines = [result['status'], result['receive_delay_warning'],
             'All metric units: seconds; slope: seconds/second.',
             'Native LiDAR (frequencies Hz, loss percent): '+json.dumps(native)]
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


class ReadonlyChain:
    """Own only processes started here; existing localization is reused by default."""
    def __init__(self, node, executor, folder):
        self.node, self.executor, self.folder = node, executor, folder
        self.children = []

    def spin_for(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.05)

    def spawn(self, argv, label, stdin=None):
        stream = (self.folder/(label+'.log')).open('w')
        try:
            process = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                                       stdin=stdin, start_new_session=True)
        except BaseException:
            stream.close()
            raise
        self.children.append((process, stream, label))
        return process

    def start(self, args):
        names = self.node.get_node_names()
        if not all(self.node.count_publishers(t) for t in ('/utlidar/cloud', '/utlidar/imu')):
            raise RuntimeError('未发现原生LiDAR/IMU发布者；不启动新定位')
        if (self.node.count_publishers('/cmd_vel_raw') or
                any(n in names for n in ('go2_safety_gate', 'science_target_approach'))):
            raise RuntimeError('运动控制仍在运行；只读诊断拒绝更改定位会话')
        if args.restart_localization:
            # Explicit opt-in: only known localization/perception processes and
            # their launchers, never an arbitrary ROS process or camera service.
            processes = subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True)
            selected = []
            leaves = {'pointlio_mapping', 'transform_everything', 'base_odom_node',
                      'coarse_target_locator', 'camera_target_to_map', 'science_target_manager'}
            for line in processes.splitlines():
                pid_text = line.strip().split(None, 1)[0]
                try:
                    argv = Path('/proc', pid_text, 'cmdline').read_bytes().decode(
                        errors='replace').rstrip('\0').split('\0')
                except (FileNotFoundError, ProcessLookupError, PermissionError):
                    continue
                if not argv[0]:
                    continue
                executable = Path(argv[0]).name
                python_leaf = (len(argv) > 1 and executable.startswith('python')
                               and Path(argv[1]).name in leaves)
                launcher = ('launch' in argv and any(x in argv for x in
                            ('mapping_utlidar.launch', 'd435_slam_target_validation.launch.py'))
                            and (executable == 'ros2' or
                                 (executable.startswith('python') and len(argv) > 1
                                  and Path(argv[1]).name == 'ros2')))
                static = (executable == 'static_transform_publisher' and any(
                    '__node:='+n in argv for n in
                    ('map_to_camera_init', 'aft_mapped_to_sensor', 'sensor_to_vehicle')))
                if executable in leaves or python_leaf or launcher or static:
                    selected.append(int(pid_text))
            (self.folder/'replaced_localization_pids.json').write_text(json.dumps(selected))
            for pid in selected:
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            self.spin_for(5)
            for pid in selected:
                if Path('/proc/'+str(pid)).exists():
                    raise RuntimeError('旧定位进程未退出，保留现场，PID='+str(pid))
            names = self.node.get_node_names()
        existing = any(n in names for n in ('laserMapping', 'transform_everything', 'sensor_transformer'))
        if existing:
            print('复用现有定位；若仍运行旧转换器，请显式使用 --restart-localization。', flush=True)
            return
        if args.reuse_only:
            raise RuntimeError('--reuse-only 未发现已有定位节点')
        if not self.node.count_publishers('/camera/camera/color/image_raw') and args.camera_host != 'existing':
            if args.camera_host != 'local':
                sys.path.insert(0, str(REPO/'scripts'))
                spec = importlib.util.spec_from_file_location('go2_existing_helper', REPO/'scripts/go2_full_test.py')
                helper = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(helper)
                ssh = ['ssh', '-i', os.environ.get('GO2_CAMERA_SSH_KEY', helper.KEY),
                       '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5']
                # Read-only companion clock check; never SSH to the source main controller.
                helper.verify_clock([*ssh, args.camera_host,
                                     shlex.join(['/usr/bin/python3', '-u', '-c', helper.CLOCK_RESPONDER])])
                check = command([*ssh, args.camera_host,
                                 "pgrep -f '[r]ealsense2_camera_node'"], timeout=10)
                if check.get('code') != 1:
                    raise RuntimeError('远端相机已有进程或检查失败；拒绝重复打开设备: '+str(check))
                self.spawn([*ssh, args.camera_host, shlex.join([
                    '/usr/bin/python3', '-c', helper.CAMERA_OWNER,
                    os.environ.get('GO2_CAMERA_LAUNCH', helper.REMOTE+'/launch_go2_d435_foxy.sh')])],
                    'camera', stdin=subprocess.PIPE)
        remote = 'false' if args.camera_host == 'local' else 'true'
        self.spawn(['ros2', 'launch', 'go2_science_perception',
                    'd435_slam_target_validation.launch.py',
                    'remote_camera:='+remote, 'use_rviz:=false'], 'readonly_slam')
        self.spin_for(15)  # IMU window + initial Point-LIO map; capture follows.

    def close(self):
        for process, stream, label in reversed(self.children):
            try:
                if process.poll() is None:
                    if label == 'camera':
                        process.stdin.close()
                    else:
                        os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
            finally:
                stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--label', default='localization')
    parser.add_argument('--time-mode', choices=('legacy', 'imu', 'explicit'),
                        default=os.environ.get('GO2_TIME_MAPPING_MODE', 'legacy'))
    parser.add_argument('--observe-only', action='store_true', help='只记录原生DDS，不构建/启动/停止节点')
    parser.add_argument('--lidar-check', action='store_true', help='仅验原生点云/LidarState健康，不以原生epoch作UTC验收')
    parser.add_argument('--require-target', action='store_true', help='同时要求真实目标精确MAP转换与CONFIRMED/selected输出')
    parser.add_argument('--prepare-only', action='store_true', help='仅由shell准备overlay，不初始化ROS')
    parser.add_argument('--restart-localization', action='store_true', help='明确结束旧只读定位并建立新地图会话')
    parser.add_argument('--reuse-only', action='store_true', help='只检查现有定位，不启动任何设备/节点')
    parser.add_argument('--keep-running', action='store_true', help='报告通过后保持本入口启动的只读链，Ctrl+C退出')
    parser.add_argument('--camera-host', default=os.environ.get('GO2_CAMERA_HOST', 'unitree@192.168.123.18'),
                        help='相机伴随主机SSH名，或local（Orin本地）、existing（仅复用）')
    args = parser.parse_args()
    os.environ['GO2_TIME_MAPPING_MODE'] = args.time_mode
    if args.lidar_check:
        args.observe_only = True
    if not 0 < args.duration <= 120:
        parser.error('duration must be in (0,120] seconds')
    if args.prepare_only:
        return
    if args.observe_only and (args.restart_localization or args.keep_running):
        parser.error('--observe-only cannot restart/keep localization')
    if args.observe_only and args.require_target:
        parser.error('--require-target requires localization')
    if args.reuse_only and args.restart_localization:
        parser.error('--reuse-only cannot restart localization')
    folder = REPO/'camera_ws/records'/('time_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.label)
    folder.mkdir(parents=True)
    metadata = dict(commit=command(['git', '-C', str(REPO), 'rev-parse', 'HEAD']),
                    pc_time_status=command(['timedatectl', 'status']),
                    pc_timesync_status=(dict(skipped='controlled PC time context: avoid D-Bus activation')
                        if os.environ.get('GO2_PC_TIME_HELD') == '1'
                        else command(['timedatectl', 'timesync-status'])),
                    route=command(['ip', 'route']),
                    interfaces=command(['ip', '-br', 'addr']),
                    domain=os.environ.get('ROS_DOMAIN_ID'),
                    rmw=os.environ.get('RMW_IMPLEMENTATION'),
                    cyclone_configuration=os.environ.get('CYCLONEDDS_URI'),
                    transport=('PC ROS2 DDS; native observation only' if args.observe_only
                               else 'Read-only time conversion/localization'),
                    hardware_model='L1', model_evidence='User confirmed L1 in this session',
                    existing_slam_policy=('observe only' if args.observe_only else
                                          'reuse existing; restart only with explicit flag'),
                    arguments=vars(args))
    rclpy.init()
    node = rclpy.create_node('go2_readonly_time_diagnostic')
    executor = TimestampExecutor()
    executor.add_node(node)
    output = (folder/'samples.jsonl').open('x', buffering=1)
    chain = ReadonlyChain(node, executor, folder)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    pending_tf = []
    result = {'status': 'NOT VERIFIED: interrupted capture'}

    def check_tf():
        for item in list(pending_tf):
            row, source, deadline = item
            try:
                transform = buffer.lookup_transform('map', source,
                    rclpy.time.Time(nanoseconds=row['query_ns']))
                row['success'] = True
                row['tf_stamp_ns'] = ns(transform.header.stamp)
            except TransformException as exc:
                if time.monotonic() < deadline:
                    continue
                row['success'] = False
                row['error'] = str(exc)
            row['query_completed_wall_ns'] = time.time_ns()
            output.write(json.dumps(row)+'\n')
            pending_tf.remove(item)

    def receive(msg, kind, info, topic):
        row = dict(kind=kind, topic=topic, ros_ns=node.get_clock().now().nanoseconds,
                   wall_ns=time.time_ns(), mono_ns=time.monotonic_ns(),
                   mono_raw_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                   dds_source_ns=info['source_timestamp'], dds_received_ns=info['received_timestamp'])
        if kind in ('mapping', 'map_debug', 'tracks'):
            row['fields'] = json.loads(msg.data)
        elif kind == 'lidar_state':
            row['stamp_ns'] = round(msg.stamp*1e9)
            row['fields'] = {name: [float(x) for x in getattr(msg, name)] if name == 'imu_rpy'
                             else getattr(msg, name) for name in msg.get_fields_and_field_types()}
            row['serial_recv_stamp_ns'] = round(msg.serial_recv_stamp*1e9)
        else:
            row['stamp_ns'] = ns(msg.header.stamp)
            row['frame'] = msg.header.frame_id
            if kind in ('coarse', 'map_point', 'selected'):
                row['position'] = [msg.point.x, msg.point.y, msg.point.z]
            if isinstance(msg, PointCloud2):
                row.update(cloud_times(msg))
            if kind in ('state', 'base'):
                p = msg.pose.pose.position
                row['position'] = [p.x, p.y, p.z]
            if kind in ('base', 'camera'):
                pending_tf.append((dict(kind='tf', ros_ns=row['ros_ns'], wall_ns=row['wall_ns'],
                    mono_ns=row['mono_ns'], query_ns=row['stamp_ns'],
                    source_frame=('vehicle' if kind == 'base' else row['frame']), stream=kind),
                    'vehicle' if kind == 'base' else row['frame'], time.monotonic()+.5))
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
        if not args.observe_only:
            chain.start(args)
        metadata['clock_publishers'] = node.count_publishers('/clock')
        metadata['laser_mapping_use_sim_time'] = command(
            ['ros2', 'param', 'get', '/laserMapping', 'use_sim_time'], timeout=5)
        topics = node.get_topic_names_and_types()
        metadata['discovered_topics'] = topics
        metadata['lidar_state_topics'] = [(name, types) for name, types in topics
                                          if any(t.endswith('/LidarState') for t in types)]
        metadata['lidar_state_discovery'] = (
            'DISCOVERED' if metadata['lidar_state_topics'] else 'NOT DISCOVERED: no guessed subscription')
        metadata['subscriptions'] = []
        specs = [('raw_cloud', '/utlidar/cloud', 'sensor_msgs/msg/PointCloud2'),
                 ('raw_imu', '/utlidar/imu', 'sensor_msgs/msg/Imu')]
        if not args.observe_only:
            specs += [('cloud', '/utlidar/transformed_cloud', 'sensor_msgs/msg/PointCloud2'),
                      ('imu', '/utlidar/transformed_imu', 'sensor_msgs/msg/Imu'),
                      ('state', '/state_estimation', 'nav_msgs/msg/Odometry'),
                      ('base', '/base_state_estimation', 'nav_msgs/msg/Odometry'),
                      ('camera', '/camera/camera/color/image_raw', 'sensor_msgs/msg/Image'),
                      ('mapping', '/utlidar/time_mapping/status', 'std_msgs/msg/String'),
                      ('map_debug', '/science/map_measurement_debug', 'std_msgs/msg/String'),
                      ('tracks', '/science/target_tracks', 'std_msgs/msg/String'),
                      ('coarse', '/science/target_coarse_point', 'geometry_msgs/msg/PointStamped'),
                      ('map_point', '/science/target_coarse_point_map', 'geometry_msgs/msg/PointStamped'),
                      ('selected', '/science/selected_target', 'geometry_msgs/msg/PointStamped')]
        specs += [('lidar_state', name, types[0]) for name, types in metadata['lidar_state_topics']]
        # These subscriptions only observe; lifecycle changes occur in ReadonlyChain.
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
        print('只读时间采集；不发布运动命令。目录:', folder, flush=True)
        print('实际LidarState发现:', metadata['lidar_state_topics'], flush=True)
        until = time.monotonic()+args.duration
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.02)
            check_tf()
        drain_until = time.monotonic()+.6
        while pending_tf and time.monotonic() < drain_until:
            executor.spin_once(timeout_sec=.02)
            check_tf()
        metadata['record_complete'] = True
        metadata['final_nodes'] = node.get_node_names()
    except Exception as exc:
        metadata['setup_or_capture_error'] = str(exc)
        metadata['setup_or_capture_traceback'] = traceback.format_exc()
        print('诊断未完成:', exc, flush=True)
        traceback.print_exc()
    finally:
        output.flush()
        metadata['record_end_wall_ns'] = time.time_ns()
        (folder/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
        try:
            result = analyze(folder)
            if args.keep_running and result['status'].startswith('PASS'):
                print('报告已完成；只读定位保持运行，Ctrl+C仅停止本入口拥有的节点。', flush=True)
                while rclpy.ok():
                    time.sleep(.2)
        except KeyboardInterrupt:
            pass
        finally:
            chain.close()
        output.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not result['status'].startswith(('PASS', 'OBSERVED')):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
