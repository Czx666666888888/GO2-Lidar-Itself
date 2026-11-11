#!/usr/bin/env python3
"""Read one failed control log and real recording; never import ROS or alter data."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from statistics import median

import cv2
import yaml


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def analyze(control, recording):
    control, recording = Path(control), Path(recording)
    lines = [json.loads(s) for s in control.read_text().splitlines()]
    cfg = lines[0]['parameters']
    rows = [r for r in lines if 'state' in r]
    events = [json.loads(s) for s in recording.read_text().splitlines()]
    attempts = []
    starts = [r for r in rows if r.get('transition') == 'SEARCH->STOP_CONFIRM']
    for index, start in enumerate(starts):
        anchor = start['stop_confirm_started']
        group = [r for r in rows if r.get('stop_confirm_started') == anchor
                 and r['state'] == 'STOP_CONFIRM']
        end = next((r for r in rows if r.get('stop_confirm_started') == anchor
                    and r.get('transition') == 'STOP_CONFIRM->SEARCH'), None)
        trigger_stamp = start['stop_candidate']['trigger_stamp_ns']
        outcomes = [a for a in rows[-1].get('stop_confirm_attempts', [])
                    if a['candidate']['trigger_stamp_ns'] == trigger_stamp]
        settles = sorted({r['settled_at'] for r in group if r.get('settled_at') is not None})
        resets = sum(a.get('settled_at') is not None and b.get('settled_at') is None
                     for a, b in zip(group, group[1:]))
        speeds = [r for r in group if max(
            r['actual_speed'], r['camera_speed'],
            math.hypot(*(r.get('sport_velocity') or [0, 0])[:2]))
            > cfg['stop_confirm_speed_mps']]
        peak = max((r.get('stationary_sample_count', 0) or 0 for r in group), default=0)
        best = max(
            group, key=lambda r: (r.get('stationary_sample_count', 0) or 0,
                                  r.get('stationary_sample_span', 0) or 0))
        attempts.append(dict(
            attempt=index+1, start_elapsed_s=start['elapsed'],
            start_utc=datetime.fromtimestamp(start['ros_stamp_ns']/1e9, timezone.utc).isoformat(),
            trigger_range_m=start['stop_request_range_m'],
            duration_s=end['elapsed']-start['elapsed'] if end else None,
            outcome=outcomes[0] if outcomes else None,
            first_settled_after_s=settles[0]-anchor if settles else None,
            last_settled_after_s=settles[-1]-anchor if settles else None,
            settled_windows=len(settles), settled_resets_in_control_ticks=resets,
            ticks=len(group), ticks_exceeding_logged_speed_limit=len(speeds),
            reason_counts=dict(Counter(r.get('stop_confirm_reason', 'transition') for r in group)),
            peak_stationary_samples=peak,
            best_stationary_span_s=best.get('stationary_sample_span'),
            best_stationary_spread_m=best.get('stationary_spread'),
            best_samples=best.get('stationary_observations', []),
            max_body_speed_mps=max(r['actual_speed'] for r in group),
            max_camera_speed_mps=max(r['camera_speed'] for r in group),
            max_sport_xy_speed_mps=max(math.hypot(*(r.get('sport_velocity') or [0, 0])[:2])
                                       for r in group),
            nonzero_command_ticks=sum(r['vx'] != 0 or r['wz'] != 0 for r in group)))
    detections = [e for e in events if e['kind'] == 'detection']
    delay = [(e['record_ros_ns']-e['detection']['depth_stamp_ns'])*1e-9 for e in detections]
    videos = {}
    for path in (recording.with_suffix('.mp4'), recording.with_name(recording.stem+'_raw.mp4')):
        cap = cv2.VideoCapture(str(path))
        expected = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        count = 0
        while True:
            ok, _ = cap.read()
            if not ok:
                break
            count += 1
        cap.release()
        videos[path.name] = dict(decoded=count, container_frames=expected, sha256=digest(path))
    return dict(
        schema=1, kind='post_hoc_failed_trial_analysis',
        source_control=control.name, control_sha256=digest(control),
        source_recording=recording.name, recording_sha256=digest(recording),
        user_result='FAILED: multiple STOP_CONFIRM, never approached; stopped by user',
        parameters=cfg, state_counts=dict(Counter(r['state'] for r in rows)),
        attempts=attempts, first_detection_range_m=rows[-1]['search_first_detection_range_m'],
        max_search_progress_m=max(r.get('search_progress', 0) or 0 for r in rows),
        final={k: rows[-1].get(k) for k in ('state', 'reason', 'elapsed', 'target_id', 'target')},
        control_references=dict(Counter(r['control_reference'] for r in rows
                                        if r.get('control_reference'))),
        detection_receipt_delay_s=dict(min=min(delay), median=median(delay), max=max(delay)),
        camera_relative_decisions=dict(Counter(r['camera_relative_association']['decision']
                                               for r in rows)),
        videos=videos,
        original_runner_summary_present=(control.parent/'run_1794409701596886177.json').exists(),
        interpretation_limits=[
            'Tick speed checks exclude unlogged pose_yaw_rate and input ages.',
            'Detection receipt delay does not distinguish clocks from transport/processing.',
            'Source-time MAP transforms and track snapshots were not continuously recorded.',
            'Control log localization timeout is distinct from user physical stop.',
            'No approach: visual/MAP switching, relock and final accuracy unverified.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control', required=True)
    parser.add_argument('--recording', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = analyze(args.control, args.recording)
    Path(args.output).write_text(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
    display = {k: v for k, v in report.items() if k not in ('parameters', 'videos')}
    print(yaml.safe_dump(display, allow_unicode=True, sort_keys=False))
