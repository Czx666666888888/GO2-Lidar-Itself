#!/usr/bin/env python3
"""Analyze one recorded docking run; no ROS/SDK, simulation or robot access."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/go2_science_perception'))
from go2_science_perception.docking_motion import WindowedTravel


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('--video-jsonl', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    data = json.loads(a.run.read_text())
    rows = data['control_reports']
    fault = next(r for r in rows if r.get('reason') ==
                 'remaining docking distance exceeds odometry budget')
    end, lost = fault['control_monotonic'], fault['visual_lost_at']
    loss = [r for r in rows if lost <= r['control_monotonic'] <= end]
    m = WindowedTravel(.5, .02)
    first_disagreement = None
    for r in loss:
        m.add(r['control_monotonic'], *r['robot'][:2], r['stop_inputs']['sport_speed'])
        if m.conflict and first_disagreement is None:
            first_disagreement = r['control_monotonic']-lost
    fine = [r for r in rows if r.get('docking_phase') == 'FINE_CONTROL']
    series, exits, buf = [], [], []
    for i, r in enumerate(rows):
        if r.get('docking_phase') == 'STOP_WAIT':
            buf.append((i, r))
        elif buf:
            group = len(exits)+1
            start = buf[0][1]
            exits.append(dict(group=group, start_index=buf[0][0], exit_index=i,
                duration_sec=r['control_monotonic']-start['control_monotonic'],
                entry_error_m=start['distance'], exit_error_m=r['distance'],
                exit_yaw_deg=math.degrees(r['grasp_docking']['yaw_error_rad']),
                vehicle_delta_xy=[r['robot'][j]-start['robot'][j] for j in (0, 1)],
                target_delta_xy=[r['target'][j]-start['target'][j] for j in (0, 1)],
                exit_reason=('distance_over_6cm' if r['distance'] > .06 else 'other'),
                max_stationary_run_sec=max((q.get('control_monotonic', 0)-
                    q['docking_stationary_since'] for _, q in buf
                    if q.get('docking_stationary_since') is not None), default=0.)))
            for index, q in [*buf, (i, r)]:
                inputs = q['stop_inputs'];g = q['grasp_docking']
                reasons = []
                if q['distance'] > .04: reasons.append('position_over_original_4cm')
                if abs(g['yaw_error_rad']) > math.radians(10): reasons.append('yaw_over_10deg')
                if max(inputs[k] for k in ('body_speed', 'camera_speed', 'sport_speed')) > .05:
                    reasons.append('speed_over_0.05mps')
                if max(inputs['pose_yaw_rate'], abs(inputs['sport_yaw_rate'])) > .10:
                    reasons.append('yaw_rate_over_0.10radps')
                row = dict(group=group, sample_index=index, monotonic=q['control_monotonic'],
                    vehicle_x=q['robot'][0], vehicle_y=q['robot'][1], vehicle_yaw=q['robot'][2],
                    target_x=q['target'][0], target_y=q['target'][1],
                    position_error_m=q['distance'], yaw_error_rad=g['yaw_error_rad'],
                    command_vx=q['vx'], command_vy=q['vy'], command_wz=q['wz'],
                    sport_vx=q['sport_velocity'][0], sport_vy=q['sport_velocity'][1],
                    sport_wz=q['sport_state']['yaw_speed'],
                    stationary_since=q.get('docking_stationary_since'),
                    timer_blockers=';'.join(reasons), **inputs)
                series.append(row)
            buf = []
    detections = [json.loads(line)['detection'] for line in a.video_jsonl.open()
                  if json.loads(line)['kind'] == 'detection']
    accepted = [d for d in detections if d.get('targets')]
    last = accepted[-1]
    after = [d for d in detections if d['color_stamp_ns'] > last['color_stamp_ns']]
    summary = dict(recorded_path_budget_m=fault['odom_travel_after_loss'],
        recorded_control_polyline_m=sum(math.dist(x['robot'][:2], y['robot'][:2])
            for x, y in zip(loss, loss[1:])),
        net_lio_displacement_m=math.dist(loss[0]['robot'][:2], loss[-1]['robot'][:2]),
        temporal_estimate=m.report(), first_disagreement_after_loss_sec=first_disagreement,
        independently_verified_physical_distance_m=None,
        independently_verified_jitter_distance_m=None,
        limitation='10Hz transformed vehicle pose is retained; raw Point-LIO Odometry is absent. '
                   'No independent physical truth. Differences are unresolved, not proven noise.',
        stop_wait_exits=exits,
        fine_samples=len(fine), positive_vx_below_015=sum(0 < r['vx'] < .15 for r in fine),
        positive_vx_below_020=sum(0 < r['vx'] < .20 for r in fine),
        fine_sport_below_001=sum(r['stop_inputs']['sport_speed'] < .01 for r in fine),
        last_valid_visual_detection=last,
        later_detection_count=len(after), later_detection_reasons=dict(Counter(d['reason'] for d in after)),
        first_budget_fault=dict(monotonic=end, ros_stamp_ns=fault['ros_stamp_ns'],
            visual_age_sec=fault['visual_age'], position_error_m=fault['distance']),
        evidence_sha256={str(f): hashlib.sha256(f.read_bytes()).hexdigest()
                         for f in (a.run, a.video_jsonl)})
    a.output.mkdir(parents=True, exist_ok=True)
    (a.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (a.output/'stop_wait_series.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(series[0]), lineterminator='\n');writer.writeheader();writer.writerows(series)
    print(json.dumps({k: summary[k] for k in ('recorded_path_budget_m',
        'recorded_control_polyline_m', 'net_lio_displacement_m', 'temporal_estimate')}, indent=2))


if __name__ == '__main__':
    main()
