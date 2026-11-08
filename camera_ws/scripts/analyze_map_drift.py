#!/usr/bin/env python3
"""Analyze diagnostic events, or audit legacy recorder data without inventing missing fields."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use('Agg')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input')
    p.add_argument('--output', required=True)
    p.add_argument('--reference', nargs=2, type=float,
                   help='Fixed reference MAP XY; use the same A reference for B/C')
    p.add_argument('--identity-csv',
                   help='Offline video identity annotation: depth_stamp_ns,target_index')
    a = p.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    events = [json.loads(line) for line in open(a.input)]
    legacy = any('detection' in e for e in events)
    detections = [e['detection'] if legacy else e['data'] for e in events
                  if e.get('kind') == 'detection']
    dt = [(d['color_stamp_ns']-d['depth_stamp_ns'])/1e6 for d in detections]
    accepted_dt = [(d['color_stamp_ns']-d['depth_stamp_ns'])/1e6 for d in detections
                   if any(t['accepted'] for t in d['targets'])]
    report = {'source': a.input, 'detections': len(detections),
              'accepted_frames': len(accepted_dt),
              'time_difference_definition': 'RGB minus depth, milliseconds',
              'legacy_float_timestamp_roundoff': legacy}
    for key, values in [('all', dt), ('accepted', accepted_dt)]:
        if values:
            report[key] = dict(zip(['min_ms', 'median_ms', 'max_ms', 'abs_p95_ms'],
                                   [min(values), float(np.median(values)), max(values),
                                    float(np.percentile(np.abs(values), 95))]))
    measurements = [e['data'] for e in events if e.get('kind') == 'measurement']
    by_stamp = {d['depth_stamp_ns']: d for d in detections}
    tracks = [e['data'] for e in events if e.get('kind') == 'tracks']
    selection = None
    if a.identity_csv:
        selection = {int(r['depth_stamp_ns']): int(r['target_index'])
                     for r in csv.DictReader(open(a.identity_csv))}
        report['identity_annotation'] = a.identity_csv
        report['unfiltered_measurements'] = len(measurements)
    rows = []
    for m in measurements:
        d = by_stamp.get(m['stamp_ns'])
        if selection is not None:
            index = selection.get(m['stamp_ns'])
            if (d is None or index is None or
                    not np.allclose(d['targets'][index]['camera_xyz'], m['camera_xyz'],
                                    atol=1e-10, rtol=0)):
                continue
        # Multiple targets can share a stamp. Match the actual camera point, not array order.
        matches = [] if d is None else [
            t for t in d['targets'] if t['accepted'] and
            np.allclose(t['camera_xyz'], m['camera_xyz'], atol=1e-10, rtol=0)]
        t = matches[0] if len(matches) == 1 else None
        row = dict(stamp_ns=m['stamp_ns'], map_x=m['map_xyz'][0], map_y=m['map_xyz'][1],
                   map_z=m['map_xyz'][2], camera_x=m['camera_xyz'][0],
                   camera_y=m['camera_xyz'][1], camera_z=m['camera_xyz'][2],
                   range_m=float(np.linalg.norm(m['camera_xyz'])),
                   rgb_depth_ms=None if d is None else
                   (d['color_stamp_ns']-d['depth_stamp_ns'])/1e6,
                   u=None if t is None else t['center'][0],
                   v=None if t is None else t['center'][1])
        candidates = [t for snapshot in tracks for t in snapshot['tracks']
                      if t['observed_ns'] == m['stamp_ns'] and
                      np.allclose(t['latest_observation'], m['map_xyz'], atol=1e-8, rtol=0)]
        ids = sorted(set(t['id'] for t in candidates))
        row['target_ids'] = ids
        row['identity_status'] = 'matched' if len(ids) == 1 else 'missing_or_ambiguous'
        row['tf_translation'] = m['translation']
        row['tf_quaternion_xyzw'] = m['quaternion_xyzw']
        row['tf_query_ns'] = m['query_ns']
        row['tf_stamp_ns'] = m['tf_stamp_ns']
        row['rgb_stamp_ns'] = None if d is None else d['color_stamp_ns']
        row['depth_stamp_ns'] = m['stamp_ns']
        row['valid_depth_points'] = None if t is None else t['valid_depth_points']
        row['median_depth_m'] = None if t is None else t['depth_m']
        rows.append(row)
    fig, axes = plt.subplots(6, 1, figsize=(11, 18))
    if rows:
        xy = np.array([[r['map_x'], r['map_y']] for r in rows])
        reference = np.array(a.reference) if a.reference else xy[0]
        error = np.linalg.norm(xy-reference, axis=1)
        report.update(reference_xy=reference.tolist(), reference_origin='explicit' if a.reference
                      else 'first observation; verify physical target identity',
                      std_xy_m=np.std(xy, axis=0).tolist(), max_error_m=float(max(error)),
                      p95_error_m=float(np.percentile(error, 95)), measurements=len(rows))
        times = (np.array([r['stamp_ns'] for r in rows])-rows[0]['stamp_ns'])/1e9
        for r, err in zip(rows, error):
            r['reference_distance_xy'] = float(err)
        axes[0].plot(times, xy[:, 0], label='MAP X')
        axes[0].plot(times, xy[:, 1], label='MAP Y')
        axes[1].plot(times, [r['range_m'] for r in rows], label='camera range m')
        pixel_axis = axes[1].twinx()
        for k in ['u', 'v']:
            pixel_axis.plot(times, [np.nan if r[k] is None else r[k] for r in rows],
                            '--', label=k)
        pixel_axis.set_ylabel('pixels')
        pixel_axis.legend(loc='upper right')
        odom = [e['data'] for e in events if e.get('kind') == 'point_lio']

        def stamp(o):
            s = o['header']['stamp']
            return s['sec']*10**9+s['nanosec']
        odom.sort(key=stamp)
        ot = np.array([stamp(o) for o in odom], dtype=np.int64)
        positions = []
        yaws = []
        for r in rows:
            i = int(np.searchsorted(ot, r['stamp_ns']))
            valid = 0 < i < len(ot) and ot[i]-ot[i-1] <= 100_000_000
            if valid:
                fraction = (r['stamp_ns']-ot[i-1])/(ot[i]-ot[i-1])
                pp = [odom[j]['pose']['pose']['position'] for j in [i-1, i]]
                pos = [(1-fraction)*pp[0][k]+fraction*pp[1][k] for k in ['x', 'y', 'z']]
                qq = [odom[j]['pose']['pose']['orientation'] for j in [i-1, i]]
                q = np.array([[q[k] for k in ['x', 'y', 'z', 'w']] for q in qq])
                if np.dot(q[0], q[1]) < 0:
                    q[1] *= -1
                v = (1-fraction)*q[0]+fraction*q[1]
                v /= np.linalg.norm(v)
                x, y, z, w = v
                yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                r.update(lio_position=pos, lio_quaternion_xyzw=v.tolist(),
                         lio_bracket_ns=[int(ot[i-1]), int(ot[i])], lio_yaw=yaw,
                         lio_frame=odom[i]['header']['frame_id'])
                positions.append(pos[:2])
                yaws.append(yaw)
            else:
                positions.append([np.nan, np.nan])
                yaws.append(np.nan)
        positions = np.array(positions)
        axes[3].scatter(yaws, error, label='error vs LIO yaw')
        axes[3].scatter(positions[:, 0], error, label='error vs LIO X')
        report['lio_interpolation'] = (
            'position linear, quaternion normalized lerp; max bracket 100ms')
        report['lio_bracketed_count'] = sum('lio_yaw' in r for r in rows)
        valid = [r for r in rows if r['rgb_depth_ms'] is not None]
        axes[4].scatter([r['rgb_depth_ms'] for r in valid],
                        [r['reference_distance_xy'] for r in valid], label='error vs RGB-depth ms')
        with open(out/'measurements.csv', 'w') as f:
            writer = csv.DictWriter(
                f, fieldnames=sorted(set().union(*(r.keys() for r in rows))),
                lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)
    else:
        report['map_statistics'] = 'NOT VERIFIED: no timestamped raw MAP measurement events'
        for ax in [axes[0], axes[3], axes[4]]:
            ax.text(.05, .5, 'NOT VERIFIED: missing synchronized measurements',
                    transform=ax.transAxes)
    associations = [item for snapshot in tracks
                    for item in snapshot.get('association_diagnostics', [])]
    unique = {(item['stamp_ns'], item['observation_index'], item['id']): item
              for item in associations}
    if unique:
        items = list(unique.values())
        start = min(item['stamp_ns'] for item in items)
        for decision in ['ACCEPT', 'REJECT']:
            subset = [item for item in items if item['decision'] == decision]
            axes[5].scatter([(item['stamp_ns']-start)/1e9 for item in subset],
                            [item['distance_xy'] for item in subset], label=decision)
        with open(out/'associations.jsonl', 'w') as f:
            for item in items:
                f.write(json.dumps(item)+'\n')
    else:
        axes[5].text(.05, .5, 'NOT VERIFIED: no association events', transform=axes[5].transAxes)
    if detections:
        ts = [(d['depth_stamp_ns']-detections[0]['depth_stamp_ns'])/1e9 for d in detections]
        axes[2].plot(ts, dt, label='RGB minus depth ms')
        if not rows:
            accepted = [(time, t) for time, d in zip(ts, detections)
                        for t in d['targets'] if t['accepted']]
            axes[1].plot([v[0] for v in accepted], [v[1]['depth_m'] for v in accepted],
                         label='depth median m (legacy)')
            pixel_axis = axes[1].twinx()
            pixel_axis.plot([v[0] for v in accepted], [v[1]['center'][1] for v in accepted],
                            '--', label='pixel v (legacy)')
            pixel_axis.legend(loc='upper right')
    for ax in axes:
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend()
        ax.grid(True)
    fig.tight_layout()
    fig.savefig(out/'diagnostics.png')
    (out/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
