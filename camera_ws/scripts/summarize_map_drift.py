#!/usr/bin/env python3
"""Summarize recorded phases without treating odometry or identities as external truth."""
import csv
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'analysis/map_drift_20261108'


def main():
    results = {}
    for label, folder in [('A', 'A'), ('B2', 'B2_selected'), ('C', 'C_selected')]:
        events = [json.loads(line) for line in
                  open(ROOT / f'records/map_drift_{label}_20261108.jsonl')]
        rows = list(csv.DictReader(open(OUT / folder / 'measurements.csv')))
        odom = [e['data'] for e in events if e['kind'] == 'point_lio']
        times = np.array([o['header']['stamp']['sec'] +
                          o['header']['stamp']['nanosec']/1e9 for o in odom])
        pos = np.array([[o['pose']['pose']['position'][k] for k in ['x', 'y', 'z']]
                        for o in odom])
        q = np.array([[o['pose']['pose']['orientation'][k] for k in ['x', 'y', 'z', 'w']]
                      for o in odom])
        yaw = np.unwrap(np.arctan2(2*(q[:, 3]*q[:, 2]+q[:, 0]*q[:, 1]),
                                   1-2*(q[:, 1]**2+q[:, 2]**2)))
        dt = np.diff(times)
        step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
        valid = [r for r in rows if r.get('lio_yaw')]
        error = [float(r['reference_distance_xy']) for r in valid]
        variables = {'abs_rgb_depth_ms': [abs(float(r['rgb_depth_ms'])) for r in valid],
                     'lio_yaw': np.unwrap([float(r['lio_yaw']) for r in valid]),
                     'pixel_u': [float(r['u']) for r in valid],
                     'camera_z': [float(r['camera_z']) for r in valid]}
        summary = json.loads((OUT / folder / 'summary.json').read_text())
        residual = []
        for r in rows:
            x, y, z, w = json.loads(r['tf_quaternion_xyzw'])
            rotation = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            camera = np.array([float(r[k]) for k in ['camera_x', 'camera_y', 'camera_z']])
            mapped = np.array([float(r[k]) for k in ['map_x', 'map_y', 'map_z']])
            residual.append(float(np.linalg.norm(rotation @ camera +
                                                 json.loads(r['tf_translation'])-mapped)))
        summary.update(
            odom_count=len(odom), odom_duration_s=float(times[-1]-times[0]),
            odom_sample_interval_min_max_s=[float(min(dt)), float(max(dt))],
            odom_position_span_m=np.ptp(pos, axis=0).tolist(),
            odom_max_step_3d_m=float(max(step)),
            odom_max_position_derivative_mps=float(max(step/dt)),
            odom_max_yaw_derivative_radps=float(max(abs(np.diff(yaw)/dt))),
            odom_yaw_relative_min_max_end_deg=np.rad2deg(
                [min(yaw)-yaw[0], max(yaw)-yaw[0], yaw[-1]-yaw[0]]).tolist(),
            odom_max_xy_displacement_m=float(max(np.linalg.norm(pos[:, :2]-pos[0, :2], axis=1))),
            odom_end_xy_displacement_m=float(np.linalg.norm(pos[-1, :2]-pos[0, :2])),
            error_pearson={k: float(np.corrcoef(v, error)[0, 1]) for k, v in variables.items()},
            tf_arithmetic_residual_max_m=max(residual),
            tf_query_matches_measurement=all(r['tf_query_ns'] == r['stamp_ns'] for r in rows),
            target_ids=sorted(set(i for r in rows for i in json.loads(r['target_ids']))),
            interpretation='Internal consistency only. Pose derivatives are not physical truth. '
            'Identity annotation assumptions and incomplete motion coverage remain material.')
        results[label] = summary
    (OUT / 'comparison.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
