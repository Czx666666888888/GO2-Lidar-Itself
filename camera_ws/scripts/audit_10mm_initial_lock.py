#!/usr/bin/env python3
"""Audit the Nov-7 initial lock using user-supplied 10 mm size; no ROS or motion."""
import csv
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'analysis/target_10mm_audit_20261108'
RUN = ROOT / 'records/run_1794043913372908704.json'
LOG = ROOT / 'records/approach_20261107_173154_217135.jsonl'
SIDE_M = 0.01


def write_csv(path, rows):
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def render_evidence(events, detections, locked_rows):
    """Match source RGB stamps, retaining the original video's baked overlays."""
    import cv2

    frames = [e for e in events if e['kind'] == 'frame']
    times = np.array([e['rgb_stamp_ns'] for e in frames], dtype=np.int64)
    selected = [('T88 source %d' % (i+1), r['color_stamp_ns'])
                for i, r in enumerate(locked_rows)]
    selected += [('T89 confirm vicinity', 1794043928000000000),
                 ('T90 confirm vicinity', 1794043931100000000),
                 ('two candidates', 1794043930590970624),
                 ('two candidates', 1794043932068484608),
                 ('late visible region', 1794043934956753664)]
    cap = cv2.VideoCapture(str(LOG.with_suffix('.mp4')))
    tiles, metadata = [], []
    for label, desired in selected:
        d = min(detections, key=lambda d: abs(d['color_stamp_ns']-desired))
        frame = frames[int(np.argmin(abs(times-d['color_stamp_ns'])))]
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame['frame_index'])
        ok, image = cap.read()
        if not ok:
            raise ValueError('Video frame decode failed')
        for target in d['targets']:
            polygons = [np.array(c, np.int32).reshape(-1, 1, 2) for c in target['contours']]
            cv2.polylines(image, polygons, True, (0, 0, 255), 1)
        cv2.putText(image, label, (4, 165), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 1)
        cv2.putText(image, f"frame={frame['frame_index']} RGB={frame['rgb_stamp_ns']}",
                    (4, 185), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
        tiles.append(cv2.resize(image, (480, 360)))
        metadata.append(dict(label=label, frame_index=frame['frame_index'],
                             rgb_stamp_ns=frame['rgb_stamp_ns'],
                             detection_rgb_ns=d['color_stamp_ns'],
                             frame_minus_detection_ns=frame['rgb_stamp_ns']-d['color_stamp_ns']))
    cap.release()
    sheet = cv2.vconcat([cv2.hconcat(tiles[i:i+2]) for i in range(0, len(tiles), 2)])
    cv2.imwrite(str(OUT / 'source_frame_review.jpg'), sheet)
    (OUT / 'source_frame_index.json').write_text(json.dumps(metadata, indent=2)+'\n')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    calibration = (ROOT / 'docs/calibration.md').read_text()
    match = re.search(r'fx=([\d.]+)`、`fy=([\d.]+)`、`cx=([\d.]+)`、`cy=([\d.]+)',
                      calibration)
    if match is None:
        raise ValueError('Documented color intrinsics not found; do not guess')
    fx, fy, cx, cy = map(float, match.groups())
    depth_match = re.search(r'raw depth CameraInfo.*?fx=fy=([\d.]+)', calibration, re.S)
    if depth_match is None:
        raise ValueError('Documented native depth focal length not found')
    depth_f = float(depth_match.group(1))
    config = yaml.safe_load((ROOT / 'src/go2_science_perception/config/'
                             'coarse_target_locator.yaml').read_text())
    params = config['coarse_target_locator']['ros__parameters']
    projection = []
    for z in [0.3, 0.5, 1.0, 2.0, 2.7]:
        projection.append(dict(depth_z_m=z, width_px=fx*SIDE_M/z, height_px=fy*SIDE_M/z,
                               front_face_area_px=fx*fy*(SIDE_M/z)**2,
                               native_depth_width_px=depth_f*SIDE_M/z,
                               passes_250_face_area=fx*fy*(SIDE_M/z)**2 >= 250))
    write_csv(OUT / 'projection.csv', projection)
    events = [json.loads(line) for line in LOG.open()]
    detections = [{**e['detection'], 'received_ns': e['record_ros_ns']}
                  for e in events if e['kind'] == 'detection']
    controls = json.loads(RUN.read_text())['control_reports']
    lock = next(c for c in controls if c.get('target_id') == 88 and c['state'] == 'LOCK_TARGET')
    observations = {
        o['observed_ns']: o for c in controls if c['ros_stamp_ns'] <= lock['ros_stamp_ns']
        for o in c.get('search_observations', [])
        if o['id'] == 88 and lock['ros_stamp_ns']-10**9 <= o['observed_ns'] <= lock['ros_stamp_ns']}
    locked_rows = []
    for stamp, observation in sorted(observations.items()):
        d = min(detections, key=lambda d: abs(d['depth_stamp_ns']-stamp))
        if abs(d['depth_stamp_ns']-stamp) > 256:
            raise ValueError('Missing timestamp match for locked observation')
        targets = [t for t in d['targets'] if t['accepted']]
        if len(targets) != 1:
            raise ValueError('Locked observation identity is not unique in this frame')
        target = targets[0]
        contours = np.concatenate([np.array(c) for c in target['contours']])
        low, high = contours.min(axis=0), contours.max(axis=0)
        z = target['depth_m']
        expected = fx*fy*(SIDE_M/z)**2
        locked_rows.append(dict(
            observed_ns=stamp, detection_depth_ns=d['depth_stamp_ns'],
            timestamp_roundoff_ns=d['depth_stamp_ns']-stamp, color_stamp_ns=d['color_stamp_ns'],
            u=target['center'][0], v=target['center'][1], area_px=target['area'],
            depth_z_m=z, valid_depth_points=target['valid_depth_points'],
            bbox_x=int(low[0]), bbox_y=int(low[1]), bbox_w=int(high[0]-low[0]+1),
            bbox_h=int(high[1]-low[1]+1), expected_face_area_px=expected,
            area_to_expected_ratio=target['area']/expected,
            equivalent_square_side_mm=1000*z*math.sqrt(target['area']/(fx*fy)),
            map_x=observation['map'][0], map_y=observation['map'][1]))
    write_csv(OUT / 'T88_lock_samples.csv', locked_rows)
    candidate_rows = []
    for d in detections:
        for index, target in enumerate(d['targets']):
            if not target['accepted']:
                continue
            z = target['depth_m']
            expected = fx*fy*(SIDE_M/z)**2
            candidate_rows.append(dict(
                received_ns=d['received_ns'], color_stamp_ns=d['color_stamp_ns'],
                depth_stamp_ns=d['depth_stamp_ns'], index=index, u=target['center'][0],
                v=target['center'][1], area_px=target['area'], depth_z_m=z,
                valid_depth_points=target['valid_depth_points'],
                expected_face_area_px=expected, area_to_expected_ratio=target['area']/expected,
                equivalent_square_side_mm=1000*z*math.sqrt(target['area']/(fx*fy))))
    write_csv(OUT / 'all_candidate_geometry.csv', candidate_rows)
    windows = {}
    for name, lo, hi in [('lock_vicinity', 1794043923, 1794043925),
                         ('T89_confirmation_vicinity', 1794043927, 1794043929),
                         ('T90_confirmation_vicinity', 1794043930, 1794043932),
                         ('late_visible_region', 1794043933, 1794043936)]:
        rows = [r for r in candidate_rows if lo <= r['received_ns']/1e9 < hi]
        windows[name] = dict(
            received_time_window_epoch_s=[lo, hi], accepted_candidates=len(rows),
            identification='Time vicinity only; not exact per-track observation assignment',
            ranges={key: [float(np.min([r[key] for r in rows])),
                          float(np.median([r[key] for r in rows])),
                          float(np.max([r[key] for r in rows]))]
                    for key in ['u', 'v', 'area_px', 'depth_z_m', 'area_to_expected_ratio',
                                'equivalent_square_side_mm']})
    intrinsics_checks = {}
    rows = []
    for folder in ['A', 'B2_selected', 'C_selected']:
        rows += list(csv.DictReader((ROOT / 'analysis/map_drift_20261108' /
                                     folder / 'measurements.csv').open()))
    for xyz, pixel in [('camera_x', 'u'), ('camera_y', 'v')]:
        matrix = np.array([[float(r[xyz])/float(r['camera_z']), 1] for r in rows if r[pixel]])
        values = np.array([float(r[pixel]) for r in rows if r[pixel]])
        fit = np.linalg.lstsq(matrix, values, rcond=None)[0]
        intrinsics_checks[pixel] = dict(f=float(fit[0]), c=float(fit[1]),
                                        max_residual_px=float(max(abs(matrix @ fit-values))))
    summary = dict(
        classification='INITIAL_TARGET_FALSE_POSITIVE',
        input_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in [RUN, LOG, LOG.with_suffix('.mp4'),
                                   ROOT / 'docs/calibration.md']},
        classification_basis=['User confirms real target absent from RGB initially',
                              'Five locked observations are upper-edge regions, not verified cubes',
                              'Reported size/depth conflicts with 10 mm cube projection'],
        user_supplied_cube_side_mm=10,
        intrinsics=dict(fx=fx, fy=fy, cx=cx, cy=cy, width=640, height=480,
                        native_depth_f=depth_f,
                        source='docs/calibration.md: recorded CameraInfo 2026-09-28',
                        cross_check='Nov-8 saved XYZ/pixels recover the same intrinsics',
                        nov7_full_camerainfo_archive='NOT AVAILABLE'),
        nov8_inverse_intrinsics_check=intrinsics_checks,
        theory='Frontoparallel square face; pinhole axial depth Z; no blur/bloom/distortion. '
        'Cube orientation changes silhouette; this is not an absolute mask-size bound.',
        projection=projection, locked_observations=locked_rows, windows=windows,
        multi_candidate_frames=[dict(received_ns=d['received_ns'],
                                     color_stamp_ns=d['color_stamp_ns'],
                                     targets=[dict(center=t['center'], area=t['area'],
                                                   depth_m=t['depth_m'])
                                              for t in d['targets']])
                                for d in detections
                                if sum(t['accepted'] for t in d['targets']) > 1],
        configured_thresholds={k: params[k] for k in [
            'min_component_area_px', 'morphology_kernel_px', 'morphology_open_iterations',
            'depth_erosion_kernel_px', 'depth_erosion_iterations', 'min_valid_depth_points',
            'min_depth_m', 'max_depth_m', 'cluster_depth_gap_m']},
        ideal_face_area_250_distance_limit_m=SIDE_M*math.sqrt(fx*fy/250),
        confirmed_reliable_10mm_distance='NOT VERIFIED at any distance in available records',
        initial_lock_event_ns=lock['ros_stamp_ns'],
        later_track_identity='T89/T90 not physically verified; windows are not exact track mapping')
    (OUT / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    render_evidence(events, detections, locked_rows)
    print('classification:', summary['classification'])
    print('T88 recovered locked samples:', len(locked_rows))
    print('ideal front-face 250-pixel distance:', summary['ideal_face_area_250_distance_limit_m'])


if __name__ == '__main__':
    main()
