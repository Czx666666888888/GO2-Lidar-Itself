#!/usr/bin/env python3
"""Audit saved evidence only; absent telemetry is unknown, never zero failures."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re


def audit(folder):
    """Count observations, attempts and source frames separately."""
    folder = Path(folder)
    rows = [json.loads(line) for path in sorted(folder.glob('approach_*.jsonl'))
            for line in path.read_text().splitlines() if line.strip()]
    grouped = {}
    for row in rows:
        grouped.setdefault(row['kind'], []).append(row)
    detections = grouped.get('detection', [])
    selections = [r['selection'] for r in grouped.get('selection', [])]
    reasons = Counter(s['reason'] for s in selections)
    reason_stamps = {}
    for s in selections:
        reason_stamps.setdefault(s['reason'], set()).add(s['stamp_ns'])
    checks = [c for s in selections for c in s.get('association_checks', [])]
    controls = [r['control'] for r in grouped.get('control', [])]
    slam = '\n'.join(p.read_text() for p in folder.glob('slam*.log'))
    counts = re.findall(r'FINAL_MAP_TRANSFORM input=(\d+) transformed=(\d+) failed=(\d+)',
                        slam)
    final = tuple(map(int, counts[-1])) if counts else None
    map_rows = grouped.get('map_measurement', [])
    source_maps = {t['observed_ns'] for c in controls for t in c.get('search_observations', [])}
    valid_stamps = [r['detection']['color_stamp_ns'] for r in detections
                    if any(t.get('accepted') for t in r['detection'].get('targets', []))]
    last_map = max(source_maps) if source_maps else None
    return dict(
        run=folder.name, frames=len(grouped.get('frame', [])),
        detection_reports=len(detections), valid_depth_frames=len(valid_stamps),
        boxes_per_frame=dict(Counter(len(r['detection'].get('targets', []))
                                     for r in detections)),
        depth_reasons=dict(Counter(t.get('depth', {}).get('reason', 'unknown')
                                   for r in detections
                                   for t in r['detection'].get('targets', []))),
        adapter_final=(dict(input=final[0], transformed=final[1], failed=final[2])
                       if final else None),
        selection_attempts=len(selections) if selections else None,
        selection_reasons=dict(reasons) if selections else None,
        selection_unique_frames_by_reason={k: len(v) for k, v in reason_stamps.items()}
        if selections else None,
        initial_gate_rejections=sum(c['initial_rejected'] for c in checks) if checks else None,
        update_gate_rejections=sum(c['update_rejected'] for c in checks) if checks else None,
        exact_tf_failed_attempts=reasons['waiting_for_exact_source_tf'] if selections else None,
        map_measurement_messages=len(map_rows) if map_rows else None,
        manager_new_track_ids=re.findall(r'NEW_TRACK id=(\d+)', slam),
        manager_confirmed_log_lines=[line for line in slam.splitlines() if
                                     re.search(r'\bCONFIRMED\b', line)],
        search_unique_map_observations=len(source_maps),
        valid_depth_frames_after_last_search_map=(sum(s > last_map for s in valid_stamps)
                                                  if last_map is not None else None),
        candidate_control_reports=sum(c.get('candidate') is not None for c in controls),
        control_states=dict(Counter(c['state'] for c in controls)),
        terminal_reason=controls[-1].get('reason') if controls else None,
        missing_evidence=[k for k in ('yolo', 'selection', 'map_measurement', 'tracks')
                          if not grouped.get(k)],
        note='null means not recorded or not derivable; failed=0 excludes adapter preflight gates')


def main():
    """Print a reproducible report without connecting ROS or hardware."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.run), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
