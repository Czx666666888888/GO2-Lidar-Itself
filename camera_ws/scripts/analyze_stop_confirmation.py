#!/usr/bin/env python3
"""Summarize a bounded selection of recorded runs; no ROS, replay, or hardware I/O."""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
from statistics import mean, median


RUNS = (
    'full_1794746510438141133', 'full_1794746842611482601',
    'full_1794747100355265452', 'full_1794747532450478669',
    'full_1794747675021281105', 'full_1794819424367985274',
    'full_1794819797246482768', 'full_1794820061800141575',
)


def stopped(row):
    s = row.get('stop_inputs', {})
    return (all(s.get(k, math.inf) <= .05 for k in ('body_speed', 'camera_speed', 'sport_speed'))
            and all(abs(s.get(k, math.inf)) <= .10 for k in ('pose_yaw_rate', 'sport_yaw_rate'))
            and s.get('odom_age', math.inf) <= .4 and s.get('camera_age', math.inf) <= .4
            and s.get('sport_age', math.inf) <= .5)


def summarize(records):
    entries, reasons, waits, savings = [], Counter(), Counter(), []
    for name in RUNS:
        folder = records/name
        for f in sorted(folder.glob('run_*.json')):
            raw = f.read_bytes()
            run = json.loads(raw)
            final = run.get('final', {})
            reports = run.get('control_reports', [])
            if not final.get('stop_confirm_attempts'):
                continue
            e = dict(path=str(f.relative_to(records)), sha256=hashlib.sha256(raw).hexdigest(),
                     run_start_local=datetime.fromtimestamp(int(name[5:])/1e9).isoformat(),
                     detector=json.loads((folder/'manifest.json').read_text()).get('detector'),
                     commit=json.loads((folder/'manifest.json').read_text()).get('commit'),
                     final_state=final.get('state'), terminal_reason=final.get('reason'),
                     first_seen_to_lock_sec=final.get('confirmation_seconds'),
                     attempts=final['stop_confirm_attempts'])
            for a in e['attempts']:
                if a['result'] == 'TIMEOUT':
                    reasons[a.get('reason', 'unknown')] += 1
            for row in reports:
                if row.get('state') == 'STOP_CONFIRM':
                    waits[row.get('stop_confirm_reason', 'unknown')] += 1
            # Constrained timing comparison: use ONLY stationary observations
            # already accepted by the old controller; no fabricated missing RGB-D.
            lock_time = final.get('confirmed_at')
            if lock_time is not None:
                for row in reports:
                    samples = row.get('stationary_observations', [])
                    seed = row.get('stop_candidate') or {}
                    if (row.get('state') != 'STOP_CONFIRM' or len(samples) < 3 or
                            row.get('stop_confirm_started') != final.get('stop_confirm_started') or
                            row.get('settled_at') is None or not stopped(row) or
                            not seed or any(x['id'] != seed['id'] for x in samples)):
                        continue
                    xy = [x['map'] for x in samples]
                    extent = max(math.dist(a, b) for a in xy for b in xy)
                    if extent >= .08 or any(math.dist(x, seed['map']) > .15 for x in xy):
                        continue
                    # Require three distinct increasing source stamps.
                    stamps = [x['observed_ns'] for x in samples]
                    if stamps != sorted(set(stamps)):
                        continue
                    saving = lock_time-row['control_monotonic']
                    if saving >= 0:
                        e['accepted_sample_timing_saving_sec'] = saving
                        savings.append(saving)
                        break
            entries.append(e)
    durations = [e['first_seen_to_lock_sec'] for e in entries
                 if e['first_seen_to_lock_sec'] is not None]
    successes = [a for e in entries for a in e['attempts'] if a['result'] == 'CONFIRMED']
    return dict(successful_attempt_mean_sec=mean(a['elapsed'] for a in successes),
                successful_attempt_stop_mean_sec=mean(a['stopped_after_sec'] for a in successes),
                successful_attempt_post_stop_mean_sec=mean(
                    a['stationary_confirmation_sec'] for a in successes),
                scope='8 fixed full folders; all run JSONs within them; no hardware execution',
                interpretation='first_seen is first eligible detection, not necessarily the final '
                               'physical target; LOCK success is not grasp proof',
                runs=entries, successful_lock_count=len(durations),
                failed_without_lock_count=len(entries)-len(durations),
                first_seen_to_lock_mean_sec=mean(durations),
                completed_attempt_count=sum(len(e['attempts']) for e in entries),
                timeout_reason_counts=dict(reasons), stop_confirm_cycle_reasons=dict(waits),
                accepted_sample_comparison_count=len(savings),
                accepted_sample_timing_saving_mean_sec=mean(savings) if savings else None,
                accepted_sample_timing_saving_median_sec=median(savings) if savings else None,
                comparison_limit='counterfactual sampling-time check, not full replay; raw Manager '
                                 'track snapshots and pre-stop evidence are incomplete; '
                                 'no new success-rate or end-to-end saving claim')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--records', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = summarize(args.records)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'runs'},
                     ensure_ascii=False, indent=2))
    for r in result['runs']:
        print(r['path'], r['first_seen_to_lock_sec'], r.get('accepted_sample_timing_saving_sec'))


if __name__ == '__main__':
    main()
