#!/usr/bin/env python3
"""Summarize timestamped video predictions separately from unreviewed detection truth."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from vision_training.common import write_json

BASE=Path(__file__).resolve().parents[1]
RUN=BASE/'runs/science_targets_mixed_v3'
REPORT=BASE/'reports/science_targets_mixed_v3'


def main():
    versions={}
    for version in ('v1','v2','v3'):
        root=RUN/f'go2_prediction_{version}'
        summary=json.loads((root/'summary.json').read_text())
        videos=summary['videos']
        versions[version]=dict(weights_sha256=summary['weights_sha256'],videos=videos,
            frames=sum(v['frames'] for v in videos),
            frames_with_color_prediction_conf025=sum(v['color_prediction_present_conf025'] for v in videos))
    assert all(v['frames']==3117 for v in versions.values())
    for video in ('r3_go2_07','r3_go2_08'):
        fig, axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
        stamps=None
        for version in versions:
            rows=[json.loads(s) for s in (RUN/f'go2_prediction_{version}/{video}.jsonl').read_text().splitlines()]
            current=[r['rgb_stamp_ns'] for r in rows]
            if stamps is None:stamps=current
            assert current==stamps
            time=(np.array(current,dtype=np.int64)-current[0])/1e9
            confidence=[r['max_color_conf_at_conf010'] if r['max_color_conf_at_conf010'] is not None else np.nan for r in rows]
            axes[0].plot(time,confidence,label=version,linewidth=1)
        axes[0].axhline(.25,color='gray',linestyle='--',label='presence threshold .25')
        axes[0].set_ylabel('Returned max color confidence');axes[0].set_ylim(0,1);axes[0].legend(ncol=4)
        axes[1].plot(time,[r['blur_metric'] for r in rows],color='black');axes[1].set_ylabel('Full-frame Laplacian variance')
        speed=[r['control_log'].get('actual_speed') for r in rows]
        axes[2].plot(time,[x if isinstance(x,(int,float)) else np.nan for x in speed],color='purple')
        axes[2].set_ylabel('Logged actual_speed (m/s)');axes[2].set_xlabel('Original RGB timestamp elapsed (seconds)')
        fig.suptitle(f'{video}: prediction diagnostics only; human per-frame GT pending; same training capture group')
        fig.tight_layout();fig.savefig(REPORT/f'{video}_confidence_timeline.png',dpi=140);plt.close(fig)
    write_json(REPORT/'go2_prediction_comparison.json',dict(results=versions,
        threshold=.25,confidence_saved=.10,
        interpretation='Class0 output presence is NOT identity/IoU-confirmed blue-cube detection. Absence is NOT GT-confirmed miss. Null confidence means no returned box >=.10, not true score0.',
        time_protocol='All versions use identical original RGB stamps. Gap duration is last absent stamp minus first absent stamp. Next-output latency is not GT-confirmed recovery speed.',
        gt_status='NOT VERIFIED: user chose later manual labeling of 90 consecutive frames. All historical clips share laboratory training group.',
        review_package='vision_training/annotations/go2_continuity_v3_review/go2_continuity_90_images.zip'))
    for version,v in versions.items():
        print(version,v['frames_with_color_prediction_conf025'],'/',v['frames'])


if __name__=='__main__':main()
