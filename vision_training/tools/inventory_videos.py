#!/usr/bin/env python3
"""Inventory raw recordings without inferring scene labels or ground truth."""

import argparse
from pathlib import Path
import sys

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import write_json  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--preview-dir')
    args = parser.parse_args()
    videos = sorted(Path(args.records).resolve().glob('*.mp4'))
    rows = []
    for path in videos:
        cap = cv2.VideoCapture(str(path))
        fps = cap.get(cv2.CAP_PROP_FPS)
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        raw = path.stem.endswith('_raw')
        row = dict(file=path.name, raw=raw, opened=cap.isOpened(), frames=count, fps=fps,
                   seconds=count/fps if fps > 0 else None,
                   width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                   height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), scene_tags='UNREVIEWED')
        if args.preview_dir and raw:
            folder = Path(args.preview_dir)
            folder.mkdir(parents=True, exist_ok=True)
            for fraction in (.1, .5, .9):
                cap.set(cv2.CAP_PROP_POS_FRAMES, int(count*fraction))
                ok, frame = cap.read()
                if ok:
                    cv2.imwrite(str(folder / f'{path.stem}_{fraction}.jpg'), frame)
        cap.release()
        rows.append(row)
    write_json(args.output, dict(videos=rows, raw_videos=sum(r['raw'] for r in rows),
                                 raw_seconds=sum(r['seconds'] or 0 for r in rows if r['raw']),
                                 labels='No annotation or scene tag inferred from detections'))
    print(f'{len(rows)} videos, {sum(r["raw"] for r in rows)} raw; metadata saved')


if __name__ == '__main__':
    main()
