#!/usr/bin/env python3
"""Explicit synthetic import, preserving group/split and leaving human review pending."""
import argparse
import json
from pathlib import Path
import shutil
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from vision_training.common import read_yaml, class_names, sha256
from vision_training.tools.dataset import add_pool, pool_manifest
from check_dataset import check


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--dataset',type=Path,required=True)
    a=p.parse_args()
    if not (a.source/'COMPLETE').is_file():
        raise ValueError('Incomplete generation')
    check(a.source)
    manifest=json.loads((a.source/'manifest.json').read_text())
    names=['color_block','rock_compact','rock_spiky']
    if class_names(read_yaml(a.dataset/'data.yaml')['names']) != class_names(names):
        raise ValueError('Class mismatch')
    batch=sha256(a.source/'manifest.json')[:12]
    copies, rows=[],[]
    for r in manifest['records']:
        group=f"synthetic_{batch}_{r['scene_id']}"
        image=Path('pool/images')/group/(r['id']+'.png')
        label=Path('pool/labels')/group/(r['id']+'.txt')
        for sub,target in [('images',image),('labels',label)]:
            if (a.dataset/target).exists():
                raise ValueError(f'Already imported: {target}')
            copies.append((a.source/sub/r['split']/(r['id']+('.png' if sub=='images' else '.txt')),a.dataset/target))
        rows.append(dict(image=str(image),group=group,source_type='synthetic',
                         source=str(a.source.resolve()),source_manifest_sha256=sha256(a.source/'manifest.json'),
                         synthetic_split=r['split'],scene_id=r['scene_id'],tags=['synthetic'],
                         reviewed=False, annotation_source='blenderproc_instance_mask',
                         synthetic_metadata=r))
    if {r['image'] for r in rows} & {r['image'] for r in pool_manifest(a.dataset)}:
        raise ValueError('Duplicate import')
    for src,dst in copies:
        dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,dst)
    add_pool(a.dataset,rows)
    print(f'{len(rows)} synthetic records imported, UNREVIEWED; preserve synthetic_split; never use as final test')


if __name__=='__main__':
    main()
