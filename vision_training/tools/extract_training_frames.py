#!/usr/bin/env python3
"""Read-only sequential video decode, conservative frame curation and CVAT image batches."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json  # noqa: E402
from vision_training.tools.dataset import add_pool, safe_group  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
NAMES = ['color_block', 'rock_compact', 'rock_spiky']
SOURCES = {'video_source_unverified', 'rendered_video', 'phone_real', 'd435_real'}


def fingerprint(gray):
    small = cv2.resize(gray, (160, 90), interpolation=cv2.INTER_AREA)
    dct = cv2.dct(cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32))[:8,:8]
    bits = dct > np.median(dct.ravel()[1:])
    bits[0,0] = False
    return small, bits.ravel()


def difference(gray, previous, settings):
    diff = cv2.absdiff(gray, previous)
    height, width = diff.shape
    # Local gates prevent a small changing object being hidden by the global average.
    padded = np.pad(diff, ((0, (-height)%32), (0, (-width)%32)), mode='constant')
    tiles = padded.reshape(padded.shape[0]//32, 32, padded.shape[1]//32, 32)
    local_mae = float(tiles.mean(axis=(1,3)).max())
    local_changed = float((tiles > settings['duplicate_pixel_delta']).mean(axis=(1,3)).max())
    mae = float(diff.mean())
    duplicate = (mae <= settings['duplicate_thumbnail_mae'] and
                 local_mae <= settings['duplicate_local_tile_mae'] and
                 local_changed <= settings['duplicate_local_changed_fraction'])
    return dict(mae=mae, max_tile_mae=local_mae, max_tile_changed_fraction=local_changed,
                similarity=1-mae/255, near_duplicate=duplicate)


def next_interval(motion, settings):
    if motion >= settings['fast_motion_mae']:
        return settings['min_interval_sec']
    if motion <= settings['slow_motion_mae']:
        return settings['max_interval_sec']
    return settings['base_interval_sec']


def sheet(root, rows, path, columns=4, tile_width=320):
    tile_height = int(tile_width*9/16)+38
    canvas = np.full((math.ceil(len(rows)/columns)*tile_height, columns*tile_width, 3), 28, np.uint8)
    for index, row in enumerate(rows):
        image = cv2.imread(str(root/row['image_file']))
        if image is None:
            raise ValueError(f"Unreadable preview source: {row['image_file']}")
        ratio = min(tile_width/image.shape[1], (tile_height-38)/image.shape[0])
        image = cv2.resize(image, (round(image.shape[1]*ratio), round(image.shape[0]*ratio)))
        x,y = (index%columns)*tile_width, (index//columns)*tile_height
        canvas[y:y+image.shape[0],x:x+image.shape[1]] = image
        cv2.putText(canvas, row['filename'], (x+3,y+tile_height-23), cv2.FONT_HERSHEY_SIMPLEX, .38, (240,240,240),1)
        cv2.putText(canvas, f"{row['source_seconds']:.2f}s L={row['sharpness']:.1f} gray={row['mean_gray']:.1f}",
                    (x+3,y+tile_height-7),cv2.FONT_HERSHEY_SIMPLEX,.36,(0,220,230),1)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), canvas, [cv2.IMWRITE_JPEG_QUALITY, 90]):
        raise ValueError(f'Cannot write sheet: {path}')


def representative_rows(rows, count=24):
    # Timeline spread plus low-light/low-sharpness cases; no automatic object labels.
    selected = {}
    for r in [rows[i] for i in np.linspace(0,len(rows)-1,max(1,count-8),dtype=int)]:
        selected[r['filename']] = r
    for key in ('mean_gray','sharpness'):
        for r in sorted(rows,key=lambda r:r[key])[:4]: selected[r['filename']] = r
    return list(selected.values())


def review_csv(path, rows):
    with path.open('w',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=['image','status','notes'])
        writer.writeheader()
        for row in rows: writer.writerow(dict(image=row['filename'],status='pending',notes=''))


def package(root, batch_size, package_dir=None, contact_dir=None):
    if batch_size < 1: raise ValueError('Positive batch size required')
    rows = json.loads((root/'manifests/images.json').read_text())
    if not rows: raise ValueError('No images to package')
    if len({r['filename'] for r in rows}) != len(rows): raise ValueError('Filename collision')
    if len({r['image_sha256'] for r in rows}) != len(rows): raise ValueError('Repeated image content')
    output = package_dir or root/'cvat_packages'
    contacts = contact_dir or root/'contact_sheets'
    output.mkdir(parents=True,exist_ok=False)
    contacts.mkdir(parents=True,exist_ok=False)
    parts = math.ceil(len(rows)/batch_size)
    batches = [[] for _ in range(parts)]
    # Each batch spans each source timeline, rather than putting adjacent frames in the first batch.
    offset = 0
    for source in dict.fromkeys(r['source_video'] for r in rows):
        source_rows = [r for r in rows if r['source_video']==source]
        for i,row in enumerate(source_rows): batches[(offset+i)%parts].append(row)
        offset += len(source_rows)
    manifest = []
    all_names,all_hashes = set(),set()
    for i, batch in enumerate(batches,1):
        name = f'batch_{i:03d}'
        archive = output/(name+'.zip')
        assert len(batch)<=batch_size
        with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_STORED) as z:
            for row in batch:
                image = root/row['image_file']
                if sha256(image)!=row['image_sha256']: raise ValueError('Image hash changed')
                z.write(image,arcname=row['filename'])
        with zipfile.ZipFile(archive) as z:
            if z.testzip() is not None or len(z.namelist())!=len(batch): raise ValueError('ZIP integrity failure')
            for row in batch:
                data = z.read(row['filename'])
                digest = hashlib.sha256(data).hexdigest()
                decoded = cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
                if decoded is None or list(decoded.shape[1::-1])!=row['resolution']: raise ValueError('ZIP image dimensions/decode mismatch')
                if digest!=row['image_sha256'] or digest in all_hashes or row['filename'] in all_names:
                    raise ValueError('Duplicate or incorrect ZIP image')
                all_hashes.add(digest); all_names.add(row['filename'])
        review_csv(output/(name+'_review.csv'),batch)
        sheet(root,representative_rows(batch),contacts/(name+'_representatives.jpg'))
        for start in range(0,len(batch),64):
            sheet(root,batch[start:start+64],contacts/f'{name}_page_{start//64+1:02d}.jpg',columns=8,tile_width=192)
        manifest.append(dict(batch=name,archive=str(archive.relative_to(root)) if archive.is_relative_to(root) else str(archive),
            images=len(batch),sha256=sha256(archive),integrity='PASS',
            source_counts=dict(Counter(r['video_id'] for r in batch)),
            records=[{k:r[k] for k in ('filename','image_file','image_sha256','source_video','frame_index')} for r in batch]))
    assert len(all_names)==len(rows)
    write_json(output/'manifest.json',dict(batch_size_preference=batch_size,images=len(rows),batches=manifest,
        account_limit_verified=False,note='Configure for actual CVAT account limits; no upload performed'))
    return manifest


def extract(config_path, output, mode='coverage', input_dir=None):
    config = yaml.safe_load(config_path.read_text())
    settings = config['sampling']
    if not 0 < settings['min_interval_sec'] <= settings['base_interval_sec'] <= settings['max_interval_sec']:
        raise ValueError('Invalid sampling interval order')
    if not 1<=settings['jpeg_quality']<=100: raise ValueError('Invalid JPEG quality')
    source_type = config.get('source_type','video_source_unverified')
    if source_type not in SOURCES: raise ValueError('Unknown source type')
    if source_type!='video_source_unverified' and not config.get('source_confirmation'):
        raise ValueError('Confirmed source type requires explicit source confirmation')
    incoming = Path(input_dir or config['input_dir'])
    if not incoming.is_absolute(): incoming = ROOT/incoming
    videos = config['videos']
    if len({v['id'] for v in videos})!=len(videos): raise ValueError('Duplicate video IDs')
    for video in videos:
        safe_group(video['id']);safe_group(video['capture_group'])
        if not (incoming/video['filename']).is_file(): raise FileNotFoundError(incoming/video['filename'])
    output.mkdir(parents=True,exist_ok=False)
    for folder in ('images','manifests','reports','pool/images'): (output/folder).mkdir(parents=True)
    cv2.setNumThreads(1)
    rows,summaries = [],[]
    encoded_hashes = set()
    for spec in videos:
        path = (incoming/spec['filename']).resolve()
        source_hash = sha256(path)
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened(): raise ValueError(f'Cannot decode video: {path}')
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        declared = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if fps<=0 or not np.isfinite(fps) or declared<=0: raise ValueError('Invalid video metadata')
        folder = output/'images'/spec['id'];folder.mkdir()
        # Standard pool compatibility for import_cvat.py, with just one full-size image copy.
        group_folder = output/'pool/images'/spec['capture_group']
        group_folder.mkdir(exist_ok=True)
        kept = []
        prev_thumb,last_gray,last_row = None,None,None
        decoded_hashes = {}
        motion = 0.; last_candidate = -1e9; previous_pts = -1.; index=0
        counts=Counter(); resolution=None
        timeline = output/'manifests'/(spec['id']+'_timeline.jsonl')
        with timeline.open('w') as stream:
            while True:
                ok,frame = cap.read()
                if not ok: break
                if resolution is None: resolution=list(frame.shape[1::-1])
                if list(frame.shape[1::-1])!=resolution: raise ValueError('Resolution changes within video')
                pts = float(cap.get(cv2.CAP_PROP_POS_MSEC))/1000
                if np.isfinite(pts) and pts>=0 and (index==0 or pts>previous_pts):
                    seconds,time_source = pts,'opencv_pos_msec'
                    previous_pts=pts
                else:
                    seconds,time_source = index/fps,'frame_index_over_fps'
                gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                small = cv2.resize(gray,(160,90),interpolation=cv2.INTER_AREA)
                delta = float(cv2.absdiff(small,prev_thumb).mean()) if prev_thumb is not None else 0.
                motion = .8*motion+.2*delta;prev_thumb=small
                candidate = mode=='coverage' and (seconds-last_candidate>=next_interval(motion,settings)-1e-6 or index==declared-1)
                entry=dict(frame_index=index,source_seconds=seconds,nominal_seconds=index/fps,time_source=time_source,
                           resolution=resolution,candidate=bool(candidate),action='timeline_only' if mode=='timeline' else 'interval_not_due')
                if candidate:
                    last_candidate=seconds;counts['candidates']+=1
                    sharpness=float(cv2.Laplacian(gray,cv2.CV_64F).var()); brightness=float(gray.mean())
                    small,phash=fingerprint(gray)
                    digest=hashlib.sha256(frame.tobytes()).hexdigest()
                    comparison=difference(gray,last_gray,settings) if last_gray is not None else None
                    reason='first_frame' if last_gray is None else 'visual_change'
                    reference=None; exclusion=None
                    if digest in decoded_hashes:
                        exclusion='exact_decoded_duplicate';reference=decoded_hashes[digest]
                    elif int(gray.max())-int(gray.min())<3:
                        exclusion='uniform_noninformative'
                    elif comparison and comparison['near_duplicate'] and sharpness<=last_row['sharpness']*1.2:
                        if seconds-last_row['source_seconds']<settings['max_near_duplicate_gap_sec']:
                            exclusion='recent_near_duplicate';reference=last_row['filename']
                        else: reason='temporal_coverage_soft_duplicate'
                    elif comparison and comparison['near_duplicate']:
                        reason='sharper_variant'
                    # A hash match alone never removes a small-object or low-contrast change.
                    if not exclusion and kept:
                        for old in kept:
                            if seconds-old['source_seconds']<settings['long_duplicate_min_gap_sec']: continue
                            if np.count_nonzero(phash!=old['_phash'])>settings['phash_hamming']:continue
                            if float(cv2.absdiff(small,old['_small']).mean())>settings['duplicate_thumbnail_mae']:continue
                            old_gray=cv2.imread(str(output/old['image_file']),cv2.IMREAD_GRAYSCALE)
                            detail=difference(gray,old_gray,settings)
                            if detail['near_duplicate'] and sharpness<=old['sharpness']*1.2:
                                exclusion='long_near_duplicate';reference=old['filename'];break
                    entry.update(sharpness=sharpness,mean_gray=brightness,similarity_to_previous_kept=None if comparison is None else comparison['similarity'],
                                 previous_kept=None if last_row is None else last_row['filename'])
                    if not exclusion:
                        filename=f"{spec['id']}_f{index:07d}.jpg"
                        ok,data=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,settings['jpeg_quality']])
                        if not ok:raise ValueError('JPEG encoding failed')
                        image_hash=hashlib.sha256(data.tobytes()).hexdigest()
                        if image_hash in encoded_hashes:
                            exclusion='exact_encoded_duplicate'
                        else:
                            target=folder/filename
                            target.write_bytes(data.tobytes());encoded_hashes.add(image_hash)
                            (group_folder/filename).symlink_to(Path('../../../images')/spec['id']/filename)
                            flags=[]
                            if brightness<45:flags.append('low_light_metric')
                            if sharpness<25:flags.append('low_sharpness_metric')
                            row=dict(filename=filename,image_file=str(target.relative_to(output)),
                                image=str(Path('pool/images')/spec['capture_group']/filename),video_id=spec['id'],
                                source=str(path),source_video=path.name,source_video_sha256=source_hash,
                                frame_index=index,source_seconds=seconds,time_source=time_source,resolution=resolution,
                                image_sha256=image_hash,sharpness=sharpness,mean_gray=brightness,
                                similarity_to_previous_kept=None if comparison is None else comparison['similarity'],
                                previous_kept=None if last_row is None else last_row['filename'],selection_reason=reason,
                                quality_flags=flags,source_type=source_type,source_confirmation=config.get('source_confirmation'),
                                capture_group=spec['capture_group'],group=spec['capture_group'],reviewed=False,
                                tags=[source_type,spec['capture_group']],_phash=phash,_small=small)
                            rows.append(row);kept.append(row);last_row=row
                            last_gray=gray.copy()
                            decoded_hashes[digest]=filename
                            counts['kept']+=1
                            entry.update(action='kept',filename=filename,reason=reason)
                    if exclusion:
                        counts[exclusion]+=1;entry.update(action='excluded',reason=exclusion,reference=reference)
                counts[entry['action']]+=int(entry['action']!='kept')
                stream.write(json.dumps(entry)+'\n')
                index+=1
        cap.release()
        if index!=declared: raise ValueError(f'Incomplete decode {path.name}: {index}/{declared}; output not accepted')
        if sha256(path)!=source_hash: raise ValueError('Original video changed during extraction')
        summary=dict(video_id=spec['id'],source_video=path.name,source_video_path=str(path),source_video_sha256=source_hash,
            source_type=source_type,capture_group=spec['capture_group'],fps=fps,resolution=resolution,
            declared_frames=declared,decoded_frames=index,duration_nominal_sec=declared/fps,counts=dict(counts),complete_decode=True,
            timeline=str(timeline.relative_to(output)))
        summaries.append(summary)
        if kept:sheet(output,representative_rows(kept,32),output/'reports'/(spec['id']+'_overview.jpg'))
        for row in kept: row.pop('_phash');row.pop('_small')
        print(json.dumps(summary),flush=True)
    write_json(output/'manifests/images.json',rows)
    write_json(output/'manifests/videos.json',summaries)
    (output/'data.yaml').write_text(yaml.safe_dump(dict(path='.',names=NAMES,nc=3,status='unlabelled_candidate_pool_no_split'),sort_keys=False))
    add_pool(output,rows)
    batches=package(output,config['batch_size']) if mode=='coverage' else []
    report=dict(mode=mode,source_type=source_type,source_confirmation=config.get('source_confirmation'),
        videos=summaries,images=len(rows),zip_count=len(batches),batch_counts=[b['images'] for b in batches],
        labels_created=0,reviewed_images=0,split_created=False,
        config=config,config_sha256=sha256(config_path),script_sha256=sha256(Path(__file__)),opencv=cv2.__version__,numpy=np.__version__)
    write_json(output/'reports/summary.json',report)
    (output/'COMPLETE').write_text(str(len(rows))+'\n')
    return report



def verify(root):
    if not (root/'COMPLETE').exists(): raise ValueError('Incomplete extraction')
    rows = json.loads((root/'manifests/images.json').read_text())
    videos = json.loads((root/'manifests/videos.json').read_text())
    summary = json.loads((root/'reports/summary.json').read_text())
    if len({r['filename'] for r in rows})!=len(rows) or len({r['image_sha256'] for r in rows})!=len(rows):
        raise ValueError('Filename/content collision')
    source_checks = 0
    for video in videos:
        path = Path(video['source_video_path'])
        if sha256(path)!=video['source_video_sha256']:raise ValueError('Source video hash mismatch')
        selected = {r['frame_index']:r for r in rows if r['video_id']==video['video_id']}
        timeline = [json.loads(line) for line in (root/video['timeline']).read_text().splitlines()]
        if [r['frame_index'] for r in timeline]!=list(range(video['decoded_frames'])):
            raise ValueError('Incomplete/non-contiguous original timeline')
        cap = cv2.VideoCapture(str(path));index=0;matched=0
        while True:
            ok, frame = cap.read()
            if not ok:break
            if index>=len(timeline):raise ValueError('Video gained frames')
            if index in selected:
                row = selected[index]
                if row['source_video_sha256']!=video['source_video_sha256']:raise ValueError('Row/video hash mismatch')
                if row['source_video']!=path.name or row['filename']!=f"{video['video_id']}_f{index:07d}.jpg":
                    raise ValueError('Source frame/name mismatch')
                if row['source_seconds']!=timeline[index]['source_seconds'] or timeline[index]['action']!='kept':
                    raise ValueError('Timeline/image mismatch')
                if row['resolution']!=list(frame.shape[1::-1]):raise ValueError('Original dimensions changed')
                ok, encoded = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,summary['config']['sampling']['jpeg_quality']])
                if not ok or hashlib.sha256(encoded.tobytes()).hexdigest()!=row['image_sha256']:
                    raise ValueError('Independent source re-decode/frame-image content mismatch')
                original = root/row['image_file']
                if sha256(original)!=row['image_sha256'] or sha256(root/row['image'])!=row['image_sha256']:
                    raise ValueError('Local image/pool hash mismatch')
                readback = cv2.imread(str(original))
                if readback is None or list(readback.shape[1::-1])!=row['resolution']:raise ValueError('Unreadable output image')
                matched+=1;source_checks+=1
            index+=1
        cap.release()
        if index!=video['decoded_frames'] or matched!=len(selected):raise ValueError('Independent decode incomplete')
    expected = {r['filename']:r['image_sha256'] for r in rows}
    seen = {}
    if rows:
        batches = json.loads((root/'cvat_packages/manifest.json').read_text())['batches']
        for batch in batches:
            path = root/batch['archive']
            if sha256(path)!=batch['sha256']:raise ValueError('ZIP hash mismatch')
            with zipfile.ZipFile(path) as z:
                if z.testzip() is not None or len(z.namelist())!=batch['images']:raise ValueError('ZIP CRC/count failure')
                for name in z.namelist():
                    if Path(name).name!=name or Path(name).suffix!='.jpg' or name in seen:raise ValueError('Duplicate or non-image ZIP entry')
                    digest=hashlib.sha256(z.read(name)).hexdigest()
                    if digest!=expected.get(name):raise ValueError('ZIP/manifest mismatch')
                    seen[name]=digest
        if seen!=expected:raise ValueError('Missing packaged images')
    if list(root.rglob('*.txt')):raise ValueError('Unexpected label text in unlabelled extraction')
    report=dict(status='PASS',images=len(rows),source_frame_content_checks=source_checks,
        original_decoded_frames=sum(v['decoded_frames'] for v in videos),
        timeline_contiguous=True,original_resolution_preserved=True,images_readable=True,
        stable_unique_names=True,cross_zip_duplicates=0,zip_images_match_manifest=True,
        labels_created=0,reviewed_images=0,split_created=False)
    write_json(root/'reports/verification.json',report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    ext=sub.add_parser('extract')
    ext.add_argument('--config',type=Path,required=True)
    ext.add_argument('--output',type=Path,required=True)
    ext.add_argument('--input-dir',type=Path)
    ext.add_argument('--mode',choices=['coverage','timeline'],default='coverage')
    pack=sub.add_parser('package')
    pack.add_argument('--dataset',type=Path,required=True)
    pack.add_argument('--batch-size',type=int,required=True)
    pack.add_argument('--output',type=Path,required=True)
    ver=sub.add_parser('verify')
    ver.add_argument('--dataset',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='extract':
        print(json.dumps(extract(args.config,args.output,args.mode,args.input_dir),indent=2))
    elif args.command=='verify':
        print(json.dumps(verify(args.dataset),indent=2))
    else:
        if not (args.dataset/'COMPLETE').exists():raise ValueError('Incomplete source dataset')
        print(json.dumps(package(args.dataset,args.batch_size,args.output,args.output/'contact_sheets'),indent=2))


if __name__=='__main__': main()
