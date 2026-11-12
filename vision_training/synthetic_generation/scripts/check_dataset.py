#!/usr/bin/env python3
"""Cross-check RGB, YOLO, visible instance masks, provenance and scene split."""
import argparse
import json
import hashlib
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw


def check(root, preview=None):
    manifest = json.loads((root/'manifest.json').read_text())
    scenes, counts, tiles = {}, [0,0,0], []
    vertex_cache = {}
    geometry_checks = 0
    records = manifest['records']
    assert records, 'Empty dataset'
    assert len({r['id'] for r in records}) == len(records), 'Duplicate IDs'
    for row in records:
        split, name = row['split'], row['id']
        assert split in ('train','val') and row['source_type'] == 'synthetic'
        assert scenes.setdefault(row['scene_id'], split) == split, 'Scene leakage'
        targets = list(row.get('targets', {}).values())
        for i,target in enumerate(targets):
            assert abs(target['floor_contact_error_m']) < 1e-6, 'Ground contact error'
            position = np.array(target['pose'])[:2,3]
            for other in targets[:i]:
                other_position = np.array(other['pose'])[:2,3]
                assert np.linalg.norm(position-other_position) + 1e-7 >= target['separation_radius_m'] + other['separation_radius_m'], 'Target overlap'
            source = target['source']
            if 'path' in source and Path(source['path']).exists():
                if source['path'] not in vertex_cache:
                    import trimesh
                    vertex_cache[source['path']] = np.asarray(trimesh.load_mesh(source['path'],process=False).vertices)
                vertices = vertex_cache[source['path']] - np.array(source['origin_translation_raw'])
                matrix = np.array(target['pose'])
                world = vertices @ matrix[:3,:3].T + matrix[:3,3]
                slope = np.deg2rad(row['slope_deg'])
                signed = world[:,0]*np.sin(slope) + world[:,2]*np.cos(slope)
                assert abs(float(signed.min())) < 2e-6, 'STL ground contact mismatch'
                geometry_checks += 1
        im = Image.open(root/'images'/split/(name+'.png')).convert('RGB')
        mask = np.load(root/'masks'/split/(name+'.npy'), allow_pickle=False)
        assert mask.shape == (im.height, im.width)
        lines = (root/'labels'/split/(name+'.txt')).read_text().splitlines()
        assert len(lines) == len(row['annotations']) and lines
        draw = ImageDraw.Draw(im)
        for line, ann in zip(lines, row['annotations']):
            values = line.split()
            assert len(values) == 5 and int(values[0]) == ann['class_id'] in (0,1,2)
            box = np.array([float(x) for x in values[1:]])
            assert np.isfinite(box).all() and ((box >= 0) & (box <= 1)).all()
            y, x = np.where(mask == ann['instance_id'])
            exact = [int(x.min()),int(y.min()),int(x.max())+1,int(y.max())+1]
            assert exact == ann['bbox_xyxy'] and len(x) == ann['visible_pixels']
            expected = [(exact[0]+exact[2])/2/im.width,(exact[1]+exact[3])/2/im.height,
                        (exact[2]-exact[0])/im.width,(exact[3]-exact[1])/im.height]
            assert np.allclose(box, expected, atol=1e-7)
            assert ann['visible_pixels'] >= manifest['config']['render']['min_pixels']
            assert ann['visible_fraction'] >= manifest['config']['render']['min_visible_fraction']
            draw.rectangle(exact, outline=['red','lime','cyan'][ann['class_id']], width=2)
            draw.text((exact[0],max(0,exact[1]-12)),str(ann['class_id']), fill='white')
        counts[row['primary_class']] += 1
        if counts[row['primary_class']] <= 4:
            im.thumbnail((320,240))
            draw = ImageDraw.Draw(im)
            draw.text((5,5), name, fill='white')
            tiles.append(im.copy())
    if preview:
        sheet = Image.new('RGB',(960,240*((len(tiles)+2)//3)), '#222222')
        for i,tile in enumerate(tiles):
            sheet.paste(tile,((i%3)*320,(i//3)*240))
        sheet.save(preview)
    digest = hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()
    previous = json.loads((root/'quality.json').read_text()) if (root/'quality.json').exists() else {}
    review = previous.get('visual_review', 'NOT VERIFIED') if previous.get('manifest_sha256') == digest else 'NOT VERIFIED'
    report = dict(manifest_sha256=digest, images=len(records), primary_class_counts=counts, scenes=len(scenes),
                  splits={s:sum(r['split']==s for r in records) for s in ('train','val')},
                  automatic_checks='PASS', stl_world_ground_checks=geometry_checks, visual_review=review)
    (root/'quality.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('dataset',type=Path)
    p.add_argument('--preview',type=Path)
    a=p.parse_args()
    check(a.dataset,a.preview)
