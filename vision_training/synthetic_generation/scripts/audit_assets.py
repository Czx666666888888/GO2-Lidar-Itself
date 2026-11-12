#!/usr/bin/env python3
"""Read-only STL audit. No mesh repair, rescaling or source overwrite."""
import argparse
import hashlib
import json
import struct
from pathlib import Path

import numpy as np
import trimesh
import yaml


def audit(path, scale):
    raw = trimesh.load_mesh(path, process=False)
    mesh = raw.copy()
    mesh.merge_vertices()  # topology analysis only, not written back
    area = mesh.area_faces
    report = dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  stl_unit='unknown', raw_vertices=len(raw.vertices),
                  unique_vertices=len(mesh.vertices), triangles=len(mesh.faces),
                  bbox_raw=raw.bounds.tolist(), dimensions_raw=raw.extents.tolist(),
                  configured_scale=scale, bbox_import_m=(raw.bounds * scale).tolist(),
                  dimensions_import_m=(raw.extents * scale).tolist(),
                  physical_size_verified=False, watertight=bool(mesh.is_watertight),
                  winding_consistent=bool(mesh.is_winding_consistent),
                  zero_area_faces=int(np.sum(area == 0)),
                  near_zero_area_faces=int(np.sum(area <= 1e-12)),
                  near_zero_area_threshold_raw_squared=1e-12,
                  nonfinite_vertices=int(np.sum(~np.isfinite(raw.vertices).all(axis=1))),
                  duplicate_faces=int(len(mesh.faces) - len(np.unique(np.sort(mesh.faces, axis=1), axis=0))),
                  self_intersections='NOT VERIFIED')
    report['signed_volume_raw_cubed'] = float(mesh.volume)
    report['outward_winding_volume'] = bool(mesh.is_volume)
    blob = path.read_bytes()
    n = struct.unpack_from('<I', blob, 80)[0] if len(blob) >= 84 else 0
    if len(blob) == 84 + 50*n:
        records = np.frombuffer(blob, offset=84, count=n, dtype=np.dtype([
            ('normal', '<f4', 3), ('vertices', '<f4', (3,3)), ('attribute', '<u2')]))
        stored = records['normal'].astype(float)
        vertices = records['vertices'].astype(float)
        calculated = np.cross(vertices[:,1]-vertices[:,0], vertices[:,2]-vertices[:,0])
        norm_s = np.linalg.norm(stored,axis=1)
        norm_c = np.linalg.norm(calculated,axis=1)
        valid = (norm_s > 0) & (norm_c > 0) & np.isfinite(stored).all(axis=1)
        cosine = np.sum(stored[valid]*calculated[valid],axis=1)/(norm_s[valid]*norm_c[valid])
        report.update(stored_zero_normals=int(np.sum(norm_s==0)),
                      stored_nonfinite_normals=int(np.sum(~np.isfinite(stored).all(axis=1))),
                      stored_normals_opposite_winding=int(np.sum(cosine < 0)),
                      stored_normals_deviation_over_1deg=int(np.sum(cosine < np.cos(np.deg2rad(1)))))
    else:
        report['stored_normal_check'] = 'NOT VERIFIED: not binary STL'
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    c = yaml.safe_load(args.config.read_text())
    assets = args.config.resolve().parents[1] / 'assets'
    rows = [audit(assets / name, c[cls + '_scale']) for cls, name in c['assets'].items()]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(rows, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
