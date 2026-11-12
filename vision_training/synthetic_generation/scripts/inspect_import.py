import blenderproc as bproc
import argparse
import hashlib
import json
from pathlib import Path
import bpy
import numpy as np
import yaml

p = argparse.ArgumentParser()
p.add_argument('--config', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
c = yaml.safe_load(a.config.read_text())
assets = a.config.resolve().parents[1] / 'assets'
bproc.init()
rows = []
for name, filename in c['assets'].items():
    path = assets / filename
    objects = bproc.loader.load_obj(str(path))
    assert len(objects) == 1
    obj = objects[0]
    mesh = obj.blender_obj.data
    mesh.calc_loop_triangles()
    before = list(obj.blender_obj.dimensions)
    obj.set_scale([c[name + '_scale']]*3)
    bpy.context.view_layer.update()
    after = list(obj.blender_obj.dimensions)
    assert np.allclose(after, np.array(before)*c[name + '_scale'], atol=1e-7)
    rows.append(dict(class_name=name, file=filename,
                     sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                     blender_vertices=len(mesh.vertices), blender_triangles=len(mesh.loop_triangles),
                     dimensions_before=before, dimensions_after_m_assumption=after,
                     scale=c[name + '_scale'], physical_size_verified=False))
a.output.write_text(json.dumps(rows, indent=2, ensure_ascii=False)+'\n')
