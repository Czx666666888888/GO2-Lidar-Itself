import blenderproc as bproc
import argparse
import json
from pathlib import Path
import bpy
import numpy as np
from PIL import Image
import yaml
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from generate import material

p=argparse.ArgumentParser()
p.add_argument('--config',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
c=yaml.safe_load(a.config.read_text())
a.output.mkdir(parents=True,exist_ok=True)
bproc.init()
bproc.renderer.set_render_devices(use_only_cpu=True)
bproc.renderer.set_max_amount_of_samples(16)
bproc.camera.set_resolution(960,240)
base=bproc.object.create_primitive('CUBE',scale=[c['block']['size_m']/2]*3)
bpy.context.view_layer.objects.active=base.blender_obj
bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
mod=base.blender_obj.modifiers.new('bevel_m','BEVEL')
mod.width,mod.segments=c['block']['bevel_m'],3
base.set_name('color_block_50mm_base')
base.set_cp('category_id',1)
base.set_cp('yolo_class_id',0)
rows=[]
objects=[]
for i,(name,rgba) in enumerate(c['block']['colors'].items()):
    mat=material(name,rgba or [0.18]*3+[1],c['block']['roughness'],name=='natural_wood')
    mat.blender_obj['confirmed']=rgba is not None
    mat.blender_obj.use_fake_user=True
    ob=base.blender_obj if i==0 else base.blender_obj.copy()
    if i:
        bpy.context.collection.objects.link(ob)
    if not ob.data.materials:
        ob.data.materials.append(mat.blender_obj)
    ob.material_slots[0].link='OBJECT'
    ob.material_slots[0].material=mat.blender_obj
    ob.name='block_'+name
    ob.location=[(i-2)*0.075,0,0.025]
    ob.hide_render=rgba is None
    ob['color_status']='confirmed' if rgba else 'pending_neutral_placeholder_not_sampled'
    bpy.context.view_layer.update()
    dims=list(ob.dimensions)
    assert np.allclose(dims,[0.05]*3,atol=1e-7),dims
    rows.append(dict(color=name,dimensions_m=dims,confirmed=rgba is not None,material=mat.blender_obj.name))
    objects.append(ob)
assert len({o.data.name for o in objects})==1,'Must share one base geometry'
bpy.ops.wm.save_as_mainfile(filepath=str((a.output/'color_block.blend').resolve()))
bproc.camera.add_camera_pose(bproc.math.build_transformation_mat([0,-0.6,0.5],bproc.camera.rotation_from_forward_vec([0,0.6,-0.475])))
lamp=bproc.types.Light()
lamp.set_type('AREA')
lamp.set_location([0,-0.15,0.4])
lamp.set_energy(3)
lamp.blender_obj.data.size=0.3
data=bproc.renderer.render()
Image.fromarray(data['colors'][0].astype(np.uint8)).save(a.output/'materials.png')
(a.output/'block_audit.json').write_text(json.dumps(dict(base_meshes=1,bevel_m=mod.width,materials=rows),indent=2)+'\n')
