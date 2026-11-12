import blenderproc as bproc
import argparse
import json
from pathlib import Path
import sys

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate import place
from sampling import (load_camera, stable_rest_poses, support_stability, camera_rotation,
                      projected_bounds, sample_image_placement, code_hashes)

p = argparse.ArgumentParser(description='No RGB/depth/mask rendering; geometry-only assertions')
p.add_argument('--config', type=Path, required=True)
p.add_argument('--report', type=Path, required=True)
a = p.parse_args()
c = yaml.safe_load(a.config.read_text())
cam = load_camera(c['camera'])
k = np.array(cam['K']).reshape(3, 3)
width, height = cam['width'], cam['height']
bproc.init()
bproc.camera.set_intrinsics_from_K_matrix(k, cam['width'], cam['height'])
rng = np.random.default_rng(17)
results = []
for cls, filename in c['assets'].items():
    obj = bproc.loader.load_obj(str(a.config.resolve().parents[1]/'assets'/filename))[0]
    mesh = obj.blender_obj.data
    mesh.calc_loop_triangles()
    vertices = np.array([v.co[:] for v in mesh.vertices])
    offset = np.r_[vertices.mean(axis=0)[:2], vertices[:,2].min()]
    for v in mesh.vertices:
        v.co -= Vector(offset)
    local = vertices-offset
    faces = np.array([tuple(t.vertices) for t in mesh.loop_triangles])
    scale = c[cls+'_scale']
    obj.set_scale([scale]*3)
    config = c['rock_poses'][cls]
    poses, com = stable_rest_poses(local*scale, faces, config)
    stable_ranks = set()
    checks = 0
    for pose in poses[:4]:
        for slope in (0., np.deg2rad(6)):
            for yaw in (0., 0.7):
                contact = place(obj, [0.12,-0.08], yaw, slope, pose['rotation'])
                matrix = np.array(obj.get_local2world_mat())
                world = local @ matrix[:3,:3].T+matrix[:3,3]
                world_com = matrix[:3,:3] @ (com/scale)+matrix[:3,3]
                assert abs(contact)<1e-6
                assert abs((world[:,0]*np.sin(slope)+world[:,2]*np.cos(slope)).min())<2e-6
                stable = support_stability(world, world_com, slope, config['support_tolerance_m'],config['support_margin_m'])
                if stable['stable']:
                    stable_ranks.add(pose['rank'])
                    checks += 1
    assert len(stable_ranks)>=2, f'{cls}: no verified multiple stable poses'
    place(obj,[0,0],0,0,poses[0]['rotation'])
    matrix = np.array(obj.get_local2world_mat())
    world = local @ matrix[:3,:3].T+matrix[:3,3]
    center = (world.min(axis=0)+world.max(axis=0))/2
    predicted = {}
    for distance in (0.4,0.8,1.5,2.5):
        position = center+distance*np.array([0,-2**-0.5,2**-0.5])
        rotation = camera_rotation(center-position,k,[0.5,0.5],width,height)
        bounds = projected_bounds(world,position,rotation,k)
        predicted[str(distance)] = (bounds[2:]-bounds[:2]).tolist()
    errors = []
    for region,settings in cam['image_regions'].items():
        position = center+np.array([0,-0.5,0.5])
        rotation, placement = sample_image_placement(rng,region,settings,world,center,position,k,width,height)
        bproc.camera.add_camera_pose(bproc.math.build_transformation_mat(position,rotation),frame=0)
        bpy.context.scene.frame_set(0)
        bpy.context.view_layer.update()
        ndc = world_to_camera_view(bpy.context.scene,bpy.context.scene.camera,Vector(center))
        pixel = np.array([ndc.x*width,(1-ndc.y)*height])
        desired = np.array(placement['requested_uv'])*[width,height]
        error = float(np.max(np.abs(pixel-desired)))
        assert error<=1.0, (region,error)
        errors.append(error)
    results.append(dict(class_name=cls, stable_pose_ranks=sorted(stable_ranks),
                        accepted_support_checks=checks, predicted_bbox_wh_px=predicted,
                        blender_projection_max_error_px=max(errors)))
a.report.write_text(json.dumps(dict(rgb_images_generated=0, results=results,
    assumption='uniform_density_not_real_prop_validation',code_hashes=code_hashes(Path(__file__).parent)),indent=2)+'\n')
print(json.dumps(results,indent=2))
