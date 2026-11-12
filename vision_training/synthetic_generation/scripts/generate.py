import blenderproc as bproc
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import subprocess

import bpy
from mathutils import Matrix, Vector
import numpy as np
from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from labels import visible_annotations
from sampling import (code_hashes, validate_config, load_camera, weighted_choice,
                      sample_image_placement, placement_matches,
                      stable_rest_poses, support_stability)


def material(name, rgba, roughness=0.8, wood=False, textured=False):
    mat = bproc.material.create(name)
    mat.set_principled_shader_value('Base Color', rgba)
    mat.set_principled_shader_value('Roughness', roughness)
    if wood or textured:
        nodes = mat.blender_obj.node_tree.nodes
        links = mat.blender_obj.node_tree.links
        noise = nodes.new('ShaderNodeTexNoise')
        noise.inputs['Scale'].default_value = 12 if wood else 25
        bump = nodes.new('ShaderNodeBump')
        bump.inputs['Strength'].default_value = 0.08
        bump.inputs['Distance'].default_value = 0.00015 if wood else 0.0008
        links.new(noise.outputs['Fac'], bump.inputs['Height'])
        links.new(bump.outputs['Normal'], nodes.get('Principled BSDF').inputs['Normal'])
    return mat


def corners(obj):
    return np.array([obj.blender_obj.matrix_world @ Vector(v) for v in obj.blender_obj.bound_box])


def place(obj, xy, yaw, slope, rest_rotation=None):
    rotation = Matrix.Rotation(slope, 3, 'Y') @ Matrix.Rotation(yaw, 3, 'Z')
    if rest_rotation is not None:
        rotation = rotation @ Matrix(rest_rotation)
    obj.set_rotation_euler(rotation.to_euler())
    obj.set_location([*xy, 0])
    bpy.context.view_layer.update()
    evaluated = obj.blender_obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    verts = np.array([evaluated.matrix_world @ v.co for v in evaluated.data.vertices])
    # Ry(slope) plane normal is (+sin(slope),0,cos(slope)): z=-x*tan(slope).
    z = np.max(-verts[:, 0] * math.tan(slope) - verts[:, 2])
    obj.set_location([*xy, float(z)])
    bpy.context.view_layer.update()
    final = obj.blender_obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    world = np.array([final.matrix_world @ v.co for v in final.data.vertices])
    return float(np.min(world[:, 2] + world[:, 0]*math.tan(slope)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--per-class', type=int, default=4)
    p.add_argument('--allow-unverified-scale', action='store_true')
    p.add_argument('--validated-preview', type=Path)
    p.add_argument('--distance-band', choices=['near','medium','far'])
    p.add_argument('--image-region', choices=['center','left','right','top_edge','bottom_edge','near_edge_full','partial_outside'])
    a = p.parse_args()
    c = yaml.safe_load(a.config.read_text())
    if a.distance_band:
        for name, band in c['camera']['distance_bands'].items():
            band['weight'] = float(name == a.distance_band)
    if a.image_region:
        for name, region in c['camera']['image_regions'].items():
            region['weight'] = float(name == a.image_region)
    validate_config(c)
    if a.per_class <= 0 or a.per_class % c['render']['views_per_scene']:
        raise ValueError('per-class must be a positive multiple of views_per_scene')
    if not c['physical_scale_confirmed'] and not a.allow_unverified_scale:
        raise ValueError('STL physical scale unknown; audit assets and confirm dimensions, or use explicit preview override')
    if a.per_class > 4:
        if a.validated_preview is None:
            raise ValueError('Run small preview and quality check first; --validated-preview required')
        quality = json.loads((a.validated_preview/'quality.json').read_text())
        previous = json.loads((a.validated_preview/'manifest.json').read_text())
        if quality['automatic_checks'] != 'PASS' or quality['visual_review'] != 'PASS' or previous['config'] != c or quality.get('manifest_sha256') != hashlib.sha256((a.validated_preview/'manifest.json').read_bytes()).hexdigest():
            raise ValueError('Preview quality not accepted or config changed')
        if previous.get('run',{}).get('code_hashes') != code_hashes(Path(__file__).parent):
            raise ValueError('Generator changed since validated preview')
    a.output.mkdir(parents=True, exist_ok=False)
    assets = a.config.resolve().parents[1] / 'assets'
    sources = {k: dict(path=str(assets / v), sha256=hashlib.sha256((assets / v).read_bytes()).hexdigest(),
                      scale=c[k + '_scale'], unit='unknown', physical_scale_confirmed=c['physical_scale_confirmed'])
               for k, v in c['assets'].items()}
    camera = load_camera(c['camera'])
    rng = np.random.default_rng(c['seed'])
    bproc.init()
    bproc.renderer.set_render_devices(use_only_cpu=True)
    bproc.camera.set_intrinsics_from_K_matrix(np.array(camera['K']).reshape(3, 3), camera['width'], camera['height'])
    bproc.renderer.set_max_amount_of_samples(c['render']['samples'])
    if c['render']['save_depth']:
        bproc.renderer.enable_depth_output(activate_antialiasing=False)
    run_info = dict(blender_version=bpy.app.version_string, blenderproc_version=bproc.__version__,
                    code_hashes=code_hashes(Path(__file__).parent),
                    config_sha256=hashlib.sha256(yaml.safe_dump(c, sort_keys=True).encode()).hexdigest(),
                    generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                    source_type='synthetic', physical_scale_confirmed=c['physical_scale_confirmed'])
    rows = []
    pose_cache = {}
    rejection_log = []
    def save_manifest():
        temporary = a.output/'manifest.tmp'
        temporary.write_text(json.dumps(dict(config=c, run=run_info, records=rows, rejected_attempts=rejection_log), indent=2))
        temporary.replace(a.output/'manifest.json')
    count_scenes = a.per_class // c['render']['views_per_scene']
    for cls in range(3):
        for scene_idx in range(count_scenes):
            bproc.clean_up(clean_up_camera=False)
            scene_id = f'c{cls}_s{scene_idx:04d}'
            # Independent placement scenes; all views of a placement share split.
            split = 'val' if scene_idx % 5 == 0 else 'train'
            slope = math.radians(rng.uniform(*c['scene']['slope_deg']))
            floor = bproc.object.create_primitive('PLANE', scale=[10, 10, 1])
            floor.set_rotation_euler([0, slope, 0])
            floor.set_cp('category_id', 0)
            gray = float(rng.uniform(0.09, 0.3))
            floor.replace_materials(material('floor', [gray]*3+[1], textured=True))
            mats = {name: material(name, rgba or [0.18]*3+[1], c['block']['roughness'], name == 'natural_wood')
                    for name, rgba in c['block']['colors'].items()}
            targets, meta, occupied = [], {}, []
            n = 1 + int(rng.integers(c['scene']['additional_targets'][0], c['scene']['additional_targets'][1]+1))
            for i in range(n):
                cid = cls if i == 0 else int(rng.integers(0, 3))
                name = c['classes'][cid]
                if cid == 0:
                    obj = bproc.object.create_primitive('CUBE', scale=[c['block']['size_m']/2]*3)
                    # Bake scale before bevel so width is measured in metres.
                    bpy.ops.object.select_all(action='DESELECT')
                    bpy.context.view_layer.objects.active = obj.blender_obj
                    obj.blender_obj.select_set(True)
                    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
                    mod = obj.blender_obj.modifiers.new('bevel_m', 'BEVEL')
                    mod.width, mod.segments = c['block']['bevel_m'], 3
                    color = rng.choice([k for k, v in c['block']['colors'].items() if v is not None])
                    obj.replace_materials(mats[color])
                    source = dict(procedural='Blender CUBE', size_m=c['block']['size_m'], color=str(color))
                else:
                    imported = bproc.loader.load_obj(str(assets / c['assets'][name]))
                    if len(imported) != 1:
                        raise ValueError('Expected one mesh per STL')
                    obj = imported[0]
                    raw_bbox = corners(obj).tolist()
                    vertices = obj.blender_obj.data.vertices
                    xyz = np.array([v.co[:] for v in vertices])
                    offset = np.array([*xyz.mean(axis=0)[:2], xyz[:,2].min()])
                    for v in vertices:
                        v.co -= Vector(offset)
                    source_offset = offset.tolist()
                    obj.set_scale([c[name + '_scale']]*3)
                    bpy.context.view_layer.update()
                    source = dict(**sources[name], bbox_before=raw_bbox, bbox_after=corners(obj).tolist(), origin_translation_raw=source_offset)
                    g = float(rng.uniform(0.18, 0.32))
                    obj.replace_materials(material(name, [g]*3+[1]))
                    source['gray_rgba'] = [g]*3+[1]
                obj.set_name(f'target_{i}')
                obj.set_cp('category_id', cid+1)  # 0 background; YOLO IDs subtract 1
                radius = float(max(np.linalg.norm(obj.blender_obj.matrix_world @ v.co) for v in obj.blender_obj.data.vertices))
                xy = np.zeros(2) if i == 0 else np.array([max(x[0]+r for x,r in occupied)+radius+0.025, 0])
                resting = None
                stability = None
                if cid == 0:
                    contact_error = place(obj, xy, float(rng.uniform(-math.pi, math.pi)), slope)
                else:
                    cfg = c['rock_poses'][name]
                    if name not in pose_cache:
                        obj.blender_obj.data.calc_loop_triangles()
                        local = np.array([v.co[:] for v in obj.blender_obj.data.vertices])
                        faces = np.array([tuple(f.vertices) for f in obj.blender_obj.data.loop_triangles])
                        pose_cache[name] = stable_rest_poses(local*c[name+'_scale'], faces, cfg)
                    candidates, scaled_com = pose_cache[name]
                    weights = np.array([v['probability'] for v in candidates])
                    for placement_attempt in range(cfg['max_placement_attempts']):
                        resting = candidates[int(rng.choice(len(candidates), p=weights/weights.sum()))]
                        contact_error = place(obj, xy, float(rng.uniform(-math.pi, math.pi)), slope, resting['rotation'])
                        world = np.array([obj.blender_obj.matrix_world @ v.co for v in obj.blender_obj.data.vertices])
                        com = np.array(obj.blender_obj.matrix_world @ Vector(scaled_com/c[name+'_scale']))
                        stability = support_stability(world, com, slope, cfg['support_tolerance_m'], cfg['support_margin_m'])
                        if stability['stable']:
                            break
                    else:
                        save_manifest()
                        raise RuntimeError(f'{name}: no stable support on this slope; geometry or real COM needs review')
                if abs(contact_error) > 1e-6:
                    raise ValueError('Target ground contact check failed')
                if any(np.linalg.norm(xy-x) < radius+r for x,r in occupied):
                    raise ValueError('Conservative target separation failed')
                occupied.append((xy, radius))
                targets.append(obj)
                meta[obj.get_name()] = dict(class_id=cid, source=source,
                    resting_pose=None if resting is None else dict(rank=resting['rank'], probability=resting['probability'], rotation=resting['rotation'].tolist()),
                    stability=stability, floor_contact_error_m=contact_error, separation_radius_m=radius, pose=obj.get_local2world_mat().tolist())
            primary = targets[0]
            center = corners(primary).mean(axis=0)
            primary_vertices = np.array([primary.blender_obj.matrix_world @ v.co for v in primary.blender_obj.evaluated_get(bpy.context.evaluated_depsgraph_get()).data.vertices])
            obstacles = []
            for j in range(int(rng.integers(*c['scene']['background_rocks']))):
                bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=1)
                rock = bproc.types.MeshObject(bpy.context.object)
                rock.set_name(f'background_{j}')
                for v in rock.blender_obj.data.vertices:
                    v.co *= float(rng.uniform(0.72, 1.25))
                size = rng.uniform(0.015, 0.06, 3)
                rock.set_scale(size)
                xy = center[:2] + rng.uniform(-0.35, 0.35, 2)
                bpy.context.view_layer.update()
                r = float(max(np.linalg.norm(rock.blender_obj.matrix_world @ v.co) for v in rock.blender_obj.data.vertices))
                if any(np.linalg.norm(xy-x) < r+rr+0.01 for x, rr in occupied):
                    rock.delete()
                    continue
                occupied.append((xy, r))
                place(rock, xy, float(rng.uniform(-math.pi, math.pi)), slope)
                rock.set_cp('category_id', 0)
                g = float(rng.uniform(0.08, 0.35))
                rock.replace_materials(material('ordinary_rock', [g]*3+[1], textured=True))
                obstacles.append(rock)
            bpy.context.scene.world.use_nodes = True
            world_strength = float(rng.uniform(*c['scene']['world_strength']))
            background = bpy.context.scene.world.node_tree.nodes.get('Background')
            background.inputs['Color'].default_value = [0.12]*3+[1]
            background.inputs['Strength'].default_value = world_strength
            lamp = bproc.types.Light()
            lamp.set_type('AREA')
            lamp.set_location(center + np.array([rng.uniform(-0.4, 0.4), -0.3, 0.6]))
            energy = float(rng.uniform(*c['scene']['light_energy']))
            lamp.set_energy(energy)
            lamp.blender_obj.data.size = 0.25
            lamp.blender_obj.rotation_euler = (Vector(center)-lamp.blender_obj.location).to_track_quat('-Z','Y').to_euler()
            supplement = bproc.types.Light()
            supplement.set_type('SPOT')
            supplement.blender_obj.data.spot_size = math.radians(65)
            supplement.blender_obj.data.spot_blend = 0.55
            accepted = 0
            request = None
            view_attempts = 0
            for attempt in range(c['render']['max_attempts_per_scene']*c['render']['views_per_scene']):
                if accepted == c['render']['views_per_scene']:
                    break
                if view_attempts >= c['render']['max_attempts_per_scene']:
                    break
                view_attempts += 1
                if request is None:
                    request = dict(distance_band=weighted_choice(rng, camera['distance_bands']),
                                   image_region=weighted_choice(rng, camera['image_regions']))
                distance = float(rng.uniform(*camera['distance_bands'][request['distance_band']]['range_m']))
                depression = math.radians(rng.uniform(*camera['depression_deg']))
                azimuth = float(rng.uniform(-math.pi, math.pi))
                position = center + distance*np.array([math.cos(depression)*math.cos(azimuth), math.cos(depression)*math.sin(azimuth), math.sin(depression)])
                k = np.array(camera['K']).reshape(3, 3)
                try:
                    rotation, placement = sample_image_placement(rng, request['image_region'],
                        camera['image_regions'][request['image_region']], primary_vertices, center, position,
                        k, camera['width'], camera['height'])
                except ValueError as error:
                    rejection_log.append(dict(scene_id=scene_id, view=accepted, attempt=attempt,
                        request=request.copy(), distance_m=distance, reason='invalid_projection', detail=str(error)))
                    continue
                optical_forward = rotation @ np.array([0., 0., -1.])
                optical_depression = math.degrees(math.asin(-optical_forward[2]))
                reason = None
                bounds = np.array(placement['projected_bbox_unclipped'])
                if not camera['depression_deg'][0] <= optical_depression <= camera['depression_deg'][1]:
                    reason = 'optical_depression_out_of_range'
                elif min(bounds[2]-bounds[0], bounds[3]-bounds[1]) < c['render']['min_bbox_side_px']:
                    reason = 'projected_target_too_small'
                if reason:
                    rejection_log.append(dict(scene_id=scene_id, view=accepted, attempt=attempt,
                        request=request.copy(), distance_m=distance, reason=reason))
                    continue
                aim = center
                supplement.set_location(position + np.array([0,0,0.015]))
                supplement_energy = float(rng.uniform(*c['scene']['supplement_energy']))
                supplement.set_energy(supplement_energy)
                supplement.blender_obj.rotation_euler = (Vector(aim)-supplement.blender_obj.location).to_track_quat('-Z','Y').to_euler()
                pose = bproc.math.build_transformation_mat(position, rotation)
                bpy.context.scene.frame_end = 0
                bproc.camera.add_camera_pose(pose, frame=0)
                data = bproc.renderer.render()
                seg = bproc.renderer.render_segmap(map_by=['instance', 'class', 'name'], default_values={'class':0})
                # Amodal in-frame silhouette: render each target alone, leaving camera unchanged.
                full_areas = {}
                primary_solo = None
                primary_instance = None
                meshes = [floor] + targets + obstacles
                for target in targets:
                    for obj in meshes:
                        obj.blender_obj.hide_render = obj != target
                    solo = bproc.renderer.render_segmap(map_by=['instance', 'name'])
                    sm = solo['instance_segmaps'][0]
                    mapping = solo['instance_attribute_maps'][0]
                    if target == primary:
                        primary_solo = sm.copy()
                        primary_instance = next((r['idx'] for r in mapping if r.get('name') == primary.get_name()), None)
                    full_areas[target.get_name()] = sum(int((sm == row['idx']).sum()) for row in mapping if row.get('name') == target.get_name())
                for obj in meshes:
                    obj.blender_obj.hide_render = False
                annotations, rejected = visible_annotations(seg['instance_segmaps'][0], seg['instance_attribute_maps'][0], meta, full_areas, c['render'])
                # Never create partial negative labels: discard a frame if any target is unlabelable.
                placement_ok = placement_matches(placement, primary_solo, primary_instance, camera['width'], camera['height'])
                if rejected or not annotations or not placement_ok:
                    rejection_log.append(dict(scene_id=scene_id, view=accepted, attempt=attempt,
                        request=request.copy(), distance_m=distance,
                        reason='mask_visibility_or_size' if rejected else 'image_region_constraint', targets=rejected))
                    continue
                stem = f'{scene_id}_v{accepted:02d}'
                for sub in ('images','labels','metadata','masks'):
                    (a.output/sub/split).mkdir(parents=True, exist_ok=True)
                Image.fromarray(data['colors'][0].astype(np.uint8)).convert('RGB').save(a.output/'images'/split/(stem+'.png'))
                np.save(a.output/'masks'/split/(stem+'.npy'), seg['instance_segmaps'][0])
                (a.output/'labels'/split/(stem+'.txt')).write_text(''.join(f"{x['class_id']} " + ' '.join(f'{v:.8f}' for v in x['yolo'])+'\n' for x in annotations))
                row = dict(id=stem, scene_id=scene_id, split=split, source_type='synthetic', primary_class=cls,
                           camera=camera, camera_to_world=pose.tolist(), distance_m=distance,
                           sampling_request=request.copy(), image_placement=placement,
                           target_ray_depression_deg=math.degrees(depression),
                           depression_deg=optical_depression, slope_deg=math.degrees(slope),
                           light_energy=energy, light_position=list(lamp.blender_obj.location), light_size=lamp.blender_obj.data.size,
                           supplement_energy=supplement_energy, supplement_position=list(supplement.blender_obj.location),
                           world_strength=world_strength, floor_gray=gray,
                           background_rocks=[dict(name=o.get_name(),pose=o.get_local2world_mat().tolist(),
                           vertices=[list(v.co) for v in o.blender_obj.data.vertices]) for o in obstacles], seed=c['seed'], annotations=annotations, targets=meta,
                           scale_unverified=not c['physical_scale_confirmed'])
                (a.output/'metadata'/split/(stem+'.json')).write_text(json.dumps(row, indent=2))
                if c['render']['save_depth']:
                    np.save(a.output/'metadata'/split/(stem+'_depth.npy'), data['depth'][0])
                rows.append(row)
                save_manifest()
                accepted += 1
                request = None
                view_attempts = 0
            if accepted != c['render']['views_per_scene']:
                save_manifest()
                (a.output/'failure.json').write_text(json.dumps(dict(scene_id=scene_id, accepted=accepted, failed_request=request, reasons=rejection_log), indent=2))
                raise RuntimeError(f'{scene_id}: insufficient valid frames; partial output preserved, not accepted')
    (a.output/'data.yaml').write_text(yaml.safe_dump(dict(path=str(a.output.resolve()), train='images/train', val='images/val', names=c['classes'])))
    (a.output/'COMPLETE').write_text(str(len(rows))+'\n')


if __name__ == '__main__':
    main()
