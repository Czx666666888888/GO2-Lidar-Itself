"""Sampling and geometry helpers; no renderer, ROS or dataset writes."""
import hashlib
from pathlib import Path

import numpy as np


def code_hashes(folder):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(Path(folder).glob('*.py')) if not p.name.startswith('test_')}


def weighted_choice(rng, entries):
    names = list(entries)
    weights = np.array([entries[n]['weight'] for n in names], dtype=float)
    if not len(names) or not np.isfinite(weights).all() or (weights < 0).any() or weights.sum() <= 0:
        raise ValueError('Sampling weights must be finite, nonnegative, with positive total')
    return str(rng.choice(names, p=weights / weights.sum()))


def validate_config(config):
    if config['classes'] != ['color_block', 'rock_compact', 'rock_spiky']:
        raise ValueError('Science classes must remain unchanged')
    for name, band in config['camera']['distance_bands'].items():
        interval = np.asarray(band['range_m'], dtype=float)
        if interval.shape != (2,) or not np.isfinite(interval).all() or not 0 < interval[0] < interval[1]:
            raise ValueError(f'Invalid literal metre distance range: {name}')
    weighted_choice(np.random.default_rng(0), config['camera']['distance_bands'])
    regions = config['camera']['image_regions']
    allowed = {'center', 'left', 'right', 'top_edge', 'bottom_edge', 'near_edge_full', 'partial_outside'}
    if not set(regions).issubset(allowed):
        raise ValueError('Unknown image region')
    weighted_choice(np.random.default_rng(0), regions)
    for name, settings in regions.items():
        keys = ('gap_px',) if name == 'near_edge_full' else ('outside_fraction',) if name == 'partial_outside' else ('u','v')
        for key in keys:
            limits = np.asarray(settings[key], dtype=float)
            if limits.shape != (2,) or not np.isfinite(limits).all() or not 0 <= limits[0] < limits[1]:
                raise ValueError('Invalid image placement interval')
            if key in ('u','v','outside_fraction') and (limits[1] > 1 or (key=='outside_fraction' and limits[1]>=1)):
                raise ValueError('Invalid normalized image interval')
    if config['render']['views_per_scene'] < 1 or config['render']['max_attempts_per_scene'] < 1:
        raise ValueError('Positive view count and retry budget required')
    if config['render']['min_pixels'] <= 0 or config['render']['min_bbox_side_px'] <= 0:
        raise ValueError('Positive visible area and bbox side thresholds required')
    if not 0 < config['render']['min_visible_fraction'] <= 1:
        raise ValueError('Invalid visible fraction')
    for name in ('rock_compact', 'rock_spiky'):
        if not np.isfinite(config[name+'_scale']) or config[name+'_scale'] <= 0:
            raise ValueError('Model scale must be finite and positive')
        pose = config['rock_poses'][name]
        if pose['max_poses'] < 1 or pose['support_tolerance_m'] <= 0 or pose['support_margin_m'] < 0 or pose['max_placement_attempts'] < 1:
            raise ValueError('Invalid support check or pose budget')
        if not 0 <= pose['min_probability'] < 1:
            raise ValueError('Invalid resting probability threshold')
        if pose['mode'] != 'stable_hull':
            raise ValueError('Only geometry-checked stable_hull rock poses supported')
    if config['block']['colors'].get('color_6_pending') is not None:
        raise ValueError('Sixth color is pending; do not sample it')


def load_camera(config):
    import yaml
    camera = dict(config)
    camera['source'] = 'synthetic_camera'
    if camera.get('camera_info'):
        path = Path(camera['camera_info'])
        info = yaml.safe_load(path.read_text())
        camera.update(K=info.get('k', info.get('K', info.get('camera_matrix', {}).get('data'))),
                      width=info.get('width', info.get('image_width')),
                      height=info.get('height', info.get('image_height')),
                      source='d435_real_camera_info', original_camera_info=info,
                      camera_info_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        distortion = info.get('d', info.get('D', info.get('distortion_coefficients', {}).get('data', [])))
        if not np.isfinite(distortion).all() or any(abs(x) > 1e-9 for x in distortion):
            raise ValueError('Supply rectified CameraInfo; nonzero/invalid distortion unsupported')
    k = np.asarray(camera['K'], dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
        raise ValueError('Invalid pinhole intrinsics')
    if not camera['width'] or not camera['height'] or min(camera['width'], camera['height']) <= 0:
        raise ValueError('Invalid image resolution')
    return camera


def camera_rotation(forward, k, uv, width, height):
    """Blender camera-to-world rotation projecting look point to requested normalized UV."""
    direction = np.asarray(forward, dtype=float)
    direction /= np.linalg.norm(direction)
    right = np.cross(direction, [0, 0, 1.0])
    if np.linalg.norm(right) < 1e-8:
        raise ValueError('Camera parallel to world vertical')
    right /= np.linalg.norm(right)
    base = np.column_stack([right, np.cross(right, direction), -direction])
    ray = np.linalg.solve(k, [uv[0]*width, uv[1]*height, 1.0]) * [1, -1, -1]
    ray /= np.linalg.norm(ray)
    axis = np.array([0., 0., -1.])
    v = np.cross(ray, axis)
    skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    correction = np.eye(3) + skew + skew @ skew / (1 + np.dot(ray, axis))
    return base @ correction


def projected_bounds(vertices, position, rotation, k):
    local = (np.asarray(vertices)-position) @ rotation
    optical = local * [1, -1, -1]
    if np.any(optical[:, 2] <= 1e-5):
        raise ValueError('Target crosses camera near plane')
    pixels = optical @ k.T
    pixels = pixels[:, :2] / pixels[:, 2:3]
    return np.r_[pixels.min(axis=0), pixels.max(axis=0)]


def sample_image_placement(rng, region, settings, vertices, center, position, k, width, height):
    """Fit a chosen edge to the projected mesh bounds; no coordinates outside image become labels."""
    if region not in ('near_edge_full', 'partial_outside'):
        uv = [float(rng.uniform(*settings['u'])), float(rng.uniform(*settings['v']))]
        rotation = camera_rotation(center-position, k, uv, width, height)
        bounds = projected_bounds(vertices, position, rotation, k)
        return rotation, dict(region=region, requested_uv=uv, projected_bbox_unclipped=bounds.tolist(), visibility='full')
    side = str(rng.choice(['left', 'right', 'top', 'bottom']))
    axis = 0 if side in ('left', 'right') else 1
    limit = width if axis == 0 else height
    edge_index = axis if side in ('left', 'top') else axis+2
    uv = np.array([0.5, 0.5])
    uv[1-axis] = float(rng.uniform(0.35, 0.65))
    gap = float(rng.uniform(*settings.get('gap_px', [4, 20])))
    fraction = float(rng.uniform(*settings.get('outside_fraction', [0.1, 0.35])))
    for _ in range(12):
        rotation = camera_rotation(center-position, k, uv, width, height)
        bounds = projected_bounds(vertices, position, rotation, k)
        size = bounds[axis+2]-bounds[axis]
        inset = gap if region == 'near_edge_full' else -fraction*size
        desired = inset if edge_index < 2 else limit-inset
        error = desired-bounds[edge_index]
        if abs(error) < 0.05:
            break
        uv[axis] += error/limit
    return rotation, dict(region=region, side=side, requested_uv=uv.tolist(),
                          projected_bbox_unclipped=bounds.tolist(), visibility='full' if region=='near_edge_full' else 'partial',
                          requested_gap_px=gap if region=='near_edge_full' else None,
                          requested_outside_fraction=fraction if region=='partial_outside' else None)


def placement_matches(placement, solo_mask, instance_id, width, height):
    bounds = np.array(placement['projected_bbox_unclipped'])
    if placement['visibility'] == 'full':
        inside = bounds[0] >= 1 and bounds[1] >= 1 and bounds[2] <= width-1 and bounds[3] <= height-1
        if placement.get('region') == 'near_edge_full':
            gap = {'left':bounds[0], 'right':width-bounds[2], 'top':bounds[1], 'bottom':height-bounds[3]}[placement['side']]
            inside = inside and abs(gap-placement['requested_gap_px']) <= 1
        return bool(inside)
    if instance_id is None:
        return False
    pixels = solo_mask == instance_id
    side = placement['side']
    border = {'left': pixels[:, 0], 'right': pixels[:, -1], 'top': pixels[0], 'bottom': pixels[-1]}[side]
    crosses = {'left': bounds[0] < 0, 'right': bounds[2] > width, 'top': bounds[1] < 0, 'bottom': bounds[3] > height}[side]
    axis = 0 if side in ('left','right') else 1
    span = bounds[axis+2]-bounds[axis]
    outside = {'left':-bounds[0], 'right':bounds[2]-width, 'top':-bounds[1], 'bottom':bounds[3]-height}[side]/span
    correct_fraction = abs(outside-placement.get('requested_outside_fraction',outside)) <= 0.02
    return bool(crosses and border.any() and pixels.any() and correct_fraction)


def stable_rest_poses(vertices, faces, config):
    """Trimesh resting orientations; uniform-density COM is an explicit modelling assumption."""
    import trimesh
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    if not mesh.is_volume or not np.isfinite(mesh.center_mass).all():
        raise ValueError('Stable resting poses require a closed outward mesh with finite COM')
    hull = mesh.convex_hull
    if not np.all(np.einsum('ij,ij->i', mesh.center_mass-hull.triangles_center, hull.face_normals) < 0):
        raise ValueError('COM outside convex hull')
    transforms, probabilities = mesh.compute_stable_poses(sigma=0.0, n_samples=1, threshold=0.0)
    ranks = config.get('pose_ranks')
    selected = [dict(rank=i, rotation=t[:3, :3], probability=float(prob))
                for i, (t, prob) in enumerate(zip(transforms, probabilities))
                if prob >= config['min_probability'] and (ranks is None or i in ranks)]
    if not selected:
        raise ValueError('No allowed stable resting poses; do not substitute arbitrary roll/pitch')
    return selected[:config['max_poses']], mesh.center_mass.copy()


def support_stability(world_vertices, world_com, slope, tolerance_m, margin_m):
    """Gravity projection must lie inside a non-degenerate support polygon on the actual slope."""
    from scipy.spatial import ConvexHull, QhullError
    points = np.asarray(world_vertices)
    normal = np.array([np.sin(slope), 0, np.cos(slope)])
    signed = points @ normal
    tangent = np.array([np.cos(slope), 0, -np.sin(slope)])
    contacts = points[signed <= tolerance_m]
    if signed.min() < -tolerance_m or len(contacts) < 3:
        return dict(stable=False, reason='penetration_or_point_contact')
    support = np.column_stack([contacts @ tangent, contacts[:, 1]])
    projected_com = np.array(world_com, copy=True)
    projected_com[2] = -projected_com[0]*np.tan(slope)
    point = np.array([projected_com @ tangent, projected_com[1]])
    try:
        hull = ConvexHull(support)
    except QhullError:
        return dict(stable=False, reason='degenerate_support_polygon')
    clearances = -(hull.equations[:, :2] @ point + hull.equations[:, 2])
    clearance = float(clearances.min())
    return dict(stable=bool(clearance >= margin_m), clearance_m=clearance,
                contact_vertices=len(contacts), support_area_m2=float(hull.volume),
                com_world=world_com.tolist(), assumption='uniform_density_not_real_prop_validation')
