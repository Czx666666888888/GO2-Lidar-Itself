"""Estimate point-to-wall distance using straight runs of occupied terrain points."""
import math

import numpy as np


def nearest_wall(target, points):
    """Return distance and observed wall endpoints in MAP XY, or None."""
    relative = np.asarray(points, dtype=float).reshape(-1, 2) - target
    relative = relative[np.isfinite(relative).all(axis=1)
                        & (np.linalg.norm(relative, axis=1) <= 1.2)]
    if len(relative) < 6:
        return None
    # Suppress repeated samples from the same spot before fitting wall lines.
    _, index = np.unique(np.round(relative / .02).astype(int), axis=0, return_index=True)
    relative = relative[index]
    walls = []
    for angle in range(0, 180, 5):
        heading = math.radians(angle)
        normal = np.array((math.cos(heading), math.sin(heading)))
        tangent = np.array((-normal[1], normal[0]))
        offsets = relative @ normal
        for band in np.unique(np.round(offsets / .06).astype(int)):
            if abs(band * .06) > .60:
                continue
            strip = relative[np.abs(offsets - band * .06) <= .04]
            if len(strip) < 6:
                continue
            along = strip @ tangent
            order = np.argsort(along)
            # Do not join separate obstacles or extrapolate past a corner.
            breaks = np.flatnonzero(np.diff(along[order]) > .15) + 1
            for support in np.split(strip[order], breaks):
                if len(support) < 6:
                    continue
                center = support.mean(axis=0)
                _, _, axes = np.linalg.svd(support-center, full_matrices=False)
                axis, perpendicular = axes
                # Trim the perpendicular face's points before refining the
                # line, especially at the join of two walls.
                support = support[np.abs((support-center) @ perpendicular) <= .018]
                if len(support) < 6:
                    continue
                center = support.mean(axis=0)
                _, singular, axes = np.linalg.svd(support-center, full_matrices=False)
                axis, perpendicular = axes
                rms = float(singular[1]) / math.sqrt(len(support))
                projected = (support-center) @ axis
                lo, hi = float(projected.min()), float(projected.max())
                if hi-lo < .45:
                    continue
                closest_on_wall = center + axis * np.clip(-center @ axis, lo, hi)
                distance = float(np.linalg.norm(closest_on_wall))
                endpoints = tuple(tuple(map(float, center + axis * value + target))
                                  for value in (lo, hi))
                strength = (len(support), -rms)
                offset = float(center @ perpendicular)
                for i, (other_normal, other_offset, other_strength, _) in enumerate(walls):
                    dot = float(other_normal @ perpendicular)
                    if abs(dot) < .99 or abs(offset - math.copysign(1., dot)*other_offset) > .06:
                        continue
                    if strength > other_strength:
                        walls[i] = (perpendicular, offset, strength, (distance, endpoints))
                    break
                else:
                    walls.append((perpendicular, offset, strength, (distance, endpoints)))
    return min((wall[3] for wall in walls), default=None, key=lambda item: item[0])
