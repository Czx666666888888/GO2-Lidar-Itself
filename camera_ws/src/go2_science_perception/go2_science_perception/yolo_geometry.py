"""Conservative RGB-aligned depth sampling, independent of detector classes."""

import math

import numpy as np


def reliable_box_depth(depth, box, min_depth=0.15, max_depth=5.0, roi_fraction=0.5,
                       min_pixels=20, min_valid_ratio=0.6, max_spread=0.08):
    """Return central-ROI median depth only when enough consistent samples exist."""
    x1, y1, x2, y2 = (float(v) for v in box)
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        return dict(valid=False, reason='invalid_box')
    height, width = depth.shape
    cx, cy = (x1+x2)/2, (y1+y2)/2
    half_w, half_h = (x2-x1)*roi_fraction/2, (y2-y1)*roi_fraction/2
    left, right = max(0, int(math.floor(cx-half_w))), min(width, int(math.ceil(cx+half_w)))
    top, bottom = max(0, int(math.floor(cy-half_h))), min(height, int(math.ceil(cy+half_h)))
    roi = depth[top:bottom, left:right]
    if not roi.size:
        return dict(valid=False, reason='empty_depth_roi')
    mask = np.isfinite(roi) & (roi >= min_depth) & (roi <= max_depth)
    values = roi[mask]
    ratio = float(values.size/roi.size)
    out = dict(valid=False, valid_pixels=int(values.size), valid_ratio=ratio,
               roi_xyxy=[left, top, right, bottom])
    if values.size < min_pixels or ratio < min_valid_ratio:
        return dict(out, reason='insufficient_valid_depth')
    spread = float(np.percentile(values, 90)-np.percentile(values, 10))
    if spread > max_spread:
        return dict(out, reason='mixed_or_noisy_depth', spread_m=spread)
    v, u = np.nonzero(mask)
    return dict(out, valid=True, reason='accepted_central_roi', spread_m=spread,
                depth_m=float(np.median(values)), u=float(np.median(u)+left),
                v=float(np.median(v)+top))


def back_project(u, v, z, k, distortion=(), model='plumb_bob'):
    """Use color CameraInfo and account for supported raw-image distortion."""
    import cv2
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(distortion, dtype=np.float64)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or z <= 0:
        raise ValueError('Invalid camera intrinsics or depth')
    if model not in ('plumb_bob', 'rational_polynomial', ''):
        raise ValueError(f'Unsupported distortion model: {model}')
    if d.size and not np.isfinite(d).all():
        raise ValueError('Invalid distortion coefficients')
    xy = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64), k,
                             d if d.size else None).reshape(2)
    return [float(xy[0]*z), float(xy[1]*z), float(z)]
