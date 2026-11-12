"""Convert BlenderProc visible instance pixels to YOLO boxes; background excluded."""
import numpy as np


def visible_annotations(mask, mapping, targets, full_areas, thresholds):
    height, width = mask.shape
    annotations, rejected = [], []
    ids = {row.get('name'): row['idx'] for row in mapping}
    for name, target in targets.items():
        idx = ids.get(name)
        ys, xs = np.where(mask == idx) if idx is not None else (np.array([]), np.array([]))
        area = len(xs)
        full = full_areas.get(name, 0)
        visible = area / full if full else 0
        if area < thresholds['min_pixels'] or visible < thresholds['min_visible_fraction']:
            rejected.append(dict(name=name, area=area, visible_fraction=visible))
            continue
        x0, y0, x1, y1 = int(xs.min()), int(ys.min()), int(xs.max())+1, int(ys.max())+1
        annotations.append(dict(name=name, instance_id=int(idx), class_id=target['class_id'],
                                bbox_xyxy=[x0,y0,x1,y1], visible_pixels=area,
                                amodal_inframe_pixels=full, visible_fraction=visible,
                                yolo=[(x0+x1)/(2*width), (y0+y1)/(2*height),
                                      (x1-x0)/width, (y1-y0)/height]))
    return annotations, rejected
