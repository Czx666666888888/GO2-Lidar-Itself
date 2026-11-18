"""Standing Go2 footprint. Inputs to edge_distance use body axes (+x forward, +y left)."""
import math
from pathlib import Path

import yaml

with (Path(__file__).resolve().parents[1] / "body_footprint.yaml").open() as stream:
    _config = yaml.safe_load(stream)

HALF_LENGTH = float(_config["length"]) / 2
HALF_WIDTH = float(_config["width"]) / 2
HARD_CLEARANCE = float(_config["hard_clearance"])
PARKING_CLEARANCE = float(_config["parking_clearance"])
if not all(math.isfinite(v) and v > 0 for v in
           (HALF_LENGTH, HALF_WIDTH, HARD_CLEARANCE, PARKING_CLEARANCE)) or PARKING_CLEARANCE <= HARD_CLEARANCE:
    raise ValueError("机身尺寸必须为有限正数，停靠净空须大于硬停净空")

CORNER_RADIUS = math.hypot(HALF_LENGTH, HALF_WIDTH)


def edge_distance(x, y):
    """Minimum planar distance from a body-frame obstacle point to the rectangle."""
    return math.hypot(max(abs(x) - HALF_LENGTH, 0.), max(abs(y) - HALF_WIDTH, 0.))
