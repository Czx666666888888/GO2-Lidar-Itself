"""Optional camera-relative association of a stationary target.

A frozen MAP anchor seeds the association. Subsequent candidates are compared
in the CURRENT optical frame using camera poses at exact source times.
This retains dependence on relative odometry; it is not an independent SLAM fix.
"""

from dataclasses import dataclass
import math


def rotate(q, v):
    """Rotate a 3D vector by an xyzw quaternion without third-party deps."""
    x, y, z, w = (float(n) for n in q)
    n2 = x*x + y*y + z*z + w*w
    if n2 < 1e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = (x/math.sqrt(n2), y/math.sqrt(n2),
                   z/math.sqrt(n2), w/math.sqrt(n2))
    vx, vy, vz = v
    tx = 2 * (y*vz-z*vy)
    ty = 2 * (z*vx-x*vz)
    tz = 2 * (x*vy-y*vx)
    return (vx+w*tx+(y*tz-z*ty),
            vy+w*ty+(z*tx-x*tz),
            vz+w*tz+(x*ty-y*tx))


def camera_prediction(world_point, camera_translation, camera_quaternion):
    """Transform a MAP point to the current camera optical frame."""
    delta = tuple(float(p)-float(t) for p, t in zip(world_point, camera_translation))
    x, y, z, w = camera_quaternion
    return rotate((-x, -y, -z, w), delta)


def angular_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


@dataclass(frozen=True)
class Measurement:
    stamp_ns: int
    source_frame: str
    camera_xyz: tuple
    map_xyz: tuple
    camera_in_map: tuple
    camera_quaternion: tuple


class CameraRelativeMatcher:
    """Unique candidate association; source frame and time are mandatory.

    A position jump after a missing frame is promoted only after two mutually
    compatible observations. No candidate in an ambiguous frame is accepted.
    """

    def __init__(self, angle_gate_rad=0.17, range_gate_m=0.25,
                 reprojection_gate_px=80.0, max_gap_s=1.0,
                 map_jump_m=0.15):
        self.angle_gate_rad = angle_gate_rad
        self.range_gate_m = range_gate_m
        self.reprojection_gate_px = reprojection_gate_px
        self.max_gap_s = max_gap_s
        self.map_jump_m = map_jump_m
        self.reset()

    def reset(self, anchor_xy=None):
        self.anchor_xy = tuple(anchor_xy) if anchor_xy is not None else None
        self.last = None
        self.pending = None
        self.pending_count = 0
        self.last_result = {'decision': 'IDLE', 'reason': 'no locked target'}

    @staticmethod
    def _valid(m):
        values = m.camera_xyz + m.map_xyz + m.camera_in_map + m.camera_quaternion
        return (m.stamp_ns > 0 and m.source_frame and
                len(m.camera_xyz) == 3 and len(m.map_xyz) == 3 and
                len(m.camera_in_map) == 3 and len(m.camera_quaternion) == 4 and
                m.camera_xyz[2] > 0 and all(math.isfinite(v) for v in values))

    @staticmethod
    def _azimuth(point):
        return math.atan2(point[0], point[2])

    @staticmethod
    def _range(point):
        return math.sqrt(sum(v*v for v in point))

    def _residual(self, old, new, intrinsics=None):
        predicted = camera_prediction(old.map_xyz, new.camera_in_map,
                                      new.camera_quaternion)
        if predicted[2] <= 0.0:
            return None
        angle = abs(angular_difference(self._azimuth(new.camera_xyz),
                                       self._azimuth(predicted)))
        distance = abs(self._range(new.camera_xyz)-self._range(predicted))
        spatial = math.dist(new.camera_xyz, predicted)
        pixel = None
        if intrinsics is not None:
            fx, fy = intrinsics
            pixel = math.hypot(fx*(new.camera_xyz[0]/new.camera_xyz[2] -
                                   predicted[0]/predicted[2]),
                               fy*(new.camera_xyz[1]/new.camera_xyz[2] -
                                   predicted[1]/predicted[2]))
        return {'angle_rad': angle, 'range_m': distance,
                'spatial_m': spatial, 'reprojection_px': pixel}

    def update(self, candidates, intrinsics=None):
        """Process one complete source-frame batch; return accepted measurement."""
        self.last_result = {'decision': 'REJECT', 'reason': 'no candidates'}
        if self.anchor_xy is None or not candidates:
            return None
        valid = [m for m in candidates if self._valid(m)]
        if not valid:
            self.last_result['reason'] = 'invalid measurement'
            return None
        if len({(m.stamp_ns, m.source_frame) for m in valid}) != 1:
            self.last_result['reason'] = 'mixed source frames or stamps'
            return None
        if self.last is None:
            seeds = [m for m in valid if math.dist(m.map_xyz[:2], self.anchor_xy)
                     <= self.map_jump_m]
            if len(seeds) != 1:
                self.last_result['reason'] = 'seed ambiguous or outside anchor'
                return None
            chosen = seeds[0]
            self.last = chosen
            self.last_result = {'decision': 'ACCEPT', 'reason': 'unique anchor seed',
                                'stamp_ns': chosen.stamp_ns}
            return chosen
        accepted = []
        scores = []
        for m in valid:
            dt = (m.stamp_ns-self.last.stamp_ns)*1e-9
            if m.source_frame != self.last.source_frame or not 0 < dt <= self.max_gap_s:
                continue
            residual = self._residual(self.last, m, intrinsics)
            if residual is None:
                continue
            scores.append((m, residual))
            if (residual['angle_rad'] < self.angle_gate_rad and
                    residual['range_m'] < self.range_gate_m and
                    (residual['reprojection_px'] is None or
                     residual['reprojection_px'] < self.reprojection_gate_px)):
                accepted.append((m, residual))
        self.last_result = {'decision': 'REJECT', 'reason': 'relative geometry gate',
                            'score_count': len(scores),
                            'scores': [result for _, result in scores]}
        if len(accepted) != 1:
            if len(accepted) > 1:
                self.last_result['reason'] = 'ambiguous camera candidates'
            self.pending = None
            self.pending_count = 0
            return None
        chosen, residual = accepted[0]
        jump = math.dist(chosen.map_xyz[:2], self.last.map_xyz[:2])
        if jump > self.map_jump_m:
            if (self.pending is not None and
                    chosen.stamp_ns > self.pending.stamp_ns and
                    chosen.source_frame == self.pending.source_frame):
                previous = self._residual(self.pending, chosen, intrinsics)
                if (previous is not None and
                        previous['angle_rad'] < self.angle_gate_rad and
                        previous['range_m'] < self.range_gate_m):
                    self.pending_count += 1
                else:
                    self.pending_count = 1
            else:
                self.pending_count = 1
            self.pending = chosen
            if self.pending_count < 2:
                self.last_result = {'decision': 'HOLD', 'reason': 'large MAP jump',
                                    'jump_m': jump, 'residual': residual}
                return None
        self.last = chosen
        self.pending = None
        self.pending_count = 0
        self.last_result = {'decision': 'ACCEPT', 'reason': 'camera relative match',
                            'jump_m': jump, 'residual': residual,
                            'stamp_ns': chosen.stamp_ns}
        return chosen
