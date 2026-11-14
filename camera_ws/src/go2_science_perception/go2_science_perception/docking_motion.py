"""Dock-only temporal motion evidence; no device I/O or physical-truth claims."""
from collections import deque
import math
from statistics import median


def window_stats(samples, window_sec, noise_m):
    """Robust trend plus a bounded envelope; a still Sport report never erases poses."""
    if len(samples) < 3:
        return dict(ready=False)
    t, x, y = samples[-1][:3]
    rows = list(samples)
    while len(rows) > 2 and rows[1][0] <= t-window_sec:
        rows.pop(0)
    span = t-rows[0][0]
    if len(rows) < 3 or span < window_sec*.9:
        return dict(ready=False)
    slopes = [((b[1]-a[1])/(b[0]-a[0]), (b[2]-a[2])/(b[0]-a[0]))
              for i, a in enumerate(rows) for b in rows[i+1:]
              if b[0]-a[0] >= window_sec*.45]
    if not slopes:
        return dict(ready=False)
    center = (median(r[1] for r in rows), median(r[2] for r in rows))
    speed = math.hypot(median(s[0] for s in slopes), median(s[1] for s in slopes))
    radius = max(math.dist(center, r[1:3]) for r in rows)
    return dict(ready=True, position=list(center), speed_mps=speed, radius_m=radius,
                duration_sec=span, bounded=radius <= noise_m,
                max_gap_sec=max(b[0]-a[0] for a, b in zip(rows, rows[1:])))


class WindowedTravel:
    """Moving-median path with a retained anchor for slow motion and reversals.

    Sub-envelope motion remains unresolved, not proven noise. Persistent disagreement
    faults the caller; it is never a reason to silently discard coherent LIO motion.
    """

    def __init__(self, window_sec, noise_m):
        self.window_sec, self.noise_m = window_sec, noise_m
        self.reset()

    def reset(self):
        self.samples = deque()
        self.last = self.anchor = self.filtered = None
        self.last_emit = self.last_audit = None
        self.committed = self.raw = self.sport = 0.
        self.disagreement_windows = 0
        self.conflict = False

    @property
    def distance(self):
        return self.committed + (math.dist(self.anchor, self.filtered)
                                 if self.anchor is not None else 0.)

    def add(self, t, x, y, sport_speed=None):
        if self.last is not None:
            dt = t-self.last[0]
            if dt <= 0:
                return
            self.raw += math.dist((x, y), self.last[1:3])
            if sport_speed is not None:
                self.sport += sport_speed*dt
        self.last = (t, x, y)
        self.samples.append((t, x, y, sport_speed))
        while len(self.samples) > 2 and self.samples[1][0] < t-2*self.window_sec:
            self.samples.popleft()
        if self.anchor is None:
            self.anchor = self.filtered = (x, y)
            self.last_emit = self.last_audit = t
        if t-self.last_emit >= self.window_sec/2-1e-9:
            rows = [r for r in self.samples if r[0] >= t-self.window_sec]
            self.filtered = (median(r[1] for r in rows), median(r[2] for r in rows))
            delta = math.dist(self.anchor, self.filtered)
            if delta >= self.noise_m:
                self.committed += delta
                self.anchor = self.filtered
            self.last_emit = t
        # Compare accumulated temporal paths, not only a net chord: both a
        # real return leg and a repeated LIO oscillation must remain visible.
        # Sport never suppresses LIO mileage; disagreement latches a fault.
        if t-self.last_audit >= max(1., 4*self.window_sec)-1e-9:
            if all(r[3] is not None for r in self.samples):
                margin = 4*self.noise_m
                contradictory = abs(self.distance-self.sport) > margin
                self.disagreement_windows = self.disagreement_windows+1 if contradictory else 0
                self.conflict |= self.disagreement_windows >= 2
            self.last_audit = t

    def report(self):
        return dict(raw_path_m=self.raw, windowed_path_m=self.distance,
                    sport_integral_m=self.sport, window_sec=self.window_sec,
                    resolution_m=self.noise_m, disagreement_windows=self.disagreement_windows,
                    disagreement_distance_m=abs(self.distance-self.sport),
                    localization_untrusted=self.conflict,
                    unresolved_path_difference_m=max(0., self.raw-self.distance),
                    evidence='pose estimate and Sport comparison; no independent physical truth')
