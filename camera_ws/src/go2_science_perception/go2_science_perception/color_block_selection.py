"""Select one measured block in MAP; retain its spatial identity until restart."""
import math


class BlockSelection:
    """A single-target compatibility boundary, never a stream of unrelated boxes."""

    def __init__(self, initial_radius=.20, update_radius=.15):
        self.initial_radius = initial_radius
        self.update_radius = update_radius
        self.initial = self.current = None

    def choose(self, points, camera_xy):
        """Return an index without changing the lock; ambiguous scenes fail closed."""
        if not points:
            return None, 'no_valid_block'
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            return None, 'nonfinite_point'
        # Two indistinguishable association regions must not create/switch identity.
        for i, left in enumerate(points):
            if any(math.dist(left[:2], right[:2]) <= 2*self.initial_radius
                   for right in points[i+1:]):
                return None, 'blocks_too_close_for_unique_association'
        if self.initial is None:
            return min(range(len(points)), key=lambda i: math.dist(points[i][:2], camera_xy)), 'nearest_valid_block'
        matches = [i for i, p in enumerate(points)
                   if math.dist(p[:2], self.initial[:2]) <= self.initial_radius
                   and math.dist(p[:2], self.current[:2]) <= self.update_radius]
        return (matches[0], 'same_spatial_block') if len(matches) == 1 else (None, 'locked_block_missing_or_ambiguous')

    def commit(self, point):
        """Commit only after exact-time MAP publication succeeds."""
        if self.initial is None:
            self.initial = tuple(point)
        self.current = tuple(point)
