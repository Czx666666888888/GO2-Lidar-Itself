"""Select one measured block in MAP; retain its spatial identity within each completed-task round."""
import math


class BlockSelection:
    """A single-target compatibility boundary, never a stream of unrelated boxes."""

    def __init__(self, initial_radius=.20, update_radius=.15):
        self.initial_radius = initial_radius
        self.update_radius = update_radius
        self.initial = self.current = None
        self.completed = []

    def complete_round(self, points):
        """Advance one successful round; repeats are inert and history cannot shrink."""
        if (not isinstance(points, list) or any(
                not isinstance(p, (list, tuple)) or len(p) != 2
                or any(isinstance(v, bool) or not isinstance(v, (float, int))
                       or not math.isfinite(v) for v in p) for p in points)):
            raise ValueError('completed targets must be finite MAP XY pairs')
        completed = [tuple(p) for p in points]
        if completed == self.completed:
            return False
        if (len(completed) != len(self.completed)+1
                or completed[:-1] != self.completed):
            raise ValueError('completed targets must append exactly one successful round')
        self.completed = completed
        self.initial = self.current = None
        return True

    def choose(self, points, camera_xy):
        """Return an index without changing the lock; ambiguous scenes fail closed."""
        if not points:
            return None, 'no_valid_block'
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            return None, 'nonfinite_point'
        eligible = [i for i, p in enumerate(points)
                    if not any(math.dist(p[:2], done) < .15 for done in self.completed)]
        if not eligible:
            return None, 'all_blocks_completed'
        # Two indistinguishable association regions must not create/switch identity.
        for offset, i in enumerate(eligible):
            left = points[i]
            if any(math.dist(left[:2], right[:2]) <= 2*self.initial_radius
                   for right in (points[j] for j in eligible[offset+1:])):
                return None, 'blocks_too_close_for_unique_association'
        if self.initial is None:
            return min(eligible, key=lambda i: math.dist(points[i][:2], camera_xy)), 'nearest_valid_block'
        matches = [i for i in eligible
                   if math.dist(points[i][:2], self.initial[:2]) <= self.initial_radius
                   and math.dist(points[i][:2], self.current[:2]) <= self.update_radius]
        return (matches[0], 'same_spatial_block') if len(matches) == 1 else (None, 'locked_block_missing_or_ambiguous')

    def commit(self, point):
        """Commit only after exact-time MAP publication succeeds."""
        if self.initial is None:
            self.initial = tuple(point)
        self.current = tuple(point)
