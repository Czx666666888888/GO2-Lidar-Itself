"""Select one measured block in MAP; retain its spatial identity within each completed-task round."""
import math

CLOSE_BLOCK_DISTANCE_M = .10
CLOSE_PAIR_MIN_FRAMES = 3
MAX_FRAME_GAP_NS = 300_000_000


class CloseBlockFilter:
    """Require consecutive co-observations before publishing a close pair."""

    def __init__(self):
        """Start with an empty frame history."""
        self.stamp = 0
        self.points = []
        self.support = []
        self.pairs = {}

    def update(self, stamp, points, confidences):
        """Return the indices of MAP measurements to publish for this frame."""
        if stamp <= self.stamp:
            return []
        previous = self.points if stamp - self.stamp <= MAX_FRAME_GAP_NS else []
        matches = {}
        used = set()
        distances = sorted(
            (math.dist(point[:2], old[:2]), i, j)
            for i, point in enumerate(points) for j, old in enumerate(previous)
        )
        for distance, i, j in distances:
            if distance >= CLOSE_BLOCK_DISTANCE_M:
                break
            if i not in matches and j not in used:
                matches[i] = j
                used.add(j)

        support = [self.support[matches[i]] + 1 if i in matches else 1
                   for i in range(len(points))]
        pairs = {}
        for i, point in enumerate(points):
            for j in range(i):
                if math.dist(point[:2], points[j][:2]) < CLOSE_BLOCK_DISTANCE_M:
                    prior = tuple(sorted((matches.get(i, -1), matches.get(j, -1))))
                    pairs[j, i] = self.pairs.get(prior, 0) + 1

        kept = []
        ranked = sorted(range(len(points)),
                        key=lambda i: (support[i], confidences[i]), reverse=True)
        for i in ranked:
            if all(pairs.get(tuple(sorted((i, j))), CLOSE_PAIR_MIN_FRAMES)
                   >= CLOSE_PAIR_MIN_FRAMES for j in kept):
                kept.append(i)
        self.stamp, self.points, self.support, self.pairs = stamp, points, support, pairs
        return sorted(kept)


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
