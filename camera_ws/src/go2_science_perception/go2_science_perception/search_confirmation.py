"""Bounded post-stop RGB-D recheck using a frozen, unique SEARCH candidate."""

from collections import deque
import math


def spread(samples):
    """Return maximum MAP XY separation of source observations."""
    return max((math.dist(a[2], b[2]) for a in samples for b in samples), default=0.0)


class FastConfirmation:
    """Keep target evidence across motion jitter; never lock before measured stop."""

    def __init__(self, settings, candidate, history, confirmed, started):
        self.p, self.candidate, self.started = settings, candidate, started
        self.history = list(history)
        self.reliable = (confirmed and len(history) >= 2 and
                         spread(history) < settings.anchor_max_spread)
        self.stationary_since = self.settled_at = None
        self.samples = deque()
        self.last_stamp = candidate['trigger_stamp_ns']
        self.extended = False

    def step(self, now, stopped, tracks):
        """Return diagnostics and a lock/search/fault/wait decision, with zero motion implied."""
        p = self.p
        if not stopped:
            self.stationary_since = self.settled_at = None
        elif self.stationary_since is None:
            self.stationary_since = now
        elif self.settled_at is None and now-self.stationary_since >= p.stop_confirm_stable_sec:
            self.settled_at = now
        # Histories survive a quiet-state interruption, but only samples after the
        # latest completed 0.2s quiet interval can authorize a lock.
        while self.samples and now-self.samples[0][0] > p.anchor_window_sec:
            self.samples.popleft()
        matches = [t for t in tracks if math.dist(t['latest_observation'][:2],
                   self.candidate['map']) <= min(p.anchor_association_radius, p.target_jump_m)]
        unique = len(matches) == 1
        current = matches[0] if unique else None
        if len(matches) > 1:
            self.samples.clear()  # Never carry a fast lock through association ambiguity.
        if current and self.settled_at is not None:
            stamp, observed = current['observed_ns'], now-current['age']
            if stamp > self.last_stamp and observed > self.settled_at:
                self.last_stamp = stamp
                if self.samples and (stamp-self.samples[-1][1])*1e-9 > p.stop_confirm_fast_gap_sec:
                    self.samples.clear()
                self.samples.append((observed, stamp, tuple(current['latest_observation'][:2]),
                                     current['id'], current['camera_range_m']))
        samples = [s for s in self.samples if self.settled_at is not None and
                   s[0] > self.settled_at and current and s[3] == current['id']]
        extent = spread(samples)
        span = (samples[-1][1]-samples[0][1])*1e-9 if samples else 0.0
        fast = self.reliable and current is not None and current['id'] == self.candidate['id']
        fast_ok = fast and len(samples) >= p.stop_confirm_fast_observations
        full_ok = (current is not None and current['state'] == 'CONFIRMED' and
                   len(samples) >= p.anchor_min_observations and
                   span >= p.stop_confirm_min_span_sec)
        blockers = []
        if not stopped or self.settled_at is None:
            blockers.append('parking_unstable')
        if len(matches) > 1:
            blockers.append('target_ambiguous')
        elif not tracks:
            blockers.append('target_lost')
        elif not unique:
            blockers.append('candidate_position_inconsistent')
        if extent >= p.anchor_max_spread:
            blockers.append('position_spread_excess')
        if not (fast_ok or full_ok):
            blockers.append('observations_insufficient')
        fresh_last = bool(current and samples and current['observed_ns'] == samples[-1][1])
        if not fresh_last and 'observations_insufficient' not in blockers:
            blockers.append('observations_insufficient')
        report = dict(action='wait', confirmation_path='fast' if fast else 'full_fallback',
                      confirmation_mode='fast_stationary', stop_confirm_reason=(
                          blockers[0] if blockers else 'ready'), confirmation_blockers=blockers,
                      confirmation_sample_count=len(samples), confirmation_sample_span=span,
                      confirmation_spread=extent, stationary_sample_count=len(samples),
                      stationary_sample_span=span, stationary_spread=extent,
                      stationary_since=self.stationary_since, settled_at=self.settled_at,
                      stopped_after_sec=(self.settled_at-self.started
                                         if self.settled_at is not None else None),
                      stop_stable_range_m=(current['camera_range_m']
                                           if current and self.settled_at is not None else None),
                      in_place_recheck=self.extended, samples=samples,
                      stationary_observations=[dict(observed_ns=s[1], map=s[2], id=s[3],
                                                    camera_range_m=s[4]) for s in samples])
        deadline = p.stop_confirm_timeout + (p.stop_confirm_recheck_sec if self.extended else 0)
        if not blockers and fresh_last and now-self.started < deadline:
            report['action'] = 'lock'
            return report
        if not blockers and now-self.started >= deadline:
            report['stop_confirm_reason'] = 'confirmation_deadline_exceeded'
            blockers.append('confirmation_deadline_exceeded')
        if now-self.started >= deadline:
            # One bounded stationary retry, only for a fresh, uniquely associated
            # target with reliable pre-stop evidence or confirmed current evidence.
            visible_stable = bool(current and extent < p.anchor_max_spread and
                                  (fast or current['state'] == 'CONFIRMED'))
            report['timeout_event'] = True
            if not self.extended and visible_stable:
                self.extended = True
                report.update(timeout_result='IN_PLACE_RECHECK', in_place_recheck=True)
            else:
                report.update(timeout_result='TIMEOUT',
                              action='fault' if visible_stable else 'search')
        return report
