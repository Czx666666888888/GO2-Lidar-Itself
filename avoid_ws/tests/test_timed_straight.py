from dataclasses import replace
import unittest

import numpy as np

from go2_reactive_avoid.controller import Direction, Scan, STOP
from go2_reactive_avoid.lidar import HALF_LENGTH, TIMED_RANGE_EDGES, timed_clearance
from go2_reactive_avoid.timed_straight import SPEED_CAP, TimedStraight


CLEAR = Scan(*(Direction.CLEAR,) * 4, can_rotate=True, body_clear=True, timed_ground_coverage=2.5)


class TimedStraightTest(unittest.TestCase):
    def step(self, now, *, scan=CLEAR, yaw=0.0, still=True, standing=True, observed=None, speed=0.0):
        return self.segment.update(now, now if observed is None else observed,
                                   scan, yaw, still, standing, speed)

    def setUp(self):
        self.segment = TimedStraight(1.0, None)

    def test_confirmation_delay_heading_and_monotonic_deadline(self):
        self.assertEqual(self.step(6), STOP)
        self.segment.confirmed_at = 7
        self.assertEqual(self.step(7.999), STOP)
        self.assertEqual(self.step(8, yaw=0.03), (SPEED_CAP, 0.0))
        self.assertLess(self.step(8.5, yaw=0.08, still=False)[1], 0)
        self.assertEqual(self.step(9, still=False), STOP)
        self.assertEqual(self.segment.phase, "SETTLE")
        self.step(9.1, observed=9.0)
        self.step(9.6, observed=9.0)
        self.assertEqual(self.segment.phase, "SETTLE")
        self.step(9.7)
        self.step(10.25)
        self.assertEqual(self.segment.phase, "DONE")

    def test_no_scan_is_required_and_lost_standing_stops(self):
        self.segment.confirmed_at = 1
        self.assertEqual(self.segment.preflight_issue(None, True, True), "")
        self.assertEqual(self.step(2, scan=None), (SPEED_CAP, 0.0))
        self.assertEqual(self.step(2.1, scan=None, standing=False), STOP)
        self.assertEqual(self.segment.phase, "HALTED")

    def test_halts_when_heading_diverges_or_robot_does_not_stop(self):
        self.segment.confirmed_at = 1
        self.step(2)
        self.assertEqual(self.step(2.1, yaw=0.3, still=False), STOP)
        self.assertEqual(self.segment.phase, "HALTED")
        self.segment = TimedStraight(1.0, None)
        self.segment.confirmed_at = 1
        self.step(2)
        self.step(3, still=False)
        self.step(6, still=False)
        self.assertEqual(self.segment.phase, "HALTED")

    def test_rejects_invalid_duration(self):
        for seconds in (0, -1, float("nan"), float("inf"), 10.01):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                TimedStraight(seconds, None)

    def test_corridor_coverage_limits_distance_and_detects_far_obstacle(self):
        x, y = np.array([(HALF_LENGTH + gap, ordinate)
                         for edge in TIMED_RANGE_EDGES[:-1]
                         for gap in (edge + 0.1, edge + 0.3)
                         for ordinate in (-0.33, 0.0, 0.33)]).T
        floor, obstacles = np.ones_like(x, dtype=bool), np.zeros_like(x, dtype=bool)
        self.assertGreater(min(timed_clearance(x, y, floor, obstacles)), 1.95)
        index = np.flatnonzero((x > HALF_LENGTH + 1.5) & (y == 0))[0]
        floor[index], obstacles[index] = False, True
        self.assertLessEqual(min(timed_clearance(x, y, floor, obstacles)), x[index] - HALF_LENGTH)
        floor[index], obstacles[index] = True, False
        floor[(x > HALF_LENGTH + 1.4) & (y > 0.2)] = False
        self.assertLess(min(timed_clearance(x, y, floor, obstacles)), 1.51)

    def test_lidar_reports_do_not_change_timed_command(self):
        scan = replace(CLEAR, front=Direction.BLOCKED, timed_front_obstacle=.002,
                       timed_ground_coverage=0.0, body_clear=False, right_near=True)
        self.assertEqual(self.segment.preflight_issue(scan, True, True), "")
        self.segment.confirmed_at = 1
        self.assertEqual(self.step(2, scan=scan), (SPEED_CAP, 0.0))
