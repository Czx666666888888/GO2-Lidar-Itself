import math
import unittest

from go2_reactive_avoid.controller import STOP
from go2_reactive_avoid.entrance import Entrance


class EntranceTest(unittest.TestCase):
    def setUp(self):
        self.controller = Entrance(50, 30)

    def step(self, now, xy=(2.0, 3.0), yaw=90, *, still=True, observed=None):
        return self.controller.update(now, now if observed is None else observed,
                                      math.radians(yaw), still, xy)

    def test_forward_projection_then_clockwise_turn_and_two_stops(self):
        self.assertGreater(self.step(0.0)[0], 0.0)
        self.assertLess(self.step(1.0, (2.0, 3.3), 94)[1], 0.0)
        self.assertEqual(self.step(2.0, (2.0, 3.49), still=False), STOP)
        self.step(2.25, (2.0, 3.50))
        self.step(2.90, (2.0, 3.50), observed=2.25)
        self.assertEqual(self.controller.phase, "DRIVE")
        self.assertEqual(self.step(3.0, (2.0, 3.50)), STOP)
        self.assertEqual(self.controller.phase, "TURN")
        vx, wz = self.step(3.1, (2.0, 3.50))
        self.assertEqual(vx, 0.0)
        self.assertLess(wz, 0.0)
        self.assertEqual(self.step(4.0, (2.0, 3.50), 60), STOP)
        self.assertEqual(self.controller.phase, "TURN")
        self.assertEqual(self.step(4.6, (2.0, 3.50), 60), STOP)
        self.assertEqual(self.controller.phase, "DONE")

    def test_cross_track_error_and_overshoot_abort(self):
        self.step(0.0)
        self.assertEqual(self.step(1.0, (2.10, 3.15)), STOP)
        self.assertEqual(self.controller.phase, "HALTED")
        self.controller = Entrance(50, 0)
        self.step(0.0)
        self.assertEqual(self.step(2.0, (2.0, 3.55)), STOP)
        self.assertEqual(self.controller.phase, "HALTED")

    def test_turn_only_counterclockwise_and_position_drift_guard(self):
        self.controller = Entrance(0, -30)
        self.assertEqual(self.step(0.0), STOP)
        self.assertEqual(self.step(0.6), STOP)
        self.assertGreater(self.step(0.7)[1], 0.0)
        self.assertEqual(self.step(0.8, (2.0, 3.09)), STOP)
        self.assertEqual(self.controller.phase, "HALTED")

    def test_distance_only_and_missing_odom_progress(self):
        self.controller = Entrance(50, 0)
        self.step(0.0)
        self.step(2.0, (2.0, 3.49))
        self.assertEqual(self.step(2.6, (2.0, 3.49)), STOP)
        self.assertEqual(self.controller.phase, "DONE")
        self.controller = Entrance(50, 30)
        self.step(0.0)
        self.assertEqual(self.step(11.1), STOP)
        self.assertEqual(self.controller.phase, "HALTED")
