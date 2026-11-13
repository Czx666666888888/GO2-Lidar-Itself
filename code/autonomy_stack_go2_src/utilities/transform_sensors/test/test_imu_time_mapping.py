"""Small pure-logic boundary checks; no ROS initialization or hardware access."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'imu_time_mapping', Path(__file__).resolve().parents[1]/'transform_sensors/imu_time_mapping.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ImuTimeMapping = module.ImuTimeMapping


class ClockMappingChecks(unittest.TestCase):
    def initialized(self):
        mapping = ImuTimeMapping()
        # 250 Hz, constant unknown one-way delay 2 ms. This fixture checks
        # arithmetic and lifecycle, not whether hardware delay is really 2 ms.
        for i in range(502):
            stamp = 100_000_000_000+i*4_000_000
            mapping.observe_imu(stamp, stamp+9_000,
                                stamp+3_390_496_000_000_000+2_000_000,
                                i*4_000_000)
        self.assertTrue(mapping.ready)
        return mapping

    def test_initialization_and_shared_fixed_offset(self):
        mapping = ImuTimeMapping()
        self.assertIsNone(mapping.map_stamp('imu', 100, 200))
        mapping = self.initialized()
        raw = 103_000_000_000
        local = raw+mapping.offset_ns
        self.assertEqual(mapping.map_stamp('imu', raw, local+1_000_000), local)
        self.assertEqual(mapping.map_stamp('cloud', raw-64_000_000, local+1_000_000,
                                           64_000_000), local-64_000_000)
        self.assertEqual(mapping.offset_ns, 3_390_496_002_000_000)

    def test_duplicate_does_not_move_stamp_forward(self):
        mapping = self.initialized()
        raw = 103_000_000_000
        local = raw+mapping.offset_ns
        self.assertEqual(mapping.map_stamp('imu', raw, local), local)
        self.assertIsNone(mapping.map_stamp('imu', raw, local))
        self.assertTrue(mapping.ready)

    def test_scan_end_guard_not_only_begin(self):
        mapping = self.initialized()
        raw = 103_000_000_000
        local = raw+mapping.offset_ns
        self.assertIsNone(mapping.map_stamp('cloud', raw, local, 101_000_000))
        self.assertFalse(mapping.ready)

    def test_mapping_frozen_and_local_clock_step_faults(self):
        mapping = self.initialized()
        offset = mapping.offset_ns
        stamp = 102_008_000_000
        mapping.observe_imu(stamp, stamp+9_000, stamp+offset+1_000_000, 2_008_000_000)
        self.assertEqual(mapping.offset_ns, offset)
        mapping.observe_imu(stamp+4_000_000, stamp+4_009_000,
                            stamp+offset+104_000_000, 2_012_000_000)
        self.assertFalse(mapping.ready)
        self.assertIn('clock stepped', mapping.fault)

    def test_inconsistent_dds_epoch_cannot_initialize(self):
        mapping = ImuTimeMapping()
        for i in range(600):
            stamp = 100_000_000_000+i*4_000_000
            mapping.observe_imu(stamp, stamp+3_390_496_000_000_000,
                                stamp+3_390_496_002_000_000, i*4_000_000)
        self.assertFalse(mapping.ready)
        self.assertEqual(mapping.status()['dds_metadata_samples'], 0)


if __name__ == '__main__':
    unittest.main()
