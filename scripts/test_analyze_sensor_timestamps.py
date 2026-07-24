#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import struct
import unittest


MODULE_PATH = Path(__file__).with_name("analyze_sensor_timestamps.py")
SPEC = importlib.util.spec_from_file_location("timestamp_analysis", MODULE_PATH)
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


class TimestampAnalysisTest(unittest.TestCase):
    def test_reads_serialized_header_without_payload_deserialization(self):
        raw_message = (
            "sensor_msgs/Image",
            struct.pack("<III", 7, 123, 456) + b"payload",
            "md5",
            (0, 0),
            object,
        )
        self.assertEqual(
            (7, 123_000_000_456),
            ANALYSIS.serialized_header(raw_message))

    def test_detects_startup_change_point(self):
        stamps = [
            1_000_000_000,
            2_400_000_000,
            2_500_000_000,
            2_600_000_000,
            2_700_000_000,
        ]
        anomalies, startup, stable_start = ANALYSIS.detect_change_points(stamps)
        self.assertEqual([0], anomalies)
        self.assertEqual([0], startup)
        self.assertEqual(1, stable_start)

    def test_exact_matching_reports_contiguous_indices(self):
        camera = [10, 20, 100, 200, 300, 400]
        lidar = [100, 200, 300, 400, 500]
        matches, longest = ANALYSIS.exact_timestamp_matches(camera, lidar)
        self.assertEqual(4, len(matches))
        self.assertEqual((2, 5, 0, 3, 4), longest)

    def test_robust_fit_rejects_one_startup_step(self):
        bag = [index * 100_000_000 for index in range(100)]
        header = list(bag)
        header[1:] = [value + 1_300_000_000 for value in header[1:]]
        _, slope = ANALYSIS.robust_linear_fit(bag[1:], header[1:])
        self.assertAlmostEqual(1.0, slope, places=12)

    def test_sequence_gap_count(self):
        self.assertEqual(3, ANALYSIS.sequence_gap_count([10, 11, 15, 16]))


if __name__ == "__main__":
    unittest.main()
