#!/usr/bin/env python3
import os
import tempfile
import unittest

from sensor_time_bridge.imu_anchor import (
    IMU_ANCHOR_CLOCK_GPS,
    IMU_TIME_ANCHOR_MAGIC,
    IMU_TIME_ANCHOR_VERSION,
    ImuAnchorSnapshot,
    LivoxImuAnchorReader,
    LivoxImuTimeMapper,
    pack_anchor,
)
from uwb_serial_driver.parser import DistanceRoundAssembler, UwbParser


class UwbImuAnchorContextTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tempdir.name, "timeshare_imu")
        self.anchor_host_ns = 500_000_000_000
        anchor = ImuAnchorSnapshot(
            IMU_TIME_ANCHOR_MAGIC,
            IMU_TIME_ANCHOR_VERSION,
            99,
            2,
            1_577_836_800_000_000_000,
            self.anchor_host_ns,
            self.anchor_host_ns + 100,
            IMU_ANCHOR_CLOCK_GPS,
            True,
            10_000_000,
        )
        with open(self.path, "wb") as stream:
            stream.write(pack_anchor(anchor))

    def tearDown(self):
        self.tempdir.cleanup()

    def test_first_nonzero_distance_context_is_the_mapping_input(self):
        assembler = DistanceRoundAssembler(
            UwbParser(parser_mode="distance_round5"),
            lines_per_round=5,
            round_timeout_s=2.0,
        )
        assembler.process(
            "[UWBDBG] target=1 dist=99", context="debug", now_s=0.0
        )
        assembler.process("[TWR] Ra=1 Rb=2", context="twr", now_s=0.01)
        contexts = [
            self.anchor_host_ns + 1_000_000,
            self.anchor_host_ns + 3_200_000,
            self.anchor_host_ns + 4_000_000,
            self.anchor_host_ns + 5_000_000,
            self.anchor_host_ns + 6_000_000,
        ]
        lines = [
            "distance[0],0",
            "distance[1],1.405",
            "distance[2],2.3",
            "distance[0],0",
            "distance[0],0",
        ]
        event = None
        for index, line in enumerate(lines):
            event = assembler.process(
                line, context=contexts[index], now_s=0.1 + index * 0.01
            )

        self.assertEqual(
            DistanceRoundAssembler.ROUND_COMPLETE, event.kind
        )
        self.assertEqual(contexts[1], event.timestamp_context)
        mapper = LivoxImuTimeMapper(LivoxImuAnchorReader(self.path))
        mapped = mapper.map_time(event.timestamp_context)
        self.assertTrue(mapped.success)
        self.assertEqual(
            1_577_836_800_003_200_000, mapped.mapped_stamp_ns
        )

    def test_mapping_failure_keeps_protocol_round_but_has_no_fusion_time(self):
        missing = os.path.join(self.tempdir.name, "missing")
        mapper = LivoxImuTimeMapper(LivoxImuAnchorReader(missing))
        raw_lines = [
            "[UWBDBG] diag=1",
            "[TWR] Ra=1",
            "distance[0],0",
            "distance[1],1.2",
            "distance[0],0",
            "distance[0],0",
            "distance[0],0",
        ]
        assembler = DistanceRoundAssembler(
            UwbParser(parser_mode="distance_round5")
        )
        event = None
        for index, line in enumerate(raw_lines):
            event = assembler.process(
                line,
                context=self.anchor_host_ns + index,
                now_s=index * 0.01,
            )
        self.assertEqual(7, len(raw_lines))  # raw retention is independent
        self.assertEqual(
            DistanceRoundAssembler.ROUND_COMPLETE, event.kind
        )
        self.assertFalse(mapper.map_time(event.timestamp_context).success)


if __name__ == "__main__":
    unittest.main()
