#!/usr/bin/env python3
import os
import tempfile
import unittest

from gnss_serial_driver.parser import GnssParser
from gnss_serial_driver.time_policy import valid_for_fusion
from sensor_time_bridge.imu_anchor import (
    IMU_ANCHOR_CLOCK_GPS,
    IMU_TIME_ANCHOR_MAGIC,
    IMU_TIME_ANCHOR_VERSION,
    ImuAnchorSnapshot,
    LivoxImuAnchorReader,
    LivoxImuTimeMapper,
    pack_anchor,
)


def nmea(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return "${}*{:02X}".format(body, checksum)


class GnssImuAnchorTimeTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tempdir.name, "timeshare_imu")
        self.host_ns = 600_000_000_000
        anchor = ImuAnchorSnapshot(
            IMU_TIME_ANCHOR_MAGIC,
            IMU_TIME_ANCHOR_VERSION,
            123,
            2,
            1_577_836_800_000_000_000,
            self.host_ns,
            self.host_ns + 100,
            IMU_ANCHOR_CLOCK_GPS,
            True,
            10_000_000,
        )
        with open(self.path, "wb") as stream:
            stream.write(pack_anchor(anchor))

    def tearDown(self):
        self.tempdir.cleanup()

    def test_complete_checksum_valid_frame_uses_receive_context(self):
        line = nmea(
            "KSXT,20210906104914.00,120.15516000,30.27413000,16.0000,"
            "72.315,1.236,72.315,1.000,-0.418,4,4,12,12,0,0,0,"
            "0.1,0.0,0.0,50,50"
        )
        parsed = GnssParser().parse(line)[0]
        original_utc_ns = parsed.utc_ns
        self.assertTrue(parsed.checksum_valid)
        self.assertTrue(parsed.position_valid)

        mapper = LivoxImuTimeMapper(LivoxImuAnchorReader(self.path))
        mapped = mapper.map_time(self.host_ns + 7_000_000)
        self.assertTrue(mapped.success)
        self.assertEqual(
            1_577_836_800_007_000_000, mapped.mapped_stamp_ns
        )
        self.assertEqual(original_utc_ns, parsed.utc_ns)
        self.assertTrue(
            valid_for_fusion(
                mapped.success, parsed.position_valid, parsed.quality, {4}
            )
        )

    def test_checksum_failure_cannot_be_fusion_valid(self):
        line = nmea(
            "KSXT,20210906104914.00,120.15516000,30.27413000,16.0000,"
            "72.315,1.236,72.315,1.000,-0.418,4,4,12,12,0,0,0,"
            "0.1,0.0,0.0,50,50"
        )
        parsed = GnssParser().parse(line[:-2] + "00")[0]
        self.assertFalse(parsed.checksum_valid)
        self.assertFalse(
            parsed.checksum_valid
            and valid_for_fusion(
                True, parsed.position_valid, parsed.quality, {4}
            )
        )

    def test_missing_anchor_yields_zero_fusion_time_decision(self):
        mapper = LivoxImuTimeMapper(
            LivoxImuAnchorReader(os.path.join(self.tempdir.name, "missing"))
        )
        result = mapper.map_time(self.host_ns)
        self.assertFalse(result.success)
        self.assertEqual(0, result.mapped_stamp_ns)


if __name__ == "__main__":
    unittest.main()
