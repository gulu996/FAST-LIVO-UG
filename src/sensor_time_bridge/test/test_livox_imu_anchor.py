#!/usr/bin/env python3
import mmap
import os
import struct
import tempfile
import threading
import time
import unittest

from sensor_time_bridge.imu_anchor import (
    IMU_ANCHOR_CLOCK_GPS,
    IMU_TIME_ANCHOR_FORMAT,
    IMU_TIME_ANCHOR_MAGIC,
    IMU_TIME_ANCHOR_SIZE,
    IMU_TIME_ANCHOR_VERSION,
    ImuAnchorSnapshot,
    LivoxImuAnchorReader,
    LivoxImuTimeMapper,
    decode_stable_snapshots,
    pack_anchor,
)


GOLDEN_HEX = (
    "494d413101000000"
    "0807060504030201"
    "1817161514131211"
    "2827262524232221"
    "3837363534333231"
    "4847464544434241"
    "02000000"
    "01000000"
    "5857565554535251"
)


def snapshot(
    *,
    epoch=7,
    sequence=2,
    imu_ns=1_000_000_000_000,
    host_ns=500_000_000_000,
    update_ns=500_000_000_100,
    source=IMU_ANCHOR_CLOCK_GPS,
    ready=True,
    uncertainty_ns=10_000_000,
):
    return ImuAnchorSnapshot(
        IMU_TIME_ANCHOR_MAGIC,
        IMU_TIME_ANCHOR_VERSION,
        epoch,
        sequence,
        imu_ns,
        host_ns,
        update_ns,
        source,
        ready,
        uncertainty_ns,
    )


def write_file(path, value):
    with open(path, "wb") as stream:
        stream.write(pack_anchor(value))


class SharedLayoutTest(unittest.TestCase):
    def test_layout_is_exactly_64_bytes_with_fixed_offsets(self):
        self.assertEqual(64, IMU_TIME_ANCHOR_SIZE)
        data = pack_anchor(snapshot())
        self.assertEqual(IMU_TIME_ANCHOR_MAGIC, struct.unpack_from("<I", data, 0)[0])
        self.assertEqual(IMU_TIME_ANCHOR_VERSION, struct.unpack_from("<I", data, 4)[0])
        self.assertEqual(7, struct.unpack_from("<Q", data, 8)[0])
        self.assertEqual(2, struct.unpack_from("<Q", data, 16)[0])
        self.assertEqual(1_000_000_000_000, struct.unpack_from("<Q", data, 24)[0])
        self.assertEqual(500_000_000_000, struct.unpack_from("<Q", data, 32)[0])
        self.assertEqual(500_000_000_100, struct.unpack_from("<Q", data, 40)[0])
        self.assertEqual(IMU_ANCHOR_CLOCK_GPS, struct.unpack_from("<I", data, 48)[0])
        self.assertEqual(1, struct.unpack_from("<I", data, 52)[0])
        self.assertEqual(10_000_000, struct.unpack_from("<Q", data, 56)[0])

    def test_cross_language_golden_bytes(self):
        golden = bytes.fromhex(GOLDEN_HEX)
        value = ImuAnchorSnapshot(
            IMU_TIME_ANCHOR_MAGIC,
            IMU_TIME_ANCHOR_VERSION,
            0x0102030405060708,
            0x1112131415161718,
            0x2122232425262728,
            0x3132333435363738,
            0x4142434445464748,
            2,
            True,
            0x5152535455565758,
        )
        self.assertEqual(golden, pack_anchor(value))
        result = decode_stable_snapshots(golden, golden)
        self.assertTrue(result.success)
        self.assertEqual(value, result.snapshot)

    def test_bad_magic_version_and_length_are_rejected(self):
        good = pack_anchor(snapshot())
        bad_magic = bytearray(good)
        struct.pack_into("<I", bad_magic, 0, 0)
        self.assertEqual(
            "bad_magic",
            decode_stable_snapshots(bytes(bad_magic), bytes(bad_magic)).failure_reason,
        )
        bad_version = bytearray(good)
        struct.pack_into("<I", bad_version, 4, 2)
        self.assertEqual(
            "bad_version",
            decode_stable_snapshots(
                bytes(bad_version), bytes(bad_version)
            ).failure_reason,
        )
        self.assertEqual(
            "bad_length", decode_stable_snapshots(good[:-1], good[:-1]).failure_reason
        )

    def test_odd_or_changed_sequence_is_never_read(self):
        odd = pack_anchor(snapshot(sequence=3))
        self.assertEqual(
            "inconsistent", decode_stable_snapshots(odd, odd).failure_reason
        )
        first = pack_anchor(snapshot(sequence=2))
        second = pack_anchor(snapshot(sequence=4, imu_ns=1_000_000_000_001))
        self.assertEqual(
            "inconsistent", decode_stable_snapshots(first, second).failure_reason
        )


class ReaderLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tempdir.name, "timeshare_imu")

    def tearDown(self):
        self.tempdir.cleanup()

    def test_missing_file_then_creation_recovers_without_reader_restart(self):
        reader = LivoxImuAnchorReader(self.path)
        self.assertEqual("missing", reader.read().failure_reason)
        write_file(self.path, snapshot())
        self.assertTrue(reader.read().success)
        reader.close()

    def test_ready_lifecycle_and_epoch_change(self):
        write_file(self.path, snapshot(ready=False))
        reader = LivoxImuAnchorReader(self.path)
        self.assertEqual("not_ready", reader.read().failure_reason)
        write_file(self.path, snapshot(sequence=4, ready=True))
        self.assertTrue(reader.read().success)
        write_file(self.path, snapshot(epoch=8, sequence=6, ready=True))
        self.assertEqual("epoch_changed", reader.read().failure_reason)
        self.assertEqual(1, reader.epoch_change_count)
        self.assertTrue(reader.read().success)
        write_file(self.path, snapshot(epoch=8, sequence=8, ready=False))
        self.assertEqual("not_ready", reader.read().failure_reason)
        reader.close()

    def test_replaced_mmap_is_reopened(self):
        write_file(self.path, snapshot(epoch=10))
        reader = LivoxImuAnchorReader(self.path)
        self.assertTrue(reader.read().success)
        replacement = self.path + ".new"
        write_file(replacement, snapshot(epoch=11, sequence=2))
        os.replace(replacement, self.path)
        self.assertEqual("epoch_changed", reader.read().failure_reason)
        self.assertTrue(reader.read().success)
        reader.close()

    def test_file_length_change_is_rejected_and_then_recovers(self):
        write_file(self.path, snapshot())
        reader = LivoxImuAnchorReader(self.path)
        self.assertTrue(reader.read().success)
        with open(self.path, "wb") as stream:
            stream.write(b"short")
        self.assertEqual("bad_length", reader.read().failure_reason)
        write_file(self.path, snapshot(sequence=4))
        self.assertTrue(reader.read().success)
        reader.close()

    def test_concurrent_writer_reader_never_returns_torn_snapshot(self):
        write_file(self.path, snapshot(imu_ns=10_000, host_ns=20_000))
        descriptor = os.open(self.path, os.O_RDWR)
        mapping = mmap.mmap(descriptor, IMU_TIME_ANCHOR_SIZE)
        stop = threading.Event()
        reader_observed_stable_write = threading.Event()

        def writer():
            sequence = 2
            for value in range(1, 5000):
                odd = sequence + 1
                candidate = snapshot(
                    sequence=odd,
                    imu_ns=10_000 + value,
                    host_ns=20_000 + value,
                    update_ns=30_000 + value,
                )
                mapping[:] = pack_anchor(candidate)
                sequence = odd + 1
                struct.pack_into("<Q", mapping, 16, sequence)
                if value == 1:
                    reader_observed_stable_write.wait(timeout=1.0)
                if value % 10 == 0:
                    time.sleep(0)
            stop.set()

        thread = threading.Thread(target=writer)
        reader = LivoxImuAnchorReader(self.path, max_attempts=32)
        thread.start()
        successful = 0
        while not stop.is_set():
            result = reader.read()
            if result.success:
                successful += 1
                reader_observed_stable_write.set()
                self.assertEqual(
                    result.snapshot.imu_stamp_ns - 10_000,
                    result.snapshot.host_monotonic_ns - 20_000,
                )
        thread.join()
        self.assertGreater(successful, 0)
        reader.close()
        mapping.close()
        os.close(descriptor)


class MappingFormulaTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tempdir.name, "timeshare_imu")

    def tearDown(self):
        self.tempdir.cleanup()

    def mapper(self, value=None):
        write_file(self.path, value or snapshot())
        return LivoxImuTimeMapper(
            LivoxImuAnchorReader(self.path), warn_age_s=0.020, max_age_s=0.100
        )

    def test_required_formula_and_zero_offset(self):
        mapper = self.mapper()
        result = mapper.map_time(500_003_200_000, 0)
        self.assertTrue(result.success)
        self.assertEqual(1_000_003_200_000, result.mapped_stamp_ns)
        self.assertEqual(3_200_000, result.anchor_delta_ns)

    def test_small_negative_delta_is_allowed(self):
        mapper = self.mapper()
        result = mapper.map_time(499_995_000_000)
        self.assertTrue(result.success)
        self.assertEqual(999_995_000_000, result.mapped_stamp_ns)
        self.assertEqual(-5_000_000, result.anchor_delta_ns)

    def test_warning_and_stale_thresholds(self):
        mapper = self.mapper()
        warning = mapper.map_time(500_021_000_000)
        self.assertTrue(warning.success)
        self.assertTrue(warning.warning)
        stale = mapper.map_time(500_100_000_001)
        self.assertFalse(stale.success)
        self.assertEqual("stale_anchor", stale.failure_reason)

    def test_offset_is_subtracted(self):
        mapper = self.mapper()
        result = mapper.map_time(500_003_200_000, 2_000_000)
        self.assertEqual(1_000_001_200_000, result.mapped_stamp_ns)

    def test_uint64_mapped_range_and_monotonic_guards(self):
        mapper = self.mapper()
        self.assertEqual(
            "invalid_receive_time",
            mapper.map_time(1 << 64).failure_reason,
        )
        low = self.mapper(snapshot(imu_ns=1, host_ns=100))
        self.assertEqual(
            "mapped_time_out_of_range", low.map_time(95).failure_reason
        )
        monotonic = self.mapper()
        self.assertTrue(monotonic.map_time(500_001_000_000).success)
        self.assertEqual(
            "nonmonotonic_mapped_time",
            monotonic.map_time(500_000_500_000).failure_reason,
        )

    def test_two_sensor_mappers_share_the_same_2020_epoch(self):
        epoch_2020_ns = 1_577_836_800_000_000_000
        write_file(
            self.path,
            snapshot(
                imu_ns=epoch_2020_ns,
                host_ns=500_000_000_000,
            ),
        )
        uwb = LivoxImuTimeMapper(LivoxImuAnchorReader(self.path))
        gnss = LivoxImuTimeMapper(LivoxImuAnchorReader(self.path))
        uwb_result = uwb.map_time(500_003_000_000)
        gnss_result = gnss.map_time(500_007_000_000)
        self.assertTrue(uwb_result.success)
        self.assertTrue(gnss_result.success)
        self.assertLess(abs(uwb_result.mapped_stamp_ns - epoch_2020_ns), 10_000_000)
        self.assertLess(abs(gnss_result.mapped_stamp_ns - epoch_2020_ns), 10_000_000)


if __name__ == "__main__":
    unittest.main()
