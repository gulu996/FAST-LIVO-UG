#!/usr/bin/env python3
import mmap
import os
import struct
import threading
import time
import unittest

import rospy
import rostest
from gnss_serial_driver.msg import GnssPvtStamped, GnssStatus
from nav_msgs.msg import Odometry
from sensor_time_bridge.imu_anchor import (
    IMU_ANCHOR_CLOCK_GPS,
    IMU_TIME_ANCHOR_MAGIC,
    IMU_TIME_ANCHOR_SIZE,
    IMU_TIME_ANCHOR_VERSION,
    ImuAnchorSnapshot,
    pack_anchor,
)
from sensor_time_msgs.msg import RawSerialFrame
from uwb_serial_driver.msg import UwbRangeArray, UwbStatus


ANCHOR_PATH = "/tmp/livox_imu_anchor_mapping_integration.bin"
DEVICE_EPOCH_NS = 1_577_836_800_000_000_000  # 2020-01-01 UTC-like epoch


class TestAnchorWriter:
    def __init__(self):
        if os.path.exists(ANCHOR_PATH):
            os.unlink(ANCHOR_PATH)
        self.fd = os.open(ANCHOR_PATH, os.O_CREAT | os.O_RDWR, 0o600)
        os.ftruncate(self.fd, IMU_TIME_ANCHOR_SIZE)
        self.mapping = mmap.mmap(self.fd, IMU_TIME_ANCHOR_SIZE)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.active = True
        self.epoch = 1001
        self.sequence = 0
        self.samples = 0
        self.host_base_ns = time.monotonic_ns()
        self.device_base_ns = DEVICE_EPOCH_NS
        self._write_locked(0, 0, False)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _write_locked(self, imu_ns, host_ns, ready):
        self.sequence += 1
        odd = self.sequence
        value = ImuAnchorSnapshot(
            IMU_TIME_ANCHOR_MAGIC,
            IMU_TIME_ANCHOR_VERSION,
            self.epoch,
            odd,
            imu_ns,
            host_ns,
            time.monotonic_ns(),
            IMU_ANCHOR_CLOCK_GPS if imu_ns else 0,
            ready,
            10_000_000,
        )
        self.mapping[:] = pack_anchor(value)
        self.sequence += 1
        struct.pack_into("<Q", self.mapping, 16, self.sequence)

    def _run(self):
        while not self.stop_event.wait(0.005):
            with self.lock:
                if not self.active:
                    continue
                host_ns = time.monotonic_ns()
                imu_ns = self.device_base_ns + (
                    host_ns - self.host_base_ns
                )
                self.samples += 1
                self._write_locked(imu_ns, host_ns, self.samples >= 3)

    def pause_without_invalidation(self):
        with self.lock:
            self.active = False

    def restart_with_new_epoch(self):
        with self.lock:
            now_ns = time.monotonic_ns()
            previous_device_ns = self.device_base_ns + (
                now_ns - self.host_base_ns
            )
            self.epoch += 1
            self.samples = 0
            self.host_base_ns = now_ns
            self.device_base_ns = previous_device_ns + 1_000_000_000
            self._write_locked(0, 0, False)
            self.active = True

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        with self.lock:
            self._write_locked(0, 0, False)
        self.mapping.close()
        os.close(self.fd)
        if os.path.exists(ANCHOR_PATH):
            os.unlink(ANCHOR_PATH)


class ImuAnchorMappingIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.uwb_raw = []
        self.uwb_ranges = []
        self.gnss_pvt = []
        self.gnss_enu = []
        self.uwb_status = None
        self.gnss_status = None
        self.anchor_writer = TestAnchorWriter()
        self.subscribers = [
            rospy.Subscriber(
                "/uwb/raw", RawSerialFrame, self._append_uwb_raw, queue_size=500
            ),
            rospy.Subscriber(
                "/uwb/ranges",
                UwbRangeArray,
                self._append_uwb_range,
                queue_size=100,
            ),
            rospy.Subscriber(
                "/gnss/pvt_local",
                GnssPvtStamped,
                self._append_gnss_pvt,
                queue_size=100,
            ),
            rospy.Subscriber(
                "/gnss/enu_odom",
                Odometry,
                self._append_gnss_enu,
                queue_size=100,
            ),
            rospy.Subscriber(
                "/uwb/status", UwbStatus, self._set_uwb_status, queue_size=20
            ),
            rospy.Subscriber(
                "/gnss/status_v2",
                GnssStatus,
                self._set_gnss_status,
                queue_size=20,
            ),
        ]

    def tearDown(self):
        self.anchor_writer.close()

    def _append_uwb_raw(self, message):
        with self.lock:
            self.uwb_raw.append(message)

    def _append_uwb_range(self, message):
        with self.lock:
            self.uwb_ranges.append(message)

    def _append_gnss_pvt(self, message):
        with self.lock:
            self.gnss_pvt.append(message)

    def _append_gnss_enu(self, message):
        with self.lock:
            self.gnss_enu.append(message)

    def _set_uwb_status(self, message):
        with self.lock:
            self.uwb_status = message

    def _set_gnss_status(self, message):
        with self.lock:
            self.gnss_status = message

    def _wait(self, predicate, detail, timeout=12.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if predicate():
                    return
            time.sleep(0.02)
        with self.lock:
            self.fail("{}: raw={} uwb={} pvt={} enu={}".format(
                detail,
                len(self.uwb_raw),
                len(self.uwb_ranges),
                len(self.gnss_pvt),
                len(self.gnss_enu),
            ))

    @staticmethod
    def _strictly_increasing(messages):
        stamps = [message.header.stamp.to_nsec() for message in messages]
        return all(current > previous for previous, current in zip(stamps, stamps[1:]))

    def test_shared_anchor_stale_and_epoch_recovery(self):
        self._wait(
            lambda: (
                len(self.uwb_ranges) >= 4
                and len(self.gnss_pvt) >= 4
                and len(self.gnss_enu) >= 4
                and self.uwb_status is not None
                and self.gnss_status is not None
            ),
            "initial mapped output not ready",
        )
        with self.lock:
            initial_uwb = list(self.uwb_ranges)
            initial_pvt = list(self.gnss_pvt)
            initial_enu = list(self.gnss_enu)
            uwb_status = self.uwb_status
            gnss_status = self.gnss_status

        self.assertTrue(self._strictly_increasing(initial_uwb))
        self.assertTrue(self._strictly_increasing(initial_pvt))
        self.assertTrue(self._strictly_increasing(initial_enu))
        for message in initial_uwb:
            self.assertEqual(
                UwbRangeArray.DEVICE_TIME_MAPPED_LOCAL,
                message.timestamp_source,
            )
            self.assertLess(
                abs(message.header.stamp.to_nsec() - DEVICE_EPOCH_NS),
                10_000_000_000,
            )
        for message in initial_pvt:
            self.assertTrue(message.local_measurement_time_valid)
            self.assertTrue(message.valid_for_fusion)
            self.assertLess(
                abs(message.header.stamp.to_nsec() - DEVICE_EPOCH_NS),
                10_000_000_000,
            )
        pvt_stamps = {
            message.header.stamp.to_nsec() for message in initial_pvt
        }
        self.assertTrue(all(
            message.header.stamp.to_nsec() in pvt_stamps
            for message in initial_enu
        ))
        self.assertIn(
            "timestamp_mode=livox_imu_anchor", uwb_status.detail
        )
        self.assertIn(
            "timestamp_mode=livox_imu_anchor", gnss_status.detail
        )

        self.anchor_writer.pause_without_invalidation()
        time.sleep(0.25)
        with self.lock:
            stale_uwb_count = len(self.uwb_ranges)
            stale_pvt_count = len(self.gnss_pvt)
            stale_raw_count = len(self.uwb_raw)
        time.sleep(0.30)
        with self.lock:
            self.assertGreater(len(self.uwb_raw), stale_raw_count)
            self.assertEqual(stale_uwb_count, len(self.uwb_ranges))
            self.assertEqual(stale_pvt_count, len(self.gnss_pvt))
            self.assertTrue(any(
                message.header.stamp.to_nsec() == 0
                for message in self.uwb_raw[-30:]
            ))

        self.anchor_writer.restart_with_new_epoch()
        self._wait(
            lambda: (
                len(self.uwb_ranges) > stale_uwb_count
                and len(self.gnss_pvt) > stale_pvt_count
                and self.uwb_status is not None
                and "epoch_change=1" in self.uwb_status.detail
                and self.gnss_status is not None
                and "epoch_change=1" in self.gnss_status.detail
            ),
            "new writer epoch did not recover",
        )
        with self.lock:
            self.assertTrue(self._strictly_increasing(self.uwb_ranges))
            self.assertTrue(self._strictly_increasing(self.gnss_pvt))
            self.assertTrue(self._strictly_increasing(self.gnss_enu))


if __name__ == "__main__":
    rospy.init_node("imu_anchor_mapping_integration_test")
    rostest.rosrun(
        "sensor_recording_bringup",
        "imu_anchor_mapping_integration",
        ImuAnchorMappingIntegrationTest,
    )
