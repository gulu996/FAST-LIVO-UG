#!/usr/bin/env python3
import re
import threading
import time
import unittest

import rospy
import rostest
from sensor_time_msgs.msg import RawSerialFrame
from sensor_time_msgs.srv import (
    HostMonotonicToLocal,
    HostMonotonicToLocalResponse,
)
from uwb_serial_driver.msg import UwbRangeArray, UwbStatus


DISTANCE_PATTERN = re.compile(
    r"^distance\s*\[\s*(\d+)\s*\]\s*,\s*"
    r"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)"
)


class DistanceRound5IntegrationTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.raw_messages = []
        self.range_messages = []
        self.status = None
        self.service = rospy.Service(
            "/sensor_time/host_monotonic_to_local",
            HostMonotonicToLocal,
            self._convert_host_time,
        )
        self.raw_subscriber = rospy.Subscriber(
            "/uwb/raw", RawSerialFrame, self._raw_callback, queue_size=100
        )
        self.range_subscriber = rospy.Subscriber(
            "/uwb/ranges",
            UwbRangeArray,
            self._range_callback,
            queue_size=50,
        )
        self.status_subscriber = rospy.Subscriber(
            "/uwb/status", UwbStatus, self._status_callback, queue_size=10
        )

    @staticmethod
    def _convert_host_time(request):
        response = HostMonotonicToLocalResponse()
        response.success = True
        response.local_stamp_ns = request.host_monotonic_ns
        response.session_id = 777
        response.writer_epoch = 888
        response.time_uncertainty_ns = 1234
        response.detail = "test mapping"
        return response

    def _raw_callback(self, message):
        with self.lock:
            self.raw_messages.append(message)

    def _range_callback(self, message):
        with self.lock:
            self.range_messages.append(message)

    def _status_callback(self, message):
        with self.lock:
            self.status = message

    def _wait_for_completion(self, timeout=12.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if (
                    self.status is not None
                    and self.status.raw_line_count == 49
                    and self.status.parsed_round_count == 6
                    and len(self.raw_messages) == 49
                    and len(self.range_messages) == 6
                ):
                    return
            time.sleep(0.02)
        with self.lock:
            raise AssertionError(
                "replay incomplete: raw={} ranges={} status={}".format(
                    len(self.raw_messages),
                    len(self.range_messages),
                    None
                    if self.status is None
                    else (
                        self.status.raw_line_count,
                        self.status.parsed_round_count,
                        self.status.parse_error_count,
                    ),
                )
            )

    def test_file_replay_publishes_strict_aggregated_rounds(self):
        self._wait_for_completion()
        with self.lock:
            raw_messages = sorted(
                self.raw_messages, key=lambda message: message.source_sequence
            )
            ranges = list(self.range_messages)
            status = self.status

        raw_text = [
            bytes(bytearray(message.data)).decode("utf-8")
            for message in raw_messages
        ]
        self.assertEqual(49, len(raw_text))
        self.assertEqual(7, sum(line.startswith("[UWBDBG]") for line in raw_text))
        self.assertEqual(7, sum(line.startswith("[TWR]") for line in raw_text))
        self.assertEqual(35, sum(line.startswith("distance[") for line in raw_text))

        expected_distances = [1.405, 1.362, 1.391, 1.374, 1.308, 1.342]
        self.assertEqual(6, len(ranges))
        self.assertEqual(expected_distances, [
            message.ranges[0].corrected_range_m for message in ranges
        ])
        for message in ranges:
            self.assertEqual(1, len(message.ranges))
            self.assertEqual(1, message.ranges[0].anchor_id)
            self.assertTrue(message.ranges[0].valid)
            self.assertEqual(
                "distance_round5", message.ranges[0].source_format
            )
            self.assertEqual(UwbRangeArray.REPLAY_REBASED, message.timestamp_source)
            self.assertEqual(777, message.session_id)
            self.assertEqual(1234, message.timestamp_uncertainty_ns)

        published_round_contexts = []
        distance_round = []
        for line, message in zip(raw_text, raw_messages):
            match = DISTANCE_PATTERN.match(line)
            if not match:
                continue
            distance_round.append((float(match.group(2)), message))
            if len(distance_round) == 5:
                if any(value > 0.0 for value, _ in distance_round):
                    published_round_contexts.append(distance_round[0][1])
                distance_round = []
        self.assertEqual([], distance_round)
        self.assertEqual(6, len(published_round_contexts))
        for array, raw in zip(ranges, published_round_contexts):
            self.assertEqual(raw.header.stamp, array.header.stamp)
            self.assertEqual(raw.host_receive_stamp, array.host_receive_stamp)
            self.assertEqual(
                raw.host_receive_monotonic_ns,
                array.host_receive_monotonic_ns,
            )

        self.assertEqual(49, status.raw_line_count)
        self.assertEqual(6, status.parsed_round_count)
        self.assertEqual(0, status.parse_error_count)
        self.assertEqual(0, status.range_reject_count)
        self.assertIn("complete=7", status.detail)
        self.assertIn("empty=1", status.detail)
        self.assertIn("zero=29", status.detail)
        self.assertIn(
            "round_timestamp_policy=first_distance_line", status.detail
        )


if __name__ == "__main__":
    rospy.init_node("distance_round5_integration_test")
    rostest.rosrun(
        "uwb_serial_driver",
        "distance_round5_file_replay",
        DistanceRound5IntegrationTest,
    )
