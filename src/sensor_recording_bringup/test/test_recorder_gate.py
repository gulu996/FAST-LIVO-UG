#!/usr/bin/env python3
import unittest

from sensor_recording_bringup.recorder_gate import (
    RecorderGate,
    effective_required_topics,
)


class RecorderGateTest(unittest.TestCase):
    def test_gnss_utc_is_not_a_gate(self):
        gate = RecorderGate(["/livox/lidar", "/gnss/raw"])
        gate.update_time(42, True)
        gate.mark_topic("/livox/lidar")
        gate.mark_topic("/gnss/raw")
        self.assertTrue(gate.ready())

    def test_requires_local_session_and_enabled_topics(self):
        gate = RecorderGate(["/uwb/raw"])
        gate.mark_topic("/uwb/raw")
        self.assertFalse(gate.ready())
        self.assertFalse(gate.update_time(1, True))
        self.assertTrue(gate.ready())
        self.assertTrue(gate.update_time(2, True))

    def test_wait_states_distinguish_local_session_and_topics(self):
        gate = RecorderGate(["/sensor_time/events", "/uwb/raw"])
        self.assertEqual("WAIT_LOCAL", gate.status())
        gate.update_time(0, True)
        self.assertEqual("WAIT_SESSION", gate.status())
        gate.update_time(7, True)
        gate.mark_topic("/sensor_time/events")
        self.assertEqual("WAIT_TOPICS missing=/uwb/raw", gate.status())

    def test_missing_topics_are_sorted_and_ready_when_complete(self):
        gate = RecorderGate(["/z/raw", "/a/raw", "/sensor_time/events"])
        gate.update_time(11, True)
        self.assertEqual(
            "WAIT_TOPICS missing=/a/raw,/sensor_time/events,/z/raw",
            gate.status(),
        )
        for topic in ("/z/raw", "/sensor_time/events", "/a/raw"):
            gate.mark_topic(topic)
        self.assertEqual("READY", gate.status())
        self.assertTrue(gate.ready())

    def test_uwb_only_required_topics_are_raw_and_local(self):
        self.assertEqual(
            ["/sensor_time/events", "/uwb/raw"],
            effective_required_topics([], enable_uwb=True),
        )

    def test_gnss_required_topics_include_raw_input(self):
        self.assertEqual(
            ["/sensor_time/events", "/gnss/raw"],
            effective_required_topics([], enable_gnss=True),
        )

    def test_disabled_sensors_and_parsed_profile_do_not_expand_gate(self):
        required = effective_required_topics([], enable_uwb=True)
        gate = RecorderGate(required)
        gate.update_time(17, True)
        gate.mark_topic("/sensor_time/events")
        for parsed_topic in ("/uwb/ranges", "/uwb/status", "/gnss/pvt_local"):
            gate.mark_topic(parsed_topic)
        self.assertEqual("WAIT_TOPICS missing=/uwb/raw", gate.status())

    def test_configured_topics_are_deduplicated_in_stable_order(self):
        self.assertEqual(
            ["/sensor_time/events", "/uwb/raw"],
            effective_required_topics(
                ["/sensor_time/events", "/uwb/raw", "/sensor_time/events"],
                enable_livox=True,
                enable_camera=True,
                enable_gnss=True,
                enable_uwb=True,
            ),
        )


if __name__ == "__main__":
    unittest.main()
