#!/usr/bin/env python3
import unittest

from sensor_recording_bringup.recorder_gate import RecorderGate


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


if __name__ == "__main__":
    unittest.main()
