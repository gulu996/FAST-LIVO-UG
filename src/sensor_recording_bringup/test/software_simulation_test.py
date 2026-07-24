#!/usr/bin/env python3
import time
import unittest

import rosnode
import rospy
import rostest
from sensor_time_msgs.msg import RawSerialFrame, TimeStatus
from std_msgs.msg import String
from uwb_serial_driver.msg import UwbRangeArray


class SoftwareSimulationTest(unittest.TestCase):
    @staticmethod
    def wait_for(predicate, topic, message_type, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            message = rospy.wait_for_message(topic, message_type, timeout=2.0)
            if predicate(message):
                return message
            time.sleep(0.05)
        raise AssertionError("timed out waiting for expected state on {}".format(topic))

    def test_local_only_records_without_fast(self):
        status = self.wait_for(
            lambda message: message.local_ready,
            "/sensor_time/status",
            TimeStatus,
        )
        self.assertTrue(status.local_ready)
        self.assertNotEqual(0, status.session_id)
        self.assertEqual(TimeStatus.LOCAL_ONLY, status.time_state)
        gnss_raw = rospy.wait_for_message("/gnss/raw", RawSerialFrame, timeout=5.0)
        self.assertNotEqual(RawSerialFrame.INVALID, gnss_raw.timestamp_source)
        uwb = rospy.wait_for_message("/uwb/ranges", UwbRangeArray, timeout=5.0)
        self.assertEqual(UwbRangeArray.REPLAY_REBASED, uwb.timestamp_source)
        self.assertGreater(uwb.timestamp_uncertainty_ns, 0)
        self.assertGreater(len(uwb.ranges), 0)
        recording = self.wait_for(
            lambda message: "RECORDING" in message.data,
            "/sensor_recording/status",
            String,
        )
        self.assertIn("RECORDING", recording.data)
        self.assertEqual(
            ["/sensor_time/events", "/gnss/raw", "/uwb/raw"],
            rospy.get_param("/session_recorder/effective_required_topics"),
        )
        nodes = rosnode.get_node_names()
        self.assertIn("/sensor_time_bridge", nodes)
        self.assertIn("/gnss_serial_driver", nodes)
        self.assertIn("/uwb_serial_driver", nodes)
        self.assertNotIn("/laserMapping", nodes)
        self.assertFalse(any("backend" in node.lower() for node in nodes))


if __name__ == "__main__":
    rospy.init_node("software_simulation_test")
    rostest.rosrun(
        "sensor_recording_bringup",
        "software_simulation_no_fast",
        SoftwareSimulationTest,
    )
