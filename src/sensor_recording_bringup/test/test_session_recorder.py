#!/usr/bin/env python3
import importlib.util
import os
import threading
import unittest
from unittest import mock

from sensor_recording_bringup.recorder_gate import RecorderGate


SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "session_recorder.py")
)
SPEC = importlib.util.spec_from_file_location("session_recorder_under_test", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class ExitedProcess:
    def __init__(self, return_code):
        self.return_code = return_code

    def poll(self):
        return self.return_code


def ready_recorder(dry_run=True):
    recorder = MODULE.SessionRecorder.__new__(MODULE.SessionRecorder)
    recorder.shutting_down = False
    recorder.failed = False
    recorder.process = None
    recorder.gate = RecorderGate(["/sensor_time/events", "/uwb/raw"])
    recorder.gate.update_time(23, True)
    recorder.gate.mark_topic("/sensor_time/events")
    recorder.gate.mark_topic("/uwb/raw")
    recorder.current_session = 0
    recorder.bag_name = "test"
    recorder.output_dir = "/tmp"
    recorder.dry_run = dry_run
    recorder.topics = ["/sensor_time/events", "/uwb/raw"]
    recorder.status_pub = RecordingPublisher()
    recorder.last_status = None
    recorder.lock = threading.Lock()
    return recorder


class SessionRecorderTest(unittest.TestCase):
    def test_explicit_name_is_exact_bag_path(self):
        recorder = ready_recorder(dry_run=False)
        recorder.bag_name = "mission_01"
        process = mock.Mock()
        with mock.patch.object(
            MODULE.subprocess, "Popen", return_value=process
        ) as popen, mock.patch.object(
            MODULE.os.path, "exists", return_value=False
        ), mock.patch.object(MODULE.rospy, "loginfo"):
            recorder._maybe_start()
        popen.assert_called_once_with(
            [
                "rosbag",
                "record",
                "--output-name",
                "/tmp/mission_01.bag",
                "/sensor_time/events",
                "/uwb/raw",
            ],
            preexec_fn=os.setsid,
        )

    def test_empty_name_uses_timestamp_only(self):
        recorder = ready_recorder(dry_run=False)
        recorder.bag_name = ""
        process = mock.Mock()
        with mock.patch.object(
            MODULE.time, "strftime", return_value="20260727_142530"
        ), mock.patch.object(
            MODULE.subprocess, "Popen", return_value=process
        ) as popen, mock.patch.object(
            MODULE.os.path, "exists", return_value=False
        ), mock.patch.object(MODULE.rospy, "loginfo"):
            recorder._maybe_start()
        self.assertEqual(
            "/tmp/20260727_142530.bag",
            popen.call_args.args[0][3],
        )

    def test_existing_bag_is_not_overwritten(self):
        recorder = ready_recorder(dry_run=False)
        with mock.patch.object(
            MODULE.os.path, "exists", side_effect=lambda path: path.endswith(".bag")
        ), mock.patch.object(
            MODULE.subprocess, "Popen"
        ) as popen, mock.patch.object(MODULE.rospy, "logerr"):
            recorder._maybe_start()
        popen.assert_not_called()
        self.assertTrue(recorder.failed)
        self.assertEqual(
            "ERROR output bag already exists: /tmp/test.bag",
            recorder.status_pub.messages[-1],
        )

    def test_bag_filename_accepts_one_optional_extension(self):
        self.assertEqual("capture.bag", MODULE.bag_filename("capture"))
        self.assertEqual("capture.bag", MODULE.bag_filename("capture.bag"))
        with self.assertRaises(ValueError):
            MODULE.bag_filename("../capture")

    def test_ready_is_published_before_recording(self):
        recorder = ready_recorder()
        with mock.patch.object(MODULE.rospy, "loginfo"):
            recorder._maybe_start()
        self.assertEqual(
            ["READY", "RECORDING session=23"],
            recorder.status_pub.messages,
        )
        self.assertEqual("DRY_RUN", recorder.process)

    def test_rosbag_start_failure_publishes_error(self):
        recorder = ready_recorder(dry_run=False)
        with mock.patch.object(
            MODULE.subprocess, "Popen", side_effect=OSError("not found")
        ), mock.patch.object(MODULE.rospy, "logerr"):
            recorder._maybe_start()
        self.assertEqual("READY", recorder.status_pub.messages[0])
        self.assertTrue(
            recorder.status_pub.messages[-1].startswith(
                "ERROR rosbag start failed:"
            )
        )
        self.assertTrue(recorder.failed)

    def test_unexpected_rosbag_exit_publishes_error(self):
        recorder = ready_recorder(dry_run=False)
        recorder.process = ExitedProcess(9)
        with mock.patch.object(MODULE.rospy, "logerr"):
            recorder._check_process(None)
        self.assertEqual(
            "ERROR rosbag exited unexpectedly code=9",
            recorder.status_pub.messages[-1],
        )
        self.assertTrue(recorder.failed)


if __name__ == "__main__":
    unittest.main()
