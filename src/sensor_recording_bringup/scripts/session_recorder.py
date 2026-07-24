#!/usr/bin/env python3
import os
import signal
import subprocess
import threading
import time

import rospy
from sensor_recording_bringup.recorder_gate import (
    RecorderGate,
    effective_required_topics,
)
from sensor_time_msgs.msg import TimeStatus
from std_msgs.msg import String


RAW_TOPICS = [
    "/sensor_time/events",
    "/sensor_time/mcu_trigger",
    "/sensor_time/mcu_pps",
    "/sensor_time/gnss_pps_capture",
    "/sensor_time/mapping",
    "/sensor_time/status",
    "/gnss/raw",
    "/ublox_driver/receiver_pvt",
    "/uwb/raw",
]
PARSED_TOPICS = [
    "/sensor_time/mapping",
    "/sensor_time/status",
    "/gnss/pvt_local",
    "/gnss/enu_odom",
    "/gnss/status_v2",
    "/uwb/ranges",
    "/uwb/status",
    "/livox/lidar",
    "/livox/imu",
    "/left_camera/image",
]


class SessionRecorder:
    def __init__(self):
        if rospy.get_param("/use_sim_time", False):
            raise RuntimeError("sensor recording requires /use_sim_time=false")
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "/tmp"))
        self.bag_prefix = rospy.get_param("~bag_prefix", "sensors")
        self.profile = rospy.get_param("~record_profile", "both")
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        enabled_topics = rospy.get_param("~enabled_topics", [])
        self.required_topics = effective_required_topics(
            rospy.get_param("~required_topics", []),
            enable_livox=rospy.get_param("~enable_livox", False),
            enable_camera=rospy.get_param("~enable_camera", False),
            enable_gnss=rospy.get_param("~enable_gnss", False),
            enable_uwb=rospy.get_param("~enable_uwb", False),
        )
        if enabled_topics:
            self.topics = list(dict.fromkeys(enabled_topics))
        elif self.profile == "raw":
            self.topics = RAW_TOPICS
        elif self.profile == "parsed":
            self.topics = PARSED_TOPICS
        else:
            self.topics = list(dict.fromkeys(RAW_TOPICS + PARSED_TOPICS))
        self.gate = RecorderGate(self.required_topics)
        self.process = None
        self.current_session = 0
        self.shutting_down = False
        self.failed = False
        self.lock = threading.Lock()
        self.last_status = None
        self.status_pub = rospy.Publisher(
            "/sensor_recording/status", String, queue_size=2, latch=True
        )
        rospy.set_param(
            "/session_recorder/effective_required_topics", self.required_topics
        )
        rospy.loginfo(
            "Recorder required topics: %s", ", ".join(self.required_topics)
        )
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            if not os.path.isdir(self.output_dir) or not os.access(
                self.output_dir, os.W_OK
            ):
                raise OSError("output directory is not writable")
        except OSError as error:
            self._publish_status(
                "ERROR output directory {}: {}".format(self.output_dir, error)
            )
            raise
        self.time_sub = rospy.Subscriber(
            "/sensor_time/status", TimeStatus, self.time_callback, queue_size=5
        )
        self.topic_subscribers = [
            rospy.Subscriber(topic, rospy.AnyMsg, self.topic_callback,
                             callback_args=topic, queue_size=1)
            for topic in self.required_topics
        ]
        self.process_timer = rospy.Timer(
            rospy.Duration(0.5), self._check_process
        )
        self._publish_status(self.gate.status())
        rospy.on_shutdown(self.shutdown)

    def _publish_status(self, status):
        if status == self.last_status:
            return
        self.last_status = status
        self.status_pub.publish(status)

    def topic_callback(self, _message, topic):
        with self.lock:
            self.gate.mark_topic(topic)
            self._maybe_start()

    def time_callback(self, message):
        with self.lock:
            changed = self.gate.update_time(message.session_id, message.local_ready)
            if changed and self.process is not None:
                self._stop_bag()
            self._maybe_start()

    def _maybe_start(self):
        if self.shutting_down or self.failed or self.process is not None:
            return
        gate_status = self.gate.status()
        self._publish_status(gate_status)
        if gate_status != "READY":
            return
        self.current_session = self.gate.session_id
        stamp = time.strftime("%Y%m%d_%H%M%S")
        basename = "{}_session_{:016x}_{}".format(
            self.bag_prefix, self.current_session, stamp
        )
        output = os.path.join(self.output_dir, basename)
        try:
            if self.dry_run:
                self.process = "DRY_RUN"
            else:
                command = ["rosbag", "record", "--output-name", output] + self.topics
                self.process = subprocess.Popen(command, preexec_fn=os.setsid)
        except (OSError, subprocess.SubprocessError) as error:
            self.process = None
            self.failed = True
            self._publish_status("ERROR rosbag start failed: {}".format(error))
            rospy.logerr("Cannot start rosbag: %s", error)
            return
        self._publish_status(
            "RECORDING session={}".format(self.current_session)
        )
        rospy.loginfo("Recorder started for LOCAL session=%d", self.current_session)

    def _check_process(self, _event):
        with self.lock:
            if (
                self.shutting_down
                or self.process is None
                or self.process == "DRY_RUN"
            ):
                return
            return_code = self.process.poll()
            if return_code is None:
                return
            self.process = None
            self.failed = True
            self._publish_status(
                "ERROR rosbag exited unexpectedly code={}".format(return_code)
            )
            rospy.logerr("rosbag exited unexpectedly with code %d", return_code)

    def _stop_bag(self):
        if self.process is None:
            return
        if self.process != "DRY_RUN":
            if self.process.poll() is None:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                try:
                    self.process.wait(timeout=15.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    self.process.wait(timeout=5.0)
        self.process = None
        self._publish_status("ROTATING")

    def shutdown(self):
        with self.lock:
            self.shutting_down = True
            self._stop_bag()


if __name__ == "__main__":
    rospy.init_node("session_recorder")
    try:
        SessionRecorder()
        rospy.spin()
    except (OSError, RuntimeError) as error:
        rospy.logfatal("%s", error)
        raise SystemExit(2)
