#!/usr/bin/env python3
import errno
import os
import threading
import time

import rospy
from sensor_time_msgs.msg import RawSerialFrame
from sensor_time_msgs.srv import HostMonotonicToLocal
from uwb_serial_driver.exclusive_serial import open_locked
from uwb_serial_driver.msg import UwbRange, UwbRangeArray, UwbStatus
from uwb_serial_driver.parser import RangeFilter, UwbParser


class UwbSerialNode:
    def __init__(self):
        self.source = rospy.get_param("~source", "serial")
        self.port = rospy.get_param("~port", "/dev/uwb")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.replay_file = rospy.get_param("~replay_file", "")
        self.replay_mode = rospy.get_param("~replay_mode", "preserve")
        self.replay_rate_hz = float(rospy.get_param("~replay_rate_hz", 10.0))
        self.replay_loop = bool(rospy.get_param("~replay_loop", False))
        self.replay_session_id = int(rospy.get_param("~replay_session_id", 1))
        self.replay_uncertainty_ns = int(
            rospy.get_param("~replay_uncertainty_ns", 1_000_000)
        )
        self.tag_id = int(rospy.get_param("~tag_id", 0))
        bias_param = rospy.get_param("~range_bias_m", {})
        biases = {int(key): float(value) for key, value in bias_param.items()}
        self.parser = UwbParser(
            parser_mode=rospy.get_param("~parser_mode", "auto"),
            anchor_order=rospy.get_param("~anchor_order", []),
            range_scale=float(rospy.get_param("~range_scale", 1.0)),
            range_bias=biases,
            min_range_m=float(rospy.get_param("~min_range_m", 0.05)),
            max_range_m=float(rospy.get_param("~max_range_m", 250.0)),
        )
        self.filter = RangeFilter(
            epsilon_m=float(rospy.get_param("~repeat_epsilon_m", 0.001)),
            max_count=int(rospy.get_param("~repeat_max_count", 3)),
            max_duration_s=float(rospy.get_param("~repeat_max_duration_s", 2.0)),
        )
        self.raw_log_path = rospy.get_param("~raw_log_path", "")
        self.parsed_log_path = rospy.get_param("~parsed_log_path", "")
        self.raw_log = open(self.raw_log_path, "a", buffering=1) if self.raw_log_path else None
        self.parsed_log = (
            open(self.parsed_log_path, "a", buffering=1) if self.parsed_log_path else None
        )
        self.stop_event = threading.Event()
        self.serial_open = False
        self.round_sequence = 0
        self.raw_line_count = 0
        self.parsed_round_count = 0
        self.parse_error_count = 0
        self.range_reject_count = 0
        self.reconnect_count = 0

        self.raw_pub = rospy.Publisher("/uwb/raw", RawSerialFrame, queue_size=100)
        self.range_pub = rospy.Publisher("/uwb/ranges", UwbRangeArray, queue_size=50)
        self.status_pub = rospy.Publisher(
            "/uwb/status", UwbStatus, queue_size=5, latch=True
        )
        self.host_converter = rospy.ServiceProxy(
            "/sensor_time/host_monotonic_to_local", HostMonotonicToLocal
        )
        self.worker = threading.Thread(target=self._run, name="uwb_source", daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        for stream in (self.raw_log, self.parsed_log):
            if stream:
                stream.close()

    def _host_local(self, host_monotonic_ns):
        try:
            response = self.host_converter(host_monotonic_ns)
            if response.success:
                return (
                    response.local_stamp_ns,
                    response.session_id,
                    response.writer_epoch,
                    response.time_uncertainty_ns,
                )
        except rospy.ServiceException:
            pass
        return 0, 0, 0, 0

    def _run(self):
        if self.source == "file":
            self._run_file()
        elif self.source == "serial":
            self._run_serial()
        else:
            rospy.logfatal("UWB source must be serial or file")
            rospy.signal_shutdown("invalid UWB source")

    def _run_file(self):
        if not self.replay_file:
            rospy.logerr("UWB replay_file is empty")
            return
        try:
            with open(os.path.expanduser(self.replay_file), "r") as stream:
                lines = [line.rstrip("\r\n") for line in stream if line.strip()]
        except OSError as error:
            rospy.logerr("Cannot open UWB replay %s: %s", self.replay_file, error)
            return
        delay = 1.0 / max(0.1, self.replay_rate_hz)
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            for record in lines:
                if self.stop_event.is_set() or rospy.is_shutdown():
                    break
                preserved_ns = 0
                line = record
                if "|" in record:
                    prefix, line = record.split("|", 1)
                    try:
                        preserved_ns = int(prefix.strip())
                    except ValueError:
                        line = record
                self._handle_line(line.strip(), True, preserved_ns)
                self.stop_event.wait(delay)
            if not self.replay_loop:
                break

    def _run_serial(self):
        line_buffer = bytearray()
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            fd = -1
            try:
                fd = open_locked(self.port, self.baud)
                self.serial_open = True
                self.reconnect_count += 1
                rospy.loginfo("UWB owns %s at %d baud", self.port, self.baud)
                while not self.stop_event.is_set() and not rospy.is_shutdown():
                    try:
                        chunk = os.read(fd, 512)
                    except BlockingIOError:
                        chunk = b""
                    if not chunk:
                        self.stop_event.wait(0.005)
                        continue
                    for byte in chunk:
                        if byte in (10, 13):
                            if line_buffer:
                                line = line_buffer.decode("ascii", errors="replace").strip()
                                line_buffer.clear()
                                if line:
                                    self._handle_line(line, False, 0)
                        elif len(line_buffer) < 4096:
                            line_buffer.append(byte)
                        else:
                            line_buffer.clear()
                            self.parse_error_count += 1
            except BlockingIOError:
                rospy.logfatal("UWB serial ownership conflict on %s", self.port)
                rospy.signal_shutdown("UWB serial ownership conflict")
                return
            except (OSError, ValueError) as error:
                if isinstance(error, OSError) and error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    continue
                rospy.logerr_throttle(5.0, "UWB serial error on %s: %s", self.port, error)
            finally:
                self.serial_open = False
                if fd >= 0:
                    os.close(fd)
            self.stop_event.wait(1.0)

    def _handle_line(self, line, replay, preserved_ns):
        host_monotonic_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
        host_wall = rospy.Time.now()
        local_ns, session_id, writer_epoch, uncertainty = self._host_local(host_monotonic_ns)
        timestamp_source = UwbRangeArray.HOST_RECEIVE_LOCAL
        if replay and self.replay_mode == "preserve" and preserved_ns > 0:
            local_ns = preserved_ns
            session_id = self.replay_session_id
            uncertainty = max(1, self.replay_uncertainty_ns)
            timestamp_source = UwbRangeArray.REPLAY_PRESERVED
        elif replay and self.replay_mode == "rebase" and local_ns > 0:
            timestamp_source = UwbRangeArray.REPLAY_REBASED
        elif local_ns == 0:
            timestamp_source = UwbRangeArray.INVALID

        self.raw_line_count += 1
        raw = RawSerialFrame()
        raw.header.stamp = rospy.Time.from_sec(local_ns / 1e9) if local_ns else rospy.Time()
        raw.header.frame_id = "local_sensor_time"
        raw.session_id = session_id
        raw.writer_epoch = writer_epoch
        raw.source_sequence = self.raw_line_count
        raw.host_receive_stamp = host_wall
        raw.host_receive_monotonic_ns = host_monotonic_ns
        raw.time_uncertainty_ns = uncertainty
        raw.timestamp_source = timestamp_source
        raw.device = self.replay_file if replay else self.port
        raw.protocol = "UWB_TEXT"
        raw.data = list(line.encode("utf-8", errors="replace"))
        self.raw_pub.publish(raw)
        if self.raw_log:
            self.raw_log.write("{} {}\n".format(host_monotonic_ns, line))

        parsed = self.parser.parse(line)
        if not parsed:
            self.parse_error_count += 1
            self._publish_status(raw, "no valid range record")
            return
        # ponytail: one input line is one round; independent lines are never time-window merged.
        filtered = self.filter.filter(parsed, local_ns / 1e9 if local_ns else time.monotonic())
        if not filtered:
            self._publish_status(raw, "round dropped by repeated-range filter")
            return
        self.round_sequence += 1
        array = UwbRangeArray()
        array.header = raw.header
        array.session_id = session_id
        array.round_sequence = self.round_sequence
        array.tag_id = self.tag_id
        array.timestamp_source = timestamp_source
        array.timestamp_uncertainty_ns = max(1, uncertainty)
        array.host_receive_stamp = host_wall
        array.host_receive_monotonic_ns = host_monotonic_ns
        for value in filtered:
            message = UwbRange()
            message.anchor_id = value.anchor_id
            message.raw_range_m = value.raw_range_m
            message.range_bias_m = value.range_bias_m
            message.corrected_range_m = value.corrected_range_m
            message.diag = value.diag
            message.valid = value.valid
            message.reject_reason = value.reject_reason
            message.source_format = value.source_format
            array.ranges.append(message)
            if not value.valid:
                self.range_reject_count += 1
        self.range_pub.publish(array)
        self.parsed_round_count += 1
        if self.parsed_log:
            self.parsed_log.write(
                "{} round={} ranges={}\n".format(
                    local_ns, self.round_sequence,
                    ",".join("{}:{:.4f}".format(v.anchor_id, v.corrected_range_m)
                             for v in filtered)
                )
            )
        self._publish_status(raw, "round published")

    def _publish_status(self, raw, detail):
        status = UwbStatus()
        status.header = raw.header
        status.session_id = raw.session_id
        status.writer_epoch = raw.writer_epoch
        status.serial_open = self.serial_open
        status.round_sequence = self.round_sequence
        status.raw_line_count = self.raw_line_count
        status.parsed_round_count = self.parsed_round_count
        status.parse_error_count = self.parse_error_count
        status.range_reject_count = self.range_reject_count
        status.repeat_drop_count = self.filter.drop_count
        status.reconnect_count = self.reconnect_count
        status.source = self.source
        status.detail = detail
        self.status_pub.publish(status)


if __name__ == "__main__":
    rospy.init_node("uwb_serial_driver")
    UwbSerialNode()
    rospy.spin()
