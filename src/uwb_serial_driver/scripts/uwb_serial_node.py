#!/usr/bin/env python3
import errno
import os
import threading
import time

import rospy
from sensor_time_bridge.imu_anchor import (
    LivoxImuAnchorReader,
    LivoxImuTimeMapper,
)
from sensor_time_msgs.msg import RawSerialFrame
from sensor_time_msgs.srv import HostMonotonicToLocal
from uwb_serial_driver.exclusive_serial import DEFAULT_DTR, DEFAULT_RTS, open_locked
from uwb_serial_driver.msg import UwbRange, UwbRangeArray, UwbStatus
from uwb_serial_driver.parser import (
    DistanceRoundAssembler,
    RangeFilter,
    UwbParser,
)


def ros_time_from_ns(stamp_ns):
    stamp = rospy.Time()
    if stamp_ns > 0:
        stamp.secs = int(stamp_ns // 1_000_000_000)
        stamp.nsecs = int(stamp_ns % 1_000_000_000)
    return stamp


class UwbSerialNode:
    def __init__(self):
        self.source = rospy.get_param("~source", "serial")
        self.port = rospy.get_param("~port", "/dev/uwb")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.dtr = bool(rospy.get_param("~dtr", DEFAULT_DTR))
        self.rts = bool(rospy.get_param("~rts", DEFAULT_RTS))
        self.replay_file = rospy.get_param("~replay_file", "")
        self.replay_mode = rospy.get_param("~replay_mode", "preserve")
        self.replay_rate_hz = float(rospy.get_param("~replay_rate_hz", 10.0))
        self.replay_start_delay_s = max(
            0.0, float(rospy.get_param("~replay_start_delay_s", 0.0))
        )
        self.replay_loop = bool(rospy.get_param("~replay_loop", False))
        self.replay_session_id = int(rospy.get_param("~replay_session_id", 1))
        self.replay_uncertainty_ns = int(
            rospy.get_param("~replay_uncertainty_ns", 1_000_000)
        )
        self.timestamp_mode = rospy.get_param("~timestamp_mode", "host_local")
        configured_imu_path = rospy.get_param("~imu_timeshare_path", "")
        self.imu_timeshare_path = os.path.expanduser(
            configured_imu_path
            or os.path.join(os.path.expanduser("~"), "timeshare_imu")
        )
        self.time_offset_s = float(rospy.get_param("~time_offset_s", 0.0))
        self.time_offset_ns = int(round(self.time_offset_s * 1_000_000_000))
        self.imu_anchor_reader = None
        self.imu_time_mapper = None
        if self.timestamp_mode == "livox_imu_anchor":
            self.imu_anchor_reader = LivoxImuAnchorReader(
                self.imu_timeshare_path
            )
            self.imu_time_mapper = LivoxImuTimeMapper(
                self.imu_anchor_reader,
                warn_age_s=float(
                    rospy.get_param("~imu_anchor_warn_age_s", 0.020)
                ),
                max_age_s=float(
                    rospy.get_param("~imu_anchor_max_age_s", 0.100)
                ),
            )
        elif self.timestamp_mode != "host_local":
            raise ValueError(
                "timestamp_mode must be host_local or livox_imu_anchor"
            )
        self.tag_id = int(rospy.get_param("~tag_id", 0))
        bias_param = rospy.get_param("~range_bias_m", {})
        biases = {int(key): float(value) for key, value in bias_param.items()}
        self.parser_mode = rospy.get_param(
            "~parser_mode", "distance_round5"
        )
        self.round_timestamp_policy = rospy.get_param(
            "~round_timestamp_policy", "first_distance_line"
        )
        self.parser = UwbParser(
            parser_mode=self.parser_mode,
            anchor_order=rospy.get_param("~anchor_order", []),
            range_scale=float(rospy.get_param("~range_scale", 1.0)),
            range_bias=biases,
            min_range_m=float(rospy.get_param("~min_range_m", 0.05)),
            max_range_m=float(rospy.get_param("~max_range_m", 250.0)),
        )
        self.round_assembler = None
        if self.parser_mode == "distance_round5":
            self.round_assembler = DistanceRoundAssembler(
                self.parser,
                lines_per_round=int(
                    rospy.get_param("~distance_lines_per_round", 5)
                ),
                round_timeout_s=float(
                    rospy.get_param("~round_timeout_s", 2.0)
                ),
                round_timestamp_policy=self.round_timestamp_policy,
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
        self.timestamp_mapping_drop_count = 0
        self.last_raw = None

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
        rospy.loginfo(
            "[UWB_TIME] mode=%s path=%s offset_s=%.9f",
            self.timestamp_mode,
            self.imu_timeshare_path,
            self.time_offset_s,
        )
        if self.round_assembler is not None:
            rospy.loginfo(
                "[UWB_PROTO] mode=distance_round5 "
                "round_timestamp_policy=%s",
                self.round_timestamp_policy,
            )

    def shutdown(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        if self.imu_anchor_reader is not None:
            self.imu_anchor_reader.close()
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

    def _timestamp_for_line(self, host_monotonic_ns, replay, preserved_ns):
        if replay and self.replay_mode == "preserve" and preserved_ns > 0:
            return (
                preserved_ns,
                self.replay_session_id,
                0,
                max(1, self.replay_uncertainty_ns),
                RawSerialFrame.REPLAY_PRESERVED,
                "replay_preserved",
            )

        if self.timestamp_mode == "livox_imu_anchor":
            result = self.imu_time_mapper.map_time(
                host_monotonic_ns, self.time_offset_ns
            )
            if result.warning:
                rospy.logwarn_throttle(
                    5.0,
                    "[UWB_TIME] anchor delta %.3f ms exceeds warning age",
                    result.anchor_delta_ns / 1e6,
                )
            if not result.success:
                rospy.logwarn_throttle(
                    5.0,
                    "[UWB_TIME] mapping unavailable: %s",
                    result.failure_reason,
                )
                return (
                    0,
                    result.mapping_session_id,
                    result.writer_epoch,
                    0,
                    RawSerialFrame.INVALID,
                    result.failure_reason,
                )
            return (
                result.mapped_stamp_ns,
                # In anchor mode session_id names the Livox IMU mapping
                # session, not the sensor_time_bridge MCU/LOCAL session.
                result.mapping_session_id,
                result.writer_epoch,
                result.anchor_uncertainty_ns,
                RawSerialFrame.DEVICE_TIME_MAPPED_LOCAL,
                "livox_imu_anchor",
            )

        local_ns, session_id, writer_epoch, uncertainty = self._host_local(
            host_monotonic_ns
        )
        timestamp_source = (
            RawSerialFrame.REPLAY_REBASED
            if replay and self.replay_mode == "rebase" and local_ns
            else RawSerialFrame.HOST_RECEIVE_LOCAL
            if local_ns
            else RawSerialFrame.INVALID
        )
        return (
            local_ns,
            session_id,
            writer_epoch,
            uncertainty,
            timestamp_source,
            "host_local" if local_ns else "host_local_unavailable",
        )

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
        if self.stop_event.wait(self.replay_start_delay_s):
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
                if (
                    self.round_assembler is not None
                    and self.round_assembler.state
                    == DistanceRoundAssembler.WAIT_DISTANCE_LINES
                ):
                    self.stop_event.wait(self.round_assembler.round_timeout_s)
                    self._expire_protocol_round()
                break

    def _run_serial(self):
        line_buffer = bytearray()
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            fd = -1
            try:
                fd = open_locked(self.port, self.baud, self.dtr, self.rts)
                self.serial_open = True
                self.reconnect_count += 1
                rospy.loginfo(
                    "UWB owns %s at %d baud, DTR=%s RTS=%s",
                    self.port,
                    self.baud,
                    self.dtr,
                    self.rts,
                )
                while not self.stop_event.is_set() and not rospy.is_shutdown():
                    try:
                        chunk = os.read(fd, 512)
                    except BlockingIOError:
                        chunk = b""
                    if not chunk:
                        self._expire_protocol_round()
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
        host_monotonic_ns = (
            time.monotonic_ns()
            if self.timestamp_mode == "livox_imu_anchor"
            else time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
        )
        host_wall = rospy.Time.now()
        (
            local_ns,
            session_id,
            writer_epoch,
            uncertainty,
            timestamp_source,
            _,
        ) = self._timestamp_for_line(
            host_monotonic_ns, replay, preserved_ns
        )

        self.raw_line_count += 1
        raw = RawSerialFrame()
        raw.header.stamp = ros_time_from_ns(local_ns)
        raw.header.frame_id = (
            "livox_imu_legacy_time"
            if self.timestamp_mode == "livox_imu_anchor"
            and timestamp_source
            == RawSerialFrame.DEVICE_TIME_MAPPED_LOCAL
            else "local_sensor_time"
        )
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
        self.last_raw = raw
        if self.raw_log:
            self.raw_log.write("{} {}\n".format(host_monotonic_ns, line))
        self._log_time_summary()

        if self.round_assembler is not None:
            self._handle_distance_round5(line, raw)
            return

        parsed = self.parser.parse(line)
        if not parsed:
            self.parse_error_count += 1
            self._publish_status(raw, "no valid range record")
            return
        self._publish_ranges(parsed, raw, raw)

    def _handle_distance_round5(self, line, raw):
        duplicate_before = self.round_assembler.duplicate_anchor_count
        event = self.round_assembler.process(
            line, context=raw, now_s=time.monotonic()
        )
        if event.incomplete_round:
            rospy.logwarn_throttle(
                5.0,
                "[UWB_PROTO] incomplete distance round discarded; "
                "waiting for current UWBDBG boundary",
            )
        if (
            self.round_assembler.duplicate_anchor_count
            > duplicate_before
        ):
            rospy.logwarn_throttle(
                5.0,
                "[UWB_PROTO] duplicate nonzero anchor discarded in round",
            )

        if event.parse_error:
            self.parse_error_count += 1
            label = (
                "malformed distance"
                if event.kind == DistanceRoundAssembler.MALFORMED_DISTANCE
                else "unknown protocol"
            )
            rospy.logwarn_throttle(
                5.0, "[UWB_PROTO] %s line: %.160s", label, line
            )
            self._publish_status(raw, label)
        elif event.kind == DistanceRoundAssembler.EMPTY_ROUND:
            rospy.logwarn_throttle(
                5.0, "[UWB_PROTO] complete round contains no nonzero range"
            )
            self._publish_status(raw, "empty distance round")
        elif event.kind == DistanceRoundAssembler.ROUND_COMPLETE:
            self._publish_ranges(event.ranges, event.timestamp_context, raw)
        elif event.incomplete_round:
            self._publish_status(raw, "incomplete distance round discarded")
        self._log_protocol_summary()

    def _publish_ranges(self, parsed, timestamp_raw, status_raw):
        stamp_ns = timestamp_raw.header.stamp.to_nsec()
        if (
            self.timestamp_mode == "livox_imu_anchor"
            and (
                stamp_ns == 0
                or timestamp_raw.timestamp_source
                != RawSerialFrame.DEVICE_TIME_MAPPED_LOCAL
            )
        ):
            self.timestamp_mapping_drop_count += 1
            rospy.logwarn_throttle(
                5.0,
                "[UWB_TIME] complete range round dropped: selected round "
                "timestamp context has no valid IMU anchor mapping",
            )
            self._publish_status(
                status_raw, "timestamp mapping unavailable; round dropped"
            )
            return False
        filtered = self.filter.filter(
            parsed, stamp_ns / 1e9 if stamp_ns else time.monotonic()
        )
        if not filtered:
            self._publish_status(
                status_raw, "round dropped by repeated-range filter"
            )
            return False
        self.round_sequence += 1
        array = UwbRangeArray()
        array.header = timestamp_raw.header
        array.session_id = timestamp_raw.session_id
        array.round_sequence = self.round_sequence
        array.tag_id = self.tag_id
        array.timestamp_source = timestamp_raw.timestamp_source
        array.timestamp_uncertainty_ns = max(
            1, timestamp_raw.time_uncertainty_ns
        )
        array.host_receive_stamp = timestamp_raw.host_receive_stamp
        array.host_receive_monotonic_ns = (
            timestamp_raw.host_receive_monotonic_ns
        )
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
                    stamp_ns, self.round_sequence,
                    ",".join("{}:{:.4f}".format(v.anchor_id, v.corrected_range_m)
                             for v in filtered)
                )
            )
        self._publish_status(status_raw, "round published")
        return True

    def _expire_protocol_round(self):
        if self.round_assembler is None:
            return
        if self.round_assembler.expire(time.monotonic()):
            rospy.logwarn_throttle(
                5.0,
                "[UWB_PROTO] distance round timed out before all lines arrived",
            )
            if self.last_raw is not None:
                self._publish_status(
                    self.last_raw, "incomplete distance round timed out"
                )
            self._log_protocol_summary()

    def _protocol_detail(self):
        if self.round_assembler is None:
            return "mode={}".format(self.parser_mode)
        protocol = self.round_assembler
        return (
            "mode=distance_round5 round_timestamp_policy={} "
            "state={} pending={} complete={} "
            "empty={} zero={} incomplete={} malformed={} ignored_debug={} "
            "duplicate={}"
        ).format(
            protocol.round_timestamp_policy,
            protocol.state,
            protocol.pending_distance_line_count,
            protocol.complete_round_count,
            protocol.empty_round_count,
            protocol.zero_slot_count,
            protocol.incomplete_round_count,
            protocol.malformed_distance_count,
            protocol.ignored_debug_line_count,
            protocol.duplicate_anchor_count,
        )

    def _time_detail(self):
        if self.imu_time_mapper is None:
            return "timestamp_mode={}".format(self.timestamp_mode)
        stats = self.imu_time_mapper.stats
        return (
            "timestamp_mode=livox_imu_anchor mapped={} not_ready={} "
            "stale={} invalid={} epoch_change={} nonmonotonic_drop={} "
            "mapping_error={} range_time_drop={} offset_s={:.9f}"
        ).format(
            stats.mapped_count,
            stats.not_ready_count,
            stats.stale_count,
            stats.invalid_count,
            stats.epoch_change_count,
            stats.nonmonotonic_drop_count,
            stats.mapping_error_count,
            self.timestamp_mapping_drop_count,
            self.time_offset_s,
        )

    def _log_time_summary(self):
        if self.imu_time_mapper is None:
            return
        stats = self.imu_time_mapper.stats
        rospy.loginfo_throttle(
            20.0,
            "[UWB_TIME] mode=livox_imu_anchor mapped=%d not_ready=%d "
            "stale=%d invalid=%d epoch_change=%d nonmonotonic_drop=%d "
            "mapping_error=%d range_time_drop=%d offset_s=%.9f",
            stats.mapped_count,
            stats.not_ready_count,
            stats.stale_count,
            stats.invalid_count,
            stats.epoch_change_count,
            stats.nonmonotonic_drop_count,
            stats.mapping_error_count,
            self.timestamp_mapping_drop_count,
            self.time_offset_s,
        )

    def _log_protocol_summary(self):
        if self.round_assembler is None:
            return
        protocol = self.round_assembler
        rospy.loginfo_throttle(
            20.0,
            "[UWB_PROTO] raw=%d complete=%d published=%d empty=%d "
            "zero_slots=%d incomplete=%d malformed=%d ignored_debug=%d "
            "duplicate_anchor=%d",
            self.raw_line_count,
            protocol.complete_round_count,
            self.parsed_round_count,
            protocol.empty_round_count,
            protocol.zero_slot_count,
            protocol.incomplete_round_count,
            protocol.malformed_distance_count,
            protocol.ignored_debug_line_count,
            protocol.duplicate_anchor_count,
        )

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
        status.detail = "{}; {}; {}".format(
            detail, self._protocol_detail(), self._time_detail()
        )
        self.status_pub.publish(status)


if __name__ == "__main__":
    rospy.init_node("uwb_serial_driver")
    UwbSerialNode()
    rospy.spin()
