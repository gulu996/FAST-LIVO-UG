#!/usr/bin/env python3
import datetime
import errno
import math
import os
import threading
import time

import rospy
from gnss_comm.msg import GnssPVTSolnMsg
from gnss_serial_driver.exclusive_serial import open_locked
from gnss_serial_driver.msg import GnssPvtStamped, GnssStatus
from gnss_serial_driver.parser import GnssParser
from gnss_serial_driver.time_policy import utc_to_local, valid_for_fusion
from sensor_time_msgs.msg import RawSerialFrame, TimeMapping, UtcObservation
from sensor_time_msgs.srv import HostMonotonicToLocal


GPS_EPOCH_UNIX_S = 315964800
GPS_WEEK_S = 604800.0


def finite_or(value, fallback):
    return value if math.isfinite(value) else fallback


class GnssSerialNode:
    def __init__(self):
        self.source = rospy.get_param("~source", "serial")
        self.port = rospy.get_param("~port", "/dev/gnss")
        self.baud = int(rospy.get_param("~baud", 921600))
        self.dtr = bool(rospy.get_param("~dtr", True))
        self.rts = bool(rospy.get_param("~rts", False))
        self.replay_file = rospy.get_param("~replay_file", "")
        self.replay_mode = rospy.get_param("~replay_mode", "preserve")
        self.replay_rate_hz = float(rospy.get_param("~replay_rate_hz", 10.0))
        self.replay_loop = bool(rospy.get_param("~replay_loop", False))
        self.replay_has_pps_association = bool(
            rospy.get_param("~replay_has_pps_association", False)
        )
        self.gps_utc_leap_seconds = int(rospy.get_param("~gps_utc_leap_seconds", 18))
        self.accepted_qualities = set(rospy.get_param("~accepted_qualities", [4]))
        self.raw_log_path = rospy.get_param("~raw_log_path", "")
        self.parsed_log_path = rospy.get_param("~parsed_log_path", "")
        self.parser = GnssParser()
        self.sequence = 0
        self.mapping = None
        self.last_local_measurement_ns = 0
        self.serial_open = False
        self.checksum_errors = 0
        self.parse_errors = 0
        self.reconnect_count = 0
        self.stop_event = threading.Event()
        self.raw_log = open(self.raw_log_path, "a", buffering=1) if self.raw_log_path else None
        self.parsed_log = (
            open(self.parsed_log_path, "a", buffering=1) if self.parsed_log_path else None
        )

        self.raw_pub = rospy.Publisher("/gnss/raw", RawSerialFrame, queue_size=100)
        self.compat_pub = rospy.Publisher(
            "/ublox_driver/receiver_pvt", GnssPVTSolnMsg, queue_size=20
        )
        self.local_pub = rospy.Publisher("/gnss/pvt_local", GnssPvtStamped, queue_size=20)
        self.status_pub = rospy.Publisher(
            "/gnss/status_v2", GnssStatus, queue_size=5, latch=True
        )
        self.utc_pub = rospy.Publisher(
            "/sensor_time/utc_observation", UtcObservation, queue_size=20
        )
        self.mapping_sub = rospy.Subscriber(
            "/sensor_time/mapping", TimeMapping, self._mapping_callback, queue_size=5
        )
        self.host_converter = rospy.ServiceProxy(
            "/sensor_time/host_monotonic_to_local", HostMonotonicToLocal
        )
        self.worker = threading.Thread(target=self._run, name="gnss_source", daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        for stream in (self.raw_log, self.parsed_log):
            if stream:
                stream.close()

    def _mapping_callback(self, message):
        self.mapping = message

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

    def _utc_to_local(self, utc_ns):
        local_ns, valid = utc_to_local(
            utc_ns, self.mapping, self.last_local_measurement_ns
        )
        if valid:
            self.last_local_measurement_ns = local_ns
        return local_ns, valid

    def _run(self):
        if self.source == "file":
            self._run_file()
        elif self.source == "serial":
            self._run_serial()
        else:
            rospy.logfatal("gnss source must be serial or file")
            rospy.signal_shutdown("invalid GNSS source")

    def _run_file(self):
        if not self.replay_file:
            rospy.logerr("GNSS replay_file is empty")
            return
        try:
            with open(os.path.expanduser(self.replay_file), "r") as stream:
                lines = [line.rstrip("\r\n") for line in stream if line.strip()]
        except OSError as error:
            rospy.logerr("Cannot open GNSS replay %s: %s", self.replay_file, error)
            return
        delay = 1.0 / max(0.1, self.replay_rate_hz)
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            for line in lines:
                if self.stop_event.is_set() or rospy.is_shutdown():
                    break
                self._handle_line(line, replay=True)
                self.stop_event.wait(delay)
            if not self.replay_loop:
                break

    def _run_serial(self):
        line_buffer = bytearray()
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            fd = -1
            try:
                fd = open_locked(self.port, self.baud, self.dtr, self.rts)
                self.serial_open = True
                self.reconnect_count += 1
                rospy.loginfo("GNSS owns %s at %d baud", self.port, self.baud)
                while not self.stop_event.is_set() and not rospy.is_shutdown():
                    try:
                        chunk = os.read(fd, 1024)
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
                                    self._handle_line(line, replay=False)
                        elif len(line_buffer) < 8192:
                            line_buffer.append(byte)
                        else:
                            line_buffer.clear()
                            self.parse_errors += 1
            except BlockingIOError:
                rospy.logfatal("GNSS serial ownership conflict on %s", self.port)
                rospy.signal_shutdown("GNSS serial ownership conflict")
                return
            except (OSError, ValueError) as error:
                if isinstance(error, OSError) and error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    continue
                rospy.logerr_throttle(5.0, "GNSS serial error on %s: %s", self.port, error)
            finally:
                self.serial_open = False
                if fd >= 0:
                    os.close(fd)
            self.stop_event.wait(1.0)

    def _handle_line(self, line, replay):
        host_monotonic_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
        host_wall = rospy.Time.now()
        receive_local_ns, session_id, writer_epoch, receive_uncertainty = self._host_local(
            host_monotonic_ns
        )
        self.sequence += 1
        raw = RawSerialFrame()
        raw.header.stamp = rospy.Time.from_sec(receive_local_ns / 1e9) if receive_local_ns else rospy.Time()
        raw.header.frame_id = "local_sensor_time"
        raw.session_id = session_id
        raw.writer_epoch = writer_epoch
        raw.source_sequence = self.sequence
        raw.host_receive_stamp = host_wall
        raw.host_receive_monotonic_ns = host_monotonic_ns
        raw.time_uncertainty_ns = receive_uncertainty
        raw.timestamp_source = (
            RawSerialFrame.REPLAY_REBASED
            if replay and self.replay_mode == "rebase" and receive_local_ns
            else RawSerialFrame.HOST_RECEIVE_LOCAL
            if receive_local_ns
            else RawSerialFrame.INVALID
        )
        raw.device = self.replay_file if replay else self.port
        raw.protocol = "NMEA_KSXT_AGRICA_LEGACY"
        raw.data = list(line.encode("utf-8", errors="replace"))
        self.raw_pub.publish(raw)
        if self.raw_log:
            self.raw_log.write("{} {}\n".format(host_monotonic_ns, line))

        parsed_values = self.parser.parse(line)
        if not parsed_values:
            self.parse_errors += 1
            self._publish_status(raw, None, "no supported record")
            return
        for parsed in parsed_values:
            if not parsed.checksum_valid:
                self.checksum_errors += 1
            if parsed.reject_reason not in ("ok", "invalid_status", "invalid_position_or_quality"):
                self.parse_errors += 1
            if self.parsed_log:
                self.parsed_log.write(
                    "{} {} utc={} quality={} position_valid={} reason={}\n".format(
                        host_monotonic_ns, parsed.source, parsed.utc_ns, parsed.quality,
                        int(parsed.position_valid), parsed.reject_reason
                    )
                )
            if parsed.position_valid:
                self._publish_pvt(parsed, raw, replay)
            elif parsed.utc_valid:
                self._publish_utc_observation(parsed, raw, replay)
            self._publish_status(raw, parsed, parsed.reject_reason)

    def _publish_utc_observation(self, parsed, raw, replay):
        observation = UtcObservation()
        observation.header = raw.header
        observation.session_id = raw.session_id
        observation.writer_epoch = raw.writer_epoch
        observation.source_sequence = raw.source_sequence
        observation.local_stamp_ns = raw.header.stamp.to_nsec()
        observation.utc_stamp_ns = parsed.utc_ns
        observation.time_uncertainty_ns = max(raw.time_uncertainty_ns, 50_000_000)
        observation.association_type = (
            UtcObservation.REPLAY_PRESERVED
            if replay and self.replay_has_pps_association
            else UtcObservation.HOST_SERIAL_ASSOCIATED
        )
        observation.valid = parsed.utc_valid and observation.local_stamp_ns > 0
        observation.detail = "{} serial epoch; not PPS unless configured replay corpus".format(
            parsed.source
        )
        self.utc_pub.publish(observation)

    def _publish_pvt(self, parsed, raw, replay):
        pvt = GnssPVTSolnMsg()
        if parsed.utc_valid:
            gps_seconds = parsed.utc_ns / 1e9 - GPS_EPOCH_UNIX_S + self.gps_utc_leap_seconds
            if gps_seconds >= 0:
                pvt.time.week = int(gps_seconds // GPS_WEEK_S)
                pvt.time.tow = gps_seconds - pvt.time.week * GPS_WEEK_S
        pvt.fix_type = parsed.fix_type
        pvt.valid_fix = parsed.valid_fix
        pvt.diff_soln = parsed.diff_soln
        pvt.carr_soln = parsed.carr_soln
        pvt.num_sv = parsed.num_sv
        pvt.latitude = finite_or(parsed.latitude, 0.0)
        pvt.longitude = finite_or(parsed.longitude, 0.0)
        pvt.altitude = finite_or(parsed.altitude, 0.0)
        pvt.height_msl = finite_or(parsed.height_msl, pvt.altitude)
        pvt.h_acc = finite_or(parsed.h_acc, 999.0)
        pvt.v_acc = finite_or(parsed.v_acc, 999.0)
        pvt.p_dop = finite_or(parsed.p_dop, 999.0)
        pvt.vel_n = finite_or(parsed.vel_n, 0.0)
        pvt.vel_e = finite_or(parsed.vel_e, 0.0)
        pvt.vel_d = finite_or(parsed.vel_d, 0.0)
        pvt.vel_acc = finite_or(parsed.vel_acc, 999.0)
        self.compat_pub.publish(pvt)

        local_ns, local_valid = self._utc_to_local(parsed.utc_ns) if parsed.utc_valid else (0, False)
        stamped = GnssPvtStamped()
        stamped.header.stamp = rospy.Time.from_sec(local_ns / 1e9) if local_valid else rospy.Time()
        stamped.header.seq = raw.source_sequence
        stamped.header.frame_id = "local_sensor_time"
        stamped.session_id = raw.session_id
        stamped.writer_epoch = raw.writer_epoch
        stamped.source_sequence = raw.source_sequence
        stamped.pvt = pvt
        stamped.utc_measurement_ns = parsed.utc_ns
        stamped.local_measurement_ns = local_ns
        stamped.mapping_version = self.mapping.mapping_version if local_valid else 0
        stamped.time_state = self.mapping.time_state if self.mapping else TimeMapping.LOCAL_ONLY
        stamped.timestamp_source = (
            GnssPvtStamped.GNSS_UTC_INVERSE_MAPPED if local_valid else GnssPvtStamped.INVALID
        )
        stamped.timestamp_uncertainty_ns = (
            max(1, self.mapping.time_uncertainty_ns) if local_valid else 0
        )
        stamped.utc_valid = parsed.utc_valid
        stamped.local_measurement_time_valid = local_valid
        stamped.valid_for_fusion = valid_for_fusion(
            local_valid, parsed.position_valid, parsed.quality,
            self.accepted_qualities
        )
        self.local_pub.publish(stamped)
        if parsed.utc_valid:
            self._publish_utc_observation(parsed, raw, replay)

    def _publish_status(self, raw, parsed, detail):
        status = GnssStatus()
        status.header = raw.header
        status.session_id = raw.session_id
        status.writer_epoch = raw.writer_epoch
        status.mapping_version = self.mapping.mapping_version if self.mapping else 0
        status.time_state = self.mapping.time_state if self.mapping else TimeMapping.LOCAL_ONLY
        status.serial_open = self.serial_open
        status.utc_valid = bool(parsed and parsed.utc_valid)
        status.local_measurement_time_valid = self.last_local_measurement_ns > 0
        status.position_valid = bool(parsed and parsed.position_valid)
        status.valid_for_fusion = (
            status.local_measurement_time_valid
            and status.position_valid
            and parsed.quality in self.accepted_qualities
        ) if parsed else False
        status.solution_quality = parsed.quality if parsed else 0
        status.source_sequence = raw.source_sequence
        status.checksum_error_count = self.checksum_errors
        status.parse_error_count = self.parse_errors
        status.reconnect_count = self.reconnect_count
        status.source_message = parsed.source if parsed else ""
        status.detail = detail
        self.status_pub.publish(status)


if __name__ == "__main__":
    rospy.init_node("gnss_serial_driver")
    GnssSerialNode()
    rospy.spin()
