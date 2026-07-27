#!/usr/bin/env python3
import mmap
import os
import struct
from dataclasses import dataclass
from typing import Optional


IMU_TIME_ANCHOR_MAGIC = 0x31414D49  # "IMA1" as little-endian bytes
IMU_TIME_ANCHOR_VERSION = 1
IMU_TIME_ANCHOR_FORMAT = "<IIQQQQQIIQ"
IMU_TIME_ANCHOR_SIZE = struct.calcsize(IMU_TIME_ANCHOR_FORMAT)
IMU_ANCHOR_CLOCK_PTP = 1
IMU_ANCHOR_CLOCK_GPS = 2
UINT64_MAX = (1 << 64) - 1
ROS_TIME_MAX_NS = ((1 << 32) - 1) * 1_000_000_000 + 999_999_999

assert IMU_TIME_ANCHOR_SIZE == 64


@dataclass(frozen=True)
class ImuAnchorSnapshot:
    magic: int
    version: int
    writer_epoch: int
    write_sequence: int
    imu_stamp_ns: int
    host_monotonic_ns: int
    update_monotonic_ns: int
    clock_source: int
    ready: bool
    uncertainty_ns: int


@dataclass(frozen=True)
class AnchorReadResult:
    success: bool
    snapshot: Optional[ImuAnchorSnapshot] = None
    failure_reason: str = ""


@dataclass(frozen=True)
class TimeMappingResult:
    success: bool
    mapped_stamp_ns: int = 0
    writer_epoch: int = 0
    write_sequence: int = 0
    anchor_delta_ns: int = 0
    anchor_uncertainty_ns: int = 0
    warning: bool = False
    failure_reason: str = ""


@dataclass
class ImuAnchorMapperStats:
    mapped_count: int = 0
    not_ready_count: int = 0
    stale_count: int = 0
    invalid_count: int = 0
    epoch_change_count: int = 0
    nonmonotonic_drop_count: int = 0
    mapping_error_count: int = 0
    warning_count: int = 0


def pack_anchor(snapshot: ImuAnchorSnapshot) -> bytes:
    return struct.pack(
        IMU_TIME_ANCHOR_FORMAT,
        snapshot.magic,
        snapshot.version,
        snapshot.writer_epoch,
        snapshot.write_sequence,
        snapshot.imu_stamp_ns,
        snapshot.host_monotonic_ns,
        snapshot.update_monotonic_ns,
        snapshot.clock_source,
        1 if snapshot.ready else 0,
        snapshot.uncertainty_ns,
    )


def decode_stable_snapshots(first: bytes, second: bytes) -> AnchorReadResult:
    if len(first) != IMU_TIME_ANCHOR_SIZE or len(second) != IMU_TIME_ANCHOR_SIZE:
        return AnchorReadResult(False, failure_reason="bad_length")
    first_values = struct.unpack(IMU_TIME_ANCHOR_FORMAT, first)
    second_values = struct.unpack(IMU_TIME_ANCHOR_FORMAT, second)
    first_sequence = first_values[3]
    second_sequence = second_values[3]
    if (
        first_sequence & 1
        or second_sequence & 1
        or first_sequence != second_sequence
        or first != second
    ):
        return AnchorReadResult(False, failure_reason="inconsistent")

    snapshot = ImuAnchorSnapshot(
        magic=second_values[0],
        version=second_values[1],
        writer_epoch=second_values[2],
        write_sequence=second_values[3],
        imu_stamp_ns=second_values[4],
        host_monotonic_ns=second_values[5],
        update_monotonic_ns=second_values[6],
        clock_source=second_values[7],
        ready=second_values[8] == 1,
        uncertainty_ns=second_values[9],
    )
    if snapshot.magic != IMU_TIME_ANCHOR_MAGIC:
        return AnchorReadResult(False, snapshot, "bad_magic")
    if snapshot.version != IMU_TIME_ANCHOR_VERSION:
        return AnchorReadResult(False, snapshot, "bad_version")
    if snapshot.writer_epoch == 0:
        return AnchorReadResult(False, snapshot, "invalid_epoch")
    if not snapshot.ready:
        return AnchorReadResult(False, snapshot, "not_ready")
    if snapshot.imu_stamp_ns == 0 or snapshot.host_monotonic_ns == 0:
        return AnchorReadResult(False, snapshot, "empty_anchor")
    if snapshot.clock_source not in (
        IMU_ANCHOR_CLOCK_PTP,
        IMU_ANCHOR_CLOCK_GPS,
    ):
        return AnchorReadResult(False, snapshot, "invalid_clock_source")
    return AnchorReadResult(True, snapshot)


class LivoxImuAnchorReader:
    """Recoverable reader for the 64-byte cross-process seqlock."""

    def __init__(self, path: str, max_attempts: int = 8):
        expanded = os.path.expanduser(path)
        if not os.path.isabs(expanded):
            raise ValueError("imu_timeshare_path must be absolute")
        self.path = expanded
        self.max_attempts = max(1, int(max_attempts))
        self.epoch_change_count = 0
        self._last_epoch = None
        self._fd = None
        self._mapping = None
        self._identity = None

    def close(self) -> None:
        if self._mapping is not None:
            self._mapping.close()
        if self._fd is not None:
            os.close(self._fd)
        self._mapping = None
        self._fd = None
        self._identity = None

    def _ensure_open(self) -> str:
        try:
            path_stat = os.stat(self.path)
        except FileNotFoundError:
            self.close()
            return "missing"
        except OSError:
            self.close()
            return "stat_error"

        identity = (path_stat.st_dev, path_stat.st_ino)
        if self._mapping is not None and self._fd is not None:
            try:
                descriptor_stat = os.fstat(self._fd)
                if (
                    identity == self._identity
                    and descriptor_stat.st_size == IMU_TIME_ANCHOR_SIZE
                    and path_stat.st_size == IMU_TIME_ANCHOR_SIZE
                ):
                    return ""
            except OSError:
                pass
            self.close()

        if path_stat.st_size != IMU_TIME_ANCHOR_SIZE:
            return "bad_length"
        fd = None
        try:
            fd = os.open(self.path, os.O_RDONLY)
            descriptor_stat = os.fstat(fd)
            if descriptor_stat.st_size != IMU_TIME_ANCHOR_SIZE:
                os.close(fd)
                return "bad_length"
            mapping = mmap.mmap(
                fd, IMU_TIME_ANCHOR_SIZE, access=mmap.ACCESS_READ
            )
        except (OSError, ValueError):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
            self.close()
            return "open_error"

        self._fd = fd
        self._mapping = mapping
        self._identity = (descriptor_stat.st_dev, descriptor_stat.st_ino)
        return ""

    def read(self) -> AnchorReadResult:
        failure = self._ensure_open()
        if failure:
            return AnchorReadResult(False, failure_reason=failure)

        result = AnchorReadResult(False, failure_reason="inconsistent")
        try:
            for _ in range(self.max_attempts):
                first = self._mapping[:IMU_TIME_ANCHOR_SIZE]
                second = self._mapping[:IMU_TIME_ANCHOR_SIZE]
                result = decode_stable_snapshots(first, second)
                if result.failure_reason != "inconsistent":
                    break
        except (BufferError, OSError, ValueError):
            self.close()
            return AnchorReadResult(False, failure_reason="read_error")

        snapshot = result.snapshot
        if snapshot is not None and snapshot.writer_epoch != 0:
            if self._last_epoch is None:
                self._last_epoch = snapshot.writer_epoch
            elif snapshot.writer_epoch != self._last_epoch:
                self._last_epoch = snapshot.writer_epoch
                self.epoch_change_count += 1
                return AnchorReadResult(False, snapshot, "epoch_changed")
        return result


class LivoxImuTimeMapper:
    def __init__(
        self,
        reader: LivoxImuAnchorReader,
        warn_age_s: float = 0.020,
        max_age_s: float = 0.100,
    ):
        if warn_age_s < 0.0 or max_age_s <= 0.0:
            raise ValueError("anchor ages must be nonnegative and max positive")
        if warn_age_s > max_age_s:
            raise ValueError("warn age must not exceed max age")
        self.reader = reader
        self.warn_delta_ns = int(round(warn_age_s * 1_000_000_000))
        self.max_delta_ns = int(round(max_age_s * 1_000_000_000))
        self.stats = ImuAnchorMapperStats()
        self.last_mapped_stamp_ns = 0

    def map_time(
        self,
        sensor_receive_monotonic_ns: int,
        configured_time_offset_ns: int = 0,
    ) -> TimeMappingResult:
        receive_ns = int(sensor_receive_monotonic_ns)
        offset_ns = int(configured_time_offset_ns)
        if receive_ns <= 0 or receive_ns > UINT64_MAX:
            self.stats.mapping_error_count += 1
            return TimeMappingResult(
                False, failure_reason="invalid_receive_time"
            )

        read_result = self.reader.read()
        self.stats.epoch_change_count = self.reader.epoch_change_count
        if not read_result.success:
            reason = read_result.failure_reason
            if reason in ("missing", "not_ready", "epoch_changed"):
                self.stats.not_ready_count += 1
            else:
                self.stats.invalid_count += 1
            return TimeMappingResult(False, failure_reason=reason)

        anchor = read_result.snapshot
        delta_ns = receive_ns - int(anchor.host_monotonic_ns)
        absolute_delta = abs(delta_ns)
        if absolute_delta > self.max_delta_ns:
            self.stats.stale_count += 1
            return TimeMappingResult(
                False,
                writer_epoch=anchor.writer_epoch,
                write_sequence=anchor.write_sequence,
                anchor_delta_ns=delta_ns,
                anchor_uncertainty_ns=anchor.uncertainty_ns,
                failure_reason="stale_anchor",
            )

        mapped_ns = int(anchor.imu_stamp_ns) + delta_ns - offset_ns
        if mapped_ns <= 0 or mapped_ns > ROS_TIME_MAX_NS:
            self.stats.mapping_error_count += 1
            return TimeMappingResult(
                False,
                writer_epoch=anchor.writer_epoch,
                write_sequence=anchor.write_sequence,
                anchor_delta_ns=delta_ns,
                anchor_uncertainty_ns=anchor.uncertainty_ns,
                failure_reason="mapped_time_out_of_range",
            )
        if mapped_ns <= self.last_mapped_stamp_ns:
            self.stats.nonmonotonic_drop_count += 1
            return TimeMappingResult(
                False,
                writer_epoch=anchor.writer_epoch,
                write_sequence=anchor.write_sequence,
                anchor_delta_ns=delta_ns,
                anchor_uncertainty_ns=anchor.uncertainty_ns,
                failure_reason="nonmonotonic_mapped_time",
            )

        warning = absolute_delta > self.warn_delta_ns
        if warning:
            self.stats.warning_count += 1
        self.last_mapped_stamp_ns = mapped_ns
        self.stats.mapped_count += 1
        return TimeMappingResult(
            True,
            mapped_stamp_ns=mapped_ns,
            writer_epoch=anchor.writer_epoch,
            write_sequence=anchor.write_sequence,
            anchor_delta_ns=delta_ns,
            anchor_uncertainty_ns=max(1, anchor.uncertainty_ns),
            warning=warning,
        )
