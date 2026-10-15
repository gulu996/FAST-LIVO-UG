"""Integer-nanosecond, shadow-only clock mapping. No ROS or hardware I/O.

Call observations in recorded MONOTONIC order. Offline callers must replay the
recorded clock/anchor observations, never a current live timeshare file.
An accepted estimate is a receive-reference clock estimate, NOT exposure time.
"""

from collections import OrderedDict, deque
from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass
from fractions import Fraction
from statistics import median

from sensor_time_bridge.imu_anchor import (
    IMU_TIME_ANCHOR_MAGIC, IMU_TIME_ANCHOR_VERSION, ImuAnchorSnapshot,
)

INT64_MIN = -(1 << 63)
INT64_MAX = (1 << 63) - 1
STEREO_STREAMS = frozenset(("left", "right", "left_info", "right_info", "image", "stereo"))


def int64(value, positive=False):
    """Do not silently accept float ns, booleans, or uint64 overflow."""
    if type(value) is not int:
        raise ValueError("NOT_INTEGER_NS")
    if not INT64_MIN <= value <= INT64_MAX:
        raise ValueError("INT64_OVERFLOW")
    if positive and value <= 0:
        raise ValueError("NONPOSITIVE_TIMESTAMP")
    return value


def round_fraction(value):
    """Nearest integer, ties away from zero; no floating epoch arithmetic."""
    n, d = value.numerator, value.denominator
    return (1 if n >= 0 else -1) * ((2 * abs(n) + d) // (2 * d))


def _median_twice(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    return 2 * ordered[middle] if len(ordered) & 1 else ordered[middle - 1] + ordered[middle]


@dataclass(frozen=True)
class Config:
    compensation_ns: int = 0
    expected_clock_source: int = 2
    max_clock_bracket_ns: int = 100_000
    clock_jump_ns: int = 2_000_000
    suspend_jump_ns: int = 1_000_000
    device_jump_ns: int = 20_000_000
    max_clock_age_ns: int = 100_000_000
    max_clock_extrapolation_ns: int = 100_000_000
    max_anchor_age_ns: int = 100_000_000
    max_anchor_delta_ns: int = 100_000_000
    warn_anchor_delta_ns: int = 20_000_000
    max_sequence_stall_ns: int = 100_000_000
    clock_residual_limit_ns: int = 100_000
    device_residual_limit_ns: int = 5_000_000
    max_clock_ppm: int = 2_000
    max_device_ppm: int = 10_000
    min_fit_points: int = 3
    min_fit_span_ns: int = 20_000_000
    clock_window: int = 64
    anchor_window: int = 128
    anchor_fit_interval_ns: int = 50_000_000
    history_size: int = 1024
    stereo_cache_size: int = 512

    def __post_init__(self):
        for name, value in asdict(self).items():
            int64(value)
            if name != "compensation_ns" and value <= 0:
                raise ValueError("INVALID_CONFIG:" + name)
        if self.expected_clock_source not in (1, 2):
            raise ValueError("INVALID_CLOCK_SOURCE_CONFIG")
        if self.min_fit_points < 3 or min(self.clock_window, self.anchor_window) < self.min_fit_points:
            raise ValueError("INVALID_FIT_WINDOW")
        if self.history_size < max(self.clock_window, self.anchor_window):
            raise ValueError("HISTORY_TOO_SMALL")
        if self.warn_anchor_delta_ns > self.max_anchor_delta_ns:
            raise ValueError("INVALID_ANCHOR_THRESHOLDS")


@dataclass(frozen=True)
class AffineFit:
    x0_ns: int
    y0_ns: int
    slope: Fraction
    intercept: Fraction
    first_x_ns: int
    last_x_ns: int
    residual_ns: int
    rejected_count: int
    sample_count: int
    version: int

    def forward(self, x):
        return Fraction(self.y0_ns) + self.intercept + self.slope * int64(x - self.x0_ns)

    def inverse(self, y):
        return int64(self.x0_ns + round_fraction((int64(y - self.y0_ns) - self.intercept) / self.slope))

    @property
    def ppm(self):
        return float((self.slope - 1) * 1_000_000)


def fit_affine(points, residual_limit_ns, max_ppm, min_points, min_span_ns, version):
    """Robust centered rational fit, bounded O(n log n), never epoch doubles.

    ponytail: paired half-window slopes give an O(n) robust seed, not an
    O(n^2) Theil-Sen fit; strongly structured contamination is rejected by
    residual/inlier/slope gates, not claimed to be a calibrated noise model.
    """
    if len(points) < min_points or points[-1][0] - points[0][0] < min_span_ns:
        return None, "FIT_WARMUP"
    x0, y0 = points[len(points) // 2]
    centered = [(int64(x - x0), int64(y - y0)) for x, y in points]
    half = len(points) // 2
    slopes = [Fraction(centered[i + half][1] - centered[i][1],
                       centered[i + half][0] - centered[i][0]) for i in range(half)]
    slope = median(slopes)
    # All residuals share a denominator: compare their integer numerators and
    # construct Fraction only for the final model, avoiding per-sample objects.
    sn, sd = slope.numerator, slope.denominator
    offsets = [y * sd - sn * x for x, y in centered]
    intercept_twice = _median_twice(offsets)
    deviations = [abs(2 * value - intercept_twice) for value in offsets]
    cutoff = min(2 * sd * residual_limit_ns, max(2 * sd * 1_000, 3 * _median_twice(deviations)))
    kept = [(x, y) for (x, y), residual in zip(centered, deviations) if residual <= cutoff]
    if len(kept) < min_points or 4 * len(kept) < 3 * len(points):
        return None, "FIT_RESIDUAL"
    n = len(kept)
    sx, sy = sum(x for x, _ in kept), sum(y for _, y in kept)
    denominator = n * sum(x * x for x, _ in kept) - sx * sx
    if denominator <= 0:
        return None, "FIT_DEGENERATE"
    slope = Fraction(n * sum(x * y for x, y in kept) - sx * sy, denominator)
    if slope <= 0 or abs(slope - 1) * 1_000_000 > max_ppm:
        return None, "FIT_SLOPE"
    intercept = Fraction(sy, n) - slope * Fraction(sx, n)
    sn, sd, bn, bd = slope.numerator, slope.denominator, intercept.numerator, intercept.denominator
    residual = Fraction(max(abs(y * sd * bd - bn * sd - sn * x * bd) for x, y in kept), sd * bd)
    if residual > residual_limit_ns:
        return None, "FIT_RESIDUAL"
    return AffineFit(x0, y0, slope, intercept, points[0][0], points[-1][0],
                     round_fraction(residual), len(points) - n, n, version), "OK"


@dataclass(frozen=True)
class MappingResult:
    raw_stamp_ns: int
    stream: str
    arrival_wall_ns: int
    arrival_mono_ns: int
    session_id: str
    mapping_version: str
    valid: bool
    reason: str
    mapped_stamp_ns: object = None
    processing_mono_ns: object = None
    mapped_mono_ns: object = None
    wall_minus_raw_ns: object = None
    clock_segment: int = 0
    anchor_segment: int = 0
    clock_fit_version: int = 0
    anchor_fit_version: int = 0
    clock_slope_num: object = None
    clock_slope_den: object = None
    device_slope_num: object = None
    device_slope_den: object = None
    clock_ppm: object = None
    device_ppm: object = None
    clock_residual_ns: object = None
    device_residual_ns: object = None
    clock_fit_rejected_count: int = 0
    device_fit_rejected_count: int = 0
    clock_bracket_ns: object = None
    writer_epoch: int = 0
    anchor_sequence: int = 0
    writer_latest_sequence: int = 0
    writer_latest_stamp_ns: object = None
    writer_latest_host_mono_ns: object = None
    writer_latest_update_mono_ns: object = None
    anchor_stamp_ns: object = None
    anchor_host_mono_ns: object = None
    anchor_update_mono_ns: object = None
    anchor_uncertainty_config_ns: object = None
    anchor_clock_source: object = None
    anchor_ready: bool = False
    anchor_age_ns: object = None
    anchor_update_age_ns: object = None
    sequence_age_ns: object = None
    anchor_delta_ns: object = None
    compensation_ns: int = 0
    warning: bool = False
    mode: str = "SHADOW_ONLY"
    physical_error_bound: str = "UNKNOWN"
    exposure_reference: str = "UNKNOWN"

    def as_dict(self):
        return asdict(self)


class ShadowMapper:
    """Single-thread owner; callers serialize event/clock/anchor observation.

    Group results are snapshots as of the first group message. A repeated
    CameraInfo retrieves that immutable result, not a fresh current mapping.
    Segment transitions retire the cache; no anchor crosses a segment.
    An unannounced raw regression locks that stream until zed_session changes.
    """

    def __init__(self, config=None):
        self.config = config or Config()
        self.boot_id = None
        self.clock_segment = 0
        self.anchor_segment = 0
        self._version = 0
        self._clock_points = deque(maxlen=self.config.clock_window)
        self._clock_models = deque(maxlen=self.config.history_size)
        self._anchor_points = deque(maxlen=self.config.anchor_window)
        self._anchor_models = deque(maxlen=self.config.history_size)
        self._anchors = deque(maxlen=self.config.history_size)
        self._anchor_times = deque(maxlen=self.config.history_size)
        self._clock_status = "CLOCK_MISSING"
        self._anchor_status = "ANCHOR_MISSING"
        self._latest_anchor = None
        self._anchor_key = None
        self._sequence_advanced_ns = None
        self._last_clock_m = None
        self._last_clock_r = None
        self._last_boot_offset = None
        self._last_bracket = None
        self._clock_start_m = None
        self._anchor_start_m = None
        self._zed_session = None
        self._camera_generation = 0
        self._retired_zed_sessions = set()
        self._last_raw = {}
        self._blocked_streams = {}
        self._last_mapped = {}
        self._groups = OrderedDict()
        self.counters = {"clock_observations": 0, "clock_rejected": 0,
                         "anchor_observations": 0, "stereo_reuses": 0,
                         "mapped": 0, "invalid": 0}

    def _new_anchor_segment(self, reason):
        self.anchor_segment += 1
        self._anchors.clear()
        self._anchor_times.clear()
        self._anchor_points.clear()
        self._anchor_models.clear()
        self._latest_anchor = None
        self._sequence_advanced_ns = None
        self._anchor_start_m = None
        self._anchor_key = None
        self._anchor_status = reason
        self._groups.clear()
        self._last_mapped.clear()

    def _new_clock_segment(self, reason):
        self.clock_segment += 1
        self._clock_points.clear()
        self._clock_models.clear()
        self._clock_start_m = None
        self._clock_status = reason
        self._last_clock_m = None
        self._last_clock_r = None
        self._last_raw.clear()
        self._blocked_streams.clear()
        self._new_anchor_segment(reason)

    def observe_clock(self, m_before_ns, realtime_ns, m_after_ns, boot_id, boottime_ns=None):
        self.counters["clock_observations"] += 1
        result = {"accepted": False, "reason": "OK"}
        try:
            for value in (m_before_ns, realtime_ns, m_after_ns):
                int64(value, True)
            if not isinstance(boot_id, str) or not boot_id:
                raise ValueError("HOST_BOOT_ID_MISSING")
            width = int64(m_after_ns - m_before_ns)
            if width < 0:
                raise ValueError("CLOCK_BRACKET_REVERSED")
            if width > self.config.max_clock_bracket_ns:
                raise ValueError("CLOCK_BRACKET_TOO_WIDE")
            middle = m_before_ns + width // 2
            event = "OK"
            if boot_id != self.boot_id:
                event = "HOST_BOOT_CHANGED" if self.boot_id is not None else "CLOCK_START"
                self.boot_id = boot_id
                self._last_boot_offset = None
                self._new_clock_segment(event)
            if boottime_ns is not None:
                offset = int64(int64(boottime_ns, True) - middle)
                if self._last_boot_offset is not None and abs(offset - self._last_boot_offset) > self.config.suspend_jump_ns:
                    event = "HOST_SUSPEND_RESUME"
                    self._new_clock_segment(event)
                self._last_boot_offset = offset
            if self._last_clock_m is not None:
                if middle <= self._last_clock_m:
                    event = "MONOTONIC_DISCONTINUITY"
                    self._new_clock_segment(event)
                else:
                    predicted = (self._clock_models[-1].forward(middle) if self._clock_models
                                 else self._last_clock_r + middle - self._last_clock_m)
                    if abs(realtime_ns - predicted) > self.config.clock_jump_ns:
                        event = "REALTIME_JUMP"
                        self._new_clock_segment(event)
            if self._clock_start_m is None:
                self._clock_start_m = middle
            self._last_clock_m, self._last_clock_r = middle, realtime_ns
            self._last_bracket = width
            self._clock_points.append((middle, realtime_ns))
            self._version += 1
            model, status = fit_affine(list(self._clock_points), self.config.clock_residual_limit_ns,
                                      self.config.max_clock_ppm, self.config.min_fit_points,
                                      self.config.min_fit_span_ns, self._version)
            self._clock_status = "OK" if model else "CLOCK_" + status
            if model:
                self._clock_models.append(model)
            result.update(accepted=True, reason=event, fit_status=self._clock_status,
                          bracket_ns=width, midpoint_mono_ns=middle,
                          clock_fit_version=model.version if model else 0,
                          clock_ppm=model.ppm if model else None,
                          clock_residual_ns=model.residual_ns if model else None,
                          clock_fit_rejected_count=model.rejected_count if model else 0)
        except ValueError as error:
            self.counters["clock_rejected"] += 1
            result["reason"] = str(error)
        result.update(clock_segment=self.clock_segment, anchor_segment=self.anchor_segment,
                      boot_id=self.boot_id)
        return result

    def observe_anchor(self, snapshot, now_mono_ns, identity=None, read_error=""):
        self.counters["anchor_observations"] += 1
        result = {"accepted": False, "reason": "OK"}
        try:
            int64(now_mono_ns, True)
            if read_error or snapshot is None:
                raise ValueError("ANCHOR_READ_" + (read_error or "missing").upper())
            if self.boot_id is None:
                raise ValueError("ANCHOR_NO_CLOCK_SESSION")
            fields = {name: getattr(snapshot, name) for name in ImuAnchorSnapshot.__dataclass_fields__}
            anchor = ImuAnchorSnapshot(**fields)
            if type(anchor.ready) is not bool or type(anchor.clock_source) is not int:
                raise ValueError("ANCHOR_PROTOCOL")
            if (type(anchor.magic) is not int or type(anchor.version) is not int
                    or anchor.magic != IMU_TIME_ANCHOR_MAGIC or anchor.version != IMU_TIME_ANCHOR_VERSION):
                raise ValueError("ANCHOR_PROTOCOL")
            for name in ("writer_epoch", "write_sequence"):
                value = getattr(anchor, name)
                if type(value) is not int or not 0 < value < (1 << 64):
                    raise ValueError("ANCHOR_PROTOCOL")
            for name in ("imu_stamp_ns", "host_monotonic_ns", "update_monotonic_ns"):
                int64(getattr(anchor, name), True)
            int64(anchor.uncertainty_ns)
            if anchor.uncertainty_ns < 0 or anchor.write_sequence & 1:
                raise ValueError("ANCHOR_PROTOCOL")
            if not anchor.ready:
                raise ValueError("ANCHOR_NOT_READY")
            if anchor.clock_source != self.config.expected_clock_source:
                raise ValueError("ANCHOR_CLOCK_SOURCE")
            if not anchor.host_monotonic_ns <= anchor.update_monotonic_ns <= now_mono_ns:
                raise ValueError("ANCHOR_MONOTONIC_ORDER")
            key = (anchor.writer_epoch, tuple(identity) if identity is not None else None)
            event = "OK"
            if self._anchor_key is not None and self._anchor_key != key:
                event = ("WRITER_EPOCH_CHANGED" if self._anchor_key[0] != key[0]
                         else "ANCHOR_INODE_CHANGED")
                self._new_anchor_segment(event)
            self._anchor_key = key
            previous = self._latest_anchor
            if previous is not None:
                if anchor.write_sequence == previous.write_sequence:
                    if anchor != previous:
                        raise ValueError("ANCHOR_SEQUENCE_REUSED")
                    if now_mono_ns - self._sequence_advanced_ns > self.config.max_sequence_stall_ns:
                        raise ValueError("ANCHOR_SEQUENCE_STALLED")
                    if max(now_mono_ns - anchor.host_monotonic_ns,
                           now_mono_ns - anchor.update_monotonic_ns) > self.config.max_anchor_age_ns:
                        raise ValueError("ANCHOR_STALE")
                    return dict(accepted=True, reason="ANCHOR_UNCHANGED", anchor_segment=self.anchor_segment,
                                fit_status=self._anchor_status,
                                anchor_age_ns=now_mono_ns - anchor.host_monotonic_ns,
                                anchor_update_age_ns=now_mono_ns - anchor.update_monotonic_ns,
                                sequence_age_ns=now_mono_ns - self._sequence_advanced_ns)
                if anchor.write_sequence < previous.write_sequence:
                    event = "ANCHOR_SEQUENCE_REGRESSION"
                elif anchor.imu_stamp_ns <= previous.imu_stamp_ns:
                    event = "LIVOX_TIME_REGRESSION"
                elif anchor.host_monotonic_ns <= previous.host_monotonic_ns:
                    event = "ANCHOR_MONOTONIC_REGRESSION"
                elif abs((anchor.imu_stamp_ns - previous.imu_stamp_ns)
                         - (anchor.host_monotonic_ns - previous.host_monotonic_ns)) > self.config.device_jump_ns:
                    event = "LIVOX_TIME_JUMP"
                elif now_mono_ns - self._sequence_advanced_ns > self.config.max_sequence_stall_ns:
                    event = "ANCHOR_UPDATE_GAP"
                if event != "OK":
                    self._new_anchor_segment(event)
                    self._anchor_key = key
            if max(now_mono_ns - anchor.host_monotonic_ns,
                   now_mono_ns - anchor.update_monotonic_ns) > self.config.max_anchor_age_ns:
                raise ValueError("ANCHOR_STALE")
            self._latest_anchor = anchor
            self._sequence_advanced_ns = now_mono_ns
            if self._anchor_start_m is None:
                self._anchor_start_m = anchor.host_monotonic_ns
            self._anchors.append(anchor)
            self._anchor_times.append(anchor.host_monotonic_ns)
            self._anchor_points.append((anchor.host_monotonic_ns, anchor.imu_stamp_ns))
            # Anchor reads/history remain at full rate; slope changes much more
            # slowly. Bound CPU with 20 Hz fits and gate each selected anchor's
            # residual before applying the held slope.
            if (self._anchor_models and anchor.host_monotonic_ns - self._anchor_models[-1].last_x_ns
                    < self.config.anchor_fit_interval_ns):
                return dict(accepted=True, reason=event, fit_status=self._anchor_status,
                            anchor_segment=self.anchor_segment,
                            anchor_fit_version=self._anchor_models[-1].version,
                            device_ppm=self._anchor_models[-1].ppm,
                            device_residual_ns=self._anchor_models[-1].residual_ns,
                            device_fit_rejected_count=self._anchor_models[-1].rejected_count)
            self._version += 1
            model, status = fit_affine(list(self._anchor_points), self.config.device_residual_limit_ns,
                                      self.config.max_device_ppm, self.config.min_fit_points,
                                      self.config.min_fit_span_ns, self._version)
            self._anchor_status = "OK" if model else "ANCHOR_" + status
            if model:
                self._anchor_models.append(model)
            result.update(accepted=True, reason=event, fit_status=self._anchor_status,
                          anchor_fit_version=model.version if model else 0,
                          device_ppm=model.ppm if model else None,
                          device_residual_ns=model.residual_ns if model else None,
                          device_fit_rejected_count=model.rejected_count if model else 0)
        except (ValueError, AttributeError, TypeError) as error:
            reason = str(error) if isinstance(error, ValueError) else "ANCHOR_PROTOCOL"
            if self._anchor_status != reason or self._latest_anchor is not None:
                self._new_anchor_segment(reason)
            result["reason"] = reason
        result["anchor_segment"] = self.anchor_segment
        return result

    @staticmethod
    def _model_for(models, target, inverse=False):
        # Recent real-time events hit the newest fit; historical replay searches
        # bounded recorded fits instead of silently applying today's offset.
        for model in reversed(models):
            coordinate = model.inverse(target) if inverse else target
            if coordinate >= model.first_x_ns:
                return model, coordinate
        return None, None

    def _session_id(self):
        return "%s/c%d/a%d/z%s/g%d" % (self.boot_id or "unknown", self.clock_segment,
                                      self.anchor_segment, self._zed_session or "unknown", self._camera_generation)

    def _freshness(self, now_mono_ns, values):
        """Retire stale sessions before even a cached group can be returned."""
        if self._clock_status != "OK":
            reason = self._clock_status
            if self._clock_models:
                self._new_clock_segment(reason)
            return reason
        if now_mono_ns < self._last_clock_m:
            return "EVENT_ARRIVAL_BEFORE_OBSERVATION"
        if now_mono_ns - self._last_clock_m > self.config.max_clock_age_ns:
            self._new_clock_segment("CLOCK_STALE")
            return "CLOCK_STALE"
        latest = self._latest_anchor
        if latest is None:
            return self._anchor_status
        ages = (int64(now_mono_ns - latest.host_monotonic_ns),
                int64(now_mono_ns - latest.update_monotonic_ns),
                int64(now_mono_ns - self._sequence_advanced_ns))
        values.update(anchor_age_ns=ages[0], anchor_update_age_ns=ages[1], sequence_age_ns=ages[2],
                      writer_epoch=latest.writer_epoch, anchor_sequence=latest.write_sequence,
                      writer_latest_sequence=latest.write_sequence, writer_latest_stamp_ns=latest.imu_stamp_ns,
                      writer_latest_host_mono_ns=latest.host_monotonic_ns,
                      writer_latest_update_mono_ns=latest.update_monotonic_ns)
        if min(ages) < 0:
            return "EVENT_ARRIVAL_BEFORE_OBSERVATION"
        reason = ("ANCHOR_SEQUENCE_STALLED" if ages[2] > self.config.max_sequence_stall_ns
                  else "ANCHOR_STALE" if max(ages[:2]) > self.config.max_anchor_age_ns
                  else self._anchor_status)
        if reason != "OK" and self._anchor_models:
            self._new_anchor_segment(reason)
        return reason

    def map_event(self, stream, raw_stamp_ns, arrival_wall_ns, arrival_mono_ns, zed_session="default", now_mono_ns=None):
        if now_mono_ns is None:
            now_mono_ns = arrival_mono_ns
        group_stream = "stereo" if stream in STEREO_STREAMS else stream
        values = dict(raw_stamp_ns=raw_stamp_ns, stream=group_stream,
                      arrival_wall_ns=arrival_wall_ns, arrival_mono_ns=arrival_mono_ns,
                      session_id=self._session_id(), mapping_version="none", valid=False, reason="UNKNOWN",
                      clock_segment=self.clock_segment, anchor_segment=self.anchor_segment,
                      compensation_ns=self.config.compensation_ns, clock_bracket_ns=self._last_bracket,
                      processing_mono_ns=now_mono_ns)
        cache_key = None
        try:
            for value in (raw_stamp_ns, arrival_wall_ns, arrival_mono_ns, now_mono_ns):
                int64(value, True)
            if now_mono_ns < arrival_mono_ns:
                raise ValueError("PROCESSING_BEFORE_ARRIVAL")
            if not isinstance(stream, str) or not stream:
                raise ValueError("INVALID_STREAM")
            if not isinstance(zed_session, str) or not zed_session:
                raise ValueError("ZED_SESSION_MISSING")
            if zed_session in self._retired_zed_sessions:
                raise ValueError("ZED_RETIRED_SESSION")
            session_changed = False
            if zed_session != self._zed_session:
                session_changed = self._zed_session is not None
                if self._zed_session is not None:
                    self._retired_zed_sessions.add(self._zed_session)
                    self._camera_generation += 1
                self._zed_session = zed_session
                self._last_raw.clear()
                self._last_mapped.clear()
                self._blocked_streams.clear()
                self._groups.clear()
            values["session_id"] = self._session_id()
            values["wall_minus_raw_ns"] = int64(arrival_wall_ns - raw_stamp_ns)
            cache_key = (values["session_id"], raw_stamp_ns) if group_stream == "stereo" else None
            if group_stream in self._blocked_streams:
                raise ValueError(self._blocked_streams[group_stream])
            if cache_key not in self._groups:
                last = self._last_raw.get(group_stream)
                if last is not None and raw_stamp_ns < last:
                    self._blocked_streams[group_stream] = "ZED_TIMESTAMP_REGRESSION"
                    self._camera_generation += 1
                    self._groups.clear()
                    self._last_mapped.clear()
                    raise ValueError("ZED_TIMESTAMP_REGRESSION")
                if last == raw_stamp_ns:
                    raise ValueError("ZED_TIMESTAMP_DUPLICATE")
                self._last_raw[group_stream] = raw_stamp_ns
            if session_changed:
                raise ValueError("ZED_SESSION_CHANGED")
            reason = self._freshness(now_mono_ns, values)
            if cache_key in self._groups and (reason == "OK" or not self._groups[cache_key].valid):
                self.counters["stereo_reuses"] += 1
                return self._groups[cache_key]
            if reason != "OK":
                raise ValueError(reason)
            clock_model, target_m = self._model_for(self._clock_models, raw_stamp_ns, True)
            if clock_model is None or target_m < self._clock_start_m:
                raise ValueError("CLOCK_OUTSIDE_SESSION")
            if target_m - clock_model.last_x_ns > self.config.max_clock_extrapolation_ns:
                raise ValueError("CLOCK_EXTRAPOLATION")
            values.update(mapped_mono_ns=target_m, clock_fit_version=clock_model.version,
                          clock_slope_num=clock_model.slope.numerator, clock_slope_den=clock_model.slope.denominator,
                          clock_ppm=clock_model.ppm, clock_residual_ns=clock_model.residual_ns,
                          clock_fit_rejected_count=clock_model.rejected_count)
            if target_m < self._anchor_start_m:
                raise ValueError("ANCHOR_OUTSIDE_SESSION")
            anchor_model, _ = self._model_for(self._anchor_models, target_m)
            if anchor_model is None:
                raise ValueError("ANCHOR_OUTSIDE_HISTORY")
            lower = bisect_left(self._anchor_times, anchor_model.first_x_ns)
            upper = bisect_right(self._anchor_times, anchor_model.last_x_ns + self.config.anchor_fit_interval_ns)
            if lower == upper:
                raise ValueError("ANCHOR_FIT_NO_INLIER")
            index = min(max(bisect_left(self._anchor_times, target_m), lower), upper - 1)
            candidates = [self._anchors[index]]
            if index > lower:
                candidates.append(self._anchors[index - 1])
            anchor = min(candidates, key=lambda a: (abs(target_m - a.host_monotonic_ns), a.host_monotonic_ns))
            if abs(anchor_model.forward(anchor.host_monotonic_ns) - anchor.imu_stamp_ns) > self.config.device_residual_limit_ns:
                raise ValueError("ANCHOR_FIT_RESIDUAL")
            delta = int64(target_m - anchor.host_monotonic_ns)
            values.update(anchor_fit_version=anchor_model.version,
                          mapping_version="r%d-d%d" % (clock_model.version, anchor_model.version),
                          device_slope_num=anchor_model.slope.numerator, device_slope_den=anchor_model.slope.denominator,
                          device_ppm=anchor_model.ppm, device_residual_ns=anchor_model.residual_ns,
                          device_fit_rejected_count=anchor_model.rejected_count,
                          anchor_sequence=anchor.write_sequence, anchor_stamp_ns=anchor.imu_stamp_ns,
                          anchor_host_mono_ns=anchor.host_monotonic_ns,
                          anchor_update_mono_ns=anchor.update_monotonic_ns,
                          anchor_uncertainty_config_ns=anchor.uncertainty_ns,
                          anchor_clock_source=anchor.clock_source, anchor_ready=anchor.ready,
                          anchor_delta_ns=delta, warning=abs(delta) > self.config.warn_anchor_delta_ns)
            if abs(delta) > self.config.max_anchor_delta_ns:
                raise ValueError("ANCHOR_TARGET_DISTANCE")
            mapped = int64(anchor.imu_stamp_ns + round_fraction(anchor_model.slope * delta)
                           - self.config.compensation_ns, True)
            previous = self._last_mapped.get(group_stream)
            if previous is not None and mapped <= previous:
                raise ValueError("MAPPED_TIMESTAMP_NONMONOTONIC")
            self._last_mapped[group_stream] = mapped
            values.update(mapped_stamp_ns=mapped, valid=True, reason="OK")
        except ValueError as error:
            values["reason"] = str(error)
        values.update(session_id=self._session_id(), clock_segment=self.clock_segment,
                      anchor_segment=self.anchor_segment)
        result = MappingResult(**values)
        self.counters["mapped" if result.valid else "invalid"] += 1
        if cache_key is not None:
            cache_key = (values["session_id"], raw_stamp_ns)
            self._groups[cache_key] = result
            if len(self._groups) > self.config.stereo_cache_size:
                self._groups.popitem(last=False)
        return result

    def status(self, now_mono_ns=None):
        current = {}
        if now_mono_ns is not None:
            current["current_reason"] = self._freshness(int64(now_mono_ns, True), current)
        return dict(mode="SHADOW_ONLY", boot_id=self.boot_id, session_id=self._session_id(),
                    clock_segment=self.clock_segment, anchor_segment=self.anchor_segment,
                    clock_status=self._clock_status, anchor_status=self._anchor_status,
                    clock_points=len(self._clock_points), anchor_points=len(self._anchor_points),
                    anchor_history=len(self._anchors), stereo_groups=len(self._groups),
                    counters=dict(self.counters), **current)
