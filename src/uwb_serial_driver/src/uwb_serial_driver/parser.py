#!/usr/bin/env python3
import math
import re
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class ParsedRange:
    anchor_id: int
    raw_range_m: float
    corrected_range_m: float
    range_bias_m: float
    diag: int = -1
    valid: bool = True
    reject_reason: int = 0
    source_format: str = "unknown"


@dataclass
class DistanceRoundEvent:
    kind: str
    ranges: List[ParsedRange] = field(default_factory=list)
    timestamp_context: Any = None
    incomplete_round: bool = False

    @property
    def parse_error(self) -> bool:
        return self.kind in ("malformed_distance", "unknown")


class UwbParser:
    ACCEPTED = 0
    INVALID_STATUS = 1
    INVALID_RAW_RANGE = 2
    INVALID_CORRECTED_RANGE = 3
    RANGE_LIMIT = 4

    _number = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
    distance_pattern = re.compile(
        r"^\s*distance\s*\[\s*(\d+)\s*\]\s*,\s*(" + _number +
        r")\s*(?:,.*)?$", re.I
    )
    number_pattern = re.compile(_number)
    numeric_record_pattern = re.compile(
        r"^\s*" + _number + r"(?:\s*(?:,|;|\s)\s*" + _number + r")*\s*$"
    )
    uwbdbg_pattern = re.compile(r"^\s*\[UWBDBG\]", re.I)
    twr_pattern = re.compile(r"^\s*\[TWR\]", re.I)

    def __init__(self, parser_mode="auto", anchor_order=None, range_scale=1.0,
                 range_bias=None, min_range_m=0.05, max_range_m=250.0):
        self.parser_mode = parser_mode
        self.anchor_order = list(anchor_order or [])
        self.range_scale = range_scale
        self.range_bias: Dict[int, float] = dict(range_bias or {})
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m

    def make_range(self, anchor_id, raw, source, diag=-1):
        scaled = raw * self.range_scale
        bias = float(self.range_bias.get(anchor_id, 0.0))
        corrected = scaled - bias
        reason = self.ACCEPTED
        if not math.isfinite(scaled) or scaled <= 0.0:
            reason = self.INVALID_RAW_RANGE
        elif not math.isfinite(corrected) or corrected <= 0.0:
            reason = self.INVALID_CORRECTED_RANGE
        elif corrected < self.min_range_m or corrected > self.max_range_m:
            reason = self.RANGE_LIMIT
        return ParsedRange(anchor_id, scaled, corrected, bias, diag,
                           reason == self.ACCEPTED, reason, source)

    @classmethod
    def parse_distance_fields(cls, line: str) -> Optional[Tuple[int, float]]:
        match = cls.distance_pattern.match(line)
        if not match:
            return None
        try:
            return int(match.group(1)), float(match.group(2))
        except ValueError:
            return None

    def parse(self, line: str) -> List[ParsedRange]:
        """Parse legacy one-line modes.

        distance_round5 is deliberately handled only by
        DistanceRoundAssembler so a caller cannot accidentally publish a
        partial hardware round.
        """
        line = line.strip()
        if not line or self.uwbdbg_pattern.match(line) or self.twr_pattern.match(line):
            return []
        if self.parser_mode == "distance_round5":
            return []

        distance = self.parse_distance_fields(line)
        if distance is not None:
            return [self.make_range(distance[0], distance[1], "distance")]
        if self.parser_mode in ("distance", "uwb"):
            return []
        if self.parser_mode not in ("auto", "pairs", "values"):
            return []

        # ponytail: legacy numeric modes accept only an all-numeric record.
        # This keeps old replay files while making arbitrary debug text
        # impossible to reinterpret as ranges.
        if not self.numeric_record_pattern.match(line):
            return []
        values = [float(value) for value in self.number_pattern.findall(line)]
        parse_pairs = self.parser_mode == "pairs"
        if self.parser_mode == "auto" and len(values) >= 2 and len(values) % 2 == 0:
            parse_pairs = all(
                value.is_integer()
                and (not self.anchor_order or int(value) in self.anchor_order)
                for value in values[0::2]
            )
        if parse_pairs:
            return [
                self.make_range(int(values[index]), values[index + 1], "pairs")
                for index in range(0, len(values) - 1, 2)
                if values[index].is_integer()
            ]
        return [
            self.make_range(
                self.anchor_order[index] if index < len(self.anchor_order) else index,
                value, "values"
            )
            for index, value in enumerate(values)
        ]


class DistanceRoundAssembler:
    WAIT_DEBUG = "WAIT_DEBUG"
    WAIT_DISTANCE_LINES = "WAIT_DISTANCE_LINES"

    DEBUG = "debug"
    TWR = "twr"
    DISTANCE_PENDING = "distance_pending"
    ROUND_COMPLETE = "round_complete"
    EMPTY_ROUND = "empty_round"
    OUT_OF_SYNC_DISTANCE = "out_of_sync_distance"
    MALFORMED_DISTANCE = "malformed_distance"
    UNKNOWN = "unknown"

    distance_prefix_pattern = re.compile(r"^\s*distance\b", re.I)

    def __init__(self, parser: UwbParser, lines_per_round=5,
                 round_timeout_s=2.0,
                 round_timestamp_policy="first_distance_line"):
        if int(lines_per_round) <= 0:
            raise ValueError("distance_lines_per_round must be positive")
        if float(round_timeout_s) <= 0.0:
            raise ValueError("round_timeout_s must be positive")
        if round_timestamp_policy not in (
            "first_distance_line",
            "first_nonzero",
        ):
            raise ValueError(
                "round_timestamp_policy must be first_distance_line "
                "or first_nonzero"
            )
        self.parser = parser
        self.lines_per_round = int(lines_per_round)
        self.round_timeout_s = float(round_timeout_s)
        self.round_timestamp_policy = round_timestamp_policy

        self.complete_round_count = 0
        self.empty_round_count = 0
        self.zero_slot_count = 0
        self.incomplete_round_count = 0
        self.ignored_debug_line_count = 0
        self.duplicate_anchor_count = 0
        self.malformed_distance_count = 0

        self.state = self.WAIT_DEBUG
        self._round_started_s = 0.0
        self._distance_line_count = 0
        self._ranges: List[ParsedRange] = []
        self._seen_nonzero_anchors = set()
        self._round_timestamp_context = None
        self._round_timestamp_selected = False

    @property
    def pending_distance_line_count(self) -> int:
        return self._distance_line_count

    def _reset(self) -> None:
        self.state = self.WAIT_DEBUG
        self._round_started_s = 0.0
        self._distance_line_count = 0
        self._ranges = []
        self._seen_nonzero_anchors = set()
        self._round_timestamp_context = None
        self._round_timestamp_selected = False

    def _start_round(self, now_s: float) -> bool:
        dropped = self.state == self.WAIT_DISTANCE_LINES
        if dropped:
            self.incomplete_round_count += 1
        self.state = self.WAIT_DISTANCE_LINES
        self._round_started_s = now_s
        self._distance_line_count = 0
        self._ranges = []
        self._seen_nonzero_anchors = set()
        self._round_timestamp_context = None
        self._round_timestamp_selected = False
        return dropped

    def expire(self, now_s: Optional[float] = None) -> bool:
        now_s = time.monotonic() if now_s is None else float(now_s)
        if (
            self.state == self.WAIT_DISTANCE_LINES
            and now_s - self._round_started_s >= self.round_timeout_s
        ):
            self.incomplete_round_count += 1
            self._reset()
            return True
        return False

    def process(self, line: str, context: Any = None,
                now_s: Optional[float] = None) -> DistanceRoundEvent:
        now_s = time.monotonic() if now_s is None else float(now_s)
        expired = self.expire(now_s)

        if UwbParser.uwbdbg_pattern.match(line):
            self.ignored_debug_line_count += 1
            dropped = self._start_round(now_s)
            return DistanceRoundEvent(
                self.DEBUG, incomplete_round=expired or dropped
            )

        if UwbParser.twr_pattern.match(line):
            self.ignored_debug_line_count += 1
            return DistanceRoundEvent(self.TWR, incomplete_round=expired)

        fields = UwbParser.parse_distance_fields(line)
        if fields is None:
            if self.distance_prefix_pattern.match(line):
                self.malformed_distance_count += 1
                return DistanceRoundEvent(
                    self.MALFORMED_DISTANCE, incomplete_round=expired
                )
            return DistanceRoundEvent(self.UNKNOWN, incomplete_round=expired)

        if self.state != self.WAIT_DISTANCE_LINES:
            return DistanceRoundEvent(
                self.OUT_OF_SYNC_DISTANCE, incomplete_round=expired
            )

        anchor_id, raw_range = fields
        if (
            self.round_timestamp_policy == "first_distance_line"
            and self._distance_line_count == 0
        ):
            self._round_timestamp_context = context
            self._round_timestamp_selected = True
        elif (
            self.round_timestamp_policy == "first_nonzero"
            and raw_range != 0.0
            and not self._round_timestamp_selected
        ):
            self._round_timestamp_context = context
            self._round_timestamp_selected = True
        self._distance_line_count += 1
        if raw_range == 0.0:
            self.zero_slot_count += 1
        else:
            if anchor_id in self._seen_nonzero_anchors:
                self.duplicate_anchor_count += 1
            else:
                self._seen_nonzero_anchors.add(anchor_id)
                self._ranges.append(
                    self.parser.make_range(
                        anchor_id, raw_range, "distance_round5"
                    )
                )

        if self._distance_line_count < self.lines_per_round:
            return DistanceRoundEvent(
                self.DISTANCE_PENDING, incomplete_round=expired
            )

        ranges = self._ranges
        timestamp_context = self._round_timestamp_context
        self.complete_round_count += 1
        self._reset()
        if not ranges:
            self.empty_round_count += 1
            return DistanceRoundEvent(
                self.EMPTY_ROUND, incomplete_round=expired
            )
        return DistanceRoundEvent(
            self.ROUND_COMPLETE,
            ranges=ranges,
            timestamp_context=timestamp_context,
            incomplete_round=expired,
        )


class RangeFilter:
    STALE_REPEAT = 5

    def __init__(self, epsilon_m=0.001, max_count=3, max_duration_s=2.0):
        self.epsilon_m = epsilon_m
        self.max_count = max_count
        self.max_duration_s = max_duration_s
        self.state = {}
        self.drop_count = 0

    def filter(self, ranges: List[ParsedRange], stamp_s: float) -> List[ParsedRange]:
        output = []
        for value in ranges:
            if not value.valid:
                output.append(value)
                continue
            previous = self.state.get(value.anchor_id)
            if previous is None or abs(value.corrected_range_m - previous[0]) > self.epsilon_m:
                self.state[value.anchor_id] = (value.corrected_range_m, stamp_s, 1)
                output.append(value)
                continue
            count = previous[2] + 1
            self.state[value.anchor_id] = (previous[0], previous[1], count)
            if count > self.max_count and stamp_s - previous[1] >= self.max_duration_s:
                self.drop_count += 1
                continue
            output.append(value)
        return output
