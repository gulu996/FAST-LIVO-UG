#!/usr/bin/env python3
import math
import re
from dataclasses import dataclass
from typing import Dict, List


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


class UwbParser:
    ACCEPTED = 0
    INVALID_STATUS = 1
    INVALID_RAW_RANGE = 2
    INVALID_CORRECTED_RANGE = 3
    RANGE_LIMIT = 4

    distance_pattern = re.compile(
        r"distance\s*\[\s*([-+]?\d+)\s*\]\s*,\s*"
        r"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)", re.I
    )
    number_pattern = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")

    def __init__(self, parser_mode="auto", anchor_order=None, range_scale=1.0,
                 range_bias=None, min_range_m=0.05, max_range_m=250.0):
        self.parser_mode = parser_mode
        self.anchor_order = list(anchor_order or [])
        self.range_scale = range_scale
        self.range_bias: Dict[int, float] = dict(range_bias or {})
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m

    def _range(self, anchor_id, raw, source, diag=-1):
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

    def parse(self, line: str) -> List[ParsedRange]:
        line = line.strip()
        match = self.distance_pattern.search(line)
        if match:
            if self._error_status(line):
                return []
            return [self._range(int(match.group(1)), float(match.group(2)), "distance")]
        if self.parser_mode == "distance":
            return []

        if "[UWBDBG]" in line or "dist=" in line:
            if self.parser_mode not in ("auto", "uwb") or self._error_status(line):
                return []
            target = re.search(r"\btarget\s*=\s*([-+]?\d+)", line, re.I)
            ok = re.search(r"\bok\s*=\s*([-+]?\d+)", line, re.I)
            distance = re.search(
                r"\bdist\s*=\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)",
                line, re.I
            )
            diag = re.search(r"\bdiag\s*=\s*([-+]?\d+)", line, re.I)
            if not target or not ok or not distance or int(ok.group(1)) != 1:
                return []
            return [self._range(int(target.group(1)), float(distance.group(1)),
                                "uwbdbg" if "[UWBDBG]" in line else "debug_distance",
                                int(diag.group(1)) if diag else -1)]
        if self.parser_mode == "uwb":
            return []

        values = [float(value) for value in self.number_pattern.findall(line)]
        if not values:
            return []
        parse_pairs = self.parser_mode == "pairs"
        if self.parser_mode == "auto" and len(values) >= 2 and len(values) % 2 == 0:
            parse_pairs = all(
                value.is_integer()
                and (not self.anchor_order or int(value) in self.anchor_order)
                for value in values[0::2]
            )
        if parse_pairs:
            return [
                self._range(int(values[index]), values[index + 1], "pairs")
                for index in range(0, len(values) - 1, 2)
                if values[index].is_integer()
            ]
        return [
            self._range(
                self.anchor_order[index] if index < len(self.anchor_order) else index,
                value, "values"
            )
            for index, value in enumerate(values)
        ]

    @staticmethod
    def _error_status(line):
        lowered = line.lower()
        return "error" in lowered or "timeout" in lowered or "ok=0" in lowered


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
