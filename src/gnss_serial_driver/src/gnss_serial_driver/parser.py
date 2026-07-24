#!/usr/bin/env python3
import datetime
import json
import math
import re
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class ParsedGnss:
    source: str = "UNKNOWN"
    checksum_valid: bool = False
    utc_ns: int = 0
    utc_valid: bool = False
    latitude: float = math.nan
    longitude: float = math.nan
    altitude: float = math.nan
    height_msl: float = math.nan
    h_acc: float = math.nan
    v_acc: float = math.nan
    p_dop: float = math.nan
    vel_n: float = 0.0
    vel_e: float = 0.0
    vel_d: float = 0.0
    vel_acc: float = math.nan
    fix_type: int = 0
    valid_fix: bool = False
    diff_soln: bool = False
    carr_soln: int = 0
    num_sv: int = 0
    quality: int = 0
    position_valid: bool = False
    reject_reason: str = "parse"
    raw_line: str = ""


def _float(value: str, default: float = math.nan) -> float:
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def _int(value: str, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _unix_ns(value: datetime.datetime) -> int:
    return int(value.timestamp() * 1_000_000_000)


def _nmea_lat_lon(value: str, hemisphere: str, latitude: bool) -> float:
    raw = _float(value)
    if not math.isfinite(raw):
        return math.nan
    degrees = int(raw / 100.0)
    minutes = raw - degrees * 100.0
    result = degrees + minutes / 60.0
    if hemisphere.upper() in ("S", "W"):
        result = -result
    limit = 90.0 if latitude else 180.0
    return result if abs(result) <= limit else math.nan


def _quality_fields(quality: int):
    if quality == 4:
        return 3, True, True, 2, 4
    if quality in (2, 5):
        return 3, True, True, 1 if quality == 5 else 0, 3 if quality == 2 else 2
    if quality == 1:
        return 3, True, False, 0, 1
    return 0, False, False, 0, 0


class GnssParser:
    """Stateful NMEA/KSXT/AGRICA/legacy parser; no ROS or FAST dependency."""

    _nmea_start = re.compile(r"\$(?:KSXT|G[PN][A-Z]{3}),")

    def __init__(self):
        self.last_date: Optional[datetime.date] = None

    @staticmethod
    def checksum_valid(line: str) -> bool:
        star = line.find("*")
        if not line.startswith("$") or star < 0 or star + 2 >= len(line):
            return False
        checksum = 0
        for character in line[1:star]:
            checksum ^= ord(character)
        try:
            return checksum == int(line[star + 1:star + 3], 16)
        except ValueError:
            return False

    @staticmethod
    def _nmea_records(line: str) -> List[str]:
        starts = [match.start() for match in GnssParser._nmea_start.finditer(line)]
        if not starts:
            return [line.strip()]
        records = []
        for index, start in enumerate(starts):
            end = starts[index + 1] if index + 1 < len(starts) else len(line)
            record = line[start:end].strip()
            star = record.find("*")
            if star >= 0:
                record = record[:star + 3]
            records.append(record)
        return records

    def parse(self, line: str) -> List[ParsedGnss]:
        line = line.strip()
        if not line:
            return []
        if "@IMUGNSS:" in line:
            return self._parse_legacy(line)
        if line.startswith("#AGRICA,"):
            return [self._parse_agrica(line)]
        output = []
        for record in self._nmea_records(line):
            if record.startswith("$"):
                output.append(self._parse_nmea(record))
        return output

    def _parse_nmea(self, line: str) -> ParsedGnss:
        result = ParsedGnss(raw_line=line)
        result.checksum_valid = self.checksum_valid(line)
        body = line[1:line.find("*")] if "*" in line else line[1:]
        fields = body.split(",")
        token = fields[0].upper() if fields else ""
        source = token if token == "KSXT" else token[-3:]
        result.source = source
        if not result.checksum_valid:
            result.reject_reason = "checksum"
            return result
        try:
            if source == "KSXT":
                if len(fields) < 22:
                    raise ValueError("field_count")
                dt = datetime.datetime.strptime(fields[1], "%Y%m%d%H%M%S.%f").replace(
                    tzinfo=datetime.timezone.utc
                )
                result.utc_ns, result.utc_valid = _unix_ns(dt), True
                self.last_date = dt.date()
                result.longitude = _float(fields[2])
                result.latitude = _float(fields[3])
                result.altitude = _float(fields[4])
                quality = _int(fields[10])
                result.num_sv = _int(fields[12]) + _int(fields[13])
                result.vel_e, result.vel_n = _float(fields[17], 0.0), _float(fields[18], 0.0)
                result.vel_d = -_float(fields[19], 0.0)
                self._finish_position(result, quality)
            elif source == "GGA":
                if len(fields) < 15:
                    raise ValueError("field_count")
                result.latitude = _nmea_lat_lon(fields[2], fields[3], True)
                result.longitude = _nmea_lat_lon(fields[4], fields[5], False)
                quality = _int(fields[6])
                result.num_sv = _int(fields[7])
                result.p_dop = _float(fields[8])
                result.height_msl = _float(fields[9])
                geoid = _float(fields[11], 0.0)
                result.altitude = result.height_msl + geoid
                result.utc_ns = self._time_of_day_ns(fields[1])
                result.utc_valid = result.utc_ns > 0
                self._finish_position(result, quality)
            elif source == "RMC":
                if len(fields) < 10:
                    raise ValueError("field_count")
                day = datetime.datetime.strptime(fields[9], "%d%m%y").date()
                self.last_date = day
                result.utc_ns = self._time_of_day_ns(fields[1])
                result.utc_valid = result.utc_ns > 0
                result.valid_fix = fields[2].upper() == "A"
                result.reject_reason = "ok" if result.valid_fix else "invalid_status"
            elif source == "ZDA":
                if len(fields) < 5:
                    raise ValueError("field_count")
                self.last_date = datetime.date(_int(fields[4]), _int(fields[3]), _int(fields[2]))
                result.utc_ns = self._time_of_day_ns(fields[1])
                result.utc_valid = result.utc_ns > 0
                result.reject_reason = "ok"
            elif source == "GST":
                if len(fields) < 9:
                    raise ValueError("field_count")
                result.h_acc = max(_float(fields[6]), _float(fields[7]))
                result.v_acc = _float(fields[8])
                result.reject_reason = "ok"
            elif source == "GSA":
                result.fix_type = _int(fields[2]) if len(fields) > 2 else 0
                result.reject_reason = "ok" if result.fix_type >= 2 else "invalid_status"
            else:
                result.reject_reason = "unsupported"
        except (ValueError, IndexError):
            result.reject_reason = "field_count_or_value"
        return result

    def _time_of_day_ns(self, value: str) -> int:
        if self.last_date is None or len(value) < 6:
            return 0
        hour, minute = int(value[0:2]), int(value[2:4])
        second_float = float(value[4:])
        second = int(second_float)
        microsecond = int(round((second_float - second) * 1_000_000))
        if microsecond == 1_000_000:
            second += 1
            microsecond = 0
        dt = datetime.datetime.combine(
            self.last_date,
            datetime.time(hour, minute, min(second, 59), microsecond),
            datetime.timezone.utc,
        )
        return _unix_ns(dt)

    @staticmethod
    def _finish_position(result: ParsedGnss, quality: int):
        result.quality = quality
        (result.fix_type, result.valid_fix, result.diff_soln,
         result.carr_soln, _) = _quality_fields(quality)
        result.position_valid = (
            result.valid_fix
            and math.isfinite(result.latitude)
            and math.isfinite(result.longitude)
            and math.isfinite(result.altitude)
            and abs(result.latitude) <= 90.0
            and abs(result.longitude) <= 180.0
            and not (result.latitude == 0.0 and result.longitude == 0.0)
        )
        result.reject_reason = "ok" if result.position_valid else "invalid_position_or_quality"

    def _parse_agrica(self, line: str) -> ParsedGnss:
        result = ParsedGnss(source="AGRICA", checksum_valid=True, raw_line=line)
        try:
            semicolon, star = line.index(";"), line.rindex("*")
            data = line[semicolon + 1:star].split(",")
            if len(data) < 55 or data[0] != "GNSS":
                raise ValueError("field_count")
            result.quality = _int(data[8])
            result.num_sv = _int(data[10]) + _int(data[11]) + _int(data[12]) + _int(data[53])
            result.vel_n, result.vel_e, result.vel_d = (
                _float(data[23], 0.0), _float(data[24], 0.0), -_float(data[25], 0.0)
            )
            result.latitude, result.longitude, result.altitude = (
                _float(data[29]), _float(data[30]), _float(data[31])
            )
            result.h_acc = max(_float(data[35]), _float(data[36]))
            result.v_acc = _float(data[37])
            self._finish_position(result, result.quality)
        except (ValueError, IndexError):
            result.reject_reason = "field_count_or_value"
        return result

    def _parse_legacy(self, line: str) -> List[ParsedGnss]:
        output = []
        for match in re.finditer(r"@IMUGNSS:(\{[^{}]*\})", line):
            result = ParsedGnss(source="LEGACY_IMUGNSS_JSON", checksum_valid=True,
                                raw_line=match.group(0))
            try:
                value = json.loads(match.group(1))
                result.latitude = float(value["lat"])
                result.longitude = float(value["lon"])
                result.altitude = float(value["alt"])
                result.vel_e = float(value.get("ve", 0.0))
                result.vel_n = float(value.get("vn", 0.0))
                result.vel_d = -float(value.get("vu", 0.0))
                self._finish_position(result, int(value.get("state", 0)))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                result.reject_reason = "json"
            output.append(result)
        return output
