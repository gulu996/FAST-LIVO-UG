#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FAST-LIVO-UG 比赛提交文件转换器（Stage 6 reference-safe 版）

最常用用法：
    python3 make_competition_submission_v2.py ~/data/2026-08-25-233337 \
        --out-dir ~/data/2026-08-25-233337/submission_stage6

固定输入：
    rtk_optimized_online.tum   # 正式轨迹，已经是 map/ENU，绝不再做刚体变换
    livo_raw_online.tum        # 提前于融合轨迹开始的评分点的首选 fallback
    rtk_backend.log            # 读取后端真正的 ALIGNMENT_SUCCESS yaw + translation
    final_frozen_config.txt    # 读取显式 result_pose_lever_arm_body_m

关键原则：
1) fused TUM 若声明 frame_id=map/ENU，则直接使用；不会对 fused 再做任何 R@p+t。
2) P01 这类早于 fused 起点的评分时刻，不再用“未来 fused 重叠段重新拟合坐标系”。
   而是优先读取 rtk_backend.log 中后端实际采用的 ALIGNMENT_SUCCESS，使用同一个
   yaw + xyz translation 将 livo_raw_online.tum 映射到 map/ENU。
3) fused 保存参考点的杆臂值必须来自明确配置或冻结参数快照；禁止从轨迹重合自动拟合。
4) 默认只生成 BODY_REFERENCE 诊断 candidate。person_ground 必须显式给出
   body->person 和 PPK_BASE->official_BASE 偏移，否则 fail-safe 停止。
5) 比赛要求 NEU，因此 map/ENU TUM 的 (x,y,z) 输出为 (N,E,U)=(y,x,z)。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np


# -----------------------------------------------------------------------------
# 比赛时刻来自外部配置；正式比赛更换附件时不修改代码。
# -----------------------------------------------------------------------------

DEFAULT_SCHEDULE_PATH = Path(__file__).with_name("petrochemical_stage6b_schedule.json")


def load_schedule(path: Path):
    path = Path(path).expanduser().resolve()
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ValueError(f"{path}: invalid schedule JSON: {exc}") from exc
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError(f"{path}: expected schema_version=1")

    score_times = []
    for row in data.get("score_times", []):
        if not isinstance(row, dict):
            raise ValueError(f"{path}: score_times entries must be objects")
        score_times.append((str(row.get("id", "")), float(row["unix_utc"])))
    if not score_times:
        raise ValueError(f"{path}: score_times must contain at least one point")
    expected_ids = [f"P{i:02d}" for i in range(1, len(score_times) + 1)]
    if [item[0] for item in score_times] != expected_ids:
        raise ValueError(
            f"{path}: score_times IDs must be sequential P01..{expected_ids[-1]} in order"
        )

    dynamic_windows = []
    for row in data.get("dynamic_windows", []):
        if not isinstance(row, dict):
            raise ValueError(f"{path}: dynamic_windows entries must be objects")
        dynamic_windows.append(
            (
                str(row.get("id", "")),
                float(row["start_unix_utc"]),
                float(row["end_unix_utc"]),
            )
        )
    if [item[0] for item in dynamic_windows] != ["01", "02", "03"]:
        raise ValueError(f"{path}: dynamic window IDs must be 01,02,03 in order")

    all_times = [t for _, t in score_times]
    all_times += [t for _, start, end in dynamic_windows for t in (start, end)]
    if not np.all(np.isfinite(all_times)):
        raise ValueError(f"{path}: timestamps contain NaN/Inf")

    sanity = data.get("bds_sanity")
    if not isinstance(sanity, dict):
        raise ValueError(f"{path}: missing bds_sanity")
    bds_sanity = (
        float(sanity["unix_utc"]),
        int(sanity["week"]),
        float(sanity["sow"]),
    )
    if not np.all(np.isfinite([bds_sanity[0], bds_sanity[2]])):
        raise ValueError(f"{path}: bds_sanity contains NaN/Inf")
    return path, score_times, dynamic_windows, bds_sanity


# ponytail: globals keep the existing small single-run CLI/test API intact. If this
# becomes an in-process multi-dataset service, pass an immutable schedule object.
SCHEDULE_PATH, SCORE_TIMES, DYNAMIC_WINDOWS, BDS_SANITY = load_schedule(
    DEFAULT_SCHEDULE_PATH
)

BDS_EPOCH_UNIX = 1136073600.0  # 2006-01-01 00:00:00 UTC
BDS_UTC_LEAP_THRESHOLDS = [
    1230768000.0,  # 2009-01-01 -> BDT-UTC +1
    1341100800.0,  # 2012-07-01 -> +2
    1435708800.0,  # 2015-07-01 -> +3
    1483228800.0,  # 2017-01-01 -> +4
]


# -----------------------------------------------------------------------------
# 数据结构
# -----------------------------------------------------------------------------

@dataclass
class Trajectory:
    path: Path
    data: np.ndarray
    comments: List[str]

    @property
    def t0(self) -> float:
        return float(self.data[0, 0])

    @property
    def t1(self) -> float:
        return float(self.data[-1, 0])

    @property
    def is_map_enu(self) -> bool:
        return any("frame_id=map/ENU" in c for c in self.comments)

    @property
    def is_odom(self) -> bool:
        return any("frame_id=odom" in c for c in self.comments)

    @property
    def saved_reference_formula_declared(self) -> bool:
        return any(
            "saved_reference=body+R_body*result_pose_lever_arm_body_m" in c
            for c in self.comments
        )


@dataclass
class BackendAlignment:
    yaw_deg: float
    translation: np.ndarray
    rmse_m: Optional[float] = None
    pairs: Optional[int] = None
    source: str = "rtk_backend.log"

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)

    def rotation_matrix(self) -> np.ndarray:
        c = math.cos(self.yaw_rad)
        s = math.sin(self.yaw_rad)
        return np.array(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float
        )

    def transform_position(self, p_odom: np.ndarray) -> np.ndarray:
        return self.rotation_matrix() @ np.asarray(p_odom, dtype=float) + self.translation


@dataclass
class ReferenceConfig:
    mode: str
    saved_lever_body_m: np.ndarray
    saved_lever_source: str
    body_to_person_m: Optional[np.ndarray]
    base_offset_enu_m: Optional[np.ndarray]
    base_offset_source: str = "UNRESOLVED"
    base_rinex_source: Optional[str] = None
    ppk_base_ecef_m: Optional[np.ndarray] = None
    official_base_ecef_m: Optional[np.ndarray] = None
    official_base_lla: Optional[np.ndarray] = None


# -----------------------------------------------------------------------------
# 四元数与插值
# -----------------------------------------------------------------------------

def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError(f"invalid quaternion: {q}")
    return q / n


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)

    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return quat_normalize(q0 + alpha * (q1 - q0))

    theta0 = math.acos(dot)
    sin0 = math.sin(theta0)
    theta = theta0 * alpha
    a = math.sin(theta0 - theta) / sin0
    b = math.sin(theta) / sin0
    return quat_normalize(a * q0 + b * q1)


# -----------------------------------------------------------------------------
# 文件自动发现
# -----------------------------------------------------------------------------

def require_directory(path: Path) -> Path:
    path = path.expanduser().resolve()
    if not path.is_dir():
        raise FileNotFoundError(f"result directory not found: {path}")
    return path


def choose_existing(result_dir: Path, names: List[str], label: str, required: bool) -> Optional[Path]:
    for name in names:
        p = result_dir / name
        if p.is_file():
            return p
    if required:
        raise FileNotFoundError(
            f"cannot find {label} in {result_dir}; tried: {', '.join(names)}"
        )
    return None


def discover_files(result_dir: Path):
    # Stage 6 正式输出固定使用 Online，不允许按真值在 Online/Final 之间择优。
    fused = choose_existing(
        result_dir, ["rtk_optimized_online.tum"], "online fused trajectory", True
    )
    raw = choose_existing(
        result_dir,
        ["livo_raw_online.tum"],
        "local LIVO trajectory",
        False,
    )
    backend_log = choose_existing(
        result_dir,
        ["rtk_backend.log"],
        "RTK backend log",
        False,
    )
    gnss = choose_existing(result_dir, ["gnss_enu.tum"], "GNSS ENU trajectory", False)
    return fused, raw, backend_log, gnss


# -----------------------------------------------------------------------------
# TUM-like 轨迹读取
# -----------------------------------------------------------------------------

def load_tum_like(path: Path) -> Trajectory:
    comments: List[str] = []
    rows: List[List[float]] = []

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line_no, line in enumerate(f, 1):
            s = line.strip()
            if not s:
                continue
            if s.startswith("#"):
                comments.append(s)
                continue

            parts = s.replace(",", " ").split()
            if len(parts) < 8:
                raise ValueError(
                    f"{path}: line {line_no}: expected >=8 fields, got {len(parts)}"
                )

            try:
                vals = [float(x) for x in parts[:8]]
            except ValueError as e:
                raise ValueError(f"{path}: line {line_no}: {e}") from e

            if not np.all(np.isfinite(vals)):
                raise ValueError(f"{path}: line {line_no}: NaN/Inf found")
            rows.append(vals)

    if not rows:
        raise ValueError(f"{path}: no trajectory rows")

    arr = np.asarray(rows, dtype=float)
    order = np.argsort(arr[:, 0], kind="stable")
    arr = arr[order]

    # online.tum 允许同时间戳后续再次优化；保留最后一次。
    dedup = {}
    for row in arr:
        dedup[float(row[0])] = row
    arr = np.asarray([dedup[t] for t in sorted(dedup)], dtype=float)

    if len(arr) < 2 or np.any(np.diff(arr[:, 0]) <= 0.0):
        raise ValueError(f"{path}: invalid/non-increasing timestamps")

    for i in range(len(arr)):
        arr[i, 4:8] = quat_normalize(arr[i, 4:8])

    return Trajectory(path=path, data=arr, comments=comments)


def interpolate_pose(
    traj: Trajectory,
    t: float,
    max_gap_s: Optional[float] = None,
) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
    arr = traj.data
    ts = arr[:, 0]

    if t < ts[0] - 1e-9 or t > ts[-1] + 1e-9:
        return None

    i = int(np.searchsorted(ts, t, side="left"))
    if i < len(ts) and abs(float(ts[i]) - t) <= 1e-8:
        return arr[i, 1:4].copy(), arr[i, 4:8].copy(), 0.0

    if i == 0 or i >= len(ts):
        return None

    i0, i1 = i - 1, i
    gap = float(ts[i1] - ts[i0])
    if gap <= 0.0:
        return None
    if max_gap_s is not None and gap > max_gap_s:
        return None

    alpha = float((t - ts[i0]) / gap)
    p = (1.0 - alpha) * arr[i0, 1:4] + alpha * arr[i1, 1:4]
    q = quat_slerp(arr[i0, 4:8], arr[i1, 4:8], alpha)
    return p, q, gap


# -----------------------------------------------------------------------------
# 从 rtk_backend.log 读取“后端真正采用”的坐标对齐
# -----------------------------------------------------------------------------

ALIGNMENT_RE = re.compile(
    r"ALIGNMENT_SUCCESS\s+"
    r"yaw_deg=(?P<yaw>[-+0-9.eE]+)\s+"
    r"translation=\[\s*(?P<tx>[-+0-9.eE]+)\s+"
    r"(?P<ty>[-+0-9.eE]+)\s+"
    r"(?P<tz>[-+0-9.eE]+)\s*\]"
    r"(?:\s+rmse=(?P<rmse>[-+0-9.eE]+))?"
    r"(?:\s+pairs=(?P<pairs>\d+))?"
)


def parse_backend_alignment(path: Path) -> Optional[BackendAlignment]:
    found = None
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = ALIGNMENT_RE.search(line)
            if not m:
                continue
            found = BackendAlignment(
                yaw_deg=float(m.group("yaw")),
                translation=np.array(
                    [float(m.group("tx")), float(m.group("ty")), float(m.group("tz"))],
                    dtype=float,
                ),
                rmse_m=float(m.group("rmse")) if m.group("rmse") else None,
                pairs=int(m.group("pairs")) if m.group("pairs") else None,
                source=str(path),
            )
    return found


# -----------------------------------------------------------------------------
# 显式 reference metadata；禁止从轨迹重合拟合杆臂
# -----------------------------------------------------------------------------

FROZEN_LEVER_KEY = "/rtk_backend/result_pose_lever_arm_body_m="
FLOAT_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


def parse_frozen_result_pose_lever(path: Path) -> Optional[np.ndarray]:
    if not path.is_file():
        return None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.startswith(FROZEN_LEVER_KEY):
            continue
        values = [float(x) for x in FLOAT_RE.findall(line[len(FROZEN_LEVER_KEY):])]
        if len(values) != 3 or not np.all(np.isfinite(values)):
            raise ValueError(f"{path}: invalid frozen result-pose lever: {line}")
        return np.asarray(values, dtype=float)
    return None


def resolve_saved_reference_lever(
    fused: Trajectory,
    result_dir: Path,
    explicit_lever: Optional[List[float]],
) -> Tuple[np.ndarray, str]:
    if not fused.saved_reference_formula_declared:
        raise RuntimeError(
            f"{fused.path.name}: missing saved-reference metadata; refusing to guess "
            "whether positions are body origin or body+R*lever"
        )
    if explicit_lever is not None:
        lever = np.asarray(explicit_lever, dtype=float)
        return lever, "explicit_cli"
    snapshot = result_dir / "final_frozen_config.txt"
    lever = parse_frozen_result_pose_lever(snapshot)
    if lever is None:
        raise RuntimeError(
            "saved-reference lever value is unresolved. Provide "
            "--saved-reference-lever-body-m E N U or a final_frozen_config.txt; "
            "trajectory-overlap inference is forbidden"
        )
    return lever, str(snapshot)


def resolve_base_reference(args, parser):
    provided = [
        args.base_offset_m is not None,
        args.official_base_ecef_m is not None,
        args.official_base_lla is not None,
    ]
    if args.reference_mode == "body":
        if any(provided) or args.base_rinex is not None or args.base_offset_source is not None:
            parser.error("BASE inputs are only applied in --reference-mode person_ground")
        return None, "UNRESOLVED", None, None, None, None

    if args.body_to_person_m is None:
        parser.error("person_ground requires explicit --body-to-person-m X Y Z")
    if not np.all(np.isfinite(args.body_to_person_m)):
        parser.error("--body-to-person-m contains NaN/Inf")
    if sum(provided) != 1:
        parser.error(
            "person_ground requires exactly one of --base-offset-m, "
            "--official-base-ecef-m, or --official-base-lla"
        )
    if args.base_rinex is None:
        parser.error("person_ground requires --base-rinex to read the PPK BASE ECEF metadata")

    rinex_path = Path(args.base_rinex).expanduser().resolve()
    ppk_ecef = read_rinex_approx_position_xyz(rinex_path)
    official_ecef = None
    official_lla = None

    if args.base_offset_m is not None:
        if args.base_offset_source is None:
            parser.error("--base-offset-m requires explicit --base-offset-source provenance")
        d_base = np.asarray(args.base_offset_m, dtype=float)
        if not np.all(np.isfinite(d_base)):
            parser.error("--base-offset-m contains NaN/Inf")
        source = args.base_offset_source
        if source == "OFFICIAL_CONFIRMATION_SAME_AS_RINEX_BASE":
            if np.linalg.norm(d_base) > 1e-12:
                parser.error(
                    "OFFICIAL_CONFIRMATION_SAME_AS_RINEX_BASE requires "
                    "--base-offset-m 0 0 0"
                )
            official_ecef = ppk_ecef.copy()
            official_lla = ecef_to_lla(official_ecef)
    else:
        if args.base_offset_source is not None:
            parser.error(
                "official BASE ECEF/LLA automatically records "
                "base_offset_source=OFFICIAL_COORDINATES"
            )
        source = "OFFICIAL_COORDINATES"
        if args.official_base_ecef_m is not None:
            official_ecef = np.asarray(args.official_base_ecef_m, dtype=float)
            d_base, official_lla = base_offset_from_official_ecef(
                ppk_ecef, official_ecef
            )
        else:
            official_lla = np.asarray(args.official_base_lla, dtype=float)
            official_ecef = lla_to_ecef(official_lla)
            d_base, computed_lla = base_offset_from_official_ecef(
                ppk_ecef, official_ecef
            )
            if not np.allclose(computed_lla, official_lla, atol=1e-8, rtol=0.0):
                raise RuntimeError("official BASE LLA/ECEF WGS84 round-trip failed")

    return d_base, source, str(rinex_path), ppk_ecef, official_ecef, official_lla


# -----------------------------------------------------------------------------
# 组合轨迹：fused 直接；提前/缺失时刻才 fallback
# -----------------------------------------------------------------------------

class CompetitionTrajectory:
    def __init__(
        self,
        fused: Trajectory,
        max_fused_gap_s: float,
        raw: Optional[Trajectory],
        alignment: Optional[BackendAlignment],
        reference: ReferenceConfig,
    ):
        self.fused = fused
        self.max_fused_gap_s = max_fused_gap_s
        self.raw = raw
        self.alignment = alignment
        self.reference = reference

    def _output_reference(self, p_body: np.ndarray, r_map_body: np.ndarray) -> np.ndarray:
        if self.reference.mode == "body":
            return p_body
        # t_body_person 在 body 系表达，必须乘 R_ENU_body；
        # d_base 已是 official-BASE ENU 中的固定平移，绝不再乘 body rotation。
        return (
            p_body
            + r_map_body @ self.reference.body_to_person_m
            + self.reference.base_offset_enu_m
        )

    def pose(self, t: float, force_aligned_raw: bool = False):
        # 已是 map/ENU：绝不再做 map<-odom alignment。只用显式杆臂还原 body。
        direct = None if force_aligned_raw else interpolate_pose(
            self.fused, t, max_gap_s=self.max_fused_gap_s
        )
        if direct is not None:
            p_saved, q, gap = direct
            r_map_body = quat_to_rot(q)
            p_body = p_saved - r_map_body @ self.reference.saved_lever_body_m
            p_out = self._output_reference(p_body, r_map_body)
            return p_out, q, "ONLINE_INTERPOLATED", gap

        # Stage 6 仅 P01 显式允许 aligned-raw fallback。
        if not force_aligned_raw:
            return None
        if self.raw is None or self.alignment is None:
            return None

        raw_item = interpolate_pose(self.raw, t, max_gap_s=0.5)
        if raw_item is None:
            return None

        p_raw, q_raw, gap = raw_item
        rz = self.alignment.rotation_matrix()
        p_body = self.alignment.transform_position(p_raw)
        r_map_body = rz @ quat_to_rot(q_raw)
        p_out = self._output_reference(p_body, r_map_body)
        return p_out, q_raw, "ALIGNED_RAW_FALLBACK", gap


# -----------------------------------------------------------------------------
# 坐标与 BDS 时间
# -----------------------------------------------------------------------------

def enu_to_neu(p_enu: np.ndarray) -> np.ndarray:
    e, n, u = np.asarray(p_enu, dtype=float)
    return np.array([n, e, u], dtype=float)


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def lla_to_ecef(lla: np.ndarray) -> np.ndarray:
    lat_deg, lon_deg, height_m = np.asarray(lla, dtype=float)
    if not np.all(np.isfinite([lat_deg, lon_deg, height_m])):
        raise ValueError("official BASE LLA contains NaN/Inf")
    if not -90.0 <= lat_deg <= 90.0 or not -180.0 <= lon_deg <= 180.0:
        raise ValueError(f"invalid latitude/longitude: {lat_deg}, {lon_deg}")
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array(
        [
            (n + height_m) * cos_lat * math.cos(lon),
            (n + height_m) * cos_lat * math.sin(lon),
            (n * (1.0 - WGS84_E2) + height_m) * sin_lat,
        ],
        dtype=float,
    )


def ecef_to_lla(ecef_m: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(ecef_m, dtype=float)
    if not np.all(np.isfinite([x, y, z])):
        raise ValueError("official BASE ECEF contains NaN/Inf")
    if np.linalg.norm([x, y, z]) < 1.0e6:
        raise ValueError("official BASE ECEF is not an Earth-surface coordinate")
    p = math.hypot(x, y)
    if p < 1e-9:
        lat = math.copysign(math.pi / 2.0, z)
        lon = 0.0
        b = WGS84_A_M * (1.0 - WGS84_F)
        return np.array([math.degrees(lat), 0.0, abs(z) - b], dtype=float)

    lon = math.atan2(y, x)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    height = 0.0
    for _ in range(12):
        sin_lat = math.sin(lat)
        n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / math.cos(lat) - n
        next_lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + height)))
        if abs(next_lat - lat) < 1e-14:
            lat = next_lat
            break
        lat = next_lat
    sin_lat = math.sin(lat)
    n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    height = p / math.cos(lat) - n
    return np.array([math.degrees(lat), math.degrees(lon), height], dtype=float)


def ecef_to_enu(
    point_ecef_m: np.ndarray,
    origin_ecef_m: np.ndarray,
    origin_lla: np.ndarray,
) -> np.ndarray:
    lat, lon = map(math.radians, np.asarray(origin_lla, dtype=float)[:2])
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    rotation = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=float,
    )
    return rotation @ (
        np.asarray(point_ecef_m, dtype=float) - np.asarray(origin_ecef_m, dtype=float)
    )


def enu_to_ecef(
    point_enu_m: np.ndarray,
    origin_ecef_m: np.ndarray,
    origin_lla: np.ndarray,
) -> np.ndarray:
    lat, lon = map(math.radians, np.asarray(origin_lla, dtype=float)[:2])
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    rotation = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=float,
    )
    return np.asarray(origin_ecef_m, dtype=float) + rotation.T @ np.asarray(
        point_enu_m, dtype=float
    )


def base_offset_from_official_ecef(
    ppk_base_ecef_m: np.ndarray,
    official_base_ecef_m: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return E,N,U coordinates of PPK BASE relative to official BASE.

    This direction is fixed: p_official_ENU = p_ppk_ENU + d_base_ENU.
    """
    official_lla = ecef_to_lla(official_base_ecef_m)
    d_base_enu = ecef_to_enu(
        ppk_base_ecef_m, official_base_ecef_m, official_lla
    )
    return d_base_enu, official_lla


def read_rinex_approx_position_xyz(path: Path) -> np.ndarray:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"BASE RINEX not found: {path}")
    found = []
    with path.open("r", encoding="ascii", errors="replace") as f:
        for line_no, line in enumerate(f, 1):
            if "APPROX POSITION XYZ" not in line:
                if "END OF HEADER" in line:
                    break
                continue
            values = FLOAT_RE.findall(line[:60])
            if len(values) < 3:
                raise ValueError(f"{path}:{line_no}: invalid APPROX POSITION XYZ")
            found.append(np.asarray([float(x) for x in values[:3]], dtype=float))
    if len(found) != 1:
        raise ValueError(f"{path}: expected exactly one APPROX POSITION XYZ, got {len(found)}")
    return found[0]


def bdt_minus_utc(unix_s: float) -> int:
    offset = 0
    for threshold in BDS_UTC_LEAP_THRESHOLDS:
        if unix_s >= threshold:
            offset += 1
    return offset


def unix_to_bds_week_sow(unix_s: float) -> Tuple[int, float]:
    bdt_seconds = unix_s - BDS_EPOCH_UNIX + bdt_minus_utc(unix_s)
    if bdt_seconds < 0:
        raise ValueError("timestamp predates BDS epoch")
    week = int(math.floor(bdt_seconds / 604800.0))
    sow = bdt_seconds - week * 604800.0
    return week, sow


def bds_week_sow_to_unix(week: int, sow: float) -> float:
    if week < 0 or not math.isfinite(sow) or not 0.0 <= sow < 604800.0:
        raise ValueError(f"invalid BDS week/SOW: {week}, {sow}")
    bdt_seconds = week * 604800.0 + sow
    # BDT-UTC 取决于 UTC epoch；用有界迭代兼容历史闰秒阶跃。
    unix_s = BDS_EPOCH_UNIX + bdt_seconds
    for _ in range(3):
        unix_s = BDS_EPOCH_UNIX + bdt_seconds - bdt_minus_utc(unix_s)
    return unix_s


def times_10hz_inclusive(start_s: float, end_s: float) -> np.ndarray:
    # 用整数 0.1 s tick，避免浮点累积误差。
    start_tick = int(round(start_s * 10.0))
    end_tick = int(round(end_s * 10.0))
    return np.arange(start_tick, end_tick + 1, dtype=np.int64) / 10.0


# -----------------------------------------------------------------------------
# 文件生成
# -----------------------------------------------------------------------------

def write_position_points(path: Path, traj: CompetitionTrajectory):
    rows = []
    with path.open("w", encoding="utf-8", newline="\n") as f:
        for pid, t in SCORE_TIMES:
            item = traj.pose(t, force_aligned_raw=(pid == "P01"))
            if item is None:
                raise RuntimeError(
                    f"{pid} @ {t:.1f}: no pose available. "
                    "Need fused coverage or raw+rtk_backend.log fallback."
                )
            p_enu, _, source, gap = item
            n, e, u = enu_to_neu(p_enu)
            f.write(f"{pid},{n:.4f},{e:.4f},{u:.4f}\n")
            rows.append((pid, t, n, e, u, source, gap))
    return rows


def write_dynamic(path: Path, traj: CompetitionTrajectory, start_s: float, end_s: float):
    rows = []
    source_counts = {}
    with path.open("w", encoding="utf-8", newline="\n") as f:
        for t in times_10hz_inclusive(start_s, end_s):
            item = traj.pose(float(t))
            if item is None:
                raise RuntimeError(f"dynamic epoch {t:.1f}: no pose available")
            p_enu, _, source, gap = item
            n, e, u = enu_to_neu(p_enu)
            week, sow = unix_to_bds_week_sow(float(t))
            sow = round(sow, 1)
            f.write(f"{week},{sow:.1f},{n:.4f},{e:.4f},{u:.4f}\n")
            rows.append((week, sow, n, e, u, source, gap))
            source_counts[source] = source_counts.get(source, 0) + 1
    return rows, source_counts


POINT_LINE_RE = re.compile(
    r"P\d{2,},-?\d+\.\d{4},-?\d+\.\d{4},-?\d+\.\d{4}"
)
DYNAMIC_LINE_RE = re.compile(
    r"\d+,-?\d+\.\d,-?\d+\.\d{4},-?\d+\.\d{4},-?\d+\.\d{4}"
)


def read_strict_ascii_lines(path: Path) -> List[str]:
    raw = path.read_bytes()
    if raw.startswith(b"\xef\xbb\xbf"):
        raise RuntimeError(f"{path.name}: UTF-8 BOM is forbidden")
    if b"\r" in raw:
        raise RuntimeError(f"{path.name}: CR/CRLF is forbidden")
    try:
        text = raw.decode("ascii")
    except UnicodeDecodeError as exc:
        raise RuntimeError(f"{path.name}: output must be ASCII") from exc
    lines = text.splitlines()
    if any(not line for line in lines):
        raise RuntimeError(f"{path.name}: blank line is forbidden")
    if any(line != line.rstrip(" \t") for line in lines):
        raise RuntimeError(f"{path.name}: trailing spaces are forbidden")
    return lines


def validate_output_files(out_dir: Path) -> List[str]:
    checks = []
    point_lines = read_strict_ascii_lines(out_dir / "position_points.txt")
    expected_ids = [pid for pid, _ in SCORE_TIMES]
    if len(point_lines) != len(expected_ids):
        raise RuntimeError(
            f"position_points.txt: expected {len(expected_ids)} lines from schedule, "
            f"got {len(point_lines)}"
        )
    actual_ids = []
    for i, line in enumerate(point_lines, 1):
        if not POINT_LINE_RE.fullmatch(line):
            raise RuntimeError(f"position_points.txt:{i}: invalid format: {line!r}")
        actual_ids.append(line.split(",", 1)[0])
    if actual_ids != expected_ids:
        raise RuntimeError(f"position_points.txt: IDs/order mismatch: {actual_ids}")
    checks.append(
        f"position_points: {len(expected_ids)} lines, "
        f"{expected_ids[0]}..{expected_ids[-1]} once, ASCII comma, NEU 4 decimals"
    )

    for name, start_s, end_s in DYNAMIC_WINDOWS:
        path = out_dir / f"position_dynamic_{name}.txt"
        lines = read_strict_ascii_lines(path)
        if len(lines) != 201:
            raise RuntimeError(f"{path.name}: expected 201 lines, got {len(lines)}")
        epochs = []
        for i, line in enumerate(lines, 1):
            if not DYNAMIC_LINE_RE.fullmatch(line):
                raise RuntimeError(f"{path.name}:{i}: invalid format: {line!r}")
            p = line.split(",")
            week, sow = int(p[0]), float(p[1])
            values = np.asarray([float(x) for x in p[2:5]])
            if not np.all(np.isfinite(values)):
                raise RuntimeError(f"{path.name}:{i}: NaN/Inf")
            epochs.append(week * 604800.0 + sow)
        diffs = np.diff(np.asarray(epochs))
        if not np.allclose(diffs, 0.1, atol=1e-7, rtol=0.0):
            raise RuntimeError(f"{path.name}: timestamps are not unique monotonic 10 Hz")
        if abs(bds_week_sow_to_unix(int(lines[0].split(',')[0]), float(lines[0].split(',')[1])) - start_s) > 1e-6:
            raise RuntimeError(f"{path.name}: first epoch does not match {start_s:.1f}")
        if abs(bds_week_sow_to_unix(int(lines[-1].split(',')[0]), float(lines[-1].split(',')[1])) - end_s) > 1e-6:
            raise RuntimeError(f"{path.name}: last epoch does not match {end_s:.1f}")
        checks.append(
            f"dynamic_{name}: 201 lines, inclusive 10 Hz, BDS SOW 1 decimal, NEU 4 decimals"
        )
    sanity_unix, sanity_week, sanity_sow = BDS_SANITY
    actual_week, actual_sow = unix_to_bds_week_sow(sanity_unix)
    if actual_week != sanity_week or abs(actual_sow - sanity_sow) > 1e-6:
        raise RuntimeError(
            "configured BDS sanity failed: "
            f"got {actual_week},{actual_sow:.1f}, expected {sanity_week},{sanity_sow:.1f}"
        )
    checks.append(
        f"configured BDS sanity: {sanity_week},{sanity_sow:.1f}"
    )
    return checks


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_submission_hashes(out_dir: Path) -> Path:
    files = [out_dir / "position_points.txt"] + [
        out_dir / f"position_dynamic_{name}.txt" for name, _, _ in DYNAMIC_WINDOWS
    ]
    path = out_dir / "submission_stage6b_sha256.txt"
    path.write_text(
        "".join(f"{sha256_file(item)}  {item.name}\n" for item in files),
        encoding="ascii",
    )
    return path


def git_head() -> str:
    repo = Path(__file__).resolve().parents[2]
    proc = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return proc.stdout.strip() if proc.returncode == 0 else "UNKNOWN"


def vector_text(value: Optional[np.ndarray]) -> str:
    if value is None:
        return "UNRESOLVED"
    return "[" + ",".join(f"{x:+.9f}" for x in value) + "]"


def write_manifest(
    path: Path,
    result_dir: Path,
    fused_path: Path,
    raw_path: Path,
    alignment: BackendAlignment,
    reference: ReferenceConfig,
    point_rows,
    schedule_path: Path,
):
    p01_source = next(row[5] for row in point_rows if row[0] == "P01")
    formal_status = (
        "READY_PERSON_GROUND"
        if reference.mode == "person_ground"
        else "BLOCKED_BASE_REFERENCE_OFFSET_UNRESOLVED"
    )
    lines = [
        "SUBMISSION_STAGE6_MANIFEST",
        f"generated_utc={datetime.now(timezone.utc).isoformat()}",
        f"baseline_run={result_dir}",
        f"source_run={result_dir}",
        "source_trajectory=ONLINE",
        f"source_mode=ONLINE_ONLY",
        f"source_trajectory_path={fused_path}",
        f"source_trajectory_sha256={sha256_file(fused_path)}",
        f"raw_fallback={raw_path}",
        f"raw_fallback_sha256={sha256_file(raw_path)}",
        f"p01_source={p01_source}",
        f"reference_mode={reference.mode.upper()}",
        "base_origin_definition=RINEX_HEADER_APPROX_POSITION_XYZ"
        if reference.base_offset_source == "OFFICIAL_CONFIRMATION_SAME_AS_RINEX_BASE"
        else "base_origin_definition=OFFICIAL_COORDINATES",
        f"saved_reference_lever_body_m={vector_text(reference.saved_lever_body_m)}",
        f"saved_reference_lever_source={reference.saved_lever_source}",
        f"body_to_person_m={vector_text(reference.body_to_person_m)}",
        f"base_offset_enu_m={vector_text(reference.base_offset_enu_m)}",
        f"d_base_ppk_to_official_enu_m={vector_text(reference.base_offset_enu_m)}",
        "d_base_definition=coordinates_of_PPK_BASE_origin_relative_to_official_BASE_origin_ENU",
        "d_base_order=E,N,U",
        "position_formula=p_official_ENU=p_ppk_ENU+d_base_ENU",
        f"base_offset_source={reference.base_offset_source}",
        f"base_rinex_source={reference.base_rinex_source or 'UNRESOLVED'}",
        f"ppk_base_ecef_m={vector_text(reference.ppk_base_ecef_m)}",
        f"official_base_ecef_m={vector_text(reference.official_base_ecef_m)}",
        f"official_base_lla_deg_deg_m={vector_text(reference.official_base_lla)}",
        f"formal_person_ground_status={formal_status}",
        f"alignment_source={alignment.source}",
        f"alignment_yaw_deg={alignment.yaw_deg:.9f}",
        f"alignment_translation_enu_m={vector_text(alignment.translation)}",
        f"alignment_pairs={alignment.pairs}",
        f"alignment_rmse_m={alignment.rmse_m}",
        f"git_head={git_head()}",
        f"script={Path(__file__).resolve()}",
        f"script_sha256={sha256_file(Path(__file__).resolve())}",
        f"schedule_file={schedule_path}",
        f"schedule_sha256={sha256_file(schedule_path)}",
        "static_point_method=SINGLE_TIMESTAMP",
        "dynamic_method=ONLINE_10HZ_INTERPOLATION",
        "coordinate_internal=ENU",
        "coordinate_output=NEU",
        "truth_fitting=false",
        "online_final_pointwise_selection=false",
        "enu_to_neu=single_swap_N_y_E_x_U_z",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_validation_report(
    path: Path,
    reference: ReferenceConfig,
    checks: List[str],
    point_rows,
    dynamic_summaries,
    reference_validation: List[str],
):
    formal_blocked = reference.mode == "body"
    lines = [
        "# Stage 6 submission validation",
        "",
        f"- Candidate reference: `{reference.mode.upper()}`",
        "- Source trajectory: `rtk_optimized_online.tum` only",
        "- P01 source: `ALIGNED_RAW_FALLBACK`",
        "- Truth fitting: `false`",
        "- Online/Final point-wise selection: `false`",
        "- `d_base_ENU`: PPK BASE origin relative to official BASE origin, ordered E,N,U",
        "- Position formula: `p_official_ENU = p_ppk_ENU + d_base_ENU`",
        f"- BASE offset source: `{reference.base_offset_source}`",
        "- BODY REFERENCE ONLY; NOT OFFICIAL PERSON-GROUND CLOSED"
        if formal_blocked
        else "- PERSON_GROUND reference inputs were explicitly supplied",
        "- Formal person-ground status: `BLOCKED_BASE_REFERENCE_OFFSET_UNRESOLVED`"
        if formal_blocked
        else "- Formal person-ground status: `READY_PERSON_GROUND`",
        "",
        "## Strict format checks",
        "",
        *[f"- PASS: {item}" for item in checks],
        "",
        "## Point sources",
        "",
        *[f"- {pid} @ {t:.1f}: `{source}` (bracket gap {gap:.9f} s)" for pid, t, _, _, _, source, gap in point_rows],
        "",
        "## Dynamic sources",
        "",
        *[f"- dynamic_{name}: {count} rows, {counts}" for name, count, counts in dynamic_summaries],
    ]
    if reference_validation:
        lines += [
            "",
            "## Post-generation reference validation only",
            "",
            "Official references were read only after output generation and did not affect any transform or sample.",
            "",
            "```text",
            *reference_validation,
            "```",
        ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# -----------------------------------------------------------------------------
# 可选参考结果校验（只在输出后比较；绝不参与变换）
# -----------------------------------------------------------------------------

def read_reference_points(path: Path):
    out = {}
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) == 4:
                out[p[0]] = np.asarray([float(p[1]), float(p[2]), float(p[3])])
    return out


def read_dynamic(path: Path):
    out = {}
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) != 5:
                continue
            key = (int(p[0]), int(round(float(p[1]) * 10.0)))
            out[key] = np.asarray([float(p[2]), float(p[3]), float(p[4])])
    return out


def summarize_errors(errors: List[np.ndarray]):
    if not errors:
        return None
    E = np.asarray(errors, dtype=float)
    h = np.linalg.norm(E[:, :2], axis=1)
    u = np.abs(E[:, 2])
    d3 = np.linalg.norm(E, axis=1)
    return {
        "count": len(E),
        "h_rmse": float(np.sqrt(np.mean(h ** 2))),
        "u_rmse": float(np.sqrt(np.mean(u ** 2))),
        "d3_rmse": float(np.sqrt(np.mean(d3 ** 2))),
        "d3_max": float(np.max(d3)),
    }


def evaluate_reference(out_dir: Path, ref_dir: Path):
    lines = []

    gp = out_dir / "position_points.txt"
    rp = ref_dir / "position_points.txt"
    if gp.exists() and rp.exists():
        gen = read_reference_points(gp)
        ref = read_reference_points(rp)
        errors = []
        lines.append("[points]")
        for pid, _ in SCORE_TIMES:
            if pid not in gen or pid not in ref:
                continue
            e = gen[pid] - ref[pid]
            errors.append(e)
            lines.append(
                f"  {pid}: dN={e[0]:+.4f} dE={e[1]:+.4f} "
                f"dU={e[2]:+.4f} 3D={np.linalg.norm(e):.4f} m"
            )
        s = summarize_errors(errors)
        if s:
            lines.append(
                f"  RMSE: H={s['h_rmse']:.4f} m, U={s['u_rmse']:.4f} m, "
                f"3D={s['d3_rmse']:.4f} m, MAX3D={s['d3_max']:.4f} m"
            )

    for name, _, _ in DYNAMIC_WINDOWS:
        gp = out_dir / f"position_dynamic_{name}.txt"
        rp = ref_dir / f"position_dynamic_{name}.txt"
        if not gp.exists() or not rp.exists():
            continue
        gen = read_dynamic(gp)
        ref = read_dynamic(rp)
        common = sorted(set(gen) & set(ref))
        s = summarize_errors([gen[k] - ref[k] for k in common])
        if s:
            lines.append(
                f"[dynamic {name}] matched={s['count']}, "
                f"H_RMSE={s['h_rmse']:.4f} m, U_RMSE={s['u_rmse']:.4f} m, "
                f"3D_RMSE={s['d3_rmse']:.4f} m, MAX3D={s['d3_max']:.4f} m"
            )
    return lines


# -----------------------------------------------------------------------------
# 主程序
# -----------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Convert one FAST-LIVO-UG run directory into competition submission files."
    )
    ap.add_argument(
        "result_dir",
        nargs="?",
        help="one run directory, e.g. ~/data/2026-08-23-202045",
    )
    ap.add_argument(
        "--schedule-file",
        default=str(DEFAULT_SCHEDULE_PATH),
        help="JSON score timestamps/dynamic windows (default: bundled petrochemical schedule)",
    )
    ap.add_argument(
        "--validate-only",
        metavar="SUBMISSION_DIR",
        default=None,
        help="strictly validate existing output files without reading a run trajectory",
    )
    ap.add_argument(
        "--out-dir",
        default=None,
        help="default: <result_dir>/submission_generated",
    )
    ap.add_argument(
        "--max-fused-gap",
        type=float,
        default=2.0,
        help="maximum interpolation bracket inside fused trajectory (default 2.0 s)",
    )
    ap.add_argument(
        "--reference-mode",
        choices=("body", "person_ground"),
        default="body",
        help="output reference point; safe default is body",
    )
    ap.add_argument(
        "--saved-reference-lever-body-m",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="explicit value used by fused saved_reference=body+R*lever metadata",
    )
    ap.add_argument(
        "--body-to-person-m",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="explicit body-frame body->person-ground vector",
    )
    base_group = ap.add_mutually_exclusive_group()
    base_group.add_argument(
        "--base-offset-m",
        type=float,
        nargs=3,
        metavar=("E", "N", "U"),
        default=None,
        help=(
            "d_base_ENU in E,N,U: coordinates of the PPK BASE origin expressed "
            "relative to the official BASE origin; p_official=p_ppk+d_base"
        ),
    )
    base_group.add_argument(
        "--official-base-ecef-m",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="official BASE WGS84 ECEF metres; d_base_ENU is computed from BASE RINEX",
    )
    base_group.add_argument(
        "--official-base-lla",
        type=float,
        nargs=3,
        metavar=("LAT", "LON", "H"),
        default=None,
        help="official BASE WGS84 latitude deg, longitude deg, ellipsoidal height m",
    )
    ap.add_argument(
        "--base-rinex",
        default=None,
        help="BASE RINEX whose APPROX POSITION XYZ defines the PPK BASE origin",
    )
    ap.add_argument(
        "--base-offset-source",
        choices=(
            "OFFICIAL_CONFIRMATION_SAME_AS_RINEX_BASE",
            "OFFICIAL_ENU_OFFSET",
            "OFFICIAL_NEU_OFFSET_CONVERTED_TO_ENU",
        ),
        default=None,
        help="required provenance for direct --base-offset-m; the numeric order remains E,N,U",
    )
    ap.add_argument(
        "--reference-dir",
        default=None,
        help="optional official reference directory; validation only",
    )
    args = ap.parse_args()

    global SCHEDULE_PATH, SCORE_TIMES, DYNAMIC_WINDOWS, BDS_SANITY
    SCHEDULE_PATH, SCORE_TIMES, DYNAMIC_WINDOWS, BDS_SANITY = load_schedule(
        Path(args.schedule_file)
    )
    if args.validate_only is not None:
        checks = validate_output_files(Path(args.validate_only).expanduser().resolve())
        for check in checks:
            print(f"PASS: {check}")
        print("PASS: strict submission format validation")
        return 0
    if args.result_dir is None:
        ap.error("result_dir is required unless --validate-only is used")

    (
        base_offset_enu,
        base_offset_source,
        base_rinex_source,
        ppk_base_ecef,
        official_base_ecef,
        official_base_lla,
    ) = resolve_base_reference(args, ap)

    result_dir = require_directory(Path(args.result_dir))
    out_dir = (
        Path(args.out_dir).expanduser().resolve()
        if args.out_dir
        else result_dir / "submission_generated"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    fused_path, raw_path, backend_log_path, gnss_path = discover_files(result_dir)

    print("=" * 80)
    print("FAST-LIVO-UG competition converter — run-directory mode")
    print("=" * 80)
    print(f"Result directory : {result_dir}")
    print(f"Fused trajectory : {fused_path.name}")
    print(f"Raw fallback     : {raw_path.name if raw_path else 'NOT FOUND'}")
    print(f"Backend log      : {backend_log_path.name if backend_log_path else 'NOT FOUND'}")
    print(f"GNSS ENU         : {gnss_path.name if gnss_path else 'NOT FOUND'}")
    print(f"Output directory : {out_dir}")

    fused = load_tum_like(fused_path)
    raw = load_tum_like(raw_path) if raw_path else None

    print()
    print(
        f"Fused poses      : {len(fused.data)} | "
        f"{fused.t0:.6f} -> {fused.t1:.6f}"
    )
    if fused.comments:
        for c in fused.comments:
            print(f"  {c}")

    if not fused.is_map_enu:
        raise RuntimeError(
            f"{fused_path.name} does not declare frame_id=map/ENU; "
            "refusing to guess and risk a second coordinate transform."
        )

    print("[OK] fused TUM is map/ENU; no map<-odom transform is applied to Online.")

    saved_lever, saved_lever_source = resolve_saved_reference_lever(
        fused, result_dir, args.saved_reference_lever_body_m
    )
    reference = ReferenceConfig(
        mode=args.reference_mode,
        saved_lever_body_m=saved_lever,
        saved_lever_source=saved_lever_source,
        body_to_person_m=(
            np.asarray(args.body_to_person_m, dtype=float)
            if args.body_to_person_m is not None
            else None
        ),
        base_offset_enu_m=base_offset_enu,
        base_offset_source=base_offset_source,
        base_rinex_source=base_rinex_source,
        ppk_base_ecef_m=ppk_base_ecef,
        official_base_ecef_m=official_base_ecef,
        official_base_lla=official_base_lla,
    )
    print(
        "[OK] explicit saved-reference lever: "
        f"{vector_text(reference.saved_lever_body_m)} from {saved_lever_source}"
    )
    print("[OK] trajectory-overlap lever inference is not implemented.")

    # P01 之后的配置点与 dynamic 必须完全由 Online 插值，禁止走 raw/Final。
    required_online = [t for pid, t in SCORE_TIMES if pid != "P01"]
    for _, start, end in DYNAMIC_WINDOWS:
        required_online.extend(times_10hz_inclusive(start, end))
    missing_online = sorted(
        {
            float(t)
            for t in required_online
            if interpolate_pose(fused, float(t), max_gap_s=args.max_fused_gap) is None
        }
    )
    if missing_online:
        raise RuntimeError(
            "Online trajectory cannot cover required configured scoring-point/dynamic epochs; "
            "raw/Final substitution is forbidden. First missing epochs: "
            + ", ".join(f"{t:.1f}" for t in missing_online[:10])
        )

    if raw is None or not raw.is_odom:
        raise RuntimeError("P01 requires livo_raw_online.tum with frame_id=odom metadata")
    if backend_log_path is None:
        raise RuntimeError("P01 requires rtk_backend.log ALIGNMENT_SUCCESS metadata")
    alignment = parse_backend_alignment(backend_log_path)
    if alignment is None:
        raise RuntimeError(
            f"No ALIGNMENT_SUCCESS found in {backend_log_path}; refusing to fit another alignment"
        )
    print(
        f"[OK] P01 alignment from backend log: yaw={alignment.yaw_deg:+.6f} deg, "
        f"t={vector_text(alignment.translation)}, rmse={alignment.rmse_m}, "
        f"pairs={alignment.pairs}"
    )
    print("[OK] P01 source is forced to ALIGNED_RAW_FALLBACK; no truth fitting.")

    traj = CompetitionTrajectory(
        fused=fused,
        max_fused_gap_s=args.max_fused_gap,
        raw=raw,
        alignment=alignment,
        reference=reference,
    )

    point_rows = write_position_points(out_dir / "position_points.txt", traj)

    dynamic_summaries = []
    for name, start, end in DYNAMIC_WINDOWS:
        rows, counts = write_dynamic(
            out_dir / f"position_dynamic_{name}.txt",
            traj,
            start,
            end,
        )

        dynamic_summaries.append((name, len(rows), counts))

    # 对配置附件中的 UTC/BDT 对照做 sanity check。
    sanity_unix, sanity_week, sanity_sow = BDS_SANITY
    week, sow = unix_to_bds_week_sow(sanity_unix)
    if week != sanity_week or abs(sow - sanity_sow) > 1e-6:
        raise RuntimeError(
            "BDS conversion sanity failed: "
            f"week={week}, sow={sow:.6f}, expected={sanity_week},{sanity_sow:.6f}"
        )
    if abs(bds_week_sow_to_unix(week, sow) - sanity_unix) > 1e-6:
        raise RuntimeError("BDS inverse conversion sanity failed")

    print()
    print("[scoring points]")
    for pid, t, n, e, u, source, _ in point_rows:
        print(
            f"  {pid} @ {t:.1f}: {source:30s} "
            f"N={n:.4f} E={e:.4f} U={u:.4f}"
        )

    print()
    print("[dynamic files]")
    for name, count, counts in dynamic_summaries:
        print(f"  {name}: rows={count}, sources={counts}")
    checks = validate_output_files(out_dir)
    print()
    print("[strict validation]")
    for check in checks:
        print(f"  PASS: {check}")

    submission_hash_path = write_submission_hashes(out_dir)
    print(f"  PASS: formal outputs hashed before optional reference validation")

    validation = []
    if args.reference_dir:
        ref_dir = Path(args.reference_dir).expanduser().resolve()
        validation = evaluate_reference(out_dir, ref_dir)
        if validation:
            print()
            print("[reference validation only — not used for generation/alignment]")
            for line in validation:
                print(line)

    manifest_path = out_dir / "submission_manifest.txt"
    report_path = out_dir / "submission_validation_report.md"
    write_manifest(
        manifest_path,
        result_dir,
        fused_path,
        raw_path,
        alignment,
        reference,
        point_rows,
        SCHEDULE_PATH,
    )
    write_validation_report(
        report_path,
        reference,
        checks,
        point_rows,
        dynamic_summaries,
        validation,
    )

    print()
    print("[generated]")
    print(f"  {out_dir / 'position_points.txt'}")
    for name, _, _ in DYNAMIC_WINDOWS:
        print(f"  {out_dir / f'position_dynamic_{name}.txt'}")
    print(f"  {manifest_path}")
    print(f"  {report_path}")
    print(f"  {submission_hash_path}")
    if reference.mode == "body":
        print("[BLOCKED] BODY REFERENCE ONLY; BASE reference offset unresolved.")
        print("[BLOCKED] Formal person-ground candidate was not generated.")
    print("=" * 80)
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
