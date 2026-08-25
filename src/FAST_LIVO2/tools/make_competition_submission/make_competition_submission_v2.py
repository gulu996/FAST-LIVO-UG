#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FAST-LIVO-UG 比赛提交文件转换器（目录自动发现版）

最常用用法：
    python3 make_competition_submission_v2.py ~/data/2026-08-23-202045

默认自动寻找：
    rtk_optimized_online.tum   # 首选，已经是 map/ENU，绝不再做刚体变换
    rtk_optimized_final.tum    # online 不存在时备用；也可 --use-final 强制使用
    livo_raw_online.tum        # 提前于融合轨迹开始的评分点的首选 fallback
    scans_pos.json             # raw TUM 不存在时备用
    rtk_backend.log            # 读取后端真正的 ALIGNMENT_SUCCESS yaw + translation

关键原则：
1) fused TUM 若声明 frame_id=map/ENU，则直接使用；不会对 fused 再做任何 R@p+t。
2) P01 这类早于 fused 起点的评分时刻，不再用“未来 fused 重叠段重新拟合坐标系”。
   而是优先读取 rtk_backend.log 中后端实际采用的 ALIGNMENT_SUCCESS，使用同一个
   yaw + xyz translation 将 livo_raw_online.tum 映射到 map/ENU。
3) 若 fused TUM 声明 saved_reference=body+R_body*result_pose_lever_arm_body_m，
   为保证提前评分点和 fused TUM 使用同一输出参考点，脚本只在 fallback 分支中
   从早期重叠数据估计这个固定 body-frame 输出杆臂；它绝不修改 fused 数据。
4) 比赛要求 NEU，因此 map/ENU TUM 的 (x,y,z) 输出为 (N,E,U)=(y,x,z)。
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np


# -----------------------------------------------------------------------------
# 当前比赛测试数据规定
# -----------------------------------------------------------------------------

SCORE_TIMES = [
    ("P01", 1785900562.0),
    ("P02", 1785900591.0),
    ("P03", 1785900626.0),
    ("P04", 1785900645.0),
    ("P05", 1785900672.0),
    ("P06", 1785900700.0),
    ("P07", 1785900720.0),
    ("P08", 1785900741.0),
    ("P09", 1785900773.0),
    ("P10", 1785900796.0),
]

DYNAMIC_WINDOWS = [
    ("01", 1785900626.0, 1785900646.0),
    ("02", 1785900675.0, 1785900695.0),
    ("03", 1785900748.0, 1785900768.0),
]

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
    def saved_reference_corrected(self) -> bool:
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
class LeverEstimate:
    lever_body_m: np.ndarray
    fit_rmse_m: float
    fit_median_m: float
    fit_p95_m: float
    pairs_used: int


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


def discover_files(result_dir: Path, use_final: bool):
    if use_final:
        fused_names = ["rtk_optimized_final.tum", "rtk_optimized_online.tum"]
    else:
        fused_names = ["rtk_optimized_online.tum", "rtk_optimized_final.tum"]

    fused = choose_existing(result_dir, fused_names, "fused trajectory", True)
    raw = choose_existing(
        result_dir,
        ["livo_raw_online.tum", "scans_pos.json"],
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
# 仅用于 fallback 的 saved-reference 输出杆臂恢复
# -----------------------------------------------------------------------------

def robust_lever_estimate(
    raw: Trajectory,
    fused: Trajectory,
    alignment: BackendAlignment,
    window_s: float = 20.0,
    max_fused_gap_s: float = 2.0,
) -> Optional[LeverEstimate]:
    """
    fused 文件若保存的是：
        p_saved = p_body + R_body * lever_body
    而 raw 文件是 odom/body，则在后端真实 map<-odom 对齐后：
        residual_i = p_fused_saved - T_map_odom(p_raw_body)
                   ~= R_map_body_i * lever_body

    这里只恢复一个固定 body-frame 输出杆臂，目的是让 P01 等提前时刻的 fallback
    与 fused TUM 的 saved_reference 定义一致。绝不会改 fused 轨迹本身。
    """
    start = max(raw.t0, fused.t0)
    end = min(raw.t1, fused.t1, start + window_s)
    if end <= start:
        return None

    rz = alignment.rotation_matrix()
    blocks = []
    rhs = []

    for row in fused.data:
        t = float(row[0])
        if t < start or t > end:
            continue

        raw_item = interpolate_pose(raw, t, max_gap_s=0.5)
        fused_item = interpolate_pose(fused, t, max_gap_s=max_fused_gap_s)
        if raw_item is None or fused_item is None:
            continue

        p_raw, q_raw, _ = raw_item
        p_fused, _, _ = fused_item

        p_map_body = alignment.transform_position(p_raw)
        r_map_body = rz @ quat_to_rot(q_raw)

        blocks.append(r_map_body)
        rhs.append(p_fused - p_map_body)

    if len(blocks) < 10:
        return None

    A = np.vstack(blocks)
    b = np.concatenate(rhs)
    lever, *_ = np.linalg.lstsq(A, b, rcond=None)

    # 一次 robust trimming：按每个 pose 的 3D 残差做 MAD 剔除。
    per_pose_err = []
    for R, r in zip(blocks, rhs):
        per_pose_err.append(float(np.linalg.norm(R @ lever - r)))
    per_pose_err = np.asarray(per_pose_err)

    med = float(np.median(per_pose_err))
    mad = float(np.median(np.abs(per_pose_err - med)))
    sigma = 1.4826 * mad
    threshold = med + (4.0 * sigma if sigma > 1e-9 else 0.05)
    threshold = max(threshold, 0.05)
    keep = per_pose_err <= threshold

    if int(np.count_nonzero(keep)) >= 10:
        A2 = np.vstack([R for R, k in zip(blocks, keep) if k])
        b2 = np.concatenate([r for r, k in zip(rhs, keep) if k])
        lever, *_ = np.linalg.lstsq(A2, b2, rcond=None)
        used_blocks = [R for R, k in zip(blocks, keep) if k]
        used_rhs = [r for r, k in zip(rhs, keep) if k]
    else:
        used_blocks = blocks
        used_rhs = rhs

    errs = np.asarray(
        [float(np.linalg.norm(R @ lever - r)) for R, r in zip(used_blocks, used_rhs)]
    )

    estimate = LeverEstimate(
        lever_body_m=np.asarray(lever, dtype=float),
        fit_rmse_m=float(np.sqrt(np.mean(errs ** 2))),
        fit_median_m=float(np.median(errs)),
        fit_p95_m=float(np.percentile(errs, 95)),
        pairs_used=int(len(errs)),
    )

    # 安全门：若根本不像固定杆臂，则不自动使用。
    if np.linalg.norm(estimate.lever_body_m) > 5.0:
        return None
    if estimate.fit_rmse_m > 0.20:
        return None

    return estimate


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
        fallback_lever_body_m: Optional[np.ndarray],
    ):
        self.fused = fused
        self.max_fused_gap_s = max_fused_gap_s
        self.raw = raw
        self.alignment = alignment
        self.fallback_lever_body_m = fallback_lever_body_m

    def pose(self, t: float):
        # 已是 map/ENU：直接用，绝不再变换。
        direct = interpolate_pose(self.fused, t, max_gap_s=self.max_fused_gap_s)
        if direct is not None:
            p, q, gap = direct
            return p, q, "fused_ENU_direct", gap

        # 仅当 fused 覆盖不到时，才走 raw fallback。
        if self.raw is None or self.alignment is None:
            return None

        raw_item = interpolate_pose(self.raw, t, max_gap_s=0.5)
        if raw_item is None:
            return None

        p_raw, q_raw, gap = raw_item
        rz = self.alignment.rotation_matrix()
        p_map = self.alignment.transform_position(p_raw)

        if self.fallback_lever_body_m is not None:
            r_map_body = rz @ quat_to_rot(q_raw)
            p_map = p_map + r_map_body @ self.fallback_lever_body_m

        return p_map, q_raw, "raw_backend_alignment_fallback", gap


# -----------------------------------------------------------------------------
# 坐标与 BDS 时间
# -----------------------------------------------------------------------------

def enu_to_neu(p_enu: np.ndarray) -> np.ndarray:
    e, n, u = np.asarray(p_enu, dtype=float)
    return np.array([n, e, u], dtype=float)


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
            item = traj.pose(t)
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
        help="one run directory, e.g. ~/data/2026-08-23-202045",
    )
    ap.add_argument(
        "--out-dir",
        default=None,
        help="default: <result_dir>/submission_generated",
    )
    ap.add_argument(
        "--use-final",
        action="store_true",
        help="prefer rtk_optimized_final.tum instead of online.tum",
    )
    ap.add_argument(
        "--max-fused-gap",
        type=float,
        default=2.0,
        help="maximum interpolation bracket inside fused trajectory (default 2.0 s)",
    )
    ap.add_argument(
        "--fallback-lever-window",
        type=float,
        default=20.0,
        help="overlap seconds used only to recover saved-reference lever for early fallback",
    )
    ap.add_argument(
        "--no-fallback-lever-inference",
        action="store_true",
        help="do not infer saved-reference lever for early raw fallback",
    )
    ap.add_argument(
        "--reference-dir",
        default=None,
        help="optional official reference directory; validation only",
    )
    args = ap.parse_args()

    result_dir = require_directory(Path(args.result_dir))
    out_dir = (
        Path(args.out_dir).expanduser().resolve()
        if args.out_dir
        else result_dir / "submission_generated"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    fused_path, raw_path, backend_log_path, gnss_path = discover_files(
        result_dir, args.use_final
    )

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

    print("[OK] fused TUM is map/ENU; fused positions are used directly.")
    print("[OK] NO rigid transform is applied to fused TUM.")

    # 哪些比赛时刻直接超出 fused 覆盖？
    required_times = [t for _, t in SCORE_TIMES]
    for _, start, end in DYNAMIC_WINDOWS:
        required_times.extend([start, end])

    missing_direct = sorted(
        {
            t
            for t in required_times
            if interpolate_pose(fused, t, max_gap_s=args.max_fused_gap) is None
        }
    )

    alignment = None
    fallback_lever = None
    lever_estimate = None

    if missing_direct:
        print()
        print(
            "[early/gap fallback] fused does not directly cover: "
            + ", ".join(f"{t:.1f}" for t in missing_direct)
        )

        if raw is None:
            raise RuntimeError(
                "Need livo_raw_online.tum (preferred) or scans_pos.json for uncovered epochs."
            )
        if backend_log_path is None:
            raise RuntimeError(
                "Need rtk_backend.log to recover the backend's actual ALIGNMENT_SUCCESS."
            )

        alignment = parse_backend_alignment(backend_log_path)
        if alignment is None:
            raise RuntimeError(
                f"No ALIGNMENT_SUCCESS found in {backend_log_path}; "
                "will not invent another local->ENU rigid alignment."
            )

        print(
            f"[OK] backend alignment: yaw={alignment.yaw_deg:+.6f} deg, "
            f"t=[{alignment.translation[0]:+.6f}, "
            f"{alignment.translation[1]:+.6f}, "
            f"{alignment.translation[2]:+.6f}]"
        )
        if alignment.rmse_m is not None:
            print(
                f"     backend alignment RMSE={alignment.rmse_m:.6f} m, "
                f"pairs={alignment.pairs}"
            )
        print(
            "[OK] uncovered epochs use the backend's own map<-odom alignment; "
            "no future-overlap coordinate re-fit is performed."
        )

        # 只有 fused 明确声明保存了 result_pose 杆臂时，才恢复同一参考点。
        if (
            fused.saved_reference_corrected
            and not args.no_fallback_lever_inference
        ):
            lever_estimate = robust_lever_estimate(
                raw,
                fused,
                alignment,
                window_s=args.fallback_lever_window,
                max_fused_gap_s=args.max_fused_gap,
            )
            if lever_estimate is not None:
                fallback_lever = lever_estimate.lever_body_m
                print(
                    "[OK] fused header says saved_reference uses result_pose lever; "
                    "estimated the same body-frame lever ONLY for early fallback:"
                )
                print(
                    "     lever_body_m=["
                    + ", ".join(f"{x:+.6f}" for x in fallback_lever)
                    + "]"
                )
                print(
                    f"     lever-fit RMSE={lever_estimate.fit_rmse_m:.4f} m, "
                    f"P95={lever_estimate.fit_p95_m:.4f} m, "
                    f"pairs={lever_estimate.pairs_used}"
                )
            else:
                print(
                    "WARNING: fused declares saved_reference correction, but a stable "
                    "fallback lever could not be recovered. Early fallback will use "
                    "the aligned raw body point only."
                )

    traj = CompetitionTrajectory(
        fused=fused,
        max_fused_gap_s=args.max_fused_gap,
        raw=raw,
        alignment=alignment,
        fallback_lever_body_m=fallback_lever,
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
        if len(rows) != 201:
            raise RuntimeError(
                f"dynamic {name}: expected 201 rows for inclusive 20 s @ 10 Hz, "
                f"got {len(rows)}"
            )
        dynamic_summaries.append((name, len(rows), counts))

    # 对测试数据做一个固定 sanity check。
    week, sow = unix_to_bds_week_sow(1785900626.0)
    if week != 1074 or abs(sow - 271830.0) > 1e-6:
        raise RuntimeError(
            f"BDS conversion sanity failed: week={week}, sow={sow:.6f}"
        )

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

    report = [
        "FAST-LIVO-UG competition conversion report",
        "=" * 80,
        f"result_dir={result_dir}",
        f"fused={fused_path}",
        f"raw={raw_path}",
        f"backend_log={backend_log_path}",
        f"fused_map_enu={fused.is_map_enu}",
        "fused_rigid_transform_applied=False",
        f"fused_saved_reference_corrected={fused.saved_reference_corrected}",
        "",
    ]

    if alignment is not None:
        report += [
            "Backend alignment used ONLY for uncovered epochs:",
            f"yaw_deg={alignment.yaw_deg:.9f}",
            "translation=" + " ".join(f"{x:.9f}" for x in alignment.translation),
            f"rmse_m={alignment.rmse_m}",
            f"pairs={alignment.pairs}",
            "",
        ]

    if lever_estimate is not None:
        report += [
            "Saved-reference lever recovered ONLY for raw fallback:",
            "lever_body_m="
            + " ".join(f"{x:.9f}" for x in lever_estimate.lever_body_m),
            f"fit_rmse_m={lever_estimate.fit_rmse_m:.9f}",
            f"fit_p95_m={lever_estimate.fit_p95_m:.9f}",
            f"pairs_used={lever_estimate.pairs_used}",
            "",
        ]

    report.append("Scoring points:")
    for pid, t, n, e, u, source, _ in point_rows:
        report.append(
            f"{pid},{t:.1f},{source},N={n:.4f},E={e:.4f},U={u:.4f}"
        )

    report += ["", "Dynamic outputs:"]
    for name, count, counts in dynamic_summaries:
        report.append(f"{name}: rows={count}, sources={counts}")

    if args.reference_dir:
        ref_dir = Path(args.reference_dir).expanduser().resolve()
        validation = evaluate_reference(out_dir, ref_dir)
        if validation:
            print()
            print("[reference validation only — not used for generation/alignment]")
            for line in validation:
                print(line)
            report += [
                "",
                "Reference validation only (not used for generation/alignment):",
                *validation,
            ]

    report_path = out_dir / "conversion_report.txt"
    report_path.write_text("\n".join(report) + "\n", encoding="utf-8")

    print()
    print("[generated]")
    print(f"  {out_dir / 'position_points.txt'}")
    for name, _, _ in DYNAMIC_WINDOWS:
        print(f"  {out_dir / f'position_dynamic_{name}.txt'}")
    print(f"  {report_path}")
    print("=" * 80)
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
