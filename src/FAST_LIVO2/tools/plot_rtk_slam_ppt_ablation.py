#!/usr/bin/env python3
"""Generate three evidence-backed Chinese PPT figures for RTK-SLAM s2.

The script intentionally keeps the three claims separate:

1. a causal online trajectory continues through a real Fixed-RTK outage;
2. accepted GNSS graph constraints, causal online output, and the non-causal
   20 s fixed-lag result are different data products;
3. fixed covariance and per-epoch covariance are compared only after an
   audited, same-input paired run.

Ground truth is always rendered as unconnected sparse checkpoint markers.
"""

import argparse
import ast
import hashlib
import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.gridspec import GridSpec
import numpy as np
import rosbag


WINDOW_SECONDS = 360.0
GNSS_TOPIC = "/gnss/fix"
MAX_ASSOCIATION_SECONDS = 2.0
CONFIG_KEYS = (
    "min_gnss_sigma_xy_m",
    "min_gnss_sigma_z_m",
    "max_gnss_sigma_xy_m",
    "max_gnss_sigma_z_m",
)
OUTPUT_NAMES = (
    "01_continuous_localization_cn.png",
    "02_multisensor_fusion_cn.png",
    "03_fixed_vs_epoch_covariance_cn.png",
)

BLUE = "#1769AA"
RED = "#D1495B"
PURPLE = "#6F4C9B"
ORANGE = "#ED8B00"
GREEN = "#2A9D5B"
GREY = "#6B7280"
GRID = "#D7DCE2"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def configure_chinese_font():
    """Register the installed Noto CJK collection under its requested SC name."""
    candidates = (
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf"),
        Path("/usr/share/fonts/truetype/noto/NotoSansCJKsc-Regular.otf"),
    )
    font_path = next((path for path in candidates if path.is_file()), None)
    if font_path is None:
        raise FileNotFoundError("未找到 Noto Sans CJK SC 字体，拒绝用缺字字体生成正式图")

    # Matplotlib 3.1 indexes only the first face in a TTC. Fontconfig confirms
    # that this installed collection also contains the SC face; the alias keeps
    # the requested family name stable while using that audited Noto CJK file.
    font_manager.fontManager.ttflist.insert(
        0, font_manager.FontEntry(fname=str(font_path), name="Noto Sans CJK SC")
    )
    matplotlib.rcParams.update(
        {
            "font.family": "Noto Sans CJK SC",
            "font.sans-serif": ["Noto Sans CJK SC"],
            "axes.unicode_minus": False,
            "font.size": 10.5,
            "axes.titlesize": 11.5,
            "axes.labelsize": 10.5,
            "legend.fontsize": 9.0,
            "axes.linewidth": 0.8,
            "savefig.facecolor": "white",
        }
    )
    return font_path


def load_official_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise ImportError("无法加载官方评测模块: {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_official_helpers(evaluator_root):
    root = Path(evaluator_root).resolve()
    candidates = (root / "eval", root)
    eval_dir = next(
        (
            candidate
            for candidate in candidates
            if (candidate / "coords.py").is_file()
            and (candidate / "readers.py").is_file()
        ),
        None,
    )
    if eval_dir is None:
        raise FileNotFoundError(
            "evaluator_root 下未找到官方 eval/coords.py 与 eval/readers.py"
        )
    coords = load_official_module("rtk_slam_official_coords", eval_dir / "coords.py")
    readers = load_official_module(
        "rtk_slam_official_readers", eval_dir / "readers.py"
    )
    return eval_dir, coords, readers


def parse_input_window(path):
    values = {}
    with Path(path).open(encoding="utf-8") as stream:
        for raw in stream:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    required = (
        "sequence",
        "bag",
        "play_start_s",
        "play_duration_s",
        "sensor_start_unix",
        "sensor_end_unix",
    )
    missing = [key for key in required if key not in values]
    if missing:
        raise ValueError("input_window.txt 缺少字段: {}".format(", ".join(missing)))
    return values


def parse_yaml_scalars(path):
    """Parse the unique leaf keys used by the frozen backend YAML snapshot."""
    values = {}
    pattern = re.compile(r"^\s+([A-Za-z0-9_]+):\s*(.*?)\s*$")
    with Path(path).open(encoding="utf-8") as stream:
        for raw in stream:
            # ponytail: these snapshots have unique leaf keys and no '#'
            # inside scalar strings; switch to PyYAML if that schema changes.
            line = raw.split("#", 1)[0].rstrip()
            match = pattern.match(line)
            if not match or not match.group(2):
                continue
            key, value = match.groups()
            if key in values:
                raise ValueError("后端 YAML 出现重复叶键，简化解析器拒绝猜测: {}".format(key))
            values[key] = value.strip()
    return values


def scalar_float(config, key):
    if key not in config:
        raise ValueError("后端配置缺少 {}".format(key))
    try:
        value = float(config[key])
    except ValueError as error:
        raise ValueError("{} 不是数值: {}".format(key, config[key])) from error
    if not math.isfinite(value):
        raise ValueError("{} 必须为有限数".format(key))
    return value


def scalar_vector(config, key):
    if key not in config:
        raise ValueError("后端配置缺少 {}".format(key))
    try:
        value = np.asarray(ast.literal_eval(config[key]), dtype=float)
    except (SyntaxError, ValueError) as error:
        raise ValueError("{} 不是数值向量: {}".format(key, config[key])) from error
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("{} 必须是 3 维有限向量".format(key))
    return value


def audit_ablation_configs(dynamic_run, fixed_run):
    dynamic_path = Path(dynamic_run) / "config/rtk_fixed_lag_backend.yaml"
    fixed_path = Path(fixed_run) / "config/rtk_fixed_lag_backend.yaml"
    dynamic = parse_yaml_scalars(dynamic_path)
    fixed = parse_yaml_scalars(fixed_path)
    if set(dynamic) != set(fixed):
        raise ValueError("两组后端配置的字段集合不同，不能作为四参数消融")
    changed = {key for key in dynamic if dynamic[key] != fixed[key]}
    if changed != set(CONFIG_KEYS):
        raise ValueError(
            "消融配置必须且只能修改四个 GNSS min/max 字段；实际差异: {}".format(
                sorted(changed)
            )
        )

    dmin_xy = scalar_float(dynamic, "min_gnss_sigma_xy_m")
    dmax_xy = scalar_float(dynamic, "max_gnss_sigma_xy_m")
    dmin_z = scalar_float(dynamic, "min_gnss_sigma_z_m")
    dmax_z = scalar_float(dynamic, "max_gnss_sigma_z_m")
    fmin_xy = scalar_float(fixed, "min_gnss_sigma_xy_m")
    fmax_xy = scalar_float(fixed, "max_gnss_sigma_xy_m")
    fmin_z = scalar_float(fixed, "min_gnss_sigma_z_m")
    fmax_z = scalar_float(fixed, "max_gnss_sigma_z_m")
    if not (0.0 < dmin_xy < dmax_xy and 0.0 < dmin_z < dmax_z):
        raise ValueError("逐历元协方差组必须保留非退化的 min/max 范围")
    if not (fmin_xy == fmax_xy and fmin_z == fmax_z):
        raise ValueError("固定协方差组必须令每一轴的 min 与 max 完全相等")

    return {
        "dynamic_path": dynamic_path,
        "fixed_path": fixed_path,
        "dynamic": dynamic,
        "fixed": fixed,
        "changed_keys": sorted(changed),
        "dynamic_xy_bounds_m": [dmin_xy, dmax_xy],
        "dynamic_z_bounds_m": [dmin_z, dmax_z],
        "fixed_xy_sigma_m": fmin_xy,
        "fixed_z_sigma_m": fmin_z,
    }


def validate_same_input(dynamic_run, fixed_run, bag):
    dynamic = parse_input_window(Path(dynamic_run) / "input_window.txt")
    fixed = parse_input_window(Path(fixed_run) / "input_window.txt")
    if dynamic["sequence"] != "stadtgarten_seq2" or fixed["sequence"] != dynamic["sequence"]:
        raise ValueError("本脚本只接受同一 stadtgarten_seq2 输入的成对运行")
    for key in (
        "play_start_s",
        "play_duration_s",
        "sensor_start_unix",
        "sensor_end_unix",
    ):
        if abs(float(dynamic[key]) - float(fixed[key])) > 1e-9:
            raise ValueError("两组运行的 {} 不一致".format(key))
    passed_bag = Path(bag).resolve()
    for label, window in (("逐历元", dynamic), ("固定", fixed)):
        if Path(window["bag"]).resolve() != passed_bag:
            raise ValueError("{}组 input_window 与 --bag 不是同一个文件".format(label))
    for label, window in (("逐历元", dynamic), ("固定", fixed)):
        duration = float(window["play_duration_s"])
        start = float(window["sensor_start_unix"])
        end = float(window["sensor_end_unix"])
        if abs(duration - WINDOW_SECONDS) > 1e-9:
            raise ValueError("{}组 play_duration_s 必须为 360".format(label))
        if abs((end - start) - WINDOW_SECONDS) > 1e-6:
            raise ValueError("{}组 sensor_start/end 未形成 360 s 窗口".format(label))
    return dynamic, fixed, float(dynamic["sensor_start_unix"])


def audit_same_native_inputs(dynamic_run, fixed_run):
    result = {}
    for relative, key in (
        ("native/livo_raw_online.tum", "raw_livo"),
        ("native/gnss_enu.tum", "gnss_constraints"),
    ):
        dynamic_path = Path(dynamic_run) / relative
        fixed_path = Path(fixed_run) / relative
        if not dynamic_path.is_file() or not fixed_path.is_file():
            raise FileNotFoundError("成对运行缺少同输入审计文件: {}".format(relative))
        dynamic_hash = sha256(dynamic_path)
        fixed_hash = sha256(fixed_path)
        if dynamic_hash != fixed_hash:
            raise ValueError("成对运行的 {} 并非逐字节相同".format(relative))
        result[key] = {
            "dynamic_path": str(dynamic_path),
            "fixed_path": str(fixed_path),
            "sha256": dynamic_hash,
        }
    return result


def read_origin(run):
    with (Path(run) / "enu_origin.json").open(encoding="utf-8") as stream:
        origin = np.asarray(json.load(stream), dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("无效 ENU 原点: {}".format(run))
    return origin


def read_tum(path):
    path = Path(path)
    data = np.loadtxt(str(path), comments="#")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != 8 or not np.isfinite(data).all():
        raise ValueError("TUM 轨迹必须为 8 列有限数: {}".format(path))
    if np.any(np.diff(data[:, 0]) <= 0.0):
        raise ValueError("正式作图要求时间戳严格递增且唯一: {}".format(path))
    return data


def trajectory_path(run, role):
    names = {
        "online": ("trajectory_eval_online.txt", "native/rtk_optimized_online.tum"),
        "final": ("trajectory_eval_offline.txt", "native/rtk_optimized_final.tum"),
    }
    for relative in names[role]:
        path = Path(run) / relative
        if path.is_file():
            return path
    raise FileNotFoundError("{} 缺少 {} 轨迹".format(run, role))


def crop(data, start, duration=WINDOW_SECONDS):
    elapsed = data[:, 0] - start
    selected = data[(elapsed >= 0.0) & (elapsed <= duration)]
    if len(selected) < 2:
        raise ValueError("前 {:.0f} s 内轨迹样本不足".format(duration))
    return selected


def enu_positions_to_utm(positions, origin, coords):
    converted = coords.convert_extracted_points_to_utm(
        [{"position": position.copy()} for position in positions], origin.tolist()
    )
    result = np.vstack([row["position"] for row in converted])
    if result.shape != positions.shape or not np.isfinite(result).all():
        raise ValueError("官方 ENU→UTM 坐标转换失败")
    return result


def ecef_to_enu_rotation(latitude, longitude):
    lat = math.radians(latitude)
    lon = math.radians(longitude)
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)
    return np.asarray(
        (
            (-slon, clon, 0.0),
            (-slat * clon, -slat * slon, clat),
            (clat * clon, clat * slon, slat),
        )
    )


def extract_gnss(bag_path, start, origin, config_audit):
    records = []
    origin_rotation = ecef_to_enu_rotation(origin[0], origin[1])
    dmin, dmax = config_audit["dynamic_xy_bounds_m"]
    fixed_sigma = config_audit["fixed_xy_sigma_m"]
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[GNSS_TOPIC]):
            stamp = message.header.stamp.to_sec()
            elapsed = stamp - start
            if elapsed < -2.0:
                continue
            if elapsed > WINDOW_SECONDS + 2.0:
                break
            covariance = np.asarray(message.position_covariance, dtype=float).reshape(3, 3)
            valid_covariance = (
                int(message.position_covariance_type) != 0
                and np.isfinite(covariance).all()
                and np.all(np.diag(covariance) > 0.0)
            )
            valid_fix = (
                int(message.status.status) == 0
                and math.isfinite(message.latitude)
                and math.isfinite(message.longitude)
                and math.isfinite(message.altitude)
                and valid_covariance
            )
            sigma_dynamic = math.nan
            sigma_fixed = math.nan
            if valid_fix:
                fix_rotation = ecef_to_enu_rotation(
                    message.latitude, message.longitude
                )
                axes = origin_rotation @ fix_rotation.T
                rotated = axes @ covariance @ axes.T
                if np.any(np.diag(rotated) <= 0.0) or not np.isfinite(rotated).all():
                    valid_fix = False
                else:
                    sigma_e = np.clip(math.sqrt(rotated[0, 0]), dmin, dmax)
                    sigma_n = np.clip(math.sqrt(rotated[1, 1]), dmin, dmax)
                    sigma_dynamic = math.hypot(sigma_e, sigma_n)
                    sigma_fixed = math.sqrt(2.0) * fixed_sigma
            records.append(
                (elapsed, valid_fix, sigma_dynamic, sigma_fixed, stamp)
            )
    if not records:
        raise ValueError("bag 中没有 {} 消息".format(GNSS_TOPIC))
    records.sort(key=lambda row: row[0])
    data = np.asarray(records, dtype=float)
    if np.any(np.diff(data[:, 0]) <= 0.0):
        raise ValueError("GNSS 消息头时间戳不是严格递增")
    return data


def fixed_gaps(gnss, max_gap_seconds=1.0):
    accepted = gnss[gnss[:, 1] == 1.0, 0]
    if len(accepted) < 2:
        raise ValueError("Fixed GNSS 有效历元不足")
    gaps = []
    for left, right in zip(accepted[:-1], accepted[1:]):
        if right - left > max_gap_seconds and right > 0.0 and left < WINDOW_SECONDS:
            gaps.append((max(0.0, float(left)), min(WINDOW_SECONDS, float(right))))
    if not gaps:
        raise ValueError("前 360 s 未识别到 Fixed GNSS 中断")
    longest = max(gaps, key=lambda pair: pair[1] - pair[0])
    if abs((longest[1] - longest[0]) - 141.6) > 0.15:
        raise ValueError(
            "s2 前 360 s 最长 Fixed 中断不是已审计的 141.6 s: {:.6f}".format(
                longest[1] - longest[0]
            )
        )
    return gaps, longest


def gap_mask(elapsed, gaps):
    mask = np.zeros(len(elapsed), dtype=bool)
    for left, right in gaps:
        mask |= (elapsed >= left) & (elapsed <= right)
    return mask


def complement_intervals(gaps):
    intervals = []
    cursor = 0.0
    for left, right in sorted(gaps):
        if left > cursor:
            intervals.append((cursor, left))
        cursor = max(cursor, right)
    if cursor < WINDOW_SECONDS:
        intervals.append((cursor, WINDOW_SECONDS))
    return intervals


def plot_masked_path(ax, xy, mask, *args, **kwargs):
    points = xy.copy()
    points[~mask] = np.nan
    return ax.plot(points[:, 0], points[:, 1], *args, **kwargs)


def plot_segmented_series(ax, x, y, max_gap, *args, **kwargs):
    cuts = np.flatnonzero(np.diff(x) > max_gap) + 1
    starts = np.r_[0, cuts]
    ends = np.r_[cuts, len(x)]
    for index, (left, right) in enumerate(zip(starts, ends)):
        segment_kwargs = dict(kwargs)
        if index:
            segment_kwargs["label"] = "_nolegend_"
        ax.plot(x[left:right], y[left:right], *args, **segment_kwargs)


def nearest_indices(source_timestamps, target_timestamps):
    right = np.searchsorted(source_timestamps, target_timestamps)
    right = np.clip(right, 0, len(source_timestamps) - 1)
    left = np.maximum(right - 1, 0)
    choose_left = (
        np.abs(source_timestamps[left] - target_timestamps)
        <= np.abs(source_timestamps[right] - target_timestamps)
    )
    return np.where(choose_left, left, right)


def quaternion_rotate(quaternions, vectors):
    q = np.asarray(quaternions, dtype=float)
    v = np.asarray(vectors, dtype=float)
    norms = np.linalg.norm(q, axis=1)
    if np.any(norms < 1e-12):
        raise ValueError("轨迹含零四元数")
    q = q / norms[:, None]
    if v.ndim == 1:
        v = np.tile(v, (len(q), 1))
    xyz = q[:, :3]
    return v + 2.0 * np.cross(xyz, np.cross(xyz, v) + q[:, 3, None] * v)


def gnss_constraints_at_base(dynamic_run, online, start, config):
    path = Path(dynamic_run) / "native/gnss_enu.tum"
    constraints = crop(read_tum(path), start)
    indices = nearest_indices(online[:, 0], constraints[:, 0])
    residual = np.abs(online[indices, 0] - constraints[:, 0])
    reuse_limit = scalar_float(config, "reuse_existing_node_time_diff_s")
    if residual.max() > reuse_limit + 1e-6:
        raise ValueError("GNSS 约束找不到同历元在线姿态，不能做杆臂换算")
    antenna = scalar_vector(config, "antenna_lever_arm_body_m")
    base = scalar_vector(config, "result_pose_lever_arm_body_m")
    correction = quaternion_rotate(online[indices, 4:8], base - antenna)
    return path, constraints[:, 0], constraints[:, 1:4] + correction, residual


def checkpoint_errors(readers, trajectory_timestamps, trajectory_utm, gt_rows):
    gt_timestamps = [row["timestamp"] for row in gt_rows]
    found = readers.find_nearest_pose(
        trajectory_timestamps,
        trajectory_utm,
        gt_timestamps,
        max_dt=MAX_ASSOCIATION_SECONDS,
    )
    if len(found) != len(gt_rows):
        raise ValueError(
            "前 360 s 的 {} 个检查点未全部按官方 ±2 s 规则匹配".format(
                len(gt_rows)
            )
        )
    errors = np.full(len(gt_rows), np.nan)
    for match in found:
        index = int(match["index"])
        errors[index] = np.linalg.norm(
            np.asarray(match["position"]) - gt_rows[index]["position"]
        )
    if not np.isfinite(errors).all():
        raise ValueError("检查点误差关联不完整")
    rmse = float(math.sqrt(np.mean(errors * errors)))
    nearest = nearest_indices(trajectory_timestamps, np.asarray(gt_timestamps))
    time_residual = np.abs(
        trajectory_timestamps[nearest] - np.asarray(gt_timestamps)
    )
    return errors, rmse, time_residual


def add_gt(ax, gt_xy, label="测量真值检查点（不连线）"):
    ax.scatter(
        gt_xy[:, 0],
        gt_xy[:, 1],
        marker="*",
        s=78,
        c="#111111",
        edgecolors="white",
        linewidths=0.45,
        zorder=20,
        label=label,
    )


def style_xy_axis(ax):
    ax.set_xlabel("东向相对坐标 / m")
    ax.set_ylabel("北向相对坐标 / m")
    ax.grid(True, color=GRID, linestyle=":", linewidth=0.7)
    ax.set_aspect("equal", adjustable="datalim")


def draw_availability_band(ax, gaps, longest_gap):
    for left, right in complement_intervals(gaps):
        ax.axvspan(left, right, color=GREEN, alpha=0.78, linewidth=0)
    for left, right in gaps:
        ax.axvspan(left, right, color=RED, alpha=0.82, linewidth=0)
    midpoint = 0.5 * (longest_gap[0] + longest_gap[1])
    ax.text(
        midpoint,
        0.5,
        "Fixed RTK 中断 {:.1f} s".format(longest_gap[1] - longest_gap[0]),
        ha="center",
        va="center",
        color="white",
        fontweight="bold",
        fontsize=8.6,
    )
    ax.text(4.0, 0.5, "Fixed 可用", ha="left", va="center", color="white", fontsize=8.6)
    ax.set_xlim(0.0, WINDOW_SECONDS)
    ax.set_ylim(0.0, 1.0)
    ax.set_yticks([])
    ax.set_xlabel("距本次评测窗口起点时间 / s")
    for spine in ax.spines.values():
        spine.set_visible(False)


def atomic_save_png(fig, path):
    path = Path(path)
    temporary = path.with_name("." + path.name + ".tmp")
    fig.savefig(
        str(temporary),
        format="png",
        dpi=300,
        facecolor="white",
        bbox_inches="tight",
        pad_inches=0.08,
    )
    plt.close(fig)
    temporary.replace(path)


def write_json_atomic(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name("." + path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    temporary.replace(path)


def self_check():
    q = np.asarray([[0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)]])
    got = quaternion_rotate(q, np.asarray([1.0, 0.0, 0.0]))[0]
    assert np.allclose(got, [0.0, 1.0, 0.0], atol=1e-12)
    source = np.asarray([0.0, 1.0, 3.0])
    target = np.asarray([0.2, 2.6])
    assert np.array_equal(nearest_indices(source, target), [0, 2])
    assert complement_intervals([(10.0, 20.0)]) == [(0.0, 10.0), (20.0, 360.0)]
    print("PASS: 四元数、最近历元与可用性区间自检")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dynamic-run", type=Path)
    parser.add_argument("--fixed-run", type=Path)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--gt", type=Path)
    parser.add_argument("--evaluator-root", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--metrics-out", type=Path)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args(argv)
    if args.self_check:
        self_check()
        return 0
    required = (
        "dynamic_run",
        "fixed_run",
        "bag",
        "gt",
        "evaluator_root",
        "output_dir",
        "metrics_out",
    )
    missing = ["--" + key.replace("_", "-") for key in required if getattr(args, key) is None]
    if missing:
        parser.error("正式作图缺少参数: {}".format(", ".join(missing)))

    dynamic_run = args.dynamic_run.resolve()
    fixed_run = args.fixed_run.resolve()
    bag_path = args.bag.resolve()
    gt_path = args.gt.resolve()
    output_dir = args.output_dir.resolve()
    metrics_out = args.metrics_out.resolve()
    for path, label in (
        (dynamic_run, "dynamic_run"),
        (fixed_run, "fixed_run"),
        (bag_path, "bag"),
        (gt_path, "gt"),
    ):
        if not path.exists():
            raise FileNotFoundError("{} 不存在: {}".format(label, path))
    if metrics_out.suffix.lower() == ".png":
        raise ValueError("metrics_out 必须是 JSON 等文本文件，不能占用三张 PNG 名额")
    output_dir.mkdir(parents=True, exist_ok=True)

    font_path = configure_chinese_font()
    eval_dir, coords, readers = load_official_helpers(args.evaluator_root)
    dynamic_window, fixed_window, start = validate_same_input(
        dynamic_run, fixed_run, bag_path
    )
    config_audit = audit_ablation_configs(dynamic_run, fixed_run)
    native_input_audit = audit_same_native_inputs(dynamic_run, fixed_run)
    dynamic_origin = read_origin(dynamic_run)
    fixed_origin = read_origin(fixed_run)
    if not np.allclose(dynamic_origin, fixed_origin, atol=1e-12, rtol=0.0):
        raise ValueError("同输入消融的 ENU 原点不一致")

    dynamic_online_path = trajectory_path(dynamic_run, "online")
    dynamic_final_path = trajectory_path(dynamic_run, "final")
    fixed_online_path = trajectory_path(fixed_run, "online")
    dynamic_online = crop(read_tum(dynamic_online_path), start)
    dynamic_final = crop(read_tum(dynamic_final_path), start)
    fixed_online = crop(read_tum(fixed_online_path), start)
    dynamic_online_utm = enu_positions_to_utm(
        dynamic_online[:, 1:4], dynamic_origin, coords
    )
    dynamic_final_utm = enu_positions_to_utm(
        dynamic_final[:, 1:4], dynamic_origin, coords
    )
    fixed_online_utm = enu_positions_to_utm(
        fixed_online[:, 1:4], fixed_origin, coords
    )

    all_gt = readers.read_ground_truth(str(gt_path))
    gt_rows = [
        row
        for row in all_gt
        if 0.0 <= row["timestamp"] - start <= WINDOW_SECONDS
    ]
    if len(gt_rows) != 8:
        raise ValueError("s2 前 360 s 应有 8 个稀疏检查点，实际 {}".format(len(gt_rows)))
    gt_xyz = np.vstack([row["position"] for row in gt_rows])
    gt_xy_origin = gt_xyz[0, :2]
    gt_xy = gt_xyz[:, :2] - gt_xy_origin

    gnss = extract_gnss(bag_path, start, dynamic_origin, config_audit)
    gaps, longest_gap = fixed_gaps(gnss)
    outage_seconds = longest_gap[1] - longest_gap[0]

    # Figure 1: one causal trajectory, split only by measured Fixed availability.
    elapsed_online = dynamic_online[:, 0] - start
    online_xy = dynamic_online_utm[:, :2] - gt_xy_origin
    online_outage = gap_mask(elapsed_online, gaps)
    fig, ax = plt.subplots(figsize=(12.8, 5.0))
    plot_masked_path(
        ax,
        online_xy,
        ~online_outage,
        color=BLUE,
        linewidth=2.1,
        label="因果在线轨迹：RTK 固定解可用",
    )
    plot_masked_path(
        ax,
        online_xy,
        online_outage,
        color=RED,
        linewidth=2.45,
        label="因果在线轨迹：RTK 固定解中断（{:.1f} s）".format(outage_seconds),
    )
    add_gt(ax, gt_xy)
    ax.scatter(online_xy[0, 0], online_xy[0, 1], marker="o", s=35, c=BLUE, zorder=21)
    ax.scatter(online_xy[-1, 0], online_xy[-1, 1], marker="s", s=34, c=GREY, zorder=21)
    style_xy_axis(ax)
    max_online_gap = float(np.diff(dynamic_online[:, 0]).max())
    outage_online_count = int(online_outage.sum())
    ax.text(
        0.985,
        0.02,
        "全局在线输出始于 {:.3f} s；中断区间保留 {} 个关键帧；最大相邻间隔 {:.3f} s".format(
            elapsed_online[0], outage_online_count, max_online_gap
        ),
        transform=ax.transAxes,
        ha="right",
        fontsize=9.0,
        color="#30343B",
        bbox={"facecolor": "white", "edgecolor": GRID, "alpha": 0.92, "pad": 4},
    )
    ax.legend(loc="best", frameon=True, ncol=1)
    ax.set_title("GNSS 中断前后的连续在线定位轨迹（城市花园序列 2，前 360 s）", fontweight="bold")
    fig.text(
        0.5,
        0.025,
        "注：轨迹为产生时因果的在线估计；黑色星号是 8 个独立测量检查点，仅作散点显示，数据集不提供连续真值轨迹。",
        ha="center",
        fontsize=9.0,
    )
    fig.subplots_adjust(left=0.08, right=0.985, top=0.89, bottom=0.17)
    figure1_path = output_dir / OUTPUT_NAMES[0]
    atomic_save_png(fig, figure1_path)

    # Figure 2: accepted factor positions are antenna measurements. Convert
    # them to the base center with the calibrated lever arms and same-epoch
    # causal orientation before comparing them with saved base-center tracks.
    gnss_path, gnss_timestamps, gnss_base_enu, gnss_orientation_dt = (
        gnss_constraints_at_base(
            dynamic_run, dynamic_online, start, config_audit["dynamic"]
        )
    )
    gnss_base_utm = enu_positions_to_utm(gnss_base_enu, dynamic_origin, coords)
    gnss_xy = gnss_base_utm[:, :2] - gt_xy_origin
    final_xy = dynamic_final_utm[:, :2] - gt_xy_origin
    fig = plt.figure(figsize=(12.8, 5.25))
    grid = GridSpec(2, 1, height_ratios=[8.0, 1.0], hspace=0.34, figure=fig)
    ax = fig.add_subplot(grid[0])
    band = fig.add_subplot(grid[1])
    ax.scatter(
        gnss_xy[:, 0],
        gnss_xy[:, 1],
        s=12,
        c=ORANGE,
        alpha=0.42,
        linewidths=0,
        label="加入图优化的 GNSS 约束（换算至基座中心）",
        zorder=3,
    )
    ax.plot(
        online_xy[:, 0],
        online_xy[:, 1],
        color=BLUE,
        linewidth=1.75,
        label="因果在线轨迹",
        zorder=6,
    )
    ax.plot(
        final_xy[:, 0],
        final_xy[:, 1],
        color=PURPLE,
        linewidth=2.1,
        linestyle=(0, (5, 2)),
        label="20 s 固定滞后最终轨迹（非因果）",
        zorder=5,
    )
    add_gt(ax, gt_xy)
    style_xy_axis(ax)
    ax.legend(loc="best", frameon=True, ncol=2, fontsize=8.5)
    ax.set_title("GNSS 约束与在线/固定滞后融合轨迹（城市花园序列 2，前 360 s）", fontweight="bold")
    draw_availability_band(band, gaps, longest_gap)
    if not np.array_equal(dynamic_online[:, 0], dynamic_final[:, 0]):
        raise ValueError("在线与固定滞后轨迹的前 360 s 时间戳不一致")
    fixed_lag_delta = np.linalg.norm(
        dynamic_final[:, 1:4] - dynamic_online[:, 1:4], axis=1
    )
    fixed_lag_rms = float(math.sqrt(np.mean(fixed_lag_delta * fixed_lag_delta)))
    fixed_lag_max_index = int(np.argmax(fixed_lag_delta))
    fixed_lag_max = float(fixed_lag_delta[fixed_lag_max_index])
    fixed_lag_max_time = float(elapsed_online[fixed_lag_max_index])
    fig.text(
        0.5,
        0.012,
        "注：GNSS 散点仅表示实际加入后端的因子；最终轨迹最多使用未来 20 s 信息，不等同于因果在线结果。",
        ha="center",
        fontsize=9.0,
    )
    fig.subplots_adjust(left=0.08, right=0.985, top=0.91, bottom=0.16)
    figure2_path = output_dir / OUTPUT_NAMES[1]
    atomic_save_png(fig, figure2_path)

    # Figure 3: paired, same-input covariance ablation. No direction of the
    # result is baked into the labels or conclusion.
    dynamic_errors, dynamic_rmse, dynamic_time_residual = checkpoint_errors(
        readers, dynamic_online[:, 0], dynamic_online_utm, gt_rows
    )
    fixed_errors, fixed_rmse, fixed_time_residual = checkpoint_errors(
        readers, fixed_online[:, 0], fixed_online_utm, gt_rows
    )
    checkpoint_elapsed = np.asarray([row["timestamp"] - start for row in gt_rows])
    outage_checkpoints = gap_mask(checkpoint_elapsed, gaps)
    if int(outage_checkpoints.sum()) != 4:
        raise ValueError("s2 前 360 s 的 Fixed 中断区间应含 4 个检查点")
    dynamic_outage_rmse = float(
        math.sqrt(np.mean(dynamic_errors[outage_checkpoints] ** 2))
    )
    fixed_outage_rmse = float(
        math.sqrt(np.mean(fixed_errors[outage_checkpoints] ** 2))
    )
    fixed_xy = fixed_online_utm[:, :2] - gt_xy_origin
    valid_sigma = (
        (gnss[:, 0] >= 0.0)
        & (gnss[:, 0] <= WINDOW_SECONDS)
        & (gnss[:, 1] == 1.0)
    )
    sigma_time = gnss[valid_sigma, 0]
    sigma_dynamic = gnss[valid_sigma, 2]
    sigma_fixed = gnss[valid_sigma, 3]
    if len(sigma_time) < 2 or np.ptp(sigma_dynamic) <= 1e-12:
        raise ValueError("逐历元水平标准差没有真实变化，不能生成动态协方差图")
    if np.ptp(sigma_fixed) > 1e-12:
        raise ValueError("固定协方差组的水平标准差不是常数")

    fig = plt.figure(figsize=(14.0, 5.35))
    grid = GridSpec(
        2,
        3,
        width_ratios=[1.50, 1.0, 1.0],
        height_ratios=[1.0, 1.0],
        hspace=0.46,
        wspace=0.34,
        figure=fig,
    )
    trajectory_ax = fig.add_subplot(grid[:, 0])
    error_ax = fig.add_subplot(grid[0, 1:])
    sigma_ax = fig.add_subplot(grid[1, 1:])
    trajectory_ax.plot(
        online_xy[:, 0],
        online_xy[:, 1],
        color=BLUE,
        linewidth=1.8,
        label="逐历元协方差（因果在线）",
    )
    trajectory_ax.plot(
        fixed_xy[:, 0],
        fixed_xy[:, 1],
        color=RED,
        linewidth=1.65,
        linestyle=(0, (5, 2)),
        label="固定协方差（因果在线）",
    )
    add_gt(trajectory_ax, gt_xy)
    style_xy_axis(trajectory_ax)
    trajectory_ax.set_title("（a）同输入在线轨迹")
    trajectory_ax.legend(loc="best", fontsize=8.0, frameon=True)

    x = np.arange(len(gt_rows))
    width = 0.37
    dynamic_bars = error_ax.bar(
        x - width / 2.0,
        dynamic_errors,
        width,
        color=BLUE,
        alpha=0.88,
        label="逐历元协方差",
    )
    fixed_bars = error_ax.bar(
        x + width / 2.0,
        fixed_errors,
        width,
        color=RED,
        alpha=0.82,
        label="固定协方差",
    )
    outage_indices = np.flatnonzero(outage_checkpoints)
    outage_span = error_ax.axvspan(
        outage_indices[0] - 0.5,
        outage_indices[-1] + 0.5,
        color=RED,
        alpha=0.07,
        linewidth=0,
        label="RTK 固定解中断内检查点",
    )
    error_ax.set_xticks(x)
    error_ax.set_xticklabels([row["id"] for row in gt_rows], fontsize=8.0)
    error_ax.set_ylabel("三维位置误差 / m")
    error_ax.set_title("（b）8 个测量检查点误差（官方 ±2 s 最近匹配）")
    error_ax.grid(axis="y", color=GRID, linestyle=":", linewidth=0.7)
    error_ax.legend(
        [dynamic_bars, fixed_bars, outage_span],
        ["逐历元协方差", "固定协方差", "RTK 固定解中断内检查点"],
        loc="upper left",
        ncol=3,
        fontsize=8.0,
        frameon=True,
    )

    plot_segmented_series(
        sigma_ax,
        sigma_time,
        sigma_dynamic,
        1.0,
        color=BLUE,
        linewidth=1.35,
        label="逐历元水平 σh",
    )
    plot_segmented_series(
        sigma_ax,
        sigma_time,
        sigma_fixed,
        1.0,
        color=RED,
        linewidth=1.45,
        linestyle="--",
        label="固定水平 σh={:.4f} m".format(float(sigma_fixed[0])),
    )
    for left, right in gaps:
        sigma_ax.axvspan(left, right, color=RED, alpha=0.10, linewidth=0)
    sigma_ax.set_xlim(0.0, WINDOW_SECONDS)
    sigma_ax.set_xlabel("距本次评测窗口起点时间 / s")
    sigma_ax.set_ylabel("水平标准差 σh / m")
    sigma_ax.set_title("（c）实际 Fixed 历元的水平观测标准差")
    sigma_ax.grid(True, color=GRID, linestyle=":", linewidth=0.7)
    sigma_ax.legend(loc="upper right", ncol=2, fontsize=8.2, frameon=True)

    if dynamic_rmse < fixed_rmse:
        conclusion = (
            "前 360 s 的 8 个检查点上，逐历元协方差组 RMSE 较低："
            "{:.3f} m 对 {:.3f} m；中断内 4 点为 {:.3f} m 对 {:.3f} m。".format(
                dynamic_rmse, fixed_rmse, dynamic_outage_rmse, fixed_outage_rmse
            )
        )
    elif fixed_rmse < dynamic_rmse:
        conclusion = (
            "前 360 s 的 8 个检查点上，固定协方差组 RMSE 较低："
            "{:.3f} m 对 {:.3f} m；中断内 4 点为 {:.3f} m 对 {:.3f} m。".format(
                fixed_rmse, dynamic_rmse, fixed_outage_rmse, dynamic_outage_rmse
            )
        )
    else:
        conclusion = (
            "前 360 s 的 8 个检查点上，两组 RMSE 数值相同；"
            "中断内 4 点为 {:.3f} m 对 {:.3f} m。".format(
                dynamic_outage_rmse, fixed_outage_rmse
            )
        )
    conclusion += " 结论仅限该同输入配对消融。"
    fig.suptitle(
        "固定协方差与逐历元协方差的在线定位配对消融（城市花园序列 2，前 360 s）",
        fontsize=13.0,
        fontweight="bold",
        y=0.985,
    )
    fig.text(0.5, 0.012, conclusion, ha="center", fontsize=9.0)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.88, bottom=0.17)
    figure3_path = output_dir / OUTPUT_NAMES[2]
    atomic_save_png(fig, figure3_path)

    generated = [figure1_path, figure2_path, figure3_path]
    if len(generated) != 3 or any(not path.is_file() for path in generated):
        raise RuntimeError("必须且只能完成三张 PNG")

    metrics = {
        "schema": "rtk-slam-ppt-ablation-v1",
        "sequence": "stadtgarten_seq2",
        "window": {
            "start_unix_s": start,
            "duration_s": WINDOW_SECONDS,
            "fixed_outage_start_s": longest_gap[0],
            "fixed_outage_end_s": longest_gap[1],
            "fixed_outage_duration_s": outage_seconds,
        },
        "semantics": {
            "online": "causal at emission",
            "final": "non-causal 20 s fixed-lag result",
            "ground_truth": "sparse measured checkpoints; plotted as unconnected black stars",
            "gnss_constraints": "accepted graph factors, antenna positions converted to base center using same-epoch causal orientation",
            "sigma_h": "sqrt(sigma_e^2 + sigma_n^2) after tangent-frame covariance rotation and configured per-axis clamps",
        },
        "sources": {
            "dynamic_run": str(dynamic_run),
            "fixed_run": str(fixed_run),
            "bag": str(bag_path),
            "bag_sha256": sha256(bag_path),
            "ground_truth": str(gt_path),
            "ground_truth_sha256": sha256(gt_path),
            "official_eval_dir": str(eval_dir),
            "official_coords_sha256": sha256(eval_dir / "coords.py"),
            "official_readers_sha256": sha256(eval_dir / "readers.py"),
            "font_requested": "Noto Sans CJK SC",
            "font_file": str(font_path),
            "dynamic_online": str(dynamic_online_path),
            "dynamic_online_sha256": sha256(dynamic_online_path),
            "dynamic_final": str(dynamic_final_path),
            "dynamic_final_sha256": sha256(dynamic_final_path),
            "fixed_online": str(fixed_online_path),
            "fixed_online_sha256": sha256(fixed_online_path),
            "gnss_constraints": str(gnss_path),
            "gnss_constraints_sha256": sha256(gnss_path),
        },
        "config_audit": {
            "changed_keys": config_audit["changed_keys"],
            "dynamic_values": {
                key: config_audit["dynamic"][key] for key in CONFIG_KEYS
            },
            "fixed_values": {
                key: config_audit["fixed"][key] for key in CONFIG_KEYS
            },
            "dynamic_config_sha256": sha256(config_audit["dynamic_path"]),
            "fixed_config_sha256": sha256(config_audit["fixed_path"]),
        },
        "same_input_audit": {
            "play_start_s": float(dynamic_window["play_start_s"]),
            "play_duration_s": float(dynamic_window["play_duration_s"]),
            "sensor_start_unix": float(dynamic_window["sensor_start_unix"]),
            "sensor_end_unix": float(dynamic_window["sensor_end_unix"]),
            "native_files_byte_identical": native_input_audit,
        },
        "counts": {
            "online_keyframes_first_360_s": int(len(dynamic_online)),
            "final_keyframes_first_360_s": int(len(dynamic_final)),
            "fixed_ablation_online_keyframes_first_360_s": int(len(fixed_online)),
            "online_keyframes_during_longest_fixed_outage": outage_online_count,
            "accepted_gnss_graph_factors_first_360_s": int(len(gnss_timestamps)),
            "valid_fixed_epochs_first_360_s": int(valid_sigma.sum()),
            "ground_truth_checkpoints_first_360_s": int(len(gt_rows)),
        },
        "continuity": {
            "first_online_output_s": float(elapsed_online[0]),
            "last_online_output_s": float(elapsed_online[-1]),
            "max_online_keyframe_gap_s": max_online_gap,
        },
        "fixed_lag_difference": {
            "rms_3d_m": fixed_lag_rms,
            "max_3d_m": fixed_lag_max,
            "max_at_elapsed_s": fixed_lag_max_time,
        },
        "ablation": {
            "checkpoint_ids": [row["id"] for row in gt_rows],
            "dynamic_errors_3d_m": dynamic_errors.tolist(),
            "fixed_errors_3d_m": fixed_errors.tolist(),
            "dynamic_rmse_3d_m": dynamic_rmse,
            "fixed_rmse_3d_m": fixed_rmse,
            "rmse_reduction_dynamic_vs_fixed_percent": float(
                100.0 * (fixed_rmse - dynamic_rmse) / fixed_rmse
            ),
            "outage_checkpoint_ids": [
                row["id"] for row, selected in zip(gt_rows, outage_checkpoints) if selected
            ],
            "dynamic_outage_rmse_3d_m": dynamic_outage_rmse,
            "fixed_outage_rmse_3d_m": fixed_outage_rmse,
            "outage_rmse_reduction_dynamic_vs_fixed_percent": float(
                100.0
                * (fixed_outage_rmse - dynamic_outage_rmse)
                / fixed_outage_rmse
            ),
            "dynamic_max_match_dt_s": float(dynamic_time_residual.max()),
            "fixed_max_match_dt_s": float(fixed_time_residual.max()),
            "dynamic_sigma_h_min_m": float(sigma_dynamic.min()),
            "dynamic_sigma_h_max_m": float(sigma_dynamic.max()),
            "fixed_sigma_h_m": float(sigma_fixed[0]),
            "conclusion_cn": conclusion,
        },
        "gnss_lever_arm_association_max_dt_s": float(gnss_orientation_dt.max()),
        "outputs": [
            {"path": str(path), "sha256": sha256(path)} for path in generated
        ],
    }
    write_json_atomic(metrics_out, metrics)
    print("PASS: 已生成且仅生成三张正式 PNG")
    for path in generated:
        print(path)
    print("指标: {}".format(metrics_out))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, ValueError, RuntimeError, AssertionError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        sys.exit(2)
