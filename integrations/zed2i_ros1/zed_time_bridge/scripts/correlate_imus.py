#!/usr/bin/env python3
"""Offline effective IMU lag audit; never an image exposure calibration.

CSV contract (one already selected continuous mapping session only):
  both IMUs: raw_stamp_ns, session_id, units, angular_velocity_x/y/z
  mapping: raw_stamp_ns, session_id, stream (imu or imu_raw), valid,
           mapped_stamp_ns; these are recorded values, never a live anchor.
MID raw_stamp_ns is in its device domain. Both CSV session_id fields identify
the common recording/mapping segment, not separate sensor serial numbers.
Absolute timestamps stay integer; only differences become floating seconds.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

NS = 1_000_000_000
I64_MAX = (1 << 63) - 1


def integer_ns(value):
    text = str(value)
    if not text.isdigit():
        raise ValueError("INVALID_INTEGER_NS: " + text)
    result = int(text)
    if not 0 < result <= I64_MAX:
        raise ValueError("NS_OUT_OF_RANGE")
    return result


def rows(path):
    with open(path, newline="") as handle:
        result = list(csv.DictReader(handle))
    if not result:
        raise ValueError("EMPTY_CSV: " + str(path))
    return result


def load_inputs(zed_path, mid_path, mapping_path):
    mapping = {}
    for row in rows(mapping_path):
        if row["stream"] not in ("imu", "imu_raw"):
            continue
        key = (row["session_id"], integer_ns(row["raw_stamp_ns"]))
        if row["valid"].lower() not in ("true", "1"):
            raise ValueError("INVALID_MAPPING: " + row.get("reason", ""))
        value = integer_ns(row["mapped_stamp_ns"])
        if key in mapping and mapping[key] != value:
            raise ValueError("CONFLICTING_MAPPING")
        mapping[key] = value
    sensors = []
    sessions = set()
    for path, zed in ((zed_path, True), (mid_path, False)):
        stamps, values, raw_stamps = [], [], []
        for row in rows(path):
            raw = integer_ns(row["raw_stamp_ns"])
            session = row["session_id"]
            if not session:
                raise ValueError("MISSING_SESSION")
            sessions.add(session)
            unit = row["units"]
            if unit not in ("rad/s", "deg/s"):
                raise ValueError("UNKNOWN_GYRO_UNITS")
            gyro = [float(row["angular_velocity_" + axis]) for axis in "xyz"]
            if not all(math.isfinite(value) for value in gyro):
                raise ValueError("NONFINITE_GYRO")
            if unit == "deg/s":
                gyro = [math.radians(value) for value in gyro]
            try:
                stamp = mapping[(session, raw)] if zed else raw
            except KeyError:
                raise ValueError("MISSING_MAPPING")
            raw_stamps.append(raw)
            stamps.append(stamp)
            values.append(gyro)
        if any(b <= a for a, b in zip(raw_stamps, raw_stamps[1:])):
            raise ValueError("RAW_NOT_STRICTLY_INCREASING")
        sensors.append((stamps, np.asarray(values, dtype=float)))
    if len(sessions) != 1:
        raise ValueError("CROSS_SESSION_INPUT")
    if set(key[0] for key in mapping) != sessions:
        raise ValueError("CROSS_SESSION_MAPPING")
    return sensors, sessions.pop()


def correlation(x, y):
    a, b = x - np.mean(x), y - np.mean(y)
    scale = float(np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.dot(a, b) / scale) if scale > 1e-15 else -1.0


def fir_norm(times, gyro, bias, cutoff_hz):
    """Uniformize at native median rate, then centered FIR before decimation.

    ponytail: interpolation assumes band-limited input; gaps/jitter are gated
    first. This cannot undo aliasing already present in a sensor's acquisition.
    """
    dt = float(np.median(np.diff(times)))
    native = np.arange(times[0], times[-1], dt)
    norm = np.linalg.norm(gyro - bias, axis=1)
    norm = np.interp(native, times, norm)
    half = int(math.ceil(4.0 / (cutoff_hz * dt)))
    tap_index = np.arange(-half, half + 1)
    taps = 2 * cutoff_hz * dt * np.sinc(2 * cutoff_hz * dt * tap_index)
    taps *= np.hamming(len(taps))
    taps /= np.sum(taps)
    if len(norm) <= len(taps):
        raise ValueError("RECORD_TOO_SHORT_FOR_FILTER")
    return native[half:-half], np.convolve(norm, taps, mode="valid")


def peak_lag(x, y, indices, max_lag, dt):
    lags = np.arange(-max_lag, max_lag + 1)
    scores = np.asarray([correlation(x[indices], y[indices + lag]) for lag in lags])
    k = int(np.argmax(scores))
    if k in (0, len(scores) - 1):
        return {"valid": False, "reason": "PEAK_AT_SEARCH_BOUNDARY"}
    # Interpolated lag is a point estimate, not sub-grid physical accuracy.
    denominator = scores[k - 1] - 2 * scores[k] + scores[k + 1]
    fraction = (0.5 * (scores[k - 1] - scores[k + 1]) / denominator
                if denominator < -1e-12 else 0.0)
    competitors = [float(scores[i]) for i in range(1, len(scores) - 1)
                   if abs(i - k) * dt >= max(0.05, 5 * dt)
                   and scores[i] >= scores[i - 1] and scores[i] >= scores[i + 1]]
    competitor = max(competitors) if competitors else None
    score = float(scores[k])
    valid = score >= 0.8 and (competitor is None or score - competitor >= 0.1)
    return {"valid": valid, "reason": "OK" if valid else "LOW_OR_AMBIGUOUS_PEAK",
            "lag_s": (float(lags[k]) + float(fraction)) * dt,
            "peak": score, "competing_peak": competitor,
            "peak_margin": None if competitor is None else score - competitor}


def periodicity(x, dt):
    # A second strong autocorrelation maximum makes a unique timing claim unsafe.
    low = max(1, int(0.2 / dt))
    high = min(len(x) // 2, int(5.0 / dt))
    if high <= low + 2:
        return 0.0
    ac = [correlation(x[:-lag], x[lag:]) for lag in range(low, high + 1)]
    peaks = [ac[k] for k in range(1, len(ac) - 1)
             if ac[k] >= ac[k - 1] and ac[k] >= ac[k + 1]]
    return max(peaks) if peaks else 0.0


def affine_fit(t, lag):
    """Centered robust line, with differences in seconds (never epoch floats)."""
    center = float(np.median(t))
    design = np.column_stack((np.ones(len(t)), np.asarray(t) - center))
    weights = np.ones(len(t))
    for _ in range(6):
        fit = np.linalg.lstsq(design * np.sqrt(weights[:, None]),
                              np.asarray(lag) * np.sqrt(weights), rcond=None)[0]
        residual = np.asarray(lag) - design @ fit
        scale = max(1e-9, 1.4826 * float(np.median(np.abs(residual))))
        weights = np.minimum(1.0, 1.5 * scale / np.maximum(np.abs(residual), 1e-12))
    return center, float(fit[0]), float(fit[1])


def analyze(sensors, bias_start_ns, bias_end_ns, window_s=12.0,
            max_lag_s=0.5, grid_hz=None, bootstrap=300, seed=17):
    if (not math.isfinite(window_s) or not math.isfinite(max_lag_s)
            or window_s < 4 or not 0 < max_lag_s < window_s / 4
            or bootstrap < 50 or bootstrap > 10000):
        raise ValueError("INVALID_ANALYSIS_PARAMETERS")
    if bias_end_ns - bias_start_ns < NS // 2:
        raise ValueError("BIAS_INTERVAL_TOO_SHORT")
    if len(sensors) != 2 or any(len(sensor[0]) < 100 for sensor in sensors):
        raise ValueError("INSUFFICIENT_IMU_SAMPLES")
    origin = min(int(sensor[0][0]) for sensor in sensors)
    arrays, rates, diagnostics = [], [], []
    for stamps, gyro in sensors:
        if len(stamps) < 100 or gyro.shape != (len(stamps), 3):
            raise ValueError("INSUFFICIENT_IMU_SAMPLES")
        if not np.all(np.isfinite(gyro)):
            raise ValueError("NONFINITE_GYRO")
        if any(not isinstance(v, (int, np.integer)) or not 0 < int(v) <= I64_MAX
               for v in stamps):
            raise ValueError("INVALID_INTEGER_TIMESTAMP")
        differences = np.asarray([int(b) - int(a) for a, b in zip(stamps, stamps[1:])])
        if np.any(differences <= 0):
            raise ValueError("MAPPED_NOT_STRICTLY_INCREASING")
        median_ns = float(np.median(differences))
        if np.max(differences) > min(50_000_000, 3 * median_ns):
            raise ValueError("LARGE_GAP")
        jitter = float(np.percentile(np.abs(differences - median_ns), 99) / median_ns)
        if jitter > 0.2:
            raise ValueError("EXCESSIVE_SAMPLING_JITTER")
        times = np.asarray([(int(stamp) - origin) / NS for stamp in stamps])
        mask = np.asarray([bias_start_ns <= int(stamp) <= bias_end_ns for stamp in stamps])
        if np.sum(mask) < 20:
            raise ValueError("BIAS_INTERVAL_NO_COVERAGE")
        bias = np.median(gyro[mask], axis=0)
        stationary_noise = float(np.sqrt(np.mean(np.sum((gyro[mask] - bias) ** 2, axis=1))))
        if stationary_noise > 0.03 or np.linalg.norm(bias) > 0.2:
            raise ValueError("DECLARED_BIAS_INTERVAL_NOT_STATIONARY")
        arrays.append((times, gyro, bias))
        rates.append(NS / median_ns)
        diagnostics.append({"samples": len(stamps), "rate_hz": rates[-1],
                            "sampling_jitter_p99_fraction": jitter,
                            "bias_rad_s": bias.tolist(), "stationary_rms_rad_s": stationary_noise})
    rate = min(rates) if grid_hz is None else float(grid_hz)
    if not math.isfinite(rate) or rate < 20 or rate > min(rates) * 1.000001:
        raise ValueError("INVALID_GRID_RATE")
    dt, cutoff = 1.0 / rate, 0.35 * rate
    filtered = [fir_norm(t, gyro, bias, cutoff) for t, gyro, bias in arrays]
    start, end = max(v[0][0] for v in filtered), min(v[0][-1] for v in filtered)
    if (end - start) / dt > 10_000_000:
        raise ValueError("GRID_TOO_LARGE")
    grid = np.arange(start, end, dt)
    zed, mid = [np.interp(grid, t, values) for t, values in filtered]
    max_lag = int(math.ceil(max_lag_s / dt))
    begin = max(max_lag, int(np.searchsorted(grid, (bias_end_ns - origin) / NS + 0.25)))
    count = int(round(window_s / dt))
    rng = np.random.default_rng(seed)
    windows = []
    # Guard also separates shifted MID samples and the centered FIR support.
    guard = 2 * max_lag + int(math.ceil(10 * rate / cutoff))
    for first in range(begin, len(grid) - max_lag - count + 1, count + guard):
        indices = np.arange(first, first + count)
        window = {"center_delta_s": float(np.mean(grid[indices])),
                  "start_ns": origin + int(round(grid[first] * NS)),
                  "end_ns": origin + int(round(grid[first + count - 1] * NS)),
                  "samples": count, "excitation_std_rad_s": float(np.std(zed[indices]))}
        if min(np.std(zed[indices]), np.std(mid[indices])) < 0.02:
            window.update(valid=False, reason="INSUFFICIENT_EXCITATION")
        else:
            auto_peak = max(periodicity(zed[indices], dt), periodicity(mid[indices], dt))
            window["periodic_autocorrelation_peak"] = auto_peak
            if auto_peak >= 0.9:
                window.update(valid=False, reason="PERIODIC_EXCITATION")
            else:
                window.update(peak_lag(zed, mid, indices, max_lag, dt))
        if window["valid"]:
            block_samples = count // 4
            decorrelation = []
            for values in (zed[indices], mid[indices]):
                crossing = next((lag * dt for lag in range(1, block_samples // 2, 4)
                                 if abs(correlation(values[:-lag], values[lag:])) < 0.2), None)
                decorrelation.append(crossing)
            window["decorrelation_time_s"] = decorrelation
            if any(value is None for value in decorrelation):
                window.update(valid=False, reason="LONG_CORRELATION_REQUIRES_LONGER_WINDOW")
        if window["valid"]:
            sub_lags = []
            for block in np.array_split(indices, 4):
                peak = peak_lag(zed, mid, block, max_lag, dt)
                if peak["valid"] and np.std(zed[block]) >= 0.02:
                    sub_lags.append(peak["lag_s"])
            window["independent_subwindow_lags_s"] = sub_lags
            if len(sub_lags) < 3:
                window.update(valid=False, reason="INSUFFICIENT_INDEPENDENT_SUBWINDOWS")
            else:
                estimates = np.median(rng.choice(sub_lags, (bootstrap, len(sub_lags)), replace=True), axis=1)
                lo, hi = np.percentile(estimates, [2.5, 97.5])
                half = max(dt / 2, abs(window["lag_s"] - lo), abs(hi - window["lag_s"]))
                window["lag_ci95_s"] = [window["lag_s"] - half, window["lag_s"] + half]
                window["ci_method"] = "independent_subwindow_cluster_bootstrap_with_half_grid_floor"
        windows.append(window)
    result = {"status": "HARDWARE_PENDING", "analysis_status": "INCONCLUSIVE",
              "reference": "effective_relative_gyro_time_not_image_exposure",
              "sign": "corr(Z(t), MID(t+delta)); add delta to ZED mapped stamp; C_ns=-delta_ns",
              "origin_ns": origin, "grid_hz": rate, "filter_cutoff_hz": cutoff,
              "bias_interval_ns": [bias_start_ns, bias_end_ns],
              "window_s": window_s, "window_guard_s": guard * dt,
              "max_lag_s": max_lag_s, "bootstrap_samples": bootstrap, "bootstrap_seed": seed,
              "filter": "centered_symmetric_FIR_before_common_grid_interpolation",
              "sensors": diagnostics, "windows": windows,
              "limitations": ["CI is conditional on locally independent motion blocks, not a physical timing bound",
                              "Unmodeled IMU filtering and scale factors can bias the effective lag",
                              "Explicit stationary interval is an operator assertion; constant slow rotation is not distinguishable",
                              "Interpolation and FIR cannot recover content aliased during acquisition",
                              "No exposure correction or mapping update is applied"]}
    accepted = [window for window in windows if window["valid"]]
    if len(accepted) < 6:
        result["reason"] = "NEED_AT_LEAST_SIX_VALID_NONOVERLAP_WINDOWS"
        return result
    train = [window for i, window in enumerate(accepted) if i % 3 != 2]
    holdout = [window for i, window in enumerate(accepted) if i % 3 == 2]
    if len(train) < 3 or len(holdout) < 2:
        result["reason"] = "INSUFFICIENT_HELD_OUT_WINDOWS"
        return result
    t = np.asarray([w["center_delta_s"] for w in train])
    lag = np.asarray([w["lag_s"] for w in train])
    center, offset, slope = affine_fit(t, lag)
    samples = []
    for _ in range(bootstrap):
        selection = rng.integers(0, len(train), len(train))
        if len(np.unique(selection)) < 3:
            continue
        c, o, s = affine_fit(t[selection], lag[selection])
        samples.append((o + s * (center - c), s))
    if len(samples) < bootstrap // 2:
        result["reason"] = "INSUFFICIENT_DRIFT_BOOTSTRAP_SUPPORT"
        return result
    samples = np.asarray(samples)
    # Grid resolution floor prevents noiseless bootstrap from implying arbitrary precision.
    offset_bounds = np.percentile(samples[:, 0], [2.5, 97.5])
    slope_bounds = np.percentile(samples[:, 1], [2.5, 97.5])
    offset_half = max(dt / 2, float(np.max(np.abs(offset_bounds - offset))))
    slope_half = max(dt / max(float(np.ptp(t)), dt), float(np.max(np.abs(slope_bounds - slope))))
    residuals = []
    for window in holdout:
        residual = window["lag_s"] - (offset + slope * (window["center_delta_s"] - center))
        residuals.append(abs(residual))
        window["role"] = "holdout"
        window["holdout_residual_s"] = residual
    for window in train:
        window["role"] = "training"
    p95 = float(np.percentile(residuals, 95))
    max_residual = float(max(residuals))
    result.update(analysis_status="SOFTWARE_ESTIMATE", reason="ESTIMATE_ONLY_NOT_HARDWARE_ACCEPTANCE",
                  reference_time_ns=origin + int(round(center * NS)), offset_s=offset,
                  offset_ci95_s=[offset - offset_half, offset + offset_half],
                  drift_ppm=slope * 1e6,
                  drift_ci95_ppm=[(slope - slope_half) * 1e6, (slope + slope_half) * 1e6],
                  training_windows=len(train), holdout_windows=len(holdout),
                  holdout_absolute_residual_p95_s=p95, holdout_absolute_residual_max_s=max_residual,
                  provisional_holdout_gate=p95 <= 0.002 and max_residual <= 0.005)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for name in ("zed-csv", "mid-csv", "mapping-csv", "output"):
        parser.add_argument("--" + name)
    parser.add_argument("--bias-start-ns", type=integer_ns)
    parser.add_argument("--bias-end-ns", type=integer_ns)
    parser.add_argument("--window-s", type=float, default=12)
    parser.add_argument("--max-lag-s", type=float, default=0.5)
    parser.add_argument("--grid-hz", type=float)
    parser.add_argument("--bootstrap", type=int, default=300)
    parser.add_argument("--provenance", choices=("synthetic", "recorded"), default="recorded")
    args = parser.parse_args(argv)
    paths = [args.zed_csv, args.mid_csv, args.mapping_csv]
    code = 0
    if not any(paths):
        result = {"status": "HARDWARE_PENDING", "reason": "NO_PAIRED_MID360_DATA",
                  "analysis_status": "NOT_RUN"}
    elif not all(paths) or args.bias_start_ns is None or args.bias_end_ns is None:
        result = {"status": "HARDWARE_PENDING", "analysis_status": "FAILED",
                  "reason": "ALL_THREE_INPUTS_AND_EXPLICIT_BIAS_INTERVAL_REQUIRED"}
        code = 2
    else:
        try:
            sensors, session = load_inputs(*paths)
            result = analyze(sensors, args.bias_start_ns, args.bias_end_ns,
                             args.window_s, args.max_lag_s, args.grid_hz, args.bootstrap)
            result.update(session_id=session, provenance=args.provenance,
                          input_sha256={str(path): hashlib.sha256(Path(path).read_bytes()).hexdigest()
                                        for path in paths})
        except (ValueError, KeyError, OSError, OverflowError) as error:
            result = {"status": "HARDWARE_PENDING", "analysis_status": "FAILED", "reason": str(error)}
            code = 2
    payload = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        # Preserve previous evidence rather than silently overwriting it.
        with open(args.output, "x") as handle:
            handle.write(payload)
    else:
        print(payload, end="")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
