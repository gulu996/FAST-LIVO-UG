#!/usr/bin/env python3
"""Standalone software checks. No ROS, devices, live clocks or shared files."""
import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "correlate_imus.py"
SPEC = importlib.util.spec_from_file_location("correlate_imus", SCRIPT)
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)
BASE = 1_590_192_000_000_000_000
RAW_BASE = 1_792_000_000_000_000_000


def synthetic(lag=0.025, drift=0.0, duration=126.0, periodic=False, static=False):
    rng = np.random.default_rng(19)
    centers = np.arange(4, duration + 2, 0.7)
    centers += rng.uniform(-0.3, 0.3, len(centers))
    widths = rng.uniform(0.1, 0.35, len(centers))
    heights = rng.uniform(0.2, 1.0, len(centers))

    def signal(t):
        amplitude = np.zeros(len(t))
        if not static:
            if periodic:
                amplitude = np.where(t > 3, 0.5 + 0.3 * np.sin(2 * np.pi * t), 0)
            else:
                for c, w, h in zip(centers, widths, heights):
                    amplitude += h * np.exp(-((t - c) / w) ** 2)
        return np.column_stack((amplitude, amplitude * 0.2 * np.sin(0.17 * t),
                                amplitude * 0.1 * np.cos(0.23 * t)))
    sensors = []
    for rate, zed in ((360, True), (200, False)):
        t = np.arange(int(duration * rate)) / rate
        true_t = t + lag + drift * (t - 60.0) if zed else t
        gyro = signal(true_t) + np.array([0.01, -0.005, 0.002])
        stamps = [BASE + int(round(value * tool.NS)) for value in t]
        sensors.append((stamps, gyro))
    return sensors


class CorrelationChecks(unittest.TestCase):
    def run_analysis(self, sensors):
        return tool.analyze(sensors, BASE + 500_000_000, BASE + 1_500_000_000,
                            bootstrap=100)

    def test_known_positive_lag_and_units(self):
        result = self.run_analysis(synthetic(lag=0.025))
        self.assertEqual(result["analysis_status"], "SOFTWARE_ESTIMATE", result)
        self.assertAlmostEqual(result["offset_s"], 0.025, delta=0.001)
        self.assertEqual(result["status"], "HARDWARE_PENDING")
        self.assertTrue(result["provisional_holdout_gate"])
        self.assertGreaterEqual(result["offset_ci95_s"][1] - result["offset_ci95_s"][0], 0.00499)

    def test_negative_lag(self):
        result = self.run_analysis(synthetic(lag=-0.035))
        self.assertAlmostEqual(result["offset_s"], -0.035, delta=0.001)

    def test_known_drift(self):
        for ppm in (500, -350):
            result = self.run_analysis(synthetic(lag=0.02, drift=ppm * 1e-6))
            self.assertAlmostEqual(result["drift_ppm"], ppm, delta=40)
            reference = (result["reference_time_ns"] - BASE) / tool.NS
            self.assertAlmostEqual(result["offset_s"], 0.02 + ppm * 1e-6 * (reference - 60), delta=0.0015)
            self.assertLess(result["holdout_absolute_residual_max_s"], 0.002)

    def test_full_csv_cli_synthetic_roundtrip(self):
        sensors = synthetic()
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            paths = [directory / name for name in ("z.csv", "m.csv", "mapping.csv")]
            fields = ["raw_stamp_ns", "session_id", "units"] + ["angular_velocity_" + a for a in "xyz"]
            with paths[2].open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["raw_stamp_ns", "session_id", "stream", "valid", "mapped_stamp_ns"])
                for stamp in sensors[0][0]:
                    writer.writerow([RAW_BASE + stamp - BASE, "s1", "imu_raw", "true", stamp])
            for index, (stamps, gyro) in enumerate(sensors):
                with paths[index].open("w", newline="") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(fields)
                    for stamp, values in zip(stamps, gyro):
                        raw = RAW_BASE + stamp - BASE if index == 0 else stamp
                        writer.writerow([raw, "s1", "rad/s"] + list(values))
            output = directory / "result.json"
            args = ["--zed-csv", str(paths[0]), "--mid-csv", str(paths[1]),
                    "--mapping-csv", str(paths[2]), "--bias-start-ns", str(BASE + 500_000_000),
                    "--bias-end-ns", str(BASE + 1_500_000_000), "--provenance", "synthetic",
                    "--bootstrap", "100", "--output", str(output)]
            self.assertEqual(tool.main(args), 0)
            result = json.loads(output.read_text())
            self.assertEqual(result["status"], "HARDWARE_PENDING")
            self.assertEqual(result["provenance"], "synthetic")
            self.assertAlmostEqual(result["offset_s"], 0.025, delta=0.001)
            self.assertEqual(len(result["input_sha256"]), 3)
            with self.assertRaises(FileExistsError):
                tool.main(["--output", str(output)])

    def test_static_and_periodic_rejected(self):
        for kwargs, reason in (({"static": True}, "INSUFFICIENT_EXCITATION"),
                               ({"periodic": True}, "PERIODIC_EXCITATION")):
            result = self.run_analysis(synthetic(duration=30, **kwargs))
            self.assertEqual(result["analysis_status"], "INCONCLUSIVE")
            self.assertTrue(any(w["reason"] == reason for w in result["windows"]), result)

    def test_gap_and_reversal_rejected(self):
        sensors = synthetic(duration=30)
        stamps, gyro = sensors[0]
        sensors[0] = (stamps[:1000] + stamps[1020:], np.vstack((gyro[:1000], gyro[1020:])))
        with self.assertRaisesRegex(ValueError, "LARGE_GAP"):
            self.run_analysis(sensors)
        sensors = synthetic(duration=30)
        sensors[0][0][100] = sensors[0][0][99]
        with self.assertRaisesRegex(ValueError, "NOT_STRICTLY_INCREASING"):
            self.run_analysis(sensors)

    def test_csv_validation_and_unit_conversion(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            zed, mid, mapping = [directory / name for name in ("z.csv", "m.csv", "mapping.csv")]
            imu_fields = ["raw_stamp_ns", "session_id", "units"] + ["angular_velocity_" + a for a in "xyz"]
            record = dict(zip(imu_fields, [RAW_BASE, "s1", "deg/s", 180, 0, 0]))

            def write(path, fields, records):
                with path.open("w", newline="") as handle:
                    writer = csv.DictWriter(handle, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(records)

            write(zed, imu_fields, [record])
            record["raw_stamp_ns"] = BASE
            write(mid, imu_fields, [record])
            fields = ["raw_stamp_ns", "session_id", "stream", "valid", "mapped_stamp_ns"]
            mapped = dict(zip(fields, [RAW_BASE, "s1", "imu_raw", "True", BASE]))
            write(mapping, fields, [mapped, mapped])
            sensors, session = tool.load_inputs(zed, mid, mapping)
            self.assertEqual(session, "s1")
            self.assertEqual(sensors[0][0][0], BASE)
            self.assertAlmostEqual(sensors[0][1][0, 0], np.pi)
            mapped["valid"] = "false"
            write(mapping, fields, [mapped])
            with self.assertRaisesRegex(ValueError, "INVALID_MAPPING"):
                tool.load_inputs(zed, mid, mapping)
            mapped["valid"] = "true"
            write(mapping, fields, [mapped])
            record["session_id"] = "s2"
            write(mid, imu_fields, [record])
            with self.assertRaisesRegex(ValueError, "CROSS_SESSION"):
                tool.load_inputs(zed, mid, mapping)

    def test_ns_validation_and_no_input_pending(self):
        self.assertEqual(tool.integer_ns(str(RAW_BASE + 1)), RAW_BASE + 1)
        for value in ("1.5e18", "100.0", "-1", "0", str(1 << 63)):
            with self.assertRaises(ValueError):
                tool.integer_ns(value)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(tool.main([]), 0)
        self.assertEqual(json.loads(output.getvalue())["status"], "HARDWARE_PENDING")


if __name__ == "__main__":
    unittest.main(verbosity=2)
