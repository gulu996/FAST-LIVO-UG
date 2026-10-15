#!/usr/bin/env python3
"""Run with unittest discover; all faults are synthetic, no system clock edits."""
import json
import unittest
from dataclasses import FrozenInstanceError, replace
from fractions import Fraction

from sensor_time_bridge.imu_anchor import ImuAnchorSnapshot, IMU_TIME_ANCHOR_MAGIC, IMU_TIME_ANCHOR_VERSION
from zed_time_bridge.time_mapping import Config, ShadowMapper, INT64_MAX, fit_affine, round_fraction

R = 1_800_000_000_000_000_000
D = 1_590_192_000_000_000_000
M = 9_000_000_000
STEP = 10_000_000


def anchor(m, d, sequence, epoch=17, **changes):
    return replace(ImuAnchorSnapshot(IMU_TIME_ANCHOR_MAGIC, IMU_TIME_ANCHOR_VERSION,
                                    epoch, sequence, d, m, m + 100, 2, True, 10_000_000), **changes)


def prepare(config=None, r_slope=Fraction(1), d_slope=Fraction(1), count=8):
    mapper = ShadowMapper(config)
    for i in range(count):
        m = M + i * STEP
        mapper.observe_clock(m - 100, R + round_fraction(r_slope * (m - M)), m + 100, "boot-a", m + 1_000_000)
        mapper.observe_anchor(anchor(m, D + round_fraction(d_slope * (m - M)), 2 * i + 2), m + 100, (4, 5))
    return mapper


def event(mapper, delta=71_000_000, stream="left", raw=None, now_delta=72_000_000, session="camera-a", **kwargs):
    return mapper.map_event(stream, R + delta if raw is None else raw,
                            R + now_delta, M + now_delta, session, **kwargs)


class TimeMappingTests(unittest.TestCase):
    def test_fixed_offset_and_signed_compensation(self):
        for compensation in (0, 123_456_789, -123_456_789):
            result = event(prepare(Config(compensation_ns=compensation)))
            self.assertTrue(result.valid, result.reason)
            self.assertEqual(result.mapped_stamp_ns, D + 71_000_000 - compensation)
            self.assertEqual(result.raw_stamp_ns, R + 71_000_000)
            self.assertEqual(result.physical_error_bound, "UNKNOWN")
            json.dumps(result.as_dict())

    def test_known_positive_and_negative_drifts_integer_reference(self):
        maximum = 0
        for r_slope, d_slope in ((Fraction(10001, 10000), Fraction(9999, 10000)),
                                  (Fraction(9998, 10000), Fraction(10003, 10000))):
            mapper = prepare(r_slope=r_slope, d_slope=d_slope)
            for target in (21_000_000, 25_000_000, 39_000_000, 71_000_000):
                raw = R + round_fraction(r_slope * target)
                result = event(mapper, raw=raw, stream="imu_raw")
                expected = D + round_fraction(d_slope * target)
                self.assertTrue(result.valid, result.reason)
                error = abs(result.mapped_stamp_ns - expected)
                maximum = max(maximum, error)
                self.assertLessEqual(error, 1)
        self.assertEqual(maximum, 0)

    def test_near_int64_maximum_and_negative_anchor_delta(self):
        mapper = ShadowMapper()
        r0, d0 = INT64_MAX - 1_000_000_000, INT64_MAX - 2_000_000_000
        for i in range(8):
            offset = i * STEP
            mapper.observe_clock(M + offset, r0 + offset, M + offset, "near-limit")
            mapper.observe_anchor(anchor(M + offset, d0 + offset, 2 * i + 2), M + offset + 100, (1, 2))
        result = mapper.map_event("left", r0 + 29_000_000, r0 + 71_000_000, M + 71_000_000)
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(result.anchor_delta_ns, -1_000_000)
        self.assertEqual(result.mapped_stamp_ns, d0 + 29_000_000)

    def test_stereo_and_camera_info_share_immutable_result(self):
        mapper = prepare()
        first = event(mapper)
        mapper.observe_clock(M + 80_000_000 - 100, R + 80_000_000, M + 80_000_000 + 100, "boot-a")
        mapper.observe_anchor(anchor(M + 80_000_000, D + 80_000_000, 18), M + 80_000_100, (4, 5))
        for stream in ("right", "left_info", "right_info", "left_info"):
            self.assertIs(first, event(mapper, stream=stream, now_delta=81_000_000))
        self.assertEqual(mapper.counters["stereo_reuses"], 4)
        with self.assertRaises(FrozenInstanceError):
            first.mapped_stamp_ns = 0

    def test_invalid_stereo_group_stays_invalid(self):
        mapper = prepare()
        mapper.observe_anchor(None, M + 71_000_000, read_error="missing")
        first = event(mapper)
        self.assertFalse(first.valid)
        self.assertEqual(first.reason, "ANCHOR_READ_MISSING")
        self.assertIs(first, event(mapper, stream="right_info"))
        self.assertIsNone(first.mapped_stamp_ns)

    def test_imu_has_independent_order_and_negative_arrival_is_not_clipped(self):
        mapper = prepare()
        self.assertTrue(event(mapper).valid)
        imu = event(mapper, delta=60_000_000, stream="imu_raw")
        self.assertTrue(imu.valid, imu.reason)
        duplicate = event(mapper, delta=60_000_000, stream="imu_raw")
        self.assertEqual(duplicate.reason, "ZED_TIMESTAMP_DUPLICATE")
        future_imu = event(mapper, delta=74_000_000, stream="imu_raw")
        self.assertTrue(future_imu.valid, future_imu.reason)
        self.assertEqual(future_imu.wall_minus_raw_ns, -2_000_000)

    def test_raw_regression_blocks_until_explicit_zed_restart(self):
        mapper = prepare()
        self.assertTrue(event(mapper, stream="imu_raw").valid)
        self.assertEqual(event(mapper, delta=65_000_000, stream="imu_raw").reason, "ZED_TIMESTAMP_REGRESSION")
        self.assertEqual(event(mapper, delta=72_000_000, stream="imu_raw").reason, "ZED_TIMESTAMP_REGRESSION")
        self.assertEqual(event(mapper, delta=65_000_000, stream="imu_raw", session="camera-b").reason, "ZED_SESSION_CHANGED")
        self.assertTrue(event(mapper, delta=66_000_000, stream="imu_raw", session="camera-b").valid)
        self.assertEqual(event(mapper, delta=73_000_000, session="camera-a").reason, "ZED_RETIRED_SESSION")

    def test_cached_results_cannot_bypass_new_raw_regression_or_staleness(self):
        mapper = prepare()
        first = event(mapper)
        self.assertTrue(first.valid)
        regressed = event(mapper, delta=65_000_000)
        self.assertEqual(regressed.reason, "ZED_TIMESTAMP_REGRESSION")
        old = event(mapper)
        self.assertFalse(old.valid)
        self.assertEqual(old.reason, "ZED_TIMESTAMP_REGRESSION")
        self.assertNotEqual(first.session_id, old.session_id)
        self.assertTrue(first.valid)  # Saved evidence remains immutable.
        for advance_clock, expected in ((False, "CLOCK_STALE"), (True, "ANCHOR_SEQUENCE_STALLED")):
            mapper = prepare()
            first = event(mapper)
            if advance_clock:
                mapper.observe_clock(M + 200_000_000, R + 200_000_000, M + 200_000_000, "boot-a")
            later = event(mapper, now_delta=201_000_000)
            self.assertFalse(later.valid)
            self.assertEqual(later.reason, expected)
            self.assertNotEqual(first.session_id, later.session_id)

    def test_clock_only_poll_reports_stopped_writer_without_mapping_event(self):
        mapper = prepare()
        stopped = anchor(M + 70_000_000, D + 70_000_000, 16)
        segment = mapper.anchor_segment
        observation = mapper.observe_anchor(stopped, M + 200_000_000, (4, 5))
        self.assertFalse(observation["accepted"])
        self.assertEqual(observation["reason"], "ANCHOR_SEQUENCE_STALLED")
        self.assertGreater(mapper.anchor_segment, segment)
        self.assertEqual(mapper.status()["anchor_history"], 0)
        self.assertEqual(mapper.status()["anchor_status"], "ANCHOR_SEQUENCE_STALLED")

    def test_snapshot_integer_ready_is_not_accepted_as_bool(self):
        mapper = prepare()
        observation = mapper.observe_anchor(anchor(M + 80_000_000, D + 80_000_000, 18, ready=1),
                                            M + 80_000_100, (4, 5))
        self.assertEqual(observation["reason"], "ANCHOR_PROTOCOL")

    def test_latest_writer_age_is_separate_from_historical_target_distance(self):
        mapper = prepare(count=40)
        historic = event(mapper, delta=25_000_000, now_delta=391_000_000)
        self.assertTrue(historic.valid, historic.reason)
        self.assertEqual(historic.anchor_delta_ns, 5_000_000)
        self.assertLess(historic.anchor_host_mono_ns, M + 50_000_000)
        self.assertEqual(historic.anchor_age_ns, 1_000_000)
        # Advance only the host bridge; the exact historical anchor still exists.
        mapper.observe_clock(M + 600_000_000, R + 600_000_000, M + 600_000_000, "boot-a")
        stale = event(mapper, delta=26_000_000, now_delta=601_000_000)
        self.assertEqual(stale.reason, "ANCHOR_SEQUENCE_STALLED")
        self.assertIsNone(stale.mapped_stamp_ns)

    def test_target_distance_rejected_with_fresh_writer(self):
        mapper = prepare(Config(max_clock_extrapolation_ns=500_000_000))
        result = event(mapper, delta=200_000_000)
        self.assertEqual(result.reason, "ANCHOR_TARGET_DISTANCE")
        self.assertLess(result.anchor_age_ns, 100_000_000)
        self.assertGreater(result.anchor_delta_ns, 100_000_000)

    def test_anchor_ready_source_and_protocol_fail_closed(self):
        for changes, reason in (({"ready": False}, "ANCHOR_NOT_READY"),
                                ({"clock_source": 1}, "ANCHOR_CLOCK_SOURCE"),
                                ({"magic": 0}, "ANCHOR_PROTOCOL"),
                                ({"write_sequence": 19}, "ANCHOR_PROTOCOL"),
                                ({"imu_stamp_ns": INT64_MAX + 1}, "INT64_OVERFLOW")):
            mapper = prepare()
            observation = mapper.observe_anchor(anchor(M + 71_000_000, D + 71_000_000, 18, **changes), M + 71_000_100, (4, 5))
            self.assertEqual(observation["reason"], reason)
            self.assertEqual(event(mapper).reason, reason)

    def test_livox_backstep_jump_epoch_inode_and_sequence_retire_history(self):
        for changed, identity, expected in (({"imu_stamp_ns": D - 1}, (4, 5), "LIVOX_TIME_REGRESSION"),
                                             ({"imu_stamp_ns": D + 500_000_000}, (4, 5), "LIVOX_TIME_JUMP"),
                                             ({"epoch": 18}, (4, 5), "WRITER_EPOCH_CHANGED"),
                                             ({}, (4, 6), "ANCHOR_INODE_CHANGED"),
                                             ({"write_sequence": 2}, (4, 5), "ANCHOR_SEQUENCE_REGRESSION")):
            mapper = prepare()
            before = mapper.anchor_segment
            parameters = dict(m=M + 80_000_000, d=D + 80_000_000, sequence=18)
            parameters.update(changed)
            a = anchor(**parameters)
            observation = mapper.observe_anchor(a, M + 80_000_100, identity)
            self.assertEqual(observation["reason"], expected)
            self.assertGreater(mapper.anchor_segment, before)
            self.assertFalse(event(mapper, now_delta=81_000_000).valid)
            self.assertEqual(mapper.status()["anchor_history"], 1)

    def test_epoch_recovery_with_smaller_device_time(self):
        mapper = prepare()
        self.assertTrue(event(mapper).valid)
        for i in range(3):
            m = M + 80_000_000 + i * STEP
            mapper.observe_clock(m, R + m - M, m, "boot-a")
            mapper.observe_anchor(anchor(m, D - 5_000_000_000 + i * STEP, 2 * i + 2, epoch=18), m + 100, (4, 5))
        result = event(mapper, delta=101_000_000, now_delta=102_000_000)
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(result.mapped_stamp_ns, D - 4_979_000_000)
        old = event(mapper, delta=75_000_000, now_delta=102_000_000, stream="new_stream")
        self.assertEqual(old.reason, "ANCHOR_OUTSIDE_SESSION")

    def test_clock_jumps_host_boot_suspend_and_mono_regression(self):
        cases = ((M + 80_000_000, R + 1_080_000_000, "boot-a", None, "REALTIME_JUMP"),
                 (M + 80_000_000, R - 920_000_000, "boot-a", None, "REALTIME_JUMP"),
                 (M + 80_000_000, R + 80_000_000, "boot-b", None, "HOST_BOOT_CHANGED"),
                 (M + 80_000_000, R + 80_000_000, "boot-a", M + 180_000_000, "HOST_SUSPEND_RESUME"),
                 (M + 60_000_000, R + 60_000_000, "boot-a", None, "MONOTONIC_DISCONTINUITY"))
        for m, r, boot, boottime, reason in cases:
            mapper = prepare()
            segment = mapper.clock_segment
            outcome = mapper.observe_clock(m, r, m, boot, boottime)
            self.assertEqual(outcome["reason"], reason)
            self.assertGreater(mapper.clock_segment, segment)
            self.assertFalse(event(mapper, now_delta=81_000_000).valid)
            self.assertEqual(mapper.status()["anchor_history"], 0)

    def test_clock_recovery_does_not_map_previous_segment(self):
        mapper = prepare()
        for i in range(3):
            m = M + 80_000_000 + i * STEP
            mapper.observe_clock(m, R + 1_000_000_000 + m - M, m, "boot-a")
            mapper.observe_anchor(anchor(m, D + m - M, 18 + 2 * i), m + 100, (4, 5))
        old = event(mapper, delta=71_000_000, now_delta=102_000_000)
        self.assertEqual(old.reason, "CLOCK_OUTSIDE_SESSION")
        good = event(mapper, raw=R + 1_101_000_000, now_delta=102_000_000)
        self.assertTrue(good.valid, good.reason)

    def test_bracket_fit_slope_and_robust_outlier(self):
        mapper = prepare()
        result = mapper.observe_clock(M + 80_000_000, R + 80_000_000, M + 80_100_001, "boot-a")
        self.assertEqual(result["reason"], "CLOCK_BRACKET_TOO_WIDE")
        self.assertEqual(mapper.counters["clock_rejected"], 1)
        points = [(M + i * STEP, R + i * STEP + (500_000 if i == 4 else 0)) for i in range(12)]
        fit, reason = fit_affine(points, 100_000, 2000, 3, 20_000_000, 1)
        self.assertEqual(reason, "OK")
        self.assertEqual(fit.rejected_count, 1)
        self.assertEqual(round_fraction(fit.forward(M + 33_000_000)), R + 33_000_000)
        points = [(M + i * STEP, R + i * STEP * 2) for i in range(5)]
        self.assertEqual(fit_affine(points, 100_000, 2000, 3, 20_000_000, 1)[1], "FIT_SLOPE")
        offsets = [0, 1_000_000, -1_000_000, 2_000_000, -2_000_000, 3_000_000, -3_000_000, 4_000_000]
        noisy = [(M + i * STEP, R + i * STEP + v) for i, v in enumerate(offsets)]
        self.assertEqual(fit_affine(noisy, 1_000, 2000, 3, 20_000_000, 1)[1], "FIT_RESIDUAL")

    def test_stale_anchor_age_and_changed_data_reusing_sequence(self):
        mapper = prepare()
        stale = mapper.observe_anchor(anchor(M + 80_000_000, D + 80_000_000, 18), M + 201_000_000, (4, 5))
        self.assertFalse(stale["accepted"])
        self.assertEqual(stale["reason"], "ANCHOR_STALE")
        mapper = prepare()
        reused = mapper.observe_anchor(anchor(M + 80_000_000, D + 80_000_000, 16), M + 80_000_100, (4, 5))
        self.assertFalse(reused["accepted"])
        self.assertEqual(reused["reason"], "ANCHOR_SEQUENCE_REUSED")

    def test_overflow_input_no_float_and_uint64_identifiers(self):
        for raw, reason in ((INT64_MAX + 1, "INT64_OVERFLOW"), (float(R), "NOT_INTEGER_NS"),
                            (0, "NONPOSITIVE_TIMESTAMP")):
            result = event(prepare(), raw=raw)
            self.assertEqual(result.reason, reason)
            self.assertIsNone(result.mapped_stamp_ns)
        result = event(prepare(Config(compensation_ns=-INT64_MAX)))
        self.assertEqual(result.reason, "INT64_OVERFLOW")
        mapper = prepare()
        outcome = mapper.observe_anchor(anchor(M + 80_000_000, D + 80_000_000, 18, epoch=(1 << 64) - 1),
                                         M + 80_000_100, (4, 5))
        self.assertTrue(outcome["accepted"])

    def test_processing_now_not_early_arrival_controls_freshness(self):
        mapper = prepare()
        result = event(mapper, now_delta=69_000_000, now_mono_ns=M + 72_000_000)
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(result.anchor_age_ns, 2_000_000)
        self.assertEqual(result.processing_mono_ns, M + 72_000_000)

    def test_deterministic_recorded_observation_replay(self):
        records = []
        for i in range(8):
            m = M + i * STEP
            records += [("clock", (m, R + i * STEP, m, "recorded-boot")),
                        ("anchor", (anchor(m, D + i * STEP, 2 * i + 2), m + 100, (4, 5)))]
        outputs = []
        for _ in range(2):
            mapper = ShadowMapper()
            for kind, arguments in records:
                getattr(mapper, "observe_" + kind)(*arguments)
            outputs.append(event(mapper).as_dict())
        self.assertEqual(outputs[0], outputs[1])
        self.assertTrue(outputs[0]["valid"])
        self.assertFalse(event(ShadowMapper()).valid)


if __name__ == "__main__":
    unittest.main(verbosity=2)
