#!/usr/bin/env python3
"""No-bag synthetic checks for the Stage 6 competition output chain."""

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

import make_competition_submission_v2 as m


def trajectory(path, rows, comments):
    return m.Trajectory(path=Path(path), data=np.asarray(rows, dtype=float), comments=comments)


def assert_close(actual, expected, atol=1e-9):
    assert np.allclose(actual, expected, atol=atol, rtol=0.0), (actual, expected)


def main():
    # External schedule is the only source of competition epochs.
    schedule_path, score_times, dynamic_windows, sanity = m.load_schedule(
        m.DEFAULT_SCHEDULE_PATH
    )
    assert schedule_path == m.DEFAULT_SCHEDULE_PATH.resolve()
    assert [pid for pid, _ in score_times] == [
        f"P{i:02d}" for i in range(1, len(score_times) + 1)
    ]
    assert [name for name, _, _ in dynamic_windows] == ["01", "02", "03"]
    assert sanity == (1785900626.0, 1074, 271830.0)

    # 1) ENU -> NEU is exactly one E/N swap; U is unchanged.
    assert_close(m.enu_to_neu([1.0, 2.0, 3.0]), [2.0, 1.0, 3.0])

    # 2) UTC <-> BDT fixed dataset sanity and exact inverse.
    week, sow = m.unix_to_bds_week_sow(1785900626.0)
    assert (week, sow) == (1074, 271830.0)
    assert abs(m.bds_week_sow_to_unix(week, sow) - 1785900626.0) < 1e-9

    # 3/4/10) Position is linear, orientation is SLERP, never nearest-neighbor.
    interp = trajectory(
        "interp.tum",
        [
            [0, 0, 0, 0, 0, 0, 0, 1],
            [2, 10, 0, 0, 0, 0, 1, 0],
        ],
        ["# frame_id=map/ENU"],
    )
    p, q, gap = m.interpolate_pose(interp, 1.0, max_gap_s=2.0)
    assert_close(p, [5, 0, 0])
    assert abs(gap - 2.0) < 1e-12
    assert_close(m.quat_to_rot(q) @ np.array([1.0, 0.0, 0.0]), [0, 1, 0])

    ident = [0, 0, 0, 1]
    comments = [
        "# frame_id=map/ENU; "
        "saved_reference=body+R_body*result_pose_lever_arm_body_m"
    ]
    alignment = m.BackendAlignment(90.0, np.array([10.0, 20.0, 30.0]))
    raw = trajectory(
        "raw.tum",
        [
            [1785900561.8, 1, 0, 0, *ident],
            [1785900562.2, 3, 0, 0, *ident],
        ],
        ["# frame_id=odom"],
    )

    # 5) Explicit zero lever remains exactly zero and does not move fused poses.
    fused_zero = trajectory(
        "online.tum",
        [
            [1785900590, 7, 8, 9, *ident],
            [1785900840, 17, 18, 19, *ident],
        ],
        comments,
    )
    ref_zero = m.ReferenceConfig("body", np.zeros(3), "test", None, None)
    combined = m.CompetitionTrajectory(fused_zero, 300.0, raw, alignment, ref_zero)
    direct = combined.pose(1785900590.0)
    assert_close(direct[0], [7, 8, 9])

    # 6) Explicit nonzero saved lever is rotated, then removed to recover body.
    qz90 = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    body = np.array([4.0, 5.0, 6.0])
    lever = np.array([1.0, 0.0, 0.0])
    saved = body + m.quat_to_rot(qz90) @ lever
    fused_lever = trajectory(
        "lever.tum",
        [[0, *saved, *qz90], [1, *saved, *qz90]],
        comments,
    )
    ref_lever = m.ReferenceConfig("body", lever, "explicit", None, None)
    lever_traj = m.CompetitionTrajectory(fused_lever, 2.0, None, None, ref_lever)
    assert_close(lever_traj.pose(0.5)[0], body)
    ref_person = m.ReferenceConfig(
        "person_ground", lever, "explicit", np.array([1.0, 0.0, 0.0]), np.array([10.0, 0.0, 0.0])
    )
    person_traj = m.CompetitionTrajectory(fused_lever, 2.0, None, None, ref_person)
    assert_close(person_traj.pose(0.5)[0], body + np.array([10.0, 1.0, 0.0]))

    # 7/12) Missing reference metadata fails; no trajectory-overlap lever estimator exists.
    missing_meta = trajectory("missing.tum", fused_zero.data, ["# frame_id=map/ENU"])
    try:
        m.resolve_saved_reference_lever(missing_meta, Path("."), [0, 0, 0])
        raise AssertionError("missing reference metadata was accepted")
    except RuntimeError:
        pass
    assert not hasattr(m, "robust_lever_estimate")

    # 8/11) P01 uses the backend alignment exactly once and is explicitly labelled.
    p01 = combined.pose(1785900562.0, force_aligned_raw=True)
    assert p01[2] == "ALIGNED_RAW_FALLBACK"
    assert_close(p01[0], [10.0, 22.0, 30.0])
    assert combined.pose(1785900590.0)[2] == "ONLINE_INTERPOLATED"
    assert_close(combined.pose(1785900590.0)[0], [7, 8, 9])

    # 9) Point count follows the schedule (20 here); dynamic files remain strict 10 Hz.
    with tempfile.TemporaryDirectory() as td:
        out = Path(td)
        test_schedule = out / "schedule_20_points.json"
        test_schedule.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "score_times": [
                        {
                            "id": f"P{i:02d}",
                            "unix_utc": 1785900562.0 if i == 1 else 1785900589.0 + i,
                        }
                        for i in range(1, 21)
                    ],
                    "dynamic_windows": [
                        {
                            "id": name,
                            "start_unix_utc": start,
                            "end_unix_utc": end,
                        }
                        for name, start, end in dynamic_windows
                    ],
                    "bds_sanity": {
                        "unix_utc": sanity[0],
                        "week": sanity[1],
                        "sow": sanity[2],
                    },
                }
            ),
            encoding="utf-8",
        )
        loaded = m.load_schedule(test_schedule)
        assert [pid for pid, _ in loaded[1]] == [f"P{i:02d}" for i in range(1, 21)]

        original_schedule = (m.SCHEDULE_PATH, m.SCORE_TIMES, m.DYNAMIC_WINDOWS, m.BDS_SANITY)
        m.SCHEDULE_PATH, m.SCORE_TIMES, m.DYNAMIC_WINDOWS, m.BDS_SANITY = loaded
        try:
            points = m.write_position_points(out / "position_points.txt", combined)
            assert len(points) == 20
            assert points[0][5] == "ALIGNED_RAW_FALLBACK"
            for name, start, end in m.DYNAMIC_WINDOWS:
                rows, sources = m.write_dynamic(
                    out / f"position_dynamic_{name}.txt", combined, start, end
                )
                assert len(rows) == 201
                assert sources == {"ONLINE_INTERPOLATED": 201}
            checks = m.validate_output_files(out)
            assert len(checks) == 5
            assert checks[0].startswith("position_points: 20 lines, P01..P20")
            hash_path = m.write_submission_hashes(out)
            hash_lines = hash_path.read_text(encoding="ascii").splitlines()
            assert len(hash_lines) == 4
            assert [line.split("  ", 1)[1] for line in hash_lines] == [
                "position_points.txt",
                "position_dynamic_01.txt",
                "position_dynamic_02.txt",
                "position_dynamic_03.txt",
            ]

            validate_proc = subprocess.run(
                [
                    sys.executable,
                    str(Path(m.__file__).resolve()),
                    "--schedule-file",
                    str(test_schedule),
                    "--validate-only",
                    str(out),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            assert validate_proc.returncode == 0, validate_proc.stderr
            assert "position_points: 20 lines, P01..P20" in validate_proc.stdout
            assert "PASS: strict submission format validation" in validate_proc.stdout

            # CRLF/BOM/trailing-space detection is part of the same strict reader.
            point_path = out / "position_points.txt"
            original = point_path.read_bytes()
            point_path.write_bytes(original.replace(b"\n", b"\r\n"))
            try:
                m.validate_output_files(out)
                raise AssertionError("CRLF output was accepted")
            except RuntimeError as exc:
                assert "CR/CRLF" in str(exc)
        finally:
            m.SCHEDULE_PATH, m.SCORE_TIMES, m.DYNAMIC_WINDOWS, m.BDS_SANITY = original_schedule

    # 13) person_ground without d_base is rejected before any run/bag access.
    proc = subprocess.run(
        [
            sys.executable,
            str(Path(m.__file__).resolve()),
            "/does/not/matter",
            "--reference-mode",
            "person_ground",
            "--body-to-person-m",
            "0.18",
            "0.01",
            "-1.192",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert proc.returncode != 0
    assert "requires exactly one of --base-offset-m" in proc.stderr

    # 14) Identical official/PPK BASE ECEF gives machine-exact zero d_base.
    ppk_lla = np.array([29.5999999996, 107.5000000002, 489.4665])
    ppk_ecef = m.lla_to_ecef(ppk_lla)
    d_same, same_lla = m.base_offset_from_official_ecef(ppk_ecef, ppk_ecef)
    assert_close(d_same, np.zeros(3), atol=1e-12)
    assert_close(same_lla, ppk_lla, atol=1e-8)

    # 15/16/17) If official BASE is +1 m E/N/U from PPK BASE, PPK relative
    # to official is respectively -E/-N/-U. Small cross terms are Earth curvature.
    for index in range(3):
        official_enu = np.zeros(3)
        official_enu[index] = 1.0
        official_ecef = m.enu_to_ecef(official_enu, ppk_ecef, ppk_lla)
        d_base, _ = m.base_offset_from_official_ecef(ppk_ecef, official_ecef)
        expected = np.zeros(3)
        expected[index] = -1.0
        assert_close(d_base, expected, atol=3e-7)

    # 18) ECEF <-> ENU conversion is an inverse in E,N,U order.
    sample_enu = np.array([3.25, -4.5, 1.75])
    sample_ecef = m.enu_to_ecef(sample_enu, ppk_ecef, ppk_lla)
    assert_close(m.ecef_to_enu(sample_ecef, ppk_ecef, ppk_lla), sample_enu, atol=1e-9)

    # 19) WGS84 LLA -> ECEF -> LLA and LLA-driven d_base agree numerically.
    official_lla = ppk_lla + np.array([1e-6, -2e-6, 0.75])
    official_ecef = m.lla_to_ecef(official_lla)
    assert_close(m.ecef_to_lla(official_ecef), official_lla, atol=1e-8)
    d_from_ecef, _ = m.base_offset_from_official_ecef(ppk_ecef, official_ecef)
    d_from_lla, _ = m.base_offset_from_official_ecef(
        ppk_ecef, m.lla_to_ecef(official_lla)
    )
    assert_close(d_from_lla, d_from_ecef, atol=1e-12)

    # 20) BASE offsets remain ENU internally; NEU conversion swaps E/N once.
    assert_close(m.enu_to_neu([1.0, 2.0, 3.0]), [2.0, 1.0, 3.0])

    # 21) BASE RINEX metadata parser reads exactly APPROX POSITION XYZ.
    with tempfile.TemporaryDirectory() as td:
        rinex = Path(td) / "BASE.26O"
        rinex.write_text(
            "     3.04           OBSERVATION DATA    C                   RINEX VERSION / TYPE\n"
            " -1669133.0421  5293813.6807  3132139.1810                  APPROX POSITION XYZ\n"
            "                                                            END OF HEADER\n",
            encoding="ascii",
        )
        assert_close(
            m.read_rinex_approx_position_xyz(rinex),
            [-1669133.0421, 5293813.6807, 3132139.1810],
        )

        # 22/23/24) All three legal official BASE interfaces resolve without truth.
        common = dict(
            reference_mode="person_ground",
            body_to_person_m=[0.180, 0.010, -1.192],
            base_rinex=str(rinex),
        )
        parser = argparse.ArgumentParser()
        rinex_ppk_ecef = np.array([-1669133.0421, 5293813.6807, 3132139.1810])
        rinex_ppk_lla = m.ecef_to_lla(rinex_ppk_ecef)
        direct = argparse.Namespace(
            **common,
            base_offset_m=[0, 0, 0],
            base_offset_source="OFFICIAL_CONFIRMATION_SAME_AS_RINEX_BASE",
            official_base_ecef_m=None,
            official_base_lla=None,
        )
        assert_close(m.resolve_base_reference(direct, parser)[0], [0, 0, 0])
        ecef = argparse.Namespace(
            **common,
            base_offset_m=None,
            base_offset_source=None,
            official_base_ecef_m=rinex_ppk_ecef.tolist(),
            official_base_lla=None,
        )
        assert_close(m.resolve_base_reference(ecef, parser)[0], [0, 0, 0], atol=1e-9)
        lla = argparse.Namespace(
            **common,
            base_offset_m=None,
            base_offset_source=None,
            official_base_ecef_m=None,
            official_base_lla=rinex_ppk_lla.tolist(),
        )
        assert_close(m.resolve_base_reference(lla, parser)[0], [0, 0, 0], atol=1e-8)

    print("make_competition_submission_v2_self_test: PASS (28 checks)")


if __name__ == "__main__":
    main()
