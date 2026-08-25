#!/usr/bin/env python3
"""Offline shadow sweep of the existing visual observability projector formula."""

import argparse
import json
import math
from pathlib import Path

from analyze_visual_tracking_dry_run import distribution, integer, percent, read_csv


THRESHOLDS = (0.05, 0.08, 0.10, 0.12, 0.15, 0.20)
BRANCHES = ("RELAXED_20_24", "RELAXED_25_29")
AXES = "xyz"


def direction_weight(eigenvalue_ratio, relative_threshold):
    if eigenvalue_ratio < 0.25 * relative_threshold:
        return 0.0
    if eigenvalue_ratio < relative_threshold:
        return math.sqrt(eigenvalue_ratio / relative_threshold)
    return 1.0


def vector(row, prefix, index):
    return [float(row[f"{prefix}_eigenvector_{index}_{axis}"]) for axis in AXES]


def simulate(row, threshold):
    directions = []
    retained = {}
    for space in ("rotation", "translation"):
        eigenvalues = [float(row[f"{space}_eigenvalue_{index}"]) for index in range(3)]
        maximum = eigenvalues[2]
        weights = []
        for index, eigenvalue in enumerate(eigenvalues):
            ratio = eigenvalue / maximum
            weight = direction_weight(ratio, threshold)
            eigenvector = vector(row, space, index)
            component = (
                abs(eigenvector[2])
                if space == "translation"
                else math.hypot(eigenvector[0], eigenvector[1])
            )
            directions.append({
                "space": space,
                "index": index,
                "ratio": ratio,
                "weight": weight,
                "dominant_component": component,
            })
            weights.append(weight)
        information_before = sum(eigenvalues)
        information_after = sum(
            eigenvalue * weight * weight
            for eigenvalue, weight in zip(eigenvalues, weights)
        )
        retained[space] = information_after / information_before

    weights = [item["weight"] for item in directions]
    if all(weight == 1.0 for weight in weights):
        classification = "unchanged"
    elif any(weight == 0.0 for weight in weights):
        classification = "full"
    else:
        classification = "partial"
    rotation_before = sum(float(row[f"rotation_eigenvalue_{i}"]) for i in range(3))
    translation_before = sum(float(row[f"translation_eigenvalue_{i}"]) for i in range(3))
    combined_retained = (
        rotation_before * retained["rotation"] +
        translation_before * retained["translation"]
    ) / (rotation_before + translation_before)
    return {
        "classification": classification,
        "minimum_weight": min(weights),
        "combined_retained": combined_retained,
        "rotation_retained": retained["rotation"],
        "translation_retained": retained["translation"],
        "directions": directions,
    }


def summarize(rows, threshold):
    simulations = [(row, simulate(row, threshold)) for row in rows]
    affected = [(row, result) for row, result in simulations
                if result["classification"] != "unchanged"]
    counts = {
        name: sum(result["classification"] == name for _, result in simulations)
        for name in ("unchanged", "partial", "full")
    }
    affected_conditions = [
        max(float(row["rotation_condition"]), float(row["translation_condition"]))
        for row, _ in affected
    ]
    unchanged_conditions = [
        max(float(row["rotation_condition"]), float(row["translation_condition"]))
        for row, result in simulations if result["classification"] == "unchanged"
    ]
    condition_tail = [
        (row, result) for row, result in simulations
        if max(float(row["rotation_condition"]), float(row["translation_condition"])) >= 20.0
    ]
    suppressed_translation = [
        direction for _, result in affected for direction in result["directions"]
        if direction["space"] == "translation" and direction["weight"] < 1.0
    ]
    suppressed_rotation = [
        direction for _, result in affected for direction in result["directions"]
        if direction["space"] == "rotation" and direction["weight"] < 1.0
    ]
    vertical_dominant = [
        (row, result) for row, result in simulations
        if abs(float(row["translation_eigenvector_0_z"])) >= 0.8
    ]
    roll_pitch_dominant = [
        (row, result) for row, result in simulations
        if math.hypot(float(row["rotation_eigenvector_0_x"]),
                      float(row["rotation_eigenvector_0_y"])) >= 0.8
    ]

    def weakest_affected(items, space):
        return sum(
            next(direction for direction in result["directions"]
                 if direction["space"] == space and direction["index"] == 0)["weight"] < 1.0
            for _, result in items
        )

    return {
        "threshold": threshold,
        "frames": len(rows),
        "counts": counts,
        "percent": {name: percent(count, len(rows)) for name, count in counts.items()},
        "minimum_weight_all": distribution(
            [result["minimum_weight"] for _, result in simulations]
        ),
        "minimum_weight_affected": distribution(
            [result["minimum_weight"] for _, result in affected]
        ),
        "suppressed_direction_weight": distribution([
            direction["weight"] for _, result in affected for direction in result["directions"]
            if direction["weight"] < 1.0
        ]),
        # This is exact for the two Schur information matrices that the real
        # projector eigendecomposes; the CSV cannot reconstruct the coupled
        # pre-Schur 6x6 pose information matrix.
        "schur_information_retained": {
            "combined_all": distribution(
                [result["combined_retained"] for _, result in simulations]
            ),
            "combined_affected": distribution(
                [result["combined_retained"] for _, result in affected]
            ),
            "rotation_affected": distribution(
                [result["rotation_retained"] for _, result in affected]
            ),
            "translation_affected": distribution(
                [result["translation_retained"] for _, result in affected]
            ),
        },
        "suppressed_translation_vertical_component": distribution(
            [direction["dominant_component"] for direction in suppressed_translation]
        ),
        "suppressed_rotation_roll_pitch_component": distribution(
            [direction["dominant_component"] for direction in suppressed_rotation]
        ),
        "condition": {
            "affected": distribution(affected_conditions),
            "unchanged": distribution(unchanged_conditions),
            "tail_frames": len(condition_tail),
            "tail_affected": sum(
                result["classification"] != "unchanged" for _, result in condition_tail
            ),
        },
        "vertical_dominant_weakest": {
            "frames": len(vertical_dominant),
            "affected": weakest_affected(vertical_dominant, "translation"),
        },
        "roll_pitch_dominant_weakest": {
            "frames": len(roll_pitch_dominant),
            "affected": weakest_affected(roll_pitch_dominant, "rotation"),
        },
    }


def analyze(csv_path):
    rows = read_csv(csv_path)
    attempted = [
        row for row in rows
        if row.get("branch") in BRANCHES and integer(row, "ekf_attempted")
    ]
    return {
        "source": str(csv_path),
        "formula": {
            "full_if_ratio_below": "0.25 * relative_threshold",
            "partial_if_ratio_below": "relative_threshold",
            "partial_weight": "sqrt(eigenvalue_ratio / relative_threshold)",
        },
        "branches": {
            branch: {
                f"{threshold:.2f}": summarize(
                    [row for row in attempted if row.get("branch") == branch], threshold
                )
                for threshold in THRESHOLDS
            }
            for branch in BRANCHES
        },
    }


def render(report):
    lines = ["# RELAXED visual observability threshold shadow sweep", ""]
    lines.append(f"Source: `{report['source']}`")
    lines.append("")
    for branch, thresholds in report["branches"].items():
        lines.extend([f"## {branch}", ""])
        lines.append(
            "| threshold | unchanged/partial/full | affected min weight median/P10 | "
            "affected Schur trace retained median/P10 | affected condition median/P90 | "
            "condition>=20 hit | vertical weakest hit | roll/pitch weakest hit |"
        )
        lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|")
        for threshold, item in thresholds.items():
            counts = item["counts"]
            weight = item["minimum_weight_affected"]
            retained = item["schur_information_retained"]["combined_affected"]
            condition = item["condition"]
            vertical = item["vertical_dominant_weakest"]
            rotation = item["roll_pitch_dominant_weakest"]
            lines.append(
                f"| {threshold} | {counts['unchanged']}/{counts['partial']}/{counts['full']} | "
                f"{weight['median']:.3f}/{weight['p10']:.3f} | "
                f"{retained['median']:.4f}/{retained['p10']:.4f} | "
                f"{condition['affected']['median']:.1f}/{condition['affected']['p90']:.1f} | "
                f"{condition['tail_affected']}/{condition['tail_frames']} | "
                f"{vertical['affected']}/{vertical['frames']} | "
                f"{rotation['affected']}/{rotation['frames']} |"
            )
        lines.append("")
    return "\n".join(lines)


def self_test():
    assert direction_weight(0.20, 0.10) == 1.0
    assert abs(direction_weight(0.075, 0.10) - math.sqrt(0.75)) < 1e-12
    assert direction_weight(0.020, 0.10) == 0.0
    assert direction_weight(0.025, 0.10) == 0.5
    print("visual observability threshold shadow self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.csv is None:
        parser.error("csv is required unless --self-test is used")
    report = analyze(args.csv)
    markdown = render(report) + "\n"
    if args.json:
        args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    if args.markdown:
        args.markdown.write_text(markdown)
    print(markdown)


if __name__ == "__main__":
    main()
