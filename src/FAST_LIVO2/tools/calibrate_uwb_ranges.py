#!/usr/bin/env python3
"""Robust offline UWB time/bias calibration against a raw LIVO TUM path."""

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np


DISTANCE_RE = re.compile(r"distance\[(\d+)\],\s*([-+0-9.eE]+)")


def parse_vec(text, size):
    values = [float(value) for value in text.split(",")]
    if len(values) != size or not np.isfinite(values).all():
        raise argparse.ArgumentTypeError(f"expected {size} finite comma-separated values")
    return np.asarray(values, dtype=float)


def parse_anchor(text):
    anchor_id, coordinates = text.split(":", 1)
    return int(anchor_id), parse_vec(coordinates, 3)


def parse_bias(text):
    anchor_id, value = text.split(":", 1)
    return int(anchor_id), float(value)


def load_tum(path):
    rows = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            values = [float(value) for value in line.split()]
            if len(values) >= 8 and np.isfinite(values[:8]).all():
                rows.append(values[:8])
    trajectory = np.asarray(rows, dtype=float)
    if len(trajectory) < 2:
        raise RuntimeError("trajectory contains fewer than two valid poses")
    order = np.argsort(trajectory[:, 0], kind="stable")
    trajectory = trajectory[order]
    unique = np.r_[True, np.diff(trajectory[:, 0]) > 1e-9]
    return trajectory[unique]


def load_ranges(path, minimum, maximum):
    rows = []
    file_origin = None
    with open(path, "r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            fields = line.split(maxsplit=1)
            if not fields:
                continue
            try:
                stamp = float(fields[0])
            except ValueError:
                continue
            if file_origin is None:
                file_origin = stamp
            match = DISTANCE_RE.search(line)
            if not match:
                continue
            anchor_id = int(match.group(1))
            distance = float(match.group(2))
            if math.isfinite(distance) and minimum <= distance <= maximum:
                rows.append((stamp, anchor_id, distance, line_number))
    if file_origin is None or not rows:
        raise RuntimeError("UWB txt contains no valid distance records")
    return np.asarray(rows, dtype=float), file_origin


def filter_triangle_inconsistent(ranges, anchors, time_tolerance,
                                 distance_tolerance):
    """Drop paired ranges that cannot originate from the configured anchors."""
    rejected = np.zeros(len(ranges), dtype=bool)
    ids = ranges[:, 1].astype(int)
    indices_by_anchor = {
        anchor_id: np.flatnonzero(ids == anchor_id) for anchor_id in anchors
    }
    for first_id, first_indices in indices_by_anchor.items():
        for second_id, second_indices in indices_by_anchor.items():
            if second_id <= first_id or not len(first_indices) or not len(second_indices):
                continue
            second_times = ranges[second_indices, 0]
            insertion = np.searchsorted(second_times, ranges[first_indices, 0])
            for local_index, candidate in enumerate(insertion):
                choices = [value for value in (candidate - 1, candidate)
                           if 0 <= value < len(second_indices)]
                if not choices:
                    continue
                nearest = min(choices, key=lambda value: abs(
                    second_times[value] - ranges[first_indices[local_index], 0]))
                first_index = first_indices[local_index]
                second_index = second_indices[nearest]
                if abs(ranges[first_index, 0] - ranges[second_index, 0]) > time_tolerance:
                    continue
                first_range = ranges[first_index, 2]
                second_range = ranges[second_index, 2]
                baseline = np.linalg.norm(anchors[first_id] - anchors[second_id])
                inconsistent = (
                    abs(first_range - second_range) > baseline + distance_tolerance or
                    first_range + second_range < baseline - distance_tolerance)
                if inconsistent:
                    rejected[first_index] = True
                    rejected[second_index] = True
    return ranges[~rejected], int(np.sum(rejected))


def interpolate_poses(trajectory, query_times):
    times = trajectory[:, 0]
    upper = np.searchsorted(times, query_times, side="left")
    valid = (upper > 0) & (upper < len(times))
    exact_first = upper == 0
    valid |= exact_first & (np.abs(query_times - times[0]) < 1e-9)
    upper = np.clip(upper, 1, len(times) - 1)
    lower = upper - 1
    span = times[upper] - times[lower]
    alpha = np.clip((query_times - times[lower]) / span, 0.0, 1.0)
    positions = ((1.0 - alpha)[:, None] * trajectory[lower, 1:4] +
                 alpha[:, None] * trajectory[upper, 1:4])
    q0 = trajectory[lower, 4:8]
    q1 = trajectory[upper, 4:8].copy()
    q1[np.sum(q0 * q1, axis=1) < 0.0] *= -1.0
    quaternions = (1.0 - alpha)[:, None] * q0 + alpha[:, None] * q1
    quaternions /= np.linalg.norm(quaternions, axis=1)[:, None]
    segment_speed = np.linalg.norm(
        trajectory[upper, 1:4] - trajectory[lower, 1:4], axis=1) / span
    return positions, quaternions, segment_speed, valid


def rotate_vectors(quaternions, vector):
    xyz = quaternions[:, :3]
    w = quaternions[:, 3:4]
    repeated = np.broadcast_to(vector, xyz.shape)
    cross = np.cross(xyz, repeated)
    return repeated + 2.0 * (w * cross + np.cross(xyz, cross))


def transformed_anchors(parameters, anchors):
    yaw, tx, ty = parameters
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return {
        anchor_id: np.r_[rotation @ point[:2] + [tx, ty], point[2]]
        for anchor_id, point in anchors.items()
    }


def huber_cost(residuals, delta):
    absolute = np.abs(residuals)
    return np.sum(np.where(absolute <= delta,
                           0.5 * residuals * residuals,
                           delta * (absolute - 0.5 * delta)))


class CalibrationProblem:
    def __init__(self, trajectory, ranges, txt_origin, lidar_origin, anchors,
                 lever_arm, minimum_speed, huber_delta, fixed_biases):
        self.trajectory = trajectory
        self.ranges = ranges
        self.txt_origin = txt_origin
        self.lidar_origin = lidar_origin
        self.anchors = anchors
        self.lever_arm = lever_arm
        self.minimum_speed = minimum_speed
        self.huber_delta = huber_delta
        self.fixed_biases = fixed_biases

    def residuals(self, parameters, offset, dynamic_only=True,
                  biases_override=None):
        query = (self.lidar_origin + self.ranges[:, 0] - self.txt_origin +
                 offset)
        positions, quaternions, speeds, valid = interpolate_poses(
            self.trajectory, query)
        valid &= np.array([int(value) in self.anchors
                           for value in self.ranges[:, 1]])
        if dynamic_only:
            valid &= speeds >= self.minimum_speed
        selected = self.ranges[valid]
        tag_positions = (positions[valid] +
                         rotate_vectors(quaternions[valid], self.lever_arm))
        anchor_positions = transformed_anchors(parameters, self.anchors)
        selected_ids = selected[:, 1].astype(int)
        predicted = np.empty(len(selected))
        for anchor_id, anchor_position in anchor_positions.items():
            mask = selected_ids == anchor_id
            predicted[mask] = np.linalg.norm(
                tag_positions[mask] - anchor_position, axis=1)
        measured = selected[:, 2]
        biases = (dict(biases_override) if biases_override is not None
                  else dict(self.fixed_biases))
        if biases_override is None:
            for anchor_id in sorted(self.anchors):
                mask = selected_ids == anchor_id
                if np.any(mask) and anchor_id not in biases:
                    biases[anchor_id] = float(np.median(
                        measured[mask] - predicted[mask]))
        corrected = measured.copy()
        for anchor_id, bias in biases.items():
            corrected[selected_ids == anchor_id] -= bias
        return predicted - corrected, selected, predicted, biases

    def optimize_transform(self, offset, initial):
        parameters = np.asarray(initial, dtype=float).copy()
        _, _, _, biases = self.residuals(parameters, offset)
        variable_bias_ids = [anchor_id for anchor_id in sorted(self.anchors)
                             if anchor_id not in self.fixed_biases]
        damping = 1e-3
        for _ in range(30):
            residuals, selected, _, _ = self.residuals(
                parameters, offset, biases_override=biases)
            if len(residuals) < 6:
                return parameters, float("inf"), {}
            steps = np.array([1e-5, 1e-3, 1e-3])
            jacobian = np.empty((len(residuals), 3 + len(variable_bias_ids)))
            for column, step in enumerate(steps):
                perturbed = parameters.copy()
                perturbed[column] += step
                shifted, _, _, _ = self.residuals(
                    perturbed, offset, biases_override=biases)
                jacobian[:, column] = (shifted - residuals) / step
            selected_ids = selected[:, 1].astype(int)
            for column, anchor_id in enumerate(variable_bias_ids, 3):
                jacobian[:, column] = (selected_ids == anchor_id).astype(float)
            absolute = np.abs(residuals)
            weights = np.where(absolute <= self.huber_delta, 1.0,
                               self.huber_delta / np.maximum(absolute, 1e-12))
            normal = jacobian.T @ (weights[:, None] * jacobian)
            gradient = jacobian.T @ (weights * residuals)
            try:
                update = -np.linalg.solve(
                    normal + damping * np.eye(jacobian.shape[1]), gradient)
            except np.linalg.LinAlgError:
                break
            update[0] = np.clip(update[0], -0.2, 0.2)
            update[1:3] = np.clip(update[1:3], -2.0, 2.0)
            update[3:] = np.clip(update[3:], -0.2, 0.2)
            current_cost = huber_cost(residuals, self.huber_delta)
            accepted_scale = 0.0
            for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
                candidate_parameters = parameters + scale * update[:3]
                candidate_parameters[0] = math.atan2(
                    math.sin(candidate_parameters[0]),
                    math.cos(candidate_parameters[0]))
                candidate_biases = dict(biases)
                for value, anchor_id in zip(update[3:], variable_bias_ids):
                    candidate_biases[anchor_id] += scale * value
                candidate_residuals, _, _, _ = self.residuals(
                    candidate_parameters, offset,
                    biases_override=candidate_biases)
                if huber_cost(candidate_residuals, self.huber_delta) < current_cost:
                    parameters = candidate_parameters
                    biases = candidate_biases
                    accepted_scale = scale
                    break
            if accepted_scale == 0.0 or np.linalg.norm(accepted_scale * update) < 1e-6:
                break
        residuals, _, _, _ = self.residuals(
            parameters, offset, biases_override=biases)
        return parameters, huber_cost(residuals, self.huber_delta), biases


def initial_transforms(problem):
    displacement = problem.trajectory[-1, 1:3] - problem.trajectory[0, 1:3]
    path_yaw = math.atan2(displacement[1], displacement[0])
    early_limit = np.quantile(problem.ranges[:, 0], 0.2)
    early = problem.ranges[problem.ranges[:, 0] <= early_limit]
    start_id = min(problem.anchors,
                   key=lambda anchor_id: np.median(
                       early[early[:, 1].astype(int) == anchor_id, 2])
                   if np.any(early[:, 1].astype(int) == anchor_id)
                   else float("inf"))
    start_tag = problem.trajectory[0, 1:3]
    seeds = []
    for yaw in (path_yaw, path_yaw + math.pi, 0.0, math.pi / 2.0):
        cosine, sine = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        translation = start_tag[:2] - rotation @ problem.anchors[start_id][:2]
        seeds.append(np.r_[yaw, translation])
    return seeds


def near_range_transform(problem, offset):
    query = (problem.lidar_origin + problem.ranges[:, 0] -
             problem.txt_origin + offset)
    positions, quaternions, _, valid = interpolate_poses(
        problem.trajectory, query)
    tags = positions + rotate_vectors(quaternions, problem.lever_arm)
    ids = problem.ranges[:, 1].astype(int)
    external = []
    local = []
    for anchor_id, anchor in sorted(problem.anchors.items()):
        indices = np.flatnonzero(valid & (ids == anchor_id))
        if not len(indices):
            continue
        nearest = indices[np.argsort(problem.ranges[indices, 2])[:10]]
        external.append(anchor[:2])
        local.append(np.median(tags[nearest, :2], axis=0))
    if len(external) < 2:
        return initial_transforms(problem)[0]
    external = np.asarray(external)
    local = np.asarray(local)
    external_center = np.mean(external, axis=0)
    local_center = np.mean(local, axis=0)
    covariance = ((external - external_center).T @
                  (local - local_center))
    left, _, right = np.linalg.svd(covariance)
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right[-1] *= -1.0
        rotation = right.T @ left.T
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    translation = local_center - rotation @ external_center
    return np.r_[yaw, translation]


def search_offset(problem, center, radius, coarse_step, fine_step):
    static_seeds = initial_transforms(problem)
    curve = []
    best = (float("inf"), None, None, None)
    coarse = np.arange(center - radius, center + radius + 0.5 * coarse_step,
                       coarse_step)
    for offset in coarse:
        seeds = [near_range_transform(problem, offset)] + static_seeds
        candidates = [problem.optimize_transform(offset, seed) for seed in seeds]
        parameters, cost, biases = min(candidates, key=lambda value: value[1])
        curve.append((float(offset), float(cost)))
        if cost < best[0]:
            best = (cost, float(offset), parameters, biases)
    fine = np.arange(best[1] - coarse_step, best[1] + coarse_step +
                     0.5 * fine_step, fine_step)
    for offset in fine:
        parameters, cost, biases = problem.optimize_transform(offset, best[2])
        curve.append((float(offset), float(cost)))
        if cost < best[0]:
            best = (cost, float(offset), parameters, biases)
    curve.sort()
    return best, curve


def distribution(values):
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "std": float(np.std(values)),
        "mad": float(np.median(np.abs(values - np.median(values)))),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "p95_abs": float(np.quantile(np.abs(values), 0.95)),
        "max_abs": float(np.max(np.abs(values))),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory")
    parser.add_argument("uwb_txt")
    parser.add_argument("--lidar-origin", type=float)
    parser.add_argument("--anchor", action="append", type=parse_anchor,
                        required=True)
    parser.add_argument("--lever-arm", type=lambda text: parse_vec(text, 3),
                        default=np.zeros(3))
    parser.add_argument("--fixed-offset", type=float)
    parser.add_argument("--fixed-bias", action="append", type=parse_bias,
                        default=[])
    parser.add_argument("--offset-radius", type=float, default=3.0)
    parser.add_argument("--coarse-step", type=float, default=0.10)
    parser.add_argument("--fine-step", type=float, default=0.01)
    parser.add_argument("--minimum-speed", type=float, default=0.10)
    parser.add_argument("--huber-delta", type=float, default=1.0)
    parser.add_argument("--minimum-range", type=float, default=0.05)
    parser.add_argument("--maximum-range", type=float, default=250.0)
    parser.add_argument("--pair-time-tolerance", type=float, default=0.02)
    parser.add_argument("--triangle-tolerance", type=float, default=1.0)
    parser.add_argument("--output-json")
    arguments = parser.parse_args()

    trajectory = load_tum(arguments.trajectory)
    ranges, txt_origin = load_ranges(arguments.uwb_txt,
                                     arguments.minimum_range,
                                     arguments.maximum_range)
    anchors = dict(arguments.anchor)
    original_range_count = len(ranges)
    ranges, triangle_rejected = filter_triangle_inconsistent(
        ranges, anchors, arguments.pair_time_tolerance,
        arguments.triangle_tolerance)
    lidar_origin = (arguments.lidar_origin if arguments.lidar_origin is not None
                    else trajectory[0, 0])
    problem = CalibrationProblem(
        trajectory, ranges, txt_origin, lidar_origin, anchors,
        arguments.lever_arm, arguments.minimum_speed, arguments.huber_delta,
        dict(arguments.fixed_bias))

    if arguments.fixed_offset is None:
        # The replay adapter rebases the txt origin onto the first LIVO stamp;
        # only the remaining fixed phase offset is searched here.
        center = 0.0
        best, curve = search_offset(problem, center, arguments.offset_radius,
                                    arguments.coarse_step, arguments.fine_step)
        cost, offset, parameters, biases = best
    else:
        offset = arguments.fixed_offset
        seeds = ([near_range_transform(problem, offset)] +
                 initial_transforms(problem))
        candidates = [problem.optimize_transform(offset, seed)
                      for seed in seeds]
        parameters, cost, biases = min(candidates, key=lambda value: value[1])
        curve = [(offset, cost)]

    corrected_residuals, selected, predicted, biases = problem.residuals(
        parameters, offset, dynamic_only=False, biases_override=biases)
    raw_residuals = predicted - selected[:, 2]
    per_anchor = {}
    for anchor_id in sorted(anchors):
        mask = selected[:, 1].astype(int) == anchor_id
        if not np.any(mask):
            continue
        per_anchor[str(anchor_id)] = {
            "count": int(np.sum(mask)),
            "bias_m": float(biases.get(anchor_id, 0.0)),
            "raw_residual": distribution(raw_residuals[mask]),
            "corrected_residual": distribution(corrected_residuals[mask]),
        }
    local_anchors = transformed_anchors(parameters, anchors)
    near_curve = sorted(curve, key=lambda value: abs(value[0] - offset))[:11]
    result = {
        "offset_semantics": "aligned_time=txt_rebased_measurement_time+offset",
        "trajectory": str(Path(arguments.trajectory).resolve()),
        "uwb_txt": str(Path(arguments.uwb_txt).resolve()),
        "trajectory_start": float(trajectory[0, 0]),
        "trajectory_end": float(trajectory[-1, 0]),
        "lidar_origin": float(lidar_origin),
        "txt_origin": float(txt_origin),
        "estimated_time_offset_s": float(offset),
        "huber_cost": float(cost),
        "valid_samples": int(len(selected)),
        "input_range_count": int(original_range_count),
        "triangle_inconsistent_rejected": triangle_rejected,
        "frame_alignment": {
            "model": "gravity_aligned_SE2_only",
            "yaw_rad": float(parameters[0]),
            "translation_xy_m": [float(parameters[1]), float(parameters[2])],
            "anchors_in_livo_frame": {
                str(anchor_id): [float(value) for value in point]
                for anchor_id, point in local_anchors.items()
            },
        },
        "per_anchor": per_anchor,
        "cost_curve_near_minimum": [
            {"offset_s": value[0], "cost": value[1]} for value in near_curve
        ],
    }
    rendered = json.dumps(result, indent=2, ensure_ascii=True)
    print(rendered)
    if arguments.output_json:
        Path(arguments.output_json).write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
