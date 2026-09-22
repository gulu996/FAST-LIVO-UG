#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <unordered_map>
#include <vector>

namespace fast_livo {
namespace p4 {

struct Association {
  int point_index = -1;
  int plane_id = -1;
  std::int64_t voxel_x = 0;
  std::int64_t voxel_y = 0;
  std::int64_t voxel_z = 0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double residual = 0.0;
};

struct DeskewDiagnostics {
  bool valid = false;
  bool point_time_monotonic = false;
  bool imu_covers_propagation = false;
  std::size_t point_count = 0;
  std::size_t imu_count = 0;
  double scan_begin_s = 0.0;
  double scan_end_s = 0.0;
  double point_offset_min_s = 0.0;
  double point_offset_max_s = 0.0;
  double imu_begin_s = 0.0;
  double imu_end_s = 0.0;
  double propagation_begin_s = 0.0;
  double propagation_end_s = 0.0;
  Eigen::Vector3d seed_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  double fixed_velocity_point_delta_mean_m = 0.0;
  double fixed_velocity_point_delta_max_m = 0.0;
};

struct ChurnMetrics {
  std::size_t previous_count = 0;
  std::size_t current_count = 0;
  std::size_t retained_count = 0;
  std::size_t added_count = 0;
  std::size_t removed_count = 0;
  std::size_t changed_plane_count = 0;
  std::size_t residual_sign_flip_count = 0;
  double retained_ratio = 0.0;
  double added_ratio = 0.0;
  double removed_ratio = 0.0;
  double voxel_jaccard = 1.0;
  double mean_normal_cosine = 1.0;
};

inline std::array<std::int64_t, 3> voxelId(const Association &value) {
  return {value.voxel_x, value.voxel_y, value.voxel_z};
}

inline ChurnMetrics correspondenceChurn(const std::vector<Association> &previous,
                                        const std::vector<Association> &current) {
  ChurnMetrics result;
  result.previous_count = previous.size();
  result.current_count = current.size();
  std::unordered_map<int, const Association *> prior_by_point;
  for (const Association &value : previous)
    prior_by_point[value.point_index] = &value;
  std::set<std::array<std::int64_t, 3>> prior_voxels;
  std::set<std::array<std::int64_t, 3>> current_voxels;
  for (const Association &value : previous) prior_voxels.insert(voxelId(value));
  double normal_cosine_sum = 0.0;
  for (const Association &value : current) {
    current_voxels.insert(voxelId(value));
    const auto found = prior_by_point.find(value.point_index);
    if (found == prior_by_point.end()) {
      ++result.added_count;
      continue;
    }
    ++result.retained_count;
    const Association &prior = *found->second;
    if (prior.plane_id != value.plane_id || voxelId(prior) != voxelId(value))
      ++result.changed_plane_count;
    const double denominator = prior.normal.norm() * value.normal.norm();
    normal_cosine_sum += denominator > 0.0
                             ? std::abs(prior.normal.dot(value.normal) / denominator)
                             : 0.0;
    if ((prior.residual < 0.0) != (value.residual < 0.0) &&
        prior.residual != 0.0 && value.residual != 0.0)
      ++result.residual_sign_flip_count;
  }
  result.removed_count = result.previous_count - result.retained_count;
  result.retained_ratio = result.previous_count
                              ? static_cast<double>(result.retained_count) /
                                    static_cast<double>(result.previous_count)
                              : (result.current_count == 0 ? 1.0 : 0.0);
  result.added_ratio = result.current_count
                           ? static_cast<double>(result.added_count) /
                                 static_cast<double>(result.current_count)
                           : 0.0;
  result.removed_ratio = result.previous_count
                             ? static_cast<double>(result.removed_count) /
                                   static_cast<double>(result.previous_count)
                             : 0.0;
  result.mean_normal_cosine = result.retained_count
                                  ? normal_cosine_sum /
                                        static_cast<double>(result.retained_count)
                                  : 1.0;
  std::size_t intersection = 0;
  for (const auto &value : prior_voxels)
    if (current_voxels.count(value)) ++intersection;
  const std::size_t union_size = prior_voxels.size() + current_voxels.size() - intersection;
  result.voxel_jaccard = union_size
                             ? static_cast<double>(intersection) /
                                   static_cast<double>(union_size)
                             : 1.0;
  return result;
}

struct SignedResidualMetrics {
  std::size_t count = 0;
  double mean = 0.0;
  double median = 0.0;
  double weighted_mean = 0.0;
  double positive_ratio = 0.0;
  double negative_ratio = 0.0;
};

inline SignedResidualMetrics signedResidualMetrics(
    const std::vector<double> &residuals,
    const std::vector<double> &inverse_variances) {
  SignedResidualMetrics result;
  if (residuals.size() != inverse_variances.size()) return result;
  std::vector<double> finite;
  double weighted_sum = 0.0;
  double weight_sum = 0.0;
  std::size_t positive = 0;
  std::size_t negative = 0;
  for (std::size_t i = 0; i < residuals.size(); ++i) {
    if (!std::isfinite(residuals[i]) || !std::isfinite(inverse_variances[i]) ||
        inverse_variances[i] < 0.0)
      continue;
    finite.push_back(residuals[i]);
    result.mean += residuals[i];
    weighted_sum += residuals[i] * inverse_variances[i];
    weight_sum += inverse_variances[i];
    positive += residuals[i] > 0.0;
    negative += residuals[i] < 0.0;
  }
  result.count = finite.size();
  if (!result.count) return result;
  result.mean /= static_cast<double>(result.count);
  std::sort(finite.begin(), finite.end());
  const std::size_t middle = finite.size() / 2;
  result.median = finite.size() % 2
                      ? finite[middle]
                      : 0.5 * (finite[middle - 1] + finite[middle]);
  result.weighted_mean = weight_sum > 0.0 ? weighted_sum / weight_sum : 0.0;
  result.positive_ratio = static_cast<double>(positive) / result.count;
  result.negative_ratio = static_cast<double>(negative) / result.count;
  return result;
}

struct NormalGeometryMetrics {
  bool valid = false;
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
  double directional_entropy = 0.0;
  double spherical_coverage = 0.0;
};

inline NormalGeometryMetrics normalGeometry(
    const std::vector<Eigen::Vector3d> &normals) {
  NormalGeometryMetrics result;
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  std::array<bool, 8> octants{};
  std::size_t count = 0;
  for (Eigen::Vector3d normal : normals) {
    if (!normal.allFinite() || normal.norm() < 1e-12) continue;
    normal.normalize();
    scatter += normal * normal.transpose();
    const int octant = (normal.x() >= 0.0 ? 4 : 0) |
                       (normal.y() >= 0.0 ? 2 : 0) |
                       (normal.z() >= 0.0 ? 1 : 0);
    octants[octant] = true;
    ++count;
  }
  if (!count) return result;
  scatter /= static_cast<double>(count);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() != Eigen::Success) return result;
  result.eigenvalues = solver.eigenvalues();
  const double sum = result.eigenvalues.sum();
  if (sum <= 0.0) return result;
  for (int i = 0; i < result.eigenvalues.size(); ++i) {
    const double value = result.eigenvalues[i];
    const double probability = std::max(0.0, value / sum);
    if (probability > 0.0)
      result.directional_entropy -= probability * std::log(probability);
  }
  result.directional_entropy /= std::log(3.0);
  result.spherical_coverage =
      static_cast<double>(std::count(octants.begin(), octants.end(), true)) / 8.0;
  result.valid = result.eigenvalues.allFinite();
  return result;
}

struct AgeMetrics {
  std::array<std::size_t, 5> support_points{{0, 0, 0, 0, 0}};
  std::size_t total_support_points = 0;
  std::array<double, 3> recent_frame_reuse{{0.0, 0.0, 0.0}};
};

inline std::size_t ageBin(double age_seconds) {
  if (age_seconds < 0.5) return 0;
  if (age_seconds < 2.0) return 1;
  if (age_seconds < 5.0) return 2;
  if (age_seconds < 20.0) return 3;
  return 4;
}

inline bool mapInsertionAllowed(bool diagnostics_enabled,
                                bool frozen_map_variant,
                                double relative_time,
                                double freeze_start) {
  return !(diagnostics_enabled && frozen_map_variant &&
           std::isfinite(relative_time) && std::isfinite(freeze_start) &&
           relative_time >= freeze_start);
}

inline std::array<double, 3> recentReuseRatios(
    const std::vector<int> &last_update_frames, int current_frame) {
  std::array<double, 3> result{{0.0, 0.0, 0.0}};
  if (last_update_frames.empty()) return result;
  constexpr std::array<int, 3> windows{{1, 3, 5}};
  for (int updated : last_update_frames) {
    const int age = current_frame - updated;
    for (std::size_t i = 0; i < windows.size(); ++i)
      result[i] += updated >= 0 && age > 0 && age <= windows[i];
  }
  for (double &value : result)
    value /= static_cast<double>(last_update_frames.size());
  return result;
}

struct FixedCorrespondenceResult {
  bool valid = false;
  Eigen::Matrix<double, 6, 1> delta =
      Eigen::Matrix<double, 6, 1>::Zero();
  int rank = 0;
};

inline FixedCorrespondenceResult solveFixedCorrespondences(
    const Eigen::MatrixXd &jacobian, const Eigen::VectorXd &residual,
    const Eigen::VectorXd &inverse_variance) {
  FixedCorrespondenceResult result;
  if (jacobian.cols() != 6 || jacobian.rows() != residual.size() ||
      residual.size() != inverse_variance.size() || residual.size() == 0 ||
      !jacobian.allFinite() || !residual.allFinite() ||
      !inverse_variance.allFinite() || inverse_variance.minCoeff() < 0.0)
    return result;
  const Eigen::MatrixXd weighted = inverse_variance.asDiagonal() * jacobian;
  const Eigen::Matrix<double, 6, 6> information =
      jacobian.transpose() * weighted;
  const Eigen::Matrix<double, 6, 1> rhs =
      -jacobian.transpose() * inverse_variance.asDiagonal() * residual;
  Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 6, 6>> solve(
      information);
  result.rank = solve.rank();
  result.delta = solve.solve(rhs);
  result.valid = result.delta.allFinite();
  return result;
}

inline double signedResidualWeakProjection(
    const std::vector<double> &residuals,
    const std::vector<Eigen::Vector3d> &normals,
    const Eigen::Vector3d &weak_direction) {
  if (residuals.size() != normals.size() || residuals.empty() ||
      !weak_direction.allFinite() || weak_direction.norm() < 1e-12)
    return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector3d weak = weak_direction.normalized();
  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t i = 0; i < residuals.size(); ++i) {
    if (!std::isfinite(residuals[i]) || !normals[i].allFinite()) continue;
    sum += residuals[i] * normals[i].dot(weak);
    ++count;
  }
  return count ? sum / static_cast<double>(count)
               : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace p4
}  // namespace fast_livo
