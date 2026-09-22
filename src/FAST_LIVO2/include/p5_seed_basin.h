#pragma once

#include "p4_frontend_diagnostics.h"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>
#include <vector>

namespace fast_livo {
namespace p5 {

inline constexpr std::array<double, 5> kDiagnosticSeedAlphas{
    {0.0, -0.5, 0.5, -1.0, 1.0}};
inline constexpr std::array<double, 9> kProfileAlphas{
    {-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0}};

inline double posePriorMahalanobis(
    const Eigen::Matrix<double, 6, 1> &pose_delta,
    const Eigen::Matrix<double, 6, 6> &pose_covariance) {
  if (!pose_delta.allFinite() || !pose_covariance.allFinite())
    return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Matrix<double, 6, 6> symmetric =
      0.5 * (pose_covariance + pose_covariance.transpose());
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solve(symmetric);
  if (solve.info() != Eigen::Success || !solve.vectorD().allFinite() ||
      solve.vectorD().minCoeff() <= 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Matrix<double, 6, 1> whitened = solve.solve(pose_delta);
  const double value = pose_delta.dot(whitened);
  return std::isfinite(value) && value >= -1e-10 ? std::max(0.0, value)
                                                  : std::numeric_limits<double>::quiet_NaN();
}

inline double associationJaccard(
    const std::vector<fast_livo::p4::Association> &a,
    const std::vector<fast_livo::p4::Association> &b) {
  using Key = std::tuple<int, int, std::int64_t, std::int64_t, std::int64_t>;
  std::set<Key> left;
  std::set<Key> right;
  for (const auto &value : a)
    left.emplace(value.point_index, value.plane_id, value.voxel_x,
                 value.voxel_y, value.voxel_z);
  for (const auto &value : b)
    right.emplace(value.point_index, value.plane_id, value.voxel_x,
                  value.voxel_y, value.voxel_z);
  std::size_t intersection = 0;
  for (const Key &value : left)
    intersection += right.count(value);
  const std::size_t union_size = left.size() + right.size() - intersection;
  return union_size ? static_cast<double>(intersection) /
                          static_cast<double>(union_size)
                    : 1.0;
}

inline std::size_t selectJointMapCandidate(
    const std::array<double, 3> &lidar_cost,
    const std::array<double, 3> &prior_cost,
    const std::array<bool, 3> &committed) {
  std::size_t selected = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < 3; ++i) {
    const double total = lidar_cost[i] + prior_cost[i];
    if (committed[i] && std::isfinite(total) && total < best) {
      selected = i;
      best = total;
    }
  }
  return selected;
}

}  // namespace p5
}  // namespace fast_livo
