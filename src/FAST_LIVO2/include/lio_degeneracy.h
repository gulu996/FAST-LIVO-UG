#ifndef FAST_LIVO_LIO_DEGENERACY_H
#define FAST_LIVO_LIO_DEGENERACY_H

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fast_livo
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct LioObservabilityMetrics
{
  bool valid = false;
  Eigen::Vector3d rotation_eigenvalues = Eigen::Vector3d::Zero();
  Eigen::Vector3d translation_eigenvalues = Eigen::Vector3d::Zero();
  Eigen::Matrix3d translation_eigenvectors = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d conditional_translation_information = Eigen::Matrix3d::Zero();
  Eigen::Vector3d weak_translation_direction_world = Eigen::Vector3d::UnitX();
  double translation_eigenvalue_ratio = 0.0;
  double translation_condition_number = std::numeric_limits<double>::infinity();
};

inline LioObservabilityMetrics analyzeLioPoseInformation(
    const Matrix6d &pose_information,
    double rotation_regularization,
    bool use_conditional_translation_information)
{
  LioObservabilityMetrics result;
  if (!pose_information.allFinite()) return result;

  // StatesGroup is [right SO(3), world translation, exposure, velocity, ...].
  // The corresponding point-to-plane Jacobian uses the same first-six order.
  const Matrix6d information = 0.5 * (pose_information + pose_information.transpose());
  if (!information.allFinite()) return result;
  const Eigen::Matrix3d h_rr = information.block<3, 3>(0, 0);
  const Eigen::Matrix3d h_rt = information.block<3, 3>(0, 3);
  const Eigen::Matrix3d h_tr = information.block<3, 3>(3, 0);
  const Eigen::Matrix3d h_tt = information.block<3, 3>(3, 3);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_solver(h_rr);
  const double rotation_negative_tolerance =
      1e-10 * std::max(1.0, h_rr.cwiseAbs().maxCoeff());
  if (rotation_solver.info() != Eigen::Success) return result;
  if (!rotation_solver.eigenvalues().allFinite() ||
      rotation_solver.eigenvalues()[0] < -rotation_negative_tolerance)
    return result;

  Eigen::Matrix3d translation_information = h_tt;
  if (use_conditional_translation_information)
  {
    const Eigen::Matrix3d regularized_rotation =
        h_rr + Eigen::Matrix3d::Identity() * std::max(rotation_regularization, 1e-12);
    const Eigen::LDLT<Eigen::Matrix3d> rotation_ldlt(regularized_rotation);
    if (rotation_ldlt.info() != Eigen::Success || !rotation_ldlt.vectorD().allFinite() ||
        rotation_ldlt.vectorD().minCoeff() <= 0.0)
      return result;
    const Eigen::Matrix3d rotation_to_translation = rotation_ldlt.solve(h_rt);
    if (!rotation_to_translation.allFinite()) return result;
    translation_information -= h_tr * rotation_to_translation;
  }
  translation_information = 0.5 * (translation_information + translation_information.transpose());
  if (!translation_information.allFinite()) return result;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> translation_solver(translation_information);
  const double translation_negative_tolerance =
      1e-10 * std::max(1.0, translation_information.cwiseAbs().maxCoeff());
  if (translation_solver.info() != Eigen::Success) return result;
  if (!translation_solver.eigenvalues().allFinite() ||
      translation_solver.eigenvalues()[0] < -translation_negative_tolerance)
    return result;

  result.rotation_eigenvalues = rotation_solver.eigenvalues().cwiseMax(0.0);
  result.translation_eigenvalues = translation_solver.eigenvalues().cwiseMax(0.0);
  result.translation_eigenvectors = translation_solver.eigenvectors();
  result.conditional_translation_information = translation_information;
  const double weak_direction_norm = result.translation_eigenvectors.col(0).norm();
  if (!std::isfinite(weak_direction_norm) || weak_direction_norm <= 1e-12) return result;
  result.weak_translation_direction_world =
      result.translation_eigenvectors.col(0) / weak_direction_norm;

  const double lambda_min = result.translation_eigenvalues[0];
  const double lambda_max = result.translation_eigenvalues[2];
  if (lambda_max > 1e-12)
  {
    result.translation_eigenvalue_ratio = lambda_min / lambda_max;
    result.translation_condition_number =
        lambda_min > 1e-12 ? lambda_max / lambda_min : std::numeric_limits<double>::infinity();
  }
  result.valid = result.rotation_eigenvalues.allFinite() &&
                 result.translation_eigenvalues.allFinite() &&
                 result.weak_translation_direction_world.allFinite();
  return result;
}

inline Eigen::Vector3d lioTranslationInformationWeights(
    const LioObservabilityMetrics &metrics,
    double damping_eigenvalue)
{
  Eigen::Vector3d weights = Eigen::Vector3d::Ones();
  if (!metrics.valid || damping_eigenvalue <= 0.0) return weights;
  for (int i = 0; i < 3; ++i)
  {
    const double eigenvalue = std::max(0.0, metrics.translation_eigenvalues[i]);
    weights[i] = eigenvalue / (eigenvalue + damping_eigenvalue);
  }
  return weights;
}

struct LioNormalEquation
{
  Matrix6d information = Matrix6d::Zero();
  Eigen::Matrix<double, 6, 1> rhs = Eigen::Matrix<double, 6, 1>::Zero();
};

// EXPERIMENTAL: retained for offline research only. The stable runtime path
// does not call this information-matrix reconstruction.
inline LioNormalEquation applyLioConditionalTranslationWeights(
    const Matrix6d &pose_information,
    const Eigen::Matrix<double, 6, 1> &pose_rhs,
    const LioObservabilityMetrics &metrics,
    const Eigen::Vector3d &translation_weights,
    double rotation_regularization)
{
  LioNormalEquation result{pose_information, pose_rhs};
  if (!metrics.valid || !pose_information.allFinite() || !pose_rhs.allFinite() ||
      !translation_weights.allFinite())
    return result;

  const Matrix6d information = 0.5 * (pose_information + pose_information.transpose());
  const Eigen::Matrix3d h_rr = information.block<3, 3>(0, 0);
  const Eigen::Matrix3d h_rt = information.block<3, 3>(0, 3);
  const Eigen::Matrix3d h_tr = information.block<3, 3>(3, 0);
  const Eigen::Matrix3d h_tt = information.block<3, 3>(3, 3);
  const Eigen::Matrix3d regularized_rotation =
      h_rr + Eigen::Matrix3d::Identity() * std::max(rotation_regularization, 1e-12);
  const Eigen::LDLT<Eigen::Matrix3d> rotation_ldlt(regularized_rotation);
  if (rotation_ldlt.info() != Eigen::Success || !rotation_ldlt.vectorD().allFinite() ||
      rotation_ldlt.vectorD().minCoeff() <= 0.0)
    return result;

  const Eigen::Matrix3d rotation_to_translation = rotation_ldlt.solve(h_rt);
  if (!rotation_to_translation.allFinite()) return result;
  Eigen::Matrix3d conditional_information =
      h_tt - h_tr * rotation_to_translation;
  conditional_information =
      0.5 * (conditional_information + conditional_information.transpose());

  const Eigen::Vector3d weights = translation_weights.cwiseMax(0.0).cwiseMin(1.0);
  const Eigen::Matrix3d weight_projection =
      metrics.translation_eigenvectors * weights.asDiagonal() *
      metrics.translation_eigenvectors.transpose();
  const Eigen::Matrix3d sqrt_weight_projection =
      metrics.translation_eigenvectors * weights.cwiseSqrt().asDiagonal() *
      metrics.translation_eigenvectors.transpose();
  const Eigen::Matrix3d weighted_conditional_information =
      sqrt_weight_projection * conditional_information * sqrt_weight_projection;

  result.information.block<3, 3>(0, 0) = h_rr;
  result.information.block<3, 3>(0, 3) = h_rt;
  result.information.block<3, 3>(3, 0) = h_tr;
  result.information.block<3, 3>(3, 3) =
      h_tr * rotation_to_translation + weighted_conditional_information;
  result.information = 0.5 * (result.information + result.information.transpose()).eval();

  const Eigen::Vector3d rotation_rhs_solution = rotation_ldlt.solve(pose_rhs.head<3>());
  const Eigen::Vector3d conditional_rhs =
      pose_rhs.tail<3>() - h_tr * rotation_rhs_solution;
  result.rhs.tail<3>() =
      h_tr * rotation_rhs_solution + weight_projection * conditional_rhs;
  return result;
}

} // namespace fast_livo

#endif
