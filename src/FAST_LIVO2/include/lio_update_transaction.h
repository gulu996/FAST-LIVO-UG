#ifndef FAST_LIVO_LIO_UPDATE_TRANSACTION_H
#define FAST_LIVO_LIO_UPDATE_TRANSACTION_H

#include "common_lib.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fast_livo
{

enum class LioCommitStatus
{
  CONVERGED,
  MAX_ITER_HEALTHY,
  NOT_CONVERGED_HEALTHY,
  REJECTED_INVALID,
  REJECTED_COST,
  REJECTED_NUMERICAL
};

inline const char *lioCommitStatusName(LioCommitStatus status)
{
  switch (status)
  {
    case LioCommitStatus::CONVERGED: return "converged";
    case LioCommitStatus::MAX_ITER_HEALTHY: return "max_iter_healthy";
    case LioCommitStatus::NOT_CONVERGED_HEALTHY: return "not_converged_but_healthy";
    case LioCommitStatus::REJECTED_INVALID: return "rejected_invalid";
    case LioCommitStatus::REJECTED_COST: return "rejected_cost";
    case LioCommitStatus::REJECTED_NUMERICAL: return "rejected_numerical";
  }
  return "rejected_invalid";
}

struct LioValidationLimits
{
  int minimum_correspondences = 1;
  double covariance_symmetry_relative_tolerance = 1e-7;
  double covariance_psd_relative_tolerance = 1e-10;
  double rotation_orthogonality_tolerance = 1e-5;
  double rotation_determinant_tolerance = 1e-5;
  double maximum_cost_ratio = 2.0;
  double maximum_cost_increase = 0.20;
  double maximum_residual = 5.0;
  double maximum_translation_increment = std::numeric_limits<double>::infinity();
  double maximum_rotation_increment_deg = std::numeric_limits<double>::infinity();
};

struct LioCandidateMetrics
{
  bool numerical_failure = false;
  bool posterior_ready = false;
  bool converged = false;
  bool reached_iteration_limit = false;
  int correspondence_count = 0;
  int valid_residual_count = 0;
  double cost_before = std::numeric_limits<double>::quiet_NaN();
  double cost_after = std::numeric_limits<double>::quiet_NaN();
  double maximum_abs_residual = std::numeric_limits<double>::quiet_NaN();
  double translation_increment_norm = std::numeric_limits<double>::quiet_NaN();
  double rotation_increment_deg = std::numeric_limits<double>::quiet_NaN();
  double velocity_increment_norm = std::numeric_limits<double>::quiet_NaN();
  bool state_increment_finite = false;
};

struct LioValidationResult
{
  bool commit = false;
  LioCommitStatus status = LioCommitStatus::REJECTED_INVALID;
  double covariance_asymmetry = std::numeric_limits<double>::quiet_NaN();
  double covariance_min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double covariance_trace = std::numeric_limits<double>::quiet_NaN();
  double covariance_min_diagonal = std::numeric_limits<double>::quiet_NaN();
  double covariance_max_diagonal = std::numeric_limits<double>::quiet_NaN();
};

inline bool lioStateFinite(const StatesGroup &state)
{
  return state.rot_end.allFinite() && state.pos_end.allFinite() &&
         state.vel_end.allFinite() && state.bias_g.allFinite() &&
         state.bias_a.allFinite() && state.gravity.allFinite() &&
         std::isfinite(state.inv_expo_time) && state.cov.allFinite();
}

inline LioValidationResult validateLioCandidate(
    const StatesGroup &candidate, const LioCandidateMetrics &metrics,
    const LioValidationLimits &limits)
{
  LioValidationResult result;
  if (candidate.cov.allFinite())
  {
    result.covariance_trace = candidate.cov.trace();
    result.covariance_min_diagonal = candidate.cov.diagonal().minCoeff();
    result.covariance_max_diagonal = candidate.cov.diagonal().maxCoeff();
    result.covariance_asymmetry =
        (candidate.cov - candidate.cov.transpose()).cwiseAbs().maxCoeff();
  }

  if (metrics.numerical_failure || !metrics.state_increment_finite ||
      !lioStateFinite(candidate))
  {
    result.status = LioCommitStatus::REJECTED_NUMERICAL;
    return result;
  }

  const double covariance_scale = std::max(1.0, candidate.cov.cwiseAbs().maxCoeff());
  if (result.covariance_asymmetry >
      limits.covariance_symmetry_relative_tolerance * covariance_scale)
  {
    result.status = LioCommitStatus::REJECTED_NUMERICAL;
    return result;
  }
  const MD(DIM_STATE, DIM_STATE) symmetric_covariance =
      0.5 * (candidate.cov + candidate.cov.transpose()).eval();
  Eigen::SelfAdjointEigenSolver<MD(DIM_STATE, DIM_STATE)> covariance_solver(
      symmetric_covariance, Eigen::EigenvaluesOnly);
  if (covariance_solver.info() != Eigen::Success ||
      !covariance_solver.eigenvalues().allFinite())
  {
    result.status = LioCommitStatus::REJECTED_NUMERICAL;
    return result;
  }
  result.covariance_min_eigenvalue = covariance_solver.eigenvalues().minCoeff();
  if (result.covariance_min_eigenvalue <
      -limits.covariance_psd_relative_tolerance * covariance_scale)
  {
    result.status = LioCommitStatus::REJECTED_NUMERICAL;
    return result;
  }

  const M3D rotation_error = candidate.rot_end.transpose() * candidate.rot_end - M3D::Identity();
  if (rotation_error.cwiseAbs().maxCoeff() > limits.rotation_orthogonality_tolerance ||
      std::fabs(candidate.rot_end.determinant() - 1.0) >
          limits.rotation_determinant_tolerance)
  {
    result.status = LioCommitStatus::REJECTED_NUMERICAL;
    return result;
  }

  if (!metrics.posterior_ready ||
      metrics.correspondence_count < limits.minimum_correspondences ||
      metrics.valid_residual_count < limits.minimum_correspondences)
  {
    result.status = LioCommitStatus::REJECTED_INVALID;
    return result;
  }

  if (!std::isfinite(metrics.cost_before) ||
      !std::isfinite(metrics.cost_after) ||
      !std::isfinite(metrics.maximum_abs_residual))
  {
    result.status = LioCommitStatus::REJECTED_NUMERICAL;
    return result;
  }

  if (
      metrics.translation_increment_norm > limits.maximum_translation_increment ||
      metrics.rotation_increment_deg > limits.maximum_rotation_increment_deg)
  {
    result.status = LioCommitStatus::REJECTED_INVALID;
    return result;
  }

  const double allowed_cost = std::min(
      metrics.cost_before * limits.maximum_cost_ratio,
      metrics.cost_before + limits.maximum_cost_increase);
  if (metrics.cost_after > allowed_cost ||
      metrics.maximum_abs_residual > limits.maximum_residual)
  {
    result.status = LioCommitStatus::REJECTED_COST;
    return result;
  }

  result.commit = true;
  result.status = metrics.converged
      ? LioCommitStatus::CONVERGED
      : (metrics.reached_iteration_limit
             ? LioCommitStatus::MAX_ITER_HEALTHY
             : LioCommitStatus::NOT_CONVERGED_HEALTHY);
  return result;
}

inline StatesGroup committedLioState(const StatesGroup &prior,
                                     const StatesGroup &candidate,
                                     const LioValidationResult &validation)
{
  return validation.commit ? candidate : prior;
}

inline bool shouldInsertLioMap(bool lio_committed, bool scheduled,
                               bool guarded)
{
  return lio_committed && scheduled && !guarded;
}

}  // namespace fast_livo

#endif
