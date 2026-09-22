#ifndef FAST_LIVO_LIO_MOTION_CONSISTENCY_H
#define FAST_LIVO_LIO_MOTION_CONSISTENCY_H

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>

namespace fast_livo
{

inline double motionDiagnosticNaN()
{
  return std::numeric_limits<double>::quiet_NaN();
}

struct NormalizedQuadratic
{
  bool valid = false;
  double value = motionDiagnosticNaN();
  int rank = 0;
  double minimum_eigenvalue = motionDiagnosticNaN();
  double maximum_eigenvalue = motionDiagnosticNaN();
  double condition_number = motionDiagnosticNaN();
  double eigenvalue_cutoff = motionDiagnosticNaN();
};

// Symmetric eigensolve avoids an explicit inverse. Positive-definite priors
// require full rank; correction covariances use the Moore-Penrose solution on
// eigenvalues above max(1e-15, 1e-12 * max(1, max_abs_eigenvalue)). The
// absolute 1e-12 floor intentionally rejects numerically empty covariance
// directions in this state representation.
inline NormalizedQuadratic normalizedQuadratic(
    const Eigen::MatrixXd &covariance,
    const Eigen::VectorXd &correction,
    bool allow_semidefinite = false)
{
  NormalizedQuadratic result;
  if (covariance.rows() == 0 || covariance.rows() != covariance.cols() ||
      correction.size() != covariance.rows() || !covariance.allFinite() ||
      !correction.allFinite())
    return result;

  const Eigen::MatrixXd symmetric =
      0.5 * (covariance + covariance.transpose()).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
    return result;

  const Eigen::VectorXd eigenvalues = solver.eigenvalues();
  const double scale = std::max(1.0, eigenvalues.cwiseAbs().maxCoeff());
  const double cutoff = std::max(1e-15, 1e-12 * scale);
  result.minimum_eigenvalue = eigenvalues.minCoeff();
  result.maximum_eigenvalue = eigenvalues.maxCoeff();
  result.eigenvalue_cutoff = cutoff;
  if (result.minimum_eigenvalue < -cutoff) return result;

  const Eigen::VectorXd projected = solver.eigenvectors().transpose() * correction;
  double value = 0.0;
  double minimum_used = std::numeric_limits<double>::infinity();
  for (int i = 0; i < eigenvalues.size(); ++i)
  {
    if (eigenvalues[i] <= cutoff) continue;
    value += projected[i] * projected[i] / eigenvalues[i];
    minimum_used = std::min(minimum_used, eigenvalues[i]);
    ++result.rank;
  }
  if (result.rank == 0 || (!allow_semidefinite && result.rank != covariance.rows()))
    return result;

  result.value = value;
  result.condition_number = result.maximum_eigenvalue / minimum_used;
  result.valid = std::isfinite(value) && value >= 0.0 &&
                 std::isfinite(result.condition_number);
  return result;
}

struct CrossBlockSummary
{
  double frobenius_norm = motionDiagnosticNaN();
  double maximum_singular_value = motionDiagnosticNaN();
  Eigen::Vector3d state_direction =
      Eigen::Vector3d::Constant(motionDiagnosticNaN());
  Eigen::Vector3d coupled_direction =
      Eigen::Vector3d::Constant(motionDiagnosticNaN());
};

inline CrossBlockSummary summarizeCrossBlock(const Eigen::Matrix3d &block)
{
  CrossBlockSummary result;
  if (!block.allFinite()) return result;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      block, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.singularValues().allFinite() || !svd.matrixU().allFinite() ||
      !svd.matrixV().allFinite())
    return result;
  result.frobenius_norm = block.norm();
  result.maximum_singular_value = svd.singularValues()[0];
  result.state_direction = svd.matrixU().col(0);
  result.coupled_direction = svd.matrixV().col(0);
  return result;
}

struct RollingCorrectionMetrics
{
  Eigen::Vector3d cumulative_dv_025 = Eigen::Vector3d::Zero();
  Eigen::Vector3d cumulative_dv_050 = Eigen::Vector3d::Zero();
  Eigen::Vector3d cumulative_dv_100 = Eigen::Vector3d::Zero();
  double consecutive_direction_cosine = motionDiagnosticNaN();
  double direction_persistence_1s = motionDiagnosticNaN();
  double dv_velocity_cosine = motionDiagnosticNaN();
  double dv_velocity_angle_deg = motionDiagnosticNaN();
  double dv_dp_cosine = motionDiagnosticNaN();
  double dv_dp_angle_deg = motionDiagnosticNaN();
  double dv_dp_norm_ratio = motionDiagnosticNaN();
};

struct PoseDirectionCorrection
{
  bool valid = false;
  Eigen::Matrix<double, 6, 1> eigenvalues =
      Eigen::Matrix<double, 6, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 6, 6> eigenvectors =
      Eigen::Matrix<double, 6, 6>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 19, 6> state_contributions =
      Eigen::Matrix<double, 19, 6>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 19, 1> reconstructed_correction =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 19, 1> unapplied_iteration_carry =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  double full_closure_norm = motionDiagnosticNaN();
  double velocity_closure_norm = motionDiagnosticNaN();
  double applied_full_closure_norm = motionDiagnosticNaN();
  double applied_velocity_closure_norm = motionDiagnosticNaN();
};

struct SourceUpdateCounterfactual
{
  bool valid = false;
  double strength = 0.0;
  Eigen::Matrix<double, 6, 1> dimensionless_information_eigenvalues =
      Eigen::Matrix<double, 6, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 6, 1> direction_weights =
      Eigen::Matrix<double, 6, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 19, 1> delta_state =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  double pose_difference_norm = motionDiagnosticNaN();
  double velocity_difference_norm = motionDiagnosticNaN();
  double localized_prior_min_eigenvalue = motionDiagnosticNaN();
  double localized_prior_asymmetry = motionDiagnosticNaN();
  double posterior_min_eigenvalue = motionDiagnosticNaN();
  double posterior_asymmetry = motionDiagnosticNaN();
  double posterior_trace = motionDiagnosticNaN();
  double pose_posterior_covariance_difference_norm = motionDiagnosticNaN();
};

inline double symmetricMinimumEigenvalue(const Eigen::Matrix<double, 19, 19> &matrix)
{
  if (!matrix.allFinite()) return motionDiagnosticNaN();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 19, 19>> solver(
      0.5 * (matrix + matrix.transpose()).eval(), Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success && solver.eigenvalues().allFinite()
      ? solver.eigenvalues().minCoeff() : motionDiagnosticNaN();
}

inline PoseDirectionCorrection decomposePoseDirectionCorrection(
    const Eigen::Matrix<double, 19, 6> &normal_rhs_gain,
    const Eigen::Matrix<double, 6, 6> &pose_information,
    const Eigen::Matrix<double, 6, 1> &pose_rhs,
    const Eigen::Matrix<double, 19, 1> &iteration_prior_offset,
    const Eigen::Matrix<double, 19, 1> &applied_delta_state,
    double step_scale)
{
  PoseDirectionCorrection result;
  if (!normal_rhs_gain.allFinite() || !pose_information.allFinite() ||
      !pose_rhs.allFinite() || !iteration_prior_offset.allFinite() ||
      !applied_delta_state.allFinite() || !std::isfinite(step_scale) ||
      step_scale < 0.0 || step_scale > 1.0)
    return result;
  const Eigen::Matrix<double, 6, 6> information =
      0.5 * (pose_information + pose_information.transpose()).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite())
    return result;

  const Eigen::Matrix<double, 6, 1> effective_rhs =
      pose_rhs - information * iteration_prior_offset.head<6>();
  const Eigen::Matrix<double, 19, 1> exact = normal_rhs_gain * effective_rhs;
  result.eigenvalues = solver.eigenvalues();
  result.eigenvectors = solver.eigenvectors();
  result.state_contributions.setZero();
  for (int direction = 0; direction < 6; ++direction)
  {
    const Eigen::Matrix<double, 6, 1> axis = result.eigenvectors.col(direction);
    result.state_contributions.col(direction) = step_scale *
        normal_rhs_gain * axis * axis.dot(effective_rhs);
  }
  result.reconstructed_correction = result.state_contributions.rowwise().sum();
  result.full_closure_norm =
      (step_scale * exact - result.reconstructed_correction).norm();
  result.velocity_closure_norm =
      (step_scale * exact.segment<3>(7) -
       result.reconstructed_correction.segment<3>(7)).norm();
  result.unapplied_iteration_carry =
      (1.0 - step_scale) * (-iteration_prior_offset);
  const Eigen::Matrix<double, 19, 1> reconstructed_applied =
      result.reconstructed_correction + result.unapplied_iteration_carry;
  result.applied_full_closure_norm =
      (applied_delta_state - reconstructed_applied).norm();
  result.applied_velocity_closure_norm =
      (applied_delta_state.segment<3>(7) -
       reconstructed_applied.segment<3>(7)).norm();
  result.valid = exact.allFinite() && result.reconstructed_correction.allFinite() &&
                 std::isfinite(result.full_closure_norm) &&
                 std::isfinite(result.velocity_closure_norm);
  return result;
}

inline SourceUpdateCounterfactual observabilityAwareTransferCounterfactual(
    const Eigen::Matrix<double, 19, 19> &prior_covariance,
    const Eigen::Matrix<double, 6, 6> &pose_information,
    const Eigen::Matrix<double, 6, 1> &pose_rhs,
    const Eigen::Matrix<double, 19, 1> &iteration_prior_offset,
    const Eigen::Matrix<double, 19, 1> &raw_delta_state,
    const Eigen::Matrix<double, 19, 19> &raw_posterior_covariance,
    double step_scale,
    double strength)
{
  SourceUpdateCounterfactual result;
  result.strength = strength;
  if (!prior_covariance.allFinite() || !pose_information.allFinite() ||
      !pose_rhs.allFinite() || !iteration_prior_offset.allFinite() ||
      !raw_delta_state.allFinite() || !raw_posterior_covariance.allFinite() ||
      !std::isfinite(step_scale) || step_scale < 0.0 ||
      step_scale > 1.0 || !std::isfinite(strength) || strength < 0.0)
    return result;

  const Eigen::Matrix<double, 19, 19> prior =
      0.5 * (prior_covariance + prior_covariance.transpose()).eval();
  const Eigen::Matrix<double, 6, 6> pose_prior = prior.block<6, 6>(0, 0);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> prior_solver(pose_prior);
  if (prior_solver.info() != Eigen::Success || !prior_solver.eigenvalues().allFinite() ||
      prior_solver.eigenvalues().minCoeff() <= 0.0)
    return result;
  const Eigen::Matrix<double, 6, 1> prior_sqrt_values =
      prior_solver.eigenvalues().cwiseSqrt();
  const Eigen::Matrix<double, 6, 6> prior_sqrt =
      prior_solver.eigenvectors() * prior_sqrt_values.asDiagonal() *
      prior_solver.eigenvectors().transpose();
  const Eigen::Matrix<double, 6, 6> prior_inverse_sqrt =
      prior_solver.eigenvectors() * prior_sqrt_values.cwiseInverse().asDiagonal() *
      prior_solver.eigenvectors().transpose();
  const Eigen::Matrix<double, 6, 6> dimensionless_information =
      0.5 * (prior_sqrt * pose_information * prior_sqrt +
             prior_sqrt * pose_information.transpose() * prior_sqrt).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> information_solver(
      dimensionless_information);
  if (information_solver.info() != Eigen::Success ||
      !information_solver.eigenvalues().allFinite())
    return result;
  result.dimensionless_information_eigenvalues =
      information_solver.eigenvalues().cwiseMax(0.0);
  for (int i = 0; i < 6; ++i)
  {
    const double value = result.dimensionless_information_eigenvalues[i];
    const double confidence = value / (1.0 + value);
    result.direction_weights[i] = strength == 0.0 ? 1.0 : std::pow(confidence, strength);
  }
  const Eigen::Matrix<double, 6, 6> contraction =
      information_solver.eigenvectors() * result.direction_weights.asDiagonal() *
      information_solver.eigenvectors().transpose();
  Eigen::Matrix<double, 19, 19> localized_prior = prior;
  const Eigen::Matrix<double, 13, 6> localized_cross =
      prior.block<13, 6>(6, 0) * prior_inverse_sqrt * contraction * prior_sqrt;
  localized_prior.block<13, 6>(6, 0) = localized_cross;
  localized_prior.block<6, 13>(0, 6) = localized_cross.transpose();
  localized_prior = 0.5 * (localized_prior + localized_prior.transpose()).eval();

  Eigen::Matrix<double, 19, 19> information_full =
      localized_prior.inverse();
  information_full.block<6, 6>(0, 0) += pose_information;
  const Eigen::Matrix<double, 19, 19> posterior = information_full.inverse();
  const Eigen::Matrix<double, 6, 1> effective_rhs =
      pose_rhs - pose_information * iteration_prior_offset.head<6>();
  const Eigen::Matrix<double, 19, 1> target_delta =
      posterior.block<19, 6>(0, 0) * effective_rhs;
  result.delta_state = step_scale * target_delta +
      (1.0 - step_scale) * (-iteration_prior_offset);
  result.pose_difference_norm =
      (result.delta_state.head<6>() - raw_delta_state.head<6>()).norm();
  result.velocity_difference_norm =
      (result.delta_state.segment<3>(7) - raw_delta_state.segment<3>(7)).norm();
  result.localized_prior_min_eigenvalue = symmetricMinimumEigenvalue(localized_prior);
  result.localized_prior_asymmetry =
      (localized_prior - localized_prior.transpose()).cwiseAbs().maxCoeff();
  result.posterior_min_eigenvalue = symmetricMinimumEigenvalue(posterior);
  result.posterior_asymmetry =
      (posterior - posterior.transpose()).cwiseAbs().maxCoeff();
  result.posterior_trace = posterior.trace();
  result.pose_posterior_covariance_difference_norm =
      (posterior.block<6, 6>(0, 0) -
       raw_posterior_covariance.block<6, 6>(0, 0)).norm();
  result.valid = result.direction_weights.allFinite() && result.delta_state.allFinite() &&
                 std::isfinite(result.localized_prior_min_eigenvalue) &&
                 std::isfinite(result.posterior_min_eigenvalue);
  return result;
}

inline SourceUpdateCounterfactual priorMetricDampingCounterfactual(
    const Eigen::Matrix<double, 19, 19> &prior_covariance,
    const Eigen::Matrix<double, 6, 6> &pose_information,
    const Eigen::Matrix<double, 6, 1> &pose_rhs,
    const Eigen::Matrix<double, 19, 1> &iteration_prior_offset,
    const Eigen::Matrix<double, 19, 1> &raw_delta_state,
    const Eigen::Matrix<double, 19, 19> &raw_posterior_covariance,
    double step_scale,
    double damping)
{
  SourceUpdateCounterfactual result;
  result.strength = damping;
  if (!prior_covariance.allFinite() || !pose_information.allFinite() ||
      !pose_rhs.allFinite() || !iteration_prior_offset.allFinite() ||
      !raw_delta_state.allFinite() || !raw_posterior_covariance.allFinite() ||
      !std::isfinite(step_scale) || step_scale < 0.0 ||
      step_scale > 1.0 || !std::isfinite(damping) || damping < 0.0)
    return result;
  const Eigen::Matrix<double, 19, 19> prior =
      0.5 * (prior_covariance + prior_covariance.transpose()).eval();
  Eigen::Matrix<double, 19, 19> damped_information =
      (1.0 + damping) * prior.inverse();
  damped_information.block<6, 6>(0, 0) += pose_information;
  const Eigen::Matrix<double, 19, 19> posterior = damped_information.inverse();
  const Eigen::Matrix<double, 6, 1> effective_rhs =
      pose_rhs - pose_information * iteration_prior_offset.head<6>();
  const Eigen::Matrix<double, 19, 1> target_delta =
      posterior.block<19, 6>(0, 0) * effective_rhs;
  result.delta_state = step_scale * target_delta +
      (1.0 - step_scale) * (-iteration_prior_offset);
  result.pose_difference_norm =
      (result.delta_state.head<6>() - raw_delta_state.head<6>()).norm();
  result.velocity_difference_norm =
      (result.delta_state.segment<3>(7) - raw_delta_state.segment<3>(7)).norm();
  result.localized_prior_min_eigenvalue = symmetricMinimumEigenvalue(prior / (1.0 + damping));
  result.localized_prior_asymmetry = (prior - prior.transpose()).cwiseAbs().maxCoeff();
  result.posterior_min_eigenvalue = symmetricMinimumEigenvalue(posterior);
  result.posterior_asymmetry =
      (posterior - posterior.transpose()).cwiseAbs().maxCoeff();
  result.posterior_trace = posterior.trace();
  result.pose_posterior_covariance_difference_norm =
      (posterior.block<6, 6>(0, 0) -
       raw_posterior_covariance.block<6, 6>(0, 0)).norm();
  result.valid = result.delta_state.allFinite() &&
                 std::isfinite(result.localized_prior_min_eigenvalue) &&
                 std::isfinite(result.posterior_min_eigenvalue);
  return result;
}

struct LioMotionConsistencyMetrics
{
  bool valid = false;
  Eigen::Matrix<double, 19, 1> delta_state =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  CrossBlockSummary p_v_theta;
  CrossBlockSummary p_v_position;
  CrossBlockSummary p_v_bg;
  CrossBlockSummary p_v_ba;
  CrossBlockSummary p_v_gravity;
  double rotation_equivalent_gain_norm = motionDiagnosticNaN();
  double position_equivalent_gain_norm = motionDiagnosticNaN();
  double velocity_equivalent_gain_norm = motionDiagnosticNaN();
  double bias_g_equivalent_gain_norm = motionDiagnosticNaN();
  double bias_a_equivalent_gain_norm = motionDiagnosticNaN();
  double gravity_equivalent_gain_norm = motionDiagnosticNaN();
  Eigen::Matrix<double, 19, 1> innovation_component =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 19, 1> relinearization_component =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 19, 1> accumulated_innovation_component =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 19, 1> accumulated_relinearization_component =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  int analyzed_iteration_count = 0;
  double maximum_iteration_linearized_nis_per_dof = motionDiagnosticNaN();
  double mean_iteration_linearized_nis_per_dof = motionDiagnosticNaN();
  double maximum_iteration_rotation_equivalent_gain_norm = motionDiagnosticNaN();
  double maximum_iteration_position_equivalent_gain_norm = motionDiagnosticNaN();
  double maximum_iteration_velocity_equivalent_gain_norm = motionDiagnosticNaN();
  double maximum_iteration_velocity_innovation_component_norm = motionDiagnosticNaN();
  NormalizedQuadratic q_velocity_prior;
  NormalizedQuadratic q_pose_prior;
  NormalizedQuadratic q_bias_prior;
  NormalizedQuadratic q_gravity_prior;
  NormalizedQuadratic q_state_prior;
  NormalizedQuadratic q_velocity_correction_approx;
  Eigen::Matrix<double, 19, 1> correction_covariance_eigenvalues =
      Eigen::Matrix<double, 19, 1>::Constant(motionDiagnosticNaN());
  Eigen::Vector3d velocity_correction_covariance_eigenvalues =
      Eigen::Vector3d::Constant(motionDiagnosticNaN());
  bool correction_covariance_psd = false;
  int correction_covariance_rank = 0;
  double correction_covariance_asymmetry = motionDiagnosticNaN();
  double residual_weighted_energy = motionDiagnosticNaN();
  double information_explained_energy = motionDiagnosticNaN();
  double linearized_nis = motionDiagnosticNaN();
  double linearized_nis_per_dof = motionDiagnosticNaN();
  bool linearized_nis_valid = false;
  Eigen::Matrix<double, 6, 1> pose_information_eigenvalues =
      Eigen::Matrix<double, 6, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 6, 1> pose_information_weak_direction =
      Eigen::Matrix<double, 6, 1>::Constant(motionDiagnosticNaN());
  Eigen::Matrix<double, 6, 1> pose_correction_eigen_projections =
      Eigen::Matrix<double, 6, 1>::Constant(motionDiagnosticNaN());
  int pose_information_rank = 0;
  double pose_information_condition_number = motionDiagnosticNaN();
  PoseDirectionCorrection pose_direction_correction;
  std::array<SourceUpdateCounterfactual, 4> transfer_counterfactuals;
  std::array<SourceUpdateCounterfactual, 4> damping_counterfactuals;
  double source_counterfactual_compute_time_ms = motionDiagnosticNaN();
  RollingCorrectionMetrics temporal;
  bool shadow_warn = false;
};

inline LioMotionConsistencyMetrics analyzeMotionConsistency(
    const Eigen::Matrix<double, 19, 1> &delta_state,
    const Eigen::Matrix<double, 19, 19> &prior_covariance,
    const Eigen::Matrix<double, 19, 19> &posterior_covariance,
    const Eigen::Matrix<double, 19, 6> &normal_rhs_gain,
    const Eigen::Matrix<double, 19, 6> &equivalent_update_operator,
    const Eigen::Matrix<double, 6, 6> &pose_information,
    const Eigen::Matrix<double, 6, 1> &pose_rhs,
    const Eigen::Matrix<double, 19, 1> &iteration_prior_offset,
    double residual_weighted_energy,
    int residual_degrees_of_freedom,
    double final_step_scale = 1.0)
{
  LioMotionConsistencyMetrics result;
  result.delta_state = delta_state;
  if (!delta_state.allFinite() || !prior_covariance.allFinite() ||
      !posterior_covariance.allFinite() || !normal_rhs_gain.allFinite() ||
      !equivalent_update_operator.allFinite() || !pose_information.allFinite() ||
      !pose_rhs.allFinite() || !iteration_prior_offset.allFinite() ||
      !std::isfinite(residual_weighted_energy))
    return result;

  result.p_v_theta = summarizeCrossBlock(prior_covariance.block<3, 3>(7, 0));
  result.p_v_position = summarizeCrossBlock(prior_covariance.block<3, 3>(7, 3));
  result.p_v_bg = summarizeCrossBlock(prior_covariance.block<3, 3>(7, 10));
  result.p_v_ba = summarizeCrossBlock(prior_covariance.block<3, 3>(7, 13));
  result.p_v_gravity = summarizeCrossBlock(prior_covariance.block<3, 3>(7, 16));
  result.rotation_equivalent_gain_norm =
      equivalent_update_operator.block<3, 6>(0, 0).norm();
  result.position_equivalent_gain_norm =
      equivalent_update_operator.block<3, 6>(3, 0).norm();
  result.velocity_equivalent_gain_norm =
      equivalent_update_operator.block<3, 6>(7, 0).norm();
  result.bias_g_equivalent_gain_norm =
      equivalent_update_operator.block<3, 6>(10, 0).norm();
  result.bias_a_equivalent_gain_norm =
      equivalent_update_operator.block<3, 6>(13, 0).norm();
  result.gravity_equivalent_gain_norm =
      equivalent_update_operator.block<3, 6>(16, 0).norm();
  result.innovation_component = normal_rhs_gain * pose_rhs;
  result.relinearization_component = iteration_prior_offset -
      equivalent_update_operator * iteration_prior_offset.head<6>();

  const auto source_counterfactual_begin = std::chrono::steady_clock::now();
  result.pose_direction_correction = decomposePoseDirectionCorrection(
      normal_rhs_gain, pose_information, pose_rhs, iteration_prior_offset,
      delta_state, final_step_scale);
  constexpr std::array<double, 4> transfer_strengths{{0.0, 0.25, 0.5, 1.0}};
  constexpr std::array<double, 4> damping_strengths{{0.0, 0.25, 1.0, 4.0}};
  for (std::size_t i = 0; i < transfer_strengths.size(); ++i)
  {
    result.transfer_counterfactuals[i] = observabilityAwareTransferCounterfactual(
        prior_covariance, pose_information, pose_rhs, iteration_prior_offset,
        delta_state, posterior_covariance, final_step_scale, transfer_strengths[i]);
    result.damping_counterfactuals[i] = priorMetricDampingCounterfactual(
        prior_covariance, pose_information, pose_rhs, iteration_prior_offset,
        delta_state, posterior_covariance, final_step_scale, damping_strengths[i]);
  }
  result.source_counterfactual_compute_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - source_counterfactual_begin).count();

  result.q_velocity_prior = normalizedQuadratic(
      prior_covariance.block<3, 3>(7, 7), delta_state.segment<3>(7));
  result.q_pose_prior = normalizedQuadratic(
      prior_covariance.block<6, 6>(0, 0), delta_state.head<6>());
  result.q_bias_prior = normalizedQuadratic(
      prior_covariance.block<6, 6>(10, 10), delta_state.segment<6>(10));
  result.q_gravity_prior = normalizedQuadratic(
      prior_covariance.block<3, 3>(16, 16), delta_state.segment<3>(16));
  result.q_state_prior = normalizedQuadratic(prior_covariance, delta_state);

  const Eigen::Matrix<double, 19, 19> correction_covariance =
      0.5 * ((prior_covariance - posterior_covariance) +
             (prior_covariance - posterior_covariance).transpose()).eval();
  result.correction_covariance_asymmetry =
      ((prior_covariance - posterior_covariance) -
       (prior_covariance - posterior_covariance).transpose()).cwiseAbs().maxCoeff();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 19, 19>> correction_solver(
      correction_covariance);
  if (correction_solver.info() == Eigen::Success &&
      correction_solver.eigenvalues().allFinite())
  {
    result.correction_covariance_eigenvalues = correction_solver.eigenvalues();
    const double scale = std::max(
        1.0, result.correction_covariance_eigenvalues.cwiseAbs().maxCoeff());
    const double cutoff = std::max(1e-15, 1e-12 * scale);
    result.correction_covariance_psd =
        result.correction_covariance_eigenvalues.minCoeff() >= -cutoff;
    result.correction_covariance_rank =
        (result.correction_covariance_eigenvalues.array() > cutoff).count();
  }
  const Eigen::Matrix3d correction_velocity_covariance =
      correction_covariance.block<3, 3>(7, 7);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> velocity_correction_solver(
      correction_velocity_covariance);
  if (velocity_correction_solver.info() == Eigen::Success &&
      velocity_correction_solver.eigenvalues().allFinite())
    result.velocity_correction_covariance_eigenvalues =
        velocity_correction_solver.eigenvalues();
  result.q_velocity_correction_approx = normalizedQuadratic(
      correction_velocity_covariance, delta_state.segment<3>(7), true);

  result.residual_weighted_energy = residual_weighted_energy;
  result.information_explained_energy =
      pose_rhs.dot(normal_rhs_gain.topRows<6>() * pose_rhs);
  result.linearized_nis =
      residual_weighted_energy - result.information_explained_energy;
  const double nis_tolerance = 1e-9 * std::max(1.0, residual_weighted_energy);
  if (result.linearized_nis >= -nis_tolerance && residual_degrees_of_freedom > 0)
  {
    result.linearized_nis = std::max(0.0, result.linearized_nis);
    result.linearized_nis_per_dof =
        result.linearized_nis / static_cast<double>(residual_degrees_of_freedom);
    result.linearized_nis_valid = true;
  }

  const Eigen::Matrix<double, 6, 6> symmetric_information =
      0.5 * (pose_information + pose_information.transpose()).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> pose_solver(
      symmetric_information);
  if (pose_solver.info() == Eigen::Success && pose_solver.eigenvalues().allFinite())
  {
    result.pose_information_eigenvalues = pose_solver.eigenvalues();
    result.pose_information_weak_direction = pose_solver.eigenvectors().col(0);
    result.pose_correction_eigen_projections =
        pose_solver.eigenvectors().transpose() * delta_state.head<6>();
    const double scale = std::max(
        1.0, result.pose_information_eigenvalues.cwiseAbs().maxCoeff());
    const double cutoff = std::max(1e-15, 1e-12 * scale);
    result.pose_information_rank =
        (result.pose_information_eigenvalues.array() > cutoff).count();
    if (result.pose_information_rank > 0)
    {
      const double minimum_used = result.pose_information_eigenvalues
          .tail(result.pose_information_rank).minCoeff();
      result.pose_information_condition_number =
          result.pose_information_eigenvalues.maxCoeff() / minimum_used;
    }
  }
  result.valid = result.q_velocity_prior.valid && result.q_pose_prior.valid &&
                 result.q_bias_prior.valid && result.q_gravity_prior.valid &&
                 result.q_state_prior.valid && result.linearized_nis_valid;
  return result;
}

class RollingCorrectionWindow
{
public:
  struct SnapshotSample
  {
    double timestamp = 0.0;
    Eigen::Vector3d delta_velocity = Eigen::Vector3d::Zero();
  };

  RollingCorrectionMetrics update(double timestamp,
                                  const Eigen::Vector3d &delta_velocity,
                                  const Eigen::Vector3d &delta_position,
                                  const Eigen::Vector3d &candidate_velocity)
  {
    if (!std::isfinite(timestamp) || !delta_velocity.allFinite() ||
        !delta_position.allFinite() || !candidate_velocity.allFinite())
      return RollingCorrectionMetrics();
    if (!samples_.empty() && timestamp < samples_.back().timestamp)
      samples_.clear();

    RollingCorrectionMetrics result;
    if (!samples_.empty())
      result.consecutive_direction_cosine = cosine(
          delta_velocity, samples_.back().delta_velocity);
    samples_.push_back({timestamp, delta_velocity});
    while (!samples_.empty() && samples_.front().timestamp < timestamp - 1.0)
      samples_.pop_front();

    double sum_norm = 0.0;
    for (const Sample &sample : samples_)
    {
      const double age = timestamp - sample.timestamp;
      if (age <= 0.25) result.cumulative_dv_025 += sample.delta_velocity;
      if (age <= 0.50) result.cumulative_dv_050 += sample.delta_velocity;
      result.cumulative_dv_100 += sample.delta_velocity;
      sum_norm += sample.delta_velocity.norm();
    }
    if (sum_norm > 1e-15)
      result.direction_persistence_1s =
          result.cumulative_dv_100.norm() / sum_norm;
    result.dv_velocity_cosine = cosine(delta_velocity, candidate_velocity);
    result.dv_velocity_angle_deg = angleDegrees(result.dv_velocity_cosine);
    result.dv_dp_cosine = cosine(delta_velocity, delta_position);
    result.dv_dp_angle_deg = angleDegrees(result.dv_dp_cosine);
    if (delta_position.norm() > 1e-15)
      result.dv_dp_norm_ratio = delta_velocity.norm() / delta_position.norm();
    return result;
  }

  void clear() { samples_.clear(); }

  std::vector<SnapshotSample> snapshot() const
  {
    std::vector<SnapshotSample> result;
    result.reserve(samples_.size());
    for (const Sample &sample : samples_)
      result.push_back({sample.timestamp, sample.delta_velocity});
    return result;
  }

  void restore(const std::vector<SnapshotSample> &snapshot)
  {
    samples_.clear();
    for (const SnapshotSample &sample : snapshot)
      samples_.push_back({sample.timestamp, sample.delta_velocity});
  }

private:
  struct Sample
  {
    double timestamp;
    Eigen::Vector3d delta_velocity;
  };

  static double cosine(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
  {
    const double denominator = a.norm() * b.norm();
    if (!(denominator > 1e-15)) return motionDiagnosticNaN();
    return std::max(-1.0, std::min(1.0, a.dot(b) / denominator));
  }

  static double angleDegrees(double cosine_value)
  {
    if (!std::isfinite(cosine_value)) return motionDiagnosticNaN();
    return std::acos(std::max(-1.0, std::min(1.0, cosine_value))) *
           57.29577951308232;
  }

  std::deque<Sample> samples_;
};

} // namespace fast_livo

#endif
