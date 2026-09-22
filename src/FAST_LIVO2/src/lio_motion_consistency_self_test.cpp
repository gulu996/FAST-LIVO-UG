#include "lio_motion_consistency.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

bool near(double actual, double expected, double tolerance = 1e-10)
{
  return std::fabs(actual - expected) <= tolerance;
}

} // namespace

int main()
{
  try
  {
    Eigen::Matrix3d covariance = Eigen::Vector3d(1.0, 2.0, 4.0).asDiagonal();
    const Eigen::Vector3d correction(1.0, 2.0, 2.0);
    const fast_livo::NormalizedQuadratic q =
        fast_livo::normalizedQuadratic(covariance, correction);
    require(q.valid && q.rank == 3, "positive-definite quadratic invalid");
    require(near(q.value, 4.0), "positive-definite quadratic value wrong");

    covariance(0, 0) = 0.0;
    const fast_livo::NormalizedQuadratic semidefinite =
        fast_livo::normalizedQuadratic(covariance, correction, true);
    require(semidefinite.valid && semidefinite.rank == 2,
            "semidefinite pseudoinverse invalid");
    require(near(semidefinite.value, 3.0),
            "semidefinite pseudoinverse value wrong");
    require(!fast_livo::normalizedQuadratic(covariance, correction).valid,
            "rank-deficient prior accepted");

    Eigen::Matrix<double, 19, 19> prior =
        Eigen::Matrix<double, 19, 19>::Identity();
    Eigen::Matrix<double, 6, 6> information =
        2.0 * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 19, 6> normal_rhs_gain =
        Eigen::Matrix<double, 19, 6>::Zero();
    normal_rhs_gain.topRows<6>() =
        Eigen::Matrix<double, 6, 6>::Identity() / 3.0;
    const Eigen::Matrix<double, 19, 6> update_operator =
        normal_rhs_gain * information;
    Eigen::Matrix<double, 19, 19> full_update =
        Eigen::Matrix<double, 19, 19>::Zero();
    full_update.leftCols<6>() = update_operator;
    const Eigen::Matrix<double, 19, 19> posterior =
        (Eigen::Matrix<double, 19, 19>::Identity() - full_update) * prior;
    Eigen::Matrix<double, 19, 1> delta =
        Eigen::Matrix<double, 19, 1>::Constant(0.1);
    const Eigen::Matrix<double, 6, 1> rhs =
        Eigen::Matrix<double, 6, 1>::Ones();
    const fast_livo::LioMotionConsistencyMetrics metrics =
        fast_livo::analyzeMotionConsistency(
            delta, prior, posterior, normal_rhs_gain, update_operator,
            information, rhs, Eigen::Matrix<double, 19, 1>::Zero(), 8.0, 6);
    require(metrics.valid, "consistent fixture rejected");
    require(near(metrics.information_explained_energy, 2.0),
            "Woodbury explained energy wrong");
    require(near(metrics.linearized_nis, 6.0) &&
            near(metrics.linearized_nis_per_dof, 1.0),
            "reduced linearized NIS wrong");
    require(metrics.correction_covariance_psd &&
            metrics.correction_covariance_rank == 6,
            "correction covariance spectrum wrong");

    Eigen::Matrix<double, 13, 6> regression =
        Eigen::Matrix<double, 13, 6>::Zero();
    regression.block<3, 3>(1, 3) = 0.2 * Eigen::Matrix3d::Identity();
    regression.block<3, 3>(4, 0) = 0.1 * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 19, 19> coupled_prior =
        Eigen::Matrix<double, 19, 19>::Zero();
    coupled_prior.topLeftCorner<6, 6>().setIdentity();
    coupled_prior.bottomLeftCorner<13, 6>() = regression;
    coupled_prior.topRightCorner<6, 13>() = regression.transpose();
    coupled_prior.bottomRightCorner<13, 13>() =
        Eigen::Matrix<double, 13, 13>::Identity() + regression * regression.transpose();
    Eigen::Matrix<double, 6, 6> directional_information =
        Eigen::Matrix<double, 6, 6>::Zero();
    directional_information.diagonal() << 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0;
    Eigen::Matrix<double, 19, 19> coupled_posterior =
        coupled_prior.inverse();
    coupled_posterior.topLeftCorner<6, 6>() += directional_information;
    coupled_posterior = coupled_posterior.inverse();
    const Eigen::Matrix<double, 19, 6> coupled_gain =
        coupled_posterior.leftCols<6>();
    const Eigen::Matrix<double, 6, 1> directional_rhs =
        Eigen::Matrix<double, 6, 1>::Ones();
    const Eigen::Matrix<double, 19, 1> zero_offset =
        Eigen::Matrix<double, 19, 1>::Zero();
    const Eigen::Matrix<double, 19, 1> raw_directional_delta =
        coupled_gain * directional_rhs;
    const fast_livo::PoseDirectionCorrection decomposition =
        fast_livo::decomposePoseDirectionCorrection(
            coupled_gain, directional_information, directional_rhs, zero_offset,
            raw_directional_delta, 1.0);
    require(decomposition.valid && decomposition.full_closure_norm < 1e-12 &&
                decomposition.velocity_closure_norm < 1e-12,
            "pose-direction correction decomposition does not close");

    const auto transfer_identity =
        fast_livo::observabilityAwareTransferCounterfactual(
            coupled_prior, directional_information, directional_rhs, zero_offset,
            raw_directional_delta, coupled_posterior, 1.0, 0.0);
    const auto transfer_strong =
        fast_livo::observabilityAwareTransferCounterfactual(
            coupled_prior, directional_information, directional_rhs, zero_offset,
            raw_directional_delta, coupled_posterior, 1.0, 1.0);
    require(transfer_identity.valid && transfer_strong.valid,
            "transfer counterfactual invalid");
    require((transfer_identity.delta_state - raw_directional_delta).norm() < 1e-10,
            "identity transfer counterfactual changed the update");
    require(transfer_strong.pose_difference_norm < 1e-10,
            "hidden-transfer localization changed the pose posterior mean");
    require(transfer_strong.pose_posterior_covariance_difference_norm < 1e-10,
            "hidden-transfer localization changed the pose posterior covariance");
    require(transfer_strong.direction_weights[0] <
                transfer_strong.direction_weights[5],
            "dimensionless information confidence ordering wrong");
    require(transfer_strong.localized_prior_min_eigenvalue > -1e-12 &&
                transfer_strong.posterior_min_eigenvalue > -1e-12,
            "transfer counterfactual covariance is not PSD");

    const auto damping_identity = fast_livo::priorMetricDampingCounterfactual(
        coupled_prior, directional_information, directional_rhs, zero_offset,
        raw_directional_delta, coupled_posterior, 1.0, 0.0);
    const auto damping_strong = fast_livo::priorMetricDampingCounterfactual(
        coupled_prior, directional_information, directional_rhs, zero_offset,
        raw_directional_delta, coupled_posterior, 1.0, 4.0);
    require(damping_identity.valid && damping_strong.valid &&
                (damping_identity.delta_state - raw_directional_delta).norm() < 1e-10,
            "identity damping counterfactual changed the update");
    require(damping_strong.delta_state.norm() < damping_identity.delta_state.norm() &&
                damping_strong.posterior_min_eigenvalue > -1e-12,
            "prior-metric damping did not contract a healthy fixture");

    fast_livo::RollingCorrectionWindow window;
    window.update(0.0, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX(),
                  Eigen::Vector3d::UnitX());
    window.update(0.4, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX(),
                  Eigen::Vector3d::UnitX());
    const fast_livo::RollingCorrectionMetrics rolling =
        window.update(0.8, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX(),
                      Eigen::Vector3d::UnitX());
    require(near(rolling.cumulative_dv_025.norm(), 1.0) &&
            near(rolling.cumulative_dv_050.norm(), 2.0) &&
            near(rolling.cumulative_dv_100.norm(), 3.0),
            "rolling windows wrong");
    require(near(rolling.direction_persistence_1s, 1.0) &&
            near(rolling.consecutive_direction_cosine, 1.0),
            "direction persistence wrong");

    std::cout << "lio_motion_consistency_self_test: PASS\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "lio_motion_consistency_self_test: FAIL: " << error.what()
              << '\n';
    return 1;
  }
}
