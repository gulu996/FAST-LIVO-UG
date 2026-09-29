#include "landmark_measurement.h"

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{

void require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

landmark::Vector6d residualAt(
    const Eigen::Matrix3d &R_world_imu,
    const Eigen::Vector3d &p_world_imu,
    const Eigen::Matrix3d &R_imu_camera,
    const Eigen::Vector3d &p_imu_camera,
    const Eigen::Matrix3d &R_camera_landmark,
    const Eigen::Vector3d &p_camera_landmark,
    const Eigen::Matrix3d &R_world_landmark,
    const Eigen::Vector3d &p_world_landmark,
    const landmark::Vector6d &dx)
{
  return landmark::linearizeLandmarkMeasurement(
      R_world_imu * landmark::so3Exp(dx.head<3>()),
      p_world_imu + dx.tail<3>(), R_imu_camera, p_imu_camera,
      R_camera_landmark, p_camera_landmark, R_world_landmark,
      p_world_landmark).residual;
}

} // namespace

int main()
{
  try
  {
    const Eigen::Matrix3d R_w_i =
        Eigen::AngleAxisd(0.25, Eigen::Vector3d(0.2, -0.4, 0.9).normalized())
            .toRotationMatrix();
    const Eigen::Vector3d p_w_i(1.0, -0.4, 0.3);
    const Eigen::Matrix3d R_i_c =
        Eigen::AngleAxisd(-0.18, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d p_i_c(0.12, -0.03, 0.08);
    const Eigen::Matrix3d R_c_l =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d p_c_l(0.1, -0.05, 3.0);
    const Eigen::Matrix3d R_w_l = R_w_i * R_i_c * R_c_l;
    const Eigen::Vector3d p_w_l =
        p_w_i + R_w_i * (p_i_c + R_i_c * p_c_l);

    const landmark::MeasurementLinearization identity =
        landmark::linearizeLandmarkMeasurement(
            R_w_i, p_w_i, R_i_c, p_i_c, R_c_l, p_c_l, R_w_l, p_w_l);
    require(identity.residual.norm() < 1e-11,
            "identity landmark residual is not zero");

    const landmark::Vector6d known_perturbation =
        (landmark::Vector6d() << 1e-3, -2e-3, 1.5e-3,
         0.01, -0.02, 0.015).finished();
    const landmark::Vector6d perturbed = residualAt(
        R_w_i, p_w_i, R_i_c, p_i_c, R_c_l, p_c_l, R_w_l, p_w_l,
        known_perturbation);
    require((perturbed + identity.H * known_perturbation).norm() < 2e-5,
            "known SE3 perturbation has wrong residual direction");

    landmark::Matrix6d numerical_H;
    const double epsilon = 1e-6;
    for (int column = 0; column < 6; ++column)
    {
      landmark::Vector6d plus = landmark::Vector6d::Zero();
      landmark::Vector6d minus = landmark::Vector6d::Zero();
      plus(column) = epsilon;
      minus(column) = -epsilon;
      numerical_H.col(column) = -(residualAt(
          R_w_i, p_w_i, R_i_c, p_i_c, R_c_l, p_c_l, R_w_l, p_w_l,
          plus) - residualAt(
          R_w_i, p_w_i, R_i_c, p_i_c, R_c_l, p_c_l, R_w_l, p_w_l,
          minus)) / (2.0 * epsilon);
    }
    require((numerical_H - identity.H).cwiseAbs().maxCoeff() < 2e-6,
            "analytic landmark Jacobian disagrees with finite difference");

    landmark::Matrix6d robot_cov = landmark::Matrix6d::Identity() * 1e-3;
    landmark::Matrix6d observation_cov = landmark::Matrix6d::Zero();
    observation_cov.diagonal().head<3>().setConstant(1e-4);
    observation_cov.diagonal().tail<3>().setConstant(4e-4);
    const landmark::Matrix6d initialized_cov =
        landmark::initializeLandmarkCovariance(
            robot_cov, observation_cov, R_w_i, R_i_c, p_i_c, p_c_l);
    Eigen::SelfAdjointEigenSolver<landmark::Matrix6d> init_eigen(initialized_cov);
    require(initialized_cov.allFinite() &&
                init_eigen.eigenvalues().minCoeff() > 0.0,
            "initialized landmark covariance is not finite positive definite");

    const Eigen::Vector3d accepted_residual(0.2, 0.0, 0.0);
    const Eigen::Matrix3d S = Eigen::Matrix3d::Identity() * 0.1;
    const landmark::NisResult accepted = landmark::evaluateNis(
        accepted_residual, S, landmark::chiSquareThreshold(3, 0.99), 1e8);
    require(accepted.valid && accepted.accepted,
            "small NIS residual was rejected");
    const landmark::NisResult rejected = landmark::evaluateNis(
        Eigen::Vector3d(2.0, 0.0, 0.0), S,
        landmark::chiSquareThreshold(3, 0.99), 1e8);
    require(rejected.valid && !rejected.accepted,
            "large NIS residual was accepted");
    require(!landmark::evaluateNis(
                accepted_residual, Eigen::Matrix3d::Zero(), 10.0, 1e8).valid,
            "singular innovation covariance was accepted");
    Eigen::Vector3d nonfinite = accepted_residual;
    nonfinite.x() = std::numeric_limits<double>::quiet_NaN();
    require(!landmark::evaluateNis(nonfinite, S, 10.0, 1e8).valid,
            "nonfinite residual was accepted");

    std::cout << "LANDMARK_RESIDUAL_IDENTITY=PASS\n"
              << "LANDMARK_KNOWN_PERTURBATION=PASS\n"
              << "LANDMARK_JACOBIAN_FINITE_DIFFERENCE=PASS\n"
              << "LANDMARK_INIT_COVARIANCE=PASS\n"
              << "LANDMARK_NIS_GATE=PASS\n"
              << "LANDMARK_NUMERICAL_REJECTION=PASS\n";
    return EXIT_SUCCESS;
  }
  catch (const std::exception &exception)
  {
    std::cerr << "landmark_measurement_self_test: FAIL: "
              << exception.what() << '\n';
    return EXIT_FAILURE;
  }
}
