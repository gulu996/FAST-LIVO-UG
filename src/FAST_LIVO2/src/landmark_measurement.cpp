#include "landmark_measurement.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace landmark
{
namespace
{

Eigen::Matrix3d leftJacobianInverse(const Eigen::Vector3d &w)
{
  const double theta = w.norm();
  const Eigen::Matrix3d W = skew(w);
  if (theta < 1e-7)
    return Eigen::Matrix3d::Identity() - 0.5 * W + W * W / 12.0;
  const double coefficient =
      1.0 / (theta * theta) -
      (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
  return Eigen::Matrix3d::Identity() - 0.5 * W + coefficient * W * W;
}

} // namespace

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(),
            v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
  return result;
}

Eigen::Matrix3d so3Exp(const Eigen::Vector3d &w)
{
  const double theta = w.norm();
  const Eigen::Matrix3d W = skew(w);
  if (theta < 1e-8)
    return Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
  return Eigen::Matrix3d::Identity() +
      std::sin(theta) / theta * W +
      (1.0 - std::cos(theta)) / (theta * theta) * W * W;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d &R)
{
  Eigen::AngleAxisd angle_axis(R);
  if (!std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  if (angle_axis.angle() < 1e-12) return Eigen::Vector3d::Zero();
  return angle_axis.angle() * angle_axis.axis();
}

MeasurementLinearization linearizeLandmarkMeasurement(
    const Eigen::Matrix3d &R_world_imu,
    const Eigen::Vector3d &p_world_imu,
    const Eigen::Matrix3d &R_imu_camera,
    const Eigen::Vector3d &p_imu_camera,
    const Eigen::Matrix3d &R_camera_landmark,
    const Eigen::Vector3d &p_camera_landmark,
    const Eigen::Matrix3d &R_world_landmark,
    const Eigen::Vector3d &p_world_landmark)
{
  MeasurementLinearization result;
  result.R_world_camera = R_world_imu * R_imu_camera;
  const Eigen::Vector3d p_imu_landmark =
      p_imu_camera + R_imu_camera * p_camera_landmark;
  result.R_world_landmark_estimate =
      result.R_world_camera * R_camera_landmark;
  result.p_world_landmark_estimate =
      p_world_imu + R_world_imu * p_imu_landmark;
  result.residual.head<3>() = so3Log(
      result.R_world_landmark_estimate.transpose() * R_world_landmark);
  result.residual.tail<3>() =
      p_world_landmark - result.p_world_landmark_estimate;

  const Eigen::Matrix3d A = R_imu_camera * R_camera_landmark;
  result.H.block<3, 3>(0, 0) =
      leftJacobianInverse(result.residual.head<3>()) * A.transpose();
  result.H.block<3, 3>(3, 0) =
      -R_world_imu * skew(p_imu_landmark);
  result.H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  return result;
}

Matrix6d initializeLandmarkCovariance(
    const Matrix6d &robot_pose_covariance,
    const Matrix6d &observation_covariance_camera,
    const Eigen::Matrix3d &R_world_imu,
    const Eigen::Matrix3d &R_imu_camera,
    const Eigen::Vector3d &p_imu_camera,
    const Eigen::Vector3d &p_camera_landmark)
{
  const Eigen::Vector3d p_imu_landmark =
      p_imu_camera + R_imu_camera * p_camera_landmark;
  Matrix6d J_robot = Matrix6d::Zero();
  J_robot.block<3, 3>(0, 0) = R_world_imu;
  J_robot.block<3, 3>(3, 0) = -R_world_imu * skew(p_imu_landmark);
  J_robot.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  Matrix6d J_observation = Matrix6d::Zero();
  const Eigen::Matrix3d R_world_camera = R_world_imu * R_imu_camera;
  J_observation.block<3, 3>(0, 0) = R_world_camera;
  J_observation.block<3, 3>(3, 3) = R_world_camera;
  return J_robot * robot_pose_covariance * J_robot.transpose() +
      J_observation * observation_covariance_camera *
      J_observation.transpose();
}

Matrix6d repeatedObservationCovariance(
    const Matrix6d &observation_covariance_camera,
    const Matrix6d &landmark_covariance_world,
    const Eigen::Matrix3d &R_world_camera,
    const Eigen::Matrix3d &R_camera_landmark,
    const Eigen::Matrix3d &R_world_landmark_estimate,
    const Eigen::Vector3d &orientation_residual)
{
  const Eigen::Matrix3d J_log =
      leftJacobianInverse(orientation_residual);
  Matrix6d J_observation = Matrix6d::Zero();
  J_observation.block<3, 3>(0, 0) =
      J_log * R_camera_landmark.transpose();
  J_observation.block<3, 3>(3, 3) = R_world_camera;
  Matrix6d J_landmark = Matrix6d::Zero();
  J_landmark.block<3, 3>(0, 0) =
      J_log * R_world_landmark_estimate.transpose();
  J_landmark.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  return J_observation * observation_covariance_camera *
             J_observation.transpose() +
      J_landmark * landmark_covariance_world * J_landmark.transpose();
}

NisResult evaluateNis(const Eigen::VectorXd &residual,
                      const Eigen::MatrixXd &innovation_covariance,
                      double threshold,
                      double max_condition)
{
  NisResult result;
  if (residual.size() == 0 || innovation_covariance.rows() != residual.size() ||
      innovation_covariance.cols() != residual.size() ||
      !residual.allFinite() || !innovation_covariance.allFinite())
  {
    result.reason = "nonfinite_or_dimension_mismatch";
    return result;
  }
  const Eigen::MatrixXd symmetric =
      0.5 * (innovation_covariance + innovation_covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(symmetric);
  if (eigensolver.info() != Eigen::Success ||
      eigensolver.eigenvalues().minCoeff() <= 0.0)
  {
    result.reason = "innovation_covariance_not_positive_definite";
    return result;
  }
  result.condition = eigensolver.eigenvalues().maxCoeff() /
      eigensolver.eigenvalues().minCoeff();
  if (!std::isfinite(result.condition) || result.condition > max_condition)
  {
    result.reason = "innovation_covariance_ill_conditioned";
    return result;
  }
  Eigen::LDLT<Eigen::MatrixXd> ldlt(symmetric);
  if (ldlt.info() != Eigen::Success)
  {
    result.reason = "innovation_covariance_factorization_failed";
    return result;
  }
  const Eigen::VectorXd solved = ldlt.solve(residual);
  result.value = residual.dot(solved);
  if (!solved.allFinite() || !std::isfinite(result.value) || result.value < 0.0)
  {
    result.reason = "nis_nonfinite";
    return result;
  }
  result.valid = true;
  result.accepted = result.value <= threshold;
  result.reason = result.accepted ? "accepted" : "nis_above_threshold";
  return result;
}

double chiSquareThreshold(int degrees_of_freedom, double confidence)
{
  const bool c95 = std::fabs(confidence - 0.95) < 1e-6;
  const bool c99 = std::fabs(confidence - 0.99) < 1e-6;
  const bool c999 = std::fabs(confidence - 0.999) < 1e-6;
  if (degrees_of_freedom == 3)
  {
    if (c95) return 7.8147279;
    if (c99) return 11.3448667;
    if (c999) return 16.2662362;
  }
  if (degrees_of_freedom == 6)
  {
    if (c95) return 12.5915872;
    if (c99) return 16.8118938;
    if (c999) return 22.4577445;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

} // namespace landmark
