#ifndef LANDMARK_MEASUREMENT_H_
#define LANDMARK_MEASUREMENT_H_

#include <Eigen/Core>

#include <limits>
#include <string>

namespace landmark
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

Eigen::Matrix3d skew(const Eigen::Vector3d &v);
Eigen::Matrix3d so3Exp(const Eigen::Vector3d &w);
Eigen::Vector3d so3Log(const Eigen::Matrix3d &R);

struct MeasurementLinearization
{
  // Ordering is [orientation residual rad, position residual m].
  Vector6d residual = Vector6d::Zero();
  // residual(x + dx) ~= residual(x) - H * dx, with state error
  // [right IMU rotation rad, world translation m].
  Matrix6d H = Matrix6d::Zero();
  Eigen::Matrix3d R_world_camera = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_world_landmark_estimate = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_world_landmark_estimate = Eigen::Vector3d::Zero();
};

MeasurementLinearization linearizeLandmarkMeasurement(
    const Eigen::Matrix3d &R_world_imu,
    const Eigen::Vector3d &p_world_imu,
    const Eigen::Matrix3d &R_imu_camera,
    const Eigen::Vector3d &p_imu_camera,
    const Eigen::Matrix3d &R_camera_landmark,
    const Eigen::Vector3d &p_camera_landmark,
    const Eigen::Matrix3d &R_world_landmark,
    const Eigen::Vector3d &p_world_landmark);

Matrix6d initializeLandmarkCovariance(
    const Matrix6d &robot_pose_covariance,
    const Matrix6d &observation_covariance_camera,
    const Eigen::Matrix3d &R_world_imu,
    const Eigen::Matrix3d &R_imu_camera,
    const Eigen::Vector3d &p_imu_camera,
    const Eigen::Vector3d &p_camera_landmark);

Matrix6d repeatedObservationCovariance(
    const Matrix6d &observation_covariance_camera,
    const Matrix6d &landmark_covariance_world,
    const Eigen::Matrix3d &R_world_camera,
    const Eigen::Matrix3d &R_camera_landmark,
    const Eigen::Matrix3d &R_world_landmark_estimate,
    const Eigen::Vector3d &orientation_residual);

struct NisResult
{
  bool valid = false;
  bool accepted = false;
  double value = std::numeric_limits<double>::quiet_NaN();
  double condition = std::numeric_limits<double>::infinity();
  std::string reason;
};

NisResult evaluateNis(const Eigen::VectorXd &residual,
                      const Eigen::MatrixXd &innovation_covariance,
                      double threshold,
                      double max_condition);

double chiSquareThreshold(int degrees_of_freedom, double confidence);

} // namespace landmark

#endif // LANDMARK_MEASUREMENT_H_
