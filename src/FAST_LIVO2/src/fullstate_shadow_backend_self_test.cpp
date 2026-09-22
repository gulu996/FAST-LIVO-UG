#include "fullstate_shadow_backend.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

fast_livo::FullStateLidarGeometry geometryMessage() {
  fast_livo::FullStateLidarGeometry message;
  message.factor_source = "L1_FINAL_POINT_TO_PLANE";
  message.geometry_valid = true;
  message.frozen_submap = true;
  message.residual_dof = 100;
  message.linearization_pose.orientation.w = 1.0;
  const double diagonal[6] = {10.0, 20.0, 30.0, 40.0, 0.0, 60.0};
  for (int i = 0; i < 6; ++i) {
    message.pose_information[i * 6 + i] = diagonal[i];
    message.pose_rhs[i] = i == 4 ? 0.0 : 0.1 * (i + 1);
  }
  return message;
}

void testImuIntervalBoundary() {
  fast_livo_shadow::ImuIntervalBuffer buffer;
  for (int i = 0; i <= 3; ++i) {
    fast_livo_shadow::ImuSample sample;
    sample.stamp = 1.0 + i;
    sample.acceleration = gtsam::Vector3(i, 0.0, 0.0);
    require(buffer.add(sample), "ordered IMU sample rejected");
  }
  fast_livo_shadow::ImuSample duplicate;
  duplicate.stamp = 4.0;
  require(!buffer.add(duplicate) && buffer.duplicateCount() == 1,
          "duplicate IMU timestamp was not rejected");
  fast_livo_shadow::ImuSample rollback;
  rollback.stamp = 3.5;
  require(!buffer.add(rollback) && buffer.nonMonotonicCount() == 1,
          "non-monotonic IMU timestamp was not rejected");
  const auto first = buffer.extract(1.5, 2.5, 1.1);
  const auto second = buffer.extract(2.5, 3.5, 1.1);
  require(first.valid && second.valid, "boundary interpolation failed");
  require(first.owned_sample_count == 1 && second.owned_sample_count == 1,
          "an IMU sample was owned by two intervals");
  require(std::abs(first.duration - 1.0) < 1e-12 &&
              std::abs(second.duration - 1.0) < 1e-12,
          "IMU interval duration did not close");
}

void testBiasCorrection() {
  auto parameters = gtsam::PreintegrationParams::MakeSharedU(0.0);
  parameters->accelerometerCovariance = gtsam::Matrix3::Identity() * 1e-6;
  parameters->gyroscopeCovariance = gtsam::Matrix3::Identity() * 1e-6;
  parameters->integrationCovariance = gtsam::Matrix3::Identity() * 1e-8;
  const gtsam::imuBias::ConstantBias zero_bias;
  gtsam::PreintegratedImuMeasurements preintegration(parameters, zero_bias);
  preintegration.integrateMeasurement(gtsam::Vector3(1.0, 0.0, 0.0),
                                      gtsam::Vector3::Zero(), 1.0);
  const gtsam::NavState initial(gtsam::Pose3(), gtsam::Vector3::Zero());
  const gtsam::NavState unbiased = preintegration.predict(initial, zero_bias);
  const gtsam::imuBias::ConstantBias changed_bias(
      gtsam::Vector3(0.2, 0.0, 0.0), gtsam::Vector3::Zero());
  const gtsam::NavState corrected = preintegration.predict(initial, changed_bias);
  require((unbiased.v() - corrected.v()).norm() > 0.1,
          "preintegration ignored bias correction");
}

void testLidarFactorAndNoDoubleCountingContract() {
  auto message = geometryMessage();
  std::string reason;
  require(fast_livo_shadow::lidarOnlyFactorContract(message, &reason),
          "valid L1 contract rejected");
  const auto result = fast_livo_shadow::buildLidarFactor(
      message, gtsam::Symbol('x', 0));
  require(result.valid && result.rank == 5 && result.factor,
          "LiDAR factor rank or construction is wrong");
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> spectrum(result.information);
  require(spectrum.info() == Eigen::Success &&
              spectrum.eigenvalues().minCoeff() > -1e-10,
          "LiDAR factor is not finite PSD");
  require(spectrum.eigenvalues().minCoeff() < 1e-10,
          "weak direction received an artificial information floor");
  message.factor_source = "RAW_LIVO_BETWEEN";
  require(!fast_livo_shadow::lidarOnlyFactorContract(message, &reason),
          "double-counting raw-LIVO factor passed the contract");
}

void testFixedLagMarginalizationAndGraphInitialization() {
  gtsam::IncrementalFixedLagSmoother smoother(0.15);
  const gtsam::Vector3 zero_vector = gtsam::Vector3::Zero();
  const auto pose_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  const auto vector_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  const auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  for (std::uint64_t i = 0; i < 5; ++i) {
    const gtsam::Key x = gtsam::Symbol('x', i);
    const gtsam::Key v = gtsam::Symbol('v', i);
    const gtsam::Key b = gtsam::Symbol('b', i);
    gtsam::NonlinearFactorGraph factors;
    if (i == 0) {
      factors.add(gtsam::PriorFactor<gtsam::Pose3>(
          x, gtsam::Pose3(), pose_noise));
      factors.add(gtsam::PriorFactor<gtsam::Vector3>(
          v, zero_vector, vector_noise));
      factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
          b, gtsam::imuBias::ConstantBias(), bias_noise));
    } else {
      factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
          gtsam::Symbol('x', i - 1), x, gtsam::Pose3(), pose_noise));
      factors.add(gtsam::BetweenFactor<gtsam::Vector3>(
          gtsam::Symbol('v', i - 1), v, zero_vector, vector_noise));
      factors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
          gtsam::Symbol('b', i - 1), b,
          gtsam::imuBias::ConstantBias(), bias_noise));
    }
    gtsam::Values values;
    values.insert(x, gtsam::Pose3());
    values.insert(v, zero_vector);
    values.insert(b, gtsam::imuBias::ConstantBias());
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
    timestamps[x] = timestamps[v] = timestamps[b] = 0.1 * i;
    smoother.update(factors, values, timestamps);
  }
  require(smoother.timestamps().count(gtsam::Symbol('x', 4)) == 1 &&
              smoother.timestamps().count(gtsam::Symbol('v', 4)) == 1 &&
              smoother.timestamps().count(gtsam::Symbol('b', 4)) == 1,
          "X/V/B graph initialization failed");
  require(smoother.timestamps().count(gtsam::Symbol('x', 0)) == 0,
          "fixed-lag smoother did not marginalize the old state");
}

}  // namespace

int main() {
  try {
    testImuIntervalBoundary();
    testBiasCorrection();
    testLidarFactorAndNoDoubleCountingContract();
    testFixedLagMarginalizationAndGraphInitialization();
    std::cout << "fullstate_shadow_backend_self_test: PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "fullstate_shadow_backend_self_test: FAIL: " << error.what()
              << '\n';
    return 1;
  }
}
