#include "fullstate_shadow_backend.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unistd.h>

#include <boost/make_shared.hpp>

namespace fast_livo_shadow {
namespace {

bool finiteVector(const gtsam::Vector3 &value) { return value.allFinite(); }

gtsam::Vector3 vectorFromMessage(const geometry_msgs::Vector3 &value) {
  return gtsam::Vector3(value.x, value.y, value.z);
}

gtsam::Key poseKey(std::uint64_t index) { return gtsam::Symbol('x', index); }
gtsam::Key velocityKey(std::uint64_t index) {
  return gtsam::Symbol('v', index);
}
gtsam::Key biasKey(std::uint64_t index) { return gtsam::Symbol('b', index); }

double residentMemoryMb() {
  std::ifstream stream("/proc/self/statm");
  long ignored_pages = 0;
  long resident_pages = 0;
  if (!(stream >> ignored_pages >> resident_pages))
    return std::numeric_limits<double>::quiet_NaN();
  return static_cast<double>(resident_pages) *
         static_cast<double>(::sysconf(_SC_PAGESIZE)) / (1024.0 * 1024.0);
}

gtsam::SharedNoiseModel diagonalSigmas(const gtsam::Vector &sigmas) {
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

}  // namespace

bool ImuIntervalBuffer::add(const ImuSample &sample) {
  if (!std::isfinite(sample.stamp) || sample.stamp <= 0.0 ||
      !sample.acceleration.allFinite() ||
      !sample.angular_velocity.allFinite()) {
    return false;
  }
  if (!samples_.empty()) {
    if (sample.stamp == samples_.back().stamp) {
      ++duplicate_count_;
      return false;
    }
    if (sample.stamp < samples_.back().stamp) {
      ++non_monotonic_count_;
      return false;
    }
  }
  samples_.push_back(sample);
  return true;
}

ImuSample ImuIntervalBuffer::interpolate(const ImuSample &left,
                                         const ImuSample &right,
                                         double stamp) {
  if (stamp == left.stamp) return left;
  if (stamp == right.stamp) return right;
  const double alpha = (stamp - left.stamp) / (right.stamp - left.stamp);
  ImuSample result;
  result.stamp = stamp;
  result.acceleration =
      (1.0 - alpha) * left.acceleration + alpha * right.acceleration;
  result.angular_velocity =
      (1.0 - alpha) * left.angular_velocity + alpha * right.angular_velocity;
  return result;
}

ImuInterval ImuIntervalBuffer::extract(double start, double end,
                                       double maximum_gap_s) {
  ImuInterval result;
  result.start = start;
  result.end = end;
  result.duration = end - start;
  if (!std::isfinite(start) || !std::isfinite(end) || end <= start) {
    result.reason = "INVALID_INTERVAL";
    return result;
  }
  if (!std::isfinite(maximum_gap_s) || maximum_gap_s <= 0.0 ||
      samples_.size() < 2) {
    result.reason = "INSUFFICIENT_IMU";
    return result;
  }

  std::size_t left_index = samples_.size();
  std::size_t right_index = samples_.size();
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    if (samples_[i].stamp <= start) left_index = i;
    if (samples_[i].stamp >= end) {
      right_index = i;
      break;
    }
  }
  if (left_index == samples_.size() || right_index == samples_.size() ||
      left_index >= right_index) {
    result.reason = "IMU_BOUNDARY_NOT_BRACKETED";
    return result;
  }
  for (std::size_t i = left_index; i < right_index; ++i) {
    const double gap = samples_[i + 1].stamp - samples_[i].stamp;
    if (!std::isfinite(gap) || gap <= 0.0 || gap > maximum_gap_s) {
      result.reason = "IMU_GAP_TOO_LARGE";
      return result;
    }
  }

  std::vector<ImuSample> knots;
  knots.reserve(right_index - left_index + 2);
  knots.push_back(interpolate(samples_[left_index],
                              samples_[left_index + 1], start));
  for (std::size_t i = left_index + 1; i <= right_index; ++i) {
    if (samples_[i].stamp > start && samples_[i].stamp < end)
      knots.push_back(samples_[i]);
    if (samples_[i].stamp > start && samples_[i].stamp <= end)
      ++result.owned_sample_count;
  }
  knots.push_back(interpolate(samples_[right_index - 1],
                              samples_[right_index], end));

  std::vector<double> durations;
  durations.reserve(knots.size() - 1);
  for (std::size_t i = 0; i + 1 < knots.size(); ++i) {
    const double dt = knots[i + 1].stamp - knots[i].stamp;
    if (!std::isfinite(dt) || dt <= 0.0) {
      result.reason = "NON_POSITIVE_IMU_SEGMENT";
      return result;
    }
    result.segments.push_back(
        {0.5 * (knots[i].acceleration + knots[i + 1].acceleration),
         0.5 * (knots[i].angular_velocity +
                knots[i + 1].angular_velocity),
         dt});
    result.sum_dt_squared += dt * dt;
    durations.push_back(dt);
  }
  if (result.segments.empty()) {
    result.reason = "EMPTY_IMU_INTERVAL";
    return result;
  }
  std::sort(durations.begin(), durations.end());
  const std::size_t middle = durations.size() / 2;
  result.median_dt = durations.size() % 2
                         ? durations[middle]
                         : 0.5 * (durations[middle - 1] + durations[middle]);
  const double integrated_duration = std::accumulate(
      durations.begin(), durations.end(), 0.0);
  if (std::abs(integrated_duration - result.duration) >
      1e-9 * std::max(1.0, result.duration)) {
    result.reason = "IMU_INTERVAL_CLOSURE_FAILED";
    return result;
  }

  while (samples_.size() > 1 && samples_[1].stamp <= end)
    samples_.pop_front();
  result.valid = true;
  result.reason = "OK";
  return result;
}

gtsam::Pose3 poseFromMessage(const geometry_msgs::Pose &pose) {
  const double norm = std::sqrt(
      pose.orientation.w * pose.orientation.w +
      pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z);
  if (!std::isfinite(norm) || norm < 1e-12 ||
      !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z)) {
    throw std::invalid_argument("non-finite pose");
  }
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(pose.orientation.w / norm,
                              pose.orientation.x / norm,
                              pose.orientation.y / norm,
                              pose.orientation.z / norm),
      gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

bool lidarOnlyFactorContract(
    const fast_livo::FullStateLidarGeometry &message,
    std::string *reason) {
  if (message.factor_source != "L1_FINAL_POINT_TO_PLANE") {
    if (reason) *reason = "FORBIDDEN_FACTOR_SOURCE";
    return false;
  }
  if (!message.frozen_submap) {
    if (reason) *reason = "SUBMAP_NOT_FROZEN";
    return false;
  }
  if (!message.geometry_valid || message.residual_dof == 0) {
    if (reason) *reason = "NO_VALID_LIDAR_GEOMETRY";
    return false;
  }
  if (reason) *reason = "OK";
  return true;
}

LidarFactorBuildResult buildLidarFactor(
    const fast_livo::FullStateLidarGeometry &message, gtsam::Key key) {
  LidarFactorBuildResult result;
  if (!lidarOnlyFactorContract(message, &result.reason)) return result;
  try {
    result.linearization_pose = poseFromMessage(message.linearization_pose);
  } catch (const std::exception &) {
    result.reason = "INVALID_LINEARIZATION_POSE";
    return result;
  }

  gtsam::Matrix6 information_fast;
  gtsam::Vector6 rhs_fast;
  for (int row = 0; row < 6; ++row) {
    rhs_fast[row] = message.pose_rhs[row];
    for (int column = 0; column < 6; ++column)
      information_fast(row, column) =
          message.pose_information[row * 6 + column];
  }
  if (!information_fast.allFinite() || !rhs_fast.allFinite()) {
    result.reason = "NONFINITE_NORMAL_EQUATION";
    return result;
  }
  information_fast = 0.5 * (information_fast + information_fast.transpose());
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> solver(information_fast);
  if (solver.info() != Eigen::Success) {
    result.reason = "INFORMATION_EIGENSOLVE_FAILED";
    return result;
  }
  result.eigenvalues = solver.eigenvalues();
  const double maximum_eigenvalue = result.eigenvalues.maxCoeff();
  if (!std::isfinite(maximum_eigenvalue) || maximum_eigenvalue <= 0.0) {
    result.reason = "NO_POSITIVE_INFORMATION";
    return result;
  }
  const double negative_tolerance = 1e-9 * maximum_eigenvalue;
  if (result.eigenvalues.minCoeff() < -negative_tolerance) {
    result.reason = "INFORMATION_NOT_PSD";
    return result;
  }
  const double rank_threshold = 1e-12 * maximum_eigenvalue;
  gtsam::Vector6 clipped = result.eigenvalues;
  gtsam::Vector6 projected_rhs = gtsam::Vector6::Zero();
  double constant = 0.0;
  for (int i = 0; i < 6; ++i) {
    if (clipped[i] < 0.0) {
      clipped[i] = 0.0;
      ++result.clipped_negative_eigenvalues;
    }
    if (clipped[i] > rank_threshold) {
      ++result.rank;
      const gtsam::Vector6 direction = solver.eigenvectors().col(i);
      const double component = direction.dot(rhs_fast);
      projected_rhs += direction * component;
      constant += component * component / clipped[i];
    }
  }
  if (result.rank == 0) {
    result.reason = "ZERO_INFORMATION_RANK";
    return result;
  }
  information_fast = solver.eigenvectors() * clipped.asDiagonal() *
                     solver.eigenvectors().transpose();

  // FAST-LIVO increments are [right-body rotation, world translation].
  // Pose3 local coordinates use [right-body rotation, body translation], so
  // delta_fast = diag(I, R_world_body) * delta_gtsam to first order.
  gtsam::Matrix6 coordinate_jacobian = gtsam::Matrix6::Identity();
  coordinate_jacobian.bottomRightCorner<3, 3>() =
      result.linearization_pose.rotation().matrix();
  result.information = coordinate_jacobian.transpose() * information_fast *
                       coordinate_jacobian;
  result.rhs = coordinate_jacobian.transpose() * projected_rhs;
  result.information =
      0.5 * (result.information + result.information.transpose());

  gtsam::HessianFactor hessian(key, result.information, result.rhs, constant);
  gtsam::Values linearization;
  linearization.insert(key, result.linearization_pose);
  result.factor = boost::make_shared<gtsam::LinearContainerFactor>(
      hessian, linearization);
  result.valid = true;
  result.reason = "OK";
  return result;
}

FullStateShadowBackend::FullStateShadowBackend(ros::NodeHandle &node) {
  loadParameters(node);
  if (!config_.enable) {
    ROS_INFO("[FULLSTATE_SHADOW] disabled");
    return;
  }
  if (config_.lag_seconds <= 0.0 || config_.maximum_imu_gap_s <= 0.0 ||
      config_.accelerometer_variance <= 0.0 ||
      config_.gyroscope_variance <= 0.0 ||
      config_.accel_bias_variance <= 0.0 ||
      config_.gyro_bias_variance <= 0.0 ||
      config_.integration_covariance <= 0.0) {
    throw std::invalid_argument("invalid fullstate shadow parameters");
  }
  if (config_.output_directory.empty())
    throw std::invalid_argument("fullstate shadow output_directory is empty");
  std::filesystem::create_directories(config_.output_directory);
  state_stream_.open(config_.output_directory +
                     "/fullstate_shadow_state.csv");
  trajectory_stream_.open(config_.output_directory +
                          "/fullstate_shadow_online.tum");
  event_stream_.open(config_.output_directory +
                     "/fullstate_shadow_events.csv");
  if (!state_stream_ || !trajectory_stream_ || !event_stream_)
    throw std::runtime_error("failed to open fullstate shadow output files");
  state_stream_
      << "timestamp,px,py,pz,qx,qy,qz,qw,vx,vy,vz,speed,delta_v_norm,"
         "bgx,bgy,bgz,bax,bay,baz,imu_normalized_error,"
         "lidar_normalized_error,update_ms,active_nodes,marginalized_nodes,"
         "active_factors,rss_mb,cov_min_eigenvalue,cov_condition,"
         "pose_cov_trace,velocity_cov_trace,bias_cov_trace,lidar_rank,"
         "lidar_eigen_0,lidar_eigen_1,lidar_eigen_2,lidar_eigen_3,"
         "lidar_eigen_4,lidar_eigen_5,production_commit,production_speed,"
         "imu_duplicates,imu_non_monotonic,missing_imu_intervals,"
         "factorization_failures\n";
  event_stream_ << "timestamp,event,detail\n";
  imu_subscriber_ = node.subscribe(config_.imu_topic, 4000,
                                   &FullStateShadowBackend::imuCallback, this);
  geometry_subscriber_ = node.subscribe(
      config_.geometry_topic, 200,
      &FullStateShadowBackend::geometryCallback, this);
  odometry_publisher_ =
      node.advertise<nav_msgs::Odometry>(config_.odometry_topic, 20);
  ROS_INFO_STREAM("[FULLSTATE_SHADOW] enabled lag=" << config_.lag_seconds
                  << " geometry=" << config_.geometry_topic
                  << " imu=" << config_.imu_topic
                  << " output=" << config_.output_directory);
}

FullStateShadowBackend::~FullStateShadowBackend() {
  if (state_stream_) state_stream_.flush();
  if (trajectory_stream_) trajectory_stream_.flush();
  if (event_stream_) event_stream_.flush();
}

void FullStateShadowBackend::loadParameters(ros::NodeHandle &node) {
  ros::NodeHandle parameters(node, "fullstate_shadow");
  parameters.param("enable", config_.enable, config_.enable);
  parameters.param("geometry_topic", config_.geometry_topic,
                   config_.geometry_topic);
  parameters.param("imu_topic", config_.imu_topic, config_.imu_topic);
  parameters.param("odometry_topic", config_.odometry_topic,
                   config_.odometry_topic);
  parameters.param("output_directory", config_.output_directory,
                   config_.output_directory);
  parameters.param("lag_seconds", config_.lag_seconds, config_.lag_seconds);
  parameters.param("maximum_imu_gap_s", config_.maximum_imu_gap_s,
                   config_.maximum_imu_gap_s);
  parameters.param("accel_bias_variance", config_.accel_bias_variance,
                   config_.accel_bias_variance);
  parameters.param("gyro_bias_variance", config_.gyro_bias_variance,
                   config_.gyro_bias_variance);
  parameters.param("integration_covariance", config_.integration_covariance,
                   config_.integration_covariance);
  node.param("imu/acc_cov", config_.accelerometer_variance,
             config_.accelerometer_variance);
  node.param("imu/gyr_cov", config_.gyroscope_variance,
             config_.gyroscope_variance);
}

void FullStateShadowBackend::imuCallback(
    const sensor_msgs::ImuConstPtr &message) {
  if (!config_.enable || halted_) return;
  ImuSample sample;
  sample.stamp = message->header.stamp.toSec();
  sample.acceleration =
      gtsam::Vector3(message->linear_acceleration.x,
                     message->linear_acceleration.y,
                     message->linear_acceleration.z);
  sample.angular_velocity =
      gtsam::Vector3(message->angular_velocity.x,
                     message->angular_velocity.y,
                     message->angular_velocity.z);
  imu_buffer_.add(sample);
}

void FullStateShadowBackend::geometryCallback(
    const fast_livo::FullStateLidarGeometryConstPtr &message) {
  if (!config_.enable || halted_) return;
  const double stamp = message->header.stamp.toSec();
  if (!std::isfinite(stamp) || stamp <= 0.0 ||
      (previous_node_stamp_ > 0.0 && stamp <= previous_node_stamp_)) {
    writeReject(stamp, "NON_MONOTONIC_GEOMETRY_TIMESTAMP");
    return;
  }

  LidarFactorBuildResult lidar_factor;
  const LidarFactorBuildResult *lidar_factor_pointer = nullptr;
  if (message->geometry_valid) {
    lidar_factor = buildLidarFactor(*message, poseKey(next_node_id_));
    if (lidar_factor.valid)
      lidar_factor_pointer = &lidar_factor;
    else {
      ++rejected_geometry_messages_;
      writeReject(stamp, "LIDAR_FACTOR_" + lidar_factor.reason);
    }
  }

  try {
    if (!initialized_) {
      initializeGraph(*message, lidar_factor_pointer);
      return;
    }
    const ImuInterval interval = imu_buffer_.extract(
        previous_node_stamp_, stamp, config_.maximum_imu_gap_s);
    if (!interval.valid) {
      ++missing_imu_intervals_;
      writeReject(stamp, interval.reason);
      return;
    }
    addNode(*message, interval, lidar_factor_pointer);
  } catch (const std::exception &error) {
    ++factorization_failures_;
    halted_ = true;
    writeReject(stamp, std::string("HALTED_") + error.what());
    ROS_ERROR_STREAM("[FULLSTATE_SHADOW] halted: " << error.what());
  }
}

bool FullStateShadowBackend::initializeGraph(
    const fast_livo::FullStateLidarGeometry &message,
    const LidarFactorBuildResult *lidar_factor) {
  const double stamp = message.header.stamp.toSec();
  const gtsam::Pose3 pose = poseFromMessage(message.production_pose);
  const gtsam::Vector3 velocity = vectorFromMessage(message.production_velocity);
  const gtsam::Vector3 gyro_bias = vectorFromMessage(message.production_gyro_bias);
  const gtsam::Vector3 accel_bias =
      vectorFromMessage(message.production_accel_bias);
  gravity_ = vectorFromMessage(message.gravity);
  if (!pose.matrix().allFinite() || !finiteVector(velocity) ||
      !finiteVector(gyro_bias) || !finiteVector(accel_bias) ||
      !finiteVector(gravity_) || gravity_.norm() < 5.0 ||
      gravity_.norm() > 15.0 || !std::isfinite(message.accel_scale) ||
      message.accel_scale <= 0.0) {
    writeReject(stamp, "INVALID_INITIAL_STATE");
    return false;
  }
  accel_scale_ = message.accel_scale;
  smoother_.reset(new gtsam::IncrementalFixedLagSmoother(config_.lag_seconds));
  const gtsam::Key x = poseKey(next_node_id_);
  const gtsam::Key v = velocityKey(next_node_id_);
  const gtsam::Key b = biasKey(next_node_id_);
  const gtsam::imuBias::ConstantBias bias(accel_bias, gyro_bias);

  gtsam::NonlinearFactorGraph factors;
  gtsam::Vector6 pose_sigmas;
  pose_sigmas << 0.1, 0.1, 0.2, 1.0, 1.0, 1.0;
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      x, pose, diagonalSigmas(pose_sigmas)));
  factors.add(gtsam::PriorFactor<gtsam::Vector3>(
      v, velocity, diagonalSigmas(gtsam::Vector3::Ones())));
  gtsam::Vector6 bias_sigmas;
  bias_sigmas << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01;
  factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      b, bias, diagonalSigmas(bias_sigmas)));
  if (lidar_factor) factors.add(lidar_factor->factor);

  gtsam::Values values;
  values.insert(x, pose);
  values.insert(v, velocity);
  values.insert(b, bias);
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  timestamps[x] = timestamps[v] = timestamps[b] = stamp;
  const auto begin = std::chrono::steady_clock::now();
  smoother_->update(factors, values, timestamps);
  const double update_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - begin)
                               .count();
  const gtsam::Values estimate = smoother_->calculateEstimate();
  writeState(stamp, estimate, x, v, b, update_ms,
             std::numeric_limits<double>::quiet_NaN(),
             lidar_factor && lidar_factor->factor
                 ? 2.0 * lidar_factor->factor->error(estimate) /
                       std::max(1, lidar_factor->rank)
                 : std::numeric_limits<double>::quiet_NaN(),
             lidar_factor, message);
  previous_velocity_ = estimate.at<gtsam::Vector3>(v);
  previous_node_stamp_ = stamp;
  initialized_ = true;
  ++next_node_id_;
  ++total_nodes_;
  return true;
}

bool FullStateShadowBackend::addNode(
    const fast_livo::FullStateLidarGeometry &message,
    const ImuInterval &interval,
    const LidarFactorBuildResult *lidar_factor) {
  const std::uint64_t previous_id = next_node_id_ - 1;
  const gtsam::Key previous_x = poseKey(previous_id);
  const gtsam::Key previous_v = velocityKey(previous_id);
  const gtsam::Key previous_b = biasKey(previous_id);
  const gtsam::Key x = poseKey(next_node_id_);
  const gtsam::Key v = velocityKey(next_node_id_);
  const gtsam::Key b = biasKey(next_node_id_);
  const gtsam::Pose3 previous_pose =
      smoother_->calculateEstimate<gtsam::Pose3>(previous_x);
  const gtsam::Vector3 previous_velocity =
      smoother_->calculateEstimate<gtsam::Vector3>(previous_v);
  const gtsam::imuBias::ConstantBias previous_bias =
      smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(previous_b);

  const boost::shared_ptr<gtsam::PreintegrationParams> parameters(
      new gtsam::PreintegrationParams(gravity_));
  // FAST-LIVO stores per-sample variances and propagates variance*dt^2.
  // GTSAM expects continuous covariance density, so q=variance*median(dt)
  // makes q*sum(dt) match variance*sum(dt^2) for near-uniform sampling.
  parameters->accelerometerCovariance =
      gtsam::Matrix3::Identity() * config_.accelerometer_variance *
      interval.median_dt;
  parameters->gyroscopeCovariance =
      gtsam::Matrix3::Identity() * config_.gyroscope_variance *
      interval.median_dt;
  parameters->integrationCovariance =
      gtsam::Matrix3::Identity() * config_.integration_covariance;
  gtsam::PreintegratedImuMeasurements preintegration(parameters,
                                                      previous_bias);
  for (const IntegratedImuSample &sample : interval.segments) {
    preintegration.integrateMeasurement(accel_scale_ * sample.acceleration,
                                        sample.angular_velocity, sample.dt);
  }
  const gtsam::NavState predicted = preintegration.predict(
      gtsam::NavState(previous_pose, previous_velocity), previous_bias);

  gtsam::NonlinearFactorGraph factors;
  const auto imu_factor = boost::make_shared<gtsam::ImuFactor>(
      previous_x, previous_v, x, v, previous_b, preintegration);
  factors.add(imu_factor);
  gtsam::Vector6 bias_sigmas;
  bias_sigmas.head<3>().setConstant(std::sqrt(
      config_.accel_bias_variance * interval.sum_dt_squared));
  bias_sigmas.tail<3>().setConstant(std::sqrt(
      config_.gyro_bias_variance * interval.sum_dt_squared));
  factors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      previous_b, b, gtsam::imuBias::ConstantBias(),
      diagonalSigmas(bias_sigmas)));
  if (lidar_factor) factors.add(lidar_factor->factor);

  gtsam::Values values;
  values.insert(x, predicted.pose());
  values.insert(v, predicted.v());
  values.insert(b, previous_bias);
  const double stamp = message.header.stamp.toSec();
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  timestamps[x] = timestamps[v] = timestamps[b] = stamp;
  const auto begin = std::chrono::steady_clock::now();
  smoother_->update(factors, values, timestamps);
  const double update_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - begin)
                               .count();
  const gtsam::Values estimate = smoother_->calculateEstimate();
  const double imu_normalized_error =
      2.0 * imu_factor->error(estimate) / 9.0;
  const double lidar_normalized_error =
      lidar_factor && lidar_factor->factor
          ? 2.0 * lidar_factor->factor->error(estimate) /
                std::max(1, lidar_factor->rank)
          : std::numeric_limits<double>::quiet_NaN();
  writeState(stamp, estimate, x, v, b, update_ms, imu_normalized_error,
             lidar_normalized_error, lidar_factor, message);
  previous_velocity_ = estimate.at<gtsam::Vector3>(v);
  previous_node_stamp_ = stamp;
  ++next_node_id_;
  ++total_nodes_;
  return true;
}

void FullStateShadowBackend::writeState(
    double stamp, const gtsam::Values &estimate, gtsam::Key x, gtsam::Key v,
    gtsam::Key b, double update_ms, double imu_normalized_error,
    double lidar_normalized_error,
    const LidarFactorBuildResult *lidar_factor,
    const fast_livo::FullStateLidarGeometry &message) {
  const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(x);
  const gtsam::Vector3 velocity = estimate.at<gtsam::Vector3>(v);
  const gtsam::imuBias::ConstantBias bias =
      estimate.at<gtsam::imuBias::ConstantBias>(b);
  const gtsam::Matrix pose_covariance = smoother_->marginalCovariance(x);
  const gtsam::Matrix velocity_covariance = smoother_->marginalCovariance(v);
  const gtsam::Matrix bias_covariance = smoother_->marginalCovariance(b);
  gtsam::Matrix covariance = gtsam::Matrix::Zero(15, 15);
  covariance.block(0, 0, 6, 6) = pose_covariance;
  covariance.block(6, 6, 3, 3) = velocity_covariance;
  covariance.block(9, 9, 6, 6) = bias_covariance;
  covariance = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix> covariance_solver(covariance);
  double covariance_minimum = std::numeric_limits<double>::quiet_NaN();
  double covariance_condition = std::numeric_limits<double>::quiet_NaN();
  if (covariance_solver.info() == Eigen::Success) {
    covariance_minimum = covariance_solver.eigenvalues().minCoeff();
    const double maximum = covariance_solver.eigenvalues().maxCoeff();
    if (covariance_minimum > 0.0)
      covariance_condition = maximum / covariance_minimum;
  }
  std::size_t active_factors = 0;
  for (const auto &factor : smoother_->getFactors())
    if (factor) ++active_factors;
  std::size_t active_nodes = 0;
  for (const auto &entry : smoother_->timestamps())
    if (gtsam::Symbol(entry.first).chr() == 'x') ++active_nodes;
  const std::size_t marginalized_nodes =
      total_nodes_ + 1 >= active_nodes ? total_nodes_ + 1 - active_nodes : 0;
  const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  const gtsam::Vector3 production_velocity =
      vectorFromMessage(message.production_velocity);
  const double delta_velocity_norm = (velocity - previous_velocity_).norm();

  state_stream_ << std::setprecision(17) << stamp << ',' << pose.x() << ','
                << pose.y() << ',' << pose.z() << ',' << quaternion.x() << ','
                << quaternion.y() << ',' << quaternion.z() << ','
                << quaternion.w() << ',' << velocity.x() << ','
                << velocity.y() << ',' << velocity.z() << ','
                << velocity.norm() << ',' << delta_velocity_norm << ','
                << bias.gyroscope().x() << ',' << bias.gyroscope().y() << ','
                << bias.gyroscope().z() << ',' << bias.accelerometer().x()
                << ',' << bias.accelerometer().y() << ','
                << bias.accelerometer().z() << ',' << imu_normalized_error
                << ',' << lidar_normalized_error << ',' << update_ms << ','
                << active_nodes << ',' << marginalized_nodes << ','
                << active_factors << ',' << residentMemoryMb() << ','
                << covariance_minimum << ',' << covariance_condition << ','
                << pose_covariance.trace() << ','
                << velocity_covariance.trace() << ','
                << bias_covariance.trace() << ','
                << (lidar_factor ? lidar_factor->rank : 0);
  for (int i = 0; i < 6; ++i)
    state_stream_ << ','
                  << (lidar_factor
                          ? lidar_factor->eigenvalues[i]
                          : std::numeric_limits<double>::quiet_NaN());
  state_stream_ << ',' << static_cast<int>(message.production_commit) << ','
                << production_velocity.norm() << ','
                << imu_buffer_.duplicateCount() << ','
                << imu_buffer_.nonMonotonicCount() << ','
                << missing_imu_intervals_ << ',' << factorization_failures_
                << '\n';

  trajectory_stream_ << std::setprecision(17) << stamp << ' ' << pose.x()
                     << ' ' << pose.y() << ' ' << pose.z() << ' '
                     << quaternion.x() << ' ' << quaternion.y() << ' '
                     << quaternion.z() << ' ' << quaternion.w() << '\n';
  nav_msgs::Odometry odometry;
  odometry.header = message.header;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "fullstate_shadow_body";
  odometry.pose.pose.position.x = pose.x();
  odometry.pose.pose.position.y = pose.y();
  odometry.pose.pose.position.z = pose.z();
  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.twist.twist.linear.x = velocity.x();
  odometry.twist.twist.linear.y = velocity.y();
  odometry.twist.twist.linear.z = velocity.z();
  odometry_publisher_.publish(odometry);
  if ((total_nodes_ + 1) % 100 == 0) {
    state_stream_.flush();
    trajectory_stream_.flush();
    event_stream_.flush();
  }
}

void FullStateShadowBackend::writeReject(double stamp,
                                         const std::string &reason) {
  if (event_stream_)
    event_stream_ << std::setprecision(17) << stamp << ",REJECT," << reason
                  << '\n';
  ROS_WARN_STREAM_THROTTLE(1.0, "[FULLSTATE_SHADOW_REJECT] stamp=" << stamp
                                  << " reason=" << reason);
}

}  // namespace fast_livo_shadow
