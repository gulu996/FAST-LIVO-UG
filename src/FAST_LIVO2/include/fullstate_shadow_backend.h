#pragma once

#include <fast_livo/FullStateLidarGeometry.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace fast_livo_shadow {

struct ImuSample {
  double stamp = 0.0;
  gtsam::Vector3 acceleration = gtsam::Vector3::Zero();
  gtsam::Vector3 angular_velocity = gtsam::Vector3::Zero();
};

struct IntegratedImuSample {
  gtsam::Vector3 acceleration = gtsam::Vector3::Zero();
  gtsam::Vector3 angular_velocity = gtsam::Vector3::Zero();
  double dt = 0.0;
};

struct ImuInterval {
  bool valid = false;
  std::string reason;
  double start = 0.0;
  double end = 0.0;
  double duration = 0.0;
  double sum_dt_squared = 0.0;
  double median_dt = 0.0;
  std::size_t owned_sample_count = 0;
  std::vector<IntegratedImuSample> segments;
};

class ImuIntervalBuffer {
 public:
  bool add(const ImuSample &sample);
  ImuInterval extract(double start, double end, double maximum_gap_s);
  std::uint64_t duplicateCount() const { return duplicate_count_; }
  std::uint64_t nonMonotonicCount() const { return non_monotonic_count_; }
  std::size_t size() const { return samples_.size(); }

 private:
  static ImuSample interpolate(const ImuSample &left, const ImuSample &right,
                               double stamp);
  std::deque<ImuSample> samples_;
  std::uint64_t duplicate_count_ = 0;
  std::uint64_t non_monotonic_count_ = 0;
};

struct LidarFactorBuildResult {
  bool valid = false;
  std::string reason;
  int rank = 0;
  int clipped_negative_eigenvalues = 0;
  gtsam::Vector6 eigenvalues = gtsam::Vector6::Zero();
  gtsam::Matrix6 information = gtsam::Matrix6::Zero();
  gtsam::Vector6 rhs = gtsam::Vector6::Zero();
  gtsam::Pose3 linearization_pose;
  gtsam::LinearContainerFactor::shared_ptr factor;
};

bool lidarOnlyFactorContract(const fast_livo::FullStateLidarGeometry &message,
                             std::string *reason);
LidarFactorBuildResult buildLidarFactor(
    const fast_livo::FullStateLidarGeometry &message, gtsam::Key pose_key);
gtsam::Pose3 poseFromMessage(const geometry_msgs::Pose &pose);

struct FullStateShadowConfig {
  bool enable = false;
  std::string geometry_topic = "/fullstate_shadow/lidar_geometry";
  std::string imu_topic = "/livox/imu";
  std::string odometry_topic = "/fullstate_shadow/optimized_odom";
  std::string output_directory;
  double lag_seconds = 20.0;
  double maximum_imu_gap_s = 0.03;
  double accelerometer_variance = 1.0;
  double gyroscope_variance = 1.0;
  double accel_bias_variance = 0.0001;
  double gyro_bias_variance = 0.0001;
  double integration_covariance = 1e-8;
};

class FullStateShadowBackend {
 public:
  explicit FullStateShadowBackend(ros::NodeHandle &node);
  ~FullStateShadowBackend();
  bool enabled() const { return config_.enable; }

 private:
  void loadParameters(ros::NodeHandle &node);
  void imuCallback(const sensor_msgs::ImuConstPtr &message);
  void geometryCallback(
      const fast_livo::FullStateLidarGeometryConstPtr &message);
  bool initializeGraph(const fast_livo::FullStateLidarGeometry &message,
                       const LidarFactorBuildResult *lidar_factor);
  bool addNode(const fast_livo::FullStateLidarGeometry &message,
               const ImuInterval &interval,
               const LidarFactorBuildResult *lidar_factor);
  void writeState(double stamp, const gtsam::Values &estimate,
                  gtsam::Key pose_key, gtsam::Key velocity_key,
                  gtsam::Key bias_key, double update_ms,
                  double imu_normalized_error, double lidar_normalized_error,
                  const LidarFactorBuildResult *lidar_factor,
                  const fast_livo::FullStateLidarGeometry &message);
  void writeReject(double stamp, const std::string &reason);

  FullStateShadowConfig config_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber geometry_subscriber_;
  ros::Publisher odometry_publisher_;
  ImuIntervalBuffer imu_buffer_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  std::ofstream state_stream_;
  std::ofstream trajectory_stream_;
  std::ofstream event_stream_;
  bool initialized_ = false;
  bool halted_ = false;
  double previous_node_stamp_ = -1.0;
  double accel_scale_ = 1.0;
  gtsam::Vector3 gravity_ = gtsam::Vector3(0.0, 0.0, -9.81);
  gtsam::Vector3 previous_velocity_ = gtsam::Vector3::Zero();
  std::uint64_t next_node_id_ = 0;
  std::uint64_t total_nodes_ = 0;
  std::uint64_t rejected_geometry_messages_ = 0;
  std::uint64_t missing_imu_intervals_ = 0;
  std::uint64_t factorization_failures_ = 0;
};

}  // namespace fast_livo_shadow
