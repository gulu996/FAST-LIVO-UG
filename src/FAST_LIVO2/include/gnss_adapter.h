/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.
*/

#ifndef GNSS_ADAPTER_H
#define GNSS_ADAPTER_H

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <fast_livo/GnssStatus.h>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <gnss_serial_driver/GnssPvtStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

enum class GnssQuality : uint8_t
{
  INVALID = 0,
  SINGLE = 1,
  DIFFERENTIAL = 2,
  RTK_FLOAT = 3,
  RTK_FIXED = 4,
  RECOVERING = 5
};

struct GnssAdapterConfig
{
  bool enable = true;
  std::string input_mode = "legacy_gnss_comm";
  std::string input_topic = "/ublox_driver/receiver_pvt";
  std::string output_odom_topic = "/gnss/enu_odom";
  std::string output_status_topic = "/gnss/status";

  std::string origin_mode = "first_fixed";
  Eigen::Vector3d origin_lla = Eigen::Vector3d::Zero();
  int origin_average_count = 30;
  double origin_average_max_gap_s = 2.0;

  int min_num_sv = 10;
  int max_num_sv = 100;
  double max_pdop = 3.0;
  double max_h_acc_m = 0.30;
  double max_v_acc_m = 0.50;
  double max_vel_acc_mps = 1.0;
  bool reject_nonpositive_pdop = true;
  bool reject_nonpositive_vel_acc = true;

  double min_sigma_xy_m = 0.03;
  double min_sigma_z_m = 0.05;
  double max_sigma_xy_m = 2.0;
  double max_sigma_z_m = 3.0;
  double min_sigma_vel_mps = 0.05;

  int fixed_confirm_count = 5;
  int fixed_lost_count = 3;
  int recovery_confirm_count = 10;

  double max_time_gap_s = 0.5;
  bool require_monotonic_time = true;

  bool accept_rtk_fixed = true;
  bool accept_rtk_float = false;
  bool accept_differential = false;
  bool accept_single = false;

  std::string frame_id = "map";
  std::string child_frame_id = "gnss_antenna";
  double log_interval_s = 1.0;
};

struct GnssAdapterResult
{
  fast_livo::GnssStatus status;
  nav_msgs::Odometry odometry;
  bool publish_odometry = false;
  bool origin_initialized_now = false;
  bool origin_average_accepted = false;
  bool origin_average_skipped = false;
  bool origin_average_reset = false;
  uint32_t origin_average_count = 0;
  uint32_t origin_average_old_count = 0;
  double origin_average_gap_s = 0.0;
  std::string origin_average_skip_reason;
  std::string origin_average_reset_reason;
  std::string warning;
  std::string detail;
};

class GnssAdapter
{
public:
  GnssAdapter();
  explicit GnssAdapter(const GnssAdapterConfig &config);

  bool initialize(ros::NodeHandle &nh);

  // Public to leave one small, ROS-master-free processing seam for replay/self-tests.
  GnssAdapterResult process(const gnss_comm::GnssPVTSolnMsg &message,
                            const ros::Time &callback_time);
  GnssAdapterResult process(const gnss_serial_driver::GnssPvtStamped &message,
                            const ros::Time &callback_time);

private:
  enum class TrackingState
  {
    WAITING = 0,
    ACTIVE_FIXED,
    LOST,
    RECOVERING
  };

  bool loadConfig(ros::NodeHandle &nh, GnssAdapterConfig &config) const;
  bool validateConfig(const GnssAdapterConfig &config) const;
  void resetRuntimeState();
  void legacyPvtCallback(const gnss_comm::GnssPVTSolnMsgConstPtr &message);
  void stampedLocalPvtCallback(
      const gnss_serial_driver::GnssPvtStampedConstPtr &message);
  GnssAdapterResult processPvt(const gnss_comm::GnssPVTSolnMsg &message,
                               const ros::Time &measurement_stamp,
                               bool local_measurement_time_valid,
                               bool valid_for_fusion,
                               bool source_identity_available,
                               uint64_t session_id,
                               uint64_t writer_epoch,
                               const ros::Time &callback_time);

  bool convertGpsToUtc(uint32_t week, double tow, ros::Time &stamp) const;
  GnssQuality classify(const gnss_comm::GnssPVTSolnMsg &message,
                       std::string &reject_reason) const;
  bool passesQualityGates(const gnss_comm::GnssPVTSolnMsg &message,
                          std::string &reject_reason,
                          std::string &warning) const;
  GnssQuality updateFixedState(bool fixed_candidate);
  bool qualityAccepted(GnssQuality quality) const;
  bool updateOrigin(GnssQuality filtered_quality,
                    const Eigen::Vector3d &lla,
                    const Eigen::Vector3d &ecef,
                    std::int64_t measurement_time_ns,
                    GnssAdapterResult &result);
  bool averageOriginPending() const;
  void markOriginAverageSkip(const std::string &reason,
                             GnssAdapterResult &result) const;
  void resetOriginAverage(const std::string &reason,
                          double gap_s,
                          GnssAdapterResult &result);
  void clearOriginAverageState();
  void fillOdometry(const gnss_comm::GnssPVTSolnMsg &message,
                    const ros::Time &stamp,
                    const Eigen::Vector3d &enu,
                    nav_msgs::Odometry &odometry) const;
  void logResult(const gnss_comm::GnssPVTSolnMsg &message,
                 const ros::Time &callback_time,
                 const GnssAdapterResult &result) const;

  GnssAdapterConfig config_;
  ros::Subscriber pvt_subscriber_;
  ros::Publisher odom_publisher_;
  ros::Publisher status_publisher_;

  mutable std::mutex mutex_;
  TrackingState tracking_state_ = TrackingState::WAITING;
  bool was_active_once_ = false;
  uint32_t consecutive_fixed_count_ = 0;
  uint32_t consecutive_lost_count_ = 0;

  bool have_last_measurement_time_ = false;
  std::int64_t last_measurement_time_ns_ = 0;

  bool have_source_identity_ = false;
  uint64_t last_session_id_ = 0;
  uint64_t last_writer_epoch_ = 0;

  bool origin_initialized_ = false;
  Eigen::Vector3d origin_lla_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> origin_ecef_samples_;
  bool have_last_origin_candidate_time_ = false;
  std::int64_t last_origin_candidate_time_ns_ = 0;
};

#endif // GNSS_ADAPTER_H
