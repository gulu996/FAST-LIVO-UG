/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.
*/

#include "gnss_adapter.h"
#include "gnss_fusion_policy.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <gnss_serial_driver/gnss_math.hpp>
#include <xmlrpcpp/XmlRpcValue.h>

namespace
{
constexpr double kGpsWeekSeconds = 604800.0;
constexpr uint32_t kMaxSupportedGpsWeek = 7000;
constexpr std::int64_t kTimestampEqualityToleranceNs = 1000;
constexpr double kUnknownOrientationVariance = 1e6;
constexpr int kSubscriberQueueSize = 100;

bool xmlRpcToDouble(const XmlRpc::XmlRpcValue &value, double &result)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    result = static_cast<double>(value);
    return std::isfinite(result);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    result = static_cast<int>(value);
    return true;
  }
  return false;
}

bool loadVector3(ros::NodeHandle &nh, const std::string &name, Eigen::Vector3d &result)
{
  XmlRpc::XmlRpcValue values;
  if (!nh.getParam(name, values)) return true;
  if (values.getType() != XmlRpc::XmlRpcValue::TypeArray || values.size() != 3)
  {
    ROS_ERROR("[GNSS_ADAPTER] Parameter %s must be an array of three numbers.", name.c_str());
    return false;
  }

  for (int i = 0; i < 3; ++i)
  {
    double value = 0.0;
    if (!xmlRpcToDouble(values[i], value))
    {
      ROS_ERROR("[GNSS_ADAPTER] Parameter %s[%d] must be finite.", name.c_str(), i);
      return false;
    }
    result[i] = value;
  }
  return true;
}

bool finiteVector(const Eigen::Vector3d &value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

double clampValue(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

uint32_t incrementSaturated(uint32_t value)
{
  return value == std::numeric_limits<uint32_t>::max() ? value : value + 1;
}

const char *qualityName(GnssQuality quality)
{
  switch (quality)
  {
    case GnssQuality::INVALID: return "INVALID";
    case GnssQuality::SINGLE: return "SINGLE";
    case GnssQuality::DIFFERENTIAL: return "DIFFERENTIAL";
    case GnssQuality::RTK_FLOAT: return "RTK_FLOAT";
    case GnssQuality::RTK_FIXED: return "RTK_FIXED";
    case GnssQuality::RECOVERING: return "RECOVERING";
  }
  return "INVALID";
}
} // namespace

GnssAdapter::GnssAdapter()
{
  resetRuntimeState();
}

GnssAdapter::GnssAdapter(const GnssAdapterConfig &config) : config_(config)
{
  resetRuntimeState();
}

bool GnssAdapter::initialize(ros::NodeHandle &nh)
{
  GnssAdapterConfig loaded_config;
  if (!loadConfig(nh, loaded_config) || !validateConfig(loaded_config)) return false;
  GnssFusionPolicy fusion_policy;
  if (!loadGnssFusionPolicy(nh, fusion_policy)) return false;
  loaded_config.enable = fusion_policy.componentEnabled(loaded_config.enable);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = loaded_config;
    resetRuntimeState();
  }

  if (!config_.enable)
  {
    ROS_INFO("[GNSS_ADAPTER] Disabled by gnss_adapter/enable=false.");
    return true;
  }

  if (config_.publish_local_enu_odometry)
    odom_publisher_ = nh.advertise<nav_msgs::Odometry>(config_.output_odom_topic, 10);
  status_publisher_ = nh.advertise<fast_livo::GnssStatus>(config_.output_status_topic, 10);
  if (config_.input_mode == "stamped_local")
  {
    pvt_subscriber_ = nh.subscribe(config_.input_topic, kSubscriberQueueSize,
                                   &GnssAdapter::stampedLocalPvtCallback, this);
  }
  else if (config_.input_mode == "pose_with_covariance_stamped")
  {
    pvt_subscriber_ = nh.subscribe(config_.input_topic, kSubscriberQueueSize,
                                   &GnssAdapter::poseWithCovarianceCallback, this);
  }
  else
  {
    pvt_subscriber_ = nh.subscribe(config_.input_topic, kSubscriberQueueSize,
                                   &GnssAdapter::legacyPvtCallback, this);
  }

  ROS_INFO("[GNSS_ADAPTER] mode=%s input=%s odom=%s status=%s origin_mode=%s local_enu=%s",
           config_.input_mode.c_str(), config_.input_topic.c_str(),
           config_.output_odom_topic.c_str(),
           config_.output_status_topic.c_str(), config_.origin_mode.c_str(),
           config_.publish_local_enu_odometry ? "enabled" : "disabled");
  if (config_.publish_local_enu_odometry && origin_initialized_)
  {
    ROS_INFO_STREAM("[GNSS_ADAPTER_ORIGIN] mode=manual lla=["
                    << std::setprecision(12) << origin_lla_.transpose()
                    << "] ecef=[" << origin_ecef_.transpose() << "]");
  }
  return true;
}

bool GnssAdapter::loadConfig(ros::NodeHandle &nh, GnssAdapterConfig &config) const
{
  ros::NodeHandle params(nh, "gnss_adapter");
  params.param<bool>("enable", config.enable, config.enable);
  params.param<std::string>("input_mode", config.input_mode, config.input_mode);
  params.param<std::string>("input_topic", config.input_topic, config.input_topic);
  params.param<std::string>("output_odom_topic", config.output_odom_topic, config.output_odom_topic);
  params.param<std::string>("output_status_topic", config.output_status_topic, config.output_status_topic);
  params.param<bool>("publish_local_enu_odometry", config.publish_local_enu_odometry,
                     config.publish_local_enu_odometry);
  params.param<int>("pose_input_quality", config.pose_input_quality,
                    config.pose_input_quality);
  params.param<int>("pose_input_num_sv", config.pose_input_num_sv,
                    config.pose_input_num_sv);
  params.param<bool>("gnss_pose_covariance_authoritative",
                     config.gnss_pose_covariance_authoritative,
                     config.gnss_pose_covariance_authoritative);

  params.param<std::string>("origin_mode", config.origin_mode, config.origin_mode);
  if (!loadVector3(params, "origin_lla", config.origin_lla)) return false;
  params.param<int>("origin_average_count", config.origin_average_count, config.origin_average_count);
  params.param<double>("origin_average_max_gap_s", config.origin_average_max_gap_s,
                       config.origin_average_max_gap_s);

  params.param<int>("min_num_sv", config.min_num_sv, config.min_num_sv);
  params.param<int>("max_num_sv", config.max_num_sv, config.max_num_sv);
  params.param<double>("max_pdop", config.max_pdop, config.max_pdop);
  params.param<double>("max_h_acc_m", config.max_h_acc_m, config.max_h_acc_m);
  params.param<double>("max_v_acc_m", config.max_v_acc_m, config.max_v_acc_m);
  params.param<double>("max_vel_acc_mps", config.max_vel_acc_mps, config.max_vel_acc_mps);
  params.param<bool>("reject_nonpositive_pdop", config.reject_nonpositive_pdop,
                     config.reject_nonpositive_pdop);
  params.param<bool>("reject_nonpositive_vel_acc", config.reject_nonpositive_vel_acc,
                     config.reject_nonpositive_vel_acc);

  params.param<double>("min_sigma_xy_m", config.min_sigma_xy_m, config.min_sigma_xy_m);
  params.param<double>("min_sigma_z_m", config.min_sigma_z_m, config.min_sigma_z_m);
  params.param<double>("max_sigma_xy_m", config.max_sigma_xy_m, config.max_sigma_xy_m);
  params.param<double>("max_sigma_z_m", config.max_sigma_z_m, config.max_sigma_z_m);
  params.param<double>("min_sigma_vel_mps", config.min_sigma_vel_mps, config.min_sigma_vel_mps);

  params.param<int>("fixed_confirm_count", config.fixed_confirm_count, config.fixed_confirm_count);
  params.param<int>("fixed_lost_count", config.fixed_lost_count, config.fixed_lost_count);
  params.param<int>("recovery_confirm_count", config.recovery_confirm_count,
                    config.recovery_confirm_count);
  params.param<double>("max_time_gap_s", config.max_time_gap_s, config.max_time_gap_s);
  params.param<bool>("require_monotonic_time", config.require_monotonic_time,
                     config.require_monotonic_time);

  params.param<bool>("accept_rtk_fixed", config.accept_rtk_fixed, config.accept_rtk_fixed);
  params.param<bool>("accept_rtk_float", config.accept_rtk_float, config.accept_rtk_float);
  params.param<bool>("accept_differential", config.accept_differential,
                     config.accept_differential);
  params.param<bool>("accept_single", config.accept_single, config.accept_single);

  params.param<std::string>("frame_id", config.frame_id, config.frame_id);
  params.param<std::string>("child_frame_id", config.child_frame_id, config.child_frame_id);
  params.param<double>("log_interval_s", config.log_interval_s, config.log_interval_s);
  return true;
}

bool GnssAdapter::validateConfig(const GnssAdapterConfig &config) const
{
  if (config.input_mode != "legacy_gnss_comm" &&
      config.input_mode != "stamped_local" &&
      config.input_mode != "pose_with_covariance_stamped")
  {
    ROS_ERROR("[GNSS_ADAPTER] input_mode must be legacy_gnss_comm, stamped_local, or pose_with_covariance_stamped.");
    return false;
  }
  if (config.pose_input_quality < static_cast<int>(GnssQuality::SINGLE) ||
      config.pose_input_quality > static_cast<int>(GnssQuality::RTK_FIXED) ||
      config.pose_input_num_sv < 0 || config.pose_input_num_sv > 255)
  {
    ROS_ERROR("[GNSS_ADAPTER] pose_input_quality must be 1..4 and pose_input_num_sv must be 0..255.");
    return false;
  }
  if (config.input_mode == "pose_with_covariance_stamped" &&
      !config.publish_local_enu_odometry)
  {
    ROS_ERROR("[GNSS_ADAPTER] pose_with_covariance_stamped requires publish_local_enu_odometry=true.");
    return false;
  }
  if (config.origin_mode != "manual" && config.origin_mode != "first_fixed" &&
      config.origin_mode != "average_fixed")
  {
    ROS_ERROR("[GNSS_ADAPTER] origin_mode must be manual, first_fixed, or average_fixed.");
    return false;
  }
  if (config.origin_mode == "manual" &&
      (!finiteVector(config.origin_lla) || config.origin_lla.x() < -90.0 ||
       config.origin_lla.x() > 90.0 || config.origin_lla.y() < -180.0 ||
       config.origin_lla.y() > 180.0))
  {
    ROS_ERROR("[GNSS_ADAPTER] manual origin_lla is invalid.");
    return false;
  }
  if (config.origin_average_count < 1 || config.fixed_confirm_count < 1 ||
      config.fixed_lost_count < 1 || config.recovery_confirm_count < 1)
  {
    ROS_ERROR("[GNSS_ADAPTER] origin/state-machine counts must be at least one.");
    return false;
  }
  if (config.min_num_sv < 0 || config.max_num_sv > 255 ||
      config.min_num_sv > config.max_num_sv)
  {
    ROS_ERROR("[GNSS_ADAPTER] Require 0 <= min_num_sv <= max_num_sv <= 255.");
    return false;
  }
  if (!(config.max_pdop > 0.0 && config.max_h_acc_m > 0.0 &&
        config.max_v_acc_m > 0.0 && config.max_vel_acc_mps > 0.0 &&
        config.min_sigma_xy_m > 0.0 && config.min_sigma_z_m > 0.0 &&
        config.min_sigma_vel_mps > 0.0 &&
        config.max_sigma_xy_m >= config.min_sigma_xy_m &&
        config.max_sigma_z_m >= config.min_sigma_z_m &&
        config.origin_average_max_gap_s > 0.0 &&
        config.max_time_gap_s > 0.0 && config.log_interval_s > 0.0))
  {
    ROS_ERROR("[GNSS_ADAPTER] Accuracy, sigma, origin-average gap, time-gap, and log parameters must be positive and ordered.");
    return false;
  }
  if (config.input_topic.empty() ||
      (config.publish_local_enu_odometry && config.output_odom_topic.empty()) ||
      config.output_status_topic.empty() || config.frame_id.empty() ||
      config.child_frame_id.empty())
  {
    ROS_ERROR("[GNSS_ADAPTER] Topics and frame IDs must be non-empty.");
    return false;
  }
  return true;
}

void GnssAdapter::resetRuntimeState()
{
  tracking_state_ = TrackingState::WAITING;
  was_active_once_ = false;
  consecutive_fixed_count_ = 0;
  consecutive_lost_count_ = 0;
  have_last_measurement_time_ = false;
  last_measurement_time_ns_ = 0;
  have_source_identity_ = false;
  last_session_id_ = 0;
  last_writer_epoch_ = 0;
  origin_initialized_ = false;
  origin_lla_.setZero();
  origin_ecef_.setZero();
  clearOriginAverageState();

  if (config_.origin_mode == "manual" && finiteVector(config_.origin_lla))
  {
    origin_lla_ = config_.origin_lla;
    origin_ecef_ = gnss_serial_driver::geodeticToEcef(origin_lla_);
    origin_initialized_ = finiteVector(origin_ecef_);
  }
}

void GnssAdapter::legacyPvtCallback(
    const gnss_comm::GnssPVTSolnMsgConstPtr &message)
{
  const ros::Time callback_time = ros::Time::now();
  const GnssAdapterResult result = process(*message, callback_time);
  status_publisher_.publish(result.status);
  if (result.publish_odometry) odom_publisher_.publish(result.odometry);
  logResult(*message, callback_time, result);
}

void GnssAdapter::stampedLocalPvtCallback(
    const gnss_serial_driver::GnssPvtStampedConstPtr &message)
{
  const ros::Time callback_time = ros::Time::now();
  const GnssAdapterResult result = process(*message, callback_time);
  status_publisher_.publish(result.status);
  if (result.publish_odometry) odom_publisher_.publish(result.odometry);
  logResult(message->pvt, callback_time, result);
}

void GnssAdapter::poseWithCovarianceCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &message)
{
  const ros::Time callback_time = ros::Time::now();
  const GnssAdapterResult result = process(*message, callback_time);
  status_publisher_.publish(result.status);
  if (result.publish_odometry) odom_publisher_.publish(result.odometry);
  logPoseResult(callback_time, result);
}

bool GnssAdapter::convertGpsToUtc(uint32_t week, double tow, ros::Time &stamp) const
{
  if (week == 0 || week > kMaxSupportedGpsWeek || !std::isfinite(tow) ||
      tow < 0.0 || tow >= kGpsWeekSeconds)
  {
    return false;
  }

  const double unix_utc =
      gnss_serial_driver::gpsWeekTowToUnixUtc(week, tow);
  if (!std::isfinite(unix_utc) || unix_utc <= 0.0 ||
      unix_utc > static_cast<double>(std::numeric_limits<uint32_t>::max()))
  {
    return false;
  }
  stamp.fromSec(unix_utc);
  return !stamp.isZero();
}

GnssQuality GnssAdapter::classify(const gnss_comm::GnssPVTSolnMsg &message,
                                  std::string &reject_reason) const
{
  if (!std::isfinite(message.latitude) || message.latitude < -90.0 || message.latitude > 90.0)
  {
    reject_reason = "INVALID_LATITUDE";
    return GnssQuality::INVALID;
  }
  if (!std::isfinite(message.longitude) || message.longitude < -180.0 || message.longitude > 180.0)
  {
    reject_reason = "INVALID_LONGITUDE";
    return GnssQuality::INVALID;
  }
  if (!std::isfinite(message.altitude))
  {
    reject_reason = "INVALID_ELLIPSOID_HEIGHT";
    return GnssQuality::INVALID;
  }
  if (!std::isfinite(message.height_msl))
  {
    reject_reason = "INVALID_MSL_HEIGHT";
    return GnssQuality::INVALID;
  }
  if (!std::isfinite(message.h_acc) || message.h_acc <= 0.0 ||
      !std::isfinite(message.v_acc) || message.v_acc <= 0.0)
  {
    reject_reason = "INVALID_POSITION_ACCURACY";
    return GnssQuality::INVALID;
  }
  if (!std::isfinite(message.p_dop))
  {
    reject_reason = "INVALID_P_DOP";
    return GnssQuality::INVALID;
  }
  if (!std::isfinite(message.vel_n) || !std::isfinite(message.vel_e) ||
      !std::isfinite(message.vel_d))
  {
    reject_reason = "INVALID_VELOCITY";
    return GnssQuality::INVALID;
  }
  if (!std::isfinite(message.vel_acc))
  {
    reject_reason = "INVALID_VELOCITY_ACCURACY";
    return GnssQuality::INVALID;
  }
  if (message.p_dop <= 0.0 && config_.reject_nonpositive_pdop)
  {
    reject_reason = "NONPOSITIVE_P_DOP";
    return GnssQuality::INVALID;
  }
  if (message.vel_acc <= 0.0 && config_.reject_nonpositive_vel_acc)
  {
    reject_reason = "NONPOSITIVE_VELOCITY_ACCURACY";
    return GnssQuality::INVALID;
  }
  if (!message.valid_fix)
  {
    reject_reason = "INVALID_FIX";
    return GnssQuality::INVALID;
  }
  if (message.fix_type < 3)
  {
    reject_reason = "FIX_TYPE_BELOW_3D";
    return GnssQuality::INVALID;
  }
  if (message.fix_type != 3)
  {
    reject_reason = "UNSUPPORTED_FIX_TYPE";
    return GnssQuality::INVALID;
  }
  if (message.carr_soln > 2)
  {
    reject_reason = "INVALID_CARRIER_SOLUTION";
    return GnssQuality::INVALID;
  }

  if (!message.diff_soln) return GnssQuality::SINGLE;
  if (message.carr_soln == 2) return GnssQuality::RTK_FIXED;
  if (message.carr_soln == 1) return GnssQuality::RTK_FLOAT;
  return GnssQuality::DIFFERENTIAL;
}

bool GnssAdapter::passesQualityGates(const gnss_comm::GnssPVTSolnMsg &message,
                                     std::string &reject_reason,
                                     std::string &warning) const
{
  if (message.num_sv < config_.min_num_sv)
  {
    reject_reason = "NUM_SV_TOO_LOW";
    return false;
  }
  if (message.num_sv > config_.max_num_sv)
  {
    reject_reason = "NUM_SV_OUT_OF_RANGE";
    return false;
  }
  if (message.p_dop > 0.0 && message.p_dop > config_.max_pdop)
  {
    reject_reason = "P_DOP_TOO_LARGE";
    return false;
  }
  if (message.p_dop <= 0.0 && !config_.reject_nonpositive_pdop)
  {
    warning = "NONPOSITIVE_P_DOP_IGNORED";
  }
  if (message.h_acc > config_.max_h_acc_m)
  {
    reject_reason = "H_ACC_TOO_LARGE";
    return false;
  }
  if (message.v_acc > config_.max_v_acc_m)
  {
    reject_reason = "V_ACC_TOO_LARGE";
    return false;
  }
  if (message.vel_acc > 0.0 && message.vel_acc > config_.max_vel_acc_mps)
  {
    reject_reason = "VEL_ACC_TOO_LARGE";
    return false;
  }
  if (message.vel_acc <= 0.0 && !config_.reject_nonpositive_vel_acc)
  {
    if (!warning.empty()) warning += ";";
    warning += "NONPOSITIVE_VELOCITY_ACCURACY_IGNORED";
  }
  return true;
}

GnssQuality GnssAdapter::updateFixedState(bool fixed_candidate)
{
  if (!fixed_candidate)
  {
    consecutive_fixed_count_ = 0;
    consecutive_lost_count_ = incrementSaturated(consecutive_lost_count_);
    if (tracking_state_ == TrackingState::ACTIVE_FIXED)
    {
      if (consecutive_lost_count_ >= static_cast<uint32_t>(config_.fixed_lost_count))
      {
        tracking_state_ = TrackingState::LOST;
        return GnssQuality::INVALID;
      }
      return GnssQuality::RTK_FIXED;
    }
    if (tracking_state_ == TrackingState::RECOVERING)
    {
      if (consecutive_lost_count_ >= static_cast<uint32_t>(config_.fixed_lost_count))
      {
        tracking_state_ = TrackingState::LOST;
        return GnssQuality::INVALID;
      }
      return GnssQuality::RECOVERING;
    }
    return GnssQuality::INVALID;
  }

  consecutive_lost_count_ = 0;
  consecutive_fixed_count_ = incrementSaturated(consecutive_fixed_count_);
  if (tracking_state_ == TrackingState::ACTIVE_FIXED) return GnssQuality::RTK_FIXED;

  if (tracking_state_ == TrackingState::LOST)
  {
    tracking_state_ = TrackingState::RECOVERING;
    consecutive_fixed_count_ = 1;
  }

  if (tracking_state_ == TrackingState::RECOVERING)
  {
    if (consecutive_fixed_count_ >= static_cast<uint32_t>(config_.recovery_confirm_count))
    {
      tracking_state_ = TrackingState::ACTIVE_FIXED;
      was_active_once_ = true;
      return GnssQuality::RTK_FIXED;
    }
    return GnssQuality::RECOVERING;
  }

  if (consecutive_fixed_count_ >= static_cast<uint32_t>(config_.fixed_confirm_count))
  {
    tracking_state_ = TrackingState::ACTIVE_FIXED;
    was_active_once_ = true;
    return GnssQuality::RTK_FIXED;
  }
  return GnssQuality::INVALID;
}

bool GnssAdapter::qualityAccepted(GnssQuality quality) const
{
  switch (quality)
  {
    case GnssQuality::RTK_FIXED: return config_.accept_rtk_fixed;
    case GnssQuality::RTK_FLOAT: return config_.accept_rtk_float;
    case GnssQuality::DIFFERENTIAL: return config_.accept_differential;
    case GnssQuality::SINGLE: return config_.accept_single;
    case GnssQuality::INVALID:
    case GnssQuality::RECOVERING: return false;
  }
  return false;
}

bool GnssAdapter::averageOriginPending() const
{
  return !origin_initialized_ && config_.origin_mode == "average_fixed";
}

void GnssAdapter::markOriginAverageSkip(const std::string &reason,
                                        GnssAdapterResult &result) const
{
  if (!averageOriginPending() || result.origin_average_accepted) return;
  result.origin_average_skipped = true;
  result.origin_average_count =
      static_cast<uint32_t>(origin_ecef_samples_.size());
  result.origin_average_skip_reason = reason.empty() ? "NOT_ORIGIN_CANDIDATE" : reason;
}

void GnssAdapter::clearOriginAverageState()
{
  origin_ecef_samples_.clear();
  have_last_origin_candidate_time_ = false;
  last_origin_candidate_time_ns_ = 0;
}

void GnssAdapter::resetOriginAverage(const std::string &reason,
                                     double gap_s,
                                     GnssAdapterResult &result)
{
  if (!averageOriginPending()) return;
  result.origin_average_reset = true;
  result.origin_average_old_count =
      static_cast<uint32_t>(origin_ecef_samples_.size());
  result.origin_average_reset_reason = reason;
  result.origin_average_gap_s = gap_s;
  clearOriginAverageState();
  result.origin_average_count = 0;
}

bool GnssAdapter::updateOrigin(GnssQuality filtered_quality,
                               const Eigen::Vector3d &lla,
                               const Eigen::Vector3d &ecef,
                               std::int64_t measurement_time_ns,
                               GnssAdapterResult &result)
{
  if (origin_initialized_ || filtered_quality != GnssQuality::RTK_FIXED) return false;

  if (config_.origin_mode == "first_fixed")
  {
    origin_lla_ = lla;
    origin_ecef_ = ecef;
    origin_initialized_ = true;
    return true;
  }
  if (config_.origin_mode != "average_fixed") return false;

  if (have_last_origin_candidate_time_)
  {
    const std::int64_t delta_ns =
        measurement_time_ns - last_origin_candidate_time_ns_;
    if (std::abs(delta_ns) <= kTimestampEqualityToleranceNs)
    {
      markOriginAverageSkip("DUPLICATE_ORIGIN_CANDIDATE", result);
      return false;
    }

    const double delta_s = static_cast<double>(delta_ns) * 1e-9;
    if (delta_s < 0.0)
    {
      resetOriginAverage("TIME_REGRESSION", delta_s, result);
    }
    else if (delta_s > config_.origin_average_max_gap_s)
    {
      resetOriginAverage("VALID_FIXED_GAP", delta_s, result);
    }
  }

  origin_ecef_samples_.push_back(ecef);
  have_last_origin_candidate_time_ = true;
  last_origin_candidate_time_ns_ = measurement_time_ns;
  result.origin_average_accepted = true;
  result.origin_average_count =
      static_cast<uint32_t>(origin_ecef_samples_.size());
  if (origin_ecef_samples_.size() < static_cast<size_t>(config_.origin_average_count))
    return false;

  origin_ecef_.setZero();
  for (const Eigen::Vector3d &sample : origin_ecef_samples_) origin_ecef_ += sample;
  origin_ecef_ /= static_cast<double>(origin_ecef_samples_.size());
  origin_lla_ = gnss_serial_driver::ecefToGeodetic(origin_ecef_);
  origin_initialized_ = finiteVector(origin_lla_) && finiteVector(origin_ecef_);
  clearOriginAverageState();
  return origin_initialized_;
}

GnssAdapterResult GnssAdapter::process(const gnss_comm::GnssPVTSolnMsg &message,
                                       const ros::Time &callback_time)
{
  ros::Time measurement_stamp;
  if (!convertGpsToUtc(message.time.week, message.time.tow, measurement_stamp))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    GnssAdapterResult result;
    fast_livo::GnssStatus &status = result.status;
    status.header.stamp = callback_time;
    status.header.frame_id = config_.frame_id;
    status.valid_fix = message.valid_fix;
    status.diff_soln = message.diff_soln;
    status.fix_type = message.fix_type;
    status.carr_soln = message.carr_soln;
    status.num_sv = message.num_sv;
    status.h_acc = message.h_acc;
    status.v_acc = message.v_acc;
    status.p_dop = message.p_dop;
    status.vel_acc = message.vel_acc;
    status.raw_quality = static_cast<uint8_t>(GnssQuality::INVALID);
    status.filtered_quality = static_cast<uint8_t>(GnssQuality::INVALID);
    status.origin_initialized = origin_initialized_;
    status.accepted = false;
    status.consecutive_fixed_count = consecutive_fixed_count_;
    status.consecutive_lost_count = consecutive_lost_count_;
    status.reject_reason = "INVALID_GPS_TIME";
    std::ostringstream detail;
    detail << "week=" << message.time.week << " tow=" << message.time.tow;
    result.detail = detail.str();
    markOriginAverageSkip("INVALID_GPS_TIME", result);
    return result;
  }
  return processPvt(message, measurement_stamp, true, true, false, 0, 0,
                    callback_time);
}

GnssAdapterResult GnssAdapter::process(
    const gnss_serial_driver::GnssPvtStamped &message,
    const ros::Time &callback_time)
{
  return processPvt(message.pvt, message.header.stamp,
                    message.local_measurement_time_valid,
                    message.valid_for_fusion, true, message.session_id,
                    message.writer_epoch, callback_time);
}

GnssAdapterResult GnssAdapter::process(
    const geometry_msgs::PoseWithCovarianceStamped &message,
    const ros::Time &callback_time)
{
  (void)callback_time;
  std::lock_guard<std::mutex> lock(mutex_);
  GnssAdapterResult result;
  fast_livo::GnssStatus &status = result.status;
  status.header.stamp = message.header.stamp;
  status.header.frame_id = config_.frame_id;
  status.num_sv = static_cast<uint8_t>(config_.pose_input_num_sv);
  status.origin_initialized = true;
  status.covariance_authoritative =
      config_.gnss_pose_covariance_authoritative;

  const auto reject = [&](const std::string &reason) {
    status.raw_quality = static_cast<uint8_t>(GnssQuality::INVALID);
    status.filtered_quality = static_cast<uint8_t>(GnssQuality::INVALID);
    status.valid_fix = false;
    status.accepted = false;
    status.reject_reason = reason;
    return result;
  };
  if (!config_.enable) return reject("ADAPTER_DISABLED");
  if (message.header.stamp.isZero()) return reject("ZERO_MEASUREMENT_TIMESTAMP");

  const geometry_msgs::Point &position = message.pose.pose.position;
  const double covariance_x = message.pose.covariance[0];
  const double covariance_y = message.pose.covariance[7];
  const double covariance_z = message.pose.covariance[14];
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z))
    return reject("INVALID_ENU_POSITION");
  if (!std::isfinite(covariance_x) || !std::isfinite(covariance_y) ||
      !std::isfinite(covariance_z) || covariance_x <= 0.0 ||
      covariance_y <= 0.0 || covariance_z <= 0.0)
    return reject("INVALID_POSITION_COVARIANCE");

  const std::int64_t measurement_time_ns =
      static_cast<std::int64_t>(message.header.stamp.toNSec());
  if (have_last_measurement_time_)
  {
    const std::int64_t delta_ns = measurement_time_ns - last_measurement_time_ns_;
    if (std::abs(delta_ns) <= kTimestampEqualityToleranceNs)
      return reject("DUPLICATE_GNSS_TIME");
    if (delta_ns < 0 && config_.require_monotonic_time)
      return reject("NON_MONOTONIC_TIME");
  }
  have_last_measurement_time_ = true;
  last_measurement_time_ns_ = measurement_time_ns;

  // ponytail: this wire format omits per-epoch solution metadata; use one
  // explicit configured quality instead of inventing a hidden heuristic.
  const GnssQuality quality =
      static_cast<GnssQuality>(config_.pose_input_quality);
  status.raw_quality = static_cast<uint8_t>(quality);
  status.filtered_quality = static_cast<uint8_t>(quality);
  status.valid_fix = true;
  status.diff_soln = quality != GnssQuality::SINGLE;
  status.fix_type = 3;
  status.carr_soln = quality == GnssQuality::RTK_FIXED
                         ? 2
                         : (quality == GnssQuality::RTK_FLOAT ? 1 : 0);
  status.h_acc = std::sqrt(std::max(covariance_x, covariance_y));
  status.v_acc = std::sqrt(covariance_z);
  status.accepted = qualityAccepted(quality);
  if (!status.accepted)
  {
    status.reject_reason = "QUALITY_NOT_ENABLED";
    return result;
  }

  result.odometry.header.stamp = message.header.stamp;
  result.odometry.header.frame_id = config_.frame_id;
  result.odometry.child_frame_id = config_.child_frame_id;
  result.odometry.pose = message.pose;
  result.odometry.pose.pose.orientation.x = 0.0;
  result.odometry.pose.pose.orientation.y = 0.0;
  result.odometry.pose.pose.orientation.z = 0.0;
  result.odometry.pose.pose.orientation.w = 1.0;
  result.odometry.twist.covariance[21] = kUnknownOrientationVariance;
  result.odometry.twist.covariance[28] = kUnknownOrientationVariance;
  result.odometry.twist.covariance[35] = kUnknownOrientationVariance;
  result.publish_odometry = true;
  return result;
}

GnssAdapterResult GnssAdapter::processPvt(
    const gnss_comm::GnssPVTSolnMsg &message,
    const ros::Time &measurement_stamp,
    bool local_measurement_time_valid,
    bool valid_for_fusion,
    bool source_identity_available,
    uint64_t session_id,
    uint64_t writer_epoch,
    const ros::Time &callback_time)
{
  (void)callback_time;
  std::lock_guard<std::mutex> lock(mutex_);
  GnssAdapterResult result;
  fast_livo::GnssStatus &status = result.status;
  status.header.stamp = measurement_stamp;
  status.header.frame_id = config_.frame_id;
  status.valid_fix = message.valid_fix;
  status.diff_soln = message.diff_soln;
  status.fix_type = message.fix_type;
  status.carr_soln = message.carr_soln;
  status.num_sv = message.num_sv;
  status.h_acc = message.h_acc;
  status.v_acc = message.v_acc;
  status.p_dop = message.p_dop;
  status.vel_acc = message.vel_acc;

  if (source_identity_available)
  {
    const bool source_changed =
        have_source_identity_ &&
        (session_id != last_session_id_ || writer_epoch != last_writer_epoch_);
    if (source_changed && averageOriginPending())
    {
      resetOriginAverage("SOURCE_EPOCH_CHANGED", 0.0, result);
      tracking_state_ = TrackingState::WAITING;
      was_active_once_ = false;
      consecutive_fixed_count_ = 0;
      consecutive_lost_count_ = 0;
      have_last_measurement_time_ = false;
      last_measurement_time_ns_ = 0;
    }
    have_source_identity_ = true;
    last_session_id_ = session_id;
    last_writer_epoch_ = writer_epoch;
  }

  const auto reject_without_state_change = [&](const std::string &reason) {
    status.raw_quality = static_cast<uint8_t>(GnssQuality::INVALID);
    status.filtered_quality = static_cast<uint8_t>(GnssQuality::INVALID);
    status.origin_initialized = origin_initialized_;
    status.accepted = false;
    status.consecutive_fixed_count = consecutive_fixed_count_;
    status.consecutive_lost_count = consecutive_lost_count_;
    status.reject_reason = reason;
    markOriginAverageSkip(reason, result);
    return result;
  };
  if (!config_.enable) return reject_without_state_change("ADAPTER_DISABLED");
  if (measurement_stamp.isZero())
    return reject_without_state_change("ZERO_MEASUREMENT_TIMESTAMP");
  if (!local_measurement_time_valid)
    return reject_without_state_change("LOCAL_MEASUREMENT_TIME_INVALID");

  const std::int64_t measurement_time_ns =
      static_cast<std::int64_t>(measurement_stamp.toNSec());
  if (averageOriginPending() && have_last_origin_candidate_time_)
  {
    const std::int64_t candidate_delta_ns =
        measurement_time_ns - last_origin_candidate_time_ns_;
    if (candidate_delta_ns < -kTimestampEqualityToleranceNs)
    {
      resetOriginAverage("TIME_REGRESSION",
                         static_cast<double>(candidate_delta_ns) * 1e-9,
                         result);
      tracking_state_ = TrackingState::WAITING;
      was_active_once_ = false;
      consecutive_fixed_count_ = 0;
      consecutive_lost_count_ = 0;
      have_last_measurement_time_ = false;
      last_measurement_time_ns_ = 0;
    }
  }
  if (!valid_for_fusion)
    return reject_without_state_change("INVALID_FOR_FUSION");

  bool time_gap = false;
  if (have_last_measurement_time_)
  {
    const std::int64_t delta_ns = measurement_time_ns - last_measurement_time_ns_;
    const double delta_s = static_cast<double>(delta_ns) * 1e-9;
    if (std::abs(delta_ns) <= kTimestampEqualityToleranceNs)
    {
      status.raw_quality = static_cast<uint8_t>(GnssQuality::INVALID);
      status.filtered_quality = static_cast<uint8_t>(GnssQuality::INVALID);
      status.origin_initialized = origin_initialized_;
      status.accepted = false;
      status.consecutive_fixed_count = consecutive_fixed_count_;
      status.consecutive_lost_count = consecutive_lost_count_;
      status.reject_reason = "DUPLICATE_GNSS_TIME";
      std::ostringstream detail;
      detail << "current_stamp_ns=" << measurement_time_ns
             << " previous_stamp_ns=" << last_measurement_time_ns_;
      result.detail = detail.str();
      markOriginAverageSkip("DUPLICATE_GNSS_TIME", result);
      return result;
    }
    if (delta_s < 0.0 && config_.require_monotonic_time)
    {
      status.raw_quality = static_cast<uint8_t>(GnssQuality::INVALID);
      status.filtered_quality = static_cast<uint8_t>(GnssQuality::INVALID);
      status.origin_initialized = origin_initialized_;
      status.accepted = false;
      status.consecutive_fixed_count = consecutive_fixed_count_;
      status.consecutive_lost_count = consecutive_lost_count_;
      status.reject_reason = "NON_MONOTONIC_TIME";
      std::ostringstream detail;
      detail << "current_stamp_ns=" << measurement_time_ns
             << " previous_stamp_ns=" << last_measurement_time_ns_;
      result.detail = detail.str();
      markOriginAverageSkip("NON_MONOTONIC_TIME", result);
      return result;
    }
    if (delta_s > config_.max_time_gap_s)
    {
      tracking_state_ = was_active_once_ ? TrackingState::LOST : TrackingState::WAITING;
      consecutive_fixed_count_ = 0;
      consecutive_lost_count_ = 0;
      time_gap = true;
      std::ostringstream detail;
      detail << "current_stamp_ns=" << measurement_time_ns
             << " previous_stamp_ns=" << last_measurement_time_ns_
             << " gap_s=" << delta_s;
      result.detail = detail.str();
    }
  }
  have_last_measurement_time_ = true;
  last_measurement_time_ns_ = measurement_time_ns;

  std::string reject_reason;
  const GnssQuality raw_quality = classify(message, reject_reason);
  bool gates_passed = false;
  if (raw_quality != GnssQuality::INVALID)
  {
    gates_passed = passesQualityGates(message, reject_reason, result.warning);
  }

  const bool fixed_solution = raw_quality == GnssQuality::RTK_FIXED;
  bool fixed_measurement_usable = fixed_solution && gates_passed;
  const GnssQuality fixed_state_quality = updateFixedState(fixed_solution);
  GnssQuality filtered_quality = GnssQuality::INVALID;
  if (raw_quality == GnssQuality::RTK_FIXED)
  {
    filtered_quality = fixed_state_quality;
  }
  else if (fixed_state_quality == GnssQuality::RTK_FIXED ||
      fixed_state_quality == GnssQuality::RECOVERING)
  {
    filtered_quality = fixed_state_quality;
  }
  else if (raw_quality != GnssQuality::INVALID && gates_passed)
  {
    filtered_quality = raw_quality;
  }

  status.raw_quality = static_cast<uint8_t>(raw_quality);
  status.filtered_quality = static_cast<uint8_t>(filtered_quality);

  Eigen::Vector3d lla = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_ecef = Eigen::Vector3d::Zero();
  if (config_.publish_local_enu_odometry && raw_quality != GnssQuality::INVALID)
  {
    lla << message.latitude, message.longitude, message.altitude; // Ellipsoid height only.
    current_ecef = gnss_serial_driver::geodeticToEcef(lla);
    if (!finiteVector(current_ecef))
    {
      gates_passed = false;
      fixed_measurement_usable = false;
      filtered_quality = GnssQuality::INVALID;
      status.filtered_quality = static_cast<uint8_t>(filtered_quality);
      reject_reason = "INVALID_ECEF_RESULT";
    }
  }

  if (config_.publish_local_enu_odometry)
  {
    result.origin_initialized_now = fixed_measurement_usable &&
                                    updateOrigin(fixed_state_quality, lla, current_ecef,
                                                 measurement_time_ns, result);
  }
  // ponytail: in status-only mode the paired Odometry already owns the absolute
  // coordinate basis, so this adapter must not establish or convert another ENU.
  status.origin_initialized = !config_.publish_local_enu_odometry || origin_initialized_;

  bool current_quality_enabled = false;
  if (raw_quality == GnssQuality::RTK_FIXED)
  {
    current_quality_enabled = fixed_measurement_usable &&
                              fixed_state_quality == GnssQuality::RTK_FIXED &&
                              config_.accept_rtk_fixed;
  }
  else if (raw_quality != GnssQuality::INVALID)
  {
    current_quality_enabled = gates_passed && qualityAccepted(raw_quality) &&
                              fixed_state_quality != GnssQuality::RECOVERING;
  }
  status.accepted = current_quality_enabled && status.origin_initialized && !time_gap;
  status.consecutive_fixed_count = consecutive_fixed_count_;
  status.consecutive_lost_count = consecutive_lost_count_;

  if (status.accepted && config_.publish_local_enu_odometry)
  {
    const Eigen::Vector3d enu = gnss_serial_driver::ecefDeltaToEnu(
        origin_lla_, current_ecef - origin_ecef_);
    if (finiteVector(enu))
    {
      fillOdometry(message, measurement_stamp, enu, result.odometry);
      result.publish_odometry = true;
    }
    else
    {
      status.accepted = false;
      reject_reason = "INVALID_ENU_RESULT";
    }
  }

  if (!status.accepted)
  {
    if (time_gap)
    {
      reject_reason = "TIME_GAP_TOO_LARGE";
    }
    else if (reject_reason.empty() && fixed_state_quality == GnssQuality::RECOVERING)
    {
      reject_reason = "GNSS_RECOVERING";
    }
    else if (reject_reason.empty() && raw_quality == GnssQuality::RTK_FIXED &&
             filtered_quality == GnssQuality::INVALID)
    {
      reject_reason = "RTK_NOT_CONFIRMED";
    }
    else if (reject_reason.empty() && !current_quality_enabled)
    {
      reject_reason = "QUALITY_NOT_ENABLED";
    }
    else if (reject_reason.empty() && !origin_initialized_)
    {
      reject_reason = "ORIGIN_NOT_INITIALIZED";
    }
  }
  status.reject_reason = reject_reason;
  markOriginAverageSkip(reject_reason, result);
  return result;
}

void GnssAdapter::fillOdometry(const gnss_comm::GnssPVTSolnMsg &message,
                               const ros::Time &stamp,
                               const Eigen::Vector3d &enu,
                               nav_msgs::Odometry &odometry) const
{
  odometry.header.stamp = stamp;
  odometry.header.frame_id = config_.frame_id;
  odometry.child_frame_id = config_.child_frame_id;
  odometry.pose.pose.position.x = enu.x();
  odometry.pose.pose.position.y = enu.y();
  odometry.pose.pose.position.z = enu.z();

  // Placeholder only: GnssPVTSolnMsg contains no attitude observation.
  odometry.pose.pose.orientation.x = 0.0;
  odometry.pose.pose.orientation.y = 0.0;
  odometry.pose.pose.orientation.z = 0.0;
  odometry.pose.pose.orientation.w = 1.0;

  const double sigma_xy = clampValue(message.h_acc, config_.min_sigma_xy_m,
                                     config_.max_sigma_xy_m);
  const double sigma_z = clampValue(message.v_acc, config_.min_sigma_z_m,
                                    config_.max_sigma_z_m);
  odometry.pose.covariance[0] = sigma_xy * sigma_xy;
  odometry.pose.covariance[7] = sigma_xy * sigma_xy;
  odometry.pose.covariance[14] = sigma_z * sigma_z;
  odometry.pose.covariance[21] = kUnknownOrientationVariance;
  odometry.pose.covariance[28] = kUnknownOrientationVariance;
  odometry.pose.covariance[35] = kUnknownOrientationVariance;

  // Receiver velocity is NED [north, east, down]; output is ENU [east, north, up].
  odometry.twist.twist.linear.x = message.vel_e;
  odometry.twist.twist.linear.y = message.vel_n;
  odometry.twist.twist.linear.z = -message.vel_d;
  const double usable_vel_acc = message.vel_acc > 0.0 ? message.vel_acc : 0.0;
  const double sigma_vel = std::max(usable_vel_acc, config_.min_sigma_vel_mps);
  const double velocity_variance = sigma_vel * sigma_vel;
  odometry.twist.covariance[0] = velocity_variance;
  odometry.twist.covariance[7] = velocity_variance;
  odometry.twist.covariance[14] = velocity_variance;
  odometry.twist.covariance[21] = kUnknownOrientationVariance;
  odometry.twist.covariance[28] = kUnknownOrientationVariance;
  odometry.twist.covariance[35] = kUnknownOrientationVariance;
}

void GnssAdapter::logResult(const gnss_comm::GnssPVTSolnMsg &message,
                            const ros::Time &callback_time,
                            const GnssAdapterResult &result) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double time_delta_s = callback_time.toSec() - result.status.header.stamp.toSec();
  if (!result.warning.empty())
  {
    ROS_WARN_STREAM_THROTTLE(config_.log_interval_s,
                             "[GNSS_ADAPTER] warning=" << result.warning);
  }
  if (result.origin_average_reset)
  {
    ROS_WARN_STREAM("[ORIGIN_AVERAGE_RESET] reason="
                    << result.origin_average_reset_reason
                    << " gap_s=" << result.origin_average_gap_s
                    << " old_count=" << result.origin_average_old_count);
  }
  if (result.origin_average_accepted)
  {
    ROS_INFO_STREAM("[ORIGIN_AVERAGE_ACCEPT] count="
                    << result.origin_average_count << "/"
                    << config_.origin_average_count
                    << " quality=RTK_FIXED stamp="
                    << std::setprecision(12)
                    << result.status.header.stamp.toSec());
  }
  if (result.origin_average_skipped)
  {
    ROS_DEBUG_STREAM_THROTTLE(
        config_.log_interval_s,
        "[ORIGIN_AVERAGE_SKIP] reason="
        << result.origin_average_skip_reason
        << " count_preserved=" << result.origin_average_count << "/"
        << config_.origin_average_count);
  }
  if (result.origin_initialized_now)
  {
    if (config_.origin_mode == "average_fixed")
    {
      ROS_INFO_STREAM("[ORIGIN_AVERAGE_SUCCESS] samples="
                      << result.origin_average_count
                      << " origin_lla=[" << std::setprecision(12)
                      << origin_lla_.transpose() << "]");
    }
    else
    {
      ROS_INFO_STREAM("[GNSS_ADAPTER_ORIGIN] mode=" << config_.origin_mode
                      << " lla=[" << std::setprecision(12)
                      << origin_lla_.transpose()
                      << "] ecef=[" << origin_ecef_.transpose() << "]");
    }
  }
  if (!result.status.accepted)
  {
    ROS_WARN_STREAM_THROTTLE(
        config_.log_interval_s,
        "[GNSS_ADAPTER_REJECT] reason=" << result.status.reject_reason
        << " raw_quality="
        << qualityName(static_cast<GnssQuality>(result.status.raw_quality))
        << " filtered_quality="
        << qualityName(static_cast<GnssQuality>(result.status.filtered_quality))
        << " fixed_count=" << result.status.consecutive_fixed_count
        << "/" << config_.fixed_confirm_count
        << " lost_count=" << result.status.consecutive_lost_count
        << "/" << config_.fixed_lost_count
        << " sv=" << static_cast<int>(message.num_sv)
        << " min_sv=" << config_.min_num_sv
        << " h_acc=" << message.h_acc << " max_h_acc=" << config_.max_h_acc_m
        << " v_acc=" << message.v_acc << " max_v_acc=" << config_.max_v_acc_m
        << " pdop=" << message.p_dop << " max_pdop=" << config_.max_pdop
        << " vel_acc=" << message.vel_acc
        << " max_vel_acc=" << config_.max_vel_acc_mps
        << " week=" << message.time.week << " tow=" << std::fixed
        << std::setprecision(3) << message.time.tow
        << " stamp=" << result.status.header.stamp.toSec()
        << " callback_time=" << callback_time.toSec()
        << " time_delta=" << time_delta_s
        << (result.detail.empty() ? "" : " detail=") << result.detail);
    return;
  }

  const geometry_msgs::Point &position = result.odometry.pose.pose.position;
  const geometry_msgs::Vector3 &velocity = result.odometry.twist.twist.linear;
  std::ostringstream position_detail;
  if (result.publish_odometry)
  {
    position_detail << " local_enu=[" << position.x << " " << position.y
                    << " " << position.z << "] vel_enu=[" << velocity.x
                    << " " << velocity.y << " " << velocity.z << "]";
  }
  else
  {
    position_detail << " position_source=external_enu local_enu=disabled";
  }
  ROS_INFO_STREAM_THROTTLE(
      config_.log_interval_s,
      "[GNSS_ADAPTER] quality="
      << qualityName(static_cast<GnssQuality>(result.status.raw_quality))
      << " active=1 sv=" << static_cast<int>(message.num_sv)
      << " h_acc=" << message.h_acc << " v_acc=" << message.v_acc
      << " pdop=" << message.p_dop
      << position_detail.str()
      << " week=" << message.time.week << " tow=" << std::fixed
      << std::setprecision(3) << message.time.tow
      << " stamp=" << result.status.header.stamp.toSec()
      << " callback_time=" << callback_time.toSec()
      << " time_delta=" << time_delta_s);
}

void GnssAdapter::logPoseResult(const ros::Time &callback_time,
                                const GnssAdapterResult &result) const
{
  const double time_delta_s =
      callback_time.toSec() - result.status.header.stamp.toSec();
  if (!result.status.accepted)
  {
    ROS_WARN_STREAM_THROTTLE(
        config_.log_interval_s,
        "[GNSS_ADAPTER_REJECT] mode=pose_with_covariance_stamped reason="
            << result.status.reject_reason
            << " stamp=" << std::setprecision(15)
            << result.status.header.stamp.toSec());
    return;
  }
  const geometry_msgs::Point &position = result.odometry.pose.pose.position;
  ROS_INFO_STREAM_THROTTLE(
      config_.log_interval_s,
      "[GNSS_ADAPTER] mode=pose_with_covariance_stamped quality="
          << qualityName(static_cast<GnssQuality>(result.status.raw_quality))
          << " quality_source=configured_no_per_epoch_metadata"
          << " covariance_authoritative="
          << (result.status.covariance_authoritative ? 1 : 0)
          << " local_enu=[" << position.x << " " << position.y << " "
          << position.z << "] stamp=" << std::setprecision(15)
          << result.status.header.stamp.toSec()
          << " callback_time=" << callback_time.toSec()
          << " time_delta=" << time_delta_s);
}
