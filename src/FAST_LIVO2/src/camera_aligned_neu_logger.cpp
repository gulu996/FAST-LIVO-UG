#include "camera_aligned_neu_logger.h"

#include "fast_livo/RtkBackendStatus.h"
#include "run_log_directory.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <utility>
#include <vector>

namespace fast_livo_camera_neu {
namespace {

constexpr double kNanosecondsPerSecond = 1e9;

bool finitePose(const TimedPose &pose) {
  return pose.position.allFinite() && pose.orientation.coeffs().allFinite() &&
         pose.orientation.norm() > 1e-12;
}

TimedPose odometryPose(const nav_msgs::Odometry &message) {
  TimedPose pose;
  pose.stamp_ns = static_cast<std::int64_t>(message.header.stamp.toNSec());
  pose.position = Eigen::Vector3d(message.pose.pose.position.x,
                                  message.pose.pose.position.y,
                                  message.pose.pose.position.z);
  pose.orientation = Eigen::Quaterniond(
      message.pose.pose.orientation.w, message.pose.pose.orientation.x,
      message.pose.pose.orientation.y, message.pose.pose.orientation.z);
  if (pose.orientation.norm() > 1e-12) pose.orientation.normalize();
  return pose;
}

TimedPose transformPose(const geometry_msgs::TransformStamped &message) {
  TimedPose pose;
  pose.stamp_ns = static_cast<std::int64_t>(message.header.stamp.toNSec());
  pose.position = Eigen::Vector3d(message.transform.translation.x,
                                  message.transform.translation.y,
                                  message.transform.translation.z);
  pose.orientation = Eigen::Quaterniond(
      message.transform.rotation.w, message.transform.rotation.x,
      message.transform.rotation.y, message.transform.rotation.z);
  if (pose.orientation.norm() > 1e-12) pose.orientation.normalize();
  return pose;
}

bool loadBodyToCameraPose(ros::NodeHandle &node, TimedPose &body_to_camera) {
  std::vector<double> body_lidar_rotation;
  std::vector<double> body_lidar_translation;
  std::vector<double> camera_lidar_rotation;
  std::vector<double> camera_lidar_translation;
  if (!node.getParam("extrin_calib/extrinsic_R", body_lidar_rotation) ||
      !node.getParam("extrin_calib/extrinsic_T", body_lidar_translation) ||
      !node.getParam("extrin_calib/Rcl", camera_lidar_rotation) ||
      !node.getParam("extrin_calib/Pcl", camera_lidar_translation) ||
      body_lidar_rotation.size() != 9 || body_lidar_translation.size() != 3 ||
      camera_lidar_rotation.size() != 9 ||
      camera_lidar_translation.size() != 3) {
    return false;
  }

  using RowMajorMatrix3d =
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  const Eigen::Matrix3d rotation_body_lidar =
      Eigen::Map<const RowMajorMatrix3d>(body_lidar_rotation.data());
  const Eigen::Vector3d translation_body_lidar =
      Eigen::Map<const Eigen::Vector3d>(body_lidar_translation.data());
  const Eigen::Matrix3d rotation_camera_lidar =
      Eigen::Map<const RowMajorMatrix3d>(camera_lidar_rotation.data());
  const Eigen::Vector3d translation_camera_lidar =
      Eigen::Map<const Eigen::Vector3d>(camera_lidar_translation.data());

  // T_body_camera = T_body_lidar * inverse(T_camera_lidar).
  const Eigen::Matrix3d rotation_body_camera =
      rotation_body_lidar * rotation_camera_lidar.transpose();
  body_to_camera.position = translation_body_lidar -
      rotation_body_camera * translation_camera_lidar;
  body_to_camera.orientation =
      Eigen::Quaterniond(rotation_body_camera).normalized();
  return finitePose(body_to_camera) &&
         std::fabs(rotation_body_camera.determinant() - 1.0) < 1e-5;
}

void insertOrReplace(std::deque<TimedPose> &poses, const TimedPose &pose) {
  const auto it = std::lower_bound(
      poses.begin(), poses.end(), pose.stamp_ns,
      [](const TimedPose &candidate, std::int64_t stamp_ns) {
        return candidate.stamp_ns < stamp_ns;
      });
  if (it != poses.end() && it->stamp_ns == pose.stamp_ns)
    *it = pose;
  else
    poses.insert(it, pose);
}

void prunePoseBuffer(std::deque<TimedPose> &poses, double buffer_seconds) {
  if (poses.size() < 3 || buffer_seconds <= 0.0) return;
  const std::int64_t cutoff = poses.back().stamp_ns - static_cast<std::int64_t>(
      std::llround(buffer_seconds * kNanosecondsPerSecond));
  // Keep one sample before the window so interpolation remains bracketed.
  while (poses.size() > 2 && poses[1].stamp_ns < cutoff) poses.pop_front();
}

class CameraAlignedNeuLogger {
 public:
  explicit CameraAlignedNeuLogger(ros::NodeHandle &node) : node_(node) {
    node_.param("camera_aligned_neu/enable", enabled_, false);
    if (!enabled_) {
      ROS_INFO("[CameraAlignedNEU] Disabled.");
      return;
    }

    node_.param("camera_aligned_neu/max_raw_interp_gap_s", max_raw_gap_s_,
                0.25);
    node_.param("camera_aligned_neu/buffer_seconds", buffer_seconds_, 5.0);
    node_.param("camera_aligned_neu/write_csv", write_csv_, true);
    node_.param("camera_aligned_neu/write_txt", write_txt_, true);
    node_.param<std::string>("common/img_topic", image_topic_,
                             "/left_camera/image");
    node_.param<std::string>("rtk_backend/raw_odom_topic", raw_odom_topic_,
                             "/backend/livo_odom_raw");
    node_.param<std::string>("rtk_backend/map_to_odom_topic",
                             map_to_odom_topic_,
                             "/rtk_backend/map_to_odom");
    node_.param<std::string>("rtk_backend/status_topic", status_topic_,
                             "/rtk_backend/status");

    if (!(max_raw_gap_s_ > 0.0) || !(buffer_seconds_ > max_raw_gap_s_)) {
      ROS_ERROR("[CameraAlignedNEU] Invalid buffer/gap configuration; logger disabled.");
      enabled_ = false;
      return;
    }
    if (!loadBodyToCameraPose(node_, body_to_camera_)) {
      ROS_ERROR("[CameraAlignedNEU] Camera extrinsic is missing or invalid; logger disabled.");
      enabled_ = false;
      return;
    }

    std::string run_directory;
    const auto directory_state = fast_livo_logging::waitForRunLogDirectory(
        node_, run_directory);
    if (directory_state != fast_livo_logging::RunLogDirectoryState::READY) {
      ROS_ERROR("[CameraAlignedNEU] Main run directory unavailable; logger disabled.");
      enabled_ = false;
      return;
    }
    if (!run_directory.empty() && run_directory.back() != '/')
      run_directory.push_back('/');
    csv_path_ = run_directory + "rtk_camera_pose_online.csv";
    txt_path_ = run_directory + "rtk_camera_pose_online.txt";

    if (write_csv_) {
      csv_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (!csv_.is_open()) {
        ROS_ERROR("[CameraAlignedNEU] Cannot open %s; logger disabled.",
                  csv_path_.c_str());
        enabled_ = false;
        return;
      }
      csv_ << "timestamp,north_m,east_m,up_m,qx_enu_camera,qy_enu_camera,"
              "qz_enu_camera,qw_enu_camera,valid,reason,raw_t0,raw_t1,"
              "map_to_odom_stamp\n";
    }
    if (write_txt_) {
      txt_.open(txt_path_, std::ios::out | std::ios::trunc);
      if (!txt_.is_open()) {
        ROS_ERROR("[CameraAlignedNEU] Cannot open %s; TXT output disabled.",
                  txt_path_.c_str());
        write_txt_ = false;
      }
    }

    image_subscriber_ = node_.subscribe(
        image_topic_, 256, &CameraAlignedNeuLogger::imageCallback, this);
    raw_odom_subscriber_ = node_.subscribe(
        raw_odom_topic_, 256, &CameraAlignedNeuLogger::rawOdomCallback, this);
    map_to_odom_subscriber_ = node_.subscribe(
        map_to_odom_topic_, 64,
        &CameraAlignedNeuLogger::mapToOdomCallback, this);
    status_subscriber_ = node_.subscribe(
        status_topic_, 32, &CameraAlignedNeuLogger::statusCallback, this);
    flush_timer_ = node_.createTimer(
        ros::Duration(1.0), &CameraAlignedNeuLogger::flushTimerCallback, this);
    ROS_INFO("[CameraAlignedNEU] Writing camera-stamped online camera pose to %s",
             csv_path_.c_str());
  }

  ~CameraAlignedNeuLogger() {
    if (!enabled_) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      drainPending(true);
      if (csv_.is_open()) csv_.flush();
      if (txt_.is_open()) txt_.flush();
    }
    printSummary();
  }

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr &message) {
    if (!enabled_) return;
    const std::int64_t stamp_ns =
        static_cast<std::int64_t>(message->header.stamp.toNSec());
    std::lock_guard<std::mutex> lock(mutex_);
    ++camera_received_;
    if (first_camera_stamp_ns_ < 0) first_camera_stamp_ns_ = stamp_ns;
    last_camera_stamp_ns_ = stamp_ns;
    const auto it = std::upper_bound(pending_camera_stamps_.begin(),
                                     pending_camera_stamps_.end(), stamp_ns);
    pending_camera_stamps_.insert(it, stamp_ns);
    drainPending(false);
  }

  void rawOdomCallback(const nav_msgs::OdometryConstPtr &message) {
    if (!enabled_) return;
    const TimedPose pose = odometryPose(*message);
    if (!finitePose(pose)) {
      ROS_WARN_THROTTLE(5.0, "[CameraAlignedNEU] Ignoring invalid raw odometry pose.");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    insertOrReplace(raw_poses_, pose);
    prunePoseBuffer(raw_poses_, buffer_seconds_);
    drainPending(false);
  }

  void mapToOdomCallback(
      const geometry_msgs::TransformStampedConstPtr &message) {
    if (!enabled_) return;
    if (message->header.frame_id != "map") {
      ROS_WARN_THROTTLE(5.0,
                        "[CameraAlignedNEU] map_to_odom parent is '%s', expected 'map'.",
                        message->header.frame_id.c_str());
    }
    const TimedPose pose = transformPose(*message);
    if (!finitePose(pose)) {
      ROS_WARN_THROTTLE(5.0, "[CameraAlignedNEU] Ignoring invalid map_to_odom pose.");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    insertOrReplace(map_to_odom_poses_, pose);
    prunePoseBuffer(map_to_odom_poses_, buffer_seconds_);
    drainPending(false);
  }

  void statusCallback(const fast_livo::RtkBackendStatusConstPtr &message) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    backend_alignment_ready_ = message->alignment_ready;
    backend_status_seen_ = true;
    if (message->alignment_ready && first_alignment_status_stamp_ns_ < 0)
      first_alignment_status_stamp_ns_ =
          static_cast<std::int64_t>(message->header.stamp.toNSec());
    drainPending(false);
  }

  void flushTimerCallback(const ros::TimerEvent &) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (csv_.is_open()) csv_.flush();
    if (txt_.is_open()) txt_.flush();
  }

  bool findRawBracket(std::int64_t stamp_ns, TimedPose &before,
                      TimedPose &after) const {
    const auto upper = std::lower_bound(
        raw_poses_.begin(), raw_poses_.end(), stamp_ns,
        [](const TimedPose &pose, std::int64_t query) {
          return pose.stamp_ns < query;
        });
    if (upper == raw_poses_.end()) return false;
    if (upper->stamp_ns == stamp_ns) {
      before = *upper;
      after = *upper;
      return true;
    }
    if (upper == raw_poses_.begin()) return false;
    before = *std::prev(upper);
    after = *upper;
    return true;
  }

  void drainPending(bool force) {
    while (!pending_camera_stamps_.empty()) {
      const std::int64_t camera_stamp_ns = pending_camera_stamps_.front();

      if (map_to_odom_poses_.empty()) {
        if (backend_status_seen_ && !backend_alignment_ready_) {
          writeInvalid(camera_stamp_ns, "GLOBAL_ALIGNMENT_NOT_READY");
          pending_camera_stamps_.pop_front();
          continue;
        }
        if (!force) return;
        writeInvalid(camera_stamp_ns, "GLOBAL_ALIGNMENT_NOT_READY");
        pending_camera_stamps_.pop_front();
        continue;
      }

      const std::int64_t first_map_stamp_ns = map_to_odom_poses_.front().stamp_ns;
      if (camera_stamp_ns < first_map_stamp_ns) {
        writeInvalid(camera_stamp_ns, "GLOBAL_ALIGNMENT_NOT_READY");
        pending_camera_stamps_.pop_front();
        continue;
      }

      TimedPose raw_before;
      TimedPose raw_after;
      if (!findRawBracket(camera_stamp_ns, raw_before, raw_after)) {
        const bool future_raw_may_arrive =
            raw_poses_.empty() || raw_poses_.back().stamp_ns < camera_stamp_ns;
        if (!force && future_raw_may_arrive) return;
        writeInvalid(camera_stamp_ns, "RAW_INTERPOLATION_GAP");
        pending_camera_stamps_.pop_front();
        continue;
      }

      TimedPose raw_interpolated;
      double bracket_span_s = 0.0;
      if (!interpolatePose(raw_before, raw_after, camera_stamp_ns,
                           max_raw_gap_s_, raw_interpolated,
                           bracket_span_s)) {
        max_raw_bracket_span_s_ =
            std::max(max_raw_bracket_span_s_, bracket_span_s);
        writeInvalid(camera_stamp_ns, "RAW_INTERPOLATION_GAP",
                     &raw_before, &raw_after, nullptr);
        pending_camera_stamps_.pop_front();
        continue;
      }
      max_raw_bracket_span_s_ =
          std::max(max_raw_bracket_span_s_, bracket_span_s);

      const TimedPose *correction =
          latestCausalPose(map_to_odom_poses_, camera_stamp_ns);
      if (correction == nullptr) {
        writeInvalid(camera_stamp_ns,
                     backend_status_seen_ && !backend_alignment_ready_
                         ? "GLOBAL_ALIGNMENT_NOT_READY"
                         : "MISSING_MAP_TO_ODOM",
                     &raw_before, &raw_after, nullptr);
        pending_camera_stamps_.pop_front();
        continue;
      }

      const TimedPose map_body =
          composePoses(*correction, raw_interpolated, camera_stamp_ns);
      const TimedPose map_camera =
          composePoses(map_body, body_to_camera_, camera_stamp_ns);
      writeValid(camera_stamp_ns, map_camera, raw_before, raw_after,
                 *correction);
      pending_camera_stamps_.pop_front();
    }
  }

  void writeInvalid(std::int64_t stamp_ns, const char *reason,
                    const TimedPose *raw_before = nullptr,
                    const TimedPose *raw_after = nullptr,
                    const TimedPose *correction = nullptr) {
    if (write_csv_ && csv_.is_open()) {
      csv_ << formatStamp(stamp_ns)
           << ",nan,nan,nan,nan,nan,nan,nan,0," << reason << ','
           << (raw_before ? formatStamp(raw_before->stamp_ns) : "nan") << ','
           << (raw_after ? formatStamp(raw_after->stamp_ns) : "nan") << ','
           << (correction ? formatStamp(correction->stamp_ns) : "nan") << '\n';
    }
    ++rows_total_;
    ++rows_invalid_;
    if (std::string(reason) == "GLOBAL_ALIGNMENT_NOT_READY")
      ++alignment_not_ready_;
    else if (std::string(reason) == "RAW_INTERPOLATION_GAP")
      ++raw_gap_invalid_;
    else if (std::string(reason) == "MISSING_MAP_TO_ODOM")
      ++missing_map_to_odom_;
  }

  void writeValid(std::int64_t stamp_ns, const TimedPose &map_camera,
                  const TimedPose &raw_before, const TimedPose &raw_after,
                  const TimedPose &correction) {
    const Eigen::Vector3d neu = enuToNeu(map_camera.position);
    const Eigen::Quaterniond quaternion = map_camera.orientation.normalized();
    if (write_csv_ && csv_.is_open()) {
      csv_ << formatStamp(stamp_ns) << ',' << std::fixed
           << std::setprecision(9) << neu.x() << ',' << neu.y() << ','
           << neu.z() << ',' << quaternion.x() << ',' << quaternion.y()
           << ',' << quaternion.z() << ',' << quaternion.w()
           << ",1,OK," << formatStamp(raw_before.stamp_ns) << ','
           << formatStamp(raw_after.stamp_ns) << ','
           << formatStamp(correction.stamp_ns) << '\n';
    }
    if (write_txt_ && txt_.is_open()) {
      txt_ << formatStamp(stamp_ns) << ' ' << std::fixed
           << std::setprecision(9) << neu.x() << ' ' << neu.y() << ' '
           << neu.z() << ' ' << quaternion.x() << ' ' << quaternion.y()
           << ' ' << quaternion.z() << ' ' << quaternion.w() << '\n';
    }
    ++rows_total_;
    ++rows_valid_;
    if (first_valid_stamp_ns_ < 0) first_valid_stamp_ns_ = stamp_ns;
  }

  void printSummary() const {
    double output_rate_hz = 0.0;
    if (rows_total_ > 1 && last_camera_stamp_ns_ > first_camera_stamp_ns_) {
      output_rate_hz = (rows_total_ - 1) * kNanosecondsPerSecond /
                       static_cast<double>(last_camera_stamp_ns_ -
                                           first_camera_stamp_ns_);
    }
    std::ostringstream summary;
    summary << "[CameraAlignedNEU] Summary\n"
            << "camera_received_for_export = " << camera_received_ << "\n"
            << "rows_total                 = " << rows_total_ << "\n"
            << "rows_valid                 = " << rows_valid_ << "\n"
            << "rows_invalid               = " << rows_invalid_ << "\n"
            << "alignment_not_ready        = " << alignment_not_ready_ << "\n"
            << "raw_gap_invalid            = " << raw_gap_invalid_ << "\n"
            << "missing_map_to_odom        = " << missing_map_to_odom_ << "\n"
            << "mean_output_rate_hz        = " << output_rate_hz << "\n"
            << "first_camera_stamp         = "
            << (first_camera_stamp_ns_ >= 0
                    ? formatStamp(first_camera_stamp_ns_)
                    : "nan") << "\n"
            << "last_camera_stamp          = "
            << (last_camera_stamp_ns_ >= 0
                    ? formatStamp(last_camera_stamp_ns_)
                    : "nan") << "\n"
            << "first_valid_stamp          = "
            << (first_valid_stamp_ns_ >= 0
                    ? formatStamp(first_valid_stamp_ns_)
                    : "nan") << "\n"
            << "max_raw_bracket_span_s     = " << max_raw_bracket_span_s_;
    // ROS logging may already be shutting down; stdout keeps the exit summary
    // visible in the roslaunch log without changing any mapping callback.
    std::cout << summary.str() << std::endl;
    ROS_INFO_STREAM(summary.str());
  }

  ros::NodeHandle node_;
  ros::Subscriber image_subscriber_;
  ros::Subscriber raw_odom_subscriber_;
  ros::Subscriber map_to_odom_subscriber_;
  ros::Subscriber status_subscriber_;
  ros::Timer flush_timer_;
  std::mutex mutex_;

  bool enabled_ = false;
  bool write_csv_ = true;
  bool write_txt_ = true;
  bool backend_status_seen_ = false;
  bool backend_alignment_ready_ = false;
  double max_raw_gap_s_ = 0.25;
  double buffer_seconds_ = 5.0;
  std::string image_topic_;
  std::string raw_odom_topic_;
  std::string map_to_odom_topic_;
  std::string status_topic_;
  std::string csv_path_;
  std::string txt_path_;
  std::ofstream csv_;
  std::ofstream txt_;

  std::deque<std::int64_t> pending_camera_stamps_;
  std::deque<TimedPose> raw_poses_;
  std::deque<TimedPose> map_to_odom_poses_;
  TimedPose body_to_camera_;

  std::uint64_t camera_received_ = 0;
  std::uint64_t rows_total_ = 0;
  std::uint64_t rows_valid_ = 0;
  std::uint64_t rows_invalid_ = 0;
  std::uint64_t alignment_not_ready_ = 0;
  std::uint64_t raw_gap_invalid_ = 0;
  std::uint64_t missing_map_to_odom_ = 0;
  std::int64_t first_camera_stamp_ns_ = -1;
  std::int64_t last_camera_stamp_ns_ = -1;
  std::int64_t first_valid_stamp_ns_ = -1;
  std::int64_t first_alignment_status_stamp_ns_ = -1;
  double max_raw_bracket_span_s_ = 0.0;
};

}  // namespace

bool interpolatePose(const TimedPose &before, const TimedPose &after,
                     std::int64_t query_stamp_ns, double max_gap_s,
                     TimedPose &interpolated, double &bracket_span_s) {
  bracket_span_s = static_cast<double>(after.stamp_ns - before.stamp_ns) /
                   kNanosecondsPerSecond;
  if (!finitePose(before) || !finitePose(after) ||
      query_stamp_ns < before.stamp_ns || query_stamp_ns > after.stamp_ns ||
      bracket_span_s < 0.0 || bracket_span_s > max_gap_s) {
    return false;
  }
  const double alpha = after.stamp_ns == before.stamp_ns
                           ? 0.0
                           : static_cast<double>(query_stamp_ns -
                                                 before.stamp_ns) /
                                 static_cast<double>(after.stamp_ns -
                                                     before.stamp_ns);
  interpolated.stamp_ns = query_stamp_ns;
  interpolated.position =
      before.position + alpha * (after.position - before.position);
  interpolated.orientation = before.orientation.slerp(alpha, after.orientation);
  interpolated.orientation.normalize();
  return finitePose(interpolated);
}

const TimedPose *latestCausalPose(const std::deque<TimedPose> &poses,
                                  std::int64_t query_stamp_ns) {
  const auto upper = std::upper_bound(
      poses.begin(), poses.end(), query_stamp_ns,
      [](std::int64_t query, const TimedPose &pose) {
        return query < pose.stamp_ns;
      });
  return upper == poses.begin() ? nullptr : &*std::prev(upper);
}

TimedPose composePoses(const TimedPose &parent_to_child,
                       const TimedPose &child_to_body,
                       std::int64_t stamp_ns) {
  TimedPose result;
  result.stamp_ns = stamp_ns;
  result.orientation =
      (parent_to_child.orientation * child_to_body.orientation).normalized();
  result.position = parent_to_child.position +
                    parent_to_child.orientation * child_to_body.position;
  return result;
}

Eigen::Vector3d enuToNeu(const Eigen::Vector3d &enu) {
  return Eigen::Vector3d(enu.y(), enu.x(), enu.z());
}

std::string formatStamp(std::int64_t stamp_ns) {
  const bool negative = stamp_ns < 0;
  const std::uint64_t magnitude = negative
      ? static_cast<std::uint64_t>(-(stamp_ns + 1)) + 1
      : static_cast<std::uint64_t>(stamp_ns);
  std::ostringstream stream;
  if (negative) stream << '-';
  stream << magnitude / 1000000000ULL << '.' << std::setw(9)
         << std::setfill('0') << magnitude % 1000000000ULL;
  return stream.str();
}

void runCameraAlignedNeuLogger(ros::NodeHandle &node) {
  CameraAlignedNeuLogger logger(node);
  ros::spin();
}

}  // namespace fast_livo_camera_neu
