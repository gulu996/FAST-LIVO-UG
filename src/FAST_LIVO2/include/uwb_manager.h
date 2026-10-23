/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.
*/

#ifndef UWB_MANAGER_H
#define UWB_MANAGER_H

#include "common_lib.h"
#include <uwb_serial_driver/UwbRangeArray.h>

#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

enum class UwbRejectReason { INVALID_CORRECTED_RANGE, INVALID_RANGE_STATUS, INVALID_RAW_RANGE, NONE, RANGE_LIMIT };

struct UwbRangeMeasurement
{
  int anchor_id = -1;
  double raw_range_m = 0.0;
  double range_bias_m = 0.0;
  // ponytail: range_m remains the canonical downstream value to keep every existing consumer on one path.
  double range_m = 0.0;
  bool range_valid = false;
  UwbRejectReason range_reject_reason = UwbRejectReason::INVALID_RANGE_STATUS;
  int64_t diag = -1;
  uint64_t measurement_uid = 0;
  size_t source_line = 0;
  std::string source_format = "unknown";
  double stamp = 0.0;
  double time_diff_s = 0.0;
  std::string raw_line;
};

bool correctUwbRangeValue(double, double, double &, UwbRejectReason &);
bool isUwbReplayCrossFormatDuplicate(const UwbRangeMeasurement &, const UwbRangeMeasurement &, double);

struct UwbAnchor
{
  int id = -1;
  bool enabled = false;
  bool estimated = false;
  std::string role;
  double range_bias_m = 0.0;
  V3D position_w = V3D::Zero();
};

// Transport only: no access to frontend state, covariance or map.
class UwbManager
{
public:
  UwbManager();
  ~UwbManager();
  bool initialize(ros::NodeHandle &, const std::string &);
  void shutdown();
  bool isRunning() const { return running_.load(); }
  bool updateEnabled() const { return en_; }
  const std::string &inputMode() const { return input_mode_; }
  void setInputMode(const std::string &mode) { input_mode_ = mode; }
  void ingestExternalRanges(const uwb_serial_driver::UwbRangeArray &);
  size_t pendingMeasurementCount() const {
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    return measurement_queue_.size();
  }
  void forwardMeasurementsAt(double current_lidar_stamp, double lidar_start_stamp);
private:
  bool loadParameters(ros::NodeHandle &nh);
  bool openSerial();
  bool configureSerial();
  void closeSerial();
  void readLoop();
  void externalRangeCallback(const uwb_serial_driver::UwbRangeArrayConstPtr &message);
  bool loadReplayFile();
  std::vector<UwbRangeMeasurement> takeReplayMeasurements(double current_lidar_stamp, double lidar_start_stamp);
  void publishBackendMeasurements(
      const std::vector<UwbRangeMeasurement> &measurements,
      double lidar_start_stamp);
  void handleLine(const std::string &line, double stamp);
  std::vector<UwbRangeMeasurement> parseLine(const std::string &line, double stamp) const;
  std::vector<UwbRangeMeasurement> takeRecentMeasurements(double now);
  void logRawLine(double stamp, const std::string &line, const std::vector<UwbRangeMeasurement> &measurements);
  void logEvent(double stamp, const std::string &level, const std::string &message);
  void logEventThrottled(double stamp, const std::string &key, double period_s,
                         const std::string &level, const std::string &message);
  void emitUwbLine(double, const std::string &, const std::string &, bool, bool);
  bool correctUwbRange(int, double, double &, double &, UwbRejectReason &) const;
  bool en_ = false;
  std::string input_mode_ = "internal";
  std::string input_source_ = "serial";
  std::string external_topic_ = "/uwb/ranges";
  bool factor_backend_output_en_ = false;
  std::string factor_backend_topic_ = "/uwb/backend_ranges";
  std::string serial_port_ = "/dev/ttyUSB0";
  int baudrate_ = 115200;
  bool dtr_high_ = true;
  bool rts_high_ = false;
  std::string parser_mode_ = "uwb";
  std::string log_filename_ = "uwb_ranges.txt";
  std::string update_log_filename_ = "uwb_updates.txt";
  int log_flush_stride_ = 1;
  bool debug_log_en_ = true;
  bool debug_to_console_ = false;
  bool debug_to_file_ = true;
  std::string replay_file_;
  double replay_start_offset_s_ = 0.0;
  double replay_match_threshold_s_ = 0.05;
  double range_scale_ = 1.0;
  double min_range_m_ = 0.05;
  double max_range_m_ = 250.0;
  double max_age_s_ = 0.5;
  int max_queue_size_ = 512;
  int serial_fd_ = -1;
  bool ownership_conflict_ = false;
  std::atomic<bool> running_{false};
  std::thread read_thread_;
  ros::Subscriber external_subscriber_;
  ros::Publisher factor_backend_publisher_;
  uint64_t factor_backend_sequence_ = 0;
  mutable std::mutex measurement_mutex_;
  std::deque<UwbRangeMeasurement> measurement_queue_;
  std::mutex log_mutex_;
  std::ofstream raw_log_file_;
  std::ofstream update_log_file_;
  int raw_log_pending_lines_ = 0;
  std::map<std::string, double> event_log_last_stamp_;
  std::map<int, UwbAnchor> anchors_;
  std::map<int, UwbAnchor> configured_anchors_;
  std::vector<int> anchor_order_;
  std::vector<UwbRangeMeasurement> replay_measurements_;
  size_t replay_index_ = 0;
  bool replay_started_ = false;
  double replay_last_slam_relative_time_ = -1.0;
  double replay_file_start_stamp_ = 0.0;
  bool replay_file_start_stamp_ready_ = false;
  uint64_t replay_consumed_measurement_count_ = 0;
  uint64_t replay_stale_measurement_count_ = 0;
};
using UwbManagerPtr = std::shared_ptr<UwbManager>;
#endif
