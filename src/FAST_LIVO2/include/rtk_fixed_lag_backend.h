#pragma once

#include <fast_livo/GnssStatus.h>
#include <fast_livo/RtkBackendStatus.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <uwb_serial_driver/UwbRangeArray.h>

#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace fast_livo_backend {

struct RtkFixedLagBackendSelfTestAccess;

class GnssPositionArmFactor final
    : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  GnssPositionArmFactor(gtsam::Key key, const gtsam::Point3 &measurement,
                        const gtsam::Point3 &lever_arm,
                        const gtsam::SharedNoiseModel &noise_model);

  gtsam::NonlinearFactor::shared_ptr clone() const override;
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> jacobian = boost::none) const override;

 private:
  gtsam::Point3 measurement_;
  gtsam::Point3 lever_arm_;
};

class UwbRangeFactor final : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  UwbRangeFactor(gtsam::Key key, const gtsam::Point3 &anchor_position,
                 double corrected_range_m, const gtsam::Point3 &tag_lever_arm,
                 const gtsam::SharedNoiseModel &noise_model);

  gtsam::NonlinearFactor::shared_ptr clone() const override;
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> jacobian = boost::none) const override;

  const gtsam::Point3 &anchorPosition() const { return anchor_position_; }
  const gtsam::Point3 &tagLeverArm() const { return tag_lever_arm_; }
  double correctedRange() const { return corrected_range_m_; }

 private:
  gtsam::Point3 anchor_position_;
  double corrected_range_m_ = 0.0;
  gtsam::Point3 tag_lever_arm_;
};

struct AlignmentPair {
  gtsam::Point3 odom_position;
  gtsam::Point3 enu_position;
  std::int64_t gnss_stamp_ns = -1;
};

struct AlignmentResult {
  bool valid = false;
  double yaw_rad = 0.0;
  gtsam::Point3 translation{0.0, 0.0, 0.0};
  double rmse_m = 0.0;
  double baseline_m = 0.0;
  std::size_t pair_count = 0;
};

struct UwbAnchorConfig {
  int id = -1;
  bool enabled = false;
  gtsam::Point3 position{0.0, 0.0, 0.0};
  double range_bias_m = 0.0;
};

struct BackendConfig {
  bool enable = true;
  bool accept_rtk_fixed = true;
  bool accept_rtk_float = false;
  bool accept_differential = false;
  bool accept_single = false;
  std::string raw_odom_topic = "/backend/livo_odom_raw";
  std::string gnss_odom_topic = "/gnss/enu_odom";
  std::string gnss_status_topic = "/gnss/status";
  std::string optimized_odom_topic = "/rtk_backend/optimized_odom";
  std::string optimized_path_topic = "/rtk_backend/optimized_path";
  std::string map_to_odom_topic = "/rtk_backend/map_to_odom";
  std::string status_topic = "/rtk_backend/status";
  double lag_seconds = 20.0;
  double keyframe_translation_m = 0.8;
  double keyframe_rotation_deg = 8.0;
  double keyframe_max_interval_s = 1.0;
  double raw_odom_buffer_seconds = 5.0;
  double max_raw_odom_interpolation_gap_s = 0.15;
  double reuse_existing_node_time_diff_s = 0.02;
  double gnss_node_min_interval_s = 0.20;
  int max_active_states = 300;
  int alignment_min_pairs = 20;
  double alignment_min_baseline_m = 8.0;
  double alignment_max_pair_time_diff_s = 0.05;
  double alignment_max_rmse_m = 0.5;
  gtsam::Point3 antenna_lever_arm_body_m{0.0, 0.0, 0.0};
  gtsam::Point3 result_pose_lever_arm_body_m{0.0, 0.0, 0.0};
  double min_gnss_sigma_xy_m = 0.03;
  double min_gnss_sigma_z_m = 0.05;
  double max_gnss_sigma_xy_m = 2.0;
  double max_gnss_sigma_z_m = 3.0;
  double rtk_float_sigma_scale = 1.0;
  int rtk_float_reference_satellites = 0;
  double rtk_float_satellite_sigma_scale_max = 1.0;
  double gnss_recovery_gap_threshold_s = 0.0;
  double gnss_recovery_ramp_duration_s = 0.0;
  double gnss_recovery_initial_sigma_scale = 1.0;
  int gnss_recovery_fixed_confirm_factors = 1;
  double gnss_recovery_fixed_max_gap_s = 0.5;
  double gnss_recovery_float_factor_rate_hz = 0.0;
  double max_gnss_residual_m = 3.0;
  double max_gnss_nis = 11.34;
  std::string robust_kernel = "huber";
  double huber_delta = 2.5;
  double livo_translation_sigma_m = 0.05;
  double livo_lateral_vertical_sigma_m = 0.001;
  double livo_rotation_sigma_rad = 0.01;
  double livo_uwb_rotation_sigma_rad = 0.0003;
  double prior_translation_sigma_m = 0.10;
  double prior_roll_pitch_sigma_rad = 0.05;
  double prior_yaw_sigma_rad = 0.20;
  double uwb_initial_prior_translation_sigma_m = 0.001;
  double uwb_initial_prior_rotation_sigma_rad = 0.001;
  bool uwb_factor_backend_en = false;
  std::string uwb_range_topic = "/uwb/backend_ranges";
  double uwb_time_offset_s = 0.0;
  double uwb_range_sigma_m = 0.10;
  double uwb_min_range_m = 0.05;
  double uwb_max_range_m = 250.0;
  double uwb_association_max_dt_s = 0.05;
  double uwb_max_prefit_residual_m = 6.0;
  double uwb_max_nis = 9.0;
  double uwb_huber_delta = 2.5;
  gtsam::Point3 uwb_tag_lever_arm_body_m{0.0, 0.0, 0.0};
  std::map<int, UwbAnchorConfig> uwb_anchors;
  std::set<int> uwb_anchor_allowlist;
  std::string frame_id = "map";
  std::string odom_frame_id = "odom";
  std::string body_frame_id = "body";
  double log_interval_s = 1.0;
  bool save_results = true;
  std::string output_directory;
  std::string raw_online_file = "livo_raw_online.tum";
  std::string optimized_online_file = "rtk_optimized_online.tum";
  std::string optimized_final_file = "rtk_optimized_final.tum";
  std::string gnss_file = "gnss_enu.tum";
  std::string uwb_status_csv_file = "uwb_backend_measurements.csv";
  std::string status_csv_file = "rtk_backend_status.csv";
  double flush_interval_s = 1.0;
  bool save_text_log = true;
  std::string text_log_file = "rtk_backend.log";
};

class RtkFixedLagBackend {
 public:
  explicit RtkFixedLagBackend(ros::NodeHandle &nh);
  ~RtkFixedLagBackend();

  bool enabled() const { return config_.enable; }

  static AlignmentResult estimateSe2Alignment(
      const std::vector<AlignmentPair> &pairs);
  static bool shouldCreateKeyframe(const gtsam::Pose3 &previous,
                                   const gtsam::Pose3 &current,
                                   double interval_s,
                                   const BackendConfig &config);
  static bool interpolatePose(const ros::Time &stamp0,
                              const gtsam::Pose3 &pose0,
                              const ros::Time &stamp1,
                              const gtsam::Pose3 &pose1,
                              const ros::Time &target,
                              double max_gap_s,
                              gtsam::Pose3 *interpolated,
                              double *interval_s,
                              std::string *reject_reason);

 private:
  friend struct RtkFixedLagBackendSelfTestAccess;
  RtkFixedLagBackend() = default;

  struct RawOdomSample {
    ros::Time stamp;
    gtsam::Pose3 pose;
  };

  struct GnssMeasurement {
    ros::Time stamp;
    gtsam::Point3 position;
    gtsam::Vector3 sigmas;
    gtsam::Vector3 reported_sigmas = gtsam::Vector3::Zero();
    std::uint8_t raw_quality = fast_livo::GnssStatus::INVALID;
    std::uint8_t filtered_quality = fast_livo::GnssStatus::INVALID;
    std::uint8_t num_sv = 0;
    double quality_sigma_scale = 1.0;
    double satellite_sigma_scale = 1.0;
    double recovery_sigma_scale = 1.0;
    bool covariance_authoritative = false;
  };

  struct UwbMeasurement {
    ros::Time raw_stamp;
    ros::Time aligned_stamp;
    int anchor_id = -1;
    double raw_range_m = 0.0;
    double range_bias_m = 0.0;
    double corrected_range_m = 0.0;
    std::uint64_t measurement_uid = 0;
    std::string source_format;
  };

  struct Keyframe {
    std::uint64_t id = 0;
    gtsam::Key key = 0;
    ros::Time stamp;
    gtsam::Pose3 raw_pose;
    gtsam::Pose3 optimized_pose;
    bool measurement_triggered = false;
  };

  struct ArchiveRecord {
    gtsam::Key key = 0;
    ros::Time stamp;
    gtsam::Pose3 pose;
  };

  struct FileBatch {
    std::vector<std::string> raw_lines;
    std::vector<std::string> optimized_online_lines;
    std::vector<std::string> optimized_final_lines;
    std::vector<std::string> gnss_lines;
    std::vector<std::string> uwb_lines;
    std::vector<std::string> status_lines;
    std::vector<std::string> text_lines;
  };

  void loadParameters(ros::NodeHandle &nh);
  void validateParameters() const;
  void setupRos(ros::NodeHandle &nh);
  void initializeResultFiles();
  void closeResultFiles();
  void rawOdomCallback(const nav_msgs::OdometryConstPtr &message);
  void gnssOdomCallback(const nav_msgs::OdometryConstPtr &message);
  void gnssStatusCallback(const fast_livo::GnssStatusConstPtr &message);
  bool gnssQualityAccepted(std::uint8_t quality) const;
  double gnssQualitySigmaScale(const fast_livo::GnssStatus &status,
                               double *satellite_scale) const;
  gtsam::Vector3 clampGnssSigmas(const gtsam::Vector3 &sigmas,
                                 double sigma_scale) const;
  gtsam::Vector3 gnssBaseSigmas(const gtsam::Vector3 &reported_sigmas,
                                const fast_livo::GnssStatus &status,
                                double *quality_scale,
                                double *satellite_scale) const;
  double updateGnssRecoverySigmaScale(const ros::Time &stamp);
  bool gnssRecoveryFloatFactorRateLimited(
      const GnssMeasurement &measurement) const;
  void commitGnssRecoveryAcceptedFactor(const GnssMeasurement &measurement);
  void uwbRangeCallback(
      const uwb_serial_driver::UwbRangeArrayConstPtr &message);
  void statusTimerCallback(const ros::TimerEvent &);
  void flushTimerCallback(const ros::TimerEvent &);

  void tryPairGnssMessages(std::uint64_t stamp_ns);
  void processAcceptedGnss(const nav_msgs::Odometry &odometry,
                           const fast_livo::GnssStatus &status);
  void insertPendingGnss(const GnssMeasurement &measurement);
  void tryCollectAlignmentPairs();
  void transitionPendingGnssAfterAlignment(
      std::int64_t alignment_cutoff_stamp_ns);
  void resetAlignmentCollection(const std::string &reason);
  bool tryFinishAlignment();
  bool initializeGraph(const RawOdomSample &sample);
  bool createGraphNode(const RawOdomSample &sample,
                       const std::string &trigger,
                       gtsam::Key *created_key);
  void maybeAddKeyframe(const RawOdomSample &sample);
  void processPendingGnss();
  bool interpolateRawPose(const ros::Time &stamp, gtsam::Pose3 *pose,
                          double *interval_s, std::string *reason) const;
  Keyframe *findReusableKeyframe(const ros::Time &stamp,
                                 double *time_difference_s);
  Keyframe *findNearestKeyframe(const ros::Time &stamp,
                                double maximum_time_difference_s,
                                double *time_difference_s);
  Keyframe *findKeyframe(gtsam::Key key);
  bool addGnssFactor(const GnssMeasurement &measurement,
                     const Keyframe &keyframe);
  void insertPendingUwb(const UwbMeasurement &measurement);
  void processPendingUwb();
  bool addUwbFactor(const UwbMeasurement &measurement,
                    const Keyframe &keyframe);
  bool updateSmoother(const gtsam::NonlinearFactorGraph &factors,
                      const gtsam::Values &values,
                      const gtsam::FixedLagSmoother::KeyTimestampMap &timestamps);
  std::vector<ArchiveRecord> collectMarginalizationCandidates(
      const gtsam::FixedLagSmoother::KeyTimestampMap &timestamps) const;
  void commitMarginalizedArchives(
      const std::vector<ArchiveRecord> &candidates);
  void archiveActiveStates();
  void refreshEstimateAndPublish(bool publish_current = true);
  void updateUwbGeometryDiagnostics(const gtsam::Values &estimate);
  void pruneRawOdomBuffer();
  void rejectGnss(const std::string &reason, double residual_m = 0.0,
                  double nis = 0.0,
                  const ros::Time *measurement_stamp = nullptr);
  void rejectUwb(const std::string &reason,
                 const UwbMeasurement *measurement = nullptr,
                 double predicted_range_m = 0.0,
                 double residual_m = 0.0, double nis = 0.0,
                 double association_dt_s = 0.0,
                 std::int64_t associated_key = -1);
  std::int64_t gnssConservationDelta() const;
  std::uint64_t gnssSilentDropCount() const;
  void publishStatus();

  void queueRawPose(const RawOdomSample &sample);
  void queueOptimizedOnlinePose(const Keyframe &keyframe);
  void queueFinalPose(const ArchiveRecord &record);
  void queueGnssPosition(const GnssMeasurement &measurement);
  void queueUwbMeasurement(const UwbMeasurement &measurement,
                           const std::string &decision,
                           const std::string &reason,
                           double predicted_range_m, double residual_m,
                           double nis, double robust_weight,
                           double association_dt_s,
                           std::int64_t associated_key);
  void queueStatusCsv();
  void queueTextEvent(const std::string &event,
                      const std::string &detail = std::string());
  FileBatch takePendingFileBatch();
  void writeFileBatch(const FileBatch &batch, bool flush);

  static gtsam::Pose3 poseFromMessage(const geometry_msgs::Pose &pose);
  static geometry_msgs::Pose poseToMessage(const gtsam::Pose3 &pose);
  static std::string tumLine(const ros::Time &stamp,
                             const gtsam::Pose3 &pose);
  static std::string gnssTumLine(const GnssMeasurement &measurement);
  static gtsam::Pose3 resultReferencePose(
      const gtsam::Pose3 &body_pose,
      const gtsam::Point3 &lever_arm_body_m);
  static bool poseIsFinite(const gtsam::Pose3 &pose);

  BackendConfig config_;
  ros::Subscriber raw_odom_subscriber_;
  ros::Subscriber gnss_odom_subscriber_;
  ros::Subscriber gnss_status_subscriber_;
  ros::Subscriber uwb_range_subscriber_;
  ros::Publisher optimized_odom_publisher_;
  ros::Publisher optimized_path_publisher_;
  ros::Publisher map_to_odom_publisher_;
  ros::Publisher status_publisher_;
  ros::Timer status_timer_;
  ros::Timer flush_timer_;
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_;

  std::map<std::uint64_t, fast_livo::GnssStatus> pending_status_;
  std::map<std::uint64_t, nav_msgs::Odometry> pending_gnss_odom_;
  std::deque<RawOdomSample> raw_odom_buffer_;
  std::deque<GnssMeasurement> pending_alignment_gnss_;
  std::deque<GnssMeasurement> pending_factor_gnss_;
  std::deque<UwbMeasurement> pending_uwb_;
  std::vector<AlignmentPair> alignment_pairs_;
  std::deque<Keyframe> keyframes_;

  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  gtsam::Pose3 initial_map_to_odom_;
  AlignmentResult alignment_;
  std::set<gtsam::Key> finalized_keys_;
  std::uint64_t next_keyframe_id_ = 0;
  std::uint64_t total_nodes_created_ = 0;
  std::uint64_t marginalized_nodes_ = 0;
  std::size_t active_factors_ = 0;
  std::size_t active_livo_factors_ = 0;
  std::size_t active_gnss_factors_ = 0;
  std::size_t active_uwb_factors_ = 0;
  std::size_t active_uwb_anchor_count_ = 0;
  std::size_t max_active_states_observed_ = 0;

  std::uint64_t raw_odom_received_ = 0;
  std::uint64_t raw_odom_published_ = 0;
  std::uint64_t raw_odom_duplicate_ = 0;
  std::uint64_t raw_odom_non_monotonic_ = 0;
  std::int64_t last_raw_odom_stamp_ns_ = -1;
  std::uint64_t tf_published_ = 0;
  std::uint64_t tf_duplicate_skipped_ = 0;
  std::int64_t last_tf_stamp_ns_ = -1;

  std::uint64_t gnss_received_ = 0;
  std::uint64_t gnss_accepted_ = 0;
  std::uint64_t gnss_rejected_ = 0;
  std::uint64_t gnss_factor_count_ = 0;
  std::uint64_t livo_factor_count_ = 0;
  std::uint64_t gnss_quality_rejected_ = 0;
  std::uint64_t gnss_time_rejected_ = 0;
  std::uint64_t gnss_too_old_ = 0;
  std::uint64_t gnss_interpolation_gap_ = 0;
  std::uint64_t gnss_interpolation_invalid_ = 0;
  std::uint64_t gnss_late_out_of_order_ = 0;
  std::uint64_t gnss_rate_limited_ = 0;
  std::uint64_t gnss_duplicate_timestamp_ = 0;
  std::uint64_t gnss_no_active_state_ = 0;
  std::uint64_t alignment_gnss_used_ = 0;
  std::uint64_t alignment_transition_to_graph_pending_ = 0;
  std::uint64_t alignment_transition_rejected_ = 0;
  std::uint64_t alignment_transition_waiting_ = 0;
  std::uint64_t gnss_duplicate_factor_count_ = 0;
  std::uint64_t gnss_odom_only_rejected_ = 0;
  std::int64_t last_enqueued_gnss_stamp_ns_ = -1;
  std::int64_t gnss_recovery_start_stamp_ns_ = -1;
  std::int64_t gnss_recovery_fade_start_stamp_ns_ = -1;
  std::int64_t gnss_recovery_last_fixed_stamp_ns_ = -1;
  std::int64_t last_added_recovery_float_factor_stamp_ns_ = -1;
  std::uint32_t gnss_recovery_consecutive_fixed_factors_ = 0;
  std::int64_t last_processed_gnss_stamp_ns_ = -1;
  std::int64_t alignment_last_used_gnss_stamp_ns_ = -1;
  std::int64_t last_gnss_triggered_node_stamp_ns_ = -1;
  std::int64_t last_added_gnss_factor_stamp_ns_ = -1;
  double last_gnss_dt_s_ = 0.0;
  double last_gnss_residual_m_ = 0.0;
  double last_gnss_nis_ = 0.0;

  struct UwbAnchorStatistics {
    std::uint64_t received = 0;
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::deque<double> accepted_abs_residuals;
  };
  bool gnss_factor_enabled_ = true;
  std::uint64_t uwb_received_ = 0;
  std::uint64_t uwb_parsed_ = 0;
  std::uint64_t uwb_accepted_ = 0;
  std::uint64_t uwb_rejected_ = 0;
  std::uint64_t uwb_factor_count_ = 0;
  std::uint64_t uwb_invalid_rejected_ = 0;
  std::uint64_t uwb_stale_rejected_ = 0;
  std::uint64_t uwb_duplicate_rejected_ = 0;
  std::uint64_t uwb_association_rejected_ = 0;
  std::uint64_t uwb_range_rejected_ = 0;
  std::uint64_t uwb_residual_rejected_ = 0;
  std::map<int, std::int64_t> last_enqueued_uwb_stamp_ns_;
  std::map<int, std::int64_t> last_added_uwb_stamp_ns_;
  std::map<int, UwbAnchorStatistics> uwb_anchor_statistics_;
  double last_uwb_association_dt_s_ = 0.0;
  double last_uwb_residual_m_ = 0.0;
  double last_uwb_nis_ = 0.0;
  gtsam::Vector3 uwb_geometry_eigenvalues_ = gtsam::Vector3::Zero();
  double uwb_geometry_condition_ = 0.0;
  int uwb_geometry_rank_ = 0;

  std::uint64_t interpolation_count_ = 0;
  double interpolation_gap_sum_s_ = 0.0;
  double interpolation_gap_max_s_ = 0.0;
  std::uint64_t optimization_count_ = 0;
  double optimization_time_ms_ = 0.0;
  double optimization_time_sum_ms_ = 0.0;
  double optimization_time_max_ms_ = 0.0;

  std::string last_reject_reason_;
  std::string backend_error_;
  ros::Time newest_sensor_stamp_;
  bool initialized_ = false;
  bool backend_halted_ = false;

  mutable std::mutex state_mutex_;
  std::mutex file_mutex_;
  bool result_files_ready_ = false;
  std::ofstream raw_online_stream_;
  std::ofstream optimized_online_stream_;
  std::ofstream optimized_final_stream_;
  std::ofstream gnss_stream_;
  std::ofstream uwb_stream_;
  std::ofstream status_csv_stream_;
  std::ofstream text_log_stream_;
  FileBatch pending_file_batch_;
};

}  // namespace fast_livo_backend
