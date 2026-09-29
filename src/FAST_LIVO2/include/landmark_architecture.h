#ifndef LANDMARK_ARCHITECTURE_H_
#define LANDMARK_ARCHITECTURE_H_

#include "landmark_frontend.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace landmark
{

enum class FusionMode
{
  ObserveOnly,
  LegacyEsikf,
  GlobalBackend
};

const char *fusionModeName(FusionMode mode);
bool parseFusionMode(const std::string &name, FusionMode *mode);

enum class PersistentLandmarkStatus
{
  Uninitialized,
  Candidate,
  Initialized,
  Active,
  Inactive,
  Rejected
};

struct PersistentLandmark
{
  int landmark_id = -1;
  Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
  bool has_initial_guess = false;
  Eigen::Isometry3d optimized_estimate = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  double first_seen_timestamp = std::numeric_limits<double>::quiet_NaN();
  double last_seen_timestamp = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t observation_count = 0;
  PersistentLandmarkStatus status = PersistentLandmarkStatus::Uninitialized;
  std::uint64_t backend_key = std::numeric_limits<std::uint64_t>::max();
  bool has_optimized_estimate = false;
  std::vector<std::uint64_t> associated_observation_ids;
  std::uint64_t episode_id = 0;
  double episode_first_timestamp = std::numeric_limits<double>::quiet_NaN();
  double episode_last_timestamp = std::numeric_limits<double>::quiet_NaN();
};

struct SparseKeyPose
{
  std::uint64_t keypose_id = 0;
  double timestamp = std::numeric_limits<double>::quiet_NaN();
  Eigen::Isometry3d initial_global_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d local_pose_reference = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> pose_covariance =
      Eigen::Matrix<double, 6, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  std::vector<std::uint64_t> source_observation_ids;
  std::vector<int> associated_landmark_ids;
  std::uint64_t backend_key = std::numeric_limits<std::uint64_t>::max();
  std::string trigger_reason;
  bool raw_lio_weak_geometry = false;
};

struct SparseKeyPosePolicy
{
  // PLACEHOLDER values: calibrate against the future sparse global backend.
  double translation_threshold_m = 1.0;
  double rotation_threshold_deg = 10.0;
  double maximum_interval_s = 2.0;
  double coalesce_time_s = 0.10;
  double coalesce_translation_m = 0.05;
  double coalesce_rotation_deg = 1.0;
  double episode_gap_s = 2.0;
};

bool shouldCreateSparseKeyPose(const Eigen::Isometry3d &previous_local_pose,
                               double previous_timestamp,
                               const Eigen::Isometry3d &candidate_local_pose,
                               double candidate_timestamp,
                               bool first_landmark_observation,
                               bool landmark_reobservation,
                               const SparseKeyPosePolicy &policy);

struct GlobalLandmarkInput
{
  std::uint64_t observation_id = 0;
  LandmarkObservation observation;
  Eigen::Isometry3d local_pose_reference = Eigen::Isometry3d::Identity();
  double local_pose_timestamp = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix<double, 6, 6> local_pose_covariance =
      Eigen::Matrix<double, 6, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  std::string local_frame_id = "odom";
  std::string camera_extrinsic_id;
  // T_body_camera maps camera coordinates into the raw LIVO IMU/body frame.
  Eigen::Isometry3d T_body_camera = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d landmark_initial_guess = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> landmark_initial_covariance =
      Eigen::Matrix<double, 6, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  bool raw_lio_weak_geometry = false;
};

enum class MotionCovarianceSource
{
  Exact,
  Approximate,
  Conservative,
  Unavailable
};

// Engineering drift envelope, NOT a statistical upper bound or calibrated noise.
// All sigmas are in the right-local tangent of T_ij: [rotation rad, translation m].
struct ConservativeMotionUncertaintyConfig
{
  double rotation_floor_rad = 0.0872664626; // 5 deg; NOT_CALIBRATED
  double rotation_per_second_rad = 0.0174532925; // 1 deg/s
  double rotation_per_meter_rad = 0.0087266463; // 0.5 deg/m
  double rotation_ceiling_rad = 3.1415926536;
  double translation_floor_m = 0.10; // NOT_CALIBRATED
  double translation_per_second_m = 0.05;
  double translation_per_meter_m = 0.03;
  double translation_per_radian_m = 0.10;
  double translation_ceiling_m = 100.0;
  double weak_geometry_multiplier = 2.0;
};

bool validConservativeMotionConfig(const ConservativeMotionUncertaintyConfig &config);
bool conservativeMotionCovariance(const Eigen::Isometry3d &relative_pose,
                                  double duration_s, bool weak_geometry,
                                  const ConservativeMotionUncertaintyConfig &config,
                                  Eigen::Matrix<double, 6, 6> *covariance);

// FAST-LIVO: right body rotation + additive world translation. GTSAM Pose3:
// right-local rotation + right-local translation. Both order [rot, trans].
Eigen::Matrix<double, 6, 6> fastLivoPoseCovarianceToGtsamRight(
    const Eigen::Matrix<double, 6, 6> &covariance,
    const Eigen::Matrix3d &world_R_body);

struct SparseMotionSummary
{
  std::uint64_t from_keypose_id = 0;
  std::uint64_t to_keypose_id = 0;
  Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  MotionCovarianceSource covariance_source = MotionCovarianceSource::Unavailable;
  bool covariance_valid = false;
  double from_timestamp = std::numeric_limits<double>::quiet_NaN();
  double to_timestamp = std::numeric_limits<double>::quiet_NaN();
};

struct GlobalCorrection
{
  double timestamp = std::numeric_limits<double>::quiet_NaN();
  Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
  bool valid = false;
};

class PersistentLandmarkBackendInterface
{
public:
  virtual ~PersistentLandmarkBackendInterface() = default;
  virtual bool submit(const GlobalLandmarkInput &input,
                      std::string *reject_reason) = 0;
  virtual bool latestCorrection(GlobalCorrection *correction) const = 0;
  virtual bool lookupLandmark(int landmark_id,
                              PersistentLandmark *landmark) const = 0;
  virtual bool lookupKeyPose(std::uint64_t keypose_id,
                             SparseKeyPose *keypose) const = 0;
};

struct RouteResult
{
  FusionMode mode = FusionMode::ObserveOnly;
  std::size_t delivered_observations = 0;
  int primary_consumer_count = 0;
  bool accepted = false;
  std::string reason;
};

class LandmarkObservationRouter
{
public:
  using Batch = std::vector<LandmarkObservation>;
  using Consumer = std::function<void(const Batch &)>;

  explicit LandmarkObservationRouter(
      FusionMode mode = FusionMode::ObserveOnly) : mode_(mode) {}

  void setMode(FusionMode mode) { mode_ = mode; }
  FusionMode mode() const { return mode_; }

  RouteResult route(const Batch &observations,
                    const Consumer &observe_only_consumer,
                    const Consumer &legacy_esikf_consumer,
                    const Consumer &global_backend_consumer) const;

private:
  FusionMode mode_;
};

} // namespace landmark

#endif // LANDMARK_ARCHITECTURE_H_
