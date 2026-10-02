#ifndef LANDMARK_SHADOW_GRAPH_H_
#define LANDMARK_SHADOW_GRAPH_H_

#include "landmark_architecture.h"
#include "landmark_counterfactual.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>

#include <map>
#include <iosfwd>
#include <set>
#include <string>
#include <tuple>

namespace landmark
{

struct ReobservationResidual
{
  Eigen::Matrix<double, 6, 1> logmap = Eigen::Matrix<double, 6, 1>::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix<double, 6, 1> unwhitened = logmap;
  Eigen::Matrix<double, 6, 1> whitened = logmap;
  double rotation_deg = std::numeric_limits<double>::quiet_NaN();
  double translation_m = std::numeric_limits<double>::quiet_NaN();
  double whitened_squared_norm = std::numeric_limits<double>::quiet_NaN();
  double factor_error = std::numeric_limits<double>::quiet_NaN();
  double logmap_factor_difference = std::numeric_limits<double>::quiet_NaN();
};

struct ReobservationDiagnostic
{
  double timestamp = 0.0;
  int landmark_id = -1;
  std::uint64_t episode_id = 0, previous_episode_id = 0;
  std::uint64_t keypose_id = 0, observation_id = 0;
  double episode_gap_s = 0.0; // previous episode's last backend observation
  bool pre_keypose_in_isam = false;
  bool pre_valid = false, post_valid = false, factor_accepted = false;
  ReobservationResidual pre, post;
  Eigen::Isometry3d measurement = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d keypose_before = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d keypose_after = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d landmark_before = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d landmark_after = Eigen::Isometry3d::Identity();
  double keypose_delta_translation_m = std::numeric_limits<double>::quiet_NaN();
  double keypose_delta_rotation_deg = std::numeric_limits<double>::quiet_NaN();
  double landmark_delta_translation_m = std::numeric_limits<double>::quiet_NaN();
  double landmark_delta_rotation_deg = std::numeric_limits<double>::quiet_NaN();
  double pre_existing_graph_error = std::numeric_limits<double>::quiet_NaN();
  double pre_pending_nonvisual_error = std::numeric_limits<double>::quiet_NaN();
  double pre_existing_plus_candidate_error = std::numeric_limits<double>::quiet_NaN();
  double pre_augmented_graph_error = std::numeric_limits<double>::quiet_NaN();
  double post_full_graph_error = std::numeric_limits<double>::quiet_NaN();
  double isam_update_latency_ms = 0.0;
  std::string motion_covariance_source = "NONE";
  Matrix6d visual_covariance_right = Matrix6d::Zero();
  bool large_correction = false; // diagnostic only: >=2 m or >=30 deg
  std::string diagnostic_error;
};

struct ShadowGraphTelemetry
{
  bool graph_initialized = false;
  bool degraded = false;
  std::uint64_t isam_update_count = 0;
  std::size_t keypose_variable_count = 0;
  std::size_t landmark_variable_count = 0;
  std::uint64_t motion_factor_count = 0;
  std::uint64_t visual_factor_count = 0;
  std::uint64_t gauge_prior_count = 0;
  std::uint64_t suppressed_visual_observation_count = 0;
  std::uint64_t duplicate_factor_count = 0;
  std::uint64_t motion_factor_reject_count = 0;
  std::uint64_t visual_factor_reject_count = 0;
  std::uint64_t isam_exception_count = 0;
  std::uint64_t cross_episode_reobservation_count = 0;
  std::uint64_t reobservation_diagnostic_failure_count = 0;
  std::uint64_t counterfactual_snapshot_failure_count = 0;
  std::string latest_graph_error;
  std::string motion_covariance_source = "NONE";
  double last_update_time = std::numeric_limits<double>::quiet_NaN();
  double update_latency_ms = 0.0;
  double max_update_latency_ms = 0.0;
};

// Independent, read-only-to-production ISAM2 graph. Caller serializes updates.
class SparseLandmarkShadowGraph
{
public:
  explicit SparseLandmarkShadowGraph(bool capture_counterfactual = false)
      : capture_counterfactual_(capture_counterfactual) {}
  bool update(const SparseKeyPose &keypose, const SparseMotionSummary *motion,
              const GlobalLandmarkInput &input, std::uint64_t episode_id,
              bool visual_eligible = true);
  ShadowGraphTelemetry telemetry() const { return telemetry_; }
  bool estimateKeyPose(std::uint64_t id, Eigen::Isometry3d *pose) const;
  bool estimateLandmark(int id, Eigen::Isometry3d *pose) const;
  const std::vector<ReobservationDiagnostic> &reobservationDiagnostics() const
  { return reobservation_diagnostics_; }
  void writeReobservationDiagnosticsCsv(std::ostream &out) const;
  void writeCounterfactualSnapshots(const std::string &directory) const;
  const std::vector<CounterfactualSnapshot> &counterfactualSnapshots() const
  { return counterfactual_snapshots_; }
  static Eigen::Isometry3d visualMeasurement(const GlobalLandmarkInput &input);
  static Matrix6d visualCovarianceRight(const LandmarkObservation &observation);
  static gtsam::Key keyposeKey(std::uint64_t id);
  static gtsam::Key landmarkKey(int id);

private:
  gtsam::ISAM2 isam_;
  gtsam::Values estimate_;
  std::set<std::uint64_t> keypose_ids_;
  std::set<int> landmark_ids_;
  std::set<std::pair<std::uint64_t, std::uint64_t>> motion_edges_;
  std::set<std::tuple<std::uint64_t, int, std::uint64_t>> visual_edges_;
  // Episodes enter this history only after a formal visual factor succeeds.
  std::map<int, std::pair<std::uint64_t, double>> landmark_factor_episodes_;
  // ponytail: retain one small record per episode transition; streaming can
  // replace this history for multi-day operation, without retaining sensor data.
  std::vector<ReobservationDiagnostic> reobservation_diagnostics_;
  // ponytail: opt-in, one frozen factor/Values snapshot per transition. Memory
  // grows with events times graph size; stream snapshots for long campaigns.
  bool capture_counterfactual_ = false;
  std::vector<CounterfactualSnapshot> counterfactual_snapshots_;
  ShadowGraphTelemetry telemetry_;
  bool failed_ = false;
};

} // namespace landmark
#endif
