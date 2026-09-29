#ifndef LANDMARK_SHADOW_GRAPH_H_
#define LANDMARK_SHADOW_GRAPH_H_

#include "landmark_architecture.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>

#include <map>
#include <set>
#include <string>
#include <tuple>

namespace landmark
{

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
  SparseLandmarkShadowGraph() = default;
  bool update(const SparseKeyPose &keypose, const SparseMotionSummary *motion,
              const GlobalLandmarkInput &input, std::uint64_t episode_id,
              bool visual_eligible = true);
  ShadowGraphTelemetry telemetry() const { return telemetry_; }
  bool estimateKeyPose(std::uint64_t id, Eigen::Isometry3d *pose) const;
  bool estimateLandmark(int id, Eigen::Isometry3d *pose) const;
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
  ShadowGraphTelemetry telemetry_;
  bool failed_ = false;
};

} // namespace landmark
#endif
