#ifndef LANDMARK_PERSISTENT_BACKEND_H_
#define LANDMARK_PERSISTENT_BACKEND_H_

#include "landmark_architecture.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

namespace landmark
{

class SparseLandmarkShadowGraph;
struct ShadowGraphTelemetry;

struct PersistentBackendConfig
{
  std::size_t queue_capacity = 128;
  SparseKeyPosePolicy keypose_policy;
  ConservativeMotionUncertaintyConfig motion_uncertainty;
};

struct PersistentBackendCounters
{
  std::uint64_t submitted = 0;
  std::uint64_t accepted = 0;
  std::uint64_t duplicate = 0;
  std::uint64_t overflow = 0;
  std::uint64_t out_of_order = 0;
  std::uint64_t processed = 0;
  std::uint64_t invalid = 0;
};

struct PersistentBackendTelemetry
{
  bool worker_started = false;
  std::uint64_t worker_start_count = 0;
  bool shutdown_requested = false;
  bool drain_started = false;
  bool drain_completed = false;
  bool worker_stopped = false;
  std::uint64_t worker_stop_count = 0;
  bool join_completed = false;
  std::size_t queue_capacity = 0;
  std::size_t queue_current_size = 0;
  std::size_t queue_peak_size = 0;
  std::size_t queue_size_at_shutdown = 0;
  PersistentBackendCounters counters;
  std::uint64_t rejected = 0; // submitted - accepted; no second counter
};

// Shadow-only input infrastructure. No graph, ESIKF update, or ROS publisher.
class PersistentLandmarkBackend final : public PersistentLandmarkBackendInterface
{
public:
  explicit PersistentLandmarkBackend(const PersistentBackendConfig &config,
                                      bool start_worker = true);
  ~PersistentLandmarkBackend() override;
  PersistentLandmarkBackend(const PersistentLandmarkBackend &) = delete;
  PersistentLandmarkBackend &operator=(const PersistentLandmarkBackend &) = delete;

  bool submit(const GlobalLandmarkInput &input,
              std::string *reject_reason) override;
  bool latestCorrection(GlobalCorrection *correction) const override;
  bool lookupLandmark(int landmark_id,
                      PersistentLandmark *landmark) const override;
  bool lookupKeyPose(std::uint64_t keypose_id,
                     SparseKeyPose *keypose) const override;

  // Deterministic drain is useful for tests and controlled shutdown.
  bool processOne();
  void shutdown();
  PersistentBackendCounters counters() const;
  PersistentBackendTelemetry telemetry() const;
  ShadowGraphTelemetry graphTelemetry() const;
  std::string graphSummary() const;
  std::size_t queued() const;
  std::size_t keyposeCount() const;
  std::vector<SparseMotionSummary> motionSummaries() const;
  bool lookupOptimizedKeyPose(std::uint64_t id, Eigen::Isometry3d *pose) const;
  bool workerAlive() const { return worker_alive_.load(); }
  bool workerStarted() const { return worker_started_.load(); }

private:
  void workerLoop();
  void processAccepted(const GlobalLandmarkInput &input);

  PersistentBackendConfig config_;
  mutable std::mutex mutex_;
  std::mutex shutdown_mutex_;
  std::condition_variable ready_;
  std::deque<GlobalLandmarkInput> queue_;
  std::set<std::uint64_t> accepted_ids_;
  std::map<int, PersistentLandmark> landmarks_;
  std::vector<SparseKeyPose> keyposes_;
  std::vector<SparseMotionSummary> motions_;
  PersistentBackendCounters counters_;
  std::unique_ptr<SparseLandmarkShadowGraph> shadow_graph_;
  std::thread worker_;
  double last_accepted_timestamp_ = -1.0;
  std::uint64_t next_keypose_id_ = 1;
  std::uint64_t next_episode_id_ = 1;
  bool stopping_ = false;
  bool shutdown_requested_ = false;
  bool drain_started_ = false;
  bool drain_completed_ = false;
  bool worker_stopped_ = false;
  bool join_completed_ = false;
  std::uint64_t worker_start_count_ = 0;
  std::uint64_t worker_stop_count_ = 0;
  std::size_t queue_peak_size_ = 0;
  std::size_t queue_size_at_shutdown_ = 0;
  std::atomic<bool> worker_started_{false};
  std::atomic<bool> worker_alive_{false};
};

} // namespace landmark

#endif // LANDMARK_PERSISTENT_BACKEND_H_
