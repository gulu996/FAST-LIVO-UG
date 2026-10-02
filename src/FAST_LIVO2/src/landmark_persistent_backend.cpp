#include "landmark_persistent_backend.h"
#include "landmark_shadow_graph.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace landmark
{
namespace
{
double rotationDegrees(const Eigen::Isometry3d &relative)
{
  return Eigen::AngleAxisd(relative.linear()).angle() * 180.0 / M_PI;
}

bool validInput(const GlobalLandmarkInput &input)
{
  return input.observation_id != 0 &&
      input.observation.landmark_id >= 0 &&
      input.observation.pose_valid && input.observation.covariance_valid &&
      std::isfinite(input.observation.timestamp) &&
      input.observation.timestamp >= 0.0 &&
      std::isfinite(input.local_pose_timestamp) &&
      input.local_pose_timestamp >= 0.0 &&
      std::abs(input.observation.timestamp - input.local_pose_timestamp) < 1e-6 &&
      input.observation.T_camera_landmark.matrix().allFinite() &&
      input.observation.pose_covariance_camera.allFinite() &&
      input.T_body_camera.matrix().allFinite() &&
      input.local_pose_reference.matrix().allFinite() &&
      input.local_pose_covariance.allFinite() &&
      input.landmark_initial_guess.matrix().allFinite() &&
      input.landmark_initial_covariance.allFinite() &&
      !input.local_frame_id.empty() && !input.camera_extrinsic_id.empty();
}
} // namespace

PersistentLandmarkBackend::PersistentLandmarkBackend(
    const PersistentBackendConfig &config, bool start_worker) : config_(config)
{
  shadow_graph_.reset(new SparseLandmarkShadowGraph(config_.counterfactual_snapshot_enable));
  const auto &p = config_.keypose_policy;
  if (config_.queue_capacity == 0 ||
      !std::isfinite(p.translation_threshold_m) || p.translation_threshold_m <= 0 ||
      !std::isfinite(p.rotation_threshold_deg) || p.rotation_threshold_deg <= 0 ||
      !std::isfinite(p.maximum_interval_s) || p.maximum_interval_s <= 0 ||
      !std::isfinite(p.coalesce_time_s) || p.coalesce_time_s < 0 ||
      !std::isfinite(p.coalesce_translation_m) || p.coalesce_translation_m < 0 ||
      !std::isfinite(p.coalesce_rotation_deg) || p.coalesce_rotation_deg < 0 ||
      !std::isfinite(p.episode_gap_s) || p.episode_gap_s <= 0 ||
      !validConservativeMotionConfig(config_.motion_uncertainty))
    throw std::invalid_argument("invalid persistent landmark backend configuration");
  if (start_worker) worker_ = std::thread(&PersistentLandmarkBackend::workerLoop, this);
}

PersistentLandmarkBackend::~PersistentLandmarkBackend() { shutdown(); }

bool PersistentLandmarkBackend::submit(const GlobalLandmarkInput &input,
                                        std::string *reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.submitted;
  auto reject = [&](const char *why) {
    if (reason) *reason = why;
    return false;
  };
  if (stopping_) return reject("BACKEND_SHUTDOWN");
  if (!validInput(input))
  {
    ++counters_.invalid;
    return reject("INVALID_OBSERVATION_INPUT");
  }
  if (accepted_ids_.count(input.observation_id))
  {
    ++counters_.duplicate;
    return reject("DUPLICATE_OBSERVATION_ID");
  }
  if (input.observation.timestamp < last_accepted_timestamp_)
  {
    ++counters_.out_of_order;
    return reject("OUT_OF_ORDER_TIMESTAMP");
  }
  // Reject newest: accepted observations retain their original time order.
  if (queue_.size() >= config_.queue_capacity)
  {
    ++counters_.overflow;
    return reject("QUEUE_OVERFLOW_REJECT_NEWEST");
  }
  queue_.push_back(input);
  queue_peak_size_ = std::max(queue_peak_size_, queue_.size());
  accepted_ids_.insert(input.observation_id);
  last_accepted_timestamp_ = input.observation.timestamp;
  ++counters_.accepted;
  if (reason) *reason = "ACCEPTED";
  ready_.notify_one();
  return true;
}

bool PersistentLandmarkBackend::processOne()
{
  GlobalLandmarkInput input;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    input = std::move(queue_.front());
    queue_.pop_front();
  }
  processAccepted(input);
  return true;
}

void PersistentLandmarkBackend::workerLoop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    worker_started_.store(true);
    worker_alive_.store(true);
    ++worker_start_count_;
  }
  for (;;)
  {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
      if (stopping_ && queue_.empty())
      {
        drain_completed_ = true;
        worker_stopped_ = true;
        ++worker_stop_count_;
        worker_alive_.store(false);
        return;
      }
    }
    processOne();
  }
}

void PersistentLandmarkBackend::processAccepted(const GlobalLandmarkInput &input)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto &obs = input.observation;
  PersistentLandmark &landmark = landmarks_[obs.landmark_id];
  const bool first = !landmark.has_initial_guess;
  const bool new_episode = first ||
      obs.timestamp - landmark.episode_last_timestamp >
          config_.keypose_policy.episode_gap_s;
  if (first)
  {
    landmark.landmark_id = obs.landmark_id;
    landmark.initial_guess = input.landmark_initial_guess;
    landmark.has_initial_guess = true;
    landmark.covariance = input.landmark_initial_covariance;
    landmark.first_seen_timestamp = obs.timestamp;
    landmark.status = PersistentLandmarkStatus::Initialized;
  }
  else
  {
    landmark.status = PersistentLandmarkStatus::Active;
  }
  if (new_episode)
  {
    landmark.episode_id = next_episode_id_++;
    landmark.episode_first_timestamp = obs.timestamp;
  }
  landmark.episode_last_timestamp = obs.timestamp;
  landmark.last_seen_timestamp = obs.timestamp;
  ++landmark.observation_count;
  landmark.associated_observation_ids.push_back(input.observation_id);

  const bool coalesce = !keyposes_.empty() && !new_episode &&
      obs.timestamp - keyposes_.back().timestamp <=
          config_.keypose_policy.coalesce_time_s &&
      (keyposes_.back().local_pose_reference.inverse() *
       input.local_pose_reference).translation().norm() <=
          config_.keypose_policy.coalesce_translation_m &&
      rotationDegrees(keyposes_.back().local_pose_reference.inverse() *
                      input.local_pose_reference) <=
          config_.keypose_policy.coalesce_rotation_deg;
  bool graph_visual_eligible = coalesce;
  bool created_keypose = false;
  if (coalesce)
  {
    SparseKeyPose &keypose = keyposes_.back();
    keypose.raw_lio_weak_geometry |= input.raw_lio_weak_geometry;
    keypose.source_observation_ids.push_back(input.observation_id);
    if (std::find(keypose.associated_landmark_ids.begin(),
                  keypose.associated_landmark_ids.end(), obs.landmark_id) ==
        keypose.associated_landmark_ids.end())
      keypose.associated_landmark_ids.push_back(obs.landmark_id);
  }
  else
  {
    const bool create = keyposes_.empty() || new_episode ||
        shouldCreateSparseKeyPose(
            keyposes_.back().local_pose_reference, keyposes_.back().timestamp,
            input.local_pose_reference, obs.timestamp, first, new_episode,
            config_.keypose_policy);
    if (create)
    {
      graph_visual_eligible = true;
      created_keypose = true;
      SparseKeyPose keypose;
      keypose.keypose_id = next_keypose_id_++;
      keypose.timestamp = obs.timestamp;
      keypose.initial_global_pose = input.local_pose_reference;
      keypose.local_pose_reference = input.local_pose_reference;
      keypose.pose_covariance = input.local_pose_covariance;
      keypose.raw_lio_weak_geometry = input.raw_lio_weak_geometry;
      keypose.source_observation_ids.push_back(input.observation_id);
      keypose.associated_landmark_ids.push_back(obs.landmark_id);
      keypose.trigger_reason = first ? "LANDMARK_FIRST_OBSERVATION" :
          new_episode ? "LANDMARK_REOBSERVATION" :
          (keypose.local_pose_reference.translation() -
           keyposes_.back().local_pose_reference.translation()).norm() >=
               config_.keypose_policy.translation_threshold_m ?
              "TRANSLATION_THRESHOLD" :
          rotationDegrees(keyposes_.back().local_pose_reference.inverse() *
                          keypose.local_pose_reference) >=
              config_.keypose_policy.rotation_threshold_deg ?
              "ROTATION_THRESHOLD" : "TIME_THRESHOLD";
      if (!keyposes_.empty())
      {
        SparseMotionSummary motion;
        motion.from_keypose_id = keyposes_.back().keypose_id;
        motion.to_keypose_id = keypose.keypose_id;
        motion.from_timestamp = keyposes_.back().timestamp;
        motion.to_timestamp = keypose.timestamp;
        motion.relative_pose = keyposes_.back().local_pose_reference.inverse() *
                               keypose.local_pose_reference;
        // No cross-time covariance is available: this is an uncalibrated drift
        // envelope, not P_i + P_j or a strict statistical covariance bound.
        motion.covariance_valid = conservativeMotionCovariance(
            motion.relative_pose, motion.to_timestamp - motion.from_timestamp,
            keyposes_.back().raw_lio_weak_geometry || keypose.raw_lio_weak_geometry,
            config_.motion_uncertainty, &motion.covariance);
        if (motion.covariance_valid)
          motion.covariance_source = MotionCovarianceSource::Conservative;
        motions_.push_back(motion);
      }
      keyposes_.push_back(std::move(keypose));
    }
    else if (!keyposes_.empty())
    {
      keyposes_.back().raw_lio_weak_geometry |= input.raw_lio_weak_geometry;
      keyposes_.back().source_observation_ids.push_back(input.observation_id);
      if (std::find(keyposes_.back().associated_landmark_ids.begin(),
                    keyposes_.back().associated_landmark_ids.end(),
                    obs.landmark_id) ==
          keyposes_.back().associated_landmark_ids.end())
        keyposes_.back().associated_landmark_ids.push_back(obs.landmark_id);
    }
  }
  ++counters_.processed;
  if (!keyposes_.empty())
  {
    const SparseMotionSummary *motion =
        created_keypose && keyposes_.size() > 1 ? &motions_.back() : nullptr;
    if (shadow_graph_->update(keyposes_.back(), motion, input,
                             landmark.episode_id, graph_visual_eligible))
    {
      Eigen::Isometry3d estimate;
      if (shadow_graph_->estimateKeyPose(keyposes_.back().keypose_id, &estimate))
        keyposes_.back().backend_key = SparseLandmarkShadowGraph::keyposeKey(
            keyposes_.back().keypose_id);
      if (shadow_graph_->estimateLandmark(obs.landmark_id, &estimate))
      {
        landmark.backend_key = SparseLandmarkShadowGraph::landmarkKey(
            obs.landmark_id);
        landmark.optimized_estimate = estimate;
        landmark.has_optimized_estimate = true;
      }
    }
  }
}

void PersistentLandmarkBackend::shutdown()
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_requested_) return;
    shutdown_requested_ = true;
    drain_started_ = true;
    queue_size_at_shutdown_ = queue_.size();
    stopping_ = true;
  }
  ready_.notify_all();
  if (worker_.joinable()) worker_.join();
  else
  {
    while (processOne()) {} // deterministic no-worker test mode
    std::lock_guard<std::mutex> lock(mutex_);
    drain_completed_ = true;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    join_completed_ = true;
  }
}

PersistentBackendTelemetry PersistentLandmarkBackend::telemetry() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  PersistentBackendTelemetry value;
  value.worker_started = worker_started_.load();
  value.worker_start_count = worker_start_count_;
  value.shutdown_requested = shutdown_requested_;
  value.drain_started = drain_started_;
  value.drain_completed = drain_completed_;
  value.worker_stopped = worker_stopped_;
  value.worker_stop_count = worker_stop_count_;
  value.join_completed = join_completed_;
  value.queue_capacity = config_.queue_capacity;
  value.queue_current_size = queue_.size();
  value.queue_peak_size = queue_peak_size_;
  value.queue_size_at_shutdown = queue_size_at_shutdown_;
  value.counters = counters_;
  value.rejected = counters_.submitted - counters_.accepted;
  return value;
}

ShadowGraphTelemetry PersistentLandmarkBackend::graphTelemetry() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return shadow_graph_->telemetry();
}

std::string PersistentLandmarkBackend::graphSummary() const
{
  const auto g = graphTelemetry();
  std::ostringstream summary;
  summary << " graph_initialized=" << g.graph_initialized
          << " graph_degraded=" << g.degraded
          << " isam_update_count=" << g.isam_update_count
          << " keypose_variable_count=" << g.keypose_variable_count
          << " landmark_variable_count=" << g.landmark_variable_count
          << " motion_factor_count=" << g.motion_factor_count
          << " visual_factor_count=" << g.visual_factor_count
          << " gauge_prior_count=" << g.gauge_prior_count
          << " suppressed_visual_observation_count="
          << g.suppressed_visual_observation_count
          << " duplicate_factor_count=" << g.duplicate_factor_count
          << " motion_factor_reject_count=" << g.motion_factor_reject_count
          << " visual_factor_reject_count=" << g.visual_factor_reject_count
          << " isam_exception_count=" << g.isam_exception_count
          << " cross_episode_reobservation_count=" << g.cross_episode_reobservation_count
          << " reobservation_diagnostic_failure_count=" << g.reobservation_diagnostic_failure_count
          << " counterfactual_snapshot_failure_count=" << g.counterfactual_snapshot_failure_count
          << " motion_covariance_source=" << g.motion_covariance_source
          << " last_update_time=" << (g.graph_initialized ?
              std::to_string(g.last_update_time) : "NONE")
          << " update_latency_ms=" << g.update_latency_ms
          << " max_update_latency_ms=" << g.max_update_latency_ms
          << " latest_graph_error=" << g.latest_graph_error;
  return summary.str();
}

void PersistentLandmarkBackend::writeReobservationDiagnosticsCsv(std::ostream &out) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  shadow_graph_->writeReobservationDiagnosticsCsv(out);
}

void PersistentLandmarkBackend::writeCounterfactualSnapshots(const std::string &directory) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  shadow_graph_->writeCounterfactualSnapshots(directory);
}

bool PersistentLandmarkBackend::latestCorrection(GlobalCorrection *correction) const
{
  if (correction) *correction = GlobalCorrection();
  return false;
}

bool PersistentLandmarkBackend::lookupLandmark(
    int id, PersistentLandmark *landmark) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = landmarks_.find(id);
  if (!landmark || it == landmarks_.end()) return false;
  *landmark = it->second;
  Eigen::Isometry3d current_estimate;
  if (shadow_graph_->estimateLandmark(id, &current_estimate))
  {
    landmark->optimized_estimate = current_estimate;
    landmark->has_optimized_estimate = true;
  }
  return true;
}

bool PersistentLandmarkBackend::lookupOptimizedKeyPose(
    std::uint64_t id, Eigen::Isometry3d *pose) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return shadow_graph_->estimateKeyPose(id, pose);
}

bool PersistentLandmarkBackend::lookupKeyPose(
    std::uint64_t id, SparseKeyPose *keypose) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!keypose || id == 0 || id > keyposes_.size()) return false;
  *keypose = keyposes_[id - 1];
  return true;
}

PersistentBackendCounters PersistentLandmarkBackend::counters() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return counters_;
}

std::size_t PersistentLandmarkBackend::queued() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

std::size_t PersistentLandmarkBackend::keyposeCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return keyposes_.size();
}

std::vector<SparseMotionSummary> PersistentLandmarkBackend::motionSummaries() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return motions_;
}

} // namespace landmark
