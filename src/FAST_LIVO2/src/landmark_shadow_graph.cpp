#include "landmark_shadow_graph.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>
#include <fstream>
#include <ostream>
#include <stdexcept>

namespace landmark
{
namespace
{
gtsam::Pose3 pose3(const Eigen::Isometry3d &pose)
{
  return gtsam::Pose3(gtsam::Rot3(pose.linear()),
                      gtsam::Point3(pose.translation()));
}

Eigen::Isometry3d eigenPose(const gtsam::Pose3 &pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.rotation().matrix();
  result.translation() = pose.translation();
  return result;
}

bool positiveCovariance(const Matrix6d &covariance)
{
  if (!covariance.allFinite() ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-8)
    return false;
  const Eigen::SelfAdjointEigenSolver<Matrix6d> eig(covariance);
  return eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 0.0;
}

const char *sourceName(MotionCovarianceSource source)
{
  switch (source)
  {
    case MotionCovarianceSource::Exact: return "EXACT";
    case MotionCovarianceSource::Approximate: return "APPROXIMATE";
    case MotionCovarianceSource::Conservative: return "CONSERVATIVE";
    default: return "UNAVAILABLE";
  }
}

ReobservationResidual residual(
    const gtsam::BetweenFactor<gtsam::Pose3> &factor,
    const gtsam::Values &values)
{
  ReobservationResidual result;
  const auto predicted = values.at<gtsam::Pose3>(factor.key1()).between(
      values.at<gtsam::Pose3>(factor.key2()));
  result.logmap = gtsam::Pose3::Logmap(factor.measured().between(predicted));
  result.unwhitened = factor.unwhitenedError(values);
  result.whitened = factor.whitenedError(values);
  result.rotation_deg = result.logmap.head<3>().norm() * 180.0 / M_PI;
  result.translation_m = result.logmap.tail<3>().norm();
  result.whitened_squared_norm = result.whitened.squaredNorm();
  result.factor_error = factor.error(values);
  result.logmap_factor_difference =
      (result.logmap - result.unwhitened).norm();
  if (!result.logmap.allFinite() || !result.unwhitened.allFinite() ||
      !result.whitened.allFinite() || !std::isfinite(result.factor_error) ||
      result.logmap_factor_difference > 1e-8)
    throw std::runtime_error("REOBSERVATION_NONFINITE_OR_FACTOR_CHART_MISMATCH");
  return result;
}
} // namespace

gtsam::Key SparseLandmarkShadowGraph::keyposeKey(std::uint64_t id)
{
  return gtsam::Symbol('x', id);
}

gtsam::Key SparseLandmarkShadowGraph::landmarkKey(int id)
{
  return gtsam::Symbol('l', static_cast<std::uint64_t>(id));
}

Eigen::Isometry3d SparseLandmarkShadowGraph::visualMeasurement(
    const GlobalLandmarkInput &input)
{
  return input.T_body_camera * input.observation.T_camera_landmark;
}

Matrix6d SparseLandmarkShadowGraph::visualCovarianceRight(
    const LandmarkObservation &observation)
{
  // Frontend perturbs R_cl on the left in camera axes and t_cl additively in
  // camera axes. Pose3/BetweenFactor uses right-local [rot, trans].
  Matrix6d J = Matrix6d::Zero();
  const Eigen::Matrix3d R_lc = observation.T_camera_landmark.linear().transpose();
  J.topLeftCorner<3, 3>() = R_lc;
  J.bottomRightCorner<3, 3>() = R_lc;
  // Fixed left multiplication by T_body_camera preserves right-local delta.
  return J * observation.pose_covariance_camera * J.transpose();
}

bool SparseLandmarkShadowGraph::update(
    const SparseKeyPose &keypose, const SparseMotionSummary *motion,
    const GlobalLandmarkInput &input, std::uint64_t episode_id,
    bool visual_eligible)
{
  if (failed_) return false;
  if (keypose.keypose_id == 0 ||
      keypose.keypose_id > ((std::uint64_t{1} << 56) - 1) ||
      (visual_eligible && input.observation.landmark_id < 0))
  {
    telemetry_.degraded = true;
    telemetry_.latest_graph_error = "GRAPH_KEY_OUT_OF_RANGE";
    return false;
  }
  const bool new_keypose = !keypose_ids_.count(keypose.keypose_id);
  const bool new_landmark = !landmark_ids_.count(input.observation.landmark_id);
  const auto visual_id = std::make_tuple(
      keypose.keypose_id, input.observation.landmark_id, episode_id);
  // ponytail: first valid factor wins per sparse keypose/board/episode;
  // no online replacement. Upgrade to batch quality selection if calibrated data warrants it.
  const bool duplicate_visual = visual_edges_.count(visual_id);
  if (!visual_eligible || duplicate_visual)
  {
    ++telemetry_.suppressed_visual_observation_count;
    if (duplicate_visual) ++telemetry_.duplicate_factor_count;
  }
  const bool add_visual = visual_eligible && !duplicate_visual;
  const int landmark_id = input.observation.landmark_id;
  const auto history = landmark_factor_episodes_.find(landmark_id);
  const bool reobservation = add_visual &&
      history != landmark_factor_episodes_.end() &&
      history->second.first != episode_id;
  if (new_keypose && !keypose_ids_.empty())
  {
    if (!motion || motion->to_keypose_id != keypose.keypose_id ||
        !keypose_ids_.count(motion->from_keypose_id) ||
        motion->covariance_source == MotionCovarianceSource::Unavailable ||
        !motion->covariance_valid || !positiveCovariance(motion->covariance) ||
        !motion->relative_pose.matrix().allFinite())
    {
      ++telemetry_.motion_factor_reject_count;
      telemetry_.degraded = true;
      telemetry_.latest_graph_error = "MOTION_FACTOR_UNAVAILABLE_OR_INVALID";
      return false; // never insert an unanchored keypose or substitute noise
    }
  }
  if (add_visual &&
      (!input.observation.pose_valid || !input.observation.covariance_valid ||
       !input.observation.T_camera_landmark.matrix().allFinite() ||
       !input.T_body_camera.matrix().allFinite() ||
       !positiveCovariance(visualCovarianceRight(input.observation))))
  {
    ++telemetry_.visual_factor_reject_count;
    telemetry_.degraded = true;
    telemetry_.latest_graph_error = "VISUAL_FACTOR_INVALID";
    return false;
  }
  if (!new_keypose && !add_visual)
  {
    if (history != landmark_factor_episodes_.end() &&
        history->second.first == episode_id)
      history->second.second = input.observation.timestamp;
    return true;
  }

  const auto start = std::chrono::steady_clock::now();
  try
  {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    ReobservationDiagnostic diagnostic;
    CounterfactualSnapshot snapshot;
    bool have_snapshot = false;
    gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr visual_factor;
    const gtsam::Key x = keyposeKey(keypose.keypose_id);
    const gtsam::Key l = landmarkKey(input.observation.landmark_id);
    if (new_keypose)
    {
      if (!keypose.local_pose_reference.matrix().allFinite())
        throw std::invalid_argument("nonfinite raw keypose");
      values.insert(x, pose3(keypose.local_pose_reference));
      if (keypose_ids_.empty())
      {
        // Gauge only: center is raw K0, never external truth or GNSS.
        gtsam::Vector6 sigma;
        sigma << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3;
        factors.add(gtsam::PriorFactor<gtsam::Pose3>(
            x, pose3(keypose.local_pose_reference),
            gtsam::noiseModel::Diagonal::Sigmas(sigma)));
      }
      else
      {
        const auto edge = std::make_pair(motion->from_keypose_id,
                                         motion->to_keypose_id);
        if (motion_edges_.count(edge))
        {
          ++telemetry_.duplicate_factor_count;
          telemetry_.latest_graph_error = "DUPLICATE_MOTION_FACTOR";
          return false;
        }
        factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
            keyposeKey(motion->from_keypose_id), x,
            pose3(motion->relative_pose),
            gtsam::noiseModel::Gaussian::Covariance(motion->covariance)));
      }
    }
    if (add_visual)
    {
      const Eigen::Isometry3d measurement = visualMeasurement(input);
      if (new_landmark)
      {
        // Initial value only. Never create a prior on a self-mapped board.
        values.insert(l, pose3(input.landmark_initial_guess));
      }
      visual_factor.reset(new gtsam::BetweenFactor<gtsam::Pose3>(
          x, l, pose3(measurement),
          gtsam::noiseModel::Gaussian::Covariance(
              visualCovarianceRight(input.observation))));
      if (reobservation)
      {
        diagnostic.timestamp = input.observation.timestamp;
        diagnostic.landmark_id = landmark_id;
        diagnostic.episode_id = episode_id;
        diagnostic.previous_episode_id = history->second.first;
        diagnostic.episode_gap_s = input.observation.timestamp - history->second.second;
        diagnostic.keypose_id = keypose.keypose_id;
        diagnostic.observation_id = input.observation_id;
        diagnostic.pre_keypose_in_isam = !new_keypose;
        diagnostic.measurement = measurement;
        diagnostic.visual_covariance_right = visualCovarianceRight(input.observation);
        diagnostic.motion_covariance_source = motion ?
            sourceName(motion->covariance_source) : telemetry_.motion_covariance_source;
        // New K_i has no current ISAM estimate: use the unchanged raw Values
        // seed, and explicitly identify it in CSV. No extra ISAM update.
        try
        {
          gtsam::Values before = estimate_;
          before.insert(values);
          diagnostic.keypose_before = eigenPose(before.at<gtsam::Pose3>(x));
          diagnostic.landmark_before = eigenPose(before.at<gtsam::Pose3>(l));
          diagnostic.pre = residual(*visual_factor, before);
          diagnostic.pre_existing_graph_error = isam_.getFactorsUnsafe().error(before);
          diagnostic.pre_pending_nonvisual_error = factors.error(before);
          diagnostic.pre_existing_plus_candidate_error =
              diagnostic.pre_existing_graph_error + diagnostic.pre.factor_error;
          diagnostic.pre_augmented_graph_error =
              diagnostic.pre_existing_plus_candidate_error +
              diagnostic.pre_pending_nonvisual_error;
          diagnostic.pre_valid = std::isfinite(diagnostic.pre_augmented_graph_error);
          if (!diagnostic.pre_valid) throw std::runtime_error("NONFINITE_PRE_GRAPH_ERROR");
        }
        catch (const std::exception &e) { diagnostic.diagnostic_error = e.what(); }
        if (capture_counterfactual_)
        {
          try
          {
            snapshot.timestamp = input.observation.timestamp;
            snapshot.observation_id = input.observation_id;
            snapshot.keypose_id = keypose.keypose_id;
            snapshot.landmark_id = landmark_id;
            snapshot.existing = isam_.getFactorsUnsafe();
            snapshot.motion = factors; // candidate visual is not yet added
            snapshot.visual.add(visual_factor);
            snapshot.existing_values = estimate_;
            snapshot.new_values = values;
            have_snapshot = true;
          }
          catch (const std::exception &) { ++telemetry_.counterfactual_snapshot_failure_count; }
        }
      }
      factors.add(visual_factor);
    }
    const auto isam_start = std::chrono::steady_clock::now();
    isam_.update(factors, values);
    const double isam_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - isam_start).count();
    estimate_ = isam_.calculateEstimate();
    if (reobservation)
    {
      diagnostic.factor_accepted = true;
      diagnostic.isam_update_latency_ms = isam_ms;
      try
      {
        diagnostic.keypose_after = eigenPose(estimate_.at<gtsam::Pose3>(x));
        diagnostic.landmark_after = eigenPose(estimate_.at<gtsam::Pose3>(l));
        diagnostic.post = residual(*visual_factor, estimate_);
        diagnostic.post_full_graph_error = isam_.getFactorsUnsafe().error(estimate_);
        if (diagnostic.pre_valid)
        {
          diagnostic.keypose_delta_translation_m =
              (diagnostic.keypose_after.translation() -
               diagnostic.keypose_before.translation()).norm();
          diagnostic.landmark_delta_translation_m =
              (diagnostic.landmark_after.translation() -
               diagnostic.landmark_before.translation()).norm();
          diagnostic.keypose_delta_rotation_deg = Eigen::AngleAxisd(
              diagnostic.keypose_before.linear().transpose() *
              diagnostic.keypose_after.linear()).angle() * 180.0 / M_PI;
          diagnostic.landmark_delta_rotation_deg = Eigen::AngleAxisd(
              diagnostic.landmark_before.linear().transpose() *
              diagnostic.landmark_after.linear()).angle() * 180.0 / M_PI;
          diagnostic.large_correction =
              std::max(diagnostic.keypose_delta_translation_m,
                       diagnostic.landmark_delta_translation_m) >= 2.0 ||
              std::max(diagnostic.keypose_delta_rotation_deg,
                       diagnostic.landmark_delta_rotation_deg) >= 30.0;
        }
        diagnostic.post_valid = std::isfinite(diagnostic.post_full_graph_error);
        if (!diagnostic.post_valid) throw std::runtime_error("NONFINITE_POST_GRAPH_ERROR");
      }
      catch (const std::exception &e) { diagnostic.diagnostic_error += e.what(); }
      if (!diagnostic.pre_valid || !diagnostic.post_valid)
        ++telemetry_.reobservation_diagnostic_failure_count;
      reobservation_diagnostics_.push_back(diagnostic);
      ++telemetry_.cross_episode_reobservation_count;
      if (have_snapshot)
      {
        try
        {
          snapshot.runtime_post = estimate_;
          counterfactual_snapshots_.push_back(std::move(snapshot));
        }
        catch (const std::exception &) { ++telemetry_.counterfactual_snapshot_failure_count; }
      }
    }
    keypose_ids_.insert(keypose.keypose_id);
    if (new_keypose)
    {
      if (motion)
      {
        motion_edges_.emplace(motion->from_keypose_id,
                              motion->to_keypose_id);
        ++telemetry_.motion_factor_count;
        telemetry_.motion_covariance_source = sourceName(motion->covariance_source);
      }
      else ++telemetry_.gauge_prior_count;
    }
    if (add_visual)
    {
      landmark_ids_.insert(input.observation.landmark_id);
      visual_edges_.insert(visual_id);
      ++telemetry_.visual_factor_count;
      landmark_factor_episodes_[landmark_id] = {episode_id, input.observation.timestamp};
    }
    else if (history != landmark_factor_episodes_.end() &&
             history->second.first == episode_id)
      history->second.second = input.observation.timestamp;
    telemetry_.graph_initialized = true;
    telemetry_.keypose_variable_count = keypose_ids_.size();
    telemetry_.landmark_variable_count = landmark_ids_.size();
    ++telemetry_.isam_update_count;
    telemetry_.last_update_time = input.observation.timestamp;
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    telemetry_.update_latency_ms = ms;
    telemetry_.max_update_latency_ms =
        std::max(telemetry_.max_update_latency_ms, ms);
    return true;
  }
  catch (const std::exception &e)
  {
    ++telemetry_.isam_exception_count;
    telemetry_.degraded = true;
    telemetry_.latest_graph_error = e.what();
    failed_ = true; // ISAM2 may be partially mutated: never retry blindly.
    return false;
  }
}

bool SparseLandmarkShadowGraph::estimateKeyPose(
    std::uint64_t id, Eigen::Isometry3d *pose) const
{
  if (!pose || !keypose_ids_.count(id)) return false;
  *pose = eigenPose(estimate_.at<gtsam::Pose3>(keyposeKey(id)));
  return true;
}

bool SparseLandmarkShadowGraph::estimateLandmark(
    int id, Eigen::Isometry3d *pose) const
{
  if (!pose || !landmark_ids_.count(id)) return false;
  *pose = eigenPose(estimate_.at<gtsam::Pose3>(landmarkKey(id)));
  return true;
}

void SparseLandmarkShadowGraph::writeReobservationDiagnosticsCsv(std::ostream &out) const
{
  out << "timestamp,landmark_id,episode_id,previous_episode_id,keypose_id,observation_id,"
         "episode_gap_s,pre_keypose_source,pre_valid,post_valid,factor_accepted,"
         "motion_covariance_source,isam_update_latency_ms,diagnostic_error";
  for (const auto *stage : {"pre", "post"})
  {
    for (const auto *kind : {"logmap", "unwhitened", "whitened"})
      for (const auto *axis : {"rx", "ry", "rz", "tx", "ty", "tz"})
        out << ',' << stage << '_' << kind << '_' << axis;
    for (const auto *metric : {"rotation_residual_deg", "translation_residual_m",
         "whitened_error", "whitened_squared_norm", "factor_error", "logmap_factor_difference"})
      out << ',' << stage << '_' << metric;
  }
  out << ",keypose_delta_translation_m,keypose_delta_rotation_deg,"
         "landmark_delta_translation_m,landmark_delta_rotation_deg,"
         "pre_existing_graph_error,pre_pending_nonvisual_error,"
         "pre_existing_plus_candidate_error,pre_augmented_graph_error,"
         "post_full_graph_error,SHADOW_REOBSERVATION_LARGE_CORRECTION";
  for (const auto *name : {"measurement", "keypose_before", "keypose_after",
                          "landmark_before", "landmark_after"})
    for (const auto *component : {"x", "y", "z", "qx", "qy", "qz", "qw"})
      out << ',' << name << '_' << component;
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < 6; ++col)
      out << ",visual_covariance_right_" << row << col;
  out << '\n' << std::setprecision(17);
  for (const auto &d : reobservation_diagnostics_)
  {
    std::string error = d.diagnostic_error;
    for (auto &c : error) if (c == ',' || c == '\r' || c == '\n') c = '|';
    out << d.timestamp << ',' << d.landmark_id << ',' << d.episode_id << ','
        << d.previous_episode_id << ',' << d.keypose_id << ',' << d.observation_id
        << ',' << d.episode_gap_s << ','
        << (d.pre_keypose_in_isam ? "CURRENT_ISAM_ESTIMATE" : "NEW_RAW_VALUES_SEED")
        << ',' << d.pre_valid << ',' << d.post_valid << ',' << d.factor_accepted
        << ',' << d.motion_covariance_source << ',' << d.isam_update_latency_ms
        << ',' << error;
    for (const auto *r : {&d.pre, &d.post})
    {
      for (const auto *v : {&r->logmap, &r->unwhitened, &r->whitened})
        for (int i = 0; i < 6; ++i) out << ',' << (*v)(i);
      out << ',' << r->rotation_deg << ',' << r->translation_m << ','
          << std::sqrt(r->whitened_squared_norm) << ',' << r->whitened_squared_norm
          << ',' << r->factor_error << ',' << r->logmap_factor_difference;
    }
    out << ',' << d.keypose_delta_translation_m << ',' << d.keypose_delta_rotation_deg
        << ',' << d.landmark_delta_translation_m << ',' << d.landmark_delta_rotation_deg
        << ',' << d.pre_existing_graph_error << ',' << d.pre_pending_nonvisual_error
        << ',' << d.pre_existing_plus_candidate_error << ',' << d.pre_augmented_graph_error
        << ',' << d.post_full_graph_error << ',' << d.large_correction;
    for (const auto *p : {&d.measurement, &d.keypose_before, &d.keypose_after,
                         &d.landmark_before, &d.landmark_after})
    {
      const Eigen::Quaterniond q(p->linear());
      out << ',' << p->translation().x() << ',' << p->translation().y()
          << ',' << p->translation().z() << ',' << q.x() << ',' << q.y()
          << ',' << q.z() << ',' << q.w();
    }
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 6; ++col) out << ',' << d.visual_covariance_right(row, col);
    out << '\n';
  }
}

void SparseLandmarkShadowGraph::writeCounterfactualSnapshots(const std::string &directory) const
{
  for (const auto &snapshot : counterfactual_snapshots_)
  {
    const auto path = directory + "/landmark_counterfactual_obs" +
                      std::to_string(snapshot.observation_id) + ".snapshot";
    std::ifstream existing(path);
    if (existing.good()) throw std::runtime_error("snapshot already exists: " + path);
    std::ofstream out(path);
    writeCounterfactualSnapshot(out, snapshot);
    out.close();
    if (!out) throw std::runtime_error("snapshot close failed: " + path);
  }
}

} // namespace landmark
