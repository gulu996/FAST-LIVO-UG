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
  if (!new_keypose && !add_visual) return true;

  const auto start = std::chrono::steady_clock::now();
  try
  {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
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
      factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
          x, l, pose3(measurement),
          gtsam::noiseModel::Gaussian::Covariance(
              visualCovarianceRight(input.observation))));
    }
    isam_.update(factors, values);
    estimate_ = isam_.calculateEstimate();
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
    }
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

} // namespace landmark
