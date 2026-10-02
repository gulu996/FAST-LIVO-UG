#include "landmark_shadow_graph.h"
#include "landmark_persistent_backend.h"

#include <gtsam/geometry/Pose3.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace
{
void require(bool ok, const char *message)
{
  if (!ok) throw std::runtime_error(message);
}

Eigen::Isometry3d pose(double x, double yaw = 0.0)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, 0.0, 0.0);
  return T;
}

landmark::GlobalLandmarkInput input(
    std::uint64_t id, double t, const Eigen::Isometry3d &raw,
    const Eigen::Isometry3d &visual, bool with_visual = true)
{
  landmark::GlobalLandmarkInput value;
  value.observation_id = id;
  value.observation.timestamp = t;
  value.local_pose_timestamp = t;
  value.observation.landmark_id = 7;
  value.observation.pose_valid = with_visual;
  value.observation.covariance_valid = with_visual;
  value.observation.T_camera_landmark = visual;
  value.observation.pose_covariance_camera =
      landmark::Matrix6d::Identity() * 0.0004;
  value.local_pose_reference = raw;
  value.local_pose_covariance = landmark::Matrix6d::Identity() * 0.01;
  value.landmark_initial_guess = raw * visual;
  value.landmark_initial_covariance = landmark::Matrix6d::Identity();
  value.camera_extrinsic_id = "synthetic";
  return value;
}

landmark::SparseKeyPose keypose(std::uint64_t id, double t,
                                const Eigen::Isometry3d &raw)
{
  landmark::SparseKeyPose value;
  value.keypose_id = id;
  value.timestamp = t;
  value.local_pose_reference = raw;
  return value;
}

landmark::SparseMotionSummary motion(
    std::uint64_t from, std::uint64_t to,
    const Eigen::Isometry3d &relative)
{
  landmark::SparseMotionSummary value;
  value.from_keypose_id = from;
  value.to_keypose_id = to;
  value.relative_pose = relative;
  value.covariance = landmark::Matrix6d::Identity() * 0.01;
  value.covariance_source = landmark::MotionCovarianceSource::Conservative;
  value.covariance_valid = true;
  return value;
}

void chainAndRandomLandmark()
{
  landmark::SparseLandmarkShadowGraph graph;
  const auto dummy = pose(0);
  for (int i = 0; i < 3; ++i)
  {
    const auto k = keypose(i + 1, i, pose(i));
    const auto in = input(i + 1, i, pose(i), dummy, false);
    const auto m = motion(i, i + 1, pose(1));
    require(graph.update(k, i ? &m : nullptr, in, 1, false),
            "motion-only chain update failed");
  }
  const auto h = graph.telemetry();
  require(h.graph_initialized && h.keypose_variable_count == 3 &&
              h.landmark_variable_count == 0 && h.gauge_prior_count == 1 &&
              h.motion_factor_count == 2 && h.visual_factor_count == 0 &&
              h.motion_covariance_source == "CONSERVATIVE",
          "basic chain variable/factor counts wrong");
  Eigen::Isometry3d x2;
  require(graph.estimateKeyPose(3, &x2) &&
              std::abs(x2.translation().x() - 2.0) < 1e-3,
          "motion-only chain estimate wrong");

  landmark::SparseLandmarkShadowGraph seen;
  auto first = input(1, 0, pose(0), pose(4));
  first.landmark_initial_guess = pose(6); // deliberately biased Values seed
  require(seen.update(keypose(1, 0, pose(0)), nullptr, first, 1),
          "first landmark observation failed");
  Eigen::Isometry3d landmark;
  require(seen.estimateLandmark(7, &landmark) &&
              std::abs(landmark.translation().x() - 4.0) < 0.02,
          "self-mapped landmark was fixed at its biased initial guess");
  for (int i = 1; i < 3; ++i)
  {
    const auto m = motion(i, i + 1, pose(1));
    require(seen.update(keypose(i + 1, i, pose(i)), &m,
                        input(i + 1, i, pose(i), pose(4 - i), i == 2),
                        1, i == 2),
            "repeated landmark chain update failed");
  }
  const auto hs = seen.telemetry();
  require(hs.keypose_variable_count == 3 && hs.landmark_variable_count == 1 &&
              hs.visual_factor_count == 2 && hs.gauge_prior_count == 1,
          "Landmark Pose3 was not shared random variable");
  std::cout << "LANDMARK_RANDOM_VARIABLE_TEST=PASS\n";
}

void loopAndDedup()
{
  landmark::SparseLandmarkShadowGraph graph(true);
  const auto landmark_world = pose(5);
  for (int i = 0; i < 4; ++i)
  {
    const auto raw = pose(1.1 * i, 0.01 * i);
    const double timestamp = i == 3 ? 30.0 : i;
    const auto k = keypose(i + 1, timestamp, raw);
    const auto visual = pose(i).inverse() * landmark_world;
    const auto in = input(i + 1, timestamp, raw, visual, i == 0 || i == 3);
    const auto m = motion(i, i + 1,
        pose(1.1 * (i - 1), 0.01 * (i - 1)).inverse() * raw);
    require(graph.update(k, i ? &m : nullptr, in, i == 3 ? 2 : 1,
                         i == 0 || i == 3), "loop chain update failed");
  }
  Eigen::Isometry3d optimized;
  require(graph.estimateKeyPose(4, &optimized) &&
              std::abs(optimized.translation().x() - 3.0) < 0.25,
          "visual closure did not reduce raw drift");
  require(graph.reobservationDiagnostics().size() == 1,
          "continuous or first observations recorded as episode transition");
  const auto &d = graph.reobservationDiagnostics().front();
  require(d.pre_valid && d.post_valid && d.factor_accepted &&
              !d.pre_keypose_in_isam && d.episode_id == 2 &&
              d.previous_episode_id == 1 && d.episode_gap_s == 28.0 &&
              d.observation_id == 4 && d.landmark_id == 7 &&
              d.pre.translation_m > 0.2 &&
              d.pre.rotation_deg > 1.0 &&
              d.post.translation_m < d.pre.translation_m &&
              d.post.rotation_deg < d.pre.rotation_deg &&
              d.post.factor_error < d.pre.factor_error &&
              d.post_full_graph_error < d.pre_augmented_graph_error &&
              std::abs(d.pre.factor_error - 0.5 * d.pre.whitened_squared_norm) < 1e-9 &&
              d.pre.logmap_factor_difference < 1e-9 &&
              d.post.logmap_factor_difference < 1e-9 &&
              d.keypose_delta_translation_m > 0.1 &&
              d.landmark_delta_translation_m > 1e-4,
          "reobservation residual/weight/chart or random-landmark diagnostics failed");
  std::cout << "SYNTHETIC_REOBSERVATION pre_translation_m=" << d.pre.translation_m
            << " post_translation_m=" << d.post.translation_m
            << " pre_rotation_deg=" << d.pre.rotation_deg
            << " post_rotation_deg=" << d.post.rotation_deg
            << " keypose_delta_m=" << d.keypose_delta_translation_m
            << " landmark_delta_m=" << d.landmark_delta_translation_m << '\n';
  std::ostringstream csv;
  require(graph.counterfactualSnapshots().size() == 1 &&
              graph.telemetry().counterfactual_snapshot_failure_count == 0,
          "read-only counterfactual capture failed");
  const auto snapshot = graph.counterfactualSnapshots().front();
  std::ostringstream serialized;
  landmark::writeCounterfactualSnapshot(serialized, snapshot);
  std::istringstream input_stream(serialized.str());
  const auto restored = landmark::readCounterfactualSnapshot(input_stream);
  gtsam::Values seed = restored.existing_values;
  seed.insert(restored.new_values);
  require(restored.existing.size() == snapshot.existing.size() &&
              restored.motion.size() == 1 && restored.visual.size() == 1 &&
              std::abs(restored.existing.error(seed) - snapshot.existing.error(seed)) < 1e-9 &&
              std::abs(restored.visual.error(restored.runtime_post) - d.post.factor_error) < 1e-9,
          "snapshot round-trip changed graph semantics");
  auto rounded = snapshot;
  const auto lk = landmark::SparseLandmarkShadowGraph::landmarkKey(7);
  const auto lp = rounded.existing_values.at<gtsam::Pose3>(lk);
  rounded.existing_values.update(lk, gtsam::Pose3(
      gtsam::Rot3(lp.rotation().matrix() * (1 + 1e-6)), lp.translation()));
  std::ostringstream rounded_text;
  landmark::writeCounterfactualSnapshot(rounded_text, rounded);
  std::istringstream rounded_input(rounded_text.str());
  const auto rounded_copy = landmark::readCounterfactualSnapshot(rounded_input);
  require((rounded_copy.existing_values.at<gtsam::Pose3>(lk).matrix() -
           rounded.existing_values.at<gtsam::Pose3>(lk).matrix()).norm() < 1e-12,
          "snapshot reader normalized real rounded calibration");
  const auto cf = landmark::optimizeCounterfactual(restored);
  Eigen::Isometry3d live_after_offline;
  require(graph.estimateKeyPose(4, &live_after_offline) &&
              live_after_offline.matrix().isApprox(optimized.matrix(), 1e-12) &&
              graph.telemetry().isam_update_count == 4,
          "offline optimizer changed live ISAM estimate or update count");
  const auto x = landmark::SparseLandmarkShadowGraph::keyposeKey(4);
  require((cf.motion_only.at<gtsam::Pose3>(x).translation() - pose(3.3, 0.03).translation()).norm() < 1e-6 &&
              (cf.motion_visual.at<gtsam::Pose3>(x).translation() -
               cf.motion_only.at<gtsam::Pose3>(x).translation()).norm() > 0.1 &&
              restored.visual.error(cf.motion_visual) < restored.visual.error(cf.motion_only) &&
              cf.motion_final_cost <= cf.motion_initial_cost && cf.visual_final_cost < cf.visual_initial_cost,
          "offline B/C isolation or visual marginal contribution failed");
  const auto scaled = landmark::scaledCounterfactualFactors(restored.existing, 2, 1);
  require(scaled[0] == restored.existing[0] &&
              (graph.counterfactualSnapshots().front().runtime_post.at<gtsam::Pose3>(x).matrix() -
               restored.runtime_post.at<gtsam::Pose3>(x).matrix()).norm() < 1e-12 &&
              graph.telemetry().cross_episode_reobservation_count == 1,
          "offline optimization changed gauge or live graph");
  bool rejected = false;
  try { std::istringstream bad("LANDMARK_COUNTERFACTUAL_V1 nan"); landmark::readCounterfactualSnapshot(bad); }
  catch (const std::exception &) { rejected = true; }
  require(rejected, "invalid offline input was not rejected");
  std::cout << "LANDMARK_COUNTERFACTUAL_ISOLATION_SELF_TEST=PASS\n";
  graph.writeReobservationDiagnosticsCsv(csv);
  const auto text = csv.str();
  require(std::count(text.begin(), text.end(), '\n') == 2 &&
              text.find("NEW_RAW_VALUES_SEED") != std::string::npos,
          "CSV must contain one header and one accepted episode event");
  const auto k = keypose(4, 30, pose(3.3, 0.03));
  const auto in = input(4, 30, pose(3.3, 0.03), pose(2));
  for (int i = 0; i < 30; ++i)
    require(graph.update(k, nullptr, in, 2), "duplicate update failed");
  const auto h = graph.telemetry();
  require(h.visual_factor_count == 2 && h.motion_factor_count == 3 &&
              h.suppressed_visual_observation_count >= 30 &&
              h.duplicate_factor_count >= 30,
          "30 Hz visual factor dedup failed");
  require(h.cross_episode_reobservation_count == 1 &&
              h.reobservation_diagnostic_failure_count == 0 &&
              graph.reobservationDiagnostics().size() == 1,
          "suppressed observations flooded reobservation records");
  const auto m = motion(4, 5, pose(3.3, 0.03).inverse() * pose(4.4, 0.04));
  const auto continuous = input(5, 31, pose(4.4, 0.04), pose(1));
  require(graph.update(keypose(5, 31, pose(4.4, 0.04)), &m, continuous, 2) &&
              graph.reobservationDiagnostics().size() == 1,
          "new keypose in the same episode became long reobservation");
  require(graph.update(keypose(5, 60, pose(4.4, 0.04)), nullptr,
                       input(6, 60, pose(4.4, 0.04), pose(1)), 3) &&
              graph.reobservationDiagnostics().size() == 2 &&
              graph.reobservationDiagnostics().back().pre_keypose_in_isam,
          "existing-keypose current-estimate diagnostic path failed");
  std::cout << "SYNTHETIC_LOOP_CONSISTENCY_TEST=PASS\n"
            << "LANDMARK_REOBSERVATION_DIAGNOSTICS_SELF_TEST=PASS\n"
            << "LANDMARK_REOBSERVATION_RANDOM_VARIABLE_BEHAVIOR=PASS\n"
            << "FACTOR_DEDUP_TEST=PASS\n";
}

void covarianceAndFailure()
{
  landmark::GlobalLandmarkInput in = input(1, 0, pose(1), pose(2, 0.4));
  in.T_body_camera = pose(0.3, -0.2);
  const Eigen::Isometry3d z = landmark::SparseLandmarkShadowGraph::visualMeasurement(in);
  require((in.local_pose_reference * z).matrix().isApprox(
              (in.local_pose_reference * in.T_body_camera *
               in.observation.T_camera_landmark).matrix(), 1e-12),
          "body-camera-landmark composition wrong");
  const auto z_g = gtsam::Pose3(gtsam::Rot3(z.linear()), z.translation());
  landmark::Matrix6d numerical = landmark::Matrix6d::Zero();
  const double step = 1e-6;
  for (int c = 0; c < 6; ++c)
  {
    auto p = in.observation.T_camera_landmark;
    if (c < 3)
      p.linear() = Eigen::AngleAxisd(step, Eigen::Vector3d::Unit(c)).toRotationMatrix() * p.linear();
    else p.translation()(c - 3) += step;
    in.observation.T_camera_landmark = p;
    const auto zp = landmark::SparseLandmarkShadowGraph::visualMeasurement(in);
    const auto zp_g = gtsam::Pose3(gtsam::Rot3(zp.linear()), zp.translation());
    numerical.col(c) = gtsam::Pose3::Logmap(z_g.between(zp_g)) / step;
    in.observation.T_camera_landmark = pose(2, 0.4);
  }
  landmark::Matrix6d expected = landmark::Matrix6d::Zero();
  const auto R = in.observation.T_camera_landmark.linear().transpose();
  expected.topLeftCorner<3, 3>() = R;
  expected.bottomRightCorner<3, 3>() = R;
  require((numerical - expected).norm() < 1e-5,
          "GTSAM Pose3 right-local tangent Jacobian mismatch");
  in.observation.pose_covariance_camera = landmark::Matrix6d::Identity();
  in.observation.pose_covariance_camera(0, 3) = 0.1;
  in.observation.pose_covariance_camera(3, 0) = 0.1;
  require(landmark::SparseLandmarkShadowGraph::visualCovarianceRight(
              in.observation).isApprox(
              numerical * in.observation.pose_covariance_camera *
              numerical.transpose(), 1e-5),
          "visual covariance frame conversion wrong");
  std::cout << "COVARIANCE_FRAME_TEST=PASS\n";

  landmark::SparseLandmarkShadowGraph graph;
  auto a = input(1, 0, pose(0), pose(2), false);
  require(graph.update(keypose(1, 0, pose(0)), nullptr, a, 1, false),
          "failure isolation setup failed");
  auto unavailable = motion(1, 2, pose(1));
  unavailable.covariance_source = landmark::MotionCovarianceSource::Unavailable;
  require(!graph.update(keypose(2, 1, pose(1)), &unavailable,
                        input(2, 1, pose(1), pose(1)), 1) &&
              graph.telemetry().motion_factor_reject_count == 1 &&
              graph.telemetry().keypose_variable_count == 1,
          "UNAVAILABLE motion entered graph");
  auto bad = input(3, 0, pose(0), pose(2));
  bad.observation.pose_covariance_camera(0, 0) =
      std::numeric_limits<double>::quiet_NaN();
  require(!graph.update(keypose(1, 0, pose(0)), nullptr, bad, 1) &&
              graph.telemetry().visual_factor_reject_count == 1 &&
              graph.telemetry().degraded &&
              !graph.telemetry().latest_graph_error.empty(),
          "graph failure was not isolated and reported");
  landmark::PersistentBackendConfig config;
  landmark::PersistentLandmarkBackend backend(config, false);
  landmark::GlobalCorrection correction;
  require(!backend.latestCorrection(&correction) && !correction.valid,
          "shadow graph acquired production correction ownership");
  std::cout << "GRAPH_FAILURE_ISOLATION_TEST=PASS\n";
}
} // namespace

int main()
{
  try
  {
    chainAndRandomLandmark();
    loopAndDedup();
    covarianceAndFailure();
    std::cout << "LANDMARK_SHADOW_GRAPH_SELF_TEST=PASS\n";
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "LANDMARK_SHADOW_GRAPH_SELF_TEST=FAIL " << e.what() << '\n';
    return 1;
  }
}
