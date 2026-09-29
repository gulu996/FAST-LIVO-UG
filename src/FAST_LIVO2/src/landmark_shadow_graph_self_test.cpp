#include "landmark_shadow_graph.h"
#include "landmark_persistent_backend.h"

#include <gtsam/geometry/Pose3.h>

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>
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
  landmark::SparseLandmarkShadowGraph graph;
  const auto landmark_world = pose(5);
  for (int i = 0; i < 4; ++i)
  {
    const auto raw = pose(1.1 * i);
    const auto k = keypose(i + 1, i, raw);
    const auto visual = pose(i).inverse() * landmark_world;
    const auto in = input(i + 1, i, raw, visual, i == 0 || i == 3);
    const auto m = motion(i, i + 1, pose(1.1));
    require(graph.update(k, i ? &m : nullptr, in, 1,
                         i == 0 || i == 3), "loop chain update failed");
  }
  Eigen::Isometry3d optimized;
  require(graph.estimateKeyPose(4, &optimized) &&
              std::abs(optimized.translation().x() - 3.0) < 0.25,
          "visual closure did not reduce raw drift");
  const auto k = keypose(4, 3, pose(3.3));
  const auto in = input(4, 3, pose(3.3), pose(2));
  for (int i = 0; i < 30; ++i)
    require(graph.update(k, nullptr, in, 1), "duplicate update failed");
  const auto h = graph.telemetry();
  require(h.visual_factor_count == 2 && h.motion_factor_count == 3 &&
              h.suppressed_visual_observation_count >= 30 &&
              h.duplicate_factor_count >= 30,
          "30 Hz visual factor dedup failed");
  std::cout << "SYNTHETIC_LOOP_CONSISTENCY_TEST=PASS\n"
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
