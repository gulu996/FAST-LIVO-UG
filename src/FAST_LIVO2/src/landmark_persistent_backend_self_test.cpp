#include "landmark_persistent_backend.h"

#include <chrono>
#include <cmath>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

landmark::GlobalLandmarkInput input(std::uint64_t id, int landmark_id,
                                     double timestamp)
{
  landmark::GlobalLandmarkInput value;
  value.observation_id = id;
  value.observation.landmark_id = landmark_id;
  value.observation.timestamp = timestamp;
  value.observation.pose_valid = true;
  value.observation.covariance_valid = true;
  value.observation.pose_covariance_camera.setIdentity();
  value.observation.T_camera_landmark.translation().z() = 2.0;
  value.local_pose_timestamp = timestamp;
  value.local_pose_covariance.setIdentity();
  value.local_pose_reference.translation().x() = timestamp * 0.01;
  value.landmark_initial_guess.translation().z() = 2.0;
  value.landmark_initial_covariance.setIdentity();
  value.camera_extrinsic_id = "self_test_calibration";
  return value;
}

void checkQueueAndIdentity()
{
  landmark::PersistentBackendConfig config;
  config.queue_capacity = 2;
  landmark::PersistentLandmarkBackend backend(config, false);
  std::string reason;
  require(backend.submit(input(1, 7, 0.0), &reason), "first enqueue failed");
  require(!backend.submit(input(1, 7, 0.0), &reason) &&
              reason == "DUPLICATE_OBSERVATION_ID", "duplicate ID accepted");
  require(backend.submit(input(2, 7, 0.03), &reason), "second enqueue failed");
  require(!backend.submit(input(3, 7, 0.06), &reason) &&
              reason == "QUEUE_OVERFLOW_REJECT_NEWEST", "overflow was silent");
  require(!backend.submit(input(4, 7, 0.01), &reason) &&
              reason == "OUT_OF_ORDER_TIMESTAMP", "out-of-order accepted");
  require(backend.processOne(), "first accepted observation not processed");
  require(backend.processOne(), "second accepted observation not processed");
  require(!backend.processOne(), "empty queue processed an observation");
  const auto count = backend.counters();
  require(count.submitted == 5 && count.accepted == 2 &&
              count.duplicate == 1 && count.overflow == 1 &&
              count.out_of_order == 1 && count.processed == 2,
          "queue counters mismatch");
  backend.shutdown();
  require(!backend.submit(input(5, 7, 0.1), &reason) &&
              reason == "BACKEND_SHUTDOWN", "shutdown accepted input");
  const auto telemetry = backend.telemetry();
  require(telemetry.shutdown_requested && telemetry.drain_started &&
              telemetry.drain_completed && telemetry.join_completed &&
              telemetry.queue_peak_size == 2 &&
              telemetry.queue_size_at_shutdown == 0 &&
              telemetry.queue_current_size == 0 &&
              telemetry.rejected == telemetry.counters.submitted -
                  telemetry.counters.accepted,
          "no-worker lifecycle telemetry or reject accounting failed");
}

void checkRegistryEpisodesAndKeyposes()
{
  landmark::PersistentBackendConfig config;
  config.queue_capacity = 256;
  landmark::PersistentLandmarkBackend backend(config, false);
  std::string reason;
  require(backend.submit(input(1, 7, 0.0), &reason), "t0 submit failed");
  require(backend.submit(input(2, 7, 0.03), &reason), "t0.03 submit failed");
  require(backend.submit(input(3, 7, 0.06), &reason), "t0.06 submit failed");
  while (backend.processOne()) {}
  landmark::PersistentLandmark first;
  require(backend.lookupLandmark(7, &first) &&
              first.observation_count == 3 &&
              first.first_seen_timestamp == 0.0 &&
              first.episode_first_timestamp == 0.0 &&
              first.episode_last_timestamp == 0.06 &&
              first.status == landmark::PersistentLandmarkStatus::Active &&
              backend.keyposeCount() == 1,
          "continuous episode did not coalesce");
  landmark::SparseKeyPose initial_keypose;
  require(backend.lookupKeyPose(1, &initial_keypose) &&
              initial_keypose.source_observation_ids.size() == 3 &&
              initial_keypose.trigger_reason == "LANDMARK_FIRST_OBSERVATION",
          "first keypose lost observation IDs");

  require(backend.submit(input(4, 7, 30.0), &reason), "t30 submit failed");
  require(backend.submit(input(5, 8, 30.01), &reason), "landmark8 submit failed");
  while (backend.processOne()) {}
  landmark::PersistentLandmark repeated, independent;
  require(backend.lookupLandmark(7, &repeated) &&
              backend.lookupLandmark(8, &independent) &&
              repeated.episode_id != first.episode_id &&
              repeated.episode_first_timestamp == 30.0 &&
              repeated.observation_count == 4 &&
              repeated.first_seen_timestamp == 0.0 &&
              independent.observation_count == 1 &&
              independent.first_seen_timestamp == 30.01 &&
              repeated.has_initial_guess && repeated.has_optimized_estimate &&
              independent.has_initial_guess && independent.has_optimized_estimate &&
              backend.keyposeCount() == 3,
          "new episode, forced keypose, or independent identity failed");
  landmark::SparseKeyPose return_keypose;
  require(backend.lookupKeyPose(2, &return_keypose) &&
              return_keypose.trigger_reason == "LANDMARK_REOBSERVATION",
          "long-gap re-observation did not force keypose");
  const auto motions = backend.motionSummaries();
  require(motions.size() == 2 &&
              motions.front().covariance_source ==
                  landmark::MotionCovarianceSource::Conservative &&
              motions.front().covariance_valid &&
              motions.front().covariance.allFinite() &&
              motions.front().covariance.diagonal().minCoeff() > 0.0 &&
              motions.front().relative_pose.matrix().allFinite(),
          "relative drift-envelope provenance is missing");
  landmark::GlobalCorrection correction;
  require(!backend.latestCorrection(&correction) && !correction.valid,
          "shadow backend emitted a correction");
}

void checkContinuousThirtyHertz()
{
  landmark::PersistentBackendConfig config;
  config.queue_capacity = 256;
  landmark::PersistentLandmarkBackend backend(config, false);
  std::string reason;
  for (std::uint64_t i = 0; i < 100; ++i)
    require(backend.submit(input(i + 1, 7, i / 30.0), &reason),
            "30 Hz submit failed");
  while (backend.processOne()) {}
  require(backend.keyposeCount() <= 3,
          "continuous 30 Hz visibility created dense keyposes");
  landmark::PersistentLandmark value;
  require(backend.lookupLandmark(7, &value) && value.observation_count == 100,
          "coalescing dropped observations");
}

void checkWorker()
{
  landmark::PersistentBackendConfig config;
  landmark::PersistentLandmarkBackend backend(config);
  std::string reason;
  require(backend.submit(input(1, 7, 0.0), &reason), "worker submit failed");
  for (int retry = 0; retry < 100 && !backend.workerStarted(); ++retry)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  require(backend.workerStarted() && backend.workerAlive(),
          "worker did not become alive");
  const auto before = backend.telemetry();
  require(before.worker_started && before.worker_start_count == 1 &&
              before.queue_peak_size > 0 && !before.shutdown_requested,
          "worker start or queue peak event missing");
  backend.shutdown();
  backend.shutdown(); // repeated shutdown must not double join or recount
  const auto after = backend.telemetry();
  require(backend.counters().processed == 1 && backend.queued() == 0 &&
              !backend.workerAlive() && after.shutdown_requested &&
              after.drain_started && after.drain_completed &&
              after.worker_stopped && after.worker_stop_count == 1 &&
              after.worker_start_count == 1 && after.join_completed &&
              after.counters.accepted == after.counters.processed &&
              after.queue_current_size == 0,
          "worker shutdown did not drain accepted observations");
  landmark::PersistentLandmarkBackend empty(config);
  for (int retry = 0; retry < 100 && !empty.workerStarted(); ++retry)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  empty.shutdown();
  const auto empty_final = empty.telemetry();
  require(empty_final.worker_started && empty_final.worker_stopped &&
              empty_final.drain_started && empty_final.drain_completed &&
              empty_final.queue_peak_size == 0 &&
              empty_final.queue_size_at_shutdown == 0 &&
              empty_final.counters.processed == 0,
          "empty-queue shutdown was not recorded as a completed drain");
}

Eigen::Isometry3d perturbFastLivoPose(
    const Eigen::Isometry3d &pose, const Eigen::Matrix<double, 6, 1> &delta)
{
  Eigen::Isometry3d result = pose;
  result.linear() *= Eigen::AngleAxisd(delta.head<3>().norm(),
      delta.head<3>().normalized()).toRotationMatrix();
  result.translation() += delta.tail<3>();
  return result;
}

Eigen::Matrix<double, 6, 1> relativeTangent(
    const Eigen::Isometry3d &first, const Eigen::Isometry3d &second,
    const Eigen::Isometry3d &nominal_relative)
{
  const Eigen::Isometry3d relative = first.inverse() * second;
  Eigen::Matrix<double, 6, 1> tangent;
  // Right-local error at the nonidentity between-pose, matching Pose3 noise.
  const Eigen::AngleAxisd rotation(
      nominal_relative.linear().transpose() * relative.linear());
  tangent.head<3>() = rotation.angle() * rotation.axis();
  tangent.tail<3>() = nominal_relative.linear().transpose() *
      (relative.translation() - nominal_relative.translation());
  return tangent;
}

void checkPoseConventionAndJacobian()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(0.7,
      Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  pose.translation() = Eigen::Vector3d(4, -2, 1);
  Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
  motion.linear() = Eigen::AngleAxisd(0.3,
      Eigen::Vector3d(-1, 3, 2).normalized()).toRotationMatrix();
  motion.translation() = Eigen::Vector3d(1, 2, -0.5);
  const Eigen::Isometry3d next = pose * motion;
  Eigen::Matrix<double, 6, 12> analytic = Eigen::Matrix<double, 6, 12>::Zero();
  analytic.block<3, 3>(0, 0) = -motion.linear().transpose();
  analytic.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d skew;
  skew << 0, -motion.translation().z(), motion.translation().y(),
          motion.translation().z(), 0, -motion.translation().x(),
          -motion.translation().y(), motion.translation().x(), 0;
  analytic.block<3, 3>(3, 0) = motion.linear().transpose() * skew;
  analytic.block<3, 3>(3, 3) = -next.linear().transpose();
  analytic.block<3, 3>(3, 9) = next.linear().transpose();
  Eigen::Matrix<double, 6, 12> numerical;
  constexpr double epsilon = 1e-6;
  for (int col = 0; col < 12; ++col)
  {
    Eigen::Matrix<double, 6, 1> plus = Eigen::Matrix<double, 6, 1>::Zero();
    plus[col % 6] = epsilon;
    Eigen::Matrix<double, 6, 1> minus = -plus;
    const auto positive = col < 6 ?
        relativeTangent(perturbFastLivoPose(pose, plus), next, motion) :
        relativeTangent(pose, perturbFastLivoPose(next, plus), motion);
    const auto negative = col < 6 ?
        relativeTangent(perturbFastLivoPose(pose, minus), next, motion) :
        relativeTangent(pose, perturbFastLivoPose(next, minus), motion);
    numerical.col(col) = (positive - negative) / (2 * epsilon);
  }
  require((numerical - analytic).cwiseAbs().maxCoeff() < 1e-8,
          "FAST-LIVO relative right-tangent Jacobian mismatch");
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Identity();
  covariance.block<3, 3>(3, 3) =
      (Eigen::Vector3d(1, 2, 3)).asDiagonal();
  const auto converted = landmark::fastLivoPoseCovarianceToGtsamRight(
      covariance, pose.linear());
  require((converted.block<3, 3>(3, 3) -
           pose.linear().transpose() * covariance.block<3, 3>(3, 3) *
           pose.linear()).norm() < 1e-12,
          "world-to-right-local translation covariance mismatch");
}

void checkMotionEnvelope()
{
  landmark::ConservativeMotionUncertaintyConfig config;
  Eigen::Isometry3d short_motion = Eigen::Isometry3d::Identity();
  short_motion.translation().x() = 1.0;
  Eigen::Isometry3d long_motion = short_motion;
  long_motion.translation().x() = 5.0;
  Eigen::Matrix<double, 6, 6> short_cov, long_cov, weak_cov;
  require(landmark::conservativeMotionCovariance(short_motion, 1.0, false,
              config, &short_cov) &&
          landmark::conservativeMotionCovariance(long_motion, 5.0, false,
              config, &long_cov) &&
          landmark::conservativeMotionCovariance(long_motion, 5.0, true,
              config, &weak_cov), "motion envelope rejected valid input");
  require((long_cov.diagonal() - short_cov.diagonal()).minCoeff() >= 0.0 &&
          (weak_cov.diagonal() - long_cov.diagonal()).minCoeff() >= 0.0,
          "longer or weak-geometry segment became more confident");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(weak_cov);
  require(eigen.info() == Eigen::Success &&
              eigen.eigenvalues().minCoeff() > 0.0 && weak_cov.allFinite(),
          "motion envelope is not finite positive definite");
  require(!landmark::conservativeMotionCovariance(long_motion, -1.0, false,
              config, &weak_cov), "negative time accepted");
}

void checkThousandObservationStress()
{
  landmark::PersistentBackendConfig config;
  config.queue_capacity = 1200;
  landmark::PersistentLandmarkBackend backend(config, false);
  std::string reason;
  for (std::uint64_t i = 0; i < 1000; ++i)
  {
    const double timestamp = i < 500 ? i / 30.0 : 40.0 + (i - 500) / 30.0;
    require(backend.submit(input(i + 1, 7 + i % 5, timestamp), &reason),
            "1000-observation injection failed");
  }
  while (backend.processOne()) {}
  require(backend.counters().processed == 1000 && backend.keyposeCount() < 100,
          "stress lost observations or created dense keyposes");
  for (int id = 7; id < 12; ++id)
  {
    landmark::PersistentLandmark value;
    require(backend.lookupLandmark(id, &value) &&
                value.observation_count == 200 && value.episode_id > 0 &&
                value.episode_first_timestamp >= 40.0,
            "multi-landmark long-gap episode failed");
  }
  config.queue_capacity = 16;
  landmark::PersistentLandmarkBackend saturated(config, false);
  for (std::uint64_t i = 0; i < 1000; ++i)
    saturated.submit(input(i + 1, 7, i / 30.0), &reason);
  require(saturated.queued() == 16 && saturated.counters().overflow == 984,
          "bounded queue overflow stress failed");
  saturated.shutdown();
  const auto saturated_final = saturated.telemetry();
  require(saturated.queued() == 0 && saturated.counters().processed == 16 &&
              saturated_final.queue_peak_size == 16 &&
              saturated_final.queue_size_at_shutdown == 16 &&
              saturated_final.drain_completed && saturated_final.rejected == 984,
          "shutdown did not drain saturated queue");

  landmark::PersistentLandmarkBackend threaded(config);
  for (std::uint64_t i = 0; i < 1000; ++i)
    threaded.submit(input(i + 1, 7 + i % 5, i / 30.0), &reason);
  threaded.shutdown();
  const auto count = threaded.counters();
  const auto threaded_final = threaded.telemetry();
  require(threaded.workerStarted() && !threaded.workerAlive() &&
              threaded.queued() == 0 && count.accepted > 0 &&
              count.accepted + count.overflow == 1000 &&
              count.processed == count.accepted &&
              threaded_final.worker_start_count == 1 &&
              threaded_final.worker_stop_count == 1 &&
              threaded_final.drain_completed && threaded_final.join_completed &&
              threaded_final.rejected == count.overflow,
          "threaded producer/consumer stress lost accepted input or deadlocked");
}
} // namespace

int main()
{
  try
  {
    checkQueueAndIdentity();
    checkRegistryEpisodesAndKeyposes();
    checkContinuousThirtyHertz();
    checkWorker();
    checkPoseConventionAndJacobian();
    checkMotionEnvelope();
    checkThousandObservationStress();
    std::cout << "L3A_QUEUE_ID_DUPLICATE_OVERFLOW_ORDER=PASS\n"
              << "L3A_REGISTRY_INITIAL_GUESS_ONLY=PASS\n"
              << "L3A_EPISODE_AND_REOBSERVATION=PASS\n"
              << "L3A_KEYPOSE_COALESCING_30HZ=PASS\n"
              << "L3A_MOTION_COVARIANCE_PROVENANCE=PASS\n"
              << "L3A_SHADOW_CORRECTION=PASS\n"
              << "L3A_WORKER_SHUTDOWN_DRAIN=PASS\n";
    std::cout << "RELATIVE_POSE_JACOBIAN_TEST=PASS\n"
              << "CONSERVATIVE_MOTION_COVARIANCE_TEST=PASS\n"
              << "SYNTHETIC_BACKEND_STRESS=PASS\n";
    std::cout << "BACKEND_LIFECYCLE_TELEMETRY_SELF_TEST=PASS\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "landmark_persistent_backend_self_test: " << error.what() << '\n';
    return 1;
  }
}
