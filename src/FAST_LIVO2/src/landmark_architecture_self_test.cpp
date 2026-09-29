#include "landmark_architecture.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

void checkFusionModes()
{
  landmark::LandmarkObservation observation;
  observation.landmark_id = 17;
  const landmark::LandmarkObservationRouter::Batch batch{observation};
  int observe_calls = 0;
  int legacy_calls = 0;
  int global_calls = 0;
  const auto observe = [&](const auto &) { ++observe_calls; };
  const auto legacy = [&](const auto &) { ++legacy_calls; };
  const auto global = [&](const auto &) { ++global_calls; };

  landmark::LandmarkObservationRouter router;
  auto result = router.route(batch, observe, legacy, global);
  require(result.accepted && result.primary_consumer_count == 1 &&
              observe_calls == 1 && legacy_calls == 0 && global_calls == 0,
          "observe_only did not isolate the diagnostic consumer");

  router.setMode(landmark::FusionMode::LegacyEsikf);
  result = router.route(batch, observe, legacy, global);
  require(result.accepted && result.primary_consumer_count == 1 &&
              observe_calls == 1 && legacy_calls == 1 && global_calls == 0,
          "legacy_esikf did not isolate the legacy consumer");

  router.setMode(landmark::FusionMode::GlobalBackend);
  result = router.route(batch, observe, legacy, {});
  require(!result.accepted && result.primary_consumer_count == 0 &&
              result.reason == "GLOBAL_BACKEND_NOT_IMPLEMENTED" &&
              legacy_calls == 1,
          "missing global backend silently fell back to legacy ESIKF");
  result = router.route(batch, observe, legacy, global);
  require(result.accepted && result.primary_consumer_count == 1 &&
              global_calls == 1 && legacy_calls == 1,
          "global_backend did not isolate the global consumer");

  landmark::FusionMode parsed;
  require(landmark::parseFusionMode("observe_only", &parsed) &&
              parsed == landmark::FusionMode::ObserveOnly,
          "observe_only parse failed");
  require(landmark::parseFusionMode("legacy_esikf", &parsed) &&
              parsed == landmark::FusionMode::LegacyEsikf,
          "legacy_esikf parse failed");
  require(landmark::parseFusionMode("global_backend", &parsed) &&
              parsed == landmark::FusionMode::GlobalBackend,
          "global_backend parse failed");
  require(!landmark::parseFusionMode("legacy_esikf+global_backend", &parsed),
          "conflicting fusion mode was accepted");
}

void checkPersistentModels()
{
  landmark::PersistentLandmark value;
  value.landmark_id = 8;
  value.initial_guess.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  value.optimized_estimate.translation() = Eigen::Vector3d(4.0, 5.0, 6.0);
  value.has_optimized_estimate = true;
  require((value.initial_guess.translation() -
           value.optimized_estimate.translation()).norm() > 1.0,
          "initial guess and optimized estimate were aliased");

  static_assert(sizeof(landmark::SparseKeyPose) < 2048,
                "SparseKeyPose unexpectedly contains a large payload");
  landmark::SparseKeyPose keypose;
  require(keypose.source_observation_ids.empty() &&
              keypose.associated_landmark_ids.empty(),
          "SparseKeyPose should contain IDs, not image/cloud payloads");
}

void checkSparseKeyPosePolicy()
{
  landmark::SparseKeyPosePolicy policy;
  Eigen::Isometry3d previous = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
  require(!landmark::shouldCreateSparseKeyPose(
              previous, 1.0, candidate, 1.1, false, false, policy),
          "stationary pose unexpectedly created a keypose");
  require(landmark::shouldCreateSparseKeyPose(
              previous, 1.0, candidate, 1.1, true, false, policy),
          "first landmark observation did not trigger a keypose");
  require(landmark::shouldCreateSparseKeyPose(
              previous, 1.0, candidate, 1.1, false, true, policy),
          "landmark re-observation did not trigger a keypose");
  candidate.translation().x() = policy.translation_threshold_m;
  require(landmark::shouldCreateSparseKeyPose(
              previous, 1.0, candidate, 1.1, false, false, policy),
          "translation threshold did not trigger a keypose");
}

} // namespace

int main()
{
  try
  {
    checkFusionModes();
    checkPersistentModels();
    checkSparseKeyPosePolicy();
    std::cout << "LANDMARK_FUSION_MODE_ROUTING=PASS\n"
              << "LANDMARK_SINGLE_PRIMARY_CONSUMER=PASS\n"
              << "LANDMARK_GLOBAL_NO_LEGACY_FALLBACK=PASS\n"
              << "PERSISTENT_LANDMARK_SEMANTICS=PASS\n"
              << "SPARSE_KEYPOSE_PAYLOAD=PASS\n"
              << "SPARSE_KEYPOSE_POLICY=PASS\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "landmark_architecture_self_test: " << error.what() << '\n';
    return 1;
  }
}
