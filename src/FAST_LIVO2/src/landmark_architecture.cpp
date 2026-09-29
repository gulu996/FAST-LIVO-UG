#include "landmark_architecture.h"

#include <algorithm>
#include <cmath>

namespace landmark
{

const char *fusionModeName(FusionMode mode)
{
  switch (mode)
  {
    case FusionMode::ObserveOnly: return "observe_only";
    case FusionMode::LegacyEsikf: return "legacy_esikf";
    case FusionMode::GlobalBackend: return "global_backend";
  }
  return "invalid";
}

bool parseFusionMode(const std::string &name, FusionMode *mode)
{
  if (!mode) return false;
  if (name == "observe_only")
    *mode = FusionMode::ObserveOnly;
  else if (name == "legacy_esikf")
    *mode = FusionMode::LegacyEsikf;
  else if (name == "global_backend")
    *mode = FusionMode::GlobalBackend;
  else
    return false;
  return true;
}

bool shouldCreateSparseKeyPose(const Eigen::Isometry3d &previous_local_pose,
                               double previous_timestamp,
                               const Eigen::Isometry3d &candidate_local_pose,
                               double candidate_timestamp,
                               bool first_landmark_observation,
                               bool landmark_reobservation,
                               const SparseKeyPosePolicy &policy)
{
  if (!previous_local_pose.matrix().allFinite() ||
      !candidate_local_pose.matrix().allFinite() ||
      !std::isfinite(previous_timestamp) ||
      !std::isfinite(candidate_timestamp) ||
      candidate_timestamp <= previous_timestamp ||
      !std::isfinite(policy.translation_threshold_m) ||
      !std::isfinite(policy.rotation_threshold_deg) ||
      !std::isfinite(policy.maximum_interval_s) ||
      policy.translation_threshold_m <= 0.0 ||
      policy.rotation_threshold_deg <= 0.0 ||
      policy.maximum_interval_s <= 0.0)
    return false;

  if (first_landmark_observation || landmark_reobservation) return true;

  const Eigen::Isometry3d relative =
      previous_local_pose.inverse() * candidate_local_pose;
  const double translation_m = relative.translation().norm();
  const double rotation_deg =
      Eigen::AngleAxisd(relative.linear()).angle() * 180.0 / M_PI;
  return translation_m >= policy.translation_threshold_m ||
      rotation_deg >= policy.rotation_threshold_deg ||
      candidate_timestamp - previous_timestamp >= policy.maximum_interval_s;
}

bool validConservativeMotionConfig(const ConservativeMotionUncertaintyConfig &c)
{
  return std::isfinite(c.rotation_floor_rad) && c.rotation_floor_rad > 0 &&
      std::isfinite(c.rotation_per_second_rad) && c.rotation_per_second_rad >= 0 &&
      std::isfinite(c.rotation_per_meter_rad) && c.rotation_per_meter_rad >= 0 &&
      std::isfinite(c.rotation_ceiling_rad) && c.rotation_ceiling_rad >= c.rotation_floor_rad &&
      std::isfinite(c.translation_floor_m) && c.translation_floor_m > 0 &&
      std::isfinite(c.translation_per_second_m) && c.translation_per_second_m >= 0 &&
      std::isfinite(c.translation_per_meter_m) && c.translation_per_meter_m >= 0 &&
      std::isfinite(c.translation_per_radian_m) && c.translation_per_radian_m >= 0 &&
      std::isfinite(c.translation_ceiling_m) && c.translation_ceiling_m >= c.translation_floor_m &&
      std::isfinite(c.weak_geometry_multiplier) && c.weak_geometry_multiplier >= 1.0;
}

bool conservativeMotionCovariance(const Eigen::Isometry3d &relative_pose,
                                  double duration_s, bool weak_geometry,
                                  const ConservativeMotionUncertaintyConfig &c,
                                  Eigen::Matrix<double, 6, 6> *covariance)
{
  if (!covariance || !validConservativeMotionConfig(c) ||
      !relative_pose.matrix().allFinite() || !std::isfinite(duration_s) ||
      duration_s < 0.0) return false;
  const double distance_m = relative_pose.translation().norm();
  const double angle_rad = Eigen::AngleAxisd(relative_pose.linear()).angle();
  if (!std::isfinite(distance_m) || !std::isfinite(angle_rad)) return false;
  const double multiplier = weak_geometry ? c.weak_geometry_multiplier : 1.0;
  // ponytail: diagonal drift envelope is intentionally simple; replace it with
  // cross-time covariance only after the ESIKF transition/update chain is audited.
  const double rotation_sigma = std::min(c.rotation_ceiling_rad,
      multiplier * (c.rotation_floor_rad + c.rotation_per_second_rad * duration_s +
                    c.rotation_per_meter_rad * distance_m));
  const double translation_sigma = std::min(c.translation_ceiling_m,
      multiplier * (c.translation_floor_m + c.translation_per_second_m * duration_s +
                    c.translation_per_meter_m * distance_m +
                    c.translation_per_radian_m * angle_rad));
  if (!std::isfinite(rotation_sigma) || !std::isfinite(translation_sigma) ||
      rotation_sigma <= 0 || translation_sigma <= 0) return false;
  covariance->setZero();
  covariance->diagonal().head<3>().setConstant(rotation_sigma * rotation_sigma);
  covariance->diagonal().tail<3>().setConstant(translation_sigma * translation_sigma);
  return covariance->allFinite();
}

Eigen::Matrix<double, 6, 6> fastLivoPoseCovarianceToGtsamRight(
    const Eigen::Matrix<double, 6, 6> &covariance,
    const Eigen::Matrix3d &world_R_body)
{
  Eigen::Matrix<double, 6, 6> transform =
      Eigen::Matrix<double, 6, 6>::Identity();
  transform.block<3, 3>(3, 3) = world_R_body.transpose();
  return transform * covariance * transform.transpose();
}

RouteResult LandmarkObservationRouter::route(
    const Batch &observations,
    const Consumer &observe_only_consumer,
    const Consumer &legacy_esikf_consumer,
    const Consumer &global_backend_consumer) const
{
  RouteResult result;
  result.mode = mode_;
  if (observations.empty())
  {
    result.accepted = true;
    result.reason = "NO_OBSERVATIONS";
    return result;
  }

  const Consumer *selected = nullptr;
  switch (mode_)
  {
    case FusionMode::ObserveOnly:
      selected = &observe_only_consumer;
      break;
    case FusionMode::LegacyEsikf:
      selected = &legacy_esikf_consumer;
      break;
    case FusionMode::GlobalBackend:
      selected = &global_backend_consumer;
      break;
  }

  if (!selected || !*selected)
  {
    result.reason = mode_ == FusionMode::GlobalBackend
        ? "GLOBAL_BACKEND_NOT_IMPLEMENTED"
        : "PRIMARY_CONSUMER_UNAVAILABLE";
    return result;
  }

  // ponytail: a single enum selects one callback, so conflicting consumers
  // are structurally impossible; add fan-out only with an explicit ownership
  // protocol if a future architecture genuinely needs it.
  (*selected)(observations);
  result.delivered_observations = observations.size();
  result.primary_consumer_count = 1;
  result.accepted = true;
  result.reason = fusionModeName(mode_);
  return result;
}

} // namespace landmark
