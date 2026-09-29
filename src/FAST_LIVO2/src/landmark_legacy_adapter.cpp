#include "landmark_legacy_adapter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace landmark
{

bool LegacySameIdBoardAdapter::configure(
    const std::vector<int> &board_ids, const LegacyBoardGeometry &geometry,
    const PoseValidationOptions &pose_options,
    const UncertaintyOptions &uncertainty_options, std::string *error)
{
  if (board_ids.empty() || !std::isfinite(geometry.marker_size_m) ||
      !std::isfinite(geometry.half_center_x_m) ||
      !std::isfinite(geometry.half_center_y_m) ||
      geometry.marker_size_m <= 0.0 ||
      geometry.half_center_x_m <= geometry.marker_size_m * 0.5 ||
      geometry.half_center_y_m <= geometry.marker_size_m * 0.5)
  {
    if (error) *error = "invalid legacy board geometry or empty IDs";
    return false;
  }
  std::map<int, LandmarkFrontend> configured;
  const std::array<Eigen::Vector3d, 4> centers = {{
      {-geometry.half_center_x_m, geometry.half_center_y_m, 0.0},
      { geometry.half_center_x_m, geometry.half_center_y_m, 0.0},
      {-geometry.half_center_x_m,-geometry.half_center_y_m, 0.0},
      { geometry.half_center_x_m,-geometry.half_center_y_m, 0.0}}};
  for (int id : board_ids)
  {
    if (id < 0 || configured.count(id))
    {
      if (error) *error = "duplicate or negative legacy board ID";
      return false;
    }
    LandmarkDefinition definition;
    definition.landmark_id = id;
    for (int slot = 0; slot < 4; ++slot)
    {
      MarkerGeometry marker;
      marker.marker_id = slot; // private slot IDs, never detector identity
      marker.size_m = geometry.marker_size_m;
      marker.T_landmark_marker.translation() = centers[slot];
      definition.markers.push_back(marker);
    }
    LandmarkFrontend frontend;
    frontend.setPoseValidationOptions(pose_options);
    frontend.setUncertaintyOptions(uncertainty_options);
    if (!frontend.configure({definition}, error)) return false;
    configured.emplace(id, std::move(frontend));
  }
  frontends_ = std::move(configured);
  return true;
}

std::vector<LegacyAssociationResult> LegacySameIdBoardAdapter::estimate(
    double timestamp, const std::vector<int> &detected_ids,
    const std::vector<std::vector<cv::Point2f>> &detected_corners,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) const
{
  std::map<int, std::vector<std::size_t>> grouped;
  for (std::size_t i = 0; i < std::min(detected_ids.size(), detected_corners.size()); ++i)
    if (frontends_.count(detected_ids[i])) grouped[detected_ids[i]].push_back(i);

  std::vector<LegacyAssociationResult> results;
  for (const auto &group : grouped)
  {
    LegacyAssociationResult result;
    auto &diag = result.diagnostic;
    diag.landmark_id = group.first;
    diag.detected_markers = static_cast<int>(group.second.size());
    result.observation.landmark_id = group.first;
    result.observation.timestamp = timestamp;
    result.observation.visible_marker_count = diag.detected_markers;
    result.observation.visible_marker_ids.assign(group.second.size(), group.first);
    if (group.second.size() > 4)
    {
      diag.reason = "MORE_THAN_FOUR_SAME_ID_DETECTIONS";
      result.observation.reject_reason = diag.reason;
      results.push_back(std::move(result));
      continue;
    }
    const std::size_t visible = group.second.size();
    std::array<int, 4> slots{{0, 1, 2, 3}};
    std::set<std::vector<int>> unique_assignments;
    double best_valid_rmse = std::numeric_limits<double>::infinity();
    double best_any_rmse = std::numeric_limits<double>::infinity();
    LandmarkObservation best_valid, best_any;
    std::array<int, 4> best_valid_slots{{-1, -1, -1, -1}};
    std::array<int, 4> best_any_slots{{-1, -1, -1, -1}};
    std::vector<std::pair<std::vector<int>, double>> assignment_errors;
    do
    {
      const std::vector<int> assignment(slots.begin(), slots.begin() + visible);
      if (!unique_assignments.insert(assignment).second) continue;
      ++diag.assignment_count;
      std::vector<std::vector<cv::Point2f>> corners;
      corners.reserve(visible);
      for (std::size_t index : group.second) corners.push_back(detected_corners[index]);
      const auto candidates = frontends_.at(group.first).estimate(
          timestamp, assignment, corners, camera_matrix, dist_coeffs);
      if (candidates.empty()) continue;
      const auto &candidate = candidates.front();
      if (!candidate.pnp_success || !std::isfinite(candidate.reprojection_rmse_px))
        continue;
      ++diag.feasible_assignment_count;
      assignment_errors.emplace_back(assignment, candidate.reprojection_rmse_px);
      if (candidate.reprojection_rmse_px < best_any_rmse)
      {
        best_any_rmse = candidate.reprojection_rmse_px;
        best_any = candidate;
        best_any_slots = {{-1, -1, -1, -1}};
        std::copy(assignment.begin(), assignment.end(), best_any_slots.begin());
      }
      if (candidate.pose_valid && candidate.covariance_valid &&
          candidate.reprojection_rmse_px < best_valid_rmse)
      {
        best_valid_rmse = candidate.reprojection_rmse_px;
        best_valid = candidate;
        best_valid_slots = {{-1, -1, -1, -1}};
        std::copy(assignment.begin(), assignment.end(), best_valid_slots.begin());
      }
    } while (std::next_permutation(slots.begin(), slots.end()));

    if (std::isfinite(best_valid_rmse))
    {
      result.observation = best_valid;
      diag.selected_slots = best_valid_slots;
      diag.pnp_feasible = true;
    }
    else if (std::isfinite(best_any_rmse))
    {
      result.observation = best_any;
      diag.selected_slots = best_any_slots;
      diag.pnp_feasible = true;
    }
    result.observation.visible_marker_ids.assign(visible, group.first);
    diag.best_rmse_px = result.observation.reprojection_rmse_px;
    // Compare distinct assignments, not the two IPPE poses of one assignment.
    for (const auto &entry : assignment_errors)
      if (!std::equal(entry.first.begin(), entry.first.end(),
                      diag.selected_slots.begin()))
        diag.second_assignment_rmse_px = std::min(
            diag.second_assignment_rmse_px, entry.second);
    diag.assignment_gap_px =
        diag.second_assignment_rmse_px - diag.best_rmse_px;
    diag.assignment_ratio = diag.second_assignment_rmse_px /
        std::max(diag.best_rmse_px, 1e-9);
    const bool quality = result.observation.pose_valid &&
                         result.observation.covariance_valid;
    // ponytail: first version admits only unambiguous 4/3-marker poses;
    // 2/1-marker results are diagnostics until real partial-board calibration.
    const double min_gap = visible == 3 ? 3.0 : 1.0;
    const double min_ratio = visible == 3 ? 2.0 : 1.5;
    diag.factor_admitted = visible >= 3 && quality &&
        std::isfinite(diag.second_assignment_rmse_px) &&
        diag.assignment_gap_px >= min_gap &&
        diag.assignment_ratio >= min_ratio;
    diag.reason = visible < 3 ? "LEGACY_PARTIAL_ASSIGNMENT_POLICY" :
        !quality ? "LEGACY_PNP_OR_COVARIANCE_INVALID" :
        !diag.factor_admitted ? "LEGACY_ASSIGNMENT_AMBIGUOUS" : "ACCEPTED";
    if (!diag.factor_admitted)
    {
      result.observation.pose_valid = false;
      result.observation.reject_reason = diag.reason;
    }
    results.push_back(std::move(result));
  }
  return results;
}

} // namespace landmark
