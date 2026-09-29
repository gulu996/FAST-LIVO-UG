#ifndef LANDMARK_LEGACY_ADAPTER_H_
#define LANDMARK_LEGACY_ADAPTER_H_

#include "landmark_frontend.h"

#include <array>
#include <map>
#include <string>

namespace landmark
{

struct LegacyBoardGeometry
{
  double marker_size_m = 0.0;
  double half_center_x_m = 0.0;
  double half_center_y_m = 0.0;
};

struct LegacyAssociationDiagnostic
{
  int landmark_id = -1;
  int detected_markers = 0;
  int assignment_count = 0;
  int feasible_assignment_count = 0;
  std::array<int, 4> selected_slots{{-1, -1, -1, -1}};
  double best_rmse_px = std::numeric_limits<double>::infinity();
  double second_assignment_rmse_px = std::numeric_limits<double>::infinity();
  double assignment_gap_px = std::numeric_limits<double>::infinity();
  double assignment_ratio = std::numeric_limits<double>::infinity();
  bool pnp_feasible = false;
  bool factor_admitted = false;
  std::string reason;
};

struct LegacyAssociationResult
{
  LandmarkObservation observation;
  LegacyAssociationDiagnostic diagnostic;
};

// Compatibility input only. The returned observation enters the unchanged
// PersistentLandmark/SparseKeyPose/ISAM2 path.
class LegacySameIdBoardAdapter
{
public:
  bool configure(const std::vector<int> &board_ids,
                 const LegacyBoardGeometry &geometry,
                 const PoseValidationOptions &pose_options,
                 const UncertaintyOptions &uncertainty_options,
                 std::string *error);
  std::vector<LegacyAssociationResult> estimate(
      double timestamp, const std::vector<int> &detected_ids,
      const std::vector<std::vector<cv::Point2f>> &detected_corners,
      const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) const;

private:
  std::map<int, LandmarkFrontend> frontends_;
};

} // namespace landmark
#endif
