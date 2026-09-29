#ifndef LANDMARK_FRONTEND_H_
#define LANDMARK_FRONTEND_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <limits>
#include <map>
#include <string>
#include <vector>

namespace landmark
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Marker coordinates follow OpenCV IPPE_SQUARE: +X right, +Y up and +Z
// out of the printed front face. Corners are TL, TR, BR, BL when viewed
// from the front. T_landmark_marker maps marker-frame points to landmark.
struct MarkerGeometry
{
  int marker_id = -1;
  double size_m = 0.0;
  Eigen::Isometry3d T_landmark_marker = Eigen::Isometry3d::Identity();
};

struct LandmarkDefinition
{
  int landmark_id = -1;
  std::vector<MarkerGeometry> markers;
};

// Estimator-independent product of detection, association and joint PnP.
// T_camera_landmark obeys p_c = R_c_l * p_l + t_c_l.
struct LandmarkObservation
{
  double timestamp = std::numeric_limits<double>::quiet_NaN();
  int landmark_id = -1;
  std::vector<int> visible_marker_ids;
  int visible_marker_count = 0;
  int visible_corner_count = 0;
  Eigen::Isometry3d T_camera_landmark = Eigen::Isometry3d::Identity();
  std::string pnp_method;
  double reprojection_rmse_px = std::numeric_limits<double>::infinity();
  double estimated_distance_m = std::numeric_limits<double>::infinity();
  Eigen::Vector3d board_normal_camera = Eigen::Vector3d::Zero();
  double view_angle_deg = std::numeric_limits<double>::infinity();
  double visible_corner_hull_area_px2 = 0.0;
  int pnp_candidate_count = 0;
  int selected_candidate_index = -1;
  double best_candidate_rmse_px = std::numeric_limits<double>::infinity();
  double second_candidate_rmse_px = std::numeric_limits<double>::infinity();
  double candidate_rmse_gap_px = std::numeric_limits<double>::infinity();
  double candidate_rmse_ratio = std::numeric_limits<double>::infinity();
  // Tangent ordering is [camera-frame rotation rad, camera translation m].
  Matrix6d pose_covariance_camera = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  bool covariance_valid = false;
  std::string covariance_method;
  std::string covariance_reason;
  bool pnp_success = false;
  bool pose_finite = false;
  bool se3_valid = false;
  bool positive_depth = false;
  bool front_facing = false;
  bool pose_valid = false;
  std::string reject_reason;
};

struct PoseValidationOptions
{
  double min_depth_m = 0.1;
  double max_distance_m = 10.0;
  double max_reprojection_rmse_px = 8.0;
};

struct UncertaintyOptions
{
  double corner_sigma_px = 0.5;
  double rotation_difference_rad = 1e-5;
  double translation_difference_m = 1e-4;
  double min_rotation_variance_rad2 = 1e-8;
  double max_rotation_variance_rad2 = 0.25;
  double min_translation_variance_m2 = 1e-8;
  double max_translation_variance_m2 = 4.0;
  double max_information_condition = 1e12;
  // Bounded fallback only; tune from real board data before relying on it.
  double fallback_rotation_sigma_rad = 0.03;
  double fallback_translation_sigma_m = 0.03;
};

class LandmarkFrontend
{
public:
  bool configure(const std::vector<LandmarkDefinition> &definitions,
                 std::string *error);
  void setPoseValidationOptions(const PoseValidationOptions &options)
  {
    options_ = options;
  }
  void setUncertaintyOptions(const UncertaintyOptions &options)
  {
    uncertainty_options_ = options;
  }

  std::vector<LandmarkObservation> estimate(
      double timestamp,
      const std::vector<int> &detected_marker_ids,
      const std::vector<std::vector<cv::Point2f>> &detected_corners,
      const cv::Mat &camera_matrix,
      const cv::Mat &dist_coeffs) const;

  const LandmarkDefinition *landmarkForMarker(int marker_id) const;
  static std::vector<cv::Point3f> markerCornersInLandmark(
      const MarkerGeometry &marker);
  static double viewAngleDeg(const Eigen::Vector3d &board_normal_camera,
                             const Eigen::Vector3d &camera_to_landmark);

private:
  std::map<int, LandmarkDefinition> landmarks_;
  std::map<int, int> marker_to_landmark_;
  PoseValidationOptions options_;
  UncertaintyOptions uncertainty_options_;
};

bool arucoDictionaryIdFromName(const std::string &name, int *dictionary_id);

} // namespace landmark

#endif // LANDMARK_FRONTEND_H_
