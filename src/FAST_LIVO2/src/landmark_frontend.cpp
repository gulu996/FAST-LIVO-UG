#include "landmark_frontend.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <sstream>

namespace landmark
{
namespace
{

bool finiteTransform(const Eigen::Isometry3d &T)
{
  return T.matrix().allFinite();
}

bool validRotation(const Eigen::Matrix3d &R)
{
  return R.allFinite() &&
      std::fabs(R.determinant() - 1.0) < 1e-6 &&
      (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-6;
}

Eigen::Isometry3d cvPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
{
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      T.linear()(row, col) = R_cv.at<double>(row, col);
  T.translation() = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
  return T;
}

void eigenPose(const Eigen::Isometry3d &T, cv::Vec3d *rvec, cv::Vec3d *tvec)
{
  cv::Mat R_cv(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      R_cv.at<double>(row, col) = T.linear()(row, col);
  cv::Rodrigues(R_cv, *rvec);
  *tvec = cv::Vec3d(T.translation().x(), T.translation().y(),
                    T.translation().z());
}

double reprojectionRmse(const std::vector<cv::Point3f> &object_points,
                        const std::vector<cv::Point2f> &image_points,
                        const Eigen::Isometry3d &T_camera_landmark,
                        const cv::Mat &camera_matrix,
                        const cv::Mat &dist_coeffs)
{
  cv::Vec3d rvec, tvec;
  eigenPose(T_camera_landmark, &rvec, &tvec);
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs,
                    projected);
  double squared_error = 0.0;
  for (size_t i = 0; i < projected.size(); ++i)
  {
    const cv::Point2f delta = projected[i] - image_points[i];
    squared_error += delta.dot(delta);
  }
  return std::sqrt(squared_error / static_cast<double>(projected.size()));
}

struct Candidate
{
  Eigen::Isometry3d T_camera_landmark = Eigen::Isometry3d::Identity();
  double reprojection_rmse_px = std::numeric_limits<double>::infinity();
  bool finite = false;
  bool se3 = false;
  bool positive_depth = false;
  bool front_facing = false;
  bool distance_valid = false;
};

Eigen::Matrix3d expRotation(const Eigen::Vector3d &w)
{
  const double theta = w.norm();
  if (theta < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}

bool projectedPixels(const std::vector<cv::Point3f> &object_points,
                     const Eigen::Isometry3d &T_camera_landmark,
                     const cv::Mat &camera_matrix,
                     const cv::Mat &dist_coeffs,
                     Eigen::VectorXd *pixels)
{
  cv::Vec3d rvec, tvec;
  eigenPose(T_camera_landmark, &rvec, &tvec);
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs,
                    projected);
  pixels->resize(static_cast<Eigen::Index>(projected.size() * 2));
  for (size_t i = 0; i < projected.size(); ++i)
  {
    (*pixels)(2 * i) = projected[i].x;
    (*pixels)(2 * i + 1) = projected[i].y;
  }
  return pixels->allFinite();
}

bool numericalPoseCovariance(
    const std::vector<cv::Point3f> &object_points,
    const Eigen::Isometry3d &T_camera_landmark,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    double reprojection_rmse_px,
    const UncertaintyOptions &options,
    Matrix6d *covariance,
    std::string *reason)
{
  Eigen::MatrixXd J(object_points.size() * 2, 6);
  for (int column = 0; column < 6; ++column)
  {
    const double step = column < 3 ? options.rotation_difference_rad
                                   : options.translation_difference_m;
    if (!std::isfinite(step) || step <= 0.0)
    {
      *reason = "invalid_finite_difference_step";
      return false;
    }
    Eigen::Isometry3d plus = T_camera_landmark;
    Eigen::Isometry3d minus = T_camera_landmark;
    if (column < 3)
    {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(column) = step;
      plus.linear() = expRotation(delta) * plus.linear();
      minus.linear() = expRotation(-delta) * minus.linear();
    }
    else
    {
      plus.translation()(column - 3) += step;
      minus.translation()(column - 3) -= step;
    }
    Eigen::VectorXd pixels_plus, pixels_minus;
    if (!projectedPixels(object_points, plus, camera_matrix, dist_coeffs,
                         &pixels_plus) ||
        !projectedPixels(object_points, minus, camera_matrix, dist_coeffs,
                         &pixels_minus))
    {
      *reason = "nonfinite_projection_jacobian";
      return false;
    }
    J.col(column) = (pixels_plus - pixels_minus) / (2.0 * step);
  }

  const double pixel_variance =
      options.corner_sigma_px * options.corner_sigma_px +
      reprojection_rmse_px * reprojection_rmse_px;
  if (!J.allFinite() || !std::isfinite(pixel_variance) || pixel_variance <= 0.0)
  {
    *reason = "invalid_reprojection_information";
    return false;
  }
  const Matrix6d information = J.transpose() * J / pixel_variance;
  Eigen::SelfAdjointEigenSolver<Matrix6d> eigensolver(information);
  if (eigensolver.info() != Eigen::Success ||
      eigensolver.eigenvalues().minCoeff() <= 0.0)
  {
    *reason = "reprojection_information_not_positive_definite";
    return false;
  }
  const double condition = eigensolver.eigenvalues().maxCoeff() /
      eigensolver.eigenvalues().minCoeff();
  if (!std::isfinite(condition) || condition > options.max_information_condition)
  {
    *reason = "reprojection_information_ill_conditioned";
    return false;
  }
  Eigen::LDLT<Matrix6d> ldlt(information);
  *covariance = ldlt.solve(Matrix6d::Identity());
  *covariance = 0.5 * (*covariance + covariance->transpose());
  covariance->diagonal().head<3>() = covariance->diagonal().head<3>().array()
      .max(options.min_rotation_variance_rad2);
  covariance->diagonal().tail<3>() = covariance->diagonal().tail<3>().array()
      .max(options.min_translation_variance_m2);
  if (!covariance->allFinite() ||
      covariance->diagonal().head<3>().maxCoeff() >
          options.max_rotation_variance_rad2 ||
      covariance->diagonal().tail<3>().maxCoeff() >
          options.max_translation_variance_m2)
  {
    *reason = "reprojection_covariance_out_of_bounds";
    return false;
  }
  return true;
}

Matrix6d boundedHeuristicCovariance(const LandmarkObservation &observation,
                                    const UncertaintyOptions &options)
{
  // ponytail: this transparent diagonal fallback is deliberately simple;
  // replace its constants with a calibrated model after real-board trials.
  const double count_scale = std::sqrt(
      4.0 / std::max(1, observation.visible_marker_count));
  const double distance_scale = std::max(0.5,
      observation.estimated_distance_m / 2.0);
  const double cosine = std::cos(observation.view_angle_deg * M_PI / 180.0);
  const double angle_scale = 1.0 / std::max(0.2, cosine);
  const double residual_scale = std::max(1.0,
      observation.reprojection_rmse_px /
          std::max(0.1, options.corner_sigma_px));
  const double scale = count_scale * distance_scale * angle_scale *
      residual_scale;
  const double rotation_variance = std::clamp(
      std::pow(options.fallback_rotation_sigma_rad * scale, 2),
      options.min_rotation_variance_rad2,
      options.max_rotation_variance_rad2);
  const double translation_variance = std::clamp(
      std::pow(options.fallback_translation_sigma_m * scale, 2),
      options.min_translation_variance_m2,
      options.max_translation_variance_m2);
  Matrix6d covariance = Matrix6d::Zero();
  covariance.diagonal().head<3>().setConstant(rotation_variance);
  covariance.diagonal().tail<3>().setConstant(translation_variance);
  return covariance;
}

Candidate evaluateCandidate(const Eigen::Isometry3d &T_camera_landmark,
                            const std::vector<cv::Point3f> &object_points,
                            const std::vector<cv::Point2f> &image_points,
                            const cv::Mat &camera_matrix,
                            const cv::Mat &dist_coeffs,
                            const PoseValidationOptions &options)
{
  Candidate candidate;
  candidate.T_camera_landmark = T_camera_landmark;
  candidate.finite = finiteTransform(T_camera_landmark);
  candidate.se3 = candidate.finite && validRotation(T_camera_landmark.linear());
  if (!candidate.se3) return candidate;

  candidate.positive_depth = true;
  for (const cv::Point3f &point : object_points)
  {
    const Eigen::Vector3d point_camera = T_camera_landmark *
        Eigen::Vector3d(point.x, point.y, point.z);
    if (!point_camera.allFinite() || point_camera.z() <= options.min_depth_m)
    {
      candidate.positive_depth = false;
      break;
    }
  }

  // The configured +Z axis is the printed front normal. A visible front face
  // points from the board towards the camera, opposite camera-to-board t_c_l.
  const Eigen::Vector3d normal_camera = T_camera_landmark.linear().col(2);
  candidate.front_facing =
      normal_camera.dot(T_camera_landmark.translation()) < 0.0;
  const double distance = T_camera_landmark.translation().norm();
  candidate.distance_valid = std::isfinite(distance) &&
      distance >= options.min_depth_m && distance <= options.max_distance_m;
  candidate.reprojection_rmse_px = reprojectionRmse(
      object_points, image_points, T_camera_landmark, camera_matrix,
      dist_coeffs);
  return candidate;
}

int validityRank(const Candidate &candidate,
                 const PoseValidationOptions &options)
{
  int rank = 0;
  rank += candidate.finite ? 16 : 0;
  rank += candidate.se3 ? 8 : 0;
  rank += candidate.positive_depth ? 4 : 0;
  rank += candidate.front_facing ? 2 : 0;
  rank += candidate.distance_valid &&
      candidate.reprojection_rmse_px <= options.max_reprojection_rmse_px ? 1 : 0;
  return rank;
}

} // namespace

bool LandmarkFrontend::configure(
    const std::vector<LandmarkDefinition> &definitions,
    std::string *error)
{
  std::map<int, LandmarkDefinition> landmarks;
  std::map<int, int> marker_to_landmark;
  if (definitions.empty())
  {
    if (error) *error = "no landmark definitions";
    return false;
  }

  for (const LandmarkDefinition &landmark : definitions)
  {
    if (landmark.landmark_id < 0 || landmark.markers.empty())
    {
      if (error) *error = "landmark ID must be non-negative and contain markers";
      return false;
    }
    if (!landmarks.emplace(landmark.landmark_id, landmark).second)
    {
      if (error) *error = "duplicate landmark_id " +
          std::to_string(landmark.landmark_id);
      return false;
    }

    std::vector<Eigen::Vector3d> marker_centers;
    for (const MarkerGeometry &marker : landmark.markers)
    {
      if (marker.marker_id < 0 || !std::isfinite(marker.size_m) ||
          marker.size_m <= 0.0)
      {
        if (error) *error = "invalid marker ID or size in landmark " +
            std::to_string(landmark.landmark_id);
        return false;
      }
      if (!finiteTransform(marker.T_landmark_marker) ||
          !validRotation(marker.T_landmark_marker.linear()))
      {
        if (error) *error = "invalid T_landmark_marker for marker " +
            std::to_string(marker.marker_id);
        return false;
      }
      const Eigen::Vector3d normal =
          marker.T_landmark_marker.linear().col(2);
      if (std::fabs(marker.T_landmark_marker.translation().z()) > 1e-6 ||
          normal.dot(Eigen::Vector3d::UnitZ()) < 1.0 - 1e-6)
      {
        if (error) *error = "marker geometry is not coplanar with landmark +Z for marker " +
            std::to_string(marker.marker_id);
        return false;
      }
      for (const Eigen::Vector3d &center : marker_centers)
      {
        if ((center - marker.T_landmark_marker.translation()).norm() < 1e-9)
        {
          if (error) *error = "overlapping marker centers in landmark " +
              std::to_string(landmark.landmark_id);
          return false;
        }
      }
      marker_centers.push_back(marker.T_landmark_marker.translation());
      if (!marker_to_landmark.emplace(marker.marker_id,
                                      landmark.landmark_id).second)
      {
        if (error) *error = "marker_id " + std::to_string(marker.marker_id) +
            " belongs to more than one landmark";
        return false;
      }
    }
  }

  landmarks_ = std::move(landmarks);
  marker_to_landmark_ = std::move(marker_to_landmark);
  return true;
}

const LandmarkDefinition *LandmarkFrontend::landmarkForMarker(int marker_id) const
{
  const auto marker_it = marker_to_landmark_.find(marker_id);
  if (marker_it == marker_to_landmark_.end()) return nullptr;
  const auto landmark_it = landmarks_.find(marker_it->second);
  return landmark_it == landmarks_.end() ? nullptr : &landmark_it->second;
}

std::vector<cv::Point3f> LandmarkFrontend::markerCornersInLandmark(
    const MarkerGeometry &marker)
{
  const double half = 0.5 * marker.size_m;
  const std::array<Eigen::Vector3d, 4> local = {
      Eigen::Vector3d(-half, half, 0.0),
      Eigen::Vector3d(half, half, 0.0),
      Eigen::Vector3d(half, -half, 0.0),
      Eigen::Vector3d(-half, -half, 0.0)};
  std::vector<cv::Point3f> corners;
  corners.reserve(4);
  for (const Eigen::Vector3d &point : local)
  {
    const Eigen::Vector3d point_landmark = marker.T_landmark_marker * point;
    corners.emplace_back(static_cast<float>(point_landmark.x()),
                         static_cast<float>(point_landmark.y()),
                         static_cast<float>(point_landmark.z()));
  }
  return corners;
}

double LandmarkFrontend::viewAngleDeg(
    const Eigen::Vector3d &board_normal_camera,
    const Eigen::Vector3d &camera_to_landmark)
{
  if (!board_normal_camera.allFinite() ||
      !camera_to_landmark.allFinite() ||
      board_normal_camera.norm() <= 0.0 || camera_to_landmark.norm() <= 0.0)
    return std::numeric_limits<double>::infinity();
  return std::acos(std::clamp(
      board_normal_camera.normalized().dot(
          -camera_to_landmark.normalized()), -1.0, 1.0)) * 180.0 / M_PI;
}

std::vector<LandmarkObservation> LandmarkFrontend::estimate(
    double timestamp,
    const std::vector<int> &detected_marker_ids,
    const std::vector<std::vector<cv::Point2f>> &detected_corners,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs) const
{
  std::map<int, std::vector<size_t>> grouped;
  const size_t detection_count =
      std::min(detected_marker_ids.size(), detected_corners.size());
  for (size_t i = 0; i < detection_count; ++i)
  {
    const auto marker_it = marker_to_landmark_.find(detected_marker_ids[i]);
    if (marker_it != marker_to_landmark_.end())
      grouped[marker_it->second].push_back(i);
  }

  std::vector<LandmarkObservation> observations;
  observations.reserve(grouped.size());
  for (const auto &group : grouped)
  {
    LandmarkObservation observation;
    observation.timestamp = timestamp;
    observation.landmark_id = group.first;
    const LandmarkDefinition &definition = landmarks_.at(group.first);
    std::map<int, const MarkerGeometry *> geometry_by_id;
    for (const MarkerGeometry &marker : definition.markers)
      geometry_by_id[marker.marker_id] = &marker;

    std::vector<size_t> indices = group.second;
    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
      return detected_marker_ids[a] < detected_marker_ids[b];
    });

    std::set<int> seen_ids;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    const MarkerGeometry *single_marker = nullptr;
    bool association_valid = true;
    for (size_t index : indices)
    {
      const int marker_id = detected_marker_ids[index];
      if (!seen_ids.insert(marker_id).second)
      {
        observation.reject_reason = "duplicate marker detection; positional identity guessing is forbidden";
        association_valid = false;
        break;
      }
      if (detected_corners[index].size() != 4)
      {
        observation.reject_reason = "marker does not have four image corners";
        association_valid = false;
        break;
      }
      for (const cv::Point2f &corner : detected_corners[index])
      {
        if (!std::isfinite(corner.x) || !std::isfinite(corner.y))
        {
          observation.reject_reason = "non-finite image corner";
          association_valid = false;
          break;
        }
      }
      if (!association_valid) break;
      const MarkerGeometry *geometry = geometry_by_id.at(marker_id);
      observation.visible_marker_ids.push_back(marker_id);
      const std::vector<cv::Point3f> marker_points =
          markerCornersInLandmark(*geometry);
      object_points.insert(object_points.end(), marker_points.begin(),
                           marker_points.end());
      image_points.insert(image_points.end(), detected_corners[index].begin(),
                          detected_corners[index].end());
      single_marker = geometry;
    }

    observation.visible_marker_count =
        static_cast<int>(observation.visible_marker_ids.size());
    observation.visible_corner_count = static_cast<int>(image_points.size());
    if (image_points.size() >= 3)
    {
      std::vector<cv::Point2f> hull;
      cv::convexHull(image_points, hull);
      observation.visible_corner_hull_area_px2 =
          std::fabs(cv::contourArea(hull));
    }
    if (!association_valid || object_points.size() < 4)
    {
      if (observation.reject_reason.empty())
        observation.reject_reason = "insufficient correspondences";
      observations.push_back(observation);
      continue;
    }

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    if (observation.visible_marker_count == 1)
    {
      const double half = 0.5 * single_marker->size_m;
      const std::vector<cv::Point3f> local_points = {
          cv::Point3f(-half, half, 0.0), cv::Point3f(half, half, 0.0),
          cv::Point3f(half, -half, 0.0), cv::Point3f(-half, -half, 0.0)};
      cv::solvePnPGeneric(local_points, image_points, camera_matrix, dist_coeffs,
                          rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE);
      observation.pnp_method = "IPPE_SQUARE";
      for (size_t i = 0; i < rvecs.size(); ++i)
      {
        const Eigen::Isometry3d T_camera_marker = cvPose(rvecs[i], tvecs[i]);
        const Eigen::Isometry3d T_camera_landmark =
            T_camera_marker * single_marker->T_landmark_marker.inverse();
        cv::Vec3d rvec, tvec;
        eigenPose(T_camera_landmark, &rvec, &tvec);
        rvecs[i] = rvec;
        tvecs[i] = tvec;
      }
    }
    else
    {
      cv::solvePnPGeneric(object_points, image_points, camera_matrix, dist_coeffs,
                          rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
      observation.pnp_method = "IPPE";
    }

    if (rvecs.empty())
    {
      cv::Vec3d rvec, tvec;
      bool ok = false;
      if (observation.visible_marker_count == 1)
      {
        const double half = 0.5 * single_marker->size_m;
        const std::vector<cv::Point3f> local_points = {
            cv::Point3f(-half, half, 0.0), cv::Point3f(half, half, 0.0),
            cv::Point3f(half, -half, 0.0), cv::Point3f(-half, -half, 0.0)};
        ok = cv::solvePnP(local_points, image_points, camera_matrix, dist_coeffs,
                          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        if (ok)
        {
          const Eigen::Isometry3d T_camera_landmark =
              cvPose(rvec, tvec) * single_marker->T_landmark_marker.inverse();
          eigenPose(T_camera_landmark, &rvec, &tvec);
        }
      }
      else
      {
        ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs,
                          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
      }
      if (ok)
      {
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
        observation.pnp_method += "+ITERATIVE_FALLBACK";
      }
    }

    observation.pnp_success = !rvecs.empty();
    observation.pnp_candidate_count = static_cast<int>(rvecs.size());
    if (!observation.pnp_success)
    {
      observation.reject_reason = "PnP returned no candidate";
      observations.push_back(observation);
      continue;
    }

    Candidate best;
    int best_rank = -1;
    std::vector<Candidate> candidates;
    candidates.reserve(rvecs.size());
    for (size_t i = 0; i < rvecs.size(); ++i)
    {
      const Candidate candidate = evaluateCandidate(
          cvPose(rvecs[i], tvecs[i]), object_points, image_points,
          camera_matrix, dist_coeffs, options_);
      candidates.push_back(candidate);
      const int rank = validityRank(candidate, options_);
      if (rank > best_rank ||
          (rank == best_rank && candidate.reprojection_rmse_px <
                                best.reprojection_rmse_px))
      {
        best = candidate;
        best_rank = rank;
        observation.selected_candidate_index = static_cast<int>(i);
      }
    }

    std::vector<double> candidate_errors;
    for (const Candidate &candidate : candidates)
      if (std::isfinite(candidate.reprojection_rmse_px))
        candidate_errors.push_back(candidate.reprojection_rmse_px);
    std::sort(candidate_errors.begin(), candidate_errors.end());
    if (!candidate_errors.empty())
      observation.best_candidate_rmse_px = candidate_errors[0];
    if (candidate_errors.size() > 1)
    {
      observation.second_candidate_rmse_px = candidate_errors[1];
      observation.candidate_rmse_gap_px =
          candidate_errors[1] - candidate_errors[0];
      observation.candidate_rmse_ratio = candidate_errors[1] /
          std::max(candidate_errors[0], 1e-12);
    }

    observation.T_camera_landmark = best.T_camera_landmark;
    observation.reprojection_rmse_px = best.reprojection_rmse_px;
    observation.estimated_distance_m =
        best.T_camera_landmark.translation().norm();
    observation.board_normal_camera =
        best.T_camera_landmark.linear().col(2);
    observation.view_angle_deg = viewAngleDeg(
        observation.board_normal_camera,
        best.T_camera_landmark.translation());
    observation.pose_finite = best.finite;
    observation.se3_valid = best.se3;
    observation.positive_depth = best.positive_depth;
    observation.front_facing = best.front_facing;
    observation.pose_valid = best.finite && best.se3 && best.positive_depth &&
        best.front_facing && best.distance_valid &&
        best.reprojection_rmse_px <= options_.max_reprojection_rmse_px;
    if (!observation.pose_valid)
    {
      std::ostringstream reason;
      reason << "pose sanity failed: finite=" << best.finite
             << " se3=" << best.se3
             << " positive_depth=" << best.positive_depth
             << " front_facing=" << best.front_facing
             << " distance=" << best.distance_valid
             << " reprojection_rmse_px=" << best.reprojection_rmse_px;
      observation.reject_reason = reason.str();
    }
    if (observation.pose_valid)
    {
      std::string numerical_reason;
      observation.covariance_valid = numericalPoseCovariance(
          object_points, best.T_camera_landmark, camera_matrix, dist_coeffs,
          best.reprojection_rmse_px, uncertainty_options_,
          &observation.pose_covariance_camera, &numerical_reason);
      if (observation.covariance_valid)
      {
        observation.covariance_method =
            "NUMERICAL_REPROJECTION_JACOBIAN";
      }
      else
      {
        observation.pose_covariance_camera = boundedHeuristicCovariance(
            observation, uncertainty_options_);
        observation.covariance_valid =
            observation.pose_covariance_camera.allFinite();
        observation.covariance_method =
            "BOUNDED_HEURISTIC_NEEDS_REAL_BOARD_CALIBRATION";
        observation.covariance_reason = numerical_reason;
      }
    }
    observations.push_back(observation);
  }
  return observations;
}

bool arucoDictionaryIdFromName(const std::string &name, int *dictionary_id)
{
  static const std::map<std::string, int> dictionaries = {
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
      {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
      {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
      {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
      {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
      {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
      {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
      {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
      {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
      {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
      {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};
  const auto it = dictionaries.find(name);
  if (it == dictionaries.end()) return false;
  if (dictionary_id) *dictionary_id = it->second;
  return true;
}

} // namespace landmark
