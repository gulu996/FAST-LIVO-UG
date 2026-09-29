#include "landmark_frontend.h"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{

void require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

landmark::LandmarkDefinition makeBoard(int landmark_id,
                                       const std::vector<int> &marker_ids)
{
  const std::array<Eigen::Vector3d, 4> centers = {
      Eigen::Vector3d(-0.28, 0.18, 0.0),
      Eigen::Vector3d(0.28, 0.18, 0.0),
      Eigen::Vector3d(-0.28, -0.18, 0.0),
      Eigen::Vector3d(0.28, -0.18, 0.0)};
  landmark::LandmarkDefinition board;
  board.landmark_id = landmark_id;
  for (size_t i = 0; i < marker_ids.size(); ++i)
  {
    landmark::MarkerGeometry marker;
    marker.marker_id = marker_ids[i];
    marker.size_m = 0.16;
    marker.T_landmark_marker.translation() = centers[i];
    board.markers.push_back(marker);
  }
  return board;
}

std::vector<cv::Point2f> projectMarker(
    const landmark::MarkerGeometry &marker,
    const Eigen::Isometry3d &T_camera_landmark,
    const cv::Mat &camera_matrix)
{
  cv::Mat R_cv(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      R_cv.at<double>(row, col) = T_camera_landmark.linear()(row, col);
  cv::Vec3d rvec;
  cv::Rodrigues(R_cv, rvec);
  const Eigen::Vector3d &t = T_camera_landmark.translation();
  const cv::Vec3d tvec(t.x(), t.y(), t.z());
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(landmark::LandmarkFrontend::markerCornersInLandmark(marker),
                    rvec, tvec, camera_matrix, cv::Mat(), image_points);
  return image_points;
}

double rotationError(const Eigen::Matrix3d &actual,
                     const Eigen::Matrix3d &expected)
{
  return Eigen::AngleAxisd(actual * expected.transpose()).angle();
}

} // namespace

int main()
{
  try
  {
    const landmark::LandmarkDefinition board = makeBoard(10, {0, 1, 2, 3});
    landmark::LandmarkFrontend frontend;
    std::string error;
    require(frontend.configure({board}, &error), "valid board rejected: " + error);
    require(frontend.landmarkForMarker(2) != nullptr &&
            frontend.landmarkForMarker(2)->landmark_id == 10,
            "marker-to-landmark association failed");

    landmark::LandmarkDefinition duplicate_board = makeBoard(11, {3});
    require(!frontend.configure({board, duplicate_board}, &error),
            "duplicate marker ID accepted");
    require(frontend.configure({board}, &error), "frontend restore failed");

    int dictionary_id = -1;
    require(landmark::arucoDictionaryIdFromName("DICT_6X6_250", &dictionary_id),
            "valid dictionary rejected");
    require(!landmark::arucoDictionaryIdFromName("DICT_NOT_REAL", &dictionary_id),
            "invalid dictionary accepted");

    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        800.0, 0.0, 640.0,
        0.0, 800.0, 360.0,
        0.0, 0.0, 1.0);
    Eigen::Isometry3d T_camera_landmark_gt = Eigen::Isometry3d::Identity();
    const Eigen::Matrix3d front_facing =
        (Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitX())).toRotationMatrix() *
        (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
    T_camera_landmark_gt.linear() = front_facing;
    T_camera_landmark_gt.translation() = Eigen::Vector3d(0.08, -0.03, 3.0);

    std::vector<int> all_ids;
    std::vector<std::vector<cv::Point2f>> all_corners;
    for (const landmark::MarkerGeometry &marker : board.markers)
    {
      all_ids.push_back(marker.marker_id);
      all_corners.push_back(projectMarker(marker, T_camera_landmark_gt,
                                          camera_matrix));
    }

    for (int visible = 1; visible <= 4; ++visible)
    {
      const std::vector<int> ids(all_ids.begin(), all_ids.begin() + visible);
      const std::vector<std::vector<cv::Point2f>> corners(
          all_corners.begin(), all_corners.begin() + visible);
      const std::vector<landmark::LandmarkObservation> observations =
          frontend.estimate(123.5, ids, corners, camera_matrix, cv::Mat());
      require(observations.size() == 1, "candidate association failed");
      const landmark::LandmarkObservation &observation = observations.front();
      require(observation.visible_marker_count == visible,
              "partial visibility count mismatch");
      require(observation.visible_corner_count == visible * 4,
              "visible corner count mismatch");
      require(observation.pnp_success, "PnP failed");
      require(observation.pose_valid, "pose sanity failed: " + observation.reject_reason);
      require(observation.positive_depth, "positive-depth audit failed");
      require((observation.T_camera_landmark.translation() -
               T_camera_landmark_gt.translation()).norm() < 1e-4,
              "translation recovery failed");
      require(rotationError(observation.T_camera_landmark.linear(),
                            T_camera_landmark_gt.linear()) < 1e-4,
              "rotation recovery failed");
      require(observation.T_camera_landmark.linear().col(2).dot(
                  T_camera_landmark_gt.linear().col(2)) > 1.0 - 1e-8,
              "board normal flipped");
      std::cout << visible << " marker: candidate=PASS pnp=PASS pose_valid=PASS "
                << "fusion_accepted=NOT_DECIDED_BY_L1 method="
                << observation.pnp_method << " rmse="
                << observation.reprojection_rmse_px << '\n';
    }

    auto observe = [&](const Eigen::Isometry3d &pose, int visible) {
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      for (int i = 0; i < visible; ++i)
      {
        ids.push_back(board.markers[i].marker_id);
        corners.push_back(projectMarker(board.markers[i], pose,
                                        camera_matrix));
      }
      const auto observations = frontend.estimate(
          200.0 + visible, ids, corners, camera_matrix, cv::Mat());
      require(observations.size() == 1, "quality scenario association failed");
      require(observations.front().pose_valid,
              "quality scenario pose invalid: " +
                  observations.front().reject_reason);
      return observations.front();
    };

    const Eigen::Vector3d camera_to_board(0.0, 0.0, 2.0);
    const Eigen::Vector3d normal0(0.0, 0.0, -1.0);
    const Eigen::Vector3d normal30 =
        Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitY()) * normal0;
    const Eigen::Vector3d normal60 =
        Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitY()) * normal0;
    require(landmark::LandmarkFrontend::viewAngleDeg(
                normal0, camera_to_board) < 1e-10,
            "front view angle is not zero");
    require(std::fabs(landmark::LandmarkFrontend::viewAngleDeg(
                normal30, camera_to_board) - 30.0) < 1e-10,
            "30 degree view angle is wrong");
    require(landmark::LandmarkFrontend::viewAngleDeg(
                normal60, camera_to_board) > 59.999,
            "oblique view angle is wrong");

    const Eigen::Isometry3d front_pose = T_camera_landmark_gt;
    const landmark::LandmarkObservation front_observation =
        observe(front_pose, 4);
    require(front_observation.board_normal_camera.dot(
                front_pose.linear().col(2)) > 1.0 - 1e-10,
            "known-front board normal sign is wrong");

    Eigen::Isometry3d far_pose = front_pose;
    far_pose.translation() =
        5.0 * front_pose.translation().normalized();
    const landmark::LandmarkObservation far_observation = observe(far_pose, 4);
    require(front_observation.visible_corner_hull_area_px2 >
                far_observation.visible_corner_hull_area_px2,
            "visible corner hull area is not distance-monotonic");

    const landmark::LandmarkObservation near_four = observe(front_pose, 4);
    const landmark::LandmarkObservation far_one = observe(far_pose, 1);
    require(near_four.covariance_valid && far_one.covariance_valid,
            "pose covariance was not produced");
    require(near_four.pose_covariance_camera.trace() <
                far_one.pose_covariance_camera.trace(),
            "typical near-4 covariance is not smaller than far-1 covariance");
    require(far_one.pnp_candidate_count >= 2 &&
                std::isfinite(far_one.second_candidate_rmse_px),
            "single-marker PnP ambiguity diagnostics are missing");

    const Eigen::Isometry3d T_world_camera =
        Eigen::Translation3d(1.0, 2.0, 0.5) *
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ());
    const Eigen::Isometry3d T_world_landmark =
        T_world_camera * T_camera_landmark_gt;
    require((T_world_camera.inverse() * T_world_landmark).matrix().isApprox(
                T_camera_landmark_gt.matrix(), 1e-12),
            "transform composition/inverse contract failed");

    std::cout << "LANDMARK_CONFIG_VALIDATION=PASS\n"
              << "LANDMARK_CORNER_ORDER_AUDIT=PASS\n"
              << "LANDMARK_BOARD_NORMAL_AUDIT=PASS\n"
              << "LANDMARK_VIEW_ANGLE_GEOMETRY=PASS\n"
              << "LANDMARK_VISIBLE_HULL_AREA=PASS\n"
              << "LANDMARK_COVARIANCE_TREND=PASS\n"
              << "LANDMARK_PNP_AMBIGUITY_DIAGNOSTICS=PASS\n"
              << "LANDMARK_TRANSFORM_CONVENTION=PASS\n";
    return EXIT_SUCCESS;
  }
  catch (const std::exception &exception)
  {
    std::cerr << "landmark_frontend_self_test: FAIL: "
              << exception.what() << '\n';
    return EXIT_FAILURE;
  }
}
