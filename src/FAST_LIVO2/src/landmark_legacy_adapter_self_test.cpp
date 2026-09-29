#include "landmark_legacy_adapter.h"
#include "landmark_persistent_backend.h"
#include "landmark_shadow_graph.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool ok, const char *message)
{
  if (!ok) throw std::runtime_error(message);
}

std::vector<std::vector<cv::Point2f>> projectedBoard(
    const cv::Mat &K, double distance)
{
  const std::array<Eigen::Vector3d, 4> centers = {{
      {-0.28, 0.18, 0.0}, {0.28, 0.18, 0.0},
      {-0.28,-0.18, 0.0}, {0.28,-0.18, 0.0}}};
  std::vector<std::vector<cv::Point2f>> result;
  for (const auto &center : centers)
  {
    landmark::MarkerGeometry marker;
    marker.size_m = 0.16;
    marker.T_landmark_marker.translation() = center;
    std::vector<cv::Point2f> corners;
    cv::projectPoints(landmark::LandmarkFrontend::markerCornersInLandmark(marker),
                      cv::Vec3d(M_PI, 0, 0), cv::Vec3d(0.1, 0.02, distance),
                      K, cv::Mat(), corners);
    result.push_back(corners);
  }
  return result;
}

void checkArucoQuarterTurn(const landmark::LegacySameIdBoardAdapter &adapter)
{
  const auto dictionary = cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_6X6_250);
  const cv::Mat K = (cv::Mat_<double>(3, 3) <<
      800.0, 0.0, 511.5, 0.0, 800.0, 511.5, 0.0, 0.0, 1.0);
  cv::Mat marker, image(1024, 1024, CV_8UC1, cv::Scalar(255));
  cv::aruco::drawMarker(dictionary, 7, 256, marker);
  const std::array<Eigen::Vector3d, 4> centers = {{
      {-0.28, 0.18, 0.0}, {0.28, 0.18, 0.0},
      {-0.28,-0.18, 0.0}, {0.28,-0.18, 0.0}}};
  const std::array<cv::Point2f, 4> source = {{
      {0, 0}, {255, 0}, {255, 255}, {0, 255}}};
  for (const auto &center : centers)
  {
    landmark::MarkerGeometry geometry;
    geometry.size_m = 0.16;
    geometry.T_landmark_marker.translation() = center;
    std::vector<cv::Point2f> destination;
    cv::projectPoints(landmark::LandmarkFrontend::markerCornersInLandmark(geometry),
                      cv::Vec3d(M_PI + 0.15, 0.03, 0.05),
                      cv::Vec3d(0.0, 0.0, 1.5), K, cv::Mat(), destination);
    cv::Mat warped;
    cv::warpPerspective(marker, warped,
        cv::getPerspectiveTransform(source.data(), destination.data()),
        image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
        cv::Scalar(255));
    cv::min(image, warped, image);
  }
  const auto detect = [&](const cv::Mat &input,
                          std::vector<std::vector<cv::Point2f>> *corners) {
    std::vector<int> ids;
    cv::aruco::detectMarkers(input, dictionary, *corners, ids);
    require(ids == std::vector<int>(4, 7),
            "synthetic four-marker detector IDs changed");
  };
  std::vector<std::vector<cv::Point2f>> original;
  detect(image, &original);
  for (int turns = 0; turns < 4; ++turns)
  {
    cv::Mat turned = image.clone();
    for (int i = 0; i < turns; ++i)
      cv::rotate(turned, turned, cv::ROTATE_90_CLOCKWISE);
    std::vector<std::vector<cv::Point2f>> detected;
    detect(turned, &detected);
    for (const auto &old_marker : original)
    {
      cv::Point2f expected_center =
          (old_marker[0] + old_marker[2]) * 0.5f;
      for (int i = 0; i < turns; ++i)
        expected_center = cv::Point2f(1023.0f - expected_center.y,
                                       expected_center.x);
      const auto found = std::find_if(detected.begin(), detected.end(),
          [&](const std::vector<cv::Point2f> &candidate) {
            return cv::norm((candidate[0] + candidate[2]) * 0.5f -
                            expected_center) < 2.0f;
          });
      require(found != detected.end(), "rotated marker was not detected");
      for (int corner = 0; corner < 4; ++corner)
      {
        cv::Point2f expected = old_marker[corner];
        for (int i = 0; i < turns; ++i)
          expected = cv::Point2f(1023.0f - expected.y, expected.x);
        require(cv::norm((*found)[corner] - expected) < 2.0f,
                "quarter-turn ArUco semantic corner order changed");
      }
    }
    const auto observation = adapter.estimate(2.4 + turns, {7, 7, 7, 7},
                                              detected, K, cv::Mat());
    if (observation.size() != 1 ||
        observation[0].diagnostic.assignment_count != 24 ||
        !observation[0].diagnostic.factor_admitted ||
        observation[0].observation.reprojection_rmse_px >= 2.0)
      throw std::runtime_error("quarter-turn synthetic board joint PnP failed " +
          std::to_string(turns) + " rmse=" +
          (observation.empty() ? "none" : std::to_string(
              observation[0].observation.reprojection_rmse_px)) + " reason=" +
          (observation.empty() ? "none" : observation[0].diagnostic.reason) +
          " slots=" + (observation.empty() ? "none" :
              std::to_string(observation[0].diagnostic.selected_slots[0])));
  }
}
} // namespace

int main()
{
  try
  {
    const cv::Mat K = (cv::Mat_<double>(3, 3) <<
        800.0, 0.0, 640.0, 0.0, 800.0, 512.0, 0.0, 0.0, 1.0);
    landmark::LegacySameIdBoardAdapter adapter;
    landmark::LegacyBoardGeometry geometry{0.16, 0.28, 0.18};
    landmark::PoseValidationOptions pose_options;
    pose_options.max_distance_m = 100.0;
    landmark::UncertaintyOptions uncertainty;
    std::string error;
    require(adapter.configure({7}, geometry, pose_options, uncertainty, &error),
            "legacy adapter configure failed");
    checkArucoQuarterTurn(adapter);
    const auto board = projectedBoard(K, 3.0);
    const std::vector<int> ids(4, 7);
    const std::vector<std::vector<cv::Point2f>> shuffled = {
        board[2], board[0], board[3], board[1]};
    const auto four = adapter.estimate(1.0, ids, shuffled, K, cv::Mat());
    require(four.size() == 1 && four[0].diagnostic.assignment_count == 24 &&
                four[0].diagnostic.factor_admitted &&
                four[0].diagnostic.selected_slots ==
                    std::array<int, 4>{{2, 0, 3, 1}} &&
                four[0].diagnostic.second_assignment_rmse_px >
                    four[0].diagnostic.best_rmse_px + 1.0,
            "four same-ID marker permutation or wrong-assignment RMSE failed");
    const auto three = adapter.estimate(1.2, {7, 7, 7},
        {board[2], board[0], board[3]}, K, cv::Mat());
    require(three.size() == 1 && three[0].diagnostic.assignment_count == 24 &&
                three[0].diagnostic.factor_admitted,
            "three-marker partial association failed");
    const auto two = adapter.estimate(1.4, {7, 7},
        {board[2], board[0]}, K, cv::Mat());
    require(two.size() == 1 && two[0].diagnostic.assignment_count == 12 &&
                !two[0].diagnostic.factor_admitted &&
                two[0].diagnostic.reason == "LEGACY_PARTIAL_ASSIGNMENT_POLICY",
            "two-marker diagnostic entered factor graph");
    const auto one = adapter.estimate(1.6, {7}, {board[2]}, K, cv::Mat());
    require(one.size() == 1 && one[0].diagnostic.assignment_count == 4 &&
                !one[0].diagnostic.factor_admitted &&
                one[0].diagnostic.reason == "LEGACY_PARTIAL_ASSIGNMENT_POLICY",
            "one-marker diagnostic entered factor graph");
    const auto distant = projectedBoard(K, 30.0);
    const auto ambiguous = adapter.estimate(2.0, {7, 7, 7},
        {distant[2], distant[0], distant[3]}, K, cv::Mat());
    require(ambiguous.size() == 1 &&
                !ambiguous[0].diagnostic.factor_admitted,
            "weakly separated three-marker assignment was admitted");

    landmark::LandmarkDefinition unique;
    unique.landmark_id = 7;
    for (int i = 0; i < 4; ++i)
    {
      landmark::MarkerGeometry marker;
      marker.marker_id = i;
      marker.size_m = 0.16;
      unique.markers.push_back(marker);
      unique.markers.back().T_landmark_marker.translation() =
          std::array<Eigen::Vector3d, 4>{{
              {-0.28, 0.18, 0.0}, {0.28, 0.18, 0.0},
              {-0.28,-0.18, 0.0}, {0.28,-0.18, 0.0}}}[i];
    }
    landmark::LandmarkFrontend formal;
    require(formal.configure({unique}, &error), "formal unique-ID frontend changed");
    require(formal.estimate(1.0, ids, shuffled, K, cv::Mat()).empty(),
            "legacy detections leaked into formal unique-ID mode");
    require(formal.estimate(1.0, {0, 1, 2, 3}, board, K, cv::Mat()).size() == 1,
            "formal unique-ID mode stopped accepting unique children");

    landmark::PersistentBackendConfig config;
    landmark::PersistentLandmarkBackend backend(config, false);
    landmark::GlobalLandmarkInput input;
    input.observation_id = 1;
    input.observation = four[0].observation;
    input.local_pose_timestamp = input.observation.timestamp;
    input.local_pose_covariance.setIdentity();
    input.landmark_initial_guess = input.observation.T_camera_landmark;
    input.landmark_initial_covariance.setIdentity();
    input.camera_extrinsic_id = "legacy_synthetic";
    require(backend.submit(input, &error) && backend.processOne() &&
                backend.graphTelemetry().visual_factor_count == 1 &&
                backend.graphTelemetry().landmark_variable_count == 1,
            "standard legacy observation did not traverse unchanged backend graph");
    landmark::GlobalCorrection correction;
    require(!backend.latestCorrection(&correction) && !correction.valid,
            "legacy replay acquired production correction ownership");
    std::cout << "LEGACY_4MARKER_ASSOCIATION=PASS\n"
              << "ARUCO_90DEG_CORNER_ORDER_AUDIT=PASS\n"
              << "LEGACY_3MARKER_ASSOCIATION=PASS\n"
              << "LEGACY_ASSIGNMENT_AMBIGUITY_REJECT=PASS\n"
              << "LEGACY_2MARKER_FACTOR_DISABLED=PASS\n"
              << "FORMAL_UNIQUE_ID_MODE_PRESERVED=PASS\n"
              << "LEGACY_STANDARD_OBSERVATION_GRAPH_FLOW=PASS\n";
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "LANDMARK_LEGACY_ADAPTER_SELF_TEST=FAIL " << e.what() << '\n';
    return 1;
  }
}
