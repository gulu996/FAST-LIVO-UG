// Diagnostic-only, exact linked-OpenCV replay of the legacy adapter on
// lossless bag frames. No production state or geometry is modified.
#include "landmark_legacy_adapter.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace
{
struct Frame
{
  std::string bag;
  double relative_time = 0.0;
  int id = -1;
  double raw_time = 0.0;
  std::string image_path;
  double runtime_rmse = 0.0;
  std::string runtime_slots;
};

bool parseFrame(const std::string &line, Frame *frame)
{
  std::istringstream in(line);
  std::string fields[7];
  for (auto &field : fields)
    if (!std::getline(in, field, ',')) return false;
  if (!fields[6].empty() && fields[6].back() == '\r') fields[6].pop_back();
  try
  {
    frame->bag = fields[0];
    frame->relative_time = std::stod(fields[1]);
    frame->id = std::stoi(fields[2]);
    frame->raw_time = std::stod(fields[3]);
    frame->image_path = fields[4];
    frame->runtime_rmse = std::stod(fields[5]);
    frame->runtime_slots = fields[6];
    return true;
  }
  catch (...) { return false; }
}

landmark::LandmarkFrontend slotFrontend()
{
  landmark::LandmarkDefinition board;
  board.landmark_id = 0;
  const std::array<cv::Point2d, 4> centers = {{
      {-0.28, 0.18}, {0.28, 0.18}, {-0.28,-0.18}, {0.28,-0.18}}};
  for (int slot = 0; slot < 4; ++slot)
  {
    landmark::MarkerGeometry marker;
    marker.marker_id = slot;
    marker.size_m = 0.16;
    marker.T_landmark_marker.translation() = Eigen::Vector3d(
        centers[slot].x, centers[slot].y, 0.0);
    board.markers.push_back(marker);
  }
  landmark::LandmarkFrontend frontend;
  std::string error;
  if (!frontend.configure({board}, &error))
    throw std::runtime_error(error);
  return frontend;
}

std::vector<cv::Point3f> slotPoints(int slot)
{
  const std::array<cv::Point2d, 4> centers = {{
      {-0.28, 0.18}, {0.28, 0.18}, {-0.28,-0.18}, {0.28,-0.18}}};
  landmark::MarkerGeometry marker;
  marker.marker_id = slot;
  marker.size_m = 0.16;
  marker.T_landmark_marker.translation() = Eigen::Vector3d(
      centers[slot].x, centers[slot].y, 0.0);
  return landmark::LandmarkFrontend::markerCornersInLandmark(marker);
}

double blurVariance(const cv::Mat &gray, const std::vector<cv::Point2f> &points)
{
  const cv::Rect image_rect(0, 0, gray.cols, gray.rows);
  const cv::Rect board_rect = cv::boundingRect(points) & image_rect;
  if (board_rect.empty()) return 0.0;
  cv::Mat laplacian;
  cv::Laplacian(gray(board_rect), laplacian, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(laplacian, mean, stddev);
  return stddev[0] * stddev[0];
}
} // namespace

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: landmark_legacy_pnp_probe frame_manifest.csv output_dir\n";
    return 2;
  }
  std::ifstream manifest(argv[1]);
  const std::string out_dir = argv[2];
  std::ofstream corner_file(out_dir + "/legacy_corner_residuals.csv");
  std::ofstream frame_file(out_dir + "/legacy_frame_summary.csv");
  std::ofstream assignment_file(out_dir + "/legacy_assignment_candidates.csv");
  std::ofstream object_file(out_dir + "/legacy_object_points.csv");
  if (!manifest || !corner_file || !frame_file || !assignment_file || !object_file)
  {
    std::cerr << "cannot open audit manifest or output files\n";
    return 2;
  }
  corner_file << "bag,timestamp,landmark_id,slot,marker_detection_index,corner_index,"
                 "observed_u,observed_v,projected_u,projected_v,du,dv,error_px\n";
  frame_file << "bag,timestamp,landmark_id,rmse_px,runtime_rmse_px,"
                "runtime_selected_slots,probe_selected_slots,assignment_gap_px,"
                "assignment_ratio,radial_distance_px,view_angle_deg,distance_m,"
                "visible_area_px2,median_marker_side_px,board_roi_laplacian_variance,"
                "board_roi_saturated_fraction\n";
  assignment_file << "bag,timestamp,landmark_id,assignment,rmse_px,pose_valid\n";
  object_file << "slot,corner_index,x_m,y_m,z_m\n";
  for (int slot = 0; slot < 4; ++slot)
  {
    const auto points = slotPoints(slot);
    for (int corner = 0; corner < 4; ++corner)
      object_file << slot << ',' << corner << ',' << points[corner].x << ','
                  << points[corner].y << ',' << points[corner].z << '\n';
  }

  const cv::Mat K = (cv::Mat_<float>(3, 3) <<
      1273.525961550808, 0, 612.4210788294631,
      0, 1277.522948544942, 492.3065791444537,
      0, 0, 1);
  const cv::Mat distortion = (cv::Mat_<float>(1, 5) <<
      -0.11113915388551408, 0.1861858287554102,
      -0.0007605139024049476, -0.001935749448109755, 0);
  landmark::LegacySameIdBoardAdapter adapter;
  std::string error;
  if (!adapter.configure({1, 2, 3, 4}, {0.16, 0.28, 0.18},
                         {}, {}, &error))
  {
    std::cerr << error << '\n';
    return 2;
  }
  const auto frontend = slotFrontend();
  const auto dictionary = cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_6X6_250);
  const auto parameters = cv::aruco::DetectorParameters::create();
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  parameters->cornerRefinementWinSize = 5;
  parameters->cornerRefinementMaxIterations = 30;
  parameters->cornerRefinementMinAccuracy = 0.1;
  corner_file << std::setprecision(12);
  frame_file << std::setprecision(12);
  assignment_file << std::setprecision(12);
  std::string line;
  std::getline(manifest, line); // header
  int frames = 0;
  int mismatches = 0;
  while (std::getline(manifest, line))
  {
    Frame frame;
    if (!parseFrame(line, &frame)) return 2;
    cv::Mat image = cv::imread(frame.image_path, cv::IMREAD_COLOR);
    if (image.empty()) return 2;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters, rejected);
    std::vector<std::size_t> group;
    for (std::size_t i = 0; i < ids.size(); ++i)
      if (ids[i] == frame.id) group.push_back(i);
    if (group.size() != 4)
    {
      std::cerr << "detector marker count changed at " << frame.relative_time
                << " id=" << frame.id << " count=" << group.size() << '\n';
      ++mismatches;
      continue;
    }
    const auto results = adapter.estimate(frame.relative_time, ids, corners,
                                          K, distortion);
    const auto result_it = std::find_if(results.begin(), results.end(),
        [&](const landmark::LegacyAssociationResult &r) {
          return r.diagnostic.landmark_id == frame.id;
        });
    if (result_it == results.end()) return 2;
    const auto &result = *result_it;
    const auto &obs = result.observation;
    const auto &diag = result.diagnostic;
    std::ostringstream slots;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    std::vector<double> side_lengths;
    for (std::size_t j = 0; j < group.size(); ++j)
    {
      const int slot = diag.selected_slots[j];
      if (j) slots << '|';
      slots << slot;
      const auto points = slotPoints(slot);
      object_points.insert(object_points.end(), points.begin(), points.end());
      image_points.insert(image_points.end(), corners[group[j]].begin(),
                          corners[group[j]].end());
      for (int corner = 0; corner < 4; ++corner)
        side_lengths.push_back(cv::norm(corners[group[j]][corner] -
            corners[group[j]][(corner + 1) % 4]));
    }
    const Eigen::Matrix3d R = obs.T_camera_landmark.linear();
    cv::Mat R_cv(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        R_cv.at<double>(row, col) = R(row, col);
    cv::Vec3d rvec, tvec(obs.T_camera_landmark.translation().x(),
                         obs.T_camera_landmark.translation().y(),
                         obs.T_camera_landmark.translation().z());
    cv::Rodrigues(R_cv, rvec);
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, K, distortion, projected);
    cv::Mat overlay = image.clone();
    for (std::size_t j = 0; j < group.size(); ++j)
      for (int c = 0; c < 4; ++c)
      {
        const std::size_t index = 4 * j + c;
        const cv::Point2f delta = projected[index] - image_points[index];
        corner_file << frame.bag << ',' << frame.relative_time << ','
                    << frame.id << ',' << diag.selected_slots[j] << ','
                    << group[j] << ',' << c << ',' << image_points[index].x
                    << ',' << image_points[index].y << ',' << projected[index].x
                    << ',' << projected[index].y << ',' << delta.x << ','
                    << delta.y << ',' << cv::norm(delta) << '\n';
        cv::line(overlay, image_points[index], projected[index],
                 cv::Scalar(0, 255, 255), 1);
        cv::circle(overlay, image_points[index], 3,
                   cv::Scalar(0, 255, 0), -1);
        cv::circle(overlay, projected[index], 3,
                   cv::Scalar(0, 0, 255), -1);
      }
    const cv::Rect roi = cv::boundingRect(image_points) &
        cv::Rect(0, 0, gray.cols, gray.rows);
    const double saturated = roi.empty() ? 0.0 :
        static_cast<double>(cv::countNonZero(gray(roi) >= 245)) / roi.area();
    std::sort(side_lengths.begin(), side_lengths.end());
    const cv::Point2f center = std::accumulate(image_points.begin(),
        image_points.end(), cv::Point2f(0, 0)) *
        (1.0f / static_cast<float>(image_points.size()));
    const double radial = cv::norm(center - cv::Point2f(
        K.at<float>(0, 2), K.at<float>(1, 2)));
    frame_file << frame.bag << ',' << frame.relative_time << ',' << frame.id
               << ',' << obs.reprojection_rmse_px << ',' << frame.runtime_rmse
               << ',' << frame.runtime_slots << ',' << slots.str() << ','
               << diag.assignment_gap_px << ',' << diag.assignment_ratio << ','
               << radial << ',' << obs.view_angle_deg << ','
               << obs.estimated_distance_m << ','
               << obs.visible_corner_hull_area_px2 << ','
               << side_lengths[side_lengths.size() / 2] << ','
               << blurVariance(gray, image_points) << ',' << saturated << '\n';
    cv::putText(overlay, "ID " + std::to_string(frame.id) +
        " RMSE " + std::to_string(obs.reprojection_rmse_px),
        cv::Point(15, 35), cv::FONT_HERSHEY_SIMPLEX, 0.8,
        cv::Scalar(0, 255, 255), 2);
    const std::string name = frame.bag + "_id" +
        std::to_string(frame.id) + "_t" +
        std::to_string(frame.relative_time) + ".png";
    cv::imwrite(out_dir + "/" + name, overlay);

    std::array<int, 4> permutation{{0, 1, 2, 3}};
    do
    {
      std::vector<std::vector<cv::Point2f>> board_corners;
      for (auto index : group) board_corners.push_back(corners[index]);
      const auto candidates = frontend.estimate(frame.relative_time,
          {permutation[0], permutation[1], permutation[2], permutation[3]},
          board_corners, K, distortion);
      const auto &candidate = candidates.front();
      assignment_file << frame.bag << ',' << frame.relative_time << ','
                      << frame.id << ',' << permutation[0] << '|'
                      << permutation[1] << '|' << permutation[2] << '|'
                      << permutation[3] << ',' << candidate.reprojection_rmse_px
                      << ',' << candidate.pose_valid << '\n';
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    ++frames;
  }
  std::cout << "AUDITED_FRAMES=" << frames
            << " DETECTOR_COUNT_MISMATCH=" << mismatches << '\n';
  return mismatches == 0 ? 0 : 1;
}
