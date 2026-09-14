// Link with the existing vio library and its normal ROS/OpenCV/vikit dependencies.
#include "vio.h"
#include <ros/master.h>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

void checkPatchScratchInitialization(ros::NodeHandle &nh)
{
  cv::Mat image(128, 128, CV_8UC1, cv::Scalar(100));
  vk::PinholeCamera camera(128, 128, 1.0, 40, 40, 64, 64);
  for (int levels : {1, 2, 3, 4})
  {
    VIOManager vio;
    vio.cam = &camera;
    vio.setImuToLidarExtrinsic(V3D::Zero(), M3D::Identity());
    std::vector<double> camera_rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::vector<double> camera_translation{0, 0, 0};
    vio.setLidarToCameraExtrinsic(camera_rotation, camera_translation);
    vio.aruco_landmarks_en = false;
    vio.raycast_en = false;
    vio.colmap_output_en = false;
    vio.grid_size = 16;
    vio.patch_size = 8;
    vio.patch_pyrimid_level = levels;
    vio.initializeVIO(nh);
    require(vio.Rci.isIdentity() && vio.Pci.isZero(), "initialization used invalid extrinsics");
    require(vio.warp_len == 64 * levels, "scratch fix changed optimizer pyramid storage");
    require(vio.patch_buffer.size() == static_cast<size_t>(64 * std::max(levels, 3)),
            "actual VIO initialization undersized search-level scratch space");
    for (int search_level : {0, 1, 2})
    {
      std::fill(vio.patch_buffer.begin(), vio.patch_buffer.end(), -1.0f);
      vio.getImagePatch(image, V2D(64, 64), vio.patch_buffer.data(), search_level);
      for (size_t i = 0; i < vio.patch_buffer.size(); ++i)
      {
        const bool in_patch = i >= static_cast<size_t>(64 * search_level) &&
            i < static_cast<size_t>(64 * (search_level + 1));
        require(vio.patch_buffer[i] == (in_patch ? 100.0f : -1.0f),
                "search patch overflowed its initialized scratch segment");
      }
    }
  }
  std::cout << "initialization: 1..4 optimizer levels with all search-level scratch segments passed\n";
}

void checkProjectionJacobian()
{
  VIOManager vio;
  double largest_old_model_error = 0.0;
  for (bool distorted : {false, true})
  {
    vk::PinholeCamera camera(1600, 1200, 0.5,
        890.6048325304455, 890.6189647772335, 780.9722805357044, 589.7898852172461,
        distorted ? -0.14707811196570506 : 0.0,
        distorted ? 0.08102897302832712 : 0.0,
        distorted ? -0.00039435857931765405 : 0.0,
        distorted ? 0.0006560130981826822 : 0.0);
    vio.cam = &camera;
    vio.fx = camera.fx();
    vio.fy = camera.fy();
    for (double depth : {1e-4, 0.25, 1.0, 30.0, 1e4})
      for (const V2D &uv : {V2D(0, 0), V2D(0.8, 0.6), V2D(-0.8, -0.6),
                            V2D(0.75, -0.55), V2D(-0.65, 0.45)})
      {
        const V3D point(depth * uv.x(), depth * uv.y(), depth);
        MD(2, 3) actual, reference, old_pinhole;
        vio.computeProjectionJacobian(point, actual);
        // Independent, fourth-order stencil and step; calls the real camera.
        const double h = 1e-4 * point.cwiseAbs().maxCoeff();
        for (int axis = 0; axis < 3; ++axis)
        {
          V3D offset = V3D::Zero();
          offset[axis] = h;
          const V3D plus = point + offset, minus = point - offset;
          const V3D plus2 = point + 2 * offset, minus2 = point - 2 * offset;
          reference.col(axis) = (-camera.world2cam(plus2) + 8 * camera.world2cam(plus) -
              8 * camera.world2cam(minus) + camera.world2cam(minus2)) / (12 * h);
        }
        require(actual.allFinite() && (actual - reference).norm() / reference.norm() < 1e-7,
                "shared projection Jacobian differs from the actual camera model");
        old_pinhole << camera.fx() / depth, 0, -camera.fx() * uv.x() / depth,
                       0, camera.fy() / depth, -camera.fy() * uv.y() / depth;
        const double old_error = (old_pinhole - reference).norm() / reference.norm();
        if (distorted) largest_old_model_error = std::max(largest_old_model_error, old_error);
        else require(old_error < 1e-8, "zero-distortion reference is inconsistent");
      }
    MD(2, 3) jacobian;
    vio.computeProjectionJacobian(V3D::Zero(), jacobian);
    require(!jacobian.allFinite(), "undefined zero-point projection accepted");
    vio.computeProjectionJacobian(V3D(std::numeric_limits<double>::infinity(), 0, 1), jacobian);
    require(!jacobian.allFinite(), "non-finite projection accepted");
    volatile double checksum = 0.0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 10000; ++i)
    {
      vio.computeProjectionJacobian(V3D(0.65, -0.4, 1.0 + 1e-5 * i), jacobian);
      checksum += jacobian(0, 0);
    }
    const double microseconds = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - start).count() / 10000;
    require(std::isfinite(checksum), "projection benchmark produced invalid values");
    std::cout << "projection " << (distorted ? "radtan" : "undistorted")
              << ": " << microseconds << " us/Jacobian (six actual projections)\n";
  }
  require(largest_old_model_error > 0.08, "projection test would not detect the old pinhole shortcut");
  std::cout << "projection: actual camera, distortion, depth-scale and invalid-input checks passed\n";
}

void checkPhotometricModel()
{
  ExposurePhotometricModel base;
  require(base.initialize(0.7, 1.3), "valid exposure rejected");
  const double residual = base.residual(90, 110);
  const double jacobian = base.exposureJacobian(90, residual);
  const double epsilon = 1e-6;
  ExposurePhotometricModel plus, minus;
  require(plus.initialize(0.7 + epsilon, 1.3) && minus.initialize(0.7 - epsilon, 1.3),
          "finite difference model initialization failed");
  require(std::abs((plus.residual(90, 110) - minus.residual(90, 110)) / (2 * epsilon) - jacobian) < 1e-7,
          "exposure Jacobian omits normalization derivative");
  for (double scale : {1e-5, 1.0, 1e5})
  {
    ExposurePhotometricModel model;
    require(model.initialize(0.7 * scale, 1.3 * scale), "scaled exposure rejected");
    require(std::abs(model.residual(90, 110) - residual) < 1e-10 &&
            std::abs(model.current_weight - base.current_weight) < 1e-12 &&
            std::abs(model.reference_weight - base.reference_weight) < 1e-12 &&
            std::abs(model.exposureJacobian(90, residual) * scale - jacobian) < 1e-8,
            "photometric model depends on common exposure units");
  }
  require(!base.initialize(0, 1) && !base.initialize(-1, 1) &&
          !base.initialize(std::numeric_limits<double>::quiet_NaN(), 1),
          "invalid exposure accepted");

  // Alternating equal-energy, non-identical patches have unchanged brightness.
  // Ordinary one-sided least squares multiplies the gain by cos(theta)<1
  // after each reference replacement; the symmetric objective must not.
  double gain = 1.0;
  double old_gain = 1.0;
  for (int frame = 0; frame < 1000; ++frame)
  {
    const double current[2] = {frame % 2 ? 120.0 : 80.0, frame % 2 ? 80.0 : 120.0};
    const double reference[2] = {current[1], current[0]};
    ExposurePhotometricModel model;
    require(model.initialize(gain, gain), "reference sequence initialization failed");
    double gradient = 0.0, information = 0.0, cross = 0.0, energy = 0.0;
    for (int i = 0; i < 2; ++i)
    {
      const double r = model.residual(current[i], reference[i]);
      const double j = model.exposureJacobian(current[i], r);
      gradient += r * j;
      information += j * j;
      cross += current[i] * reference[i];
      energy += current[i] * current[i];
    }
    gain -= gradient / information;
    old_gain *= cross / energy;
  }
  require(std::abs(gain - 1.0) < 1e-12 && old_gain < 1e-20,
          "reference replacement still drives equal-brightness gain to zero");
  std::cout << "photometric model: derivative, units and reference lifecycle checks passed\n";
}

void checkReferencePatchScores()
{
  VIOManager vio;
  VisualPoint point(V3D(0, 0, 5));
  point.is_normal_initialized_ = true;
  point.normal_ = V3D(0, 0, 1);
  vio.visual_submap = new SubSparseMap;
  vio.visual_submap->voxel_points.push_back(&point);
  vio.total_points = 1;
  vio.patch_size_total = 8;
  vio.update_flag.push_back(1);
  for (int i = 0; i < 6; ++i)
  {
    auto patch = new float[8];
    for (int j = 0; j < 8; ++j) patch[j] = 20 + ((i * 37 + j * j * 11 + i * j * 19) % 190);
    auto feature = new Feature(&point, patch, V2D::Zero(), V3D(0, 0, 1), SE3(), 0);
    feature->id_ = i;
    feature->mean_ = std::accumulate(patch, patch + 8, 0.0) / 8;
    point.addFrameRef(feature);
  }
  const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> empty_plane_map;
  for (int iteration = 0; iteration < 3; ++iteration)
  {
    vio.updateReferencePatch(empty_plane_map);
    for (Feature *feature : point.obs_)
    {
      double expected = 1;
      for (Feature *other : point.obs_)
        if (other != feature) expected += std::abs(vio.calculateNCC(feature->patch_, other->patch_, 8)) / 5;
      require(std::isfinite(feature->score_) && std::abs(feature->score_ - expected) < 1e-6,
              "reference NCC depends on cached means or observation order");
    }
    point.obs_.reverse();
  }
  // Constant patches must score zero correlation, not 0/0.
  for (Feature *feature : point.obs_) std::fill(feature->patch_, feature->patch_ + 8, 40.0f);
  vio.updateReferencePatch(empty_plane_map);
  for (Feature *feature : point.obs_)
    require(std::isfinite(feature->score_) && std::abs(feature->score_ - 1) < 1e-6,
            "constant reference patch produced invalid NCC");
  std::cout << "reference NCC: real caller, cached means, order and constant-patch checks passed\n";
}

bool checkVisualVoxelLookup(double voxel_size, double x, bool raycast)
{
  cv::Mat image(64, 64, CV_8UC1, cv::Scalar(100));
  vk::PinholeCamera camera(64, 64, 1.0, 20, 20, 32, 32);
  StatesGroup state;
  VIOManager vio;
  vio.visual_submap = new SubSparseMap;
  vio.cam = &camera;
  vio.state = &state;
  vio.new_frame_.reset(new Frame(&camera, image));
  vio.width = vio.height = 64;
  vio.grid_size = 16;
  vio.grid_n_width = vio.grid_n_height = 4;
  vio.length = 16;
  vio.border = 4;
  vio.patch_size_half = 1;
  vio.warp_len = 4;
  vio.normal_en = true;
  vio.raycast_en = raycast;
  vio.visual_voxel_size = voxel_size;
  vio.visual_map_fov_fallback_en = false;
  vio.visual_map_supply_diagnostics_en = true;
  vio.grid_num.resize(vio.length);
  vio.map_index.resize(vio.length);
  vio.map_dist.resize(vio.length);
  vio.scan_value.resize(vio.length);
  vio.border_flag.assign(vio.length, 0);
  vio.rays_with_sample_points.resize(vio.length);
  vio.resetGrid();

  const V3D position(x, 0, 4);
  auto *point = new VisualPoint(position);
  point->normal_ = V3D(0, 0, 1);
  // ponytail: stop after actual spatial retrieval; no photometric update is
  // needed to test the voxel address contract. Full image tracking is tested separately.
  point->is_normal_initialized_ = false;
  point->addFrameRef(new Feature(point, new float[4]{100, 100, 100, 100},
                                camera.world2cam(position), position.normalized(), SE3(), 0));
  if (!vio.insertPointIntoVoxelMap(point)) return false;
  const VOXEL_LOCATION expected(std::floor(x / voxel_size), 0, std::floor(4 / voxel_size));
  const bool floor_key = vio.feat_map.count(expected) == 1;
  std::vector<pointWithVar> pg;
  if (raycast) vio.rays_with_sample_points[0].push_back(position);
  else
  {
    pointWithVar measurement;
    measurement.point_w = position;
    pg.push_back(measurement);
  }
  const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> empty_plane_map;
  vio.retrieveFromVisualSparseMap(image, pg, empty_plane_map);
  const bool retrieved = vio.last_visual_projected_candidates == 1 &&
      vio.last_visual_grid_candidates == 1;
  std::cout << "voxel=" << voxel_size << " x=" << x
            << " path=" << (raycast ? "raycast" : "point_cloud")
            << " floor_key=" << floor_key << " retrieved=" << retrieved << '\n';
  return floor_key && retrieved;
}

void checkVisualVoxelBoundaries()
{
  VIOManager vio;
  vio.visual_submap = new SubSparseMap;
  VOXEL_LOCATION key;
  for (double size : {0.5, 0.25, 0.7})
  {
    vio.visual_voxel_size = size;
    for (double cell : {0.0, 1.0, -1.0, -1.25, -0.75})
    {
      const V3D point = V3D::Constant(cell * size);
      const int64_t expected = static_cast<int64_t>(std::floor(point.x() / size));
      require(vio.getVisualVoxelLocation(point, key) &&
              key.x == expected && key.y == expected && key.z == expected,
              "visual voxel floor convention changed at zero/negative cell boundary");
    }
  }
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (double size : {0.0, -1.0, 1e-7, inf, nan})
  {
    vio.visual_voxel_size = size;
    require(!vio.getVisualVoxelLocation(V3D::Zero(), key),
            "invalid visual voxel size accepted");
    require(!vio.insertPointIntoVoxelMap(new VisualPoint(V3D::Zero())) && vio.feat_map.empty(),
            "invalid visual voxel size inserted a map point");
  }
  vio.visual_voxel_size = 1.0;
  const double lower = static_cast<double>(std::numeric_limits<int64_t>::min());
  const double upper = -lower;
  require(vio.getVisualVoxelLocation(V3D(lower, 0, 0), key) &&
          key.x == std::numeric_limits<int64_t>::min(), "valid int64 minimum rejected");
  require(vio.getVisualVoxelLocation(V3D(std::nextafter(upper, 0.0), 0, 0), key),
          "largest representable in-range voxel key rejected");
  for (int axis = 0; axis < 3; ++axis)
    for (double invalid : {upper, std::nextafter(lower, -inf), inf, -inf, nan})
    {
      V3D point = V3D::Zero();
      point[axis] = invalid;
      require(!vio.getVisualVoxelLocation(point, key),
              "out-of-range or non-finite visual voxel index accepted");
    }
  std::cout << "visual voxel key: floor, configured size, finite and int64 bounds passed\n";
}

bool checkVisualSearchScale(int magnification)
{
  constexpr int width = 128, center = width / 2;
  cv::Mat reference_image(width, width, CV_8UC1);
  for (int y = 0; y < width; ++y)
    for (int x = 0; x < width; ++x)
      reference_image.at<unsigned char>(y, x) = 25 + ((x * x * 17 + y * y * 29 + x * y * 11) % 200);
  cv::Mat current_image;
  const cv::Mat magnify = (cv::Mat_<double>(2, 3) << magnification, 0, center * (1 - magnification),
                                                     0, magnification, center * (1 - magnification));
  cv::warpAffine(reference_image, current_image, magnify, reference_image.size(), cv::INTER_LINEAR);
  vk::PinholeCamera camera(width, width, 1.0, 40, 40, center, center);
  StatesGroup state;
  VIOManager vio;
  vio.visual_submap = new SubSparseMap;
  vio.cam = &camera;
  vio.state = &state;
  vio.new_frame_.reset(new Frame(&camera, current_image));
  vio.new_frame_->T_f_w_ = SE3(M3D::Identity(), V3D(0, 0, -(4.0 - 4.0 / magnification)));
  vio.width = vio.height = width;
  vio.grid_size = 16;
  vio.grid_n_width = vio.grid_n_height = 8;
  vio.length = 64;
  vio.border = 20;
  vio.patch_size = 8;
  vio.patch_size_half = 4;
  vio.patch_size_total = 64;
  vio.patch_pyrimid_level = 3;
  vio.warp_len = 64 * 3;
  vio.patch_buffer.resize(vio.warp_len);
  vio.normal_en = true;
  vio.raycast_en = false;
  vio.visual_voxel_size = 0.5;
  vio.visual_map_fov_fallback_en = false;
  vio.visual_map_supply_diagnostics_en = false;
  vio.visual_patch_quality_gate_en = true;
  vio.ncc_en = false;
  vio.outlier_threshold = 1000;
  vio.console_timing_print_en = false;
  vio.grid_num.resize(vio.length);
  vio.map_index.resize(vio.length);
  vio.map_dist.resize(vio.length);
  vio.scan_value.resize(vio.length);
  vio.border_flag.assign(vio.length, 0);
  vio.rays_with_sample_points.resize(vio.length);
  vio.resetGrid();

  const V3D position(0, 0, 4);
  auto *point = new VisualPoint(position);
  point->normal_ = V3D(0, 0, 1);
  point->is_normal_initialized_ = true;
  auto *patch = new float[64];
  vio.getImagePatch(reference_image, V2D(center, center), patch, 0);
  auto *feature = new Feature(point, patch, V2D(center, center), V3D(0, 0, 1), SE3(), 0);
  feature->img_ = reference_image;
  feature->id_ = 0;
  point->addFrameRef(feature);
  if (!vio.insertPointIntoVoxelMap(point)) return false;
  pointWithVar measurement;
  measurement.point_w = position;
  std::vector<pointWithVar> pg{measurement};
  const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> empty_plane_map;

  // Independent footprint check through the real affine and sampling helpers.
  Matrix2d warp;
  vio.getWarpMatrixAffineHomography(camera, feature->px_, position, point->normal_,
      vio.new_frame_->T_f_w_, 0, warp);
  const int search_level = vio.getBestSearchLevel(warp, 2);
  std::vector<float> warped(192), zero_level(192), matched_level(192);
  vio.warpAffine(warp, reference_image, feature->px_, 0, search_level, 0, 4, warped.data());
  vio.getImagePatch(current_image, feature->px_, zero_level.data(), 0);
  vio.getImagePatch(current_image, feature->px_, matched_level.data(), search_level);
  double wrong_mse = 0, correct_mse = 0;
  for (int i = 0; i < 64; ++i)
  {
    wrong_mse += std::pow(zero_level[i] - warped[i], 2) / 64;
    correct_mse += std::pow(matched_level[64 * search_level + i] - warped[i], 2) / 64;
  }
  vio.retrieveFromVisualSparseMap(current_image, pg, empty_plane_map);
  const int expected_level = magnification == 4 ? 2 : magnification == 2 ? 1 : 0;
  const bool passed = search_level == expected_level && correct_mse < 1e-8 &&
      (magnification == 1 || wrong_mse > vio.outlier_threshold) &&
      vio.last_visual_grid_candidates == 1 && vio.total_points == 1 &&
      vio.visual_submap->search_levels.at(0) == expected_level &&
      vio.visual_submap->photometric_mses.at(0) < 1e-8;
  std::cout << "magnification=" << magnification << " level=" << search_level
            << " wrong_mse=" << wrong_mse << " correct_mse=" << correct_mse
            << " grid=" << vio.last_visual_grid_candidates << " tracked=" << vio.total_points
            << " photo_rejected=" << vio.last_visual_photometric_rejects << " pass=" << passed << '\n';
  return passed;
}

void check(bool inverse)
{
  cv::Mat image(64, 64, CV_8UC1);
  for (int y = 0; y < image.rows; ++y)
    for (int x = 0; x < image.cols; ++x)
      image.at<unsigned char>(y, x) = 30 + x + y;
  vk::PinholeCamera camera(64, 64, 1.0, 20, 20, 32, 32,
      -0.14707811196570506, 0.08102897302832712,
      -0.00039435857931765405, 0.0006560130981826822);
  VisualPoint point(V3D(0, 0, 4));
  Feature reference(&point, new float[4], V2D(32, 32), V3D(0, 0, 1), SE3(), 0);
  reference.img_ = image.clone();
  point.ref_patch = &reference;
  StatesGroup state, propagated;
  VIOManager vio;
  vio.visual_submap = new SubSparseMap;
  vio.state = &state;
  vio.state_propagat = &propagated;
  vio.cam = &camera;
  vio.new_frame_.reset(new Frame(&camera, image));
  vio.Rci.setIdentity();
  vio.Pci.setZero();
  vio.Jdphi_dR.setIdentity();
  vio.Jdp_dR.setZero();
  vio.fx = vio.fy = 20;
  vio.cx = vio.cy = 32;
  vio.total_points = 1;
  vio.patch_size = 2;
  vio.patch_size_half = 1;
  vio.patch_size_total = 4;
  vio.patch_pyrimid_level = 2;
  vio.max_iterations = 1;
  vio.min_update_meas = 1;
  vio.img_point_cov = 100;
  vio.exposure_estimate_en = false;
  vio.inverse_composition_en = inverse;
  vio.visual_robust_kernel_en = false;
  vio.visual_observability_gate_en = false;
  vio.has_ref_patch_cache = false;
  vio.visual_submap->voxel_points.push_back(&point);
  vio.visual_submap->search_levels.push_back(0);
  vio.visual_submap->inv_expo_list.push_back(1.0);
  vio.visual_submap->errors.resize(1);
  std::vector<float> patch(8);
  for (int level = 0; level < 2; ++level)
    vio.getImagePatch(image, V2D(32, 32), patch.data(), level);
  for (float &value : patch) value -= 5;
  vio.visual_submap->warp_patch.push_back(patch);

  const StatesGroup before = state;
  vio.evaluateVisualAdaptiveCovarianceShadow(image);
  require(vio.last_visual_adaptive_covariance_shadow.valid &&
          vio.last_visual_adaptive_covariance_shadow.measurement_dof == 4 &&
          (state - before).norm() == 0 && (state.cov - before.cov).norm() == 0,
          "shadow caller failed or changed state with the actual distorted camera");
  const auto one_level = [&](int level) {
    vio.has_ref_patch_cache = false;
    return inverse ? vio.updateStateInverse(image, level) : vio.updateState(image, level);
  };
  require(one_level(1), "coarse level must really update before rollback test");
  require((state - before).norm() > 1e-8, "coarse test update is ineffective");

  state = before;
  // A valid coarse update followed by an invalid fine residual must restore
  // pose, velocity, biases, exposure, covariance, gain and cached frame pose.
  vio.visual_submap->warp_patch[0][0] = std::numeric_limits<float>::quiet_NaN();
  require(!vio.computeJacobianAndUpdateEKF(image), "invalid fine level was committed");
  require(vio.last_visual_numerical_rejected, "missing numerical rejection diagnostic");
  require((state - before).norm() == 0 && (state.cov - before.cov).norm() == 0,
          "coarse state or covariance survived fine-level rejection");
  require(vio.G.norm() == 0 && vio.H_T_H.norm() == 0, "rejected frame retained information");
  require(vio.new_frame_->T_f_w_.translation().norm() == 0,
          "visual map would use a rejected cached frame pose");

  // The coarse stencil still sees gradients outside this constant centre;
  // the fine stencil is unobservable after the coarse update.
  const cv::Mat ramp = image.clone();
  image(cv::Rect(29, 29, 7, 7)).setTo(94);
  reference.img_ = image.clone();
  for (int level = 0; level < 2; ++level)
    vio.getImagePatch(image, V2D(32, 32), vio.visual_submap->warp_patch[0].data(), level);
  for (float &value : vio.visual_submap->warp_patch[0]) value -= 5;
  vio.visual_observability_gate_en = true;
  require(one_level(1), "observable coarse stencil must update");
  require((state - before).norm() > 1e-8, "observable coarse update is ineffective");
  state = before;
  require(!vio.computeJacobianAndUpdateEKF(image), "unobservable fine level was committed");
  require(vio.last_visual_observability_rejected, "missing observability rejection diagnostic");
  require((state - before).norm() == 0 && (state.cov - before.cov).norm() == 0 &&
          vio.G.norm() == 0, "unobservable fine level retained coarse update");
  image = ramp;
  reference.img_ = image.clone();
  vio.visual_observability_gate_en = false;

  // Force an uphill first trial with an intentionally reversed derivative.
  // The second evaluation must undo BOTH its state and information gain.
  vio.visual_submap->warp_patch[0] = patch;
  state = before;
  state.cov.setIdentity();
  state.cov *= 1e-12;
  state.cov.block<3, 3>(0, 0).setIdentity();
  propagated = state;
  const StatesGroup uphill_before = state;
  vio.Jdphi_dR = -M3D::Identity();
  if (inverse) reference.img_ = 255 - image;
  vio.G.setIdentity();
  vio.G *= 0.25;
  const auto gain_before = vio.G;
  vio.max_iterations = 2;
  require(!one_level(0), "uphill first trial incorrectly reports an accepted update");
  require((state - uphill_before).norm() == 0, "uphill trial state was not restored");
  require((vio.G - gain_before).norm() == 0, "uphill trial retained its rejected gain");
  require((state.cov - uphill_before.cov).norm() == 0, "level update altered covariance");
  std::cout << (inverse ? "inverse" : "forward") << ": transaction checks passed\n";

  // Exercise the actual two-level estimator, including Huber weights and
  // posterior covariance. Exposure and its covariance/cross-covariance must
  // change units together; pose and NIS must not change at all.
  reference.img_ = image.clone();
  vio.Jdphi_dR.setIdentity();
  vio.max_iterations = 1;
  vio.exposure_estimate_en = true;
  vio.visual_robust_kernel_en = true;
  for (float &value : vio.visual_submap->warp_patch[0]) value -= 35;
  StatesGroup expected;
  double expected_nis = 0.0;
  for (double scale : {1.0, 1e-5, 1e5})
  {
    state = before;
    state.cov.setIdentity();
    state.cov *= 0.01;
    state.cov(0, 6) = state.cov(6, 0) = 0.001;
    state.inv_expo_time = scale;
    state.cov.row(6) *= scale;
    state.cov.col(6) *= scale;
    propagated = state;
    reference.inv_expo_time_ = scale;
    vio.visual_submap->inv_expo_list[0] = scale;
    require(vio.computeJacobianAndUpdateEKF(image), "scaled real estimator update rejected");
    state.inv_expo_time /= scale;
    state.cov.row(6) /= scale;
    state.cov.col(6) /= scale;
    if (scale == 1.0)
    {
      expected = state;
      expected_nis = vio.last_visual_normalized_nis;
      require((expected - before).norm() > 1e-8, "scale test did not update state");
    }
    else
    {
      require((state - expected).norm() < 1e-8, "real pose/exposure update depends on gain units");
      require((state.cov - expected.cov).norm() < 1e-9, "real covariance depends on gain units");
      require(std::abs(vio.last_visual_normalized_nis - expected_nis) < 1e-8,
              "real NIS depends on gain units");
    }
  }
  std::cout << (inverse ? "inverse" : "forward") << ": real estimator scale checks passed\n";
}
}

int main(int argc, char **argv)
{
  try
  {
    // Keep the normal mathematical regressions independent of a ROS master.
    // Run this integration check separately with an isolated ROS_MASTER_URI.
    if (argc == 2 && std::string(argv[1]) == "--initialization-check")
    {
      ros::init(argc, argv, "vio_initialization_self_test",
                ros::init_options::AnonymousName | ros::init_options::NoSigintHandler |
                ros::init_options::NoRosout);
      require(ros::master::check(), "initialization check needs an isolated running ROS master");
      ros::NodeHandle nh;
      checkPatchScratchInitialization(nh);
      return 0;
    }
    setStateSnapForDeterminismEnabled(false);
    checkProjectionJacobian();
    checkPhotometricModel();
    checkReferencePatchScores();
    checkVisualVoxelBoundaries();
    for (int magnification : {1, 2, 4})
      require(checkVisualSearchScale(magnification), "visual retrieval compared unequal patch footprints");
    for (bool raycast : {false, true})
      for (double size : {0.5, 0.25, 0.7})
        for (double x : {0.0, 0.25, -0.25, -0.5, -0.500001, -0.499999})
          require(checkVisualVoxelLookup(size, x, raycast), "actual visual voxel lookup mismatch");
    check(false);
    check(true);
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "VIO transaction self-test failed: " << error.what() << '\n';
    return 1;
  }
}
