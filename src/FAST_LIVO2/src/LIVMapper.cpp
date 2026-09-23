/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include "gnss_fusion_policy.h"
#include "livo_point_time_split.h"
#include "run_log_directory.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cctype>
#include <filesystem>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <limits>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>

namespace fs = std::filesystem;

namespace
{
std::string formatLocalTime(const char *fmt)
{
  const std::time_t now = std::time(nullptr);
  std::tm tm_now;
  localtime_r(&now, &tm_now);
  char buf[64] = {0};
  std::strftime(buf, sizeof(buf), fmt, &tm_now);
  return std::string(buf);
}

std::string currentTimeForFilename()
{
  return formatLocalTime("%Y-%m-%d-%H%M%S");
}

std::string ensureTrailingSlash(std::string path)
{
  if (!path.empty() && path.back() != '/') path += '/';
  return path;
}

std::string makeRunOutputDir(const std::string &base_dir, bool &ready)
{
  ready = false;
  const std::string normalized_base = ensureTrailingSlash(base_dir);
  const std::string run_dir = normalized_base + currentTimeForFilename();
  std::error_code ec;
  fs::create_directories(run_dir, ec);
  if (ec)
  {
    std::cerr << "[ LIVMapper ] Warning: failed to create output dir: "
              << run_dir << " (" << ec.message() << ")" << std::endl;
    return normalized_base;
  }
  ready = true;
  return ensureTrailingSlash(run_dir);
}

size_t countOctreeNodes(const VoxelOctoTree *node)
{
  if (node == nullptr) return 0;
  size_t count = 1;
  for (const VoxelOctoTree *child : node->leaves_) count += countOctreeNodes(child);
  return count;
}
}

LIVMapper::LIVMapper(ros::NodeHandle &nh)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(nh);
  GnssFusionPolicy fusion_policy;
  if (!loadGnssFusionPolicy(nh, fusion_policy))
    throw std::runtime_error("missing GNSS fusion unified enable parameter");
  gnss_fusion_fixed_lag_mode_ = fusion_policy.managed && fusion_policy.enabled;
  if (gnss_fusion_fixed_lag_mode_)
  {
    ROS_WARN("[GNSS_FUSION] RTK fixed-lag mode active: legacy GPS input and "
             "front-end GPS/UWB absolute state updates are suppressed; "
             "/backend/livo_odom_raw remains pure LIVO.");
  }
  VoxelMapConfig voxel_config;
  loadVoxelConfig(nh, voxel_config);
  if (deterministic_debug_en_) voxel_config.deterministic_lio_update_en = true;

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  p_imu->enable_p4_diagnostics(voxel_config.p4_frontend_diagnostics_enable);
  vio_manager.reset(new VIOManager());
  uwb_manager.reset(new UwbManager());
  gnss_manager.reset(new GnssManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  uwb_manager->initialize(nh, save_path);
  gnss_manager->initialize(nh, save_path);
  initializeComponents(nh);
  fast_livo::p4b::HarnessConfig p4b_config;
  p4b_config.enabled = voxel_config.p4b_snapshot_enable;
  nh.param<bool>("p4b_snapshot/run_all_branches",
                 p4b_config.run_all_branches, false);
  nh.param<bool>("p4b_snapshot/pause_input_during_capture",
                 p4b_config.pause_input_during_capture, false);
  nh.param<double>("p4b_snapshot/snapshot_relative_time_s",
                   p4b_config.snapshot_relative_time_s, 895.0);
  nh.param<double>("p4b_snapshot/end_relative_time_s",
                   p4b_config.end_relative_time_s, 910.0);
  nh.param<std::string>("p4b_snapshot/output_directory",
                        p4b_config.output_directory,
                        save_path + "p4b_same_snapshot");
  p4b_config.voxel_leaf_size_m = filter_size_surf_min;
  p4b_config.lidar_max_range_m = voxel_lidar_max_range_m_;
  p4b_config.map_sliding_enabled = voxel_config.map_sliding_en;
  p4b_config.map_update_stride = lio_map_update_stride_;
  if (p4b_config.enabled)
  {
    if (!voxel_config.p4b_retain_support_points)
      throw std::runtime_error(
          "P4-B requires p4b_snapshot/retain_support_points=true");
    // Diagnostic-only: production and every fork must execute the same
    // correspondence ordering for the restore parity gate.
    voxel_config.deterministic_lio_update_en = true;
    voxelmap_manager->config_setting_.deterministic_lio_update_en = true;
    deterministic_lio_feature_sort_en_ = true;
    p4b_fork_harness_.reset(
        new fast_livo::p4b::P4ForkHarness(p4b_config));
    ROS_WARN("[P4B] diagnostic harness enabled snapshot=%.3f end=%.3f all_branches=%d pause_input=%d output=%s",
             p4b_config.snapshot_relative_time_s,
             p4b_config.end_relative_time_s,
             static_cast<int>(p4b_config.run_all_branches),
             static_cast<int>(p4b_config.pause_input_during_capture),
             p4b_config.output_directory.c_str());
  }
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper()
{
  logRuntimeEventCounts(true);
  if (fout_lio_degeneracy.is_open()) fout_lio_degeneracy.flush();
  if (fout_lio_transaction.is_open()) fout_lio_transaction.flush();
  if (fout_lio_motion_consistency.is_open()) fout_lio_motion_consistency.flush();
  if (fout_runtime_memory.is_open()) fout_runtime_memory.flush();
  if (fout_runtime_events.is_open()) fout_runtime_events.flush();
  if (fout_visual_image_flow.is_open()) fout_visual_image_flow.flush();
  if (fout_livo_scan_contract.is_open()) fout_livo_scan_contract.flush();
  if (gnss_manager) gnss_manager->shutdown();
  if (uwb_manager) uwb_manager->shutdown();
  if (udp_socket_fd_ >= 0)
  {
    ::close(udp_socket_fd_);
    udp_socket_fd_ = -1;
  }
}

void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
  nh.param<int>("common/img_en", img_en, 1);
  nh.param<int>("common/lidar_en", lidar_en, 1);
  nh.param<string>("common/img_topic", img_topic, "/left_camera/image");
  nh.param<int>("common/sub_lidar_queue_size", sub_lidar_queue_size_, 128);
  nh.param<int>("common/sub_imu_queue_size", sub_imu_queue_size_, 512);
  nh.param<int>("common/sub_img_queue_size", sub_img_queue_size_, 16);
  nh.param<int>("common/max_lidar_buffer_size", max_lidar_buffer_size_, 32);
  nh.param<int>("common/max_imu_buffer_size", max_imu_buffer_size_, 3000);
  nh.param<int>("common/max_img_buffer_size", max_img_buffer_size_, 12);
  nh.param<int>("common/max_prop_imu_buffer_size", max_prop_imu_buffer_size_, 3000);
  nh.param<int>("common/sync_img_buffer_min_size", sync_img_buffer_min_size_, 1);
  nh.param<double>("common/sync_img_lookahead_time", sync_img_lookahead_time_, 0.0);
  sync_img_buffer_min_size_ = std::max(1, sync_img_buffer_min_size_);
  sync_img_lookahead_time_ = std::max(0.0, sync_img_lookahead_time_);

  nh.param<bool>("deterministic_debug/en", deterministic_debug_en_, false);
  bool legacy_visual_update_serial = true;
  nh.param<bool>("vio/deterministic_visual_update_en", legacy_visual_update_serial, true);
  nh.param<bool>("deterministic_debug/state_snap_en", deterministic_state_snap_en_, true);
  nh.param<bool>("deterministic_debug/pixel_snap_en", deterministic_pixel_snap_en_, true);
  nh.param<bool>("deterministic_debug/camera_point_snap_en", deterministic_camera_point_snap_en_, true);
  nh.param<bool>("deterministic_debug/contiguous_image_copy_en", deterministic_contiguous_image_copy_en_, true);
  nh.param<bool>("deterministic_debug/visual_update_serial_en", vio_deterministic_visual_update_en_, legacy_visual_update_serial);
  nh.param<bool>("deterministic_debug/imu_accept_out_of_order_en", deterministic_imu_accept_out_of_order_en_, true);
  nh.param<bool>("deterministic_debug/imu_buffer_sort_en", deterministic_imu_buffer_sort_en_, true);
  nh.param<bool>("deterministic_debug/prop_imu_buffer_sort_en", deterministic_prop_imu_buffer_sort_en_, true);
  nh.param<bool>("deterministic_debug/image_buffer_sort_en", deterministic_image_buffer_sort_en_, true);
  nh.param<bool>("deterministic_debug/sync_wait_for_image_lookahead_en", deterministic_sync_wait_for_image_lookahead_en_, true);
  nh.param<bool>("deterministic_debug/pending_vio_image_en", deterministic_pending_vio_image_en_, true);
  nh.param<bool>("deterministic_debug/lio_feature_sort_en", deterministic_lio_feature_sort_en_, true);
  nh.param<bool>("deterministic_debug/visual_observed_voxel_sort_en", deterministic_visual_observed_voxel_sort_en_, true);
  nh.param<bool>("deterministic_debug/visual_voxel_key_sort_en", deterministic_visual_voxel_key_sort_en_, true);
  if (deterministic_debug_en_)
  {
    deterministic_state_snap_en_ = true;
    deterministic_pixel_snap_en_ = true;
    deterministic_camera_point_snap_en_ = true;
    deterministic_contiguous_image_copy_en_ = true;
    vio_deterministic_visual_update_en_ = true;
    deterministic_imu_accept_out_of_order_en_ = true;
    deterministic_imu_buffer_sort_en_ = true;
    deterministic_prop_imu_buffer_sort_en_ = true;
    deterministic_image_buffer_sort_en_ = true;
    deterministic_sync_wait_for_image_lookahead_en_ = true;
    deterministic_pending_vio_image_en_ = true;
    deterministic_lio_feature_sort_en_ = true;
    deterministic_visual_observed_voxel_sort_en_ = true;
    deterministic_visual_voxel_key_sort_en_ = true;
  }
  setStateSnapForDeterminismEnabled(deterministic_state_snap_en_);

  nh.param<bool>("vio/normal_en", normal_en, true);
  nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
  nh.param<int>("vio/max_iterations", max_iterations, 5);
  nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 500.0);
  if (!std::isfinite(IMG_POINT_COV) || IMG_POINT_COV <= 0.0)
    throw std::invalid_argument("vio/img_point_cov must be positive and finite");
  nh.param<bool>("vio/raycast_en", raycast_en, false);
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
  nh.param<bool>("vio/visual_map_prune_en", visual_map_prune_en, true);
  nh.param<bool>("vio/visual_map_supply_diagnostics_en", visual_map_supply_diagnostics_en_, false);
  nh.param<bool>("vio/visual_map_fov_fallback_en", visual_map_fov_fallback_en_, false);
  nh.param<bool>("vio/visual_tracking_only_dry_run_en", visual_tracking_only_dry_run_en_, false);
  nh.param<bool>("vio/visual_shadow_no_commit_en", visual_shadow_no_commit_en_, false);
  nh.param<bool>("vio/visual_adaptive_covariance_shadow_en",
                 visual_adaptive_covariance_shadow_en_, false);
  nh.param<bool>("vio/visual_adaptive_covariance_relaxed_en",
                 visual_adaptive_covariance_relaxed_en_, false);
  nh.param<int>("vio/visual_adaptive_covariance_relaxed_min_points",
                visual_adaptive_covariance_relaxed_min_points_, 15);
  nh.param<double>("vio/visual_adaptive_covariance_k_ncc",
                   visual_adaptive_covariance_k_ncc_, 1.0);
  nh.param<double>("vio/visual_adaptive_covariance_k_level",
                   visual_adaptive_covariance_k_level_, 1.0);
  nh.param<double>("vio/visual_adaptive_covariance_k_photo",
                   visual_adaptive_covariance_k_photo_, 0.5);
  nh.param<double>("vio/visual_adaptive_covariance_scale_max",
                   visual_adaptive_covariance_scale_max_, 4.0);
  nh.param<int>("vio/visual_map_fov_fallback_target_grid_candidates",
                visual_map_fov_fallback_target_grid_candidates_, 60);
  nh.param<int>("vio/visual_map_max_voxels", visual_map_max_voxels, 1800);
  nh.param<int>("vio/visual_map_max_points_per_voxel", visual_map_max_points_per_voxel, 10);
  nh.param<int>("vio/visual_map_max_total_points", visual_map_max_total_points, 20000);
  nh.param<int>("vio/visual_map_max_add_per_frame", visual_map_max_add_per_frame_, 300);
  nh.param<double>("vio/visual_map_min_shi_tomasi_score", visual_map_min_shi_tomasi_score_, 10.0);
  nh.param<int>("vio/grid_size", grid_size, 5);
  nh.param<int>("vio/grid_n_height", grid_n_height, 17);
  nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  nh.param<int>("vio/patch_size", patch_size, 8);
  nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);
  vio_min_retrieve_points_ = 45;
  vio_min_update_meas_ = 900;
  vio_low_track_force_update_stride_ = 0;
  vio_low_track_force_min_points_ = 8;
  vio_max_state_update_rot_deg_ = 0.8;
  vio_max_state_update_trans_m_ = 0.08;
  nh.param<int>("vio/min_retrieve_points", vio_min_retrieve_points_, 45);
  visual_map_fov_fallback_target_grid_candidates_ =
      std::max(vio_min_retrieve_points_, visual_map_fov_fallback_target_grid_candidates_);
  nh.param<bool>("vio/visual_spatial_coverage_gate_en", vio_visual_spatial_coverage_gate_en_, false);
  nh.param<int>("vio/relaxed_min_retrieve_points", vio_relaxed_min_retrieve_points_, 20);
  nh.param<int>("vio/relaxed_min_occupied_good_tiles", vio_relaxed_min_occupied_good_tiles_, 8);
  nh.param<double>("vio/relaxed_min_good_tile_ratio", vio_relaxed_min_good_tile_ratio_, 0.85);
  nh.param<double>("vio/relaxed_min_horizontal_coverage", vio_relaxed_min_horizontal_coverage_, 0.50);
  nh.param<double>("vio/relaxed_min_vertical_coverage", vio_relaxed_min_vertical_coverage_, 0.50);
  nh.param<bool>("vio/degeneracy_relaxed_track_gate_en", vio_degeneracy_relaxed_track_gate_en_, false);
  nh.param<int>("vio/degeneracy_relaxed_min_retrieve_points", vio_degeneracy_relaxed_min_retrieve_points_, 5);
  nh.param<int>("vio/degeneracy_relaxed_min_occupied_good_tiles", vio_degeneracy_relaxed_min_occupied_good_tiles_, 4);
  nh.param<double>("vio/degeneracy_relaxed_min_good_tile_ratio", vio_degeneracy_relaxed_min_good_tile_ratio_, 0.60);
  nh.param<double>("vio/degeneracy_relaxed_min_horizontal_coverage", vio_degeneracy_relaxed_min_horizontal_coverage_, 0.40);
  nh.param<double>("vio/degeneracy_relaxed_min_vertical_coverage", vio_degeneracy_relaxed_min_vertical_coverage_, 0.40);
  nh.param<double>("vio/degeneracy_visual_position_scale", vio_degeneracy_visual_position_scale_, 0.05);
  nh.param<double>("vio/degeneracy_visual_max_position_step_m", vio_degeneracy_visual_max_position_step_m_, 0.003);
  nh.param<bool>("vio/visual_reference_refresh_en", vio_visual_reference_refresh_en_, false);
  nh.param<int>("vio/visual_reference_refresh_max_age_frames", vio_visual_reference_refresh_max_age_frames_, 30);
  nh.param<int>("vio/visual_reference_refresh_min_tracked_points", vio_visual_reference_refresh_min_tracked_points_, 8);
  nh.param<int>("vio/visual_reference_refresh_min_occupied_good_tiles", vio_visual_reference_refresh_min_occupied_good_tiles_, 4);
  nh.param<double>("vio/visual_reference_refresh_min_horizontal_coverage", vio_visual_reference_refresh_min_horizontal_coverage_, 0.30);
  nh.param<double>("vio/visual_reference_refresh_min_vertical_coverage", vio_visual_reference_refresh_min_vertical_coverage_, 0.30);
  nh.param<int>("vio/visual_reference_refresh_max_per_frame", vio_visual_reference_refresh_max_per_frame_, 30);
  nh.param<bool>("vio/visual_search_level_low_contrast_retry_en",
                 vio_visual_search_level_low_contrast_retry_en_, false);
  vio_relaxed_min_retrieve_points_ = std::max(1, vio_relaxed_min_retrieve_points_);
  vio_relaxed_min_occupied_good_tiles_ = std::max(1, vio_relaxed_min_occupied_good_tiles_);
  vio_relaxed_min_good_tile_ratio_ =
      std::min(1.0, std::max(0.0, vio_relaxed_min_good_tile_ratio_));
  vio_relaxed_min_horizontal_coverage_ =
      std::min(1.0, std::max(0.0, vio_relaxed_min_horizontal_coverage_));
  vio_relaxed_min_vertical_coverage_ =
      std::min(1.0, std::max(0.0, vio_relaxed_min_vertical_coverage_));
  vio_degeneracy_relaxed_min_retrieve_points_ = std::max(1, vio_degeneracy_relaxed_min_retrieve_points_);
  vio_degeneracy_relaxed_min_occupied_good_tiles_ = std::max(1, vio_degeneracy_relaxed_min_occupied_good_tiles_);
  vio_degeneracy_relaxed_min_good_tile_ratio_ =
      std::min(1.0, std::max(0.0, vio_degeneracy_relaxed_min_good_tile_ratio_));
  vio_degeneracy_relaxed_min_horizontal_coverage_ =
      std::min(1.0, std::max(0.0, vio_degeneracy_relaxed_min_horizontal_coverage_));
  vio_degeneracy_relaxed_min_vertical_coverage_ =
      std::min(1.0, std::max(0.0, vio_degeneracy_relaxed_min_vertical_coverage_));
  vio_degeneracy_visual_position_scale_ =
      std::min(1.0, std::max(0.0, vio_degeneracy_visual_position_scale_));
  vio_degeneracy_visual_max_position_step_m_ =
      std::max(0.0, vio_degeneracy_visual_max_position_step_m_);
  vio_visual_reference_refresh_max_age_frames_ = std::max(1, vio_visual_reference_refresh_max_age_frames_);
  vio_visual_reference_refresh_min_tracked_points_ = std::max(1, vio_visual_reference_refresh_min_tracked_points_);
  vio_visual_reference_refresh_min_occupied_good_tiles_ = std::max(1, vio_visual_reference_refresh_min_occupied_good_tiles_);
  vio_visual_reference_refresh_min_horizontal_coverage_ =
      std::min(1.0, std::max(0.0, vio_visual_reference_refresh_min_horizontal_coverage_));
  vio_visual_reference_refresh_min_vertical_coverage_ =
      std::min(1.0, std::max(0.0, vio_visual_reference_refresh_min_vertical_coverage_));
  vio_visual_reference_refresh_max_per_frame_ = std::max(1, vio_visual_reference_refresh_max_per_frame_);
  visual_adaptive_covariance_relaxed_min_points_ =
      std::max(15, std::min(29, visual_adaptive_covariance_relaxed_min_points_));
  visual_adaptive_covariance_k_ncc_ = std::max(0.0, visual_adaptive_covariance_k_ncc_);
  visual_adaptive_covariance_k_level_ = std::max(0.0, visual_adaptive_covariance_k_level_);
  visual_adaptive_covariance_k_photo_ = std::max(0.0, visual_adaptive_covariance_k_photo_);
  visual_adaptive_covariance_scale_max_ = std::max(1.0, visual_adaptive_covariance_scale_max_);
  nh.param<int>("vio/min_update_meas", vio_min_update_meas_, 900);
  nh.param<int>("vio/low_track_force_update_stride", vio_low_track_force_update_stride_, 0);
  nh.param<int>("vio/low_track_force_min_points", vio_low_track_force_min_points_, 8);
  nh.param<bool>("vio/visual_update_guard_en", vio_visual_update_guard_en_, true);
  nh.param<double>("vio/visual_update_max_trans_m", vio_visual_update_max_trans_m_, 0.12);
  nh.param<double>("vio/visual_update_max_rot_deg", vio_visual_update_max_rot_deg_, 2.0);
  nh.param<double>("vio/visual_update_max_backward_m", vio_visual_update_max_backward_m_, 0.03);
  nh.param<double>("vio/visual_update_max_backward_ratio", vio_visual_update_max_backward_ratio_, 0.08);
  nh.param<double>("vio/visual_update_backward_abs_floor_m", vio_visual_update_backward_abs_floor_m_, 0.003);
  nh.param<double>("vio/visual_update_max_lateral_m", vio_visual_update_max_lateral_m_, 0.08);
  nh.param<double>("vio/visual_update_max_lateral_ratio", vio_visual_update_max_lateral_ratio_, 0.35);
  nh.param<double>("vio/visual_update_max_exposure_delta", vio_visual_update_max_exposure_delta_, 0.30);
  nh.param<double>("vio/visual_update_max_velocity_increment_mps",
                   vio_visual_update_max_velocity_increment_mps_, 0.15);
  nh.param<double>("vio/visual_update_max_acc_bias_increment_mps2",
                   vio_visual_update_max_acc_bias_increment_mps2_, 0.03);
  nh.param<double>("vio/visual_update_max_gyro_bias_increment_rps",
                   vio_visual_update_max_gyro_bias_increment_rps_, 0.005);
  nh.param<double>("vio/visual_update_normalized_nis_max",
                   vio_visual_update_normalized_nis_max_, 0.0);
  nh.param<bool>("vio/ncc_en", vio_ncc_en_, false);
  nh.param<double>("vio/ncc_threshold", vio_ncc_threshold_, 0.4);
  nh.param<bool>("vio/visual_robust_kernel_en", vio_visual_robust_kernel_en_, true);
  nh.param<double>("vio/visual_robust_delta", vio_visual_robust_delta_, 20.0);
  nh.param<bool>("vio/visual_observability_gate_en", vio_visual_observability_gate_en_, true);
  nh.param<double>("vio/visual_observability_relative_eigen_threshold",
                   vio_visual_observability_relative_eigen_threshold_, 0.02);
  nh.param<double>("vio/visual_relaxed_observability_relative_eigen_threshold",
                   vio_visual_relaxed_observability_relative_eigen_threshold_,
                   vio_visual_observability_relative_eigen_threshold_);
  nh.param<double>("vio/visual_observability_absolute_eigen_threshold",
                   vio_visual_observability_absolute_eigen_threshold_, 0.0);
  nh.param<int>("diagnostics/console_interval_frames", diagnostics_console_interval_frames_, 20);
  nh.param<int>("diagnostics/csv_flush_interval_rows", diagnostics_csv_flush_interval_rows_, 100);
  nh.param<bool>("diagnostics/livo_scan_contract_en",
                 livo_scan_contract_diagnostics_en_, false);
  diagnostics_console_interval_frames_ = std::max(1, diagnostics_console_interval_frames_);
  diagnostics_csv_flush_interval_rows_ = std::max(1, diagnostics_csv_flush_interval_rows_);
  nh.param<bool>("vio/image_quality_gate_en", vio_image_quality_gate_en_, false);
  nh.param<double>("vio/image_quality_max_saturated_fraction", vio_image_quality_max_saturated_fraction_, 0.20);
  nh.param<double>("vio/image_quality_max_tile_saturated_fraction", vio_image_quality_max_tile_saturated_fraction_, 0.35);
  nh.param<double>("vio/image_quality_max_dark_fraction", vio_image_quality_max_dark_fraction_, 0.98);
  nh.param<double>("vio/image_quality_min_intensity_std", vio_image_quality_min_intensity_std_, 6.0);
  nh.param<bool>("vio/image_quality_tile_mask_en", vio_image_quality_tile_mask_en_, false);
  nh.param<double>("vio/image_quality_min_usable_tile_ratio", vio_image_quality_min_usable_tile_ratio_, 0.25);
  nh.param<double>("vio/image_quality_min_usable_horizontal_coverage",
                   vio_image_quality_min_usable_horizontal_coverage_, 0.50);
  nh.param<double>("vio/image_quality_min_usable_vertical_coverage",
                   vio_image_quality_min_usable_vertical_coverage_, 0.50);
  vio_image_quality_min_usable_tile_ratio_ =
      std::min(1.0, std::max(0.0, vio_image_quality_min_usable_tile_ratio_));
  vio_image_quality_min_usable_horizontal_coverage_ =
      std::min(1.0, std::max(0.0, vio_image_quality_min_usable_horizontal_coverage_));
  vio_image_quality_min_usable_vertical_coverage_ =
      std::min(1.0, std::max(0.0, vio_image_quality_min_usable_vertical_coverage_));
  nh.param<bool>("vio/visual_patch_quality_gate_en", vio_visual_patch_quality_gate_en_, true);
  nh.param<double>("vio/visual_patch_max_saturated_fraction", vio_visual_patch_max_saturated_fraction_, 0.10);
  nh.param<double>("vio/visual_patch_min_intensity_std", vio_visual_patch_min_intensity_std_, 2.0);
  nh.param<int>("vio/image_quality_saturated_pixel_value", vio_image_quality_saturated_pixel_value_, 250);
  nh.param<int>("vio/image_quality_dark_pixel_value", vio_image_quality_dark_pixel_value_, 5);
  nh.param<int>("vio/image_quality_tile_rows", vio_image_quality_tile_rows_, 4);
  nh.param<int>("vio/image_quality_tile_cols", vio_image_quality_tile_cols_, 4);

  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  nh.param<bool>("uav/gravity_align_en", gravity_align_en, false);
  nh.param<bool>("uwb/output_correction_en", uwb_output_correction_en_, false);
  nh.param<bool>("uwb/output_smooth_en", uwb_output_smooth_en_, true);
  nh.param<double>("uwb/output_smooth_alpha", uwb_output_smooth_alpha_, 0.15);
  nh.param<double>("uwb/output_smooth_max_step_m", uwb_output_smooth_max_step_m_, 0.05);
  nh.param<int>("uwb/pause_map_update_frames", external_update_pause_map_frames_after_correction_, 3);
  nh.param<int>("uwb/pause_map_update_frames_default",
                external_update_pause_map_frames_after_correction_, external_update_pause_map_frames_after_correction_);
  nh.param<double>("uwb/pause_map_update_min_correction_m", external_update_pause_map_min_correction_m_, 0.05);
  nh.param<double>("uwb/pause_map_update_threshold",
                   external_update_pause_map_min_correction_m_, external_update_pause_map_min_correction_m_);
  uwb_output_smooth_alpha_ = std::max(0.0, std::min(1.0, uwb_output_smooth_alpha_));
  uwb_output_smooth_max_step_m_ = std::max(0.0, uwb_output_smooth_max_step_m_);
  external_update_pause_map_frames_after_correction_ = std::max(0, external_update_pause_map_frames_after_correction_);
  external_update_pause_map_min_correction_m_ = std::max(0.0, external_update_pause_map_min_correction_m_);
  if (uwb_output_correction_en_)
  {
    ROS_WARN("[UWB] uwb/output_correction_en is debug-only. Default UWB fusion updates the internal EKF state xy instead of output_offset.");
  }
  nh.param<bool>("pos_output/enable_timestamp", pos_output_enable_timestamp_, true);
  nh.param<string>("pos_output/format", pos_output_format_, "timestamp_xyz_quat");
  std::string pos_output_format_lower = pos_output_format_;
  std::transform(pos_output_format_lower.begin(), pos_output_format_lower.end(), pos_output_format_lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (pos_output_format_lower == "timestamp_xyz_quat")
  {
    pos_output_enable_timestamp_ = true;
  }
  else if (pos_output_format_lower == "legacy_xyz_quat" || pos_output_format_lower == "xyz_quat")
  {
    pos_output_enable_timestamp_ = false;
  }
  if (pos_output_enable_timestamp_)
  {
    ROS_INFO("[POS] output format: timestamp x y z qx qy qz qw");
  }
  else
  {
    ROS_WARN("[POS] legacy output format enabled: x y z qw qx qy qz");
  }

  nh.param<string>("evo/seq_name", seq_name, "01");
  nh.param<bool>("evo/pose_output_en", pose_output_en, false);
  nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
  nh.param<double>("imu/acc_cov", acc_cov, 1.0);
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);
  nh.param<bool>("fullstate_shadow/enable", fullstate_shadow_enable_, false);
  nh.param<std::string>("fullstate_shadow/geometry_topic",
                        fullstate_shadow_geometry_topic_,
                        fullstate_shadow_geometry_topic_);

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<double>("voxel_safety/lidar_max_range_m", voxel_lidar_max_range_m_, 450.0);
  nh.param<bool>("preprocess/hilti_en", hilti_en, false);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  nh.setParam(fast_livo_logging::kRunLogDirectoryParam,
              fast_livo_logging::kRunLogDirectoryPending);
  nh.param<string>("/laserMapping/save_path",save_path,""); 
  if (save_path.empty()) nh.param<string>("pcd_save/save_path",save_path,"/home/jetson/data");
  nh.param<bool>("pcd_save/save_log_en", save_log_en, true);
  bool run_output_directory_ready = false;
  save_path = makeRunOutputDir(save_path, run_output_directory_ready);
  if (save_log_en && run_output_directory_ready)
  {
    nh.setParam(fast_livo_logging::kRunLogDirectoryParam, save_path);
    ROS_INFO("[RUN_LOG] GNSS logs share run directory: %s", save_path.c_str());
  }
  else
  {
    if (save_log_en)
    {
      ROS_ERROR("[RUN_LOG] Run directory creation failed; file logging disabled.");
      save_log_en = false;
      nh.setParam("pcd_save/save_log_en", false);
    }
    nh.setParam(fast_livo_logging::kRunLogDirectoryParam,
                fast_livo_logging::kRunLogDirectoryDisabled);
  }
  nh.param<bool>("pcd_save/global_map_pub", global_map_pub, false);  // 新增：读取 global_map_pub 参数，默认 false
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<bool>("pcd_save/colmap_output_en", colmap_output_en, false);
  nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
  nh.param<int>("pcd_save/max_cache_points", pcd_cache_max_points, 300000);
  nh.param<bool>("udp_report/en", udp_report_en, false);
  nh.param<std::string>("udp_report/target_ip", udp_target_ip_, "127.0.0.1");
  nh.param<int>("udp_report/target_port", udp_report_port_, 9000);
  nh.param<std::string>("udp_report/device_id", udp_device_id_, "fast_livo2");
  nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
  nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());
  nh.param<double>("debug/plot_time", plot_time, -10);
  nh.param<int>("debug/frame_cnt", frame_cnt, 6);

  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
  nh.param<bool>("publish/dense_map_en", dense_map_en, false);
  nh.param<bool>("publish/colorize_cloud_en", colorize_cloud_en_, true);
  nh.param<int>("publish/publish_img_stride", publish_img_stride_, 1);

  nh.param<int>("lio/map_update_stride", lio_map_update_stride_, 1);

  nh.param<bool>("debug/print_console_timing_en", print_console_timing_en_, true);
  nh.param<int>("debug/print_console_timing_stride", print_console_timing_stride_, 1);

  nh.param<bool>("aruco_landmarks/aruco_landmarks_en", aruco_landmarks_en, false);

  nh.param<bool>("adaptive_selector/en", adaptive_visual_selector_en, false);
  keyframe_trans_thresh_min_ = 0.08;
  keyframe_trans_thresh_max_ = 0.18;
  keyframe_rot_thresh_min_deg_ = 1.2;
  keyframe_rot_thresh_max_deg_ = 2.5;
  keyframe_constraint_ratio_full_ = 0.2;
  keyframe_max_skip_frames_ = 4;
  nh.param<double>("adaptive_selector/trans_thresh_min", keyframe_trans_thresh_min_, keyframe_trans_thresh_min_);
  nh.param<double>("adaptive_selector/trans_thresh_max", keyframe_trans_thresh_max_, keyframe_trans_thresh_max_);
  nh.param<double>("adaptive_selector/rot_thresh_min_deg", keyframe_rot_thresh_min_deg_, keyframe_rot_thresh_min_deg_);
  nh.param<double>("adaptive_selector/rot_thresh_max_deg", keyframe_rot_thresh_max_deg_, keyframe_rot_thresh_max_deg_);
  nh.param<double>("adaptive_selector/constraint_ratio_full", keyframe_constraint_ratio_full_, keyframe_constraint_ratio_full_);
  nh.param<int>("adaptive_selector/max_skip_frames", keyframe_max_skip_frames_, keyframe_max_skip_frames_);
  keyframe_trans_thresh_min_ = std::max(0.0, keyframe_trans_thresh_min_);
  keyframe_trans_thresh_max_ = std::max(keyframe_trans_thresh_min_, keyframe_trans_thresh_max_);
  keyframe_rot_thresh_min_deg_ = std::max(0.0, keyframe_rot_thresh_min_deg_);
  keyframe_rot_thresh_max_deg_ = std::max(keyframe_rot_thresh_min_deg_, keyframe_rot_thresh_max_deg_);
  keyframe_constraint_ratio_full_ = std::max(1e-6, keyframe_constraint_ratio_full_);
  keyframe_max_skip_frames_ = std::max(0, keyframe_max_skip_frames_);

  keyframe_trans_thresh_min_nominal_ = keyframe_trans_thresh_min_;
  keyframe_trans_thresh_max_nominal_ = keyframe_trans_thresh_max_;
  keyframe_rot_thresh_min_deg_nominal_ = keyframe_rot_thresh_min_deg_;
  keyframe_rot_thresh_max_deg_nominal_ = keyframe_rot_thresh_max_deg_;
  keyframe_max_skip_frames_nominal_ = keyframe_max_skip_frames_;
  vio_max_iterations_nominal_ = max_iterations;

  pub_scan_num = std::max(1, pub_scan_num);
  publish_img_stride_ = std::max(1, publish_img_stride_);
  lio_map_update_stride_ = std::max(1, lio_map_update_stride_);
  print_console_timing_stride_ = std::max(1, print_console_timing_stride_);

  pub_scan_num_nominal_ = pub_scan_num;
  dense_map_en_nominal_ = dense_map_en;
  pcd_save_en_nominal_ = pcd_save_en;
  colorize_cloud_en_nominal_ = colorize_cloud_en_;

  nh.param<bool>("runtime_guard/enable", runtime_guard_en_, true);
  nh.param<double>("runtime_guard/frame_time_budget_s", frame_time_budget_s_, frame_time_budget_s_);
  if (frame_time_budget_s_ <= 0.0)
  {
    ROS_WARN("[RuntimeGuard] Invalid frame_time_budget_s=%.6f, fallback to 0.100000 s", frame_time_budget_s_);
    frame_time_budget_s_ = 0.1;
  }
  ROS_INFO("[RuntimeGuard] enable=%d, frame_time_budget_s=%.6f s", static_cast<int>(runtime_guard_en_), frame_time_budget_s_);
  ROS_INFO("[AdaptiveSelector] enable=%d, trans=[%.3f %.3f] m, rot=[%.3f %.3f] deg, ratio_full=%.3f, max_skip=%d",
           static_cast<int>(adaptive_visual_selector_en),
           keyframe_trans_thresh_min_,
           keyframe_trans_thresh_max_,
           keyframe_rot_thresh_min_deg_,
           keyframe_rot_thresh_max_deg_,
           keyframe_constraint_ratio_full_,
           keyframe_max_skip_frames_);

  pub_scan_num_degraded_ = std::max(1, pub_scan_num_degraded_);
  runtime_over_budget_trigger_frames_ = std::max(1, runtime_over_budget_trigger_frames_);
  runtime_recover_trigger_frames_ = std::max(1, runtime_recover_trigger_frames_);

  nh.param<bool>("startup/wait_for_sensor_publishers", startup_wait_for_sensor_publishers_, true);
  nh.param<int>("startup/imu_warmup_min_samples", startup_imu_warmup_min_samples_, 30);
  nh.param<double>("startup/imu_warmup_min_duration_s", startup_imu_warmup_min_duration_s_, 0.15);
  nh.param<bool>("startup/require_lidar_frame", startup_require_lidar_frame_, true);
  nh.param<bool>("startup/require_image_buffer", startup_require_image_buffer_, true);
  nh.param<bool>("diagnostics/runtime_memory_monitor_en", runtime_memory_monitor_en_, true);
  nh.param<double>("diagnostics/runtime_memory_monitor_period_s", runtime_memory_monitor_period_s_, 1.0);
  startup_imu_warmup_min_samples_ = std::max(1, startup_imu_warmup_min_samples_);
  startup_imu_warmup_min_duration_s_ = std::max(0.0, startup_imu_warmup_min_duration_s_);
  runtime_memory_monitor_period_s_ = std::max(0.1, runtime_memory_monitor_period_s_);
  vio_ncc_threshold_ = std::max(-1.0, std::min(1.0, vio_ncc_threshold_));
  vio_visual_robust_delta_ = std::max(1e-6, vio_visual_robust_delta_);
  vio_visual_observability_relative_eigen_threshold_ =
      std::max(0.0, std::min(1.0, vio_visual_observability_relative_eigen_threshold_));
  vio_visual_relaxed_observability_relative_eigen_threshold_ =
      std::max(0.0, std::min(1.0,
          vio_visual_relaxed_observability_relative_eigen_threshold_));
  vio_visual_observability_absolute_eigen_threshold_ =
      std::max(0.0, vio_visual_observability_absolute_eigen_threshold_);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::updateRuntimeGuard(double frame_time_s)
{
  if (!runtime_guard_en_) return;

  if (frame_time_s > frame_time_budget_s_)
  {
    runtime_over_budget_count_++;
    runtime_under_budget_count_ = 0;
  }
  else
  {
    runtime_under_budget_count_++;
    runtime_over_budget_count_ = 0;
  }

  if (!runtime_degraded_mode_ && runtime_over_budget_count_ >= std::max(1, runtime_over_budget_trigger_frames_))
  {
    runtime_degraded_mode_ = true;
    runtime_over_budget_count_ = 0;
    runtime_under_budget_count_ = 0;

    pub_scan_num = std::max(pub_scan_num_nominal_, pub_scan_num_degraded_);
    if (disable_dense_map_in_degraded_) dense_map_en = false;
    if (disable_pcd_save_in_degraded_) pcd_save_en = false;
    if (disable_colorize_cloud_in_degraded_) colorize_cloud_en_ = false;
    suppress_image_pub_ = disable_image_publish_in_degraded_;
    publish_img_counter_ = 0;

    if (vio_manager)
    {
      const int degraded_iters = std::max(1, std::min(vio_max_iterations_nominal_, vio_max_iterations_degraded_));
      vio_manager->max_iterations = degraded_iters;
    }

    if (adaptive_visual_selector_en)
    {
      keyframe_trans_thresh_min_ = keyframe_trans_thresh_min_nominal_ * std::max(1.0, keyframe_trans_scale_degraded_);
      keyframe_trans_thresh_max_ = keyframe_trans_thresh_max_nominal_ * std::max(1.0, keyframe_trans_scale_degraded_);
      keyframe_rot_thresh_min_deg_ = keyframe_rot_thresh_min_deg_nominal_ * std::max(1.0, keyframe_rot_scale_degraded_);
      keyframe_rot_thresh_max_deg_ = keyframe_rot_thresh_max_deg_nominal_ * std::max(1.0, keyframe_rot_scale_degraded_);
      keyframe_max_skip_frames_ = std::max(keyframe_max_skip_frames_nominal_, keyframe_max_skip_frames_degraded_);
      skipped_visual_frames_ = 0;
    }

    ROS_WARN("[RuntimeGuard] Enter degraded mode, frame_time=%.4f s > budget=%.4f s, pub_scan_num=%d, dense_map=%d, pcd_save=%d, colorize=%d, img_pub=%d",
             frame_time_s,
             frame_time_budget_s_,
             pub_scan_num,
             static_cast<int>(dense_map_en),
             static_cast<int>(pcd_save_en),
             static_cast<int>(colorize_cloud_en_),
             static_cast<int>(!suppress_image_pub_));
    return;
  }

  if (runtime_degraded_mode_ && runtime_under_budget_count_ >= std::max(1, runtime_recover_trigger_frames_))
  {
    runtime_degraded_mode_ = false;
    runtime_over_budget_count_ = 0;
    runtime_under_budget_count_ = 0;

    pub_scan_num = std::max(1, pub_scan_num_nominal_);
    dense_map_en = dense_map_en_nominal_;
    pcd_save_en = pcd_save_en_nominal_;
    colorize_cloud_en_ = colorize_cloud_en_nominal_;
    suppress_image_pub_ = false;
    publish_img_counter_ = 0;

    if (vio_manager)
    {
      vio_manager->max_iterations = std::max(1, vio_max_iterations_nominal_);
    }

    keyframe_trans_thresh_min_ = keyframe_trans_thresh_min_nominal_;
    keyframe_trans_thresh_max_ = keyframe_trans_thresh_max_nominal_;
    keyframe_rot_thresh_min_deg_ = keyframe_rot_thresh_min_deg_nominal_;
    keyframe_rot_thresh_max_deg_ = keyframe_rot_thresh_max_deg_nominal_;
    keyframe_max_skip_frames_ = keyframe_max_skip_frames_nominal_;
    skipped_visual_frames_ = 0;

    ROS_INFO("[RuntimeGuard] Recover nominal mode, frame_time=%.4f s, pub_scan_num=%d, dense_map=%d, pcd_save=%d, colorize=%d, img_pub=%d",
             frame_time_s,
             pub_scan_num,
             static_cast<int>(dense_map_en),
             static_cast<int>(pcd_save_en),
             static_cast<int>(colorize_cloud_en_),
             static_cast<int>(!suppress_image_pub_));
  }
}

bool LIVMapper::startupWarmupReady()
{
  if (!startup_wait_for_sensor_publishers_) return true;
  if (startup_warmup_announced_) return true;

  const bool lidar_ready = !lidar_en || !startup_require_lidar_frame_ ||
                           !lid_raw_data_buffer.empty();
  bool imu_ready = !imu_en;
  double imu_duration_s = 0.0;
  if (imu_en && !imu_buffer.empty())
  {
    double min_stamp = std::numeric_limits<double>::infinity();
    double max_stamp = -std::numeric_limits<double>::infinity();
    for (const auto &imu : imu_buffer)
    {
      if (!imu) continue;
      const double stamp = imu->header.stamp.toSec();
      min_stamp = std::min(min_stamp, stamp);
      max_stamp = std::max(max_stamp, stamp);
    }
    if (std::isfinite(min_stamp) && std::isfinite(max_stamp))
      imu_duration_s = std::max(0.0, max_stamp - min_stamp);
    imu_ready = static_cast<int>(imu_buffer.size()) >= startup_imu_warmup_min_samples_ &&
                imu_duration_s >= startup_imu_warmup_min_duration_s_;
  }

  bool image_ready = !img_en || !startup_require_image_buffer_;
  if (img_en && startup_require_image_buffer_)
  {
    image_ready = static_cast<int>(img_buffer.size()) >= sync_img_buffer_min_size_;
    if (image_ready && sync_img_lookahead_time_ > 0.0 && img_time_buffer.size() >= 2)
      image_ready = img_time_buffer.back() - img_time_buffer.front() >= sync_img_lookahead_time_;
  }

  const bool ready = lidar_ready && imu_ready && image_ready;
  if (ready && !startup_warmup_announced_)
  {
    startup_warmup_announced_ = true;
    ROS_INFO("[STARTUP] SENSOR_WARMUP_READY imu_samples=%zu imu_duration=%.3f s lidar_frames=%zu image_frames=%zu",
             imu_buffer.size(), imu_duration_s, lid_raw_data_buffer.size(), img_buffer.size());
  }
  else if (!ready)
  {
    ROS_INFO_THROTTLE(1.0,
                      "[STARTUP] WAIT_SENSOR_DATA imu=%zu/%d duration=%.3f/%.3f lidar=%zu image=%zu/%d",
                      imu_buffer.size(), startup_imu_warmup_min_samples_, imu_duration_s,
                      startup_imu_warmup_min_duration_s_, lid_raw_data_buffer.size(),
                      img_buffer.size(), img_en && startup_require_image_buffer_ ? sync_img_buffer_min_size_ : 0);
  }
  return ready;
}

void LIVMapper::updateMappingReady()
{
  if (mapping_ready_published_) return;
  const bool imu_ready = !imu_en || !p_imu->imu_need_init;
  const bool lidar_ready = !lidar_en || lidar_map_inited;
  if (!imu_ready || !lidar_ready) return;

  std_msgs::Bool status;
  status.data = true;
  pubMappingReady.publish(status);
  mapping_ready_published_ = true;
  ROS_INFO("[STARTUP] MAPPING_READY imu_initialized=%d lidar_map_initialized=%d mode=%d",
           static_cast<int>(imu_ready), static_cast<int>(lidar_ready),
           static_cast<int>(slam_mode_));
}

void LIVMapper::logRuntimeMemory()
{
  if (!runtime_memory_monitor_en_ || !fout_runtime_memory.is_open()) return;
  const double wall_time_s = ros::WallTime::now().toSec();
  if (last_runtime_memory_log_wall_s_ >= 0.0 &&
      wall_time_s - last_runtime_memory_log_wall_s_ < runtime_memory_monitor_period_s_)
    return;
  last_runtime_memory_log_wall_s_ = wall_time_s;

  double rss_mb = std::numeric_limits<double>::quiet_NaN();
  std::ifstream status("/proc/self/status");
  std::string key;
  while (status >> key)
  {
    if (key == "VmRSS:")
    {
      double rss_kb = 0.0;
      std::string unit;
      status >> rss_kb >> unit;
      rss_mb = rss_kb / 1024.0;
      break;
    }
    status.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  const auto &lio_voxel_map = voxelmap_manager->voxel_map_;
  size_t octree_nodes = 0;
  for (const auto &entry : lio_voxel_map) octree_nodes += countOctreeNodes(entry.second);
  const size_t visual_voxels = vio_manager ? vio_manager->feat_map.size() : 0;
  const size_t visual_points = vio_manager ? vio_manager->getVisualPointCount() : 0;
  const size_t pcd_cache_points =
      (pcl_wait_save ? pcl_wait_save->size() : 0) +
      (pcl_wait_save_intensity ? pcl_wait_save_intensity->size() : 0);
  const double sensor_time_s = LidarMeasures.last_lio_update_time;
  fout_runtime_memory << std::fixed << std::setprecision(6)
                      << wall_time_s << ',' << sensor_time_s << ',' << rss_mb << ','
                      << lio_voxel_map.size() << ',' << octree_nodes << ','
                      << visual_voxels << ',' << visual_points << ',' << pcd_cache_points << ','
                      << img_buffer.size() << ',' << imu_buffer.size() << ','
                      << lid_raw_data_buffer.size() << '\n';
  fout_runtime_memory.flush();
}

void LIVMapper::initializeComponents(ros::NodeHandle &nh) 
{
  validateVoxelLeafOrThrow(filter_size_surf_min, "preprocess/filter_size_surf");
  validateVoxelLeafOrThrow(filter_size_pcd, "pcd_save/filter_size_pcd");
  if (!std::isfinite(voxel_lidar_max_range_m_) || voxel_lidar_max_range_m_ <= 0.0)
    throw std::runtime_error("Invalid voxel_safety/lidar_max_range_m");
  ROS_INFO("[VOXEL_CONFIG] tag=VOXEL_FRAME leaf=[%.6f %.6f %.6f] frame=lidar/body max_range_m=%.3f",
           filter_size_surf_min, filter_size_surf_min, filter_size_surf_min, voxel_lidar_max_range_m_);
  ROS_INFO("[VOXEL_CONFIG] tags=VOXEL_PUBLISH_RGB,VOXEL_PUBLISH_INTENSITY,VOXEL_SAVE leaf=[%.6f %.6f %.6f] frames=world/global",
           filter_size_pcd, filter_size_pcd, filter_size_pcd);
  if (filter_size_surf_min < 0.02 || filter_size_surf_min > 1.0)
    ROS_WARN("[VOXEL_CONFIG] preprocess/filter_size_surf=%.6f is outside the suggested LIO range [0.02, 1.0] m",
             filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);
  voxelmap_manager->configureP5SeedBasin(filter_size_surf_min, save_path);

  if (!vk::camera_loader::loadFromRosNs("laserMapping", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->min_retrieve_points = vio_min_retrieve_points_;
  vio_manager->visual_spatial_coverage_gate_en = vio_visual_spatial_coverage_gate_en_;
  vio_manager->relaxed_min_retrieve_points = vio_relaxed_min_retrieve_points_;
  vio_manager->relaxed_min_occupied_good_tiles = vio_relaxed_min_occupied_good_tiles_;
  vio_manager->relaxed_min_good_tile_ratio = vio_relaxed_min_good_tile_ratio_;
  vio_manager->relaxed_min_horizontal_coverage = vio_relaxed_min_horizontal_coverage_;
  vio_manager->relaxed_min_vertical_coverage = vio_relaxed_min_vertical_coverage_;
  vio_manager->degeneracy_relaxed_track_gate_en = vio_degeneracy_relaxed_track_gate_en_;
  vio_manager->degeneracy_relaxed_min_retrieve_points = vio_degeneracy_relaxed_min_retrieve_points_;
  vio_manager->degeneracy_relaxed_min_occupied_good_tiles = vio_degeneracy_relaxed_min_occupied_good_tiles_;
  vio_manager->degeneracy_relaxed_min_good_tile_ratio = vio_degeneracy_relaxed_min_good_tile_ratio_;
  vio_manager->degeneracy_relaxed_min_horizontal_coverage = vio_degeneracy_relaxed_min_horizontal_coverage_;
  vio_manager->degeneracy_relaxed_min_vertical_coverage = vio_degeneracy_relaxed_min_vertical_coverage_;
  vio_manager->degeneracy_visual_position_scale = vio_degeneracy_visual_position_scale_;
  vio_manager->degeneracy_visual_max_position_step_m = vio_degeneracy_visual_max_position_step_m_;
  vio_manager->visual_reference_refresh_en = vio_visual_reference_refresh_en_;
  vio_manager->visual_reference_refresh_max_age_frames = vio_visual_reference_refresh_max_age_frames_;
  vio_manager->visual_reference_refresh_min_tracked_points = vio_visual_reference_refresh_min_tracked_points_;
  vio_manager->visual_reference_refresh_min_occupied_good_tiles = vio_visual_reference_refresh_min_occupied_good_tiles_;
  vio_manager->visual_reference_refresh_min_horizontal_coverage = vio_visual_reference_refresh_min_horizontal_coverage_;
  vio_manager->visual_reference_refresh_min_vertical_coverage = vio_visual_reference_refresh_min_vertical_coverage_;
  vio_manager->visual_reference_refresh_max_per_frame = vio_visual_reference_refresh_max_per_frame_;
  vio_manager->visual_search_level_low_contrast_retry_en =
      vio_visual_search_level_low_contrast_retry_en_;
  vio_manager->visual_adaptive_covariance_relaxed_en =
      visual_adaptive_covariance_relaxed_en_;
  vio_manager->visual_adaptive_covariance_relaxed_min_points =
      visual_adaptive_covariance_relaxed_min_points_;
  vio_manager->visual_adaptive_covariance_k_ncc = visual_adaptive_covariance_k_ncc_;
  vio_manager->visual_adaptive_covariance_k_level = visual_adaptive_covariance_k_level_;
  vio_manager->visual_adaptive_covariance_k_photo = visual_adaptive_covariance_k_photo_;
  vio_manager->visual_adaptive_covariance_scale_max = visual_adaptive_covariance_scale_max_;
  vio_manager->min_update_meas = vio_min_update_meas_;
  vio_manager->low_track_force_update_stride = vio_low_track_force_update_stride_;
  vio_manager->low_track_force_min_points = vio_low_track_force_min_points_;
  vio_manager->deterministic_visual_update_en = vio_deterministic_visual_update_en_;
  vio_manager->deterministic_pixel_snap_en = deterministic_pixel_snap_en_;
  vio_manager->deterministic_camera_point_snap_en = deterministic_camera_point_snap_en_;
  vio_manager->deterministic_contiguous_image_copy_en = deterministic_contiguous_image_copy_en_;
  vio_manager->deterministic_visual_voxel_key_sort_en = deterministic_visual_voxel_key_sort_en_;
  vio_manager->visual_update_guard_en = vio_visual_update_guard_en_;
  vio_manager->visual_update_max_trans_m = vio_visual_update_max_trans_m_;
  vio_manager->visual_update_max_rot_deg = vio_visual_update_max_rot_deg_;
  vio_manager->visual_update_max_backward_m = vio_visual_update_max_backward_m_;
  vio_manager->visual_update_max_backward_ratio = vio_visual_update_max_backward_ratio_;
  vio_manager->visual_update_backward_abs_floor_m = vio_visual_update_backward_abs_floor_m_;
  vio_manager->visual_update_max_lateral_m = vio_visual_update_max_lateral_m_;
  vio_manager->visual_update_max_lateral_ratio = vio_visual_update_max_lateral_ratio_;
  vio_manager->visual_update_max_exposure_delta = vio_visual_update_max_exposure_delta_;
  vio_manager->visual_update_max_velocity_increment_mps = vio_visual_update_max_velocity_increment_mps_;
  vio_manager->visual_update_max_acc_bias_increment_mps2 = vio_visual_update_max_acc_bias_increment_mps2_;
  vio_manager->visual_update_max_gyro_bias_increment_rps = vio_visual_update_max_gyro_bias_increment_rps_;
  vio_manager->visual_update_normalized_nis_max = vio_visual_update_normalized_nis_max_;
  vio_manager->ncc_en = vio_ncc_en_;
  vio_manager->ncc_thre = vio_ncc_threshold_;
  vio_manager->visual_robust_kernel_en = vio_visual_robust_kernel_en_;
  vio_manager->visual_robust_delta = vio_visual_robust_delta_;
  vio_manager->visual_observability_gate_en = vio_visual_observability_gate_en_;
  vio_manager->visual_observability_relative_eigen_threshold =
      vio_visual_observability_relative_eigen_threshold_;
  vio_manager->visual_relaxed_observability_relative_eigen_threshold =
      vio_visual_relaxed_observability_relative_eigen_threshold_;
  vio_manager->visual_observability_absolute_eigen_threshold =
      vio_visual_observability_absolute_eigen_threshold_;
  vio_manager->diagnostics_console_interval_frames = diagnostics_console_interval_frames_;
  vio_manager->image_quality_gate_en = vio_image_quality_gate_en_;
  vio_manager->image_quality_max_saturated_fraction = vio_image_quality_max_saturated_fraction_;
  vio_manager->image_quality_max_tile_saturated_fraction = vio_image_quality_max_tile_saturated_fraction_;
  vio_manager->image_quality_max_dark_fraction = vio_image_quality_max_dark_fraction_;
  vio_manager->image_quality_min_intensity_std = vio_image_quality_min_intensity_std_;
  vio_manager->image_quality_tile_mask_en = vio_image_quality_tile_mask_en_;
  vio_manager->image_quality_min_usable_tile_ratio = vio_image_quality_min_usable_tile_ratio_;
  vio_manager->image_quality_min_usable_horizontal_coverage =
      vio_image_quality_min_usable_horizontal_coverage_;
  vio_manager->image_quality_min_usable_vertical_coverage =
      vio_image_quality_min_usable_vertical_coverage_;
  vio_manager->visual_patch_quality_gate_en = vio_visual_patch_quality_gate_en_;
  vio_manager->visual_patch_max_saturated_fraction = vio_visual_patch_max_saturated_fraction_;
  vio_manager->visual_patch_min_intensity_std = vio_visual_patch_min_intensity_std_;
  vio_manager->image_quality_saturated_pixel_value = vio_image_quality_saturated_pixel_value_;
  vio_manager->image_quality_dark_pixel_value = vio_image_quality_dark_pixel_value_;
  vio_manager->image_quality_tile_rows = vio_image_quality_tile_rows_;
  vio_manager->image_quality_tile_cols = vio_image_quality_tile_cols_;
  vio_manager->max_state_update_rot_deg = vio_max_state_update_rot_deg_;
  vio_manager->max_state_update_trans_m = vio_max_state_update_trans_m_;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->visual_map_prune_en = visual_map_prune_en;
  vio_manager->visual_map_supply_diagnostics_en = visual_map_supply_diagnostics_en_;
  vio_manager->visual_map_fov_fallback_en = visual_map_fov_fallback_en_;
  vio_manager->visual_adaptive_covariance_shadow_en = visual_adaptive_covariance_shadow_en_;
  vio_manager->visual_shadow_no_commit_en = visual_shadow_no_commit_en_;
  vio_manager->visual_map_fov_fallback_target_grid_candidates =
      visual_map_fov_fallback_target_grid_candidates_;
  vio_manager->visual_map_max_voxels = visual_map_max_voxels;
  vio_manager->visual_map_max_points_per_voxel = visual_map_max_points_per_voxel;
  vio_manager->visual_map_max_total_points = visual_map_max_total_points;
  vio_manager->visual_map_max_add_per_frame = visual_map_max_add_per_frame_;
  vio_manager->visual_map_min_shi_tomasi_score = static_cast<float>(visual_map_min_shi_tomasi_score_);
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->aruco_landmarks_en = aruco_landmarks_en;
  vio_manager->timing_log_dir = save_path;
  vio_manager->timing_log_enable = save_log_en;
  vio_manager->initializeVIO(nh);
  ROS_INFO("[VIO_GUARD_CONFIG] enable=%d pos=%.3f rot=%.3f vel=%.3f ba=%.3f bg=%.6f normalized_nis=%.3f",
           static_cast<int>(vio_visual_update_guard_en_),
           vio_visual_update_max_trans_m_, vio_visual_update_max_rot_deg_,
           vio_visual_update_max_velocity_increment_mps_,
           vio_visual_update_max_acc_bias_increment_mps2_,
           vio_visual_update_max_gyro_bias_increment_rps_,
           vio_visual_update_normalized_nis_max_);
  ROS_INFO("[VIO_QUALITY_CONFIG] ncc=%d threshold=%.3f robust_huber=%d delta=%.3f observability=%d relative_eigen=%.6f relaxed_relative_eigen=%.6f absolute_eigen=%.6f",
           static_cast<int>(vio_ncc_en_), vio_ncc_threshold_,
           static_cast<int>(vio_visual_robust_kernel_en_), vio_visual_robust_delta_,
           static_cast<int>(vio_visual_observability_gate_en_),
           vio_visual_observability_relative_eigen_threshold_,
           vio_visual_relaxed_observability_relative_eigen_threshold_,
           vio_visual_observability_absolute_eigen_threshold_);
  ROS_INFO("[VIO_TILE_MASK_CONFIG] enable=%d grid=%dx%d min_good_ratio=%.3f min_coverage=%.3f/%.3f",
           static_cast<int>(vio_image_quality_tile_mask_en_),
           vio_image_quality_tile_rows_, vio_image_quality_tile_cols_,
           vio_image_quality_min_usable_tile_ratio_,
           vio_image_quality_min_usable_horizontal_coverage_,
           vio_image_quality_min_usable_vertical_coverage_);
  ROS_INFO("[VIO_SPATIAL_GATE_CONFIG] enable=%d normal_min=%d relaxed_min=%d occupied_tiles=%d good_ratio=%.3f coverage=%.3f/%.3f",
           static_cast<int>(vio_visual_spatial_coverage_gate_en_), vio_min_retrieve_points_,
           vio_relaxed_min_retrieve_points_, vio_relaxed_min_occupied_good_tiles_,
           vio_relaxed_min_good_tile_ratio_,
           vio_relaxed_min_horizontal_coverage_, vio_relaxed_min_vertical_coverage_);
  ROS_INFO("[VIO_DEGENERACY_GATE_CONFIG] enable=%d relaxed_min=%d occupied_tiles=%d good_ratio=%.3f coverage=%.3f/%.3f position_scale=%.3f max_step=%.4f m",
           static_cast<int>(vio_degeneracy_relaxed_track_gate_en_),
           vio_degeneracy_relaxed_min_retrieve_points_,
           vio_degeneracy_relaxed_min_occupied_good_tiles_,
           vio_degeneracy_relaxed_min_good_tile_ratio_,
           vio_degeneracy_relaxed_min_horizontal_coverage_,
           vio_degeneracy_relaxed_min_vertical_coverage_,
           vio_degeneracy_visual_position_scale_,
           vio_degeneracy_visual_max_position_step_m_);
  ROS_INFO("[VIO_ADAPTIVE_COV_CONFIG] enable=%d relaxed_min=%d k(ncc/level/photo)=%.3f/%.3f/%.3f scale_max=%.3f baseline_cov=%.3f",
           static_cast<int>(visual_adaptive_covariance_relaxed_en_),
           visual_adaptive_covariance_relaxed_min_points_,
           visual_adaptive_covariance_k_ncc_, visual_adaptive_covariance_k_level_,
           visual_adaptive_covariance_k_photo_, visual_adaptive_covariance_scale_max_,
           IMG_POINT_COV);
  initializeUdpReporter();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);
  p_imu->set_log_dir(save_path);
  bool competition_startup_telemetry_en = false;
  nh.param<bool>("diagnostics/competition_startup_telemetry_en",
                 competition_startup_telemetry_en, false);
  p_imu->enable_competition_startup_telemetry(
      competition_startup_telemetry_en);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(save_path + "points3D.txt", std::ios::out);
  if(save_log_en) fout_pcd_pos.open(save_path + "scans_pos.json", std::ios::out);
  if (save_log_en)
  {
    if (runtime_memory_monitor_en_)
    {
      fout_runtime_memory.open(save_path + "runtime_memory.csv", std::ios::out);
      if (fout_runtime_memory.is_open())
      {
        fout_runtime_memory
            << "wall_time_s,sensor_time_s,rss_mb,lio_root_voxels,lio_octree_nodes,visual_voxels,visual_points,pcd_cache_points,image_buffer_size,imu_buffer_size,lidar_buffer_size\n";
      }
    }
    fout_lio_degeneracy.open(save_path + "lio_degeneracy.csv", std::ios::out);
    if (fout_lio_degeneracy.is_open())
    {
      fout_lio_degeneracy
          << "timestamp,frame_id,effective_feature_count,average_point_plane_residual,"
          << "predicted_px,predicted_py,predicted_pz,predicted_vx,predicted_vy,predicted_vz,"
          << "updated_px,updated_py,updated_pz,updated_vx,updated_vy,updated_vz,"
          << "delta_px,delta_py,delta_pz,delta_vx,delta_vy,delta_vz,"
          << "rotation_eigenvalue_0,rotation_eigenvalue_1,rotation_eigenvalue_2,"
          << "translation_eigenvalue_0,translation_eigenvalue_1,translation_eigenvalue_2,"
          << "translation_eigenvalue_ratio,translation_condition_number,"
          << "weak_translation_direction_world_x,weak_translation_direction_world_y,weak_translation_direction_world_z,"
          << "predicted_speed_mps,velocity_weak_direction_cos,"
          << "raw_position_correction_x,raw_position_correction_y,raw_position_correction_z,"
          << "raw_velocity_correction_x,raw_velocity_correction_y,raw_velocity_correction_z,"
          << "velocity_projection_on_weak_direction,position_correction_on_weak_direction,"
          << "translation_weight_0,translation_weight_1,translation_weight_2,"
          << "raw_is_degenerate,is_degenerate,is_severely_degenerate,direction_conflict,direction_conflict_consecutive_frames,"
          << "direction_guard_triggered,state_intervention_applied,"
          << "update_was_suppressed,update_was_significantly_suppressed,"
          << "map_guard_requested,map_guard_enforced,map_insert_skipped,map_insert_skip_reason,"
          << "input_feature_count,downsampled_feature_count,valid_plane_count,observability_feature_count,inlier_ratio,"
          << "abs_residual_median,abs_residual_p90,abs_residual_p95,residual_rmse,abs_residual_max,"
          << "measurement_variance_mean,measurement_variance_median,measurement_variance_p90,measurement_variance_p95,"
          << "measurement_variance_min,measurement_variance_max,"
          << "rotation_eigenvalue_ratio,rotation_condition_number,"
          << "weak_rotation_direction_body_x,weak_rotation_direction_body_y,weak_rotation_direction_body_z,"
          << "weak_translation_direction_body_x,weak_translation_direction_body_y,weak_translation_direction_body_z,"
          << "weak_translation_vertical_abs,delta_rotation_vector_x,delta_rotation_vector_y,delta_rotation_vector_z,"
          << "delta_roll_deg,delta_pitch_deg,delta_yaw_deg,delta_rotation_deg\n";
    }
    else
    {
      ROS_WARN("[LIO_DEGEN] Failed to open %slio_degeneracy.csv", save_path.c_str());
    }
    fout_lio_transaction.open(save_path + "lio_frame_transaction.csv", std::ios::out);
    if (fout_lio_transaction.is_open())
    {
      fout_lio_transaction
          << "timestamp,frame_id,iteration_count,convergence_status,converged,reached_iteration_limit,"
             "correspondence_count,valid_residual_count,cost_before,cost_after,"
             "translation_increment_norm,rotation_increment_deg,velocity_increment_norm,"
             "covariance_trace_before,covariance_trace_after,covariance_min_diagonal,"
             "covariance_max_diagonal,candidate_covariance_min_eigenvalue,candidate_covariance_asymmetry,"
             "map_inserted,commit\n";
    }
    fout_lio_motion_consistency.open(
        save_path + "motion_consistency_shadow.csv", std::ios::out);
    if (fout_lio_motion_consistency.is_open())
    {
      auto header_vector = [&](const std::string &prefix, int size) {
        for (int i = 0; i < size; ++i)
          fout_lio_motion_consistency << ',' << prefix << '_' << i;
      };
      auto header_cross = [&](const std::string &prefix) {
        fout_lio_motion_consistency << ',' << prefix << "_frobenius_norm,"
            << prefix << "_maximum_singular_value";
        header_vector(prefix + "_velocity_direction", 3);
        header_vector(prefix + "_coupled_direction", 3);
      };
      auto header_quadratic = [&](const std::string &prefix) {
        fout_lio_motion_consistency << ',' << prefix << "_valid," << prefix
            << "_value," << prefix << "_rank," << prefix
            << "_minimum_eigenvalue," << prefix << "_maximum_eigenvalue,"
            << prefix << "_condition_number," << prefix << "_eigenvalue_cutoff";
      };
      auto header_source_counterfactual = [&](const std::string &prefix) {
        fout_lio_motion_consistency << ',' << prefix << "_valid," << prefix
            << "_strength";
        header_vector(prefix + "_dimensionless_information_eigenvalue", 6);
        header_vector(prefix + "_direction_weight", 6);
        header_vector(prefix + "_delta_state", DIM_STATE);
        fout_lio_motion_consistency
            << ',' << prefix << "_delta_pose_norm"
            << ',' << prefix << "_delta_exposure_abs"
            << ',' << prefix << "_delta_velocity_norm"
            << ',' << prefix << "_delta_bias_g_norm"
            << ',' << prefix << "_delta_bias_a_norm"
            << ',' << prefix << "_delta_gravity_norm"
            << ',' << prefix << "_pose_difference_norm"
            << ',' << prefix << "_velocity_difference_norm"
            << ',' << prefix << "_localized_prior_min_eigenvalue"
            << ',' << prefix << "_localized_prior_asymmetry"
            << ',' << prefix << "_posterior_min_eigenvalue"
            << ',' << prefix << "_posterior_asymmetry"
            << ',' << prefix << "_posterior_trace"
            << ',' << prefix << "_pose_posterior_covariance_difference_norm";
      };
      fout_lio_motion_consistency
          << "timestamp,frame_id,shadow_warn,shadow_reason,diagnostic_valid,commit";
      fout_lio_motion_consistency
          << ",delta_rotation_x,delta_rotation_y,delta_rotation_z,"
             "delta_position_x,delta_position_y,delta_position_z,delta_exposure,"
             "delta_velocity_x,delta_velocity_y,delta_velocity_z,"
             "delta_bias_g_x,delta_bias_g_y,delta_bias_g_z,"
             "delta_bias_a_x,delta_bias_a_y,delta_bias_a_z,"
             "delta_gravity_x,delta_gravity_y,delta_gravity_z";
      fout_lio_motion_consistency
          << ",delta_rotation_norm_rad,delta_rotation_norm_deg,delta_position_norm_m,"
             "delta_exposure_abs,delta_velocity_norm_mps,delta_bias_g_norm,delta_bias_a_norm,"
             "delta_gravity_norm";
      header_cross("p_v_theta");
      header_cross("p_v_position");
      header_cross("p_v_bg");
      header_cross("p_v_ba");
      header_cross("p_v_gravity");
      fout_lio_motion_consistency
          << ",rotation_equivalent_gain_norm,position_equivalent_gain_norm,"
             "velocity_equivalent_gain_norm,bias_g_equivalent_gain_norm,"
             "bias_a_equivalent_gain_norm,gravity_equivalent_gain_norm";
      header_vector("last_iteration_innovation_component", DIM_STATE);
      header_vector("last_iteration_relinearization_component", DIM_STATE);
      header_vector("accumulated_innovation_component", DIM_STATE);
      header_vector("accumulated_relinearization_component", DIM_STATE);
      fout_lio_motion_consistency
          << ",analyzed_iteration_count,maximum_iteration_linearized_nis_per_dof,"
             "mean_iteration_linearized_nis_per_dof,"
             "maximum_iteration_rotation_equivalent_gain_norm,"
             "maximum_iteration_position_equivalent_gain_norm,"
             "maximum_iteration_velocity_equivalent_gain_norm,"
             "maximum_iteration_velocity_innovation_component_norm";
      header_quadratic("q_velocity_prior");
      header_quadratic("q_pose_prior");
      header_quadratic("q_bias_prior");
      header_quadratic("q_gravity_prior");
      header_quadratic("q_state_prior");
      header_quadratic("q_velocity_correction_approx");
      header_vector("correction_covariance_eigenvalue", DIM_STATE);
      header_vector("velocity_correction_covariance_eigenvalue", 3);
      fout_lio_motion_consistency
          << ",correction_covariance_psd,correction_covariance_rank,"
             "correction_covariance_asymmetry,residual_weighted_energy,"
             "information_explained_energy,linearized_nis,linearized_nis_per_dof,"
             "linearized_nis_valid";
      header_vector("pose_information_eigenvalue", 6);
      header_vector("pose_information_weak_direction", 6);
      header_vector("pose_correction_eigen_projection", 6);
      fout_lio_motion_consistency << ",pose_direction_decomposition_valid";
      header_vector("pose_direction_eigenvalue", 6);
      for (int direction = 0; direction < 6; ++direction)
      {
        header_vector("pose_direction_" + std::to_string(direction) + "_axis", 6);
        header_vector("pose_direction_" + std::to_string(direction) +
                          "_velocity_contribution", 3);
        fout_lio_motion_consistency << ",pose_direction_" << direction
            << "_velocity_contribution_norm";
      }
      header_vector("pose_direction_unapplied_carry", DIM_STATE);
      fout_lio_motion_consistency
          << ",pose_direction_full_closure_norm,pose_direction_velocity_closure_norm,"
             "pose_direction_applied_full_closure_norm,"
             "pose_direction_applied_velocity_closure_norm,"
             "source_counterfactual_compute_time_ms";
      header_source_counterfactual("transfer_identity");
      header_source_counterfactual("transfer_mild");
      header_source_counterfactual("transfer_medium");
      header_source_counterfactual("transfer_stronger");
      header_source_counterfactual("damping_identity");
      header_source_counterfactual("damping_mild");
      header_source_counterfactual("damping_medium");
      header_source_counterfactual("damping_stronger");
      fout_lio_motion_consistency
          << ",pose_information_rank,pose_information_condition_number,"
             "raw_is_degenerate,is_degenerate,observability_valid";
      header_vector("rotation_information_eigenvalue", 3);
      header_vector("translation_information_eigenvalue", 3);
      fout_lio_motion_consistency
          << ",rotation_information_ratio,rotation_information_condition,"
             "translation_information_ratio,translation_information_condition";
      header_vector("weak_rotation_direction_body", 3);
      header_vector("weak_translation_direction_world", 3);
      header_vector("cumulative_dv_025", 3);
      fout_lio_motion_consistency << ",cumulative_dv_025_norm";
      header_vector("cumulative_dv_050", 3);
      fout_lio_motion_consistency << ",cumulative_dv_050_norm";
      header_vector("cumulative_dv_100", 3);
      fout_lio_motion_consistency
          << ",cumulative_dv_100_norm,consecutive_dv_direction_cosine,"
             "dv_direction_persistence_1s,dv_velocity_cosine,dv_velocity_angle_deg,"
             "dv_dp_cosine,dv_dp_angle_deg,dv_dp_norm_ratio\n";
    }
    else
    {
      ROS_WARN("[LIO_MOTION_SHADOW] Failed to open %smotion_consistency_shadow.csv",
               save_path.c_str());
    }
    fout_runtime_events.open(save_path + "runtime_event_counts.csv", std::ios::out);
    if (fout_runtime_events.is_open())
    {
      fout_runtime_events
          << "timestamp,final,lidar_received,imu_received,image_received,image_synced,"
             "image_processed,lio_attempted,lio_committed,lio_rejected,vio_attempted,"
             "vio_accepted,vio_rejected,buffer_overflow\n";
    }
    if (voxelmap_manager && voxelmap_manager->config_setting_.directional_shadow_enable)
    {
      fout_lio_directional_shadow.open(
          save_path + "lio_directional_shadow.csv", std::ios::out);
      if (fout_lio_directional_shadow.is_open())
      {
        fout_lio_directional_shadow
            << "timestamp,frame_id,iteration_index,iteration_count,valid,relative_threshold,"
               "effective_feature_count,residual_mean,residual_median,residual_p90,residual_rmse,"
               "measurement_variance_mean,measurement_variance_median,"
               "rotation_eigenvalue_0,rotation_eigenvalue_1,rotation_eigenvalue_2,"
               "rotation_eigenvalue_ratio,rotation_condition,weak_rotation_x,weak_rotation_y,weak_rotation_z,"
               "translation_eigenvalue_0,translation_eigenvalue_1,translation_eigenvalue_2,"
               "translation_eigenvalue_ratio,translation_condition,weak_translation_x,weak_translation_y,"
               "weak_translation_z,weak_translation_vertical_abs,"
               "rotation_weight_0,rotation_weight_1,rotation_weight_2,"
               "translation_weight_0,translation_weight_1,translation_weight_2,"
               "affected_direction_count,partial_suppression_count,full_suppression_count,"
               "information_trace_raw,information_trace_shadow,information_trace_retained_ratio,"
               "rotation_information_trace_raw,rotation_information_trace_shadow,"
               "translation_information_trace_raw,translation_information_trace_shadow,"
               "raw_dx,raw_dy,raw_dz,raw_dp_norm,raw_droll_deg,raw_dpitch_deg,raw_dyaw_deg,raw_dR_deg,"
               "shadow_dx,shadow_dy,shadow_dz,shadow_dp_norm,shadow_droll_deg,shadow_dpitch_deg,"
               "shadow_dyaw_deg,shadow_dR_deg,removed_dx,removed_dy,removed_dz,removed_dp_norm,"
               "removed_droll_deg,removed_dpitch_deg,removed_dyaw_deg,removed_dR_deg,"
               "raw_weak_translation_projection,shadow_weak_translation_projection,"
               "raw_weak_rotation_projection_deg,shadow_weak_rotation_projection_deg,"
               "raw_posterior_pose_cov_trace,shadow_posterior_pose_cov_trace,"
               "raw_posterior_rotation_cov_eigenvalue_0,raw_posterior_rotation_cov_eigenvalue_1,"
               "raw_posterior_rotation_cov_eigenvalue_2,shadow_posterior_rotation_cov_eigenvalue_0,"
               "shadow_posterior_rotation_cov_eigenvalue_1,shadow_posterior_rotation_cov_eigenvalue_2,"
               "raw_posterior_translation_cov_eigenvalue_0,raw_posterior_translation_cov_eigenvalue_1,"
               "raw_posterior_translation_cov_eigenvalue_2,shadow_posterior_translation_cov_eigenvalue_0,"
               "shadow_posterior_translation_cov_eigenvalue_1,shadow_posterior_translation_cov_eigenvalue_2,"
               "raw_posterior_cov_diag_r0,raw_posterior_cov_diag_r1,raw_posterior_cov_diag_r2,"
               "raw_posterior_cov_diag_t0,raw_posterior_cov_diag_t1,raw_posterior_cov_diag_t2,"
               "shadow_posterior_cov_diag_r0,shadow_posterior_cov_diag_r1,shadow_posterior_cov_diag_r2,"
               "shadow_posterior_cov_diag_t0,shadow_posterior_cov_diag_t1,shadow_posterior_cov_diag_t2,"
               "shadow_solve_time_ms\n";
      }
      else
      {
        ROS_WARN("[LIO_SHADOW] Failed to open %slio_directional_shadow.csv",
                 save_path.c_str());
      }
    }
    fout_visual_image_flow.open(save_path + "visual_image_flow.csv", std::ios::out);
    if (fout_visual_image_flow.is_open())
    {
      fout_visual_image_flow << "timestamp,event,detail\n";
    }
    else
    {
      ROS_WARN("[VIO_FLOW] Failed to open %svisual_image_flow.csv", save_path.c_str());
    }
    if (livo_scan_contract_diagnostics_en_)
    {
      fout_livo_scan_contract.open(
          save_path + "livo_scan_contract.csv", std::ios::out);
      if (fout_livo_scan_contract.is_open())
      {
        fout_livo_scan_contract
            << "scan_id,scan_begin_s,scan_end_s,raw_point_count,image_events,"
               "lio_transactions,map_insertions,state_time_monotonic\n";
      }
      else
      {
        ROS_WARN("[LIVO_CONTRACT] Failed to open %slivo_scan_contract.csv",
                 save_path.c_str());
      }
    }
  }
}

void LIVMapper::logVisualImageFlow(double timestamp, const char *event,
                                   const std::string &detail)
{
  if (!fout_visual_image_flow.is_open()) return;
  fout_visual_image_flow << std::setprecision(17) << timestamp << ','
                         << (event ? event : "unknown") << ','
                         << (detail.empty() ? "none" : detail) << '\n';
  if (++visual_image_flow_pending_rows_ >= std::max(1, diagnostics_csv_flush_interval_rows_))
  {
    fout_visual_image_flow.flush();
    visual_image_flow_pending_rows_ = 0;
  }
}

void LIVMapper::logLivoScanContract()
{
  if (!fout_livo_scan_contract.is_open() || !livo_scan_lifecycle_.active())
    return;
  const auto &scan = livo_scan_lifecycle_.snapshot();
  fout_livo_scan_contract << std::setprecision(17)
      << scan.scan_id << ',' << scan.scan_begin_time << ','
      << scan.scan_end_time << ',' << scan.raw_point_count << ','
      << scan.image_events << ',' << scan.lio_transactions << ','
      << scan.map_insertions << ','
      << static_cast<int>(scan.state_time_monotonic) << '\n';
  fout_livo_scan_contract.flush();
}

void LIVMapper::logLioTransaction()
{
  if (!fout_lio_transaction.is_open() || !voxelmap_manager) return;
  const LioUpdateDiagnostics &diagnostics = voxelmap_manager->getLastLioDiagnostics();
  fout_lio_transaction << std::setprecision(17)
      << LidarMeasures.last_lio_update_time << ','
      << voxelmap_manager->current_frame_id_ << ','
      << diagnostics.iteration_count << ',' << diagnostics.convergence_status << ','
      << static_cast<int>(diagnostics.converged) << ','
      << static_cast<int>(diagnostics.reached_iteration_limit) << ','
      << diagnostics.correspondence_count << ',' << diagnostics.valid_residual_count << ','
      << diagnostics.cost_before << ',' << diagnostics.cost_after << ','
      << diagnostics.translation_increment_norm << ','
      << diagnostics.rotation_increment_deg << ','
      << diagnostics.velocity_increment_norm << ','
      << diagnostics.covariance_trace_before << ','
      << diagnostics.covariance_trace_after << ','
      << diagnostics.covariance_min_diagonal << ','
      << diagnostics.covariance_max_diagonal << ','
      << diagnostics.covariance_min_eigenvalue << ','
      << diagnostics.covariance_asymmetry << ','
      << static_cast<int>(diagnostics.map_inserted) << ','
      << static_cast<int>(diagnostics.commit) << '\n';
  if (++lio_transaction_pending_rows_ >= diagnostics_csv_flush_interval_rows_)
  {
    fout_lio_transaction.flush();
    lio_transaction_pending_rows_ = 0;
  }
}

void LIVMapper::logLioMotionConsistency()
{
  if (!fout_lio_motion_consistency.is_open() || !voxelmap_manager) return;
  const LioUpdateDiagnostics &diagnostics =
      voxelmap_manager->getLastLioDiagnostics();
  const fast_livo::LioMotionConsistencyMetrics &motion =
      diagnostics.motion_consistency;
  auto write_vector = [&](const auto &value) {
    for (int i = 0; i < value.size(); ++i)
      fout_lio_motion_consistency << ',' << value[i];
  };
  auto write_cross = [&](const fast_livo::CrossBlockSummary &value) {
    fout_lio_motion_consistency << ',' << value.frobenius_norm << ','
        << value.maximum_singular_value;
    write_vector(value.state_direction);
    write_vector(value.coupled_direction);
  };
  auto write_quadratic = [&](const fast_livo::NormalizedQuadratic &value) {
    fout_lio_motion_consistency << ',' << static_cast<int>(value.valid) << ','
        << value.value << ',' << value.rank << ',' << value.minimum_eigenvalue
        << ',' << value.maximum_eigenvalue << ',' << value.condition_number
        << ',' << value.eigenvalue_cutoff;
  };
  auto write_source_counterfactual = [&] (
      const fast_livo::SourceUpdateCounterfactual &value) {
    fout_lio_motion_consistency << ',' << static_cast<int>(value.valid) << ','
        << value.strength;
    write_vector(value.dimensionless_information_eigenvalues);
    write_vector(value.direction_weights);
    write_vector(value.delta_state);
    fout_lio_motion_consistency
        << ',' << value.delta_state.head<6>().norm()
        << ',' << std::fabs(value.delta_state[6])
        << ',' << value.delta_state.segment<3>(7).norm()
        << ',' << value.delta_state.segment<3>(10).norm()
        << ',' << value.delta_state.segment<3>(13).norm()
        << ',' << value.delta_state.segment<3>(16).norm()
        << ',' << value.pose_difference_norm
        << ',' << value.velocity_difference_norm
        << ',' << value.localized_prior_min_eigenvalue
        << ',' << value.localized_prior_asymmetry
        << ',' << value.posterior_min_eigenvalue
        << ',' << value.posterior_asymmetry
        << ',' << value.posterior_trace
        << ',' << value.pose_posterior_covariance_difference_norm;
  };

  const auto &delta = motion.delta_state;
  fout_lio_motion_consistency << std::setprecision(17)
      << LidarMeasures.last_lio_update_time << ','
      << voxelmap_manager->current_frame_id_ << ','
      << static_cast<int>(motion.shadow_warn) << ','
      << diagnostics.motion_shadow_reason << ','
      << static_cast<int>(motion.valid) << ','
      << static_cast<int>(diagnostics.commit);
  write_vector(delta);
  fout_lio_motion_consistency
      << ',' << delta.head<3>().norm()
      << ',' << delta.head<3>().norm() * 57.29577951308232
      << ',' << delta.segment<3>(3).norm() << ',' << delta[6]
      << ',' << delta.segment<3>(7).norm()
      << ',' << delta.segment<3>(10).norm()
      << ',' << delta.segment<3>(13).norm()
      << ',' << delta.segment<3>(16).norm();
  write_cross(motion.p_v_theta);
  write_cross(motion.p_v_position);
  write_cross(motion.p_v_bg);
  write_cross(motion.p_v_ba);
  write_cross(motion.p_v_gravity);
  fout_lio_motion_consistency
      << ',' << motion.rotation_equivalent_gain_norm
      << ',' << motion.position_equivalent_gain_norm
      << ',' << motion.velocity_equivalent_gain_norm
      << ',' << motion.bias_g_equivalent_gain_norm
      << ',' << motion.bias_a_equivalent_gain_norm
      << ',' << motion.gravity_equivalent_gain_norm;
  write_vector(motion.innovation_component);
  write_vector(motion.relinearization_component);
  write_vector(motion.accumulated_innovation_component);
  write_vector(motion.accumulated_relinearization_component);
  fout_lio_motion_consistency
      << ',' << motion.analyzed_iteration_count
      << ',' << motion.maximum_iteration_linearized_nis_per_dof
      << ',' << motion.mean_iteration_linearized_nis_per_dof
      << ',' << motion.maximum_iteration_rotation_equivalent_gain_norm
      << ',' << motion.maximum_iteration_position_equivalent_gain_norm
      << ',' << motion.maximum_iteration_velocity_equivalent_gain_norm
      << ',' << motion.maximum_iteration_velocity_innovation_component_norm;
  write_quadratic(motion.q_velocity_prior);
  write_quadratic(motion.q_pose_prior);
  write_quadratic(motion.q_bias_prior);
  write_quadratic(motion.q_gravity_prior);
  write_quadratic(motion.q_state_prior);
  write_quadratic(motion.q_velocity_correction_approx);
  write_vector(motion.correction_covariance_eigenvalues);
  write_vector(motion.velocity_correction_covariance_eigenvalues);
  fout_lio_motion_consistency
      << ',' << static_cast<int>(motion.correction_covariance_psd)
      << ',' << motion.correction_covariance_rank
      << ',' << motion.correction_covariance_asymmetry
      << ',' << motion.residual_weighted_energy
      << ',' << motion.information_explained_energy
      << ',' << motion.linearized_nis
      << ',' << motion.linearized_nis_per_dof
      << ',' << static_cast<int>(motion.linearized_nis_valid);
  write_vector(motion.pose_information_eigenvalues);
  write_vector(motion.pose_information_weak_direction);
  write_vector(motion.pose_correction_eigen_projections);
  const auto &directions = motion.pose_direction_correction;
  fout_lio_motion_consistency << ',' << static_cast<int>(directions.valid);
  write_vector(directions.eigenvalues);
  for (int direction = 0; direction < 6; ++direction)
  {
    write_vector(directions.eigenvectors.col(direction));
    write_vector(directions.state_contributions.block<3, 1>(7, direction));
    fout_lio_motion_consistency << ','
        << directions.state_contributions.block<3, 1>(7, direction).norm();
  }
  write_vector(directions.unapplied_iteration_carry);
  fout_lio_motion_consistency
      << ',' << directions.full_closure_norm
      << ',' << directions.velocity_closure_norm
      << ',' << directions.applied_full_closure_norm
      << ',' << directions.applied_velocity_closure_norm
      << ',' << motion.source_counterfactual_compute_time_ms;
  for (const auto &counterfactual : motion.transfer_counterfactuals)
    write_source_counterfactual(counterfactual);
  for (const auto &counterfactual : motion.damping_counterfactuals)
    write_source_counterfactual(counterfactual);
  fout_lio_motion_consistency
      << ',' << motion.pose_information_rank
      << ',' << motion.pose_information_condition_number
      << ',' << static_cast<int>(diagnostics.raw_is_degenerate)
      << ',' << static_cast<int>(diagnostics.is_degenerate)
      << ',' << static_cast<int>(diagnostics.observability.valid);
  write_vector(diagnostics.observability.rotation_eigenvalues);
  write_vector(diagnostics.observability.translation_eigenvalues);
  fout_lio_motion_consistency
      << ',' << diagnostics.observability.rotation_eigenvalue_ratio
      << ',' << diagnostics.observability.rotation_condition_number
      << ',' << diagnostics.observability.translation_eigenvalue_ratio
      << ',' << diagnostics.observability.translation_condition_number;
  write_vector(diagnostics.observability.weak_rotation_direction_body);
  write_vector(diagnostics.observability.weak_translation_direction_world);
  write_vector(motion.temporal.cumulative_dv_025);
  fout_lio_motion_consistency << ','
      << motion.temporal.cumulative_dv_025.norm();
  write_vector(motion.temporal.cumulative_dv_050);
  fout_lio_motion_consistency << ','
      << motion.temporal.cumulative_dv_050.norm();
  write_vector(motion.temporal.cumulative_dv_100);
  fout_lio_motion_consistency
      << ',' << motion.temporal.cumulative_dv_100.norm()
      << ',' << motion.temporal.consecutive_direction_cosine
      << ',' << motion.temporal.direction_persistence_1s
      << ',' << motion.temporal.dv_velocity_cosine
      << ',' << motion.temporal.dv_velocity_angle_deg
      << ',' << motion.temporal.dv_dp_cosine
      << ',' << motion.temporal.dv_dp_angle_deg
      << ',' << motion.temporal.dv_dp_norm_ratio << '\n';
  if (++lio_motion_consistency_pending_rows_ >=
      diagnostics_csv_flush_interval_rows_)
  {
    fout_lio_motion_consistency.flush();
    lio_motion_consistency_pending_rows_ = 0;
  }
}

void LIVMapper::logRuntimeEventCounts(bool final_snapshot)
{
  const bool periodic_snapshot = runtime_events_.lio_attempted > 0 &&
      runtime_events_.lio_attempted % 200 == 0;
  if (!final_snapshot && !periodic_snapshot) return;
  const double timestamp = LidarMeasures.last_lio_update_time;
  if (fout_runtime_events.is_open())
  {
    fout_runtime_events << std::setprecision(17) << timestamp << ','
        << static_cast<int>(final_snapshot) << ','
        << runtime_events_.lidar_received << ',' << runtime_events_.imu_received << ','
        << runtime_events_.image_received << ',' << runtime_events_.image_synced << ','
        << runtime_events_.image_processed << ',' << runtime_events_.lio_attempted << ','
        << runtime_events_.lio_committed << ',' << runtime_events_.lio_rejected << ','
        << runtime_events_.vio_attempted << ',' << runtime_events_.vio_accepted << ','
        << runtime_events_.vio_rejected << ',' << runtime_events_.buffer_overflow << '\n';
    if (final_snapshot) fout_runtime_events.flush();
  }
  if (final_snapshot || periodic_snapshot)
  {
    ROS_INFO("[EVENT_COUNTS] final=%d lidar=%lu imu=%lu image=%lu image_synced=%lu image_processed=%lu lio=%lu/%lu/%lu vio=%lu/%lu/%lu overflow=%lu",
             static_cast<int>(final_snapshot), runtime_events_.lidar_received,
             runtime_events_.imu_received, runtime_events_.image_received,
             runtime_events_.image_synced, runtime_events_.image_processed,
             runtime_events_.lio_attempted, runtime_events_.lio_committed,
             runtime_events_.lio_rejected, runtime_events_.vio_attempted,
             runtime_events_.vio_accepted, runtime_events_.vio_rejected,
             runtime_events_.buffer_overflow);
  }
}

void LIVMapper::logLioDegeneracy(bool map_insert_skipped,
                                 const std::string &map_insert_skip_reason,
                                 bool map_guard_requested,
                                 bool map_guard_enforced)
{
  if (!voxelmap_manager) return;
  const LioUpdateDiagnostics &diagnostics = voxelmap_manager->getLastLioDiagnostics();
  const StatesGroup &predicted = diagnostics.predicted_state;
  const StatesGroup &updated = diagnostics.updated_state;
  const V3D delta_position = updated.pos_end - predicted.pos_end;
  const V3D delta_velocity = updated.vel_end - predicted.vel_end;
  const M3D delta_rotation = predicted.rot_end.transpose() * updated.rot_end;
  const V3D delta_rotation_vector = Log(delta_rotation);
  constexpr double rad_to_deg = 57.29577951308232;
  const double delta_pitch = std::asin(std::min(1.0, std::max(-1.0, -delta_rotation(2, 0))));
  const double delta_roll = std::atan2(delta_rotation(2, 1), delta_rotation(2, 2));
  const double delta_yaw = std::atan2(delta_rotation(1, 0), delta_rotation(0, 0));
  const auto &metrics = diagnostics.observability;
  V3D weak_translation_body = V3D::Zero();
  if (metrics.valid)
    weak_translation_body = predicted.rot_end.transpose() * metrics.weak_translation_direction_world;
  const double timestamp = LidarMeasures.last_lio_update_time;
  const int lio_frame_id = voxelmap_manager->current_frame_id_;

  std::ostringstream line;
  line << std::setprecision(9)
       << "[LIO_DEGEN] timestamp=" << timestamp
       << " frame_id=" << lio_frame_id
       << " effective_feature_count=" << diagnostics.effective_feature_count
       << " input_feature_count=" << diagnostics.input_feature_count
       << " downsampled_feature_count=" << diagnostics.downsampled_feature_count
       << " inlier_ratio=" << diagnostics.inlier_ratio
       << " average_point_plane_residual=" << diagnostics.average_point_plane_residual
       << " predicted_position=(" << predicted.pos_end.transpose() << ")"
       << " predicted_velocity=(" << predicted.vel_end.transpose() << ")"
       << " updated_position=(" << updated.pos_end.transpose() << ")"
       << " updated_velocity=(" << updated.vel_end.transpose() << ")"
       << " delta_position=(" << delta_position.transpose() << ")"
       << " delta_velocity=(" << delta_velocity.transpose() << ")"
       << " delta_rotation_vector=(" << delta_rotation_vector.transpose() << ")"
       << " rotation_eigenvalues=(" << metrics.rotation_eigenvalues.transpose() << ")"
       << " translation_eigenvalues=(" << metrics.translation_eigenvalues.transpose() << ")"
       << " translation_eigenvalue_ratio=" << metrics.translation_eigenvalue_ratio
       << " translation_condition_number=" << metrics.translation_condition_number
       << " weak_translation_direction_world=(" << metrics.weak_translation_direction_world.transpose() << ")"
       << " weak_rotation_direction_body=(" << metrics.weak_rotation_direction_body.transpose() << ")"
       << " predicted_speed_mps=" << diagnostics.predicted_speed_mps
       << " velocity_weak_direction_cos=" << diagnostics.velocity_weak_direction_cos
       << " raw_position_correction=(" << diagnostics.raw_position_correction.transpose() << ")"
       << " raw_velocity_correction=(" << diagnostics.raw_velocity_correction.transpose() << ")"
       << " velocity_projection_on_weak_direction=" << diagnostics.velocity_projection_on_weak_direction
       << " position_correction_on_weak_direction=" << diagnostics.position_correction_on_weak_direction
       << " is_degenerate=" << static_cast<int>(diagnostics.is_degenerate)
       << " is_severely_degenerate=" << static_cast<int>(diagnostics.is_severely_degenerate)
       << " direction_conflict=" << static_cast<int>(diagnostics.direction_conflict)
       << " direction_conflict_consecutive_frames=" << diagnostics.direction_conflict_consecutive_frames
       << " state_intervention_applied=" << static_cast<int>(diagnostics.state_intervention_applied)
       << " map_guard_requested=" << static_cast<int>(map_guard_requested)
       << " map_guard_enforced=" << static_cast<int>(map_guard_enforced)
       << " map_insert_skipped=" << static_cast<int>(map_insert_skipped)
       << " map_insert_skip_reason=" << map_insert_skip_reason;
  if (lio_frame_id % diagnostics_console_interval_frames_ == 0)
    std::cout << line.str() << std::endl;

  if (!fout_lio_degeneracy.is_open()) return;
  fout_lio_degeneracy << std::setprecision(12)
      << timestamp << ',' << lio_frame_id << ','
      << diagnostics.effective_feature_count << ',' << diagnostics.average_point_plane_residual << ','
      << predicted.pos_end.x() << ',' << predicted.pos_end.y() << ',' << predicted.pos_end.z() << ','
      << predicted.vel_end.x() << ',' << predicted.vel_end.y() << ',' << predicted.vel_end.z() << ','
      << updated.pos_end.x() << ',' << updated.pos_end.y() << ',' << updated.pos_end.z() << ','
      << updated.vel_end.x() << ',' << updated.vel_end.y() << ',' << updated.vel_end.z() << ','
      << delta_position.x() << ',' << delta_position.y() << ',' << delta_position.z() << ','
      << delta_velocity.x() << ',' << delta_velocity.y() << ',' << delta_velocity.z() << ','
      << metrics.rotation_eigenvalues[0] << ',' << metrics.rotation_eigenvalues[1] << ','
      << metrics.rotation_eigenvalues[2] << ','
      << metrics.translation_eigenvalues[0] << ',' << metrics.translation_eigenvalues[1] << ','
      << metrics.translation_eigenvalues[2] << ',' << metrics.translation_eigenvalue_ratio << ','
      << metrics.translation_condition_number << ','
      << metrics.weak_translation_direction_world.x() << ','
      << metrics.weak_translation_direction_world.y() << ','
      << metrics.weak_translation_direction_world.z() << ','
      << diagnostics.predicted_speed_mps << ','
      << diagnostics.velocity_weak_direction_cos << ','
      << diagnostics.raw_position_correction.x() << ','
      << diagnostics.raw_position_correction.y() << ','
      << diagnostics.raw_position_correction.z() << ','
      << diagnostics.raw_velocity_correction.x() << ','
      << diagnostics.raw_velocity_correction.y() << ','
      << diagnostics.raw_velocity_correction.z() << ','
      << diagnostics.velocity_projection_on_weak_direction << ','
      << diagnostics.position_correction_on_weak_direction << ','
      << diagnostics.translation_information_weights[0] << ','
      << diagnostics.translation_information_weights[1] << ','
      << diagnostics.translation_information_weights[2] << ','
      << static_cast<int>(diagnostics.raw_is_degenerate) << ','
      << static_cast<int>(diagnostics.is_degenerate) << ','
      << static_cast<int>(diagnostics.is_severely_degenerate) << ','
      << static_cast<int>(diagnostics.direction_conflict) << ','
      << diagnostics.direction_conflict_consecutive_frames << ','
      << static_cast<int>(diagnostics.direction_guard_triggered) << ','
      << static_cast<int>(diagnostics.state_intervention_applied) << ','
      << static_cast<int>(diagnostics.update_was_suppressed) << ','
      << static_cast<int>(diagnostics.update_was_significantly_suppressed) << ','
      << static_cast<int>(map_guard_requested) << ','
      << static_cast<int>(map_guard_enforced) << ','
      << static_cast<int>(map_insert_skipped) << ',' << map_insert_skip_reason << ','
      << diagnostics.input_feature_count << ','
      << diagnostics.downsampled_feature_count << ','
      << diagnostics.valid_plane_count << ','
      << diagnostics.observability_feature_count << ','
      << diagnostics.inlier_ratio << ','
      << diagnostics.median_abs_point_plane_residual << ','
      << diagnostics.p90_abs_point_plane_residual << ','
      << diagnostics.p95_abs_point_plane_residual << ','
      << diagnostics.point_plane_residual_rmse << ','
      << diagnostics.max_abs_point_plane_residual << ','
      << diagnostics.measurement_variance_mean << ','
      << diagnostics.measurement_variance_median << ','
      << diagnostics.measurement_variance_p90 << ','
      << diagnostics.measurement_variance_p95 << ','
      << diagnostics.measurement_variance_min << ','
      << diagnostics.measurement_variance_max << ','
      << metrics.rotation_eigenvalue_ratio << ','
      << metrics.rotation_condition_number << ','
      << metrics.weak_rotation_direction_body.x() << ','
      << metrics.weak_rotation_direction_body.y() << ','
      << metrics.weak_rotation_direction_body.z() << ','
      << weak_translation_body.x() << ','
      << weak_translation_body.y() << ','
      << weak_translation_body.z() << ','
      << std::fabs(metrics.weak_translation_direction_world.z()) << ','
      << delta_rotation_vector.x() << ','
      << delta_rotation_vector.y() << ','
      << delta_rotation_vector.z() << ','
      << delta_roll * rad_to_deg << ','
      << delta_pitch * rad_to_deg << ','
      << delta_yaw * rad_to_deg << ','
      << delta_rotation_vector.norm() * rad_to_deg << '\n';
  ++lio_diagnostics_pending_rows_;
  if (lio_diagnostics_pending_rows_ >= diagnostics_csv_flush_interval_rows_)
  {
    fout_lio_degeneracy.flush();
    lio_diagnostics_pending_rows_ = 0;
  }
}

void LIVMapper::logLioDirectionalShadow()
{
  if (!fout_lio_directional_shadow.is_open() || !voxelmap_manager) return;
  const LioUpdateDiagnostics &diagnostics = voxelmap_manager->getLastLioDiagnostics();
  const double timestamp = LidarMeasures.last_lio_update_time;
  const int frame_id = voxelmap_manager->current_frame_id_;
  for (const auto &shadow : diagnostics.directional_shadows)
  {
    const auto &metrics = shadow.observability;
    fout_lio_directional_shadow << std::setprecision(17)
        << timestamp << ',' << frame_id << ',' << shadow.iteration_index << ','
        << shadow.iteration_count << ',' << static_cast<int>(shadow.valid) << ','
        << shadow.relative_threshold << ',' << shadow.effective_feature_count << ','
        << shadow.residual_mean << ',' << shadow.residual_median << ','
        << shadow.residual_p90 << ',' << shadow.residual_rmse << ','
        << shadow.measurement_variance_mean << ','
        << shadow.measurement_variance_median << ','
        << metrics.rotation_eigenvalues[0] << ',' << metrics.rotation_eigenvalues[1] << ','
        << metrics.rotation_eigenvalues[2] << ',' << metrics.rotation_eigenvalue_ratio << ','
        << metrics.rotation_condition_number << ','
        << metrics.weak_rotation_direction_body.x() << ','
        << metrics.weak_rotation_direction_body.y() << ','
        << metrics.weak_rotation_direction_body.z() << ','
        << metrics.translation_eigenvalues[0] << ',' << metrics.translation_eigenvalues[1] << ','
        << metrics.translation_eigenvalues[2] << ',' << metrics.translation_eigenvalue_ratio << ','
        << metrics.translation_condition_number << ','
        << metrics.weak_translation_direction_world.x() << ','
        << metrics.weak_translation_direction_world.y() << ','
        << metrics.weak_translation_direction_world.z() << ','
        << std::fabs(metrics.weak_translation_direction_world.z()) << ','
        << shadow.rotation_weights[0] << ',' << shadow.rotation_weights[1] << ','
        << shadow.rotation_weights[2] << ',' << shadow.translation_weights[0] << ','
        << shadow.translation_weights[1] << ',' << shadow.translation_weights[2] << ','
        << shadow.affected_direction_count << ',' << shadow.partial_suppression_count << ','
        << shadow.full_suppression_count << ',' << shadow.information_trace_raw << ','
        << shadow.information_trace_shadow << ','
        << shadow.information_trace_retained_ratio << ','
        << shadow.rotation_information_trace_raw << ','
        << shadow.rotation_information_trace_shadow << ','
        << shadow.translation_information_trace_raw << ','
        << shadow.translation_information_trace_shadow << ','
        << shadow.raw_delta_position.x() << ',' << shadow.raw_delta_position.y() << ','
        << shadow.raw_delta_position.z() << ',' << shadow.raw_delta_position_norm << ','
        << shadow.raw_delta_rpy_deg.x() << ',' << shadow.raw_delta_rpy_deg.y() << ','
        << shadow.raw_delta_rpy_deg.z() << ',' << shadow.raw_delta_rotation_deg << ','
        << shadow.shadow_delta_position.x() << ',' << shadow.shadow_delta_position.y() << ','
        << shadow.shadow_delta_position.z() << ',' << shadow.shadow_delta_position_norm << ','
        << shadow.shadow_delta_rpy_deg.x() << ',' << shadow.shadow_delta_rpy_deg.y() << ','
        << shadow.shadow_delta_rpy_deg.z() << ',' << shadow.shadow_delta_rotation_deg << ','
        << shadow.removed_delta_position.x() << ',' << shadow.removed_delta_position.y() << ','
        << shadow.removed_delta_position.z() << ',' << shadow.removed_delta_position_norm << ','
        << shadow.removed_delta_rpy_deg.x() << ',' << shadow.removed_delta_rpy_deg.y() << ','
        << shadow.removed_delta_rpy_deg.z() << ',' << shadow.removed_delta_rotation_deg << ','
        << shadow.raw_weak_translation_projection << ','
        << shadow.shadow_weak_translation_projection << ','
        << shadow.raw_weak_rotation_projection_deg << ','
        << shadow.shadow_weak_rotation_projection_deg << ','
        << shadow.raw_posterior_pose_cov_trace << ','
        << shadow.shadow_posterior_pose_cov_trace << ','
        << shadow.raw_posterior_rotation_cov_eigenvalues[0] << ','
        << shadow.raw_posterior_rotation_cov_eigenvalues[1] << ','
        << shadow.raw_posterior_rotation_cov_eigenvalues[2] << ','
        << shadow.shadow_posterior_rotation_cov_eigenvalues[0] << ','
        << shadow.shadow_posterior_rotation_cov_eigenvalues[1] << ','
        << shadow.shadow_posterior_rotation_cov_eigenvalues[2] << ','
        << shadow.raw_posterior_translation_cov_eigenvalues[0] << ','
        << shadow.raw_posterior_translation_cov_eigenvalues[1] << ','
        << shadow.raw_posterior_translation_cov_eigenvalues[2] << ','
        << shadow.shadow_posterior_translation_cov_eigenvalues[0] << ','
        << shadow.shadow_posterior_translation_cov_eigenvalues[1] << ','
        << shadow.shadow_posterior_translation_cov_eigenvalues[2] << ',';
    for (int i = 0; i < 6; ++i)
      fout_lio_directional_shadow << shadow.raw_posterior_pose_cov_diagonal[i] << ',';
    for (int i = 0; i < 6; ++i)
      fout_lio_directional_shadow << shadow.shadow_posterior_pose_cov_diagonal[i] << ',';
    fout_lio_directional_shadow << shadow.solve_time_ms << '\n';
    ++lio_directional_shadow_pending_rows_;
  }
  if (lio_directional_shadow_pending_rows_ >= diagnostics_csv_flush_interval_rows_)
  {
    fout_lio_directional_shadow.flush();
    lio_directional_shadow_pending_rows_ = 0;
  }
}

void LIVMapper::initializeUdpReporter()
{
  if (!udp_report_en) return;
  if (udp_target_ip_.empty() || udp_report_port_ <= 0 || udp_report_port_ > 65535)
  {
    ROS_WARN("[UDP] Invalid target config, reporting disabled.");
    udp_report_en = false;
    return;
  }

  udp_socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket_fd_ < 0)
  {
    ROS_WARN("[UDP] Failed to create socket, reporting disabled.");
    udp_report_en = false;
    return;
  }

  udp_target_addr_ = {};
  udp_target_addr_.sin_family = AF_INET;
  udp_target_addr_.sin_port = htons(static_cast<uint16_t>(udp_report_port_));
  if (::inet_pton(AF_INET, udp_target_ip_.c_str(), &udp_target_addr_.sin_addr) != 1)
  {
    ROS_WARN("[UDP] Invalid target IP '%s', reporting disabled.", udp_target_ip_.c_str());
    ::close(udp_socket_fd_);
    udp_socket_fd_ = -1;
    udp_report_en = false;
    return;
  }

  udp_socket_ready_ = true;
  sendUdpMessage(std::string("DEVICE_ID ") + udp_device_id_);
  ROS_INFO("[UDP] Reporting enabled for %s:%d", udp_target_ip_.c_str(), udp_report_port_);
}

void LIVMapper::sendUdpMessage(const std::string &message)
{
  if (!udp_report_en || !udp_socket_ready_ || udp_socket_fd_ < 0 || message.empty()) return;

  const ssize_t sent = ::sendto(udp_socket_fd_, message.c_str(), message.size(), 0,
                                reinterpret_cast<const sockaddr *>(&udp_target_addr_), sizeof(udp_target_addr_));
  if (sent < 0)
  {
    ROS_WARN_THROTTLE(5.0, "[UDP] Failed to send packet to %s:%d.", udp_target_ip_.c_str(), udp_report_port_);
  }
}

void LIVMapper::sendUdpPose(const Eigen::Vector3d &position)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "POSE " << ros::Time::now().toSec() << ' '
      << position.x() << ' ' << position.y() << ' ' << position.z();
  sendUdpMessage(oss.str());
}

void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it) 
{
  if (lidar_en)
  {
    sub_pcl = p_pre->lidar_type == AVIA ?
              nh.subscribe(lid_topic, sub_lidar_queue_size_, &LIVMapper::livox_pcl_cbk, this):
              nh.subscribe(lid_topic, sub_lidar_queue_size_, &LIVMapper::standard_pcl_cbk, this);
  }
  if (imu_en) sub_imu = nh.subscribe(imu_topic, sub_imu_queue_size_, &LIVMapper::imu_cbk, this);
  if (img_en) sub_img = nh.subscribe(img_topic, sub_img_queue_size_, &LIVMapper::img_cbk, this);
  
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);
  pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/mapping/globalMap", 100);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  nh.param<std::string>("rtk_backend/raw_odom_topic", raw_backend_odom_topic_,
                        raw_backend_odom_topic_);
  nh.param<std::string>("rtk_backend/odom_frame_id",
                        raw_backend_odom_frame_id_,
                        raw_backend_odom_frame_id_);
  nh.param<std::string>("rtk_backend/body_frame_id",
                        raw_backend_body_frame_id_,
                        raw_backend_body_frame_id_);
  pubRawBackendOdom =
      nh.advertise<nav_msgs::Odometry>(raw_backend_odom_topic_, 100);
  if (fullstate_shadow_enable_)
    pubFullStateShadowGeometry =
        nh.advertise<fast_livo::FullStateLidarGeometry>(
            fullstate_shadow_geometry_topic_, 100);
  pubPath = nh.advertise<nav_msgs::Path>("/mapping/path", 10);
  plane_pub = nh.advertise<visualization_msgs::Marker>("/planner_normal", 1);
  voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = nh.advertise<nav_msgs::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);
  voxelmap_manager->voxel_map_pub_= nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
  pubSubscribersReady = nh.advertise<std_msgs::Bool>("/fast_livo/subscribers_ready", 1, true);
  pubMappingReady = nh.advertise<std_msgs::Bool>("/fast_livo/mapping_ready", 1, true);
  std_msgs::Bool status;
  status.data = false;
  pubMappingReady.publish(status);
  status.data = true;
  pubSubscribersReady.publish(status);
  ROS_INFO("[STARTUP] SUBSCRIBERS_READY lidar=%d imu=%d image=%d",
           lidar_en, static_cast<int>(imu_en), img_en);
}

bool LIVMapper::publishRawBackendOdometry()
{
  nav_msgs::Odometry odometry;
  odometry.header.stamp.fromSec(LidarMeasures.last_lio_update_time);
  odometry.header.frame_id = raw_backend_odom_frame_id_;
  odometry.child_frame_id = raw_backend_body_frame_id_;
  odometry.pose.pose.position.x = _state.pos_end.x();
  odometry.pose.pose.position.y = _state.pos_end.y();
  odometry.pose.pose.position.z = _state.pos_end.z();
  Eigen::Quaterniond quaternion(_state.rot_end);
  quaternion.normalize();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();

  ++raw_backend_odom_attempted_;
  const std::int64_t stamp_ns =
      static_cast<std::int64_t>(odometry.header.stamp.toNSec());
  if (last_raw_backend_odom_stamp_ns_ >= 0 &&
      stamp_ns <= last_raw_backend_odom_stamp_ns_)
  {
    if (stamp_ns == last_raw_backend_odom_stamp_ns_)
      ++raw_backend_odom_duplicate_;
    else
      ++raw_backend_odom_non_monotonic_;
    ROS_INFO_STREAM_THROTTLE(
        5.0, "[RAW_BACKEND_ODOM] suppressed duplicate="
                 << raw_backend_odom_duplicate_
                 << " non_monotonic=" << raw_backend_odom_non_monotonic_
                 << " attempted=" << raw_backend_odom_attempted_
                 << " published=" << raw_backend_odom_published_
                 << " last_stamp_ns=" << last_raw_backend_odom_stamp_ns_);
    return false;
  }

  last_raw_backend_odom_stamp_ns_ = stamp_ns;
  pubRawBackendOdom.publish(odometry);
  ++raw_backend_odom_published_;
  return true;
}

void LIVMapper::publishFullStateShadowGeometry()
{
  if (!fullstate_shadow_enable_ || !pubFullStateShadowGeometry) return;
  const LioUpdateDiagnostics &diagnostics =
      voxelmap_manager->getLastLioDiagnostics();
  fast_livo::FullStateLidarGeometry message;
  message.header.stamp.fromSec(LidarMeasures.last_lio_update_time);
  message.header.frame_id = raw_backend_odom_frame_id_;
  message.factor_source = "L1_FINAL_POINT_TO_PLANE";
  message.geometry_valid = diagnostics.lidar_geometry_valid;
  message.production_commit = diagnostics.commit;
  // StateEstimation finishes before handleLIO mutates the voxel map.
  message.frozen_submap = true;
  message.correspondence_count = std::max(0, diagnostics.correspondence_count);
  message.residual_dof =
      std::max(0, diagnostics.lidar_geometry_residual_dof);

  const StatesGroup &linearization =
      diagnostics.lidar_geometry_linearization_state;
  Eigen::Quaterniond quaternion(linearization.rot_end);
  quaternion.normalize();
  message.linearization_pose.position.x = linearization.pos_end.x();
  message.linearization_pose.position.y = linearization.pos_end.y();
  message.linearization_pose.position.z = linearization.pos_end.z();
  message.linearization_pose.orientation.w = quaternion.w();
  message.linearization_pose.orientation.x = quaternion.x();
  message.linearization_pose.orientation.y = quaternion.y();
  message.linearization_pose.orientation.z = quaternion.z();

  Eigen::Quaterniond production_quaternion(_state.rot_end);
  production_quaternion.normalize();
  message.production_pose.position.x = _state.pos_end.x();
  message.production_pose.position.y = _state.pos_end.y();
  message.production_pose.position.z = _state.pos_end.z();
  message.production_pose.orientation.w = production_quaternion.w();
  message.production_pose.orientation.x = production_quaternion.x();
  message.production_pose.orientation.y = production_quaternion.y();
  message.production_pose.orientation.z = production_quaternion.z();

  auto assign_vector = [](geometry_msgs::Vector3 &destination,
                          const V3D &source) {
    destination.x = source.x();
    destination.y = source.y();
    destination.z = source.z();
  };
  assign_vector(message.production_velocity, _state.vel_end);
  assign_vector(message.production_gyro_bias, _state.bias_g);
  assign_vector(message.production_accel_bias, _state.bias_a);
  assign_vector(message.gravity, _state.gravity);
  message.accel_scale =
      p_imu->IMU_mean_acc_norm > 1e-12
          ? G_m_s2 / p_imu->IMU_mean_acc_norm
          : std::numeric_limits<double>::quiet_NaN();
  for (int row = 0; row < 6; ++row)
  {
    message.pose_rhs[row] = diagnostics.lidar_geometry_rhs[row];
    for (int column = 0; column < 6; ++column)
      message.pose_information[row * 6 + column] =
          diagnostics.lidar_geometry_information(row, column);
  }
  message.residual_weighted_energy =
      diagnostics.lidar_geometry_residual_weighted_energy;
  pubFullStateShadowGeometry.publish(message);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  const double event_time = LidarMeasures.lio_vio_flg != VIO
      ? LidarMeasures.measures.back().lio_time
      : LidarMeasures.measures.back().vio_time;
  if (event_time < LidarMeasures.last_lio_update_time - 1e-6)
    throw std::logic_error("sensor event would move the estimator timestamp backwards");

  p_imu->Process2(LidarMeasures, _state, feats_undistort);
  // IMU initialization returns before UndistortPcl updates this boundary.
  // The scheduler still owns the event and has consumed IMU through it.
  LidarMeasures.last_lio_update_time = event_time;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == LIO &&
      livo_scan_lifecycle_.active() &&
      livo_scan_lifecycle_.snapshot().measurement_required)
    livo_scan_lifecycle_.noteLioTransaction(event_time);

  if (voxelmap_manager->config_setting_.p4_frontend_diagnostics_enable)
  {
    voxelmap_manager->p4SetDeskewDiagnostics(
        p_imu->p4_deskew_diagnostics());
    voxelmap_manager->p4_raw_cloud_ = p_imu->p4_raw_cloud();
    voxelmap_manager->p4_fixed_velocity_cloud_ =
        p_imu->p4_fixed_velocity_cloud();
  }

  if (gravity_align_en) gravityAlignment();

  snapStateForDeterminism(_state);
  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  static int vio_dispatch_count = 0;
  static int lio_dispatch_count = 0;

  switch (LidarMeasures.lio_vio_flg)
  {
    case VIO:
      vio_dispatch_count++;
      if (vio_dispatch_count % 20 == 1)
      {
        std::cout << "[ Flow ] Dispatch VIO frame #" << vio_dispatch_count
                  << " (LIO dispatched=" << lio_dispatch_count << ")" << std::endl;
      }
      handleVIO();
      if (slam_mode_ == LIVO)
        p_imu->updateLidarScanPose(LidarMeasures.last_lio_update_time, _state);
      break;
    case LIO:
    case LO:
      lio_dispatch_count++;
      if (lio_dispatch_count % 50 == 1)
      {
        std::cout << "[ Flow ] Dispatch LIO/LO frame #" << lio_dispatch_count
                  << " (VIO dispatched=" << vio_dispatch_count << ")" << std::endl;
      }
      handleLIO();
      if (slam_mode_ == LIVO) logLivoScanContract();
      break;
  }
  snapStateForDeterminism(_state);
  voxelmap_manager->state_ = _state;
  if (state_update_flg) latest_ekf_state = _state;
}

void LIVMapper::applyUwbUpdate(const char *stage)
{
  if (!uwb_manager || !uwb_manager->updateEnabled()) return;

  V3D output_delta = V3D::Zero();
  UwbUpdateResult result;
  const double current_lidar_stamp = LidarMeasures.last_lio_update_time;
  const double lidar_start_stamp = _first_lidar_time > 0.0 ? _first_lidar_time : current_lidar_stamp;
  const bool lidar_degenerated = voxelmap_manager && voxelmap_manager->isLidarDegenerated();
  uwb_manager->setDegenerateMode(lidar_degenerated || runtime_degraded_mode_);
  if (uwb_output_correction_en_)
  {
    StatesGroup corrected_state = _state;
    corrected_state.pos_end += uwb_output_target_offset_;
    const V3D pos_before = corrected_state.pos_end;
    result = uwb_manager->applyRangeUpdateAt(corrected_state, current_lidar_stamp, lidar_start_stamp);
    if (!result.state_updated && !result.request_relocalization) return;
    if (!result.state_updated && result.request_relocalization)
    {
      // Leave output offset unchanged; the candidate is reported through the common log below.
    }
    else
    {
      output_delta = corrected_state.pos_end - pos_before;
      uwb_output_target_offset_ += output_delta;
      if (!uwb_output_smooth_en_)
      {
        uwb_output_pos_offset_ = uwb_output_target_offset_;
      }
    }
  }
  else
  {
    const V3D pos_before = _state.pos_end;
    result = uwb_manager->applyRangeUpdateAt(_state, current_lidar_stamp, lidar_start_stamp);
    if (result.used_count <= 0 && !result.request_relocalization) return;
    if (!result.state_updated && !result.covariance_updated && !result.request_relocalization) return;
    output_delta = _state.pos_end - pos_before;

    if (result.state_updated || result.covariance_updated)
    {
      snapStateForDeterminism(_state);
      voxelmap_manager->state_ = _state;
      if (vio_manager) vio_manager->updateFrameState(_state);

      if (imu_prop_enable)
      {
        ekf_finish_once = true;
        latest_ekf_state = _state;
        latest_ekf_time = LidarMeasures.last_lio_update_time;
        state_update_flg = true;
      }
    }
  }

  if (result.relocalization_confirmed && result.request_relocalization)
  {
    handleUwbRelocalizationConfirmed(result, stage);
  }

  if (result.state_updated && result.xy_correction_applied > external_update_pause_map_min_correction_m_)
  {
    external_update_pause_map_frames_ = std::max(external_update_pause_map_frames_,
                                                 external_update_pause_map_frames_after_correction_);
    ROS_INFO_THROTTLE(1.0,
                      "[UWB_DEBUG_INJECTION] attempt=%lu action=pause_map_insert correction_norm=%.3f xy_correction_applied=%.3f external_pause_map_update_frames=%d",
                      static_cast<unsigned long>(result.attempt_id),
                      result.correction_norm, result.xy_correction_applied, external_update_pause_map_frames_);
  }

  if (vio_manager)
  {
    std::vector<std::string> lines;
    std::ostringstream oss;
    oss << "[UWB_DEBUG_INJECTION] attempt=" << result.attempt_id
        << " action=" << result.action
        << " used=" << result.used_count
        << " state_updated=" << static_cast<int>(result.state_updated)
        << " covariance_updated=" << static_cast<int>(result.covariance_updated)
        << " residual_rms=" << result.residual_rms
        << " max_abs_residual=" << result.max_abs_residual
        << " xy_raw=" << result.xy_correction_raw
        << " xy_applied=" << result.xy_correction_applied
        << " time_diff=" << result.time_diff
        << " correction_norm=" << result.correction_norm
        << " external_pause_map_update_frames=" << external_update_pause_map_frames_
        << " relocalization_request=" << static_cast<int>(result.request_relocalization)
        << " relocalization_confirmed=" << static_cast<int>(result.relocalization_confirmed)
        << " local_map_reset=" << static_cast<int>(result.local_map_reset)
        << " visual_cache_reset=" << static_cast<int>(result.visual_cache_reset)
        << " covariance_inflated=" << static_cast<int>(result.covariance_inflated)
        << " after " << (stage ? stage : "state") << " update.";
    lines.push_back(oss.str());
    if (uwb_output_correction_en_)
    {
      std::ostringstream offset_oss;
      offset_oss << "[UWB_DEBUG_INJECTION] attempt=" << result.attempt_id
                 << " output_delta=" << output_delta.transpose()
                 << " output_offset=" << uwb_output_pos_offset_.transpose()
                 << " output_target=" << uwb_output_target_offset_.transpose();
      lines.push_back(offset_oss.str());
    }
    vio_manager->appendTimingLogLines(lines);
  }
}

void LIVMapper::applyGnssUpdate(const char *stage)
{
  if (!gnss_manager || !gnss_manager->updateEnabled()) return;

  const V3D pos_before = _state.pos_end;
  const double current_lidar_stamp = LidarMeasures.last_lio_update_time;
  const double lidar_start_stamp = _first_lidar_time > 0.0 ? _first_lidar_time : current_lidar_stamp;
  GnssUpdateResult result = gnss_manager->applyPositionUpdateAt(_state, current_lidar_stamp, lidar_start_stamp);
  if (result.action == "no_measurements" || result.action == "disabled") return;

  const V3D output_delta = _state.pos_end - pos_before;
  if (result.state_updated)
  {
    snapStateForDeterminism(_state);
    voxelmap_manager->state_ = _state;
    if (vio_manager) vio_manager->updateFrameState(_state);

    if (imu_prop_enable)
    {
      ekf_finish_once = true;
      latest_ekf_state = _state;
      latest_ekf_time = LidarMeasures.last_lio_update_time;
      state_update_flg = true;
    }
  }

  if (result.request_pause_map_insert)
  {
    external_update_pause_map_frames_ = std::max(external_update_pause_map_frames_,
                                                 result.pause_map_update_frames);
    ROS_INFO_THROTTLE(1.0,
                      "[GNSS] action=pause_map_insert_after_gnss_correction correction_norm=%.3f external_pause_map_update_frames=%d",
                      result.correction_norm, external_update_pause_map_frames_);
  }

  ROS_INFO_THROTTLE(1.0,
                    "[GNSS] action=%s seq=%d state_updated=%d state=%d convergence=%s residual=%.3f maha=%.3f raw=%.3f applied=%.3f time_diff=%.3f delta=[%.4f %.4f %.4f] after %s update.",
                    result.action.c_str(), result.seq, static_cast<int>(result.state_updated),
                    result.device_state, result.convergence_state.c_str(),
                    result.residual_norm, result.mahalanobis_distance,
                    result.correction_raw_norm, result.correction_applied_norm,
                    result.time_diff_s, output_delta.x(), output_delta.y(), output_delta.z(),
                    stage ? stage : "state");

  if (vio_manager)
  {
    std::vector<std::string> lines;
    std::ostringstream oss;
    oss << "[ GNSS ] action=" << result.action
        << " seq=" << result.seq
        << " state_updated=" << static_cast<int>(result.state_updated)
        << " device_state=" << result.device_state
        << " convergence=" << result.convergence_state
        << " residual=" << result.residual_norm
        << " mahalanobis=" << result.mahalanobis_distance
        << " raw=" << result.correction_raw_norm
        << " applied=" << result.correction_applied_norm
        << " time_diff=" << result.time_diff_s
        << " external_pause_map_update_frames=" << external_update_pause_map_frames_
        << " after " << (stage ? stage : "state") << " update.";
    lines.push_back(oss.str());
    vio_manager->appendTimingLogLines(lines);
  }
}

void LIVMapper::handleUwbRelocalizationConfirmed(UwbUpdateResult &result, const char *stage)
{
  const V3D pos_before = _state.pos_end;
  _state.pos_end.x() = result.filtered_uwb_position.x();
  _state.pos_end.y() = result.filtered_uwb_position.y();
  result.xy_correction_applied = std::hypot(_state.pos_end.x() - pos_before.x(),
                                            _state.pos_end.y() - pos_before.y());
  result.correction_norm = (_state.pos_end - pos_before).norm();
  result.state_updated = true;
  result.request_pause_map_insert = true;

  _state.cov(3, 3) = std::max(_state.cov(3, 3), 1.0);
  _state.cov(4, 4) = std::max(_state.cov(4, 4), 1.0);
  _state.cov(2, 2) = std::max(_state.cov(2, 2), 0.25);
  result.covariance_inflated = true;

  snapStateForDeterminism(_state);
  if (voxelmap_manager)
  {
    voxelmap_manager->state_ = _state;
    voxelmap_manager->clearLocalMap();
    result.local_map_reset = true;
  }
  voxel_map.clear();
  lidar_map_inited = false;
  _pv_list.clear();

  if (vio_manager)
  {
    vio_manager->updateFrameState(_state);
    vio_manager->clearVisualMap();
    result.visual_cache_reset = true;
  }
  if (visual_sub_map) visual_sub_map->clear();

  external_update_pause_map_frames_ = std::max(external_update_pause_map_frames_,
                                               std::max(1, external_update_pause_map_frames_after_correction_));

  if (imu_prop_enable)
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  ROS_WARN("[UWB] relocalization_confirmed after %s: pos_before=[%.3f %.3f %.3f] pos_after=[%.3f %.3f %.3f] local_map_reset=%d visual_cache_reset=%d covariance_inflated=%d external_pause_map_update_frames=%d",
           stage ? stage : "state",
           pos_before.x(), pos_before.y(), pos_before.z(),
           _state.pos_end.x(), _state.pos_end.y(), _state.pos_end.z(),
           static_cast<int>(result.local_map_reset),
           static_cast<int>(result.visual_cache_reset),
           static_cast<int>(result.covariance_inflated),
           external_update_pause_map_frames_);
}

void LIVMapper::advanceUwbOutputCorrection()
{
  if (!uwb_output_correction_en_) return;
  if (!uwb_output_smooth_en_)
  {
    uwb_output_pos_offset_ = uwb_output_target_offset_;
    return;
  }

  const V3D residual = uwb_output_target_offset_ - uwb_output_pos_offset_;
  if (residual.norm() < 1e-6) return;

  V3D step = residual * uwb_output_smooth_alpha_;
  const double step_norm = step.norm();
  if (uwb_output_smooth_max_step_m_ > 0.0 && step_norm > uwb_output_smooth_max_step_m_)
  {
    step *= uwb_output_smooth_max_step_m_ / std::max(step_norm, 1e-9);
  }
  uwb_output_pos_offset_ += step;
}

V3D LIVMapper::outputPosition() const
{
  if (!uwb_output_correction_en_) return _state.pos_end;
  return _state.pos_end + uwb_output_pos_offset_;
}

void LIVMapper::handleVIO() 
{
  const LioUpdateDiagnostics &lio_diagnostics = voxelmap_manager->getLastLioDiagnostics();
  vio_manager->lidar_degenerated = voxelmap_manager->isLidarDegenerated();
  vio_manager->lidar_weak_translation_direction_valid =
      lio_diagnostics.observability.valid &&
      lio_diagnostics.observability.weak_translation_direction_world.allFinite() &&
      lio_diagnostics.observability.weak_translation_direction_world.norm() > 1e-6;
  vio_manager->lidar_weak_translation_direction_world =
      vio_manager->lidar_weak_translation_direction_valid
          ? lio_diagnostics.observability.weak_translation_direction_world.normalized()
          : V3D::Zero();

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  const bool has_fresh_lidar_features =
      pcl_w_wait_pub != nullptr && !pcl_w_wait_pub->empty();
  const bool has_retained_visual_support =
      lidar_map_inited && (!_pv_list.empty() || !vio_manager->feat_map.empty());
  if (!has_fresh_lidar_features && !has_retained_visual_support)
  {
    if (visual_tracking_only_dry_run_en_ && vio_manager != nullptr)
    {
      // ponytail: an empty pg/plane map is enough because this diagnostic path
      // deliberately tracks only the already-built visual map.
      vector<pointWithVar> no_lidar_features;
      const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> no_lidar_plane_map;
      logVisualImageFlow(LidarMeasures.last_lio_update_time,
                         "image_tracking_only_dry_run", "no_lidar_features");
      ++runtime_events_.vio_attempted;
      vio_manager->processFrame(
          LidarMeasures.measures.back().img, no_lidar_features,
          no_lidar_plane_map,
          LidarMeasures.last_lio_update_time - _first_lidar_time, true);
      ++runtime_events_.image_processed;
      if (vio_manager->last_visual_update_accepted)
        ++runtime_events_.vio_accepted;
      else
        ++runtime_events_.vio_rejected;
      return;
    }
    logVisualImageFlow(LidarMeasures.last_lio_update_time,
                       "image_processing_rejected", "no_lidar_features");
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  std::cout << "[ VIO ] Raw feature num: "
            << (has_fresh_lidar_features ? pcl_w_wait_pub->points.size() : 0)
            << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  const bool use_visual_frame = shouldSelectVisualFrame();
  if (!use_visual_frame)
  {
    logVisualImageFlow(LidarMeasures.last_lio_update_time,
                       "image_processing_rejected", "selector_" + last_selector_reason_);
    static int adaptive_skip_count = 0;
    adaptive_skip_count++;
    if (adaptive_skip_count % 10 == 1)
    {
      std::cout << "[ VIO ] Skip by selector: reason=" << last_selector_reason_
                << ", count=" << adaptive_skip_count
                << ", delta(t/r)=" << last_selector_trans_delta_ << "m/" << last_selector_rot_delta_deg_ << "deg"
                << ", thresh(t/r)=" << last_selector_trans_thresh_ << "m/" << last_selector_rot_thresh_deg_ << "deg"
                << ", ratio=" << last_selector_constraint_ratio_
                << ", skip=" << skipped_visual_frames_ << "/" << keyframe_max_skip_frames_
                << std::endl;
    }

    if (imu_prop_enable)
    {
      ekf_finish_once = true;
      latest_ekf_state = _state;
      latest_ekf_time = LidarMeasures.last_lio_update_time;
      state_update_flg = true;
    }

    if (vio_manager)
    {
      vio_manager->last_visual_guard_time = LidarMeasures.last_lio_update_time - _first_lidar_time;
      vio_manager->last_visual_guard_pos = _state.pos_end;
      vio_manager->has_last_visual_guard_pos = true;
      vio_manager->last_visual_measurement_dof = 0;
      vio_manager->last_visual_total_nis = std::numeric_limits<double>::quiet_NaN();
      vio_manager->last_visual_normalized_nis = std::numeric_limits<double>::quiet_NaN();
      vio_manager->logVisualDelta(
          LidarMeasures.last_lio_update_time - _first_lidar_time, 0,
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          "selector_" + last_selector_reason_, _state, _state,
          std::numeric_limits<double>::quiet_NaN(), false);
    }

    publishRawBackendOdometry();
    applyUwbUpdate("VIO-skip");
    applyGnssUpdate("VIO-skip");
    advanceUwbOutputCorrection();

    publish_frame_world(pubLaserCloudFullRes, pubLaserCloudMap, vio_manager);

    euler_cur = RotMtoEuler(_state.rot_end);
    const V3D out_pos = outputPosition();
    fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
             << out_pos.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
             << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;

    if (vio_manager)
    {
      auto formatDouble6 = [](double value)
      {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << value;
        return oss.str();
      };

      auto makeTableRow = [](const std::string &left, const std::string &right)
      {
        std::ostringstream oss;
        oss << "| " << std::left << std::setw(29) << left
            << " | " << std::left << std::setw(27) << right << " |";
        return oss.str();
      };

      std::vector<std::string> lines;
      std::ostringstream oss;
      oss << "[ VIO ] Skip by selector: reason=" << last_selector_reason_
          << ", count=" << adaptive_skip_count
          << ", delta(t/r)=" << formatDouble6(last_selector_trans_delta_) << "m/" << formatDouble6(last_selector_rot_delta_deg_) << "deg"
          << ", thresh(t/r)=" << formatDouble6(last_selector_trans_thresh_) << "m/" << formatDouble6(last_selector_rot_thresh_deg_) << "deg"
          << ", ratio=" << formatDouble6(last_selector_constraint_ratio_)
          << ", skip=" << skipped_visual_frames_ << "/" << keyframe_max_skip_frames_;
      lines.push_back(oss.str());
      lines.push_back("+-------------------------------------------------------------+");
      lines.push_back(makeTableRow("Raw Feature Num", std::to_string(pcl_w_wait_pub->points.size())));
      lines.push_back(makeTableRow("Selector Reason", last_selector_reason_));
      lines.push_back(makeTableRow("Skip Count", std::to_string(adaptive_skip_count)));
      lines.push_back(makeTableRow("Skip Limit", std::to_string(keyframe_max_skip_frames_)));
      lines.push_back(makeTableRow("Delta t/r", formatDouble6(last_selector_trans_delta_) + "m/" + formatDouble6(last_selector_rot_delta_deg_) + "deg"));
      lines.push_back(makeTableRow("Thresh t/r", formatDouble6(last_selector_trans_thresh_) + "m/" + formatDouble6(last_selector_rot_thresh_deg_) + "deg"));
      lines.push_back(makeTableRow("Constraint Ratio", formatDouble6(last_selector_constraint_ratio_)));
      lines.push_back("+-------------------------------------------------------------+");
      vio_manager->appendTimingLogLines(lines);
    }
    return;
  }

  logVisualImageFlow(LidarMeasures.last_lio_update_time,
                     "image_processed", last_selector_reason_);
  ++runtime_events_.vio_attempted;
  // A LiDAR scan contributes new visual-map candidates once. Images later in
  // the same scan still run the full frontend against the retained maps, but
  // must not reinsert the previous scan's points.
  vector<pointWithVar> no_new_lidar_features;
  vector<pointWithVar> &visual_lidar_features =
      has_fresh_lidar_features ? _pv_list : no_new_lidar_features;
  vio_manager->processFrame(LidarMeasures.measures.back().img,
                            visual_lidar_features,
                            voxelmap_manager->voxel_map_,
                            LidarMeasures.last_lio_update_time - _first_lidar_time,
                            false);
  ++runtime_events_.image_processed;
  if (vio_manager->last_visual_update_accepted)
    ++runtime_events_.vio_accepted;
  else
    ++runtime_events_.vio_rejected;
  snapStateForDeterminism(_state);
  vio_manager->updateFrameState(_state);
  if (!visual_shadow_no_commit_en_)
    updateVisualObservationHints();
  publishRawBackendOdometry();
  applyUwbUpdate("VIO");
  applyGnssUpdate("VIO");
  advanceUwbOutputCorrection();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, pubLaserCloudMap, vio_manager);
  if (!suppress_image_pub_)
  {
    publish_img_counter_++;
    if (publish_img_counter_ % std::max(1, publish_img_stride_) == 0)
    {
      publish_img_rgb(pubImage, vio_manager);
    }
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  const V3D out_pos = outputPosition();
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << out_pos.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;

}

bool LIVMapper::shouldSelectVisualFrame()
{
  last_selector_constraint_ratio_ = 0.0;
  last_selector_trans_thresh_ = 0.0;
  last_selector_rot_thresh_deg_ = 0.0;
  last_selector_trans_delta_ = 0.0;
  last_selector_rot_delta_deg_ = 0.0;
  last_selector_reach_pose_keyframe_ = false;
  last_selector_reach_skip_limit_ = false;

  if (!adaptive_visual_selector_en)
  {
    last_selector_reason_ = "adaptive_off";
    return true;
  }
  if (!img_en)
  {
    last_selector_reason_ = "img_disabled";
    return false;
  }

  if (voxelmap_manager->isLidarDegenerated())
  {
    last_selector_reason_ = "lidar_degenerated_force";
    skipped_visual_frames_ = 0;
    has_last_visual_keyframe_state_ = true;
    last_visual_keyframe_state_ = _state;
    return true;
  }

  if (!has_last_visual_keyframe_state_)
  {
    last_selector_reason_ = "first_visual_keyframe";
    has_last_visual_keyframe_state_ = true;
    last_visual_keyframe_state_ = _state;
    skipped_visual_frames_ = 0;
    return true;
  }

  const double ratio_full = std::max(1e-6, keyframe_constraint_ratio_full_);
  const double constraint_ratio = std::max(0.0, std::min(1.0, voxelmap_manager->getLidarConstraintRatio() / ratio_full));
  const double trans_thresh = keyframe_trans_thresh_min_ +
                              constraint_ratio * (keyframe_trans_thresh_max_ - keyframe_trans_thresh_min_);
  const double rot_thresh_deg = keyframe_rot_thresh_min_deg_ +
                                constraint_ratio * (keyframe_rot_thresh_max_deg_ - keyframe_rot_thresh_min_deg_);

  const double trans_delta = (_state.pos_end - last_visual_keyframe_state_.pos_end).norm();
  Eigen::Matrix3d dR = last_visual_keyframe_state_.rot_end.transpose() * _state.rot_end;
  const double rot_delta_deg = Eigen::AngleAxisd(dR).angle() * 57.29577951308232;

  const bool reach_pose_keyframe = (trans_delta >= trans_thresh) || (rot_delta_deg >= rot_thresh_deg);
  const bool reach_skip_limit = skipped_visual_frames_ >= keyframe_max_skip_frames_;

  last_selector_constraint_ratio_ = constraint_ratio;
  last_selector_trans_thresh_ = trans_thresh;
  last_selector_rot_thresh_deg_ = rot_thresh_deg;
  last_selector_trans_delta_ = trans_delta;
  last_selector_rot_delta_deg_ = rot_delta_deg;
  last_selector_reach_pose_keyframe_ = reach_pose_keyframe;
  last_selector_reach_skip_limit_ = reach_skip_limit;

  if (reach_pose_keyframe || reach_skip_limit)
  {
    last_selector_reason_ = reach_pose_keyframe ? "pose_keyframe" : "max_skip";
    last_visual_keyframe_state_ = _state;
    skipped_visual_frames_ = 0;
    return true;
  }

  last_selector_reason_ = "below_threshold";
  skipped_visual_frames_++;
  return false;
}

void LIVMapper::updateVisualObservationHints()
{
  std::vector<VOXEL_LOCATION> observed_voxels;
  observed_voxels.reserve(vio_manager->feat_map.size());
  for (const auto &kv : vio_manager->feat_map)
  {
    if (kv.second != nullptr && !kv.second->voxel_points.empty())
    {
      observed_voxels.push_back(kv.first);
    }
  }
  if (deterministic_visual_observed_voxel_sort_en_)
  {
    std::sort(observed_voxels.begin(), observed_voxels.end(),
              [](const VOXEL_LOCATION &a, const VOXEL_LOCATION &b) {
                if (a.x != b.x) return a.x < b.x;
                if (a.y != b.y) return a.y < b.y;
                return a.z < b.z;
              });
  }
  voxelmap_manager->setVisualObservedVoxels(observed_voxels);
}

void LIVMapper::handleLIO() 
{
  bool livo_map_transaction_recorded = false;
  const auto note_livo_map_transaction = [&]() {
    if (livo_map_transaction_recorded || slam_mode_ != LIVO ||
        !livo_scan_lifecycle_.active() ||
        !livo_scan_lifecycle_.snapshot().measurement_required)
      return;
    livo_scan_lifecycle_.noteMapInsertion();
    livo_map_transaction_recorded = true;
  };
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (!feats_undistort || feats_undistort->empty())
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  VoxelSafetyContext voxel_context;
  voxel_context.coordinate_frame = "lidar/body (undistorted current frame)";
  voxel_context.enforce_max_range = true;
  voxel_context.max_range_m = voxel_lidar_max_range_m_;
  if (!safeVoxelFilter<PointType>(feats_undistort, feats_down_body,
                                  Eigen::Vector3f::Constant(static_cast<float>(filter_size_surf_min)),
                                  "VOXEL_FRAME", voxel_context))
  {
    ROS_ERROR_THROTTLE(5.0, "[VOXEL_REJECT] tag=VOXEL_FRAME action=skip_lio_frame");
    return;
  }
  if (deterministic_lio_feature_sort_en_)
  {
    std::sort(feats_down_body->points.begin(), feats_down_body->points.end(),
              [](const PointType &a, const PointType &b) {
                if (a.x != b.x) return a.x < b.x;
                if (a.y != b.y) return a.y < b.y;
                return a.z < b.z;
              });
  }
  if (voxelmap_manager->config_setting_.p4_frontend_diagnostics_enable)
  {
    voxelmap_manager->p4_fixed_velocity_down_body_->clear();
    if (voxelmap_manager->p4_fixed_velocity_cloud_ &&
        !voxelmap_manager->p4_fixed_velocity_cloud_->empty())
    {
      const bool p4_filtered = safeVoxelFilter<PointType>(
          voxelmap_manager->p4_fixed_velocity_cloud_,
          voxelmap_manager->p4_fixed_velocity_down_body_,
          Eigen::Vector3f::Constant(
              static_cast<float>(filter_size_surf_min)),
          "P4_FIXED_VELOCITY_DESKEW", voxel_context);
      if (!p4_filtered)
        voxelmap_manager->p4_fixed_velocity_down_body_->clear();
      else if (deterministic_lio_feature_sort_en_)
        std::sort(
            voxelmap_manager->p4_fixed_velocity_down_body_->points.begin(),
            voxelmap_manager->p4_fixed_velocity_down_body_->points.end(),
            [](const PointType &a, const PointType &b) {
              if (a.x != b.x) return a.x < b.x;
              if (a.y != b.y) return a.y < b.y;
              return a.z < b.z;
            });
    }
  }
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap(LidarMeasures.last_lio_update_time);
    note_livo_map_transaction();
  }

  double t1 = omp_get_wtime();

  std::ostringstream lio_iteration_log;
  const bool record_lio_iterations = save_log_en && vio_manager;
  ++runtime_events_.lio_attempted;
  voxelmap_manager->StateEstimation(
      state_propagat, LidarMeasures.last_lio_update_time,
      record_lio_iterations ? &lio_iteration_log : nullptr);
  voxelmap_manager->runP5SeedBasinShadow(
      state_propagat, LidarMeasures.last_lio_update_time,
      LidarMeasures.last_lio_update_time - _first_lidar_time);
  if (voxelmap_manager->getLastLioDiagnostics().commit)
    ++runtime_events_.lio_committed;
  else
    ++runtime_events_.lio_rejected;
  _state = voxelmap_manager->state_;
  publishFullStateShadowGeometry();
  if (save_log_en && vio_manager)
  {
    // Observe the unmodified LIO transaction, before external corrections.
    std::ostringstream health;
    health << std::setprecision(17) << "[LIO_STATE_HEALTH] timestamp="
           << LidarMeasures.last_lio_update_time;
    for (const auto &entry : {std::make_pair("predicted", &state_propagat),
                              std::make_pair("updated", &_state)})
    {
      const StatesGroup &value = *entry.second;
      double minimum_eigenvalue = std::numeric_limits<double>::quiet_NaN();
      if (value.cov.allFinite())
      {
        Eigen::SelfAdjointEigenSolver<MD(DIM_STATE, DIM_STATE)> spectrum(
            0.5 * (value.cov + value.cov.transpose()).eval(), Eigen::EigenvaluesOnly);
        if (spectrum.info() == Eigen::Success)
          minimum_eigenvalue = spectrum.eigenvalues().minCoeff();
      }
      health << " " << entry.first << "_bias_a=" << value.bias_a.transpose()
             << " " << entry.first << "_bias_g=" << value.bias_g.transpose()
             << " " << entry.first << "_gravity=" << value.gravity.transpose()
             << " " << entry.first << "_cov_min_eigenvalue=" << minimum_eigenvalue
             << " " << entry.first << "_cov_asymmetry="
             << (value.cov - value.cov.transpose()).cwiseAbs().maxCoeff();
    }
    vio_manager->appendTimingLogLines({lio_iteration_log.str(), health.str()});
  }
  _pv_list = voxelmap_manager->pv_list_;
  snapStateForDeterminism(_state);
  voxelmap_manager->state_ = _state;
  publishRawBackendOdometry();
  applyUwbUpdate("LIO");
  applyGnssUpdate("LIO");
  advanceUwbOutputCorrection();

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(save_path + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      evoFile.open(save_path + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed << std::setprecision(9);
    const V3D out_pos = outputPosition();
    evoFile << LidarMeasures.last_lio_update_time << " " << out_pos[0] << " " << out_pos[1] << " " << out_pos[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  const int map_update_stride = std::max(1, lio_map_update_stride_);
  lio_map_update_counter_++;
  const bool do_map_update = (map_update_stride <= 1) || ((lio_map_update_counter_ % map_update_stride) == 0);
  const LioUpdateDiagnostics &lio_diagnostics = voxelmap_manager->getLastLioDiagnostics();
  const VoxelMapConfig &lio_config = voxelmap_manager->config_setting_;
  const bool map_guard_enabled = lio_config.map_guard_mode != "off";
  const bool map_guard_enforce = lio_config.map_guard_mode == "enforce";
  const bool direction_map_guard_request = map_guard_enabled &&
      lio_config.map_guard_freeze_on_direction_reject &&
      lio_diagnostics.direction_conflict;
  const bool degeneracy_map_guard_request = map_guard_enabled &&
      lio_config.map_guard_freeze_on_degeneracy &&
      lio_diagnostics.is_severely_degenerate;
  const bool map_guard_request = direction_map_guard_request || degeneracy_map_guard_request;
  std::string map_guard_reason = "none";

  if (!map_guard_enforce)
  {
    lio_map_guard_active_ = false;
    lio_map_guard_hard_limit_latched_ = false;
    lio_map_guard_recovery_frames_ = 0;
    lio_map_guard_freeze_frames_ = 0;
    if (lio_config.map_guard_mode == "diagnostic" && map_guard_request)
      ROS_INFO_THROTTLE(1.0, "[LIO_MAP_GUARD] diagnostic freeze request observed; insertion unchanged");
  }
  else if (lio_map_guard_hard_limit_latched_)
  {
    if (!map_guard_request)
    {
      lio_map_guard_hard_limit_latched_ = false;
      ROS_INFO("[LIO_MAP_GUARD] hard-limit latch cleared after guard request ended");
    }
    map_guard_reason = "hard_limit_degraded_mapping";
  }
  else if (map_guard_request)
  {
    lio_map_guard_recovery_frames_ = 0;
    if (!lio_map_guard_active_)
    {
      lio_map_guard_freeze_frames_ = 0;
      lio_map_guard_active_ = true;
      ROS_WARN("[LIO_MAP_GUARD] map insertion freeze started");
    }
    map_guard_reason = direction_map_guard_request ? "direction_guard" : "lidar_degenerate";
  }
  else if (lio_map_guard_active_)
  {
    ++lio_map_guard_recovery_frames_;
    map_guard_reason = "recovery_hysteresis";
    if (lio_map_guard_recovery_frames_ >= lio_config.map_guard_recovery_consecutive_frames)
    {
      lio_map_guard_active_ = false;
      lio_map_guard_recovery_frames_ = 0;
      lio_map_guard_freeze_frames_ = 0;
      map_guard_reason = "recovered";
      ROS_INFO("[LIO_MAP_GUARD] map insertion freeze cleared after %d stable frames",
               lio_config.map_guard_recovery_consecutive_frames);
    }
  }

  const bool external_map_guard = external_update_pause_map_frames_ > 0;
  const bool p4_map_mutation_allowed = voxelmap_manager->p4MapMutationAllowed(
      LidarMeasures.last_lio_update_time);
  bool lio_map_guard_enforced = false;
  if (do_map_update && map_guard_enforce && lio_map_guard_active_)
  {
    if (lio_map_guard_freeze_frames_ >= lio_config.map_guard_maximum_freeze_frames)
    {
      lio_map_guard_active_ = false;
      lio_map_guard_hard_limit_latched_ = true;
      lio_map_guard_recovery_frames_ = 0;
      map_guard_reason = "hard_limit_degraded_mapping";
      ROS_WARN("[LIO_MAP_GUARD] hard maximum_freeze_frames=%d reached; insertion resumed and re-freeze latched until the request clears",
               lio_config.map_guard_maximum_freeze_frames);
    }
    else
    {
      lio_map_guard_enforced = true;
      ++lio_map_guard_freeze_frames_;
    }
  }
  const bool lio_measurement_rejected = !lio_diagnostics.commit;
  const bool map_guarded = external_map_guard || lio_map_guard_enforced ||
                           !p4_map_mutation_allowed;
  const bool insert_lio_map = fast_livo::shouldInsertLioMap(
      lio_diagnostics.commit, do_map_update, map_guarded);
  const bool skip_map_insert = do_map_update && !insert_lio_map;
  std::string map_insert_skip_reason = "none";
  if (lio_measurement_rejected)
    map_insert_skip_reason = "lio_" + lio_diagnostics.convergence_status;
  else if (!do_map_update)
    map_insert_skip_reason = "map_update_stride";
  else if (external_map_guard && lio_map_guard_enforced)
    map_insert_skip_reason = "external_update+" + map_guard_reason;
  else if (external_map_guard)
    map_insert_skip_reason = "external_update";
  else if (lio_map_guard_enforced)
    map_insert_skip_reason = map_guard_reason;
  else if (!p4_map_mutation_allowed)
    map_insert_skip_reason = "p4_frozen_map_counterfactual";
  const int pause_map_update_frames_before = external_update_pause_map_frames_;
  double t4 = t3;

  if (skip_map_insert)
  {
    if (external_map_guard)
      external_update_pause_map_frames_ = std::max(0, external_update_pause_map_frames_ - 1);
    ROS_INFO_THROTTLE(1.0,
                      "[LIO_MAP_GUARD] skip_map_insert=1 reason=%s external_pause=%d->%d freeze_frames=%d recovery_frames=%d",
                      map_insert_skip_reason.c_str(), pause_map_update_frames_before,
                      external_update_pause_map_frames_, lio_map_guard_freeze_frames_,
                      lio_map_guard_recovery_frames_);
  }
  else if (insert_lio_map)
  {
    PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
    transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
    for (size_t i = 0; i < world_lidar->points.size(); i++)
    {
      voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
      M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
      M3D var = voxelmap_manager->body_cov_list_[i];
      var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
            (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
      voxelmap_manager->pv_list_[i].var = var;
      if (lio_config.p4_frontend_diagnostics_enable ||
          lio_config.p4b_snapshot_enable)
      {
        voxelmap_manager->pv_list_[i].source_frame_id =
            voxelmap_manager->current_frame_id_;
        voxelmap_manager->pv_list_[i].source_point_index =
            static_cast<int>(i);
        voxelmap_manager->pv_list_[i].source_timestamp_s =
            LidarMeasures.last_lio_update_time;
        voxelmap_manager->pv_list_[i].source_origin_w = _state.pos_end;
      }
    }
    voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
    note_livo_map_transaction();
    voxelmap_manager->markLastLioMapInserted(true);
    if (print_console_timing_en_ && (frame_num % std::max(1, print_console_timing_stride_) == 0))
    {
      std::cout << "[ LIO ] Update Voxel Map" << std::endl;
    }
    _pv_list = voxelmap_manager->pv_list_;

    t4 = omp_get_wtime();

  }

  // Sliding/cropping is memory management, not map insertion. It must continue
  // even while an external or experimental guard pauses new point insertion.
  if (do_map_update && p4_map_mutation_allowed &&
      voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }

  voxelmap_manager->p4RecordMapDecision(
      LidarMeasures.last_lio_update_time, insert_lio_map,
      insert_lio_map ? "inserted" : map_insert_skip_reason,
      state_propagat.pos_end, _state.pos_end);

  logLioDegeneracy(skip_map_insert, map_insert_skip_reason,
                   map_guard_request, lio_map_guard_enforced);
  logLioTransaction();
  logLioMotionConsistency();
  logRuntimeEventCounts(false);
  logLioDirectionalShadow();

  if (p4b_fork_harness_)
  {
    p4b_fork_harness_->observeProductionResult(
        _state, *voxelmap_manager, *feats_undistort,
        LidarMeasures.last_lio_update_time,
        runtime_events_.vio_accepted);
    const double relative_time_s =
        LidarMeasures.last_lio_update_time - _first_lidar_time;
    if (p4b_fork_harness_->shouldCapture(relative_time_s))
    {
      std::string error;
      if (!p4b_fork_harness_->beginCapture(error))
      {
        ROS_ERROR("[P4B] capture pause failed: %s", error.c_str());
        return;
      }
      fast_livo::p4b::P4ForkSnapshot snapshot;
      snapshot.boundary_timestamp_s = LidarMeasures.last_lio_update_time;
      snapshot.state = _state;
      snapshot.propagated_state = state_propagat;
      snapshot.imu = p_imu->captureSnapshot();
      snapshot.map = voxelmap_manager->captureSnapshot();
      snapshot.lifecycle.lidar_map_initialized = lidar_map_inited;
      snapshot.lifecycle.gravity_alignment_finished = gravity_align_finished;
      snapshot.lifecycle.lidar_map_update_counter = lio_map_update_counter_;
      snapshot.lifecycle.lidar_map_guard_active = lio_map_guard_active_;
      snapshot.lifecycle.lidar_map_guard_hard_limit_latched =
          lio_map_guard_hard_limit_latched_;
      snapshot.lifecycle.lidar_map_guard_recovery_frames =
          lio_map_guard_recovery_frames_;
      snapshot.lifecycle.lidar_map_guard_freeze_frames =
          lio_map_guard_freeze_frames_;
      snapshot.lifecycle.external_update_pause_map_frames =
          external_update_pause_map_frames_;
      snapshot.lifecycle.lidar_frame_begin_time =
          LidarMeasures.lidar_frame_beg_time;
      snapshot.lifecycle.lidar_frame_end_time =
          LidarMeasures.lidar_frame_end_time;
      snapshot.lifecycle.last_lidar_update_time =
          LidarMeasures.last_lio_update_time;
      snapshot.lifecycle.lidar_scan_index =
          LidarMeasures.lidar_scan_index_now;
      if (!p4b_fork_harness_->capture(
              snapshot, voxelmap_manager->config_setting_,
              runtime_events_.vio_accepted, error))
        ROS_ERROR("[P4B] snapshot capture failed: %s", error.c_str());
      else
        ROS_WARN("[P4B] snapshot captured at relative_time=%.6f frame_id=%d",
                 relative_time_s, voxelmap_manager->current_frame_id_);
    }
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  if (!img_en) publish_frame_world(pubLaserCloudFullRes, pubLaserCloudMap, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  double t5 = omp_get_wtime();

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  const bool print_console_timing = print_console_timing_en_ &&
                                    ((frame_num % std::max(1, print_console_timing_stride_)) == 0);
  if (print_console_timing)
  {
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
    printf("\033[1;36m| %-29s | %-27d |\033[0m\n", "skip_map_insert", static_cast<int>(skip_map_insert));
    printf("\033[1;36m| %-29s | %-27d |\033[0m\n", "pause_map_update_frames", external_update_pause_map_frames_);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "postProcess+Publish", t5 - t4);
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t5 - t0);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  }

  if (vio_manager)
  {
    auto formatDouble6 = [](double value)
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(6) << value;
      return oss.str();
    };

    auto makeTableRow = [](const std::string &left, const std::string &right)
    {
      std::ostringstream oss;
      oss << "| " << std::left << std::setw(29) << left
          << " | " << std::left << std::setw(27) << right << " |";
      return oss.str();
    };

    const double lio_total_time = t5 - t0;
    std::vector<std::string> lines;
    lines.push_back("+-------------------------------------------------------------+");
    lines.push_back("|                         LIO Mapping Time                    |");
    lines.push_back("+-------------------------------------------------------------+");
    lines.push_back(makeTableRow("Algorithm Stage", "Time (secs)"));
    lines.push_back("+-------------------------------------------------------------+");
    lines.push_back(makeTableRow("DownSample", formatDouble6(t_down - t0)));
    lines.push_back(makeTableRow("ICP", formatDouble6(t2 - t1)));
    lines.push_back(makeTableRow("updateVoxelMap", formatDouble6(t4 - t3)));
    lines.push_back(makeTableRow("skip_map_insert", std::to_string(static_cast<int>(skip_map_insert))));
    lines.push_back(makeTableRow("pause_map_update_frames", std::to_string(external_update_pause_map_frames_)));
    lines.push_back(makeTableRow("postProcess+Publish", formatDouble6(t5 - t4)));
    lines.push_back("+-------------------------------------------------------------+");
    lines.push_back(makeTableRow("Current Total Time", formatDouble6(lio_total_time)));
    lines.push_back(makeTableRow("Average Total Time", formatDouble6(aver_time_consu)));
    lines.push_back(makeTableRow("Budget (s)", formatDouble6(frame_time_budget_s_)));
    lines.push_back("+-------------------------------------------------------------+");
    vio_manager->appendTimingLogLines(lines);
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  const V3D out_pos = outputPosition();
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << out_pos.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    //std::string raw_points_dir = std::string(ROOT_DIR) + "Log/PCD/all_raw_points.pcd";
    //std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/PCD/all_downsampled_points.pcd";
    string all_points_dir(save_path + string("map_dense.pcd"));
    string downsampled_points_dir(save_path + string("map.pcd"));
    string downsampled_points_dir2(save_path + string("pose.pcd"));
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      VoxelSafetyContext voxel_context;
      voxel_context.coordinate_frame = "world/global saved map";
      if (!safeVoxelFilter<pcl::PointXYZRGB>(pcl_wait_save, downsampled_cloud,
                                             Eigen::Vector3f::Constant(static_cast<float>(filter_size_pcd)),
                                             "VOXEL_SAVE", voxel_context))
      {
        ROS_ERROR("[VOXEL_REJECT] tag=VOXEL_SAVE action=abort_map_save");
        return;
      }
 
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the raw point cloud data
      pcd_writer.writeBinary(downsampled_points_dir2, *downsampled_cloud); 
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);

      std::cout << GREEN << "All point cloud data saved to: " << all_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "All point cloud data saved to: " << all_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::print_landmarks()
{
  if (vio_manager->board_world_flag_.empty())
  {
    std::cout << YELLOW << "[Aruco] No board entries to print." << RESET << std::endl;
    return;
  }

  std::cout << YELLOW << "[Aruco] Final board first-observation positions:" << RESET << std::endl;

  for (const auto& item : vio_manager->board_world_flag_)
  {
    const int id = item.first;
    const bool initialized = item.second;

    auto pos_it = vio_manager->board_world_positions_.find(id);
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    if (pos_it != vio_manager->board_world_positions_.end())
    {
      position = pos_it->second;
    }

    if (initialized)
    {
      std::cout << YELLOW << "  [INIT] Board " << id
                << " -> (" << position.x() << ", "
                << position.y() << ", " << position.z() << ")"
                << RESET << std::endl;
    }
    else
    {
      std::cout << YELLOW << "  [UNINIT] Board " << id
                << " -> not observed yet"
                << RESET << std::endl;
    }
  }
}

void LIVMapper::run() 
{
  auto formatDouble6 = [](double value)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
  };

  auto makeTableRow = [](const std::string &left, const std::string &right)
  {
    std::ostringstream oss;
    oss << "| " << std::left << std::setw(29) << left
        << " | " << std::left << std::setw(27) << right << " |";
    return oss.str();
  };

  ros::Rate rate(5000);
  int i = 0;
  double sum = 0, t = 0;
  while (ros::ok()) 
  {
    const double t1 = omp_get_wtime();
    ros::spinOnce();
    const double t_spin_end = omp_get_wtime();
    logRuntimeMemory();
    if (!startupWarmupReady())
    {
      rate.sleep();
      continue;
    }
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    const double t_sync_end = omp_get_wtime();
    handleFirstFrame();

    if (p4b_fork_harness_ && p4b_fork_harness_->captured() &&
        !p4b_fork_harness_->finished() &&
        LidarMeasures.lio_vio_flg != VIO)
    {
      std::string error;
      if (!p4b_fork_harness_->beginForkFrame(error))
      {
        ROS_ERROR("[P4B] fork-frame input pause failed: %s", error.c_str());
        return;
      }
      const double packet_time = !LidarMeasures.measures.empty()
          ? LidarMeasures.measures.back().lio_time
          : LidarMeasures.last_lio_update_time;
      p4b_fork_harness_->processImmutableLioPacket(
          LidarMeasures, packet_time - _first_lidar_time);
    }

    processImu();
    const double t_imu_end = omp_get_wtime();

    // if (!p_imu->imu_time_init) continue;

    const EKF_STATE frame_mode = LidarMeasures.lio_vio_flg;
    stateEstimationAndMapping();
    if (p4b_fork_harness_) p4b_fork_harness_->endForkFrame();
    updateMappingReady();

    const double t2 = omp_get_wtime();
    const double frame_time = t2 - t1;
    updateRuntimeGuard(frame_time);

    if (vio_manager)
    {
      std::vector<std::string> lines;
      const double t_spin = t_spin_end - t1;
      const double t_sync = t_sync_end - t_spin_end;
      const double t_imu = t_imu_end - t_sync_end;
      const double t_mapping = t2 - t_imu_end;
      const double t_other = std::max(0.0, frame_time - (t_spin + t_sync + t_imu + t_mapping));

      std::string mode_str = "WAIT";
      if (frame_mode == VIO) mode_str = "VIO";
      else if (frame_mode == LIO) mode_str = "LIO";
      else if (frame_mode == LO) mode_str = "LO";

      std::ostringstream line;
      line << "[ Frame ] idx=" << (i + 1)
           << ", mode=" << mode_str
           << ", current=" << formatDouble6(frame_time)
           << " s, budget=" << formatDouble6(frame_time_budget_s_);
      lines.push_back(line.str());

      std::ostringstream line_cost;
      line_cost << "[ Frame Cost ] spin=" << formatDouble6(t_spin)
                << ", sync=" << formatDouble6(t_sync)
                << ", imu=" << formatDouble6(t_imu)
                << ", mapping=" << formatDouble6(t_mapping)
                << ", other=" << formatDouble6(t_other);
      lines.push_back(line_cost.str());

      vio_manager->appendTimingLogLines(lines);
    }

    i ++;
    t += frame_time;
    if (i % 2 == 0)
    {
      sum += t;
      const bool print_console_timing = print_console_timing_en_ &&
                                        (((i / 2) % std::max(1, print_console_timing_stride_)) == 0);
      if (print_console_timing)
      {
        printf("\033[1;45m+-------------------------------------------------------------+\033[0m\n");
        printf("\033[1;95m| %-29s | %-27d |\033[0m\n", "Frame number", i/2);
        printf("\033[1;95m| %-29s | %-27f |\033[0m\n", "Current frame time", t);
        printf("\033[1;95m| %-29s | %-27f |\033[0m\n", "Average frame time", sum / (i/2));
        printf("\033[1;45m+-------------------------------------------------------------+\033[0m\n");
      }

      if (vio_manager)
      {
        std::vector<std::string> lines;
        lines.push_back("+-------------------------------------------------------------+");
        lines.push_back("|                         Frame Time                          |");
        lines.push_back("+-------------------------------------------------------------+");
        lines.push_back(makeTableRow("Frame number", std::to_string(i / 2)));
        lines.push_back(makeTableRow("Current frame time", formatDouble6(t)));
        lines.push_back(makeTableRow("Average frame time", formatDouble6(sum / (i / 2))));
        lines.push_back(makeTableRow("Budget (s)", formatDouble6(frame_time_budget_s_)));
        lines.push_back("+-------------------------------------------------------------+");
        vio_manager->appendTimingLogLines(lines);
      }

      t = 0;
    }
  }
  savePCD();
  if (aruco_landmarks_en) print_landmarks();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    if (deterministic_prop_imu_buffer_sort_en_)
    {
      std::sort(prop_imu_buffer.begin(), prop_imu_buffer.end(),
                [](const sensor_msgs::Imu &a, const sensor_msgs::Imu &b) {
                  return a.header.stamp.toSec() < b.header.stamp.toSec();
                });
    }
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom.publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!lidar_en) return;
  ++runtime_events_.lidar_received;
  mtx_buffer.lock();

  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  while (max_lidar_buffer_size_ > 0 && static_cast<int>(lid_raw_data_buffer.size()) > max_lidar_buffer_size_)
  {
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    ++runtime_events_.buffer_overflow;
  }
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  if (!lidar_en) return;
  ++runtime_events_.lidar_received;
  mtx_buffer.lock();
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = msg->header.stamp.toSec();
  ROS_INFO_THROTTLE(1.0, "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  while (max_lidar_buffer_size_ > 0 && static_cast<int>(lid_raw_data_buffer.size()) > max_lidar_buffer_size_)
  {
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    ++runtime_events_.buffer_overflow;
  }
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  if (!imu_en) return;
  ++runtime_events_.imu_received;

  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
  double timestamp = msg->header.stamp.toSec();

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = ros::Time().fromSec(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    if (!deterministic_imu_accept_out_of_order_en_)
    {
      ROS_ERROR("imu loop back. \n");
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      return;
    }
    ROS_WARN("imu loop back, offset: %lf, inserting anyway\n", last_timestamp_imu - timestamp);
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  if (deterministic_imu_accept_out_of_order_en_)
  {
    if (timestamp > last_timestamp_imu) last_timestamp_imu = timestamp;
  }
  else
  {
    last_timestamp_imu = timestamp;
  }

  imu_buffer.push_back(msg);
  while (max_imu_buffer_size_ > 0 && static_cast<int>(imu_buffer.size()) > max_imu_buffer_size_)
  {
    imu_buffer.pop_front();
    ++runtime_events_.buffer_overflow;
  }
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init)
    {
      prop_imu_buffer.push_back(*msg);
      while (max_prop_imu_buffer_size_ > 0 && static_cast<int>(prop_imu_buffer.size()) > max_prop_imu_buffer_size_)
      {
        prop_imu_buffer.pop_front();
        ++runtime_events_.buffer_overflow;
      }
    }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  if (!img_en) return;
  ++runtime_events_.image_received;
  const double received_time = msg_in->header.stamp.toSec() + img_time_offset;
  logVisualImageFlow(received_time, "image_received", "subscriber_callback");
  sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0)
    {
      logVisualImageFlow(received_time, "image_dropped", "hilti_stride");
      return;
    }
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = msg->header.stamp.toSec() + img_time_offset;
  if (!deterministic_image_buffer_sort_en_)
  {
    if (std::fabs(msg_header_time - last_timestamp_img) < 0.001)
    {
      logVisualImageFlow(msg_header_time, "image_dropped", "duplicate_timestamp");
      return;
    }
    if (msg_header_time < last_timestamp_img)
    {
      logVisualImageFlow(msg_header_time, "image_dropped", "timestamp_loopback");
      ROS_ERROR("image loop back. \n");
      return;
    }
  }
  ROS_INFO_THROTTLE(1.0, "Get image, its header time: %.6f", msg_header_time);

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (deterministic_image_buffer_sort_en_)
  {
    for (const double buffered_time : img_time_buffer)
    {
      if (std::fabs(buffered_time - img_time_correct) < 0.001)
      {
        logVisualImageFlow(img_time_correct, "image_dropped", "duplicate_buffered_timestamp");
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
      }
    }
  }
  else if (img_time_correct - last_timestamp_img < 0.02)
  {
    logVisualImageFlow(img_time_correct, "image_dropped", "minimum_interval");
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  if (deterministic_image_buffer_sort_en_)
  {
    const auto insert_it = std::lower_bound(img_time_buffer.begin(), img_time_buffer.end(), img_time_correct);
    const auto insert_idx = std::distance(img_time_buffer.begin(), insert_it);
    img_time_buffer.insert(insert_it, img_time_correct);
    img_buffer.insert(img_buffer.begin() + insert_idx, img_cur);
  }
  else
  {
    img_buffer.push_back(img_cur);
    img_time_buffer.push_back(img_time_correct);
  }
  logVisualImageFlow(img_time_correct, "image_buffered", "none");
  while (max_img_buffer_size_ > 0 && static_cast<int>(img_buffer.size()) > max_img_buffer_size_)
  {
    logVisualImageFlow(img_time_buffer.front(), "image_dropped", "buffer_overflow");
    img_buffer.pop_front();
    img_time_buffer.pop_front();
    ++runtime_events_.buffer_overflow;
  }

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  if (deterministic_image_buffer_sort_en_)
  {
    if (img_time_correct > last_timestamp_img) last_timestamp_img = img_time_correct;
  }
  else
  {
    last_timestamp_img = img_time_correct;
  }
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (slam_mode_ != LIVO && lid_raw_data_buffer.empty() && lidar_en)
    return false;
  if (slam_mode_ != LIVO && imu_buffer.empty() && imu_en)
    return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    // 确保 imu_buffer 按时间戳有序，消除乱序送达导致的 draining 非确定性
    if (deterministic_imu_buffer_sort_en_)
    {
      std::sort(imu_buffer.begin(), imu_buffer.end(),
                [](const sensor_msgs::Imu::ConstPtr &a, const sensor_msgs::Imu::ConstPtr &b) {
                  return a->header.stamp.toSec() < b->header.stamp.toSec();
                });
    }
    while (!imu_buffer.empty())
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    // 确保 IMU 按时间戳严格有序，消除回调乱序导致的非确定性
    if (deterministic_imu_buffer_sort_en_)
    {
      std::sort(m.imu.begin(), m.imu.end(),
                [](const sensor_msgs::Imu::ConstPtr &a, const sensor_msgs::Imu::ConstPtr &b) {
                  return a->header.stamp.toSec() < b->header.stamp.toSec();
                });
    }

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    constexpr double kTimeEpsilon = 1e-6;

    if (!lidar_pushed)
    {
      if (lid_raw_data_buffer.empty()) return false;
      meas.lidar = lid_raw_data_buffer.front();
      if (!meas.lidar || meas.lidar->points.size() <= 1) return false;
      meas.lidar_frame_beg_time = lid_header_time_buffer.front();
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time +
          meas.lidar->points.back().curvature / 1000.0;
      if (meas.last_lio_update_time < 0.0)
        meas.last_lio_update_time = meas.lidar_frame_beg_time;
      meas.pcl_proc_cur = meas.lidar;
      meas.pcl_proc_next->clear();
      livo_scan_lifecycle_.begin(
          ++next_livo_scan_id_, meas.lidar_frame_beg_time,
          meas.lidar_frame_end_time, meas.lidar->points.size(),
          !p_imu->imu_need_init, meas.last_lio_update_time);
      p_imu->beginLidarScan(meas.lidar_frame_beg_time, _state);
      lidar_pushed = true;
    }

    // Images older than the already committed state cannot be propagated.
    // Keep an equal-time image: the scan-end LIO is deliberately dispatched
    // first and that image is then a valid zero-dt VIO event.
    mtx_buffer.lock();
    while (!img_time_buffer.empty() &&
           img_time_buffer.front() + exposure_time_init <
               meas.last_lio_update_time - kTimeEpsilon)
    {
      logVisualImageFlow(img_time_buffer.front() + exposure_time_init,
                         "image_dropped", "older_than_state");
      img_buffer.pop_front();
      img_time_buffer.pop_front();
    }
    mtx_buffer.unlock();

    const bool image_inside_scan =
        !p_imu->imu_need_init && !img_time_buffer.empty() &&
        img_time_buffer.front() + exposure_time_init <
            meas.lidar_frame_end_time - kTimeEpsilon;

    double event_time = meas.lidar_frame_end_time;
    if (image_inside_scan)
    {
      if (deterministic_sync_wait_for_image_lookahead_en_ &&
          static_cast<int>(img_time_buffer.size()) < sync_img_buffer_min_size_)
        return false;
      if (deterministic_sync_wait_for_image_lookahead_en_ &&
          sync_img_lookahead_time_ > 0.0 &&
          img_time_buffer.back() <
              img_time_buffer.front() + sync_img_lookahead_time_)
        return false;
      event_time = img_time_buffer.front() + exposure_time_init;
    }
    else if (!p_imu->imu_need_init &&
             last_timestamp_img + exposure_time_init <
                 meas.lidar_frame_end_time - kTimeEpsilon)
    {
      // Do not close the scan until the image stream has advanced beyond its
      // endpoint; otherwise a late in-scan image would force backward time.
      return false;
    }

    if (imu_en && last_timestamp_imu < event_time - kTimeEpsilon)
      return false;

    MeasureGroup m;
    mtx_buffer.lock();
    if (deterministic_imu_buffer_sort_en_)
    {
      std::sort(imu_buffer.begin(), imu_buffer.end(),
                [](const sensor_msgs::Imu::ConstPtr &a,
                   const sensor_msgs::Imu::ConstPtr &b) {
                  return a->header.stamp.toSec() < b->header.stamp.toSec();
                });
    }
    while (!imu_buffer.empty())
    {
      const double imu_time = imu_buffer.front()->header.stamp.toSec();
      if (imu_time > event_time) break;
      if (imu_time > meas.last_lio_update_time + kTimeEpsilon)
        m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }

    meas.measures.clear();
    if (image_inside_scan)
    {
      m.vio_time = event_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      meas.lio_vio_flg = VIO;
    }
    else
    {
      m.lio_time = meas.lidar_frame_end_time;
      lid_raw_data_buffer.pop_front();
      lid_header_time_buffer.pop_front();
      meas.lio_vio_flg = LIO;
      lidar_pushed = false;
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.measures.push_back(m);
    if (image_inside_scan)
    {
      logVisualImageFlow(event_time, "image_synced", "vio_measurement");
      ++runtime_events_.image_synced;
      livo_scan_lifecycle_.noteImage(event_time);
    }
    return true;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  ROS_ERROR("out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes,const ros::Publisher &pubLaserCloudMap, VIOManagerPtr vio_manager)
{

  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  const bool need_rgb_cloud = img_en && (colorize_cloud_en_ || pcd_save_en);
  if (need_rgb_cloud)
  {
    static int pub_num = 1;
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num == pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255;
          // else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255;
          // else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255;
          // else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
    else
    {
      pub_num++;
    }
  }
  else
  {
    PointCloudXYZI().swap(*pcl_wait_pub);
  }

  /*** Publish Frame ***/
  sensor_msgs::PointCloud2 laserCloudmsg;
  if (need_rgb_cloud)
  {
    // cout << "RGB pointcloud size: " << laserCloudWorldRGB->size() << endl;
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  else 
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes.publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    static int scan_wait_num = 0;
    
    if (need_rgb_cloud)
    {
      //global map

      if (global_map_pub)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_wait_save_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
        VoxelSafetyContext voxel_context;
        voxel_context.coordinate_frame = "world/current RGB frame";
        voxel_context.enforce_max_range = true;
        voxel_context.max_range_m = voxel_lidar_max_range_m_;
        voxel_context.range_origin = _state.pos_end;
        if (!safeVoxelFilter<pcl::PointXYZRGB>(laserCloudWorldRGB, pcl_wait_save_filter,
                                               Eigen::Vector3f::Constant(static_cast<float>(filter_size_pcd)),
                                               "VOXEL_PUBLISH_RGB", voxel_context))
        {
          ROS_ERROR_THROTTLE(5.0, "[VOXEL_REJECT] tag=VOXEL_PUBLISH_RGB action=skip_global_map_append");
          pcl_wait_save_filter->clear();
        }
        
        //*pcl_wait_save += *laserCloudWorldRGB;          //总地图添加未过滤点云
        *pcl_wait_save += *pcl_wait_save_filter;        //添加过滤后的点云

        pcl::toROSMsg(*pcl_wait_save, laserCloudmsg);   //发布
        pubLaserCloudMap.publish(laserCloudmsg);
      }
      else
      {
        *pcl_wait_save += *laserCloudWorldRGB;
      }
      if (pcd_cache_max_points > 0 && pcl_wait_save->size() > static_cast<size_t>(pcd_cache_max_points))
      {
        size_t overflow = pcl_wait_save->size() - static_cast<size_t>(pcd_cache_max_points);
        pcl_wait_save->points.erase(pcl_wait_save->points.begin(), pcl_wait_save->points.begin() + overflow);
        pcl_wait_save->width = pcl_wait_save->points.size();
        pcl_wait_save->height = 1;
        pcl_wait_save->is_dense = false;
      }
    }
    else
    {
      if (global_map_pub)
      {
        pcl::PointCloud<PointType>::Ptr pcl_wait_save_filter(new pcl::PointCloud<PointType>);
        VoxelSafetyContext voxel_context;
        voxel_context.coordinate_frame = "world/current intensity frame";
        voxel_context.enforce_max_range = true;
        voxel_context.max_range_m = voxel_lidar_max_range_m_;
        voxel_context.range_origin = _state.pos_end;
        if (!safeVoxelFilter<PointType>(pcl_w_wait_pub, pcl_wait_save_filter,
                                        Eigen::Vector3f::Constant(static_cast<float>(filter_size_pcd)),
                                        "VOXEL_PUBLISH_INTENSITY", voxel_context))
        {
          ROS_ERROR_THROTTLE(5.0, "[VOXEL_REJECT] tag=VOXEL_PUBLISH_INTENSITY action=skip_global_map_append");
          pcl_wait_save_filter->clear();
        }
        
        //*pcl_wait_save_intensity += *pcl_w_wait_pub;          //总地图添加未过滤点云
        *pcl_wait_save_intensity += *pcl_wait_save_filter;        //添加过滤后的点云

        pcl::toROSMsg(*pcl_wait_save_intensity, laserCloudmsg);   //发布
        pubLaserCloudMap.publish(laserCloudmsg);
      }
      else
      {
        *pcl_wait_save_intensity += *pcl_w_wait_pub;
      }
      if (pcd_cache_max_points > 0 && pcl_wait_save_intensity->size() > static_cast<size_t>(pcd_cache_max_points))
      {
        size_t overflow = pcl_wait_save_intensity->size() - static_cast<size_t>(pcd_cache_max_points);
        pcl_wait_save_intensity->points.erase(pcl_wait_save_intensity->points.begin(), pcl_wait_save_intensity->points.begin() + overflow);
        pcl_wait_save_intensity->width = pcl_wait_save_intensity->points.size();
        pcl_wait_save_intensity->height = 1;
        pcl_wait_save_intensity->is_dense = false;
      }
    }

    scan_wait_num++;
    
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      //string all_points_dir(string(string(ROOT_DIR) + "Log/PCD/") + to_string(pcd_index) + string(".pcd"));
      string all_points_dir(save_path + to_string(pcd_index) + string(".pcd"));
      /*string all_points_dir(save_path + string("map_dense.pcd"));
      string downsampled_points_dir(save_path + string("map.pcd"));
      string downsampled_points_dir2(save_path + string("pose.pcd"));*/
      pcl::PCDWriter pcd_writer;
      if (pcd_save_en)
      {
        cout << "current scan saved to /PCD/" << all_points_dir << endl;
        if (img_en)
        {
          /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
          voxel_filter.setInputCloud(pcl_wait_save);
          voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
          voxel_filter.filter(*downsampled_cloud);*/

          //pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the raw point cloud data
          //pcd_writer.writeBinary(downsampled_points_dir2, *downsampled_cloud); 
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save);    //清空缓存
        }
        else
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
          PointCloudXYZI().swap(*pcl_wait_save_intensity);
        }        
        scan_wait_num = 0;
      }
    }
  }

  if (save_log_en)
  {
    Eigen::Quaterniond q(_state.rot_end);
    const V3D out_pos = outputPosition();
    fout_pcd_pos << std::fixed << std::setprecision(9);
    if (pos_output_enable_timestamp_)
    {
      fout_pcd_pos << LidarMeasures.last_lio_update_time << " " << out_pos[0] << " " << out_pos[1] << " " << out_pos[2] << " "
                   << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << endl;
    }
    else
    {
      fout_pcd_pos << out_pos[0] << " " << out_pos[1] << " " << out_pos[2] << " " << q.w() << " " << q.x() << " " << q.y()
                   << " " << q.z() << " " << endl;
    }
  }
  
  if (need_rgb_cloud && laserCloudWorldRGB->size() > 0) PointCloudXYZI().swap(*pcl_wait_pub);
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const ros::Publisher &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap.publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  const V3D out_pos = outputPosition();
  out.position.x = out_pos(0);
  out.position.y = out_pos(1);
  out.position.z = out_pos(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = ros::Time::now(); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  const V3D out_pos = outputPosition();
  transform.setOrigin(tf::Vector3(out_pos(0), out_pos(1), out_pos(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped") );
  pubOdomAftMapped.publish(odomAftMapped);
  sendUdpPose(out_pos);
}

void LIVMapper::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher.publish(msg_body_pose);
}

void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}
