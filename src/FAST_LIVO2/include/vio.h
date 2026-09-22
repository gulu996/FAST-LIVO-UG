/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VIO_H_
#define VIO_H_

#include "voxel_map.h"
#include "feature.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/filters/voxel_grid.h>
#include <cstdint>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

struct SubSparseMap
{
  vector<float> propa_errors;
  vector<float> errors;
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<double> ncc_scores;
  vector<double> photometric_mses;
  vector<double> depths;
  vector<VisualPoint *> voxel_points;
  vector<double> inv_expo_list;
  vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    ncc_scores.reserve(SIZE_LARGE);
    photometric_mses.reserve(SIZE_LARGE);
    depths.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    ncc_scores.clear();
    photometric_mses.clear();
    depths.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

class Warp
{
public:
  Matrix2d A_cur_ref;
  int search_level;
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};

class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};

struct VisualScalarDistribution
{
  int count = 0;
  double mean = 0.0;
  double p10 = 0.0;
  double median = 0.0;
  double p90 = 0.0;
  double p95 = 0.0;
  double max = 0.0;
};

struct VisualMapSupplyDiagnostics
{
  int global_in_front = 0;
  int global_inside_image = 0;
  int global_border_valid = 0;
  int global_usable_tile = 0;
  int global_grid_cells = 0;
  int spatial_voxels = 0;
  int spatial_map_points = 0;
  int spatial_in_front = 0;
  int spatial_inside_image = 0;
  int spatial_border_valid = 0;
  int spatial_usable_tile = 0;
  int spatial_grid_cells = 0;
  VisualScalarDistribution global_all_distance;
  VisualScalarDistribution global_view_distance;
  VisualScalarDistribution spatial_all_distance;
  VisualScalarDistribution spatial_view_distance;
  VisualScalarDistribution global_view_abs_cos;
  VisualScalarDistribution spatial_view_abs_cos;
  VisualScalarDistribution global_view_reference_age;
  VisualScalarDistribution spatial_view_reference_age;
  VisualScalarDistribution global_view_last_seen_age;
  VisualScalarDistribution spatial_view_last_seen_age;
  VisualScalarDistribution global_view_observation_count;
  VisualScalarDistribution spatial_view_observation_count;
};

struct VisualCandidateDiagnostics
{
  int observation_count = 0;
  int last_seen_age = 0;
  int reference_level = 0;
  int grid_index = -1;
  bool from_fov_fallback = false;
  double view_angle_deg = 0.0;
  double warp_condition = 0.0;
  double warp_frobenius = 0.0;
  double warp_max_singular = 0.0;
  double depth_z = 0.0;
  double range_m = 0.0;
  double image_u = 0.0;
  double image_v = 0.0;
  V3D current_view_direction = V3D::Zero();
  V3D reference_view_direction = V3D::Zero();
};

struct VisualAdaptiveCovarianceShadowDiagnostics
{
  bool evaluated = false;
  bool valid = false;
  bool min_measurement_pass = false;
  bool observability_pass = false;
  bool nis_pass = false;
  int pyramid_level = -1;
  int tracked_points = 0;
  int measurement_dof = 0;
  int suppressed_directions = 0;
  double residual_rms = std::numeric_limits<double>::quiet_NaN();
  double robust_residual_rms = std::numeric_limits<double>::quiet_NaN();
  double total_nis = std::numeric_limits<double>::quiet_NaN();
  double normalized_nis = std::numeric_limits<double>::quiet_NaN();
  double rotation_min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double rotation_max_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double rotation_condition = std::numeric_limits<double>::quiet_NaN();
  double translation_min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double translation_max_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double translation_condition = std::numeric_limits<double>::quiet_NaN();
};

struct VisualPoseObservabilityDiagnostics
{
  bool valid = false;
  V3D rotation_eigenvalues = V3D::Constant(std::numeric_limits<double>::quiet_NaN());
  M3D rotation_eigenvectors = M3D::Constant(std::numeric_limits<double>::quiet_NaN());
  V3D rotation_direction_weights = V3D::Ones();
  V3D translation_eigenvalues = V3D::Constant(std::numeric_limits<double>::quiet_NaN());
  M3D translation_eigenvectors = M3D::Constant(std::numeric_limits<double>::quiet_NaN());
  V3D translation_direction_weights = V3D::Ones();
};

// Equal pixel-noise variance in the two images gives variance proportional
// to a^2+b^2. The sqrt(2) preserves the existing units when a=b=1.
struct ExposurePhotometricModel
{
  double current_weight = 0.0;
  double reference_weight = 0.0;
  double inverse_scale = 0.0;

  bool initialize(double current_inverse_exposure, double reference_inverse_exposure)
  {
    if (!std::isfinite(current_inverse_exposure) || current_inverse_exposure <= 0.0 ||
        !std::isfinite(reference_inverse_exposure) || reference_inverse_exposure <= 0.0)
      return false;
    const double scale = std::hypot(current_inverse_exposure, reference_inverse_exposure) / std::sqrt(2.0);
    current_weight = current_inverse_exposure / scale;
    reference_weight = reference_inverse_exposure / scale;
    inverse_scale = 1.0 / scale;
    return std::isfinite(current_weight) && std::isfinite(reference_weight) &&
        std::isfinite(inverse_scale) && inverse_scale > 0.0;
  }

  double residual(double current_intensity, double reference_intensity) const
  {
    return current_weight * current_intensity - reference_weight * reference_intensity;
  }

  double exposureJacobian(double current_intensity, double normalized_residual) const
  {
    // Differentiate the normalization too; freezing it recreates gain decay.
    return (current_intensity - 0.5 * current_weight * normalized_residual) * inverse_scale;
  }
};

class VIOManager
{
public:
  int grid_size;
  vk::AbstractCamera *cam;
  vk::PinholeCamera *pinhole_cam;
  StatesGroup *state;
  StatesGroup *state_propagat;
  M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  V3D Pli, Pci, Pcl, Pcw;
  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false, colmap_output_en = false;
  bool visual_robust_kernel_en = true;
  double visual_robust_delta = 20.0;
  bool visual_observability_gate_en = true;
  double visual_observability_relative_eigen_threshold = 0.02;
  double visual_relaxed_observability_relative_eigen_threshold = 0.02;
  double visual_observability_absolute_eigen_threshold = 0.0;

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;

  double fx, fy, cx, cy;
  double d0, d1, d2, d3;
  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double marker_size;
  bool aruco_landmarks_en = false;
  double aruco_min_quad_area_px = 300.0;
  double aruco_pair_distance_rel_tol = 0.2;
  double aruco_max_normal_diff_deg = 15.0;
  double aruco_max_marker_depth_diff = 0.6;
  double aruco_min_marker_depth = 0.1;
  double aruco_max_marker_depth = 10.0;
  double aruco_max_position_residual = 0.6;
  double aruco_max_orientation_residual_deg = 25.0;
  double aruco_position_noise_base = 0.01;
  double aruco_orientation_noise_base = 0.1;
  int aruco_process_stride = 4;
  bool aruco_use_orientation_update = false;
  double aruco_normal_gate_deg = 35.0;
  double aruco_update_max_rot_step_deg = 1.0;
  double aruco_update_max_trans_step_m = 0.08;

  struct BoardObservation 
  {
    int board_id;                     // 地标板子ID
    Eigen::Vector3d center_tvec;      // 地标中心点在相机坐标系下的位置
    Eigen::Matrix3d center_R_cam_board; // 地标中心点到相机的旋转
    int valid_count;                  // 有效的Aruco码数量（固定为4个）
    bool geometry_valid = false;      // 同ID四码几何一致性是否通过
    double center_spread_m = 0.0;     // 四码中心离散度（米）
    double rotation_dispersion_deg = 0.0; // 四码姿态离散度（度）
    //double timestamp;
  };

  struct ArucoObservation 
  {
    int id;
    Eigen::Vector3d tvec;
    Eigen::Matrix3d R_cam_marker;
    //double timestamp;
  };

  std::map<int, bool> board_world_flag_;
  std::map<int, Eigen::Vector3d> board_world_positions_;  // 地标中心点的世界坐标
  std::map<int, Eigen::Matrix3d> board_world_orientations_; // 地标中心点的世界姿态
  std::vector<BoardObservation> current_board_observations_;

  struct BoardConfig 
  {
    double width;     // 宽度（X方向）
    double height;    // 高度（Y方向）
    double marker_size; // Aruco码尺寸 
    double delta_width_qr_center;
    double delta_height_qr_center;
  };
  BoardConfig board_config_;
    
  // 四个Aruco码在板子坐标系下的相对位置
  std::map<int, Eigen::Vector3d> aruco_relative_positions_;

  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  enum class WarpRejectReason
  {
    None,
    InvalidImage,
    NonfiniteMatrix,
    NonfiniteCoordinate,
    OutOfBounds
  };
  WarpRejectReason last_warp_reject_reason = WarpRejectReason::None;
  int last_visual_warp_invalid_image_rejects = 0;
  int last_visual_warp_nonfinite_matrix_rejects = 0;
  int last_visual_warp_nonfinite_coordinate_rejects = 0;
  int last_visual_warp_oob_rejects = 0;
  int max_iterations, total_points;
  int min_retrieve_points = 30;
  bool visual_spatial_coverage_gate_en = false;
  int relaxed_min_retrieve_points = 20;
  int relaxed_min_occupied_good_tiles = 8;
  double relaxed_min_good_tile_ratio = 0.85;
  double relaxed_min_horizontal_coverage = 0.50;
  double relaxed_min_vertical_coverage = 0.50;
  bool lidar_degenerated = false;
  bool lidar_weak_translation_direction_valid = false;
  V3D lidar_weak_translation_direction_world = V3D::Zero();
  bool degeneracy_relaxed_track_gate_en = false;
  int degeneracy_relaxed_min_retrieve_points = 5;
  int degeneracy_relaxed_min_occupied_good_tiles = 4;
  double degeneracy_relaxed_min_good_tile_ratio = 0.60;
  double degeneracy_relaxed_min_horizontal_coverage = 0.40;
  double degeneracy_relaxed_min_vertical_coverage = 0.40;
  double degeneracy_visual_position_scale = 0.05;
  double degeneracy_visual_max_position_step_m = 0.003;
  bool visual_reference_refresh_en = false;
  int visual_reference_refresh_max_age_frames = 30;
  int visual_reference_refresh_min_tracked_points = 8;
  int visual_reference_refresh_min_occupied_good_tiles = 4;
  double visual_reference_refresh_min_horizontal_coverage = 0.30;
  double visual_reference_refresh_min_vertical_coverage = 0.30;
  int visual_reference_refresh_max_per_frame = 30;
  bool visual_search_level_low_contrast_retry_en = false;
  bool visual_adaptive_covariance_relaxed_en = false;
  int visual_adaptive_covariance_relaxed_min_points = 15;
  double visual_adaptive_covariance_k_ncc = 1.0;
  double visual_adaptive_covariance_k_level = 1.0;
  double visual_adaptive_covariance_k_photo = 0.5;
  double visual_adaptive_covariance_scale_max = 4.0;
  bool last_visual_tracked_gate_pass = false;
  bool last_visual_relaxed_track_gate_pass = false;
  bool last_visual_adaptive_covariance_relaxed_candidate = false;
  bool last_visual_adaptive_covariance_relaxed_gate_pass = false;
  bool current_visual_adaptive_covariance_relaxed_update = false;
  bool last_visual_degeneracy_relaxed_track_gate_pass = false;
  double last_visual_degeneracy_constrained_step_m = 0.0;
  int min_update_meas = 600;
  int low_track_force_update_stride = 0;
  int low_track_force_min_points = 8;
  bool deterministic_visual_update_en = true;
  bool deterministic_pixel_snap_en = true;
  bool deterministic_camera_point_snap_en = true;
  bool deterministic_contiguous_image_copy_en = true;
  bool deterministic_visual_voxel_key_sort_en = true;
  bool visual_update_guard_en = true;
  double visual_update_max_trans_m = 0.12;
  double visual_update_max_rot_deg = 2.0;
  double visual_update_max_backward_m = 0.03;
  double visual_update_max_backward_ratio = 0.08;
  double visual_update_backward_abs_floor_m = 0.003;
  double visual_update_max_lateral_m = 0.08;
  double visual_update_max_lateral_ratio = 0.35;
  double visual_update_max_exposure_delta = 0.30;
  double visual_update_max_velocity_increment_mps = 0.15;
  double visual_update_max_acc_bias_increment_mps2 = 0.03;
  double visual_update_max_gyro_bias_increment_rps = 0.005;
  double visual_update_normalized_nis_max = 0.0;
  bool last_visual_nis_rejected = false;
  bool last_visual_numerical_rejected = false;
  int last_visual_measurement_dof = 0; // one scalar photometric residual per measurement
  double last_visual_total_nis = std::numeric_limits<double>::quiet_NaN();
  double last_visual_normalized_nis = std::numeric_limits<double>::quiet_NaN();
  double last_visual_rotation_min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double last_visual_rotation_max_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double last_visual_rotation_condition = std::numeric_limits<double>::quiet_NaN();
  double last_visual_translation_min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double last_visual_translation_max_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double last_visual_translation_condition = std::numeric_limits<double>::quiet_NaN();
  int last_visual_observability_suppressed_directions = 0;
  int last_visual_candidate_patches = 0;
  int last_visual_patch_quality_rejects = 0;
  int last_visual_search_level_low_contrast_retries = 0;
  int last_visual_search_level_low_contrast_recovered = 0;
  int last_visual_depth_discontinuity_rejects = 0;
  int last_visual_normal_uninitialized_rejects = 0;
  int last_visual_warp_invalid_candidates = 0;
  int last_visual_ncc_rejects = 0;
  int last_visual_photometric_rejects = 0;
  int last_visual_ref_age_count = 0;
  int64_t last_visual_ref_age_sum = 0;
  int last_visual_ref_age_max = 0;
  int last_visual_ncc_pass_ref_age_count = 0;
  int64_t last_visual_ncc_pass_ref_age_sum = 0;
  int last_visual_ncc_pass_ref_age_max = 0;
  int last_visual_tracked_ref_age_count = 0;
  int64_t last_visual_tracked_ref_age_sum = 0;
  int last_visual_tracked_ref_age_max = 0;
  int last_visual_converged_ref_candidates = 0;
  int last_visual_reference_refreshes = 0;
  int diagnostics_console_interval_frames = 20;

  double img_point_cov, outlier_threshold, ncc_thre = 0.4;
  bool image_quality_gate_en = false;
  double image_quality_max_saturated_fraction = 0.20;
  double image_quality_max_tile_saturated_fraction = 0.35;
  double image_quality_max_dark_fraction = 0.98;
  double image_quality_min_intensity_std = 6.0;
  bool image_quality_tile_mask_en = false;
  double image_quality_min_usable_tile_ratio = 0.25;
  double image_quality_min_usable_horizontal_coverage = 0.50;
  double image_quality_min_usable_vertical_coverage = 0.50;
  bool visual_patch_quality_gate_en = true;
  double visual_patch_max_saturated_fraction = 0.10;
  double visual_patch_min_intensity_std = 2.0;
  int image_quality_saturated_pixel_value = 250;
  int image_quality_dark_pixel_value = 5;
  int image_quality_tile_rows = 4;
  int image_quality_tile_cols = 4;
  std::vector<uint8_t> image_quality_usable_tiles;
  int image_quality_mask_width = 0;
  int image_quality_mask_height = 0;
  std::vector<uint8_t> tracked_occupied_tiles;
  double last_image_saturated_fraction = 0.0;
  double last_image_max_tile_saturated_fraction = 0.0;
  double last_image_dark_fraction = 0.0;
  double last_image_contrast = 0.0;
  int last_image_usable_tiles = 0;
  int last_image_unusable_tiles = 0;
  int last_image_overexposed_tiles = 0;
  int last_image_underexposed_tiles = 0;
  int last_image_low_contrast_tiles = 0;
  double last_image_usable_tile_ratio = 0.0;
  double last_image_usable_horizontal_coverage = 0.0;
  double last_image_usable_vertical_coverage = 0.0;
  bool last_image_global_saturation_fail = false;
  bool last_image_global_dark_fail = false;
  bool last_image_global_contrast_fail = false;
  std::string last_image_quality_reject_reason = "none";
  double max_state_update_rot_deg = 0.8;
  double max_state_update_trans_m = 0.08;
  bool visual_map_prune_en = true;
  int visual_map_max_voxels = 1800;
  int visual_map_max_points_per_voxel = 10;
  int visual_map_max_total_points = 20000;
  int visual_map_max_add_per_frame = 300;
  float visual_map_min_shi_tomasi_score = 10.0f;
  int last_visual_map_pg_size = 0;
  int last_visual_map_candidate_slots = 0;
  int last_visual_map_patch_rejects = 0;
  int last_visual_map_added_points = 0;
  int last_visual_map_insert_reject_invalid = 0;
  int last_visual_map_insert_reject_voxel_full = 0;
  int last_visual_map_insert_reject_voxel_cap = 0;
  int last_visual_map_evicted_points = 0;
  int last_visual_map_evicted_voxels = 0;
  size_t last_visual_map_total_points_before = 0;
  size_t last_visual_map_total_points_after = 0;
  size_t last_visual_map_voxels_before = 0;
  size_t last_visual_map_voxels_after = 0;
  size_t last_visual_map_points_at_retrieve = 0;
  size_t last_visual_map_voxels_at_retrieve = 0;
  int last_visual_projected_candidates = 0;
  int last_visual_inside_image_candidates = 0;
  int last_visual_good_tile_candidates = 0;
  int last_visual_grid_candidates = 0;
  int last_visual_occupied_good_tiles = 0;
  double last_visual_occupied_tile_ratio = 0.0;
  double last_visual_horizontal_coverage = 0.0;
  double last_visual_vertical_coverage = 0.0;
  double visual_voxel_size = 0.5;
  bool visual_map_supply_diagnostics_en = false;
  bool visual_map_fov_fallback_en = false;
  bool visual_adaptive_covariance_shadow_en = false;
  int visual_map_fov_fallback_target_grid_candidates = 60;
  int last_visual_fov_fallback_added_grid_candidates = 0;
  VisualMapSupplyDiagnostics last_visual_map_supply;
  VisualCandidateDiagnostics current_visual_candidate_diagnostics;
  VisualAdaptiveCovarianceShadowDiagnostics last_visual_adaptive_covariance_shadow;
  VisualPoseObservabilityDiagnostics last_visual_pose_observability;
  VisualScalarDistribution last_visual_adaptive_scale_distribution;
  VisualScalarDistribution last_visual_adaptive_ncc_distribution;
  VisualScalarDistribution last_visual_adaptive_photo_distribution;
  VisualScalarDistribution last_visual_adaptive_depth_distribution;
  std::array<int, 3> last_visual_adaptive_search_level_hist{{0, 0, 0}};
  std::vector<double> current_visual_adaptive_point_scales;
  double last_visual_huber_pose_information_trace = std::numeric_limits<double>::quiet_NaN();
  double last_visual_weighted_pose_information_trace = std::numeric_limits<double>::quiet_NaN();
  double last_visual_suppressed_pose_information_trace = std::numeric_limits<double>::quiet_NaN();
  double current_visual_process_begin_wall_time = 0.0;
  bool current_visual_tracking_only_dry_run = false;
  std::string current_visual_frame_mode = "NORMAL_LIDAR_SUPPORTED";
  bool console_timing_print_en = true;
  int console_timing_print_stride = 1;
  
  SubSparseMap *visual_submap;
  std::vector<std::vector<V3D>> rays_with_sample_points;

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  double aruco_time_detect_markers = 0.0;
  double aruco_time_draw = 0.0;
  double aruco_time_group_gate = 0.0;
  double aruco_time_pose_estimate = 0.0;
  double aruco_time_pnp = 0.0;
  double aruco_time_update = 0.0;
  double aruco_time_total = 0.0;
  double aruco_ave_time_total = 0.0;
  int aruco_profile_frames = 0;
  int aruco_board_candidates = 0;
  int aruco_board_accepted = 0;
  double last_visual_guard_time = -1.0;
  bool has_last_visual_guard_pos = false;
  V3D last_visual_guard_pos = V3D::Zero();
  bool last_visual_update_accepted = false;

  string timing_log_dir;
  string timing_log_file_path;
  bool timing_log_enable = true;
  bool timing_log_ready = false;
  int timing_log_flush_stride = 10;
  int timing_log_pending_frames = 0;

  Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  ofstream timing_log_file;
  ofstream visual_patch_quality_file;
  ofstream visual_funnel_file;
  ofstream visual_map_supply_file;
  ofstream visual_adaptive_covariance_shadow_file;
  ofstream visual_adaptive_covariance_relaxed_file;
  int visual_patch_quality_pending_rows = 0;
  int visual_funnel_pending_rows = 0;
  int visual_map_supply_pending_rows = 0;
  int visual_adaptive_covariance_shadow_pending_rows = 0;
  int visual_adaptive_covariance_relaxed_pending_rows = 0;
  double current_visual_time = 0.0;
  bool last_visual_observability_rejected = false;
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;
  unordered_map<VOXEL_LOCATION, int> sub_feat_map; 
  std::unordered_set<const VisualPoint *> protected_visual_points_;
  unordered_map<int, Warp *> warp_map;
  vector<VisualPoint *> retrieve_voxel_points;
  vector<uint8_t> retrieve_voxel_from_fov_fallback;
  vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_;
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();
  bool updateStateInverse(cv::Mat img, int level);
  bool updateState(cv::Mat img, int level);
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg,
                    const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map,
                    double img_time, bool tracking_only_dry_run = false);
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);
  void clearVisualMap();
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  void initializeVIO(ros::NodeHandle &nh);
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  bool getVisualVoxelLocation(const V3D &point, VOXEL_LOCATION &location) const;
  bool computeJacobianAndUpdateEKF(cv::Mat img);
  void resetGrid();
  void updateVisualMapPoints(cv::Mat img);
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3 &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3 &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);
  bool warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  bool insertPointIntoVoxelMap(VisualPoint *pt_new);
  size_t getVisualPointCount() const;
  void refreshProtectedVisualPointSet();
  bool isVisualPointProtected(const VisualPoint *pt) const;
  int getVisualPointOldestFrameId(const VisualPoint *pt) const;
  bool eraseOldestVisualPointFromVoxel(VOXEL_POINTS *voxel, bool protect_current_submap);
  size_t pruneOldestVisualPointsTo(size_t target_points, bool protect_current_submap);
  size_t pruneOldestVisualVoxelsTo(size_t target_voxels, bool protect_current_submap);
  void pruneVisualMap();
  void plotTrackedPoints();
  void updateFrameState(StatesGroup state);
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void precomputeReferencePatches(int level);
  void dumpDataForColmap();
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);
  void detect_qr(cv::Mat img);
  void draw_qr(std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners, std::vector<std::vector<cv::Point2f>>& rejectedCandidates);
  void updateStateWithBoardObservation();
  Eigen::Matrix3d Exp(const Eigen::Vector3d& w);
  Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
  void initializeTimingLogFileIfNeeded();
  void appendTimingLogLines(const vector<string> &lines);
  void logVisualPatchQuality(double photometric_mse, double ncc,
                             const char *decision, int ref_age,
                             int search_level, double warp_determinant,
                             bool warp_valid, double ref_inv_exposure,
                             double current_inv_exposure);
  void logVisualFunnel(const std::string &skip_reason, bool ekf_attempted,
                       bool final_guard_rejected, bool accepted);
  void logVisualMapSupply();
  void evaluateVisualAdaptiveCovarianceShadow(const cv::Mat &img);
  void logVisualAdaptiveCovarianceShadow();
  void prepareVisualAdaptiveCovarianceDiagnostics(bool compute_scales);
  void applyVisualAdaptiveCovarianceWhitening(Eigen::VectorXd &residuals,
                                              Eigen::MatrixXd &jacobian) const;
  void logVisualAdaptiveCovarianceRelaxed(const std::string &decision,
                                          bool ekf_attempted, bool accepted,
                                          bool guard_rejected, bool rollback,
                                          const StatesGroup &before,
                                          const StatesGroup &attempted);
  bool isPixelInUsableTile(const V2D &px) const;
  void updateTrackedSpatialCoverage();
  void refreshTrackedReferencePatches(cv::Mat img);
  void applyPatchRobustWeights(Eigen::VectorXd &residuals,
                               Eigen::MatrixXd &jacobian) const;
  void recordHuberMeasurementInformation(const Eigen::MatrixXd &jacobian);
  double activeVisualObservabilityRelativeThreshold() const;
  bool applyPoseObservabilityGate(Eigen::MatrixXd &jacobian);
  void logVisualDelta(double timestamp, int tracked_point_count,
                      double image_saturated_fraction,
                      double image_tile_saturated_fraction,
                      double image_contrast, const std::string &skip_reason,
                      const StatesGroup &before, const StatesGroup &attempted,
                      double visual_total_nis, bool accepted);
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

#endif // VIO_H_
