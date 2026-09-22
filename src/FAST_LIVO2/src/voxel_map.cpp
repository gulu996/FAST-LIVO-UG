/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "voxel_map.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <pcl/kdtree/kdtree_flann.h>

int voxel_plane_id = 0;

namespace
{

struct ScalarDiagnostics
{
  double mean = 0.0;
  double median = 0.0;
  double p90 = 0.0;
  double p95 = 0.0;
  double rmse = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
};

ScalarDiagnostics summarizeFiniteValues(std::vector<double> values)
{
  values.erase(std::remove_if(values.begin(), values.end(),
                              [](double value) { return !std::isfinite(value); }),
               values.end());
  ScalarDiagnostics summary;
  if (values.empty()) return summary;
  std::sort(values.begin(), values.end());
  double sum = 0.0;
  double sum_squares = 0.0;
  for (const double value : values)
  {
    sum += value;
    sum_squares += value * value;
  }
  const auto percentile = [&values](double fraction) {
    const double index = fraction * static_cast<double>(values.size() - 1);
    const size_t lower = static_cast<size_t>(std::floor(index));
    const size_t upper = std::min(values.size() - 1, lower + 1);
    const double alpha = index - static_cast<double>(lower);
    return values[lower] * (1.0 - alpha) + values[upper] * alpha;
  };
  summary.mean = sum / static_cast<double>(values.size());
  summary.median = percentile(0.50);
  summary.p90 = percentile(0.90);
  summary.p95 = percentile(0.95);
  summary.rmse = std::sqrt(sum_squares / static_cast<double>(values.size()));
  summary.minimum = values.front();
  summary.maximum = values.back();
  return summary;
}

Eigen::Vector3d relativeRpyDegrees(const StatesGroup &from, const StatesGroup &to)
{
  const Eigen::Matrix3d relative = from.rot_end.transpose() * to.rot_end;
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, -relative(2, 0))));
  const double roll = std::atan2(relative(2, 1), relative(2, 2));
  const double yaw = std::atan2(relative(1, 0), relative(0, 0));
  constexpr double kRadToDeg = 57.29577951308232;
  return Eigen::Vector3d(roll, pitch, yaw) * kRadToDeg;
}

Eigen::Vector3d symmetricEigenvalues(const Eigen::Matrix3d &matrix)
{
  const Eigen::Matrix3d symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  return solver.info() == Eigen::Success && solver.eigenvalues().allFinite()
      ? solver.eigenvalues()
      : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

} // namespace

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov)
{
  if (pb[2] == 0) pb[2] = 0.0001;
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config)
{
  nh.param<bool>("publish/pub_plane_en", voxel_config.is_pub_plane_map_, false);
  
  nh.param<int>("lio/max_layer", voxel_config.max_layer_, 1);
  nh.param<double>("lio/voxel_size", voxel_config.max_voxel_size_, 0.5);
  nh.param<double>("lio/min_eigen_value", voxel_config.planner_threshold_, 0.01);
  nh.param<double>("lio/sigma_num", voxel_config.sigma_num_, 3);
  nh.param<double>("lio/beam_err", voxel_config.beam_err_, 0.02);
  nh.param<double>("lio/dept_err", voxel_config.dept_err_, 0.05);
  nh.param<vector<int>>("lio/layer_init_num", voxel_config.layer_init_num_, vector<int>{5,5,5,5,5});
  nh.param<int>("lio/max_points_num", voxel_config.max_points_num_, 50);
  nh.param<int>("lio/max_iterations", voxel_config.max_iterations_, 5);

  nh.param<bool>("local_map/map_sliding_en", voxel_config.map_sliding_en, false);
  nh.param<int>("local_map/half_map_size", voxel_config.half_map_size, 100);
  nh.param<double>("local_map/sliding_thresh", voxel_config.sliding_thresh, 8);

  nh.param<bool>("local_map/long_term_visual_map_en", voxel_config.long_term_visual_map_en, true);
  nh.param<int>("local_map/long_term_visual_max_voxels", voxel_config.long_term_visual_max_voxels, 5000);

  nh.param<bool>("lio_degeneracy/enable_observability_diagnostics",
                 voxel_config.observability_diagnostics_enable, true);
  nh.param<bool>("lio_degeneracy/enable_state_intervention",
                 voxel_config.state_intervention_enable, false);
  nh.param<bool>("lio_degeneracy/use_conditional_translation_information",
                 voxel_config.use_conditional_translation_information, true);
  nh.param<double>("lio_degeneracy/rotation_regularization",
                   voxel_config.degeneracy_rotation_regularization, 1e-6);
  nh.param<int>("lio_degeneracy/min_effective_features",
                voxel_config.degeneracy_min_effective_features, 50);
  nh.param<double>("lio_degeneracy/min_translation_eigenvalue",
                   voxel_config.degeneracy_min_translation_eigenvalue, 0.0);
  nh.param<double>("lio_degeneracy/max_translation_condition_number",
                   voxel_config.degeneracy_max_translation_condition_number, 1000.0);
  nh.param<double>("lio_degeneracy/min_translation_eigenvalue_ratio",
                   voxel_config.degeneracy_ratio_thresh, 0.02);
  nh.param<int>("lio_degeneracy/enter_consecutive_frames",
                voxel_config.degeneracy_enter_consecutive_frames, 3);
  nh.param<int>("lio_degeneracy/exit_consecutive_frames",
                voxel_config.degeneracy_exit_consecutive_frames, 8);
  nh.param<bool>("lio_degeneracy/directional_shadow_enable",
                 voxel_config.directional_shadow_enable, false);
  nh.param<std::vector<double>>(
      "lio_degeneracy/directional_shadow_relative_thresholds",
      voxel_config.directional_shadow_relative_thresholds,
      std::vector<double>{0.02, 0.05, 0.08, 0.10, 0.15, 0.20});

  nh.param<std::string>("lio_direction_guard/mode",
                        voxel_config.direction_guard_mode, "diagnostic");
  nh.param<double>("lio_direction_guard/min_predicted_speed_mps",
                   voxel_config.direction_guard_min_predicted_speed_mps, 0.30);
  nh.param<double>("lio_direction_guard/min_velocity_weak_direction_cos",
                   voxel_config.direction_guard_min_velocity_weak_direction_cos, 0.70);
  nh.param<double>("lio_direction_guard/max_opposite_correction_m",
                   voxel_config.direction_guard_max_opposite_correction_m, 0.03);
  nh.param<double>("lio_direction_guard/max_opposite_velocity_correction_mps",
                   voxel_config.direction_guard_max_opposite_velocity_correction_mps, 0.03);
  nh.param<int>("lio_direction_guard/enter_consecutive_frames",
                voxel_config.direction_guard_enter_consecutive_frames, 2);
  nh.param<int>("lio_direction_guard/exit_consecutive_frames",
                voxel_config.direction_guard_exit_consecutive_frames, 5);

  nh.param<std::string>("lio_map_guard/mode", voxel_config.map_guard_mode, "off");
  nh.param<bool>("lio_map_guard/freeze_on_degeneracy",
                 voxel_config.map_guard_freeze_on_degeneracy, true);
  nh.param<bool>("lio_map_guard/freeze_on_direction_reject",
                 voxel_config.map_guard_freeze_on_direction_reject, true);
  nh.param<double>("lio_map_guard/severe_translation_eigenvalue_ratio",
                   voxel_config.map_guard_severe_translation_eigenvalue_ratio, 0.005);
  nh.param<int>("lio_map_guard/recovery_consecutive_frames",
                voxel_config.map_guard_recovery_consecutive_frames, 8);
  nh.param<int>("lio_map_guard/maximum_freeze_frames",
                voxel_config.map_guard_maximum_freeze_frames, 300);
  voxel_config.degeneracy_rotation_regularization =
      std::max(1e-12, voxel_config.degeneracy_rotation_regularization);
  voxel_config.degeneracy_min_effective_features =
      std::max(1, voxel_config.degeneracy_min_effective_features);
  voxel_config.degeneracy_min_translation_eigenvalue =
      std::max(0.0, voxel_config.degeneracy_min_translation_eigenvalue);
  voxel_config.degeneracy_max_translation_condition_number =
      std::max(1.0, voxel_config.degeneracy_max_translation_condition_number);
  voxel_config.degeneracy_ratio_thresh =
      std::max(0.0, std::min(1.0, voxel_config.degeneracy_ratio_thresh));
  voxel_config.degeneracy_enter_consecutive_frames =
      std::max(1, voxel_config.degeneracy_enter_consecutive_frames);
  voxel_config.degeneracy_exit_consecutive_frames =
      std::max(1, voxel_config.degeneracy_exit_consecutive_frames);
  for (double &threshold : voxel_config.directional_shadow_relative_thresholds)
    threshold = std::max(0.0, std::min(1.0, threshold));
  std::sort(voxel_config.directional_shadow_relative_thresholds.begin(),
            voxel_config.directional_shadow_relative_thresholds.end());
  voxel_config.directional_shadow_relative_thresholds.erase(
      std::unique(voxel_config.directional_shadow_relative_thresholds.begin(),
                  voxel_config.directional_shadow_relative_thresholds.end()),
      voxel_config.directional_shadow_relative_thresholds.end());
  voxel_config.direction_guard_min_predicted_speed_mps =
      std::max(0.0, voxel_config.direction_guard_min_predicted_speed_mps);
  voxel_config.direction_guard_min_velocity_weak_direction_cos =
      std::max(0.0, std::min(1.0, voxel_config.direction_guard_min_velocity_weak_direction_cos));
  voxel_config.direction_guard_max_opposite_correction_m =
      std::max(0.0, voxel_config.direction_guard_max_opposite_correction_m);
  voxel_config.direction_guard_max_opposite_velocity_correction_mps =
      std::max(0.0, voxel_config.direction_guard_max_opposite_velocity_correction_mps);
  voxel_config.direction_guard_enter_consecutive_frames =
      std::max(1, voxel_config.direction_guard_enter_consecutive_frames);
  voxel_config.direction_guard_exit_consecutive_frames =
      std::max(1, voxel_config.direction_guard_exit_consecutive_frames);
  voxel_config.map_guard_recovery_consecutive_frames =
      std::max(1, voxel_config.map_guard_recovery_consecutive_frames);
  voxel_config.map_guard_severe_translation_eigenvalue_ratio =
      std::max(0.0, std::min(1.0, voxel_config.map_guard_severe_translation_eigenvalue_ratio));
  voxel_config.map_guard_maximum_freeze_frames =
      std::max(0, voxel_config.map_guard_maximum_freeze_frames);

  auto sanitize_mode = [](std::string &mode, const char *parameter, const char *fallback) {
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode != "off" && mode != "diagnostic" && mode != "enforce")
    {
      ROS_WARN("Invalid %s='%s'; using '%s'.", parameter, mode.c_str(), fallback);
      mode = fallback;
    }
  };
  sanitize_mode(voxel_config.direction_guard_mode, "lio_direction_guard/mode", "diagnostic");
  sanitize_mode(voxel_config.map_guard_mode, "lio_map_guard/mode", "off");

  if (voxel_config.state_intervention_enable)
  {
    ROS_WARN("LIO degeneracy state intervention is experimental and unavailable in the stable baseline; using diagnostic-only ESIKF updates.");
    voxel_config.state_intervention_enable = false;
  }
  if (voxel_config.direction_guard_mode == "enforce")
  {
    ROS_WARN("LIO direction guard enforce mode has not passed real stop, turn-around, or return validation; falling back to diagnostic mode.");
    voxel_config.direction_guard_mode = "diagnostic";
  }

  ROS_INFO("LIO degeneracy observability: %s",
           voxel_config.observability_diagnostics_enable ? "enabled" : "disabled");
  ROS_INFO("LIO degeneracy state intervention: disabled");
  ROS_INFO("[LIO_DEGEN_CONFIG] diagnostics=%d intervention=%d conditional=%d rot_reg=%.3e min_features=%d min_eigen=%.6g max_condition=%.3f min_ratio=%.6g enter=%d exit=%d",
           static_cast<int>(voxel_config.observability_diagnostics_enable),
           static_cast<int>(voxel_config.state_intervention_enable),
           static_cast<int>(voxel_config.use_conditional_translation_information),
           voxel_config.degeneracy_rotation_regularization,
           voxel_config.degeneracy_min_effective_features,
           voxel_config.degeneracy_min_translation_eigenvalue,
           voxel_config.degeneracy_max_translation_condition_number,
           voxel_config.degeneracy_ratio_thresh,
           voxel_config.degeneracy_enter_consecutive_frames,
           voxel_config.degeneracy_exit_consecutive_frames);
  std::ostringstream shadow_thresholds;
  for (size_t i = 0; i < voxel_config.directional_shadow_relative_thresholds.size(); ++i)
  {
    if (i != 0) shadow_thresholds << ',';
    shadow_thresholds << voxel_config.directional_shadow_relative_thresholds[i];
  }
  ROS_INFO("[LIO_SHADOW_CONFIG] enabled=%d relative_thresholds=[%s] real_state_intervention=0",
           static_cast<int>(voxel_config.directional_shadow_enable),
           shadow_thresholds.str().c_str());
  ROS_INFO("[LIO_GUARD_CONFIG] direction_mode=%s speed=%.3f weak_cos=%.3f opposite_position=%.3f opposite_velocity=%.3f enter=%d exit=%d map_mode=%s severe_ratio=%.6g recovery=%d max_freeze=%d",
           voxel_config.direction_guard_mode.c_str(),
           voxel_config.direction_guard_min_predicted_speed_mps,
           voxel_config.direction_guard_min_velocity_weak_direction_cos,
           voxel_config.direction_guard_max_opposite_correction_m,
           voxel_config.direction_guard_max_opposite_velocity_correction_mps,
           voxel_config.direction_guard_enter_consecutive_frames,
           voxel_config.direction_guard_exit_consecutive_frames,
           voxel_config.map_guard_mode.c_str(),
           voxel_config.map_guard_severe_translation_eigenvalue_ratio,
           voxel_config.map_guard_recovery_consecutive_frames,
           voxel_config.map_guard_maximum_freeze_frames);

  nh.param<int>("lio/icp_min_iterations", voxel_config.icp_min_iterations, 2);
  nh.param<double>("lio/icp_early_stop_residual_ratio", voxel_config.icp_early_stop_residual_ratio, 0.03);
  nh.param<double>("lio/icp_max_rot_step_deg", voxel_config.icp_max_rot_step_deg, 1.2);
  nh.param<double>("lio/icp_max_trans_step_m", voxel_config.icp_max_trans_step_m, 0.20);

  bool legacy_deterministic_lio_update_en = true;
  nh.param<bool>("lio/deterministic_lio_update_en", legacy_deterministic_lio_update_en, true);
  nh.param<bool>("deterministic_debug/lio_update_serial_en",
                 voxel_config.deterministic_lio_update_en,
                 legacy_deterministic_lio_update_en);
  nh.param<int>("lio_commit/minimum_correspondences",
                voxel_config.commit_minimum_correspondences, 6);
  nh.param<double>("lio_commit/maximum_cost_ratio",
                   voxel_config.commit_maximum_cost_ratio, 2.0);
  nh.param<double>("lio_commit/maximum_cost_increase",
                   voxel_config.commit_maximum_cost_increase, 0.20);
  nh.param<double>("lio_commit/maximum_residual",
                   voxel_config.commit_maximum_residual, 5.0);
  nh.param<double>("lio_commit/covariance_symmetry_relative_tolerance",
                   voxel_config.commit_covariance_symmetry_relative_tolerance, 1e-7);
  nh.param<double>("lio_commit/covariance_psd_relative_tolerance",
                   voxel_config.commit_covariance_psd_relative_tolerance, 1e-10);
  voxel_config.commit_minimum_correspondences =
      std::max(1, voxel_config.commit_minimum_correspondences);
  voxel_config.commit_maximum_cost_ratio =
      std::max(1.0, voxel_config.commit_maximum_cost_ratio);
  voxel_config.commit_maximum_cost_increase =
      std::max(0.0, voxel_config.commit_maximum_cost_increase);
  voxel_config.commit_maximum_residual =
      std::max(0.0, voxel_config.commit_maximum_residual);
  voxel_config.commit_covariance_symmetry_relative_tolerance =
      std::max(0.0, voxel_config.commit_covariance_symmetry_relative_tolerance);
  voxel_config.commit_covariance_psd_relative_tolerance =
      std::max(0.0, voxel_config.commit_covariance_psd_relative_tolerance);
  ROS_INFO("[LIO_COMMIT_CONFIG] min_correspondences=%d cost_ratio=%.3f cost_increase=%.3f max_residual=%.3f cov_sym=%.3e cov_psd=%.3e",
           voxel_config.commit_minimum_correspondences,
           voxel_config.commit_maximum_cost_ratio,
           voxel_config.commit_maximum_cost_increase,
           voxel_config.commit_maximum_residual,
           voxel_config.commit_covariance_symmetry_relative_tolerance,
           voxel_config.commit_covariance_psd_relative_tolerance);

  nh.param<bool>("p4_frontend/enable",
                 voxel_config.p4_frontend_diagnostics_enable, false);
  nh.param<std::string>("p4_frontend/variant",
                        voxel_config.p4_frontend_variant, "observe");
  nh.param<std::string>("p4_frontend/output_directory",
                        voxel_config.p4_frontend_output_directory, "");
  nh.param<double>("p4_frontend/analysis_start_s",
                   voxel_config.p4_analysis_start_s, 0.0);
  nh.param<double>("p4_frontend/analysis_end_s",
                   voxel_config.p4_analysis_end_s,
                   std::numeric_limits<double>::infinity());
  nh.param<double>("p4_frontend/detail_start_s",
                   voxel_config.p4_detail_start_s,
                   std::numeric_limits<double>::infinity());
  nh.param<double>("p4_frontend/detail_end_s",
                   voxel_config.p4_detail_end_s,
                   -std::numeric_limits<double>::infinity());
  nh.param<double>("p4_frontend/frozen_map_start_s",
                   voxel_config.p4_frozen_map_start_s,
                   std::numeric_limits<double>::infinity());
  nh.param<int>("p4_frontend/detail_frame_stride",
                voxel_config.p4_detail_frame_stride, 1);
  nh.param<int>("p4_frontend/counterfactual_frame_stride",
                voxel_config.p4_counterfactual_frame_stride, 1);
  nh.param<bool>("p4b_snapshot/enable",
                 voxel_config.p4b_snapshot_enable, false);
  nh.param<bool>("p4b_snapshot/retain_support_points",
                 voxel_config.p4b_retain_support_points,
                 voxel_config.p4b_snapshot_enable);
  nh.param<bool>("p5_seed_basin/enable",
                 voxel_config.p5_seed_basin_enable, false);
  nh.param<bool>("p5_seed_basin/prototype_enable",
                 voxel_config.p5_weak_axis_multistart_enable, false);
  nh.param<std::string>("p5_seed_basin/output_directory",
                        voxel_config.p5_seed_basin_output_directory, "");
  nh.param<double>("p5_seed_basin/start_s",
                   voxel_config.p5_seed_basin_start_s, 0.0);
  nh.param<double>("p5_seed_basin/end_s",
                   voxel_config.p5_seed_basin_end_s, 0.0);
  nh.param<int>("p5_seed_basin/frame_stride",
                voxel_config.p5_seed_basin_frame_stride, 10);
  voxel_config.p4_detail_frame_stride =
      std::max(1, voxel_config.p4_detail_frame_stride);
  voxel_config.p4_counterfactual_frame_stride =
      std::max(1, voxel_config.p4_counterfactual_frame_stride);
  voxel_config.p5_seed_basin_frame_stride =
      std::max(1, voxel_config.p5_seed_basin_frame_stride);
  std::transform(voxel_config.p4_frontend_variant.begin(),
                 voxel_config.p4_frontend_variant.end(),
                 voxel_config.p4_frontend_variant.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (voxel_config.p4_frontend_variant != "observe" &&
      voxel_config.p4_frontend_variant != "frozen_map")
  {
    ROS_WARN("Invalid p4_frontend/variant='%s'; using observe.",
             voxel_config.p4_frontend_variant.c_str());
    voxel_config.p4_frontend_variant = "observe";
  }
  ROS_INFO("[P4_FRONTEND_CONFIG] enable=%d variant=%s analysis=[%.3f,%.3f] detail=[%.3f,%.3f]/%d counterfactual_stride=%d freeze=%.3f output=%s",
           static_cast<int>(voxel_config.p4_frontend_diagnostics_enable),
           voxel_config.p4_frontend_variant.c_str(),
           voxel_config.p4_analysis_start_s, voxel_config.p4_analysis_end_s,
           voxel_config.p4_detail_start_s, voxel_config.p4_detail_end_s,
           voxel_config.p4_detail_frame_stride,
           voxel_config.p4_counterfactual_frame_stride,
           voxel_config.p4_frozen_map_start_s,
           voxel_config.p4_frontend_output_directory.c_str());
  ROS_INFO("[P4B_SNAPSHOT_CONFIG] enable=%d retain_support_points=%d",
           static_cast<int>(voxel_config.p4b_snapshot_enable),
           static_cast<int>(voxel_config.p4b_retain_support_points));
  ROS_INFO("[P5_SEED_BASIN_CONFIG] enable=%d prototype=%d window=[%.3f,%.3f] stride=%d output=%s",
           static_cast<int>(voxel_config.p5_seed_basin_enable),
           static_cast<int>(voxel_config.p5_weak_axis_multistart_enable),
           voxel_config.p5_seed_basin_start_s,
           voxel_config.p5_seed_basin_end_s,
           voxel_config.p5_seed_basin_frame_stride,
           voxel_config.p5_seed_basin_output_directory.c_str());
}

VoxelOctoTreeSnapshot::VoxelOctoTreeSnapshot(
    const VoxelOctoTreeSnapshot &other)
{
  *this = other;
}

VoxelOctoTreeSnapshot &VoxelOctoTreeSnapshot::operator=(
    const VoxelOctoTreeSnapshot &other)
{
  if (this == &other) return *this;
  temp_points = other.temp_points;
  plane = other.plane;
  layer = other.layer;
  octo_state = other.octo_state;
  voxel_center = other.voxel_center;
  layer_init_num = other.layer_init_num;
  quarter_length = other.quarter_length;
  planer_threshold = other.planer_threshold;
  points_size_threshold = other.points_size_threshold;
  update_size_threshold = other.update_size_threshold;
  max_points_num = other.max_points_num;
  max_layer = other.max_layer;
  new_points = other.new_points;
  init_octo = other.init_octo;
  update_enable = other.update_enable;
  p4b_retain_support = other.p4b_retain_support;
  for (std::size_t i = 0; i < leaves.size(); ++i)
    leaves[i] = other.leaves[i]
        ? std::unique_ptr<VoxelOctoTreeSnapshot>(
              new VoxelOctoTreeSnapshot(*other.leaves[i]))
        : nullptr;
  return *this;
}

VoxelOctoTree::VoxelOctoTree()
    : plane_ptr_(new VoxelPlane), layer_(0), octo_state_(0),
      quater_length_(0.0f), planer_threshold_(0.0f),
      points_size_threshold_(0), update_size_threshold_(5),
      max_points_num_(0), max_layer_(0), new_points_(0),
      init_octo_(false), update_enable_(true)
{
  voxel_center_[0] = voxel_center_[1] = voxel_center_[2] = 0.0;
  for (VoxelOctoTree *&leaf : leaves_) leaf = nullptr;
}

VoxelOctoTreeSnapshot VoxelOctoTree::captureSnapshot() const
{
  VoxelOctoTreeSnapshot result;
  result.temp_points = temp_points_;
  if (plane_ptr_) result.plane = *plane_ptr_;
  result.layer = layer_;
  result.octo_state = octo_state_;
  for (int i = 0; i < 8; ++i)
    if (leaves_[i])
      result.leaves[i].reset(
          new VoxelOctoTreeSnapshot(leaves_[i]->captureSnapshot()));
  for (int i = 0; i < 3; ++i) result.voxel_center[i] = voxel_center_[i];
  result.layer_init_num = layer_init_num_;
  result.quarter_length = quater_length_;
  result.planer_threshold = planer_threshold_;
  result.points_size_threshold = points_size_threshold_;
  result.update_size_threshold = update_size_threshold_;
  result.max_points_num = max_points_num_;
  result.max_layer = max_layer_;
  result.new_points = new_points_;
  result.init_octo = init_octo_;
  result.update_enable = update_enable_;
  result.p4b_retain_support = p4b_retain_support_;
  return result;
}

std::unique_ptr<VoxelOctoTree> VoxelOctoTree::restoreSnapshot(
    const VoxelOctoTreeSnapshot &snapshot)
{
  std::unique_ptr<VoxelOctoTree> result(new VoxelOctoTree(
      snapshot.max_layer, snapshot.layer, snapshot.points_size_threshold,
      snapshot.max_points_num, snapshot.planer_threshold));
  result->temp_points_ = snapshot.temp_points;
  *result->plane_ptr_ = snapshot.plane;
  result->octo_state_ = snapshot.octo_state;
  for (int i = 0; i < 3; ++i)
    result->voxel_center_[i] = snapshot.voxel_center[i];
  result->layer_init_num_ = snapshot.layer_init_num;
  result->quater_length_ = snapshot.quarter_length;
  result->update_size_threshold_ = snapshot.update_size_threshold;
  result->new_points_ = snapshot.new_points;
  result->init_octo_ = snapshot.init_octo;
  result->update_enable_ = snapshot.update_enable;
  result->p4b_retain_support_ = snapshot.p4b_retain_support;
  for (int i = 0; i < 8; ++i)
    if (snapshot.leaves[i])
      result->leaves_[i] = restoreSnapshot(*snapshot.leaves[i]).release();
  return result;
}

void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
  const Eigen::Vector3d previous_center = plane->center_;
  const Eigen::Vector3d previous_normal = plane->normal_;
  const bool had_plane = plane->is_plane_;
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
  plane->covariance_ = Eigen::Matrix3d::Zero();
  plane->center_ = Eigen::Vector3d::Zero();
  plane->normal_ = Eigen::Vector3d::Zero();
  plane->points_size_ = points.size();
  plane->radius_ = 0;
  plane->p4_source_frame_ids_.clear();
  plane->p4_source_timestamps_s_.clear();
  plane->p4_source_origins_w_.clear();
  plane->p4_source_frame_ids_.reserve(points.size());
  plane->p4_source_timestamps_s_.reserve(points.size());
  plane->p4_source_origins_w_.reserve(points.size());
  plane->p4b_support_points_.clear();
  for (auto pv : points)
  {
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
    plane->center_ += pv.point_w;
    if (pv.source_frame_id >= 0)
    {
      plane->p4_source_frame_ids_.push_back(pv.source_frame_id);
      plane->p4_source_timestamps_s_.push_back(pv.source_timestamp_s);
      plane->p4_source_origins_w_.push_back(pv.source_origin_w);
    }
  }
  plane->center_ = plane->center_ / plane->points_size_;
  plane->covariance_ = plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
  Eigen::Matrix3cd evecs = es.eigenvectors();
  Eigen::Vector3cd evals = es.eigenvalues();
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();
  Eigen::Matrix3f::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax;
  Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
  Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
  Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
  Eigen::Matrix3d J_Q;
  J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_;
  // && evalsReal(evalsMid) > 0.05
  //&& evalsReal(evalsMid) > 0.01
  if (evalsReal(evalsMin) < planer_threshold_)
  {
    for (int i = 0; i < points.size(); i++)
    {
      Eigen::Matrix<double, 6, 3> J;
      Eigen::Matrix3d F;
      for (int m = 0; m < 3; m++)
      {
        if (m != (int)evalsMin)
        {
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].point_w - plane->center_).transpose() / ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
              (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
          F.row(m) = F_m;
        }
        else
        {
          Eigen::Matrix<double, 1, 3> F_m;
          F_m << 0, 0, 0;
          F.row(m) = F_m;
        }
      }
      J.block<3, 3>(0, 0) = evecs.real() * F;
      J.block<3, 3>(3, 0) = J_Q;
      plane->plane_var_ += J * points[i].var * J.transpose();
    }

    plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
    plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
    plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
    plane->min_eigen_value_ = evalsReal(evalsMin);
    plane->mid_eigen_value_ = evalsReal(evalsMid);
    plane->max_eigen_value_ = evalsReal(evalsMax);
    plane->radius_ = sqrt(evalsReal(evalsMax));
    plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) + plane->normal_(2) * plane->center_(2));
    plane->is_plane_ = true;
    if (p4b_retain_support_) plane->p4b_support_points_.assign(points);
    plane->is_update_ = true;
    ++plane->p4_update_count_;
    if (had_plane)
    {
      plane->p4_last_center_shift_m_ =
          (plane->center_ - previous_center).norm();
      const double cosine = std::max(
          -1.0, std::min(1.0, std::abs(previous_normal.normalized().dot(
                                     plane->normal_.normalized()))));
      plane->p4_last_normal_change_deg_ =
          std::acos(cosine) * 57.29577951308232;
    }
    if (!plane->is_init_)
    {
      plane->id_ = voxel_plane_id;
      voxel_plane_id++;
      plane->is_init_ = true;
    }
  }
  else
  {
    plane->is_update_ = true;
    plane->is_plane_ = false;
  }
}

void VoxelOctoTree::init_octo_tree()
{
  if (temp_points_.size() > points_size_threshold_)
  {
    init_plane(temp_points_, plane_ptr_);
    if (plane_ptr_->is_plane_ == true)
    {
      octo_state_ = 0;
      // new added
      if (temp_points_.size() > max_points_num_)
      {
        update_enable_ = false;
        std::vector<pointWithVar>().swap(temp_points_);
        new_points_ = 0;
      }
    }
    else
    {
      octo_state_ = 1;
      cut_octo_tree();
    }
    init_octo_ = true;
    new_points_ = 0;
  }
}

void VoxelOctoTree::cut_octo_tree()
{
  if (layer_ >= max_layer_)
  {
    octo_state_ = 0;
    return;
  }
  for (size_t i = 0; i < temp_points_.size(); i++)
  {
    int xyz[3] = {0, 0, 0};
    if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
    if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
    if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] == nullptr)
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
      leaves_[leafnum]->p4b_retain_support_ = p4b_retain_support_;
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
    }
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    leaves_[leafnum]->new_points_++;
  }
  for (uint i = 0; i < 8; i++)
  {
    if (leaves_[i] != nullptr)
    {
      if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_)
      {
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
        if (leaves_[i]->plane_ptr_->is_plane_)
        {
          leaves_[i]->octo_state_ = 0;
          // new added
          if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_)
          {
            leaves_[i]->update_enable_ = false;
            std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
            new_points_ = 0;
          }
        }
        else
        {
          leaves_[i]->octo_state_ = 1;
          leaves_[i]->cut_octo_tree();
        }
        leaves_[i]->init_octo_ = true;
        leaves_[i]->new_points_ = 0;
      }
    }
  }
}

void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv)
{
  if (!init_octo_)
  {
    new_points_++;
    temp_points_.push_back(pv);
    if (temp_points_.size() > points_size_threshold_) { init_octo_tree(); }
  }
  else
  {
    if (plane_ptr_->is_plane_)
    {
      if (update_enable_)
      {
        new_points_++;
        temp_points_.push_back(pv);
        if (new_points_ > update_size_threshold_)
        {
          init_plane(temp_points_, plane_ptr_);
          new_points_ = 0;
        }
        if (temp_points_.size() >= max_points_num_)
        {
          update_enable_ = false;
          std::vector<pointWithVar>().swap(temp_points_);
          new_points_ = 0;
        }
      }
    }
    else
    {
      if (layer_ < max_layer_)
      {
        int xyz[3] = {0, 0, 0};
        if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
        if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
        if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) { leaves_[leafnum]->UpdateOctoTree(pv); }
        else
        {
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
          leaves_[leafnum]->p4b_retain_support_ = p4b_retain_support_;
          leaves_[leafnum]->layer_init_num_ = layer_init_num_;
          leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
          leaves_[leafnum]->quater_length_ = quater_length_ / 2;
          leaves_[leafnum]->UpdateOctoTree(pv);
        }
      }
      else
      {
        if (update_enable_)
        {
          new_points_++;
          temp_points_.push_back(pv);
          if (new_points_ > update_size_threshold_)
          {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
          }
          if (temp_points_.size() > max_points_num_)
          {
            update_enable_ = false;
            std::vector<pointWithVar>().swap(temp_points_);
            new_points_ = 0;
          }
        }
      }
    }
  }
}

VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw)
{
  if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

  int xyz[3] = {0, 0, 0};
  xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
  xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
  xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
  int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

  // printf("leafnum: %d. \n", leafnum);

  return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv)
{
  if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) || (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_)))
  {
    new_points_++;
    temp_points_.push_back(pv);
    return this;
  }

  if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_))
  {
    int xyz[3] = {0, 0, 0};
    xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] != nullptr) { return leaves_[leafnum]->Insert(pv); }
    else
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
      leaves_[leafnum]->p4b_retain_support_ = p4b_retain_support_;
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      return leaves_[leafnum]->Insert(pv);
    }
  }
  return nullptr;
}

void VoxelMapManager::StateEstimation(StatesGroup &state_propagat, double timestamp,
                                      std::ostream *iteration_log)
{
  if (config_setting_.p5_weak_axis_multistart_enable)
  {
    StateEstimationP5Prototype(state_propagat, timestamp, iteration_log);
    return;
  }
  StateEstimationInternal(state_propagat, timestamp, iteration_log, nullptr,
                          false);
}

void VoxelMapManager::StateEstimationInternal(
    StatesGroup &state_propagat, double timestamp, std::ostream *iteration_log,
    const StatesGroup *initial_guess, bool shadow_mode)
{
  if (!shadow_mode)
  {
    p4b_refitted_plane_cache_.clear();
    ++current_frame_id_;
  }
  p4_current_timestamp_s_ = timestamp;
  if (!shadow_mode && config_setting_.p4_frontend_diagnostics_enable)
  {
    if (!std::isfinite(p4_first_timestamp_s_)) p4_first_timestamp_s_ = timestamp;
    p4InitializeOutput();
    p4_previous_associations_.clear();
    if (p4_deskew_csv_.is_open() && p4_deskew_diagnostics_.valid &&
        p4InAnalysisWindow(timestamp))
    {
      const auto &d = p4_deskew_diagnostics_;
      p4_deskew_csv_ << std::setprecision(17)
          << timestamp << ',' << p4RelativeTime(timestamp) << ','
          << current_frame_id_ << ',' << d.point_count << ',' << d.imu_count
          << ',' << d.scan_begin_s << ',' << d.scan_end_s << ','
          << d.point_offset_min_s << ',' << d.point_offset_max_s << ','
          << d.point_time_monotonic << ',' << d.imu_begin_s << ','
          << d.imu_end_s << ',' << d.propagation_begin_s << ','
          << d.propagation_end_s << ',' << d.imu_covers_propagation << ','
          << d.seed_velocity.transpose().format(Eigen::IOFormat(
                 Eigen::FullPrecision, Eigen::DontAlignCols, ",", ","))
          << ',' << d.end_velocity.transpose().format(Eigen::IOFormat(
                 Eigen::FullPrecision, Eigen::DontAlignCols, ",", ","))
          << ',' << d.gyro_bias.transpose().format(Eigen::IOFormat(
                 Eigen::FullPrecision, Eigen::DontAlignCols, ",", ","))
          << ',' << d.accel_bias.transpose().format(Eigen::IOFormat(
                 Eigen::FullPrecision, Eigen::DontAlignCols, ",", ","))
          << ',' << d.gravity.transpose().format(Eigen::IOFormat(
                 Eigen::FullPrecision, Eigen::DontAlignCols, ",", ","))
          << ',' << d.fixed_velocity_point_delta_mean_m << ','
          << d.fixed_velocity_point_delta_max_m << '\n';
    }
  }
  last_lio_diagnostics_ = LioUpdateDiagnostics();
  const StatesGroup committed_state = state_propagat;
  state_ = initial_guess ? *initial_guess : committed_state;
  state_.cov = committed_state.cov;
  last_lio_diagnostics_.predicted_state = state_propagat;
  last_lio_diagnostics_.covariance_trace_before = committed_state.cov.trace();
  last_lio_diagnostics_.input_feature_count =
      feats_undistort_ ? static_cast<int>(feats_undistort_->size()) : 0;
  last_lio_diagnostics_.downsampled_feature_count = feats_down_size_;

  // These diagnostics influence later frame selection/map guards, so stage
  // their mutable hysteresis state with the measurement transaction too.
  const bool committed_lidar_degenerated = lidar_degenerated_;
  const double committed_lidar_constraint_ratio = lidar_constraint_ratio_;
  const int committed_degeneracy_bad_frames = degeneracy_bad_frame_count_;
  const int committed_degeneracy_good_frames = degeneracy_good_frame_count_;
  const int committed_direction_conflict_frames = direction_conflict_frame_count_;
  const int committed_direction_clear_frames = direction_clear_frame_count_;
  const bool committed_direction_guard_active = direction_guard_active_;

  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  body_cov_list_.clear();
  body_cov_list_.reserve(feats_down_size_);

  // build_residual_time = 0.0;
  // ekf_time = 0.0;
  // double t0 = omp_get_wtime();

  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    if (point_this[2] == 0) { point_this[2] = 0.001; }
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    body_cov_list_.push_back(var);
    point_this = extR_ * point_this + extT_;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }

  vector<pointWithVar>().swap(pv_list_);
  pv_list_.resize(feats_down_size_);
  if (!shadow_mode && config_setting_.p4_frontend_diagnostics_enable &&
      p4InDetailWindow(timestamp) &&
      current_frame_id_ % config_setting_.p4_detail_frame_stride == 0 &&
      p4_raw_cloud_ && feats_undistort_ &&
      p4_raw_cloud_->size() == feats_undistort_->size() &&
      !feats_undistort_->empty())
  {
    pcl::KdTreeFLANN<PointType> nearest_deskewed;
    nearest_deskewed.setInputCloud(feats_undistort_);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (std::size_t i = 0; i < feats_down_body_->size(); ++i)
    {
      if (nearest_deskewed.nearestKSearch(
              feats_down_body_->points[i], 1, indices, distances) == 1)
      {
        const PointType &raw = p4_raw_cloud_->points[indices[0]];
        pv_list_[i].point_raw << raw.x, raw.y, raw.z;
      }
    }
  }

  int rematch_num = 0;
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
  const int min_icp_iterations = std::max(1, std::min(config_setting_.icp_min_iterations, config_setting_.max_iterations_));
  const double residual_ratio_thresh = std::max(0.0, config_setting_.icp_early_stop_residual_ratio);
  const double max_rot_step_deg = std::max(0.1, config_setting_.icp_max_rot_step_deg);
  const double max_trans_step_m = std::max(0.01, config_setting_.icp_max_trans_step_m);
  double last_avg_residual = std::numeric_limits<double>::infinity();
  bool numerical_failure = false;
  bool posterior_ready = false;
  bool candidate_generated = false;
  bool candidate_converged = false;
  bool reached_iteration_limit = false;
  int iteration_count = 0;
  double cost_before = std::numeric_limits<double>::quiet_NaN();
  bool frame_observability_initialized = false;
  fast_livo::LioObservabilityMetrics frame_observability;
  bool motion_inputs_ready = false;
  MD(DIM_STATE, DIM_STATE) motion_prior_covariance =
      MD(DIM_STATE, DIM_STATE)::Zero();
  MD(DIM_STATE, DIM_STATE) motion_posterior_covariance =
      MD(DIM_STATE, DIM_STATE)::Zero();
  Eigen::Matrix<double, DIM_STATE, 6> motion_normal_rhs_gain =
      Eigen::Matrix<double, DIM_STATE, 6>::Zero();
  Eigen::Matrix<double, DIM_STATE, 6> motion_update_operator =
      Eigen::Matrix<double, DIM_STATE, 6>::Zero();
  fast_livo::Matrix6d motion_pose_information = fast_livo::Matrix6d::Zero();
  Eigen::Matrix<double, 6, 1> motion_pose_rhs =
      Eigen::Matrix<double, 6, 1>::Zero();
  VD(DIM_STATE) motion_iteration_prior_offset = VD(DIM_STATE)::Zero();
  double motion_residual_weighted_energy = std::numeric_limits<double>::quiet_NaN();
  int motion_residual_degrees_of_freedom = 0;
  double motion_final_step_scale = 1.0;
  VD(DIM_STATE) motion_accumulated_innovation = VD(DIM_STATE)::Zero();
  VD(DIM_STATE) motion_accumulated_relinearization = VD(DIM_STATE)::Zero();
  int motion_analyzed_iteration_count = 0;
  int motion_nis_iteration_count = 0;
  double motion_iteration_nis_sum = 0.0;
  double motion_max_iteration_nis = -std::numeric_limits<double>::infinity();
  double motion_max_rotation_gain = 0.0;
  double motion_max_position_gain = 0.0;
  double motion_max_velocity_gain = 0.0;
  double motion_max_velocity_innovation = 0.0;

  for (int iterCount = 0; iterCount < config_setting_.max_iterations_; iterCount++)
  {
    double total_residual = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
    M3D rot_var = state_.cov.block<3, 3>(0, 0);
    M3D t_var = state_.cov.block<3, 3>(3, 3);
    for (size_t i = 0; i < feats_down_body_->size(); i++)
    {
      pointWithVar &pv = pv_list_[i];
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.source_point_index = static_cast<int>(i);
      if (config_setting_.p4_frontend_diagnostics_enable ||
          config_setting_.p4b_snapshot_enable)
      {
        pv.source_frame_id = current_frame_id_;
        pv.source_timestamp_s = timestamp;
        pv.source_origin_w = state_.pos_end;
      }
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;

      M3D cov = body_cov_list_[i];
      M3D point_crossmat = cross_mat_list_[i];
      cov = state_.rot_end * cov * state_.rot_end.transpose() + (-point_crossmat) * rot_var * (-point_crossmat.transpose()) + t_var;
      pv.var = cov;
      pv.body_var = body_cov_list_[i];
    }
    ptpl_list_.clear();

    // double t1 = omp_get_wtime();

    BuildResidualListOMP(pv_list_, ptpl_list_);

    // build_residual_time += omp_get_wtime() - t1;

    for (int i = 0; i < ptpl_list_.size(); i++)
    {
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
    effct_feat_num_ = ptpl_list_.size();

    if (effct_feat_num_ == 0)
    {
      if (!shadow_mode)
        std::cout << "[ LIO ] No effective point-to-plane constraints, skip ICP update in this scan." << std::endl;
      if (config_setting_.observability_diagnostics_enable && !frame_observability_initialized)
      {
        updateLidarDegeneracyHysteresis(true);
        updateDirectionGuardHysteresis(false);
        last_lio_diagnostics_.raw_is_degenerate = true;
        last_lio_diagnostics_.is_degenerate = lidar_degenerated_;
        last_lio_diagnostics_.is_severely_degenerate = lidar_degenerated_;
      }
      break;
    }

    const double avg_residual = total_residual / static_cast<double>(effct_feat_num_);
    iteration_count = iterCount + 1;
    last_lio_diagnostics_.effective_feature_count = effct_feat_num_;
    last_lio_diagnostics_.valid_plane_count = effct_feat_num_;
    last_lio_diagnostics_.inlier_ratio = feats_down_size_ > 0 ?
        static_cast<double>(effct_feat_num_) / static_cast<double>(feats_down_size_) : 0.0;
    last_lio_diagnostics_.average_point_plane_residual = avg_residual;
    std::vector<double> absolute_residuals;
    absolute_residuals.reserve(ptpl_list_.size());
    for (const auto &ptpl : ptpl_list_)
      absolute_residuals.push_back(std::fabs(static_cast<double>(ptpl.dis_to_plane_)));
    const ScalarDiagnostics residual_summary = summarizeFiniteValues(absolute_residuals);
    last_lio_diagnostics_.median_abs_point_plane_residual = residual_summary.median;
    last_lio_diagnostics_.p90_abs_point_plane_residual = residual_summary.p90;
    last_lio_diagnostics_.p95_abs_point_plane_residual = residual_summary.p95;
    last_lio_diagnostics_.point_plane_residual_rmse = residual_summary.rmse;
    last_lio_diagnostics_.max_abs_point_plane_residual = residual_summary.maximum;
    if (!shadow_mode)
      cout << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_
           << " effective feature num: " << effct_feat_num_ << " average residual: " << avg_residual << endl;

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
    MatrixXd Hsub(effct_feat_num_, 6);
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_);
    VectorXd R_inv(effct_feat_num_);
    VectorXd meas_vec(effct_feat_num_);
    std::vector<double> measurement_variances;
    measurement_variances.reserve(effct_feat_num_);
    meas_vec.setZero();
    for (int i = 0; i < effct_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/

      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end;
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;

      M3D var;
      // V3D normal_b = state_.rot_end.inverse() * ptpl_list_[i].normal_;
      // V3D point_b = ptpl_list_[i].point_b_;
      // double cos_theta = fabs(normal_b.dot(point_b) / point_b.norm());
      // ptpl_list_[i].body_cov_ = ptpl_list_[i].body_cov_ * (1.0 / cos_theta) * (1.0 / cos_theta);

      // point_w cov
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) + (-point_crossmat) * state_propagat.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose();

      // point_w cov (another_version)
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) - point_crossmat * state_propagat.cov.block<3, 3>(0, 0) * point_crossmat;

      // point_body cov
      var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose();

      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();

      R_inv(i) = 1.0 / (0.001 + sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);
      measurement_variances.push_back(1.0 / R_inv(i));
      // R_inv(i) = 1.0 / (sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      /*** calculate the Measuremnt Jacobian matrix H ***/
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
    }
    const ScalarDiagnostics variance_summary = summarizeFiniteValues(measurement_variances);
    last_lio_diagnostics_.measurement_variance_mean = variance_summary.mean;
    last_lio_diagnostics_.measurement_variance_median = variance_summary.median;
    last_lio_diagnostics_.measurement_variance_p90 = variance_summary.p90;
    last_lio_diagnostics_.measurement_variance_p95 = variance_summary.p95;
    last_lio_diagnostics_.measurement_variance_min = variance_summary.minimum;
    last_lio_diagnostics_.measurement_variance_max = variance_summary.maximum;
    EKF_stop_flg = false;
    flg_EKF_converged = false;
    /*** Iterative Kalman Filter Update ***/
    const Eigen::Matrix<double, 6, 1> raw_pose_rhs = Hsub_T_R_inv * meas_vec;
    const fast_livo::Matrix6d raw_pose_information = Hsub_T_R_inv * Hsub;
    const double residual_weighted_energy =
        (R_inv.array() * meas_vec.array().square()).sum();
    if (!shadow_mode && config_setting_.p4_frontend_diagnostics_enable &&
        p4InAnalysisWindow(timestamp))
    {
      const fast_livo::LioObservabilityMetrics p4_observability =
          fast_livo::analyzeLioPoseInformation(
              raw_pose_information,
              config_setting_.degeneracy_rotation_regularization,
              config_setting_.use_conditional_translation_information);
      p4RecordIteration(timestamp, iterCount + 1, state_, Hsub, R_inv,
                        p4_observability);
    }
    if (!raw_pose_information.allFinite() || !raw_pose_rhs.allFinite() || !state_.cov.allFinite())
    {
      numerical_failure = true;
      ROS_ERROR_THROTTLE(1.0, "[LIO_NUMERIC] Non-finite normal equation or covariance; skipping this scan update.");
      break;
    }

    // Stable baseline: this is the original FAST-LIVO2 ESIKF normal-equation
    // path. Observability diagnostics below only inspect its const inputs.
    H_T_H.block<6, 6>(0, 0) = raw_pose_information;
    MD(DIM_STATE, DIM_STATE) K_1 =
        (H_T_H.block<DIM_STATE, DIM_STATE>(0, 0) +
         state_.cov.block<DIM_STATE, DIM_STATE>(0, 0).inverse()).inverse();
    if (!K_1.allFinite())
    {
      numerical_failure = true;
      ROS_ERROR_THROTTLE(1.0, "[LIO_NUMERIC] Non-finite original ESIKF solve; skipping this scan update.");
      break;
    }
    G.block<DIM_STATE, 6>(0, 0) =
        K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
    const auto vec = state_propagat - state_;
    VD(DIM_STATE) solution =
        K_1.block<DIM_STATE, 6>(0, 0) * raw_pose_rhs + vec.block<DIM_STATE, 1>(0, 0) -
        G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
    if (!solution.allFinite())
    {
      numerical_failure = true;
      ROS_ERROR_THROTTLE(1.0, "[LIO_NUMERIC] Non-finite original ESIKF increment; skipping this scan update.");
      break;
    }
    const VD(DIM_STATE) iteration_innovation =
        K_1.block<DIM_STATE, 6>(0, 0) * raw_pose_rhs;
    const VD(DIM_STATE) iteration_relinearization =
        vec - G.block<DIM_STATE, 6>(0, 0) * vec.head<6>();
    double iteration_nis_per_dof = std::numeric_limits<double>::quiet_NaN();
    const double iteration_explained_energy =
        raw_pose_rhs.dot(K_1.block<6, 6>(0, 0) * raw_pose_rhs);
    const double iteration_nis =
        residual_weighted_energy - iteration_explained_energy;
    const double iteration_nis_tolerance =
        1e-9 * std::max(1.0, residual_weighted_energy);
    if (iteration_nis >= -iteration_nis_tolerance && effct_feat_num_ > 0)
      iteration_nis_per_dof =
          std::max(0.0, iteration_nis) / static_cast<double>(effct_feat_num_);

    if (config_setting_.observability_diagnostics_enable && !frame_observability_initialized)
    {
      frame_observability = fast_livo::analyzeLioPoseInformation(
          raw_pose_information,
          config_setting_.degeneracy_rotation_regularization,
          config_setting_.use_conditional_translation_information);
      frame_observability_initialized = true;
      last_lio_diagnostics_.observability = frame_observability;
      last_lio_diagnostics_.observability_feature_count = effct_feat_num_;
      if (!frame_observability.valid)
        ROS_WARN_THROTTLE(1.0, "[LIO_DEGEN] Observability decomposition invalid; diagnostics marked invalid and original ESIKF update retained.");
      lidar_constraint_ratio_ = frame_observability.valid ?
          frame_observability.translation_eigenvalue_ratio : 0.0;

      const bool raw_degenerate = classifyLidarDegeneracy(frame_observability, effct_feat_num_);
      updateLidarDegeneracyHysteresis(raw_degenerate);

      last_lio_diagnostics_.raw_is_degenerate = raw_degenerate;
      last_lio_diagnostics_.is_degenerate = lidar_degenerated_;
      last_lio_diagnostics_.is_severely_degenerate =
          last_lio_diagnostics_.is_degenerate &&
          (!frame_observability.valid ||
           effct_feat_num_ < config_setting_.degeneracy_min_effective_features ||
           frame_observability.translation_eigenvalue_ratio <
               config_setting_.map_guard_severe_translation_eigenvalue_ratio);

      bool direction_conflict = false;
      if (config_setting_.direction_guard_mode != "off" && frame_observability.valid)
      {
        const double predicted_speed = state_propagat.vel_end.norm();
        last_lio_diagnostics_.predicted_speed_mps = predicted_speed;
        if (predicted_speed >= config_setting_.direction_guard_min_predicted_speed_mps)
        {
          const V3D velocity_direction = state_propagat.vel_end / predicted_speed;
          const double velocity_weak_cos =
              std::fabs(velocity_direction.dot(frame_observability.weak_translation_direction_world));
          const V3D raw_candidate_position = state_.pos_end + solution.block<3, 1>(3, 0);
          const V3D raw_position_correction = raw_candidate_position - state_propagat.pos_end;
          const V3D raw_candidate_velocity = state_.vel_end + solution.block<3, 1>(7, 0);
          const V3D raw_velocity_correction = raw_candidate_velocity - state_propagat.vel_end;
          last_lio_diagnostics_.velocity_weak_direction_cos = velocity_weak_cos;
          last_lio_diagnostics_.raw_position_correction = raw_position_correction;
          last_lio_diagnostics_.raw_velocity_correction = raw_velocity_correction;
          const double correction_along_velocity = raw_position_correction.dot(velocity_direction);
          const double velocity_correction_along_velocity =
              raw_velocity_correction.dot(velocity_direction);
          direction_conflict = lidar_degenerated_ &&
              velocity_weak_cos >= config_setting_.direction_guard_min_velocity_weak_direction_cos &&
              (correction_along_velocity < -config_setting_.direction_guard_max_opposite_correction_m ||
               velocity_correction_along_velocity <
                   -config_setting_.direction_guard_max_opposite_velocity_correction_mps);
        }
      }
      updateDirectionGuardHysteresis(direction_conflict);
      last_lio_diagnostics_.direction_conflict = direction_conflict;
      last_lio_diagnostics_.direction_conflict_consecutive_frames =
          direction_conflict_frame_count_;
      // Enforce mode intentionally falls back to diagnostic in loadVoxelConfig().
      last_lio_diagnostics_.direction_guard_triggered = false;
      last_lio_diagnostics_.state_intervention_applied = false;
    }
    int minRow, minCol;
    const StatesGroup iteration_state_before = state_;
    Eigen::Matrix<double, 9, 1> diagnostic_raw_pose_velocity;
    if (iteration_log)
    {
      diagnostic_raw_pose_velocity.head<6>() = solution.head<6>();
      diagnostic_raw_pose_velocity.tail<3>() = solution.segment<3>(7);
    }

    auto rot_add = solution.block<3, 1>(0, 0);
    auto t_add = solution.block<3, 1>(3, 0);
    const double rot_step_deg = rot_add.norm() * 57.3;
    const double trans_step_m = t_add.norm();
    double step_scale = 1.0;
    if (rot_step_deg > max_rot_step_deg) { step_scale = std::min(step_scale, max_rot_step_deg / std::max(rot_step_deg, 1e-6)); }
    if (trans_step_m > max_trans_step_m) { step_scale = std::min(step_scale, max_trans_step_m / std::max(trans_step_m, 1e-6)); }
    if (step_scale < 1.0)
    {
      solution *= step_scale;
      rot_add = solution.block<3, 1>(0, 0);
      t_add = solution.block<3, 1>(3, 0);
    }

    motion_accumulated_innovation += iteration_innovation * step_scale;
    motion_accumulated_relinearization += iteration_relinearization * step_scale;
    ++motion_analyzed_iteration_count;
    if (std::isfinite(iteration_nis_per_dof))
    {
      ++motion_nis_iteration_count;
      motion_iteration_nis_sum += iteration_nis_per_dof;
      motion_max_iteration_nis =
          std::max(motion_max_iteration_nis, iteration_nis_per_dof);
    }
    motion_max_rotation_gain = std::max(
        motion_max_rotation_gain, G.block<3, 6>(0, 0).norm());
    motion_max_position_gain = std::max(
        motion_max_position_gain, G.block<3, 6>(3, 0).norm());
    motion_max_velocity_gain = std::max(
        motion_max_velocity_gain, G.block<3, 6>(7, 0).norm());
    motion_max_velocity_innovation = std::max(
        motion_max_velocity_innovation,
        (iteration_innovation * step_scale).segment<3>(7).norm());

    state_ += solution;
    candidate_generated = true;

    std::vector<LioUpdateDiagnostics::DirectionalShadow> current_iteration_shadows;
    if (config_setting_.directional_shadow_enable &&
        !config_setting_.directional_shadow_relative_thresholds.empty())
    {
      const fast_livo::LioObservabilityMetrics shadow_observability =
          fast_livo::analyzeLioPoseInformation(
              raw_pose_information,
              config_setting_.degeneracy_rotation_regularization,
              config_setting_.use_conditional_translation_information);
      const MD(DIM_STATE, DIM_STATE) prior_covariance = iteration_state_before.cov;
      const MD(DIM_STATE, DIM_STATE) prior_information = prior_covariance.inverse();
      current_iteration_shadows.reserve(
          config_setting_.directional_shadow_relative_thresholds.size());

      for (const double threshold : config_setting_.directional_shadow_relative_thresholds)
      {
        const double shadow_begin = omp_get_wtime();
        LioUpdateDiagnostics::DirectionalShadow diagnostic;
        diagnostic.iteration_index = iterCount;
        diagnostic.iteration_count = iterCount + 1;
        diagnostic.effective_feature_count = effct_feat_num_;
        diagnostic.relative_threshold = threshold;
        diagnostic.residual_mean = avg_residual;
        diagnostic.residual_median = residual_summary.median;
        diagnostic.residual_p90 = residual_summary.p90;
        diagnostic.residual_rmse = residual_summary.rmse;
        diagnostic.measurement_variance_mean = variance_summary.mean;
        diagnostic.measurement_variance_median = variance_summary.median;
        diagnostic.observability = shadow_observability;
        diagnostic.rotation_weights = fast_livo::lioRelativeEigenDirectionWeights(
            shadow_observability.rotation_eigenvalues, threshold);
        diagnostic.translation_weights = fast_livo::lioRelativeEigenDirectionWeights(
            shadow_observability.translation_eigenvalues, threshold);

        auto count_weights = [&](const Eigen::Vector3d &weights) {
          for (int i = 0; i < 3; ++i)
          {
            if (weights[i] >= 1.0 - 1e-12) continue;
            ++diagnostic.affected_direction_count;
            if (weights[i] <= 1e-12)
              ++diagnostic.full_suppression_count;
            else
              ++diagnostic.partial_suppression_count;
          }
        };
        count_weights(diagnostic.rotation_weights);
        count_weights(diagnostic.translation_weights);

        const fast_livo::LioNormalEquation shadow_normal =
            fast_livo::applyLioDirectionalProjectors(
                raw_pose_information, raw_pose_rhs, shadow_observability,
                diagnostic.rotation_weights, diagnostic.translation_weights);
        diagnostic.information_trace_raw = raw_pose_information.trace();
        diagnostic.information_trace_shadow = shadow_normal.information.trace();
        diagnostic.information_trace_retained_ratio =
            std::fabs(diagnostic.information_trace_raw) > 1e-12
                ? diagnostic.information_trace_shadow / diagnostic.information_trace_raw
                : 1.0;
        diagnostic.rotation_information_trace_raw =
            raw_pose_information.block<3, 3>(0, 0).trace();
        diagnostic.rotation_information_trace_shadow =
            shadow_normal.information.block<3, 3>(0, 0).trace();
        diagnostic.translation_information_trace_raw =
            raw_pose_information.block<3, 3>(3, 3).trace();
        diagnostic.translation_information_trace_shadow =
            shadow_normal.information.block<3, 3>(3, 3).trace();

        MD(DIM_STATE, DIM_STATE) shadow_information_full =
            MD(DIM_STATE, DIM_STATE)::Zero();
        shadow_information_full.block<6, 6>(0, 0) = shadow_normal.information;
        const MD(DIM_STATE, DIM_STATE) shadow_K =
            (shadow_information_full + prior_information).inverse();
        MD(DIM_STATE, DIM_STATE) shadow_G = MD(DIM_STATE, DIM_STATE)::Zero();
        shadow_G.block<DIM_STATE, 6>(0, 0) =
            shadow_K.block<DIM_STATE, 6>(0, 0) * shadow_normal.information;
        VD(DIM_STATE) shadow_solution =
            shadow_K.block<DIM_STATE, 6>(0, 0) * shadow_normal.rhs + vec -
            shadow_G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);

        if (shadow_observability.valid && shadow_normal.information.allFinite() &&
            shadow_normal.rhs.allFinite() && shadow_K.allFinite() &&
            shadow_solution.allFinite())
        {
          const double shadow_rot_step_deg =
              shadow_solution.block<3, 1>(0, 0).norm() * 57.3;
          const double shadow_trans_step_m =
              shadow_solution.block<3, 1>(3, 0).norm();
          double shadow_step_scale = 1.0;
          if (shadow_rot_step_deg > max_rot_step_deg)
            shadow_step_scale = std::min(
                shadow_step_scale,
                max_rot_step_deg / std::max(shadow_rot_step_deg, 1e-6));
          if (shadow_trans_step_m > max_trans_step_m)
            shadow_step_scale = std::min(
                shadow_step_scale,
                max_trans_step_m / std::max(shadow_trans_step_m, 1e-6));
          shadow_solution *= shadow_step_scale;

          StatesGroup shadow_candidate = iteration_state_before;
          shadow_candidate += shadow_solution;
          diagnostic.raw_delta_position = state_.pos_end - state_propagat.pos_end;
          diagnostic.shadow_delta_position =
              shadow_candidate.pos_end - state_propagat.pos_end;
          diagnostic.removed_delta_position =
              diagnostic.raw_delta_position - diagnostic.shadow_delta_position;
          diagnostic.raw_delta_position_norm = diagnostic.raw_delta_position.norm();
          diagnostic.shadow_delta_position_norm =
              diagnostic.shadow_delta_position.norm();
          diagnostic.removed_delta_position_norm =
              diagnostic.removed_delta_position.norm();
          diagnostic.raw_delta_rpy_deg = relativeRpyDegrees(state_propagat, state_);
          diagnostic.shadow_delta_rpy_deg =
              relativeRpyDegrees(state_propagat, shadow_candidate);
          diagnostic.removed_delta_rpy_deg =
              relativeRpyDegrees(shadow_candidate, state_);
          constexpr double kRadToDeg = 57.29577951308232;
          const Eigen::Vector3d raw_rotation_vector =
              Log((state_propagat.rot_end.transpose() * state_.rot_end).eval());
          const Eigen::Vector3d shadow_rotation_vector =
              Log((state_propagat.rot_end.transpose() * shadow_candidate.rot_end).eval());
          diagnostic.raw_delta_rotation_deg = raw_rotation_vector.norm() * kRadToDeg;
          diagnostic.shadow_delta_rotation_deg =
              shadow_rotation_vector.norm() * kRadToDeg;
          diagnostic.removed_delta_rotation_deg =
              Log((shadow_candidate.rot_end.transpose() * state_.rot_end).eval()).norm() *
              kRadToDeg;
          diagnostic.raw_weak_translation_projection =
              diagnostic.raw_delta_position.dot(
                  shadow_observability.weak_translation_direction_world);
          diagnostic.shadow_weak_translation_projection =
              diagnostic.shadow_delta_position.dot(
                  shadow_observability.weak_translation_direction_world);
          diagnostic.raw_weak_rotation_projection_deg =
              raw_rotation_vector.dot(
                  shadow_observability.weak_rotation_direction_body) * kRadToDeg;
          diagnostic.shadow_weak_rotation_projection_deg =
              shadow_rotation_vector.dot(
                  shadow_observability.weak_rotation_direction_body) * kRadToDeg;

          const MD(DIM_STATE, DIM_STATE) raw_posterior =
              (I_STATE - G) * prior_covariance;
          const MD(DIM_STATE, DIM_STATE) shadow_posterior =
              (I_STATE - shadow_G) * prior_covariance;
          const Eigen::Matrix<double, 6, 6> raw_pose_covariance =
              0.5 * (raw_posterior.block<6, 6>(0, 0) +
                     raw_posterior.block<6, 6>(0, 0).transpose());
          const Eigen::Matrix<double, 6, 6> shadow_pose_covariance =
              0.5 * (shadow_posterior.block<6, 6>(0, 0) +
                     shadow_posterior.block<6, 6>(0, 0).transpose());
          diagnostic.raw_posterior_pose_cov_trace = raw_pose_covariance.trace();
          diagnostic.shadow_posterior_pose_cov_trace = shadow_pose_covariance.trace();
          diagnostic.raw_posterior_pose_cov_diagonal = raw_pose_covariance.diagonal();
          diagnostic.shadow_posterior_pose_cov_diagonal =
              shadow_pose_covariance.diagonal();
          diagnostic.raw_posterior_rotation_cov_eigenvalues =
              symmetricEigenvalues(raw_pose_covariance.block<3, 3>(0, 0));
          diagnostic.shadow_posterior_rotation_cov_eigenvalues =
              symmetricEigenvalues(shadow_pose_covariance.block<3, 3>(0, 0));
          diagnostic.raw_posterior_translation_cov_eigenvalues =
              symmetricEigenvalues(raw_pose_covariance.block<3, 3>(3, 3));
          diagnostic.shadow_posterior_translation_cov_eigenvalues =
              symmetricEigenvalues(shadow_pose_covariance.block<3, 3>(3, 3));
          diagnostic.valid = raw_posterior.allFinite() && shadow_posterior.allFinite();
        }
        diagnostic.solve_time_ms = (omp_get_wtime() - shadow_begin) * 1000.0;
        current_iteration_shadows.push_back(diagnostic);
      }
    }
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }
    candidate_converged = flg_EKF_converged;
    reached_iteration_limit = iterCount == config_setting_.max_iterations_ - 1;
    V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0);

    const bool has_last_residual = std::isfinite(last_avg_residual);
    const double residual_improve_ratio =
        has_last_residual ? (last_avg_residual - avg_residual) / std::max(last_avg_residual, 1e-6) : 1.0;
    last_avg_residual = avg_residual;

    const bool enough_constraints = effct_feat_num_ >= config_setting_.degeneracy_min_effective_features;
    const bool reached_min_iters = (iterCount + 1) >= min_icp_iterations;
    const bool tiny_residual_improvement = has_last_residual && (residual_improve_ratio >= -0.02) &&
                                           (residual_improve_ratio <= residual_ratio_thresh);

    /*** Rematch Judgement ***/

    if ((flg_EKF_converged && enough_constraints) || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2))))
    {
      rematch_num++;
    }

    if (!EKF_stop_flg && reached_min_iters && enough_constraints && flg_EKF_converged && tiny_residual_improvement)
    {
      rematch_num = std::max(rematch_num, 2);
    }

    /*** Convergence Judgements and Covariance Update ***/
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1)))
    {
      /*** Covariance Update ***/
      // _state.cov = (I_STATE - G) * _state.cov;
      motion_prior_covariance = state_.cov;
      motion_posterior_covariance =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) -
           G.block<DIM_STATE, DIM_STATE>(0, 0)) * motion_prior_covariance;
      motion_normal_rhs_gain = K_1.block<DIM_STATE, 6>(0, 0);
      motion_update_operator = G.block<DIM_STATE, 6>(0, 0);
      motion_pose_information = raw_pose_information;
      motion_pose_rhs = raw_pose_rhs;
      motion_iteration_prior_offset = vec;
      motion_residual_weighted_energy = residual_weighted_energy;
      motion_residual_degrees_of_freedom = effct_feat_num_;
      motion_final_step_scale = step_scale;
      motion_inputs_ready = true;
      last_lio_diagnostics_.lidar_geometry_valid = true;
      last_lio_diagnostics_.lidar_geometry_linearization_state =
          iteration_state_before;
      last_lio_diagnostics_.lidar_geometry_information =
          raw_pose_information;
      last_lio_diagnostics_.lidar_geometry_rhs = raw_pose_rhs;
      last_lio_diagnostics_.lidar_geometry_residual_weighted_energy =
          residual_weighted_energy;
      last_lio_diagnostics_.lidar_geometry_residual_dof = effct_feat_num_;
      state_.cov = motion_posterior_covariance;
      posterior_ready = true;
      cost_before = avg_residual;
      last_lio_diagnostics_.directional_shadows = current_iteration_shadows;

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
      EKF_stop_flg = true;
    }
    if (iteration_log)
    {
      // ponytail: one bounded row per existing ICP iteration, no matrix dumps.
      *iteration_log << std::setprecision(17)
          << "[LIO_ITER] frame_id=" << current_frame_id_
          << " iter=" << iterCount + 1 << " features=" << effct_feat_num_
          << " residual_before_update=" << avg_residual
          << " raw_dr_rad=(" << diagnostic_raw_pose_velocity.head<3>().transpose() << ")"
          << " raw_dp_m=(" << diagnostic_raw_pose_velocity.segment<3>(3).transpose() << ")"
          << " raw_dv_mps=(" << diagnostic_raw_pose_velocity.tail<3>().transpose() << ")"
          << " step_scale=" << step_scale
          << " applied_dr_rad=(" << solution.head<3>().transpose() << ")"
          << " applied_dp_m=(" << solution.segment<3>(3).transpose() << ")"
          << " applied_dv_mps=(" << solution.segment<3>(7).transpose() << ")"
          << " converged=" << flg_EKF_converged << " rematch=" << rematch_num
          << " tiny_residual_improvement=" << tiny_residual_improvement
          << " iteration_limit=" << (iterCount == config_setting_.max_iterations_ - 1)
          << " covariance_updated=" << EKF_stop_flg << '\n';
    }
    if (EKF_stop_flg) break;
  }

  // double t2 = omp_get_wtime();
  // scan_count++;
  // ekf_time = t2 - t0 - build_residual_time;

  // ave_build_residual_time = ave_build_residual_time * (scan_count - 1) / scan_count + build_residual_time / scan_count;
  // ave_ekf_time = ave_ekf_time * (scan_count - 1) / scan_count + ekf_time / scan_count;

  // cout << "[ Mapping ] ekf_time: " << ekf_time << "s, build_residual_time: " << build_residual_time << "s" << endl;
  // cout << "[ Mapping ] ave_ekf_time: " << ave_ekf_time << "s, ave_build_residual_time: " << ave_build_residual_time << "s" << endl;
  StatesGroup candidate_state = state_;
  last_lio_diagnostics_.candidate_state = candidate_state;

  int final_correspondence_count = 0;
  int final_valid_residual_count = 0;
  double final_cost = std::numeric_limits<double>::quiet_NaN();
  double final_max_abs_residual = std::numeric_limits<double>::quiet_NaN();
  auto rebuild_correspondences = [&](const StatesGroup &evaluation_state) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
        new pcl::PointCloud<pcl::PointXYZI>);
    TransformLidar(evaluation_state.rot_end, evaluation_state.pos_end,
                   feats_down_body_, world_lidar);
    const M3D rot_var = evaluation_state.cov.block<3, 3>(0, 0);
    const M3D t_var = evaluation_state.cov.block<3, 3>(3, 3);
    for (size_t i = 0; i < feats_down_body_->size(); ++i)
    {
      pointWithVar &pv = pv_list_[i];
      pv.point_b << feats_down_body_->points[i].x,
          feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y,
          world_lidar->points[i].z;
      const M3D point_crossmat = cross_mat_list_[i];
      pv.var = evaluation_state.rot_end * body_cov_list_[i] *
                   evaluation_state.rot_end.transpose() +
               (-point_crossmat) * rot_var * (-point_crossmat.transpose()) +
               t_var;
      pv.body_var = body_cov_list_[i];
    }
    BuildResidualListOMP(pv_list_, ptpl_list_);
  };

  if (candidate_generated && fast_livo::lioStateFinite(candidate_state))
  {
    // Compare the candidate against the exact correspondences that generated
    // its final increment. Re-matching here could hide a bad step by silently
    // changing the objective being validated.
    final_correspondence_count = static_cast<int>(ptpl_list_.size());
    double residual_sum = 0.0;
    double residual_max = 0.0;
    for (const PointToPlane &residual : ptpl_list_)
    {
      const V3D point_lidar = extR_ * residual.point_b_ + extT_;
      const V3D point_world =
          candidate_state.rot_end * point_lidar + candidate_state.pos_end;
      const double value = std::fabs(
          residual.normal_.dot(point_world) + residual.d_);
      if (!std::isfinite(value)) continue;
      residual_sum += value;
      residual_max = std::max(residual_max, value);
      ++final_valid_residual_count;
    }
    if (final_valid_residual_count > 0)
    {
      final_cost = residual_sum / static_cast<double>(final_valid_residual_count);
      final_max_abs_residual = residual_max;
    }
  }

  const VD(DIM_STATE) state_increment = candidate_state - committed_state;
  fast_livo::LioCandidateMetrics metrics;
  metrics.numerical_failure = numerical_failure;
  metrics.posterior_ready = posterior_ready;
  metrics.converged = candidate_converged;
  metrics.reached_iteration_limit = reached_iteration_limit;
  metrics.correspondence_count = final_correspondence_count;
  metrics.valid_residual_count = final_valid_residual_count;
  metrics.cost_before = cost_before;
  metrics.cost_after = final_cost;
  metrics.maximum_abs_residual = final_max_abs_residual;
  metrics.translation_increment_norm = state_increment.segment<3>(3).norm();
  metrics.rotation_increment_deg = state_increment.head<3>().norm() * 57.29577951308232;
  metrics.velocity_increment_norm = state_increment.segment<3>(7).norm();
  metrics.state_increment_finite = state_increment.allFinite();

  fast_livo::LioValidationLimits limits;
  limits.minimum_correspondences = config_setting_.commit_minimum_correspondences;
  limits.maximum_cost_ratio = config_setting_.commit_maximum_cost_ratio;
  limits.maximum_cost_increase = config_setting_.commit_maximum_cost_increase;
  limits.maximum_residual = config_setting_.commit_maximum_residual;
  limits.covariance_symmetry_relative_tolerance =
      config_setting_.commit_covariance_symmetry_relative_tolerance;
  limits.covariance_psd_relative_tolerance =
      config_setting_.commit_covariance_psd_relative_tolerance;
  // ponytail: total pose bounds derive from the existing per-iteration safety
  // bounds; replace with a motion-model innovation gate if one is introduced.
  limits.maximum_translation_increment =
      config_setting_.icp_max_trans_step_m * config_setting_.max_iterations_ + 1e-9;
  limits.maximum_rotation_increment_deg =
      config_setting_.icp_max_rot_step_deg * config_setting_.max_iterations_ + 1e-9;

  const fast_livo::LioValidationResult validation =
      fast_livo::validateLioCandidate(candidate_state, metrics, limits);
  state_ = fast_livo::committedLioState(
      committed_state, candidate_state, validation);
  last_lio_diagnostics_.valid_update = validation.commit;
  last_lio_diagnostics_.commit = validation.commit;
  last_lio_diagnostics_.convergence_status =
      fast_livo::lioCommitStatusName(validation.status);
  last_lio_diagnostics_.iteration_count = iteration_count;
  last_lio_diagnostics_.converged = candidate_converged;
  last_lio_diagnostics_.reached_iteration_limit = reached_iteration_limit;
  last_lio_diagnostics_.correspondence_count = final_correspondence_count;
  last_lio_diagnostics_.valid_residual_count = final_valid_residual_count;
  last_lio_diagnostics_.cost_before = cost_before;
  last_lio_diagnostics_.cost_after = final_cost;
  last_lio_diagnostics_.translation_increment_norm =
      metrics.translation_increment_norm;
  last_lio_diagnostics_.rotation_increment_deg =
      metrics.rotation_increment_deg;
  last_lio_diagnostics_.velocity_increment_norm =
      metrics.velocity_increment_norm;
  last_lio_diagnostics_.covariance_trace_after = state_.cov.trace();
  last_lio_diagnostics_.covariance_min_diagonal = state_.cov.diagonal().minCoeff();
  last_lio_diagnostics_.covariance_max_diagonal = state_.cov.diagonal().maxCoeff();
  last_lio_diagnostics_.covariance_min_eigenvalue =
      validation.covariance_min_eigenvalue;
  last_lio_diagnostics_.covariance_asymmetry = validation.covariance_asymmetry;

  if (motion_inputs_ready)
  {
    last_lio_diagnostics_.motion_consistency =
        fast_livo::analyzeMotionConsistency(
            state_increment, motion_prior_covariance,
            motion_posterior_covariance, motion_normal_rhs_gain,
            motion_update_operator, motion_pose_information, motion_pose_rhs,
            motion_iteration_prior_offset, motion_residual_weighted_energy,
            motion_residual_degrees_of_freedom, motion_final_step_scale);
  }
  fast_livo::LioMotionConsistencyMetrics &motion =
      last_lio_diagnostics_.motion_consistency;
  if (motion_inputs_ready)
  {
    motion.accumulated_innovation_component = motion_accumulated_innovation;
    motion.accumulated_relinearization_component =
        motion_accumulated_relinearization;
    motion.analyzed_iteration_count = motion_analyzed_iteration_count;
    if (motion_analyzed_iteration_count > 0)
    {
      motion.maximum_iteration_rotation_equivalent_gain_norm =
          motion_max_rotation_gain;
      motion.maximum_iteration_position_equivalent_gain_norm =
          motion_max_position_gain;
      motion.maximum_iteration_velocity_equivalent_gain_norm =
          motion_max_velocity_gain;
      motion.maximum_iteration_velocity_innovation_component_norm =
          motion_max_velocity_innovation;
    }
    if (motion_nis_iteration_count > 0)
    {
      motion.mean_iteration_linearized_nis_per_dof =
          motion_iteration_nis_sum /
          static_cast<double>(motion_nis_iteration_count);
      motion.maximum_iteration_linearized_nis_per_dof = motion_max_iteration_nis;
    }
  }
  if (state_increment.allFinite() && candidate_state.vel_end.allFinite())
  {
    motion.temporal = motion_correction_window_.update(
        timestamp, state_increment.segment<3>(7), state_increment.segment<3>(3),
        candidate_state.vel_end);
  }
  // Fixed distribution references only; these are shadow labels, not gates.
  constexpr double kChiSquare3P999 = 16.26623619623813;
  constexpr double kChiSquare19P999 = 43.82019596451753;
  if (!motion.valid)
  {
    motion.shadow_warn = true;
    last_lio_diagnostics_.motion_shadow_reason = "diagnostic_invalid";
  }
  else if (motion.q_velocity_prior.value > kChiSquare3P999)
  {
    motion.shadow_warn = true;
    last_lio_diagnostics_.motion_shadow_reason = "q_v_prior_chi2_3_p999";
  }
  else if (motion.q_state_prior.value > kChiSquare19P999)
  {
    motion.shadow_warn = true;
    last_lio_diagnostics_.motion_shadow_reason = "q_state_prior_chi2_19_p999";
  }
  else
  {
    last_lio_diagnostics_.motion_shadow_reason = "none";
  }

  if (validation.commit)
  {
    position_last_ = state_.pos_end;
    const V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0);
    geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(
        euler_cur(0), euler_cur(1), euler_cur(2));
  }
  else
  {
    lidar_degenerated_ = committed_lidar_degenerated;
    lidar_constraint_ratio_ = committed_lidar_constraint_ratio;
    degeneracy_bad_frame_count_ = committed_degeneracy_bad_frames;
    degeneracy_good_frame_count_ = committed_degeneracy_good_frames;
    direction_conflict_frame_count_ = committed_direction_conflict_frames;
    direction_clear_frame_count_ = committed_direction_clear_frames;
    direction_guard_active_ = committed_direction_guard_active;
    if (!shadow_mode)
      ROS_WARN_THROTTLE(1.0,
                        "[LIO_COMMIT] rejected frame=%d status=%s iterations=%d correspondences=%d cost=%.6f->%.6f",
                        current_frame_id_, last_lio_diagnostics_.convergence_status.c_str(),
                        iteration_count, final_correspondence_count, cost_before, final_cost);
  }

  // A rejected candidate may have left transient associations at an
  // intermediate pose; rebuild only that rare path at the restored prior.
  if (!validation.commit && fast_livo::lioStateFinite(state_))
    rebuild_correspondences(state_);

  if (validation.commit && !config_setting_.observability_diagnostics_enable)
    updateLidarDegeneracyStatus();
  last_lio_diagnostics_.is_degenerate = lidar_degenerated_;
  last_lio_diagnostics_.updated_state = state_;
  if (iteration_log)
    *iteration_log << "[LIO_ITER_END] frame_id=" << current_frame_id_
                   << " valid_update=" << last_lio_diagnostics_.valid_update
                   << " status=" << last_lio_diagnostics_.convergence_status
                   << " cost_before=" << cost_before
                   << " cost_after=" << final_cost
                   << " covariance_finite=" << state_.cov.allFinite() << '\n';
  if (last_lio_diagnostics_.observability.valid)
  {
    const V3D weak = last_lio_diagnostics_.observability.weak_translation_direction_world;
    last_lio_diagnostics_.velocity_projection_on_weak_direction = state_propagat.vel_end.dot(weak);
    last_lio_diagnostics_.position_correction_on_weak_direction =
        (state_.pos_end - state_propagat.pos_end).dot(weak);
  }
}

VoxelMapManager::P5Objective VoxelMapManager::p5EvaluateObjective(
    const StatesGroup &evaluation_state,
    const StatesGroup &propagated_state)
{
  P5Objective result;
  std::vector<pointWithVar> points(feats_down_body_->size());
  std::vector<PointToPlane> matches;
  pcl::PointCloud<pcl::PointXYZI>::Ptr world(
      new pcl::PointCloud<pcl::PointXYZI>);
  TransformLidar(evaluation_state.rot_end, evaluation_state.pos_end,
                 feats_down_body_, world);
  const M3D rot_var = evaluation_state.cov.block<3, 3>(0, 0);
  const M3D trans_var = evaluation_state.cov.block<3, 3>(3, 3);
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    pointWithVar &point = points[i];
    point.point_b << feats_down_body_->points[i].x,
        feats_down_body_->points[i].y, feats_down_body_->points[i].z;
    point.source_point_index = static_cast<int>(i);
    point.point_w << world->points[i].x, world->points[i].y,
        world->points[i].z;
    point.body_var = body_cov_list_[i];
    const M3D &cross = cross_mat_list_[i];
    point.var = evaluation_state.rot_end * point.body_var *
                    evaluation_state.rot_end.transpose() +
                (-cross) * rot_var * (-cross.transpose()) + trans_var;
  }
  BuildResidualListOMP(points, matches);
  fast_livo::Matrix6d information = fast_livo::Matrix6d::Zero();
  double absolute_sum = 0.0;
  double weighted_sum = 0.0;
  result.associations.reserve(matches.size());
  for (const PointToPlane &match : matches)
  {
    const V3D point_lidar = extR_ * match.point_b_ + extT_;
    M3D point_cross;
    point_cross << SKEW_SYM_MATRX(point_lidar);
    const V3D point_world =
        propagated_state.rot_end * point_lidar + propagated_state.pos_end;
    Eigen::Matrix<double, 1, 6> plane_jacobian;
    plane_jacobian.block<1, 3>(0, 0) = point_world - match.center_;
    plane_jacobian.block<1, 3>(0, 3) = -match.normal_;
    const M3D body_world_covariance =
        propagated_state.rot_end * extR_ * match.body_cov_ *
        (propagated_state.rot_end * extR_).transpose();
    const double variance =
        0.001 +
        static_cast<double>(plane_jacobian * match.plane_var_ *
                            plane_jacobian.transpose()) +
        static_cast<double>(match.normal_.transpose() *
                            body_world_covariance * match.normal_);
    if (!(variance > 0.0) || !std::isfinite(variance)) continue;
    const double inverse_variance = 1.0 / variance;
    const V3D rotation_jacobian =
        point_cross * evaluation_state.rot_end.transpose() * match.normal_;
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian << rotation_jacobian.transpose(), match.normal_.transpose();
    information += jacobian.transpose() * inverse_variance * jacobian;
    const double residual = static_cast<double>(match.dis_to_plane_);
    absolute_sum += std::fabs(residual);
    weighted_sum += inverse_variance * residual * residual;
    fast_livo::p4::Association association;
    association.point_index = match.point_index_;
    association.plane_id = match.plane_id_;
    association.voxel_x = match.voxel_x_;
    association.voxel_y = match.voxel_y_;
    association.voxel_z = match.voxel_z_;
    association.normal = match.normal_;
    association.residual = residual;
    result.associations.push_back(association);
  }
  result.correspondences = static_cast<int>(result.associations.size());
  if (result.correspondences > 0)
  {
    result.mean_abs_residual =
        absolute_sum / static_cast<double>(result.correspondences);
    result.lidar_cost = weighted_sum;
    result.weighted_rms =
        std::sqrt(weighted_sum / static_cast<double>(result.correspondences));
    result.observability = fast_livo::analyzeLioPoseInformation(
        information, config_setting_.degeneracy_rotation_regularization, true);
  }
  return result;
}

void VoxelMapManager::StateEstimationP5Prototype(
    StatesGroup &state_propagat, double timestamp,
    std::ostream *iteration_log)
{
  p5InitializeOutput();
  const double total_begin = omp_get_wtime();

  struct MutableState
  {
    StatesGroup state;
    V3D position_last;
    geometry_msgs::Quaternion quaternion;
    LioUpdateDiagnostics diagnostics;
    int effective_features = 0;
    std::vector<M3D> cross;
    std::vector<M3D> body_covariance;
    std::vector<pointWithVar> points;
    std::vector<PointToPlane> matches;
    bool degenerated = false;
    double constraint_ratio = 0.0;
    int bad_frames = 0;
    int good_frames = 0;
    int conflict_frames = 0;
    int clear_frames = 0;
    bool direction_guard = false;
    std::vector<fast_livo::RollingCorrectionWindow::SnapshotSample>
        motion_window;
  };

  auto capture_mutable = [&]() {
    MutableState value;
    value.state = state_;
    value.position_last = position_last_;
    value.quaternion = geoQuat_;
    value.diagnostics = last_lio_diagnostics_;
    value.effective_features = effct_feat_num_;
    value.cross = cross_mat_list_;
    value.body_covariance = body_cov_list_;
    value.points = pv_list_;
    value.matches = ptpl_list_;
    value.degenerated = lidar_degenerated_;
    value.constraint_ratio = lidar_constraint_ratio_;
    value.bad_frames = degeneracy_bad_frame_count_;
    value.good_frames = degeneracy_good_frame_count_;
    value.conflict_frames = direction_conflict_frame_count_;
    value.clear_frames = direction_clear_frame_count_;
    value.direction_guard = direction_guard_active_;
    value.motion_window = motion_correction_window_.snapshot();
    return value;
  };
  auto restore_mutable = [&](const MutableState &value) {
    state_ = value.state;
    position_last_ = value.position_last;
    geoQuat_ = value.quaternion;
    last_lio_diagnostics_ = value.diagnostics;
    effct_feat_num_ = value.effective_features;
    cross_mat_list_ = value.cross;
    body_cov_list_ = value.body_covariance;
    pv_list_ = value.points;
    ptpl_list_ = value.matches;
    lidar_degenerated_ = value.degenerated;
    lidar_constraint_ratio_ = value.constraint_ratio;
    degeneracy_bad_frame_count_ = value.bad_frames;
    degeneracy_good_frame_count_ = value.good_frames;
    direction_conflict_frame_count_ = value.conflict_frames;
    direction_clear_frame_count_ = value.clear_frames;
    direction_guard_active_ = value.direction_guard;
    motion_correction_window_.restore(value.motion_window);
  };

  struct Candidate
  {
    const char *label = "";
    double alpha = 0.0;
    MutableState mutable_state;
    P5Objective objective;
    double prior_cost = std::numeric_limits<double>::quiet_NaN();
    double solve_time_ms = 0.0;
  };

  const MutableState before = capture_mutable();
  const double s0_begin = omp_get_wtime();
  StateEstimationInternal(state_propagat, timestamp, iteration_log, nullptr,
                          false);
  Candidate s0;
  s0.label = "S0";
  s0.solve_time_ms = (omp_get_wtime() - s0_begin) * 1000.0;
  s0.objective = p5EvaluateObjective(state_, state_propagat);
  StatesGroup s0_state = state_;
  s0.prior_cost = fast_livo::p5::posePriorMahalanobis(
      (s0_state - state_propagat).head<6>(),
      state_propagat.cov.block<6, 6>(0, 0));
  s0.mutable_state = capture_mutable();

  const int frame_id = current_frame_id_;
  const double p4_timestamp = p4_current_timestamp_s_;
  const Eigen::Vector3d weak =
      s0.mutable_state.diagnostics.observability.weak_translation_direction_world;
  std::vector<Candidate> candidates;
  candidates.reserve(3);
  candidates.push_back(std::move(s0));

  if (candidates.front().mutable_state.diagnostics.observability.valid &&
      weak.allFinite() &&
      weak.norm() > 1e-12 && std::isfinite(p5_leaf_scale_m_))
  {
    const std::array<std::pair<const char *, double>, 2> seeds{{
        {"S-1", -0.5}, {"S+1", 0.5}}};
    for (const auto &seed : seeds)
    {
      restore_mutable(before);
      current_frame_id_ = frame_id;
      p4_current_timestamp_s_ = p4_timestamp;
      StatesGroup initial = state_propagat;
      initial.pos_end += seed.second * p5_leaf_scale_m_ * weak;
      StatesGroup prior = state_propagat;
      const double begin = omp_get_wtime();
      StateEstimationInternal(prior, timestamp, nullptr, &initial, true);
      Candidate candidate;
      candidate.label = seed.first;
      candidate.alpha = seed.second;
      candidate.solve_time_ms = (omp_get_wtime() - begin) * 1000.0;
      candidate.objective = p5EvaluateObjective(state_, state_propagat);
      StatesGroup final_state = state_;
      candidate.prior_cost = fast_livo::p5::posePriorMahalanobis(
          (final_state - state_propagat).head<6>(),
          state_propagat.cov.block<6, 6>(0, 0));
      candidate.mutable_state = capture_mutable();
      candidates.push_back(std::move(candidate));
    }
  }

  auto joint_cost = [](const Candidate &candidate) {
    if (!candidate.mutable_state.diagnostics.commit ||
        !std::isfinite(candidate.objective.lidar_cost) ||
        !std::isfinite(candidate.prior_cost))
      return std::numeric_limits<double>::infinity();
    return candidate.objective.lidar_cost + candidate.prior_cost;
  };
  std::size_t selected = 0;
  if (candidates.size() == 3)
  {
    std::array<double, 3> lidar_cost;
    std::array<double, 3> prior_cost;
    std::array<bool, 3> committed;
    for (std::size_t i = 0; i < 3; ++i)
    {
      lidar_cost[i] = candidates[i].objective.lidar_cost;
      prior_cost[i] = candidates[i].prior_cost;
      committed[i] = candidates[i].mutable_state.diagnostics.commit;
    }
    selected = fast_livo::p5::selectJointMapCandidate(
        lidar_cost, prior_cost, committed);
  }
  restore_mutable(candidates[selected].mutable_state);
  current_frame_id_ = frame_id;
  p4_current_timestamp_s_ = p4_timestamp;

  const double total_time_ms = (omp_get_wtime() - total_begin) * 1000.0;
  if (p5_multistart_csv_.is_open())
  {
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
      const Candidate &candidate = candidates[i];
      const StatesGroup &final_state = candidate.mutable_state.state;
      const auto &diagnostics = candidate.mutable_state.diagnostics;
      p5_multistart_csv_ << std::setprecision(17) << timestamp << ','
          << frame_id << ',' << p5_leaf_scale_m_ << ',' << weak.x() << ','
          << weak.y() << ',' << weak.z() << ',' << candidate.label << ','
          << candidate.alpha << ',' << (i == selected) << ','
          << final_state.pos_end.x() << ',' << final_state.pos_end.y() << ','
          << final_state.pos_end.z() << ','
          << candidate.objective.correspondences << ','
          << candidate.objective.lidar_cost << ','
          << candidate.objective.weighted_rms << ',' << candidate.prior_cost
          << ',' << joint_cost(candidate) << ','
          << diagnostics.iteration_count << ',' << diagnostics.commit << ','
          << final_state.cov.allFinite() << ','
          << diagnostics.covariance_asymmetry << ','
          << diagnostics.covariance_min_eigenvalue << ','
          << candidate.solve_time_ms << ',' << total_time_ms << '\n';
    }
    p5_multistart_csv_.flush();
  }
}

void VoxelMapManager::configureP5SeedBasin(
    double leaf_scale_m, const std::string &default_output_directory)
{
  if (!config_setting_.p5_seed_basin_enable &&
      !config_setting_.p5_weak_axis_multistart_enable) return;
  if (!std::isfinite(leaf_scale_m) || leaf_scale_m <= 0.0)
    throw std::runtime_error("P5 requires a finite positive production leaf scale");
  if (config_setting_.p5_seed_basin_enable &&
      config_setting_.p5_weak_axis_multistart_enable)
    throw std::runtime_error("P5 diagnostic and prototype modes are mutually exclusive");
  if (config_setting_.p4_frontend_diagnostics_enable ||
      config_setting_.p4b_snapshot_enable)
    throw std::runtime_error("P5 cannot be combined with P4 diagnostics");
  p5_leaf_scale_m_ = leaf_scale_m;
  if (config_setting_.p5_seed_basin_output_directory.empty())
    config_setting_.p5_seed_basin_output_directory =
        default_output_directory + "p5_seed_basin";
  ROS_WARN("[P5_SEED_BASIN] mode=%s L=%.6f output=%s",
           config_setting_.p5_weak_axis_multistart_enable
               ? "weak_axis_multistart" : "read_only_shadow",
           p5_leaf_scale_m_,
           config_setting_.p5_seed_basin_output_directory.c_str());
}

void VoxelMapManager::p5InitializeOutput()
{
  if (p5_output_initialized_ ||
      (!config_setting_.p5_seed_basin_enable &&
       !config_setting_.p5_weak_axis_multistart_enable)) return;
  p5_output_initialized_ = true;
  std::error_code error;
  std::filesystem::create_directories(
      config_setting_.p5_seed_basin_output_directory, error);
  if (error)
  {
    ROS_ERROR("[P5_SEED_BASIN] cannot create output directory '%s': %s",
              config_setting_.p5_seed_basin_output_directory.c_str(),
              error.message().c_str());
    return;
  }
  const std::string prefix =
      config_setting_.p5_seed_basin_output_directory + "/p5_seed_basin_";
  if (config_setting_.p5_seed_basin_enable)
  {
    p5_seed_csv_.open(prefix + "seeds.csv", std::ios::out);
    p5_profile_csv_.open(prefix + "profile.csv", std::ios::out);
    p5_parity_csv_.open(prefix + "parity.csv", std::ios::out);
    if (p5_seed_csv_.is_open())
      p5_seed_csv_ << "timestamp_s,relative_time_s,frame_id,L_m,weak_x,weak_y,weak_z,weak_eval_0,weak_eval_1,weak_eval_2,seed_label,seed_alpha,initial_x,initial_y,initial_z,final_x,final_y,final_z,final_weak_offset_m,correspondences,association_jaccard_s0,mean_abs_residual_m,weighted_lidar_cost,weighted_rms,j_prior,j_total,iterations,commit,solve_time_ms\n";
    if (p5_profile_csv_.is_open())
      p5_profile_csv_ << "timestamp_s,relative_time_s,frame_id,L_m,weak_x,weak_y,weak_z,alpha,x,y,z,correspondences,association_jaccard_alpha0,mean_abs_residual_m,weighted_lidar_cost,weighted_rms,conditional_eval_0,conditional_eval_1,conditional_eval_2\n";
    if (p5_parity_csv_.is_open())
      p5_parity_csv_ << "timestamp_s,relative_time_s,frame_id,state_max_abs,covariance_max_abs,correspondence_jaccard,point_count_equal,pass\n";
  }
  if (config_setting_.p5_weak_axis_multistart_enable)
  {
    p5_multistart_csv_.open(prefix + "multistart.csv", std::ios::out);
    if (p5_multistart_csv_.is_open())
      p5_multistart_csv_ << "timestamp_s,frame_id,L_m,weak_x,weak_y,weak_z,seed_label,seed_alpha,selected,final_x,final_y,final_z,correspondences,weighted_lidar_cost,weighted_rms,j_prior,j_total,iterations,commit,covariance_finite,covariance_asymmetry,covariance_min_eigenvalue,solve_time_ms,total_multistart_time_ms\n";
  }
}

void VoxelMapManager::runP5SeedBasinShadow(
    const StatesGroup &propagated_state, double timestamp,
    double relative_time_s)
{
  if (!config_setting_.p5_seed_basin_enable ||
      relative_time_s < config_setting_.p5_seed_basin_start_s ||
      relative_time_s > config_setting_.p5_seed_basin_end_s ||
      current_frame_id_ % config_setting_.p5_seed_basin_frame_stride != 0)
    return;
  p5InitializeOutput();
  if (!p5_seed_csv_.is_open() || !p5_profile_csv_.is_open() ||
      !std::isfinite(p5_leaf_scale_m_) || !propagated_state.cov.allFinite())
    return;

  const LioUpdateDiagnostics production_diagnostics = last_lio_diagnostics_;
  if (!production_diagnostics.observability.valid) return;
  const Eigen::Vector3d weak =
      production_diagnostics.observability.weak_translation_direction_world;
  if (!weak.allFinite() || weak.norm() <= 1e-12) return;

  struct SeedResult
  {
    const char *label = "";
    double alpha = 0.0;
    StatesGroup initial;
    StatesGroup final;
    LioUpdateDiagnostics diagnostics;
    P5Objective objective;
    double prior_cost = std::numeric_limits<double>::quiet_NaN();
    double solve_time_ms = 0.0;
  };

  const StatesGroup saved_state = state_;
  const V3D saved_position_last = position_last_;
  const geometry_msgs::Quaternion saved_quaternion = geoQuat_;
  const int saved_frame_id = current_frame_id_;
  const double saved_p4_timestamp = p4_current_timestamp_s_;
  const LioUpdateDiagnostics saved_diagnostics = last_lio_diagnostics_;
  const int saved_effective_features = effct_feat_num_;
  const auto saved_cross = cross_mat_list_;
  const auto saved_body_covariance = body_cov_list_;
  const auto saved_points = pv_list_;
  const auto saved_matches = ptpl_list_;
  const bool saved_degenerated = lidar_degenerated_;
  const double saved_constraint_ratio = lidar_constraint_ratio_;
  const int saved_bad_frames = degeneracy_bad_frame_count_;
  const int saved_good_frames = degeneracy_good_frame_count_;
  const int saved_conflict_frames = direction_conflict_frame_count_;
  const int saved_clear_frames = direction_clear_frame_count_;
  const bool saved_direction_guard = direction_guard_active_;
  const auto saved_motion_window = motion_correction_window_.snapshot();

  auto restore_production = [&]() {
    state_ = saved_state;
    position_last_ = saved_position_last;
    geoQuat_ = saved_quaternion;
    current_frame_id_ = saved_frame_id;
    p4_current_timestamp_s_ = saved_p4_timestamp;
    last_lio_diagnostics_ = saved_diagnostics;
    effct_feat_num_ = saved_effective_features;
    cross_mat_list_ = saved_cross;
    body_cov_list_ = saved_body_covariance;
    pv_list_ = saved_points;
    ptpl_list_ = saved_matches;
    lidar_degenerated_ = saved_degenerated;
    lidar_constraint_ratio_ = saved_constraint_ratio;
    degeneracy_bad_frame_count_ = saved_bad_frames;
    degeneracy_good_frame_count_ = saved_good_frames;
    direction_conflict_frame_count_ = saved_conflict_frames;
    direction_clear_frame_count_ = saved_clear_frames;
    direction_guard_active_ = saved_direction_guard;
    motion_correction_window_.restore(saved_motion_window);
  };

  constexpr std::array<const char *, 5> labels{
      {"S0", "S-1", "S+1", "S-2", "S+2"}};
  std::vector<SeedResult> seeds;
  seeds.reserve(fast_livo::p5::kDiagnosticSeedAlphas.size());
  for (std::size_t i = 0; i < fast_livo::p5::kDiagnosticSeedAlphas.size(); ++i)
  {
    SeedResult result;
    result.label = labels[i];
    result.alpha = fast_livo::p5::kDiagnosticSeedAlphas[i];
    result.initial = propagated_state;
    result.initial.pos_end += result.alpha * p5_leaf_scale_m_ * weak;
    StatesGroup prior = propagated_state;
    const double begin = omp_get_wtime();
    StateEstimationInternal(prior, timestamp, nullptr, &result.initial, true);
    result.solve_time_ms = (omp_get_wtime() - begin) * 1000.0;
    result.final = state_;
    result.diagnostics = last_lio_diagnostics_;
    result.objective = p5EvaluateObjective(result.final, propagated_state);
    StatesGroup final_copy = result.final;
    const Eigen::Matrix<double, 6, 1> pose_delta =
        (final_copy - propagated_state).head<6>();
    result.prior_cost = fast_livo::p5::posePriorMahalanobis(
        pose_delta, propagated_state.cov.block<6, 6>(0, 0));
    seeds.push_back(std::move(result));
    restore_production();
  }

  const auto &reference_associations = seeds.front().objective.associations;
  for (const SeedResult &seed : seeds)
  {
    const double overlap = fast_livo::p5::associationJaccard(
        reference_associations, seed.objective.associations);
    const double total = seed.objective.lidar_cost + seed.prior_cost;
    p5_seed_csv_ << std::setprecision(17)
        << timestamp << ',' << relative_time_s << ',' << saved_frame_id << ','
        << p5_leaf_scale_m_ << ',' << weak.x() << ',' << weak.y() << ','
        << weak.z() << ','
        << production_diagnostics.observability.translation_eigenvalues[0] << ','
        << production_diagnostics.observability.translation_eigenvalues[1] << ','
        << production_diagnostics.observability.translation_eigenvalues[2] << ','
        << seed.label << ',' << seed.alpha << ','
        << seed.initial.pos_end.x() << ',' << seed.initial.pos_end.y() << ','
        << seed.initial.pos_end.z() << ',' << seed.final.pos_end.x() << ','
        << seed.final.pos_end.y() << ',' << seed.final.pos_end.z() << ','
        << (seed.final.pos_end - propagated_state.pos_end).dot(weak) << ','
        << seed.objective.correspondences << ',' << overlap << ','
        << seed.objective.mean_abs_residual << ',' << seed.objective.lidar_cost
        << ',' << seed.objective.weighted_rms << ',' << seed.prior_cost << ','
        << total << ',' << seed.diagnostics.iteration_count << ','
        << seed.diagnostics.commit << ',' << seed.solve_time_ms << '\n';
  }

  std::vector<std::pair<double, P5Objective>> profile;
  profile.reserve(fast_livo::p5::kProfileAlphas.size());
  for (const double alpha : fast_livo::p5::kProfileAlphas)
  {
    StatesGroup pose = propagated_state;
    pose.pos_end += alpha * p5_leaf_scale_m_ * weak;
    profile.emplace_back(alpha, p5EvaluateObjective(pose, propagated_state));
  }
  const auto zero = std::find_if(
      profile.begin(), profile.end(),
      [](const auto &entry) { return entry.first == 0.0; });
  const std::vector<fast_livo::p4::Association> empty;
  const auto &zero_associations =
      zero == profile.end() ? empty : zero->second.associations;
  for (const auto &entry : profile)
  {
    const double alpha = entry.first;
    const P5Objective &objective = entry.second;
    const V3D position =
        propagated_state.pos_end + alpha * p5_leaf_scale_m_ * weak;
    const Eigen::Vector3d eigenvalues = objective.observability.valid
        ? objective.observability.translation_eigenvalues
        : Eigen::Vector3d::Constant(
              std::numeric_limits<double>::quiet_NaN());
    p5_profile_csv_ << std::setprecision(17)
        << timestamp << ',' << relative_time_s << ',' << saved_frame_id << ','
        << p5_leaf_scale_m_ << ',' << weak.x() << ',' << weak.y() << ','
        << weak.z() << ',' << alpha << ',' << position.x() << ','
        << position.y() << ',' << position.z() << ','
        << objective.correspondences << ','
        << fast_livo::p5::associationJaccard(zero_associations,
                                             objective.associations)
        << ',' << objective.mean_abs_residual << ',' << objective.lidar_cost
        << ',' << objective.weighted_rms << ',' << eigenvalues[0] << ','
        << eigenvalues[1] << ',' << eigenvalues[2] << '\n';
  }
  p5_seed_csv_.flush();
  p5_profile_csv_.flush();
  restore_production();
  if (p5_parity_csv_.is_open())
  {
    auto associations_from_matches = [](const std::vector<PointToPlane> &matches) {
      std::vector<fast_livo::p4::Association> result;
      result.reserve(matches.size());
      for (const PointToPlane &match : matches)
      {
        fast_livo::p4::Association value;
        value.point_index = match.point_index_;
        value.plane_id = match.plane_id_;
        value.voxel_x = match.voxel_x_;
        value.voxel_y = match.voxel_y_;
        value.voxel_z = match.voxel_z_;
        result.push_back(value);
      }
      return result;
    };
    StatesGroup restored_state = state_;
    const double state_difference =
        (restored_state - saved_state).cwiseAbs().maxCoeff();
    const double covariance_difference =
        (state_.cov - saved_state.cov).cwiseAbs().maxCoeff();
    const double correspondence_parity = fast_livo::p5::associationJaccard(
        associations_from_matches(saved_matches),
        associations_from_matches(ptpl_list_));
    const bool point_count_equal = pv_list_.size() == saved_points.size();
    const bool pass = state_difference == 0.0 && covariance_difference == 0.0 &&
                      correspondence_parity == 1.0 && point_count_equal;
    p5_parity_csv_ << std::setprecision(17) << timestamp << ','
        << relative_time_s << ',' << saved_frame_id << ',' << state_difference
        << ',' << covariance_difference << ',' << correspondence_parity << ','
        << point_count_equal << ',' << pass << '\n';
    p5_parity_csv_.flush();
  }
}

void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void VoxelMapManager::BuildVoxelMap(double timestamp)
{
  if (config_setting_.p4b_snapshot_enable)
    p4b_accepted_map_frame_ids_.push_back(current_frame_id_);
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;

  std::vector<pointWithVar> input_points;

  for (size_t i = 0; i < feats_down_world_->size(); i++)
  {
    pointWithVar pv;
    pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    var = (state_.rot_end * extR_) * var * (state_.rot_end * extR_).transpose() +
          (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + state_.cov.block<3, 3>(3, 3);
    pv.var = var;
    if (config_setting_.p4_frontend_diagnostics_enable ||
        config_setting_.p4b_snapshot_enable)
    {
      pv.source_frame_id = current_frame_id_;
      pv.source_point_index = static_cast<int>(i);
      pv.source_timestamp_s = timestamp;
      pv.source_origin_w = state_.pos_end;
    }
    input_points.push_back(pv);
  }

  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
    }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      octo_tree->p4b_retain_support_ = config_setting_.p4b_retain_support_points;
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
    }
  }
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter)
  {
    iter->second->init_octo_tree();
  }
}

V3F VoxelMapManager::RGBFromVoxel(const V3D &input_point)
{
  int64_t loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = floor(input_point[j] / config_setting_.max_voxel_size_);
  }

  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  int64_t ind = loc_xyz[0] + loc_xyz[1] + loc_xyz[2];
  uint k((ind + 100000) % 3);
  V3F RGB((k == 0) * 255.0, (k == 1) * 255.0, (k == 2) * 255.0);
  // cout<<"RGB: "<<RGB.transpose()<<endl;
  return RGB;
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
  if (config_setting_.p4b_snapshot_enable &&
      (p4b_accepted_map_frame_ids_.empty() ||
       p4b_accepted_map_frame_ids_.back() != current_frame_id_))
    p4b_accepted_map_frame_ids_.push_back(current_frame_id_);
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    if (config_setting_.p4_frontend_diagnostics_enable ||
        config_setting_.p4b_snapshot_enable)
      p4_voxel_last_update_frame_[position] = current_frame_id_;
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) { voxel_map_[position]->UpdateOctoTree(p_v); }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      octo_tree->p4b_retain_support_ = config_setting_.p4b_retain_support_points;
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->UpdateOctoTree(p_v);
    }
  }
}

void VoxelMapManager::clearLocalMap()
{
  std::unordered_set<VoxelOctoTree *> deleted;
  auto delete_map = [&deleted](std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &map) {
    for (auto &kv : map)
    {
      if (kv.second != nullptr && deleted.insert(kv.second).second)
      {
        delete kv.second;
      }
    }
    map.clear();
  };

  delete_map(voxel_map_);
  delete_map(long_term_visual_map_);
  visual_observed_voxels_.clear();
  cross_mat_list_.clear();
  body_cov_list_.clear();
  pv_list_.clear();
  ptpl_list_.clear();
  lidar_degenerated_ = false;
  lidar_constraint_ratio_ = 0.0;
  last_lio_diagnostics_ = LioUpdateDiagnostics();
  degeneracy_bad_frame_count_ = 0;
  degeneracy_good_frame_count_ = 0;
  direction_conflict_frame_count_ = 0;
  direction_clear_frame_count_ = 0;
  direction_guard_active_ = false;
  motion_correction_window_.clear();
  p4_previous_associations_.clear();
  p4_voxel_last_update_frame_.clear();
  p4b_accepted_map_frame_ids_.clear();
  if (p4_iteration_csv_.is_open()) p4_iteration_csv_.close();
  if (p4_correspondence_csv_.is_open()) p4_correspondence_csv_.close();
  if (p4_map_csv_.is_open()) p4_map_csv_.close();
  if (p4_deskew_csv_.is_open()) p4_deskew_csv_.close();
  if (p5_seed_csv_.is_open()) p5_seed_csv_.close();
  if (p5_profile_csv_.is_open()) p5_profile_csv_.close();
  if (p5_parity_csv_.is_open()) p5_parity_csv_.close();
  if (p5_multistart_csv_.is_open()) p5_multistart_csv_.close();
  p4_output_initialized_ = false;
  p5_output_initialized_ = false;
  p4_first_timestamp_s_ = std::numeric_limits<double>::quiet_NaN();
  p4_current_timestamp_s_ = std::numeric_limits<double>::quiet_NaN();
  current_frame_id_ = 0;
  scan_count = 0;
  last_slide_position = V3D::Zero();
}

namespace
{
bool p4bVoxelKeyLess(const VOXEL_LOCATION &a, const VOXEL_LOCATION &b)
{
  if (a.x != b.x) return a.x < b.x;
  if (a.y != b.y) return a.y < b.y;
  return a.z < b.z;
}
}

VoxelMapManagerSnapshot VoxelMapManager::captureSnapshot() const
{
  VoxelMapManagerSnapshot result;
  auto capture_map = [](const auto &source, auto &destination) {
    destination.reserve(source.size());
    for (const auto &entry : source)
      if (entry.second)
        destination.emplace_back(entry.first,
                                 entry.second->captureSnapshot());
    std::sort(destination.begin(), destination.end(),
              [](const auto &a, const auto &b) {
                return p4bVoxelKeyLess(a.first, b.first);
              });
  };
  capture_map(voxel_map_, result.local_map);
  capture_map(long_term_visual_map_, result.long_term_map);
  result.visual_observed_voxels.assign(visual_observed_voxels_.begin(),
                                       visual_observed_voxels_.end());
  std::sort(result.visual_observed_voxels.begin(),
            result.visual_observed_voxels.end(), p4bVoxelKeyLess);
  result.current_frame_id = current_frame_id_;
  result.scan_count = scan_count;
  result.lidar_rotation_to_imu = extR_;
  result.lidar_translation_to_imu = extT_;
  result.state = state_;
  result.position_last = position_last_;
  result.last_slide_position = last_slide_position;
  result.lidar_degenerated = lidar_degenerated_;
  result.lidar_constraint_ratio = lidar_constraint_ratio_;
  result.degeneracy_bad_frame_count = degeneracy_bad_frame_count_;
  result.degeneracy_good_frame_count = degeneracy_good_frame_count_;
  result.direction_conflict_frame_count = direction_conflict_frame_count_;
  result.direction_clear_frame_count = direction_clear_frame_count_;
  result.direction_guard_active = direction_guard_active_;
  result.motion_correction_samples = motion_correction_window_.snapshot();
  result.p4_voxel_last_update_frame.reserve(
      p4_voxel_last_update_frame_.size());
  for (const auto &entry : p4_voxel_last_update_frame_)
    result.p4_voxel_last_update_frame.push_back(entry);
  std::sort(result.p4_voxel_last_update_frame.begin(),
            result.p4_voxel_last_update_frame.end(),
            [](const auto &a, const auto &b) {
              return p4bVoxelKeyLess(a.first, b.first);
            });
  result.p4_first_timestamp_s = p4_first_timestamp_s_;
  result.p4_current_timestamp_s = p4_current_timestamp_s_;
  result.next_plane_id = voxel_plane_id;
  result.accepted_map_frame_ids.assign(p4b_accepted_map_frame_ids_.begin(),
                                       p4b_accepted_map_frame_ids_.end());
  return result;
}

void VoxelMapManager::restoreSnapshot(
    const VoxelMapManagerSnapshot &snapshot)
{
  clearLocalMap();
  auto restore_map = [](const auto &source, auto &destination) {
    for (const auto &entry : source)
      destination.emplace(entry.first,
                          VoxelOctoTree::restoreSnapshot(entry.second).release());
  };
  restore_map(snapshot.local_map, voxel_map_);
  restore_map(snapshot.long_term_map, long_term_visual_map_);
  visual_observed_voxels_.insert(snapshot.visual_observed_voxels.begin(),
                                 snapshot.visual_observed_voxels.end());
  current_frame_id_ = snapshot.current_frame_id;
  scan_count = snapshot.scan_count;
  extR_ = snapshot.lidar_rotation_to_imu;
  extT_ = snapshot.lidar_translation_to_imu;
  state_ = snapshot.state;
  position_last_ = snapshot.position_last;
  last_slide_position = snapshot.last_slide_position;
  lidar_degenerated_ = snapshot.lidar_degenerated;
  lidar_constraint_ratio_ = snapshot.lidar_constraint_ratio;
  degeneracy_bad_frame_count_ = snapshot.degeneracy_bad_frame_count;
  degeneracy_good_frame_count_ = snapshot.degeneracy_good_frame_count;
  direction_conflict_frame_count_ = snapshot.direction_conflict_frame_count;
  direction_clear_frame_count_ = snapshot.direction_clear_frame_count;
  direction_guard_active_ = snapshot.direction_guard_active;
  motion_correction_window_.restore(snapshot.motion_correction_samples);
  for (const auto &entry : snapshot.p4_voxel_last_update_frame)
    p4_voxel_last_update_frame_.emplace(entry);
  p4_first_timestamp_s_ = snapshot.p4_first_timestamp_s;
  p4_current_timestamp_s_ = snapshot.p4_current_timestamp_s;
  voxel_plane_id = snapshot.next_plane_id;
  p4b_accepted_map_frame_ids_.assign(snapshot.accepted_map_frame_ids.begin(),
                                     snapshot.accepted_map_frame_ids.end());
}

void VoxelMapManager::setP4bRecentPointExclusion(
    int accepted_lidar_frames)
{
  p4b_recent_point_exclusion_frames_ =
      std::max(0, accepted_lidar_frames);
  p4b_refitted_plane_cache_.clear();
}

const VoxelPlane *VoxelMapManager::p4bQueryPlane(
    const VoxelOctoTree &node)
{
  const VoxelPlane *original = node.plane_ptr_;
  if (!original || !original->is_plane_ ||
      p4b_recent_point_exclusion_frames_ <= 0)
    return original;

  const auto found = p4b_refitted_plane_cache_.find(original);
  if (found != p4b_refitted_plane_cache_.end())
    return found->second.get();

  std::vector<pointWithVar> historical;
  historical.reserve(original->p4b_support_points_.size());
  std::unordered_set<int> excluded_frames;
  for (auto it = p4b_accepted_map_frame_ids_.rbegin();
       it != p4b_accepted_map_frame_ids_.rend() &&
       excluded_frames.size() <
           static_cast<std::size_t>(p4b_recent_point_exclusion_frames_);
       ++it)
    excluded_frames.insert(*it);
  for (const P4bSupportPoint &support :
       original->p4b_support_points_.points())
  {
    const pointWithVar point = support.toPointWithVar();
    if (point.source_frame_id >= 0 &&
        excluded_frames.count(point.source_frame_id) == 0)
      historical.push_back(point);
  }

  std::unique_ptr<VoxelPlane> refitted;
  if (historical.size() >
      static_cast<std::size_t>(node.points_size_threshold_))
  {
    refitted.reset(new VoxelPlane(*original));
    VoxelOctoTree fitter(node.max_layer_, node.layer_,
                         node.points_size_threshold_, node.max_points_num_,
                         node.planer_threshold_);
    fitter.init_plane(historical, refitted.get());
    if (!refitted->is_plane_) refitted.reset();
  }
  const VoxelPlane *result = refitted.get();
  p4b_refitted_plane_cache_.emplace(original, std::move(refitted));
  return result;
}

const VoxelPlane *VoxelMapManager::p4bRefittedPlaneForTest(
    const VoxelOctoTree &node)
{
  return p4bQueryPlane(node);
}

std::size_t VoxelMapManager::p4bCurrentUpdatedVoxelCount() const
{
  std::size_t count = 0;
  for (const auto &entry : p4_voxel_last_update_frame_)
    if (entry.second == current_frame_id_) ++count;
  return count;
}

void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
  int max_layer = config_setting_.max_layer_;
  double voxel_size = config_setting_.max_voxel_size_;
  double sigma_num = config_setting_.sigma_num_;
  std::mutex mylock;
  ptpl_list.clear();
  std::vector<PointToPlane> all_ptpl_list(pv_list.size());
  std::vector<bool> useful_ptpl(pv_list.size());
  std::vector<size_t> index(pv_list.size());
  for (size_t i = 0; i < index.size(); ++i)
  {
    index[i] = i;
    useful_ptpl[i] = false;
  }
  #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for if(!config_setting_.deterministic_lio_update_en)
  #endif
  for (int i = 0; i < index.size(); i++)
  {
    auto find_voxel_tree = [&](const VOXEL_LOCATION &loc) -> VoxelOctoTree *
    {
      auto local_it = voxel_map_.find(loc);
      if (local_it != voxel_map_.end()) return local_it->second;
      if (!config_setting_.long_term_visual_map_en) return nullptr;
      auto long_it = long_term_visual_map_.find(loc);
      if (long_it != long_term_visual_map_.end()) return long_it->second;
      return nullptr;
    };

    pointWithVar &pv = pv_list[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = pv.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    VOXEL_LOCATION matched_position = position;
    VoxelOctoTree *current_octo = find_voxel_tree(position);
    if (current_octo != nullptr)
    {
      PointToPlane single_ptpl;
      bool is_sucess = false;
      double prob = 0;
      build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl);
      if (!is_sucess)
      {
        VOXEL_LOCATION near_position = position;
        if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) { near_position.x = near_position.x + 1; }
        else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) { near_position.x = near_position.x - 1; }
        if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) { near_position.y = near_position.y + 1; }
        else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) { near_position.y = near_position.y - 1; }
        if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) { near_position.z = near_position.z + 1; }
        else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) { near_position.z = near_position.z - 1; }
        VoxelOctoTree *near_octo = find_voxel_tree(near_position);
        if (near_octo != nullptr)
        {
          build_single_residual(pv, near_octo, 0, is_sucess, prob, single_ptpl);
          if (is_sucess) matched_position = near_position;
        }
      }
      if (is_sucess)
      {
        mylock.lock();
        single_ptpl.voxel_x_ = matched_position.x;
        single_ptpl.voxel_y_ = matched_position.y;
        single_ptpl.voxel_z_ = matched_position.z;
        useful_ptpl[i] = true;
        all_ptpl_list[i] = single_ptpl;
        mylock.unlock();
      }
      else
      {
        mylock.lock();
        useful_ptpl[i] = false;
        mylock.unlock();
      }
    }
  }
  for (size_t i = 0; i < useful_ptpl.size(); i++)
  {
    if (useful_ptpl[i]) { ptpl_list.push_back(all_ptpl_list[i]); }
  }
}

void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess,
                                            double &prob, PointToPlane &single_ptpl)
{
  int max_layer = config_setting_.max_layer_;
  double sigma_num = config_setting_.sigma_num_;

  double radius_k = 3;
  Eigen::Vector3d p_w = pv.point_w;
  const VoxelPlane *query_plane = p4bQueryPlane(*current_octo);
  if (query_plane && query_plane->is_plane_)
  {
    const VoxelPlane &plane = *query_plane;
    Eigen::Vector3d p_world_to_center = p_w - plane.center_;
    float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);
    float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                          (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

    if (range_dis <= radius_k * plane.radius_)
    {
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
      J_nq.block<1, 3>(0, 3) = -plane.normal_;
      double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
      sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
      if (dis_to_plane < sigma_num * sqrt(sigma_l))
      {
        is_sucess = true;
        double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
        if (this_prob > prob)
        {
          prob = this_prob;
          pv.normal = plane.normal_;
          single_ptpl.body_cov_ = pv.body_var;
          single_ptpl.point_index_ = pv.source_point_index;
          single_ptpl.raw_point_ = pv.point_raw;
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          single_ptpl.plane_var_ = plane.plane_var_;
          single_ptpl.normal_ = plane.normal_;
          single_ptpl.center_ = plane.center_;
          single_ptpl.d_ = plane.d_;
          single_ptpl.layer_ = current_layer;
          single_ptpl.eigen_value_ = plane.min_eigen_value_;
          single_ptpl.plane_id_ = plane.id_;
          single_ptpl.source_plane_ = &plane;
          single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        }
        return;
      }
      else
      {
        // is_sucess = false;
        return;
      }
    }
    else
    {
      // is_sucess = false;
      return;
    }
  }
  else
  {
    if (current_layer < max_layer)
    {
      for (size_t leafnum = 0; leafnum < 8; leafnum++)
      {
        if (current_octo->leaves_[leafnum] != nullptr)
        {

          VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
          build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
        }
      }
      return;
    }
    else { return; }
  }
}

void VoxelMapManager::pubVoxelMap()
{
  double max_trace = 0.25;
  double pow_num = 0.2;
  ros::Rate loop(500);
  float use_alpha = 0.8;
  visualization_msgs::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);
  std::vector<VoxelPlane> pub_plane_list;
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
  {
    GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
  }
  for (size_t i = 0; i < pub_plane_list.size(); i++)
  {
    V3D plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) { trace = max_trace; }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    double alpha;
    if (pub_plane_list[i].is_plane_) { alpha = use_alpha; }
    else { alpha = 0; }
    pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
  }
  voxel_map_pub_.publish(voxel_plane);
  loop.sleep();
}

void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list)
{
  if (current_octo->layer_ > pub_max_voxel_layer) { return; }
  if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
  if (current_octo->layer_ < current_octo->max_layer_)
  {
    if (!current_octo->plane_ptr_->is_plane_)
    {
      for (size_t i = 0; i < 8; i++)
      {
        if (current_octo->leaves_[i] != nullptr) { GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list); }
      }
    }
  }
  return;
}

void VoxelMapManager::pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane,
                                     const float alpha, const Eigen::Vector3d rgb)
{
  visualization_msgs::Marker plane;
  plane.header.frame_id = "camera_init";
  plane.header.stamp = ros::Time();
  plane.ns = plane_ns;
  plane.id = single_plane.id_;
  plane.type = visualization_msgs::Marker::CYLINDER;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = single_plane.center_[0];
  plane.pose.position.y = single_plane.center_[1];
  plane.pose.position.z = single_plane.center_[2];
  geometry_msgs::Quaternion q;
  CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
  plane.pose.orientation = q;
  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_);
  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = ros::Duration();
  plane_pub.markers.push_back(plane);
}

void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                                      geometry_msgs::Quaternion &q)
{
  Eigen::Matrix3d rot;
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
  Eigen::Matrix3d rotation = rot.transpose();
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) { v = vmin; }

  if (v > vmax) { v = vmax; }

  double dr, dg, db;

  if (v < 0.1242)
  {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  }
  else if (v < 0.3747)
  {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  }
  else if (v < 0.6253)
  {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  }
  else if (v < 0.8758)
  {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  }
  else
  {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void VoxelMapManager::mapSliding()
{
  if((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh)
  {
    std::cout<<RED<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
    return;
  }

  //get global id now
  last_slide_position = position_last_;
  double t_sliding_start = omp_get_wtime();
  float loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
  clearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size, (int64_t)loc_xyz[0] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[1] + config_setting_.half_map_size, (int64_t)loc_xyz[1] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[2] + config_setting_.half_map_size, (int64_t)loc_xyz[2] - config_setting_.half_map_size);

  if (config_setting_.long_term_visual_map_en &&
      static_cast<int>(long_term_visual_map_.size()) > config_setting_.long_term_visual_max_voxels)
  {
    int to_remove = static_cast<int>(long_term_visual_map_.size()) - config_setting_.long_term_visual_max_voxels;
    std::vector<std::pair<double, VOXEL_LOCATION>> sorted;
    sorted.reserve(long_term_visual_map_.size());
    for (const auto &kv : long_term_visual_map_) {
      V3D center(kv.first.x * config_setting_.max_voxel_size_,
                 kv.first.y * config_setting_.max_voxel_size_,
                 kv.first.z * config_setting_.max_voxel_size_);
      sorted.emplace_back((center - position_last_).squaredNorm(), kv.first);
    }
    std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
    for (const auto &p : sorted) {
      if (to_remove <= 0) break;
      auto it = long_term_visual_map_.find(p.second);
      if (it != long_term_visual_map_.end()) {
        delete it->second;
        long_term_visual_map_.erase(it);
        --to_remove;
      }
    }
  }

  double t_sliding_end = omp_get_wtime();
  std::cout<<RED<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs, local="
           << voxel_map_.size() << ", long_term_visual=" << long_term_visual_map_.size() << RESET <<"\n";
  return;
}

void VoxelMapManager::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
  int delete_voxel_cout = 0;
  int transfer_voxel_count = 0;
  std::vector<VOXEL_LOCATION> sorted_keys;
  sorted_keys.reserve(voxel_map_.size());
  for (const auto &kv : voxel_map_) sorted_keys.push_back(kv.first);
  std::sort(sorted_keys.begin(), sorted_keys.end(),
            [](const VOXEL_LOCATION &a, const VOXEL_LOCATION &b) {
              if (a.x != b.x) return a.x < b.x;
              if (a.y != b.y) return a.y < b.y;
              return a.z < b.z;
            });
  for (const auto &loc : sorted_keys)
  {
    auto it = voxel_map_.find(loc);
    if (it == voxel_map_.end()) continue;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      const bool has_visual_observation = visual_observed_voxels_.find(loc) != visual_observed_voxels_.end();
      if (config_setting_.long_term_visual_map_en && has_visual_observation)
      {
        auto long_term_it = long_term_visual_map_.find(loc);
        if (long_term_it != long_term_visual_map_.end())
        {
          delete long_term_it->second;
          long_term_it->second = it->second;
        }
        else
        {
          long_term_visual_map_[loc] = it->second;
        }
        voxel_map_.erase(it);
        ++transfer_voxel_count;
        continue;
      }
      delete it->second;
      voxel_map_.erase(it);
      delete_voxel_cout++;
    }
  }
  std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels, transfer "
           <<transfer_voxel_count<<" to long-term visual map"<<RESET<<"\n";
  // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}

void VoxelMapManager::setVisualObservedVoxels(const std::vector<VOXEL_LOCATION> &observed_voxels)
{
  visual_observed_voxels_.clear();
  visual_observed_voxels_.reserve(observed_voxels.size());
  for (const auto &loc : observed_voxels)
  {
    visual_observed_voxels_.insert(loc);
  }
}

void VoxelMapManager::updateLidarDegeneracyStatus()
{
  if (effct_feat_num_ < config_setting_.degeneracy_min_effective_features || ptpl_list_.empty())
  {
    lidar_constraint_ratio_ = 0.0;
    lidar_degenerated_ = true;
    return;
  }

  Eigen::Matrix3d normal_cov = Eigen::Matrix3d::Zero();
  int valid_normal_count = 0;
  for (const auto &ptpl : ptpl_list_)
  {
    if (ptpl.normal_.norm() < 1e-6) continue;
    Eigen::Vector3d n = ptpl.normal_.normalized();
    normal_cov += n * n.transpose();
    ++valid_normal_count;
  }

  if (valid_normal_count < config_setting_.degeneracy_min_effective_features)
  {
    lidar_constraint_ratio_ = 0.0;
    lidar_degenerated_ = true;
    return;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(normal_cov);
  if (es.info() != Eigen::Success)
  {
    lidar_constraint_ratio_ = 0.0;
    lidar_degenerated_ = true;
    return;
  }

  const Eigen::Vector3d evals = es.eigenvalues();
  const double lambda_max = std::max(1e-6, evals[2]);
  const double lambda_min = std::max(0.0, evals[0]);
  lidar_constraint_ratio_ = lambda_min / lambda_max;
  // Legacy detector is retained only when observability diagnostics are disabled.
  lidar_degenerated_ = lidar_constraint_ratio_ < 0.10;
}

bool VoxelMapManager::classifyLidarDegeneracy(
    const fast_livo::LioObservabilityMetrics &metrics,
    int effective_features) const
{
  if (effective_features < config_setting_.degeneracy_min_effective_features || !metrics.valid)
    return true;
  if (config_setting_.degeneracy_min_translation_eigenvalue > 0.0 &&
      metrics.translation_eigenvalues[0] < config_setting_.degeneracy_min_translation_eigenvalue)
    return true;
  if (metrics.translation_eigenvalue_ratio < config_setting_.degeneracy_ratio_thresh)
    return true;
  return !std::isfinite(metrics.translation_condition_number) ||
         metrics.translation_condition_number >
             config_setting_.degeneracy_max_translation_condition_number;
}

void VoxelMapManager::updateLidarDegeneracyHysteresis(bool raw_degenerate)
{
  const bool was_degenerated = lidar_degenerated_;
  if (raw_degenerate)
  {
    ++degeneracy_bad_frame_count_;
    degeneracy_good_frame_count_ = 0;
    if (!lidar_degenerated_ &&
        degeneracy_bad_frame_count_ >= config_setting_.degeneracy_enter_consecutive_frames)
      lidar_degenerated_ = true;
  }
  else
  {
    ++degeneracy_good_frame_count_;
    degeneracy_bad_frame_count_ = 0;
    if (lidar_degenerated_ &&
        degeneracy_good_frame_count_ >= config_setting_.degeneracy_exit_consecutive_frames)
      lidar_degenerated_ = false;
  }
  if (!was_degenerated && lidar_degenerated_)
    ROS_WARN("[LIO_DEGEN] state NORMAL -> DEGRADED");
  else if (was_degenerated && !lidar_degenerated_)
    ROS_INFO("[LIO_DEGEN] state DEGRADED -> NORMAL");
}

void VoxelMapManager::updateDirectionGuardHysteresis(bool conflict)
{
  if (!config_setting_.observability_diagnostics_enable ||
      config_setting_.direction_guard_mode == "off")
  {
    direction_conflict_frame_count_ = 0;
    direction_clear_frame_count_ = 0;
    direction_guard_active_ = false;
    return;
  }

  const bool was_active = direction_guard_active_;
  if (conflict)
  {
    ++direction_conflict_frame_count_;
    direction_clear_frame_count_ = 0;
    if (!direction_guard_active_ &&
        direction_conflict_frame_count_ >= config_setting_.direction_guard_enter_consecutive_frames)
      direction_guard_active_ = true;
  }
  else
  {
    ++direction_clear_frame_count_;
    direction_conflict_frame_count_ = 0;
    if (direction_guard_active_ &&
        direction_clear_frame_count_ >= config_setting_.direction_guard_exit_consecutive_frames)
      direction_guard_active_ = false;
  }
  if (!was_active && direction_guard_active_)
    ROS_WARN("[LIO_DIRECTION] conflict started (diagnostic only; state unchanged)");
  else if (was_active && !direction_guard_active_)
    ROS_INFO("[LIO_DIRECTION] conflict cleared (diagnostic only; state unchanged)");
}

bool VoxelMapManager::isLidarDegenerated() const
{
  return lidar_degenerated_;
}

double VoxelMapManager::getLidarConstraintRatio() const
{
  return lidar_constraint_ratio_;
}

const LioUpdateDiagnostics &VoxelMapManager::getLastLioDiagnostics() const
{
  return last_lio_diagnostics_;
}

void VoxelMapManager::markLastLioMapInserted(bool inserted)
{
  last_lio_diagnostics_.map_inserted = inserted;
}

double VoxelMapManager::p4RelativeTime(double timestamp)
{
  if (!std::isfinite(p4_first_timestamp_s_)) p4_first_timestamp_s_ = timestamp;
  return timestamp - p4_first_timestamp_s_;
}

bool VoxelMapManager::p4InAnalysisWindow(double timestamp)
{
  const double relative = p4RelativeTime(timestamp);
  return relative >= config_setting_.p4_analysis_start_s &&
         relative <= config_setting_.p4_analysis_end_s;
}

bool VoxelMapManager::p4InDetailWindow(double timestamp)
{
  const double relative = p4RelativeTime(timestamp);
  return relative >= config_setting_.p4_detail_start_s &&
         relative <= config_setting_.p4_detail_end_s;
}

void VoxelMapManager::p4InitializeOutput()
{
  if (p4_output_initialized_ ||
      !config_setting_.p4_frontend_diagnostics_enable)
    return;
  p4_output_initialized_ = true;
  if (config_setting_.p4_frontend_output_directory.empty())
  {
    ROS_WARN("[P4_FRONTEND] output_directory is empty; runtime counterfactual remains active but CSV logging is disabled");
    return;
  }
  std::error_code error;
  std::filesystem::create_directories(
      config_setting_.p4_frontend_output_directory, error);
  if (error)
  {
    ROS_ERROR("[P4_FRONTEND] cannot create output directory '%s': %s",
              config_setting_.p4_frontend_output_directory.c_str(),
              error.message().c_str());
    return;
  }
  const std::string prefix =
      config_setting_.p4_frontend_output_directory + "/p4_frontend_";
  p4_iteration_csv_.open(prefix + "iterations.csv", std::ios::out);
  p4_correspondence_csv_.open(prefix + "correspondences.csv", std::ios::out);
  p4_map_csv_.open(prefix + "map.csv", std::ios::out);
  p4_deskew_csv_.open(prefix + "deskew.csv", std::ios::out);
  if (p4_iteration_csv_.is_open())
    p4_iteration_csv_ << "timestamp_s,relative_time_s,frame_id,iteration,correspondences,previous_correspondences,retained_ratio,added_ratio,removed_ratio,changed_plane_ratio,voxel_jaccard,mean_normal_cosine,sign_flip_ratio,signed_mean_m,signed_median_m,weighted_signed_mean_m,positive_ratio,negative_ratio,weak_signed_projection_m,normal_eval_0,normal_eval_1,normal_eval_2,normal_entropy,normal_octant_coverage,translation_eval_0,translation_eval_1,translation_eval_2,conditional_eval_0,conditional_eval_1,conditional_eval_2,weak_x,weak_y,weak_z,weak_step_m,age_lt_0_5_ratio,age_0_5_2_ratio,age_2_5_ratio,age_5_20_ratio,age_gt_20_ratio,support_recent_1_ratio,support_recent_3_ratio,support_recent_5_ratio,recent_voxel_1_ratio,recent_voxel_3_ratio,recent_voxel_5_ratio,source_distance_mean_m,plane_center_shift_mean_m,plane_normal_change_mean_deg,fixed_rank,fixed_dr_x,fixed_dr_y,fixed_dr_z,fixed_dp_x,fixed_dp_y,fixed_dp_z,u2_correspondences,u2_plane_jaccard,u2_signed_mean_m,u2_fixed_weak_dp_m\n";
  if (p4_correspondence_csv_.is_open())
    p4_correspondence_csv_ << "timestamp_s,relative_time_s,frame_id,iteration,point_index,raw_x,raw_y,raw_z,deskewed_x,deskewed_y,deskewed_z,world_x,world_y,world_z,voxel_x,voxel_y,voxel_z,plane_id,plane_center_x,plane_center_y,plane_center_z,normal_x,normal_y,normal_z,plane_eval_min,plane_eval_mid,plane_eval_max,support_count,residual_m,variance_m2,normalized_residual,j_rot_x,j_rot_y,j_rot_z,j_pos_x,j_pos_y,j_pos_z,distance_to_center_m,plane_age_min_s,plane_age_max_s,support_recent_1_ratio,support_recent_3_ratio,support_recent_5_ratio,changed_from_previous,plane_update_count,plane_center_shift_m,plane_normal_change_deg,support_frames\n";
  if (p4_map_csv_.is_open())
    p4_map_csv_ << "timestamp_s,relative_time_s,frame_id,variant,inserted,reason,predicted_x,predicted_y,predicted_z,inserted_x,inserted_y,inserted_z,insertion_delta_m,known_updated_root_voxels\n";
  if (p4_deskew_csv_.is_open())
    p4_deskew_csv_ << "timestamp_s,relative_time_s,frame_id,point_count,imu_count,scan_begin_s,scan_end_s,point_offset_min_s,point_offset_max_s,point_time_monotonic,imu_begin_s,imu_end_s,propagation_begin_s,propagation_end_s,imu_covers_propagation,seed_vx,seed_vy,seed_vz,end_vx,end_vy,end_vz,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,g_x,g_y,g_z,fixed_velocity_point_delta_mean_m,fixed_velocity_point_delta_max_m\n";
}

void VoxelMapManager::p4RecordIteration(
    double timestamp, int iteration,
    const StatesGroup &iteration_state_before,
    const Eigen::MatrixXd &jacobian,
    const Eigen::VectorXd &inverse_variance,
    const fast_livo::LioObservabilityMetrics &observability)
{
  if (!p4_iteration_csv_.is_open()) return;
  std::vector<fast_livo::p4::Association> associations;
  std::vector<double> residuals;
  std::vector<Eigen::Vector3d> normals;
  std::vector<int> last_update_frames;
  associations.reserve(ptpl_list_.size());
  residuals.reserve(ptpl_list_.size());
  normals.reserve(ptpl_list_.size());
  last_update_frames.reserve(ptpl_list_.size());
  std::array<std::size_t, 5> age_counts{{0, 0, 0, 0, 0}};
  std::array<std::size_t, 3> recent_support{{0, 0, 0}};
  std::size_t total_support = 0;
  double source_distance_sum = 0.0;
  std::size_t source_distance_count = 0;
  double center_shift_sum = 0.0;
  double normal_change_sum = 0.0;
  std::unordered_map<int, const fast_livo::p4::Association *> previous_by_point;
  for (const auto &value : p4_previous_associations_)
    previous_by_point[value.point_index] = &value;

  for (const PointToPlane &ptpl : ptpl_list_)
  {
    fast_livo::p4::Association association;
    association.point_index = ptpl.point_index_;
    association.plane_id = ptpl.plane_id_;
    association.voxel_x = ptpl.voxel_x_;
    association.voxel_y = ptpl.voxel_y_;
    association.voxel_z = ptpl.voxel_z_;
    association.normal = ptpl.normal_;
    association.residual = ptpl.dis_to_plane_;
    associations.push_back(association);
    residuals.push_back(ptpl.dis_to_plane_);
    normals.push_back(ptpl.normal_);
    const VOXEL_LOCATION voxel(ptpl.voxel_x_, ptpl.voxel_y_, ptpl.voxel_z_);
    const auto updated = p4_voxel_last_update_frame_.find(voxel);
    last_update_frames.push_back(
        updated == p4_voxel_last_update_frame_.end() ? -1 : updated->second);
    if (!ptpl.source_plane_) continue;
    const VoxelPlane &plane = *ptpl.source_plane_;
    center_shift_sum += plane.p4_last_center_shift_m_;
    normal_change_sum += plane.p4_last_normal_change_deg_;
    for (std::size_t source = 0;
         source < plane.p4_source_frame_ids_.size(); ++source)
    {
      const int frame_age = current_frame_id_ - plane.p4_source_frame_ids_[source];
      if (frame_age > 0 && frame_age <= 1) ++recent_support[0];
      if (frame_age > 0 && frame_age <= 3) ++recent_support[1];
      if (frame_age > 0 && frame_age <= 5) ++recent_support[2];
      if (source < plane.p4_source_timestamps_s_.size())
        ++age_counts[fast_livo::p4::ageBin(
            timestamp - plane.p4_source_timestamps_s_[source])];
      if (source < plane.p4_source_origins_w_.size() &&
          plane.p4_source_origins_w_[source].allFinite())
      {
        source_distance_sum +=
            (iteration_state_before.pos_end -
             plane.p4_source_origins_w_[source]).norm();
        ++source_distance_count;
      }
      ++total_support;
    }
  }
  const auto churn = fast_livo::p4::correspondenceChurn(
      p4_previous_associations_, associations);
  const std::vector<double> weights(
      inverse_variance.data(), inverse_variance.data() + inverse_variance.size());
  const auto signed_metrics =
      fast_livo::p4::signedResidualMetrics(residuals, weights);
  const auto normal_geometry = fast_livo::p4::normalGeometry(normals);
  const auto voxel_reuse = fast_livo::p4::recentReuseRatios(
      last_update_frames, current_frame_id_);
  const Eigen::Vector3d weak = observability.valid
      ? observability.weak_translation_direction_world
      : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  const double weak_signed =
      fast_livo::p4::signedResidualWeakProjection(residuals, normals, weak);
  const Eigen::MatrixXd translation_jacobian =
      jacobian.block(0, 3, jacobian.rows(), 3);
  const Eigen::Vector3d raw_translation_eigenvalues = symmetricEigenvalues(
      translation_jacobian.transpose() * inverse_variance.asDiagonal() *
      translation_jacobian);
  const auto fixed = fast_livo::p4::solveFixedCorrespondences(
      jacobian, Eigen::Map<const Eigen::VectorXd>(residuals.data(), residuals.size()),
      inverse_variance);
  int u2_correspondence_count = -1;
  double u2_plane_jaccard = std::numeric_limits<double>::quiet_NaN();
  double u2_signed_mean = std::numeric_limits<double>::quiet_NaN();
  double u2_fixed_weak_dp = std::numeric_limits<double>::quiet_NaN();
  if (iteration == 1 && p4InDetailWindow(timestamp) &&
      current_frame_id_ % config_setting_.p4_counterfactual_frame_stride == 0 &&
      p4_fixed_velocity_down_body_ &&
      !p4_fixed_velocity_down_body_->empty())
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr u2_world(
        new pcl::PointCloud<pcl::PointXYZI>());
    TransformLidar(iteration_state_before.rot_end,
                   iteration_state_before.pos_end,
                   p4_fixed_velocity_down_body_, u2_world);
    std::vector<pointWithVar> u2_points(p4_fixed_velocity_down_body_->size());
    for (std::size_t i = 0; i < u2_points.size(); ++i)
    {
      V3D body(p4_fixed_velocity_down_body_->points[i].x,
               p4_fixed_velocity_down_body_->points[i].y,
               p4_fixed_velocity_down_body_->points[i].z);
      if (body.z() == 0.0) body.z() = 0.001;
      M3D body_cov;
      calcBodyCov(body, config_setting_.dept_err_, config_setting_.beam_err_,
                  body_cov);
      u2_points[i].point_b = body;
      u2_points[i].point_w << u2_world->points[i].x, u2_world->points[i].y,
          u2_world->points[i].z;
      u2_points[i].body_var = body_cov;
      u2_points[i].source_point_index = static_cast<int>(i);
      u2_points[i].var = iteration_state_before.rot_end * body_cov *
                            iteration_state_before.rot_end.transpose() +
                        iteration_state_before.cov.block<3, 3>(3, 3);
    }
    std::vector<PointToPlane> u2_planes;
    BuildResidualListOMP(u2_points, u2_planes);
    u2_correspondence_count = static_cast<int>(u2_planes.size());
    if (!u2_planes.empty())
    {
      Eigen::MatrixXd u2_jacobian(u2_planes.size(), 6);
      Eigen::VectorXd u2_residual(u2_planes.size());
      Eigen::VectorXd u2_inverse_variance(u2_planes.size());
      std::set<int> production_planes;
      std::set<int> alternate_planes;
      for (const PointToPlane &value : ptpl_list_)
        production_planes.insert(value.plane_id_);
      for (std::size_t i = 0; i < u2_planes.size(); ++i)
      {
        const PointToPlane &value = u2_planes[i];
        alternate_planes.insert(value.plane_id_);
        const V3D point_imu = extR_ * value.point_b_ + extT_;
        M3D cross;
        cross << SKEW_SYM_MATRX(point_imu);
        const V3D rotation_column =
            cross * iteration_state_before.rot_end.transpose() * value.normal_;
        u2_jacobian.row(i) << rotation_column.transpose(),
            value.normal_.transpose();
        u2_residual[i] = value.dis_to_plane_;
        const V3D point_world =
            iteration_state_before.rot_end * point_imu +
            iteration_state_before.pos_end;
        Eigen::Matrix<double, 1, 6> plane_jacobian;
        plane_jacobian << (point_world - value.center_).transpose(),
            -value.normal_.transpose();
        const M3D world_body_cov =
            iteration_state_before.rot_end * extR_ * value.body_cov_ *
            (iteration_state_before.rot_end * extR_).transpose();
        const double variance = 0.001 +
            (plane_jacobian * value.plane_var_ *
             plane_jacobian.transpose())(0, 0) +
            (value.normal_.transpose() * world_body_cov * value.normal_)(0, 0);
        u2_inverse_variance[i] = variance > 0.0 ? 1.0 / variance : 0.0;
      }
      std::size_t plane_intersection = 0;
      for (int plane_id : production_planes)
        plane_intersection += alternate_planes.count(plane_id);
      const std::size_t plane_union = production_planes.size() +
          alternate_planes.size() - plane_intersection;
      u2_plane_jaccard = plane_union
          ? static_cast<double>(plane_intersection) / plane_union : 1.0;
      const auto u2_signed = fast_livo::p4::signedResidualMetrics(
          std::vector<double>(u2_residual.data(),
                              u2_residual.data() + u2_residual.size()),
          std::vector<double>(u2_inverse_variance.data(),
                              u2_inverse_variance.data() +
                                  u2_inverse_variance.size()));
      u2_signed_mean = u2_signed.mean;
      const auto u2_fixed = fast_livo::p4::solveFixedCorrespondences(
          u2_jacobian, u2_residual, u2_inverse_variance);
      if (u2_fixed.valid && weak.allFinite())
        u2_fixed_weak_dp = u2_fixed.delta.tail<3>().dot(weak);
    }
  }
  const double changed_plane_ratio = churn.retained_count
      ? static_cast<double>(churn.changed_plane_count) / churn.retained_count
      : 0.0;
  const double sign_flip_ratio = churn.retained_count
      ? static_cast<double>(churn.residual_sign_flip_count) / churn.retained_count
      : 0.0;
  const double weak_step = weak.allFinite()
      ? (state_.pos_end - iteration_state_before.pos_end).dot(weak)
      : std::numeric_limits<double>::quiet_NaN();
  p4_iteration_csv_ << std::setprecision(17)
      << timestamp << ',' << p4RelativeTime(timestamp) << ','
      << current_frame_id_ << ',' << iteration << ',' << associations.size()
      << ',' << churn.previous_count << ',' << churn.retained_ratio << ','
      << churn.added_ratio << ',' << churn.removed_ratio << ','
      << changed_plane_ratio << ',' << churn.voxel_jaccard << ','
      << churn.mean_normal_cosine << ',' << sign_flip_ratio << ','
      << signed_metrics.mean << ',' << signed_metrics.median << ','
      << signed_metrics.weighted_mean << ',' << signed_metrics.positive_ratio
      << ',' << signed_metrics.negative_ratio << ',' << weak_signed << ','
      << normal_geometry.eigenvalues[0] << ','
      << normal_geometry.eigenvalues[1] << ','
      << normal_geometry.eigenvalues[2] << ','
      << normal_geometry.directional_entropy << ','
      << normal_geometry.spherical_coverage << ','
      << raw_translation_eigenvalues[0] << ','
      << raw_translation_eigenvalues[1] << ','
      << raw_translation_eigenvalues[2] << ','
      << observability.translation_eigenvalues[0] << ','
      << observability.translation_eigenvalues[1] << ','
      << observability.translation_eigenvalues[2] << ','
      << weak.x() << ',' << weak.y() << ',' << weak.z() << ',' << weak_step;
  for (std::size_t count : age_counts)
    p4_iteration_csv_ << ',' << (total_support
        ? static_cast<double>(count) / total_support : 0.0);
  for (std::size_t count : recent_support)
    p4_iteration_csv_ << ',' << (total_support
        ? static_cast<double>(count) / total_support : 0.0);
  p4_iteration_csv_ << ',' << voxel_reuse[0] << ',' << voxel_reuse[1] << ','
      << voxel_reuse[2] << ','
      << (source_distance_count ? source_distance_sum / source_distance_count : 0.0)
      << ',' << (associations.empty() ? 0.0 : center_shift_sum / associations.size())
      << ',' << (associations.empty() ? 0.0 : normal_change_sum / associations.size())
      << ',' << fixed.rank;
  for (int index = 0; index < 6; ++index)
    p4_iteration_csv_ << ',' << fixed.delta[index];
  p4_iteration_csv_ << ',' << u2_correspondence_count << ','
      << u2_plane_jaccard << ',' << u2_signed_mean << ','
      << u2_fixed_weak_dp << '\n';

  if (p4_correspondence_csv_.is_open() && p4InDetailWindow(timestamp) &&
      current_frame_id_ % config_setting_.p4_detail_frame_stride == 0)
  {
    for (std::size_t i = 0; i < ptpl_list_.size(); ++i)
    {
      const PointToPlane &ptpl = ptpl_list_[i];
      const VoxelPlane *plane = ptpl.source_plane_;
      double minimum_age = std::numeric_limits<double>::infinity();
      double maximum_age = -std::numeric_limits<double>::infinity();
      std::array<std::size_t, 3> recent{{0, 0, 0}};
      std::size_t support = 0;
      std::ostringstream source_frames;
      if (plane)
      {
        for (std::size_t source = 0;
             source < plane->p4_source_frame_ids_.size(); ++source)
        {
          if (source) source_frames << ';';
          source_frames << plane->p4_source_frame_ids_[source];
          const int frame_age =
              current_frame_id_ - plane->p4_source_frame_ids_[source];
          if (frame_age > 0 && frame_age <= 1) ++recent[0];
          if (frame_age > 0 && frame_age <= 3) ++recent[1];
          if (frame_age > 0 && frame_age <= 5) ++recent[2];
          if (source < plane->p4_source_timestamps_s_.size())
          {
            const double age = timestamp - plane->p4_source_timestamps_s_[source];
            minimum_age = std::min(minimum_age, age);
            maximum_age = std::max(maximum_age, age);
          }
          ++support;
        }
      }
      const auto previous = previous_by_point.find(ptpl.point_index_);
      const bool changed = previous != previous_by_point.end() &&
          (previous->second->plane_id != ptpl.plane_id_ ||
           previous->second->voxel_x != ptpl.voxel_x_ ||
           previous->second->voxel_y != ptpl.voxel_y_ ||
           previous->second->voxel_z != ptpl.voxel_z_);
      const double variance = i < static_cast<std::size_t>(inverse_variance.size()) &&
              inverse_variance[i] > 0.0
          ? 1.0 / inverse_variance[i]
          : std::numeric_limits<double>::quiet_NaN();
      p4_correspondence_csv_ << std::setprecision(17)
          << timestamp << ',' << p4RelativeTime(timestamp) << ','
          << current_frame_id_ << ',' << iteration << ','
          << ptpl.point_index_ << ',' << ptpl.raw_point_.x() << ','
          << ptpl.raw_point_.y() << ',' << ptpl.raw_point_.z() << ','
          << ptpl.point_b_.x() << ',' << ptpl.point_b_.y() << ','
          << ptpl.point_b_.z() << ',' << ptpl.point_w_.x() << ','
          << ptpl.point_w_.y() << ',' << ptpl.point_w_.z() << ','
          << ptpl.voxel_x_ << ',' << ptpl.voxel_y_ << ',' << ptpl.voxel_z_
          << ',' << ptpl.plane_id_ << ',' << ptpl.center_.x() << ','
          << ptpl.center_.y() << ',' << ptpl.center_.z() << ','
          << ptpl.normal_.x() << ',' << ptpl.normal_.y() << ','
          << ptpl.normal_.z() << ','
          << (plane ? plane->min_eigen_value_ : 0.0) << ','
          << (plane ? plane->mid_eigen_value_ : 0.0) << ','
          << (plane ? plane->max_eigen_value_ : 0.0) << ',' << support << ','
          << ptpl.dis_to_plane_ << ',' << variance << ','
          << (variance > 0.0 ? ptpl.dis_to_plane_ / std::sqrt(variance) : 0.0);
      for (int column = 0; column < 6; ++column)
        p4_correspondence_csv_ << ',' << jacobian(i, column);
      p4_correspondence_csv_ << ',' << (ptpl.point_w_ - ptpl.center_).norm()
          << ',' << (std::isfinite(minimum_age) ? minimum_age : -1.0)
          << ',' << (std::isfinite(maximum_age) ? maximum_age : -1.0);
      for (std::size_t count : recent)
        p4_correspondence_csv_ << ',' << (support
            ? static_cast<double>(count) / support : 0.0);
      p4_correspondence_csv_ << ',' << changed << ','
          << (plane ? plane->p4_update_count_ : 0) << ','
          << (plane ? plane->p4_last_center_shift_m_ : 0.0) << ','
          << (plane ? plane->p4_last_normal_change_deg_ : 0.0) << ",\""
          << source_frames.str() << "\"\n";
    }
  }
  p4_previous_associations_ = std::move(associations);
}

bool VoxelMapManager::p4MapMutationAllowed(double timestamp)
{
  return fast_livo::p4::mapInsertionAllowed(
      config_setting_.p4_frontend_diagnostics_enable,
      config_setting_.p4_frontend_variant == "frozen_map",
      p4RelativeTime(timestamp), config_setting_.p4_frozen_map_start_s);
}

void VoxelMapManager::p4SetDeskewDiagnostics(
    const fast_livo::p4::DeskewDiagnostics &value)
{
  p4_deskew_diagnostics_ = value;
}

void VoxelMapManager::p4RecordMapDecision(
    double timestamp, bool inserted, const std::string &reason,
    const Eigen::Vector3d &predicted_position,
    const Eigen::Vector3d &inserted_position)
{
  if (!config_setting_.p4_frontend_diagnostics_enable) return;
  p4InitializeOutput();
  if (!p4_map_csv_.is_open() || !p4InAnalysisWindow(timestamp)) return;
  p4_map_csv_ << std::setprecision(17) << timestamp << ','
      << p4RelativeTime(timestamp) << ',' << current_frame_id_ << ','
      << config_setting_.p4_frontend_variant << ',' << inserted << ','
      << reason << ',' << predicted_position.x() << ','
      << predicted_position.y() << ',' << predicted_position.z() << ','
      << inserted_position.x() << ',' << inserted_position.y() << ','
      << inserted_position.z() << ','
      << (inserted_position - predicted_position).norm() << ','
      << p4_voxel_last_update_frame_.size() << '\n';
}
int p4bGetVoxelPlaneIdCounter() { return voxel_plane_id; }
void p4bSetVoxelPlaneIdCounter(int value)
{
  voxel_plane_id = std::max(0, value);
}
