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
#include <limits>
#include <sstream>
#include <unordered_set>

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
}

void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
  plane->covariance_ = Eigen::Matrix3d::Zero();
  plane->center_ = Eigen::Vector3d::Zero();
  plane->normal_ = Eigen::Vector3d::Zero();
  plane->points_size_ = points.size();
  plane->radius_ = 0;
  for (auto pv : points)
  {
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
    plane->center_ += pv.point_w;
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
    plane->is_update_ = true;
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

void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{
  ++current_frame_id_;
  last_lio_diagnostics_ = LioUpdateDiagnostics();
  last_lio_diagnostics_.predicted_state = state_propagat;
  last_lio_diagnostics_.input_feature_count =
      feats_undistort_ ? static_cast<int>(feats_undistort_->size()) : 0;
  last_lio_diagnostics_.downsampled_feature_count = feats_down_size_;

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
  bool frame_observability_initialized = false;
  fast_livo::LioObservabilityMetrics frame_observability;

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
    if (!raw_pose_information.allFinite() || !raw_pose_rhs.allFinite() || !state_.cov.allFinite())
    {
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
      ROS_ERROR_THROTTLE(1.0, "[LIO_NUMERIC] Non-finite original ESIKF increment; skipping this scan update.");
      break;
    }

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

    state_ += solution;

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
      state_.cov.block<DIM_STATE, DIM_STATE>(0, 0) =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) - G.block<DIM_STATE, DIM_STATE>(0, 0)) * state_.cov.block<DIM_STATE, DIM_STATE>(0, 0);
      last_lio_diagnostics_.directional_shadows = current_iteration_shadows;
      last_lio_diagnostics_.valid_update = true;
      // total_distance += (_state.pos_end - position_last).norm();
      position_last_ = state_.pos_end;
      geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
      EKF_stop_flg = true;
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
  if (!config_setting_.observability_diagnostics_enable) updateLidarDegeneracyStatus();
  last_lio_diagnostics_.is_degenerate = lidar_degenerated_;
  last_lio_diagnostics_.updated_state = state_;
  if (last_lio_diagnostics_.observability.valid)
  {
    const V3D weak = last_lio_diagnostics_.observability.weak_translation_direction_world;
    last_lio_diagnostics_.velocity_projection_on_weak_direction = state_propagat.vel_end.dot(weak);
    last_lio_diagnostics_.position_correction_on_weak_direction =
        (state_.pos_end - state_propagat.pos_end).dot(weak);
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

void VoxelMapManager::BuildVoxelMap()
{
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
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) { voxel_map_[position]->UpdateOctoTree(p_v); }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
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
  current_frame_id_ = 0;
  scan_count = 0;
  last_slide_position = V3D::Zero();
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
        if (near_octo != nullptr) { build_single_residual(pv, near_octo, 0, is_sucess, prob, single_ptpl); }
      }
      if (is_sucess)
      {
        mylock.lock();
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
  if (current_octo->plane_ptr_->is_plane_)
  {
    VoxelPlane &plane = *current_octo->plane_ptr_;
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
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          single_ptpl.plane_var_ = plane.plane_var_;
          single_ptpl.normal_ = plane.normal_;
          single_ptpl.center_ = plane.center_;
          single_ptpl.d_ = plane.d_;
          single_ptpl.layer_ = current_layer;
          single_ptpl.eigen_value_ = plane.min_eigen_value_;
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
