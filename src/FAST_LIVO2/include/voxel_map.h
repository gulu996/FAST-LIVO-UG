/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include "lio_update_transaction.h"
#include "lio_degeneracy.h"
#include "lio_motion_consistency.h"
#include "p4_frontend_diagnostics.h"
#include "p5_seed_basin.h"
#include <Eigen/Dense>
#include <array>
#include <fstream>
#include <memory>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

extern int voxel_plane_id;
int p4bGetVoxelPlaneIdCounter();
void p4bSetVoxelPlaneIdCounter(int value);

typedef struct VoxelMapConfig
{
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  std::vector<int> layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double sigma_num_;
  bool is_pub_plane_map_;

  // config of local map sliding
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;

  // config of long-term sparse visual map
  bool long_term_visual_map_en;
  int long_term_visual_max_voxels;

  // Whole-frame LiDAR pose observability; unrelated to lio/min_eigen_value,
  // which only decides whether one voxel represents a plane.
  bool observability_diagnostics_enable;
  bool state_intervention_enable;
  bool use_conditional_translation_information;
  double degeneracy_rotation_regularization;
  int degeneracy_min_effective_features;
  double degeneracy_min_translation_eigenvalue;
  double degeneracy_max_translation_condition_number;
  double degeneracy_ratio_thresh;
  int degeneracy_enter_consecutive_frames;
  int degeneracy_exit_consecutive_frames;
  bool directional_shadow_enable;
  std::vector<double> directional_shadow_relative_thresholds;

  std::string direction_guard_mode;
  double direction_guard_min_predicted_speed_mps;
  double direction_guard_min_velocity_weak_direction_cos;
  double direction_guard_max_opposite_correction_m;
  double direction_guard_max_opposite_velocity_correction_mps;
  int direction_guard_enter_consecutive_frames;
  int direction_guard_exit_consecutive_frames;

  std::string map_guard_mode;
  bool map_guard_freeze_on_degeneracy;
  bool map_guard_freeze_on_direction_reject;
  double map_guard_severe_translation_eigenvalue_ratio;
  int map_guard_recovery_consecutive_frames;
  int map_guard_maximum_freeze_frames;

  // config of adaptive ICP early-stop
  int icp_min_iterations;
  double icp_early_stop_residual_ratio;

  // config of ICP stability guard
  double icp_max_rot_step_deg;
  double icp_max_trans_step_m;

  bool deterministic_lio_update_en;

  // Frame-level measurement transaction validation. These are broad safety
  // bounds, not dataset-accuracy tuning knobs.
  int commit_minimum_correspondences;
  double commit_maximum_cost_ratio;
  double commit_maximum_cost_increase;
  double commit_maximum_residual;
  double commit_covariance_symmetry_relative_tolerance;
  double commit_covariance_psd_relative_tolerance;

  // P4-A is default-off and may only observe or run recorded offline
  // counterfactuals. None of these values is consumed by the production
  // measurement update.
  bool p4_frontend_diagnostics_enable;
  std::string p4_frontend_variant;
  std::string p4_frontend_output_directory;
  double p4_analysis_start_s;
  double p4_analysis_end_s;
  double p4_detail_start_s;
  double p4_detail_end_s;
  double p4_frozen_map_start_s;
  int p4_detail_frame_stride;
  int p4_counterfactual_frame_stride;

  // P4-B diagnostic harness. Default-off; support retention is required for
  // point-level exclusion before plane refitting.
  bool p4b_snapshot_enable;
  bool p4b_retain_support_points;

  // P5 is default-off. Diagnostic mode is strictly read-only; the separately
  // enabled prototype commits exactly one of its three candidate snapshots.
  bool p5_seed_basin_enable;
  bool p5_weak_axis_multistart_enable;
  std::string p5_seed_basin_output_directory;
  double p5_seed_basin_start_s;
  double p5_seed_basin_end_s;
  int p5_seed_basin_frame_stride;
} VoxelMapConfig;

struct VoxelPlane;

typedef struct PointToPlane
{
  int point_index_ = -1;
  Eigen::Vector3d raw_point_ = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
  int plane_id_ = -1;
  int64_t voxel_x_ = 0;
  int64_t voxel_y_ = 0;
  int64_t voxel_z_ = 0;
  const VoxelPlane *source_plane_ = nullptr;
} PointToPlane;

struct LioUpdateDiagnostics
{
  // Frozen final point-to-plane normal equation. This shadow interface is
  // never read by the production estimator.
  bool lidar_geometry_valid = false;
  StatesGroup lidar_geometry_linearization_state;
  fast_livo::Matrix6d lidar_geometry_information =
      fast_livo::Matrix6d::Zero();
  Eigen::Matrix<double, 6, 1> lidar_geometry_rhs =
      Eigen::Matrix<double, 6, 1>::Zero();
  double lidar_geometry_residual_weighted_energy =
      std::numeric_limits<double>::quiet_NaN();
  int lidar_geometry_residual_dof = 0;
  bool valid_update = false;
  bool commit = false;
  bool map_inserted = false;
  std::string convergence_status = "rejected_invalid";
  int iteration_count = 0;
  bool converged = false;
  bool reached_iteration_limit = false;
  int correspondence_count = 0;
  int valid_residual_count = 0;
  double cost_before = std::numeric_limits<double>::quiet_NaN();
  double cost_after = std::numeric_limits<double>::quiet_NaN();
  double translation_increment_norm = std::numeric_limits<double>::quiet_NaN();
  double rotation_increment_deg = std::numeric_limits<double>::quiet_NaN();
  double velocity_increment_norm = std::numeric_limits<double>::quiet_NaN();
  double covariance_trace_before = std::numeric_limits<double>::quiet_NaN();
  double covariance_trace_after = std::numeric_limits<double>::quiet_NaN();
  double covariance_min_diagonal = std::numeric_limits<double>::quiet_NaN();
  double covariance_max_diagonal = std::numeric_limits<double>::quiet_NaN();
  double covariance_min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double covariance_asymmetry = std::numeric_limits<double>::quiet_NaN();
  int input_feature_count = 0;
  int downsampled_feature_count = 0;
  int effective_feature_count = 0;
  int valid_plane_count = 0;
  int observability_feature_count = 0;
  double inlier_ratio = 0.0;
  double average_point_plane_residual = 0.0;
  double median_abs_point_plane_residual = 0.0;
  double p90_abs_point_plane_residual = 0.0;
  double p95_abs_point_plane_residual = 0.0;
  double point_plane_residual_rmse = 0.0;
  double max_abs_point_plane_residual = 0.0;
  double measurement_variance_mean = 0.0;
  double measurement_variance_median = 0.0;
  double measurement_variance_p90 = 0.0;
  double measurement_variance_p95 = 0.0;
  double measurement_variance_min = 0.0;
  double measurement_variance_max = 0.0;
  StatesGroup predicted_state;
  StatesGroup candidate_state;
  StatesGroup updated_state;
  fast_livo::LioObservabilityMetrics observability;
  Eigen::Vector3d translation_information_weights = Eigen::Vector3d::Ones();
  Eigen::Vector3d raw_position_correction = Eigen::Vector3d::Zero();
  Eigen::Vector3d raw_velocity_correction = Eigen::Vector3d::Zero();
  double predicted_speed_mps = 0.0;
  double velocity_weak_direction_cos = 0.0;
  double velocity_projection_on_weak_direction = 0.0;
  double position_correction_on_weak_direction = 0.0;
  bool raw_is_degenerate = false;
  bool is_degenerate = false;
  bool direction_conflict = false;
  int direction_conflict_consecutive_frames = 0;
  bool direction_guard_triggered = false;
  bool state_intervention_applied = false;
  bool update_was_suppressed = false;
  bool update_was_significantly_suppressed = false;
  bool is_severely_degenerate = false;

  struct DirectionalShadow
  {
    bool valid = false;
    int iteration_index = -1;
    int iteration_count = 0;
    int effective_feature_count = 0;
    double relative_threshold = 0.0;
    double residual_mean = 0.0;
    double residual_median = 0.0;
    double residual_p90 = 0.0;
    double residual_rmse = 0.0;
    double measurement_variance_mean = 0.0;
    double measurement_variance_median = 0.0;
    fast_livo::LioObservabilityMetrics observability;
    Eigen::Vector3d rotation_weights = Eigen::Vector3d::Ones();
    Eigen::Vector3d translation_weights = Eigen::Vector3d::Ones();
    int affected_direction_count = 0;
    int partial_suppression_count = 0;
    int full_suppression_count = 0;
    double information_trace_raw = 0.0;
    double information_trace_shadow = 0.0;
    double information_trace_retained_ratio = 1.0;
    double rotation_information_trace_raw = 0.0;
    double rotation_information_trace_shadow = 0.0;
    double translation_information_trace_raw = 0.0;
    double translation_information_trace_shadow = 0.0;
    Eigen::Vector3d raw_delta_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d shadow_delta_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d removed_delta_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_delta_rpy_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d shadow_delta_rpy_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d removed_delta_rpy_deg = Eigen::Vector3d::Zero();
    double raw_delta_position_norm = 0.0;
    double shadow_delta_position_norm = 0.0;
    double removed_delta_position_norm = 0.0;
    double raw_delta_rotation_deg = 0.0;
    double shadow_delta_rotation_deg = 0.0;
    double removed_delta_rotation_deg = 0.0;
    double raw_weak_translation_projection = 0.0;
    double shadow_weak_translation_projection = 0.0;
    double raw_weak_rotation_projection_deg = 0.0;
    double shadow_weak_rotation_projection_deg = 0.0;
    double raw_posterior_pose_cov_trace = 0.0;
    double shadow_posterior_pose_cov_trace = 0.0;
    Eigen::Vector3d raw_posterior_rotation_cov_eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Vector3d shadow_posterior_rotation_cov_eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_posterior_translation_cov_eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Vector3d shadow_posterior_translation_cov_eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 1> raw_posterior_pose_cov_diagonal =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> shadow_posterior_pose_cov_diagonal =
        Eigen::Matrix<double, 6, 1>::Zero();
    double solve_time_ms = 0.0;
  };
  std::vector<DirectionalShadow> directional_shadows;
  fast_livo::LioMotionConsistencyMetrics motion_consistency;
  std::string motion_shadow_reason = "diagnostic_invalid";
};

// P4-B plane refitting only consumes the world point, its covariance, and
// provenance. Keeping the full pointWithVar here duplicates several unused
// 3-vectors/matrices for every mature plane and makes a three-way fork
// impractical on the target host.
struct P4bSupportPoint
{
  Eigen::Vector3d point_w = Eigen::Vector3d::Zero();
  Eigen::Matrix3d var = Eigen::Matrix3d::Zero();
  int source_frame_id = -1;
  int source_point_index = -1;
  double source_timestamp_s =
      std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d source_origin_w = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());

  P4bSupportPoint() = default;
  explicit P4bSupportPoint(const pointWithVar &point)
      : point_w(point.point_w), var(point.var),
        source_frame_id(point.source_frame_id),
        source_point_index(point.source_point_index),
        source_timestamp_s(point.source_timestamp_s),
        source_origin_w(point.source_origin_w) {}

  pointWithVar toPointWithVar() const
  {
    pointWithVar point;
    point.point_w = point_w;
    point.var = var;
    point.source_frame_id = source_frame_id;
    point.source_point_index = source_point_index;
    point.source_timestamp_s = source_timestamp_s;
    point.source_origin_w = source_origin_w;
    return point;
  }
};

// Immutable sharing is the P4-B copy-on-write boundary. Plane updates replace
// the complete support set; the explicit mutable accessor always detaches and
// exists for the executable isolation test.
class P4bSupportStorage
{
public:
  using Container = std::vector<P4bSupportPoint>;

  bool empty() const { return !points_ || points_->empty(); }
  std::size_t size() const { return points_ ? points_->size() : 0; }
  const Container &points() const
  {
    static const Container empty_points;
    return points_ ? *points_ : empty_points;
  }
  void clear() { points_.reset(); }
  void assign(const std::vector<pointWithVar> &points)
  {
    Container compact;
    compact.reserve(points.size());
    for (const pointWithVar &point : points) compact.emplace_back(point);
    assignCompact(std::move(compact));
  }
  void assignCompact(Container points)
  {
    points_ = points.empty()
        ? std::shared_ptr<const Container>()
        : std::make_shared<const Container>(std::move(points));
  }
  Container &mutablePoints()
  {
    auto detached = std::make_shared<Container>(points());
    points_ = detached;
    return *detached;
  }

private:
  std::shared_ptr<const Container> points_;
};

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  std::vector<int> p4_source_frame_ids_;
  std::vector<double> p4_source_timestamps_s_;
  std::vector<Eigen::Vector3d> p4_source_origins_w_;
  int p4_update_count_ = 0;
  double p4_last_center_shift_m_ = 0.0;
  double p4_last_normal_change_deg_ = 0.0;
  // P4-B only: actual values used by the last fit. Production keeps this
  // empty, including after temp_points_ is released for a mature plane.
  P4bSupportStorage p4b_support_points_;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

struct VoxelOctoTreeSnapshot
{
  std::vector<pointWithVar> temp_points;
  VoxelPlane plane;
  int layer = 0;
  int octo_state = 0;
  std::array<std::unique_ptr<VoxelOctoTreeSnapshot>, 8> leaves;
  std::array<double, 3> voxel_center{{0.0, 0.0, 0.0}};
  std::vector<int> layer_init_num;
  float quarter_length = 0.0f;
  float planer_threshold = 0.0f;
  int points_size_threshold = 0;
  int update_size_threshold = 0;
  int max_points_num = 0;
  int max_layer = 0;
  int new_points = 0;
  bool init_octo = false;
  bool update_enable = true;
  bool p4b_retain_support = false;

  VoxelOctoTreeSnapshot() = default;
  VoxelOctoTreeSnapshot(const VoxelOctoTreeSnapshot &other);
  VoxelOctoTreeSnapshot &operator=(const VoxelOctoTreeSnapshot &other);
};

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

struct DS_POINT
{
  float xyz[3];
  float intensity;
  int count = 0;
};

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{

public:
  VoxelOctoTree();
  std::vector<pointWithVar> temp_points_;
  VoxelPlane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  VoxelOctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_init_num_;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;
  bool p4b_retain_support_ = false;

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
  void init_octo_tree();
  void cut_octo_tree();
  void UpdateOctoTree(const pointWithVar &pv);

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
  VoxelOctoTree *Insert(const pointWithVar &pv);
  VoxelOctoTreeSnapshot captureSnapshot() const;
  static std::unique_ptr<VoxelOctoTree> restoreSnapshot(
      const VoxelOctoTreeSnapshot &snapshot);
};

struct VoxelMapManagerSnapshot
{
  std::vector<std::pair<VOXEL_LOCATION, VoxelOctoTreeSnapshot>> local_map;
  std::vector<std::pair<VOXEL_LOCATION, VoxelOctoTreeSnapshot>> long_term_map;
  std::vector<VOXEL_LOCATION> visual_observed_voxels;
  int current_frame_id = 0;
  int scan_count = 0;
  M3D lidar_rotation_to_imu = M3D::Identity();
  V3D lidar_translation_to_imu = V3D::Zero();
  StatesGroup state;
  V3D position_last = V3D::Zero();
  V3D last_slide_position = V3D::Zero();
  bool lidar_degenerated = false;
  double lidar_constraint_ratio = 0.0;
  int degeneracy_bad_frame_count = 0;
  int degeneracy_good_frame_count = 0;
  int direction_conflict_frame_count = 0;
  int direction_clear_frame_count = 0;
  bool direction_guard_active = false;
  std::vector<fast_livo::RollingCorrectionWindow::SnapshotSample>
      motion_correction_samples;
  std::vector<std::pair<VOXEL_LOCATION, int>> p4_voxel_last_update_frame;
  double p4_first_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  double p4_current_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  int next_plane_id = 0;
  std::vector<int> accepted_map_frame_ids;
};

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

class VoxelMapManager
{
public:
  VoxelMapManager() = default;
  ~VoxelMapManager() { clearLocalMap(); }
  VoxelMapConfig config_setting_;
  int current_frame_id_ = 0;
  ros::Publisher voxel_map_pub_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> long_term_visual_map_;
  std::unordered_set<VOXEL_LOCATION> visual_observed_voxels_;
  
  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;
  PointCloudXYZI::ConstPtr p4_raw_cloud_;
  PointCloudXYZI::ConstPtr p4_fixed_velocity_cloud_;
  PointCloudXYZI::Ptr p4_fixed_velocity_down_body_;

  M3D extR_;
  V3D extT_;
  float build_residual_time, ekf_time;
  float ave_build_residual_time = 0.0;
  float ave_ekf_time = 0.0;
  int scan_count = 0;
  StatesGroup state_;
  V3D position_last_;

  V3D last_slide_position = {0,0,0};

  geometry_msgs::Quaternion geoQuat_;

  int feats_down_size_;
  int effct_feat_num_;
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;
  bool lidar_degenerated_ = false;
  double lidar_constraint_ratio_ = 0.0;
  LioUpdateDiagnostics last_lio_diagnostics_;

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    p4_fixed_velocity_down_body_.reset(new PointCloudXYZI());
  };

  void StateEstimation(StatesGroup &state_propagat, double timestamp,
                       std::ostream *iteration_log = nullptr);
  void configureP5SeedBasin(double leaf_scale_m,
                            const std::string &default_output_directory);
  void runP5SeedBasinShadow(const StatesGroup &propagated_state,
                            double timestamp, double relative_time_s);
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  void BuildVoxelMap(double timestamp);
  V3F RGBFromVoxel(const V3D &input_point);

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);
  void clearLocalMap();

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  void pubVoxelMap();

  void mapSliding();
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );
  void setVisualObservedVoxels(const std::vector<VOXEL_LOCATION> &observed_voxels);
  void updateLidarDegeneracyStatus();
  bool isLidarDegenerated() const;
  double getLidarConstraintRatio() const;
  const LioUpdateDiagnostics &getLastLioDiagnostics() const;
  void markLastLioMapInserted(bool inserted);
  bool p4MapMutationAllowed(double timestamp);
  void p4SetDeskewDiagnostics(const fast_livo::p4::DeskewDiagnostics &value);
  void p4RecordMapDecision(double timestamp, bool inserted,
                           const std::string &reason,
                           const Eigen::Vector3d &predicted_position,
                           const Eigen::Vector3d &inserted_position);
  VoxelMapManagerSnapshot captureSnapshot() const;
  void restoreSnapshot(const VoxelMapManagerSnapshot &snapshot);
  void setP4bRecentPointExclusion(int accepted_lidar_frames);
  const VoxelPlane *p4bRefittedPlaneForTest(
      const VoxelOctoTree &node);
  std::size_t p4bCurrentUpdatedVoxelCount() const;

private:
  int degeneracy_bad_frame_count_ = 0;
  int degeneracy_good_frame_count_ = 0;
  int direction_conflict_frame_count_ = 0;
  int direction_clear_frame_count_ = 0;
  bool direction_guard_active_ = false;
  fast_livo::RollingCorrectionWindow motion_correction_window_;
  double p4_first_timestamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double p4_current_timestamp_s_ = std::numeric_limits<double>::quiet_NaN();
  fast_livo::p4::DeskewDiagnostics p4_deskew_diagnostics_;
  std::ofstream p4_iteration_csv_;
  std::ofstream p4_correspondence_csv_;
  std::ofstream p4_map_csv_;
  std::ofstream p4_deskew_csv_;
  bool p4_output_initialized_ = false;
  std::vector<fast_livo::p4::Association> p4_previous_associations_;
  std::unordered_map<VOXEL_LOCATION, int> p4_voxel_last_update_frame_;
  int p4b_recent_point_exclusion_frames_ = 0;
  std::unordered_map<const VoxelPlane *, std::unique_ptr<VoxelPlane>>
      p4b_refitted_plane_cache_;
  std::deque<int> p4b_accepted_map_frame_ids_;

  double p5_leaf_scale_m_ = std::numeric_limits<double>::quiet_NaN();
  bool p5_output_initialized_ = false;
  std::ofstream p5_seed_csv_;
  std::ofstream p5_profile_csv_;
  std::ofstream p5_parity_csv_;
  std::ofstream p5_multistart_csv_;

  struct P5Objective
  {
    int correspondences = 0;
    double mean_abs_residual = std::numeric_limits<double>::quiet_NaN();
    double lidar_cost = std::numeric_limits<double>::quiet_NaN();
    double weighted_rms = std::numeric_limits<double>::quiet_NaN();
    fast_livo::LioObservabilityMetrics observability;
    std::vector<fast_livo::p4::Association> associations;
  };

  const VoxelPlane *p4bQueryPlane(const VoxelOctoTree &node);

  void StateEstimationInternal(StatesGroup &state_propagat, double timestamp,
                               std::ostream *iteration_log,
                               const StatesGroup *initial_guess,
                               bool shadow_mode);
  void StateEstimationP5Prototype(StatesGroup &state_propagat,
                                  double timestamp,
                                  std::ostream *iteration_log);
  P5Objective p5EvaluateObjective(const StatesGroup &evaluation_state,
                                  const StatesGroup &propagated_state);
  void p5InitializeOutput();

  double p4RelativeTime(double timestamp);
  bool p4InAnalysisWindow(double timestamp);
  bool p4InDetailWindow(double timestamp);
  void p4InitializeOutput();
  void p4RecordIteration(double timestamp, int iteration,
                         const StatesGroup &iteration_state_before,
                         const Eigen::MatrixXd &jacobian,
                         const Eigen::VectorXd &inverse_variance,
                         const fast_livo::LioObservabilityMetrics &observability);

  bool classifyLidarDegeneracy(const fast_livo::LioObservabilityMetrics &metrics,
                               int effective_features) const;
  void updateLidarDegeneracyHysteresis(bool raw_degenerate);
  void updateDirectionGuardHysteresis(bool conflict);

  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_
