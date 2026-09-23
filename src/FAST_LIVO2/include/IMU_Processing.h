/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include "common_lib.h"
#include "p4_frontend_diagnostics.h"
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>
#include <fstream>
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Snapshot
  {
    PointCloudXYZI pcl_wait_proc;
    sensor_msgs::Imu last_imu;
    bool has_last_imu = false;
    PointCloudXYZI cur_pcl_un;
    std::vector<Pose6D> imu_pose;
    M3D lidar_rotation_to_imu = M3D::Identity();
    V3D lidar_offset_to_imu = V3D::Zero();
    V3D mean_acc = V3D::Zero();
    V3D mean_gyr = V3D::Zero();
    V3D angular_velocity_last = V3D::Zero();
    V3D specific_acceleration_last = V3D::Zero();
    double last_propagation_end_time = 0.0;
    double last_scan_time = 0.0;
    int initialization_iteration = 1;
    int maximum_initialization_count = 20;
    bool first_frame = true;
    bool imu_enabled = true;
    bool gravity_estimation_enabled = true;
    bool bias_estimation_enabled = true;
    bool exposure_estimation_enabled = true;
    double imu_mean_acc_norm = 0.0;
    V3D unbiased_gyr = V3D::Zero();
    V3D covariance_acc = V3D::Zero();
    V3D covariance_gyr = V3D::Zero();
    V3D covariance_bias_gyr = V3D::Zero();
    V3D covariance_bias_acc = V3D::Zero();
    double covariance_inverse_exposure = 0.0;
    double first_lidar_time = 0.0;
    bool imu_time_initialized = false;
    bool imu_needs_initialization = true;
    bool scan_history_active = false;
    double scan_history_origin_time = 0.0;
    int lidar_type = 0;
    M3D identity3 = M3D::Identity();
    V3D zero3 = V3D::Zero();
  };

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov_scale(const V3D &scaler);
  void set_acc_cov_scale(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_inv_expo_cov(const double &inv_expo);
  void set_imu_init_frame_num(const int &num);
  void disable_imu();
  void disable_gravity_est();
  void disable_bias_est();
  void disable_exposure_est();
  void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  void beginLidarScan(double scan_begin_time, const StatesGroup &state);
  void updateLidarScanPose(double timestamp, const StatesGroup &state);
  void set_log_dir(const std::string &log_dir) { log_dir_ = log_dir; }
  void enable_competition_startup_telemetry(bool enabled);
  void enable_p4_diagnostics(bool enabled) { p4_diagnostics_enabled_ = enabled; }
  const fast_livo::p4::DeskewDiagnostics &p4_deskew_diagnostics() const
  {
    return p4_deskew_diagnostics_;
  }
  PointCloudXYZI::ConstPtr p4_raw_cloud() const { return p4_raw_cloud_; }
  PointCloudXYZI::ConstPtr p4_fixed_velocity_cloud() const
  {
    return p4_fixed_velocity_cloud_;
  }
  Snapshot captureSnapshot() const;
  void restoreSnapshot(const Snapshot &snapshot);

  ofstream fout_imu;
  double IMU_mean_acc_norm;
  V3D unbiased_gyr;

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double cov_inv_expo;
  double first_lidar_time;
  bool imu_time_init = false;
  bool imu_need_init = true;
  M3D Eye3d;
  V3D Zero3d;
  int lidar_type;

private:
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  PointCloudXYZI pcl_wait_proc;
  sensor_msgs::ImuConstPtr last_imu;
  PointCloudXYZI::Ptr cur_pcl_un_;
  vector<Pose6D> IMUpose;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double last_prop_end_time;
  bool scan_history_active_ = false;
  double scan_history_origin_time_ = 0.0;
  double time_last_scan;
  int init_iter_num = 1, MAX_INI_COUNT = 20;
  bool b_first_frame = true;
  bool imu_en = true;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  bool exposure_estimate_en = true;
  std::string log_dir_;
  bool competition_startup_telemetry_enabled_ = false;
  std::ofstream competition_startup_telemetry_;
  bool p4_diagnostics_enabled_ = false;
  fast_livo::p4::DeskewDiagnostics p4_deskew_diagnostics_;
  PointCloudXYZI::Ptr p4_raw_cloud_{new PointCloudXYZI()};
  PointCloudXYZI::Ptr p4_fixed_velocity_cloud_{new PointCloudXYZI()};
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif
