#include "p4_fork_snapshot.h"
#include "p4_fork_harness.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <unordered_map>

namespace
{

pointWithVar makePoint(double x, int frame, int index)
{
  pointWithVar point;
  point.point_b = V3D(x, x + 1.0, x + 2.0);
  point.point_raw = V3D(x + 3.0, x + 4.0, x + 5.0);
  point.point_i = V3D(x + 6.0, x + 7.0, x + 8.0);
  point.point_w = V3D(x + 9.0, x + 10.0, x + 11.0);
  point.var_nostate = M3D::Identity() * (x + 0.1);
  point.body_var = M3D::Identity() * (x + 0.2);
  point.var = M3D::Identity() * (x + 0.3);
  point.point_crossmat = M3D::Identity() * (x + 0.4);
  point.normal = V3D(0.0, 0.0, 1.0);
  point.source_frame_id = frame;
  point.source_point_index = index;
  point.source_timestamp_s = 1000.0 + frame;
  point.source_origin_w = V3D(frame, frame + 1.0, frame + 2.0);
  return point;
}

VoxelOctoTreeSnapshot makeRoot(bool mature, bool valid)
{
  VoxelOctoTreeSnapshot root;
  root.layer = 0;
  root.octo_state = valid ? 0 : 1;
  root.voxel_center = {{1.0, 2.0, 3.0}};
  root.layer_init_num = {2, 2, 2};
  root.quarter_length = 0.25f;
  root.planer_threshold = 0.01f;
  root.points_size_threshold = 2;
  root.update_size_threshold = 5;
  root.max_points_num = 3;
  root.max_layer = 2;
  root.new_points = mature ? 0 : 1;
  root.init_octo = true;
  root.update_enable = !mature;
  root.p4b_retain_support = true;
  if (!mature) root.temp_points.push_back(makePoint(1.0, 8, 0));
  root.plane.center_ = V3D(10.0, 20.0, 30.0);
  root.plane.normal_ = V3D::UnitZ();
  root.plane.x_normal_ = V3D::UnitX();
  root.plane.y_normal_ = V3D::UnitY();
  root.plane.covariance_ = M3D::Identity() * 0.5;
  root.plane.plane_var_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.25;
  root.plane.radius_ = 3.0f;
  root.plane.min_eigen_value_ = 0.001f;
  root.plane.mid_eigen_value_ = 0.2f;
  root.plane.max_eigen_value_ = 0.8f;
  root.plane.d_ = -30.0f;
  root.plane.points_size_ = 3;
  root.plane.is_plane_ = valid;
  root.plane.is_init_ = valid;
  root.plane.id_ = 17;
  root.plane.is_update_ = true;
  root.plane.p4_source_frame_ids_ = {5, 8, 9};
  root.plane.p4_source_timestamps_s_ = {1005.0, 1008.0, 1009.0};
  root.plane.p4_source_origins_w_ = {
      V3D(5, 6, 7), V3D(8, 9, 10), V3D(9, 10, 11)};
  root.plane.p4_update_count_ = mature ? 4 : 1;
  root.plane.p4_last_center_shift_m_ = 0.12;
  root.plane.p4_last_normal_change_deg_ = 0.34;
  root.plane.p4b_support_points_.assign(std::vector<pointWithVar>{
      makePoint(2.0, 5, 0), makePoint(3.0, 8, 1), makePoint(4.0, 9, 2)});
  return root;
}

fast_livo::p4b::P4ForkSnapshot makeSnapshot()
{
  fast_livo::p4b::P4ForkSnapshot snapshot;
  snapshot.boundary_timestamp_s = 1895.0;
  snapshot.state.pos_end = V3D(1.0, 2.0, 3.0);
  snapshot.state.vel_end = V3D(4.0, 5.0, 6.0);
  snapshot.state.bias_g = V3D(0.01, 0.02, 0.03);
  snapshot.state.bias_a = V3D(0.04, 0.05, 0.06);
  snapshot.state.gravity = V3D(0.0, 0.0, -9.81);
  snapshot.state.inv_expo_time = 1.25;
  snapshot.state.cov(3, 3) = 0.123;
  snapshot.propagated_state = snapshot.state;
  snapshot.propagated_state.pos_end.x() += 0.5;

  auto &imu = snapshot.imu;
  imu.pcl_wait_proc.push_back(PointType());
  imu.pcl_wait_proc.back().x = 1.0f;
  imu.pcl_wait_proc.width = 1;
  imu.pcl_wait_proc.height = 1;
  imu.has_last_imu = true;
  imu.last_imu.header.seq = 12;
  imu.last_imu.header.stamp = ros::Time(123, 456);
  imu.last_imu.header.frame_id = "imu";
  imu.last_imu.angular_velocity.x = 0.3;
  imu.last_imu.linear_acceleration.z = 9.7;
  imu.cur_pcl_un = imu.pcl_wait_proc;
  Pose6D pose{};
  pose.offset_time = 0.01;
  pose.acc[0] = 1.0;
  pose.rot[0] = pose.rot[4] = pose.rot[8] = 1.0;
  imu.imu_pose.push_back(pose);
  imu.mean_acc = V3D(0.0, 0.0, -9.7);
  imu.mean_gyr = V3D(0.1, 0.2, 0.3);
  imu.angular_velocity_last = V3D(0.4, 0.5, 0.6);
  imu.specific_acceleration_last = V3D(1.4, 1.5, 1.6);
  imu.last_propagation_end_time = 1895.0;
  imu.last_scan_time = 1894.9;
  imu.initialization_iteration = 33;
  imu.maximum_initialization_count = 20;
  imu.first_frame = false;
  imu.imu_enabled = true;
  imu.gravity_estimation_enabled = true;
  imu.bias_estimation_enabled = true;
  imu.exposure_estimation_enabled = false;
  imu.imu_mean_acc_norm = 9.7;
  imu.covariance_acc = V3D::Constant(0.01);
  imu.covariance_gyr = V3D::Constant(0.02);
  imu.covariance_bias_gyr = V3D::Constant(0.03);
  imu.covariance_bias_acc = V3D::Constant(0.04);
  imu.covariance_inverse_exposure = 0.2;
  imu.first_lidar_time = 1000.0;
  imu.imu_time_initialized = true;
  imu.imu_needs_initialization = false;
  imu.lidar_type = AVIA;

  VoxelOctoTreeSnapshot multi_layer = makeRoot(false, false);
  multi_layer.leaves[3].reset(new VoxelOctoTreeSnapshot(makeRoot(false, true)));
  multi_layer.leaves[3]->layer = 1;
  snapshot.map.local_map.emplace_back(VOXEL_LOCATION(2, -1, 4),
                                      std::move(multi_layer));
  snapshot.map.long_term_map.emplace_back(VOXEL_LOCATION(-3, 8, 1),
                                          makeRoot(true, true));
  snapshot.map.visual_observed_voxels = {
      VOXEL_LOCATION(2, -1, 4), VOXEL_LOCATION(-3, 8, 1)};
  snapshot.map.current_frame_id = 90;
  snapshot.map.scan_count = 77;
  snapshot.map.lidar_rotation_to_imu =
      Eigen::AngleAxisd(0.1, V3D::UnitX()).toRotationMatrix();
  snapshot.map.lidar_translation_to_imu = V3D(0.1, 0.2, 0.3);
  snapshot.map.state = snapshot.state;
  snapshot.map.position_last = V3D(8, 9, 10);
  snapshot.map.last_slide_position = V3D(7, 8, 9);
  snapshot.map.lidar_degenerated = true;
  snapshot.map.lidar_constraint_ratio = 0.4;
  snapshot.map.degeneracy_bad_frame_count = 2;
  snapshot.map.direction_conflict_frame_count = 3;
  snapshot.map.direction_guard_active = true;
  snapshot.map.motion_correction_samples.push_back(
      {1894.8, V3D(0.1, 0.2, 0.3)});
  snapshot.map.p4_voxel_last_update_frame.push_back(
      {VOXEL_LOCATION(2, -1, 4), 89});
  snapshot.map.p4_first_timestamp_s = 1000.0;
  snapshot.map.p4_current_timestamp_s = 1895.0;
  snapshot.map.next_plane_id = 19;
  snapshot.map.accepted_map_frame_ids = {80, 81, 82, 83, 85, 86, 87, 88, 89};

  auto &life = snapshot.lifecycle;
  life.lidar_map_initialized = true;
  life.gravity_alignment_finished = true;
  life.lidar_map_update_counter = 90;
  life.lidar_map_guard_active = true;
  life.lidar_map_guard_recovery_frames = 2;
  life.lidar_frame_begin_time = 1894.9;
  life.lidar_frame_end_time = 1895.0;
  life.last_lidar_update_time = 1895.0;
  life.lidar_scan_index = 123;
  return snapshot;
}

void assertIsolation(const fast_livo::p4b::P4ForkSnapshot &source)
{
  using fast_livo::p4b::logicalHashes;
  const auto expected = logicalHashes(source);
  auto branch_a = source;
  const auto branch_b = source;
  const auto branch_c = source;

  branch_a.state.pos_end.x() += 1.0;
  assert(logicalHashes(branch_a).state != expected.state);
  assert(logicalHashes(branch_b) == expected);
  assert(logicalHashes(branch_c) == expected);
  branch_a = source;

  branch_a.state.cov(0, 0) += 1.0;
  assert(logicalHashes(branch_a).covariance != expected.covariance);
  assert(logicalHashes(branch_b) == expected);
  branch_a = source;

  branch_a.map.local_map.front().second.temp_points.front().point_w.x() += 1.0;
  assert(logicalHashes(branch_a).map != expected.map);
  assert(logicalHashes(branch_b) == expected);
  branch_a = source;

  branch_a.map.long_term_map.front().second.plane
      .p4b_support_points_.mutablePoints().front().point_w.y() += 1.0;
  assert(logicalHashes(branch_a).map != expected.map);
  assert(logicalHashes(branch_b) == expected);
  assert(logicalHashes(branch_c) == expected);
  branch_a = source;

  branch_a.map.local_map.front().second.plane.radius_ += 1.0f;
  assert(logicalHashes(branch_a).map != expected.map);
  assert(logicalHashes(branch_c) == expected);
  branch_a = source;

  branch_a.map.local_map.front().second.plane.p4_source_frame_ids_.front()++;
  assert(logicalHashes(branch_a).map != expected.map);
  assert(logicalHashes(branch_b) == expected);
  assert(logicalHashes(branch_c) == expected);
}

} // namespace

int main()
{
  using namespace fast_livo::p4b;
  const P4ForkSnapshot source = makeSnapshot();
  assertIsolation(source);

  VoxelMapConfig config{};
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> empty_map;
  VoxelMapManager restored_manager(config, empty_map);
  restored_manager.restoreSnapshot(source.map);
  P4ForkSnapshot manager_roundtrip = source;
  manager_roundtrip.map = restored_manager.captureSnapshot();
  assert(logicalHashes(manager_roundtrip).map == logicalHashes(source).map);

  VoxelOctoTree *query_node =
      restored_manager.voxel_map_.begin()->second;
  assert(query_node->leaves_[3] != nullptr);
  query_node = query_node->leaves_[3];
  query_node->points_size_threshold_ = 2;
  query_node->planer_threshold_ = 0.01f;
  query_node->plane_ptr_->p4b_support_points_.clear();
  std::vector<pointWithVar> support_points;
  const std::array<V3D, 6> support_positions = {{
      V3D(0, 0, 0), V3D(3, 0, 0), V3D(0, 1, 0), V3D(2, 1, 0),
      V3D(0, 0, 2), V3D(1, 0, 2)}};
  for (std::size_t i = 0; i < support_positions.size(); ++i)
  {
    pointWithVar point = makePoint(static_cast<double>(i),
                                   i < 4 ? 80 + i : 88 + i - 4,
                                   static_cast<int>(i));
    point.point_w = support_positions[i];
    point.var = M3D::Identity() * 1e-4;
    support_points.push_back(point);
  }
  query_node->plane_ptr_->p4b_support_points_.assign(support_points);
  restored_manager.setP4bRecentPointExclusion(5);
  const VoxelPlane *refitted =
      restored_manager.p4bRefittedPlaneForTest(*query_node);
  assert(refitted != nullptr);
  assert(refitted != query_node->plane_ptr_);
  assert(refitted->points_size_ == 4);
  assert(refitted->p4_source_frame_ids_.size() == 4);
  assert(std::abs(refitted->center_.z()) < 1e-12);
  assert(std::abs(std::abs(refitted->normal_.z()) - 1.0) < 1e-12);
  assert(query_node->plane_ptr_->p4b_support_points_.size() == 6);

  ImuProcess restored_imu;
  restored_imu.restoreSnapshot(source.imu);
  P4ForkSnapshot imu_roundtrip = source;
  imu_roundtrip.imu = restored_imu.captureSnapshot();
  assert(logicalHashes(imu_roundtrip).imu == logicalHashes(source).imu);

  const std::string path = "/tmp/fast_livo_p4b_snapshot_self_test.bin";
  std::string error;
  assert(saveSnapshot(source, path, error));
  P4ForkSnapshot disk_roundtrip;
  assert(loadSnapshot(path, disk_roundtrip, error));
  assert(logicalHashes(disk_roundtrip) == logicalHashes(source));
  std::remove(path.c_str());

  P4ForkSnapshot empty;
  assert(saveSnapshot(empty, path, error));
  P4ForkSnapshot empty_roundtrip;
  assert(loadSnapshot(path, empty_roundtrip, error));
  assert(logicalHashes(empty_roundtrip) == logicalHashes(empty));
  std::remove(path.c_str());

  P4ForkSnapshot reordered = source;
  std::reverse(reordered.map.visual_observed_voxels.begin(),
               reordered.map.visual_observed_voxels.end());
  assert(logicalHashes(reordered) == logicalHashes(source));

  {
    HarnessConfig harness_config;
    harness_config.enabled = true;
    harness_config.run_all_branches = true;
    harness_config.output_directory =
        "/tmp/fast_livo_p4b_harness_self_test";
    VoxelMapConfig voxel_config{};
    voxel_config.map_guard_mode = "off";
    P4ForkHarness harness(harness_config);
    assert(harness.capture(source, voxel_config, 0, error));
    assert(harness.captured());
  }

  std::cout << "P4_FORK_SNAPSHOT_SELF_TEST=PASS\n"
            << "DEEP_COPY_ISOLATION=PASS\n"
            << "MAP_CLONE_CASES=empty,populated,multi_layer,valid_plane,"
               "invalid_plane,mature,recent,provenance,sliding\n"
            << "DISK_ROUNDTRIP=PASS\n"
            << "THREE_BRANCH_FORK_PARITY=PASS\n"
            << "RECENT5_POINT_REFIT=PASS\n";
  return 0;
}
