#include "p4_fork_snapshot.h"

#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace fast_livo
{
namespace p4b
{
namespace
{

class Writer
{
public:
  template <typename T>
  void pod(const T &value)
  {
    static_assert(std::is_arithmetic<T>::value, "arithmetic only");
    bytes_.append(reinterpret_cast<const char *>(&value), sizeof(value));
  }

  void boolean(bool value)
  {
    const std::uint8_t encoded = value ? 1 : 0;
    pod(encoded);
  }

  void string(const std::string &value)
  {
    size(value.size());
    bytes_.append(value);
  }

  void size(std::size_t value)
  {
    pod(static_cast<std::uint64_t>(value));
  }

  const std::string &bytes() const { return bytes_; }

private:
  std::string bytes_;
};

class Reader
{
public:
  explicit Reader(const std::string &bytes) : bytes_(bytes) {}

  template <typename T>
  T pod()
  {
    static_assert(std::is_arithmetic<T>::value, "arithmetic only");
    require(sizeof(T));
    T value;
    std::memcpy(&value, bytes_.data() + offset_, sizeof(T));
    offset_ += sizeof(T);
    return value;
  }

  bool boolean()
  {
    const std::uint8_t value = pod<std::uint8_t>();
    if (value > 1) throw std::runtime_error("invalid boolean");
    return value != 0;
  }

  std::size_t size()
  {
    const std::uint64_t value = pod<std::uint64_t>();
    if (value > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
      throw std::runtime_error("size overflow");
    return static_cast<std::size_t>(value);
  }

  std::string string()
  {
    const std::size_t count = size();
    require(count);
    std::string value(bytes_.data() + offset_, count);
    offset_ += count;
    return value;
  }

  bool done() const { return offset_ == bytes_.size(); }

private:
  void require(std::size_t count)
  {
    if (count > bytes_.size() - offset_)
      throw std::runtime_error("truncated snapshot");
  }

  const std::string &bytes_;
  std::size_t offset_ = 0;
};

template <typename Derived>
void writeEigen(Writer &writer, const Eigen::MatrixBase<Derived> &value)
{
  for (int row = 0; row < value.rows(); ++row)
    for (int col = 0; col < value.cols(); ++col)
      writer.pod(static_cast<double>(value(row, col)));
}

template <typename Matrix>
void readEigen(Reader &reader, Matrix &value)
{
  for (int row = 0; row < value.rows(); ++row)
    for (int col = 0; col < value.cols(); ++col)
      value(row, col) = reader.pod<double>();
}

void writePoint(Writer &writer, const PointType &point)
{
  writer.pod(point.x);
  writer.pod(point.y);
  writer.pod(point.z);
  writer.pod(point.intensity);
  writer.pod(point.normal_x);
  writer.pod(point.normal_y);
  writer.pod(point.normal_z);
  writer.pod(point.curvature);
}

PointType readPoint(Reader &reader)
{
  PointType point;
  point.x = reader.pod<float>();
  point.y = reader.pod<float>();
  point.z = reader.pod<float>();
  point.intensity = reader.pod<float>();
  point.normal_x = reader.pod<float>();
  point.normal_y = reader.pod<float>();
  point.normal_z = reader.pod<float>();
  point.curvature = reader.pod<float>();
  return point;
}

void writeCloud(Writer &writer, const PointCloudXYZI &cloud)
{
  writer.pod(cloud.width);
  writer.pod(cloud.height);
  writer.boolean(cloud.is_dense);
  writer.size(cloud.points.size());
  for (const PointType &point : cloud.points) writePoint(writer, point);
}

void readCloud(Reader &reader, PointCloudXYZI &cloud)
{
  cloud.clear();
  cloud.width = reader.pod<std::uint32_t>();
  cloud.height = reader.pod<std::uint32_t>();
  cloud.is_dense = reader.boolean();
  const std::size_t count = reader.size();
  cloud.points.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
    cloud.points.push_back(readPoint(reader));
  if (cloud.width * cloud.height != cloud.points.size())
  {
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
  }
}

void writeStateNominal(Writer &writer, const StatesGroup &state)
{
  writeEigen(writer, state.rot_end);
  writeEigen(writer, state.pos_end);
  writeEigen(writer, state.vel_end);
  writeEigen(writer, state.bias_g);
  writeEigen(writer, state.bias_a);
  writeEigen(writer, state.gravity);
  writer.pod(state.inv_expo_time);
}

void writeState(Writer &writer, const StatesGroup &state)
{
  writeStateNominal(writer, state);
  writeEigen(writer, state.cov);
}

void readState(Reader &reader, StatesGroup &state)
{
  readEigen(reader, state.rot_end);
  readEigen(reader, state.pos_end);
  readEigen(reader, state.vel_end);
  readEigen(reader, state.bias_g);
  readEigen(reader, state.bias_a);
  readEigen(reader, state.gravity);
  state.inv_expo_time = reader.pod<double>();
  readEigen(reader, state.cov);
}

void writeImuMessage(Writer &writer, const sensor_msgs::Imu &imu)
{
  writer.pod(imu.header.seq);
  writer.pod(imu.header.stamp.sec);
  writer.pod(imu.header.stamp.nsec);
  writer.string(imu.header.frame_id);
  writer.pod(imu.orientation.x);
  writer.pod(imu.orientation.y);
  writer.pod(imu.orientation.z);
  writer.pod(imu.orientation.w);
  for (double value : imu.orientation_covariance) writer.pod(value);
  writer.pod(imu.angular_velocity.x);
  writer.pod(imu.angular_velocity.y);
  writer.pod(imu.angular_velocity.z);
  for (double value : imu.angular_velocity_covariance) writer.pod(value);
  writer.pod(imu.linear_acceleration.x);
  writer.pod(imu.linear_acceleration.y);
  writer.pod(imu.linear_acceleration.z);
  for (double value : imu.linear_acceleration_covariance) writer.pod(value);
}

void readImuMessage(Reader &reader, sensor_msgs::Imu &imu)
{
  imu.header.seq = reader.pod<std::uint32_t>();
  imu.header.stamp.sec = reader.pod<std::uint32_t>();
  imu.header.stamp.nsec = reader.pod<std::uint32_t>();
  imu.header.frame_id = reader.string();
  imu.orientation.x = reader.pod<double>();
  imu.orientation.y = reader.pod<double>();
  imu.orientation.z = reader.pod<double>();
  imu.orientation.w = reader.pod<double>();
  for (double &value : imu.orientation_covariance) value = reader.pod<double>();
  imu.angular_velocity.x = reader.pod<double>();
  imu.angular_velocity.y = reader.pod<double>();
  imu.angular_velocity.z = reader.pod<double>();
  for (double &value : imu.angular_velocity_covariance) value = reader.pod<double>();
  imu.linear_acceleration.x = reader.pod<double>();
  imu.linear_acceleration.y = reader.pod<double>();
  imu.linear_acceleration.z = reader.pod<double>();
  for (double &value : imu.linear_acceleration_covariance) value = reader.pod<double>();
}

void writePose(Writer &writer, const Pose6D &pose)
{
  writer.pod(pose.offset_time);
  for (double value : pose.acc) writer.pod(value);
  for (double value : pose.gyr) writer.pod(value);
  for (double value : pose.vel) writer.pod(value);
  for (double value : pose.pos) writer.pod(value);
  for (double value : pose.rot) writer.pod(value);
}

Pose6D readPose(Reader &reader)
{
  Pose6D pose;
  pose.offset_time = reader.pod<double>();
  for (double &value : pose.acc) value = reader.pod<double>();
  for (double &value : pose.gyr) value = reader.pod<double>();
  for (double &value : pose.vel) value = reader.pod<double>();
  for (double &value : pose.pos) value = reader.pod<double>();
  for (double &value : pose.rot) value = reader.pod<double>();
  return pose;
}

void writeImu(Writer &writer, const ImuProcess::Snapshot &imu)
{
  writeCloud(writer, imu.pcl_wait_proc);
  writer.boolean(imu.has_last_imu);
  if (imu.has_last_imu) writeImuMessage(writer, imu.last_imu);
  writeCloud(writer, imu.cur_pcl_un);
  writer.size(imu.imu_pose.size());
  for (const Pose6D &pose : imu.imu_pose) writePose(writer, pose);
  writeEigen(writer, imu.lidar_rotation_to_imu);
  writeEigen(writer, imu.lidar_offset_to_imu);
  writeEigen(writer, imu.mean_acc);
  writeEigen(writer, imu.mean_gyr);
  writeEigen(writer, imu.angular_velocity_last);
  writeEigen(writer, imu.specific_acceleration_last);
  writer.pod(imu.last_propagation_end_time);
  writer.pod(imu.last_scan_time);
  writer.pod(imu.initialization_iteration);
  writer.pod(imu.maximum_initialization_count);
  writer.boolean(imu.first_frame);
  writer.boolean(imu.imu_enabled);
  writer.boolean(imu.gravity_estimation_enabled);
  writer.boolean(imu.bias_estimation_enabled);
  writer.boolean(imu.exposure_estimation_enabled);
  writer.pod(imu.imu_mean_acc_norm);
  writeEigen(writer, imu.unbiased_gyr);
  writeEigen(writer, imu.covariance_acc);
  writeEigen(writer, imu.covariance_gyr);
  writeEigen(writer, imu.covariance_bias_gyr);
  writeEigen(writer, imu.covariance_bias_acc);
  writer.pod(imu.covariance_inverse_exposure);
  writer.pod(imu.first_lidar_time);
  writer.boolean(imu.imu_time_initialized);
  writer.boolean(imu.imu_needs_initialization);
  writer.pod(imu.lidar_type);
  writeEigen(writer, imu.identity3);
  writeEigen(writer, imu.zero3);
}

void readImu(Reader &reader, ImuProcess::Snapshot &imu)
{
  readCloud(reader, imu.pcl_wait_proc);
  imu.has_last_imu = reader.boolean();
  if (imu.has_last_imu) readImuMessage(reader, imu.last_imu);
  readCloud(reader, imu.cur_pcl_un);
  imu.imu_pose.resize(reader.size());
  for (Pose6D &pose : imu.imu_pose) pose = readPose(reader);
  readEigen(reader, imu.lidar_rotation_to_imu);
  readEigen(reader, imu.lidar_offset_to_imu);
  readEigen(reader, imu.mean_acc);
  readEigen(reader, imu.mean_gyr);
  readEigen(reader, imu.angular_velocity_last);
  readEigen(reader, imu.specific_acceleration_last);
  imu.last_propagation_end_time = reader.pod<double>();
  imu.last_scan_time = reader.pod<double>();
  imu.initialization_iteration = reader.pod<int>();
  imu.maximum_initialization_count = reader.pod<int>();
  imu.first_frame = reader.boolean();
  imu.imu_enabled = reader.boolean();
  imu.gravity_estimation_enabled = reader.boolean();
  imu.bias_estimation_enabled = reader.boolean();
  imu.exposure_estimation_enabled = reader.boolean();
  imu.imu_mean_acc_norm = reader.pod<double>();
  readEigen(reader, imu.unbiased_gyr);
  readEigen(reader, imu.covariance_acc);
  readEigen(reader, imu.covariance_gyr);
  readEigen(reader, imu.covariance_bias_gyr);
  readEigen(reader, imu.covariance_bias_acc);
  imu.covariance_inverse_exposure = reader.pod<double>();
  imu.first_lidar_time = reader.pod<double>();
  imu.imu_time_initialized = reader.boolean();
  imu.imu_needs_initialization = reader.boolean();
  imu.lidar_type = reader.pod<int>();
  readEigen(reader, imu.identity3);
  readEigen(reader, imu.zero3);
}

void writePointWithVar(Writer &writer, const pointWithVar &point)
{
  writeEigen(writer, point.point_b);
  writeEigen(writer, point.point_raw);
  writeEigen(writer, point.point_i);
  writeEigen(writer, point.point_w);
  writeEigen(writer, point.var_nostate);
  writeEigen(writer, point.body_var);
  writeEigen(writer, point.var);
  writeEigen(writer, point.point_crossmat);
  writeEigen(writer, point.normal);
  writer.pod(point.source_frame_id);
  writer.pod(point.source_point_index);
  writer.pod(point.source_timestamp_s);
  writeEigen(writer, point.source_origin_w);
}

pointWithVar readPointWithVar(Reader &reader)
{
  pointWithVar point;
  readEigen(reader, point.point_b);
  readEigen(reader, point.point_raw);
  readEigen(reader, point.point_i);
  readEigen(reader, point.point_w);
  readEigen(reader, point.var_nostate);
  readEigen(reader, point.body_var);
  readEigen(reader, point.var);
  readEigen(reader, point.point_crossmat);
  readEigen(reader, point.normal);
  point.source_frame_id = reader.pod<int>();
  point.source_point_index = reader.pod<int>();
  point.source_timestamp_s = reader.pod<double>();
  readEigen(reader, point.source_origin_w);
  return point;
}

void writePointVector(Writer &writer,
                      const std::vector<pointWithVar> &points)
{
  writer.size(points.size());
  for (const pointWithVar &point : points) writePointWithVar(writer, point);
}

void readPointVector(Reader &reader, std::vector<pointWithVar> &points)
{
  points.resize(reader.size());
  for (pointWithVar &point : points) point = readPointWithVar(reader);
}

void writeSupportStorage(Writer &writer, const P4bSupportStorage &storage)
{
  writer.size(storage.size());
  for (const P4bSupportPoint &point : storage.points())
  {
    writeEigen(writer, point.point_w);
    writeEigen(writer, point.var);
    writer.pod(point.source_frame_id);
    writer.pod(point.source_point_index);
    writer.pod(point.source_timestamp_s);
    writeEigen(writer, point.source_origin_w);
  }
}

void readSupportStorage(Reader &reader, P4bSupportStorage &storage)
{
  P4bSupportStorage::Container points(reader.size());
  for (P4bSupportPoint &point : points)
  {
    readEigen(reader, point.point_w);
    readEigen(reader, point.var);
    point.source_frame_id = reader.pod<int>();
    point.source_point_index = reader.pod<int>();
    point.source_timestamp_s = reader.pod<double>();
    readEigen(reader, point.source_origin_w);
  }
  storage.assignCompact(std::move(points));
}

void writePlane(Writer &writer, const VoxelPlane &plane)
{
  writeEigen(writer, plane.center_);
  writeEigen(writer, plane.normal_);
  writeEigen(writer, plane.y_normal_);
  writeEigen(writer, plane.x_normal_);
  writeEigen(writer, plane.covariance_);
  writeEigen(writer, plane.plane_var_);
  writer.pod(plane.radius_);
  writer.pod(plane.min_eigen_value_);
  writer.pod(plane.mid_eigen_value_);
  writer.pod(plane.max_eigen_value_);
  writer.pod(plane.d_);
  writer.pod(plane.points_size_);
  writer.boolean(plane.is_plane_);
  writer.boolean(plane.is_init_);
  writer.pod(plane.id_);
  writer.boolean(plane.is_update_);
  writer.size(plane.p4_source_frame_ids_.size());
  for (int value : plane.p4_source_frame_ids_) writer.pod(value);
  writer.size(plane.p4_source_timestamps_s_.size());
  for (double value : plane.p4_source_timestamps_s_) writer.pod(value);
  writer.size(plane.p4_source_origins_w_.size());
  for (const V3D &value : plane.p4_source_origins_w_) writeEigen(writer, value);
  writer.pod(plane.p4_update_count_);
  writer.pod(plane.p4_last_center_shift_m_);
  writer.pod(plane.p4_last_normal_change_deg_);
  writeSupportStorage(writer, plane.p4b_support_points_);
}

void readPlane(Reader &reader, VoxelPlane &plane)
{
  readEigen(reader, plane.center_);
  readEigen(reader, plane.normal_);
  readEigen(reader, plane.y_normal_);
  readEigen(reader, plane.x_normal_);
  readEigen(reader, plane.covariance_);
  readEigen(reader, plane.plane_var_);
  plane.radius_ = reader.pod<float>();
  plane.min_eigen_value_ = reader.pod<float>();
  plane.mid_eigen_value_ = reader.pod<float>();
  plane.max_eigen_value_ = reader.pod<float>();
  plane.d_ = reader.pod<float>();
  plane.points_size_ = reader.pod<int>();
  plane.is_plane_ = reader.boolean();
  plane.is_init_ = reader.boolean();
  plane.id_ = reader.pod<int>();
  plane.is_update_ = reader.boolean();
  plane.p4_source_frame_ids_.resize(reader.size());
  for (int &value : plane.p4_source_frame_ids_) value = reader.pod<int>();
  plane.p4_source_timestamps_s_.resize(reader.size());
  for (double &value : plane.p4_source_timestamps_s_)
    value = reader.pod<double>();
  plane.p4_source_origins_w_.resize(reader.size());
  for (V3D &value : plane.p4_source_origins_w_) readEigen(reader, value);
  plane.p4_update_count_ = reader.pod<int>();
  plane.p4_last_center_shift_m_ = reader.pod<double>();
  plane.p4_last_normal_change_deg_ = reader.pod<double>();
  readSupportStorage(reader, plane.p4b_support_points_);
}

void writeNode(Writer &writer, const VoxelOctoTreeSnapshot &node)
{
  writePointVector(writer, node.temp_points);
  writePlane(writer, node.plane);
  writer.pod(node.layer);
  writer.pod(node.octo_state);
  for (double value : node.voxel_center) writer.pod(value);
  writer.size(node.layer_init_num.size());
  for (int value : node.layer_init_num) writer.pod(value);
  writer.pod(node.quarter_length);
  writer.pod(node.planer_threshold);
  writer.pod(node.points_size_threshold);
  writer.pod(node.update_size_threshold);
  writer.pod(node.max_points_num);
  writer.pod(node.max_layer);
  writer.pod(node.new_points);
  writer.boolean(node.init_octo);
  writer.boolean(node.update_enable);
  writer.boolean(node.p4b_retain_support);
  for (const auto &child : node.leaves)
  {
    writer.boolean(static_cast<bool>(child));
    if (child) writeNode(writer, *child);
  }
}

void readNode(Reader &reader, VoxelOctoTreeSnapshot &node)
{
  readPointVector(reader, node.temp_points);
  readPlane(reader, node.plane);
  node.layer = reader.pod<int>();
  node.octo_state = reader.pod<int>();
  for (double &value : node.voxel_center) value = reader.pod<double>();
  node.layer_init_num.resize(reader.size());
  for (int &value : node.layer_init_num) value = reader.pod<int>();
  node.quarter_length = reader.pod<float>();
  node.planer_threshold = reader.pod<float>();
  node.points_size_threshold = reader.pod<int>();
  node.update_size_threshold = reader.pod<int>();
  node.max_points_num = reader.pod<int>();
  node.max_layer = reader.pod<int>();
  node.new_points = reader.pod<int>();
  node.init_octo = reader.boolean();
  node.update_enable = reader.boolean();
  node.p4b_retain_support = reader.boolean();
  for (auto &child : node.leaves)
  {
    if (reader.boolean())
    {
      child.reset(new VoxelOctoTreeSnapshot());
      readNode(reader, *child);
    }
    else
      child.reset();
  }
}

void writeKey(Writer &writer, const VOXEL_LOCATION &key)
{
  writer.pod(key.x);
  writer.pod(key.y);
  writer.pod(key.z);
}

VOXEL_LOCATION readKey(Reader &reader)
{
  const std::int64_t x = reader.pod<std::int64_t>();
  const std::int64_t y = reader.pod<std::int64_t>();
  const std::int64_t z = reader.pod<std::int64_t>();
  return VOXEL_LOCATION(x, y, z);
}

void writeMap(Writer &writer, const VoxelMapManagerSnapshot &map)
{
  auto write_entries = [&writer](const auto &entries) {
    writer.size(entries.size());
    std::vector<const typename std::decay_t<decltype(entries)>::value_type *>
        sorted;
    sorted.reserve(entries.size());
    for (const auto &entry : entries) sorted.push_back(&entry);
    std::sort(sorted.begin(), sorted.end(), [](const auto *a, const auto *b) {
      if (a->first.x != b->first.x) return a->first.x < b->first.x;
      if (a->first.y != b->first.y) return a->first.y < b->first.y;
      return a->first.z < b->first.z;
    });
    for (const auto *entry : sorted)
    {
      writeKey(writer, entry->first);
      writeNode(writer, entry->second);
    }
  };
  write_entries(map.local_map);
  write_entries(map.long_term_map);
  std::vector<VOXEL_LOCATION> visual = map.visual_observed_voxels;
  auto key_less = [](const VOXEL_LOCATION &a, const VOXEL_LOCATION &b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  };
  std::sort(visual.begin(), visual.end(), key_less);
  writer.size(visual.size());
  for (const VOXEL_LOCATION &key : visual) writeKey(writer, key);
  writer.pod(map.current_frame_id);
  writer.pod(map.scan_count);
  writeEigen(writer, map.lidar_rotation_to_imu);
  writeEigen(writer, map.lidar_translation_to_imu);
  writeState(writer, map.state);
  writeEigen(writer, map.position_last);
  writeEigen(writer, map.last_slide_position);
  writer.boolean(map.lidar_degenerated);
  writer.pod(map.lidar_constraint_ratio);
  writer.pod(map.degeneracy_bad_frame_count);
  writer.pod(map.degeneracy_good_frame_count);
  writer.pod(map.direction_conflict_frame_count);
  writer.pod(map.direction_clear_frame_count);
  writer.boolean(map.direction_guard_active);
  writer.size(map.motion_correction_samples.size());
  for (const auto &sample : map.motion_correction_samples)
  {
    writer.pod(sample.timestamp);
    writeEigen(writer, sample.delta_velocity);
  }
  auto updates = map.p4_voxel_last_update_frame;
  std::sort(updates.begin(), updates.end(),
            [&key_less](const auto &a, const auto &b) {
              return key_less(a.first, b.first);
            });
  writer.size(updates.size());
  for (const auto &entry : updates)
  {
    writeKey(writer, entry.first);
    writer.pod(entry.second);
  }
  writer.pod(map.p4_first_timestamp_s);
  writer.pod(map.p4_current_timestamp_s);
  writer.pod(map.next_plane_id);
  writer.size(map.accepted_map_frame_ids.size());
  for (int value : map.accepted_map_frame_ids) writer.pod(value);
}

void readMap(Reader &reader, VoxelMapManagerSnapshot &map)
{
  auto read_entries = [&reader](auto &entries) {
    entries.clear();
    const std::size_t count = reader.size();
    entries.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
      VOXEL_LOCATION key = readKey(reader);
      VoxelOctoTreeSnapshot node;
      readNode(reader, node);
      entries.emplace_back(key, std::move(node));
    }
  };
  read_entries(map.local_map);
  read_entries(map.long_term_map);
  map.visual_observed_voxels.resize(reader.size());
  for (VOXEL_LOCATION &key : map.visual_observed_voxels)
    key = readKey(reader);
  map.current_frame_id = reader.pod<int>();
  map.scan_count = reader.pod<int>();
  readEigen(reader, map.lidar_rotation_to_imu);
  readEigen(reader, map.lidar_translation_to_imu);
  readState(reader, map.state);
  readEigen(reader, map.position_last);
  readEigen(reader, map.last_slide_position);
  map.lidar_degenerated = reader.boolean();
  map.lidar_constraint_ratio = reader.pod<double>();
  map.degeneracy_bad_frame_count = reader.pod<int>();
  map.degeneracy_good_frame_count = reader.pod<int>();
  map.direction_conflict_frame_count = reader.pod<int>();
  map.direction_clear_frame_count = reader.pod<int>();
  map.direction_guard_active = reader.boolean();
  map.motion_correction_samples.resize(reader.size());
  for (auto &sample : map.motion_correction_samples)
  {
    sample.timestamp = reader.pod<double>();
    readEigen(reader, sample.delta_velocity);
  }
  map.p4_voxel_last_update_frame.clear();
  const std::size_t update_count = reader.size();
  map.p4_voxel_last_update_frame.reserve(update_count);
  for (std::size_t i = 0; i < update_count; ++i)
  {
    VOXEL_LOCATION key = readKey(reader);
    map.p4_voxel_last_update_frame.emplace_back(key, reader.pod<int>());
  }
  map.p4_first_timestamp_s = reader.pod<double>();
  map.p4_current_timestamp_s = reader.pod<double>();
  map.next_plane_id = reader.pod<int>();
  map.accepted_map_frame_ids.resize(reader.size());
  for (int &value : map.accepted_map_frame_ids) value = reader.pod<int>();
}

void writeLifecycle(Writer &writer, const MapperLifecycleSnapshot &value)
{
  writer.boolean(value.lidar_map_initialized);
  writer.boolean(value.gravity_alignment_finished);
  writer.pod(value.lidar_map_update_counter);
  writer.boolean(value.lidar_map_guard_active);
  writer.boolean(value.lidar_map_guard_hard_limit_latched);
  writer.pod(value.lidar_map_guard_recovery_frames);
  writer.pod(value.lidar_map_guard_freeze_frames);
  writer.pod(value.external_update_pause_map_frames);
  writer.pod(value.lidar_frame_begin_time);
  writer.pod(value.lidar_frame_end_time);
  writer.pod(value.last_lidar_update_time);
  writer.pod(value.lidar_scan_index);
}

void readLifecycle(Reader &reader, MapperLifecycleSnapshot &value)
{
  value.lidar_map_initialized = reader.boolean();
  value.gravity_alignment_finished = reader.boolean();
  value.lidar_map_update_counter = reader.pod<int>();
  value.lidar_map_guard_active = reader.boolean();
  value.lidar_map_guard_hard_limit_latched = reader.boolean();
  value.lidar_map_guard_recovery_frames = reader.pod<int>();
  value.lidar_map_guard_freeze_frames = reader.pod<int>();
  value.external_update_pause_map_frames = reader.pod<int>();
  value.lidar_frame_begin_time = reader.pod<double>();
  value.lidar_frame_end_time = reader.pod<double>();
  value.last_lidar_update_time = reader.pod<double>();
  value.lidar_scan_index = reader.pod<int>();
}

std::string encode(const P4ForkSnapshot &snapshot)
{
  Writer writer;
  writer.string("FAST_LIVO2_P4B_SNAPSHOT");
  writer.pod(P4ForkSnapshot::kFormatVersion);
  writer.pod(snapshot.boundary_timestamp_s);
  writeState(writer, snapshot.state);
  writeState(writer, snapshot.propagated_state);
  writeImu(writer, snapshot.imu);
  writeMap(writer, snapshot.map);
  writeLifecycle(writer, snapshot.lifecycle);
  return writer.bytes();
}

void decode(const std::string &bytes, P4ForkSnapshot &snapshot)
{
  Reader reader(bytes);
  if (reader.string() != "FAST_LIVO2_P4B_SNAPSHOT")
    throw std::runtime_error("invalid snapshot magic");
  if (reader.pod<std::uint32_t>() != P4ForkSnapshot::kFormatVersion)
    throw std::runtime_error("unsupported snapshot version");
  snapshot.boundary_timestamp_s = reader.pod<double>();
  readState(reader, snapshot.state);
  readState(reader, snapshot.propagated_state);
  readImu(reader, snapshot.imu);
  readMap(reader, snapshot.map);
  readLifecycle(reader, snapshot.lifecycle);
  if (!reader.done()) throw std::runtime_error("trailing snapshot bytes");
}

// Compact FIPS 180-4 SHA-256; kept local so the diagnostic harness adds no
// runtime dependency to the production node.
std::string sha256(const std::string &input)
{
  static constexpr std::array<std::uint32_t, 64> k = {{
      0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
      0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
      0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
      0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
      0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
      0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
      0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
      0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2}};
  const std::uint64_t bit_length =
      static_cast<std::uint64_t>(input.size()) * 8;
  const std::size_t padded_size =
      ((input.size() + 1 + 8 + 63) / 64) * 64;
  std::array<std::uint32_t, 8> h = {{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
                                     0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19}};
  auto rotr = [](std::uint32_t value, unsigned shift) {
    return (value >> shift) | (value << (32 - shift));
  };
  auto byte_at = [&](std::size_t index) -> std::uint8_t {
    if (index < input.size())
      return static_cast<std::uint8_t>(input[index]);
    if (index == input.size()) return 0x80;
    if (index >= padded_size - 8)
    {
      const unsigned shift =
          static_cast<unsigned>((padded_size - 1 - index) * 8);
      return static_cast<std::uint8_t>(bit_length >> shift);
    }
    return 0;
  };
  for (std::size_t offset = 0; offset < padded_size; offset += 64)
  {
    std::array<std::uint32_t, 64> w{};
    for (int i = 0; i < 16; ++i)
      w[i] = (static_cast<std::uint32_t>(byte_at(offset + 4*i)) << 24) |
             (static_cast<std::uint32_t>(byte_at(offset + 4*i + 1)) << 16) |
             (static_cast<std::uint32_t>(byte_at(offset + 4*i + 2)) << 8) |
             static_cast<std::uint32_t>(byte_at(offset + 4*i + 3));
    for (int i = 16; i < 64; ++i)
    {
      const std::uint32_t s0 = rotr(w[i-15],7) ^ rotr(w[i-15],18) ^ (w[i-15] >> 3);
      const std::uint32_t s1 = rotr(w[i-2],17) ^ rotr(w[i-2],19) ^ (w[i-2] >> 10);
      w[i] = w[i-16] + s0 + w[i-7] + s1;
    }
    std::uint32_t a=h[0], b=h[1], c=h[2], d=h[3], e=h[4], f=h[5], g=h[6], q=h[7];
    for (int i = 0; i < 64; ++i)
    {
      const std::uint32_t s1 = rotr(e,6) ^ rotr(e,11) ^ rotr(e,25);
      const std::uint32_t ch = (e & f) ^ ((~e) & g);
      const std::uint32_t t1 = q + s1 + ch + k[i] + w[i];
      const std::uint32_t s0 = rotr(a,2) ^ rotr(a,13) ^ rotr(a,22);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t t2 = s0 + maj;
      q=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d;
    h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=q;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::uint32_t value : h) output << std::setw(8) << value;
  return output.str();
}

} // namespace

bool LogicalHashes::operator==(const LogicalHashes &other) const
{
  return state == other.state && covariance == other.covariance &&
         imu == other.imu && map == other.map &&
         lifecycle == other.lifecycle && full == other.full;
}

LogicalHashes logicalHashes(const P4ForkSnapshot &snapshot)
{
  LogicalHashes result;
  {
    Writer writer;
    writeStateNominal(writer, snapshot.state);
    writeStateNominal(writer, snapshot.propagated_state);
    result.state = sha256(writer.bytes());
  }
  {
    Writer writer;
    writeEigen(writer, snapshot.state.cov);
    writeEigen(writer, snapshot.propagated_state.cov);
    result.covariance = sha256(writer.bytes());
  }
  {
    Writer writer;
    writeImu(writer, snapshot.imu);
    result.imu = sha256(writer.bytes());
  }
  {
    Writer writer;
    writeMap(writer, snapshot.map);
    result.map = sha256(writer.bytes());
  }
  {
    Writer writer;
    writer.pod(snapshot.boundary_timestamp_s);
    writeLifecycle(writer, snapshot.lifecycle);
    result.lifecycle = sha256(writer.bytes());
  }
  {
    const std::string bytes = encode(snapshot);
    result.full = sha256(bytes);
  }
  return result;
}

bool saveSnapshot(const P4ForkSnapshot &snapshot, const std::string &path,
                  std::string &error)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output)
  {
    error = "cannot open snapshot for writing: " + path;
    return false;
  }
  const std::string bytes = encode(snapshot);
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (!output)
  {
    error = "failed writing snapshot: " + path;
    return false;
  }
  error.clear();
  return true;
}

bool loadSnapshot(const std::string &path, P4ForkSnapshot &snapshot,
                  std::string &error)
{
  std::ifstream input(path, std::ios::binary);
  if (!input)
  {
    error = "cannot open snapshot for reading: " + path;
    return false;
  }
  input.seekg(0, std::ios::end);
  const std::streamoff size = input.tellg();
  if (size < 0)
  {
    error = "failed reading snapshot: " + path;
    return false;
  }
  input.seekg(0, std::ios::beg);
  std::string bytes(static_cast<std::size_t>(size), '\0');
  input.read(bytes.data(), size);
  if (!input)
  {
    error = "failed reading snapshot: " + path;
    return false;
  }
  try
  {
    decode(bytes, snapshot);
  }
  catch (const std::exception &exception)
  {
    error = exception.what();
    return false;
  }
  error.clear();
  return true;
}

} // namespace p4b
} // namespace fast_livo
