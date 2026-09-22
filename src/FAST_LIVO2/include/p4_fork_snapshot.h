#ifndef P4_FORK_SNAPSHOT_H_
#define P4_FORK_SNAPSHOT_H_

#include "IMU_Processing.h"
#include "voxel_map.h"

#include <string>

namespace fast_livo
{
namespace p4b
{

struct MapperLifecycleSnapshot
{
  bool lidar_map_initialized = false;
  bool gravity_alignment_finished = false;
  int lidar_map_update_counter = 0;
  bool lidar_map_guard_active = false;
  bool lidar_map_guard_hard_limit_latched = false;
  int lidar_map_guard_recovery_frames = 0;
  int lidar_map_guard_freeze_frames = 0;
  int external_update_pause_map_frames = 0;
  double lidar_frame_begin_time = 0.0;
  double lidar_frame_end_time = 0.0;
  double last_lidar_update_time = 0.0;
  int lidar_scan_index = 0;
};

struct P4ForkSnapshot
{
  static constexpr std::uint32_t kFormatVersion = 2;

  double boundary_timestamp_s = 0.0;
  StatesGroup state;
  StatesGroup propagated_state;
  ImuProcess::Snapshot imu;
  VoxelMapManagerSnapshot map;
  MapperLifecycleSnapshot lifecycle;
};

struct LogicalHashes
{
  std::string state;
  std::string covariance;
  std::string imu;
  std::string map;
  std::string lifecycle;
  std::string full;

  bool operator==(const LogicalHashes &other) const;
};

LogicalHashes logicalHashes(const P4ForkSnapshot &snapshot);
bool saveSnapshot(const P4ForkSnapshot &snapshot, const std::string &path,
                  std::string &error);
bool loadSnapshot(const std::string &path, P4ForkSnapshot &snapshot,
                  std::string &error);

} // namespace p4b
} // namespace fast_livo

#endif
