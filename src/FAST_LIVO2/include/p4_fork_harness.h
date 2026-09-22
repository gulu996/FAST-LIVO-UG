#ifndef P4_FORK_HARNESS_H_
#define P4_FORK_HARNESS_H_

#include "p4_fork_snapshot.h"

#include <fstream>
#include <memory>

namespace fast_livo
{
namespace p4b
{

class CapturePauseGuard;

struct HarnessConfig
{
  bool enabled = false;
  bool run_all_branches = false;
  bool pause_input_during_capture = false;
  double snapshot_relative_time_s = 895.0;
  double end_relative_time_s = 910.0;
  std::string output_directory;
  double voxel_leaf_size_m = 0.5;
  double lidar_max_range_m = 450.0;
  bool map_sliding_enabled = true;
  int map_update_stride = 1;
};

class P4ForkHarness
{
public:
  explicit P4ForkHarness(const HarnessConfig &config);
  ~P4ForkHarness();

  bool enabled() const { return config_.enabled; }
  bool captured() const { return captured_; }
  bool finished() const { return finished_; }
  bool shouldCapture(double relative_time_s) const;
  bool beginCapture(std::string &error);
  bool beginForkFrame(std::string &error);
  void endForkFrame();
  bool capture(const P4ForkSnapshot &snapshot,
               const VoxelMapConfig &voxel_config,
               std::uint64_t accepted_vio_updates,
               std::string &error);
  void processImmutableLioPacket(const LidarMeasureGroup &measurement,
                                 double relative_time_s);
  void observeProductionResult(const StatesGroup &state,
                               const VoxelMapManager &map,
                               const PointCloudXYZI &undistorted,
                               double timestamp_s,
                               std::uint64_t accepted_vio_updates);

private:
  enum class Mode { Evolving, Frozen, RecentPointExclusion };

  struct Result
  {
    bool valid = false;
    double timestamp_s = 0.0;
    StatesGroup state;
    std::string deskew_hash;
    LioUpdateDiagnostics diagnostics;
    std::vector<fast_livo::p4::Association> associations;
    std::vector<PointToPlane> matches;
    std::array<double, 5> provenance_age_ratios{{0, 0, 0, 0, 0}};
    std::size_t map_voxels = 0;
    std::size_t map_nodes = 0;
    std::size_t map_points = 0;
    std::size_t updated_voxels = 0;
    std::size_t new_points = 0;
    std::string map_hash;
  };

  struct Branch;

  static LidarMeasureGroup deepCloneMeasurement(
      const LidarMeasureGroup &measurement);
  Result runBranch(Branch &branch, const LidarMeasureGroup &measurement);
  void writeResults(double relative_time_s,
                    const std::vector<std::pair<std::string, Result>> &results);
  void writeValidation(const char *kind, const Result &reference,
                       const Result &candidate);

  HarnessConfig config_;
  bool captured_ = false;
  bool finished_ = false;
  std::unique_ptr<Branch> b0_;
  std::unique_ptr<Branch> b1_;
  std::unique_ptr<Branch> b2_;
  Result pending_b0_;
  bool first_packet_ = true;
  bool b1_map_diverged_ = false;
  bool b2_map_diverged_ = false;
  std::uint64_t accepted_vio_at_capture_ = 0;
  std::ofstream frame_csv_;
  std::ofstream validation_csv_;
  std::ofstream parity_txt_;
  std::unique_ptr<CapturePauseGuard> capture_pause_;
};

} // namespace p4b
} // namespace fast_livo

#endif
