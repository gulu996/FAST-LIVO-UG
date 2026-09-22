#include "p4_fork_harness.h"

#include "voxel_filter_utils.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <set>
#include <thread>

namespace fast_livo
{
namespace p4b
{
namespace
{

struct MapCounts
{
  std::size_t nodes = 0;
  std::size_t points = 0;
  std::size_t updated = 0;
  std::size_t new_points = 0;
};

} // namespace

class CapturePauseGuard
{
public:
  CapturePauseGuard(const HarnessConfig &config, std::string &error)
      : enabled_(config.pause_input_during_capture),
        directory_(config.output_directory)
  {
    if (!enabled_) return;
    const std::filesystem::path request =
        std::filesystem::path(directory_) / "capture_pause.request";
    const std::filesystem::path ack =
        std::filesystem::path(directory_) / "capture_pause.ack";
    const std::filesystem::path done =
        std::filesystem::path(directory_) / "capture_pause.done";
    const std::filesystem::path resumed =
        std::filesystem::path(directory_) / "capture_pause.resumed";
    std::error_code ignored;
    std::filesystem::remove(ack, ignored);
    std::filesystem::remove(done, ignored);
    std::filesystem::remove(resumed, ignored);
    std::ofstream marker(request);
    marker << "pause owned rosbag process group\n";
    marker.close();
    for (int attempt = 0; attempt < 1000; ++attempt)
    {
      if (std::filesystem::is_regular_file(ack))
      {
        ready_ = true;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    error = "capture pause handshake timed out";
  }

  ~CapturePauseGuard()
  {
    if (!enabled_) return;
    std::ofstream done(
        std::filesystem::path(directory_) / "capture_pause.done");
    done << (ready_ ? "capture complete\n" : "capture aborted\n");
    done.close();
    std::error_code ignored;
    std::filesystem::remove(
        std::filesystem::path(directory_) / "capture_pause.request", ignored);
    if (ready_)
    {
      const std::filesystem::path resumed =
          std::filesystem::path(directory_) / "capture_pause.resumed";
      for (int attempt = 0; attempt < 1000 && !std::filesystem::exists(resumed);
           ++attempt)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  bool ready() const { return !enabled_ || ready_; }

private:
  bool enabled_ = false;
  bool ready_ = false;
  std::string directory_;
};

namespace
{

void countNode(const VoxelOctoTree *node, MapCounts &counts)
{
  if (!node) return;
  ++counts.nodes;
  if (node->plane_ptr_ && node->plane_ptr_->is_plane_)
    counts.points += std::max<std::size_t>(
        node->temp_points_.size(), node->plane_ptr_->points_size_);
  else if (node->octo_state_ == 0)
    counts.points += node->temp_points_.size();
  counts.new_points += std::max(0, node->new_points_);
  if (node->plane_ptr_ && node->plane_ptr_->is_update_) ++counts.updated;
  for (const VoxelOctoTree *child : node->leaves_) countNode(child, counts);
}

double stateMaxDifference(const StatesGroup &a, const StatesGroup &b)
{
  return std::max({(a.rot_end - b.rot_end).cwiseAbs().maxCoeff(),
                   (a.pos_end - b.pos_end).cwiseAbs().maxCoeff(),
                   (a.vel_end - b.vel_end).cwiseAbs().maxCoeff(),
                   (a.bias_g - b.bias_g).cwiseAbs().maxCoeff(),
                   (a.bias_a - b.bias_a).cwiseAbs().maxCoeff(),
                   (a.gravity - b.gravity).cwiseAbs().maxCoeff(),
                   std::abs(a.inv_expo_time - b.inv_expo_time)});
}

double covarianceMaxDifference(const StatesGroup &a, const StatesGroup &b)
{
  return (a.cov - b.cov).cwiseAbs().maxCoeff();
}

double matrixMaxDifference(const Eigen::MatrixXd &a,
                           const Eigen::MatrixXd &b)
{
  return (a - b).cwiseAbs().maxCoeff();
}

bool associationsEqual(
    const std::vector<fast_livo::p4::Association> &a,
    const std::vector<fast_livo::p4::Association> &b)
{
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (a[i].point_index != b[i].point_index ||
        a[i].plane_id != b[i].plane_id ||
        a[i].voxel_x != b[i].voxel_x ||
        a[i].voxel_y != b[i].voxel_y ||
        a[i].voxel_z != b[i].voxel_z ||
        !(a[i].normal.array() == b[i].normal.array()).all() ||
        a[i].residual != b[i].residual)
      return false;
  return true;
}

std::string cloudHash(const PointCloudXYZI &cloud)
{
  P4ForkSnapshot snapshot;
  snapshot.imu.cur_pcl_un = cloud;
  return logicalHashes(snapshot).imu;
}

std::vector<fast_livo::p4::Association> associations(
    const std::vector<PointToPlane> &matches)
{
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
    value.normal = match.normal_;
    value.residual = match.dis_to_plane_;
    result.push_back(value);
  }
  std::sort(result.begin(), result.end(), [](const auto &a, const auto &b) {
    if (a.point_index != b.point_index) return a.point_index < b.point_index;
    if (a.voxel_x != b.voxel_x) return a.voxel_x < b.voxel_x;
    if (a.voxel_y != b.voxel_y) return a.voxel_y < b.voxel_y;
    if (a.voxel_z != b.voxel_z) return a.voxel_z < b.voxel_z;
    return a.plane_id < b.plane_id;
  });
  return result;
}

std::array<double, 5> provenanceAgeRatios(
    const std::vector<PointToPlane> &matches, double timestamp_s)
{
  std::array<double, 5> counts{{0, 0, 0, 0, 0}};
  double total = 0.0;
  for (const PointToPlane &match : matches)
  {
    if (!match.source_plane_) continue;
    for (double source_time : match.source_plane_->p4_source_timestamps_s_)
    {
      if (!std::isfinite(source_time)) continue;
      const double age = std::max(0.0, timestamp_s - source_time);
      std::size_t bucket = age < 0.5 ? 0 : age < 2.0 ? 1 :
                           age < 5.0 ? 2 : age < 20.0 ? 3 : 4;
      counts[bucket] += 1.0;
      total += 1.0;
    }
  }
  if (total > 0.0)
    for (double &value : counts) value /= total;
  return counts;
}

std::pair<double, double> planeDifferences(
    const std::vector<PointToPlane> &baseline,
    const std::vector<PointToPlane> &candidate)
{
  std::unordered_map<int, const PointToPlane *> by_point;
  for (const PointToPlane &match : baseline)
    by_point[match.point_index_] = &match;
  double center_sum = 0.0;
  double angle_sum = 0.0;
  std::size_t count = 0;
  for (const PointToPlane &match : candidate)
  {
    const auto found = by_point.find(match.point_index_);
    if (found == by_point.end()) continue;
    const PointToPlane &prior = *found->second;
    center_sum += (prior.center_ - match.center_).norm();
    const double denominator = prior.normal_.norm() * match.normal_.norm();
    if (denominator > 1e-15)
    {
      const double cosine = std::max(-1.0, std::min(
          1.0, std::abs(prior.normal_.dot(match.normal_) / denominator)));
      angle_sum += std::acos(cosine) * 57.29577951308232;
    }
    ++count;
  }
  return count ? std::make_pair(center_sum / count, angle_sum / count)
               : std::make_pair(0.0, 0.0);
}

} // namespace

bool P4ForkHarness::beginCapture(std::string &error)
{
  if (capture_pause_) return capture_pause_->ready();
  std::error_code filesystem_error;
  std::filesystem::create_directories(config_.output_directory,
                                      filesystem_error);
  if (filesystem_error)
  {
    error = filesystem_error.message();
    return false;
  }
  capture_pause_.reset(new CapturePauseGuard(config_, error));
  return capture_pause_->ready();
}

bool P4ForkHarness::beginForkFrame(std::string &error)
{
  if (capture_pause_) return capture_pause_->ready();
  capture_pause_.reset(new CapturePauseGuard(config_, error));
  return capture_pause_->ready();
}

void P4ForkHarness::endForkFrame()
{
  capture_pause_.reset();
}

struct P4ForkHarness::Branch
{
  Branch(const P4ForkSnapshot &snapshot, VoxelMapConfig config, Mode mode)
      : mode(mode), state(snapshot.state), propagated(snapshot.propagated_state),
        lifecycle(snapshot.lifecycle), plane_id_counter(snapshot.map.next_plane_id)
  {
    config.p4_frontend_diagnostics_enable = false;
    config.p4b_snapshot_enable = true;
    config.p4b_retain_support_points = true;
    config.deterministic_lio_update_en = true;
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> empty;
    map.reset(new VoxelMapManager(config, empty));
    map->restoreSnapshot(snapshot.map);
    if (mode == Mode::RecentPointExclusion)
      map->setP4bRecentPointExclusion(5);
    imu.reset(new ImuProcess());
    imu->restoreSnapshot(snapshot.imu);
  }

  Mode mode;
  StatesGroup state;
  StatesGroup propagated;
  MapperLifecycleSnapshot lifecycle;
  int plane_id_counter = 0;
  ImuProcessPtr imu;
  VoxelMapManagerPtr map;
};

P4ForkHarness::P4ForkHarness(const HarnessConfig &config) : config_(config) {}
P4ForkHarness::~P4ForkHarness() = default;

bool P4ForkHarness::shouldCapture(double relative_time_s) const
{
  return config_.enabled && !captured_ &&
         relative_time_s >= config_.snapshot_relative_time_s;
}

bool P4ForkHarness::capture(const P4ForkSnapshot &snapshot,
                            const VoxelMapConfig &voxel_config,
                            std::uint64_t accepted_vio_updates,
                            std::string &error)
{
  struct ResumeOnReturn
  {
    explicit ResumeOnReturn(P4ForkHarness &harness) : harness(harness) {}
    ~ResumeOnReturn() { harness.capture_pause_.reset(); }
    P4ForkHarness &harness;
  } resume_on_return(*this);
  if (!config_.enabled || captured_)
  {
    error = "P4-B capture requested in invalid state";
    return false;
  }
  if (config_.map_update_stride != 1 || voxel_config.map_guard_mode != "off" ||
      snapshot.lifecycle.external_update_pause_map_frames != 0)
  {
    error = "P4-B requires map_update_stride=1, map_guard=off, and no external map pause";
    return false;
  }
  std::error_code filesystem_error;
  std::filesystem::create_directories(config_.output_directory,
                                      filesystem_error);
  if (filesystem_error)
  {
    error = filesystem_error.message();
    return false;
  }
  if (!beginCapture(error)) return false;
  const std::string disk_path =
      config_.output_directory + "/p4b_snapshot.bin";
  if (!saveSnapshot(snapshot, disk_path, error)) return false;
  P4ForkSnapshot disk_snapshot;
  if (!loadSnapshot(disk_path, disk_snapshot, error)) return false;
  const std::string disk_full_hash = logicalHashes(disk_snapshot).full;
  if (!config_.run_all_branches)
    disk_snapshot = P4ForkSnapshot();

  b0_.reset(new Branch(snapshot, voxel_config, Mode::Evolving));
  if (config_.run_all_branches)
  {
    b1_.reset(new Branch(disk_snapshot, voxel_config, Mode::Frozen));
    b2_.reset(new Branch(snapshot, voxel_config,
                         Mode::RecentPointExclusion));
  }

  auto branch_snapshot = [&snapshot](const Branch &branch) {
    P4ForkSnapshot value = snapshot;
    value.state = branch.state;
    value.propagated_state = branch.propagated;
    value.imu = branch.imu->captureSnapshot();
    value.map = branch.map->captureSnapshot();
    value.map.next_plane_id = branch.plane_id_counter;
    value.lifecycle = branch.lifecycle;
    return value;
  };
  const LogicalHashes expected = logicalHashes(snapshot);
  const LogicalHashes b0_hashes = logicalHashes(branch_snapshot(*b0_));
  bool parity = expected == b0_hashes;
  LogicalHashes b1_hashes;
  LogicalHashes b2_hashes;
  if (config_.run_all_branches)
  {
    b1_hashes = logicalHashes(branch_snapshot(*b1_));
    b2_hashes = logicalHashes(branch_snapshot(*b2_));
    parity = parity && expected == b1_hashes && expected == b2_hashes;
  }
  parity_txt_.open(config_.output_directory + "/p4b_parity.txt",
                   std::ios::out | std::ios::trunc);
  parity_txt_ << "SNAPSHOT_PARITY=" << (parity ? "PASS" : "FAIL") << '\n'
              << "state_sha256=" << expected.state << '\n'
              << "covariance_sha256=" << expected.covariance << '\n'
              << "imu_sha256=" << expected.imu << '\n'
              << "map_sha256=" << expected.map << '\n'
              << "lifecycle_sha256=" << expected.lifecycle << '\n'
              << "full_sha256=" << expected.full << '\n'
              << "disk_full_sha256=" << disk_full_hash << '\n'
              << "run_all_branches=" << config_.run_all_branches << '\n'
              << "first_expected_divergence_frame="
              << (config_.run_all_branches ? 1 : -1) << '\n';
  parity_txt_.flush();
  if (!parity)
  {
    error = "logical hashes differ immediately after fork";
    b0_.reset(); b1_.reset(); b2_.reset();
    return false;
  }

  frame_csv_.open(config_.output_directory + "/p4b_frames.csv",
                  std::ios::out | std::ios::trunc);
  validation_csv_.open(config_.output_directory + "/p4b_validation.csv",
                       std::ios::out | std::ios::trunc);
  if (!frame_csv_ || !validation_csv_)
  {
    error = "cannot open P4-B output CSV";
    return false;
  }
  frame_csv_ << "timestamp_s,relative_time_s,branch,commit,position_x,position_y,position_z,rotation_qx,rotation_qy,rotation_qz,rotation_qw,velocity_x,velocity_y,velocity_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,gravity_x,gravity_y,gravity_z,correspondences,plane_count,translation_lambda_0,translation_lambda_1,translation_lambda_2,weak_x,weak_y,weak_z,residual_median_m,residual_p95_m,signed_weak_projection_m,map_voxels,map_nodes,map_points,updated_voxels,new_points,age_lt_0_5_ratio,age_0_5_2_ratio,age_2_5_ratio,age_5_20_ratio,age_gt_20_ratio,map_sha256,b0_position_separation_m,b0_rotation_separation_deg,b0_speed_separation_mps,association_overlap,mean_normal_cosine,mean_plane_center_difference_m,mean_plane_normal_difference_deg\n";
  validation_csv_ << "timestamp_s,kind,state_max_abs,covariance_max_abs,deskew_hash_equal,correspondence_equal,information_max_abs,rhs_max_abs,pass\n";
  accepted_vio_at_capture_ = accepted_vio_updates;
  captured_ = true;
  error.clear();
  return true;
}

LidarMeasureGroup P4ForkHarness::deepCloneMeasurement(
    const LidarMeasureGroup &source)
{
  LidarMeasureGroup result;
  result.lidar_frame_beg_time = source.lidar_frame_beg_time;
  result.lidar_frame_end_time = source.lidar_frame_end_time;
  result.last_lio_update_time = source.last_lio_update_time;
  result.lio_vio_flg = source.lio_vio_flg;
  result.lidar_scan_index_now = source.lidar_scan_index_now;
  *result.lidar = *source.lidar;
  *result.pcl_proc_cur = *source.pcl_proc_cur;
  *result.pcl_proc_next = *source.pcl_proc_next;
  for (const MeasureGroup &measure : source.measures)
  {
    MeasureGroup clone;
    clone.vio_time = measure.vio_time;
    clone.lio_time = measure.lio_time;
    clone.img = measure.img.clone();
    for (const sensor_msgs::Imu::ConstPtr &imu : measure.imu)
      clone.imu.push_back(imu ? sensor_msgs::Imu::ConstPtr(
                                  new sensor_msgs::Imu(*imu))
                              : sensor_msgs::Imu::ConstPtr());
    result.measures.push_back(std::move(clone));
  }
  return result;
}

P4ForkHarness::Result P4ForkHarness::runBranch(
    Branch &branch, const LidarMeasureGroup &immutable_measurement)
{
  Result result;
  const int production_plane_counter = p4bGetVoxelPlaneIdCounter();
  p4bSetVoxelPlaneIdCounter(branch.plane_id_counter);
  LidarMeasureGroup measurement = deepCloneMeasurement(immutable_measurement);
  PointCloudXYZI::Ptr undistorted(new PointCloudXYZI());
  branch.imu->Process2(measurement, branch.state, undistorted);
  if (undistorted->empty())
  {
    branch.plane_id_counter = p4bGetVoxelPlaneIdCounter();
    p4bSetVoxelPlaneIdCounter(production_plane_counter);
    return result;
  }
  branch.propagated = branch.state;
  PointCloudXYZI::Ptr down_body(new PointCloudXYZI());
  VoxelSafetyContext context;
  context.coordinate_frame = "p4b fork lidar/body";
  context.enforce_max_range = true;
  context.max_range_m = config_.lidar_max_range_m;
  if (!safeVoxelFilter<PointType>(
          undistorted, down_body,
          Eigen::Vector3f::Constant(
              static_cast<float>(config_.voxel_leaf_size_m)),
          "P4B_FORK", context))
  {
    branch.plane_id_counter = p4bGetVoxelPlaneIdCounter();
    p4bSetVoxelPlaneIdCounter(production_plane_counter);
    return result;
  }
  std::sort(down_body->points.begin(), down_body->points.end(),
            [](const PointType &a, const PointType &b) {
              if (a.x != b.x) return a.x < b.x;
              if (a.y != b.y) return a.y < b.y;
              return a.z < b.z;
            });
  pcl::PointCloud<pcl::PointXYZI>::Ptr down_world(
      new pcl::PointCloud<pcl::PointXYZI>());
  branch.map->state_ = branch.state;
  branch.map->feats_undistort_ = undistorted;
  branch.map->feats_down_body_ = down_body;
  branch.map->feats_down_size_ = static_cast<int>(down_body->size());
  branch.map->TransformLidar(branch.state.rot_end, branch.state.pos_end,
                             down_body, down_world);
  branch.map->StateEstimation(branch.propagated,
                              measurement.last_lio_update_time);
  branch.state = branch.map->state_;

  const LioUpdateDiagnostics diagnostics = branch.map->getLastLioDiagnostics();
  result.valid = true;
  result.timestamp_s = measurement.last_lio_update_time;
  result.state = branch.state;
  result.deskew_hash = cloudHash(*undistorted);
  result.diagnostics = diagnostics;
  result.associations = associations(branch.map->ptpl_list_);
  result.matches = branch.map->ptpl_list_;
  result.provenance_age_ratios = provenanceAgeRatios(
      result.matches, measurement.last_lio_update_time);
  // The source planes belong to the pre-insertion map. Freeze all values above
  // before mutation, then make accidental post-mutation dereference impossible.
  for (PointToPlane &match : result.matches) match.source_plane_ = nullptr;

  if (diagnostics.commit && branch.mode != Mode::Frozen)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr world(
        new pcl::PointCloud<pcl::PointXYZI>());
    branch.map->TransformLidar(branch.state.rot_end, branch.state.pos_end,
                               down_body, world);
    for (std::size_t i = 0; i < world->size(); ++i)
    {
      pointWithVar &point = branch.map->pv_list_[i];
      point.point_w = V3D(world->points[i].x, world->points[i].y,
                          world->points[i].z);
      M3D covariance = branch.map->body_cov_list_[i];
      const M3D cross = branch.map->cross_mat_list_[i];
      const M3D rotation = branch.state.rot_end * branch.map->extR_;
      point.var = rotation * covariance * rotation.transpose() +
          (-cross) * branch.state.cov.block<3, 3>(0, 0) *
              (-cross).transpose() +
          branch.state.cov.block<3, 3>(3, 3);
      point.source_frame_id = branch.map->current_frame_id_;
      point.source_point_index = static_cast<int>(i);
      point.source_timestamp_s = measurement.last_lio_update_time;
      point.source_origin_w = branch.state.pos_end;
    }
    branch.map->UpdateVoxelMap(branch.map->pv_list_);
    branch.map->markLastLioMapInserted(true);
    if (config_.map_sliding_enabled) branch.map->mapSliding();
  }

  MapCounts counts;
  for (const auto &entry : branch.map->voxel_map_) countNode(entry.second, counts);
  for (const auto &entry : branch.map->long_term_visual_map_)
    countNode(entry.second, counts);
  result.map_voxels = branch.map->voxel_map_.size() +
                      branch.map->long_term_visual_map_.size();
  result.map_nodes = counts.nodes;
  result.map_points = counts.points;
  result.updated_voxels = branch.map->p4bCurrentUpdatedVoxelCount();
  result.new_points = diagnostics.commit && branch.mode != Mode::Frozen
      ? branch.map->pv_list_.size() : 0;
  if (first_packet_)
  {
    P4ForkSnapshot map_snapshot;
    map_snapshot.map = branch.map->captureSnapshot();
    map_snapshot.map.next_plane_id = p4bGetVoxelPlaneIdCounter();
    result.map_hash = logicalHashes(map_snapshot).map;
  }
  branch.plane_id_counter = p4bGetVoxelPlaneIdCounter();
  p4bSetVoxelPlaneIdCounter(production_plane_counter);
  return result;
}

void P4ForkHarness::processImmutableLioPacket(
    const LidarMeasureGroup &measurement, double relative_time_s)
{
  if (!captured_ || finished_ || measurement.lio_vio_flg == VIO) return;
  std::vector<std::pair<std::string, Result>> results;
  results.emplace_back("B0_EVOLVING", runBranch(*b0_, measurement));
  pending_b0_ = results.front().second;
  if (config_.run_all_branches)
  {
    results.emplace_back("B1_FROZEN", runBranch(*b1_, measurement));
    results.emplace_back("B2_RECENT5", runBranch(*b2_, measurement));
    if (first_packet_)
      writeValidation("DISK_RELOAD_NEXT_FRAME", results[0].second,
                      results[1].second);
    auto record_map_divergence = [&](std::size_t index, bool &observed,
                                     Branch &branch) {
      if (observed) return;
      const auto churn = fast_livo::p4::correspondenceChurn(
          results[0].second.associations, results[index].second.associations);
      const bool trigger = first_packet_ || churn.retained_ratio < 1.0 - 1e-12 ||
          (results[index].second.state.pos_end -
           results[0].second.state.pos_end).norm() > 1e-12;
      if (!trigger) return;
      auto map_hash = [](const Branch &value) {
        P4ForkSnapshot snapshot;
        snapshot.map = value.map->captureSnapshot();
        snapshot.map.next_plane_id = value.plane_id_counter;
        return logicalHashes(snapshot).map;
      };
      const std::string baseline_hash = results[0].second.map_hash.empty()
          ? map_hash(*b0_) : results[0].second.map_hash;
      const std::string candidate_hash = results[index].second.map_hash.empty()
          ? map_hash(branch) : results[index].second.map_hash;
      if (baseline_hash != candidate_hash)
      {
        observed = true;
        parity_txt_ << "first_observed_map_divergence_"
                    << results[index].first << "=" << std::setprecision(17)
                    << relative_time_s << '\n';
        parity_txt_.flush();
      }
    };
    record_map_divergence(1, b1_map_diverged_, *b1_);
    record_map_divergence(2, b2_map_diverged_, *b2_);
  }
  writeResults(relative_time_s, results);
  first_packet_ = false;
  if (relative_time_s >= config_.end_relative_time_s)
  {
    finished_ = true;
    if (parity_txt_)
    {
      parity_txt_ << "HARNESS_HORIZON_COMPLETE=YES\n";
      parity_txt_.flush();
    }
  }
}

void P4ForkHarness::observeProductionResult(
    const StatesGroup &state, const VoxelMapManager &map,
    const PointCloudXYZI &undistorted, double timestamp_s,
    std::uint64_t accepted_vio_updates)
{
  if (!pending_b0_.valid ||
      std::abs(timestamp_s - pending_b0_.timestamp_s) > 1e-6)
    return;
  Result production;
  production.valid = true;
  production.timestamp_s = timestamp_s;
  production.state = state;
  production.deskew_hash = cloudHash(undistorted);
  production.diagnostics = map.getLastLioDiagnostics();
  production.associations = associations(map.ptpl_list_);
  writeValidation("MEMORY_RESTORE_NEXT_FRAME", production, pending_b0_);
  if (parity_txt_ && accepted_vio_updates > accepted_vio_at_capture_)
    parity_txt_ << "VIO_ACCEPTED_DURING_FORK="
                << (accepted_vio_updates - accepted_vio_at_capture_) << '\n';
  pending_b0_ = Result();
}

void P4ForkHarness::writeValidation(const char *kind, const Result &reference,
                                    const Result &candidate)
{
  const double state_difference =
      stateMaxDifference(reference.state, candidate.state);
  const double covariance_difference =
      covarianceMaxDifference(reference.state, candidate.state);
  const bool deskew_equal = reference.deskew_hash.empty() ||
      candidate.deskew_hash.empty() ||
      reference.deskew_hash == candidate.deskew_hash;
  const bool correspondence_equal =
      associationsEqual(reference.associations, candidate.associations);
  const double information_difference = matrixMaxDifference(
      reference.diagnostics.lidar_geometry_information,
      candidate.diagnostics.lidar_geometry_information);
  const double rhs_difference = matrixMaxDifference(
      reference.diagnostics.lidar_geometry_rhs,
      candidate.diagnostics.lidar_geometry_rhs);
  const bool pass = state_difference <= 1e-10 &&
                    covariance_difference <= 1e-10 && deskew_equal &&
                    correspondence_equal && information_difference <= 1e-10 &&
                    rhs_difference <= 1e-10;
  validation_csv_ << std::setprecision(17) << candidate.timestamp_s << ','
                  << kind << ',' << state_difference << ','
                  << covariance_difference << ',' << deskew_equal << ','
                  << correspondence_equal << ',' << information_difference
                  << ',' << rhs_difference << ',' << pass << '\n';
  validation_csv_.flush();
}

void P4ForkHarness::writeResults(
    double relative_time_s,
    const std::vector<std::pair<std::string, Result>> &results)
{
  if (results.empty() || !results.front().second.valid) return;
  const Result &baseline = results.front().second;
  Eigen::Quaterniond baseline_q(baseline.state.rot_end);
  for (const auto &entry : results)
  {
    const Result &value = entry.second;
    if (!value.valid) continue;
    const auto churn = fast_livo::p4::correspondenceChurn(
        baseline.associations, value.associations);
    const auto plane_difference = planeDifferences(
        baseline.matches, value.matches);
    Eigen::Quaterniond q(value.state.rot_end);
    const double rotation_separation =
        baseline_q.angularDistance(q) * 57.29577951308232;
    const auto &diagnostics = value.diagnostics;
    const auto &geometry = diagnostics.observability;
    std::set<int> planes;
    for (const auto &association : value.associations)
      planes.insert(association.plane_id);
    frame_csv_ << std::setprecision(17) << value.timestamp_s << ','
        << relative_time_s << ',' << entry.first << ','
        << diagnostics.commit << ',' << value.state.pos_end.x() << ','
        << value.state.pos_end.y() << ',' << value.state.pos_end.z() << ','
        << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
        << value.state.vel_end.x() << ',' << value.state.vel_end.y() << ','
        << value.state.vel_end.z() << ',' << value.state.bias_g.x() << ','
        << value.state.bias_g.y() << ',' << value.state.bias_g.z() << ','
        << value.state.bias_a.x() << ',' << value.state.bias_a.y() << ','
        << value.state.bias_a.z() << ',' << value.state.gravity.x() << ','
        << value.state.gravity.y() << ',' << value.state.gravity.z() << ','
        << diagnostics.correspondence_count << ',' << planes.size() << ','
        << geometry.translation_eigenvalues.x() << ','
        << geometry.translation_eigenvalues.y() << ','
        << geometry.translation_eigenvalues.z() << ','
        << geometry.weak_translation_direction_world.x() << ','
        << geometry.weak_translation_direction_world.y() << ','
        << geometry.weak_translation_direction_world.z() << ','
        << diagnostics.median_abs_point_plane_residual << ','
        << diagnostics.p95_abs_point_plane_residual << ','
        << diagnostics.position_correction_on_weak_direction << ','
        << value.map_voxels << ',' << value.map_nodes << ','
        << value.map_points << ',' << value.updated_voxels << ','
        << value.new_points << ','
        << value.provenance_age_ratios[0] << ','
        << value.provenance_age_ratios[1] << ','
        << value.provenance_age_ratios[2] << ','
        << value.provenance_age_ratios[3] << ','
        << value.provenance_age_ratios[4] << ','
        << value.map_hash << ','
        << (value.state.pos_end - baseline.state.pos_end).norm() << ','
        << rotation_separation << ','
        << (value.state.vel_end.norm() - baseline.state.vel_end.norm()) << ','
        << churn.retained_ratio << ',' << churn.mean_normal_cosine << ','
        << plane_difference.first << ',' << plane_difference.second << '\n';
  }
  frame_csv_.flush();
}

} // namespace p4b
} // namespace fast_livo
