//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"
#include "livox_ros_driver2/timeshare_path.h"

#include <inttypes.h>
#include <algorithm>
#include <atomic>
#include <errno.h>
#include <exception>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "include/ros_headers.h"

#include "driver_node.h"
#include "lds_lidar.h"

#ifdef BUILDING_ROS1
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#endif

namespace livox_ros
{
namespace {

#ifdef BUILDING_ROS1
ros::Time RosTimeFromNs(uint64_t timestamp_ns) {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);
  return stamp;
}

diagnostic_msgs::KeyValue DiagnosticValue(const std::string& key,
                                          const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
#endif

uint32_t SharedClockSourceFromTimestampType(uint8_t timestamp_type) {
  switch (timestamp_type) {
    case kTimestampTypeGptpOrPtp:
      return kLidarBaseTimeLegacyPtp;
    case kTimestampTypeGps:
      return kLidarBaseTimeLegacyGps;
    case kTimestampTypeNoSync:
      return kLidarBaseTimeLegacyNoSync;
    default:
      return kClockSourceUnknown;
  }
}

uint32_t ImuAnchorClockSourceFromTimestampType(uint8_t timestamp_type) {
  switch (timestamp_type) {
    case kTimestampTypeGptpOrPtp:
      return kImuAnchorDevicePtp;
    case kTimestampTypeGps:
      return kImuAnchorDeviceGps;
    default:
      return kImuAnchorClockUnknown;
  }
}

const char* TimestampTypeName(uint8_t timestamp_type) {
  switch (timestamp_type) {
    case kTimestampTypeGptpOrPtp:
      return "PTP";
    case kTimestampTypeGps:
      return "GPS";
    case kTimestampTypeNoSync:
      return "NoSync";
    default:
      return "Unknown";
  }
}

uint64_t MakeWriterEpoch() {
  struct timespec realtime = {};
  clock_gettime(CLOCK_REALTIME, &realtime);
  const uint64_t realtime_ns =
      static_cast<uint64_t>(realtime.tv_sec) * kNsPerSecond +
      static_cast<uint64_t>(realtime.tv_nsec);
  return realtime_ns ^ MonotonicNowNs() ^
         (static_cast<uint64_t>(getpid()) << 32U);
}

uint64_t MakeImuAnchorWriterEpoch() {
  static std::atomic<uint64_t> instance_counter{1};
  uint64_t epoch =
      MonotonicNowNs() ^ (static_cast<uint64_t>(getpid()) << 32U) ^
      instance_counter.fetch_add(1, std::memory_order_relaxed);
  return epoch == 0 ? 1 : epoch;
}

}  // namespace

/** Lidar Data Distribute Control--------------------------------------------*/
#ifdef BUILDING_ROS1
  Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
             double frq, std::string &frame_id, bool lidar_bag, bool imu_bag)
      : transfer_format_(format),
        use_multi_topic_(multi_topic),
        data_src_(data_src),
        output_type_(output_type),
        publish_frq_(frq),
        frame_id_(frame_id),
        enable_lidar_bag_(lidar_bag),
        enable_imu_bag_(imu_bag)
  {
    publish_period_ns_ = kNsPerSecond / publish_frq_;
    lds_ = nullptr;
    memset(private_pub_, 0, sizeof(private_pub_));
    memset(private_imu_pub_, 0, sizeof(private_imu_pub_));
    global_pub_ = nullptr;
    global_imu_pub_ = nullptr;
    cur_node_ = nullptr;
    bag_ = nullptr;
    last_lidar_time_type_.fill(0xff);
    last_imu_time_type_.fill(0xff);
  }
#elif defined BUILDING_ROS2
  Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
             double frq, std::string &frame_id)
      : transfer_format_(format),
        use_multi_topic_(multi_topic),
        data_src_(data_src),
        output_type_(output_type),
        publish_frq_(frq),
        frame_id_(frame_id)
  {
    publish_period_ns_ = kNsPerSecond / publish_frq_;
    lds_ = nullptr;
#if 0
  bag_ = nullptr;
#endif
  }
#endif

  Lddc::~Lddc()
  {
#ifdef BUILDING_ROS1
    if (global_pub_)
    {
      delete global_pub_;
    }

    if (global_imu_pub_)
    {
      delete global_imu_pub_;
    }
#endif

    PrepareExit();

#ifdef BUILDING_ROS1
    for (uint32_t i = 0; i < kMaxSourceLidar; i++)
    {
      if (private_pub_[i])
      {
        delete private_pub_[i];
      }
    }

    for (uint32_t i = 0; i < kMaxSourceLidar; i++)
    {
      if (private_imu_pub_[i])
      {
        delete private_imu_pub_[i];
      }
    }
#endif
    ShutdownImuAnchorWriter();
    ShutdownSharedTimestampState();
    std::cout << "lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  }

  void Lddc::SetRosNode(livox_ros::DriverNode *node)
  {
    cur_node_ = node;
#ifdef BUILDING_ROS1
    ros::NodeHandle private_node("~");
    std::string configured_timeshare_path;
    private_node.param<std::string>(
        "timeshare_path", configured_timeshare_path, "");
    try
    {
      timeshare_path_ =
          ResolveTimesharePath(configured_timeshare_path);
    }
    catch (const std::exception& error)
    {
      DRIVER_FATAL(*cur_node_, "Cannot resolve timeshare path: %s",
                   error.what());
      throw;
    }
    DRIVER_INFO(*cur_node_, "[TIMESHARE] role=writer path=%s",
                timeshare_path_.c_str());
    if (timeshare_path_.compare(0, 6, "/root/") == 0)
    {
      DRIVER_WARN(*cur_node_,
                  "[TIMESHARE] writer resolved under /root; "
                  "do not run sensor nodes with sudo.");
    }
    private_node.param("shared_timestamp_open_retry_sec",
                       shared_open_retry_sec_, 1.0);
    private_node.param("timestamp_diagnostic_period_sec",
                       diagnostic_period_sec_, 1.0);

    int shared_lidar_index = 0;
    int ready_samples = 3;
    double max_stamp_gap_sec = 3.0 / publish_frq_;
    double min_arrival_ratio = 0.25;
    double max_arrival_ratio = 3.0;
    private_node.param("shared_timestamp_lidar_index",
                       shared_lidar_index, 0);
    private_node.param("shared_timestamp_ready_samples", ready_samples, 3);
    private_node.param("shared_timestamp_max_stamp_gap_sec",
                       max_stamp_gap_sec, max_stamp_gap_sec);
    private_node.param("shared_timestamp_min_arrival_ratio",
                       min_arrival_ratio, 0.25);
    private_node.param("shared_timestamp_max_arrival_ratio",
                       max_arrival_ratio, 3.0);

    shared_lidar_index_ = static_cast<uint8_t>(
        std::max(0, std::min(shared_lidar_index,
                             static_cast<int>(kMaxSourceLidar - 1))));
    ready_samples = std::max(2, ready_samples);
    shared_open_retry_sec_ = std::max(0.1, shared_open_retry_sec_);
    diagnostic_period_sec_ = std::max(0.1, diagnostic_period_sec_);
    max_stamp_gap_sec = std::max(1.0 / publish_frq_, max_stamp_gap_sec);
    min_arrival_ratio = std::max(0.0, min_arrival_ratio);
    max_arrival_ratio = std::max(min_arrival_ratio, max_arrival_ratio);
    shared_ready_gate_.reset(new LegacyTimestampReadyGate(
        static_cast<uint32_t>(ready_samples),
        static_cast<uint64_t>(max_stamp_gap_sec * kNsPerSecond),
        static_cast<uint64_t>(publish_period_ns_ * min_arrival_ratio),
        static_cast<uint64_t>(publish_period_ns_ * max_arrival_ratio)));

    timestamp_diagnostic_pub_ =
        private_node.advertise<diagnostic_msgs::DiagnosticArray>(
            "timestamp_diagnostics", 10);
    InitializeSharedTimestampState();
    DRIVER_WARN(*cur_node_,
                "Shared timestamp interface is LIDAR_BASE_TIME_LEGACY; "
                "it is not a camera trigger association.");

    private_node.param("imu_anchor_enable", imu_anchor_enable_, true);
    std::string configured_imu_timeshare_path;
    private_node.param<std::string>(
        "imu_timeshare_path", configured_imu_timeshare_path, "");
    int imu_anchor_min_ready_samples = 3;
    int imu_anchor_uncertainty_ns = 10000000;
    double imu_anchor_max_stamp_step_s = 0.1;
    int imu_anchor_lidar_index = 0;
    private_node.param("imu_anchor_min_ready_samples",
                       imu_anchor_min_ready_samples, 3);
    private_node.param("imu_anchor_uncertainty_ns",
                       imu_anchor_uncertainty_ns, 10000000);
    private_node.param("imu_anchor_max_stamp_step_s",
                       imu_anchor_max_stamp_step_s, 0.1);
    private_node.param("imu_anchor_lidar_index",
                       imu_anchor_lidar_index, 0);
    private_node.param("imu_anchor_log_period_s",
                       imu_anchor_log_period_sec_, 20.0);

    imu_anchor_config_.min_ready_samples =
        static_cast<uint32_t>(std::max(2, imu_anchor_min_ready_samples));
    imu_anchor_config_.uncertainty_ns =
        static_cast<uint64_t>(std::max(1, imu_anchor_uncertainty_ns));
    imu_anchor_config_.max_stamp_step_ns = static_cast<uint64_t>(
        std::max(0.001, imu_anchor_max_stamp_step_s) * kNsPerSecond);
    imu_anchor_lidar_index_ = static_cast<uint8_t>(
        std::max(0, std::min(imu_anchor_lidar_index,
                             static_cast<int>(kMaxSourceLidar - 1))));
    imu_anchor_log_period_sec_ =
        std::max(1.0, imu_anchor_log_period_sec_);

    try
    {
      imu_timeshare_path_ = configured_imu_timeshare_path.empty()
          ? ResolveTimesharePath("") + "_imu"
          : ResolveTimesharePath(configured_imu_timeshare_path);
    }
    catch (const std::exception& error)
    {
      DRIVER_FATAL(*cur_node_, "Cannot resolve imu_timeshare_path: %s",
                   error.what());
      throw;
    }
    if (imu_timeshare_path_ == timeshare_path_)
    {
      DRIVER_FATAL(*cur_node_,
                   "imu_timeshare_path must not reuse camera timeshare: %s",
                   imu_timeshare_path_.c_str());
      throw std::invalid_argument(
          "imu_timeshare_path must differ from timeshare_path");
    }
    DRIVER_INFO(*cur_node_, "[IMU_ANCHOR] role=writer path=%s",
                imu_timeshare_path_.c_str());
    if (imu_anchor_enable_)
    {
      imu_anchor_writer_.reset(
          new ImuTimeAnchorWriter(imu_anchor_config_));
      InitializeImuAnchorWriter();
    }
#endif
  }

  bool Lddc::InitializeImuAnchorWriter()
  {
#ifndef BUILDING_ROS1
    return false;
#else
    if (!imu_anchor_enable_ || !imu_anchor_writer_)
    {
      return false;
    }
    if (imu_anchor_writer_->is_open())
    {
      return true;
    }
    last_imu_anchor_open_attempt_ns_ = MonotonicNowNs();
    imu_anchor_writer_epoch_ = MakeImuAnchorWriterEpoch();
    std::string error;
    if (!imu_anchor_writer_->Open(
            imu_timeshare_path_, imu_anchor_writer_epoch_, &error))
    {
      DRIVER_ERROR(*cur_node_,
                   "[IMU_ANCHOR] cannot initialize %s: %s",
                   imu_timeshare_path_.c_str(), error.c_str());
      return false;
    }
    last_imu_anchor_ready_ = false;
    ImuTimeAnchorSnapshot snapshot;
    imu_anchor_writer_->ReadSnapshot(&snapshot);
    DRIVER_INFO(*cur_node_,
                "[IMU_ANCHOR] ready=false epoch=%" PRIu64
                " sequence=%" PRIu64,
                imu_anchor_writer_epoch_, snapshot.write_sequence);
    return true;
#endif
  }

  void Lddc::ShutdownImuAnchorWriter()
  {
    if (!imu_anchor_writer_)
    {
      return;
    }
    if (imu_anchor_writer_->is_open())
    {
#ifdef BUILDING_ROS1
      DRIVER_INFO(*cur_node_,
                  "[IMU_ANCHOR] ready=false epoch=%" PRIu64 " shutdown=true",
                  imu_anchor_writer_->writer_epoch());
#endif
      imu_anchor_writer_->Close();
    }
    imu_anchor_writer_.reset();
    last_imu_anchor_ready_ = false;
  }

  void Lddc::UpdateImuAnchor(uint8_t index, const ImuData& imu_data)
  {
#ifdef BUILDING_ROS1
    if (!imu_anchor_enable_ || index != imu_anchor_lidar_index_)
    {
      return;
    }
    const uint64_t update_ns = MonotonicNowNs();
    if (!imu_anchor_writer_ || !imu_anchor_writer_->is_open())
    {
      const uint64_t retry_ns = static_cast<uint64_t>(
          shared_open_retry_sec_ * kNsPerSecond);
      if (last_imu_anchor_open_attempt_ns_ != 0 &&
          update_ns - last_imu_anchor_open_attempt_ns_ < retry_ns)
      {
        return;
      }
      if (!InitializeImuAnchorWriter())
      {
        return;
      }
    }

    const bool was_ready = imu_anchor_writer_->ready();
    imu_anchor_writer_->Observe(
        imu_data.time_stamp, imu_data.host_monotonic_ns,
        ImuAnchorClockSourceFromTimestampType(imu_data.timestamp_type),
        update_ns);
    const bool is_ready = imu_anchor_writer_->ready();
    if (was_ready != is_ready || last_imu_anchor_ready_ != is_ready)
    {
      ImuTimeAnchorSnapshot snapshot;
      imu_anchor_writer_->ReadSnapshot(&snapshot);
      DRIVER_INFO(*cur_node_,
                  "[IMU_ANCHOR] ready=%s epoch=%" PRIu64
                  " sequence=%" PRIu64 " samples=%u",
                  is_ready ? "true" : "false",
                  imu_anchor_writer_->writer_epoch(),
                  snapshot.write_sequence,
                  imu_anchor_writer_->consecutive_samples());
      last_imu_anchor_ready_ = is_ready;
    }
    LogImuAnchorSummary(update_ns);
#else
    (void)index;
    (void)imu_data;
#endif
  }

  void Lddc::LogImuAnchorSummary(uint64_t now_ns)
  {
#ifdef BUILDING_ROS1
    if (!imu_anchor_writer_ || !imu_anchor_writer_->is_open())
    {
      return;
    }
    const uint64_t period_ns = static_cast<uint64_t>(
        imu_anchor_log_period_sec_ * kNsPerSecond);
    if (last_imu_anchor_log_ns_ != 0 &&
        now_ns - last_imu_anchor_log_ns_ < period_ns)
    {
      return;
    }
    const ImuTimeAnchorWriterStats& stats = imu_anchor_writer_->stats();
    ImuTimeAnchorSnapshot snapshot;
    imu_anchor_writer_->ReadSnapshot(&snapshot);
    double rate_hz = 0.0;
    if (last_imu_anchor_log_ns_ != 0 && now_ns > last_imu_anchor_log_ns_)
    {
      rate_hz = static_cast<double>(
          stats.accepted - last_imu_anchor_log_accepted_) *
          static_cast<double>(kNsPerSecond) /
          static_cast<double>(now_ns - last_imu_anchor_log_ns_);
    }
    last_imu_anchor_log_ns_ = now_ns;
    last_imu_anchor_log_accepted_ = stats.accepted;
    DRIVER_INFO(*cur_node_,
                "[IMU_ANCHOR] ready=%s epoch=%" PRIu64
                " sequence=%" PRIu64 " imu_stamp_ns=%" PRIu64
                " anchor_rate_hz=%.2f accepted=%" PRIu64
                " invalid_stamp=%" PRIu64 " backward=%" PRIu64
                " duplicate=%" PRIu64 " jump=%" PRIu64
                " clock_change=%" PRIu64,
                imu_anchor_writer_->ready() ? "true" : "false",
                imu_anchor_writer_->writer_epoch(),
                snapshot.write_sequence, snapshot.imu_stamp_ns, rate_hz,
                stats.accepted, stats.invalid_stamp, stats.backward,
                stats.duplicate, stats.jump, stats.clock_source_change);
#else
    (void)now_ns;
#endif
  }

  bool Lddc::InitializeSharedTimestampState()
  {
#ifndef BUILDING_ROS1
    return false;
#else
    if (shared_state_ != nullptr)
    {
      return true;
    }
    last_shared_open_attempt_ns_ = MonotonicNowNs();
    if (timeshare_path_.empty())
    {
      DRIVER_ERROR(*cur_node_, "Shared timestamp path is empty.");
      return false;
    }

    const int fd = open(timeshare_path_.c_str(),
                        O_CREAT | O_RDWR, 0666);
    if (fd < 0)
    {
      DRIVER_ERROR(*cur_node_, "Cannot open shared timestamp file %s: %s",
                   timeshare_path_.c_str(), strerror(errno));
      return false;
    }
    if (ftruncate(fd, sizeof(SharedTimestampState)) != 0)
    {
      DRIVER_ERROR(*cur_node_, "Cannot resize shared timestamp file %s: %s",
                   timeshare_path_.c_str(), strerror(errno));
      close(fd);
      return false;
    }

    void* mapping = mmap(nullptr, sizeof(SharedTimestampState),
                         PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    const int mmap_errno = errno;
    if (close(fd) != 0)
    {
      DRIVER_WARN(*cur_node_, "Closing shared timestamp fd failed: %s",
                  strerror(errno));
    }
    if (mapping == MAP_FAILED)
    {
      DRIVER_ERROR(*cur_node_, "Cannot mmap shared timestamp file %s: %s",
                   timeshare_path_.c_str(), strerror(mmap_errno));
      return false;
    }

    shared_state_ = static_cast<SharedTimestampState*>(mapping);
    writer_epoch_ = MakeWriterEpoch();
    if (shared_ready_gate_)
    {
      shared_ready_gate_->Reset();
    }
    WriteSharedTimestampState(shared_state_, writer_epoch_, 0,
                              kClockSourceUnknown, false,
                              MonotonicNowNs());
    DRIVER_INFO(*cur_node_, "Shared timestamp writer initialized: epoch=%" PRIu64,
                writer_epoch_);
    return true;
#endif
  }

  void Lddc::ShutdownSharedTimestampState()
  {
    if (shared_state_ == nullptr)
    {
      return;
    }
    WriteSharedTimestampState(shared_state_, writer_epoch_, 0,
                              kClockSourceUnknown, false,
                              MonotonicNowNs());
    if (munmap(shared_state_, sizeof(SharedTimestampState)) != 0)
    {
#ifdef BUILDING_ROS1
      if (cur_node_)
      {
        DRIVER_WARN(*cur_node_, "munmap shared timestamp failed: %s",
                    strerror(errno));
      }
#endif
    }
    shared_state_ = nullptr;
  }

  void Lddc::UpdateSharedTimestamp(uint8_t index, uint64_t stamp_ns,
                                   uint8_t timestamp_type)
  {
#ifdef BUILDING_ROS1
    if (index != shared_lidar_index_ || stamp_ns == 0)
    {
      return;
    }
    const uint64_t now_ns = MonotonicNowNs();
    if (shared_state_ == nullptr)
    {
      const uint64_t retry_ns =
          static_cast<uint64_t>(shared_open_retry_sec_ * kNsPerSecond);
      if (last_shared_open_attempt_ns_ != 0 &&
          now_ns - last_shared_open_attempt_ns_ < retry_ns)
      {
        return;
      }
      if (!InitializeSharedTimestampState())
      {
        return;
      }
    }

    const uint32_t clock_source =
        SharedClockSourceFromTimestampType(timestamp_type);
    const bool ready = shared_ready_gate_ &&
        shared_ready_gate_->Observe(stamp_ns, clock_source, now_ns);
    WriteSharedTimestampState(shared_state_, writer_epoch_, stamp_ns,
                              clock_source, ready, now_ns);
#else
    (void)index;
    (void)stamp_ns;
    (void)timestamp_type;
#endif
  }

  uint32_t Lddc::DropStartupBacklog(LidarDataQueue* queue, uint8_t index)
  {
    if (startup_queue_initialized_[index])
    {
      return 0;
    }
    startup_queue_initialized_[index] = true;
    uint32_t dropped = 0;
    while (QueueUsedSize(queue) > 1)
    {
      StoragePacket discarded;
      if (!QueuePop(queue, &discarded))
      {
        break;
      }
      ++dropped;
    }
    startup_drain_count_[index] += dropped;
#ifdef BUILDING_ROS1
    if (dropped != 0)
    {
      DRIVER_WARN(*cur_node_,
                  "Dropped %u startup pointcloud packets for lidar index %u "
                  "instead of replaying stale timestamps.",
                  dropped, index);
    }
#endif
    return dropped;
  }

  void Lddc::TrackClockSource(uint8_t index, uint8_t timestamp_type,
                              bool imu_stream)
  {
#ifdef BUILDING_ROS1
    auto& previous = imu_stream ? last_imu_time_type_ : last_lidar_time_type_;
    auto& switches =
        imu_stream ? imu_clock_switch_count_ : lidar_clock_switch_count_;
    if (previous[index] != 0xff && previous[index] != timestamp_type)
    {
      ++switches[index];
      DRIVER_WARN(*cur_node_,
                  "Livox %s clock source changed for index %u: %s -> %s "
                  "(switches=%" PRIu64 ")",
                  imu_stream ? "IMU" : "LiDAR", index,
                  TimestampTypeName(previous[index]),
                  TimestampTypeName(timestamp_type), switches[index]);
      if (!imu_stream && index == shared_lidar_index_ &&
          shared_ready_gate_)
      {
        shared_ready_gate_->Reset();
      }
    }
    previous[index] = timestamp_type;
    if (timestamp_type == kTimestampTypeNoSync)
    {
      ROS_WARN_THROTTLE(5.0,
                        "Livox %s index %u is NoSync; timestamps use host "
                        "system clock.",
                        imu_stream ? "IMU" : "LiDAR", index);
    }
#else
    (void)index;
    (void)timestamp_type;
    (void)imu_stream;
#endif
  }

  void Lddc::PublishTimestampDiagnostic(uint8_t index,
                                        uint8_t timestamp_type,
                                        uint64_t stamp_ns,
                                        uint32_t queue_depth,
                                        bool imu_stream)
  {
#ifdef BUILDING_ROS1
    if (!timestamp_diagnostic_pub_)
    {
      return;
    }
    const uint64_t now_ns = MonotonicNowNs();
    auto& last_diag = imu_stream ? last_imu_diag_ns_ : last_lidar_diag_ns_;
    const uint64_t period_ns =
        static_cast<uint64_t>(diagnostic_period_sec_ * kNsPerSecond);
    if (last_diag[index] != 0 && now_ns - last_diag[index] < period_ns)
    {
      return;
    }
    last_diag[index] = now_ns;

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.level = timestamp_type == kTimestampTypeNoSync
                       ? diagnostic_msgs::DiagnosticStatus::WARN
                       : diagnostic_msgs::DiagnosticStatus::OK;
    status.name = std::string("livox_ros_driver2/") +
                  (imu_stream ? "imu_timestamp" : "lidar_timestamp");
    status.hardware_id =
        index < lds_->lidar_count_
            ? IpNumToString(lds_->lidars_[index].handle)
            : std::to_string(index);
    status.message = timestamp_type == kTimestampTypeNoSync
                         ? "NoSync: host system clock fallback"
                         : "Device timestamp synchronized";
    status.values.push_back(
        DiagnosticValue("device_index", std::to_string(index)));
    status.values.push_back(
        DiagnosticValue("stream", imu_stream ? "imu" : "lidar"));
    status.values.push_back(DiagnosticValue(
        "time_type", std::to_string(timestamp_type)));
    status.values.push_back(DiagnosticValue(
        "timestamp_synced",
        timestamp_type == kTimestampTypeNoSync ? "false" : "true"));
    status.values.push_back(
        DiagnosticValue("clock_source", TimestampTypeName(timestamp_type)));
    status.values.push_back(
        DiagnosticValue("base_time", std::to_string(stamp_ns)));
    status.values.push_back(
        DiagnosticValue("queue_depth", std::to_string(queue_depth)));
    status.values.push_back(DiagnosticValue(
        "startup_drain_count",
        std::to_string(startup_drain_count_[index])));
    const auto& switches =
        imu_stream ? imu_clock_switch_count_ : lidar_clock_switch_count_;
    status.values.push_back(DiagnosticValue(
        "clock_source_switch_count", std::to_string(switches[index])));

    if (!imu_stream && index == shared_lidar_index_)
    {
      SharedTimestampSnapshot snapshot;
      const SharedTimestampReadResult result =
          ReadSharedTimestampState(shared_state_, &snapshot);
      status.values.push_back(
          DiagnosticValue("shared_semantics", "LIDAR_BASE_TIME_LEGACY"));
      status.values.push_back(DiagnosticValue(
          "shared_ready",
          result == kSharedTimestampReadOk ? "true" : "false"));
      status.values.push_back(DiagnosticValue(
          "writer_epoch", std::to_string(writer_epoch_)));
      status.values.push_back(DiagnosticValue(
          "write_sequence", std::to_string(snapshot.write_sequence)));
    }
    array.status.push_back(std::move(status));
    timestamp_diagnostic_pub_.publish(array);
#else
    (void)index;
    (void)timestamp_type;
    (void)stamp_ns;
    (void)queue_depth;
    (void)imu_stream;
#endif
  }

  int Lddc::RegisterLds(Lds *lds)
  {
    if (lds_ == nullptr)
    {
      lds_ = lds;
      return 0;
    }
    else
    {
      return -1;
    }
  }

  void Lddc::DistributePointCloudData(void)
  {
    if (!lds_)
    {
      std::cout << "lds is not registered" << std::endl;
      return;
    }
    if (lds_->IsRequestExit())
    {
      std::cout << "DistributePointCloudData is RequestExit" << std::endl;
      return;
    }

    lds_->pcd_semaphore_.Wait();
    for (uint32_t i = 0; i < lds_->lidar_count_; i++)
    {
      uint32_t lidar_id = i;
      LidarDevice *lidar = &lds_->lidars_[lidar_id];
      LidarDataQueue *p_queue = &lidar->data;
      if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr))
      {
        continue;
      }
      PollingLidarPointCloudData(lidar_id, lidar);
    }
  }

  void Lddc::DistributeImuData(void)
  {
    if (!lds_)
    {
      std::cout << "lds is not registered" << std::endl;
      return;
    }
    if (lds_->IsRequestExit())
    {
      std::cout << "DistributeImuData is RequestExit" << std::endl;
      return;
    }

    lds_->imu_semaphore_.Wait();
    for (uint32_t i = 0; i < lds_->lidar_count_; i++)
    {
      uint32_t lidar_id = i;
      LidarDevice *lidar = &lds_->lidars_[lidar_id];
      LidarImuDataQueue *p_queue = &lidar->imu_data;
      if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr))
      {
        continue;
      }
      PollingLidarImuData(lidar_id, lidar);
    }
  }
  void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar)
  {
    LidarDataQueue *p_queue = &lidar->data;
    if (p_queue == nullptr || p_queue->storage_packet == nullptr)
    {
      return;
    }

    DropStartupBacklog(p_queue, index);

    while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue))
    {
      if (kPointCloud2Msg == transfer_format_)
      {
        PublishPointcloud2(p_queue, index);
      }
      else if (kLivoxCustomMsg == transfer_format_)
      {
        PublishCustomPointcloud(p_queue, index);
      }
      else if (kPclPxyziMsg == transfer_format_)
      {
        PublishPclMsg(p_queue, index);
      }
    }
  }

  void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar)
  {
    LidarImuDataQueue &p_queue = lidar->imu_data;
    while (!lds_->IsRequestExit() && !p_queue.Empty())
    {
      PublishImuData(p_queue, index);
    }
  }

  void Lddc::PrepareExit(void)
  {
#ifdef BUILDING_ROS1
    if (bag_)
    {
      DRIVER_INFO(*cur_node_, "Waiting to save the bag file!");
      bag_->close();
      DRIVER_INFO(*cur_node_, "Save the bag file successfully!");
      bag_ = nullptr;
    }
#endif
    if (lds_)
    {
      lds_->PrepareExit();
      lds_ = nullptr;
    }
  }

  void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index)
  {
    while (!QueueIsEmpty(queue))
    {
      StoragePacket pkg;
      QueuePop(queue, &pkg);
      if (pkg.points.empty())
      {
        printf("Publish point cloud2 failed, the pkg points is empty.\n");
        continue;
      }

      PointCloud2 cloud;
      uint64_t timestamp = 0;
      InitPointcloud2Msg(pkg, cloud, timestamp);
      TrackClockSource(index, pkg.timestamp_type, false);
      PublishPointcloud2Data(index, timestamp, cloud);
      UpdateSharedTimestamp(index, timestamp, pkg.timestamp_type);
      PublishTimestampDiagnostic(index, pkg.timestamp_type, timestamp,
                                 QueueUsedSize(queue), false);
    }
  }

  void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index)
  {
    while (!QueueIsEmpty(queue))
    {
      StoragePacket pkg;
      QueuePop(queue, &pkg);
      if (pkg.points.empty())
      {
        printf("Publish custom point cloud failed, the pkg points is empty.\n");
        continue;
      }

      CustomMsg livox_msg;
      uint64_t timestamp = 0;
      InitCustomMsg(livox_msg, pkg, index);
      FillPointsToCustomMsg(livox_msg, pkg);
      
      if (!pkg.points.empty())
      {
        timestamp = pkg.base_time;
      }
      TrackClockSource(index, pkg.timestamp_type, false);
      PublishCustomPointData(livox_msg, index);
      UpdateSharedTimestamp(index, timestamp, pkg.timestamp_type);
      PublishTimestampDiagnostic(index, pkg.timestamp_type, timestamp,
                                 QueueUsedSize(queue), false);
    }
  }

  /* for pcl::pxyzi */
  void Lddc::PublishPclMsg(LidarDataQueue *queue, uint8_t index)
  {
#ifdef BUILDING_ROS2
    static bool first_log = true;
    if (first_log)
    {
      std::cout << "error: message type 'pcl::PointCloud' is NOT supported in ROS2, "
                << "please modify the 'xfer_format' field in the launch file"
                << std::endl;
    }
    first_log = false;
    return;
#endif
    while (!QueueIsEmpty(queue))
    {
      StoragePacket pkg;
      QueuePop(queue, &pkg);
      if (pkg.points.empty())
      {
        printf("Publish point cloud failed, the pkg points is empty.\n");
        continue;
      }

      PointCloud cloud;
      uint64_t timestamp = 0;
      InitPclMsg(pkg, cloud, timestamp);
      FillPointsToPclMsg(pkg, cloud);
      TrackClockSource(index, pkg.timestamp_type, false);
      PublishPclData(index, timestamp, cloud);
      UpdateSharedTimestamp(index, timestamp, pkg.timestamp_type);
      PublishTimestampDiagnostic(index, pkg.timestamp_type, timestamp,
                                 QueueUsedSize(queue), false);
    }
    return;
  }

  void Lddc::InitPointcloud2MsgHeader(PointCloud2 &cloud)
  {
    cloud.header.frame_id.assign(frame_id_);
    cloud.height = 1;
    cloud.width = 0;
    cloud.fields.resize(7);
    cloud.fields[0].offset = 0;
    cloud.fields[0].name = "x";
    cloud.fields[0].count = 1;
    cloud.fields[0].datatype = PointField::FLOAT32;
    cloud.fields[1].offset = 4;
    cloud.fields[1].name = "y";
    cloud.fields[1].count = 1;
    cloud.fields[1].datatype = PointField::FLOAT32;
    cloud.fields[2].offset = 8;
    cloud.fields[2].name = "z";
    cloud.fields[2].count = 1;
    cloud.fields[2].datatype = PointField::FLOAT32;
    cloud.fields[3].offset = 12;
    cloud.fields[3].name = "intensity";
    cloud.fields[3].count = 1;
    cloud.fields[3].datatype = PointField::FLOAT32;
    cloud.fields[4].offset = 16;
    cloud.fields[4].name = "tag";
    cloud.fields[4].count = 1;
    cloud.fields[4].datatype = PointField::UINT8;
    cloud.fields[5].offset = 17;
    cloud.fields[5].name = "line";
    cloud.fields[5].count = 1;
    cloud.fields[5].datatype = PointField::UINT8;
    cloud.fields[6].offset = 18;
    cloud.fields[6].name = "timestamp";
    cloud.fields[6].count = 1;
    cloud.fields[6].datatype = PointField::FLOAT64;
    cloud.point_step = sizeof(LivoxPointXyzrtlt);
  }

  void Lddc::InitPointcloud2Msg(const StoragePacket &pkg, PointCloud2 &cloud, uint64_t &timestamp)
  {
    InitPointcloud2MsgHeader(cloud);

    cloud.point_step = sizeof(LivoxPointXyzrtlt);

    cloud.width = pkg.points_num;
    cloud.row_step = cloud.width * cloud.point_step;

    cloud.is_bigendian = false;
    cloud.is_dense = true;

    if (!pkg.points.empty())
    {
      timestamp = pkg.base_time;
    }

#ifdef BUILDING_ROS1
    cloud.header.stamp = RosTimeFromNs(timestamp);
#elif defined BUILDING_ROS2
    cloud.header.stamp = rclcpp::Time(timestamp);
#endif

    std::vector<LivoxPointXyzrtlt> points;
    for (size_t i = 0; i < pkg.points_num; ++i)
    {
      LivoxPointXyzrtlt point;
      point.x = pkg.points[i].x;
      point.y = pkg.points[i].y;
      point.z = pkg.points[i].z;
      point.reflectivity = pkg.points[i].intensity;
      point.tag = pkg.points[i].tag;
      point.line = pkg.points[i].line;
      point.timestamp = static_cast<double>(pkg.points[i].offset_time);
      points.push_back(std::move(point));
    }
    cloud.data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt));
    memcpy(cloud.data.data(), points.data(), pkg.points_num * sizeof(LivoxPointXyzrtlt));
  }

  void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp, const PointCloud2 &cloud)
  {
#ifdef BUILDING_ROS1
    PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
#elif defined BUILDING_ROS2
    Publisher<PointCloud2>::SharedPtr publisher_ptr =
        std::dynamic_pointer_cast<Publisher<PointCloud2>>(GetCurrentPublisher(index));
#endif

    if (kOutputToRos == output_type_)
    {
      publisher_ptr->publish(cloud);
    }
    else
    {
#ifdef BUILDING_ROS1
      if (bag_ && enable_lidar_bag_)
      {
        bag_->write(publisher_ptr->getTopic(), RosTimeFromNs(timestamp), cloud);
      }
#endif
    }
  }

  void Lddc::InitCustomMsg(CustomMsg &livox_msg, const StoragePacket &pkg, uint8_t index)
  {
    livox_msg.header.frame_id.assign(frame_id_);

#ifdef BUILDING_ROS1
    static uint32_t msg_seq = 0;
    livox_msg.header.seq = msg_seq;
    ++msg_seq;
#endif

    uint64_t timestamp = 0;
    if (!pkg.points.empty())
    {
      timestamp = pkg.base_time;
    }
    livox_msg.timebase = timestamp;

#ifdef BUILDING_ROS1
    livox_msg.header.stamp = RosTimeFromNs(timestamp);
#elif defined BUILDING_ROS2
    livox_msg.header.stamp = rclcpp::Time(timestamp);
#endif

    livox_msg.point_num = pkg.points_num;
    if (lds_->lidars_[index].lidar_type == kLivoxLidarType)
    {
      livox_msg.lidar_id = lds_->lidars_[index].handle;
    }
    else
    {
      printf("Init custom msg lidar id failed, the index:%u.\n", index);
      livox_msg.lidar_id = 0;
    }
  }

  void Lddc::FillPointsToCustomMsg(CustomMsg &livox_msg, const StoragePacket &pkg)
  {
    uint32_t points_num = pkg.points_num;
    const std::vector<PointXyzlt> &points = pkg.points;
    for (uint32_t i = 0; i < points_num; ++i)
    {
      CustomPoint point;
      point.x = points[i].x;
      point.y = points[i].y;
      point.z = points[i].z;
      point.reflectivity = points[i].intensity;
      point.tag = points[i].tag;
      point.line = points[i].line;
      point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

      livox_msg.points.push_back(std::move(point));
    }
  }

  void Lddc::PublishCustomPointData(const CustomMsg &livox_msg, const uint8_t index)
  {
#ifdef BUILDING_ROS1
    PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
#elif defined BUILDING_ROS2
    Publisher<CustomMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<CustomMsg>>(GetCurrentPublisher(index));
#endif

    if (kOutputToRos == output_type_)
    {
      publisher_ptr->publish(livox_msg);
    }
    else
    {
#ifdef BUILDING_ROS1
      if (bag_ && enable_lidar_bag_)
      {
        bag_->write(publisher_ptr->getTopic(),
                    RosTimeFromNs(livox_msg.timebase), livox_msg);
      }
#endif
    }
  }

  void Lddc::InitPclMsg(const StoragePacket &pkg, PointCloud &cloud, uint64_t &timestamp)
  {
#ifdef BUILDING_ROS1
    cloud.header.frame_id.assign(frame_id_);
    cloud.height = 1;
    cloud.width = pkg.points_num;

    if (!pkg.points.empty())
    {
      timestamp = pkg.base_time;
    }
    cloud.header.stamp = timestamp / 1000U; // PCL header uses microseconds.
#elif defined BUILDING_ROS2
    std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
              << "please check code logic"
              << std::endl;
#endif
    return;
  }

  void Lddc::FillPointsToPclMsg(const StoragePacket &pkg, PointCloud &pcl_msg)
  {
#ifdef BUILDING_ROS1
    if (pkg.points.empty())
    {
      return;
    }

    uint32_t points_num = pkg.points_num;
    const std::vector<PointXyzlt> &points = pkg.points;
    for (uint32_t i = 0; i < points_num; ++i)
    {
      pcl::PointXYZI point;
      point.x = points[i].x;
      point.y = points[i].y;
      point.z = points[i].z;
      point.intensity = points[i].intensity;

      pcl_msg.points.push_back(std::move(point));
    }
#elif defined BUILDING_ROS2
    std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
              << "please check code logic"
              << std::endl;
#endif
    return;
  }

  void Lddc::PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud &cloud)
  {
#ifdef BUILDING_ROS1
    PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
    if (kOutputToRos == output_type_)
    {
      publisher_ptr->publish(cloud);
    }
    else
    {
      if (bag_ && enable_lidar_bag_)
      {
        bag_->write(publisher_ptr->getTopic(), RosTimeFromNs(timestamp), cloud);
      }
    }
#elif defined BUILDING_ROS2
    std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
              << "please check code logic"
              << std::endl;
#endif
    return;
  }

  void Lddc::InitImuMsg(const ImuData &imu_data, ImuMsg &imu_msg, uint64_t &timestamp)
  {
    imu_msg.header.frame_id = "livox_frame";

    timestamp = imu_data.time_stamp;
#ifdef BUILDING_ROS1
    imu_msg.header.stamp = RosTimeFromNs(timestamp);
#elif defined BUILDING_ROS2
    imu_msg.header.stamp = rclcpp::Time(timestamp); // to ros time stamp
#endif

    imu_msg.angular_velocity.x = imu_data.gyro_x;
    imu_msg.angular_velocity.y = imu_data.gyro_y;
    imu_msg.angular_velocity.z = imu_data.gyro_z;
    imu_msg.linear_acceleration.x = imu_data.acc_x;
    imu_msg.linear_acceleration.y = imu_data.acc_y;
    imu_msg.linear_acceleration.z = imu_data.acc_z;
  }

  void Lddc::PublishImuData(LidarImuDataQueue &imu_data_queue, const uint8_t index)
  {
    ImuData imu_data;
    if (!imu_data_queue.Pop(imu_data))
    {
      // printf("Publish imu data failed, imu data queue pop failed.\n");
      return;
    }

    ImuMsg imu_msg;
    uint64_t timestamp;
    InitImuMsg(imu_data, imu_msg, timestamp);
    TrackClockSource(index, imu_data.timestamp_type, true);
    UpdateImuAnchor(index, imu_data);

#ifdef BUILDING_ROS1
    PublisherPtr publisher_ptr = GetCurrentImuPublisher(index);
#elif defined BUILDING_ROS2
    Publisher<ImuMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<ImuMsg>>(GetCurrentImuPublisher(index));
#endif

    if (kOutputToRos == output_type_)
    {
      publisher_ptr->publish(imu_msg);
    }
    else
    {
#ifdef BUILDING_ROS1
      if (bag_ && enable_imu_bag_)
      {
        bag_->write(publisher_ptr->getTopic(), RosTimeFromNs(timestamp), imu_msg);
      }
#endif
    }
    PublishTimestampDiagnostic(index, imu_data.timestamp_type, timestamp,
                               static_cast<uint32_t>(imu_data_queue.Size()),
                               true);
  }

#ifdef BUILDING_ROS2
  std::shared_ptr<rclcpp::PublisherBase> Lddc::CreatePublisher(uint8_t msg_type,
                                                               std::string &topic_name, uint32_t queue_size)
  {
    if (kPointCloud2Msg == msg_type)
    {
      DRIVER_INFO(*cur_node_,
                  "%s publish use PointCloud2 format", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud2>(topic_name, queue_size);
    }
    else if (kLivoxCustomMsg == msg_type)
    {
      DRIVER_INFO(*cur_node_,
                  "%s publish use livox custom format", topic_name.c_str());
      return cur_node_->create_publisher<CustomMsg>(topic_name, queue_size);
    }
#if 0
    else if (kPclPxyziMsg == msg_type)  {
      DRIVER_INFO(*cur_node_,
          "%s publish use pcl PointXYZI format", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud>(topic_name, queue_size);
    }
#endif
    else if (kLivoxImuMsg == msg_type)
    {
      DRIVER_INFO(*cur_node_,
                  "%s publish use imu format", topic_name.c_str());
      return cur_node_->create_publisher<ImuMsg>(topic_name,
                                                 queue_size);
    }
    else
    {
      PublisherPtr null_publisher(nullptr);
      return null_publisher;
    }
  }
#endif

#ifdef BUILDING_ROS1
  PublisherPtr Lddc::GetCurrentPublisher(uint8_t index)
  {
    ros::Publisher **pub = nullptr;
    uint32_t queue_size = kMinEthPacketQueueSize;

    if (use_multi_topic_)
    {
      pub = &private_pub_[index];
      queue_size = queue_size / 8; // queue size is 4 for only one lidar
    }
    else
    {
      pub = &global_pub_;
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
    }

    if (*pub == nullptr)
    {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      if (use_multi_topic_)
      {
        std::string ip_string = IpNumToString(lds_->lidars_[index].handle);
        snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
                 ReplacePeriodByUnderline(ip_string).c_str());
        DRIVER_INFO(*cur_node_, "Support multi topics.");
      }
      else
      {
        DRIVER_INFO(*cur_node_, "Support only one topic.");
        snprintf(name_str, sizeof(name_str), "livox/lidar");
      }

      *pub = new ros::Publisher;
      if (kPointCloud2Msg == transfer_format_)
      {
        **pub =
            cur_node_->GetNode().advertise<sensor_msgs::PointCloud2>(name_str, queue_size);
        DRIVER_INFO(*cur_node_,
                    "%s publish use PointCloud2 format, set ROS publisher queue size %d",
                    name_str, queue_size);
      }
      else if (kLivoxCustomMsg == transfer_format_)
      {
        **pub = cur_node_->GetNode().advertise<livox_ros_driver2::CustomMsg>(name_str,
                                                                             queue_size);
        DRIVER_INFO(*cur_node_,
                    "%s publish use livox custom format, set ROS publisher queue size %d",
                    name_str, queue_size);
      }
      else if (kPclPxyziMsg == transfer_format_)
      {
        **pub = cur_node_->GetNode().advertise<PointCloud>(name_str, queue_size);
        DRIVER_INFO(*cur_node_,
                    "%s publish use pcl PointXYZI format, set ROS publisher queue "
                    "size %d",
                    name_str, queue_size);
      }
    }

    return *pub;
  }

  PublisherPtr Lddc::GetCurrentImuPublisher(uint8_t handle)
  {
    ros::Publisher **pub = nullptr;
    uint32_t queue_size = kMinEthPacketQueueSize;

    if (use_multi_topic_)
    {
      pub = &private_imu_pub_[handle];
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
    }
    else
    {
      pub = &global_imu_pub_;
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
    }

    if (*pub == nullptr)
    {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      if (use_multi_topic_)
      {
        DRIVER_INFO(*cur_node_, "Support multi topics.");
        std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
        snprintf(name_str, sizeof(name_str), "livox/imu_%s",
                 ReplacePeriodByUnderline(ip_string).c_str());
      }
      else
      {
        DRIVER_INFO(*cur_node_, "Support only one topic.");
        snprintf(name_str, sizeof(name_str), "livox/imu");
      }

      *pub = new ros::Publisher;
      **pub = cur_node_->GetNode().advertise<sensor_msgs::Imu>(name_str, queue_size);
      DRIVER_INFO(*cur_node_, "%s publish imu data, set ROS publisher queue size %d", name_str,
                  queue_size);
    }

    return *pub;
  }
#elif defined BUILDING_ROS2
  std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentPublisher(uint8_t handle)
  {
    uint32_t queue_size = kMinEthPacketQueueSize;
    if (use_multi_topic_)
    {
      if (!private_pub_[handle])
      {
        char name_str[48];
        memset(name_str, 0, sizeof(name_str));

        std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
        snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
                 ReplacePeriodByUnderline(ip_string).c_str());
        std::string topic_name(name_str);
        queue_size = queue_size * 2; // queue size is 64 for only one lidar
        private_pub_[handle] = CreatePublisher(transfer_format_, topic_name, queue_size);
      }
      return private_pub_[handle];
    }
    else
    {
      if (!global_pub_)
      {
        std::string topic_name("livox/lidar");
        queue_size = queue_size * 8; // shared queue size is 256, for all lidars
        global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
      }
      return global_pub_;
    }
  }

  std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentImuPublisher(uint8_t handle)
  {
    uint32_t queue_size = kMinEthPacketQueueSize;
    if (use_multi_topic_)
    {
      if (!private_imu_pub_[handle])
      {
        char name_str[48];
        memset(name_str, 0, sizeof(name_str));
        std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
        snprintf(name_str, sizeof(name_str), "livox/imu_%s",
                 ReplacePeriodByUnderline(ip_string).c_str());
        std::string topic_name(name_str);
        queue_size = queue_size * 2; // queue size is 64 for only one lidar
        private_imu_pub_[handle] = CreatePublisher(kLivoxImuMsg, topic_name,
                                                   queue_size);
      }
      return private_imu_pub_[handle];
    }
    else
    {
      if (!global_imu_pub_)
      {
        std::string topic_name("livox/imu");
        queue_size = queue_size * 8; // shared queue size is 256, for all lidars
        global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
      }
      return global_imu_pub_;
    }
  }
#endif

  void Lddc::CreateBagFile(const std::string &file_name)
  {
#ifdef BUILDING_ROS1
    if (!bag_)
    {
      bag_ = new rosbag::Bag;
      bag_->open(file_name, rosbag::bagmode::Write);
      DRIVER_INFO(*cur_node_, "Create bag file :%s!", file_name.c_str());
    }
#endif
  }

} // namespace livox_ros
