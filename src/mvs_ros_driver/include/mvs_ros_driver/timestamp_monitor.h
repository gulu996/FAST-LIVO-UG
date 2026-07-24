#ifndef MVS_ROS_DRIVER_TIMESTAMP_MONITOR_H_
#define MVS_ROS_DRIVER_TIMESTAMP_MONITOR_H_

#include <stdint.h>

#include <livox_ros_driver2/shared_timestamp_state.h>

namespace mvs_ros_driver {

enum class StampValidationResult {
  kAccepted,
  kWriterEpochChanged,
  kStale,
  kDuplicate,
  kNonMonotonic,
  kLargeStep,
};

class TimestampMonitor {
 public:
  TimestampMonitor(uint64_t stale_timeout_ns, uint64_t max_step_ns)
      : stale_timeout_ns_(stale_timeout_ns), max_step_ns_(max_step_ns) {}

  StampValidationResult Validate(
      const livox_ros::SharedTimestampSnapshot& snapshot,
      uint64_t now_monotonic_ns) {
    if (writer_epoch_ != 0 && writer_epoch_ != snapshot.writer_epoch) {
      writer_epoch_ = snapshot.writer_epoch;
      last_stamp_ns_ = 0;
      ++writer_restart_count_;
      ++invalid_shared_stamp_count_;
      return StampValidationResult::kWriterEpochChanged;
    }
    writer_epoch_ = snapshot.writer_epoch;

    if (snapshot.last_update_monotonic_ns == 0 ||
        now_monotonic_ns < snapshot.last_update_monotonic_ns ||
        now_monotonic_ns - snapshot.last_update_monotonic_ns >
            stale_timeout_ns_) {
      ++stale_stamp_count_;
      ++invalid_shared_stamp_count_;
      return StampValidationResult::kStale;
    }
    if (last_stamp_ns_ != 0 && snapshot.stamp_ns == last_stamp_ns_) {
      ++duplicate_stamp_count_;
      ++invalid_shared_stamp_count_;
      return StampValidationResult::kDuplicate;
    }
    if (last_stamp_ns_ != 0 && snapshot.stamp_ns < last_stamp_ns_) {
      ++non_monotonic_stamp_count_;
      ++invalid_shared_stamp_count_;
      return StampValidationResult::kNonMonotonic;
    }
    if (last_stamp_ns_ != 0 &&
        snapshot.stamp_ns - last_stamp_ns_ > max_step_ns_) {
      // ponytail: one bad transition is dropped, then the next valid sample
      // establishes a new observed epoch; true clock jumps remain diagnosed.
      last_stamp_ns_ = 0;
      ++large_step_count_;
      ++invalid_shared_stamp_count_;
      return StampValidationResult::kLargeStep;
    }

    last_stamp_ns_ = snapshot.stamp_ns;
    return StampValidationResult::kAccepted;
  }

  void ObserveCameraMetadata(uint32_t frame_num, uint32_t frame_counter,
                             uint32_t trigger_index,
                             uint64_t device_timestamp) {
    if (has_metadata_) {
      frame_gap_count_ += ForwardGap(previous_frame_num_, frame_num);
      frame_counter_gap_count_ +=
          ForwardGap(previous_frame_counter_, frame_counter);
      trigger_gap_count_ +=
          ForwardGap(previous_trigger_index_, trigger_index);
      if (device_timestamp < previous_device_timestamp_) {
        ++device_time_wrap_count_;
      }
    }
    previous_frame_num_ = frame_num;
    previous_frame_counter_ = frame_counter;
    previous_trigger_index_ = trigger_index;
    previous_device_timestamp_ = device_timestamp;
    has_metadata_ = true;
  }

  void CountInvalidSharedStamp() { ++invalid_shared_stamp_count_; }

  uint64_t frame_gap_count() const { return frame_gap_count_; }
  uint64_t frame_counter_gap_count() const {
    return frame_counter_gap_count_;
  }
  uint64_t trigger_gap_count() const { return trigger_gap_count_; }
  uint64_t device_time_wrap_count() const {
    return device_time_wrap_count_;
  }
  uint64_t duplicate_stamp_count() const { return duplicate_stamp_count_; }
  uint64_t non_monotonic_stamp_count() const {
    return non_monotonic_stamp_count_;
  }
  uint64_t invalid_shared_stamp_count() const {
    return invalid_shared_stamp_count_;
  }
  uint64_t stale_stamp_count() const { return stale_stamp_count_; }
  uint64_t large_step_count() const { return large_step_count_; }
  uint64_t writer_restart_count() const { return writer_restart_count_; }
  uint64_t writer_epoch() const { return writer_epoch_; }

 private:
  static uint64_t ForwardGap(uint32_t previous, uint32_t current) {
    const uint32_t delta = current - previous;
    return delta > 1U && delta < 0x80000000U
               ? static_cast<uint64_t>(delta - 1U)
               : 0U;
  }

  uint64_t stale_timeout_ns_;
  uint64_t max_step_ns_;
  uint64_t writer_epoch_ = 0;
  uint64_t last_stamp_ns_ = 0;

  bool has_metadata_ = false;
  uint32_t previous_frame_num_ = 0;
  uint32_t previous_frame_counter_ = 0;
  uint32_t previous_trigger_index_ = 0;
  uint64_t previous_device_timestamp_ = 0;

  uint64_t frame_gap_count_ = 0;
  uint64_t frame_counter_gap_count_ = 0;
  uint64_t trigger_gap_count_ = 0;
  uint64_t device_time_wrap_count_ = 0;
  uint64_t duplicate_stamp_count_ = 0;
  uint64_t non_monotonic_stamp_count_ = 0;
  uint64_t invalid_shared_stamp_count_ = 0;
  uint64_t stale_stamp_count_ = 0;
  uint64_t large_step_count_ = 0;
  uint64_t writer_restart_count_ = 0;
};

}  // namespace mvs_ros_driver

#endif  // MVS_ROS_DRIVER_TIMESTAMP_MONITOR_H_
