#ifndef LIVOX_ROS_DRIVER2_SHARED_TIMESTAMP_STATE_H_
#define LIVOX_ROS_DRIVER2_SHARED_TIMESTAMP_STATE_H_

#include <stdint.h>
#include <time.h>

namespace livox_ros {

constexpr uint32_t kSharedTimestampMagic = 0x4c545331U;  // "LTS1"
constexpr uint32_t kSharedTimestampVersion = 1U;

enum SharedTimestampClockSource : uint32_t {
  kClockSourceUnknown = 0,
  kLidarBaseTimeLegacyNoSync = 1,
  kLidarBaseTimeLegacyPtp = 2,
  kLidarBaseTimeLegacyGps = 3,
};

// This remains a single latest-LiDAR-time slot. It is not a trigger queue and
// does not establish a physical camera-trigger association.
struct alignas(64) SharedTimestampState {
  uint32_t magic;
  uint32_t version;
  uint64_t writer_epoch;
  uint64_t write_sequence;
  uint64_t stamp_ns;
  uint32_t clock_source;
  uint32_t ready;
  uint64_t last_update_monotonic_ns;
  uint64_t reserved[2];
};

struct SharedTimestampSnapshot {
  uint32_t magic = 0;
  uint32_t version = 0;
  uint64_t writer_epoch = 0;
  uint64_t write_sequence = 0;
  uint64_t stamp_ns = 0;
  uint32_t clock_source = kClockSourceUnknown;
  bool ready = false;
  uint64_t last_update_monotonic_ns = 0;
};

enum SharedTimestampReadResult {
  kSharedTimestampReadOk = 0,
  kSharedTimestampReadInconsistent,
  kSharedTimestampReadBadProtocol,
  kSharedTimestampReadNotReady,
  kSharedTimestampReadEmpty,
};

inline uint64_t MonotonicNowNs() {
  struct timespec now = {};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    return 0;
  }
  return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(now.tv_nsec);
}

inline const char* SharedTimestampClockSourceName(uint32_t source) {
  switch (source) {
    case kLidarBaseTimeLegacyNoSync:
      return "LIDAR_BASE_TIME_LEGACY_NOSYNC_HOST";
    case kLidarBaseTimeLegacyPtp:
      return "LIDAR_BASE_TIME_LEGACY_PTP";
    case kLidarBaseTimeLegacyGps:
      return "LIDAR_BASE_TIME_LEGACY_GPS";
    default:
      return "UNKNOWN";
  }
}

inline void WriteSharedTimestampState(SharedTimestampState* state,
                                      uint64_t writer_epoch,
                                      uint64_t stamp_ns,
                                      uint32_t clock_source,
                                      bool ready,
                                      uint64_t update_monotonic_ns) {
  uint64_t sequence =
      __atomic_load_n(&state->write_sequence, __ATOMIC_RELAXED);
  sequence = (sequence & 1U) ? sequence + 2U : sequence + 1U;
  __atomic_store_n(&state->write_sequence, sequence, __ATOMIC_SEQ_CST);

  __atomic_store_n(&state->magic, kSharedTimestampMagic, __ATOMIC_RELAXED);
  __atomic_store_n(&state->version, kSharedTimestampVersion, __ATOMIC_RELAXED);
  __atomic_store_n(&state->writer_epoch, writer_epoch, __ATOMIC_RELAXED);
  __atomic_store_n(&state->stamp_ns, stamp_ns, __ATOMIC_RELAXED);
  __atomic_store_n(&state->clock_source, clock_source, __ATOMIC_RELAXED);
  __atomic_store_n(&state->ready, ready ? 1U : 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&state->last_update_monotonic_ns, update_monotonic_ns,
                   __ATOMIC_RELAXED);

  __atomic_store_n(&state->write_sequence, sequence + 1U, __ATOMIC_RELEASE);
}

inline SharedTimestampReadResult ReadSharedTimestampState(
    const SharedTimestampState* state, SharedTimestampSnapshot* snapshot,
    unsigned int max_attempts = 8) {
  if (state == nullptr || snapshot == nullptr) {
    return kSharedTimestampReadInconsistent;
  }

  for (unsigned int attempt = 0; attempt < max_attempts; ++attempt) {
    const uint64_t begin =
        __atomic_load_n(&state->write_sequence, __ATOMIC_ACQUIRE);
    if (begin & 1U) {
      continue;
    }

    SharedTimestampSnapshot candidate;
    candidate.magic = __atomic_load_n(&state->magic, __ATOMIC_RELAXED);
    candidate.version = __atomic_load_n(&state->version, __ATOMIC_RELAXED);
    candidate.writer_epoch =
        __atomic_load_n(&state->writer_epoch, __ATOMIC_RELAXED);
    candidate.stamp_ns =
        __atomic_load_n(&state->stamp_ns, __ATOMIC_RELAXED);
    candidate.clock_source =
        __atomic_load_n(&state->clock_source, __ATOMIC_RELAXED);
    candidate.ready =
        __atomic_load_n(&state->ready, __ATOMIC_RELAXED) != 0;
    candidate.last_update_monotonic_ns =
        __atomic_load_n(&state->last_update_monotonic_ns, __ATOMIC_RELAXED);

    const uint64_t end =
        __atomic_load_n(&state->write_sequence, __ATOMIC_ACQUIRE);
    if (begin == end && !(end & 1U)) {
      candidate.write_sequence = end;
      *snapshot = candidate;
      if (candidate.magic != kSharedTimestampMagic ||
          candidate.version != kSharedTimestampVersion) {
        return kSharedTimestampReadBadProtocol;
      }
      if (!candidate.ready) {
        return kSharedTimestampReadNotReady;
      }
      if (candidate.stamp_ns == 0) {
        return kSharedTimestampReadEmpty;
      }
      return kSharedTimestampReadOk;
    }
  }
  return kSharedTimestampReadInconsistent;
}

class LegacyTimestampReadyGate {
 public:
  LegacyTimestampReadyGate(uint32_t required_samples,
                           uint64_t max_stamp_gap_ns,
                           uint64_t min_arrival_gap_ns,
                           uint64_t max_arrival_gap_ns)
      : required_samples_(required_samples < 2 ? 2 : required_samples),
        max_stamp_gap_ns_(max_stamp_gap_ns),
        min_arrival_gap_ns_(min_arrival_gap_ns),
        max_arrival_gap_ns_(max_arrival_gap_ns) {}

  bool Observe(uint64_t stamp_ns, uint32_t source,
               uint64_t arrival_monotonic_ns) {
    const bool continuous =
        consecutive_samples_ != 0 && source == last_source_ &&
        stamp_ns > last_stamp_ns_ &&
        stamp_ns - last_stamp_ns_ <= max_stamp_gap_ns_ &&
        arrival_monotonic_ns > last_arrival_monotonic_ns_ &&
        arrival_monotonic_ns - last_arrival_monotonic_ns_ >=
            min_arrival_gap_ns_ &&
        arrival_monotonic_ns - last_arrival_monotonic_ns_ <=
            max_arrival_gap_ns_;

    consecutive_samples_ = continuous ? consecutive_samples_ + 1U : 1U;
    last_stamp_ns_ = stamp_ns;
    last_source_ = source;
    last_arrival_monotonic_ns_ = arrival_monotonic_ns;
    return consecutive_samples_ >= required_samples_;
  }

  void Reset() {
    consecutive_samples_ = 0;
    last_stamp_ns_ = 0;
    last_source_ = kClockSourceUnknown;
    last_arrival_monotonic_ns_ = 0;
  }

  uint32_t consecutive_samples() const { return consecutive_samples_; }

 private:
  uint32_t required_samples_;
  uint64_t max_stamp_gap_ns_;
  uint64_t min_arrival_gap_ns_;
  uint64_t max_arrival_gap_ns_;
  uint32_t consecutive_samples_ = 0;
  uint64_t last_stamp_ns_ = 0;
  uint32_t last_source_ = kClockSourceUnknown;
  uint64_t last_arrival_monotonic_ns_ = 0;
};

static_assert(sizeof(SharedTimestampState) == 64,
              "Shared timestamp protocol size changed");

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_SHARED_TIMESTAMP_STATE_H_
