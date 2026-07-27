#include "livox_ros_driver2/imu_time_anchor.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>

namespace livox_ros {
namespace {

uint64_t MonotonicNowNsForAnchor() {
  struct timespec value = {};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    return 0;
  }
  return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(value.tv_nsec);
}

}  // namespace

void WriteImuTimeAnchorState(ImuTimeAnchorState* state,
                             uint64_t writer_epoch,
                             uint64_t imu_stamp_ns,
                             uint64_t host_monotonic_ns,
                             uint64_t update_monotonic_ns,
                             uint32_t clock_source,
                             bool ready,
                             uint64_t uncertainty_ns) {
  if (state == nullptr) {
    return;
  }
  uint64_t sequence =
      __atomic_load_n(&state->write_sequence, __ATOMIC_RELAXED);
  sequence = (sequence & 1U) ? sequence + 2U : sequence + 1U;
  __atomic_store_n(&state->write_sequence, sequence, __ATOMIC_SEQ_CST);

  __atomic_store_n(&state->magic, kImuTimeAnchorMagic, __ATOMIC_RELAXED);
  __atomic_store_n(&state->version, kImuTimeAnchorVersion, __ATOMIC_RELAXED);
  __atomic_store_n(&state->writer_epoch, writer_epoch, __ATOMIC_RELAXED);
  __atomic_store_n(&state->imu_stamp_ns, imu_stamp_ns, __ATOMIC_RELAXED);
  __atomic_store_n(&state->host_monotonic_ns, host_monotonic_ns,
                   __ATOMIC_RELAXED);
  __atomic_store_n(&state->update_monotonic_ns, update_monotonic_ns,
                   __ATOMIC_RELAXED);
  __atomic_store_n(&state->clock_source, clock_source, __ATOMIC_RELAXED);
  __atomic_store_n(&state->ready, ready ? 1U : 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&state->uncertainty_ns, uncertainty_ns,
                   __ATOMIC_RELAXED);

  __atomic_store_n(&state->write_sequence, sequence + 1U, __ATOMIC_RELEASE);
}

ImuTimeAnchorReadResult ReadImuTimeAnchorState(
    const ImuTimeAnchorState* state, ImuTimeAnchorSnapshot* snapshot,
    unsigned int max_attempts) {
  if (state == nullptr || snapshot == nullptr) {
    return kImuTimeAnchorReadInconsistent;
  }
  for (unsigned int attempt = 0; attempt < max_attempts; ++attempt) {
    const uint64_t begin =
        __atomic_load_n(&state->write_sequence, __ATOMIC_ACQUIRE);
    if (begin & 1U) {
      continue;
    }

    ImuTimeAnchorSnapshot candidate;
    candidate.magic = __atomic_load_n(&state->magic, __ATOMIC_RELAXED);
    candidate.version = __atomic_load_n(&state->version, __ATOMIC_RELAXED);
    candidate.writer_epoch =
        __atomic_load_n(&state->writer_epoch, __ATOMIC_RELAXED);
    candidate.imu_stamp_ns =
        __atomic_load_n(&state->imu_stamp_ns, __ATOMIC_RELAXED);
    candidate.host_monotonic_ns =
        __atomic_load_n(&state->host_monotonic_ns, __ATOMIC_RELAXED);
    candidate.update_monotonic_ns =
        __atomic_load_n(&state->update_monotonic_ns, __ATOMIC_RELAXED);
    candidate.clock_source =
        __atomic_load_n(&state->clock_source, __ATOMIC_RELAXED);
    candidate.ready =
        __atomic_load_n(&state->ready, __ATOMIC_RELAXED) != 0;
    candidate.uncertainty_ns =
        __atomic_load_n(&state->uncertainty_ns, __ATOMIC_RELAXED);

    const uint64_t end =
        __atomic_load_n(&state->write_sequence, __ATOMIC_ACQUIRE);
    if (begin != end || (end & 1U)) {
      continue;
    }
    candidate.write_sequence = end;
    *snapshot = candidate;
    if (candidate.magic != kImuTimeAnchorMagic ||
        candidate.version != kImuTimeAnchorVersion) {
      return kImuTimeAnchorReadBadProtocol;
    }
    if (candidate.writer_epoch == 0 || !candidate.ready) {
      return kImuTimeAnchorReadNotReady;
    }
    if (candidate.imu_stamp_ns == 0 ||
        candidate.host_monotonic_ns == 0) {
      return kImuTimeAnchorReadEmpty;
    }
    return kImuTimeAnchorReadOk;
  }
  return kImuTimeAnchorReadInconsistent;
}

ImuTimeAnchorWriter::ImuTimeAnchorWriter(
    const ImuTimeAnchorWriterConfig& config)
    : config_(config) {
  config_.min_ready_samples = std::max(2U, config_.min_ready_samples);
  config_.max_stamp_step_ns =
      std::max<uint64_t>(1, config_.max_stamp_step_ns);
  config_.uncertainty_ns =
      std::max<uint64_t>(1, config_.uncertainty_ns);
}

ImuTimeAnchorWriter::~ImuTimeAnchorWriter() {
  Close();
}

bool ImuTimeAnchorWriter::Open(const std::string& path,
                               uint64_t writer_epoch,
                               std::string* error) {
  Close();
  if (path.empty() || path.front() != '/') {
    if (error != nullptr) {
      *error = "imu_timeshare_path must be absolute";
    }
    return false;
  }
  if (writer_epoch == 0) {
    if (error != nullptr) {
      *error = "writer_epoch must be nonzero";
    }
    return false;
  }

  const int fd = open(path.c_str(), O_CREAT | O_RDWR, 0660);
  if (fd < 0) {
    if (error != nullptr) {
      *error = std::string("open failed: ") + strerror(errno);
    }
    return false;
  }
  if (ftruncate(fd, sizeof(ImuTimeAnchorState)) != 0) {
    if (error != nullptr) {
      *error = std::string("ftruncate failed: ") + strerror(errno);
    }
    close(fd);
    return false;
  }
  void* mapping = mmap(nullptr, sizeof(ImuTimeAnchorState),
                       PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  const int mmap_error = errno;
  close(fd);
  if (mapping == MAP_FAILED) {
    if (error != nullptr) {
      *error = std::string("mmap failed: ") + strerror(mmap_error);
    }
    return false;
  }

  state_ = static_cast<ImuTimeAnchorState*>(mapping);
  path_ = path;
  writer_epoch_ = writer_epoch;
  ResetContinuity();
  Commit(0, 0, MonotonicNowNsForAnchor(),
         kImuAnchorClockUnknown, false);
  return true;
}

void ImuTimeAnchorWriter::Close() {
  if (state_ == nullptr) {
    return;
  }
  Invalidate(MonotonicNowNsForAnchor());
  munmap(state_, sizeof(ImuTimeAnchorState));
  state_ = nullptr;
  path_.clear();
  writer_epoch_ = 0;
}

bool ImuTimeAnchorWriter::Observe(uint64_t imu_stamp_ns,
                                  uint64_t host_monotonic_ns,
                                  uint32_t clock_source,
                                  uint64_t update_monotonic_ns) {
  if (state_ == nullptr) {
    return false;
  }
  if (imu_stamp_ns == 0) {
    ++stats_.invalid_stamp;
    Invalidate(update_monotonic_ns);
    return false;
  }
  if (host_monotonic_ns == 0) {
    ++stats_.invalid_host_time;
    Invalidate(update_monotonic_ns);
    return false;
  }
  if (clock_source != kImuAnchorDevicePtp &&
      clock_source != kImuAnchorDeviceGps) {
    ++stats_.invalid_clock_source;
    Invalidate(update_monotonic_ns);
    return false;
  }

  bool continuous = consecutive_samples_ != 0;
  if (continuous && clock_source != last_clock_source_) {
    ++stats_.clock_source_change;
    continuous = false;
  }
  if (continuous && imu_stamp_ns == last_imu_stamp_ns_) {
    ++stats_.duplicate;
    continuous = false;
  } else if (continuous && imu_stamp_ns < last_imu_stamp_ns_) {
    ++stats_.backward;
    continuous = false;
  } else if (continuous &&
             imu_stamp_ns - last_imu_stamp_ns_ >
                 config_.max_stamp_step_ns) {
    ++stats_.jump;
    continuous = false;
  }
  if (continuous && host_monotonic_ns <= last_host_monotonic_ns_) {
    ++stats_.invalid_host_time;
    continuous = false;
  }

  consecutive_samples_ = continuous ? consecutive_samples_ + 1U : 1U;
  last_imu_stamp_ns_ = imu_stamp_ns;
  last_host_monotonic_ns_ = host_monotonic_ns;
  last_clock_source_ = clock_source;
  ready_ = consecutive_samples_ >= config_.min_ready_samples;
  ++stats_.accepted;
  Commit(imu_stamp_ns, host_monotonic_ns, update_monotonic_ns,
         clock_source, ready_);
  return ready_;
}

void ImuTimeAnchorWriter::Invalidate(uint64_t update_monotonic_ns) {
  ResetContinuity();
  Commit(0, 0, update_monotonic_ns, kImuAnchorClockUnknown, false);
}

ImuTimeAnchorReadResult ImuTimeAnchorWriter::ReadSnapshot(
    ImuTimeAnchorSnapshot* snapshot) const {
  return ReadImuTimeAnchorState(state_, snapshot);
}

void ImuTimeAnchorWriter::ResetContinuity() {
  last_imu_stamp_ns_ = 0;
  last_host_monotonic_ns_ = 0;
  last_clock_source_ = kImuAnchorClockUnknown;
  consecutive_samples_ = 0;
  ready_ = false;
}

void ImuTimeAnchorWriter::Commit(uint64_t imu_stamp_ns,
                                 uint64_t host_monotonic_ns,
                                 uint64_t update_monotonic_ns,
                                 uint32_t clock_source,
                                 bool ready) {
  WriteImuTimeAnchorState(
      state_, writer_epoch_, imu_stamp_ns, host_monotonic_ns,
      update_monotonic_ns, clock_source, ready, config_.uncertainty_ns);
}

}  // namespace livox_ros
