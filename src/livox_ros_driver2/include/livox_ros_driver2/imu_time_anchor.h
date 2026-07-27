#ifndef LIVOX_ROS_DRIVER2_IMU_TIME_ANCHOR_H_
#define LIVOX_ROS_DRIVER2_IMU_TIME_ANCHOR_H_

#include <stddef.h>
#include <stdint.h>

#include <string>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "IMU time anchor protocol requires a little-endian target"
#endif

namespace livox_ros {

constexpr uint32_t kImuTimeAnchorMagic = 0x31414d49U;  // "IMA1"
constexpr uint32_t kImuTimeAnchorVersion = 1U;

enum ImuTimeAnchorClockSource : uint32_t {
  kImuAnchorClockUnknown = 0,
  kImuAnchorDevicePtp = 1,
  kImuAnchorDeviceGps = 2,
};

#pragma pack(push, 1)
struct alignas(8) ImuTimeAnchorState {
  uint32_t magic;
  uint32_t version;
  uint64_t writer_epoch;
  uint64_t write_sequence;
  uint64_t imu_stamp_ns;
  uint64_t host_monotonic_ns;
  uint64_t update_monotonic_ns;
  uint32_t clock_source;
  uint32_t ready;
  uint64_t uncertainty_ns;
};
#pragma pack(pop)

struct ImuTimeAnchorSnapshot {
  uint32_t magic = 0;
  uint32_t version = 0;
  uint64_t writer_epoch = 0;
  uint64_t write_sequence = 0;
  uint64_t imu_stamp_ns = 0;
  uint64_t host_monotonic_ns = 0;
  uint64_t update_monotonic_ns = 0;
  uint32_t clock_source = kImuAnchorClockUnknown;
  bool ready = false;
  uint64_t uncertainty_ns = 0;
};

enum ImuTimeAnchorReadResult {
  kImuTimeAnchorReadOk = 0,
  kImuTimeAnchorReadInconsistent,
  kImuTimeAnchorReadBadProtocol,
  kImuTimeAnchorReadNotReady,
  kImuTimeAnchorReadEmpty,
};

void WriteImuTimeAnchorState(ImuTimeAnchorState* state,
                             uint64_t writer_epoch,
                             uint64_t imu_stamp_ns,
                             uint64_t host_monotonic_ns,
                             uint64_t update_monotonic_ns,
                             uint32_t clock_source,
                             bool ready,
                             uint64_t uncertainty_ns);

ImuTimeAnchorReadResult ReadImuTimeAnchorState(
    const ImuTimeAnchorState* state, ImuTimeAnchorSnapshot* snapshot,
    unsigned int max_attempts = 8);

struct ImuTimeAnchorWriterConfig {
  uint32_t min_ready_samples = 3;
  uint64_t max_stamp_step_ns = 100000000ULL;
  uint64_t uncertainty_ns = 10000000ULL;
};

struct ImuTimeAnchorWriterStats {
  uint64_t accepted = 0;
  uint64_t invalid_stamp = 0;
  uint64_t invalid_host_time = 0;
  uint64_t invalid_clock_source = 0;
  uint64_t backward = 0;
  uint64_t duplicate = 0;
  uint64_t jump = 0;
  uint64_t clock_source_change = 0;
};

class ImuTimeAnchorWriter {
 public:
  explicit ImuTimeAnchorWriter(const ImuTimeAnchorWriterConfig& config);
  ~ImuTimeAnchorWriter();

  ImuTimeAnchorWriter(const ImuTimeAnchorWriter&) = delete;
  ImuTimeAnchorWriter& operator=(const ImuTimeAnchorWriter&) = delete;

  bool Open(const std::string& path, uint64_t writer_epoch,
            std::string* error);
  void Close();
  bool Observe(uint64_t imu_stamp_ns, uint64_t host_monotonic_ns,
               uint32_t clock_source, uint64_t update_monotonic_ns);
  void Invalidate(uint64_t update_monotonic_ns);
  ImuTimeAnchorReadResult ReadSnapshot(
      ImuTimeAnchorSnapshot* snapshot) const;

  bool is_open() const { return state_ != nullptr; }
  bool ready() const { return ready_; }
  uint64_t writer_epoch() const { return writer_epoch_; }
  uint32_t consecutive_samples() const { return consecutive_samples_; }
  const ImuTimeAnchorWriterStats& stats() const { return stats_; }

 private:
  void ResetContinuity();
  void Commit(uint64_t imu_stamp_ns, uint64_t host_monotonic_ns,
              uint64_t update_monotonic_ns, uint32_t clock_source,
              bool ready);

  ImuTimeAnchorWriterConfig config_;
  ImuTimeAnchorWriterStats stats_;
  ImuTimeAnchorState* state_ = nullptr;
  std::string path_;
  uint64_t writer_epoch_ = 0;
  uint64_t last_imu_stamp_ns_ = 0;
  uint64_t last_host_monotonic_ns_ = 0;
  uint32_t last_clock_source_ = kImuAnchorClockUnknown;
  uint32_t consecutive_samples_ = 0;
  bool ready_ = false;
};

static_assert(sizeof(ImuTimeAnchorState) == 64,
              "IMU time anchor protocol size changed");
static_assert(offsetof(ImuTimeAnchorState, magic) == 0, "bad magic offset");
static_assert(offsetof(ImuTimeAnchorState, version) == 4, "bad version offset");
static_assert(offsetof(ImuTimeAnchorState, writer_epoch) == 8,
              "bad writer_epoch offset");
static_assert(offsetof(ImuTimeAnchorState, write_sequence) == 16,
              "bad write_sequence offset");
static_assert(offsetof(ImuTimeAnchorState, imu_stamp_ns) == 24,
              "bad imu_stamp_ns offset");
static_assert(offsetof(ImuTimeAnchorState, host_monotonic_ns) == 32,
              "bad host_monotonic_ns offset");
static_assert(offsetof(ImuTimeAnchorState, update_monotonic_ns) == 40,
              "bad update_monotonic_ns offset");
static_assert(offsetof(ImuTimeAnchorState, clock_source) == 48,
              "bad clock_source offset");
static_assert(offsetof(ImuTimeAnchorState, ready) == 52, "bad ready offset");
static_assert(offsetof(ImuTimeAnchorState, uncertainty_ns) == 56,
              "bad uncertainty_ns offset");

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_IMU_TIME_ANCHOR_H_
