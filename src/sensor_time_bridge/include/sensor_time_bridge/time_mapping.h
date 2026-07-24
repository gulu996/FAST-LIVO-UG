#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace sensor_time_bridge
{

enum class TimeState : uint8_t
{
  LOCAL_ONLY = 0,
  ACQUIRING = 1,
  LOCKED = 2,
  HOLDOVER = 3,
  RELOCKING = 4,
  INVALID = 5
};

struct MappingParameters
{
  size_t mapping_window_size = 120;
  size_t min_pps_pairs = 10;
  uint64_t min_fit_span_ns = 9000000000ULL;
  double max_pps_residual_ns = 2000000.0;
  double max_scale_ppm = 10000.0;
  uint64_t pps_timeout_ns = 3500000000ULL;
  uint64_t holdover_invalid_timeout_ns = 3600000000000ULL;
  size_t relock_confirm_count = 10;
  uint64_t max_pair_interval_error_ns = 100000000ULL;
  uint64_t max_relock_step_ns = 100000000ULL;
  size_t mapping_history_size = 64;
  double holdover_uncertainty_growth_ns_per_s = 1000.0;
};

struct TimePair
{
  uint64_t local_ns = 0;
  uint64_t utc_ns = 0;
  uint64_t uncertainty_ns = 0;
};

struct AffineMapping
{
  bool valid = false;
  uint64_t local_reference_ns = 0;
  uint64_t utc_reference_ns = 0;
  double utc_ns_per_local_ns = 1.0;
  double slope_ppm = 0.0;
  uint64_t valid_from_local_ns = 0;
  uint64_t fit_span_ns = 0;
  uint32_t sample_count = 0;
  double residual_rms_ns = 0.0;
  double residual_max_ns = 0.0;
  uint64_t time_uncertainty_ns = 0;

  bool localToUtc(uint64_t local_ns, uint64_t &utc_ns) const;
  bool utcToLocal(uint64_t utc_ns, uint64_t &local_ns) const;
};

bool robustAffineFit(const std::vector<TimePair> &pairs,
                     double huber_limit_ns,
                     AffineMapping &mapping);

struct MappingSnapshot
{
  uint32_t version = 0;
  AffineMapping mapping;
};

class MappingStateMachine
{
public:
  explicit MappingStateMachine(const MappingParameters &parameters = MappingParameters{});

  bool addPair(const TimePair &pair, std::string *reason = nullptr);
  void update(uint64_t current_local_ns);
  void reset();

  TimeState state() const { return state_; }
  const AffineMapping &mapping() const { return active_; }
  uint32_t mappingVersion() const { return mapping_version_; }
  uint32_t acceptedPairCount() const { return accepted_pair_count_; }
  uint32_t rejectedPairCount() const { return rejected_pair_count_; }
  uint64_t secondsSinceLastPair(uint64_t current_local_ns) const;
  uint64_t uncertainty(uint64_t current_local_ns) const;
  bool mappingUsable() const;
  const std::deque<MappingSnapshot> &history() const { return history_; }

private:
  bool appendChecked(std::deque<TimePair> &pairs, const TimePair &pair,
                     std::string *reason);
  bool candidateReady(AffineMapping &fit) const;
  bool relockContinuous(const AffineMapping &candidate,
                        uint64_t local_ns) const;
  void install(const AffineMapping &mapping);
  std::vector<TimePair> candidateVector() const;

  MappingParameters parameters_;
  TimeState state_ = TimeState::LOCAL_ONLY;
  AffineMapping active_;
  std::deque<MappingSnapshot> history_;
  std::deque<TimePair> pairs_;
  std::deque<TimePair> candidate_pairs_;
  uint32_t mapping_version_ = 0;
  uint32_t accepted_pair_count_ = 0;
  uint32_t rejected_pair_count_ = 0;
  uint64_t last_pair_local_ns_ = 0;
  uint64_t holdover_start_local_ns_ = 0;
  bool have_last_pair_ = false;
};

class HostMonotonicMapper
{
public:
  explicit HostMonotonicMapper(size_t window_size = 120);
  bool add(uint64_t host_monotonic_ns, uint64_t local_ns);
  bool toLocal(uint64_t host_monotonic_ns, uint64_t &local_ns,
               uint64_t &uncertainty_ns) const;
  void reset();

private:
  size_t window_size_;
  std::deque<TimePair> pairs_;
  AffineMapping mapping_;
};

} // namespace sensor_time_bridge
