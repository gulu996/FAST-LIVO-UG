#pragma once

#include <cstdint>
#include <string>

namespace sensor_time_bridge
{

constexpr uint64_t kSyntheticEpochNs = 946684800000000000ULL;

bool tickToLocalNs(uint64_t local_tick, uint32_t local_tick_hz,
                   uint64_t &local_stamp_ns, std::string *error = nullptr);

// Host-side reference for the STM32 16-bit CNT/UIF snapshot algorithm.
// A false return means the overflow ISR changed the high word and the caller
// must take a fresh snapshot.
bool extendCounter16Snapshot(uint32_t high_before, uint16_t counter,
                             bool update_pending, uint32_t high_after,
                             uint64_t &extended_tick);

class StreamMonotonicGate
{
public:
  bool accept(uint64_t session_id, uint32_t stream_id, uint64_t stamp_ns);
  void reset();

private:
  uint64_t session_id_ = 0;
  bool have_session_ = false;
  struct Entry
  {
    uint32_t stream_id;
    uint64_t stamp_ns;
  };
  Entry entries_[16]{};
  size_t entry_count_ = 0;
};

} // namespace sensor_time_bridge
