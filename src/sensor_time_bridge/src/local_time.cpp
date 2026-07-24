#include "sensor_time_bridge/local_time.h"

#include <limits>

namespace sensor_time_bridge
{

bool tickToLocalNs(uint64_t local_tick, uint32_t local_tick_hz,
                   uint64_t &local_stamp_ns, std::string *error)
{
  if (local_tick_hz == 0)
  {
    if (error) *error = "local_tick_hz is zero";
    return false;
  }
  const unsigned __int128 scaled =
      static_cast<unsigned __int128>(local_tick) * 1000000000ULL;
  const unsigned __int128 elapsed = scaled / local_tick_hz;
  const unsigned __int128 result =
      static_cast<unsigned __int128>(kSyntheticEpochNs) + elapsed;
  if (result > std::numeric_limits<uint64_t>::max())
  {
    if (error) *error = "LOCAL_SENSOR_TIME overflow";
    return false;
  }
  local_stamp_ns = static_cast<uint64_t>(result);
  return true;
}

bool extendCounter16Snapshot(uint32_t high_before, uint16_t counter,
                             bool update_pending, uint32_t high_after,
                             uint64_t &extended_tick)
{
  if (high_before != high_after) return false;
  uint64_t high = high_before;
  if (update_pending && counter < 0x8000U) ++high;
  extended_tick = (high << 16U) | counter;
  return true;
}

bool StreamMonotonicGate::accept(uint64_t session_id, uint32_t stream_id,
                                 uint64_t stamp_ns)
{
  if (!have_session_ || session_id != session_id_)
  {
    reset();
    have_session_ = true;
    session_id_ = session_id;
  }
  for (size_t i = 0; i < entry_count_; ++i)
  {
    if (entries_[i].stream_id != stream_id) continue;
    if (stamp_ns <= entries_[i].stamp_ns) return false;
    entries_[i].stamp_ns = stamp_ns;
    return true;
  }
  if (entry_count_ >= sizeof(entries_) / sizeof(entries_[0])) return false;
  entries_[entry_count_++] = Entry{stream_id, stamp_ns};
  return true;
}

void StreamMonotonicGate::reset()
{
  have_session_ = false;
  session_id_ = 0;
  entry_count_ = 0;
}

} // namespace sensor_time_bridge
