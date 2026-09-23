#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace fast_livo {

struct LivoScanContractSnapshot {
  std::uint64_t scan_id = 0;
  double scan_begin_time = 0.0;
  double scan_end_time = 0.0;
  std::size_t raw_point_count = 0;
  std::uint64_t image_events = 0;
  std::uint64_t lio_transactions = 0;
  std::uint64_t map_insertions = 0;
  bool measurement_required = false;
  bool state_time_monotonic = true;
};

class LivoScanLifecycle {
public:
  void begin(std::uint64_t scan_id, double scan_begin_time,
             double scan_end_time, std::size_t raw_point_count,
             bool measurement_required, double state_time)
  {
    validateCompleted();
    if (scan_end_time <= scan_begin_time || raw_point_count == 0)
      throw std::logic_error("invalid raw LiDAR scan lifecycle");
    snapshot_ = {};
    snapshot_.scan_id = scan_id;
    snapshot_.scan_begin_time = scan_begin_time;
    snapshot_.scan_end_time = scan_end_time;
    snapshot_.raw_point_count = raw_point_count;
    snapshot_.measurement_required = measurement_required;
    active_ = true;
    last_state_time_ = state_time;
  }

  void noteImage(double timestamp)
  {
    requireActive();
    noteStateTime(timestamp);
    ++snapshot_.image_events;
  }

  void noteLioTransaction(double timestamp)
  {
    requireActive();
    noteStateTime(timestamp);
    if (++snapshot_.lio_transactions != 1)
      throw std::logic_error("raw LiDAR scan dispatched more than one LIO transaction");
    if (timestamp < snapshot_.scan_end_time - kTimeEpsilon ||
        timestamp > snapshot_.scan_end_time + kTimeEpsilon)
      throw std::logic_error("LIO transaction was not dispatched at raw scan endpoint");
  }

  void noteMapInsertion()
  {
    requireActive();
    if (snapshot_.lio_transactions != 1)
      throw std::logic_error("map insertion without a LIO transaction");
    if (++snapshot_.map_insertions != 1)
      throw std::logic_error("raw LiDAR scan inserted into the map more than once");
  }

  void validateCompleted() const
  {
    if (!active_ || !snapshot_.measurement_required) return;
    if (snapshot_.lio_transactions != 1)
      throw std::logic_error("completed raw LiDAR scan did not produce exactly one LIO transaction");
    if (snapshot_.map_insertions > 1)
      throw std::logic_error("completed raw LiDAR scan produced multiple map insertions");
    if (!snapshot_.state_time_monotonic)
      throw std::logic_error("state timestamp moved backwards inside raw LiDAR scan");
  }

  bool active() const { return active_; }
  const LivoScanContractSnapshot &snapshot() const { return snapshot_; }

private:
  static constexpr double kTimeEpsilon = 1e-6;

  void requireActive() const
  {
    if (!active_) throw std::logic_error("LIVO event without an active raw LiDAR scan");
  }

  void noteStateTime(double timestamp)
  {
    if (timestamp < last_state_time_ - kTimeEpsilon)
    {
      snapshot_.state_time_monotonic = false;
      throw std::logic_error("LIVO state timestamp moved backwards");
    }
    if (timestamp > last_state_time_) last_state_time_ = timestamp;
  }

  bool active_ = false;
  double last_state_time_ = -1.0;
  LivoScanContractSnapshot snapshot_;
};

}  // namespace fast_livo
