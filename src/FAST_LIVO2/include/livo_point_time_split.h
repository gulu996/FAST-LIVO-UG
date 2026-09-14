#pragma once

#include <cmath>
#include <stdexcept>

namespace fast_livo {

// Both fresh scans and carried points must obey the same image-time cut.
// Curvature stores milliseconds relative to the supplied source epoch.
template <typename Points>
void appendLivoTimeSplit(const Points &input, double source_epoch_s,
                        double begin_s, double end_s, Points &current,
                        Points &pending) {
  if (!std::isfinite(source_epoch_s) || !std::isfinite(begin_s) ||
      !std::isfinite(end_s) || end_s <= begin_s ||
      &input == &current || &input == &pending || &current == &pending)
    throw std::invalid_argument("invalid LIVO point-time split");
  const double duration_ms = (end_s - begin_s) * 1000.0;
  for (const auto &point : input) {
    const double offset_ms = point.curvature + (source_epoch_s - begin_s) * 1000.0;
    if (!std::isfinite(offset_ms))
      throw std::invalid_argument("non-finite LIVO point timestamp");
    auto adjusted = point;
    if (offset_ms < duration_ms) {
      adjusted.curvature = static_cast<float>(offset_ms);
      current.push_back(adjusted);
    } else {
      adjusted.curvature = static_cast<float>(offset_ms - duration_ms);
      pending.push_back(adjusted);
    }
  }
}

}  // namespace fast_livo
