#include "sensor_time_bridge/time_mapping.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sensor_time_bridge
{
namespace
{
uint64_t clampU64(long double value)
{
  if (!(value >= 0.0L) ||
      value > static_cast<long double>(std::numeric_limits<uint64_t>::max()))
    return std::numeric_limits<uint64_t>::max();
  return static_cast<uint64_t>(std::llround(value));
}

double median(std::vector<double> values)
{
  if (values.empty()) return 0.0;
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0) return upper;
  std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
  return 0.5 * (upper + values[middle - 1]);
}
} // namespace

bool AffineMapping::localToUtc(uint64_t local_ns, uint64_t &utc_ns) const
{
  if (!valid || !std::isfinite(utc_ns_per_local_ns) ||
      utc_ns_per_local_ns <= 0.0)
    return false;
  const long double delta =
      static_cast<long double>(local_ns) - static_cast<long double>(local_reference_ns);
  const long double result = static_cast<long double>(utc_reference_ns) +
                             static_cast<long double>(utc_ns_per_local_ns) * delta;
  const uint64_t converted = clampU64(result);
  if (converted == std::numeric_limits<uint64_t>::max()) return false;
  utc_ns = converted;
  return true;
}

bool AffineMapping::utcToLocal(uint64_t utc_ns, uint64_t &local_ns) const
{
  if (!valid || !std::isfinite(utc_ns_per_local_ns) ||
      utc_ns_per_local_ns <= 0.0)
    return false;
  const long double delta =
      static_cast<long double>(utc_ns) - static_cast<long double>(utc_reference_ns);
  const long double result = static_cast<long double>(local_reference_ns) +
                             delta / static_cast<long double>(utc_ns_per_local_ns);
  const uint64_t converted = clampU64(result);
  if (converted == std::numeric_limits<uint64_t>::max()) return false;
  local_ns = converted;
  return true;
}

bool robustAffineFit(const std::vector<TimePair> &pairs,
                     double huber_limit_ns,
                     AffineMapping &mapping)
{
  if (pairs.size() < 2) return false;
  const TimePair &reference = pairs[pairs.size() / 2];
  std::vector<long double> x(pairs.size());
  std::vector<long double> y(pairs.size());
  std::vector<double> base_weights(pairs.size(), 1.0);
  std::vector<double> weights(pairs.size(), 1.0);
  uint64_t minimum_uncertainty_ns = std::numeric_limits<uint64_t>::max();
  for (const TimePair &pair : pairs)
    minimum_uncertainty_ns =
        std::min(minimum_uncertainty_ns, std::max<uint64_t>(1, pair.uncertainty_ns));
  for (size_t i = 0; i < pairs.size(); ++i)
  {
    x[i] = static_cast<long double>(pairs[i].local_ns) - reference.local_ns;
    y[i] = static_cast<long double>(pairs[i].utc_ns) - reference.utc_ns;
    const double sigma = std::max<uint64_t>(1, pairs[i].uncertainty_ns);
    const double relative_sigma =
        static_cast<double>(minimum_uncertainty_ns) / sigma;
    base_weights[i] = relative_sigma * relative_sigma;
    weights[i] = base_weights[i];
  }

  long double slope = 1.0L;
  long double intercept = 0.0L;
  const auto fit = [&x, &y](const std::vector<double> &fit_weights,
                            long double &fit_slope,
                            long double &fit_intercept)
  {
    long double sw = 0.0L, sx = 0.0L, sy = 0.0L;
    long double sxx = 0.0L, sxy = 0.0L;
    for (size_t i = 0; i < x.size(); ++i)
    {
      const long double w = fit_weights[i];
      sw += w;
      sx += w * x[i];
      sy += w * y[i];
      sxx += w * x[i] * x[i];
      sxy += w * x[i] * y[i];
    }
    const long double denominator = sw * sxx - sx * sx;
    if (sw <= 0.0L || std::fabs(denominator) < 1.0L) return false;
    fit_slope = (sw * sxy - sx * sy) / denominator;
    fit_intercept = (sy - fit_slope * sx) / sw;
    return fit_slope > 0.0L &&
           std::isfinite(static_cast<double>(fit_slope));
  };

  for (int iteration = 0; iteration < 8; ++iteration)
  {
    if (!fit(weights, slope, intercept)) return false;

    std::vector<double> residuals;
    residuals.reserve(pairs.size());
    for (size_t i = 0; i < pairs.size(); ++i)
      residuals.push_back(static_cast<double>(y[i] - (intercept + slope * x[i])));
    std::vector<double> absolute;
    absolute.reserve(residuals.size());
    const double center = median(residuals);
    for (double residual : residuals) absolute.push_back(std::fabs(residual - center));
    const double robust_sigma = std::max(1.0, 1.4826 * median(absolute));
    const double limit = std::max(robust_sigma * 1.5, std::max(1.0, huber_limit_ns));
    for (size_t i = 0; i < pairs.size(); ++i)
    {
      const double abs_residual = std::fabs(residuals[i] - center);
      const double huber_weight = abs_residual <= limit ? 1.0 : limit / abs_residual;
      weights[i] = base_weights[i] * huber_weight;
    }
  }

  std::vector<double> residuals;
  residuals.reserve(pairs.size());
  for (size_t i = 0; i < pairs.size(); ++i)
    residuals.push_back(
        static_cast<double>(y[i] - (intercept + slope * x[i])));
  const double center = median(residuals);
  std::vector<double> absolute;
  absolute.reserve(residuals.size());
  for (double residual : residuals)
    absolute.push_back(std::fabs(residual - center));
  const double robust_sigma = std::max(1.0, 1.4826 * median(absolute));
  const double inlier_limit =
      std::max(std::max(1.0, huber_limit_ns), robust_sigma * 4.0);
  size_t inlier_count = 0;
  for (size_t i = 0; i < pairs.size(); ++i)
  {
    if (std::fabs(residuals[i] - center) <= inlier_limit)
    {
      weights[i] = base_weights[i];
      ++inlier_count;
    }
    else
    {
      weights[i] = 0.0;
    }
  }
  if (inlier_count < 2 || !fit(weights, slope, intercept)) return false;

  long double squared_sum = 0.0L;
  double max_abs = 0.0;
  for (size_t i = 0; i < pairs.size(); ++i)
  {
    if (weights[i] <= 0.0) continue;
    const double residual =
        static_cast<double>(y[i] - (intercept + slope * x[i]));
    squared_sum += static_cast<long double>(residual) * residual;
    max_abs = std::max(max_abs, std::fabs(residual));
  }

  const long double utc_reference =
      static_cast<long double>(reference.utc_ns) + intercept;
  const uint64_t utc_reference_ns = clampU64(utc_reference);
  if (utc_reference_ns == std::numeric_limits<uint64_t>::max()) return false;
  mapping.valid = true;
  mapping.local_reference_ns = reference.local_ns;
  mapping.utc_reference_ns = utc_reference_ns;
  mapping.utc_ns_per_local_ns = static_cast<double>(slope);
  mapping.slope_ppm = (mapping.utc_ns_per_local_ns - 1.0) * 1.0e6;
  mapping.valid_from_local_ns = pairs.front().local_ns;
  mapping.fit_span_ns = pairs.back().local_ns - pairs.front().local_ns;
  mapping.sample_count = static_cast<uint32_t>(inlier_count);
  mapping.residual_rms_ns =
      std::sqrt(static_cast<double>(squared_sum / inlier_count));
  mapping.residual_max_ns = max_abs;
  mapping.time_uncertainty_ns = static_cast<uint64_t>(
      std::ceil(std::max(mapping.residual_rms_ns,
                         static_cast<double>(reference.uncertainty_ns))));
  return true;
}

MappingStateMachine::MappingStateMachine(const MappingParameters &parameters)
    : parameters_(parameters)
{
  parameters_.mapping_window_size = std::max<size_t>(2, parameters_.mapping_window_size);
  parameters_.min_pps_pairs = std::max<size_t>(2, parameters_.min_pps_pairs);
  parameters_.relock_confirm_count = std::max<size_t>(2, parameters_.relock_confirm_count);
  parameters_.mapping_history_size =
      std::max<size_t>(1, parameters_.mapping_history_size);
}

bool MappingStateMachine::appendChecked(std::deque<TimePair> &pairs,
                                        const TimePair &pair,
                                        std::string *reason)
{
  if (pair.local_ns == 0 || pair.utc_ns == 0)
  {
    if (reason) *reason = "zero timestamp";
    return false;
  }
  if (!pairs.empty())
  {
    const TimePair &previous = pairs.back();
    if (pair.local_ns <= previous.local_ns || pair.utc_ns <= previous.utc_ns)
    {
      if (reason) *reason = "non-monotonic pair";
      return false;
    }
    const uint64_t local_delta = pair.local_ns - previous.local_ns;
    const uint64_t utc_delta = pair.utc_ns - previous.utc_ns;
    const uint64_t interval_error = local_delta > utc_delta
                                        ? local_delta - utc_delta
                                        : utc_delta - local_delta;
    if (interval_error > parameters_.max_pair_interval_error_ns)
    {
      if (reason) *reason = "PPS interval mismatch";
      return false;
    }
  }
  pairs.push_back(pair);
  while (pairs.size() > parameters_.mapping_window_size) pairs.pop_front();
  return true;
}

std::vector<TimePair> MappingStateMachine::candidateVector() const
{
  const std::deque<TimePair> &source =
      state_ == TimeState::RELOCKING ? candidate_pairs_ : pairs_;
  return std::vector<TimePair>(source.begin(), source.end());
}

bool MappingStateMachine::candidateReady(AffineMapping &fit) const
{
  const std::vector<TimePair> candidate = candidateVector();
  const size_t required =
      state_ == TimeState::RELOCKING ? parameters_.relock_confirm_count
                                    : parameters_.min_pps_pairs;
  if (candidate.size() < required) return false;
  if (candidate.back().local_ns - candidate.front().local_ns <
      parameters_.min_fit_span_ns)
    return false;
  if (!robustAffineFit(candidate, parameters_.max_pps_residual_ns, fit)) return false;
  return std::fabs(fit.slope_ppm) <= parameters_.max_scale_ppm &&
         fit.residual_rms_ns <= parameters_.max_pps_residual_ns &&
         fit.residual_max_ns <= parameters_.max_pps_residual_ns * 4.0;
}

bool MappingStateMachine::relockContinuous(
    const AffineMapping &candidate, uint64_t local_ns) const
{
  if (!active_.valid) return true;
  uint64_t old_utc_ns = 0;
  uint64_t candidate_utc_ns = 0;
  if (!active_.localToUtc(local_ns, old_utc_ns) ||
      !candidate.localToUtc(local_ns, candidate_utc_ns))
    return false;
  const uint64_t step_ns = old_utc_ns > candidate_utc_ns
                               ? old_utc_ns - candidate_utc_ns
                               : candidate_utc_ns - old_utc_ns;
  return step_ns <= parameters_.max_relock_step_ns;
}

void MappingStateMachine::install(const AffineMapping &mapping)
{
  active_ = mapping;
  ++mapping_version_;
  history_.push_back(MappingSnapshot{mapping_version_, active_});
  while (history_.size() > parameters_.mapping_history_size)
    history_.pop_front();
}

bool MappingStateMachine::addPair(const TimePair &pair, std::string *reason)
{
  std::deque<TimePair> *target =
      state_ == TimeState::RELOCKING ? &candidate_pairs_ : &pairs_;
  if (state_ == TimeState::HOLDOVER)
  {
    state_ = TimeState::RELOCKING;
    candidate_pairs_.clear();
    target = &candidate_pairs_;
  }
  if (!appendChecked(*target, pair, reason))
  {
    ++rejected_pair_count_;
    return false;
  }
  ++accepted_pair_count_;
  last_pair_local_ns_ = pair.local_ns;
  have_last_pair_ = true;
  if (state_ == TimeState::LOCAL_ONLY) state_ = TimeState::ACQUIRING;

  AffineMapping fit;
  if (candidateReady(fit))
  {
    if (state_ == TimeState::RELOCKING &&
        !relockContinuous(fit, pair.local_ns))
    {
      if (reason) *reason = "candidate would step active UTC mapping";
      ++rejected_pair_count_;
      return false;
    }
    install(fit);
    state_ = TimeState::LOCKED;
    if (!candidate_pairs_.empty())
    {
      pairs_ = candidate_pairs_;
      candidate_pairs_.clear();
    }
  }
  else if (state_ == TimeState::LOCKED)
  {
    std::vector<TimePair> current(pairs_.begin(), pairs_.end());
    if (robustAffineFit(current, parameters_.max_pps_residual_ns, fit) &&
        std::fabs(fit.slope_ppm) <= parameters_.max_scale_ppm &&
        fit.residual_rms_ns <= parameters_.max_pps_residual_ns)
    {
      install(fit);
    }
  }
  return true;
}

void MappingStateMachine::update(uint64_t current_local_ns)
{
  if (!have_last_pair_) return;
  const uint64_t since =
      current_local_ns > last_pair_local_ns_ ? current_local_ns - last_pair_local_ns_ : 0;
  if ((state_ == TimeState::LOCKED || state_ == TimeState::ACQUIRING) &&
      since > parameters_.pps_timeout_ns)
  {
    if (active_.valid)
    {
      state_ = TimeState::HOLDOVER;
      holdover_start_local_ns_ = last_pair_local_ns_;
    }
    else
    {
      state_ = TimeState::LOCAL_ONLY;
      pairs_.clear();
    }
  }
  if ((state_ == TimeState::HOLDOVER || state_ == TimeState::RELOCKING) &&
      since > parameters_.holdover_invalid_timeout_ns)
  {
    state_ = TimeState::INVALID;
  }
}

void MappingStateMachine::reset()
{
  state_ = TimeState::LOCAL_ONLY;
  active_ = AffineMapping{};
  history_.clear();
  pairs_.clear();
  candidate_pairs_.clear();
  mapping_version_ = 0;
  accepted_pair_count_ = 0;
  rejected_pair_count_ = 0;
  last_pair_local_ns_ = 0;
  holdover_start_local_ns_ = 0;
  have_last_pair_ = false;
}

uint64_t MappingStateMachine::secondsSinceLastPair(uint64_t current_local_ns) const
{
  if (!have_last_pair_ || current_local_ns <= last_pair_local_ns_) return 0;
  return (current_local_ns - last_pair_local_ns_) / 1000000000ULL;
}

uint64_t MappingStateMachine::uncertainty(uint64_t current_local_ns) const
{
  if (!active_.valid) return std::numeric_limits<uint64_t>::max();
  long double uncertainty = active_.time_uncertainty_ns;
  if (state_ == TimeState::HOLDOVER || state_ == TimeState::RELOCKING)
  {
    const uint64_t elapsed =
        current_local_ns > holdover_start_local_ns_
            ? current_local_ns - holdover_start_local_ns_
            : 0;
    uncertainty += parameters_.holdover_uncertainty_growth_ns_per_s *
                   (static_cast<long double>(elapsed) / 1.0e9L);
  }
  return clampU64(uncertainty);
}

bool MappingStateMachine::mappingUsable() const
{
  return active_.valid &&
         (state_ == TimeState::LOCKED || state_ == TimeState::HOLDOVER ||
          state_ == TimeState::RELOCKING);
}

HostMonotonicMapper::HostMonotonicMapper(size_t window_size)
    : window_size_(std::max<size_t>(3, window_size))
{
}

bool HostMonotonicMapper::add(uint64_t host_monotonic_ns, uint64_t local_ns)
{
  if (!pairs_.empty() &&
      (host_monotonic_ns <= pairs_.back().local_ns ||
       local_ns <= pairs_.back().utc_ns))
    return false;
  pairs_.push_back(TimePair{host_monotonic_ns, local_ns, 1000000});
  while (pairs_.size() > window_size_) pairs_.pop_front();
  if (pairs_.size() < 3) return true;
  std::vector<TimePair> values(pairs_.begin(), pairs_.end());
  return robustAffineFit(values, 5000000.0, mapping_);
}

bool HostMonotonicMapper::toLocal(uint64_t host_monotonic_ns,
                                  uint64_t &local_ns,
                                  uint64_t &uncertainty_ns) const
{
  if (!mapping_.localToUtc(host_monotonic_ns, local_ns)) return false;
  uncertainty_ns = std::max<uint64_t>(1000000ULL, mapping_.time_uncertainty_ns);
  return true;
}

void HostMonotonicMapper::reset()
{
  pairs_.clear();
  mapping_ = AffineMapping{};
}

} // namespace sensor_time_bridge
