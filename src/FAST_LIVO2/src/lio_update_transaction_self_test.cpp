#include "lio_update_transaction.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace
{

fast_livo::LioCandidateMetrics healthyMetrics(bool converged,
                                              bool iteration_limit)
{
  fast_livo::LioCandidateMetrics metrics;
  metrics.posterior_ready = true;
  metrics.converged = converged;
  metrics.reached_iteration_limit = iteration_limit;
  metrics.correspondence_count = 100;
  metrics.valid_residual_count = 100;
  metrics.cost_before = 0.10;
  metrics.cost_after = 0.08;
  metrics.maximum_abs_residual = 0.20;
  metrics.translation_increment_norm = 0.10;
  metrics.rotation_increment_deg = 0.20;
  metrics.velocity_increment_norm = 0.02;
  metrics.state_increment_finite = true;
  return metrics;
}

bool sameState(const StatesGroup &a, const StatesGroup &b)
{
  return a.rot_end.isApprox(b.rot_end, 0.0) &&
         a.pos_end.isApprox(b.pos_end, 0.0) &&
         a.vel_end.isApprox(b.vel_end, 0.0) &&
         a.bias_g.isApprox(b.bias_g, 0.0) &&
         a.bias_a.isApprox(b.bias_a, 0.0) &&
         a.gravity.isApprox(b.gravity, 0.0) &&
         a.inv_expo_time == b.inv_expo_time &&
         a.cov.isApprox(b.cov, 0.0);
}

}  // namespace

int main()
{
  fast_livo::LioValidationLimits limits;
  limits.minimum_correspondences = 50;
  limits.maximum_translation_increment = 1.0;
  limits.maximum_rotation_increment_deg = 6.0;

  StatesGroup prior;
  prior.pos_end = V3D(1.0, 2.0, 3.0);
  prior.cov = MD(DIM_STATE, DIM_STATE)::Identity() * 0.02;
  StatesGroup candidate = prior;
  candidate.pos_end.x() += 0.1;
  candidate.cov *= 0.5;

  const auto converged = fast_livo::validateLioCandidate(
      candidate, healthyMetrics(true, false), limits);
  assert(converged.commit);
  assert(converged.status == fast_livo::LioCommitStatus::CONVERGED);

  const auto max_iter_healthy = fast_livo::validateLioCandidate(
      candidate, healthyMetrics(false, true), limits);
  assert(max_iter_healthy.commit);
  assert(max_iter_healthy.status ==
         fast_livo::LioCommitStatus::MAX_ITER_HEALTHY);

  const auto not_converged_healthy = fast_livo::validateLioCandidate(
      candidate, healthyMetrics(false, false), limits);
  assert(not_converged_healthy.commit);
  assert(not_converged_healthy.status ==
         fast_livo::LioCommitStatus::NOT_CONVERGED_HEALTHY);

  StatesGroup non_finite = candidate;
  non_finite.pos_end.x() = std::numeric_limits<double>::quiet_NaN();
  const auto nan_result = fast_livo::validateLioCandidate(
      non_finite, healthyMetrics(false, true), limits);
  assert(!nan_result.commit);
  assert(nan_result.status ==
         fast_livo::LioCommitStatus::REJECTED_NUMERICAL);

  StatesGroup infinite = candidate;
  infinite.vel_end.y() = std::numeric_limits<double>::infinity();
  const auto inf_result = fast_livo::validateLioCandidate(
      infinite, healthyMetrics(false, true), limits);
  assert(!inf_result.commit);
  assert(inf_result.status ==
         fast_livo::LioCommitStatus::REJECTED_NUMERICAL);

  auto bad_cost_metrics = healthyMetrics(false, true);
  bad_cost_metrics.cost_after = 1.0;
  const auto bad_cost = fast_livo::validateLioCandidate(
      candidate, bad_cost_metrics, limits);
  assert(!bad_cost.commit);
  assert(bad_cost.status == fast_livo::LioCommitStatus::REJECTED_COST);

  auto ratio_bad_metrics = healthyMetrics(false, true);
  ratio_bad_metrics.cost_after = 0.25;
  const auto ratio_bad = fast_livo::validateLioCandidate(
      candidate, ratio_bad_metrics, limits);
  assert(!ratio_bad.commit);
  assert(ratio_bad.status == fast_livo::LioCommitStatus::REJECTED_COST);

  const StatesGroup rolled_back = fast_livo::committedLioState(
      prior, candidate, bad_cost);
  assert(sameState(rolled_back, prior));
  assert(rolled_back.cov.isApprox(prior.cov, 0.0));
  assert(!rolled_back.cov.isApprox(candidate.cov, 0.0));

  assert(!fast_livo::shouldInsertLioMap(false, true, false));
  assert(!fast_livo::shouldInsertLioMap(true, true, true));
  assert(fast_livo::shouldInsertLioMap(true, true, false));

  auto invalid_metrics = healthyMetrics(false, true);
  invalid_metrics.posterior_ready = false;
  const auto invalid = fast_livo::validateLioCandidate(
      candidate, invalid_metrics, limits);
  assert(!invalid.commit);
  assert(invalid.status == fast_livo::LioCommitStatus::REJECTED_INVALID);

  StatesGroup indefinite = candidate;
  indefinite.cov(0, 0) = -1.0;
  const auto indefinite_result = fast_livo::validateLioCandidate(
      indefinite, healthyMetrics(false, true), limits);
  assert(!indefinite_result.commit);
  assert(indefinite_result.status ==
         fast_livo::LioCommitStatus::REJECTED_NUMERICAL);

  std::cout << "lio_update_transaction_self_test: PASS\n";
  return 0;
}
