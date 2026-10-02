#ifndef LANDMARK_COUNTERFACTUAL_H_
#define LANDMARK_COUNTERFACTUAL_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <cstdint>
#include <iosfwd>

namespace landmark
{
// Read-only event snapshot; optimizers run in the standalone offline probe.
struct CounterfactualSnapshot
{
  double timestamp = 0.0;
  std::uint64_t observation_id = 0, keypose_id = 0;
  int landmark_id = -1;
  gtsam::NonlinearFactorGraph existing, motion, visual;
  gtsam::Values existing_values, new_values, runtime_post;
};

void writeCounterfactualSnapshot(std::ostream &out, const CounterfactualSnapshot &snapshot);
CounterfactualSnapshot readCounterfactualSnapshot(std::istream &in);

struct CounterfactualResult
{
  gtsam::Values motion_only, motion_visual;
  double motion_initial_cost = 0, motion_final_cost = 0;
  double visual_initial_cost = 0, visual_final_cost = 0;
  std::size_t motion_iterations = 0, visual_iterations = 0;
};
CounterfactualResult optimizeCounterfactual(const CounterfactualSnapshot &snapshot,
                                          double motion_scale = 1.0,
                                          double visual_scale = 1.0);
// Same factor type/measurement, scaled covariance; gauge prior stays fixed.
gtsam::NonlinearFactorGraph scaledCounterfactualFactors(
    const gtsam::NonlinearFactorGraph &graph, double motion_scale, double visual_scale);
} // namespace landmark
#endif
