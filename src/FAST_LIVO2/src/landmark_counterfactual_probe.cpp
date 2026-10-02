#include "landmark_counterfactual.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Row = std::vector<std::pair<std::string, double>>;
void add(Row &r, const std::string &name, double value)
{
  if (!std::isfinite(value)) throw std::runtime_error("nonfinite metric: " + name);
  r.emplace_back(name, value);
}
void delta(Row &r, const std::string &name, const gtsam::Pose3 &a, const gtsam::Pose3 &b)
{
  const auto t = b.translation() - a.translation();
  add(r, name + "_translation_m", t.norm());
  add(r, name + "_rotation_deg", gtsam::Rot3::Logmap(a.rotation().between(b.rotation())).norm() * 180 / M_PI);
  for (int i = 0; i < 3; ++i) add(r, name + "_world_d" + std::to_string(i), t(i));
}
void pose(Row &r, const std::string &name, const gtsam::Pose3 &p)
{
  const auto q = p.rotation().toQuaternion();
  for (int i = 0; i < 3; ++i) add(r, name + "_t" + std::to_string(i), p.translation()(i));
  add(r, name + "_qx", q.x()); add(r, name + "_qy", q.y());
  add(r, name + "_qz", q.z()); add(r, name + "_qw", q.w());
}
void residual(Row &r, const std::string &name, const gtsam::NonlinearFactorGraph &graph,
              const gtsam::Values &values)
{
  if (graph.size() != 1) throw std::runtime_error("expected one return factor");
  const auto *f = dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3> *>(graph[0].get());
  if (!f) throw std::runtime_error("expected Pose3 BetweenFactor");
  const auto u = f->unwhitenedError(values), w = f->whitenedError(values);
  const auto log = gtsam::Pose3::Logmap(f->measured().between(
      values.at<gtsam::Pose3>(f->key1()).between(values.at<gtsam::Pose3>(f->key2()))));
  if ((log - u).norm() > 1e-8 || std::abs(f->error(values) - 0.5 * w.squaredNorm()) >
      1e-8 * (1 + f->error(values))) throw std::runtime_error("residual chart/error mismatch");
  add(r, name + "_rotation_deg", u.head<3>().norm() * 180 / M_PI);
  add(r, name + "_translation_m", u.tail<3>().norm());
  add(r, name + "_whitened_norm", w.norm());
  add(r, name + "_error", f->error(values));
  for (int i = 0; i < 6; ++i)
  {
    add(r, name + "_unwhitened_" + std::to_string(i), u(i));
    add(r, name + "_whitened_" + std::to_string(i), w(i));
  }
}
Row analyze(const landmark::CounterfactualSnapshot &s, double ms, double vs)
{
  const auto result = landmark::optimizeCounterfactual(s, ms, vs);
  const auto old = landmark::scaledCounterfactualFactors(s.existing, ms, vs);
  const auto motion = landmark::scaledCounterfactualFactors(s.motion, ms, vs);
  const auto visual = landmark::scaledCounterfactualFactors(s.visual, ms, vs);
  gtsam::Values raw = s.existing_values;
  raw.insert(s.new_values);
  const auto x = gtsam::Symbol('x', s.keypose_id), l = gtsam::Symbol('l', s.landmark_id);
  const auto &a = raw.at<gtsam::Pose3>(x), &b = result.motion_only.at<gtsam::Pose3>(x),
             &c = result.motion_visual.at<gtsam::Pose3>(x), &runtime = s.runtime_post.at<gtsam::Pose3>(x);
  const auto &la = raw.at<gtsam::Pose3>(l), &lb = result.motion_only.at<gtsam::Pose3>(l),
             &lc = result.motion_visual.at<gtsam::Pose3>(l), &lr = s.runtime_post.at<gtsam::Pose3>(l);
  Row r;
  add(r, "timestamp", s.timestamp); add(r, "observation_id", s.observation_id);
  add(r, "keypose_id", s.keypose_id); add(r, "landmark_id", s.landmark_id);
  add(r, "motion_covariance_scale", ms); add(r, "visual_covariance_scale", vs);
  add(r, "existing_factors", old.size()); add(r, "existing_variables", s.existing_values.size());
  delta(r, "raw_to_B", a, b); delta(r, "raw_to_C", a, c); delta(r, "B_to_C", b, c);
  delta(r, "raw_to_runtime_C", a, runtime); delta(r, "batch_C_to_runtime_C", c, runtime);
  delta(r, "L_pre_to_B", la, lb); delta(r, "L_pre_to_C", la, lc);
  delta(r, "L_B_to_C", lb, lc); delta(r, "L_batch_C_to_runtime_C", lc, lr);
  for (const auto &v : std::vector<std::pair<std::string, const gtsam::Values *>>{
           {"RAW", &raw}, {"B", &result.motion_only}, {"C", &result.motion_visual}, {"RUNTIME_C", &s.runtime_post}})
  {
    residual(r, "visual_" + v.first, visual, *v.second);
    residual(r, "motion_" + v.first, motion, *v.second);
    add(r, "common_old_cost_" + v.first, old.error(*v.second));
  }
  add(r, "B_initial_cost", result.motion_initial_cost); add(r, "B_final_cost", result.motion_final_cost);
  add(r, "C_initial_cost", result.visual_initial_cost); add(r, "C_final_cost", result.visual_final_cost);
  add(r, "B_iterations", result.motion_iterations); add(r, "C_iterations", result.visual_iterations);
  pose(r, "K_RAW", a); pose(r, "K_B", b); pose(r, "K_C", c); pose(r, "K_RUNTIME_C", runtime);
  pose(r, "L_PRE", la); pose(r, "L_B", lb); pose(r, "L_C", lc); pose(r, "L_RUNTIME_C", lr);
  return r;
}
} // namespace

int main(int argc, char **argv)
{
  try
  {
    if (argc != 3) throw std::runtime_error("usage: landmark_counterfactual_probe event.snapshot output.csv");
    std::ifstream input(argv[1]);
    const auto snapshot = landmark::readCounterfactualSnapshot(input);
    if (snapshot.new_values.size() != 1 || snapshot.motion.size() != 1 ||
        !snapshot.new_values.exists(gtsam::Symbol('x', snapshot.keypose_id)))
      throw std::runtime_error("probe requires a new return keypose with one normal motion factor");
    std::ifstream existing(argv[2]);
    if (existing.good()) throw std::runtime_error("refusing to overwrite existing output");
    std::ofstream output(argv[2]);
    output << std::setprecision(17);
    const auto baseline = analyze(snapshot, 1, 1);
    for (std::size_t i = 0; i < baseline.size(); ++i) output << (i ? "," : "") << baseline[i].first;
    output << '\n';
    const auto write = [&](const Row &row) {
      if (row.size() != baseline.size()) throw std::runtime_error("CSV schema mismatch");
      for (std::size_t i = 0; i < row.size(); ++i) output << (i ? "," : "") << row[i].second;
      output << '\n';
    };
    write(baseline); // default first; each subsequent row scales one family only
    for (double scale : {0.5, 2.0, 5.0, 10.0}) write(analyze(snapshot, 1, scale));
    for (double scale : {0.5, 2.0, 5.0, 10.0}) write(analyze(snapshot, scale, 1));
    output.close();
    if (!output) throw std::runtime_error("counterfactual CSV write failed");
    for (const auto &entry : baseline)
      if (entry.first.find("_translation_m") != std::string::npos ||
          entry.first.find("_rotation_deg") != std::string::npos)
        std::cout << entry.first << '=' << std::setprecision(12) << entry.second << '\n';
    std::cout << "OFFLINE_COUNTERFACTUAL_CASES=9\nCOUNTERFACTUAL_PROBE=PASS\n";
    return 0;
  }
  catch (const std::exception &e) { std::cerr << "COUNTERFACTUAL_PROBE=FAIL " << e.what() << '\n'; return 1; }
}
