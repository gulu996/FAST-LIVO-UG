#include "landmark_counterfactual.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iomanip>
#include <istream>
#include <ostream>
#include <stdexcept>

namespace landmark
{
namespace
{
using Between = gtsam::BetweenFactor<gtsam::Pose3>;
using Prior = gtsam::PriorFactor<gtsam::Pose3>;
void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}
void tag(std::istream &in, const char *expected)
{
  std::string value;
  require(bool(in >> value) && value == expected, "invalid snapshot section");
}
std::size_t count(std::istream &in)
{
  std::size_t n;
  require(bool(in >> n) && n <= 100000, "invalid snapshot count");
  return n;
}
double scalar(std::istream &in)
{
  double value;
  require(bool(in >> value) && std::isfinite(value), "invalid snapshot scalar");
  return value;
}
void writePose(std::ostream &out, const gtsam::Pose3 &pose)
{
  const auto R = pose.rotation().matrix();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) out << ' ' << R(r, c);
  for (int i = 0; i < 3; ++i) out << ' ' << pose.translation()(i);
}
gtsam::Pose3 readPose(std::istream &in)
{
  gtsam::Matrix3 R;
  gtsam::Vector3 t;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) R(r, c) = scalar(in);
  for (int i = 0; i < 3; ++i) t(i) = scalar(in);
  // ponytail: preserve the rounded real calibration (observed SO(3) error
  // 2.8e-6), never project/normalize it. Reject grossly invalid matrices.
  require((R.transpose() * R - gtsam::Matrix3::Identity()).norm() < 1e-4 &&
              std::abs(R.determinant() - 1.0) < 1e-4, "invalid snapshot rotation");
  return gtsam::Pose3(gtsam::Rot3(R), t);
}
gtsam::Matrix covariance(const gtsam::NoiseModelFactor &factor)
{
  const auto gaussian = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(factor.noiseModel());
  require(bool(gaussian) && gaussian->dim() == 6, "unsupported snapshot noise model");
  return gaussian->covariance();
}
void writeValues(std::ostream &out, const char *name, const gtsam::Values &values)
{
  out << name << ' ' << values.size() << '\n';
  for (auto key : values.keys())
  {
    out << key;
    writePose(out, values.at<gtsam::Pose3>(key));
    out << '\n';
  }
}
gtsam::Values readValues(std::istream &in, const char *name)
{
  tag(in, name);
  const auto n = count(in);
  gtsam::Values values;
  for (std::size_t i = 0; i < n; ++i)
  {
    gtsam::Key key;
    require(bool(in >> key), "invalid snapshot key");
    const char family = gtsam::Symbol(key).chr();
    require(family == 'x' || family == 'l', "unsupported snapshot variable");
    values.insert(key, readPose(in));
  }
  return values;
}
void writeGraph(std::ostream &out, const char *name, const gtsam::NonlinearFactorGraph &graph)
{
  out << name << ' ' << graph.size() << '\n';
  for (const auto &factor : graph)
  {
    if (const auto *between = dynamic_cast<const Between *>(factor.get()))
    {
      out << "B " << between->key1() << ' ' << between->key2();
      writePose(out, between->measured());
    }
    else if (const auto *prior = dynamic_cast<const Prior *>(factor.get()))
    {
      out << "P " << prior->key();
      writePose(out, prior->prior());
    }
    else throw std::runtime_error("unsupported snapshot factor");
    const auto C = covariance(*dynamic_cast<const gtsam::NoiseModelFactor *>(factor.get()));
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c) out << ' ' << C(r, c);
    out << '\n';
  }
}
gtsam::NonlinearFactorGraph readGraph(std::istream &in, const char *name)
{
  tag(in, name);
  const auto n = count(in);
  gtsam::NonlinearFactorGraph graph;
  for (std::size_t i = 0; i < n; ++i)
  {
    char kind;
    gtsam::Key k1, k2 = 0;
    require(bool(in >> kind >> k1) && (kind == 'P' || kind == 'B'), "invalid snapshot factor");
    if (kind == 'B') require(bool(in >> k2), "invalid snapshot second key");
    const auto pose = readPose(in);
    gtsam::Matrix6 C;
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c) C(r, c) = scalar(in);
    const Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eig(C);
    require((C - C.transpose()).norm() < 1e-8 && eig.info() == Eigen::Success &&
                eig.eigenvalues().minCoeff() > 0, "invalid snapshot covariance");
    const auto noise = gtsam::noiseModel::Gaussian::Covariance(C);
    if (kind == 'B') graph.add(Between(k1, k2, pose, noise));
    else graph.add(Prior(k1, pose, noise));
  }
  return graph;
}
} // namespace

void writeCounterfactualSnapshot(std::ostream &out, const CounterfactualSnapshot &s)
{
  out << std::setprecision(17) << "LANDMARK_COUNTERFACTUAL_V1\n"
      << s.timestamp << ' ' << s.observation_id << ' ' << s.keypose_id << ' ' << s.landmark_id << '\n';
  writeValues(out, "EXISTING_VALUES", s.existing_values);
  writeValues(out, "NEW_VALUES", s.new_values);
  writeGraph(out, "EXISTING_FACTORS", s.existing);
  writeGraph(out, "MOTION_FACTORS", s.motion);
  writeGraph(out, "VISUAL_FACTORS", s.visual);
  writeValues(out, "RUNTIME_POST", s.runtime_post);
  require(bool(out), "snapshot write failed");
}

CounterfactualSnapshot readCounterfactualSnapshot(std::istream &in)
{
  tag(in, "LANDMARK_COUNTERFACTUAL_V1");
  CounterfactualSnapshot s;
  s.timestamp = scalar(in);
  require(bool(in >> s.observation_id >> s.keypose_id >> s.landmark_id) &&
              s.observation_id > 0 && s.keypose_id > 0 && s.landmark_id >= 0,
          "invalid snapshot metadata");
  s.existing_values = readValues(in, "EXISTING_VALUES");
  s.new_values = readValues(in, "NEW_VALUES");
  s.existing = readGraph(in, "EXISTING_FACTORS");
  s.motion = readGraph(in, "MOTION_FACTORS");
  s.visual = readGraph(in, "VISUAL_FACTORS");
  s.runtime_post = readValues(in, "RUNTIME_POST");
  in >> std::ws;
  require(in.eof(), "trailing snapshot data");
  gtsam::Values initial = s.existing_values;
  initial.insert(s.new_values);
  for (const auto *graph : {&s.existing, &s.motion, &s.visual})
    require(std::isfinite(graph->error(initial)) && std::isfinite(graph->error(s.runtime_post)),
            "invalid snapshot graph values");
  require(s.visual.size() == 1, "snapshot must have exactly one candidate visual factor");
  const auto *visual = dynamic_cast<const Between *>(s.visual[0].get());
  require(visual && visual->key1() == gtsam::Symbol('x', s.keypose_id) &&
              visual->key2() == gtsam::Symbol('l', s.landmark_id), "snapshot event key mismatch");
  return s;
}

gtsam::NonlinearFactorGraph scaledCounterfactualFactors(
    const gtsam::NonlinearFactorGraph &graph, double motion_scale, double visual_scale)
{
  require(std::isfinite(motion_scale) && motion_scale > 0 &&
              std::isfinite(visual_scale) && visual_scale > 0, "invalid covariance scale");
  gtsam::NonlinearFactorGraph result;
  for (const auto &factor : graph)
  {
    if (dynamic_cast<const Prior *>(factor.get())) { result.add(factor); continue; }
    const auto *between = dynamic_cast<const Between *>(factor.get());
    require(between != nullptr, "unsupported counterfactual factor");
    const char a = gtsam::Symbol(between->key1()).chr(), b = gtsam::Symbol(between->key2()).chr();
    require(a == 'x' && (b == 'x' || b == 'l'), "unsupported counterfactual factor family");
    const double scale = b == 'x' ? motion_scale : visual_scale;
    if (scale == 1.0) result.add(factor);
    else result.add(Between(between->key1(), between->key2(), between->measured(),
        gtsam::noiseModel::Gaussian::Covariance(covariance(*between) * scale)));
  }
  return result;
}

CounterfactualResult optimizeCounterfactual(const CounterfactualSnapshot &s,
                                          double motion_scale, double visual_scale)
{
  gtsam::Values initial = s.existing_values;
  initial.insert(s.new_values);
  auto B = scaledCounterfactualFactors(s.existing, motion_scale, visual_scale);
  B.push_back(scaledCounterfactualFactors(s.motion, motion_scale, visual_scale));
  auto C = B;
  C.push_back(scaledCounterfactualFactors(s.visual, motion_scale, visual_scale));
  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(100);
  params.setRelativeErrorTol(1e-9);
  params.setAbsoluteErrorTol(1e-9);
  params.setLinearSolverType("MULTIFRONTAL_QR");
  gtsam::LevenbergMarquardtOptimizer b(B, initial, params), c(C, initial, params);
  CounterfactualResult result;
  result.motion_initial_cost = B.error(initial);
  result.visual_initial_cost = C.error(initial);
  result.motion_only = b.optimize();
  result.motion_visual = c.optimize();
  result.motion_final_cost = B.error(result.motion_only);
  result.visual_final_cost = C.error(result.motion_visual);
  result.motion_iterations = b.iterations();
  result.visual_iterations = c.iterations();
  require(std::isfinite(result.motion_initial_cost) && std::isfinite(result.visual_initial_cost) &&
              std::isfinite(result.motion_final_cost) && std::isfinite(result.visual_final_cost) &&
              result.motion_final_cost <= result.motion_initial_cost + 1e-7 &&
              result.visual_final_cost <= result.visual_initial_cost + 1e-7,
          "nonfinite or increasing counterfactual cost");
  for (const auto *values : {&result.motion_only, &result.motion_visual})
    for (auto key : values->keys())
      require(values->at<gtsam::Pose3>(key).matrix().allFinite(), "nonfinite optimized pose");
  return result;
}
} // namespace landmark
