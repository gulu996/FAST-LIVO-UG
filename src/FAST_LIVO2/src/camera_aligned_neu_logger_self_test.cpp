#include "camera_aligned_neu_logger.h"

#include <cassert>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>

namespace {

bool near(double lhs, double rhs, double tolerance = 1e-9) {
  return std::fabs(lhs - rhs) <= tolerance;
}

void requireVector(const Eigen::Vector3d &actual,
                   const Eigen::Vector3d &expected,
                   double tolerance = 1e-9) {
  assert((actual - expected).norm() <= tolerance);
}

}  // namespace

int main() {
  using fast_livo_camera_neu::TimedPose;

  TimedPose before;
  before.stamp_ns = 0;
  TimedPose after;
  after.stamp_ns = 1000000000LL;
  after.position = Eigen::Vector3d(10.0, 20.0, 30.0);
  after.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  TimedPose middle;
  double span_s = 0.0;
  assert(fast_livo_camera_neu::interpolatePose(
      before, after, 500000000LL, 1.0, middle, span_s));
  requireVector(middle.position, Eigen::Vector3d(5.0, 10.0, 15.0));
  assert(near(middle.orientation.norm(), 1.0));
  requireVector(middle.orientation * Eigen::Vector3d::UnitX(),
                Eigen::Vector3d::UnitY(), 1e-8);

  requireVector(fast_livo_camera_neu::enuToNeu(
                    Eigen::Vector3d(100.0, 200.0, 5.0)),
                Eigen::Vector3d(200.0, 100.0, 5.0));

  TimedPose map_odom;
  map_odom.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  map_odom.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  TimedPose odom_body;
  odom_body.position = Eigen::Vector3d(2.0, 0.0, 0.0);
  const TimedPose map_body = fast_livo_camera_neu::composePoses(
      map_odom, odom_body, 15);
  requireVector(map_body.position, Eigen::Vector3d(10.0, 2.0, 0.0), 1e-8);

  TimedPose body_camera;
  body_camera.position = Eigen::Vector3d(0.5, 0.0, 0.0);
  body_camera.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
  const TimedPose map_camera = fast_livo_camera_neu::composePoses(
      map_body, body_camera, 15);
  requireVector(map_camera.position, Eigen::Vector3d(10.0, 2.5, 0.0), 1e-8);
  assert(near(map_camera.orientation.norm(), 1.0));

  std::deque<TimedPose> corrections(2);
  corrections[0].stamp_ns = 10000000000LL;
  corrections[1].stamp_ns = 20000000000LL;
  const TimedPose *causal =
      fast_livo_camera_neu::latestCausalPose(corrections, 15000000000LL);
  assert(causal != nullptr && causal->stamp_ns == 10000000000LL);

  TimedPose rejected;
  assert(!fast_livo_camera_neu::interpolatePose(
      before, after, 500000000LL, 0.25, rejected, span_s));
  assert(span_s > 0.25);
  assert(fast_livo_camera_neu::latestCausalPose(
             std::deque<TimedPose>(), 15000000000LL) == nullptr);
  const double invalid_position = std::numeric_limits<double>::quiet_NaN();
  assert(std::isnan(invalid_position));

  assert(fast_livo_camera_neu::formatStamp(1785900584024918556LL) ==
         "1785900584.024918556");
  std::cout << "camera_aligned_neu_logger_self_test: PASS\n";
  return 0;
}
