#ifndef FAST_LIVO_CAMERA_ALIGNED_NEU_LOGGER_H
#define FAST_LIVO_CAMERA_ALIGNED_NEU_LOGGER_H

#include <Eigen/Geometry>

#include <cstdint>
#include <deque>
#include <string>

namespace fast_livo_camera_neu {

struct TimedPose {
  std::int64_t stamp_ns = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

bool interpolatePose(const TimedPose &before, const TimedPose &after,
                     std::int64_t query_stamp_ns, double max_gap_s,
                     
                     TimedPose &interpolated, double &bracket_span_s);

const TimedPose *latestCausalPose(const std::deque<TimedPose> &poses,
                                  std::int64_t query_stamp_ns);

TimedPose composePoses(const TimedPose &parent_to_child,
                       const TimedPose &child_to_body,
                       std::int64_t stamp_ns);

Eigen::Vector3d enuToNeu(const Eigen::Vector3d &enu);

std::string formatStamp(std::int64_t stamp_ns);

}  // namespace fast_livo_camera_neu

#endif
