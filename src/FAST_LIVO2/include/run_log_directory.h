#pragma once

#include <ros/ros.h>

#include <filesystem>
#include <string>

namespace fast_livo_logging
{
// ponytail: one active FAST-LIVO run per ROS master; namespace this protocol
// if concurrent mapping runs are required later.
constexpr char kRunLogDirectoryParam[] = "/fast_livo/run_log_directory";
constexpr char kRunLogDirectoryPending[] = "__PENDING__";
constexpr char kRunLogDirectoryDisabled[] = "__DISABLED__";
constexpr double kRunLogDirectoryWaitSeconds = 30.0;

enum class RunLogDirectoryState
{
  PENDING,
  READY,
  DISABLED
};

inline RunLogDirectoryState classifyRunLogDirectory(
    const std::string &value, std::string *directory = nullptr)
{
  if (directory) directory->clear();
  if (value.empty() || value == kRunLogDirectoryPending)
  {
    return RunLogDirectoryState::PENDING;
  }
  if (value == kRunLogDirectoryDisabled)
  {
    return RunLogDirectoryState::DISABLED;
  }
  if (directory) *directory = value;
  return RunLogDirectoryState::READY;
}

inline RunLogDirectoryState waitForRunLogDirectory(
    ros::NodeHandle &nh, std::string &directory,
    double timeout_seconds = kRunLogDirectoryWaitSeconds)
{
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(timeout_seconds);
  do
  {
    std::string value;
    if (nh.getParam(kRunLogDirectoryParam, value))
    {
      const RunLogDirectoryState state =
          classifyRunLogDirectory(value, &directory);
      if (state != RunLogDirectoryState::PENDING) return state;
    }
    ros::WallDuration(0.02).sleep();
  } while (ros::ok() && ros::WallTime::now() < deadline);

  directory.clear();
  return RunLogDirectoryState::PENDING;
}

inline std::string resolveRunLogFile(const std::string &configured_path,
                                     const std::string &run_directory)
{
  const std::filesystem::path path(configured_path);
  if (path.is_absolute()) return path.lexically_normal().string();
  return (std::filesystem::path(run_directory) / path)
      .lexically_normal()
      .string();
}
}  // namespace fast_livo_logging
