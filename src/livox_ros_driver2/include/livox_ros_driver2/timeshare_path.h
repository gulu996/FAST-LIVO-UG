#ifndef LIVOX_ROS_DRIVER2_TIMESHARE_PATH_H_
#define LIVOX_ROS_DRIVER2_TIMESHARE_PATH_H_

#include <cstdlib>
#include <pwd.h>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <unistd.h>

namespace livox_ros {

inline std::string RequireAbsoluteTimesharePath(const std::string& path) {
  if (path.empty() || path.front() != '/') {
    throw std::invalid_argument(
        "timeshare_path must be absolute; '~' and relative paths are not "
        "supported: " +
        path);
  }
  return path;
}

inline std::string ResolveTimesharePathFromInputs(
    const std::string& configured_path, const char* environment_home,
    const char* passwd_home) {
  if (!configured_path.empty()) {
    return RequireAbsoluteTimesharePath(configured_path);
  }

  const char* selected_home =
      environment_home != nullptr && environment_home[0] != '\0'
          ? environment_home
          : passwd_home;
  if (selected_home == nullptr || selected_home[0] == '\0') {
    throw std::runtime_error(
        "Unable to resolve current user's home directory");
  }

  std::string home = RequireAbsoluteTimesharePath(selected_home);
  while (home.size() > 1 && home.back() == '/') {
    home.pop_back();
  }
  return home == "/" ? "/timeshare" : home + "/timeshare";
}

inline std::string ResolveTimesharePath(
    const std::string& configured_path) {
  if (!configured_path.empty()) {
    return ResolveTimesharePathFromInputs(
        configured_path, nullptr, nullptr);
  }

  const char* environment_home = std::getenv("HOME");
  const char* passwd_home = nullptr;
  if (environment_home == nullptr || environment_home[0] == '\0') {
    const struct passwd* password_entry = getpwuid(geteuid());
    if (password_entry != nullptr) {
      passwd_home = password_entry->pw_dir;
    }
  }
  return ResolveTimesharePathFromInputs(
      configured_path, environment_home, passwd_home);
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_TIMESHARE_PATH_H_
