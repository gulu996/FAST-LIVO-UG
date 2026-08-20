#include "rtk_fixed_lag_backend.h"
#include "gnss_fusion_policy.h"
#include "run_log_directory.h"

#include <exception>

namespace {
bool configureRunLogDirectory(ros::NodeHandle &node) {
  ros::NodeHandle params(node, "rtk_backend");
  bool enabled = true;
  bool uwb_enabled = false;
  bool save_results = true;
  bool save_text_log = true;
  std::string output_directory;
  params.param("enable", enabled, enabled);
  params.param("uwb_factor_backend_en", uwb_enabled, uwb_enabled);
  GnssFusionPolicy fusion_policy;
  if (!loadGnssFusionPolicy(node, fusion_policy)) return false;
  enabled = enabled &&
            ((!fusion_policy.managed || fusion_policy.enabled) || uwb_enabled);
  params.param("save_results", save_results, save_results);
  params.param("save_text_log", save_text_log, save_text_log);
  params.param("output_directory", output_directory, output_directory);
  if (!enabled || (!save_results && !save_text_log) ||
      !output_directory.empty()) {
    return true;
  }

  ROS_INFO("[RTK_BACKEND_FILE] Waiting for the main run log directory.");
  std::string run_directory;
  const auto state = fast_livo_logging::waitForRunLogDirectory(
      node, run_directory);
  if (state == fast_livo_logging::RunLogDirectoryState::DISABLED) {
    params.setParam("save_results", false);
    params.setParam("save_text_log", false);
    ROS_INFO("[RTK_BACKEND_FILE] File output disabled by pcd_save/save_log_en.");
    return true;
  }
  if (state != fast_livo_logging::RunLogDirectoryState::READY) {
    ROS_ERROR("[RTK_BACKEND_FILE] Timed out waiting for %s.",
              fast_livo_logging::kRunLogDirectoryParam);
    return false;
  }

  params.setParam("output_directory", run_directory);
  ROS_INFO("[RTK_BACKEND_FILE] Using main run directory: %s",
           run_directory.c_str());
  return true;
}
}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "rtk_fixed_lag_backend");
  ros::NodeHandle node;
  if (!configureRunLogDirectory(node)) return 1;
  try {
    fast_livo_backend::RtkFixedLagBackend backend(node);
    if (!backend.enabled()) {
      ros::spin();
      return 0;
    }
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM("[RTK_BACKEND] startup failed: " << error.what());
    return 1;
  }
  return 0;
}
