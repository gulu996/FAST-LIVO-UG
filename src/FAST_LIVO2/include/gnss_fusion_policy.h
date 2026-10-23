#pragma once

#include <ros/ros.h>

struct GnssFusionPolicy
{
  bool managed = false;
  bool enabled = false;

  bool componentEnabled(bool configured) const
  {
    return configured && (!managed || enabled);
  }

};

inline bool loadGnssFusionPolicy(ros::NodeHandle &nh,
                                 GnssFusionPolicy &policy)
{
  // Removed modes must fail explicitly, even when the backend is disabled.
  for (const char *key : {"gps/en", "gps/enable", "gps/update_en", "gps/update_enable", "gps/use_gps",
                          "uwb/en", "uwb/enable", "uwb/update_en", "uwb/update_enable", "uwb/output_correction_en",
                          "uwb/relocalization_en", "uwb/tag_offset_estimate_en",
                          "uwb/anchor_position_estimate_en", "uwb/baseline_anchor_init_en"}) {
    bool requested = false;
    if (nh.getParam(key, requested) && requested) {
      ROS_ERROR("Removed frontend fusion parameter %s=true. Use GNSS adapter and fixed-lag factors.", key);
      return false;
    }
  }
  std::string mode;
  if (nh.getParam("uwb/mode", mode) && !mode.empty()) {
    ROS_ERROR("uwb/mode selected removed frontend fusion. Remove it; configure fixed-lag factors instead.");
    return false;
  }
  ros::NodeHandle private_nh("~");
  private_nh.param<bool>("use_gnss_fusion_enable", policy.managed, false);
  if (!policy.managed) return true;
  if (nh.getParam("gnss_fusion/enable", policy.enabled)) return true;
  ROS_ERROR("[GNSS_FUSION] Missing required gnss_fusion/enable parameter.");
  return false;
}
