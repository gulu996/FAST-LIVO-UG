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

  bool absoluteUpdateEnabled(bool configured) const
  {
    return configured && !(managed && enabled);
  }
};

inline bool loadGnssFusionPolicy(ros::NodeHandle &nh,
                                 GnssFusionPolicy &policy)
{
  ros::NodeHandle private_nh("~");
  private_nh.param<bool>("use_gnss_fusion_enable", policy.managed, false);
  if (!policy.managed) return true;
  if (nh.getParam("gnss_fusion/enable", policy.enabled)) return true;
  ROS_ERROR("[GNSS_FUSION] Missing required gnss_fusion/enable parameter.");
  return false;
}
