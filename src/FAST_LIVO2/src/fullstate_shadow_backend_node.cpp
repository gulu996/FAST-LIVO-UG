#include "fullstate_shadow_backend.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "fullstate_shadow_backend");
  ros::NodeHandle node;
  try {
    fast_livo_shadow::FullStateShadowBackend backend(node);
    ros::spin();
    return 0;
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM("[FULLSTATE_SHADOW] startup failed: " << error.what());
    return 1;
  }
}
