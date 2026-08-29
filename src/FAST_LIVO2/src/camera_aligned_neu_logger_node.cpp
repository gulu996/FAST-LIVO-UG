#include <ros/ros.h>

namespace fast_livo_camera_neu {
void runCameraAlignedNeuLogger(ros::NodeHandle &node);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_aligned_neu_logger");
  ros::NodeHandle node;
  fast_livo_camera_neu::runCameraAlignedNeuLogger(node);
  return 0;
}
