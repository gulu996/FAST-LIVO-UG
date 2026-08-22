#include "preprocess.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace
{
xyzirt_ros::Point makePoint(float x, double offset_time)
{
  xyzirt_ros::Point point;
  point.x = x;
  point.y = 1.0f;
  point.z = 2.0f;
  point.intensity = 42.0f;
  point.offset_time = offset_time;
  point.ring = 1;
  return point;
}
} // namespace

int main()
{
  pcl::PointCloud<xyzirt_ros::Point> input;
  input.push_back(makePoint(3.0f, 0.1999754725));
  input.push_back(makePoint(1.0f, 0.00322424));
  input.push_back(makePoint(2.0f, 0.100));
  input.push_back(makePoint(std::numeric_limits<float>::quiet_NaN(), 0.0));

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(input, msg);

  Preprocess preprocess;
  preprocess.set(false, XYZIRT, 0.5, 1);
  preprocess.blind_sqr = preprocess.blind * preprocess.blind;
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  preprocess.process(boost::make_shared<const sensor_msgs::PointCloud2>(msg), output);

  assert(output->size() == 3);
  assert(std::fabs(output->points[0].curvature - 3.22424) < 1e-4);
  assert(std::fabs(output->points[1].curvature - 100.0) < 1e-4);
  assert(std::fabs(output->points[2].curvature - 199.9754725) < 1e-3);
  assert(output->points[0].x == 1.0f);
  assert(output->points[2].x == 3.0f);

  std::cout << "preprocess_xyzirt_self_test: PASS\n";
  return 0;
}
