#include "preprocess.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>

namespace xyzirt_both_test
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double offset_time;
  double time;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xyzirt_both_test
POINT_CLOUD_REGISTER_POINT_STRUCT(
    xyzirt_both_test::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (double, offset_time, offset_time)(double, time, time)
    (std::uint16_t, ring, ring))

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

xyzirt_time_ros::Point makeTimePoint(float x, double time)
{
  xyzirt_time_ros::Point point;
  point.x = x;
  point.y = 1.0f;
  point.z = 2.0f;
  point.intensity = 42.0f;
  point.time = time;
  point.ring = 1;
  return point;
}

PointCloudXYZI::Ptr preprocessMessage(const sensor_msgs::PointCloud2 &message)
{
  Preprocess preprocess;
  preprocess.set(false, XYZIRT, 0.5, 1);
  preprocess.blind_sqr = preprocess.blind * preprocess.blind;
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  preprocess.process(
      boost::make_shared<const sensor_msgs::PointCloud2>(message), output);
  return output;
}
} // namespace

int main()
{
  ros::Time::init();

  // Historical offset_time-only layout remains byte-for-byte compatible.
  pcl::PointCloud<xyzirt_ros::Point> input;
  input.push_back(makePoint(3.0f, 0.1999754725));
  input.push_back(makePoint(1.0f, 0.00322424));
  input.push_back(makePoint(2.0f, 0.100));
  input.push_back(makePoint(std::numeric_limits<float>::quiet_NaN(), 0.0));

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(input, msg);
  PointCloudXYZI::Ptr output = preprocessMessage(msg);

  assert(output->size() == 3);
  assert(std::fabs(output->points[0].curvature - 3.22424) < 1e-4);
  assert(std::fabs(output->points[1].curvature - 100.0) < 1e-4);
  assert(std::fabs(output->points[2].curvature - 199.9754725) < 1e-3);
  assert(output->points[0].x == 1.0f);
  assert(output->points[2].x == 3.0f);

  // New dataset layout: FLOAT64 seconds relative to scan start.
  pcl::PointCloud<xyzirt_time_ros::Point> time_input;
  for (int i = 0; i < 5; ++i)
    time_input.push_back(makeTimePoint(static_cast<float>(i + 1), 0.05 * i));
  sensor_msgs::PointCloud2 time_msg;
  pcl::toROSMsg(time_input, time_msg);
  output = preprocessMessage(time_msg);
  assert(output->size() == 5);
  const double expected_ms[] = {0.0, 50.0, 100.0, 150.0, 200.0};
  std::size_t nonzero_count = 0;
  std::set<double> unique_times;
  for (std::size_t i = 0; i < output->size(); ++i)
  {
    assert(std::fabs(output->points[i].curvature - expected_ms[i]) < 1e-9);
    if (output->points[i].curvature > 0.0) ++nonzero_count;
    unique_times.insert(output->points[i].curvature);
  }
  assert(nonzero_count == 4 && unique_times.size() == 5);

  // Missing point time is rejected instead of silently becoming all-zero dt.
  pcl::PointCloud<pcl::PointXYZI> no_time_input;
  pcl::PointXYZI no_time_point;
  no_time_point.x = 1.0f;
  no_time_point.y = 1.0f;
  no_time_point.z = 1.0f;
  no_time_input.push_back(no_time_point);
  sensor_msgs::PointCloud2 no_time_msg;
  pcl::toROSMsg(no_time_input, no_time_msg);
  assert(preprocessMessage(no_time_msg)->empty());

  // If both fields exist, the historical offset_time field has fixed priority.
  pcl::PointCloud<xyzirt_both_test::Point> both_input;
  for (int i = 0; i < 2; ++i)
  {
    xyzirt_both_test::Point point;
    point.x = static_cast<float>(i + 1);
    point.y = 1.0f;
    point.z = 2.0f;
    point.intensity = 42.0f;
    point.offset_time = 0.01 * (i + 1);
    point.time = 0.10 * (i + 1);
    point.ring = 1;
    both_input.push_back(point);
  }
  sensor_msgs::PointCloud2 both_msg;
  pcl::toROSMsg(both_input, both_msg);
  output = preprocessMessage(both_msg);
  assert(output->size() == 2);
  assert(std::fabs(output->points[0].curvature - 10.0) < 1e-9);
  assert(std::fabs(output->points[1].curvature - 20.0) < 1e-9);

  std::cout << "preprocess_xyzirt_self_test: PASS\n";
  return 0;
}
