#include <gnss_serial_driver/gnss_math.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <gnss_serial_driver/GnssPvtStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class GnssAdapterNode
{
public:
  GnssAdapterNode()
      : private_node_("~")
  {
    private_node_.param<std::string>("frame_id", frame_id_, "gnss_enu");
    private_node_.param<std::string>("child_frame_id", child_frame_id_, "gnss");
    private_node_.param<std::string>("origin_mode", origin_mode_, "first_valid");

    if (origin_mode_ == "manual")
    {
      std::vector<double> values;
      if (!private_node_.getParam("origin_lla", values) || values.size() != 3)
      {
        throw std::invalid_argument(
            "manual origin_lla must be [lat, lon, altitude]");
      }
      origin_lla_ = Eigen::Vector3d(values[0], values[1], values[2]);
      origin_ecef_ = gnss_serial_driver::geodeticToEcef(origin_lla_);
      if (!origin_ecef_.allFinite())
      {
        throw std::invalid_argument("manual origin_lla is invalid");
      }
      origin_initialized_ = true;
    }
    else if (origin_mode_ != "first_valid")
    {
      throw std::invalid_argument("origin_mode must be first_valid or manual");
    }

    publisher_ =
        node_.advertise<nav_msgs::Odometry>("/gnss/enu_odom", 20);
    subscriber_ = node_.subscribe(
        "/gnss/pvt_local", 20, &GnssAdapterNode::callback, this);
  }

private:
  void callback(
      const gnss_serial_driver::GnssPvtStampedConstPtr &message)
  {
    if (!message->local_measurement_time_valid ||
        !message->valid_for_fusion)
    {
      return;
    }

    const gnss_comm::GnssPVTSolnMsg &pvt = message->pvt;
    const Eigen::Vector3d current_lla(
        pvt.latitude, pvt.longitude, pvt.altitude);
    const Eigen::Vector3d current_ecef =
        gnss_serial_driver::geodeticToEcef(current_lla);
    if (!current_ecef.allFinite())
    {
      ROS_WARN_THROTTLE(5.0, "GNSS adapter rejected invalid WGS84 position");
      return;
    }

    if (!origin_initialized_)
    {
      origin_lla_ = current_lla;
      origin_ecef_ = current_ecef;
      origin_initialized_ = true;
      ROS_INFO("GNSS ENU origin initialized from first fusion-valid LOCAL PVT");
    }

    const Eigen::Vector3d enu = gnss_serial_driver::ecefDeltaToEnu(
        origin_lla_, current_ecef - origin_ecef_);
    if (!enu.allFinite()) return;

    nav_msgs::Odometry odometry;
    odometry.header = message->header;
    odometry.header.frame_id = frame_id_;
    odometry.child_frame_id = child_frame_id_;
    odometry.pose.pose.position.x = enu.x();
    odometry.pose.pose.position.y = enu.y();
    odometry.pose.pose.position.z = enu.z();
    odometry.pose.pose.orientation.w = 1.0;
    const double sigma_xy = std::max(0.01, pvt.h_acc);
    const double sigma_z = std::max(0.01, pvt.v_acc);
    odometry.pose.covariance[0] = sigma_xy * sigma_xy;
    odometry.pose.covariance[7] = sigma_xy * sigma_xy;
    odometry.pose.covariance[14] = sigma_z * sigma_z;
    odometry.pose.covariance[21] = 1e6;
    odometry.pose.covariance[28] = 1e6;
    odometry.pose.covariance[35] = 1e6;
    odometry.twist.twist.linear.x = pvt.vel_e;
    odometry.twist.twist.linear.y = pvt.vel_n;
    odometry.twist.twist.linear.z = -pvt.vel_d;
    odometry.twist.covariance[0] = pvt.vel_acc * pvt.vel_acc;
    odometry.twist.covariance[7] = pvt.vel_acc * pvt.vel_acc;
    odometry.twist.covariance[14] = pvt.vel_acc * pvt.vel_acc;
    publisher_.publish(odometry);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string origin_mode_;
  bool origin_initialized_ = false;
  Eigen::Vector3d origin_lla_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnss_adapter");
  try
  {
    GnssAdapterNode adapter;
    ros::spin();
  }
  catch (const std::invalid_argument &error)
  {
    ROS_FATAL("%s", error.what());
    return 2;
  }
  return 0;
}
