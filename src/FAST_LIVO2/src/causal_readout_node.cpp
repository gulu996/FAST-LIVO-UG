#include "causal_readout.h"
#include <ros/ros.h>
#include <chrono>
using fast_livo_causal::Readout;
static std::uint64_t steady(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char**argv){
 ros::init(argc,argv,"causal_readout");ros::NodeHandle n;Readout state;
 auto output=n.advertise<fast_livo::CausalFusedOutput>("/rtk_backend/causal_fused",500);
 auto odom=n.advertise<nav_msgs::Odometry>("/rtk_backend/causal_fused_odom",500);
 std::uint64_t last_correction=0;
 auto corrections=n.subscribe<fast_livo::CorrectionEvent>("/rtk_backend/correction_event",500,[&](const fast_livo::CorrectionEvent::ConstPtr& e){const auto now=steady();last_correction=e->valid?now:0;state.onCorrection(*e,ros::WallTime::now().toNSec(),now);});
 auto quality=n.subscribe<fast_livo::FrontendQuality>("/backend/livo_odom_raw/quality",500,[&](const fast_livo::FrontendQuality::ConstPtr& q){
  const auto arrival=steady();if(last_correction&&arrival-last_correction>2000000000ULL)state.stop("CORRECTION_STREAM_TIMEOUT");
  auto e=state.onRaw(*q,ros::WallTime::now().toNSec(),arrival);
  e.publish_ros_time=ros::Time::now();e.publish_wall_ns=ros::WallTime::now().toNSec();e.publish_steady_ns=steady();output.publish(e);if(e.valid)odom.publish(e.odometry);
 });
 // Single callback queue defines an actual receive order. No history is revised.
 ros::spin();return 0;
}
