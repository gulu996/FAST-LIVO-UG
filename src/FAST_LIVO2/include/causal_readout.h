#pragma once
#include "camera_aligned_neu_logger.h"
#include "frontend_quality_contract.h"
#include <fast_livo/CausalFusedOutput.h>
#include <deque>
#include <stdexcept>
namespace fast_livo_causal {
using fast_livo_camera_neu::TimedPose;
inline TimedPose pose(const geometry_msgs::Pose& p) {
 TimedPose a; a.position={p.position.x,p.position.y,p.position.z};
 a.orientation={p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z};
 if(!a.position.allFinite()||!a.orientation.coeffs().allFinite()||a.orientation.norm()<1e-12) throw std::invalid_argument("INVALID_POSE");
 a.orientation.normalize();return a;
}
inline geometry_msgs::Pose message(const TimedPose& p) {
 geometry_msgs::Pose a;a.position.x=p.position.x();a.position.y=p.position.y();a.position.z=p.position.z();
 a.orientation.w=p.orientation.w();a.orientation.x=p.orientation.x();a.orientation.y=p.orientation.y();a.orientation.z=p.orientation.z();return a;
}
inline TimedPose correction(const fast_livo::CorrectionEvent& e) {
 geometry_msgs::Pose p;p.position.x=e.map_to_odom.translation.x;p.position.y=e.map_to_odom.translation.y;p.position.z=e.map_to_odom.translation.z;p.orientation=e.map_to_odom.rotation;return pose(p);
}
struct Received {fast_livo::CorrectionEvent event;std::uint64_t wall=0,steady=0;};
class Readout {
 std::deque<Received> corrections;
 fast_livo::FrontendQualityStream quality;
 std::string backend,frontend,failure;
 std::uint64_t event_id=0,version=0,next_raw=1,last_raw_ns=0;
 bool active=false;
 public:
 void stop(const std::string& reason){failure=reason;active=false;corrections.clear();}
 bool onCorrection(const fast_livo::CorrectionEvent& e,std::uint64_t wall,std::uint64_t steady){
  if(!failure.empty())return false;
  if(backend.empty())backend=e.backend_session;
  if(e.backend_session.empty()||e.backend_session!=backend){stop("BACKEND_SESSION_CHANGED");return false;}
  if(e.event_id!=event_id+1||e.graph_version<version||e.publish_steady_ns>steady){stop("CORRECTION_IDENTITY_OR_TIME_GAP");return false;}
  event_id=e.event_id;version=e.graph_version;
  if(!e.frontend_session.empty()){
   if(frontend.empty())frontend=e.frontend_session;
   if(frontend!=e.frontend_session){stop("FRONTEND_SESSION_CHANGED");return false;}
  }
  if(!e.valid){active=false;corrections.clear();if(e.lifecycle!="UNINITIALIZED"&&e.lifecycle!="PAUSED")stop(e.reason);return true;}
  if(e.lifecycle!="ACTIVE"||e.frontend_session.empty()||e.header.frame_id!="map"||e.child_frame_id!="odom"||e.body_frame_id!="body"||e.dependency_watermark<e.header.stamp||e.last_gnss_factor>e.dependency_watermark){stop("INVALID_CORRECTION_CONTRACT");return false;}
  try {pose(e.source_raw_pose);pose(e.source_optimized_pose);correction(e);}catch(const std::exception&){stop("INVALID_CORRECTION_POSE");return false;}
  active=true;corrections.push_back({e,wall,steady});
  // ponytail: retain up to 128 already received versions (~25 s at 5 Hz).
  // Older versions may expire; a missing eligible version produces unavailable, never extrapolation.
  if(corrections.size()>128)corrections.pop_front();
  return true;
 }
 fast_livo::CausalFusedOutput onRaw(const fast_livo::FrontendQuality& q,std::uint64_t wall,std::uint64_t steady){
  fast_livo::CausalFusedOutput out;out.raw_quality=q;out.raw_arrival_wall_ns=wall;out.raw_arrival_steady_ns=steady;out.odometry.header=q.odometry.header;out.odometry.header.frame_id="map";out.odometry.child_frame_id="body";out.gnss_constraint_age_ns=-1;
  if(frontend.empty())frontend=q.session_id;
  const auto n=q.odometry.header.stamp.toNSec();
  if(failure.empty()&&(q.session_id.empty()||q.session_id!=frontend||q.output_sequence!=next_raw||n<=last_raw_ns||!q.lio_status_known||!q.lio_committed||q.quality==q.FRONTEND_REJECTED||q.quality==q.INPUT_UNKNOWN||q.odometry.header.frame_id!="odom"||q.odometry.child_frame_id!="body"))stop("RAW_IDENTITY_OR_FRONTEND_FAILURE");
  if(failure.empty()){auto ready=quality.push(q,wall/1e9);if(!quality.failure.empty()||ready.size()!=1)stop("QUALITY_CONTRACT:"+quality.failure);}
  ++next_raw;last_raw_ns=n;
  if(!failure.empty()){out.reason=failure;return out;}
  if(!active){out.reason="NO_ACTIVE_CORRECTION";return out;}
  for(auto i=corrections.rbegin();i!=corrections.rend();++i){
   if(i->event.dependency_watermark.toNSec()>n||i->steady>steady||i->event.applied_quality_sequence>q.output_sequence)continue;
   out.correction=i->event;out.correction_arrival_wall_ns=i->wall;out.correction_arrival_steady_ns=i->steady;
   try {out.odometry.pose.pose=message(fast_livo_camera_neu::composePoses(correction(i->event),pose(q.odometry.pose.pose),n));}
   catch(const std::exception&){stop("INVALID_RAW_POSE");out.reason=failure;return out;}
   out.valid=true;out.reason="CAUSAL_FRONTEND_PROPAGATION";
   if(!i->event.last_gnss_factor.isZero())out.gnss_constraint_age_ns=n-i->event.last_gnss_factor.toNSec();
   return out;
  }
  out.reason="NO_PAST_DEPENDENCY_CORRECTION";return out;
 }
};
}
