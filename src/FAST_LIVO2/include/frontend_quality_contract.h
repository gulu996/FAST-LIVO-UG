#pragma once
#include <fast_livo/FrontendQuality.h>
#include <ros/serialization.h>
#include <map>
#include <string>
#include <vector>

namespace fast_livo {
// Bounded single-stream contract. The atomic envelope removes cross-topic join
// races; unchanged legacy raw odom remains available to other subscribers.
class FrontendQualityStream {
 public:
  using Q=FrontendQuality;
  static constexpr size_t capacity=64;
  static constexpr double expiry_s=2.;
  static constexpr double max_sensor_gap_s=.5;
  std::string failure,session,producer;
  std::string lifecycle="ACTIVE", lifecycle_reason="WAITING_FOR_INPUT";
  double lifecycle_wall=0.;
  uint64_t next=1,last_lio_frame=0,duplicates=0;
  double last_progress_wall=0.,gap_since=0.;
  ros::Time last_stamp,last_success;
  Q last;
  std::map<uint64_t,Q> pending;
  static std::string bytes(const Q& q) {
    std::string b(ros::serialization::serializationLength(q),'\0');
    ros::serialization::OStream s(reinterpret_cast<uint8_t*>(&b[0]),b.size());ros::serialization::serialize(s,q);return b;
  }
  void stop(const std::string& why){if(failure.empty())failure=why;pending.clear();lifecycle="SESSION_FAILED";lifecycle_reason=failure;}
  // Explicit trusted controller boundary. It certifies drain; silence never certifies EOF.
  bool control(const std::string& action,const std::string& id,uint64_t watermark,double now,std::string& error) {
    if(session.empty()||id!=session||watermark!=next-1||!pending.empty()){error="CONTROL_IDENTITY_OR_DRAIN_MISMATCH";return false;}
    if(!failure.empty()||lifecycle=="TIMEOUT"){error="RECOVERY_UNSUPPORTED";return false;}
    if(action=="PAUSE" && lifecycle=="ACTIVE")lifecycle="PAUSED";
    else if(action=="RESUME" && lifecycle=="PAUSED"){lifecycle="ACTIVE";last_progress_wall=now;}
    else if(action=="EOF" && (lifecycle=="ACTIVE"||lifecycle=="PAUSED"))lifecycle="EOF";
    else {error="CONTROL_TRANSITION_INVALID";return false;}
    lifecycle_reason="EXPLICIT_"+action;lifecycle_wall=now;error.clear();return true;
  }
  void expire(double now){
    if(!failure.empty()||lifecycle!="ACTIVE")return;
    if(gap_since>0 && now-gap_since>=expiry_s)stop("QUALITY_GAP_EXPIRED");
    else if(last_progress_wall>0 && now-last_progress_wall>=expiry_s){lifecycle="TIMEOUT";lifecycle_reason="QUALITY_STREAM_EXPIRED";lifecycle_wall=now;}
  }
  std::vector<Q> push(const Q& q,double now) {
    std::vector<Q> ready;if(!failure.empty()||lifecycle!="ACTIVE")return ready;
    expire(now);if(!failure.empty()||lifecycle!="ACTIVE")return ready;
    if(q.session_id.empty()||q.producer_node.empty()||q.output_sequence==0||q.odometry.header.stamp.isZero()) {stop("QUALITY_IDENTITY_INVALID");return ready;}
    if(session.empty()){session=q.session_id;producer=q.producer_node;}
    if(session!=q.session_id||producer!=q.producer_node){stop("SESSION_OR_SOURCE_CHANGED");return ready;}
    if(q.output_sequence<next){
      if(q.output_sequence==last.output_sequence && bytes(q)==bytes(last)){++duplicates;return ready;}
      stop("STALE_OR_CONFLICTING_QUALITY");return ready;
    }
    auto old=pending.find(q.output_sequence);
    if(old!=pending.end()){
      if(bytes(old->second)!=bytes(q))stop("CONFLICTING_PENDING_QUALITY");else ++duplicates;
      return ready;
    }
    if(pending.size()>=capacity){stop("QUALITY_QUEUE_LIMIT");return ready;}
    pending.emplace(q.output_sequence,q);
    if(q.output_sequence>next && gap_since==0)gap_since=now;
    while(pending.count(next)) {
      Q x=pending.at(next);pending.erase(next);
      const auto stamp=x.odometry.header.stamp;
      if(!last_stamp.isZero()&&(stamp<=last_stamp||(stamp-last_stamp).toSec()>max_sensor_gap_s)){stop("SENSOR_TIME_GAP_OR_REORDER");break;}
      if(!x.lio_status_known||!x.lio_frame_id||x.quality==Q::INPUT_UNKNOWN){stop("INPUT_UNKNOWN");break;}
      if(!x.lio_committed||x.quality==Q::FRONTEND_REJECTED||x.consecutive_lio_rejections){stop("FRONTEND_REJECTED");break;}
      if(x.quality!=Q::NORMAL_COMMITTED&&x.quality!=Q::SUSPECT_COMMITTED){stop("QUALITY_ENUM_INVALID");break;}
      if(x.source_kind==Q::LIO_SCAN){
        if(x.lio_frame_id!=last_lio_frame+1||x.lio_timestamp!=stamp||x.last_successful_lio!=stamp){stop("LIO_TRANSACTION_GAP");break;}
        last_lio_frame=x.lio_frame_id;last_success=stamp;
      }else if(x.source_kind==Q::VIO_QUERY){
        if(!last_lio_frame||x.lio_frame_id!=last_lio_frame||!x.vio_status_known||!x.vio_accepted||x.last_successful_lio!=last_success||x.lio_timestamp!=last.lio_timestamp){stop("VIO_PROVENANCE_UNKNOWN");break;}
      }else{stop("SOURCE_KIND_UNKNOWN");break;}
      if(x.state_covariance_validity==Q::INVALID||x.imu_coverage==Q::INVALID||x.input_integrity==Q::INVALID){stop("INVALID_STATE_OR_INPUT");break;}
      if(!last_stamp.isZero()&&(x.odometry.header.frame_id!=last.odometry.header.frame_id||x.odometry.child_frame_id!=last.odometry.child_frame_id||x.slam_mode!=last.slam_mode)){stop("COORDINATE_OR_MODE_CHANGED");break;}
      const Q exact=x;
      // UNKNOWN completeness cannot certify NORMAL, even if a producer says so.
      if(x.input_integrity!=Q::VALID||x.state_covariance_validity!=Q::VALID||x.imu_coverage!=Q::VALID)x.quality=Q::SUSPECT_COMMITTED;
      last=exact;last_stamp=stamp;last_progress_wall=now;++next;ready.push_back(x);gap_since=0.;
    }
    if(!pending.empty()&&gap_since==0)gap_since=now;
    return ready;
  }
};
}
