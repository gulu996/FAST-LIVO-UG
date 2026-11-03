#include "rtk_fixed_lag_backend.h"
#include <iostream>
#include <stdexcept>
#include <Eigen/QR>
#include <gtsam/inference/Symbol.h>
static void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
using Q=fast_livo::FrontendQuality;
static Q packet(uint64_t n,double origin=100.,const std::string& session="test-session") {
 Q q;q.session_id=session;q.producer_node="/native";q.output_sequence=n;q.source_kind=Q::LIO_SCAN;q.slam_mode="LIVO";
 q.lio_frame_id=n;q.lio_status_known=q.lio_committed=true;q.quality=Q::SUSPECT_COMMITTED;
 q.odometry.header.stamp.fromSec(origin+n*.1);q.odometry.header.frame_id="odom";q.odometry.child_frame_id="body";
 q.odometry.pose.pose.orientation.w=1.;q.odometry.pose.pose.position.x=(n-1)*.1;
 q.lio_timestamp=q.last_successful_lio=q.odometry.header.stamp;q.state_covariance_validity=q.imu_coverage=Q::VALID;q.input_integrity=Q::UNKNOWN;return q;
}
namespace fast_livo_backend {
struct RtkFixedLagBackendSelfTestAccess {
 static void prepare(RtkFixedLagBackend& b,bool fixture_anchor=true) {
  b.config_.enable=false;b.config_.save_results=false;b.config_.save_text_log=false;
  b.frontend_quality_required_=true;b.alignment_.valid=fixture_anchor;b.backend_session_id_="native-instance";
  // ponytail: test-only identity map anchor represents a fully known synthetic initial pose;
  // no runtime recovery endpoint or claim that GNSS alone supplies this anchor.
 }
 static void send(RtkFixedLagBackend& b,Q q){b.qualityCallback(boost::make_shared<Q>(q));}
 static bool control(RtkFixedLagBackend& b,const std::string& action) {
  fast_livo::BackendStreamControl::Request r;fast_livo::BackendStreamControl::Response s;
  r.command=action;r.frontend_session_id=b.frontend_quality_.session;r.last_output_sequence=b.frontend_quality_.next-1;
  require(b.streamControl(r,s),"service transport false");return s.accepted;
 }
 static void run() {
  RtkFixedLagBackend old;prepare(old);
  for(int i=1;i<=601;++i)send(old,packet(i));
  require(old.initialized_&&!old.backend_halted_&&old.config_.lag_seconds==20.,"T1 graph failed");
  require(old.marginalized_nodes_>0&&old.total_nodes_created_>old.keyframes_.size(),"T1 no real native marginalization");
  require(old.smoother_->timestamps().size()==old.keyframes_.size(),"T1 stale graph keys");
  for(const auto& k:old.smoother_->timestamps())require(k.second>=140.,"T1 expired key retained");
  std::cout<<"T1 PASS sensor_span=60s lag=20s nodes="<<old.total_nodes_created_<<" marginalized="<<old.marginalized_nodes_<<" active="<<old.keyframes_.size()<<"\n";
  const auto estimate=old.smoother_->calculateEstimate();const auto nodes=old.total_nodes_created_;const auto factors=old.livo_factor_count_;const auto optimizations=old.optimization_count_;
  require(control(old,"PAUSE"),"T9 pause rejected");old.frontend_quality_.expire(ros::WallTime::now().toSec()+30.);
  require(old.frontend_quality_.lifecycle=="PAUSED"&&old.frontend_quality_.failure.empty()&&!old.backend_halted_,"T9 pause is physical failure");
  require(old.smoother_->calculateEstimate().equals(estimate,0.),"T9 paused history changed");
  require(control(old,"RESUME"),"T9 resume failed");send(old,packet(602));require(old.frontend_quality_.next==603,"T9 continuous resume not accepted");
  require(control(old,"EOF"),"T10 EOF rejected");const auto eof_estimate=old.smoother_->calculateEstimate();const auto eof_opt=old.optimization_count_;
  old.frontend_quality_.expire(ros::WallTime::now().toSec()+30.);send(old,packet(603));
  require(old.frontend_quality_.lifecycle=="EOF"&&old.frontend_quality_.failure.empty()&&!control(old,"RESUME"),"T10 EOF resurrected");
  require(old.optimization_count_==eof_opt&&old.smoother_->calculateEstimate().equals(eof_estimate,0.)&&old.quality_after_boundary_==1,"T10 mutated EOF graph");
  std::cout<<"T9,T10 PASS explicit pause/resume/EOF history exact; post-EOF input rejected\n";
  for(int count:{1,20}) {
   RtkFixedLagBackend b;prepare(b);send(b,packet(1));send(b,packet(2));auto before=b.smoother_->calculateEstimate();auto n=b.total_nodes_created_;
   for(int i=3;i<3+count;++i){auto q=packet(i);q.lio_committed=false;q.quality=Q::FRONTEND_REJECTED;q.odometry.pose.pose.position.x=1e6;send(b,q);}
   send(b,packet(count+3));send(b,packet(1,200.,"new-session"));
   require(b.backend_halted_&&b.frontend_quality_.lifecycle=="SESSION_FAILED"&&b.total_nodes_created_==n&&b.smoother_->calculateEstimate().equals(before,0.),"T2-4 failure bridged");
   require(!control(b,"RESUME"),"T4 failure silently cleared");
  }
  std::cout<<"T2,T3,T4 PASS single/20 rejects and new committed session do not reconnect\n";
  // Rank counterexample: one GNSS antenna position constrains only three Pose3 directions.
  GnssPositionArmFactor f(gtsam::Symbol('x',0),gtsam::Point3(.023,-.023,.09),gtsam::Point3(.023,-.023,.09),gtsam::noiseModel::Isotropic::Sigma(3,.1));
  gtsam::Matrix H;f.evaluateError(gtsam::Pose3(),H);require(H.fullPivLu().rank()==3,"T5 GNSS rank counterexample");
  RtkFixedLagBackend missing;prepare(missing,false);send(missing,packet(1,200.,"unanchored"));
  require(!missing.initialized_&&!missing.smoother_&&missing.total_nodes_created_==0,"T5 unanchored graph manufactured");
  RtkFixedLagBackend fresh;prepare(fresh);fresh.backend_session_id_="fresh-anchored-fixture";
  for(int i=1;i<=401;++i)send(fresh,packet(i,200.,"new-anchored-fixture"));
  require(fresh.initialized_&&fresh.marginalized_nodes_>0&&fresh.frontend_quality_.session!="test-session","T6 isolated known-anchor graph failed");
  require(fresh.config_.prior_roll_pitch_sigma_rad==.05&&fresh.config_.prior_yaw_sigma_rad==.2&&fresh.config_.prior_translation_sigma_m==.1,"T6 fake strong prior");
  for(const auto& k:fresh.keyframes_)require(fresh.smoother_->marginalCovariance(k.key).allFinite(),"T6 singular anchor");
  require(old.smoother_->calculateEstimate().equals(eof_estimate,0.),"T6 old object changed");
  nav_msgs::Odometry gnss;gnss.header.frame_id="map";gnss.header.stamp.fromSec(150.);gnss.pose.covariance[0]=gnss.pose.covariance[7]=gnss.pose.covariance[14]=.01;
  fast_livo::GnssStatus fixed;fixed.accepted=true;fixed.raw_quality=fixed.filtered_quality=4;
  auto nf=fresh.gnss_factor_count_;auto nreject=fresh.gnss_rejected_;fresh.processAcceptedGnss(gnss,fixed);fresh.processPendingGnss();
  require(fresh.gnss_factor_count_==nf&&fresh.gnss_rejected_>nreject&&fresh.pending_factor_gnss_.empty(),"T7 old timestamp GNSS leaked");
  std::cout<<"T5,T6,T7 PASS rank3, missing-anchor unavailable, independent synthetic known-anchor graph, old-time GNSS rejected; automatic recovery NOT_IMPLEMENTED\n";
 }
};
}
int main(){ros::Time::init();try {
 using S=fast_livo::FrontendQualityStream;std::string error;
 {S s;s.push(packet(1),1.);s.expire(3.1);require(s.lifecycle=="TIMEOUT"&&s.failure.empty(),"silence inferred EOF/reject");require(!s.control("RESUME",s.session,1,4.,error),"timeout auto-resumed");}
 {S s;s.push(packet(1),1.);s.push(packet(3),1.1);s.push(packet(2),1.2);require(s.next==4,"T8 reorder");s.push(packet(3),1.3);require(s.duplicates==1,"T8 duplicate");auto q=packet(4);q.lio_status_known=false;s.push(q,1.4);require(s.failure=="INPUT_UNKNOWN","T8 unknown passed");}
 {S s;s.push(packet(1),1.);require(!s.control("EOF","wrong",1,1.1,error),"wrong session control passed");require(!s.control("EOF",s.session,2,1.1,error),"wrong watermark passed");}
 fast_livo_backend::RtkFixedLagBackendSelfTestAccess::run();std::cout<<"P8_B25_NATIVE_PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
