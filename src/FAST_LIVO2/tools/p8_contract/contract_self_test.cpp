#include "rtk_fixed_lag_backend.h"
#include "gnss_adapter.h"
#include "gnss_covariance_contract.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
static void require(bool x,const char*s){if(!x)throw std::runtime_error(s);}
using Q=fast_livo::FrontendQuality;
static Q packet(uint64_t n,bool commit=true){Q q;q.session_id="native-session";q.producer_node="/native";q.output_sequence=n;q.source_kind=Q::LIO_SCAN;q.slam_mode="LIO";q.lio_frame_id=n;q.lio_status_known=true;q.lio_committed=commit;q.quality=commit?Q::NORMAL_COMMITTED:Q::FRONTEND_REJECTED;q.odometry.header.stamp.fromSec(100.+n*.1);q.odometry.header.frame_id="odom";q.odometry.child_frame_id="body";q.odometry.pose.pose.orientation.w=1.;q.odometry.pose.pose.position.x=n-1.;q.lio_timestamp=q.odometry.header.stamp;q.last_successful_lio=q.lio_timestamp;q.state_covariance_validity=Q::VALID;q.imu_coverage=Q::VALID;q.input_integrity=Q::VALID;return q;}
namespace fast_livo_backend {
struct RtkFixedLagBackendSelfTestAccess {
 static void run(){
  for(int count:{1,20}) {
   RtkFixedLagBackend b;b.config_.enable=false;b.config_.save_results=false;b.config_.save_text_log=false;b.alignment_.valid=true;b.frontend_quality_required_=true;
   b.qualityCallback(boost::make_shared<Q>(packet(1)));b.qualityCallback(boost::make_shared<Q>(packet(2)));
   require(b.initialized_&&b.livo_factor_count_==1&&b.keyframes_.size()==2,"T1 committed chain failed");
   const auto nodes=b.total_nodes_created_,factors=b.livo_factor_count_,optimizations=b.optimization_count_;
   const auto before=b.smoother_->calculateEstimate();
   for(int i=0;i<count;++i){auto q=packet(i+3,false);q.odometry.pose.pose.position.x=1e6*(i+1);q.consecutive_lio_rejections=i+1;b.qualityCallback(boost::make_shared<Q>(q));}
   b.qualityCallback(boost::make_shared<Q>(packet(count+3)));
   require(b.backend_halted_&&b.frontend_quality_.failure=="FRONTEND_REJECTED","T2/T3 failure not latched");
   require(b.total_nodes_created_==nodes&&b.livo_factor_count_==factors&&b.optimization_count_==optimizations,"T4 recovery bridged failure or mutated graph");
   require(b.smoother_->calculateEstimate().equals(before,0.),"T9 graph changed after failure");
   for(const auto& k:b.keyframes_)require(b.smoother_->marginalCovariance(k.key).allFinite(),"T9 orphan graph");
   std::cout<<"T1,T2,T3,T4,T9 PASS rejected_frames="<<count<<" nodes="<<nodes<<" livo="<<factors<<" RECOVERY_UNSUPPORTED\n";
  }
  GnssAdapterConfig c;c.input_mode="pose_with_covariance_stamped";c.pose_input_quality=4;c.pose_input_frame_id="enu";GnssAdapter a(c);
  geometry_msgs::PoseWithCovarianceStamped m;m.header.frame_id="enu";m.header.stamp.fromNSec(1762270030500000000ULL);
  const double real[]={.0020114398963223673,.000502859998960048,.0036598541773855686,.000502859998960048,.01463941681457076,.0012672676239162683,.0036598541773855686,.0012672676239162683,.005069070543879173};
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)m.pose.covariance[i*6+j]=real[i*3+j];
  auto invalid=a.process(m,ros::Time(200));require(!invalid.publish_odometry&&invalid.status.reject_reason=="COV_NOT_PSD","T7 real nonPSD accepted");
  RtkFixedLagBackend b;b.config_.enable=false;b.config_.save_results=false;b.config_.save_text_log=false;
  nav_msgs::Odometry raw;raw.header=m.header;raw.pose=m.pose;fast_livo::GnssStatus status;status.raw_quality=status.filtered_quality=4;status.accepted=true;
  b.processAcceptedGnss(raw,status);require(b.pending_alignment_gnss_.empty()&&b.last_enqueued_gnss_stamp_ns_<0,"T7 invalid cov poisoned pending/time");
  m.pose.covariance.fill(0.);m.pose.covariance[0]=m.pose.covariance[7]=m.pose.covariance[14]=.01;
  auto valid=a.process(m,ros::Time(201));require(valid.publish_odometry,"T6 invalid input poisoned adapter timestamp");b.processAcceptedGnss(valid.odometry,valid.status);require(b.pending_alignment_gnss_.size()==1,"T6 valid GNSS not queued");
  require(!a.process(m,ros::Time(202)).publish_odometry,"duplicate GNSS accepted");
  std::cout<<"T6,T7 PASS real_epoch=1762270030500000000 invalid_not_queued valid_same_time_accepted\n";
 }
};
}
int main(int argc,char**argv){ros::Time::init();try{
 if(argc==3){
  GnssAdapterConfig cfg;cfg.input_mode="pose_with_covariance_stamped";cfg.pose_input_frame_id="enu";cfg.pose_input_quality=4;GnssAdapter adapter(cfg);
  std::ifstream f(argv[1]);std::ofstream out(argv[2]);require(bool(f)&&bool(out),"dataset files");out<<"stamp_ns,accepted,reason,min_eigenvalue,tolerance_m2\n"<<std::setprecision(17);
  uint64_t ns;int accepted=0,rejected=0;while(f>>ns){geometry_msgs::PoseWithCovarianceStamped m;m.header.stamp.fromNSec(ns);m.header.frame_id="enu";
   f>>m.pose.pose.position.x>>m.pose.pose.position.y>>m.pose.pose.position.z;
   for(int i=0;i<3;++i)for(int j=0;j<3;++j)f>>m.pose.covariance[i*6+j];require(bool(f),"truncated dataset row");
   auto c=fast_livo::checkPositionCovariance(fast_livo::positionCovariance(m.pose.covariance));auto r=adapter.process(m,ros::Time(1));out<<ns<<","<<r.publish_odometry<<","<<r.status.reject_reason<<","<<c.eigenvalues.minCoeff()<<","<<c.tolerance_m2<<"\n";
   if(r.publish_odometry)++accepted;else{++rejected;require(ns==1762270030500000000ULL&&r.status.reject_reason=="COV_NOT_PSD","unexpected Fixed rejection");}
  }require(accepted==3540&&rejected==1,"Fixed data conservation");std::cout<<"DATASET_FIXED_AUDIT_PASS accepted="<<accepted<<" rejected="<<rejected<<"\n";
 }

 using S=fast_livo::FrontendQualityStream;
 {S s;require(s.push(packet(1),1.).size()==1,"stream1");require(s.push(packet(3),1.1).empty(),"out of order leaked");require(s.push(packet(2),1.2).size()==2,"order not restored");require(s.push(packet(3),1.3).empty()&&s.duplicates==1&&s.failure.empty(),"duplicate not idempotent");}
 {S s;s.push(packet(1),1.);s.push(packet(3),1.1);s.expire(3.2);require(s.failure=="QUALITY_GAP_EXPIRED","missing quality accepted");}
 {S s;for(int i=2;i<=66;++i)s.push(packet(i),1.);require(s.failure=="QUALITY_QUEUE_LIMIT"&&s.pending.empty(),"unbounded quality queue");}
 {S s;s.push(packet(1),1.);auto q=packet(2);q.lio_frame_id=3;s.push(q,1.1);require(s.failure=="LIO_TRANSACTION_GAP","hidden LIO gap");}
 {S s;s.push(packet(1),1.);auto q=packet(2);q.session_id="restart";s.push(q,1.1);require(s.failure=="SESSION_OR_SOURCE_CHANGED","session bridged");}
 {S s;auto q=packet(1);q.input_integrity=Q::UNKNOWN;require(s.push(q,1.)[0].quality==Q::SUSPECT_COMMITTED,"unknown promoted NORMAL");require(s.push(q,1.1).empty()&&s.failure.empty(),"diagnostic downgrade broke duplicate identity");}
 {S s;auto q=packet(1);q.lio_status_known=false;s.push(q,1.);require(s.failure=="INPUT_UNKNOWN","unknown transaction accepted");}
 std::cout<<"T5 PASS atomic envelope, reorder, duplicate, loss expiry, capacity, session, hidden transaction gap\n";
 Eigen::Matrix3d C=Eigen::Matrix3d::Identity()*.01;
 require(fast_livo::checkPositionCovariance(C).valid,"normal cov");C(0,1)=.002;require(!fast_livo::checkPositionCovariance(C).valid,"asym accepted");C.setZero();require(!fast_livo::checkPositionCovariance(C).valid,"zero cov accepted");C.setIdentity();C(0,0)=NAN;require(!fast_livo::checkPositionCovariance(C).valid,"nan accepted");
 C.setOnes();require(fast_livo::checkPositionCovariance(C).valid,"PSD singular positive diagonal rejected");C(0,1)=C(1,0)=1.+1e-12;require(fast_livo::checkPositionCovariance(C).valid,"roundoff contract");C(0,1)=C(1,0)=1.001;require(!fast_livo::checkPositionCovariance(C).valid,"negative eig accepted");
 fast_livo_backend::RtkFixedLagBackendSelfTestAccess::run();std::cout<<"P8_CONTRACT_NATIVE_PASS\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
