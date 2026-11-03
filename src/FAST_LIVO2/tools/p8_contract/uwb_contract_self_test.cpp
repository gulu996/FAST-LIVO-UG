#include "rtk_fixed_lag_backend.h"
#include "uwb_manager.h"
#include <gtsam/inference/Symbol.h>
#include <Eigen/QR>
#include <iostream>
#include <fstream>
#include <chrono>
#include <stdexcept>
#include <ros/serialization.h>
static void check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
using Q=fast_livo::FrontendQuality;
using U=uwb_serial_driver::UwbRangeArray;
static Q packet(int n){Q q;q.session_id="synthetic";q.producer_node="/fixture";q.output_sequence=n;q.source_kind=Q::LIO_SCAN;q.slam_mode="LIVO";q.lio_frame_id=n;q.lio_status_known=q.lio_committed=true;q.quality=Q::SUSPECT_COMMITTED;q.odometry.header.stamp=ros::Time(100+n/10,(n%10)*100000000);q.odometry.header.frame_id="odom";q.odometry.child_frame_id="body";q.odometry.pose.pose.orientation.w=1.;double t=n*.1;q.odometry.pose.pose.position.x=.2*t+.015*t;q.odometry.pose.pose.position.y=.1*sin(t*.1)+.01*t;q.lio_timestamp=q.last_successful_lio=q.odometry.header.stamp;q.state_covariance_validity=q.imu_coverage=Q::VALID;q.input_integrity=Q::UNKNOWN;return q;}
static U measure(int n,int anchor=0){U u;u.header.stamp=packet(n).odometry.header.stamp;u.header.frame_id="sensor_clock";u.session_id=42;u.tag_id=3;u.round_sequence=n;u.timestamp_source=U::DEVICE_TIME_MAPPED_LOCAL;u.timestamp_uncertainty_ns=1000000;u.host_receive_stamp=ros::Time(1800000000,123456789);u.host_receive_monotonic_ns=123456789123ULL;uwb_serial_driver::UwbRange r;r.anchor_id=anchor;r.raw_range_m=5.;r.corrected_range_m=5.;r.valid=true;r.source_format="SYNTHETIC_ONLY";r.diag=n;u.ranges.push_back(r);return u;}
struct UwbManagerSelfTestAccess {
 static void rosBridge(){ros::NodeHandle nh;UwbManager m;m.setInputMode("external_topic");m.factor_backend_output_en_=true;m.factor_backend_publisher_=nh.advertise<U>("/p8c_bridge",8);std::vector<U> got;auto sub=nh.subscribe<U>("/p8c_bridge",8,[&](const U::ConstPtr& x){got.push_back(*x);});auto end=ros::WallTime::now()+ros::WallDuration(5);while(m.factor_backend_publisher_.getNumSubscribers()==0&&ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.01).sleep();}check(m.factor_backend_publisher_.getNumSubscribers()==1,"bridge ROS connection");auto u=measure(1);u.header.stamp=ros::Time(1762267490,276699543);u.header.seq=57;auto bad=u.ranges[0];bad.anchor_id=2;bad.valid=false;bad.reject_reason=uwb_serial_driver::UwbRange::STALE_REPEAT;u.ranges.push_back(bad);m.ingestExternalRanges(u);std::vector<UwbRangeMeasurement> data(m.measurement_queue_.begin(),m.measurement_queue_.end());m.publishBackendMeasurements(data,100.);end=ros::WallTime::now()+ros::WallDuration(5);while(got.size()<2&&ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.01).sleep();}check(got.size()==2,"bridge ROS output loss");for(size_t i=0;i<2;++i){U expected=u;expected.ranges={u.ranges[i]};std::cout<<"ROS_HEADER_SEQUENCE input="<<expected.header.seq<<" output="<<got[i].header.seq<<" stamp_ns="<<got[i].header.stamp.toNSec()<<"\n";auto before=ros::serialization::serializeMessage(expected),received=ros::serialization::serializeMessage(got[i]);check(before.num_bytes==received.num_bytes,"wire length changed");for(size_t j=0;j<before.num_bytes;++j)if(before.buf[j]!=received.buf[j])std::cout<<"WIRE_BYTE_DIFFERENCE offset="<<j<<"\n";check(got[i].header.seq==i,"unexpected ROS publisher sequence");expected.header.seq=got[i].header.seq;auto a=ros::serialization::serializeMessage(expected),b=received;check(a.num_bytes==b.num_bytes&&!memcmp(a.buf.get(),b.buf.get(),a.num_bytes),"bridge measurement metadata changed");}std::ifstream maps("/proc/self/maps");std::cout<<maps.rdbuf();std::cout<<"ROS_BRIDGE_WIRE_PARITY_PASS 2 packets including invalid range; exact sec/nsec and measurement metadata; ROS Header.seq is publisher-local\n";}
 static void run(){UwbManager m;m.setInputMode("external_topic");auto u=measure(1);u.header.stamp=ros::Time(1762267490,276699543);u.header.seq=57;auto bad=u.ranges[0];bad.anchor_id=2;bad.valid=false;bad.reject_reason=uwb_serial_driver::UwbRange::STALE_REPEAT;u.ranges.push_back(bad);m.ingestExternalRanges(u);check(m.measurement_queue_.size()==2,"bridge candidate loss");for(size_t i=0;i<2;++i){const auto& x=m.measurement_queue_[i];check(x.external_range_index==i,"bridge point order");auto a=ros::serialization::serializeMessage(u),b=ros::serialization::serializeMessage(*x.external_packet);check(a.num_bytes==b.num_bytes&&!memcmp(a.buf.get(),b.buf.get(),a.num_bytes),"bridge metadata bits changed");}std::cout<<"BRIDGE_IMMUTABLE_PACKET_PASS nanoseconds/session/tag/uncertainty/host/invalid-range\n";}
};
namespace fast_livo_backend {
struct RtkFixedLagBackendSelfTestAccess {
 static gtsam::Point3 anchor(int i){static const double a[4][3]={{-4,-5,1},{15,-5,3},{10,8,-2},{-3,6,5}};return gtsam::Point3(a[i][0],a[i][1],a[i][2]);}
 static void prepare(RtkFixedLagBackend& b,bool uwb=true){b.config_.enable=false;b.config_.save_results=false;b.config_.save_text_log=false;b.config_.uwb_factor_backend_en=uwb;b.frontend_quality_required_=true;b.alignment_.valid=true;b.backend_session_id_="synthetic-backend";for(int i=0;i<4;++i){UwbAnchorConfig a;a.id=i;a.enabled=true;a.position=anchor(i);b.config_.uwb_anchors[i]=a;}}
 static void send(RtkFixedLagBackend& b,U u){b.uwbRangeCallback(boost::make_shared<U>(u));}
 static void quality(RtkFixedLagBackend& b,int n){b.qualityCallback(boost::make_shared<Q>(packet(n)));}
 static void run(const char* path){
  for (bool with_gnss : {false, true}) {
   RtkFixedLagBackend off,on;prepare(off,false);prepare(on,true);
   for (auto* b : {&off,&on}) {
    b->config_.livo_gravity_consistency_en=with_gnss;
    b->config_.raw_odom_gravity_aligned=with_gnss;
   }
   for(int n=1;n<=401;++n){
    quality(off,n);quality(on,n);
    if(with_gnss && n%10==0){
     nav_msgs::Odometry g;g.header.stamp=packet(n).odometry.header.stamp;g.header.frame_id="map";
     g.pose.pose.position.x=.2*n*.1;g.pose.pose.position.y=.1*sin(n*.01);
     g.pose.covariance[0]=g.pose.covariance[7]=g.pose.covariance[14]=.01;
     fast_livo::GnssStatus status;status.accepted=true;status.raw_quality=status.filtered_quality=4;status.num_sv=20;
     for(auto* b:{&off,&on}){b->processAcceptedGnss(g,status);b->processPendingGnss();}
    }
    check(off.total_nodes_created_==on.total_nodes_created_&&off.optimization_count_==on.optimization_count_&&off.gnss_factor_count_==on.gnss_factor_count_,"empty UWB graph identity");
    for(const auto& k:off.keyframes_){
     auto a=off.smoother_->calculateEstimate<gtsam::Pose3>(k.key).matrix(),b=on.smoother_->calculateEstimate<gtsam::Pose3>(k.key).matrix();
     check((a.array()==b.array()).all(),"empty UWB state mismatch");
     auto ca=off.smoother_->marginalCovariance(k.key),cb=on.smoother_->marginalCovariance(k.key);
     check((ca.array()==cb.array()).all(),"empty UWB covariance mismatch");
    }
   }
   check(on.marginalized_nodes_>0,"empty UWB no marginalization");
   check(!with_gnss || (on.gnss_factor_count_>0 && on.gravity_factor_count_>0),"GNSS/gravity parity path not exercised");
   std::cout<<"NO_MEASUREMENT_PARITY_PASS 401 packets all active states/covariances bit exact gnss_and_gravity="<<with_gnss<<" gnss_factors="<<on.gnss_factor_count_<<"\n";
  }
  RtkFixedLagBackend waiting;prepare(waiting);waiting.alignment_.valid=false;waiting.config_.max_active_states=2;
  for(int n=1;n<=50;++n)send(waiting,measure(n));
  check(waiting.pending_uwb_.size()==40&&waiting.uwb_rejected_==10&&waiting.uwb_received_==50,"unaligned pending not bounded/conserved");
  auto saved=waiting.pending_uwb_.back();check(saved.session_id==42&&saved.tag_id==3&&saved.round_sequence==50&&saved.timestamp_uncertainty_ns==1000000&&saved.host_receive_stamp.toNSec()==1800000000123456789ULL,"pending metadata changed");
  auto rejected=waiting.uwb_rejected_;auto invalid=measure(51);invalid.timestamp_source=U::HOST_RECEIVE_LOCAL;send(waiting,invalid);invalid=measure(51);invalid.timestamp_source=U::REPLAY_PRESERVED;send(waiting,invalid);invalid=measure(51);invalid.timestamp_uncertainty_ns=0;send(waiting,invalid);invalid=measure(51);invalid.timestamp_uncertainty_ns=50000001;send(waiting,invalid);invalid=measure(51);invalid.session_id=43;send(waiting,invalid);invalid=measure(51);invalid.tag_id=4;send(waiting,invalid);invalid=measure(51);invalid.round_sequence=50;send(waiting,invalid);invalid=measure(51);invalid.ranges[0].raw_range_m=NAN;send(waiting,invalid);invalid=measure(51);invalid.ranges[0].raw_range_m=251;send(waiting,invalid);invalid=measure(51,99);send(waiting,invalid);send(waiting,measure(50));
  check(waiting.uwb_rejected_==rejected+11&&waiting.pending_uwb_.size()==40,"invalid input accepted");waiting.config_.uwb_anchors[0].position.x()=NAN;send(waiting,measure(51));check(waiting.uwb_rejected_==rejected+12,"nonfinite anchor passed");
  waiting.rejectPendingMeasurementsForStream("EXPLICIT_EOF");waiting.frontend_quality_.lifecycle="EOF";send(waiting,measure(52));check(waiting.pending_uwb_.empty()&&waiting.uwb_received_==waiting.uwb_rejected_,"EOF accounting");
  std::cout<<"QUEUE_IDENTITY_TIME_RANGE_EOF_PASS received="<<waiting.uwb_received_<<" rejected="<<waiting.uwb_rejected_<<"\n";
  // Instantaneous position geometry, not the existing time-aggregated diagnostic.
  gtsam::Matrix3 info=gtsam::Matrix3::Zero();for(int i=0;i<4;++i){gtsam::Vector3 v=anchor(i).normalized();info+=v*v.transpose();check(info.fullPivLu().rank()==std::min(i+1,3),"range geometry rank");}std::cout<<"GEOMETRY_RANK_PASS 1/2/3/4 anchors give position rank 1/2/3/3 for this fixture only\n";
  std::ofstream out(path);check(bool(out),"output CSV");out<<"anchors,gnss,n,stamp_ns,truth_x,truth_y,raw_x,raw_y,fused_x,fused_y,fused_z,uwb_factors,gnss_factors,active,marginalized,wall_ms\n";
  for(int count=0;count<=4;++count)for(int gnss=0;gnss<=1;++gnss){RtkFixedLagBackend b;prepare(b,count>0);auto t0=std::chrono::steady_clock::now();for(int n=1;n<=601;++n){quality(b,n);double t=n*.1;gtsam::Point3 truth(.2*t,.1*sin(.1*t),0.);
    if(n%10==0){if(gnss&&n<=100){nav_msgs::Odometry g;g.header.stamp=packet(n).odometry.header.stamp;g.header.frame_id="map";g.pose.pose.position.x=truth.x();g.pose.pose.position.y=truth.y();g.pose.covariance[0]=g.pose.covariance[7]=g.pose.covariance[14]=.01;fast_livo::GnssStatus s;s.accepted=true;s.raw_quality=s.filtered_quality=4;s.num_sv=20;b.processAcceptedGnss(g,s);b.processPendingGnss();}for(int i=0;i<count;++i){auto u=measure(n,i);u.ranges[0].raw_range_m=(truth-anchor(i)).norm()+.02*sin(n+i);u.ranges[0].corrected_range_m=u.ranges[0].raw_range_m;send(b,u);}}
    check(!b.backend_halted_,"synthetic optimizer halted");const auto& k=b.keyframes_.back();auto p=b.smoother_->calculateEstimate<gtsam::Pose3>(k.key); // Project latest correction onto the current raw pose, as causal adapter does.
    auto raw=RtkFixedLagBackend::poseFromMessage(packet(n).odometry.pose.pose);p=p.compose(k.raw_pose.inverse()).compose(raw);check(p.matrix().allFinite(),"synthetic nonfinite");double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();out<<count<<","<<gnss<<","<<n<<","<<packet(n).odometry.header.stamp.toNSec()<<","<<truth.x()<<","<<truth.y()<<","<<raw.x()<<","<<raw.y()<<","<<p.x()<<","<<p.y()<<","<<p.z()<<","<<b.uwb_factor_count_<<","<<b.gnss_factor_count_<<","<<b.keyframes_.size()<<","<<b.marginalized_nodes_<<","<<ms<<"\n";
   }check(b.marginalized_nodes_>0,"synthetic no marginalization");if(count)check(b.uwb_factor_count_>0,"synthetic UWB did not enter graph");if(gnss)check(b.gnss_factor_count_>0,"synthetic GNSS did not enter graph");check(b.uwb_received_==b.uwb_factor_count_+b.uwb_rejected_+b.pending_uwb_.size(),"UWB conservation");std::cout<<"SYNTHETIC_GRAPH_PASS anchors="<<count<<" gnss="<<gnss<<" uwb_factors="<<b.uwb_factor_count_<<" rejected="<<b.uwb_rejected_<<" gnss_factors="<<b.gnss_factor_count_<<" marginalized="<<b.marginalized_nodes_<<"\n";
  }
 }
};}
int main(int argc,char**argv){if(argc==2&&std::string(argv[1])=="--bridge"){ros::init(argc,argv,"p8c_bridge_test");try{UwbManagerSelfTestAccess::rosBridge();return 0;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}ros::Time::init();try{check(argc==2,"CSV path required");UwbManagerSelfTestAccess::run();fast_livo_backend::RtkFixedLagBackendSelfTestAccess::run(argv[1]);std::cout<<"P8_C_NATIVE_PASS\n";return 0;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
