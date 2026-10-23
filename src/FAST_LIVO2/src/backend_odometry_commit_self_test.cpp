// Test-only negative fixture: real b2dedce mapper dispatch, ROS publication,
// Process2/WorkingCheckpoint, and real GTSAM backend; no SLAM bag or truth claim.
#include "LIVMapper.h"
#include "rtk_fixed_lag_backend.h"
#include <fstream>
#include <iomanip>
#include <unistd.h>
static void check(bool p,const char* text){if(!p)throw std::runtime_error(text);}
static sensor_msgs::ImuConstPtr sample(double t){auto m=boost::make_shared<sensor_msgs::Imu>();m->header.stamp.fromSec(t);m->linear_acceleration.z=9.81;return m;}
namespace fast_livo_backend {
struct RtkFixedLagBackendSelfTestAccess {
 static std::unique_ptr<RtkFixedLagBackend> make(){
  auto b=std::unique_ptr<RtkFixedLagBackend>(new RtkFixedLagBackend());
  b->config_.enable=false; // no unadvertised status publisher; native algorithm remains enabled.
  b->config_.save_results=false;b->config_.save_text_log=false;
  b->alignment_.valid=true; // explicitly known fixture frame, not real ENU alignment evidence.
  b->config_.uwb_factor_backend_en=true;
  UwbAnchorConfig a;a.id=0;a.enabled=true;a.position=gtsam::Point3(5,0,0);b->config_.uwb_anchors[0]=a;
  return b;
 }
 static void raw(RtkFixedLagBackend&b,const nav_msgs::OdometryConstPtr&m){b.rawOdomCallback(m);}
 static void external(RtkFixedLagBackend&b,const nav_msgs::Odometry&m,std::ostream&o){
  check(b.initialized_&&!b.keyframes_.empty(),"backend failed to initialize from received raw odom");
  auto &key=b.keyframes_.back();RtkFixedLagBackend::GnssMeasurement g;
  g.stamp=m.header.stamp;g.position=key.optimized_pose.translation();g.sigmas=gtsam::Vector3::Constant(.1);g.raw_quality=fast_livo::GnssStatus::RTK_FIXED;g.filtered_quality=g.raw_quality;
  check(b.addGnssFactor(g,key),"GNSS native factor failed");
  const auto count=b.gnss_factor_count_;check(!b.addGnssFactor(g,key)&&b.gnss_factor_count_==count,"duplicate GNSS entered graph");
  auto old_g=g;old_g.stamp=ros::Time(9,0);
  check(!b.addGnssFactor(old_g,key)&&b.gnss_factor_count_==count,"stale GNSS entered graph");
  const auto published=b.raw_odom_published_;const auto optimizations=b.optimization_count_;
  b.rawOdomCallback(boost::make_shared<nav_msgs::Odometry>(m));
  auto old_raw=boost::make_shared<nav_msgs::Odometry>(m);old_raw->header.stamp=ros::Time(9,0);b.rawOdomCallback(old_raw);
  check(b.raw_odom_published_==published&&b.optimization_count_==optimizations&&
        b.raw_odom_duplicate_==1&&b.raw_odom_non_monotonic_==1,"duplicate/out-of-order raw odometry changed graph");
  auto u=boost::make_shared<uwb_serial_driver::UwbRangeArray>();u->header.stamp=m.header.stamp;u->round_sequence=42;
  uwb_serial_driver::UwbRange r;r.anchor_id=0;r.raw_range_m=(key.optimized_pose.translation()-b.config_.uwb_anchors[0].position).norm();r.valid=true;r.source_format="EXPLICIT_SYNTHETIC_NATIVE_FIXTURE";u->ranges.push_back(r);
  b.uwbRangeCallback(u);check(b.uwb_factor_count_==1,"UWB real callback/factor failed");
  b.uwbRangeCallback(u);check(b.uwb_factor_count_==1,"duplicate UWB entered graph");
  u->header.stamp=ros::Time(10,0);b.uwbRangeCallback(u);check(b.uwb_factor_count_==1,"stale UWB entered graph");
  u->header.stamp=ros::Time(10,200000000);u->ranges[0].raw_range_m=std::nan("");b.uwbRangeCallback(u);check(b.uwb_factor_count_==1,"NaN UWB entered graph");
  const auto estimate=b.smoother_->calculateEstimate<gtsam::Pose3>(key.key);
  o<<"\"backend_raw_pose_x\":"<<key.raw_pose.x()<<",\"backend_optimized_pose_x\":"<<estimate.x()<<",\"backend_key\":"<<key.key<<",\"backend_key_time\":"<<key.stamp.toSec()<<",\"gnss_factors\":"<<b.gnss_factor_count_<<",\"uwb_factors\":"<<b.uwb_factor_count_<<",\"optimization_count\":"<<b.optimization_count_<<",\"duplicate_and_invalid_fixture_rejected\":true,";
 }
};
}
static void state(std::ostream&f,const StatesGroup&s){f<<'[';bool first=true;auto put=[&](double x){if(!first)f<<',';first=false;f<<x;};for(int r=0;r<3;++r)for(int c=0;c<3;++c)put(s.rot_end(r,c));for(const auto*v:{&s.pos_end,&s.vel_end,&s.bias_g,&s.bias_a,&s.gravity})for(int j=0;j<3;++j)put((*v)(j));put(s.inv_expo_time);for(int r=0;r<DIM_STATE;++r)for(int c=0;c<DIM_STATE;++c)put(s.cov(r,c));f<<']';}
int main(int argc,char**argv){ros::init(argc,argv,"fusion_ownership_fixture");try{
 check(argc>=2,"output directory required");std::string out=argv[1];ros::NodeHandle nh;
 // External topic must override a stale text-replay source without rebasing its stamp.
 nh.setParam("rtk_backend/uwb_factor_backend_en",true);nh.setParam("uwb/input_mode",std::string("external_topic"));
 nh.setParam("uwb/source",std::string("file"));
 {
  UwbManager bridge;check(bridge.initialize(nh,out+"/external_"),"external bridge initialization");
  std::vector<uwb_serial_driver::UwbRangeArray> forwarded;
  auto receiver=nh.subscribe<uwb_serial_driver::UwbRangeArray>("/uwb/backend_ranges",10,[&](const uwb_serial_driver::UwbRangeArrayConstPtr&x){forwarded.push_back(*x);});
  for(int i=0;i<30;++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
  uwb_serial_driver::UwbRangeArray x;x.header.stamp=ros::Time(100,123456789);x.round_sequence=99;
  x.timestamp_source=uwb_serial_driver::UwbRangeArray::HOST_RECEIVE_LOCAL;x.timestamp_uncertainty_ns=1;
  uwb_serial_driver::UwbRange range;range.anchor_id=1;range.raw_range_m=3.;range.corrected_range_m=3.;range.valid=true;x.ranges.push_back(range);
  bridge.ingestExternalRanges(x);check(bridge.pendingMeasurementCount()==1,"external ingress failed");
  bridge.forwardMeasurementsAt(x.header.stamp.toSec(),99.);bridge.forwardMeasurementsAt(x.header.stamp.toSec(),99.);
  for(int i=0;i<30;++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
  check(forwarded.size()==1&&forwarded[0].header.stamp.toSec()==x.header.stamp.toSec()&&forwarded[0].ranges[0].raw_range_m==3.,"external range lost, rebased or duplicated");
 }
 nh.setParam("rtk_backend/uwb_factor_backend_en",false);nh.deleteParam("uwb/input_mode");
 LIVMapper m(nh);image_transport::ImageTransport it(nh);m.initializeSubscribersAndPublishers(nh,it);
 auto backend=fast_livo_backend::RtkFixedLagBackendSelfTestAccess::make();std::vector<nav_msgs::Odometry> received;
 auto sub=nh.subscribe<nav_msgs::Odometry>(m.raw_backend_odom_topic_,10,[&](const nav_msgs::OdometryConstPtr&p){received.push_back(*p);fast_livo_backend::RtkFixedLagBackendSelfTestAccess::raw(*backend,p);});
 auto end=ros::WallTime::now()+ros::WallDuration(5);while(m.pubRawBackendOdom.getNumSubscribers()==0&&ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.01).sleep();}check(m.pubRawBackendOdom.getNumSubscribers()>0,"real ROS subscriber not connected");
 LidarMeasureGroup g;g.lio_vio_flg=LIO;MeasureGroup a;a.lio_time=10.102;for(int i=0;i<=65;++i)a.imu.push_back(sample(9.775+i*.005));g.measures.push_back(a);
 m.p_imu->Process2(g,m._state,m.feats_undistort);check(!m.p_imu->imu_need_init,"fixture initialization failed");m._state.vel_end.x()=1; // declared constant-velocity prior, not a production reset.
 auto prior=m._state;double before=m.p_imu->coverageStateTime();m.imu_working_.begin(m._state,*m.p_imu);m.imu_vio_accepted_before_=m.runtime_events_.vio_accepted;
 g.lio_vio_flg=VIO;g.measures.back().vio_time=10.102;g.measures.back().imu={sample(10.105)};m.p_imu->Process2(g,m._state,m.feats_undistort);check(m.p_imu->coveragePredictionComplete(10.102),"actual prediction invalid");auto predicted=m._state;
 m.LidarMeasures=g;m.LidarMeasures.last_lio_update_time=10.102;m._first_lidar_time=10.;m.slam_mode_=LIVO;m.adaptive_visual_selector_en=true;m.img_en=0;m.imu_prop_enable=false; // explicit selector-rejection fixture, not actual visual acceptance test.
 m.pcl_w_wait_pub->push_back(PointType());m.stateEstimationAndMapping();
 for(int i=0;i<20;++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
 check(received.empty(),"rolled-back query leaked to backend");
 check((m._state.cov.array()==prior.cov.array()).all()&&(m._state.pos_end.array()==prior.pos_end.array()).all(),"front rollback incomplete");check(m.p_imu->coverageStateTime()==before&&!m.imu_working_.active(),"cursor/checkpoint rollback incomplete");
 const auto restored=m._state;
 // Conditional accepted-measurement handoff fixture; this is not a real VIO acceptance.
 m.imu_working_.begin(m._state,*m.p_imu);g.measures.back().vio_time=10.104;m.p_imu->Process2(g,m._state,m.feats_undistort);m.LidarMeasures.last_lio_update_time=10.104;
 const auto accepted=m._state;check(!m.publishRawBackendOdometry(),"working publication not deferred");
 bool early=false;try{m.finishRawBackendOdometry(true);}catch(const std::logic_error&){early=true;}check(early,"early publication allowed");
 m.imu_working_.finish(m._state,*m.p_imu,true,false);m.finishRawBackendOdometry(true);m.finishRawBackendOdometry(true);
 end=ros::WallTime::now()+ros::WallDuration(5);while(received.empty()&&ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.01).sleep();}
 check(received.size()==1&&received[0].header.stamp==ros::Time(10,104000000),"committed handoff missing or duplicated");
 check(received[0].pose.pose.position.x==accepted.pos_end.x(),"committed payload changed");
 check((m._state.cov.array()==accepted.cov.array()).all()&&(m._state.pos_end.array()==accepted.pos_end.array()).all(),"publication modified estimator state");
 std::ofstream f(out+"/counterexample.json");f<<std::setprecision(17)<<"{\"fixture\":\"synthetic stationary IMU with constant-velocity prior, real mapper selector rejection and ROS raw odometry\",\"reproduced_uncommitted_backend_pose\":false,\"front_rollback_exact\":true,\"prior_time\":"<<before<<",\"query_time\":10.102,\"post_rollback_time\":"<<before<<",\"conditional_commit_time\":"<<m.p_imu->coverageStateTime()<<",\"received_count\":"<<received.size()<<",\"received_stamp_ns\":"<<received[0].header.stamp.toNSec()<<",\"prior_state_P\":";state(f,prior);f<<",\"prediction_state_P\":";state(f,predicted);f<<",\"restored_state_P\":";state(f,restored);f<<',';
 fast_livo_backend::RtkFixedLagBackendSelfTestAccess::external(*backend,received[0],f);
 f<<"\"rejected_query_publication_blocked\":true,\"conditional_committed_publication_once\":true,\"full_compatibility_pass\":false}";f.close();std::ifstream maps("/proc/self/maps");std::ofstream(out+"/counterexample.maps")<<maps.rdbuf();std::cout<<"SCOPED_PUBLICATION_PATCH_PASS\n";ros::shutdown();return 0;
}catch(const std::exception&e){std::cerr<<"FIXTURE_ERROR "<<e.what()<<'\n';return 1;}}
