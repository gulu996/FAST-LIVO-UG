#include "rtk_fixed_lag_backend.h"
#include "gnss_adapter.h"
#include "uwb_manager.h"
#include <ros/serialization.h>
#include <fstream>
#include <sstream>
#include <iomanip>
static void require(bool b,const char*s){if(!b)throw std::runtime_error(s);}
namespace fast_livo_backend {
struct RtkFixedLagBackendSelfTestAccess {
 static void raw(RtkFixedLagBackend&b,const nav_msgs::OdometryConstPtr&m){b.rawOdomCallback(m);}
 static void gnss(RtkFixedLagBackend&b,const GnssAdapterResult&r){
  b.gnssStatusCallback(boost::make_shared<fast_livo::GnssStatus>(r.status));
  if(r.publish_odometry)b.gnssOdomCallback(boost::make_shared<nav_msgs::Odometry>(r.odometry));
 }
 static void result(RtkFixedLagBackend&b,std::ostream&o){
  o<<"\"initialized\":"<<b.initialized_<<",\"aligned\":"<<b.alignment_.valid<<",\"halted\":"<<b.backend_halted_<<",\"raw_received\":"<<b.raw_odom_received_<<",\"gnss_factors\":"<<b.gnss_factor_count_<<",\"uwb_factors\":"<<b.uwb_factor_count_<<",\"optimizations\":"<<b.optimization_count_<<",\"tf_published\":"<<b.tf_published_;
  require(!b.backend_halted_,"backend halted");require(b.initialized_&&b.optimization_count_>0,"backend not initialized/optimized");
 }
 static bool factors(RtkFixedLagBackend&b,bool uwb){return uwb?b.uwb_factor_count_>0:b.gnss_factor_count_>0;}
};
}
int main(int argc,char**argv){ros::init(argc,argv,"real_backend_short");try{
 require(argc==6,"mode raw_tum gnss_bin lidar_start output");bool uwb=std::string(argv[1])=="uwb";ros::NodeHandle nh;
 fast_livo_backend::RtkFixedLagBackend b(nh);GnssAdapter adapter;UwbManager bridge;
 if(uwb)require(bridge.initialize(nh,std::string(argv[5])+"_"),"bridge initialization");else require(adapter.initialize(nh),"adapter initialization");
 std::vector<nav_msgs::Odometry> odom;std::ifstream f(argv[2]);require(f.good(),"raw trajectory missing");std::string line;
 while(std::getline(f,line)){if(line.empty()||line[0]=='#')continue;std::istringstream in(line);double t;nav_msgs::Odometry m;auto&p=m.pose.pose.position;auto&q=m.pose.pose.orientation;
  if(!(in>>t>>p.x>>p.y>>p.z>>q.x>>q.y>>q.z>>q.w))throw std::runtime_error("invalid TUM row");m.header.stamp.fromSec(t);m.header.frame_id="odom";m.child_frame_id="body";if(!odom.empty()&&t>odom.front().header.stamp.toSec()+90)break;odom.push_back(m);}
 require(!odom.empty(),"empty odometry");std::vector<gnss_serial_driver::GnssPvtStamped> gnss;
 if(!uwb){std::ifstream input(argv[3],std::ios::binary);require(input.good(),"GNSS input missing");while(input.peek()!=EOF){uint32_t n;input.read((char*)&n,4);require(input.good()&&n<1000000,"GNSS record length");std::vector<uint8_t> bytes(n);input.read((char*)bytes.data(),n);require(input.good(),"truncated GNSS");gnss_serial_driver::GnssPvtStamped m;ros::serialization::IStream s(bytes.data(),n);ros::serialization::deserialize(s,m);gnss.push_back(m);}}
 ros::WallDuration(.3).sleep();size_t i=0,j=0,origins=0,accepted=0;
 while(i<odom.size()||j<gnss.size()){
  if(i<odom.size()&&(j==gnss.size()||odom[i].header.stamp<=gnss[j].header.stamp)){
   ros::Time::setNow(odom[i].header.stamp);fast_livo_backend::RtkFixedLagBackendSelfTestAccess::raw(b,boost::make_shared<nav_msgs::Odometry>(odom[i]));
   if(uwb){bridge.forwardMeasurementsAt(odom[i].header.stamp.toSec(),std::stod(argv[4]));ros::WallDuration(.001).sleep();ros::spinOnce();}++i;
  }else{ros::Time::setNow(gnss[j].header.stamp);auto r=adapter.process(gnss[j],gnss[j].header.stamp);origins+=r.origin_initialized_now;accepted+=r.publish_odometry;if(r.publish_odometry)require(r.odometry.header.stamp==gnss[j].header.stamp,"timestamp changed");fast_livo_backend::RtkFixedLagBackendSelfTestAccess::gnss(b,r);++j;}
 }
 for(int k=0;k<20;++k){ros::spinOnce();ros::WallDuration(.01).sleep();}
 std::ofstream out(argv[5]);out<<std::boolalpha<<std::setprecision(17)<<"{\"mode\":\""<<(uwb?"real0709_UWB":"real804_GNSS")<<"\",\"scope\":\"90s recorded raw trajectory with real sensor observations; software backend regression, no accuracy certification\",\"raw_rows\":"<<i<<",\"gnss_rows\":"<<j<<",\"origin_initializations\":"<<origins<<",\"adapter_accepted\":"<<accepted<<',';
 fast_livo_backend::RtkFixedLagBackendSelfTestAccess::result(b,out);out<<",\"factor_added\":"<<fast_livo_backend::RtkFixedLagBackendSelfTestAccess::factors(b,uwb)<<"}";
 require(fast_livo_backend::RtkFixedLagBackendSelfTestAccess::factors(b,uwb),"no measurement factor");ros::shutdown();return 0;
}catch(const std::exception&e){std::cerr<<"REAL_BACKEND_SHORT "<<e.what()<<'\n';return 1;}}
