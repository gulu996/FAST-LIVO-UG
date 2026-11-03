# Primary atomic output. Invalid records carry no usable pose.
nav_msgs/Odometry odometry
FrontendQuality raw_quality
CorrectionEvent correction
bool valid
string reason
bool covariance_known
uint64 raw_arrival_wall_ns
uint64 raw_arrival_steady_ns
uint64 correction_arrival_wall_ns
uint64 correction_arrival_steady_ns
uint64 publish_wall_ns
uint64 publish_steady_ns
time publish_ros_time
int64 gnss_constraint_age_ns
