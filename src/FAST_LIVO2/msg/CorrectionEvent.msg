# Atomic read-only estimate event. header.stamp is the graph source epoch.
std_msgs/Header header
string child_frame_id
string body_frame_id
string backend_session
string frontend_session
uint64 event_id
uint64 graph_version
uint64 source_key
uint64 applied_quality_sequence
# Conservative latest sensor input seen by the backend, including pending input.
time dependency_watermark
time last_gnss_factor
geometry_msgs/Pose source_raw_pose
geometry_msgs/Pose source_optimized_pose
geometry_msgs/Transform map_to_odom
bool valid
string lifecycle
string reason
# Time sampled immediately before roscpp publish handoff, NOT hardware receipt.
uint64 publish_wall_ns
uint64 publish_steady_ns
time publish_ros_time
