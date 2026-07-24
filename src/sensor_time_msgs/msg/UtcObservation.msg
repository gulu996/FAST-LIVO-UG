std_msgs/Header header
uint64 session_id
uint64 writer_epoch
uint32 source_sequence
uint64 local_stamp_ns
uint64 utc_stamp_ns
uint64 time_uncertainty_ns
uint8 association_type
bool valid
string detail

uint8 INVALID=0
uint8 HOST_SERIAL_ASSOCIATED=1
uint8 HARDWARE_PPS_ASSOCIATED=2
uint8 REPLAY_PRESERVED=3
