std_msgs/Header header

uint64 session_id
uint64 writer_epoch
uint64 source_sequence

time host_receive_stamp
uint64 host_receive_monotonic_ns
uint64 time_uncertainty_ns

uint8 timestamp_source
string device
string protocol
uint8[] data

uint8 INVALID=0
uint8 HOST_RECEIVE_LOCAL=1
uint8 DEVICE_TIME_MAPPED_LOCAL=2
uint8 GNSS_UTC_INVERSE_MAPPED=3
uint8 REPLAY_PRESERVED=4
uint8 REPLAY_REBASED=5
