std_msgs/Header header

uint64 session_id
uint64 mcu_boot_id
uint64 writer_epoch

uint32 protocol_version
uint32 event_sequence
uint8 message_type
uint32 source_sequence

uint64 local_tick
uint32 local_tick_hz
uint64 local_stamp_ns

uint64 host_receive_monotonic_ns
uint64 time_uncertainty_ns
uint32 flags

uint8 BOOT=1
uint8 LOCAL_PPS_OUTPUT=2
uint8 CAMERA_TRIGGER=3
uint8 GNSS_PPS_CAPTURE=4
uint8 STATUS=5
uint8 ERROR=6
