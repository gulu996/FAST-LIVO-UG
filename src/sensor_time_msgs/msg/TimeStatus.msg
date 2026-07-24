std_msgs/Header header
uint64 session_id
uint64 mcu_boot_id
uint64 writer_epoch
uint32 mapping_version
uint8 time_state
uint8 clock_source

bool local_ready
bool utc_mapping_valid

uint32 last_event_sequence
uint64 event_gap_count
uint64 crc_error_count
uint64 frame_error_count
uint64 duplicate_event_count
uint64 mcu_dropped_event_count

uint64 bridge_queue_depth
uint64 seconds_since_last_gnss_pps
uint64 time_uncertainty_ns

string detail

uint8 CLOCK_INVALID=0
uint8 MCU_HSE_FREE_RUN=1
uint8 LOCAL_ONLY=0
uint8 ACQUIRING=1
uint8 LOCKED=2
uint8 HOLDOVER=3
uint8 RELOCKING=4
uint8 INVALID=5
