std_msgs/Header header

uint64 session_id
uint64 mcu_boot_id
uint64 writer_epoch
uint32 mapping_version

uint8 time_state

uint64 local_reference_ns
uint64 utc_reference_ns
float64 utc_ns_per_local_ns
float64 slope_ppm

uint64 valid_from_local_ns
uint64 fit_span_ns
uint32 sample_count

float64 residual_rms_ns
float64 residual_max_ns
uint64 time_uncertainty_ns

uint32 accepted_pair_count
uint32 rejected_pair_count
uint32 flags

uint8 LOCAL_ONLY=0
uint8 ACQUIRING=1
uint8 LOCKED=2
uint8 HOLDOVER=3
uint8 RELOCKING=4
uint8 INVALID=5
