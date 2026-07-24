uint16 anchor_id
float64 raw_range_m
float64 range_bias_m
float64 corrected_range_m
int64 diag
bool valid
uint8 reject_reason
string source_format

uint8 ACCEPTED=0
uint8 INVALID_STATUS=1
uint8 INVALID_RAW_RANGE=2
uint8 INVALID_CORRECTED_RANGE=3
uint8 RANGE_LIMIT=4
uint8 STALE_REPEAT=5
