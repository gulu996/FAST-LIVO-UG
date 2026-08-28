std_msgs/Header header

uint8 INVALID=0
uint8 SINGLE=1
uint8 DIFFERENTIAL=2
uint8 RTK_FLOAT=3
uint8 RTK_FIXED=4
uint8 RECOVERING=5

uint8 raw_quality
uint8 filtered_quality

bool valid_fix
bool diff_soln
uint8 fix_type
uint8 carr_soln
uint8 num_sv

float64 h_acc
float64 v_acc
float64 p_dop
float64 vel_acc

bool origin_initialized
bool accepted
# True only when the source position covariance is the complete uncertainty
# model and downstream quality/satellite inflation must not be applied again.
bool covariance_authoritative

uint32 consecutive_fixed_count
uint32 consecutive_lost_count

string reject_reason
