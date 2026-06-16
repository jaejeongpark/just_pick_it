std_msgs/Header header
string episode_id

int32 sample_index
int32 phase
# 0=WAITING, 1=FREE_DRIVE, 2=GRIPPING, 3=WAITING_RESULT, 4=RESULT, 5=READY_TO_RELEASE

float64[6] joint_angles
float64[6] delta_angles
float64 time_since_prev_sample

float64 gripper_value
bool gripper_closed

bool grip_triggered
bool result_recorded
bool grip_success

float64 time_since_ibvs_done
