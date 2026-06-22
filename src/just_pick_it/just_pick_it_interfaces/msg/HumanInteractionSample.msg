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

# Closed-loop NN 입력용 detection(대상 물체 image-space). human 정밀보정 구간에
# 매 샘플마다 함께 기록한다. det_valid=false 면 이 프레임에 fresh detection이 없었다는
# 뜻이며, 학습/추론에서 마지막 유효값 freeze 처리한다. cx/cy는 image top-left 원점 px,
# area_norm = bbox_w*bbox_h / (image_width*image_height).
bool det_valid
float64 det_cx
float64 det_cy
float64 det_area_norm
float64 det_image_width
float64 det_image_height
