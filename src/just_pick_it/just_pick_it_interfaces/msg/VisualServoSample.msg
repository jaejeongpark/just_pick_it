std_msgs/Header header

int32 PHASE_WAIT_TARGET=0
int32 PHASE_VISUAL_SERVO=1
int32 PHASE_TERMINAL_APPROACH=2
int32 PHASE_GRIP_DONE=3

int32 phase
uint64 sample_index

bool detected
bool has_seen_target

string target_class_label
int32 target_class_id
int32 track_id
float32 confidence

# cx/cy are pixel coordinates from image top-left origin.
# If detected=true, these are current detection values.
# If detected=false and has_seen_target=true, these are last valid values.
float32 cx
float32 cy

# Current detection only. NaN if detection is currently lost.
float32 current_cx
float32 current_cy
float32 current_area_norm

# Last valid detection.
float32 last_valid_cx
float32 last_valid_cy
float32 last_valid_area_norm

float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h

float32 mask_cx
float32 mask_cy

float32 image_width
float32 image_height

float32 center_error_u
float32 center_error_v
float32 center_norm

float32 area_norm
float32 orientation_angle

float32 time_since_last_detection

float64[6] joint_angles

bool terminal_ready
int32 terminal_ready_count

bool terminal_anchor_valid
float64[6] q_anchor

float32 anchor_cx
float32 anchor_cy
float32 anchor_center_error_u
float32 anchor_center_error_v
float32 anchor_center_norm
float32 anchor_area_norm
float32 anchor_bbox_w
float32 anchor_bbox_h
float32 anchor_confidence

float32 terminal_progress
float32 time_since_terminal_start
int32 terminal_step_count

float32 gripper_value
bool gripper_closed
bool stop_signal