std_msgs/Header header
int32 track_id
string class_label
int32 class_id
float32 confidence

float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h

float32 mask_cx
float32 mask_cy
float32 orientation_angle

# segmentation polygon (이미지 px). [x0, y0, x1, y1, ...] 평탄화. 비어 있으면 mask 없음.
float32[] mask_polygon

geometry_msgs/PoseStamped estimated_pose
bool pose_valid

int32 frame_count
