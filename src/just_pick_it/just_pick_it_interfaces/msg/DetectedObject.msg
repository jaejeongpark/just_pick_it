# Example custom message — reference template for just_pick_it project
# Usage: from just_pick_it_interfaces.msg import DetectedObject

std_msgs/Header header       # timestamp + frame_id (e.g. "camera_frame")
int32            object_id   # ArUco marker ID or object class index
string           label       # human-readable label (e.g. "box_A")
geometry_msgs/Pose pose      # 6DOF pose in the header frame
float32          confidence  # detection confidence [0.0, 1.0]
