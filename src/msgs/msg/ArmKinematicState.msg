# ArmKinematicState.msg — 机械臂运动学状态 (middle layer 2026-08-09).
# feedback_source: 0=encoder 1=open_loop_high_conf 2=open_loop 3=unknown

std_msgs/Header header
string[] joint_names
float64[] position
float64[] velocity
uint8[] feedback_source
float64[] confidence
geometry_msgs/Pose tool_pose
bool reachable
