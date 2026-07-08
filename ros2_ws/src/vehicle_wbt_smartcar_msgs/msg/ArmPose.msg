# Current arm pose. Mirrors ArmController runtime state.
# Published by smartcar_bridge_node at 50 Hz.
int32 arm_id
float32 x
float32 y
float32 pitch
string arm_dir
string hand_dir
bool grasping
bool is_homed