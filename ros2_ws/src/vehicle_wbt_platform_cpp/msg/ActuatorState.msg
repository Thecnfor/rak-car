# ActuatorState.msg — Generic actuator state (joint positions/velocities/effort).
# Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §自定义消息
#
# Replaces the old ArmState.msg. N-dim arrays (not 6-element) so this works
# for any arm topology: 2-joint, 6-joint, 7-joint, dual-arm (length * 2).

std_msgs/Header header
string[] joint_names         # order matches positions/velocities/effort
float64[] positions
float64[] velocities
float64[] effort
bool vacuum                  # current vacuum pump state (false if no pump)
bool valve                   # current valve state (false if no valve)
string actuator_id           # which actuator instance this is (e.g. "arm_main")
