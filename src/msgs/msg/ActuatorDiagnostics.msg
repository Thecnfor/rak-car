# ActuatorDiagnostics.msg — 执行器诊断 (middle layer 2026-08-09).
# state: 0=ok 1=stale 2=blocked 3=fault 4=unknown

std_msgs/Header header
string[] actuator_names
uint8[] state
float32[] last_commanded
float32[] last_measured
uint32[] sequence
