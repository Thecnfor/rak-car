# Current chassis state. Mirrors MecanumDriver.get_odometry + wheel velocities.
# Published by smartcar_bridge_node at 50 Hz (matches existing
# /state/odom + mecanum_chassis_node rate).
float32 x
float32 y
float32 z
float32 vx
float32 vy
float32 wz