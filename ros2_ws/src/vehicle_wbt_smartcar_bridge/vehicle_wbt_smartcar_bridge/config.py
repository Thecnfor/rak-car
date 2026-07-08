"""Constants for the smartcar bridge.

Topic namespaces mirror the existing rak-car /vehicle_wbt/v1/ prefix.
Service names follow the official MyCar API surface (car_wrap_2026.py).
"""
from __future__ import annotations

# Topics (existing — we subscribe)
CMD_VEL_SAFE = '/vehicle_wbt/v1/cmd/vel_safe'
ARM_TRAJECTORY_CMD = '/vehicle_wbt/v1/cmd/arm/main/trajectory'
STATE_ODOM = '/vehicle_wbt/v1/state/odom'
STATE_ACTUATORS = '/vehicle_wbt/v1/state/actuators/main'

# Topics (new — we publish)
STATE_CHASSIS_ODOMETRY = '/vehicle_wbt/v1/state/chassis/odometry'
STATE_ARM_POSE = '/vehicle_wbt/v1/state/arm/pose'
STATE_BRIDGE_STATUS = '/vehicle_wbt/v1/state/bridge/status'

# Peripheral trigger topics (for hardware to subscribe; MC602 wiring is the
# future real impl). For now the bridge publishes these as event topics; the
# real C++ node that actually drives the buzzer/servo/PoutD will subscribe.
TRIGGER_BEEP = '/vehicle_wbt/v1/cmd/peripheral/beep_event'
TRIGGER_STORAGE = '/vehicle_wbt/v1/cmd/peripheral/storage_event'
TRIGGER_SHOOT = '/vehicle_wbt/v1/cmd/peripheral/shoot_event'

# Service namespaces
SRV_CHASSIS_PREFIX = '/vehicle_wbt/v1/cmd/chassis/'
SRV_ARM_PREFIX = '/vehicle_wbt/v1/cmd/arm/'
SRV_PERIPHERAL_PREFIX = '/vehicle_wbt/v1/cmd/peripheral/'

# Joint ordering for trajectory_msgs (must match arm_node.cpp joint_names_)
ARM_JOINT_NAMES = ['horiz_m6', 'vert_stepper3', 'rotate_s3', 'grip_s7']

# Default geometry / control rates
DEFAULT_CHASSIS_RATE_HZ = 50.0
DEFAULT_ARM_RATE_HZ = 50.0
DEFAULT_STATUS_RATE_HZ = 1.0

# Motion convergence thresholds
DEFAULT_POSITION_TOL = 0.02  # meters
DEFAULT_ANGLE_TOL = 0.05  # radians
DEFAULT_LINEAR_SPEED = 0.3  # m/s
DEFAULT_ANGULAR_SPEED = 0.5  # rad/s

# Service timeouts
SERVICE_TIMEOUT_S = 30.0  # max blocking time for any service call
LANE_FOLLOW_DEFAULT_TIMEOUT_S = 30.0