# Hardware Inventory

Reference catalog of every sensor and actuator wired into the rak
platform. **This file is documentation only** — it is NOT loaded by any
node or launch file. ROS2-native convention: every node declares its own
parameters in source and accepts overrides via launch files
(`launch/full_system.launch.py` etc.). Adding new hardware therefore
means:

1. Add a link + joint to `src/bringup/urdf/rak.urdf.xacro`.
2. Add the node entry to `launch/full_system.launch.py` (or its mock/stub
   variant), wiring the right topic, port, rate, and `mc602_serial_port`/
   `mc602_baud` shared with other MC602 nodes.
3. If the hardware drives a new task, add a `REGISTER_TASK("name", ClassName)`
   in a new `.cpp` under `src/` and link it into `mission_runner_node`.

This document exists so people can see at a glance what's wired without
grovelling through launch files. When a value here disagrees with
`full_system.launch.py`, the launch file wins (it is the runtime source
of truth).

---

## Sensors (5 wired, 1 reserved)

| id | type | port | rate | topic | msg_type |
|---|---|---|---|---|---|
| left_ir | ir | P8 (MC602 port 8) | 20 Hz | /rak/sensors/ir/left | std_msgs/Float32 |
| right_ir | ir | P7 (MC602 port 7) | 20 Hz | /rak/sensors/ir/right | std_msgs/Float32 |
| camera_front | camera | /dev/cam2 (UVC) | 30 Hz | /rak/sensors/camera/front/image_raw | sensor_msgs/Image (bgr8) |
| camera_arm | camera | /dev/cam1 (UVC) | 30 Hz | /rak/sensors/camera/arm/image_raw | sensor_msgs/Image (bgr8) |
| vert_limit | analog_input | P6 | 50 Hz | /rak/sensors/analog/vert_limit | std_msgs/Int32 |
| ultrasonic_front | ultrasonic | P5 | (reserved) | /rak/sensors/ultrasonic/front | std_msgs/Float32 |

**Cameras** publish 5 streams each (image_raw + image_compressed +
camera_info + camera_status + camera_meta) under
`/rak/sensors/camera/<id>/`. `camera_info` is only published
when calibration YAML has non-zero intrinsics. Each camera publishes a
`/tf_static` link `base_link → <id>_camera_optical_frame`.

**IR port convention**: P8 left, P7 right (per hardware-port-mapping.md).
In `full_system.launch.py`, `mc602_port: 8` for `ir_left`, `7` for `ir_right`.

## Actuators (10 wired, 3 reserved)

| id | type | port | rate | topic | msg_type |
|---|---|---|---|---|---|
| motor_ejection | motor | M5 | 50 Hz | /rak/actuators/motor/m5/state | Float32 |
| motor_arm_horiz | motor | M6 | 50 Hz | /rak/actuators/motor/m6/state | Float32 |
| stepper_ejection_angle | stepper | STEP1 | 50 Hz | /rak/actuators/stepper/1/state | Float32 |
| stepper_arm_vert | stepper | STEP3 | 50 Hz | /rak/actuators/stepper/3/state | Float32 |
| servo_hand_rotate | servo_bus | S3 | 50 Hz | /rak/actuators/servo/s3/state | Float32 |
| servo_hand_grip | servo_pwm | S7 (270° mode) | 50 Hz | /rak/actuators/servo/s7/state | Float32 |
| servo_weather | servo_bus | S2 | 50 Hz | /rak/actuators/servo/s2/state | Float32 |
| vacuum_pump | dout | P2 | 20 Hz | /rak/actuators/io/p2/state | Bool |
| vacuum_valve | dout | P3 | 20 Hz | /rak/actuators/io/p3/state | Bool |
| ejection_valve | dout | P4 | 20 Hz | /rak/actuators/io/p4/state | Bool |

(Reserved: stepper STEP2, servo_bus S1, servo_pwm S4.)

## Shared hardware params (every MC602-using node)

```
mc602_serial_port: /dev/ttyUSB0
mc602_baud: 1000000
```

These are declared as launch args `serial_port` / `baud` in
`full_system.launch.py` and passed to every node that drives the MC602
(`infrared_node`, `mecanum_chassis_node`, `arm_node`). Override per
machine: pass `serial_port:=/dev/ttyUSB1` or `baud:=380400` to the launch.

## Chassis geometry

| param | value | notes |
|---|---|---|
| chassis_Lx | 0.15 m | half of xacro `chassis_length` (0.30) |
| chassis_Ly | 0.10 m | half of xacro `chassis_width` (0.20) |
| wheel_radius | 0.03 m | matches xacro |
| publish_rate_hz | 50 Hz | odom/tf cadence |

These must agree with `urdf/rak.urdf.xacro` `chassis_length /
chassis_width / wheel_radius` properties.

## Camera frame geometry (must match `camera_node` tf_* params)

```
tf_parent_frame: base_link
tf_x/y/z and tf_roll/pitch/yaw are set per camera so the
camera_node-published tf_static matches the xacro link positions.
Default front:    x = +chassis_length/2 - 0.01 = 0.14
Default arm:      y = +chassis_width/2  - 0.01 = 0.09
Both at z = chassis_height/2 = 0.04.
```

(Per-machine calibration overrides via launch arg `tf_*` if a camera is
physically repositioned.)

## Mission tasks

Currently registered (see `src/seeding_task.cpp` for the reference):

| name | status |
|---|---|
| seeding | reference task (stub execute — publish Twist for 3 ticks per station) |
| pest_scout / shoot_pest / harvest / read_order / delivery | not yet implemented |

Tasks self-register via `REGISTER_TASK(name, ClassName)` at static-init.
Adding a new task: drop a `src/<name>_task.cpp` implementing
`hardware::BaseTask` and linking into
`mission_runner_node` (see `src/seeding_task.cpp` + `CMakeLists.txt`).

## Adding hardware — 3-step recipe

```bash
# 1. URDF link + joint
$EDITOR src/bringup/urdf/rak.urdf.xacro

# 2. Launch entry (topic, port, rate, mc602_serial_port/baud if MC602)
$EDITOR src/bringup/launch/full_system.launch.py

# 3. If it's a new task type, register a BaseTask:
$EDITOR src/hardware/src/<name>_task.cpp   # + CMakeLists.txt
```