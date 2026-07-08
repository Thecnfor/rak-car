# CLAUDE.md — Robot-Side (Jetson Orin Nano 4GB)

> **You are working on the robot control side.** Other team members
> develop on their dev machines and control this robot remotely over the
> LAN via ROS2 topics. This Jetson should be stable, predictable, and
> never need a "developer setup" — only `colcon build` and `ros2 launch`.

## Project Overview

**vehicle_wbt** is a ROS2 Humble autonomous vehicle robot. This branch
(`robot-stable`) is the slimmed runtime-only subset that lives on this
Jetson. The companion `develop/ros2-sidecar` branch is where dev work
happens; **never commit dev experiments directly here** — push to
`develop/ros2-sidecar` first, then merge back to `robot-stable` after
testing on real hardware.

The platform:
- **Hardware**: NVIDIA Jetson Orin Nano 4GB, 4 mecanum wheels, 4-joint
  arm with vacuum gripper, 2 Aveo SP2812 cameras, 2 IR sensors.
- **Software**: 7 rclcpp nodes + `ros2_control` hardware interface, all
  under `ROS_DOMAIN_ID=42`.
- **Network**: this Jetson is `192.168.3.69` (Wi-Fi `wlP1p1s0`); dev
  boxes connect from `192.168.3.50 ~ 200`.

## Architecture (7 nodes live here)

| Node | Subscribes | Publishes | Rate |
|------|------------|-----------|------|
| `camera_node` (×2: front, arm) | — | 5 streams per camera (see below) | 30 Hz image, 1 Hz status |
| `infrared_node` (×2: left, right) | — | `/vehicle_wbt/v1/sensors/ir/<id>` | 20 Hz |
| `mecanum_chassis_node` | `/cmd/vel_safe` | `/state/odom`, `/tf` | 50 Hz |
| `arm_node` | `/cmd/arm/main/trajectory` | `/state/actuators/main` | 50 Hz |
| `safety_gate_node` | `/cmd/vel_raw` + `/safety/*` | `/cmd/vel_safe` | continuous |
| `MC602HardwareInterface` (ros2_control plugin) | controller_manager | wheel state | |
| `mission_runner_node` (added Phase 1.5) | task_list param | per-task state | |

Each `camera_node` publishes `/tf_static: base_link → <id>_camera_optical_frame`.

## Topic namespace

**ALL** topics under `/vehicle_wbt/v1/...`. Enforced by
`config_loader.py` and the C++ nodes. Adding a topic outside this prefix
is a violation.

### Camera schema (5 streams per camera, locked commit `82fc1d6`)
Under `/vehicle_wbt/v1/sensors/camera/<id>/`:
- `image_raw`         — `sensor_msgs/Image` (bgr8)        QoS: BEST_EFFORT depth=1, 30 Hz
- `image_compressed`  — `sensor_msgs/CompressedImage`      QoS: BEST_EFFORT depth=1, 30 Hz, JPEG q=85
- `camera_info`       — `sensor_msgs/CameraInfo`           QoS: TRANSIENT_LOCAL, only if YAML has real K (else NOT published)
- `camera_status`     — `diagnostic_msgs/DiagnosticArray`  QoS: RELIABLE, 1 Hz (OK / WARN / ERROR)
- `camera_meta`       — `vehicle_wbt_platform_cpp/msg/CameraMeta` (custom)  QoS: RELIABLE, 1 Hz

`camera_info_manager::CameraInfoManager` loads `params/camera_<id>.yaml`
at startup; all-zero K means "no calibration yet" → `camera_info` is
**not** published (NEVER fake intrinsics).

## Repository layout (this branch)

```
rak-car/  (you are here, on robot-stable)
├── ros2_ws/src/
│   ├── vehicle_wbt_platform_cpp/    # C++: 7 rclcpp nodes + ros2_control plugin + custom msgs
│   └── vehicle_wbt_platform/        # Python: config_loader, SidecarOrchestrator, ENABLE_ROS2 gate
├── config_sensors.yml               # hardware source of truth
├── urdf/vehicle_wbt.urdf.xacro      # robot description
├── scripts/calibrate_camera.py      # operator-only: lens swap → re-calibrate
├── calibration_session/             # saved calibration artifacts
├── CLAUDE.md                        # this file
└── README.md                        # quick-start for this robot
```

## Conventions (load-bearing — read before editing)

### No mocks in production code
If a sensor source is missing, the node must `throw` with a clear error
and die. NEVER publish synthetic frames or plausible-looking fakes
(NaN/null is acceptable; plausible numbers are not).

### No hardcoded paths in source
`/home/<user>/...`, IPs, usernames are FORBIDDEN in source. Per-machine
`/dev/cam*N*` allowed as launch-arg defaults but NEVER the only way to
configure. Calibration YAMLs come from
`package://vehicle_wbt_platform_cpp/params/camera_<id>.yaml`.

### Other rules
- Never `eval()` LLM output.
- Never bare `except:` — use `except Exception as e:` with logging.
- Never `while True: time.sleep(1)` to mask errors — raise or return codes.
- Never hardcode API keys — use env vars or `.env`.

### ENABLE_ROS2 gate
`os.environ["ENABLE_ROS2"]` controls whether the Python sidecar is
active. When unset, `__main__.py` returns 0 immediately without
importing rclpy. Main behavior is byte-identical to pre-sidecar state.

## Hardware quirks (memorized 2026-07-08)

### Aveo SP2812 cameras (vendor `1871:0110`)
On this dev box: `/dev/cam4` (front), `/dev/cam3` (arm). They only
advertise **MJPG** in `v4l2-ctl --list-formats-ext`. OpenCV's `CAP_V4L2`
defaults to negotiating YUYV → `select()` times out forever. The fix is
`cap_->set(CAP_PROP_FOURCC, fourcc('M','J','P','G'))` **before**
`cap_->open()`. Do NOT set `CAP_PROP_FPS` for UVC cams (can stall driver).

`udev` rule `/etc/udev/rules.d/99-usbvideo.rules` maps USB `devpath` →
`/dev/cam<N>` symlinks. Without this, `front_device:=/dev/cam4` doesn't
resolve.

## Build / run on this robot

```bash
# Build (after pulling new code from origin/robot-stable)
cd ~/workspace/rak-car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_wbt_platform_cpp vehicle_wbt_platform

# Hardware launch (real cameras, real motors)
source install/setup.bash
ros2 launch vehicle_wbt_platform_cpp full_system.launch.py \
    front_device:=/dev/cam4 \
    arm_device:=/dev/cam3
# ROS_DOMAIN_ID=42 is set inside the launch file; do NOT export manually

# Mock launch (sensor side real, motor side stubbed)
ros2 launch vehicle_wbt_platform_cpp mock_system.launch.py
```

## Daily workflow on this robot

```bash
# 1. Pull latest from origin (after dev box pushes to robot-stable)
cd ~/workspace/rak-car && git pull --ff-only

# 2. Rebuild only what changed
cd ros2_ws && colcon build --packages-up-to vehicle_wbt_platform_cpp vehicle_wbt_platform

# 3. Restart the launch
pkill -f full_system.launch.py     # or mock_system.launch.py
source install/setup.bash
ros2 launch vehicle_wbt_platform_cpp full_system.launch.py \
    front_device:=/dev/cam4 arm_device:=/dev/cam3
```

## Merging dev work into robot-stable

When the dev box has new code ready for robot testing:
```bash
# On this Jetson:
cd ~/workspace/rak-car
git fetch origin
git checkout robot-stable
git merge --no-ff origin/develop/ros2-sidecar
# resolve any conflicts, then rebuild + smoke test
git push origin robot-stable
```

## Calibration (operator runs once after lens/sensor swap)

```bash
# Headless (no GUI):
python3 scripts/calibrate_camera.py /path/to/chessboard_*.png 8 6 0.025 \
    --out ros2_ws/install/vehicle_wbt_platform_cpp/share/vehicle_wbt_platform_cpp/params/camera_front.yaml

# Interactive (if you have GUI):
ros2 run camera_calibration cameracalibrator.py --size 8x6 --square 0.025 \
    image:=/vehicle_wbt/v1/sensors/camera/front/image_raw

# Then restart the launch — camera_info will start publishing.
```

## CLI quick reference

```bash
# After sourcing install/setup.bash:
ros2 topic list                                       # all topics
ros2 topic hz /vehicle_wbt/v1/sensors/camera/front/image_raw  # ~30 Hz
ros2 topic echo /vehicle_wbt/v1/sensors/camera/front/camera_status --once
ros2 node list
ros2 node info /camera_front

# Stop everything:
pkill -f full_system.launch.py
```

## Song playback (Happy Birthday on MC602 buzzer)

```bash
# Terminal A — launch bridge + mc602 peripheral node:
source /opt/ros/humble/setup.bash
cd ~/workspace/rak-car/ros2_ws
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch vehicle_wbt_smartcar_bridge \
    smartcar_bridge.launch.py \
    serial_port:=/dev/ttyUSB1 baud:=1000000

# Terminal B — trigger (ROS_DOMAIN_ID must match):
ROS_DOMAIN_ID=42 ros2 topic pub --once \
    /vehicle_wbt/v1/cmd/peripheral/beep_event std_msgs/Empty

# Expected: ~17 s of Happy Birthday through the MC602 buzzer.
# Node logs show `melody[0..24] f=NNNHz d=0.Ns: sent 77 68 ...`.
# Override `serial_port` if your MC602 is at /dev/ttyUSB0 (or other).
```