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

Two active branches:
- `main` — frozen for 2026-08-10 → 08-12 competition. Legacy Python+ZMQ stack.
- `develop/ros2-sidecar` — the ROS2 future. Pure rclcpp/rclpy implementation. **Most development happens here.** After 2026-08-12 this branch merges to main per `docs/contributing/branch-strategy.md`.
- `robot-stable` — slim runtime that lives **on the Jetson itself** (IP `192.168.3.69`). Stripped of all dev-only stuff at commit `30f9620`. Keeps only `ros2_ws/`, `config_sensors.yml`, `urdf/`, and `scripts/calibrate_camera.py`. Drivers + minimal app that publishes the `/vehicle_wbt/v1/...` contract to dev desktops over DDS. **New contributors: never commit here directly — push to `develop/ros2-sidecar` first**, then Thecnfor cherry-picks to robot-stable after a real-hardware smoke test. Full split rationale and merge rules: [`docs/driver-app-interface.md`](docs/driver-app-interface.md) and [`docs/contributing/branch-strategy.md`](docs/contributing/branch-strategy.md).

The full platform rationale (why move from ZMQ to DDS) is in the root `README.md`. The 1885-line architecture spec is in `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md`.

## 团队开发约定 (any Claude must internalize)

> **这 3 条是整个团队的开发模式。** 任何 dev 机和 Jetson 都在同一个局域网下；任何 team member 都在自己 dev 机上开发。

1. **LAN 共享 + Jetson IP 永远是 `192.168.3.69`**
   - 全队硬约定，dev 机 / Jetson / 文档 / 脚本同步维护（改它 = 改全队）
   - dev 机连上团队 Wi-Fi/路由器，DHCP 自动落在 `192.168.3.50 ~ 192.168.3.200`
   - 验证：`ping -c 3 192.168.3.69` 通即可
   - 详 [`docs/team-constants.md`](docs/team-constants.md)

2. **Jetson 端实际在发什么话题 → team member 自己用 RViz 看（不假设）**
   - **不要凭 spec / 代码推断**话题名、类型、频率、QoS
   - 标准做法（让 team member 跑）：
     ```bash
     bash scripts/start_team_rviz.sh    # 一键 RViz2 看相机 + 列所有 /vehicle_wbt/v1/... 话题
     # 或纯 CLI：
     ros2 topic list
     ros2 topic info /vehicle_wbt/v1/sensors/camera/front/image_compressed --verbose
     ros2 topic hz /vehicle_wbt/v1/sensors/camera/front/image_compressed
     ```
   - `config_sensors.yml` 是话题的"权威清单"，但**实时状态**（是否在发、实际频率、QoS 兼容性）必须现场查
   - 帮 team member 加新功能前，先确认 Jetson 端对应话题已发 + QoS 兼容

3. **开发 + 测试永远在自己 dev 机上做（不直接动 Jetson）**
   - 代码改完 → dev 端 `colcon build`（Jetson 端 ABI 不兼容，**不要 push install/**）
   - 无硬件 smoke test → dev 端 `ros2 launch ... mock_system.launch.py`
   - 真机联调 → dev 端订阅 Jetson 的话题（同 `ROS_DOMAIN_ID=42` 自动发现），dev 端发指令、Jetson 端节点执行
   - 真机部署 → `git push` + ssh Jetson `colcon build` + ssh Jetson launch
   - 详下文 "Daily dev workflow"

## Architecture: dev/target dual-machine

| Dev desktop (Ubuntu 22.04+ + ROS2 Humble) | Target (Jetson Orin Nano 4GB) |
|---|---|
| Edit, simulation (RViz2, Gazebo), CI, linters | Real hardware I/O — no GUI |
| `ROS_DOMAIN_ID=42` | `ROS_DOMAIN_ID=42` |
| Discovers Jetson topics via DDS over LAN | Publishes sensors on real hardware |

Both machines MUST share `ROS_DOMAIN_ID=42`. The `full_system.launch.py` enforces this via `SetEnvironmentVariable`; you don't need to export it manually.

The 7 rclcpp nodes (live today):

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
│   ├── vehicle_wbt_platform/        # Python: config_loader, SidecarOrchestrator, ENABLE_ROS2 gate
│   ├── vehicle_wbt_smartcar_hw/         # Phase 1: MC602 protocol layer (pure Python lib; 14 device classes SDK 1:1)
│   ├── vehicle_wbt_smartcar_msgs/        # Phase 1: 12 new .srv + 1 new .msg (additive; legacy 19 .srv + 3 .msg preserved)
│   ├── vehicle_wbt_smartcar_bridge/      # Phase 1: mc602_node (12 service + 2 topic gateway) + smartcar_bridge_node (legacy compat)
│   ├── vehicle_wbt_smartcar_sdk/         # Legacy: high-level MyCar API for old chassis/arm/shooter code
│   └── vehicle_wbt_smartcar_chassis/     # Phase 1.5+: 4-colleague dev (chassis)
│       ├── vehicle_wbt_smartcar_arm/         # 4-colleague dev (arm)
│       ├── vehicle_wbt_smartcar_shooter/     # 4-colleague dev (shooter)
│       └── vehicle_wbt_smartcar_perception/  # 4-colleague dev (LLM/vision)
├── deploy/                              # Phase 1: systemd unit + env + DDS + udev (T16 installs to /etc/...)
├── docs/integration/                    # Phase 1: LOWLEVEL_API.md + DEV_QUICKSTART.md
├── scripts/                             # Phase 1: quick_beep.sh + check_link.sh (dev-box verification)
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

# Phase 1: MC602 IO gateway (under development; runs alongside above)
# systemd unit `vehicle-wbt-mc602` will start this on boot
# To start manually: ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py serial_port:=/dev/ttyUSB1 baud:=1000000
```

## API for dev boxes (LAN collaborators)

Phase 1: Jetson exposes 12 generic services + 2 topics via `/vehicle_wbt/v1/mc602/*` for 4 colleagues (chassis/arm/shooter/perception) developing on LAN dev boxes. They DO NOT SSH Jetson or deploy code here — they:

1. `git clone` + `colcon build --packages-up-to vehicle_wbt_smartcar_msgs` (only msgs needed; no hw/bridge)
2. `export ROS_DOMAIN_ID=42`
3. `./scripts/quick_beep.sh` (trust signal: hear beep on Jetson → bridge works)
4. Develop their own business package under `ros2_ws/src/vehicle_wbt_smartcar_<their>/`
5. `colcon build --packages-select <theirs>` + `ros2 run ...` → DDS auto-discovers Jetson's `/vehicle_wbt/v1/mc602/*`

Full API + Python rclpy examples: `docs/integration/LOWLEVEL_API.md`
5-min onboarding: `docs/integration/DEV_QUICKSTART.md`

**Key invariants:**
- Dev boxes MUST `export ROS_DOMAIN_ID=42` (or set in `~/.bashrc`)
- Dev boxes use CycloneDDS: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Service/topic paths are under `/vehicle_wbt/v1/mc602/*` (NOT bare `/mc602/*`)
- Old C++ `ros2_control` stack (`vehicle_wbt_platform_cpp`) still runs in parallel during transition — colleagues should ignore `/cmd/vel_safe`, `/state/odom`, etc. and use only `/vehicle_wbt/v1/mc602/*` during Phase 1. Old stack will be retired in Phase 4.

Two launch files coexist:
- `smartcar_bridge.launch.py` (legacy) — runs `smartcar_bridge_node` for `vehicle_wbt_smartcar_sdk` users
- `mc602.launch.py` (Phase 1) — runs `mc602_io` for the new gateway

Both can run simultaneously (different node names, no conflicts).

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

# Phase 1 alternative: use the new mc602_node gateway (also supports beep via /vehicle_wbt/v1/mc602/buzzer)
# Terminal A:
ROS_DOMAIN_ID=42 ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py \
    serial_port:=/dev/ttyUSB1 baud:=1000000
# Terminal B (or any dev box with ROS_DOMAIN_ID=42):
./scripts/quick_beep.sh 880 300
# Expected: 0.3 s 880Hz beep (single tone, not the full melody)
```