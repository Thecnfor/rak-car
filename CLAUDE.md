# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **New here?** Start with [README.md](README.md) for project context, then
> [docs/onboarding/README.md](docs/onboarding/README.md) for day-1 setup
> (4-command onboarding). Then read [docs/team-constants.md](docs/team-constants.md)
> for the hard team conventions. Come back to this file once you start coding.

> **Using Claude Code to help develop?** Read
> [docs/claude-code-workflow.md](docs/claude-code-workflow.md) for the
> project's preferred prompt patterns and tool conventions. The 1-line
> TL;DR of this whole file is: **never touch `cyclonedds.xml`, never
> modify `start_team_rviz.sh` behavior, run `bash scripts/diagnose.sh`
> before claiming anything works**.

## Project Overview

**vehicle_wbt** is a ROS2 Humble autonomous vehicle robot. It runs on NVIDIA Jetson Orin Nano (4GB) and is observed/controlled via standard ROS2 topics, services, and ros2_control hardware interfaces. The platform performs lane following, object detection, robotic-arm manipulation, and AI-assisted task execution.

Two active branches:
- `main` — frozen for 2026-08-10 → 08-12 competition. Legacy Python+ZMQ stack.
- `develop/ros2-sidecar` — the ROS2 future. Pure rclcpp/rclpy implementation. **Most development happens here.** After 2026-08-12 this branch merges to main per `docs/contributing/branch-strategy.md`.

The full platform rationale (why move from ZMQ to DDS) is in the root `README.md`. The 1885-line architecture spec is in `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md`.

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
| `camera_node` (×2: front, arm) | — | `image_raw`+`image_compressed`+`camera_status`+`camera_meta` per camera | 10 Hz image, 1 Hz status |
| `infrared_node` (×2: left, right) | — | `/vehicle_wbt/v1/sensors/ir/<id>` | 20 Hz |
| `mecanum_chassis_node` | `/cmd/vel_safe` | `/state/odom`, `/tf` | 50 Hz |
| `arm_node` | `/cmd/arm/main/trajectory` | `/state/actuators/main` | 50 Hz |
| `safety_gate_node` | `/cmd/vel_raw` + `/safety/*` | `/cmd/vel_safe` | continuous |
| `MC602HardwareInterface` (ros2_control plugin) | controller_manager | wheel state |  |
| `mission_runner_node` (added Phase 1.5) | task_list param | per-task state |  |

Each `camera_node` also publishes `/tf_static: base_link → <id>_camera_optical_frame` so tf2 consumers can place the camera in robot space.

## Repository structure

`config_sensors.yml` at the repo root is the **single source of truth** for what hardware is wired — both Python loader and C++ nodes key off it.

See root [README.md](README.md) for the complete directory layout. See [docs/README.md](docs/README.md) for the doc index. See [docs/team-constants.md](docs/team-constants.md) for hard team conventions (Jetson IP, ROS_DOMAIN_ID, etc.).

## Conventions (load-bearing — read before editing)

### Topic namespace
**ALL** topics under `/vehicle_wbt/v1/...`. Enforced by `config_loader.py` (rejects anything else) and the C++ nodes. Adding a topic outside this prefix is a violation.

### Adding new hardware
1. Add 1 entry to `config_sensors.yml` (sensors or actuators list).
2. Add 1 link to `urdf/vehicle_wbt.urdf.xacro`.
3. (Optional) Add reserved topic namespace in the spec.

**Zero business code touched.** This is the platform-level abstraction principle (documented in CONTRIBUTING.md).

### ENABLE_ROS2 gate
`os.environ["ENABLE_ROS2"]` controls whether the Python sidecar is active. When unset, `__main__.py` returns 0 immediately without importing rclpy. **Main behavior is byte-identical to pre-sidecar state.**

### Camera topic schema (5 streams, per camera)
Locked in by commit `82fc1d6`. Each camera under `/vehicle_wbt/v1/sensors/camera/<id>/`:
- `image_raw`         — `sensor_msgs/Image` (bgr8)        QoS: BEST_EFFORT depth=1, 10 Hz
- `image_compressed`  — `sensor_msgs/CompressedImage`      QoS: BEST_EFFORT depth=1, 10 Hz, JPEG q=85
- `camera_info`       — `sensor_msgs/CameraInfo`           QoS: TRANSIENT_LOCAL, only if YAML has real K (else NOT published)
- `camera_status`     — `diagnostic_msgs/DiagnosticArray`  QoS: RELIABLE, 1 Hz (OK / WARN / ERROR)
- `camera_meta`       — `vehicle_wbt_platform_cpp/msg/CameraMeta` (custom)  QoS: RELIABLE, 1 Hz

`camera_info_manager::CameraInfoManager` loads `params/camera_<id>.yaml` at startup; all-zero K means "no calibration yet" → `camera_info` topic is **not** published (NEVER fake intrinsics).

### Camera hardware quirks (memorized 2026-07-08)
The Aveo SP2812 cameras (vendor `1871:0110`, on this dev box `/dev/cam3` and `/dev/cam4`) only advertise **MJPG** in `v4l2-ctl --list-formats-ext`. OpenCV's `CAP_V4L2` defaults to negotiating YUYV → `select()` times out forever. The fix is `cap_->set(CAP_PROP_FOURCC, fourcc('M','J','P','G'))` **before** `cap_->open()`. Do NOT set `CAP_PROP_FPS` for UVC cams (can stall driver).

`udev` rule file (`/etc/udev/rules.d/99-usbvideo.rules`) maps USB `devpath` → `/dev/cam<N>` symlinks. Without this, `front_device:=/dev/cam4` doesn't resolve to anything.

## Critical warnings

These are non-negotiable. Each is anchored in real bugs we hit, not generic advice.

1. **No mocks in production code** — see `memory/coding-rules-no-mocks.md` (also loaded via `/home/xrak/.claude/projects/-home-xrak-workspace-rak-car/memory/coding-rules-no-mocks.md`). If a sensor source is missing, the node must `throw` with a clear error and die; never silently publish synthetic frames or plausible-looking fakes (NaN/null is acceptable; plausible numbers are not).
2. **No hardcoded paths in source** — `/home/<user>/...`, IPs, usernames. Per-machine /dev/cam*N* allowed as launch-arg defaults but NEVER as the only way to configure. Calibration YAMLs come from `package://vehicle_wbt_platform_cpp/params/camera_<id>.yaml`.
3. **Never `eval()` LLM output** (legacy rule) — `ernie_bot/base/answer.py` style vulnerabilities are out of scope but the principle stands.
4. **Never bare `except:`** — use `except Exception as e:` with logging.
5. **Never `while True: time.sleep(1)`** to mask errors** — raise or return error codes.
6. **Never hardcode API keys** — use env vars or `.env`.

## Build / test commands

```bash
# Build everything
cd ros2_ws && colcon build --packages-up-to vehicle_wbt_platform_cpp vehicle_wbt_platform

# Build with tests enabled (CI does this)
cd ros2_ws && colcon build --packages-up-to vehicle_wbt_platform_cpp \
                      --cmake-args -DBUILD_TESTING=ON

# Python tests (no ROS2 needed, fast — < 0.5s)
cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python3 -m pytest test/ -q
# → 45/45 pass

# C++ tests (gtest, requires ROS2 Humble)
cd ros2_ws && colcon test --packages-select vehicle_wbt_platform_cpp
# → 5 binaries, 58 testcases total, 0 failures

# Single gtest binary (handy for iteration)
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/vehicle_wbt_platform_cpp/test_safety_gate_logic
./build/vehicle_wbt_platform_cpp/test_base_task
# (one binary per test/ file)

# Dev mode (no hardware): mock launch
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vehicle_wbt_platform_cpp mock_system.launch.py
# → 5 nodes spin up; verify in another terminal with RViz2 / `ros2 topic list`

# Hardware mode (Jetson target, or this dev box with real cameras):
ros2 launch vehicle_wbt_platform_cpp full_system.launch.py \
    front_device:=/dev/cam4 \
    arm_device:=/dev/cam3
# Both device paths are launch args; per-machine override.

# Calibration (operator runs once after lens/sensor swap)
# See scripts/README.md for the full workflow.
# Interactive: ros2 run camera_calibration cameracalibrator.py --size 8x6 --square 0.025 \
#   image:=/vehicle_wbt/v1/sensors/camera/front/image_raw
# Headless:  python3 scripts/calibrate_camera.py /path/to/*.png 8 6 0.025 \
#             --out /path/to/install/.../params/camera_front.yaml
```

## Onboarding & diagnostics scripts

> 团队成员日常工具。详见 [scripts/README.md](scripts/README.md) 和 [docs/onboarding/README.md](docs/onboarding/README.md)。

```bash
# 新成员 onboarding（克隆后跑一次）
bash scripts/onboard.sh                  # 一键装依赖 + build + 验证

# 现场健康检查（出问题 / 赛前）
bash scripts/diagnose.sh                 # 15 项检查（dev + Jetson + DDS）
bash scripts/diagnose.sh --json         # JSON 输出（CI 用）
bash scripts/diagnose.sh --no-remote    # 只查 dev 端
```

## Daily dev workflow

```bash
# 1. Edit
$EDITOR ros2_ws/src/...              # or scripts/, config_sensors.yml, urdf/

# 2. Run fast unit tests (Python, < 0.5s)
cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python3 -m pytest test/ -q

# 3. Full build + test when C++ touched
cd ros2_ws && colcon build --cmake-args -DBUILD_TESTING=ON \
    && colcon test --packages-select vehicle_wbt_platform_cpp

# 4. Smoke test with mock system (no hardware)
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch vehicle_wbt_platform_cpp mock_system.launch.py
# In another shell: ros2 topic list  (should see /vehicle_wbt/v1/*)
#                    ros2 node list   (should see 5 nodes)

# 5. Push to Jetson for real-hardware test
#    IMPORTANT: dev install/ is **not** ABI-compatible with Jetson (Humble).
#    Only push source code. Jetson side does its own `colcon build` under
#    Humble. See docs/team-constants.md for "Jetson vs dev" notes.
git push origin develop/ros2-sidecar
ssh xrak@192.168.3.69 "cd ~/workspace/rak-car/ros2_ws && git pull && colcon build --packages-up-to vehicle_wbt_platform_cpp"
ssh xrak@192.168.3.69 "ros2 launch vehicle_wbt_platform_cpp full_system.launch.py"
```

## CI / branch strategy

- CI runs on push and PR to `develop/ros2-sidecar`, `develop/ros2-humble-post-flash`, `main`. See `.github/workflows/ci.yml`.
- Jobs: `lint-py` (flake8), `test-py` (pytest), `test-cpp` (colcon gtest in `ros:humble-ros-base`), `cpp-lint` (ament_lint_common, non-blocking), `xacro-check` (URDF validates).
- `main` is frozen for the competition window (2026-07-13 → 2026-08-12). Do not merge new work into it during that period. See `docs/contributing/branch-strategy.md`.

## CLI quick reference

```bash
# After sourcing setup.bash:
ros2 topic list                                    # all topics
ros2 topic info /vehicle_wbt/v1/sensors/camera/front/image_raw --verbose   # QoS
ros2 topic hz /vehicle_wbt/v1/sensors/camera/front/image_raw            # publish rate
ros2 topic echo /vehicle_wbt/v1/sensors/camera/front/camera_status --once # status keys

ros2 node list
ros2 node info /camera_front

# Live image in a terminal (no RViz):
ros2 run rqt_image_view rqt_image_view    # GUI; for headless see `image_view` or RViz
```


## Claude Code 工具链

完整使用指南（4 个 prompt 模板 / memory 体系 / 故障排除 / 进阶）见
[docs/claude-code-workflow.md](docs/claude-code-workflow.md)。
