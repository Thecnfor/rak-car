# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**vehicle_wbt** is a ROS2 Humble-based autonomous vehicle robot. It runs on NVIDIA Jetson Orin Nano (4GB) and is observed/controlled via standard ROS2 topics, services, and ros2_control hardware interfaces. The system performs lane following, object detection, robotic arm manipulation, and AI-assisted task execution.

**Branch**: This branch (`develop/ros2-sidecar`) is the ROS2 future. The legacy Python+ZMQ stack on `main` is frozen for the 8.10-8.12 competition.

## Architecture: dev/target dual-machine

- **Dev desktop** (Ubuntu 22.04/24.04/26.04 + ROS2 desktop): editing, testing, simulation, RViz visualization
- **Target** (Jetson Orin Nano 4GB, JetPack 6 + ROS2 Humble base): real hardware I/O, sidecar nodes, no GUI
- SSH: `ssh xrak@orin` (orin hostname)
- Both communicate via ROS_DOMAIN_ID=42 over DDS
- See [`docs/development/README.md`](docs/development/README.md) for the complete dev/target workflow

## Repository structure

This branch is a **complete ROS2 project**. All legacy Python/ZMQ code has been removed.

```
ros2_ws/src/
├── vehicle_wbt_platform_cpp/      # C++ core (5 rclcpp nodes + ros2_control plugin)
│   ├── include/                   # public headers (BaseController, MecanumChassis, ...)
│   ├── src/                       # 5 rclcpp nodes + MecanumChassis + MC602Adapter + ros2_control plugin
│   ├── msg/                       # custom messages (LaneResult, DetectionArray, ActuatorState)
│   ├── launch/                    # full_system.launch.py + mock_system.launch.py
│   ├── config/                    # cyclonedds.xml + safety.yaml
│   ├── urdf/                      # robot description
│   ├── test/                      # gtest
│   └── CMakeLists.txt
└── vehicle_wbt_platform/          # Python orchestrator (config_loader, SidecarOrchestrator)
    ├── vehicle_wbt_platform/
    │   ├── config_loader.py        # strict schema validation
    │   ├── component_base.py       # BaseComponent + ComponentContext
    │   ├── controller_base.py      # BaseControllerHardwareInterface
    │   ├── mc602_adapter.py        # MC602Adapter (Python fallback)
    │   ├── chassis_base.py         # BaseChassis
    │   ├── orchestrator.py         # SidecarOrchestrator
    │   └── __main__.py             # ENABLE_ROS2 gate
    └── test/                       # pytest (45 tests, all passing)
```

`config_sensors.yml` at repo root is the **single source of truth** for what hardware is wired.

## Key conventions

### Topic namespace

**ALL** ROS2 topics under `/vehicle_wbt/v1/...`. The namespace is enforced by:
- `config_loader.py` (Python): rejects topics not starting with `/vehicle_wbt/v1/`
- The C++ sidecar publishes to the same namespace

### Adding new hardware

1. Add 1 line to `config_sensors.yml`
2. Add 1 link to `urdf/vehicle_wbt.urdf.xacro`
3. (Optional) Add reserved topic namespace in the spec

**Zero business code touched.** This is the platform-level abstraction principle.

### ENABLE_ROS2 gate

`os.environ["ENABLE_ROS2"]` controls whether the sidecar is active. When unset, `__main__.py` returns 0 immediately without importing rclpy. **Main behavior is byte-identical to pre-sidecar state.**

## Critical warnings (carried over from legacy)

The following still apply even after the rewrite:

1. **Never `eval()` LLM output** (legacy CLAUDE.md rule) — `ernie_bot/base/answer.py` style vulnerabilities are out of scope now but the principle stands.
2. **Never bare `except:`** — use `except Exception as e:` with logging.
3. **Never `while True: time.sleep(1)`** to mask errors — raise or return error codes.
4. **Never hardcode API keys** — use env vars or `.env`.
5. **Never `eval(chassis_type)`** — use dict lookup.

## Build / test commands

```bash
# Build
cd ros2_ws && colcon build --packages-up-to vehicle_wbt_platform_cpp vehicle_wbt_platform

# Test (Python, no ROS2 required)
cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python3 -m pytest test/ -v
# → 45/45 pass

# Test (C++, requires ROS2 Humble)
cd ros2_ws && colcon test --packages-select vehicle_wbt_platform_cpp
# → 25 gtest cases

# Run mock system (dev, no hardware)
source /opt/ros/humble/setup.bash  # or jazzy
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch vehicle_wbt_platform_cpp mock_system.launch.py

# Run full system (Jetson, real hardware)
ssh xrak@orin
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch vehicle_wbt_platform_cpp full_system.launch.py
```

## Daily dev workflow

```bash
# 1. Edit
$EDITOR ros2_ws/src/...

# 2. Run unit tests (dev, < 1s)
cd ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python3 -m pytest test/ -q

# 3. Run integration tests (dev, with mock hardware)
cd ros2_ws && colcon build
ros2 launch vehicle_wbt_platform_cpp mock_system.launch.py
# → In another terminal: rviz2 to see published topics

# 4. Push to Jetson for real-hardware test
git push origin develop/ros2-sidecar
ssh orin "cd ~/ros2_ws && git pull && colcon build"
ssh orin "ros2 launch vehicle_wbt_platform_cpp full_system.launch.py"

# 5. dev: subscribe to Jetson topics via DDS
export ROS_DOMAIN_ID=42
ros2 topic list
rviz2
```

## Competition window (2026-08-10 to 08-12)

- `main` branch is **frozen** for the competition
- `develop/ros2-sidecar` continues post-competition work
- After 2026-08-12, main can merge from develop/ros2-sidecar (after Plan B/D)
- See [`docs/contributing/branch-strategy.md`](docs/contributing/branch-strategy.md)
