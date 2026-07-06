# vehicle_wbt — ROS2 Platform

> **An autonomous vehicle robot built on ROS2 Humble** — runs on NVIDIA Jetson Orin Nano, controlled by 4 mecanum wheels + a 4-joint arm with vacuum gripper, observed through 2 cameras and 2 IR sensors.

**Branch**: `develop/ros2-sidecar` (this branch is the post-competition ROS2 future; `main` keeps the legacy Python+ZMQ stack for the 8.10-8.12 competition).

## Why ROS2?

This branch is a complete rewrite of the project's runtime architecture on top of ROS2 Humble. The legacy stack (ZMQ inference + direct function calls in a 1438-line `car_wrap.py` God Object) worked for the 2026 competition but does not scale:

- **Single-machine bottleneck** — only one Jetson can test; 4-6 developers queued
- **No observability** — debugging required `print()` + post-hoc log grepping
- **No simulation** — every test required real hardware
- **No type safety across modules** — `import vehicle` triggers serial-port scan; any change can silently break a controller

ROS2 fixes all of these with topics, services, ros2_control, ros2 bag, RViz2, Gazebo.

## Quick start

### Dev machine (any Linux, any ROS2 version)

```bash
# 1. Install ROS2 (Jazzy recommended, or Humble)
#    See docs/development/dev-machine-setup.md for Docker + bare-metal instructions

# 2. Clone and build
git clone https://github.com/Thecnfor/rak-car.git
cd rak-car
# Edit src/vehicle_wbt_platform_cpp/config/cyclonedds.xml if needed
colcon build --packages-up-to vehicle_wbt_platform_cpp vehicle_wbt_platform

# 3. Source + run mock system
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch vehicle_wbt_platform_cpp mock_system.launch.py
# → 5 nodes spin up; view in RViz2
rviz2
```

### Target machine (Jetson Orin Nano 4GB, real hardware)

```bash
ssh xrak@orin
sudo apt install -y python3-colcon-common-extensions  # one-time
cd ~/ros2_ws/src  # rsync or git clone from dev
cd ~/ros2_ws && colcon build --packages-up-to vehicle_wbt_platform_cpp
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch vehicle_wbt_platform_cpp full_system.launch.py
```

## Architecture

The platform is built around a **dev/target split** (detailed in [`docs/development/README.md`](docs/development/README.md)):

```
Dev desktop (Ubuntu + ROS2 desktop)           Target (Jetson Orin Nano)
  ┌─────────────────────────────┐            ┌──────────────────────┐
  │ rclcpp + rclpy              │            │ rclcpp + rclpy        │
  │ RViz2 + Gazebo + ros2 bag   │  ROS2 DDS  │ No GUI (4GB RAM)      │
  │ Tests + linters + CI        │  ────────► │ Real hardware I/O     │
  │ Edit + iterate              │            │ Publish sensors       │
  └─────────────────────────────┘            └──────────────────────┘
```

Components run as **rclcpp nodes** under `ros2_control`:

| Node | Subscribes | Publishes |
|------|------------|-----------|
| `camera_node` (×2) | — | `/vehicle_wbt/v1/sensors/camera/<id>/image_raw` |
| `infrared_node` (×2) | — | `/vehicle_wbt/v1/sensors/ir/<id>` |
| `mecanum_chassis_node` | `/cmd/vel_safe` | `/state/odom`, `/tf` |
| `arm_node` | `/cmd/arm/trajectory` | `/state/actuators/<id>` |
| `safety_gate_node` | `/cmd/vel_raw`, `/safety/*` | `/cmd/vel_safe` |
| `MC602HardwareInterface` (ros2_control) | — | wheel/arm state |

## Repository layout

```
rak-car/
├── ros2_ws/                              # colcon workspace
│   └── src/
│       ├── vehicle_wbt_platform_cpp/     # C++ core (5 rclcpp nodes + ros2_control plugin)
│       │   ├── include/                  # public headers (BaseController, BaseChassis, MecanumChassis, ...)
│       │   ├── src/                      # implementations + 5 rclcpp node .cpp
│       │   ├── msg/                      # LaneResult, DetectionArray, ActuatorState
│       │   ├── launch/                   # full_system.launch.py + mock_system.launch.py
│       │   ├── config/                   # cyclonedds.xml + safety.yaml
│       │   ├── urdf/                     # vehicle_wbt.urdf.xacro + README
│       │   ├── test/                     # gtest (3 files)
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       └── vehicle_wbt_platform/         # Python orchestrator (config_loader, SidecarOrchestrator, __main__)
│           ├── test/                     # pytest (5 files, 45 tests)
│           └── ...
├── docs/                                 # comprehensive documentation
│   ├── README.md                         # doc index
│   ├── architecture.md                   # 6-layer architecture
│   ├── hardware-port-mapping.md          # M口/S口/P口/步进 物理映射 (PR #5)
│   ├── adr/                              # architecture decision records
│   ├── development/                      # dev/target workflow (5 docs)
│   ├── migration/                        # JetPack 6 migration (already done!)
│   ├── contributing/                     # branch strategy
│   └── superpowers/                      # spec + plan (design docs)
├── config_sensors.yml                    # sensors/actuators registry (single source of truth)
├── urdf/                                 # symlink to ros2_ws/.../urdf/
├── .github/workflows/                    # CI
├── .devcontainer/                       # dev container (Docker)
├── CLAUDE.md                             # Claude Code guidance
├── CONTRIBUTING.md                       # contribution guide
├── LICENSE                               # license
└── README.md                             # this file
```

## Documentation

| Doc | Purpose |
|-----|---------|
| [docs/README.md](docs/README.md) | Full doc index |
| [docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md](docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md) | 1885-line platform spec |
| [docs/development/README.md](docs/development/README.md) | dev + target dual-machine workflow |
| [docs/development/jetson-target-setup.md](docs/development/jetson-target-setup.md) | Orin Nano 4GB setup |
| [docs/adr/](docs/adr/) | Architecture decision records |
| [docs/hardware-port-mapping.md](docs/hardware-port-mapping.md) | M口/S口/P口/步进 physical port mapping |

## License

See [LICENSE](LICENSE).
