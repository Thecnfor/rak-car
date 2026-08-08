# rak — ROS2 Platform

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
# Edit src/bringup/config/cyclonedds.xml if needed
colcon build --packages-up-to bringup

# 3. Source + run mock system
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch bringup mock_system.launch.py
# → 6 nodes spin up; view in RViz2
rviz2
```

### Target machine (Jetson Orin Nano 4GB, real hardware)

```bash
ssh xrak@192.168.3.69
sudo apt install -y python3-colcon-common-extensions  # one-time
cd ~/rak && colcon build --packages-up-to bringup     # workspace = repo root
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch bringup full_system.launch.py
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
| `camera_node` (×2) | — | `/rak/sensors/camera/<id>/image_raw` |
| `infrared_node` (×2) | — | `/rak/sensors/ir/<id>` |
| `mecanum_chassis_node` | `/cmd/vel_safe` | `/state/odom`, `/tf` |
| `arm_node` | `/cmd/arm/trajectory` | `/state/actuators/<id>` |
| `safety_gate_node` | `/cmd/vel_raw`, `/safety/*` | `/cmd/vel_safe` |
| `MC602HardwareInterface` (ros2_control) | — | wheel/arm state |

## Repository layout

```
rak-car/                          # repo root = colcon workspace (src/ directly under root)
├── src/
│   ├── hardware/                 # C++ drivers: MC602 adapter + serial, chassis kinematics,
│   │   │                         # hardware nodes (camera/ir/chassis/arm/safety), ros2_control plugin
│   │   ├── include/              # public headers (BaseController, BaseChassis, MecanumChassis, MissionRunner, ...)
│   │   ├── src/                  # implementations + rclcpp node .cpp
│   │   ├── test/                 # gtest (8 binaries)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── msgs/                     # custom interfaces (CameraMeta, LaneResult, DetectionArray, ActuatorState)
│   │   ├── msg/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── bringup/                  # assembly: launch files, URDF, calibration params, DDS config
│   │   ├── launch/               # full_system / mock_system / dev_all
│   │   ├── urdf/                 # rak.urdf.xacro (+ .rviz)
│   │   ├── params/               # camera_{front,arm}.yaml calibration
│   │   ├── config/               # cyclonedds.xml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── cognition/                # Python feature layer: inference bridge (+ future features)
│       ├── cognition/            # package module (inference/bridge.py)
│       ├── test/                 # pytest (smoke tests)
│       ├── setup.py
│       └── package.xml
├── scripts/                      # team tooling (onboard / diagnose / start_team_rviz / calibrate_camera)
├── docs/                         # documentation
│   ├── README.md                 # doc index
│   ├── hardware-inventory.md     # what's wired (reference only — node params are the runtime truth)
│   ├── hardware-port-mapping.md  # M口/S口/P口/步进 物理映射 (PR #5)
│   ├── adr/                      # architecture decision records
│   ├── development/              # dev/target workflow
│   ├── migration/                # JetPack 6 migration (already done!)
│   ├── contributing/             # branch strategy
│   └── superpowers/              # historical spec (superseded)
├── .devcontainer/               # dev container (Docker)
├── CLAUDE.md                     # Claude Code guidance
├── CONTRIBUTING.md               # contribution guide
├── LICENSE                       # license
└── README.md                     # this file
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
