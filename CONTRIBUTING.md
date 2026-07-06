# Contributing

## For everyone

1. Read [`docs/README.md`](docs/README.md) — doc index
2. Read [`docs/development/README.md`](docs/development/README.md) — dev + target dual-machine workflow
3. Follow the 5-step daily workflow in [`CLAUDE.md`](CLAUDE.md) (bottom section)

## For ROS2 component developers

The platform follows a strict **platform-level abstraction**:

> **Adding new hardware = 1 line in `config_sensors.yml` + 1 link in URDF + reserved topic. No business code touched.**

Read [`docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md`](docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md) before writing any C++ or Python node.

## For the next competition cycle

This branch (`develop/ros2-sidecar`) will become `main` after 2026-08-12. Don't merge into `main` during the competition freeze window (2026-07-13 to 2026-08-12). See [`docs/contributing/branch-strategy.md`](docs/contributing/branch-strategy.md).
