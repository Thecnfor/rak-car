# Driver ↔ App Interface Contract

> **The contract between `origin/robot-stable` (Jetson runtime) and
> `origin/develop/ros2-sidecar` (dev tree) is the ROS2 topic schema under
> `/rak/...`. This doc names it and locks down the rules.**

This file was created to fill a gap: the spec
[`docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md`](superpowers/specs/2026-07-05-ros2-sidecar-design.md)
promised `docs/component-contract.md` (line 1702) but it was never written.
This doc is its functional replacement, framed for the dual-branch world
that commit `30f9620` introduced.

## Why this doc exists

After commit `30f9620 chore(robot-stable): strip dev-only docs/scripts/CI
for robot-side runtime`, the project runs on **two branches with one
contract**:

| Branch | Where it lives | What it ships |
|---|---|---|
| **`origin/robot-stable`** | On the Jetson (git checkout at `192.168.3.69`) | `src/hardware` + `src/msgs` drivers + minimal app that **publishes** the contract topics |
| **`origin/develop/ros2-sidecar`** | On dev desktops | Full dev tree + `mock_system.launch.py` that **also publishes** the contract topics but with `/dev/null` stubs so everyone can test app code locally |

Both branches speak the same `/rak/...` topic schema. `ROS_DOMAIN_ID=42` makes DDS auto-discovery glue them together over LAN. That's the entire interface — there is no other hand-shake.

## The contract

The contract is a **topic schema**, frozen at commit `82fc1d6`. Below is the canonical version; if you change anything here you must change it on **both** branches in the same PR.

### Camera topic schema (5 streams, per camera)

Each camera under `/rak/sensors/camera/<id>/`:

| Stream | Type | QoS | Rate | Notes |
|---|---|---|---|---|
| `image_raw` | `sensor_msgs/Image` (bgr8) | BEST_EFFORT, depth=1 | 10 Hz (dev) / 30 Hz (Jetson, see note) | Heavy: ~900 KB/frame |
| `image_compressed` | `sensor_msgs/CompressedImage` | BEST_EFFORT, depth=1 | 10 Hz (dev) / 30 Hz (Jetson) | JPEG q=85, ~50 KB/frame. The bandwidth-friendly alternative |
| `camera_info` | `sensor_msgs/CameraInfo` | TRANSIENT_LOCAL | on-change | **NOT published** when YAML has all-zero K. See [calibration rules](#calibration-required-for-camera_info) below |
| `camera_status` | `diagnostic_msgs/DiagnosticArray` | RELIABLE, depth=1 | 1 Hz | OK / WARN / ERROR |
| `camera_meta` | `msgs/msg/CameraMeta` | RELIABLE, depth=1 | 1 Hz | Custom message — see `src/hardware/msg/CameraMeta.msg` |

> **Rate note**: develop's docs say 10 Hz (the current dev-box default for SP2812 cameras in low-light), robot-stable's docs say 30 Hz (the competition target). Both are valid for the same hardware under different lighting. **The contract doesn't pin a single rate** — it pins the schema. Pick whatever rate fits your environment.

### The 7 rclcpp nodes (who publishes / who consumes)

| Node | Subscribes | Publishes | Rate |
|---|---|---|---|
| `camera_node` (×2: front, arm) | — | camera topics (above) | 10–30 Hz |
| `infrared_node` (×2: left, right) | — | `/rak/sensors/ir/<id>` | 20 Hz |
| `mecanum_chassis_node` | `/cmd/vel_safe` | `/state/odom`, `/tf` | 50 Hz |
| `arm_node` | `/cmd/arm/main/trajectory` | `/state/actuators/main` | 50 Hz |
| `safety_gate_node` | `/cmd/vel_raw` + `/safety/*` | `/cmd/vel_safe` | continuous |
| `MC602HardwareInterface` (ros2_control plugin) | controller_manager | wheel/arm state | — |
| `mission_runner_node` (Phase 1.5+) | task_list param | per-task state | — |

**Topic prefix**: everything lives under `/rak/`. `config_loader.py` and the C++ nodes reject anything outside this prefix. Adding a topic outside `/rak/...` is a contract violation.

### Cmd / state / safety channels

| Channel | Direction (Jetson ↔ dev) | Notes |
|---|---|---|
| `/rak/cmd/vel_raw` | dev → Jetson | Raw cmd (before safety gate) |
| `/rak/cmd/vel_safe` | dev → Jetson (consumed by `mecanum_chassis_node`) | Post-safety cmd |
| `/rak/cmd/arm/main/trajectory` | dev → Jetson (consumed by `arm_node`) | Arm motion plan |
| `/rak/state/odom` | Jetson → dev | Chassis odometry |
| `/rak/state/actuators/main` | Jetson → dev | Arm joint state |
| `/rak/safety/{estop,heartbeat,mode_cmd}` | both | Safety handshake |

## Working on drivers (publisher side)

You are on the **Jetson side** if you're changing `camera_node.cpp`, `infrared_node.cpp`, `mecanum_chassis_node.cpp` V4L2/serial code, or adding a new physical sensor to `config_sensors.yml`.

1. Branch off `develop/ros2-sidecar` (not `robot-stable` directly).
2. Add the new topic to the **schema above** in this same doc, in the same PR.
3. Build + test on the Jetson. Verify the dev-side `ros2 topic list` sees your new stream.
4. PR against `develop/ros2-sidecar`. Get a reviewer.
5. After merge, **Thecnfor cherry-picks to `robot-stable`** and re-builds on the Jetson.

> **Never publish a topic whose name isn't in this doc.** It's the contract.

## Working on app / business logic (consumer side)

You are on the **dev side** if you're building lane-following, sign detection, mission planner, AI inference, dashboard UI — anything that reads the contract and writes a higher-level decision back.

1. Branch off `develop/ros2-sidecar`.
2. Subscribe to existing contract topics. **Do not add new topics** — if you need a stream that doesn't exist, talk to the driver owner first.
3. Test against `ros2 launch bringup mock_system.launch.py` — it satisfies the publisher side of the contract using `/dev/null` stubs, so you can iterate without the Jetson.
4. When you need a real-hardware check, ask the team who's at the car, or run `bash scripts/diagnose.sh` to confirm DDS discovery is alive.
5. PR against `develop/ros2-sidecar`. Thecnfor merges and then **separately** cherry-picks to `robot-stable` if the change touches runtime code paths.

> **App-only subscribers** are allowed without touching the publisher side. Examples: a logging node that just `/ros2 bag record`s every camera stream; a viz node that aggregates IR sensors.

## Testing without hardware

The dev-side stub is **`src/bringup/launch/mock_system.launch.py`**. It is the **functional counterpart** of `full_system.launch.py` (which ships in `robot-stable` and runs on the Jetson):

- Same node set
- Same topic schema
- Hardware parameters overridden to `/dev/null` so no Jetson needed

```bash
# Spin up the dev stub
source /opt/ros/<your_distro>/setup.bash
source install/setup.bash
ros2 launch bringup mock_system.launch.py

# In another terminal: verify the contract is being published
ros2 topic list | grep rak/v1
# → expect: cmd/..., state/..., sensors/...

# Iterate your app node against the stub
ros2 run my_app_pkg my_feature_node --ros-args -p use_mock:=true
```

For a one-click camera preview: `bash scripts/start_team_rviz.sh` (which auto-falls-back to software OpenGL on Wayland+XWayland — known issue, see `docs/operations/troubleshooting.md`).

## Calibration required for `camera_info`

`camera_info` is special: it's **NOT published** unless the camera's YAML has real intrinsics (non-zero K matrix). The rule is in `camera_node.cpp` — when `camera_info_manager` loads an all-zero K, the camera_info publisher is never created. **NEVER fake intrinsics**. See [`CLAUDE.md`](../CLAUDE.md) "Critical warnings" #1 (no mocks in production code).

Operator runs `scripts/calibrate_camera.py` after a lens or sensor swap. The output YAML lands at `install/.../share/bringup/params/camera_<id>.yaml` and is referenced via `front_calibration_url` / `arm_calibration_url` launch args.

## Interface changes are breaking

Changing the schema (renaming a topic, changing a type, changing QoS, adding/removing a stream) is a **cross-branch breaking change**. The procedure:

1. Bump `config_sensors.yml` revision (add a comment with the change date + reason).
2. Update the schema tables in **this file** in the same PR.
3. Update both `CLAUDE.md` files (develop + robot-stable) if they quote the schema.
4. Open one PR against `develop/ros2-sidecar`. After merge:
5. Thecnfor cherry-picks to `robot-stable` on the Jetson.
6. Coordinate a Jetson redeploy (`colcon build` + restart `full_system.launch.py`) — the dev-side RViz will go dark until both sides match.
7. Roll back is non-trivial: a mismatched pair leaves dev unable to subscribe to Jetson's topics. Don't ship a schema change without a confirmed Jetson redeploy window.

If you only need **a new topic** (additive, no rename), the procedure is lighter — no version bump, just update this doc. But still cross-branch, still needs the redeploy.

## Related

- [`docs/contributing/branch-strategy.md`](contributing/branch-strategy.md) — when to merge where, including a row for cross-branch schema changes
- [`CLAUDE.md`](../CLAUDE.md) — root conventions including the no-mocks rule and the camera 5-stream schema in its original form
- [`config_sensors.yml`](../config_sensors.yml) — single source of truth for what's wired to the robot
- [`src/bringup/launch/full_system.launch.py`](../src/bringup/launch/full_system.launch.py) — Jetson launch (in `robot-stable`)
- [`src/bringup/launch/mock_system.launch.py`](../src/bringup/launch/mock_system.launch.py) — dev stub (this branch)