# vehicle_wbt — Robot Side (Jetson Orin Nano 4GB)

This branch (`robot-stable`) is the runtime-only code that lives on the
Jetson. Devs control this robot remotely over the LAN via ROS2 topics
(`ROS_DOMAIN_ID=42`, Jetson IP `192.168.3.69`).

**If you are a developer setting up a dev box, you are on the wrong
branch.** Use `origin/develop/ros2-sidecar` instead — that branch has the
full `docs/` tree, `scripts/onboard.sh`, `scripts/diagnose.sh`, and other
dev tooling.

## Build & launch

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_wbt_platform_cpp vehicle_wbt_platform
source install/setup.bash
ros2 launch vehicle_wbt_platform_cpp full_system.launch.py \
    front_device:=/dev/cam4 arm_device:=/dev/cam3
```

`ROS_DOMAIN_ID=42` is set inside the launch file — do not export manually.

## Verify

```bash
ros2 topic list                                  # ~15 topics under /vehicle_wbt/v1/
ros2 node list                                   # 7 nodes alive
ros2 topic hz /vehicle_wbt/v1/sensors/camera/front/image_raw   # ~30 Hz
```

## Re-calibrate after lens/sensor swap

```bash
python3 scripts/calibrate_camera.py /path/to/*.png 8 6 0.025 \
    --out ros2_ws/install/vehicle_wbt_platform_cpp/share/vehicle_wbt_platform_cpp/params/camera_front.yaml
```

Then restart the launch — `camera_info` will start publishing.

## Layout

- `ros2_ws/src/vehicle_wbt_platform_cpp/` — 7 rclcpp nodes + ros2_control plugin
- `ros2_ws/src/vehicle_wbt_platform/` — Python orchestrator (config_loader, ENABLE_ROS2 gate)
- `config_sensors.yml` — hardware registry (source of truth)
- `urdf/vehicle_wbt.urdf.xacro` — robot description
- `scripts/calibrate_camera.py` — operator tool
- `CLAUDE.md` — detailed Claude/operator guidance (read this first)

## Hardware on this Jetson

- Cameras: Aveo SP2812, `/dev/cam4` (front) and `/dev/cam3` (arm)
  - Driver quirk: only MJPG works — see CLAUDE.md "Hardware quirks"
  - `udev` rule `/etc/udev/rules.d/99-usbvideo.rules` provides the symlinks
- Network: Wi-Fi `wlP1p1s0`, static IP `192.168.3.69`

## License

See [LICENSE](LICENSE).