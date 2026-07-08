# LAN RViz View of Jetson Cameras

> Watch the Jetson's front + arm cameras from any dev desktop on the same LAN
> via RViz2. Both machines run ROS2 Humble (Jetson via the `ros:humble-ros-base`
> image, dev via apt `ros-humble-desktop`); DDS discovery finds the Jetson
> automatically as long as both share `ROS_DOMAIN_ID=42`.

## Prerequisites (one-time per dev machine)

1. **Same LAN subnet.** Jetson (`192.168.3.69`) and dev desktop must be on the same
   IPv4 subnet (multicast routing is unblocked). `ping 192.168.3.69` from the dev
   desktop should work.

2. **Same DDS** — we use **CycloneDDS** so config is identical across
   Humble ↔ Jazzy:

   ```bash
   # On both machines — copy the file from the repo, don't handwrite:
   sudo cp ros2_ws/src/vehicle_wbt_platform_cpp/config/cyclonedds.xml \
           /etc/cyclonedds/cyclonedds.xml
   ```

3. **Same `ROS_DOMAIN_ID=42`** in BOTH shells before any `ros2` command:

   ```bash
   export ROS_DOMAIN_ID=42
   ```

   The Jetson's `full_system.launch.py` already sets this internally, but
   the dev shell needs the export (or the alias in `.bashrc`).

4. **(Optional) Same `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`** — only
   required if the dev box has both Cyclone and FastDDS installed and you
   want to force consistency.

## Confirm discovery works (dev machine)

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
# or: source /opt/ros/jazzy/setup.bash  (Foxy+ has the cross-distro bridge)

# 1. Jetson's nodes visible?
ros2 node list
# expect at least:
#   /camera_arm
#   /camera_front
#   /mecanum_chassis
#   /arm_main
#   /ir_left  /ir_right  /safety_gate

# 2. Cam topics visible?
ros2 topic list | grep vehicle_wbt/v1/sensors/camera
# expect 10 topics (5 per camera × 2):
#   /vehicle_wbt/v1/sensors/camera/front/{image_raw,image_compressed,camera_meta,camera_status}
#   /vehicle_wbt/v1/sensors/camera/arm/{image_raw,image_compressed,camera_meta,camera_status}
# camera_info NOT present (calibration YAML still sentinel — that is correct)

# 3. tf_static frame_ids visible? (set by Jetson's camera_node)
ros2 topic echo /tf_static --once
```

If `ros2 node list` returns nothing: firewall on the Jetson is blocking UDP
multicast (239.255.0.0/16 for Cyclone, or 224.0.0.0/24 for mDNS discovery).
Run `sudo ufw allow from 192.168.0.0/16 to any port 7400-7500 proto udp` on
the Jetson (replace 192.168.0.0/16 with your actual subnet).

## RViz subscription recipe

1. **Subscribe to image_compressed** (10x smaller than raw, ~50 KB/frame
   at q=85, fine for visual confirmation over wifi):

   ```bash
   rviz2
   # In RViz:
   #   Add → Image
   #   Image Topic: /vehicle_wbt/v1/sensors/camera/front/image_compressed
   ```

2. **Side-by-side front + arm**: open a second Image display in RViz,
   set its topic to `/vehicle_wbt/v1/sensors/camera/arm/image_compressed`
   and move it next to the first one.

3. **Static transforms visible** — RViz should show
   `front_camera_optical_frame` and `arm_camera_optical_frame` under
   `base_link` automatically, **because camera_node publishes
   `/tf_static`**. Add a `TF` display to verify:
   - `Add → TF` (or it's listed in Default tree)
   - You should see `base_link` with two small frame markers
     (`front_camera_optical_frame`, `arm_camera_optical_frame`)
   - Positions are currently `(0,0,0)` because we haven't overridden
     `tf_x/y/z/roll/pitch/yaw` yet — the cameras are nominally at the
     robot origin until someone measures their physical mount and sets
     the launch args.

4. **For image_raw, subscribe to image_raw instead.** Same `Add →
   Image` flow, but the topic. Useful if you actually want pixel data
   (e.g. for AI inspection) instead of just visual confirmation. Note
   `image_raw` is **~900 KB/frame at 30 fps = 27 MB/s**; over wifi RViz
   may stutter. Stay on `image_compressed` for human visual review.

## Bandwidth notes

| Topic | Size × 30Hz × 2 cams | Practical |
|-------|---------------------|-----------|
| `image_raw` (bgr8) | ~54 MB/s | local Ethernet only |
| `image_compressed` (jpeg q=85) | ~3 MB/s | fine over wifi |
| `camera_meta` (one sensor each) | ~1 KB/s | trivial |
| `camera_status` (diagnostic_msgs) | ~2 KB/s | trivial |

For routine RViz viewing on a typical wifi office link, stick to
`image_compressed`. Switch to `image_raw` only when you're on
wired Ethernet and need full pixel data (e.g. image_proc, calibration
inspection).

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `ros2 topic list` empty on dev | UDP multicast blocked | Open firewall on Jetson (see above) |
| `ros2 daemon restart` needed on dev | Stale DDS state | `pkill -9 -f ros2_daemon; ros2 daemon start` |
| One cam visible, other not | One device path wrong | Re-check udev: `ls -la /dev/cam*` |
| RViz shows images at ~5 fps | Subscribing to `image_raw` over wifi | Switch to `image_compressed` |
| `camera_status` shows `achieved_rate_hz: 10` instead of `30` | Low light → SP2812 firmware caps at 10fps | Add light; do **not** adjust config |
| `total_drops` is rising | V4L2 capture catching USB errors | Check cable, see `consecutive_failures` in `camera_status` |
