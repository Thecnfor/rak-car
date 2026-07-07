# Calibration workflow — vehicle_wbt Platform

This directory contains the operator-facing tools for camera intrinsic
calibration. The pipeline produces real measured intrinsics — it does
**not** fabricate values. If the chessboard isn't visible in the
captured frames, the script fails loudly and writes nothing.

When to run calibration
----------------------
After physical changes that change intrinsics:

- Camera lens replaced / swapped
- Sensor swapped (different revision of SP2812, new Aveo model, …)
- Camera is uncalibrated (the `camera_status` topic reports
  `calibration_loaded = false`)

A correct calibration is required before the image pipeline can run
image rectification, depth-from-stereo, or any pixel↔3D conversion.
Lane following / detection in 2D works without calibration but is less
robust.

Workflow (interactive, operator at a desktop)
--------------------------------------------
### 1. Print the chessboard

Use a calibration pattern with known square size. The defaults below
match a standard chessboard:

- **8 × 6** INNER corners (9 × 7 squares)
- Square side: **25 mm**

```bash
# mounted in front of the front camera, then the arm camera
```

### 2. Start the camera_node pipeline

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch vehicle_wbt_platform_cpp full_system.launch.py \
    front_device:=/dev/cam4 \
    arm_device:=/dev/cam3
```

Confirm `image_raw` is flowing:

```bash
ros2 topic hz /vehicle_wbt/v1/sensors/camera/front/image_raw
```

### 3. Capture ~20 frames per camera

With the chessboard in view, capturing **at varied angles and distances**:

```bash
mkdir -p ~/calib_session
ros2 bag record -o ~/calib_session/front.bag \
    /vehicle_wbt/v1/sensors/camera/front/image_raw
# move the chessboard around, hold at varying tilt and distance
# Ctrl-C after you have ~20 frames
```

Alternatively, copy the raw frames out of the bag with `ros2 bag play`
and `image_conversion` — or use any recording tool that writes PNGs.
The script `calibrate_camera.py` accepts PNGs.

### 4. Run the calibration

**Option A — interactive (recommended for one-shot calibration):**

```bash
ros2 run camera_calibration cameracalibrator.py \
    --size 8x6 --square 0.025 \
    image:=/vehicle_wbt/v1/sensors/camera/front/image_raw \
    camera:=/vehicle_wbt/v1/sensors/camera/front
```

1. Move the chessboard through all angles while watching the X / Y / Size
   sliders.
2. Click **Calibrate** when the bar is high enough.
3. Click **Save**, point the dialog at:

   ```bash
   /home/xrak/workspace/rak-car/ros2_ws/src/vehicle_wbt_platform_cpp/\
       params/camera_front.yaml
   ```

   (camera_info_manager will overwrite it.)

**Option B — non-interactive (script + recorded frames):**

```bash
# Extract frames from the bag (or use any tool to write PNGs)
mkdir -p ~/calib_session/front_png
# ...save PNGs to ~/calib_session/front_png/*.png

python3 scripts/calibrate_camera.py \
    ~/calib_session/front_png/*.png 8 6 0.025 \
    --out /home/xrak/workspace/rak-car/ros2_ws/src/\
vehicle_wbt_platform_cpp/params/camera_front.yaml
```

The script:

1. Runs `cv2.findChessboardCorners` on every input image. **Skips any
   image where the pattern is not detected.**
2. If fewer than `--min-frames` (default 10) frames have the chessboard,
   the script exits non-zero and **writes no output file**. params YAML
   stays in the all-zero sentinel state.
3. If sufficient frames are detected, runs `cv2.calibrateCamera` and
   saves the result in camera_info_manager's YAML schema.

### 5. Restart camera_node

```bash
# kill the old one
pkill -f camera_node
# relaunch the full system
ros2 launch vehicle_wbt_platform_cpp full_system.launch.py \
    front_device:=/dev/cam4 \
    arm_device:=/dev/cam3
```

After relaunch:

```bash
ros2 topic info /vehicle_wbt/v1/sensors/camera/front/camera_info
# QoS: TRANSIENT_LOCAL; Publisher: 1
ros2 topic echo --once /vehicle_wbt/v1/sensors/camera/front/camera_status | grep calibration_loaded
# - value: 'true'
```

A `true` value on `calibration_loaded` confirms the YAML has real K and
camera_info is being published.

Calibrating the arm camera
--------------------------
Repeat steps 2-5 with `camera_id:=arm` (or whatever the launch arg
maps to). Use the same procedure but **mount the chessboard in the arm
camera's field of view**.

Troubleshooting
--------------

- **"chessboard detected in 0/N images"** — the chessboard isn't visible
  in the captured frames. Check (1) lighting, (2) focus, (3) that the
  pattern is the right size, (4) that cv::findChessboardCorners
  accepts your pattern (must be inner corners — 8×6 means 9×7 squares).

- **RMS reprojection error > 1.0 px in script Option B** — too few good
  frames or the board didn't cover enough of the field. Capture more
  diverse views and re-run.

- **camera_node says "calibration_loaded: false" after launching** —
  the YAML probably has K all-zeros (sentinel). Run calibration again.

- **`cv2.findChessboardCorners` returns False on every frame** — the
  Python-OpenCV backend may not see the pattern. Try `--no-fix_aspect`
  in script Option B (rare; default already doesn't fix aspect unless
  `--fix-aspect`).

See also
--------
- `docs/camera-system.md` (planned) — camera node architecture
- `params/camera_front.yaml` — sentinel file structure with comments
- `camera_node.cpp:has_real_calibration()` — the gate that decides
  whether to publish camera_info
