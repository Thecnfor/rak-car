# Full Vision RViz Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run both TensorRT lane models and PP-YOLOE on the two real cameras and let a local RViz2 instance display live front/side recognition overlays.

**Architecture:** Keep the existing camera, lane, and detector nodes unchanged as the inference sources. Add one lightweight Python overlay node that caches the latest compressed images and structured inference messages, draws lane guidance or detection boxes, and publishes reliable raw `sensor_msgs/Image` overlays. Add a cognition full-vision launch for the Orin runtime and a bringup RViz-only launch/config for the dev desktop; both use ROS_DOMAIN_ID=42.

**Tech Stack:** ROS2 Humble/Lyrical-compatible Python launch, rclpy, OpenCV, sensor_msgs, custom `msgs/LaneResult` and `msgs/DetectionArray`, RViz2.

## Global Constraints

- All ROS topics remain under `/rak/...`.
- No synthetic camera frames or synthetic inference results in the production full-vision path.
- TensorRT engines are preferred and missing engines must fail rather than silently switch to mock output.
- The Orin camera mapping is `/dev/cam4` front and `/dev/cam3` side.
- Do not modify `cyclonedds.xml` or `start_team_rviz.sh` behavior.
- Dev RViz runs with `ROS_DOMAIN_ID=42` and does not open camera devices or load models.
- Validate with `bash scripts/diagnose.sh` before claiming the stack is healthy; report environment-only failures separately.

---

### Task 1: Add live recognition overlay node

**Files:**
- Create: `src/cognition/cognition/visualization/vision_overlay.py`
- Create: `src/cognition/cognition/visualization/__init__.py`
- Modify: `src/cognition/setup.py:24-29`
- Test: `src/cognition/test/test_vision_overlay.py`

**Interfaces:**
- Consumes compressed front/side images, `msgs/msg/LaneResult`, and `msgs/msg/DetectionArray`.
- Produces `sensor_msgs/msg/Image` on `/rak/visualization/front_overlay` and `/rak/visualization/side_overlay`.
- Parameters: `front_image_topic`, `side_image_topic`, `lane_topic`, `detection_topic`, and `publish_rate_hz` (default 10.0).
- Pure helpers must be importable without rclpy: `draw_lane_overlay(frame, lane)` and `draw_detection_overlay(frame, detections)` return BGR `numpy.ndarray` copies.

- [ ] **Step 1: Write failing helper tests**

```python
def test_draw_lane_overlay_changes_frame_and_labels_result():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    lane = LaneResult()
    lane.valid = True
    lane.deviation_distance = 0.2
    lane.deviation_angle = 0.1
    lane.inference_ms = 8.0
    out = draw_lane_overlay(frame, lane)
    assert out.shape == frame.shape
    assert not np.array_equal(out, frame)


def test_draw_detection_overlay_draws_bbox_and_class():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = DetectionArray()
    detections.image_width = 640
    detections.image_height = 480
    detections.class_names = ["ball"]
    detections.scores = [0.91]
    detections.xs = [100.0]
    detections.ys = [120.0]
    detections.widths = [80.0]
    detections.heights = [60.0]
    out = draw_detection_overlay(frame, detections)
    assert out.shape == frame.shape
    assert not np.array_equal(out, frame)
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `cd src/cognition && PYTHONPATH=. python3 -m pytest test/test_vision_overlay.py -q`

Expected: FAIL because `draw_lane_overlay` and `draw_detection_overlay` do not exist.

- [ ] **Step 3: Implement the pure overlay helpers**

Use OpenCV drawing only. Copy the input frame first. For lane, draw a center line, a clamped deviation marker, a heading arrow from `deviation_angle`, and text containing `valid`, `deviation_distance`, `deviation_angle`, and `inference_ms`. For detections, scale coordinates from `DetectionArray.image_width/image_height` to the actual frame, draw rectangles, and label each with `class_names[i]` and `scores[i]`; skip malformed parallel-array entries without crashing.

- [ ] **Step 4: Implement the ROS2 node**

Subscribe with reliable depth-5 QoS to the configured compressed image topics and structured result topics. Decode images with `cv2.imdecode`, cache the latest frame/result under a lock, and publish overlays from a 10Hz timer. Build `sensor_msgs.msg.Image` manually with `encoding='bgr8'`, `step=width*3`, and the original frame stamp. Do not publish an overlay until a real source frame exists.

- [ ] **Step 5: Add the console entry point and run tests**

Add:

```python
"vision-overlay = cognition.visualization.vision_overlay:main"
```

Run: `cd src/cognition && PYTHONPATH=. python3 -m pytest test/ -q`

Expected: all existing smoke tests plus the two overlay tests pass.

- [ ] **Step 6: Commit the overlay unit**

```bash
git add src/cognition/cognition/visualization src/cognition/setup.py src/cognition/test/test_vision_overlay.py
git commit -m "feat(cognition): publish RViz recognition overlays"
```

---

### Task 2: Add full-vision runtime and RViz-only launch/config

**Files:**
- Create: `src/cognition/launch/full_vision.launch.py`
- Create: `src/bringup/launch/vision_rviz.launch.py`
- Create: `src/bringup/config/vision_overlay.rviz`
- Modify: `src/cognition/launch/dual_camera_ai.launch.py:21-73`

**Interfaces:**
- Orin command: `ros2 launch cognition full_vision.launch.py`.
- Dev command: `ros2 launch bringup vision_rviz.launch.py`.
- Full vision publishes `/rak/perception/lane`, `/rak/perception/detections/task`, `/rak/visualization/front_overlay`, and `/rak/visualization/side_overlay`.

- [ ] **Step 1: Add `full_vision.launch.py`**

Start two `hardware/camera_node` processes, `cognition/lane-follower`, `cognition/detector-node`, and `cognition/vision-overlay`. Set `ROS_DOMAIN_ID=42`; default devices `/dev/cam4` and `/dev/cam3`; use front topic for lane and side topic for detector; pass the three engine paths and labels path as launch arguments; set lane rate 30Hz, detector requested rate 20Hz, and overlay rate 10Hz. Do not start `inference-bridge`, since it is mock by default and would publish competing perception topics.

- [ ] **Step 2: Align `dual_camera_ai.launch.py` defaults**

Change only its default physical devices to `/dev/cam4` and `/dev/cam3`, and add the overlay node with the same output topics. Keep explicit device overrides and existing model arguments. This preserves the existing launch name while making it safe for the current Orin wiring.

- [ ] **Step 3: Add RViz-only launch**

`vision_rviz.launch.py` sets `ROS_DOMAIN_ID=42`, resolves `vision_overlay.rviz` from the `bringup` share directory, and starts only:

```python
Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config])
```

It must not start cameras or inference nodes, so the dev desktop can observe the Orin over DDS without opening local devices.

- [ ] **Step 4: Add RViz configuration**

Create an RViz config with fixed frame `base_link` and four Image displays:

```text
/rak/visualization/front_overlay
/rak/visualization/side_overlay
/rak/sensors/camera/front/image_compressed
/rak/sensors/camera/side/image_compressed
```

Use the overlay images as the enabled displays and keep raw streams disabled by default. Include TF display only; do not depend on a robot model or lower-controller topics.

- [ ] **Step 5: Build and syntax-check launch files**

Run:

```bash
python3 -m py_compile src/cognition/launch/full_vision.launch.py src/bringup/launch/vision_rviz.launch.py
colcon build --packages-select cognition bringup
```

- [ ] **Step 6: Commit launch/config integration**

```bash
git add src/cognition/launch/full_vision.launch.py src/cognition/launch/dual_camera_ai.launch.py src/bringup/launch/vision_rviz.launch.py src/bringup/config/vision_overlay.rviz
git commit -m "feat(vision): add full TRT stack and RViz launch"
```

---

### Task 3: Sync Orin, run full visual smoke test, and validate local RViz path

**Files:**
- No source changes unless a verification failure identifies a concrete defect.

- [ ] **Step 1: Push the ROS2 branch and sync Orin**

```bash
git push origin develop/ros2-sidecar
ssh Orin 'cd ~/rak && git pull --ff-only origin develop/ros2-sidecar'
```

If GitHub is temporarily unreachable, transfer the commits with `git format-patch`/`git am` and report the remote branch state explicitly.

- [ ] **Step 2: Build on Orin**

```bash
ssh Orin 'cd ~/rak && source /opt/ros/humble/setup.bash && env -u COLCON_PREFIX_PATH colcon build --packages-select cognition bringup'
```

- [ ] **Step 3: Start the full visual stack on Orin**

```bash
ssh Orin 'cd ~/rak && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=42 && ros2 launch cognition full_vision.launch.py'
```

Confirm logs contain both lane backends as `trt`, PP-YOLOE engine loading with 23 labels, and the overlay node startup. Confirm no MC602/controller nodes are launched.

- [ ] **Step 4: Start RViz locally**

On the dev desktop:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=42
source install/setup.bash
ros2 launch bringup vision_rviz.launch.py
```

Confirm RViz sees both overlay topics and displays boxes/lane annotations.

- [ ] **Step 5: Capture objective topic evidence**

```bash
export ROS_DOMAIN_ID=42
ros2 topic hz /rak/visualization/front_overlay
ros2 topic hz /rak/visualization/side_overlay
ros2 topic echo /rak/perception/lane --once
ros2 topic echo /rak/perception/detections/task --once
```

Report the actual rates, including the known PP-YOLOE throughput near 6.7Hz rather than claiming its requested 20Hz.

- [ ] **Step 6: Run project diagnostics and final test report**

Run `bash scripts/diagnose.sh --no-remote` and report pass/fail/warnings exactly. Run the Python tests and any Orin hardware tests that are relevant to the changed packages. Do not claim a clean diagnostic if the environment still reports domain/SSH/DDS failures.
