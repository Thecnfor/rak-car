# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Competition code for the 百度智能车 (Baidu Smartcar) 2026 智慧农业 (smart-agriculture) track — a Jetson Nano + MC602 controller running on a WhalesBot mecanum-wheel chassis. A single run executes **8 fixed-order tasks** (seed → scout pests → water → shoot pests → harvest → sort → read order via OCR → deliver). The repo is a frozen-for-competition codebase; per-track calibration happens in `config_car.yml` and the odometry offsets hardcoded inside `car_task_function.py`.

## Run / build commands

There are no build steps, no tests, no linter, and no `requirements.txt`. The runtime is Python 3.8+ on a Jetson Nano with PaddlePaddle Inference (CPU/CUDA or TensorRT, configured via `config_car.yml → infer_cfg[].run_mode`).

- **Full mission**: `python car_start_2026.py`
- **Single task**: comment out the other steps inside `car_start_2026.main()` (see README §"单独测试某个任务")
- **Test lane-only traversal**: call `auto_lane_tracing(speed=0.3, dis_hold=99)` from a REPL after `init()` — `dis_hold=99` means "follow the lane until you lose it or hit the base"
- **Data collection (two-cam over LAN)**: `python collect_data.py`, then browse `http://<car-ip>:5000/`. Bluetooth gamepad; key `3` collects lane frames into `./dataset/image_set_lane/`, key `4` collects object frames into `./dataset/image_set_object/`. Both dirs are gitignored (`**/dataset/`).
- **Convert collected frames to COCO**: `smartcar/whalesbot/tools/x2coco.py`

The four ZMQ inference backends (ports 5001–5004, see below) are **auto-spawned** the first time `MyCar()` is constructed — you should not start them by hand.

## Big-picture architecture

### Three layers, top-down

1. **Competition orchestration** (root-level scripts)
   - `car_start_2026.py` — thin `main()` that calls `init()` then each task in sequence.
   - `car_task_function.py` — the 8 task functions. Each is mostly a hand-tuned sequence of `my_car.move_to_position(...)`, `my_car.move_to_detection_target()`, `my_car.arm.grasp(...)`, etc. Coordinates here (e.g. `cylinder_loc` in `auto_seeding`) are *track-specific magic numbers* — they are the calibration surface when porting to a different venue.
   - `car_wrap_2026.py` — defines `class MyCar(MecanumDriver)`. This is the single facade the task layer talks to. `init()` in `car_task_function.py` constructs it as the global `my_car`. Construction order is: chassis → screen → streamer → arm → sensor (`Key4Btn`, `ServoPwm` for storage, `BluetoothPad`, `PoutD` for shooter) → PID → cameras → paddle infer → ERNIE bot → key-press daemon thread.

2. **WhalesBot hardware SDK** (`smartcar/whalesbot/`)
   - `vehicle/driver/mecanum.py` + `vehicle_base.py` — `MecanumDriver` (base of `MyCar`), implements `move_for`, `move_to_position` (waypoint with `location_pid`), `get_odometry`, `lane_dis_offset` (follow lane for `dis_hold` meters), and chassis geometry from `cfg_vehicle.yaml` (`track=0.30`, `wheel_base=0.28`).
   - `vehicle/arm/arm_base.py` — `ArmController`: `reset_position`, `set_arm_pose(arm_id, pitch, "LEFT"/"RIGHT", "UP"/"DOWN")`, `grasp(bool)` (vacuum on/off), `move_x_position`, `move_y_position`. Poses referenced by string direction constants, not by joint angles — look up enum in `arm_base.py` before adding joints.
   - `vehicle/base/controller_wrap.py` — per-pin peripherals (`Beep`, `Key4Btn`, `Infrared`, `NixieTube`, `ServoPwm`, `BluetoothPad`, `PoutD`, `Motor4`, `Battry`, etc.) wired to MC602 serial via `serial_wrap.py` / `mc602_ctl2.py`.
   - `tools/` — `Camera`, `Streamer` (Flask-based LAN preview on `:5000`), `logger` (write to `.remember/logs/`), `PID` / `PidWrap`, `CountRecord`, `IndexWrap`, `CollectControlCar`.

3. **PaddlePaddle inference** (`smartcar/paddlebaidu/`)
   - `paddle_jetson/base/infer_wrap.py` — *the local wrapper*. Three classes are what `MyCar` consumes: `YoloeInfer`, `LaneInfer`, `OCRReco`. Each loads Paddle weights from `smartcar/paddlebaidu/models/<dir>/` and supports `paddle` / `trt_fp32` / `trt_fp16` run modes.
   - `paddle_jetson/base/deploy/` — **vendored upstream PaddleDetection**. Read-only. New inference code goes in the wrapper layer above, not here.
   - `infer_cs/base/infer_back_end.py` — `InferServer`: spins up one ZMQ REQ/REP per entry in `config_car.yml → infer_cfg`, each in a daemon thread on its configured port.
   - `infer_cs/base/infer_front.py` — `ClintInterface(name)`: ZMQ client. **First-time construction triggers `check_back_python()` → `subprocess.Popen("python3 infer_back_end.py &")`** if the backend isn't already running, then polls `get_state()` until ready.
   - `ernie_bot/base/ernie_bot_wrap.py` — `ErnieBotWrap` with prompt subclasses `HumAttrPrompt`, `ActionPrompt`, `ImagePrompt`, `OrderPrompt`. Used by `get_order()` and other NLP-driven steps. Auth token is `config_car.yml → ernie_access_token`.

### Data-flow during a typical task

`my_car.cap_side.read()` → image → `my_car.task_det(img)` (REQs `tcp://127.0.0.1:5002`) → JSON boxes back → `my_car.get_detection_results()` → `Bbox` objects with normalised `pos_from_center()` → used as the error signal for a PD loop that drives `move_to_detection_target()`. OCR uses port 5004 with no resize (`img_size: Null`). Lane following uses port 5001 (`img_size: [128,128]`) and is consumed by `lane_dis_offset` in the chassis layer.

## Configuration surface

All knobs live in **`config_car.yml`** at repo root:

| Block | What it controls | When to touch |
| --- | --- | --- |
| `camera.front/side` | OpenCV video indices for the two cams | Camera re-plug |
| `speed.{x,y,angle}.limit` | Hard saturation on velocity commands in `m/s` or `rad/s` | Need to slow down for unstable tracking |
| `lane_pid` / `det_pid` (cfg_pid_y, cfg_pid_angle) | PD gains used by `lane_dis_offset` and detection-following | Lighting/reflectivity changes the lane response |
| `location_pid` (pid_x, pid_y) | Used by `move_to_position` waypoint control | Drift in odometry |
| `infer_cfg[]` | One entry per model: `name`, `infer_type` (must match class name in `infer_wrap.py`), `model_dir` (relative to `models/`), `port`, `img_size`, `run_mode` | Adding a new model or switching to TensorRT |
| `ernie_access_token` | Baidu ERNIE API bearer token | Rotating credentials |

For chassis geometry (`track`, `wheel_base`) and per-motor velocity PID, see `smartcar/whalesbot/vehicle/driver/cfg_vehicle.yaml` — these rarely change.

## Conventions and gotchas

- **Module alias `my_car`**: `car_task_function.py` declares `global my_car` inside `init()` and assumes every task function is called after `init()`. Don't import `my_car` directly; invoke through the top-level task functions.
- **`STOP_PARAM = True` is a class var on `MyCar`** that gates emergency-stop checks. `init()` sets it to `False` before each run.
- **No unit tests**; verification is *observed behaviour on the physical track*. New code paths should be exercised via `car_start_2026.py` with the upstream tasks commented out (see README workflow).
- **Chinese-only comments**: most module/function docstrings are in Chinese — match the style when adding new code.
- **`config_car.yml` uses `infer_type` strings that must exactly equal class names** in `infer_wrap.py` (`YoloeInfer`, `LaneInfer`, `OCRReco`). A typo silently produces `KeyError` at first inference. Adding a new type requires both the class and the YAML entry.
- **OCR is two-stage** (`det_model_dir` + `rec_model_dir`); detection models use `model_dir`. `infer_back_end.py` branches on `InferType == OCRReco`.
- **`smartcar/paddlebaidu/paddle_jetson/base/deploy/`** is a frozen vendor copy — don't edit. If you need a Paddle upstream change, bump the vendored copy and re-validate all four model classes.
- **Dataset dirs are gitignored** (`**/dataset/`); collected images never enter version control.
