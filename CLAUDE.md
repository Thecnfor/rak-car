# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Python robotics project for an autonomous vehicle robot running on NVIDIA Jetson. The robot performs lane following, object detection, robotic arm manipulation, food sorting, and AI-powered natural language interaction. Runs as a systemd service (`py_boot.service`) at boot.

## Running

No build step — pure Python. The inference backend must be running separately before task scripts.

```bash
# Start inference server (must run first, serves ZMQ on ports 5001-5004)
python infer_cs/base/infer_back_end.py

# Primary boot entry point (also the systemd service target)
python main/qqq.py

# Competition entry points
python main/main.py
python car_start.py
python important_car.py

# HRI robot face display (PySide2/QML)
python main/hri/main.py

# Monitor service — Flask MJPEG + keypress relay on port 5000.
# The persistent frontend in /home/jetson/workspace/rak-hri is
# manifest-driven: it fetches /api/panels and renders whatever the
# backend declares. Self-contained, no import from vehicle/ or car_wrap.
python3 -m tools.streamer
# RAK_CAMERA_FORCE_MOCK=1 python3 -m tools.streamer  # no hardware needed
# RAK_STREAMER_PORT=5000 python3 -m tools.streamer   # default

# Install systemd auto-start service
sudo bash main/boot_py.sh
```

No formal test framework or linter configuration exists. Test files are in `vehicle/test/`.

## Critical Warnings

**Never run `eval()` on LLM output.** `ernie_bot/base/answer.py` lines 95/122/144 use `eval(answer)` — this is a known security vulnerability. Use `json.loads()` instead.

**Never add bare `except:` clauses.** Several exist in the codebase — always use `except Exception as e:` with logging.

**Never replace error conditions with `while True: time.sleep(1)`.** This pattern appears in `serial_wrap.py`, `controller_wrap.py`, and `vehicle_base.py` — it hangs the program forever. Raise exceptions or return error codes instead.

**Never hardcode API keys/secrets in source.** Several exist — use environment variables or `.env` files.

**Never use `eval(chassis_type)` from config.** `vehicle_base.py:309` does this — use a dict lookup instead.

## Architecture

Six-layer design, bottom-up:

**Hardware Drivers** (`vehicle/base/`) — Serial communication with MC601/MC602 motor controllers via USB/CH340. `controller_wrap.py` wraps Motors, ServoBus, Infrared, LedLight, Beep, StepperWrap, etc. `serial_wrap.py` auto-detects controller type (MC601 at 380400 baud, MC602 at 1000000 baud, MC602 wireless at 115200 baud). MC601 encoders are **simulated** (velocity×time integration); MC602 encoders are **real hardware** values.

**Vehicle Kinematics** (`vehicle/driver/`) — `CarBase` with pluggable chassis: Mecanum, Diff2, Diff4, Tricycle. Dead-reckoning odometry from wheel encoders. Forward/inverse kinematics. Velocity PID control.

**Perception** (`camera/`, `infer_cs/`, `paddle_jetson/`) — Camera captures from `/dev/cam*`. ZMQ client-server inference: `ClintInterface` sends images to `InferServer` which hosts PaddlePaddle models (YOLOE detection, lane segmentation, OCR, human attributes, MOT tracking).

**Task Execution** (`task_func.py`, `car_wrap.py`) — `MyTask` controls arm/ejection/servos. `MyCar` (extends `CarBase`) adds lane-following PID, object-detection navigation, sensor navigation, target localization.

**Application Scripts** (top-level `.py`, `main/`) — Competition scripts combining driving, detection, arm manipulation. Tasks: Hanoi tower, food sorting, plant watering, BMI analysis, object fetching. AI integration via ErnieBot/OpenAI for NL understanding.

**HRI** (`main/hri/`) — PySide2/QML robot face display.

## Key Classes

- `MyCar` (`car_wrap.py`) — Central orchestrator: driving + perception + tasks. **1438-line God Object** — see docs/ for decomposition plan.
- `CarBase` (`vehicle/driver/vehicle_base.py`) — Chassis kinematics and motor control
- `ArmBase` (`vehicle/arm/arm_base.py`) — Robotic arm with stepper motors, servos, vacuum pump
- `MyTask` / `Ejection` (`task_func.py`) — Task-level primitives. Uses two-phase `arm_set` pattern.
- `ClintInterface` (`infer_cs/base/infer_front.py`) — ZMQ inference client. Auto-launches `infer_back_end.py` if not running.
- `InferServer` (`infer_cs/base/infer_back_end.py`) — ZMQ inference server hosting models
- `Camera` (`camera/base/camera.py`) — Threaded USB camera capture (daemon thread, no lock on `self.frame`)
- `Camera` (`tools/base/camera.py`) — Lightweight cv2-only Camera used by `tools/streamer.py`. Auto-falls-back to synthetic frames when `/dev/cam*` is missing. Deliberately does NOT import the rest of rak-car.
- `Streamer` (`tools/streamer.py`) — Flask service on port 5000. Exposes a *manifest-driven* panel surface. Endpoints: `GET /api/panels` (manifest), `GET /api/health`, `GET /api/cameras/<front|side>/{mjpeg,snapshot}`, `GET /api/cameras/<front|side>/detections/{mjpeg,snapshot.jpg}` (with boxes drawn server-side), `GET /api/state/{pose,velocity,battery}`, `GET /api/sensors/all`, `GET /api/detections`, `POST/GET/DELETE /api/keypress`. Standalone — `python3 -m tools.streamer`.
- `Panel` + `PanelRegistry` (`tools/panels.py`) — Self-describing panel model. The frontend fetches `/api/panels` and renders the grid from the manifest. Adding a new feature is `REGISTRY.register(Panel(...))` in `_register_panels` of `Streamer`.
- `MockDetector` + `draw_detections` (`tools/detector.py`) — Synthesizes plausible detection tracks (PERSON/VEHICLE/SIGN/PLANT/PEST/QR with smooth motion + confidence oscillation + trails) and draws them on frames. Replace with a real ZMQ-backed detector (using `MyCar`'s `ClintInterface.task_det`) to switch to real inference.
- `RobotState` (`tools/state.py`) — Synthetic in-memory robot state (pose, velocity, battery, IR, last key) that animates over time so the UI has data when no real commands are coming in.
- `ErnieBotWrap` / `OpenAiWrap` (`ernie_bot/`) — LLM integration with JSON schema prompts

## The `ctl_id` Global Dispatch Pattern

This is the most pervasive architectural pattern. Every hardware class in `controller_wrap.py` holds both MC601 and MC602 instances and dispatches via a global `ctl_id` (0 or 1):

```python
ctl_id = get_devid()  # Set at import time, never changes

class Motors():
    def __init__(self, port_id):
        self.motor_1 = Motor_1(port=port_id)   # MC601 impl
        self.motor_2 = Motor_2(port_id=port_id) # MC602 impl

    def set_speed(self, speed):
        fucs = [self.motor_1.rotate, self.motor_2.set_speed]
        fucs[ctl_id](speed)  # Global dispatch
```

This pattern repeats in 20+ classes (Motors, ServoBus, Infrared, Key4Btn, etc.). `NoneDev` is used as a placeholder for unsupported features on MC601 — calling its methods hangs forever.

## Import Mechanics

The project uses `sys.path.append` extensively (30+ occurrences) instead of proper package structure. Many `*/base/` subdirectories lack `__init__.py`. When adding new modules, follow the existing `sys.path.append(os.path.abspath(...))` pattern rather than trying to fix the package structure.

**Import-time side effects:** Importing `vehicle` triggers serial port scanning (`serial_wrap.py:352`), controller detection (`controller_wrap.py:37`), and potentially firmware download. This means `import vehicle` cannot run without hardware connected.

## Configuration Files

- `config_car.yml` — Camera indices, IO pins, speed limits, PID params (lane/detection/location)
- `vehicle/driver/cfg_vehicle.yaml` — Chassis type, wheel dimensions, velocity PID
- `vehicle/arm/arm_cfg.yaml` — Arm motor ports, stepper/servo config, PID, limits
- `infer_cs/base/infer.yaml` — Inference service definitions (ports 5001-5004)
- `vehicle/base/mc602_cfg.yaml` — MC602 controller calibration

Config loading is inconsistent: `config_car.yml` and `infer.yaml` use `get_yaml()` from `tools/`; others use `yaml.load()` directly. When modifying config, match the existing pattern for that file.

## Inference Services

| Name  | Type       | Port | Model              | Purpose            |
|-------|------------|------|--------------------|--------------------|
| lane  | LaneInfer  | 5001 | (built-in)         | Lane segmentation  |
| task  | YoloeInfer | 5002 | task_wbt2025       | Task object detect |
| front | YoloeInfer | 5003 | front_model2       | Front detection    |
| ocr   | OCRReco    | 5004 | ch_PP-OCRv3_rec    | Text recognition   |

## Competition Script Duplication

The `main/` directory contains many near-copies of competition scripts (`qqq.py`, `main.py`, `finalall.py`, `scripy1-5.py`). Only `main/qqq.py` (systemd boot target) and `main/main.py` (most complete) are active. Functions like `get_key_by_value()` and `index_form` are copy-pasted across 15+ files.

## Dependencies

Python packages (pre-installed on Jetson): opencv-python, numpy, pyserial, simple_pid, paddlepaddle, erniebot, pyzmq, PySide2, psutil, PyYAML, jsonschema.

## Detailed Documentation

Comprehensive project docs are in `docs/` — see `docs/README.md` for the full index. Key topics:
- `docs/hardware-comm.md` — MC601/MC602 frame formats, full command tables, encoding/decoding formulas
- `docs/vehicle-system.md` — Mecanum kinematics formulas, odometry update algorithm
- `docs/known-issues.md` — All known bugs, security issues, and technical debt with severity ratings
