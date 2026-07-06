# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Python robotics project for an autonomous vehicle robot running on NVIDIA Jetson. The robot performs lane following, object detection, robotic arm manipulation, food sorting, and AI-powered natural language interaction. Runs as a systemd service (`py_boot.service`) at boot.

## Development vs Target Architecture

> **重要**: 本项目采用 **dev/target 双机架构**，所有开发、测试、可视化在 dev 桌面完成，Jetson 只跑 sidecar 节点。

- **Dev 桌面机**（当前 Ubuntu 26.04，需装 ROS2 — 推荐 Docker 跑 Jazzy）：编辑、pytest、gtest、Gazebo 仿真、RViz 可视化
- **Jetson Orin Nano 4GB**（JetPack 6 + Ubuntu 22.04 + ROS2 Humble base）：SSH `xrak@orin`，跑 sidecar 节点发布真实传感器数据
- 详见 [`docs/development/README.md`](docs/development/README.md)

**Dev 工作流**（5 步日常循环）：

```bash
# 1. 编辑
$EDITOR ~/work/rak-car/...

# 2. 推 Jetson
push2orin  # 见 ssh-workflow.md

# 3. 远程 build
ssh orin "cd ~/ros2_ws && colcon build --packages-up-to vehicle_wbt_*"

# 4. dev 上跑测试
dev-ros2 pytest ~/work/rak-car/ros2_ws/src/vehicle_wbt_platform/test/

# 5. dev 上启 RViz 看 Jetson 真实数据
dev-ros2 rviz2
```

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
