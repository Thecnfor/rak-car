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

# Install systemd auto-start service
sudo bash main/boot_py.sh
```

No formal test framework or linter configuration exists. Test files are in `vehicle/test/`.

## Architecture

Six-layer design, bottom-up:

**Hardware Drivers** (`vehicle/base/`) — Serial communication with MC601/MC602 motor controllers. `controller_wrap.py` wraps Motors, ServoBus, Infrared, LedLight, Beep, StepperWrap, etc. `serial_wrap.py` auto-detects controller type.

**Vehicle Kinematics** (`vehicle/driver/`) — `CarBase` with pluggable chassis: Mecanum, Diff2, Diff4, Tricycle. Dead-reckoning odometry from wheel encoders. Forward/inverse kinematics. Velocity PID control.

**Perception** (`camera/`, `infer_cs/`, `paddle_jetson/`) — Camera captures from `/dev/cam*`. ZMQ client-server inference: `ClintInterface` sends images to `InferServer` which hosts PaddlePaddle models (YOLOE detection, lane segmentation, OCR, human attributes, MOT tracking).

**Task Execution** (`task_func.py`, `car_wrap.py`) — `MyTask` controls arm/ejection/servos. `MyCar` (extends `CarBase`) adds lane-following PID, object-detection navigation, sensor navigation, target localization.

**Application Scripts** (top-level `.py`, `main/`) — Competition scripts combining driving, detection, arm manipulation. Tasks: Hanoi tower, food sorting, plant watering, BMI analysis, object fetching. AI integration via ErnieBot/OpenAI for NL understanding.

**HRI** (`main/hri/`) — PySide2/QML robot face display.

## Key Classes

- `MyCar` (`car_wrap.py`) — Central orchestrator: driving + perception + tasks
- `CarBase` (`vehicle/driver/vehicle_base.py`) — Chassis kinematics and motor control
- `ArmBase` (`vehicle/arm/arm_base.py`) — Robotic arm with stepper motors, servos, vacuum pump
- `MyTask` / `Ejection` (`task_func.py`) — Task-level primitives
- `ClintInterface` (`infer_cs/base/infer_front.py`) — ZMQ inference client
- `InferServer` (`infer_cs/base/infer_back_end.py`) — ZMQ inference server hosting models
- `Camera` (`camera/base/camera.py`) — Threaded USB camera capture
- `ErnieBotWrap` / `OpenAiWrap` (`ernie_bot/`) — LLM integration with JSON schema prompts

## Configuration Files

- `config_car.yml` — Camera indices, IO pins, speed limits, PID params (lane/detection/location)
- `vehicle/driver/cfg_vehicle.yaml` — Chassis type, wheel dimensions, velocity PID
- `vehicle/arm/arm_cfg.yaml` — Arm motor ports, stepper/servo config, PID, limits
- `infer_cs/base/infer.yaml` — Inference service definitions (ports 5001-5004)

## Inference Services

| Name  | Type       | Port | Model              | Purpose            |
|-------|------------|------|--------------------|--------------------|
| lane  | LaneInfer  | 5001 | (built-in)         | Lane segmentation  |
| task  | YoloeInfer | 5002 | task_wbt2025       | Task object detect |
| front | YoloeInfer | 5003 | front_model2       | Front detection    |
| ocr   | OCRReco    | 5004 | ch_PP-OCRv3_rec    | Text recognition   |

## Dependencies

Python packages (pre-installed on Jetson): opencv-python, numpy, pyserial, simple_pid, paddlepaddle, erniebot, pyzmq, PySide2, psutil, PyYAML, jsonschema.
