# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Competition code for the 百度智能车 (Baidu Smartcar) 2026 智慧农业 (smart-agriculture) track — a Jetson Nano + MC602 controller running on a WhalesBot mecanum-wheel chassis. A single run executes **8 fixed-order tasks** (seed → scout pests → water → shoot pests → harvest → sort → read order via OCR → deliver). The repo is a frozen-for-competition codebase; per-track calibration happens in `config_car.yml` and the odometry offsets hardcoded inside `car_task_function.py`.

## Branches (check before reading)

- **`main`** — the live Python monolith + runtime FastAPI service. **You are here.** This file documents `main`.
- **`develop/ros2-sidecar`** — a ROS2 experiment with a different top level (`ros2_ws/`, `urdf/`, `config_sensors.yml`). The Python-monolith docs do **not** apply there.
- **`feat/chassis-p0-mecanum-8.10`** / **`legacy/main`** / **`local-snapshot-0712`** — work-in-progress and historical branches; do not target unless the user says so.

## Three entry points

The codebase has **three independent entry surfaces**. Pick the one that matches what you're doing:

### 1. Legacy monolith script (CLI / REPL — one-shot runs)

```bash
python car_start_2026.py          # full 8-task mission
python main/quick_start.py        # (legacy) sanity check
```

`car_start_2026.py` calls `init()` then each task in sequence — comment out the others to run a single task. This path constructs `MyCar()` directly in the calling process; inference backends are **auto-spawned** the first time `MyCar()` is constructed.

### 2. Runtime API service (production / remote / debug — preferred for daily work)

The car is normally driven over HTTP from a separate machine (or the Jetson itself). The runtime service is a FastAPI app that owns the `MyCar()` singleton and exposes everything as POST endpoints:

```bash
# install once
/usr/bin/python3 -m pip install -r /home/jetson/workspace/rak-car/runtime/requirements.txt

# dev
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m runtime.server           # serves 0.0.0.0:5050

# production (what the car actually runs under)
pm2 start ecosystem.config.js               # → process name "rak-car-api"
pm2 logs rak-car-api
pm2 restart rak-car-api                     # after pulling new code
```

Default URLs (override via env vars — see "Config surface" below):
- API: `http://192.168.3.60:5050`
- FastAPI docs: `http://192.168.3.60:5050/docs`
- Stream page: `http://192.168.3.60:5050/stream/`
- cam1 MJPEG: `http://192.168.3.60:5050/video_feed/cam1`
- cam2 MJPEG: `http://192.168.3.60:5050/video_feed/cam2`

The runtime's job is to:
- Hold a single `MyCar()` instance and serialize access through `car_lock`.
- Run `auto_init` in the background — if the MC602 reboots, runtime rebuilds `MyCar()` automatically (see `RAK_CAR_AUTO_INIT`).
- Provide a job queue (`/v1/jobs`, `/v1/execute`) so callers don't deadlock against an init in progress.
- Expose vision results and camera streams without each caller rebuilding the inference backends.

If you only need to drive the car (no internal changes), you should be writing a script in `main/` against `RuntimeApiClient` — **not** importing `MyCar` directly.

### 3. Business client (`main/`)

`main/` is a separate Python package that depends **only** on the runtime API via HTTP. It splits into three subpackages — pick the one matching your area:

```bash
export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
/usr/bin/python3 -m pip install -r /home/jetson/workspace/rak-car/main/requirements.txt
python3 /home/jetson/workspace/rak-car/main/quick_start.py    # connectivity check
python3 /home/jetson/workspace/rak-car/main/car_start_api.py # API-style mission template
```

| 子包 | 用途 | 自己的 doc |
| --- | --- | --- |
| `main/arm/` | 机械臂业务：ArmClient + ArmRunner + S 曲线 dry-run + 软限位 + OriginCalibrator | [README.md](./main/arm/README.md) / [ARM_API.md](./main/arm/ARM_API.md) / [QUICKSTART.md](./main/arm/QUICKSTART.md) |
| `main/chassis/` | 底盘外环：ChassisClient + 50Hz `controllers/` + `loops/` 主循环 + 安全门 | [README.md](./main/chassis/README.md) |
| `main/misc/` | 单文件 mini 任务（射击、边走边打等），每个脚本可直接 `python3` 跑 | [README.md](./main/misc/README.md) |

The two base clients — `RuntimeApiClient` (HTTP, `main/api_client.py`) and `RuntimeWsClient` (WebSocket, `main/ws_client.py`) — are used by all three subpackages. Full action surface and parameters: [main/API_REFERENCE.md](./main/API_REFERENCE.md) / [main/API.md](./main/API.md) / [main/CAPABILITY_LIST.md](./main/CAPABILITY_LIST.md) / [main/BUSINESS_API_GUIDE.md](./main/BUSINESS_API_GUIDE.md).

## Big-picture architecture

Three layers, top-down. The names in **bold** are the files you'll touch most.

### A. Top-level scripts (monolith path only)

- `car_start_2026.py` — thin `main()` that calls `init()` then each task in sequence.
- `car_task_function.py` — the 8 task functions. Each is mostly a hand-tuned sequence of `my_car.move_to_position(...)`, `my_car.move_to_detection_target()`, `my_car.arm.grasp(...)`, etc. Coordinates here (e.g. `cylinder_loc` in `auto_seeding`) are *track-specific magic numbers* — they are the calibration surface when porting to a different venue.
- **`car_wrap_2026.py`** — defines `class MyCar(MecanumDriver)`. This is the single facade the task layer talks to; it's also what the runtime owns. Construction order is: chassis → screen → streamer → arm → sensor (`Key4Btn`, `ServoPwm` for storage, `BluetoothPad`, `PoutD` for shooter) → PID → cameras → paddle infer → ERNIE bot → key-press daemon thread.

### B. WhalesBot hardware SDK (`smartcar/whalesbot/`)

- `vehicle/driver/mecanum.py` + `vehicle_base.py` — `MecanumDriver` (base of `MyCar`), implements `move_for`, `move_to_position` (waypoint with `location_pid`), `get_odometry`, `lane_dis_offset` (follow lane for `dis_hold` meters), and chassis geometry from `cfg_vehicle.yaml` (`track=0.30`, `wheel_base=0.28`).
- `vehicle/arm/arm_base.py` — `ArmController`: `reset_position`, `set_arm_pose(arm_id, pitch, "LEFT"/"RIGHT", "UP"/"DOWN")`, `grasp(bool)` (vacuum on/off), `move_x_position`, `move_y_position`. Poses are referenced by string direction constants, not by joint angles — look up the enum in `arm_base.py` before adding joints.
- `vehicle/base/controller_wrap.py` — per-pin peripherals (`Beep`, `Key4Btn`, `Infrared`, `NixieTube`, `ServoPwm`, `BluetoothPad`, `PoutD`, `Motor4`, `Battry`, etc.) wired to MC602 serial via `serial_wrap.py` / `mc602_ctl2.py`.
- `tools/` — `Camera`, `Streamer` (Flask-based LAN preview on `:5000`, now superseded by the runtime service), `logger` (writes to `.remember/logs/`), `PID` / `PidWrap`, `CountRecord`, `IndexWrap`, `CollectControlCar`.

### C. PaddlePaddle inference (`smartcar/paddlebaidu/`)

- `paddle_jetson/base/infer_wrap.py` — *the local wrapper*. Three classes are what `MyCar` consumes: `YoloeInfer`, `LaneInfer`, `OCRReco`. Each loads Paddle weights from `smartcar/paddlebaidu/models/<dir>/` and supports `paddle` / `trt_fp32` / `trt_fp16` run modes.
- `paddle_jetson/base/deploy/` — **vendored upstream PaddleDetection**. Read-only. New inference code goes in the wrapper layer above, not here.
- `infer_cs/base/infer_back_end.py` — `InferServer`: spins up one ZMQ REQ/REP per entry in `config_car.yml → infer_cfg`, each in a daemon thread on its configured port.
- `infer_cs/base/infer_front.py` — `ClintInterface(name)`: ZMQ client. First-time construction triggers `check_back_python()` → `subprocess.Popen("python3 infer_back_end.py &")` if the backend isn't already running, then polls `get_state()` until ready.
- `ernie_bot/base/ernie_bot_wrap.py` — `ErnieBotWrap` with prompt subclasses `HumAttrPrompt`, `ActionPrompt`, `ImagePrompt`, `OrderPrompt`. Used by `get_order()` and other NLP-driven steps. Auth token is `config_car.yml → ernie_access_token`.

### D. Runtime service (`runtime/`)

Owns the `MyCar()` singleton, exposes POST endpoints under `/v1/*` (and legacy `/api/*`), serializes access through `car_lock`, runs the auto-init background thread, and manages the inference ZMQ backends. Full surface and architecture in [runtime/README.md](./runtime/README.md) — don't duplicate it here.

## Data-flow during a typical task

**Legacy monolith:** `my_car.cap_side.read()` → image → `my_car.task_det(img)` (REQs `tcp://127.0.0.1:5002`) → JSON boxes → `my_car.get_detection_results()` → `Bbox` objects with normalised `pos_from_center()` → used as the error signal for a PD loop that drives `move_to_detection_target()`.

**Runtime path:** Caller POSTs `{"target":"car","name":"move_to_detection_target",...}` to `/v1/execute`. The service enqueues a job, holds `car_lock`, dispatches onto the same `MyCar()` singleton, returns the result. The same flow applies for vision reads (`/v1/vision/task`, `/v1/vision/lane`, `/v1/vision/ocr`) — see `runtime/VISION_API.md`.

Lane following uses ZMQ port 5001 (`img_size: [128,128]`), task detection uses 5002, front detection uses 5003, OCR uses 5004 with no resize (`img_size: Null`).

## Config surface

| Where | What it controls | When to touch |
| --- | --- | --- |
| `config_car.yml` (root) | Cameras, speed limits, PID gains, infer_cfg, ERNIE token | Track-specific calibration |
| `runtime/core/settings.py` | Runtime bind/public host:port, auto-init flags, job queue limits | Sharing IP/port with teammates |
| `main/settings.py` | Where `RuntimeApiClient` points (env-var driven) | Running `main/` from a different host |
| `ecosystem.config.js` | PM2 process definition (path, env, restart policy) | Changing the production daemon |
| `smartcar/whalesbot/vehicle/driver/cfg_vehicle.yaml` | Chassis geometry + per-motor velocity PID | Rarely; only for hardware swaps |

### Runtime env vars (set in `ecosystem.config.js` or your shell)

| Var | Default | Purpose |
| --- | --- | --- |
| `RAK_CAR_BIND_HOST` | `0.0.0.0` | API listen address |
| `RAK_CAR_BIND_PORT` | `5050` | API listen port |
| `RAK_CAR_PUBLIC_HOST` | `192.168.3.60` | Address returned to LAN clients |
| `RAK_CAR_PUBLIC_STREAM_PORT` | = BIND_PORT | Where the camera stream is reachable |
| `RAK_CAR_PUBLIC_STREAM_PATH` | `/stream/` | Stream page path |
| `RAK_CAR_AUTO_INIT` | `1` | Background auto-recover `MyCar()` when MC602 reboots |
| `RAK_CAR_RESET_ARM` | `0` | Reset arm on auto-init |
| `RAK_CAR_RESET_POSITION_ON_INIT` | `1` | Zero odometry on init |
| `RAK_CAR_STOP_AFTER_ACTION` | `0` | Hard-stop chassis after each action |
| `RAK_CAR_INFER_AUTO_START` | `1` | Spawn `infer_back_end.py` ZMQ servers on startup |
| `RAK_CAR_INFER_POLL_INTERVAL` | `1.0` | Seconds between backend-ready polls |
| `RAK_CAR_INFER_READY_TIMEOUT` | `45` | Max seconds to wait for a backend before failing health |
| `RAK_CAR_INFER_HEALTH_TIMEOUT` | `2.0` | Per-call timeout used by `/v1/health` when probing inference |

## Conventions and gotchas

- **Module alias `my_car`**: `car_task_function.py` declares `global my_car` inside `init()` and assumes every task function is called after `init()`. Don't import `my_car` directly; invoke through the top-level task functions.
- **`STOP_PARAM = True` is a class var on `MyCar`** that gates emergency-stop checks. `init()` sets it to `False` before each run.
- **No unit tests**; verification is *observed behaviour on the physical track*. New code paths should be exercised via `car_start_2026.py` with the upstream tasks commented out.
- **Chinese-only comments**: most module/function docstrings are in Chinese — match the style when adding new code.

## MC602 reboot behavior (read before touching runtime init)

The MC602 periodically reboots; the runtime must rebuild `MyCar()` after each reboot. Three concurrency hazards (USB re-enumeration races, init-queue jams, lock contention) are tracked in `debug-*.md` files at the repo root — read them before changing init/lock code, and check the `# debug-point runtime-init-queue-session` instrumentation in `runtime/services/runtime_service.py`. **Status on the controller-download-stuck issue: OPEN — don't refactor the recovery layer until it's closed.**

## Debug instrumentation

`.dbg/` contains environment snapshots and structured logs from the most recent debug sessions (`.env` files and Trae-format `.ndjson` traces). They are committed alongside the corresponding `debug-*.md` so a future session can resume the same line of investigation. When you open a new debug session, mirror this layout: a `debug-<short-slug>.md` at the repo root and matching `.dbg/<short-slug>.{env,ndjson}` artefacts. `runtime/services/runtime_service.py` already emits debug points under `DEBUG_SERVER_URL` / `TRAE_DEBUG_API_URL`; check for them before adding new ones.

## Submission intake

`incoming/submission/` is the staging area for incoming submission tarballs (lib + model). It is gitignored — contents there are not part of the repo. If a teammate says "drop the new model here", this is the path. Cleanup after merging into `smartcar/paddlebaidu/models/`.

## Controller-only workspace (`test/controller_lab/`)

`test/` is a **separate** Python package — a controller-only lab for poking the MC602 without spinning up the full car. `run_controller_lab.py` boots an interactive harness (see `test/OPERATION_GUIDE.md` + `test/PROTOCOL_NOTES.md`). Useful when debugging serial/recovery without risking the chassis. Not part of `main/`.

## Pointers to deeper docs

- **Legacy monolith path:** this file (sections A–C above) + `car_wrap_2026.py` + `config_car.yml` comments.
- **Runtime HTTP API:** `runtime/README.md`, `runtime/STREAM_API.md`, `runtime/VISION_API.md`.
- **Business client:** `main/README.md`, `main/API.md`, `main/API_REFERENCE.md`, `main/BUSINESS_API_GUIDE.md`, `main/CAPABILITY_LIST.md`.
- **Business client — subpackages:** `main/arm/README.md` + `main/arm/ARM_API.md` + `main/arm/QUICKSTART.md`; `main/chassis/README.md`; `main/misc/README.md`.
- **User-facing intro:** `README.md` (the original competition-tasks overview in Chinese).
- **Controller lab:** `test/README.md`, `test/OPERATION_GUIDE.md`, `test/PROTOCOL_NOTES.md`.
- **Debug sessions:** `debug-*.md` at repo root (each is self-contained).