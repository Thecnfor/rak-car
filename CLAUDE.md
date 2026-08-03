# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Competition code for the 百度智能车 (Baidu Smartcar) 2026 智慧农业 (smart-agriculture) track — an NVIDIA Orin Nano (JetPack/L4T) + MC602 controller running on a WhalesBot mecanum-wheel chassis. A single run executes **fixed-order tasks** (seed → water → scout/shoot pests → harvest → sort → read order via OCR → deliver; registry in `main/task/`). The repo is a frozen-for-competition codebase; per-track calibration happens in `config_car.yml` (chassis/PID/inference) and `task_config.yml` (per-task arm poses, slot maps, waypoint list).

## Branches (check before reading)

- **`main`** — runtime FastAPI service (car-side, `runtime/`) + HTTP/WS business client (`main/`) + the `run.py` mission orchestrator. The legacy Python monolith was deleted in commit `c23c871`. **You are here.** This file documents `main`. **All active work happens on `main`**; feature branches get merged back here quickly (e.g. `worktree-task4-pose-p` was merged in `9f5cf38`).
- **`develop/ros2-sidecar`** — a ROS2 experiment with a different top level (`ros2_ws/`, `urdf/`, `config_sensors.yml`). The Python-monolith docs do **not** apply there.
- **`feat/chassis-p0-mecanum-8.10`** / **`legacy/main`** / **`local-snapshot-0712`** — historical branches; do not target unless the user says so.

## Before you start coding

1. Read `MEMORY.md` for active constraints — especially the odom-`move_for`-only rule ([[project-odom-selfconsistent-movefor]]) and any OPEN runtime bugs (e.g. chassis realtime-velocity no-motion).
2. Glance at `.remember/recent.md` for the latest in-flight work / context the user just touched.
3. Confirm the env vars you assume match `ecosystem.config.js` — the doc table below lists defaults; production overrides win.
4. Jetson connection: **IP only, never the `orin` alias** ([[project-jetson-ip-only]]). Current reachable IP lives in [[jetson-runtime-host-current]], not in this file.

## Three entry points

The codebase has **three independent entry surfaces**. Pick the one that matches what you're doing:

### 1. Legacy monolith script — REMOVED (commit `c23c871`)

`car_start_2026.py`, `car_task_function.py`, and `car_wrap_2026.py` no longer exist, and neither does the nav-only placeholder `main/start/whole_no_task.py`. The `MyCar` facade now lives **only** inside the runtime service (`runtime/services/my_car/`). Don't write code that constructs or imports `MyCar` outside `runtime/` — use the mission entry (§1.5) or the HTTP API (§2/§3).

### 1.5 Mission orchestrator (`run.py`)

```bash
python run.py                      # full mission via main.start.orchestrator
python run.py --task 1             # single task: lane-follow to its waypoint → trigger → run → stop
python run.py --lane-hz 30         # slower outer loop for tuning
```

`run.py` is a thin shell that delegates to `main.start.orchestrator.Orchestrator` — the canonical mission entry. The orchestrator runs a 50Hz lane-following outer loop in a background thread (`DoubleLoopRunner`, pause/resume), accumulates wheel odometry in a second background thread (20 Hz), and the main thread advances through the waypoint list (tasks + 1 finish). Each waypoint waits on an IR + distance trigger (default AND), then pauses the outer loop, dispatches `main.task.TASK_RUNNERS[task_id](client)`, and resumes. The waypoint list is loaded from the `waypoints:` section of `task_config.yml` at startup; `DEFAULT_WAYPOINTS` in `main/start/orchestrator.py` is only the fallback when that load fails.

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

Default URLs (override via env vars — see "Config surface" below). **The reachable Jetson IP changes frequently — do NOT hardcode `192.168.6.231` in scripts.** Read [[jetson-runtime-host-current]] from MEMORY.md before writing any URL.
- API: `http://<JETSON_IP>:5050`
- FastAPI docs: `http://<JETSON_IP>:5050/docs`
- Stream page: `http://<JETSON_IP>:5050/stream/`
- cam1 MJPEG: `http://<JETSON_IP>:5050/video_feed/cam1`
- cam2 MJPEG: `http://<JETSON_IP>:5050/video_feed/cam2`

The runtime's job is to:
- Hold a single `MyCar()` instance; access is serialized by the two-tier `_ref_lock` / `_realtime_gate` model (the old `car_lock` is a `RuntimeError`-raising property — see "Runtime concurrency model" below).
- Run `auto_init` in the background — if the MC602 reboots, runtime rebuilds `MyCar()` automatically (see `RAK_CAR_AUTO_INIT`).
- Provide a job queue (`/v1/jobs`, `/v1/execute`) so callers don't deadlock against an init in progress.
- Expose vision results and camera streams without each caller rebuilding the inference backends.
- **Default-on `lane_feed` daemon (20 Hz)** that keeps `lane_state` fresh for the chassis outer loop — started at init, idempotent on reuse. Toggle via `/v1/execute` actions `start_lane_feed` / `stop_lane_feed`; see `runtime/VISION_API.md` for `/v1/vision/lane` and the lane-overlay stream toggle.
- **Default-on `arm_feed` daemon (20 Hz)** mirrors the same pattern for the arm: it keeps `arm_state` (y/x position, `ref_encoder`) fresh for UI / debugging. No action-level toggle — read via `GET /v1/realtime/arm/state` or WS `subscribe_arm_state`; see [main/arm/ARM_API.md](./main/arm/ARM_API.md) §2.

If you only need to drive the car (no internal changes), you should be writing a script in `main/` against `RuntimeApiClient` — **not** importing `MyCar` directly.

### 3. Business client (`main/`)

`main/` is a separate Python package that depends **only** on the runtime API via HTTP (it never imports `runtime/` or `MyCar`). It splits into subpackages — pick the one matching your area:

```bash
export RAK_CAR_SERVER_ORIGIN=http://192.168.6.231
/usr/bin/python3 -m pip install -r /home/jetson/workspace/rak-car/main/requirements.txt
python3 /home/jetson/workspace/rak-car/main/quick_start.py    # connectivity check
python3 /home/jetson/workspace/rak-car/main/car_start_api.py # API-style mission template
```

| 子包 | 用途 | 自己的 doc |
| --- | --- | --- |
| `main/arm/` | 机械臂业务：`api/`（ArmClient 聚合 8 个 mixin）、`vision/`（4-DOF 视觉伺服 + depth-aware PID + RealtimeLoop）、ArmRunner + S 曲线 dry-run + 软限位 + OriginCalibrator；`loops/`（闭环 + `VisualOrchestrator`）、`each_task/`（task1/2/4/5/6/7 业务逻辑）、`tasks/` 流程、`examples/` 模板、`tests/` 141 单测、`arm_origin.yaml` 零点标定 | [README.md](./main/arm/README.md) / [ARM_API.md](./main/arm/ARM_API.md) / [QUICKSTART.md](./main/arm/QUICKSTART.md) / [VISUAL_SERVO_QUICKREF.md](./main/arm/VISUAL_SERVO_QUICKREF.md) |
| `main/chassis/` | 底盘外环：ChassisClient + 50Hz 主循环；`controllers/` (P / Stanley / curvature_adaptive) + `loops/` (closed_loop, safety, telemetry) + `tasks/` (read_ir) + `cli/` (run_lane_follow, read_ir) + `config/` (lane_follow) | [README.md](./main/chassis/README.md) |
| `main/task/` | 8 任务编排索引：`TASK_RUNNERS = {1..7: run}` + `_config.py`（`load_task_config` 读 `task_config.yml`）；task1/2/6 逻辑在本目录（走 ArmRunner），task4/5 包装 `main/arm/each_task/`，task3/7 抛 `NotImplementedError`（orchestrator 捕获跳过）；`tests/` 14 单测 | [README.md](./main/task/README.md) |
| `main/misc/` | 单文件 mini 任务（射击、边走边打等），每个脚本可直接 `python3` 跑 | [README.md](./main/misc/README.md) |
| `main/test/` | 离线硬件冒烟脚本（arm / storage / x / 循迹），**非正式测试**，绕过 runtime 直接打硬件；改动 main/ 任务前先在这里验证 | — |

The two base clients — `RuntimeApiClient` (HTTP, `main/api_client.py`) and `RuntimeWsClient` (WebSocket, `main/ws_client.py`) — are used by all subpackages. Full action surface and parameters: [main/API_INDEX.md](./main/API_INDEX.md)（HTTP / WS / 客户端方法权威速查，含 runtime `core/actions.py` 注册表）；5-minute onboarding: [main/QUICKSTART.md](./main/QUICKSTART.md). The older `API.md` / `API_REFERENCE.md` / `CAPABILITY_LIST.md` / `BUSINESS_API_GUIDE.md` referenced in some docs are gone — `API_INDEX.md` replaced them.

## Daily operations

```bash
# --- Runtime service (on Jetson, via PM2) ---
pm2 start ecosystem.config.js           # production daemon → name "rak-car-api"
pm2 restart rak-car-api                 # after pulling new code
pm2 logs rak-car-api --lines 200
pm2 status                              # uptime + RSS
# Quick liveness probe (current reachable IP lives in MEMORY.md, not here):
curl -s http://<JETSON_IP>:5050/health

# --- Mission ---
python run.py                           # full mission via main.start.orchestrator
python run.py --task 4                  # single task: lane → trigger → run → stop
python run.py --lane-hz 30              # slower outer loop for tuning

# --- Unit tests (stdlib unittest, offline, no hardware) ---
/usr/bin/python3 -m unittest discover -s main/arm/tests -p 'test_*.py'
/usr/bin/python3 -m unittest discover -s main/task/tests -p 'test_*.py'
# Single file:
/usr/bin/python3 -m unittest main.arm.tests.test_vision_find_target -v

# --- Dependency installs ---
/usr/bin/python3 -m pip install -r runtime/requirements.txt     # runtime (FastAPI, pyudev)
/usr/bin/python3 -m pip install -r main/requirements.txt       # business client
```

**No `make`/`tox`/`poetry`/`pytest` in this repo.** Tests use stdlib `unittest`, deps are plain `pip` + `requirements.txt`. There's no linter configured — code style lives in the surrounding files (Chinese docstrings, named-args conventions, aggregator + mixin splits).

## Visual servo (机械臂视觉伺服, `main/arm/vision/`)

The arm's vision-feedback loop lives in `main/arm/vision/` (`ArmVisionClient = ServoLoop + RealtimeLoop`). It drives the **XY cross-slide only** (`x_mm` / `y_mm`); the big arm (`arm_angle`) and gripper (`hand`) are extra DOFs **frozen during the PID loop**, adjusted via the `on_strategic_4dof` callback (decoupled — the algorithm never drives them directly). **No IK by design**: 4 independent targets are packed into `composite_run`, which runs the 4 motors concurrently via `ThreadPoolExecutor`.

Stack (bottom-up): bbox parse → `TargetSelector` (8 strategies) → depth-aware PID (`mm_per_norm_eff = mm_per_norm_base × D/ref_depth_m`; `focal_length_px=600` is a heuristic, **not self-calibrated**) → S-curve dry-run → `composite_run` + soft-limit net → SDK.

Two closed-loop transports (see [TEST_PREFLIGHT.md](./main/arm/TEST_PREFLIGHT.md) §13 for the on-track learnings):

- **Position loop** — one `goto_position` per frame via `find_target` / `find_target_pid` (HTTP `/v1/vision/task`, 30 Hz). ⚠️ Position closed-loop is ~500 ms/step and queues into `arm_queue` — **backlog is the root cause of "discrete / random-walk" visual servo**; not suitable for high-frequency tracking.
- **Velocity mode (recommended)** — `POST /v1/realtime/arm-velocity` `{"x_vel": 0.0, "y_vel": 0.0}` (m/s): goes through `_realtime_gate` (no `car_lock`, **no job_queue**), direct `x_speed()/y_speed()`. y has a magnetic-safety-gate + end deceleration; **x has no soft limit — the caller must send 0 when the target is lost**. Examples: `examples/07_velocity_track_yellow_ball.py` / `08_servo4_track.py`.

Before a visual-servo run, `stop_arm_feed(force=True)` frees the serial port from the 20 Hz `arm_feed` poll (else its `goto_position` polls starve the queue); restore with `start_arm_feed`.

**`VisualOrchestrator`** (`main/arm/loops/orch_visual.py`, import from `main.arm.loops`) is the combined chassis→arm pick pipeline used by task1 — three stages: ① `track_chassis()` centers the target in frame using the chassis' two velocity DOFs; ② `track_velocity_pick()` aligns the 4-DOF arm onto the suction-nozzle setpoint in velocity mode; ③ y drops to 0 → `grasp(True)`. One-call entry: `track_and_grasp(label, chassis_max_seconds=..., arm_timeout=...)`; stages are also usable individually (`chassis_only` / `arm_only` / `grasp`). The screen-axis → motion sign mapping is field-calibrated in [orch_visual.md](./main/arm/loops/orch_visual.md) (cx→chassis vx/vy, dx→arm_angle, dy→x — read it before flipping any sign).

Docs: [VISUAL_SERVO_QUICKREF.md](./main/arm/VISUAL_SERVO_QUICKREF.md) (1-page) / [VISION_SERVO_DESIGN.md](./main/arm/VISION_SERVO_DESIGN.md) / [VISION_REALTIME_DESIGN.md](./main/arm/VISION_REALTIME_DESIGN.md) / [TEST_PREFLIGHT.md](./main/arm/TEST_PREFLIGHT.md).

## Big-picture architecture

Three layers, top-down. The names in **bold** are the files you'll touch most.

### A. Mission layer (`run.py` → orchestrator → `main/task/`)

- **`run.py`** — thin CLI shell (adds repo root to `sys.path`, parses args) delegating to `main.start.orchestrator.Orchestrator`. Flags: `--lane-hz`, `--ir-interval-s`, `--task N` (single-task run: lane-follow to that waypoint → trigger → run → stop).
- **`main/start/orchestrator.py`** — background thread A: 50 Hz lane-follow outer loop (`main.chassis.loops.closed_loop.DoubleLoopRunner`, pause/resume); thread B: 20 Hz wheel-odometry accumulation; main thread: walks waypoints (loaded from `task_config.yml` `waypoints:`, fallback `DEFAULT_WAYPOINTS`), each waits on an IR + odometry trigger (default AND) → pauses the loop → `main.task.TASK_RUNNERS[task_id](client)` → resumes. Only `main.start` should import this file — it serves `run.py` exclusively.
- **`main/task/`** — numbered task registry. Each `task{N}_*.py` exposes `run(client=None) -> Dict`; unimplemented tasks (3/7) raise `NotImplementedError`, which the orchestrator catches and skips. Business logic lives in the wrapper itself (task1/2/6, built on `main.arm.ArmRunner` + `composite_pick` / `composite_release` / `composite_run`) or in `main/arm/each_task/` (task4/5). Per-task poses/slots come from `task_config.yml` via `main/task/_config.py::load_task_config()` — **that yaml is the calibration surface when porting to a different venue**, not the task code.
- task1's vision grasp runs through `VisualOrchestrator` (see "Visual servo" above): chassis track → 4-DOF align → grasp.

The legacy monolith (`car_start_2026.py` / `car_task_function.py` / `car_wrap_2026.py`) was deleted in `c23c871`; the `MyCar(MecanumDriver, *Mixins)` facade now lives only in `runtime/services/my_car/` (§D).

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

Owns the `MyCar()` singleton, exposes POST endpoints under `/v1/*` (and legacy `/api/*`), runs the auto-init background thread, and manages the inference ZMQ backends. Full surface and architecture in [runtime/README.md](./runtime/README.md) — don't duplicate it here.

All three big modules were split into **aggregator + mixins** (2026-07):
- `runtime/services/my_car/` — `class MyCar(MecanumDriver, *Mixins)`: `pid.py` + `state_mixin` / `sensors_mixin` / `hardware_io_mixin` / `detection_mixin` / `motion_mixin` + `feeds.py` (5 daemon caches: lane / arm / task / ir / odom). `__init__` / `close` must stay in the aggregate class — `super().close()` has to resolve `MecanumDriver` along the MRO.
- `runtime/services/` — `car_runtime_service.py` aggregates 4 mixins (`controller_watcher` / `lifecycle_mixin` / `jobs_mixin` / `loops_mixin`).
- `runtime/api/` — `app.py` + `router_registry.py` + `routers/` (jobs / keypress / legacy / realtime / stream / system / vision / ws). The old monolithic `api/routes.py` is gone.

## Data-flow during a typical task

Caller POSTs `{"target":"car","name":"move_to_detection_target",...}` to `/v1/execute` (async by default → returns `job_id`). The job lands on the `arm_queue` or `car_queue` worker, which takes the `MyCar` reference under `_ref_lock` and dispatches onto the singleton (see "Runtime concurrency model" below). The same flow applies for vision reads (`/v1/vision/task`, `/v1/vision/lane`, `/v1/vision/ocr`) — see `runtime/VISION_API.md`. Detection itself: camera frame → ZMQ REQ to the inference backend → JSON boxes → normalised `pos_from_center()` → PD error signal driving `move_to_detection_target()`.

Lane following uses ZMQ port 5001 (`img_size: [128,128]`), task detection uses 5002, front detection uses 5003, OCR uses 5004 with no resize (`img_size: Null`).

## Config surface

| Where | What it controls | When to touch |
| --- | --- | --- |
| `config_car.yml` (root) | Cameras, speed limits, PID gains, infer_cfg, ERNIE token | Track-specific calibration (chassis / inference side) |
| `task_config.yml` (root) | Per-task arm poses & slot maps (`task_cfg:` section) + the orchestrator waypoint list (`waypoints:` section); loaded by `main/task/_config.py` and `Orchestrator` | Track-specific calibration (task / arm side) — port venues by editing this, not task code |
| `runtime/core/settings.py` | Runtime bind/public host:port, auto-init flags, job queue limits | Sharing IP/port with teammates |
| `main/settings.py` | Where `RuntimeApiClient` points (env-var driven) | Running `main/` from a different host |
| `ecosystem.config.js` | PM2 process definition (path, env, restart policy) | Changing the production daemon |
| `smartcar/whalesbot/vehicle/driver/cfg_vehicle.yaml` | Chassis geometry + per-motor velocity PID | Rarely; only for hardware swaps |

### Runtime env vars (set in `ecosystem.config.js` or your shell)

| Var | Default | Purpose |
| --- | --- | --- |
| `RAK_CAR_BIND_HOST` | `0.0.0.0` | API listen address |
| `RAK_CAR_BIND_PORT` | `5050` | API listen port |
| `RAK_CAR_PUBLIC_HOST` | `192.168.6.231` | Address returned to LAN clients |
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
| `RAK_INFER_EAGER_MODELS` | `lane` | Comma-separated model names that pre-load at startup. Default only `lane` (others lazy-loaded on first call) |
| `RAK_INFER_IDLE_UNLOAD_SECONDS` | `300` (doc default; **production sets `60`** in `ecosystem.config.js` for Jetson's 4GB RAM) | Idle threshold for the backend's LRU unload loop; models not called within this window are dropped |
| `RAK_INFER_FRAME_TIMEOUT_S` | `5.0` | Per-frame inference timeout. `image` request exceeding this returns `[]` instead of blocking the REP loop |
| `RAK_INFER_RSS_LIMIT_MB` | `1200` | Backend-process RSS soft cap; passed through for monitoring |
| `RAK_INFER_OOM_POLICY` | `drop_oldest` | OOM unload policy: `drop_oldest` (LRU) / `drop_ocr` / `none` |
| `RAK_CAR_MEMORY_PRESSURE_MB` | `1500` | runtime RSS threshold that triggers feed-degrade (ir→odom→arm→task, lane is never degraded) |
| `RAK_CAR_RSS_LIMIT_MB` | `1800` | runtime RSS hard limit; `>95%` also drops MJPEG quality (80→60) and resolution (×0.5) |
| `RAK_CAR_RESET_X_VELOCITY` | unset (doc); `0.04` in production | X-axis reset velocity used on `RAK_CAR_RESET_ARM` init |

**Production overrides win.** The values in `ecosystem.config.js` are what the car actually runs under; this table documents the library defaults. Also: `ecosystem.config.js` sets `max_memory_restart: "1200M"` — PM2 force-restarts the daemon if RSS exceeds 1.2GB (the runtime parent only; `infer_back_end.py` subprocess is not covered).

## Conventions and gotchas

- **Business layer never touches hardware directly**: `main/` code talks to the runtime only via `RuntimeApiClient` / `RuntimeWsClient` — it must not import `runtime/` or `MyCar`. `/v1/execute` is async by default (returns `job_id`; chained calls need explicit `sync=True` + `wait_job`), and clearing `_stop_flag` after an e-stop / cancel requires `POST /v1/control/reset-stop` before the feed daemons will run again.
- **底盘平移一律 `move_for([dx,0,0])`，禁止给 `move_to_position` 世界坐标绝对目标**：实车麦克纳姆轮 odom theta 漂移极大（单次运行实测 0.8–3.2 rad），但车头物理上是正的、odom x/y/theta 自洽。SDK 内部 `world_to_car_velocity(v, 当前theta)` 拿这个垃圾 theta 配轮速 —— 只有闭式用法（`move_for` 正用反用各一次）能抵消：世界坐标目标会转头（theta=0）或斜走（保持 theta），拿 odom x 算网格还会偏远 1/cosθ 倍。网格用沿车头相对位移自记账（参考 `main/task/task1_seeding.py` 的 `pos_along`）。2026-08-03 实车验证。
- **`STOP_PARAM`** is a class var on the runtime-side `MyCar` (`runtime/services/my_car/`) that gates emergency-stop checks.
- **Unit tests live in `main/arm/tests/` (141) and `main/task/tests/` (14)** — stdlib `unittest`, *not* pytest; they mock the HTTP/WS layer (offline, no hardware). Run: `/usr/bin/python3 -m unittest discover -s main/arm/tests -p 'test_*.py'` and `/usr/bin/python3 -m unittest discover -s main/task/tests -p 'test_*.py'`. Physical behaviour is still verified on-track via `main/arm/examples/*` against the [TEST_PREFLIGHT.md](./main/arm/TEST_PREFLIGHT.md) checklist.
- **Chinese-only comments**: most module/function docstrings are in Chinese — match the style when adding new code.

## MC602 reboot behavior (read before touching runtime init)

The MC602 periodically reboots; the runtime must rebuild `MyCar()` after each reboot. Three concurrency hazards (USB re-enumeration races, init-queue jams, lock contention) have been investigated in past debug sessions — check `.dbg/` for environment snapshots and Trae-format `.ndjson` traces (a session's artefacts live in `.dbg/<slug>.{env,ndjson}`, sometimes paired with a `debug-<slug>.md` note at the repo root — the latest session, `mc602-download-stuck`, has only the `.dbg/` artefacts). Read them before changing init/lock code, and check the `# debug-point runtime-init-queue-session` instrumentation in `runtime/services/runtime_service.py`. **Status on the controller-download-stuck issue: OPEN — don't refactor the recovery layer until it's closed.**

## Runtime concurrency model (replaces `car_lock`)

`runtime/services/runtime_service.py` 把旧 `car_lock`（RLock，全程持锁导致 arm 长动作挡住 lane 外环）拆成两层：

- **`_ref_lock`** — 只保护 `self.car` 引用替换（init / recover / close），微秒级
- **`_realtime_gate`** — realtime 端点（`/v1/realtime/*`）入口微秒级取引用
- 旧 `car_lock` 改成抛 `RuntimeError` 的 property，漏改的代码路径立即崩

`job_queue` 拆成 `arm_queue` + `car_queue` 两个独立 worker：arm worker 卡在 1-3s PID 闭环里不影响 car worker。字节流物理串行不变，但 2026-08-03 起由 SDK 的 **SerialEngine**（`serial_wrap.py`，单 io 线程 + 帧队列）统一调度，替代旧的调用方持 `serial_mc602.lock` 一问一答：写合并（`coalesce_key`，轮速突发只发最新帧）、读共享（`share_key`，encoder/模拟量并发读合并成一次物理读）、URGENT 插队（零速/急停帧优先）、心跳静默检测（有业务流量就不发 ping 帧）。`RAK_CAR_SERIAL_ENGINE=0` 可整体退回旧同步路径。离线单测：`smartcar/test/test_serial_engine.py`（16 项，无硬件）。

`/v1/execute` 默认改成异步（立即返回 `job_id`，状态查 `/v1/jobs/{id}`），旧同步调用方加 `"sync": true`。`/v1/jobs/{id}/stop` 协作取消（2026-08-03 起 SDK arm 长循环每帧查 `arm._must_stop()`，cancel/emergency-stop 会立刻中止 PID 闭环——旧的"cancel 后自然跑完"限制已解除；接线见 `my_car/__init__.py` 的 `arm._stop_flag_provider` / `arm._estop`）。

`lane_feed` / `arm_feed` 守护线程检查 `self._stop_flag`：急停或 cancel_job 后 `_stop_flag=True`，守护线程 break 退出，`lane_state.active` 变 `false`。**清 stop 必须 POST `/v1/control/reset-stop` + 重启 lane_feed**。

完整说明见 [runtime/README.md §并发任务模型](./runtime/README.md#并发任务模型)。

## Debug instrumentation

`.dbg/` contains environment snapshots and structured logs from the most recent debug sessions (`.env` files and Trae-format `.ndjson` traces). They are committed so a future session can resume the same line of investigation (the latest session, `.dbg/mc602-download-stuck.*`, has no root-level note). When you open a new debug session, mirror this layout: a `debug-<short-slug>.md` at the repo root and matching `.dbg/<short-slug>.{env,ndjson}` artefacts. `runtime/services/runtime_service.py` already emits debug points under `DEBUG_SERVER_URL` / `TRAE_DEBUG_API_URL`; check for them before adding new ones.

`.remember/` (separate, also gitignored-friendly) is the **daily-notes buffer**: `now.md` (in-progress), `today-*.md` (daily), `recent.md` (7-day rolling), `archive.md` and `archive-YYYY-MM-DD*.md` (older). The SessionStart hook injects `now.md` + `recent.md`; on questions that reach past those, grep the rotated archives yourself.

## Submission intake

`incoming/submission/` is the staging area for incoming submission tarballs (lib + model). It is gitignored — contents there are not part of the repo. If a teammate says "drop the new model here", this is the path. Cleanup after merging into `smartcar/paddlebaidu/models/`.

## Controller-only workspace (`test/controller_lab/`)

`test/` is a **separate** Python package — a controller-only lab for poking the MC602 without spinning up the full car. `test/run_controller_lab.py` boots an interactive harness (see `test/OPERATION_GUIDE.md` + `test/PROTOCOL_NOTES.md`). Useful when debugging serial/recovery without risking the chassis. Not part of `main/`.

## Pointers to deeper docs

- **Mission layer:** `main/start/orchestrator.py` docstring + `main/task/README.md` + `task_config.yml` comments (the venue-calibration surface).
- **Runtime HTTP API:** `runtime/README.md` (含并发任务模型、锁层次、双 worker 队列、/v1/execute 异步语义; ⚠️ 其目录清单早于 mixin 拆分, 实际结构见上文 §D), `runtime/STREAM_API.md`, `runtime/VISION_API.md`.
- **Business client:** `main/README.md`, `main/QUICKSTART.md`, `main/API_INDEX.md`（权威 action / HTTP / WS 速查）.
- **Business client — subpackages:** `main/arm/README.md` + `main/arm/ARM_API.md` + `main/arm/QUICKSTART.md` + `main/arm/TEST_PREFLIGHT.md`（真机测试前检查）; 视觉伺服：`main/arm/VISUAL_SERVO_QUICKREF.md` / `VISION_SERVO_DESIGN.md` / `VISION_SERVO_PLAN.md` / `VISION_REALTIME_DESIGN.md`; chassis→arm 联调流水线：`main/arm/loops/orch_visual.md`; `main/chassis/README.md`; `main/task/README.md`; `main/misc/README.md`.
- **User-facing intro:** `README.md` (the original competition-tasks overview in Chinese).
- **Controller lab:** `test/README.md`, `test/OPERATION_GUIDE.md`, `test/PROTOCOL_NOTES.md`.
- **本地端到端验证脚本：** `main/test/verify_concurrent.py`（gitignored，本地用），跑双线程探针测 lane + arm 并发。
- **Debug sessions:** `debug-*.md` notes at repo root and/or `.dbg/` artefacts (e.g. `.dbg/mc602-download-stuck.*`); each session is self-contained.