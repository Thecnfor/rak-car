# Repository Guidelines

Guidance for AI assistants working in `rak-car`. `CLAUDE.md` is the authoritative deep reference; this file is the operational summary. When they disagree, trust code over docs.

## Project Overview

Baidu Smartcar 2026 competition robot (smart-agriculture track): a WhalesBot mecanum-wheel car with an NVIDIA Jetson upper computer (≈2–4 GB RAM, L4T) and an MC602 MCU lower computer over USB serial. The car executes an 8-task autonomous mission (seeding → scouting → watering → shooting → harvesting → sorting → order pickup → delivery) using lane following, YOLO detection, OCR, and a 4-DOF arm.

## Architecture & Data Flow

Three layers split by a **hard network boundary**:

```
[Controller laptop: main/]                 [Jetson car: runtime/  (PM2 app "rak-car-api", :5050)]
 run.py → Orchestrator                      FastAPI (runtime/api/app.py)
   ├ ChassisClient (50 Hz lane loop)          ├ /v1/* HTTP + /v1/ws WebSocket + MJPEG /video_feed/*
   ├ TASK_RUNNERS[1..7] (main/task/)          ├ CarRuntimeService (arm_queue/car_queue workers)
   └ ArmClient (main/arm/)                    │    └ CAR_ACTIONS/ARM_ACTIONS → MyCar (runtime/services/my_car/)
        │                                     ├ InferBackendService ──subprocess──► infer_back_end.py
  RuntimeApiClient / RuntimeWsClient ────────►│    (ZMQ REQ/REP: lane:5001 task:5002 ocr:5004)
        HTTP + WS (only channel!)             └ ControllerSessionManager → SerialEngine ─serial─► MC602
```

- **`runtime/` (car-side server)** owns ALL hardware: the sole `MyCar` instance, cameras (cam1=front/lane, cam2=side/task), and the managed Paddle inference subprocess. FastAPI + uvicorn, port 5050, **no auth** (LAN trust model).
- **`main/` (business client)** NEVER imports `runtime/` or `smartcar/` — it talks only via `RuntimeApiClient` (HTTP) and `RuntimeWsClient` (WS).
- **`smartcar/` (SDK)** hardware drivers + Paddle wrappers, consumed by runtime only.
- **Mission flow**: `run.py` → `main/start/orchestrator.py` runs a 50 Hz lane-follow outer loop + odometry thread, walks waypoints from `task_config.yml` (IR + odometry triggers), and dispatches `TASK_RUNNERS[N](client)`.

### Runtime concurrency model (load-bearing — do not break)

- `/v1/execute` is **async by default** (returns `job_id`; pass `"sync": true` for blocking). Jobs land in two isolated queues: `arm_queue` (long PID actions) / `car_queue` (short actions).
- `/v1/realtime/*` and WS `realtime/*` ops **bypass queues** (µs `_realtime_gate`) — the only path safe for 50 Hz wheel/arm velocity control.
- Lock hierarchy: `_ref_lock` (car ref swap) / `_realtime_gate` (ref grab). `car_lock` is a RuntimeError-raising property — never use it.
- Five feed daemon threads (lane/arm/task/ir/odom) cache state in `CameraStreamService._StateCache`; readers never touch hardware.
- Serial bytes are serialized by `SerialEngine` (single IO thread, write-coalescing via `coalesce_key`, read-sharing via `share_key`, URGENT>NORMAL>READ).
- After e-stop/cancel, `_stop_flag` kills feeds: `POST /v1/control/reset-stop` + restart `start_lane_feed` before loops work again.
- OOM governance (Jetson): lazy model load (`lane` eager via `RAK_INFER_EAGER_MODELS`), LRU unload (`RAK_INFER_IDLE_UNLOAD_SECONDS=60`), feed Hz degrade under RSS pressure, `POST /v1/infer/drop-oldest` escape hatch, PM2 restarts at 1.2 GB.

## Key Directories

| Path | Purpose |
|---|---|
| `runtime/` | Car-side FastAPI service: `api/routers/`, `core/` (settings, actions), `services/` (MyCar + mixins, jobs, feeds, inference supervisor), `hardware/` (MC602 session/probe/recovery) |
| `main/` | Business client: `arm/` (ArmClient mixins, visual servo, S-curve), `chassis/` (50 Hz outer loop, controllers: P/Stanley/curvature), `task/` (TASK_RUNNERS 1–7), `start/` (orchestrator), `misc/` (LLM scripts), `test/` (HTTP smoke, gitignored) |
| `smartcar/whalesbot/` | Vehicle SDK, 4 tiers: `vehicle/base/serial_wrap.py` (SerialEngine + MC602 framing) → `mc602_ctl2.py` (`*_2` device classes) → `controller_wrap.py` (unit conversion) → `arm/arm_base.py` + `driver/mecanum.py` (behavior). `tools/` = Camera, Streamer, PID, logger |
| `smartcar/paddlebaidu/` | Paddle Inference wrappers (`paddle_jetson/base/infer_wrap.py`: YoloeInfer, OCRReco, LaneBlendInfer), ZMQ client/server (`infer_cs/`), ERNIE wrapper. `deploy/` is **vendored PaddleDetection — read-only**. Models live in `smartcar/paddlebaidu/models/` on the Jetson only |
| `test/` | Standalone MC602 protocol lab CLI (`run_controller_lab.py`) — independent of business code; motion gated behind `--dangerous`. Gitignored |
| `docs/superpowers/`, `.trae/` | Design specs/plans (history + invariants); not runtime docs |
| `web/` | Vite vanilla-TS multi-page frontend, built on the dev machine (`cd web && npm run build` → `runtime/static_web/`, served at `/console/` by FastAPI StaticFiles; car needs no node): `monitor/` = migrated stream page, `teach/` = teach pendant (WASD/QE/RF jog via `/v1/realtime/arm-velocity`, pose library in localStorage, soft e-stop) |

## Development Commands

```bash
# Install (Jetson system Python; range-pinned deps, no lockfiles)
/usr/bin/python3 -m pip install -r runtime/requirements.txt   # car-side
python3 -m pip install -r main/requirements.txt               # client-side

# Run runtime (dev) — binds 0.0.0.0:5050
/usr/bin/python3 -m runtime.server

# Production (Jetson only — paths are /home/jetson/workspace/rak-car)
pm2 start ecosystem.config.js            # registers "rak-car-api"
pm2 restart rak-car-api                  # after code pull
pm2 delete rak-car-api && pm2 start ecosystem.config.js   # REQUIRED after env changes
pm2 logs rak-car-api

# Mission
python3 run.py [--lane-hz 50] [--ir-interval-s 0.02] [--task 1-7]

# Client checks
export RAK_CAR_SERVER_ORIGIN=http://192.168.6.231   # production car IP
python3 main/quick_start.py                          # connectivity smoke
python3 -m main.chassis.cli.run_lane_follow --dry-run

# Tests (stdlib unittest — see Testing & QA)
/usr/bin/python3 -m unittest discover -s main/arm/tests -p 'test_*.py'
/usr/bin/python3 -m unittest discover -s main/task/tests -p 'test_*.py'
RAK_CAR_SERIAL_AUTO_CONNECT=0 /usr/bin/python3 -m unittest smartcar.test.test_serial_engine -v

# MC602 lab (hardware on /dev/ttyUSB*)
python3 test/run_controller_lab.py probe | raw "02 01 10" | sensor infrared
# Motion/flashing commands require explicit --dangerous / --yes-download flags

# Health triage order (against the car)
GET /v1/health → GET /v1/infer/state → GET /stream/health
```

## Code Conventions & Common Patterns

- **Hard boundaries**: `main/` never imports `runtime/`/`smartcar/`; `MyCar` exists only inside `runtime/services/my_car/`. Frozen zones for `main/`-only work: `runtime/`, `smartcar/`, `config_car.yml`, `ecosystem.config.js`. Public endpoints must stay byte-compatible.
- **Chinese comments/docstrings** everywhere — match the style. Calibration YAMLs carry datestamped Chinese tuning notes (e.g. `2026-08-02 21:36 用户:`); preserve them.
- **Action registries, not decorators**: `runtime/core/actions.py` defines `CAR_ACTIONS`/`ARM_ACTIONS` as plain lambda tables; `/v1/execute` dispatches `{target, name, args}`. `/v1/execute` accepts targets `car`/`arm`/`system` only (task actions were removed — tasks are client-side).
- **Mixin composition**: `CarRuntimeService(ControllerWatcherMixin, LifecycleMixin, JobsMixin, LoopsMixin)`; `MyCar(MecanumDriver, StateMixin, SensorsMixin, HardwareIOMixin, DetectionMixin, MotionMixin, FeedsMixin)`; `ArmClient` = 8 mixins. Keep new methods in the right mixin.
- **Module singletons**: `service`/`camera_stream_service` in `runtime/api/app.py`, `get_controller_session()`, module-level `serial_wrap`. Don't add new instantiation paths.
- **Threading, not asyncio** for business logic; FastAPI sync endpoints run in threadpool, MJPEG uses async generators. Do not use `BaseHTTPMiddleware` (breaks MJPEG).
- **Config loading**: env-over-defaults getters (`runtime/core/settings.py`, `_bool_env` accepts `1/true/yes/on`; client mirror in `main/settings.py` frozen dataclass). YAML via `yaml.safe_load` (business) or `get_yaml()`/file-relative loaders (smartcar SDK). No schema validation — errors are `KeyError`/`FileNotFoundError` with Chinese diagnostics.
- **Units**: raw `/v1/execute` arm args are **meters**; business `ArmClient` uses **mm**. Mixing them is the classic footgun.
- **Chassis moves**: mecanum odometry theta drifts 0.8–3.2 rad/run — use relative `move_for([dx,0,0])` only; never absolute world targets via `move_to_position`.
- **Arm velocity servo**: y has a magnetic safety gate; **x has no soft limit — caller must zero `x_vel` on target loss**. Prefer velocity mode over position-loop servo.
- **Feeds**: `stop_*_feed(force=False)` is a deliberate NO-OP; only `MyCar.close()` passes `force=True`.
- **Logging**: custom `logger` from `smartcar/whalesbot/tools/log_wrap.py` in SDK; stdlib logging elsewhere.
- **WS subscriptions** open a dedicated second connection per subscriber (websocket-client is single-recv).

## Important Files

| File | Role |
|---|---|
| `runtime/server.py` | uvicorn entry → `runtime.api.app:app` |
| `runtime/api/app.py` | FastAPI factory + singleton wiring + startup hooks |
| `runtime/core/settings.py` / `actions.py` | Env config getters; action registries |
| `runtime/services/car_runtime_service.py` | Aggregate service: locks, queues, realtime gate |
| `runtime/services/my_car/` | The car facade (mixins) + `feeds.py` daemon caches |
| `runtime/hardware/controller_session.py` | MC602 USB session state machine (`PROGRAM_READY`) |
| `main/api_client.py` / `ws_client.py` | The ONLY allowed runtime channels |
| `main/settings.py` | Client settings from `RAK_CAR_*` env vars |
| `main/start/orchestrator.py` | Mission loop: lane follow + waypoints + task dispatch |
| `smartcar/whalesbot/vehicle/base/serial_wrap.py` | SerialEngine + MC602 framing (baud 1,000,000) |
| `smartcar/whalesbot/vehicle/arm/arm_base.py` | ArmController (PID, wall/magnetic resets, composite moves) |
| `smartcar/paddlebaidu/infer_cs/base/infer_front.py` | `ClintInterface` ZMQ client (auto-spawns backend) |
| `run.py`, `collect_data.py` | Mission entry; teleop data collector |
| `config_car.yml` | Cameras, PID, inference ports — chassis/inference calibration |
| `task_config.yml` | **Venue calibration surface**: per-task arm poses + `waypoints:` list. Port venues by editing this file, never task code |
| `ecosystem.config.js` | PM2 app definition + all production `RAK_CAR_*` env vars |
| `CLAUDE.md`, `main/API_INDEX.md` | Authoritative conventions; full HTTP/WS/action API reference |
| `runtime/VISION_API.md`, `STREAM_API.md` | Vision + MJPEG endpoint contracts |

### Key env vars

Server: `RAK_CAR_BIND_HOST/PORT`, `RAK_CAR_PUBLIC_HOST`, `RAK_CAR_AUTO_INIT`, `RAK_CAR_RESET_ARM`, `RAK_CAR_INFER_AUTO_START`, `RAK_INFER_EAGER_MODELS` (must keep `lane`), `RAK_INFER_IDLE_UNLOAD_SECONDS`, `RAK_CAR_SERIAL_ENGINE=0` (legacy lock fallback), `RAK_CAR_SERIAL_AUTO_CONNECT=0` (no hardware on import).
Client: `RAK_CAR_SERVER_ORIGIN` (default `http://192.168.5.230` — production is `192.168.6.231`; always set explicitly), `RAK_CAR_API_PORT` (5050).
LLM: `ERNIE_ACCESS_TOKEN` overrides `llm_config.yml`/`config_car.yml` tokens.

## Runtime/Tooling Preferences

- **Runtime**: system `/usr/bin/python3` on JetPack/L4T (Jetson); no version pin anywhere; `from __future__ import annotations` ubiquitous. Node is dev-machine-only (Vite build of `web/`); on the car the only JS is `ecosystem.config.js` (PM2 config).
- **Process manager**: PM2, single app `rak-car-api` (fork mode, autorestart, `max_memory_restart 1200M` — parent RSS only, inference children have their own `RAK_INFER_RSS_LIMIT_MB`).
- **Package manager**: plain pip with range pins (`fastapi>=0.110,<1.0`); no lockfiles, no venv convention.
- **Heavy deps** (PaddlePaddle, ZMQ) run only in the inference subprocess; keep the FastAPI process lean.
- Paths: production = `/home/jetson/workspace/rak-car`; this dev checkout may differ — adapt path-sensitive commands (PM2 config, `INFER_BACKEND_SCRIPT`).
- Storage servo angles are pinned to official values (`[-42, 165]`, RIGHT→protocol 255): never change; `set_storage` requires the y<-100 mm safety gate.

## Testing & QA

- **Framework**: stdlib `unittest` only — **no pytest anywhere** (no conftest/pytest.ini). Tests mock the HTTP/WS layer and run offline, no hardware.
- **Automated suites**: `main/arm/tests/` (18 files — ArmClient, servo PID, vision parsers), `main/task/tests/`, `smartcar/test/test_serial_engine.py` (SerialEngine with fakes; needs `RAK_CAR_SERIAL_AUTO_CONNECT=0`). Run via the `unittest discover` commands above.
- **Manual/hardware-in-the-loop** (not framework tests): `smartcar/test/tasks|functions/*` (real arm), `main/test/*` (HTTP smoke against a live runtime — gitignored), `test/controller_lab` (MC602 protocol lab; motion/flashing gated by `--dangerous`/`--yes-download`). ⚠️ Many manual scripts use `test_*` names — do NOT add pytest collection; it would drive real hardware.
- **Gitignore note**: `**/test/**` and `docs` are ignored — test trees exist locally but are untracked.
- **Physical verification** happens on-track via `main/arm/examples/*` per TEST_PREFLIGHT.md; new code touching init/lock/recovery must first read `.dbg/<slug>.{env,ndjson}` notes (MC602 download-stuck issue is OPEN — don't refactor the recovery layer until closed).
- **No CI**; verification = unittest suites + `GET /v1/health` triage + on-car smoke runs.

### Gotchas quick list

1. `README.md` and `runtime/README.md` are partially **stale** (deleted monolith, old file layout) — CLAUDE.md supersedes them; trust code for arm reset semantics (`runtime/core/actions.py`).
2. `mc602_cfg.yaml` is dead config (no consumer). Vision model `front` (port 5003) is configured but disabled — don't build against it.
3. `main/misc/llm_config.yml` has a byte-identical copy at `main/task/task3/detect/llm_config.yml` — edit both; the real ERNIE token is committed in plaintext there.
4. Camera numbering disagrees between `config_car.yml` (front=2/side=1, OpenCV indices) and stream aliases (cam2=side).
5. `arm_origin.yaml` is generated at runtime (`RAK_CAR_RESET_ARM=1`) — never hand-commit.
6. Auto-init (`RAK_CAR_AUTO_INIT=1`) performs wall-bump arm homing (15–25 s); smoke tests that reset x deliberately slam the axis — ensure physical clearance.
