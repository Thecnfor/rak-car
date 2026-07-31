# Tasks

按依赖顺序排列；每个 task 完成后立刻勾选。任务粒度刻意拆小，方便随时停下来。

## Task 1: env var 接入 + defaults
**Owner**: runtime 改动
**Files**: `runtime/core/settings.py`
**Steps**:
- 新增 4 个常量 + getter:
  - `INFER_EAGER_MODELS` (default `"lane"`)
  - `INFER_IDLE_UNLOAD_SECONDS` (default `300`)
  - `INFER_FRAME_TIMEOUT_S` (default `5.0`)
  - `INFER_RSS_LIMIT_MB` (default `1200`)
  - `INFER_OOM_POLICY` (default `"drop_oldest"`)
  - `CAR_MEMORY_PRESSURE_MB` (default `1500`)
  - `CAR_RSS_LIMIT_MB` (default `1800`)
- 加入 `get_runtime_settings()` 暴露给 `/v1/config`。
**Verify**: `python3 -c "from runtime.core import settings; print(settings.get_runtime_settings())"` 打印新字段。

## Task 2: infer_back_end.py — 默认懒加载 + LRU 卸载 + 单帧超时 + socket cleanup
**Owner**: infer 后端
**Files**: `smartcar/paddlebaidu/infer_cs/base/infer_back_end.py`
**Dependencies**: Task 1

**Steps**:
1. 把 `__init__` 的"全部 eager load + warmup 3 轮"改成"只加载 `_eager_models`（默认 `lane`）"，warmup 也只跑常驻子集。
2. 引入 `ModelRegistry`：保存每个 `name` 的 `loaded: bool`、`last_used_at: float`、`load_lock: threading.Lock`。`lazy_infer` 第一次调用走 `_load_model` 加锁；后续走 fast path。
3. 新增后台 `_idle_unload_loop`：每 60s 扫一次，`last_used_at` 超过 `INFER_IDLE_UNLOAD_SECONDS` 且非 `_eager_models` 的，从 `infer_dict` pop 后 `gc.collect()`。
4. `process_demo` 的 `func(img)` 包 `signal.alarm(INFER_FRAME_TIMEOUT_S)` 风格超时——Python 无 threading.Timer 中断推理的可移植方案，改用 `concurrent.futures.ThreadPoolExecutor(max_workers=1)` 提交 + `future.result(timeout=INFER_FRAME_TIMEOUT_S)`；超时返回 `[]` 并 log warn；线程复用避免开销。
5. 注册 `atexit` + `signal.SIGTERM`/`SIGINT` handler：`for s in self.server_dict.values(): s.close(linger=0)` + `context.term()`。
6. `_load_model` 内部加 `tracemalloc` snapshot，diff 给 `mem_estimate_mb` 字段；`__init__` 启动后留一个 baseline。

**Verify**:
- `python3 smartcar/paddlebaidu/infer_cs/base/infer_back_end.py`（前台跑）日志显示 `lane loaded`，task/ocr 不在列表。
- 在另一个 shell 跑 `ps aux | grep infer_back_end | awk '{print $6}'`，启动后 RSS 应 < 700MB（vs 旧版 ~1.1GB）。
- `echo "image$(base64 ...)" | nc 127.0.0.1 5002` 触发 task 加载，第二次再发即可命中；5 分钟不发再观察 RSS 下降。

## Task 3: inference_service.py — 暴露 `last_used_at` / `mem_estimate_mb` / `drop_oldest`
**Owner**: runtime
**Files**: `runtime/services/inference_service.py`
**Dependencies**: Task 2

**Steps**:
1. 新增 `record_use(name)` 接口：被前端调用后通知后端更新 `last_used_at`（通过现有 ZMQ REQ 发 `b"ATATA"` → 后端解析分支记录）。最简实现：让现有 `probe` 顺带捎带模型 last_used（懒：直接读后端 `/proc/<pid>/status` 估算）。
2. `probe()` 返回字段每个 model 增加 `loaded` / `mem_estimate_mb` / `last_used_at`；`get_state()` 透传。
3. 新增 `drop_oldest()`：给后端发新指令 `b"DROP_OLDEST"`，后端按 `last_used_at` 升序卸载非 eager 模型（前端调用）。

**Verify**: `curl http://127.0.0.1:5050/v1/infer/state` 返回字段齐；`curl -X POST http://127.0.0.1:5050/v1/infer/drop-oldest` 触发卸载。

## Task 4: MyCar 启动瘦身 — paddle_infer_init lazy + ERNIE lazy
**Owner**: SDK
**Files**: `runtime/services/my_car.py`
**Dependencies**: Task 1

**Steps**:
1. `paddle_infer_init()` 只 `self.crusie = ClintInterface("lane")`；`task_det` / `ocr_rec` 改为 `@property` 懒连接（首次访问时建 ClintInterface 并缓存）。
2. `ernie_bot_init()` 改为只存 `self._ernie_image = None` / `self._ernie_order = None`；原 `image_analysis` / `order_analysis` 改为 `@property`，首次访问时 `ErnieBotWrap()` 实例化。
3. `aniyan_get_humattr` / `yiyan_get_actions` 旧路径保留（兼容），但底层同样走 lazy 属性。

**Verify**: 冷启动 MyCar 后 `cap_side` 仍在；`task_det` 第一次调用才发起 ZMQ 连接；ERNIE wrapper 第一次访问才构造。

## Task 5: runtime_service — ResourceProbeThread + feeds.degraded 降档
**Owner**: runtime
**Files**: `runtime/services/runtime_service.py`
**Dependencies**: Task 1

**Steps**:
1. 新增 `_ResourceProbeThread`（daemon）：启动后 60s 起，每 30s 读 `resource.getrusage(resource.RUSAGE_SELF).ru_maxrss`（KB，Linux）→ MB。
2. RSS > `CAR_MEMORY_PRESSURE_MB` 持续 1 个 tick 触发降档：调用各 feed 的 `degrade()`（新增方法）；降档顺序 ir→odom→arm→task；lane 不降档。
3. 每个 feed 类（lane/arm/task/ir/odom 在 `my_car.py`）新增 `degrade(ratio=0.5)` / `restore()`，调整 `period` 但不重启线程（直接改 `self._<name>_feed_hz` + 重置 `stop_event.wait` 的 period）。
4. RSS < `CAR_MEMORY_PRESSURE_MB - 200` 持续 60s 触发恢复（按反向）。
5. `feeds.degraded: list[str]` 字段加入 `get_state()`。
6. 注意：不能用 `resource.getrusage` 在多线程下语义不同；改用 `psutil.Process(os.getpid()).memory_info().rss`（psutil 已 import）。

**Verify**: 启动 runtime，触发一次手动 `set_memory_pressure_for_test()`（debug 入口）；观察 `curl /v1/health | jq .state.feeds.degraded` 返回非空；mock 释放内存后回空。

## Task 6: camera_stream_service — 编码器高水位降级
**Owner**: runtime
**Files**: `runtime/services/camera_stream_service.py`
**Dependencies**: Task 5

**Steps**:
1. `_encoder_loop` 改成可调 quality / 分辨率；新增 `_high_water: bool` 状态。
2. runtime 在降档回调里调 `camera_stream_service.set_encode_quality(60, scale=0.5)`；恢复调 `set_encode_quality(80, scale=1.0)`。
3. `set_encode_quality` 线程安全：写 `self._quality` / `self._scale` 由 `self._encoder_lock` 保护。

**Verify**: 手动调接口一次，前端 stream 抓一帧 JPEG 查看分辨率变化。

## Task 7: HTTP `/v1/health` / `/v1/infer/state` / `/v1/config` 字段补齐
**Owner**: API
**Files**: `runtime/api/routes.py`（或 `runtime/services/runtime_service.py` 的 `get_state`）
**Dependencies**: Task 3, 5

**Steps**:
1. `/v1/infer/state` 已由 Task 3 透传。
2. `/v1/config` 已由 Task 1 暴露新 env vars。
3. `/v1/health` 在 `state.feeds.degraded` 上报。

**Verify**: `curl /v1/health | jq '.state.feeds, .state.components.infer.models[0].loaded'` 全部为真。

## Task 8: 验证 — 7×24 小时 soak test（脱机）+ 比赛场景回归
**Owner**: 集成
**Dependencies**: Task 1-7

**Steps**:
1. 写一个 `test/oom_soak.py` 脚本（gitignored，本地用）：跑 100 次 `/_init` → `lane_time(0.3, 30)` → `close`，中间穿插 `/_init_lane` → 30s 等；同时用 psutil 采样 RSS，每 10s 打印一次；要求 100 次后 RSS < 1000MB。
2. 跑 `main/test/verify_concurrent.py`（双线程探针）确保并发模型仍正常。
3. `runtime/VISION_API.md` 增补一行说明 `infer.state` 新字段。

**Verify**: 脚本输出 + verify_concurrent 全部 pass。

## Task 9: 文档与回滚
**Owner**: docs
**Files**: `runtime/README.md`、`runtime/VISION_API.md`、`CLAUDE.md`
**Dependencies**: Task 8

**Steps**:
1. `runtime/README.md` 加"内存管理"小节：env vars 表 + 降档顺序。
2. `runtime/VISION_API.md` 增补 `/v1/infer/state` 新字段。
3. `CLAUDE.md` 在"Runtime env vars"表追加 6 个新 env var。
4. 不写 README 增量（CLAUDE.md 已涵盖）。

**Verify**: 三处文档都搜得到新 env var。

# Task Dependencies

- Task 2 依赖 Task 1
- Task 3 依赖 Task 2
- Task 4 依赖 Task 1
- Task 5 依赖 Task 1
- Task 6 依赖 Task 5
- Task 7 依赖 Task 3 + 5
- Task 8 依赖 Task 1-7
- Task 9 依赖 Task 8