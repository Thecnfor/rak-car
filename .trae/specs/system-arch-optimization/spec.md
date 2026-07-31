# 系统架构优化 Spec（OOM 韧性 + Runtime 健壮性）

## Why

比赛前 Jetson Nano 长期跑 `rak-car-api` 后会被老师 OOM kill：4 个 Paddle 推理模型 (YoloeInfer lane/task/ocr + ERNIE) 在 `infer_back_end.py` 启动时一次性全部常驻加载，加上 5 个守护线程（lane/arm/task/ir/odom）+ MJPEG 编码器，2GB 内存吃到顶。同时 runtime 的 `_safe_close_locked`、`_auto_init_loop` 长期运行后会出现 MC602 USB 句柄泄漏 / ZMQ socket EFSM。本次只解决已知的 OOM 与稳定性痛点，不动协议、不动端点契约。

## What Changes

- **推理后端按需加载 + 闲置回收**：保留 `RAK_INFER_EAGER_MODELS` 入口，但默认改为只预热 `lane`（巡线必须常驻），其余 `task` / `ocr` 走懒加载；新增空闲 LRU 卸载（默认 5 分钟无请求自动 `del infer_dict[name]`，gc 后内存归还）；新增 `/v1/infer/state` 的 `mem_estimate_mb` 与每模型 `loaded` 字段。
- **推理进程内存画像**：启动时读 `/proc/self/status` 与 `tracemalloc` 抓基线，每次 lazy load 后 diff，对外暴露；OOM 发生时按 `RAK_INFER_OOM_POLICY`（默认 `drop_oldest`）卸载最久未用模型。
- **守护线程内存压力降频**：当 runtime 检测到系统 RSS > `RAK_CAR_MEMORY_PRESSURE_MB`（默认 1500MB）时，按"ir → odom → arm → task → lane"顺序自动把守护线程 hz 降档（不可低于下限），并通过 `/v1/health` 的 `feeds.degraded` 字段上报。`/v1/control/reset-stop` + lane_feed 重启可恢复。
- **infer_back_end.py REP 线程加固**：已有 2026-07-31 修复（异常不再杀线程）。本次补：单帧推理加超时（`RAK_INFER_FRAME_TIMEOUT_S`，默认 5s），超时返回 `[]` 而非阻塞后续；EFSM 时强制重连 socket。
- **runtime 进程内存上限**：新增 `RAK_CAR_RSS_LIMIT_MB`（默认 1800MB）软限。runtime 启动后 60s 起一个 `ResourceProbeThread` 每 30s 读 RSS；超过软限只 warn + 上报到 `/v1/health`，硬限 95% → 主动走 `gc.collect()` + `infer_service.drop_oldest()`（不动业务）。
- **MC602 句柄清理**：`infer_back_end.py` 增加 `atexit` 钩子 + `signal.SIGTERM` handler，主动 `socket.close(linger=0)` + `context.term()`，避免 pm2 重启时 `Address already in use`。runtime 端 `shutdown()` 路径走同样清理。
- **MJPEG 编码器降级**：`_encoder_loop` 在 RSS 高水位（>85% 软限）时自动把 `quality` 从 80 降到 60，把帧从原始分辨率降采样到 320×240，保持 20fps。
- **MyCar 启动瘦身**：`paddle_infer_init()` 默认只 `ClintInterface("lane")`，其它模型按需 `ClintInterface(name)` 首次调用时连接；ERNIE 两个 wrapper 改为 lazy 属性。

约束：
- 不改 MCU 协议
- 不改 HTTP/WS 端点路径与返回字段
- MyCar 公开方法签名不变
- `/v1/infer/state` 在 `last_error` / `models[*].error` 上做**字段新增**（`loaded`, `mem_estimate_mb`, `last_used_at`, `lazy_load_count`），向后兼容

## Impact

- **Affected specs**：runtime 进程模型、infer_back_end 加载策略、feed 守护线程调度、HTTP `/v1/health` & `/v1/infer/state` 返回字段（向后兼容新增）。
- **Affected code**：
  - `smartcar/paddlebaidu/infer_cs/base/infer_back_end.py`（核心：eager→lazy、LRU、per-frame 超时、socket cleanup）
  - `runtime/services/inference_service.py`（get_state 暴露新字段；新增 `drop_oldest()`、`record_use()`）
  - `runtime/core/settings.py`（新增 4 个 env vars）
  - `runtime/services/runtime_service.py`（`ResourceProbeThread`、`feeds.degraded` 计算）
  - `runtime/services/my_car.py`（`paddle_infer_init` 拆分 lazy；ERNIE 改为 lazy）
  - `runtime/services/camera_stream_service.py`（`_encoder_loop` 高水位降级）

## ADDED Requirements

### Requirement: 推理模型按需加载
系统 SHALL 在 `infer_back_end.py` 启动时只常驻加载 `lane`（或 `RAK_INFER_EAGER_MODELS` 显式列出的子集），其余模型 `YoloeInfer(task)` / `OCRReco` / ERNIE 在首次收到请求时延迟加载。

#### Scenario: 启动时只 lane 常驻
- **WHEN** `RAK_INFER_EAGER_MODELS` 未设置且 `infer_back_end.py` 启动
- **THEN** `infer_dict` 仅含 `lane`；`task` / `ocr` 在 `get_infer_func` 首次调用时触发 `_load_model`；启动 RSS 较旧版下降 ≥30%（从 base 测量）

#### Scenario: 闲置 5 分钟自动卸载
- **WHEN** 模型 300 秒内无 `get_infer_func(name)` 调用
- **THEN** 后台 tick（每 60s 扫一次）将 `infer_dict[name]` 引用清掉，触发 `gc.collect()`；下一次调用走 lazy load 路径
- **AND** `/v1/infer/state` 该模型 `loaded=false`，`last_used_at` 保留

### Requirement: 推理进程 OOM 软卸载
系统 SHALL 在检测到 `infer_back_end.py` RSS > `RAK_INFER_RSS_LIMIT_MB`（默认 1200MB）时，按 `RAK_INFER_OOM_POLICY`（`drop_oldest`/`drop_ocr`/`none`）卸载模型直到 RSS 回到 80% 软限。

#### Scenario: RSS 超限 drop_oldest
- **WHEN** RSS 连续 2 个 probe 周期 > 软限
- **THEN** 按 `last_used_at` 升序卸载非 lane 模型；lane 永不卸载（巡线刚需）；卸载完成后 `_debug_emit` 上报 OOM session

### Requirement: 守护线程内存压力降档
runtime SHALL 在系统级 RSS > `RAK_CAR_MEMORY_PRESSURE_MB` 时按 `ir → odom → arm → task → lane` 顺序对守护线程 hz 减半（每轮，最低不可低于下限）。

#### Scenario: 高水位触发降档
- **WHEN** RSS 1500MB < x ≤ 1700MB
- **THEN** ir/odom 由 50→25Hz；arm 20→10Hz；task 30→15Hz；lane 保持 50Hz（巡线刚需）
- **AND** `feeds.degraded=[<feed_name>, ...]` 在 `/v1/health` 中体现

#### Scenario: RSS 回落自动恢复
- **WHEN** RSS ≤ 1300MB 持续 60s
- **THEN** 按相反顺序把 hz 恢复原值；`feeds.degraded=[]`

### Requirement: 单帧推理超时
infer_back_end 的 REP 循环 SHALL 在 `infer_tmp(img)` 上加超时（默认 5s，可由 `RAK_INFER_FRAME_TIMEOUT_S` 覆盖）。

#### Scenario: 推理超时
- **WHEN** 单帧推理 > `RAK_INFER_FRAME_TIMEOUT_S`
- **THEN** 返回 `[]` 并 log warn；REP 线程继续循环，下一帧走正常路径

### Requirement: ZMQ socket 清理
infer_back_end SHALL 在 `atexit` / `SIGTERM` / `SIGINT` 时显式 `socket.close(linger=0)` + `context.term()`。

#### Scenario: pm2 重启
- **WHEN** pm2 发 SIGTERM
- **THEN** 1s 内所有 socket 与 context 释放；下次 pm2 start 不报 `Address already in use`

### Requirement: MJPEG 编码高水位降级
`_encoder_loop` SHALL 在 RSS > 85% 软限时自动降 quality 与分辨率。

#### Scenario: RSS > 85% 软限
- **WHEN** runtime 检测到资源紧张
- **THEN** `_encoder_loop` 把 `quality` 80→60 + 分辨率降到 320×240；恢复条件同上

### Requirement: HTTP/WS 字段向后兼容
新增字段 SHALL 不破坏既有字段；既有 main/ 客户端读取 `last_error`、`models[*].error` 仍按原 schema 工作。

#### Scenario: 旧客户端拉 /v1/infer/state
- **WHEN** 客户端 JSON 解析
- **THEN** 既有字段全保留；新增字段 `loaded`、`mem_estimate_mb`、`last_used_at` 默认 null/0，旧客户端忽略

## REMOVED Requirements

无。HTTP/WS 端点、MyCar 公开方法、MCU 协议均保留。