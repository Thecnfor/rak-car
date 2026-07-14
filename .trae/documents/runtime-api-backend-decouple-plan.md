# runtime 独立 API 后端改造计划（解耦版）

## 1. Summary

本计划目标是把当前 `rak-car` 的运行链路从“单体脚本式自管理推理进程”切换为“runtime 统一托管的独立 API 后端”，并满足以下硬约束：

* 直接切换：不再依赖 legacy `ClintInterface` 的自拉起/自杀后端逻辑。

* 后端+文档同步落地：代码改造与 `runtime` 对外文档一起更新。

* 强解耦：推理、摄像头流、下位机控制三者隔离，任何一方异常不应把另外两方拖死。

## 2. Current State Analysis

### 2.1 runtime 已是独立 API 框架，但推理生命周期仍被 legacy 影响

* `runtime` 已通过 FastAPI 常驻化，入口与组织清晰：`runtime/api/app.py`、`runtime/services/runtime_service.py`、`runtime/core/settings.py`。

* PM2 已按独立 API 服务配置：`ecosystem.config.js` 中 `rak-car-api -> runtime/server.py`。

* 但 `MyCar` 初始化仍会走 `car_wrap_2026.py -> paddle_infer_init() -> infer_front.ClintInterface(...)`。

* `ClintInterface` 里仍有“健康探测失败后 stop+restart infer\_back\_end.py”的本地单体逻辑，位于 `smartcar/paddlebaidu/infer_cs/base/infer_front.py`。

### 2.2 已观测风险（会破坏 API 后端稳定性）

* 推理后端冷启动期间，legacy 健康探测会误判超时并反复重启，导致 runtime 一直 `initializing=true`。

* runtime 初始化未完成时，摄像头流只剩占位帧，业务层感知为“Web 预览坏掉”。

* 下位机状态抖动与推理初始化相互影响，出现“program 在线但服务不可用”的混合故障体验。

### 2.3 文档与实现存在偏差

* `runtime/README.md` 强调独立 API，但没有明确“推理进程托管边界”和“组件隔离策略”。

* `runtime/VISION_API.md` 只定义调用面，缺少“推理服务冷启动/不可用时的状态语义与降级行为”。

## 3. Proposed Changes

### 3.1 推理进程托管切换到 runtime（核心）

**文件：**

* `runtime/services/runtime_service.py`

* `runtime/core/settings.py`

* `runtime/api/routes.py`

**改造内容：**

* 在 `CarRuntimeService` 内新增“推理服务会话管理”能力（仅负责进程生命周期与健康状态，不与摄像头/控制器锁绑定）。

* 增加推理健康状态缓存（如 `infer_state`、最近失败原因、最近成功探测时间）。

* `ensure_initialized()` 里移除对 legacy 自拉起机制的隐式依赖，改为：

  * 先确保控制器状态满足最低要求；

  * 再以“非阻塞+可超时”的方式确认推理服务可用；

  * 推理未就绪时返回结构化错误，不无限阻塞初始化主流程。

* 新增只读运维接口（建议挂 `v1`）暴露推理状态，便于 API 调用方区分：

  * 控制器异常

  * 推理未就绪

  * 摄像头无新帧

**为什么：**

* 统一由 runtime 托管，消除多处自拉起和相互误杀。

* 保证 API 后端是“状态可观测、失败可分层”的服务系统，而不是单体脚本行为。

### 3.2 禁用 legacy 自启/自杀链路（直接切换）

**文件：**

* `smartcar/paddlebaidu/infer_cs/base/infer_front.py`

**改造内容：**

* 去掉（或默认关闭）`check_back_python()` 自动启动逻辑。

* 去掉（或默认关闭）`unhealthy_count >= 3` 后 `stop_process(infer_back_end.py)` 的强制重启逻辑。

* 保留最小客户端职责：连接、请求、超时反馈；不负责进程管理。

* 增加清晰错误信息，提示“推理服务应由 runtime 托管”。

**为什么：**

* 满足“直接切换”要求，避免继续由旧逻辑破坏 runtime 托管边界。

### 3.3 推理 / 摄像头 / 下位机三向解耦（稳定性）

**文件：**

* `runtime/services/runtime_service.py`

* `runtime/services/camera_stream_service.py`

**改造内容：**

* 推理失败不关闭共享摄像头对象，不触发摄像头线程停摆。

* 下位机探测失败不直接带崩流服务线程；流服务继续提供状态化占位输出和最近错误元信息。

* 初始化锁与摄像头锁进一步缩小临界区，避免“初始化卡住导致流与控制全阻塞”。

* 健康接口增加分层状态字段（controller/camera/infer 分开），避免单一 `initialized` 掩盖真实瓶颈。

**为什么：**

* 满足“摄像头服务和下位机控制不能连带都死”的硬约束。

### 3.4 文档同步为“独立 API 后端规范”

**文件：**

* `runtime/README.md`

* `runtime/VISION_API.md`

**改造内容：**

* 明确写入：runtime 是唯一推理托管方；legacy 客户端不得管理后端进程。

* 增加“组件隔离与失败语义”章节：控制器、推理、摄像头异常的区别、可见症状、排查入口。

* 增加运维建议：PM2 管理、健康检查顺序、冷启动等待窗口、常见误判处理。

**为什么：**

* 让团队按 API 后端模式协作，避免回退到单体服务心智。

## 4. Assumptions & Decisions

### 4.1 已锁定决策

* 范围：后端 + 文档（不在本次计划内做 `main/` 客户端大改）。

* 兼容策略：直接切换，不保留 legacy 自拉起/自杀为默认路径。

* 架构原则：runtime 统一托管推理进程，业务与客户端仅通过 API 消费状态。

### 4.2 执行期假设

* 允许在切换期间短时重启 `rak-car-api` 进行受控验证。

* 现有 `config_car.yml` 的模型路径与端口配置保持不变（只改生命周期管理，不改模型定义本身）。

## 5. Verification Steps

### 5.1 功能验证

* `pm2 start/restart rak-car-api` 后，`/v1/health` 不再长期卡在 `initializing=true`。

* 新增推理状态接口可清晰看到推理冷启动、就绪、异常原因。

* `VISION_API` 相关端点在推理就绪后可返回结果；推理未就绪时返回结构化错误而非整体卡死。

### 5.2 解耦验证

* 人工制造推理不可用：摄像头流仍可访问（至少占位+状态可读），下位机状态探测不被拖死。

* 人工制造下位机不可用：推理状态可独立观测，摄像头流服务线程不退出。

* 人工制造摄像头不可用：控制与推理状态仍可独立反馈，不触发后端自杀式重启。

### 5.3 文档一致性验证

* `runtime/README.md` 与 `runtime/VISION_API.md` 的“托管边界、失败语义、运维流程”与实现一致。

* 团队按文档执行时，无需再依赖 `infer_front.py` 的隐式进程控制。

