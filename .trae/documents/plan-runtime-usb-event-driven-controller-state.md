# runtime USB 事件驱动控制器优化计划

## Summary

目标是把当前 `runtime` 的控制器状态机，从“固定周期轮询 + 恢复链路里顺便探测”的模式，升级为**USB 事件驱动 + 分层探测 + 仅在必要时握手/拉起**的智能机制。

本次优化聚焦用户指出的真实现场特性：

* 换电池/断电时，USB 串口会直接消失

* USB 一旦重新出现，通常先是 bootloader

* 进入 program 后，bootloader 不再回显

* 因此最合理的策略不是持续无脑轮询，而是：

  * **无 USB：直接判定无电，等待 USB 重新出现**

  * **有 USB：才做 bootloader/program 探测**

  * **只要有 USB，就必须通过 probe/握手确认它到底是不是 program**

成功标准：

* 无 USB 时 runtime 不再反复做串口握手/恢复

* USB 刚出现时，runtime 能优先进入 bootloader/program 探测

* bootloader 在线时只尝试拉起 program，不自动 download

* program 在线时，只做轻量心跳确认，不再误用 bootloader 探测

* `/v1/health` 能清晰体现“无 USB、USB 已出现、bootloader、program、过渡中”这些状态

## Current State Analysis

### 1. 当前 `probe_controller()` 已经拆成纯探测，但仍是主动轮询驱动

当前文件：

* `runtime/hardware/controller_probe.py`

当前已具备：

* `list_candidate_ports()`：列出 USB/CH340 串口

* `probe_port_mode(port_name)`：区分 `program` / `bootloader` / `unknown`

* `probe_controller()`：返回：

  * `mode="no_port"`

  * `mode="program"`

  * `mode="bootloader"`

  * `mode="unknown"`

这已经比旧版更干净，但仍然存在一个问题：

* **USB 是否存在** 与 **串口握手** 仍然被放在同一个“定时探测”节奏里

也就是说，当前虽然语义更好了，但在“断电导致 USB 消失”的场景下，runtime 还是只能靠周期性触发探测才知道 USB 回来了。

### 2. 当前 `controller_session` 还没有“USB 存在性事件”这一层

当前文件：

* `runtime/hardware/controller_session.py`

当前状态机已经有：

* `NO_PORT`

* `UNKNOWN`

* `BOOTLOADER_READY`

* `PROGRAM_TRANSITION`

* `PROGRAM_READY`

* `RUNTIME_LOST`

* `RECOVERING`

但当前驱动状态变化的主要机制仍是：

* `ensure_ready()` 主动触发 `_recover_once()`

* `_heartbeat_loop()` 定时检查 `ping_current()`

当前缺失的是：

* **USB 热插拔事件源**

* **“USB 不存在”和“USB 存在但未握手”之间的分层驱动**

### 3. 当前 `_auto_init_loop()` 仍然是固定周期推进

当前文件：

* `runtime/services/runtime_service.py`

当前逻辑：

* `_auto_init_loop()` 每隔 `auto_init_retry_interval` 做一次推进

* 若 `PROGRAM_READY` 则初始化 car

* 否则调用 `controller_session.ensure_ready()`

问题在于：

* 即便下位机已经断电、USB 已经消失，仍然靠固定周期去推进恢复

* 这在“换电池时 USB 整体消失”的现场下是不经济的，也不够智能

### 4. 当前依赖里没有 `pyudev`

当前文件：

* `runtime/requirements.txt`

当前依赖只有：

* `fastapi`

* `uvicorn`

因此如果要做真正的 USB 热插拔事件监听，需要新增依赖，例如：

* `pyudev`

### 5. 当前 runtime 启动结构适合挂 USB 监听线程

当前文件：

* `runtime/api/app.py`

当前结构：

* `service = CarRuntimeService()`

* FastAPI `startup` 时启动：

  * `service.start_background_services()`

  * `camera_stream_service.start()`

  * `service.start_auto_init()`

因此很适合把：

* USB 事件监听线程

* 事件驱动的控制器状态推进

挂在 `CarRuntimeService` / `ControllerSessionManager` 的后台线程中，而不是单独搞新服务。

## Proposed Changes

### 1. 新增 USB 事件监听层

涉及文件：

* `runtime/hardware/controller_session.py`

* `runtime/requirements.txt`

新增依赖：

* 在 `runtime/requirements.txt` 中加入 `pyudev`

实现思路：

* 在 `ControllerSessionManager` 中新增 USB 监听线程

* 使用 `pyudev.Monitor` 监听 `/dev/ttyUSB*` 或 CH340/USB 串口设备的新增/移除事件

* 将“USB 是否存在”从主动轮询中分离出来，作为状态机的一级驱动信号

建议新增内部职责：

* `_usb_monitor_thread`

* `_usb_present`

* `_last_usb_seen_at`

* `_last_usb_event_at`

* `_last_usb_event`

建议新增内部方法：

* `_start_usb_monitor()`

* `_usb_monitor_loop()`

* `_handle_usb_event(action, device_info)`

### 2. 把“USB 存在性”变成状态机第一层判断

涉及文件：

* `runtime/hardware/controller_session.py`

核心策略：

* **USB 不存在**

  * 直接进入 `NO_PORT`

  * 不做任何串口握手

  * 不做任何 bootloader/program 探测

  * 只等 USB 事件

* **USB 一出现**

  * 立即触发一次纯探测

  * 若是 bootloader，进入 `BOOTLOADER_READY`

  * 若是 program，进入 `PROGRAM_READY`

  * 若未知，进入 `UNKNOWN`

这样状态机的推进顺序会变成：

1. 先看 USB 是否存在
2. 只有存在 USB 才做 probe
3. 只有 probe 到 bootloader 才尝试 `RUNCODE`
4. program 在线后只做轻量 program 心跳

### 3. 新增“USB 驱动”的状态推进，而不是统一定时推进

涉及文件：

* `runtime/hardware/controller_session.py`

建议新增状态推进规则：

#### 3.1 无 USB

状态：

* `NO_PORT`

行为：

* 不进入 `_recover_once()`

* 不调 `probe_controller()`

* 不做 `ping_mc602()` / `boot_ping()`

* 等待 `pyudev` 事件

这是本次“不要无脑轮询”的核心优化。

#### 3.2 USB 刚出现

状态迁移：

* `NO_PORT -> UNKNOWN/BOOTLOADER_READY/PROGRAM_READY`

行为：

* 收到 USB add 事件后立即调用一次 `probe_controller()`

* 如果 `program`：

  * 直接切 `PROGRAM_READY`

* 如果 `bootloader`：

  * 切 `BOOTLOADER_READY`

  * 进入一次受控拉起

* 如果 `unknown`：

  * 切 `UNKNOWN`

  * 做有限次短窗口复探测

#### 3.3 USB 已存在且为 bootloader

行为：

* 只尝试 `RUNCODE`

* 不自动下载

* 进入 `PROGRAM_TRANSITION`

* 在 transition 窗口内不重复拉起

* 只做 program 探测确认是否进入 program

#### 3.4 USB 已存在且为 program

行为：

* 进入 `PROGRAM_READY`

* 心跳阶段只做 `ping_current()` / program 握手确认

* 不再做 bootloader 探测

### 4. 给 `ControllerSessionManager` 增加“USB 事件优先，轮询兜底”的执行模型

涉及文件：

* `runtime/hardware/controller_session.py`

虽然用户选的是事件监听，但实现上仍应保留轻量兜底：

* 主驱动：`pyudev` 事件

* 兜底：

  * 若监听线程异常退出

  * 或偶发漏事件

  * 保留低频 USB presence 校验

但这个兜底不应再等价于“持续握手轮询”。

建议拆成两类后台循环：

* **USB presence loop**

  * 低频，仅检查“当前是否存在 USB”

  * 只有监听异常或状态不可信时才启用

* **program heartbeat loop**

  * 仅在 `PROGRAM_READY` 时启用

  * 使用 `serial_wrap.ping_current()` 轻量判断是否仍在 program

这样真正昂贵的握手只发生在：

* USB 出现后

* program 掉线后

* bootloader 过渡窗口内

### 5. 明确“每次 USB 存在都必须验证是不是 program”

涉及文件：

* `runtime/hardware/controller_probe.py`

* `runtime/hardware/controller_session.py`

这是用户明确要求的行为：

* “需要确保每次存在 USB 都是 program 模式”

因此计划上必须固定以下原则：

* **不能只因为 USB 存在，就假设它可控**

* 只要 USB 新出现，就必须经过：

  * `probe_controller()`

  * 区分 `program` / `bootloader` / `unknown`

也就是说：

* USB presence 是前提

* program handshake 才是最终可控判据

### 6. 优化 `_auto_init_loop()`，让它不再承担“发现 USB”的职责

涉及文件：

* `runtime/services/runtime_service.py`

建议改为：

* `_auto_init_loop()` 不再负责“发现控制器是否回来”

* 它只消费 `controller_session` 已经稳定给出的状态

具体行为：

* 若 `controller_session.state == PROGRAM_READY` 且 car 未初始化：

  * 才尝试 `ensure_initialized()`

* 若状态是：

  * `NO_PORT`

  * `BOOTLOADER_READY`

  * `PROGRAM_TRANSITION`

  * `UNKNOWN`

  * `RUNTIME_LOST`

  * 则不直接构造 car

  * 等待 `controller_session` 自己推进到 `PROGRAM_READY`

这样 runtime 初始化线程就不会和控制器状态机抢职责。

### 7. 优化运行中掉电场景

涉及文件：

* `runtime/hardware/controller_session.py`

* `runtime/services/runtime_service.py`

目标行为：

* program 在线时执行动作

* 突然掉电，USB 消失

* 当前动作快失败

* `controller_session` 立刻识别为：

  * `RUNTIME_LOST`

  * 若随即 USB 消失，则转 `NO_PORT`

* 后续等待 USB add 事件

* 一旦 USB 重新出现，再按 bootloader/program 重新分流

这比现在“动作失败后继续靠恢复循环摸索”更贴近真实硬件行为。

### 8. 增强 `/v1/health` 的 USB / program 状态可观测性

涉及文件：

* `runtime/hardware/controller_session.py`

* `runtime/services/runtime_service.py`

当前 `get_state()` 已经会输出 `controller_session` 快照。

建议在 `snapshot()` 中新增这些字段：

* `usb_present`

* `last_usb_seen_at`

* `last_usb_event_at`

* `last_usb_event`

* `last_probe_mode`

* `last_program_ok_at`

* `last_transition_at`

这样健康接口可直接区分：

* 无 USB，正在等设备回来

* USB 已回来，但还在 bootloader

* 刚发过 `RUNCODE`，等待 program 接管

* program 已就绪

### 9. 调试埋点围绕“USB 事件驱动”重做

涉及文件：

* `runtime/hardware/controller_probe.py`

* `runtime/hardware/controller_session.py`

* `runtime/hardware/controller_recover.py`

新增埋点重点：

* USB add/remove 事件

* USB 出现后第一次 probe 的模式

* `BOOTLOADER_READY -> PROGRAM_TRANSITION -> PROGRAM_READY` 的迁移

* `PROGRAM_READY -> RUNTIME_LOST -> NO_PORT` 的迁移

* 说明当前没有做握手的原因：

  * 因为 USB 不存在

  * 因为当前是 transition 抑制窗口

  * 因为已经在 program

这类日志对现场判断会非常关键。

## Assumptions & Decisions

* 决策：采用 `pyudev` 做真正的 USB 热插拔事件监听。

* 决策：USB 不存在时，runtime 不做任何串口握手或恢复，只进入等待状态。

* 决策：USB 一旦出现，必须通过 `probe_controller()` 明确判断当前是 `program`、`bootloader` 还是 `unknown`。

* 决策：bootloader 下只尝试 `RUNCODE` 拉起 program，永不自动 download。

* 决策：program 在线后只做轻量 program 心跳，不再用 bootloader 探测作为在线判据。

* 决策：运行中掉线仍保持“当前请求快失败，后台自愈”的策略。

* 假设：换电池/断电时 USB 设备节点会真实消失，这是当前硬件环境的稳定特征。

* 假设：USB 重新出现后，多数情况下先落在 bootloader，这就是优先做 bootloader/program 分流的依据。

## Verification Steps

### 1. 静态验证

* 检查以下文件无语法/诊断错误：

  * `runtime/hardware/controller_probe.py`

  * `runtime/hardware/controller_session.py`

  * `runtime/hardware/controller_recover.py`

  * `runtime/services/runtime_service.py`

  * `runtime/requirements.txt`

### 2. 无 USB 场景验证

步骤：

* 拔掉控制器 / 断电

期望：

* `controller_session.state == NO_PORT`

* `/v1/health` 能显示 USB 不存在

* runtime 不再持续做握手或恢复

### 3. USB 出现但处于 bootloader

步骤：

* 恢复供电，让 `/dev/ttyUSB0` 回来

期望：

* 先收到 USB add 事件

* 再做一次 probe

* 状态进入 `BOOTLOADER_READY`

* 随后进入 `PROGRAM_TRANSITION`

* 之后若成功进入 `PROGRAM_READY`

### 4. USB 出现即 program

步骤：

* 控制器已经是 program，再插回 USB

期望：

* USB 事件触发后做一次 probe

* 直接识别为 `PROGRAM_READY`

* 不再去做 bootloader 探测或恢复

### 5. program 在线后的稳定性验证

步骤：

* 控制器进入 program 后持续运行 runtime

期望：

* 心跳只做 program 级确认

* health 持续显示 `PROGRAM_READY`

* bootloader 无回显不会触发误恢复

### 6. 运行中掉电验证

步骤：

* program 在线时执行动作

* 中途断电或换电池

期望：

* 当前动作快速失败

* 状态进入 `RUNTIME_LOST`

* 若 USB 消失，随后切 `NO_PORT`

* 待 USB 回来后，再自动进入 probe/拉起流程

