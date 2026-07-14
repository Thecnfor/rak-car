# runtime 控制器智能状态机改造计划

## Summary

目标是修复 `runtime` 当前对下位机状态的判断与恢复策略，使其从“探测失败就立刻恢复，恢复失败又可能级联触发 download”的粗暴逻辑，升级为**显式状态机 + 分层探测 + 快失败自愈**的智能控制策略。

本次计划聚焦以下问题：

- 避免 `runtime` 在 bootloader / program 过渡窗口内误判并反复触发恢复
- 避免任何自动 `download` 行为；下载仅允许通过独立 CLI 手动执行
- 区分“无电/无串口”“有串口但无 bootloader”“bootloader 在线但未拉起”“program 在线”“运行中掉线”几种语义完全不同的状态
- 让运行中动作失败采用“当前请求快失败，后台进入自愈”的策略
- 让 `/v1/health` 通过现有 `get_state()` 自动暴露更有意义的控制器状态细节，无需修改 API 路由接口契约

成功标准：

- `runtime` 不再无脑触发 `download...`
- 已处于 `program` 时，`boot-ping` 不响应不会再被误当作异常恢复依据
- bootloader 下优先执行“识别 bootloader + 尝试拉起 program”，不自动烧录
- 动作期间掉线时，当前请求快失败，`controller_session` 进入受控恢复
- `/v1/health` 能看出控制器当前处于哪种状态与最近一次状态迁移原因

## Current State Analysis

### 1. 当前真实现象

根据用户提供的终端现象，当前控制器行为已经明确：

- 在某些时刻，`recover --port /dev/ttyUSB0` 会得到：
  - `bootloader 已响应，但 RUNCODE 未拉起 program`
- 但随后 `probe` 又显示：
  - `MC602 program 握手成功`
- 且此时 `boot-ping` 不再回显，说明：
  - **进入 program 后，bootloader 探测本来就应该无响应**

这说明当前控制器存在一个重要事实：

- `recover` 的一次调用并不能稳定代表“最终状态”
- `bootloader` 到 `program` 可能存在异步过渡窗口
- 一次 `RUNCODE` 未拿到确认，并不等于最终没有进入 `program`

换句话说，**当前 runtime 把“短暂未确认”误当成“恢复失败”** 的概率很高。

### 2. 当前 runtime 的问题链路

已确认以下关键文件参与控制器恢复链路：

- `runtime/server.py`
- `runtime/core/settings.py`
- `runtime/services/runtime_service.py`
- `runtime/hardware/controller_session.py`
- `runtime/hardware/controller_probe.py`
- `runtime/hardware/controller_recover.py`
- `runtime/hardware/controller_download.py`
- `smartcar/whalesbot/vehicle/base/serial_wrap.py`

### 3. 当前探测与恢复耦合过深

#### 3.1 `probe_controller()` 不是纯探测

`runtime/hardware/controller_probe.py` 中的 `probe_controller()` 当前行为是：

1. 列举串口
2. 尝试 `ping_mc602()`
3. 尝试 `ping_mc601()`
4. 失败后直接调用 `recover_controller()`

这意味着名字叫“probe”的函数，实际上做了：

- 探测
- 恢复
- 状态迁移

这会导致：

- 任何调用 probe 的地方，实际上都可能触发恢复副作用
- `controller_session`、`runtime_service`、心跳线程无法控制恢复的触发节奏

#### 3.2 `ensure_ready()` 调用路径会放大这个问题

`runtime/hardware/controller_session.py` 中：

- `ensure_ready()`
- `_recover_once()`
- 心跳线程 `_heartbeat_loop()`

都会走到 `probe_controller()`。

而 `runtime/services/runtime_service.py` 中：

- `_wait_until_ready()`
- `_recover_controller_runtime()`
- `_handle_dispatch_failure()`
- `_auto_init_loop()`

又会继续调用 `controller_session.ensure_ready()` 或 `_probe_controller()`。

因此现在的实际行为是：

- 启动时自动初始化可能触发恢复
- 心跳失败可能触发恢复
- 动作异常可能触发恢复
- 任务失败收尾又可能触发一次 probe

即使 `_recover_lock` 能挡住并发恢复，也挡不住**过于频繁、语义混乱的恢复触发**。

### 4. `serial_wrap` 还保留了自动 download 后门

`smartcar/whalesbot/vehicle/base/serial_wrap.py` 当前逻辑：

- `SerialWrap.sync_with_probe()` 如果根据 probe 无法连上 program，会回退到 `ping_port()`
- `ping_port()` 在握手失败后会遍历 `ctl_dev.download_bin(self)`
- `MC602.download_bin()` 中如果命中 bootloader：
  - 先尝试 `RUNCODE`
  - 若失败且配置允许，会继续 `download_mc602_program()`

虽然 `runtime/core/settings.py` 默认 `AUTO_DOWNLOAD_ON_BOOTLOADER = False`，但从结构上看：

- runtime 当前依然保留“失败后退回 ping_port，再进入 download_bin”这条路径
- 这正是“卡在 `download...`”这类问题的根源之一

### 5. 当前状态模型不够表达现场

`runtime/hardware/controller_session.py` 当前仅有：

- `DISCONNECTED`
- `DEGRADED`
- `PROGRAM_READY`
- `RECOVERING`

这不足以表达以下关键现场：

- 串口不存在 / 下位机无电
- 串口存在但完全无响应
- bootloader 在线但尚未进入 program
- 刚发送过 `RUNCODE`，正在等待 program 起来
- program 刚掉线，等待后台恢复

结果就是：

- `DEGRADED` 同时承载了太多语义
- `PROGRAM_READY` 与“刚刚恢复成功”没有区分
- runtime 无法根据状态做差异化决策

### 6. 当前用户需求已锁定

用户已明确确认以下策略：

- **永远不自动下载**
  - bootloader 下若需要烧录，统一走 `test/run_controller_lab.py` 这套 CLI 手动执行
- **运行时动作失败采用“先快失败，后台自愈”**
  - 当前请求应尽快返回失败，而不是长时间阻塞等待恢复

## Proposed Changes

### 1. 将 `probe` 与 `recover` 拆成两层

涉及文件：

- `runtime/hardware/controller_probe.py`
- `runtime/hardware/controller_recover.py`
- `runtime/hardware/controller_session.py`

改造方向：

- 让 `controller_probe.py` 只负责**无副作用探测**
- 让 `controller_recover.py` 只负责**显式恢复**

具体做法：

#### 1.1 新增纯探测结果模型

在 `controller_probe.py` 中扩展 `ControllerProbeResult`，至少包含：

- `ready: bool`
- `port: str | None`
- `controller: str | None`
- `mode: str | None`
  - 取值建议：
    - `no_port`
    - `program`
    - `bootloader`
    - `unknown`
- `detail: str`
- `bootloader_seen: bool`
- `program_seen: bool`

#### 1.2 新增纯探测接口

在 `controller_probe.py` 中新增或替换为以下分层接口：

- `probe_port_mode(port_name) -> ControllerProbeResult`
  - 只判断单端口当前处于 program / bootloader / unknown
- `probe_controller() -> ControllerProbeResult`
  - 遍历端口，仅返回探测结果
  - **不再调用 `recover_controller()`**

这样 `probe_controller()` 的语义才名副其实。

### 2. 将恢复逻辑改为“基于探测结果的单一入口”

涉及文件：

- `runtime/hardware/controller_recover.py`
- `runtime/hardware/controller_session.py`

改造方向：

- 恢复逻辑不再自己先探测一遍、再做一遍隐式判断
- 而是由 `controller_session` 先拿到纯探测结果，再决定是否调用恢复

具体做法：

#### 2.1 `recover_controller()` 接收探测结果或模式

建议把 `recover_controller()` 改成：

- 输入：
  - `port_name`
  - `probe_result`
  - `port_supplier`
  - `debug_hook`
- 行为：
  - 如果 `probe_result.mode == "program"`：直接返回成功，不做恢复
  - 如果 `probe_result.mode == "bootloader"`：只做 `RUNCODE` 拉起，不做下载
  - 如果 `probe_result.mode == "no_port"`：直接返回“无串口/无电”
  - 如果 `probe_result.mode == "unknown"`：只做有限次数的复探测，不进入下载

#### 2.2 明确禁止 runtime 自动下载

在 `controller_recover.py` 与 `serial_wrap.py` 层同时收口：

- runtime 恢复链路里不再调用任何 `download_mc602_program()`
- `serial_wrap.sync_with_probe()` 在 runtime 场景下不允许回退到 `ping_port() -> download_bin()`

这是本次计划中最重要的硬约束之一。

### 3. 改造 `controller_session` 为显式状态机

涉及文件：

- `runtime/hardware/controller_session.py`

改造方向：

- 用更细粒度状态替代现在的 `DEGRADED` 大杂烩
- 把“探测”“等待 program”“运行中掉线恢复”分开表示

建议新增状态：

- `NO_PORT`
  - 未发现串口，倾向于无电/USB 断开
- `UNKNOWN`
  - 有串口但既非 program 也非 bootloader
- `BOOTLOADER_READY`
  - bootloader 在线，可尝试 `RUNCODE`
- `PROGRAM_READY`
  - program 在线，可执行动作
- `PROGRAM_TRANSITION`
  - 已触发 `RUNCODE`，正在等待 program 接管
- `RUNTIME_LOST`
  - 运行中动作失败，已判定从 program 掉线
- `RECOVERING`
  - 正在做恢复流程

保留或弱化：

- `DISCONNECTED`
- `DEGRADED`

推荐做法是用上述更具体状态替代当前的 `DEGRADED`。

#### 3.1 记录最近一次观测

新增内部字段：

- `_last_probe_mode`
- `_last_probe_at`
- `_last_program_ok_at`
- `_last_bootloader_seen_at`
- `_last_transition_at`
- `_last_runtime_failure_at`
- `_last_recover_reason`
- `_recover_suppressed_until`

这样状态机能区分：

- 是“刚看到 bootloader”
- 还是“刚发过 RUNCODE，先等等”
- 还是“已经多次恢复失败，应暂时抑制重试”

### 4. 引入“智能恢复策略”，替代固定周期无脑恢复

涉及文件：

- `runtime/hardware/controller_session.py`
- `runtime/services/runtime_service.py`

核心策略如下。

#### 4.1 无电 / 无串口

判定条件：

- `probe_controller().mode == "no_port"`

策略：

- 直接进入 `NO_PORT`
- 不执行 `RUNCODE`
- 不执行下载
- 后台仅做低频纯探测
- 动作请求快失败，错误信息明确为“下位机无电/串口不存在”

#### 4.2 有串口但处于 bootloader

判定条件：

- `probe_controller().mode == "bootloader"`

策略：

- 进入 `BOOTLOADER_READY`
- 若最近没有触发过恢复，则执行一次 `RUNCODE`
- 进入 `PROGRAM_TRANSITION`
- 在一个短窗口内只做 program 探测，不重复发 `RUNCODE`
- 若窗口后仍未进入 program，再允许下一轮恢复

这里的关键不是“写死等待多久”，而是：

- **状态驱动抑制重复恢复**
- “刚发过 RUNCODE” 与 “bootloader 长时间稳定存在” 是两种不同情形

#### 4.3 已在 program

判定条件：

- `probe_controller().mode == "program"`
- 或 `serial_wrap.ping_current()` 成功

策略：

- 直接进入 `PROGRAM_READY`
- 清空失败计数
- 清空恢复抑制标记
- 后续不再触发 bootloader 探测

#### 4.4 运行中突然掉电 / 掉线

触发条件：

- 正在执行动作时，命中 `_should_probe_controller(exc)` 所识别的控制器异常

策略：

- 当前动作请求立即失败
- `controller_session` 切到 `RUNTIME_LOST`
- 关闭当前 car 实例
- 后台进入纯探测
- 根据探测结果再决定是否进入 `BOOTLOADER_READY` / `NO_PORT` / `PROGRAM_READY`

这正符合用户要求的“先快失败后自愈”。

### 5. 重构 `runtime_service` 的恢复触发点

涉及文件：

- `runtime/services/runtime_service.py`

改造方向：

- `runtime_service` 不直接参与“探测 + 恢复”的内部细节
- 只消费 `controller_session` 的显式状态机结果

具体做法：

#### 5.1 `_auto_init_loop()` 只在必要状态下初始化

现在 `_auto_init_loop()` 每轮都可能：

- `_probe_controller()`
- `ensure_initialized()`

建议改为：

- 先读取 `controller_session.snapshot()`
- 仅当状态为 `PROGRAM_READY` 且 car 未就绪时，尝试 `ensure_initialized()`
- 对于 `NO_PORT` / `BOOTLOADER_READY` / `PROGRAM_TRANSITION`：
  - 不直接构造 `MyCar()`
  - 只让 `controller_session` 自己推进状态

#### 5.2 `_wait_until_ready()` 不再主动拉复杂恢复链

当前 `_wait_until_ready()` 内部失败后会：

- `_probe_controller()`
- `_mark_controller_offline()`

建议改为：

- 只向 `controller_session` 上报当前失败
- 读取状态机状态
- 若当前不是 `PROGRAM_READY`，直接返回更具体错误
- 不在单次请求中重复套娃恢复

#### 5.3 `_dispatch()` 动作失败后快失败

当前 `_dispatch()` 在控制器异常时会：

1. `_recover_controller_runtime(exc)`
2. 若恢复成功，再 `_wait_until_ready()`
3. 再自动重试一次动作

这与用户要求不一致。

建议改为：

- 动作异常时：
  - `controller_session.note_io_failure(...)`
  - 关闭 car
  - 异步/后台推进恢复
  - 当前请求直接失败返回

也就是：

- 取消“当前请求内自动恢复并重试一次”的策略

### 6. 从 `serial_wrap` 中移除 runtime 自动 download 入口

涉及文件：

- `smartcar/whalesbot/vehicle/base/serial_wrap.py`

改造方向：

- 避免 runtime 场景下 `sync_with_probe()` 失败后，落到 `ping_port() -> download_bin()`

建议做法：

#### 6.1 `sync_with_probe()` 失败时不再回退到 `ping_port()`

当前逻辑：

- `probe_result` 连不上 → 回退到 `ping_port()`

建议改为：

- 若传入了 `probe_result`：
  - 只尝试按 `probe_result.port` / `probe_result.controller` 连 program
  - 失败就直接抛 `ControllerNotReadyError`
  - 不回退到全串口扫描与 `download_bin()`

#### 6.2 `download_bin()` 仅保留给手工路径

保留代码实现，但 runtime 恢复链路不再依赖它。

这样可以避免：

- 明明只是 bootloader 到 program 的自然过渡
- 却被 `serial_wrap` 当成要进入 download 流程

### 7. 调整健康接口展示，但不改 API 契约

涉及文件：

- `runtime/hardware/controller_session.py`
- `runtime/services/runtime_service.py`
- `runtime/api/routes.py`（无需接口结构改动，仅确认兼容）

当前 `routes.py` 中 `health_payload()` 已直接返回：

- `service.get_state()`

因此无需新增 API 路由，只需增强 `get_state()` 输出即可。

建议在 `get_state()` / `snapshot()` 中新增：

- `controller_session.mode`
- `controller_session.last_probe_mode`
- `controller_session.last_recover_reason`
- `controller_session.recovering`
- `controller_session.recover_suppressed_until`
- `controller_session.last_runtime_failure_at`

这样 `/v1/health` 就能直接体现：

- 当前是没电、bootloader、program 还是过渡中
- 是否刚刚发过 `RUNCODE`
- 为什么当前不继续恢复

### 8. 补充调试埋点，验证“智能”是否生效

涉及文件：

- `runtime/hardware/controller_probe.py`
- `runtime/hardware/controller_recover.py`
- `runtime/hardware/controller_session.py`
- `runtime/services/runtime_service.py`

新增埋点重点：

- 纯探测结果：`no_port / bootloader / program / unknown`
- 状态迁移：例如 `BOOTLOADER_READY -> PROGRAM_TRANSITION`
- 恢复抑制原因：例如“刚发过 RUNCODE，等待 program 接管”
- 动作失败触发的快失败与后台自愈
- 明确区分：
  - “本轮没有恢复”
  - “本轮尝试恢复但未成功”
  - “已看到 program，不再执行 bootloader 探测”

### 9. 保留现有手工 CLI 作为唯一烧录路径

不改文件，但在设计上明确：

- `test/run_controller_lab.py recover --port /dev/ttyUSB0`
  - 作为人工拉起 program 的独立工具
- `test/run_controller_lab.py download ...`
  - 作为人工烧录入口

runtime 本身只负责：

- 探测
- 识别
- 拉起 program
- 状态上报

不负责烧录。

## Assumptions & Decisions

- 决策：runtime 永远不自动下载，下位机烧录仅允许通过 `test/` 下独立 CLI 手工触发。
- 决策：bootloader 在线时，runtime 最多尝试 `RUNCODE` 拉起 program，不做 download。
- 决策：动作期间串口/控制器异常采用“当前请求快失败，后台自愈”。
- 决策：`probe_controller()` 必须改造成纯探测函数，不能继续带恢复副作用。
- 决策：`/v1/health` 继续沿用现有 `health_payload()` 输出结构，不新增路由，只增强 `state` 细节。
- 假设：控制器从 bootloader 到 program 可能存在异步过渡窗口，因此不能把一次 `RUNCODE` 未确认直接等价为恢复失败。
- 假设：进入 program 后，`boot-ping` 不回显是正常行为，不能作为进一步恢复或 download 的依据。
- 假设：当前主要目标控制器是 MC602，mc601 仅保留基础兼容。

## Verification Steps

### 1. 静态验证

- 检查以下文件无语法/诊断错误：
  - `runtime/hardware/controller_probe.py`
  - `runtime/hardware/controller_recover.py`
  - `runtime/hardware/controller_session.py`
  - `runtime/services/runtime_service.py`
  - `smartcar/whalesbot/vehicle/base/serial_wrap.py`

### 2. 纯探测验证

在不同真实现场下验证：

- 无 USB 串口
  - `/v1/health` 显示 `NO_PORT`
- bootloader 在线
  - `/v1/health` 显示 `BOOTLOADER_READY`
- program 在线
  - `/v1/health` 显示 `PROGRAM_READY`

并确认：

- 纯探测不会触发 download
- 纯探测不会隐式做恢复

### 3. bootloader 到 program 过渡验证

现场步骤：

1. 控制器进入 bootloader
2. 观察 runtime 状态：
   - 先识别 `BOOTLOADER_READY`
   - 再短暂进入 `PROGRAM_TRANSITION`
   - 最终进入 `PROGRAM_READY`

确认：

- 不会连续无脑发 `RUNCODE`
- 不会进入 `download...`

### 4. program 在线验证

在控制器已进入 program 后验证：

- `probe` 能成功
- bootloader 探测无回显
- runtime 不会因为 bootloader 无回显而误判异常

### 5. 运行中掉线验证

现场步骤：

1. 先让 runtime 已初始化完成
2. 执行一个真实动作
3. 中途断电或让串口失效

确认：

- 当前动作请求快速失败
- runtime 不会长时间卡住请求
- `/v1/health` 状态切换到 `RUNTIME_LOST` 或等价状态
- 后台开始探测，并根据现场进入 `NO_PORT` / `BOOTLOADER_READY` / `PROGRAM_READY`

### 6. 自动初始化验证

确认：

- `_auto_init_loop()` 仅在 `PROGRAM_READY` 且 car 未初始化时尝试构造 `MyCar`
- bootloader 状态下不会继续构造 car
- 无电状态下不会反复构造 car

### 7. download 抑制验证

重点检查：

- `runtime` 日志中不再出现自动 `downloading program`
- `serial_wrap.sync_with_probe()` 在 runtime 路径下不会回落到 `download_bin()`
- 即使恢复失败，也只会上报状态，不会自动烧录

