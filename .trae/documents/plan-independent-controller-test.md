# 独立下位机通信与操作测试计划

## Summary

目标是在 `/home/jetson/workspace/rak-car/test/` 下新建一套**完全独立**的下位机研究与测试工具，不依赖现有 `runtime/`、`main/`、`smartcar.whalesbot.vehicle` 的运行时封装；现有代码只作为协议、动作映射和参数取值的参考来源。

这套工具覆盖全链路：
- 串口枚举与候选端口识别
- `program` 模式握手
- `bootloader` 模式探测
- `RUNCODE` 拉起
- 下载/烧录链路
- 原始业务命令发包与回包解析
- 最小底盘/机械臂动作封装
- 面向人工操作的独立 CLI 与文档

实现结果应满足：
- 所有新代码与新文档均落在 `/home/jetson/workspace/rak-car/test/`
- 新代码不 `import` 现有仓库里的控制 SDK、runtime、下载包装器
- 可以只用 `pyserial` + Python 标准库完成探测、通信、下载、动作测试
- 危险操作必须显式开关，默认安全

## Current State Analysis

基于当前仓库已确认的事实如下。

### 1. 现有仓库没有独立测试目录

- `/home/jetson/workspace/rak-car/test/` 当前为空。
- 仓库没有现成的 `pytest` / `unittest` 体系，主验证方式是独立脚本加真机观察。

### 2. 下位机通信现有实现的真实分层

- 串口总入口在 `smartcar/whalesbot/vehicle/base/serial_wrap.py`
- 设备协议封装在 `smartcar/whalesbot/vehicle/base/mc602_ctl2.py`
- 上层设备语义封装在 `smartcar/whalesbot/vehicle/base/controller_wrap.py`
- runtime 探测/恢复链路在：
  - `runtime/hardware/controller_probe.py`
  - `runtime/hardware/controller_recover.py`
  - `runtime/hardware/controller_download.py`
  - `runtime/hardware/controller_session.py`
- 老下载器实现在 `smartcar/whalesbot/vehicle/base/pydownload.py`

### 3. 关键协议事实已经明确

#### 3.1 program 模式握手与业务帧

- MC602 的 program 模式握手命令参考值是 `02 01 10`
- 现有 `serial_wrap.py` 会将其封装为：
  - 发送帧格式：`77 68 <len> <payload> 0A`
  - 对应握手完整帧：`77 68 07 02 01 10 0A`
- MC602 program 模式回包解析规则：
  - 先读 3 字节
  - 第 3 字节是总帧长
  - 整帧首字节应为 `77 68`
  - 末尾应为 `0A`
  - 业务层取回包体 `res[3:-1]`

#### 3.2 bootloader 模式探测与拉起

- bootloader 探测帧参考值：
  - `BOOT_PING = 55 AA 00 01 08 00 00 F7`
- bootloader 应答由 `runtime/hardware/controller_recover.py` 的 `_is_bootloader_ack()` 判定，当前实现不直接做整包硬编码全等，而是按长度、头、功能码和校验判断。
- 拉起 program 的命令参考值：
  - `RUN_CODE = 55 AA 00 40 0B 00 00 D0 00 08 DD`
- 成功时应出现 `RUNCODE ACK`，之后还需要再次轮询 program 握手确认真正进入 program 模式。

#### 3.3 下载/烧录链路

- 当前仓库真实下载器在 `pydownload.py`
- 其流程是：
  - 枚举串口
  - `ConnectControl()`
  - `PingControl()`
  - `Download()`
  - `SaveNameToStm32()`
  - `RunCode()`
- `Scratch_Download_MC602P()` 支持 `RunA` 到 `RunF` 的分区选择，以及 `isrun=True` 的直接运行模式。

### 4. 动作语义可参考，但不能直接复用

从 `controller_wrap.py` 和 `mc602_ctl2.py` 可以确认最低限度动作映射：
- 蜂鸣器：`beep`
- 四轮底盘：`motor4`
- 单电机：`motor`
- 编码器：`encoder` / `encoder4`
- PWM 舵机：`servo_pwm`
- 总线舵机：`servo_bus`
- 数字输出：`dout`
- 步进：`stepper`
- 红外/模拟量/板载按键/蓝牙手柄等传感器均有明确 `dev_id`

同时可确认：
- 这些设备命令都是由 `dev_id + mode + port + args` 打包后发送
- `ctl602_dev_list` 已给出每类设备的 `dev_id` 与 `struct` 格式，可作为独立实现时的参考字典

### 5. 用户已锁定的范围

本次实现范围已经明确为：
- 全链路
- 包含下载/烧录
- 包含真实动作执行
- 代码必须独立于现有实现，仅参考其协议和参数

## Proposed Changes

下面的改动均以“后续执行阶段将在 `/home/jetson/workspace/rak-car/test/` 下新增文件”为前提。

### 1. 建立独立工具包骨架

新增目录：
- `/home/jetson/workspace/rak-car/test/controller_lab/`

新增文件：
- `/home/jetson/workspace/rak-car/test/controller_lab/__init__.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/constants.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/serial_utils.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/protocol.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/probe.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/bootloader.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/downloader.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/devices.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/actions.py`
- `/home/jetson/workspace/rak-car/test/controller_lab/cli.py`

设计意图：
- `constants.py` 只放常量、设备字典、默认串口参数
- `serial_utils.py` 只放串口打开、枚举、读满、十六进制输出、超时工具
- `protocol.py` 只负责 program 业务帧封装/解包
- `probe.py` 只负责候选端口探测与 program 握手验证
- `bootloader.py` 只负责 bootloader ping、ACK 校验、`RUNCODE`
- `downloader.py` 独立重写最小可用下载逻辑，不调用现有 `pydownload.py`
- `devices.py` 只负责 `dev_id/mode/port/args` 的通用设备发包器
- `actions.py` 在设备层之上提供底盘/机械臂常用动作快捷方法
- `cli.py` 提供统一命令入口

这样可以把“研究通信”和“操作方法”同时沉淀为代码结构，而不是单个大脚本。

### 2. 独立实现 program 模式协议层

新增文件：
- `/home/jetson/workspace/rak-car/test/controller_lab/protocol.py`

实现内容：
- `build_program_frame(payload: bytes) -> bytes`
- `read_program_frame(serial_obj, timeout_s: float) -> bytes`
- `exchange_program_payload(serial_obj, payload: bytes, timeout_s: float) -> bytes`
- `ping_program_mode(serial_obj, timeout_s: float = 0.05) -> bool`

规则固定为：
- 帧头 `77 68`
- 帧长为 `len(payload) + 4`
- 帧尾 `0A`
- 收包后返回 payload 本体

原因：
- 后续所有业务命令、编码器读取、舵机控制都建立在这个统一协议层之上
- 如果不先单独抽出协议层，设备层和 CLI 会被迫复制串口读写细节

### 3. 独立实现 bootloader 探测与拉起

新增文件：
- `/home/jetson/workspace/rak-car/test/controller_lab/bootloader.py`

实现内容：
- `boot_ping(serial_obj) -> tuple[bool, bytes]`
- `is_boot_ack(frame: bytes) -> bool`
- `send_runcode(serial_obj) -> tuple[bool, bytes]`
- `is_runcode_ack(frame: bytes) -> bool`
- `recover_to_program(port_name: str, ...) -> dict`

关键行为：
- 先做端口稳定等待
- 再发 `BOOT_PING`
- 若 bootloader 在场，则尝试多次 `RUN_CODE`
- 每次 `RUN_CODE` 后都轮询 program 握手
- 返回结构化结果，而不是只返回布尔值

原因：
- 用户明确要求“深度研究与操作方法”，不仅要能跑，还要能清楚知道“当前在哪个状态失败”
- 独立工具必须自解释，方便后续手工排障

### 4. 独立实现下载/烧录最小闭环

新增文件：
- `/home/jetson/workspace/rak-car/test/controller_lab/downloader.py`

实现策略：
- 参考 `pydownload.py` 的流程与地址选择，但不调用它
- 第一阶段只实现 MC602 所需的最小链路
- 默认读取仓库现有 `Run.bin` 作为研究对象，但路径通过 CLI 参数显式传入，不做硬编码依赖

最小功能：
- 连接 bootloader
- 识别/校验目标分区：`RunA` 到 `RunF`
- 分块下载
- 下载结果回显
- 可选写入运行槽名
- 可选 `--run-after-download`

安全约束：
- 默认不下载
- 只有显式传入 `--yes-download` 才允许真正写入
- 下载前打印端口、目标槽位、文件大小、运行方式并二次确认

原因：
- 用户已要求纳入下载/烧录
- 但此能力危险度最高，必须通过显式开关与确认语义降低误操作概率

### 5. 独立实现通用设备命令层

新增文件：
- `/home/jetson/workspace/rak-car/test/controller_lab/devices.py`

实现内容：
- 参考 `ctl602_dev_list` 重建独立字典，不从现有模块导入
- 提供通用类：
  - `DeviceCommand`
  - `BatchDeviceCommand`
- 提供通用方法：
  - `set(...)`
  - `get(...)`
  - `reset(...)`
  - `act_mode(...)`
- 支持的首批设备：
  - `motor4`
  - `motor`
  - `encoder4`
  - `encoder`
  - `servo_pwm`
  - `servo_bus`
  - `dout`
  - `stepper`
  - `beep`
  - `sensor_infrared`
  - `sensor_analog`
  - `board_key`
  - `bluetooth`

原因：
- 这是“完全独立、不依赖旧代码”的核心层
- 一旦这层完成，后续动作脚本和 CLI 都可以复用，不必继续参考旧 SDK

### 6. 提供最小动作语义层

新增文件：
- `/home/jetson/workspace/rak-car/test/controller_lab/actions.py`

实现内容：
- `beep(freq=..., duration=...)`
- `set_motor4(speeds)`
- `stop_chassis()`
- `read_encoder4()`
- `set_motor(port, speed)`
- `read_encoder(port)`
- `set_servo_pwm(port, angle, speed)`
- `set_servo_bus(port, angle, speed)`
- `set_dout(port, value)`
- `set_stepper(port, velocity, position)`
- `read_infrared(port)`
- `read_board_key()`
- `read_bluetooth_pad()`

说明：
- 这里只做“最小通用动作”，不复制 `MyCar`、`MecanumDriver` 或 `ArmController` 的复杂业务逻辑
- 底盘动作以“直接四轮速度”和“急停”优先
- 机械臂动作以“单轴/单舵机/单数字输出”优先

原因：
- 用户要求“研究与操作方法”，需要能脱离业务代码直接驱动底层件
- 但不应在 `test/` 下再复制整套车体业务模型

### 7. 提供统一 CLI 与分层命令

新增文件：
- `/home/jetson/workspace/rak-car/test/controller_lab/cli.py`
- `/home/jetson/workspace/rak-car/test/run_controller_lab.py`

CLI 子命令设计：
- `ports`
  - 列候选串口
- `probe`
  - program 握手探测
- `boot-ping`
  - bootloader 探测
- `recover`
  - `RUNCODE` 拉起
- `download`
  - 下载/烧录
- `raw`
  - 发送原始 payload，打印回包十六进制
- `device-set`
  - 通用设备写命令
- `device-get`
  - 通用设备读命令
- `beep`
  - 蜂鸣器测试
- `chassis`
  - 四轮速度测试
- `chassis-stop`
  - 立即停车
- `servo`
  - PWM/总线舵机测试
- `stepper`
  - 步进测试
- `dout`
  - 数字输出测试
- `sensor`
  - 红外/板键/蓝牙手柄读取

CLI 约束：
- 所有高风险命令默认要求显式 `--dangerous`
- `download` 额外要求 `--yes-download`
- `chassis`、`servo`、`stepper` 等真实动作命令执行前打印风险提示

### 8. 补齐研究文档与操作手册

新增文件：
- `/home/jetson/workspace/rak-car/test/README.md`
- `/home/jetson/workspace/rak-car/test/PROTOCOL_NOTES.md`
- `/home/jetson/workspace/rak-car/test/OPERATION_GUIDE.md`

每份文档职责：

`README.md`
- 工具集总览
- 安装依赖
- 最常用命令
- 安全提示

`PROTOCOL_NOTES.md`
- program 帧结构
- bootloader 帧结构
- 关键握手帧
- `dev_id/mode/port/args` 编码方式
- 已验证设备字典

`OPERATION_GUIDE.md`
- 推荐操作顺序：
  - 查串口
  - 探测 program
  - 若失败再测 bootloader
  - `recover`
  - `raw`
  - 单设备
  - 底盘/机械臂
  - 下载/烧录
- 常见故障现象与对应排查动作

原因：
- 用户要的是“深度研究与操作方法”，仅给脚本不够
- 文档必须沉淀成 `test/` 下的长期资产

### 9. 控制依赖与环境边界

新增文件：
- `/home/jetson/workspace/rak-car/test/requirements.txt`

内容原则：
- 尽量只保留 `pyserial`
- 不依赖 FastAPI、requests、OpenCV、Paddle、runtime

原因：
- 用户明确要求“独立测试，不依赖任何现有代码”
- 保持测试环境最小，有利于在控制器异常时快速单独验证

### 10. 明确不做的内容

本轮不纳入：
- 不复制 `runtime` 的会话守护线程和后台状态机
- 不复制 `MyCar` 层的视觉、任务、摄像头、推理能力
- 不复制里程计 PID、任务级闭环导航、业务编排
- 不把 `test/` 做成长期守护服务，仅提供按次执行的 CLI/脚本

原因：
- 本次目标是“独立研究下位机通信与操作”
- 若把视觉和任务编排也拉进来，会重新引入对现有系统的耦合

## Assumptions & Decisions

- 决策：实现语言使用 Python 3，原因是仓库现有参考实现均为 Python，且 Jetson 环境已具备运行条件。
- 决策：独立工具只依赖 `pyserial` 和标准库，避免引入 runtime 级依赖。
- 决策：所有新资产落在 `/home/jetson/workspace/rak-car/test/`，不改动仓库其他业务目录。
- 决策：允许参考现有代码中的协议常量、帧格式、设备字典、地址映射，但最终在 `test/` 中重新定义。
- 决策：CLI 为主入口，原因是它最适合“研究 + 手工操作 + 单步验证”。
- 决策：危险命令默认关闭，必须通过显式参数解锁。
- 假设：当前硬件主要目标是 MC602，mc601 仅保留基础探测兼容，不作为本轮优先动作对象。
- 假设：下载所需二进制文件后续仍可从现有仓库提供，但测试工具不会直接依赖旧下载模块。
- 假设：真实动作验证依赖真机，无法通过纯本地自动化替代。

## Verification Steps

后续执行阶段完成后，按以下顺序验证。

### 1. 静态校验

- 检查 `test/` 下新文件均可导入
- 运行语法检查
- 对核心模块做一次 `GetDiagnostics`

### 2. 只读链路验证

- `python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py ports`
- `python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py probe`
- 若 probe 失败，再执行：
  - `python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py boot-ping`
  - `python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py recover`

期望结果：
- 能清楚分辨“未发现串口 / 在 bootloader / 已在 program / program 无响应”

### 3. 原始协议验证

- 用 `raw` 子命令发送 program 握手 payload `02 01 10`
- 验证回包能够被独立协议层正确解析并打印十六进制

### 4. 单设备验证

- `beep`
- `device-set`/`device-get`
- `sensor`
- `servo`
- `dout`
- `stepper`

期望结果：
- 每个命令都能打印发送帧、回包 payload、结构化解析结果

### 5. 底盘动作验证

- 低速执行 `chassis --lf ... --rf ... --lr ... --rr ...`
- 紧接着执行 `chassis-stop`

期望结果：
- 小车按预期低速动作
- `stop` 可立即停车

### 6. 下载/烧录验证

- 显式传入 `download --file ... --slot RunA --yes-download`
- 可选 `--run-after-download`

期望结果：
- 能完成下载过程并输出阶段日志
- 下载后能重新完成 `probe`

### 7. 文档可用性验证

- 按 `test/OPERATION_GUIDE.md` 从头执行一遍，不查旧代码
- 确认文档本身足以支持独立操作

