# 工程分层 + topic/service/action 接口设计

- 日期: 2026-08-09
- 分支: `develop/ros2-sidecar`
- 状态: 已批准(用户确认三项推荐 + "不要有历史遗迹压力,直接开始")
- 相关: `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md`(旧架构,被本节取代/演进)

## 1. 背景与目标

当前 `develop/ros2-sidecar` 已有完整驱动层(MC602 协议 + 桥仲裁 + 类型化设备 API),
但**层与层之间没有清晰接口契约**:

- `arm_node` 直接订阅 `JointTrajectory` topic(单向 fire-and-forget,无结果/取消)。
- `chassis` 只吃 `Twist` topic(行为层要自己拼运动学)。
- `mission_runner` 是极薄的行为层前身(只发 String 进度)。
- 驱动层 typed API 只被组件内部用,没有暴露成稳定的 ROS 接口。

同时**串口降速到 115200**(旧默认 1M 的 ~1/8.7):读策略必须显式预算,
否则 50Hz×多节点 会把总线挤爆。

本次设计目标:

1. 明确四层:driver / component(arm+chassis 两个核心组件) / behavior / cognition(未来)。
2. 为每一层定义**稳定、类型化、省串口**的 topic/service/action 契约。
3. 组件层对外只暴露语义接口,**不暴露原始 MC602 帧**;串口唯一所有者仍是 bridge。
4. 行为层只做编排(调组件 Action),**不做运动学、不碰原始帧**。
5. cognition 层本期只**预占命名空间**,不创建任何 publisher/service。

## 2. 分层架构

```
┌──────────────────────────────────────────────────────────────┐
│ cognition  认知层(未来拓展,namespace 预占,本期不实现)         │
│   /rak/cognition/*        AI 任务规划 / 意图 / 决策            │
├──────────────────────────────────────────────────────────────┤
│ behavior  功能层(编排,不做运动学)                             │
│   behavior/<task>_node   订阅感知+状态 → 调组件 Action         │
│                            → 发 /rak/state/task/<id>/status    │
├──────────────────────────────────────────────────────────────┤
│ component  组件层(两个核心组件)                                │
│   chassis_node    ChassisNavigate action · odom · vel_safe     │
│   arm_node        ArmExecuteTrajectory action · 关节状态        │
│   system_io_node  非臂/底盘杂项:beep/led/nixie/按键/传感器按需读 │
├──────────────────────────────────────────────────────────────┤
│ driver  驱动层(单串口 115200 唯一所有者)                      │
│   mc602_bridge_node   仲裁 + 突发打包 + 程序模式拉起           │
│   camera_node ×2 · infrared_node  只读流(USB/V4L2,不经串口)     │
└──────────────────────────────────────────────────────────────┘
```

铁律:

- **串口只有一个所有者**(bridge)。所有设备访问一律经 `/rak/hw/mc602/transaction`
  service(原始帧批量,现状实现不动)。组件/系统节点是它的客户端。
- **组件层 = 类型化设备门面**:用 `MC602Adapter` typed API 包一层,向外只暴露
  Action / Service / State topic 语义接口。
- **传感器读**:连续状态 → 组件自持轮询、频率收敛(§4);一次性读 → 按需 service。
- **相机/IR 不经串口**(USB/V4L2),driver 层直发流,不占串口预算。

## 3. 节点清单(目标态)

| 节点 | 归属层 | 职责 | 状态 |
|---|---|---|---|
| `mc602_bridge_node` | driver | 串口唯一所有者,Mc602Transaction srv,突发仲裁,程序模式拉起 | 现成不动 |
| `camera_node` ×2 | driver | V4L2 读流 + tf_static | 现成不动 |
| `infrared_node` ×2 | driver | IR 流(频率收敛 20→10Hz 默认) | 小改(频率参数) |
| `chassis_node` | component | **ChassisNavigate action** + vel_safe 订阅 + odom + reset_encoders | 改造 |
| `arm_node` | component | **ArmExecuteTrajectory action** + 关节状态 + set_vacuum/set_valve | 改造 |
| `system_io_node` | component | beep/led_light/led_show/nixie/read_sensor/read_key/read_pad service | **新增** |
| `safety_gate_node` | driver/安全 | vel_raw→vel_safe 过滤 | 现成不动 |
| `behavior/*` | behavior | 编排,调组件 Action,发 TaskStatus | mission_runner 演进 |
| (cognition) | cognition | 预留 | 本期不建 |

## 4. 串口预算(115200 约束,硬数字)

115200 baud 8N1 ≈ **11,520 B/s**;一次帧往返 ≈ 30 B → 理论上限 ~380 tx/s,
工程按 **~150-250 tx/s** 设计,留余量。

| 消费者 | 频率 | 帧/tick | tx/s | B/s(估 30B/往返) |
|---|---|---|---|---|
| chassis 控制+odom | 50 Hz | 1 写(motor4)+1 读(encoder4)=突发 | 100 | 3000 |
| arm 控制 | 20 Hz | 1 突发(3-4 写) | 20 | 600 |
| IR 流 | 10 Hz | 1 读 | 10 | 300 |
| 系统一次性操作 | 按需 | ~0 均值 | ~5 | 150 |
| **合计** | | | **~135 tx/s** | **~4050 B/s(35%)** |

留 **~65% 余量** 给:突发/重试/按需读/诊断。

**读策略(组件自持轮询 + 按需读):**

- chassis tick = `begin_burst → set_motor4 + read_encoder4 → commit_burst` 一个
  service 调用原子下发(现有 `submit_batch` 支持)。
- 一次性读(IR 单测、超声波、编码器单口)走 `SensorQuery` service,按需、不占周期。
- 组件内用 `MC602Adapter` typed API + `set_injection` 单测,测试不打真串口。
- **所有频率走 launch 参数,节点不写死。** 新默认: arm 20Hz、IR 10Hz、chassis 50Hz。

## 5. Topic 清单(全部 `/rak/...`)

### 5.1 感知/传感器(现状,除 IR 频率参数外不动)

| 命名空间 | 类型 | 频率 | QoS |
|---|---|---|---|
| `/rak/sensors/camera/<id>/image_raw` | `sensor_msgs/Image` | 30 Hz | BEST_EFFORT d=1 |
| `/rak/sensors/camera/<id>/image_compressed` | `sensor_msgs/CompressedImage` | 30 Hz | BEST_EFFORT d=1 |
| `/rak/sensors/camera/<id>/camera_info` | `sensor_msgs/CameraInfo` | 校准有效才发 | TRANSIENT_LOCAL |
| `/rak/sensors/camera/<id>/camera_status` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | RELIABLE |
| `/rak/sensors/camera/<id>/camera_meta` | `msgs/msg/CameraMeta` | 1 Hz | RELIABLE |
| `/rak/sensors/ir/<id>` | `sensor_msgs/Range` | **10 Hz(新默认)** | RELIABLE |

### 5.2 组件状态(组件自持轮询)

| 命名空间 | 类型 | 频率 | QoS | 说明 |
|---|---|---|---|---|
| `/rak/state/odom` | `nav_msgs/Odometry` | 20-50 Hz | RELIABLE d=10 | chassis 自持 |
| `/rak/state/joint_states` | `sensor_msgs/JointState` | 10-20 Hz | RELIABLE d=10 | arm 4 关节 + 4 轮合并 |

> 决定:关节状态统一 `sensor_msgs/JointState`,废弃 `msgs/msg/ActuatorState`(冗余变体)。
> `/rak/state/actuators/<id>` 并入 `/rak/state/joint_states`,不重复发。

### 5.3 命令与安全(现状,不动)

| 命名空间 | 类型 | 说明 |
|---|---|---|
| `/rak/cmd/vel_raw` | `geometry_msgs/Twist` | 行为层/认知层可写,经安全门 |
| `/rak/cmd/vel_safe` | `geometry_msgs/Twist` | 安全门输出,chassis 低层覆盖(逃生舱) |
| `/rak/safety/estop` `_heartbeat` `_mode_cmd` | 现状 | 不动 |

### 5.4 行为与认知

| 命名空间 | 类型 | 频率 | QoS | 说明 |
|---|---|---|---|---|
| `/rak/state/task/<task_id>/status` | **`msgs/msg/TaskStatus`(新)** | 行为层心跳 | TRANSIENT_LOCAL | 任务状态机 |
| `/rak/perception/detections/<model_id>` | `DetectionArray` | 现状 | 现状 | 认知层未来消费 |
| `/rak/perception/lane` | `LaneResult` | 现状 | 现状 | 同上 |
| `/rak/cognition/*` | 预留 | — | — | **本期不创建 publisher** |

## 6. Service 清单

| Service | 位置 | 类型 | 说明 |
|---|---|---|---|
| `/rak/hw/mc602/transaction` | bridge | `Mc602Transaction.srv` | 原始帧批量(内部通道,现状不动) |
| `/rak/chassis/reset_encoders` | chassis | `std_srvs/Trigger` | 一次性,离线 |
| `/rak/arm/<id>/set_vacuum` | arm | `std_srvs/SetBool` | 真值泵开/关 |
| `/rak/arm/<id>/set_valve` | arm | `std_srvs/SetBool` | 电磁阀开/关 |
| `/rak/hw/system/read_sensor` | system_io | **`SensorQuery.srv`(新)** | 通用按需读:`port+type → value` |
| `/rak/hw/system/beep` | system_io | **`Beep.srv`(新)** | freq + duration |
| `/rak/hw/system/led_light` | system_io | **`SetRgbLed.srv`(新)** | led_id + r,g,b |
| `/rak/hw/system/led_show` | system_io | **`LedShow.srv`(新)** | 点阵文本 ≤100 字符 |
| `/rak/hw/system/nixie` | system_io | **`Nixie.srv`(新)** | int32 值 |
| `/rak/hw/system/read_key` | system_io | **`ReadIntArray.srv`(新)** | 板载按键(自定义响应,Trigger 装不下 int64[]) |
| `/rak/hw/system/read_pad` | system_io | **`ReadIntArray.srv`(新)** | 蓝牙手柄(同前) |

> 决定:`read_sensor` 用**一个带 `type` 枚举的通用 service**(映射驱动层 `read_sensor(port, type)`),
> 不建 6 个近重复 srv。beep/led/nixie 签名各不同,单独成 srv。
> `read_key`/`read_pad` 需返回 `int64[]`,`std_srvs/Trigger` 装不下 → 用一个
> `ReadIntArray.srv(source: "key"|"pad" → int64[] values)` 复盖两个设备。

## 7. Action 清单(两个自定义 action —— 核心新接口)

### 7.1 `ChassisNavigate.action`(chassis_node 服务端)

```text
# Goal
geometry_msgs/Pose2D target_pose      # 底盘系内目标位姿 (x, y, theta)
float32 max_linear_speed              # m/s
float32 max_angular_speed             # rad/s
float32 tolerance_lin                 # 到位容差 (m)
float32 tolerance_ang                 # 到位容差 (rad)
float32 timeout_sec                   # 硬超时 → ABORTED
---
# Result
bool success
string error                          # timeout | transport | stale | none
float32 traveled_distance
---
# Feedback
geometry_msgs/Pose2D current_pose     # 当前里程估计
float32 remaining_distance
```

- Action 名字:`/rak/chassis/navigate`(展开为 goal/feedback/result/status 四 topic)。
- chassis 内部自持里程计做闭环点到位;行为层不再自己拼运动学。
- 平移"距离 + 角度"场景 = `target_pose=(dx, dy, dtheta)`。
- 逃生舱:`/rak/cmd/vel_safe` Twist 仍可直发(手动/调试)。

### 7.2 `ArmExecuteTrajectory.action`(arm_node 服务端)

```text
# Goal
string arm_id
trajectory_msgs/JointTrajectory trajectory   # 现有关节名 + 目标点(沿用现状语义)
float32 max_execution_time                    # 硬超时 → ABORTED
---
# Result
bool success
string error                                  # timeout | transport | out_of_range | none
float32[] final_positions
---
# Feedback
trajectory_msgs/JointTrajectoryPoint current  # 执行中的关节位置
```

- Action 名字:`/rak/arm/<arm_id>/execute_trajectory`。
- 服务端内部每 tick `begin/commit_burst`;行为层一次 goal = 组件内部 N 个突发帧。
- 预emption:新 goal 自动取消旧 goal(行为层可随时打断)。
- `JointTrajectory` low-level topic 保留为逃生舱,不再是行为层唯一通道。

## 8. 新增消息/服务/动作文件汇总(msgs 包)

| 文件 | 类型 | 备注 |
|---|---|---|
| `msg/TaskStatus.msg` | msg | `task_id/state/current_step/progress/message` |
| `srv/SensorQuery.srv` | srv | `uint8 port + string type → float64 value + bool ok + string error` |
| `srv/Beep.srv` | srv | `uint16 freq + float32 duration_s → bool ok` |
| `srv/SetRgbLed.srv` | srv | `uint8 led_id + uint8 r,g,b → bool ok` |
| `srv/LedShow.srv` | srv | `string text → bool ok` |
| `srv/Nixie.srv` | srv | `int32 value → bool ok` |
| `srv/ReadIntArray.srv` | srv | `string source → bool ok + string error + int64[] values` |
| `action/ChassisNavigate.action` | action | §7.1 |
| `action/ArmExecuteTrajectory.action` | action | §7.2 |
| 废弃 | — | `msg/ActuatorState.msg` 删除(被 JointState 取代) |

`TaskStatus.msg` 定义:

```text
std_msgs/Header header
string task_id
string state          # IDLE | RUNNING | SUCCEEDED | FAILED | ABORTED
string current_step
float32 progress      # 0.0 - 1.0
string message
```

## 9. 行为层与认知层

- **behavior**:mission_runner 演进。每任务一节点或 manager 挂多任务;
  订阅感知 + 组件状态,调组件 Action 编排,心跳发 TaskStatus。
  不写运动学、不碰原始帧。
- **cognition**:本期只预占 `/rak/cognition/*`(spec 留位),不创建任何 publisher/service。
  未来(LLM/AI 规划)消费 `/rak/perception/*` 并向 `/rak/cmd/vel_raw` 或行为层发意图。

## 10. 错误处理

- 组件对串口失败 `throw`(no-mocks 规则)→ Action 以 `error` 字段 + `ABORTED` 返回。
- 行为层收到非 SUCCEEDED → 发 `TaskStatus FAILED`,可重试。
- Action 预emption:新 goal 取消旧 goal。

## 11. 测试

- 驱动层:现有 76 gtest 金帧测试不动。
- 组件层:`set_injection` 假响应 → 验证 `goal → 帧 → feedback → result` 契约(新增 action 契约 gtest)。
- 行为层:mock 组件(Python)跑编排冒烟,PyTest <0.5s。
- 串口预算:bench.py 已有,补充 115200 下满载吞吐断言。

## 12. 迁移/兼容

- `/rak/state/actuators/<id>`(JointState)废弃,并入 `/rak/state/joint_states`。
- `msgs/msg/ActuatorState` 删除。
- arm 的 `/rak/cmd/arm/<id>/trajectory` topic 保留(逃生舱),但行为层走 Action。
- IR 频率默认 20→10 Hz(参数可调)。
- 桥 / 相机 / 安全门 / 感知 topic 全部不变。

## 13. 实施顺序(供 writing-plans)

1. msgs 包:新增 msg/srv/action + 删 ActuatorState(改 CMakeLists、package.xml)。
2. system_io_node(新,基于 MC602Adapter typed API + bridge service 客户端)。
3. chassis_node:加 ChassisNavigate action server + reset_encoders service。
4. arm_node:加 ArmExecuteTrajectory action server + set_vacuum/set_valve services,状态并入 joint_states。
5. infrared_node:频率参数化(默认 10Hz)。
6. 行为层:mission_runner 演进为调 Action 的编排(至少一个示例任务)。
7. 契约 gtest + mock 编排 PyTest。
8. colcon build + 全量测试 + smoke。
