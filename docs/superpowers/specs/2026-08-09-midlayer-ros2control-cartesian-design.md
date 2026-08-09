# 中间层设计：ros2_control 主控制平面 + Cartesian 机械臂

- **日期**: 2026-08-09
- **分支**: develop/ros2-sidecar
- **状态**: 已获用户批准（2026-08-09 逐节确认）
- **Supersedes**: 不取代 2026-07-05-ros2-sidecar-design.md（那是历史架构档案），本文件是中间层增量设计

## 1. 目标

在已完成的 MC602 驱动层之上，建立统一的 ROS2 中间层：

1. **ros2_control 作为唯一主控制平面**（底盘 + 机械臂 + 泵阀），现有 `mecanum_chassis_node` / `arm_node` 退役为迁移兼容层。
2. **动作语义用 ROS2 Action**：`ChassisNavigate` / `ArmCartesianMove` / `ArmExecuteTrajectory`。
3. **机械臂支持任务空间（Cartesian）控制**，但诚实限定为：
   - `frame_id=arm_base`，`x / z / yaw` + 可选固定 `y` + grip/泵/阀
   - `y != 固定平面` → 明确返回 `UNSUPPORTED_DIMENSION`
   - 结果区分 `REACHED / UNREACHABLE / TIMEOUT / HARDWARE_FAULT / CANCELLED / PARTIAL / DEGRADED_NO_FEEDBACK`
4. **尽量复用现成 ROS2 生态**，自己只写 4 块（见 §7）。
5. MC602 保持单总线单 owner（唯一 fd 在 `mc602_bridge_node`）。

## 2. 硬件参数（用户提供, 2026-08-09）

```text
底盘: 4 轮全向麦克纳姆
机械臂: 4 轴
  x    0 ~ +300 mm   (M6 平移; 原 -300~0, 已翻正; 方向仍由标定确认)
  z    0 ~ +300 mm   (stepper3 平移; 原 -300~0, 已翻正; 方向仍由标定确认)
  yaw -150° ~ +150°  (S3 旋转, PWM 舵机, 位置准但无编码器反馈)
  grip ±90°          (S7 夹爪/末端, PWM 舵机)
反馈:
  M6, stepper3 有编码器回读
  S3, S7 无编码器 → open_loop 但置信度 HIGH
轴间长度: 仅粗略值 → 几何模型参数化, 上电标定流程确认方向/零点
```

## 3. 总体架构

```text
行为层 (tasks / AI)
  ChassisNavigate.action
  ArmCartesianMove.action
  ArmExecuteTrajectory.action
  Peripheral 服务 (beep/led/nixie/dout)
        │
        ▼
controller_manager (ros2_control)
  ├── mecanum_drive_controller      (现成, ros-humble-mecanum-drive-controller)
  ├── joint_trajectory_controller   (现成)
  ├── joint_state_broadcaster       (现成)
  └── gpio_controllers              (现成, 泵/阀)
        │
        ▼
MC602HardwareInterface              (唯一自写硬件插件, 走 bridge)
        │
        ▼
mc602_bridge_node → SerialScheduler → SerialPort → MC602
```

### 3.1 依赖（Humble 已发布, 已核对 rosdistro）

| 包 | 用途 | Humble |
|---|---|---|
| ros-humble-mecanum-drive-controller | 底盘 IK/odom/TF/cmd_vel | ✅ (2.53.3) |
| ros-humble-joint-trajectory-controller | 机械臂标准轨迹 + FollowJointTrajectory action | ✅ |
| ros-humble-joint-state-broadcaster | 关节状态广播 | ✅ |
| ros-humble-gpio-controllers | 泵/阀 digital output | ✅ |
| ros-humble-kdl-parser / liborocos-kdl | 串联链 FK | ✅ |

> `omni_wheel_drive_controller` 未在 Humble 发布（仅在仓库），本平台是 mecanum，不受影响。

## 4. 接口层（src/msgs 新增）

### 4.1 Actions

**ChassisNavigate.action**

```text
# goal
geometry_msgs/Pose2D target_pose
bool use_pose
geometry_msgs/Twist target_velocity
float32 linear_tolerance
float32 angular_tolerance
float32 timeout_sec
---
# result
bool success
string error
float32 travelled_distance
float32 duration_sec
---
# feedback
geometry_msgs/Pose2D current_pose
float32 remaining_distance
```

**ArmCartesianMove.action**

```text
# goal
string frame_id                # 第一版仅 "arm_base"
float32 x                      # mm
float32 z                      # mm
float32 yaw_deg
bool y_enabled                 # false=固定工作平面
float32 y                      # 仅 y_enabled 且固定平面时有效
uint8 gripper_action           # 0=none 1=grip 2=release 3=pump_on 4=pump_off
float32 velocity_scale         # 0.0~1.0
float32 position_tolerance     # mm
float32 timeout_sec
---
# result
uint8 status                   # REACHED/UNREACHABLE/TIMEOUT/HARDWARE_FAULT/CANCELLED/PARTIAL/DEGRADED_NO_FEEDBACK
string error
sensor_msgs/JointState final_joints
---
# feedback
sensor_msgs/JointState joint_state
geometry_msgs/Pose current_cartesian
float32 progress
```

**ArmExecuteTrajectory.action**

```text
# goal
trajectory_msgs/JointTrajectory trajectory
float32 timeout_sec
# result / feedback 与 Cartesion 风格一致
```

### 4.2 状态消息

**ArmKinematicState.msg**

```text
std_msgs/Header header
string[] joint_names
float64[] position             # rad / m, 按模型
float64[] velocity
uint8[] feedback_source        # 0=encoder 1=open_loop_high_conf 2=open_loop 3=unknown
float64[] confidence
geometry_msgs/Pose tool_pose   # FK 结果
bool reachable
```

**ActuatorDiagnostics.msg**

```text
std_msgs/Header header
string[] actuator_names
uint8[] state                  # 0=ok 1=stale 2=blocked 3=fault 4=unknown
float32[] last_commanded
float32[] last_measured
uint32[] sequence
```

### 4.3 外设服务

```text
SetBeep.srv       # freq, duration_ms
SetRgbLed.srv     # r,g,b
LedShow.srv       # buffer / 显示模式
SetNixie.srv      # 数字
SetDigitalOut.srv # port, on/off
```

## 5. 底盘（复用 mecanum_drive_controller）

- 参数映射：`wheel_radius ← wheel_radius`, `wheel_separation_x ← 2*Lx`, `wheel_separation_y ← 2*Ly`（现有 `mecanum_chassis` 常量）。
- `cmd_vel` 输入 = `/rak/cmd/vel_safe`（安全门输出），odom/TF 由 controller 发布到 `/rak/state/odom` + `/tf`。
- 现有 `mecanum_chassis_node` 的 odom/IK 退役。
- `ChassisNavigate` action server = 薄位姿控制器：读 odom → 位姿闭环 → 发布 `cmd_vel`。第一版不接 Nav2。

## 6. 机械臂

### 6.1 关节模型

关节名（order 固定）：`arm_x / arm_z / arm_yaw / arm_grip`

```text
arm_x     prismatic  0..300mm  encoder       (M6)
arm_z     prismatic  0..300mm  encoder       (stepper3)
arm_yaw   revolute   ±150°     open_loop_high_conf (S3)
arm_grip  revolute   ±90°      open_loop_high_conf (S7)
```

### 6.2 ArmKinematicModel（纯参数/数据, rclcpp-free）

- 每轴：类型、origin、limits、scale（mm→raw：M6 周长、stepper 步数；deg→raw：servo 字节）、feedback 类型
- 方向/零点为参数，上电标定（home_and_verify）确认
- 用 `kdl_parser` 生成 KDL chain 做 FK；解析 IK 针对 x/z/yaw（3DoF，确定性最近邻+限位），不可达 → `UNREACHABLE`

### 6.3 ArmCartesianMove 执行链

```text
goal(x,z,yaw,grip/pump/valve)
  -> 参数/限位校验 (y!=固定平面 → UNSUPPORTED_DIMENSION)
  -> KDL FK 校验当前可达性
  -> 解析 IK -> 最近邻 + 限位
  -> 生成 JointTrajectory
  -> 调 joint_trajectory_controller 的 FollowJointTrajectory action
  -> 周期读 state (encoder 回读 / open_loop_high_conf)
  -> 判定 REACHED / TIMEOUT / BLOCKED / PARTIAL / ...
```

### 6.4 反馈语义

```text
ReachedFeedback:
  encoder_verified   (有编码器且误差<=tolerance)
  open_loop_high     (无编码器但舵机指令可信任)
  stale              (反馈太久)
  blocked            (编码器不回读+命令超时)
  unknown
```

堵转检测（第一版）：编码器不回读 + 命令超时 → BLOCKED；不读电流/故障寄存器（除非确认 MC602 暴露）。

## 7. 自写范围（收敛到 4 块）

```text
1. BridgeTransport 异步化          # 改现有, P0 地基, executor-safe
2. MC602HardwareInterface 完整实现  # 补现有 stub, read/write 走 bridge
3. ArmCartesianMove action server  # KDL FK + 解析 IK + 反馈语义
4. PeripheralNode + CommandArbiter + ChassisNavigate action
```

### 7.1 BridgeTransport 异步化

- 禁止在 controller/hardware 回调内 `spin_until_future_complete`
- 方案 A（推荐）：hardware 插件侧独立 client node + 专用线程；等待在阻塞线程，不重入主 executor
- 备选：B. 回调式异步（`async_send_request` + response 队列，控制周期轮询）；C. 插件在 `on_activate` 起专用 executor/线程持有 client。三者都遵守“回调内不递归 spin”

### 7.2 MC602HardwareInterface

```text
on_init:     读 URDF serial/baud + 关节
on_activate: 连 bridge (bridge transport), 不直接开串口; 失败 → lifecycle ERROR + 重试
read():      encoder4 / IR / 关节状态 -> state interfaces
write():     motor4 / 关节 position / 泵阀 DOUT -> bridge burst
```

- 全部经 bridge，保持单 fd owner
- 未进入 program mode 时不报告 READY

## 8. 外设与仲裁

- beep/RGB LED/点阵/数码管 → `PeripheralNode` service（复用 led_show/nixietube/beep 驱动）
- 泵/阀/继电器 → `gpio_controllers` command interface
- `CommandArbiter`（rclcpp-free）：CHASSIS / ARM / PUMP / PERIPHERAL 资源互斥；URGENT 永远赢；ARM 与 CHASSIS 长动作互斥；泵阀跟随机械臂
- 仲裁状态发布 `/rak/state/arbitration`

## 9. launch（middleware.launch.py）

```text
1. mc602_bridge_node            (唯一 fd, 先起, lifecycle/readiness)
2. controller_manager + MC602HardwareInterface
3. mecanum / joint_trajectory / joint_state_broadcaster / gpio controllers
4. ChassisNavigate / ArmCartesianMove / PeripheralNode
5. behavior 层 (可选)
```

- 启动顺序用 event_handler / lifecycle_manager，不靠构造时死等
- bridge 未就绪 → 插件 on_activate ERROR + 重试，不对外报 READY

## 10. 测试策略

- 纯逻辑 gtest（rclcpp-free）：解析 IK、KDL FK、CommandArbiter、反馈判定
- 集成：fake bridge service + 真 BridgeTransport（验证 executor-safe）、controller 冒烟
- launch_testing：middleware.launch.py 无硬件（dev-only stub transport）可起
- 金标：MC602 帧保持字节级对齐（沿用 gold-frame 测试）
- 退役路径：mecanum_chassis_node / arm_node 移入兼容层，不再作为主控制路径

## 11. 明确不做（第一版）

- 完整 6D pose 任意控制（4DoF 拓扑限制）
- 完整 URDF/MoveIt（无完整几何与反馈）
- Nav2 自主导航（第二版）
- 电流/故障寄存器读取（除非确认 MC602 暴露）
- 通用任意 XYZ 机械臂（无 y 平移轴；必须靠底盘联合，属第二版 MobileManipulatorController）
