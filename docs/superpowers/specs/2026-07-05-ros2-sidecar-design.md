# ROS2 Sidecar 架构设计 (2026-07-05)

> 状态: 设计稿 (Draft)
> 作者: 架构组
> 关联 ADR: [ADR-001](../adr/ADR-001-ros-noetic-integration.md), [ADR-003](../adr/ADR-003-ros2-sidecar-integration.md), [branch-strategy](../contributing/branch-strategy.md), [migration-plan](../migration/jetpack6-ros2-humble.md)

---

## 概述

本文定义 `vehicle_wbt` 在保留现有 ZMQ + 串口 + 直接函数调用架构的前提下，叠加一个**独立 ROS2 sidecar 节点**的完整设计。sidecar 是只读观察者与命令路由器 (observe + route)，不取代现有控制路径；通过 DDS 在同一 Jetson 上运行，也允许开发者笔记本、远程调试机参与订阅。系统需满足三条底线：(a) 不修改现有运动学、推理、机械臂、任务调度任一核心文件 (见 CLAUDE.md "DO_NOT_MODIFY")；(b) ENABLE_ROS2=0 时 sidecar 完全停摆，主程序行为字节级一致；(c) 仿真回路可在无硬件笔记本上独立运行，便于 4-6 人并行开发。

---

## 背景与约束

| 维度 | 现实约束 |
|------|---------|
| 团队规模 | 4-6 人并行工作；ROS2 技能有但偏 ROS Noetic 经验 |
| 硬件 | 单台 NVIDIA Jetson (L4T R35.3.1, JetPack 5.x, Ubuntu 20.04) |
| 控制器 | MC601 @ 380400 baud / MC602 @ 1000000 baud / MC602 无线 @ 115200 baud，通过 CH340 USB 转串口 |
| 编码器 | MC601: velocity × time 积分；MC602: 真实硬件编码器 |
| 竞赛窗口 | 2026-08-10 至 2026-08-12 (3 天)，距今 ~36 天 |
| 刷机 | 计划中 — 刷机后必须能用 ROS2 sidecar (`/opt/ros/humble` LTS) |
| 当前 ROS 状态 | Noetic 预装 (零代码引用)，ROS2 未装 (colcon 工具有) |
| 不可破坏 | systemd `py_boot.service`、`main/qqq.py` 启动目标、MC601/MC602 双协议派发、ZMQ REP 端口 5001-5004、Camera daemon 线程、`sys.path.append` 约定、`import vehicle` 硬件副作用、global 状态 (`serial_wrap` / `ctl_id` / `serial_mc601` / `serial_mc602` / `encoder_motor_all_sim1`) |
| 不可新增 | 裸 `except:`、`while True: time.sleep(1)` 代替错误处理、硬编码密钥、`eval(chassis_type)`、`eval()` 解析 LLM 输出 |
| 协作模式 | 单 Jetson + 开发者笔记本并行；VPN 偶尔；4 个独立 sidecar 包可分配给 4 人 |

---

## 架构

sidecar 不在主进程中嵌入，而是由 `main/qqq.py` 在 `ENABLE_ROS2=1` 时 fork 一个独立 ROS2 Python 进程。两者通过 DDS 通信 + 文件 (共享内存 / 命名管道) 共享感知快照，无侵入式耦合。

### 完整拓扑

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                            NVIDIA Jetson (L4T R35.3.1)                        │
│                                                                                │
│  ┌────────────────────────────────────────┐    ┌────────────────────────┐    │
│  │  Main Process (main/qqq.py)            │    │  ROS2 Sidecar          │    │
│  │  ──────────────────────────            │    │  (subprocess.Popen)    │    │
│  │  MyCar (car_wrap.py, 1438 行)          │    │                        │    │
│  │    ├── ClintInterface ─┐               │    │  ros2_ws/src/          │    │
│  │    ├── CarBase         │ ZMQ (loopback │    │  ├── camera_node       │    │
│  │    ├── ArmBase         │   127.0.0.1)  │    │  ├── car_status_node   │    │
│  │    └── MyTask          │               │    │  ├── inference_bridge  │    │
│  │         │              │               │    │  ├── tf_static_pub     │    │
│  │         ▼              ▼               │    │  ├── safety_gate       │    │
│  │  Camera (daemon thread)                │    │  └── mock_node (sim)   │    │
│  │  ──────────────────────                │    │         │              │    │
│  │  ┌────────────────────────────────┐    │    │         ▼              │    │
│  │  │ SidecarAdapter (singleton)     │──publish-shared-fd──┐          │    │
│  │  │  - ringbuffer frame stream     │    │    │  ┌─────────────────────┐│    │
│  │  │  - hooks into CarBase events   │    │    │  │ ROS2 DDS Domain     ││    │
│  │  │  - emits on every Ctrl/Vel     │    │    │  │ (cyclonedds, local) ││    │
│  │  └────────────────────────────────┘    │    │  └─────────────────────┘│    │
│  └────────────────────────────────────────┘    └────────────────────────┘    │
│           │                                          │                         │
│           ▼                                          ▼                         │
│  ┌─────────────────────┐                  ┌─────────────────────┐             │
│  │ MC601/MC602 (CH340) │                  │ /dev/cam* (V4L2)    │             │
│  │ 串口 (USB)           │                  │ 摄像头 (USB)         │             │
│  └─────────────────────┘                  └─────────────────────┘             │
│                                                                                │
│  ┌─────────────────────────────────────────────────────────────────┐         │
│  │  InferServer (subprocess, 4 ZMQ REP)                            │         │
│  │  lane:5001 / task:5002 / front:5003 / ocr:5004                  │         │
│  └─────────────────────────────────────────────────────────────────┘         │
└──────────────────────────────────────────────────────────────────────────────┘
           │                                                  │
           │ ROS2 DDS multicast (UDP 7400-7500)               │
           ▼                                                  ▼
┌────────────────────┐                          ┌────────────────────┐
│ Developer Laptop 1 │                          │ Developer Laptop 2 │
│  - RViz2           │                          │  - rqt_graph       │
│  - ros2 bag record │                          │  - ros2 topic echo │
│  - ros2 topic hz   │                          │  - teleop_twist    │
└────────────────────┘                          └────────────────────┘
           │                                                  │
           │ (可选) ROS_DOMAIN_ID=42 + VPN                     │
           ▼                                                  ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                    Gazebo Sim (ros_gz + ros2_control)                          │
│  ─────────────────────────────────────────────────────────────────             │
│  仿真回路 (开发笔记本, 无硬件):                                                  │
│  ┌──────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐        │
│  │ mock_node│──▶│ camera_node  │──▶│ car_status   │──▶│ safety_gate  │        │
│  │ (URDF +  │   │ (合成图像)    │   │ (里程计合成)  │   │ (逻辑测试)    │        │
│  │  ros2_   │   └──────────────┘   └──────────────┘   └──────────────┘        │
│  │ control) │                                                                │
│  └──────────┘                                                                │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 数据方向铁律

| 来源 | 去向 | 通道 | 频率 |
|------|------|------|------|
| `Camera.frame` (主进程) | ROS2 Image topic | shared memory fd | 30 Hz |
| `ClintInterface` (主进程) | ROS2 Detection topic | shared memory fd | 30 Hz |
| `CarBase.get_pose()` (主进程) | ROS2 Odometry + TF | shared memory fd | 50 Hz |
| ROS2 `/safety/mode_cmd` (外部) | 仅由 safety_gate 接收 | ROS2 DDS | 10 Hz |
| ROS2 `/cmd_vel_safe` (外部) | **不直接驱动电机** — 仅作 simulator feed | ROS2 DDS | 20 Hz |
| `/diagnostics` (sidecar) | 主进程读 | shared memory fd | 1 Hz |

---

## Topic Schema

所有 topic 使用 `vehicle_wbt/` 命名空间前缀，便于多机器人/多 domain 隔离。

### 1. Sensors (主进程 → sidecar → 外部)

| Topic 名 | 类型 | 频率 | QoS | 用途 |
|---------|------|:----:|-----|------|
| `/vehicle_wbt/camera/front/image_raw` | `sensor_msgs/Image` | 10 Hz | BEST_EFFORT, depth=2 | 前向 V4L2 摄像头（与下游 10 Hz 推理匹配） |
| `/vehicle_wbt/camera/side/image_raw` | `sensor_msgs/Image` | 10 Hz | BEST_EFFORT, depth=2 | 侧向摄像头 (可选) |
| `/vehicle_wbt/imu/data` | `sensor_msgs/Imu` | 100 Hz | BEST_EFFORT | 9-DOF IMU (若装备) |
| `/vehicle_wbt/ir/front` | `std_msgs/Int32MultiArray` | 20 Hz | RELIABLE | 红外测距（P7 右侧 + P8 左侧，见 hardware-port-mapping.md） |
| `/vehicle_wbt/encoders/wheel` | `vehicle_wbt_msgs/WheelEncoders` | 50 Hz | BEST_EFFORT | M1-M6 编码器 |

### 2. Perception (推理结果)

| Topic 名 | 类型 | 频率 | 用途 |
|---------|------|:----:|------|
| `/vehicle_wbt/perception/lane` | `vehicle_wbt_msgs/LaneResult` | 20 Hz | 车道线分割结果 |
| `/vehicle_wbt/perception/detections/task` | `vehicle_wbt_msgs/DetectionArray` | 10 Hz | task_wbt2025 模型输出 |
| `/vehicle_wbt/perception/detections/front` | `vehicle_wbt_msgs/DetectionArray` | 10 Hz | front_model2 输出 |
| `/vehicle_wbt/perception/ocr` | `vehicle_wbt_msgs/OcrResult` | 5 Hz (按需) | OCR 文本识别 |
| `/vehicle_wbt/perception/tracks` | `vehicle_wbt_msgs/TrackArray` | 20 Hz | MOT 跟踪 ID |

### 3. State (主进程 → sidecar)

| Topic 名 | 类型 | 频率 | 用途 |
|---------|------|:----:|------|
| `/vehicle_wbt/odom` | `nav_msgs/Odometry` | 50 Hz | 里程计 (base_link → odom) |
| `/vehicle_wbt/tf` | `tf2_msgs/TFMessage` | 50 Hz | 动态 TF (base_link → camera/arm_link) |
| `/vehicle_wbt/tf_static` | `tf2_msgs/TFMessage` | latched | 静态 TF (底盘尺寸 → URDF) |
| `/vehicle_wbt/joint_states` | `sensor_msgs/JointState` | 50 Hz | 机械臂 2 步进 + 2 舵机 + 真空 + 阀（不是 6 关节） |
| `/vehicle_wbt/arm/vacuum` | `std_msgs/Bool` | 10 Hz | 真空泵状态 |
| `/vehicle_wbt/battery` | `sensor_msgs/BatteryState` | 1 Hz | 电池电量 |
| `/vehicle_wbt/system/ctl_id` | `std_msgs/Int32` | latched | 当前 ctl_id (0=MC601, 1=MC602) |

### 4. Task / Control Signals (sidecar 内部 + 部分外部)

| Topic 名 | 类型 | 频率 | 方向 | 用途 |
|---------|------|:----:|:----:|------|
| `/vehicle_wbt/task/state` | `vehicle_wbt_msgs/TaskState` | 10 Hz | 主→sidecar | 当前任务 FSM 状态 |
| `/vehicle_wbt/task/event` | `vehicle_wbt_msgs/TaskEvent` | event | 主→sidecar | 关键事件流 (拾起/放下/失败) |
| `/vehicle_wbt/safety/mode_cmd` | `std_msgs/String` | 10 Hz | 外部→safety_gate | AUTO/MANUAL/ESTOP/SIM |
| `/vehicle_wbt/safety/heartbeat` | `std_msgs/Int32` | 5 Hz | safety_gate→主 | 5s 心跳 (失联自动 E-stop) |
| `/vehicle_wbt/cmd_vel_safe` | `geometry_msgs/Twist` | 20 Hz | 外部→sidecar | 仅 simulator 用，**不驱动真机** |

### 5. Diagnostics

| Topic 名 | 类型 | 频率 | 用途 |
|---------|------|:----:|------|
| `/vehicle_wbt/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | 串口/ZMQ/摄像头健康 |
| `/vehicle_wbt/diagnostics/serial` | `diagnostic_msgs/DiagnosticStatus` | 1 Hz | serial_wrap 健康 |
| `/vehicle_wbt/diagnostics/infer` | `diagnostic_msgs/DiagnosticStatus` | 1 Hz | 4 端口 REP 健康 |

### 命名约定

- 所有 topic 以 `/vehicle_wbt/` 前缀，与 ROS_DOMAIN_ID=42 配合
- 摄像头 `/camera/<position>/<type>` 子命名空间
- 任务事件以 `task/<verb>` 子命名空间
- 安全相关一律 `safety/`，避免与控制命令混淆

---

## 自定义消息

文件位于 `ros2_ws/src/vehicle_wbt_msgs/msg/`。

### `LaneResult.msg`

```
# 车道线分割结果 (来自 infer_cs port 5001)
std_msgs/Header header

# 左右车道线像素坐标 (u, v)
int32[20] left_line_u
int32[20] left_line_v
int32[20] right_line_u
int32[20] right_line_v

# 拟合参数
float32 left_slope
float32 left_intercept
float32 right_slope
float32 right_intercept

# 中心偏移 (相对 base_link)
float32 center_offset_m
float32 heading_error_rad

# 置信度 0.0-1.0
float32 confidence

# 推理耗时 (毫秒)
float32 infer_ms
```

### `DetectionArray.msg`

```
std_msgs/Header header
Detection[] detections

================================================================================
Detection.msg
================================================================================
string class_name            # task_wbt2025/front_model2 类别名
int32 class_id
float32 score                # 置信度
float32[4] bbox              # [x1, y1, x2, y2]
float32[3] center_3d         # 相机坐标系 (X, Y, Z) 米
int32 track_id               # MOT ID (-1 = 无跟踪)
```

### `TaskEvent.msg`

```
std_msgs/Header header

# FSM 事件类型 (强类型枚举)
uint8 EVENT_NONE = 0
uint8 EVENT_TASK_START = 1
uint8 EVENT_ARM_PICKUP_DONE = 2
uint8 EVENT_ARM_DROPOFF_DONE = 3
uint8 EVENT_EJECTION_DONE = 4
uint8 EVENT_TARGET_FOUND = 5
uint8 EVENT_TARGET_LOST = 6
uint8 EVENT_NAVIGATION_FAILED = 7
uint8 EVENT_LANE_LOST = 8
uint8 EVENT_SAFETY_ESTOP = 9
uint8 EVENT_RECOVERY_START = 10

uint8 event_type
string task_name             # "food_sorting", "hanoi_tower" 等
string detail                # 自由文本 (失败原因等)
float32 confidence
```

### `ArmState.msg`

```
std_msgs/Header header

# 6 关节角度 (弧度) — 来自 ArmBase.stepper/servo
float32[6] joint_positions
float32[6] joint_velocities
float32[6] joint_efforts

# 真空泵 / 夹爪
bool vacuum_on
bool gripper_closed

# 任务进度
int8 task_phase              # 0=idle, 1=moving, 2=picking, 3=placed, 4=failed
string current_target        # 物体类别名

# 步进电机限位状态
bool[3] stepper_limit_hit    # stepper_1, stepper_3, 备用
```

### `WheelEncoders.msg`

```
std_msgs/Header header

# M1-M6 编码器原始计数 (注意 MC601 为仿真值)
int32[6] raw_count
float32[6] velocity_rpm
uint8 ctl_id                 # 当前 ctl_id (0=MC601, 1=MC602)
bool encoder_is_simulated    # 重要 — MC601 总是 true
```

### `OcrResult.msg`

```
std_msgs/Header header
string text
float32 score
float32[4] bbox
```

### `TaskState.msg`

```
std_msgs/Header header

# FSM 状态枚举
uint8 STATE_IDLE = 0
uint8 STATE_NAVIGATING = 1
uint8 STATE_DETECTING = 2
uint8 STATE_PICKING = 3
uint8 STATE_PLACING = 4
uint8 STATE_EJECTING = 5
uint8 STATE_RECOVERING = 6
uint8 STATE_ESTOP = 7

uint8 state
string active_task
float32 progress             # 0.0-1.0
string last_error
```

### `TrackArray.msg`

```
std_msgs/Header header
Track[] tracks

================================================================================
Track.msg
================================================================================
int32 track_id
string class_name
float32[8] bbox_history     # 最近 8 帧 bbox [x1,y1,x2,y2]
float32[3] velocity_3d
int32 age                    # 跟踪帧数
bool lost
```

---

## 安全设计

sidecar **绝不**直接驱动电机，所有 `/cmd_vel*` 在 safety_gate 中经过 4 层闸门。安全模式状态机是独立模块，可单测。

### 4 层安全闸门

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 0: 物理急停                                                            │
│  ─────────────                                                               │
│  Hardware button3 (Key4Btn.BUTTON3) 拉低 → 主进程轮询线程检测                  │
│  → MyCar.set_speed(0,0,0,0) 直接走 Motors.set_speed(MC601/MC602)              │
│  → 不经过 ROS、不经过 safety_gate、不经过 ArmBase                              │
│  → 优先级最高 (CLAUDE.md: "never replace error conditions with sleep")        │
└─────────────────────────────────────────────────────────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 1: 心跳闸门 (safety_gate)                                              │
│  ──────────────────                                                          │
│  safety_gate 每 1s publish /safety/heartbeat                                 │
│  主进程 (SidecarAdapter) 5s 未收到 → 触发 E-stop callback                    │
│  E-stop 动作: car.stop() + arm.emergency_release() + 通知 main/qqq.py        │
│  同时 publish /vehicle_wbt/safety/estop 事件 (供其他 sidecar 节点知晓)        │
└─────────────────────────────────────────────────────────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 2: 模式闸门                                                            │
│  ────────────                                                                │
│  收到 /safety/mode_cmd 必须为枚举:                                            │
│    "AUTO"     → 允许主进程自主决策                                            │
│    "MANUAL"   → 主进程必须收到 /cmd_vel_safe 才能移动 (但仍写真机)              │
│    "SIM"      → /cmd_vel_safe 仅驱动 Gazebo mock，**真机电机必须为 0**        │
│    "ESTOP"    → 不论其他输入，硬停                                            │
│  状态机非法转移 → reject + log error                                         │
└─────────────────────────────────────────────────────────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 3: 数据完整性闸门                                                       │
│  ────────────────────                                                        │
│  检查上游 topic 质量:                                                          │
│    /odom 时间戳 > 0.5s 旧 → 拒绝 /cmd_vel_safe                                │
│    /perception/lane.confidence < 0.3 → 拒绝 lane-follow                       │
│    /diagnostics 任何 ERROR → 拒绝所有自主动作                                  │
│  不通过则自动降级到 IDLE                                                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Mode 状态机

```
                       ┌─────────────┐
        ┌─────────────►│    IDLE     │◄─────────────┐
        │              └──────┬──────┘              │
   mode_cmd=ESTOP             │ mode_cmd=AUTO       │ mode_cmd=ESTOP
   (任意状态)                  ▼                     │
                       ┌─────────────┐              │
                       │    AUTO     │              │
                       └──────┬──────┘              │
                              │ mode_cmd=SIM        │ mode_cmd=MANUAL
                              ▼                     ▼
                       ┌─────────────┐       ┌─────────────┐
                       │    SIM      │       │   MANUAL    │
                       └─────────────┘       └─────────────┘
                              │                     │
                              └─────────┬───────────┘
                                        ▼
                                 mode_cmd=ESTOP
                                        │
                                        ▼
                                publish /safety/estop
                                主进程收到 → E-stop
```

### Deadman Switch

```
┌───────────────────────────────────────────────────────────────┐
│  远程手动控制 (/cmd_vel_safe) 必须每 200ms 重新发送               │
│  ──────────────────────────────────────────────────             │
│  safety_gate 维护: last_cmd_vel_ts                              │
│  每 250ms 检查: now - last_cmd_vel_ts > 250ms                  │
│  超时 → 自动停 0 速度 + 通知主进程                              │
│  这是 `/cmd_vel_safe` 在 MANUAL/SIM 下的硬要求                   │
└───────────────────────────────────────────────────────────────┘
```

### Button3 优先级

- `Key4Btn.BUTTON3` 是物理急停，独立于 ROS
- 主进程中 Key4Btn 后台线程检测到 BUTTON3=0 → 直接调 `Motors.set_speed(0)` 绕过一切
- sidecar 通过 `/vehicle_wbt/safety/estop` 收到事件，仅作日志/可视化
- **绝不允许 ROS 关闭时 BUTTON3 失效** — 测试用例必须包含：sidecar 进程 kill -9 时 BUTTON3 仍能停机

---

## 仿真回路

采用 A+C 组合策略：开发/调试用 mock + rosbag2 (轻量、纯 Python)；最终验证用 Gazebo + ros2_control (含物理、可跑 motion)。

### A. Mock + rosbag2 (笔记本, 95% 调试)

适用：单测、集成测试、CI、4 人并行开发。

```
┌───────────────────────────────────────────────────────────────┐
│  仿真包: vehicle_wbt_sim_mock                                  │
│  ──────────────────────────────                                │
│  nodes/                                                        │
│    ├── mock_camera_node.py      # 读视频文件 / 合成图           │
│    ├── mock_serial_node.py      # 合成 MC602 编码器 / odometry  │
│    ├── mock_inference_node.py   # stub 返回固定 bbox           │
│    └── replay_node.py           # rosbag2 play 时钟驱动         │
│                                                                │
│  启动:                                                         │
│    ros2 launch vehicle_wbt_sim_mock mock_session.launch.py     │
│      video_path:=/path/to/recorded.mp4                         │
│      bag_path:=/path/to/recorded.db3                           │
└───────────────────────────────────────────────────────────────┘
```

#### mock_node.py 骨架

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

class MockSerialNode(Node):
    """合成 MC602 真实编码器 + chassis odometry，模拟 Jetson 真机行为"""

    def __init__(self):
        super().__init__('mock_serial_node')
        self.declare_parameter('ctl_id', 1)  # 默认模拟 MC602
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('update_rate_hz', 50.0)

        self._odom_pub = self.create_publisher(Odometry, '/vehicle_wbt/odom', 10)
        self._joint_pub = self.create_publisher(JointState, '/vehicle_wbt/joint_states', 10)

        self._bridge = CvBridge()
        self._video_cap = None  # 由 launch 注入视频文件
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/vehicle_wbt/cmd_vel_safe', self._on_cmd_vel, 10
        )

        period = 1.0 / self.get_parameter('update_rate_hz').value
        self._timer = self.create_timer(period, self._tick)

    def _on_cmd_vel(self, msg: Twist) -> None:
        """累计虚拟速度，不直接驱动任何硬件"""
        self._vx = msg.linear.x
        self._wz = msg.angular.z

    def _tick(self) -> None:
        """按真实周期发布 odometry, 严格模拟 vehicle_base.py 公式"""
        # 此处调用 vehicle_base.py 的 Mecanum 运动学 (不修改原文件, 仅 import)
        from vehicle.driver.vehicle_base import forward_kinematics
        vx, vy, wz = forward_kinematics(self._m1, self._m2, self._m3, self._m4)
        # publish odom ...

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MockSerialNode())
    rclpy.shutdown()
```

### C. Gazebo Sim + ros2_control (桌面级验证, 赛前最终)

适用：完整物理仿真，验证 motion planning、碰撞、传感噪声。

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  仿真包: vehicle_wbt_gz                                                          │
│  ────────────────────                                                            │
│  worlds/                                                                          │
│    ├── competition_track.sdf       # 复刻竞赛场地                              │
│    └── lab_test.sdf                # 实验室测试                                │
│  urdf/                                                                            │
│    ├── vehicle_wbt.urdf.xacro      # 从 hardware-port-mapping.md 自动生成       │
│    └── chassis.urdf.xacro                                                     │
│  controllers/                                                                    │
│    ├── chassis_controller.yaml     # ros2_control diff_drive_controller          │
│    └── arm_controller.yaml         # ros2_control position_controllers/JointTrajectoryController  │
│                                                                                  │
│  launch/                                                                          │
│    ├── gz_sim.launch.py            # 启 Gazebo + ros_gz_bridge                  │
│    ├── spawn_robot.launch.py       # spawn URDF 到 Gazebo                        │
│    └── bringup_sim.launch.py       # 一键启动全套 (gz + robot + controllers)      │
└──────────────────────────────────────────────────────────────────────────────┘
```

依赖包: `ros_gz`, `ros2_control`, `controller_manager`, `gz_ros2_control_plugins`, `xacro`, `joint_state_publisher_gui`。

#### URDF 来源

```
hardware-port-mapping.md (权威)
            │
            ▼
scripts/urdf_gen.py (从 md 表格生成 xacro)
            │
            ▼
urdf/vehicle_wbt.urdf.xacro (机器可读)
            │
            ▼
Gazebo / RViz
```

#### 为什么 URDF 用 md 而非 YAML？

`hardware-port-mapping.md` 是团队当前唯一的硬件约定源，已经按关节/电机/限位分组。脚本读取 markdown 表格生成 xacro，避免配置漂移。

### sim↔real 切换

```
ENABLE_ROS2=1 ENABLE_SIM=1   → /cmd_vel_safe → Gazebo chassis
ENABLE_ROS2=1 ENABLE_SIM=0   → /cmd_vel_safe → 主进程不订阅, 仅 RViz 可视化
ENABLE_ROS2=0                → sidecar 不启动, 主程序保持原样
```

`safety_gate` 在 `mode_cmd == "SIM"` 时自动拒绝一切非 Gazebo 节点的 `/cmd_vel_safe` 注入。

---

## 远程接入

### DDS 配置

| 场景 | 部署 | ROS_DOMAIN_ID | DDS 实现 | RMW |
|------|------|:---:|---------|-----|
| 单 Jetson 本机 | Jetson 单独跑 sidecar | 42 | cyclonedds | rmw_cyclonedds_cpp |
| 同 LAN 多端 | Jetson + 开发笔记本 | 42 | cyclonedds | rmw_cyclonedds_cpp |
| VPN 跨网 | Jetson + 异地开发机 | 42 | cyclonedds | rmw_cyclonedds_cpp |
| Gazebo 离线 | 开发笔记本独立运行 | 43 (避开主 domain) | fastrtps | rmw_fastrtps_cpp |

#### cyclonedds 配置 (`/etc/cyclonedds.xml`)

```xml
<?xml version="1.0"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="42">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Internal>
      <SocketReceiveBufferSize>10MB</SocketReceiveBufferSize>
    </Internal>
  </Domain>
</CycloneDDS>
```

### 网络拓扑

```
LAN 场景 (ROS_DOMAIN_ID=42):
  Jetson   <─── UDP multicast 7400-7500 ───>  Developer Laptop
  Jetson   <─── UDP multicast 7400-7500 ───>  Test Bench Laptop
  全部 cyclonedds, 无需 master

VPN 场景 (WireGuard 或 Tailscale):
  Jetson (10.0.0.1) <── WireGuard tunnel ──> 异地 Laptop (10.0.0.2)
  调 cyclonedds <NetworkInterface> 绑定 wg0 接口
  带宽: 摄像头流受限 (BEST_EFFORT + depth=2 已限流)

离线 Gazebo:
  笔记本单跑, ROS_DOMAIN_ID=43 避免与真机冲突
```

### 防火墙端口

| 端口 | 协议 | 用途 |
|------|------|------|
| 7400-7500 | UDP multicast | DDS 发现 + 数据 |
| 5001-5004 | TCP | ZMQ 推理 (本地, 不暴露) |
| 22 | TCP | SSH 调 Jetson |

---

## 生命周期

sidecar 由 `main/qqq.py` 在启动时通过 `subprocess.Popen` fork，独立 ROS2 Python 进程 (`ros2_ws/src/` 下的可执行节点)。**主进程任何崩溃不应影响 sidecar 反之亦然**。

### 启动顺序

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  systemd 启动 main/qqq.py                                                     │
│  ──────────────────────────                                                   │
│  1. main/qqq.py 启动                                                          │
│     ├── 加载 config_car.yml                                                    │
│     ├── import vehicle.* (触发串口扫描, ctl_id 探测)                          │
│     ├── 启动 InferServer (subprocess, 4 端口)                                 │
│     ├── 启动 Camera daemon 线程                                                │
│     └── 实例化 MyCar                                                            │
│                                                                                │
│  2. if ENABLE_ROS2 == "1":                                                     │
│     ├── spawn ros2_daemon: ros2 run vehicle_wbt_bringup sidecar_main          │
│     │     ├── wait for /vehicle_wbt/system/ctl_id (latched)                    │
│     │     ├── wait for /vehicle_wbt/odom first msg                             │
│     │     └── ready=publish /safety/heartbeat                                  │
│     │                                                                          │
│     └── 主进程 SidecarAdapter 启动 publish 线程                                │
│            ├── Camera.frame → /camera/front/image_raw                          │
│            ├── ClintInterface result → /perception/detections/*                │
│            ├── CarBase.get_pose() → /odom + /tf                               │
│            └── MyCar.fsm_state → /task/state + /task/event                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 关闭顺序

```
SIGTERM → main/qqq.py
  ├── MyCar.task.stop()
  ├── ArmBase.emergency_release()
  ├── Motors.set_speed(0,0,0,0)
  ├── SidecarAdapter.flush()           # 等待 shared memory 排空
  ├── ros2_daemon.send_signal(SIGINT)  # graceful shutdown
  │     └── sidecar_main 收到 SIGINT
  │           ├── publish /safety/estop (reason=shutdown)
  │           └── rclpy.shutdown()
  └── main/qqq.py 退出码 0
```

### systemd 集成 (`/etc/systemd/system/py_boot.service` 修订)

```ini
[Unit]
Description=vehicle_wbt boot service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/workspace/vehicle_wbt
Environment="ENABLE_ROS2=0"
Environment="ENABLE_SIM=0"
Environment="ROS_DOMAIN_ID=42"
ExecStart=/usr/bin/python3 main/qqq.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

切换 ROS2 时: `sudo systemctl edit py_boot.service` 添加 `Environment="ENABLE_ROS2=1"`，不修改 systemd 主单元文件 (符合 "DO_NOT_MODIFY" 中对 systemd service 的隐含保护)。

### ENABLE_ROS2=0 行为验证

测试用例必须包含:
- `ENABLE_ROS2=0` 启动 main/qqq.py → 行为与 2026-07-04 之前的部署字节级一致
- 不引入 `ros2` 进程 (ps -ef | grep ros2 = empty)
- 不引入 cyclonedds 端口 (netstat | grep 7400 = empty)
- `/vehicle_wbt/*` 所有 topic 无人 publish，无人订阅
- main/qqq.py exit code = 0 与是否启用 ROS2 无关

---

## 实施阶段

| Phase | 周次 | 工作量 | 主要交付 | 负责人 |
|:----:|:---:|:---:|------|:---:|
| 1 | W1 | 3-4 天 | mock 节点 + rosbag2 录放 | 4 人并行 |
| 2 | W2 | 4-5 天 | URDF + Gazebo 仿真 + 消息定义 | 2 人 |
| 3 | W3 | 4-5 天 | control bridge + safety_gate | 2 人 |
| 4 | W4 | 3-4 天 | 多机验证 + 刷机后部署 | 1 人 |

### Phase 1: Mock + rosbag2 (W1, 3-4 天)

**Day 1-2: 基础设施**
- 安装 ROS2 Humble (`apt install ros-humble-desktop ros-humble-ros2-control ros-humble-ros-gz`)
- 创建 `ros2_ws/src/vehicle_wbt_msgs/` 包，定义 LaneResult / Detection / ArmState / TaskEvent / OcrResult / TrackArray / TaskState / WheelEncoders
- 创建 `ros2_ws/src/vehicle_wbt_bringup/` 包含 launch + config
- 配置 cyclonedds

**Day 3-4: mock 节点**
- `mock_camera_node.py`: 读视频文件 / 合成棋盘格 → publish Image
- `mock_serial_node.py`: 调用 `vehicle_base.forward_kinematics` 合成 odom
- `mock_inference_node.py`: 订阅 Image → publish 固定 DetectionArray
- 主进程 `SidecarAdapter` 在 `ENABLE_ROS2=1` 时启动，publish Camera/CarBase/ClintInterface 数据

**验收**:
- `ros2 topic list` 显示所有 `/vehicle_wbt/*` topic
- `ros2 bag record -a -o test_session.db3` 录制 60s 成功
- `ros2 bag play test_session.db3` 回放 RViz 显示一致

### Phase 2: URDF + Gazebo (W2, 4-5 天)

**Day 1-2: URDF 生成**
- 写 `scripts/urdf_gen.py` 从 `docs/hardware-port-mapping.md` 读取表格 → 输出 xacro
- 编写 `vehicle_wbt.urdf.xacro` (麦轮 4 轮 + 6 关节机械臂 + 2 摄像头 + IMU)
- 编写 `chassis.urdf.xacro` (麦轮运动学参数)

**Day 3-4: Gazebo 集成**
- 写 `competition_track.sdf` (基于竞赛场地图纸)
- 配置 `ros2_control` chassis_controller (diff_drive_controller 或 mecanum_controller)
- 配置 `ros2_control` arm_controller (JointTrajectoryController)
- launch 一键启动 Gazebo + spawn robot + controllers

**Day 5: 仿真回归测试**
- 启动 Gazebo → ros2_control → 验证 tf tree 正确
- 验证 topic 在 Gazebo 模式下正常 publish

**验收**:
- `ros2 launch vehicle_wbt_gz bringup_sim.launch.py` 启动 30s 内 Gazebo 显示机器人
- RViz 显示 base_link → camera_link / arm_link TF 树
- ros2_control controllers list 全部 active

### Phase 3: Control Bridge + safety_gate (W3, 4-5 天)

**Day 1-2: control bridge**
- 订阅 `/cmd_vel_safe` → 调 `MyCar.move()` / `Mecanum.forward(vx, vy, wz)`
- **关键: ENABLE_SIM=0 时此订阅器不创建, 仅作 simulator feed**
- 订阅 `/vehicle_wbt/safety/mode_cmd` → 调主进程模式切换

**Day 3-4: safety_gate**
- 实现 4 层闸门 (物理急停/心跳/模式/数据完整性)
- 实现 Mode 状态机 (IDLE/AUTO/MANUAL/SIM/ESTOP)
- Deadman switch: 250ms 超时自动停

**Day 5: 单元测试**
- pytest 覆盖状态机所有转移
- mock safety_gate 在 mode=SIM 下拒绝外部 /cmd_vel_safe

**验收**:
- `pytest tests/test_safety_gate.py` 100% 通过
- 手工: kill -9 sidecar 进程 → 主进程 5s 内进入 E-stop
- 手工: BUTTON3 按下 → 立即停 (不依赖 ROS)

### Phase 4: 多机验证 + 刷机部署 (W4, 3-4 天)

**Day 1-2: 多机验证**
- 2 台开发笔记本 + Jetson 同时订阅 `/vehicle_wbt/*` 验证 DDS 发现
- 录制 → 回放 → 在另一台笔记本 RViz 看
- 跨网: WireGuard 隧道下 DDS 工作

**Day 3-4: 刷机后部署**
- 编写 `scripts/post_flash_setup.sh` 一键安装 ROS2 + colcon build + 配置 cyclonedds
- 验证 `ENABLE_ROS2=1` 在新系统上工作
- 验证 `ENABLE_ROS2=0` 旧部署完全无影响

**验收**:
- 刷机后 ≤ 30 分钟 sidecar 可用
- ENABLE_ROS2=0/1 切换无 main/qqq.py 修改

---

## 测试策略

### 仿真中测什么

| 测试类型 | 在哪跑 | 工具 | 频率 |
|---------|--------|------|:---:|
| 单元测试 (safety_gate / 状态机) | 笔记本 | pytest | 每次 commit |
| 集成测试 (mock + rosbag2 replay) | 笔记本 | ros2 bag + RViz | PR 前 |
| 物理仿真 (motion / collision) | 笔记本 | Gazebo + ros2_control | 每周 |
| 系统集成 (主进程 + sidecar) | Jetson 真机 | ros2 bag + rqt | W3/W4 |
| 真实机验收 | Jetson 真机 | 手动 + rosbag | 赛前 |

### Sim-to-Real Gap 清单

| 项 | Sim | Real | 风险 |
|------|-----|------|------|
| 摄像头延迟 | 0ms | 30-80ms (V4L2 + USB) | 🟡 中 |
| 编码器噪声 | 无 | MC602 有 1-2 计数抖动 | 🟡 中 |
| 串口往返 | 0ms | 2-5ms | 🟢 低 |
| ArmBase 步进电机 | 物理仿真 | 真实步进 (非线性) | 🔴 高 — 需真机标定 |
| 真空泵响应 | 理想 | 30-50ms 真空建立 | 🟡 中 |
| 红外测距 | 合成 | 模拟量 + 噪声 | 🟢 低 |
| IMU (若装) | 无噪声 | 温漂 + 振动 | 🟡 中 |

**必须真机标定的项**: arm stepper PID (`vehicle/arm/arm_cfg.yaml`)、lane-follow PID (`config_car.yml` lane_pid)。

### 真实机验收清单

W4 末必须全绿:

- [ ] **启动**: `ENABLE_ROS2=1` 启动 main/qqq.py + sidecar，5s 内 sidecar `/safety/heartbeat` 出现
- [ ] **topic 完整**: `ros2 topic list` 包含所有 §Topic Schema 中定义的 topic
- [ ] **/odom 50Hz**: `ros2 topic hz /vehicle_wbt/odom` ≈ 50Hz
- [ ] **/camera 30Hz**: `ros2 topic hz /vehicle_wbt/camera/front/image_raw` ≈ 30Hz
- [ ] **/tf 完整**: `ros2 run tf2_tools view_frames.py` 生成完整 TF 树 (base_link → camera_link, arm_link)
- [ ] **rosbag 录制**: `ros2 bag record -a -o test.db3` 录制 60s 成功，文件 > 100MB
- [ ] **rosbag 回放**: `ros2 bag play test.db3` 在 RViz 显示一致
- [ ] **物理急停**: 按 BUTTON3 → 0.2s 内车速归零 (不依赖 ROS)
- [ ] **sidecar 杀死**: `kill -9 $(pgrep -f vehicle_wbt_bringup)` → 5s 内主进程进入 E-stop
- [ ] **E-stop 恢复**: safety_gate 重启 → 主进程自动恢复 AUTO (按设计) 或人工 ACK (由 mode_cmd 决定)
- [ ] **mode 切换**: `/safety/mode_cmd: "ESTOP"` → 立即停；`"AUTO"` → 恢复
- [ ] **deadman**: 远程 teleop 停发 → 250ms 内车速归零
- [ ] **ENABLE_ROS2=0**: `systemctl edit py_boot.service` 改 ENABLE_ROS2=0 → main/qqq.py 启动后无 ROS 进程、无 DDS 端口、行为字节级一致
- [ ] **CLI**: `ros2 run vehicle_wbt_bringup sidecar_main` 单跑也能启动 (便于调试)
- [ ] **ros2 doctor**: `ros2 doctor --report` 报告无 WARNING/ERROR
- [ ] **刷机后**: 全清系统后 30 分钟内 sidecar 可用

---

## 风险与缓解

| # | 风险 | 概率 | 影响 | 缓解措施 |
|---|------|:---:|:---:|---------|
| 1 | 主进程引入 shared memory 拖累实时性 | 中 | 🟡 | 共享内存用 ringbuffer + lock-free SPSC; publish 线程 priority=NORMAL |
| 2 | ROS2 Python 进程崩溃导致主进程感知失明 | 中 | 🟡 | safety_gate 5s 心跳闸门; 主进程 E-stop on heartbeat lost |
| 3 | cyclonedds 在 VPN 高延迟下丢帧 | 中 | 🟡 | BEST_EFFORT QoS + depth=2; 不用于关键控制 |
| 4 | Gazebo 在 Jetson 真机跑不动 (GPU/CPU 不足) | 高 | 🟡 | Gazebo 仅在开发笔记本跑; Jetson 永远用真机模式 |
| 5 | ENABLE_ROS2=1 引入新代码路径引入 race condition | 中 | 🔴 | 必须 ENABLE_ROS2=0/1 两套行为在 CI 中跑同一组测试 |
| 6 | URDF 与 hardware-port-mapping.md 不一致 | 中 | 🟡 | urdf_gen.py 是 md → xacro 单向; md 是 source of truth, lint check in CI |
| 7 | 团队不熟悉 ROS2 ament_python | 中 | 🟡 | Phase 1 优先做 1 个最小节点跑通; 配 PR 模板 |
| 8 | ros2_control 与现有 ArmBase 步进电机协议冲突 | 中 | 🔴 | Gazebo 模式只用 ros2_control; 真机模式只用 ArmBase, 互斥 |
| 9 | BUTTON3 与 ROS 紧耦合 (CLAUDE.md 明确禁止) | 低 | 🔴 | BUTTON3 走 Key4Btn 独立线程, ROS 完全旁路; 测试用例强制验证 |
| 10 | 刷机后 ROS2 装错版本 (humble vs iron) | 中 | 🟡 | post_flash_setup.sh 固定 humble; CI 验证 |
| 11 | 4 个节点包并行开发冲突 | 低 | 🟡 | 每个包独立 package.xml, 不共享文件; 集成只在 launch 层 |
| 12 | ZMQ REP 与 sidecar publish 资源竞争 | 低 | 🟡 | 主进程 SidecarAdapter 在独立线程, 不阻塞 motion control loop |
| 13 | DDS 发现在网络切换 (WiFi/LAN) 中断 | 中 | 🟡 | cyclonedds 配置 NetworkInterface autodetermine; 重连后自动恢复 |
| 14 | rosbag2 录制文件过大 (摄像头 + odom) | 中 | 🟡 | image_transport compressed; 录制时启动 jpeg 压缩订阅器 |
| 15 | 竞赛现场不允许带笔记本跑 RViz | — | 🔴 | RViz/Gazebo 仅在开发期用; 比赛当天 ENABLE_ROS2=0 |
| 16 | arm stepper PID 在 sim 中表现完美, real 不行 | 高 | 🔴 | Phase 4 真机标定; sim 仅做 motion 验证 |

---

## 决策记录

### 决策摘要

1. **保留现有架构**: 主进程不重构 (CLAUDE.md "DO_NOT_MODIFY" 列表保持完整)
2. **sidecar 是只读观察者 + 命令路由器**: 不取代 ZMQ/串口/直接调用
3. **ROS2 Humble (LTS)**: 对应 Ubuntu 22.04 (jammy) + JetPack 6.x — Humble 支持到 2027-05
4. **DDS 用 cyclonedds**: 比 fastdds 占用更小, Jetson 友好
5. **仿真 A+C**: mock + rosbag2 (主) + Gazebo + ros2_control (最终)
6. **ROS_DOMAIN_ID=42** 真机 / **43** 仿真, 避免冲突
7. **URDF 从 hardware-port-mapping.md 自动生成**: md 是 source of truth
8. **safety_gate 4 层闸门**: 物理急停 > 心跳 > 模式 > 数据完整性
9. **ENABLE_ROS2=0 必须零影响**: CI 强制验证两套行为一致
10. **BUTTON3 永不经 ROS**: Key4Btn 后台线程独立路径, 测试用例覆盖

### 关联文档

| 文档 | 链接 | 状态 |
|------|------|:---:|
| ADR-001 ROS Noetic 集成方案 | [../adr/ADR-001-ros-noetic-integration.md](../adr/ADR-001-ros-noetic-integration.md) | 提议中 (与本文互补, 本文是其 ROS2 演化方向) |
| ADR-002 Python 环境管理 | [../adr/ADR-002-python-environment.md](../adr/ADR-002-python-environment.md) | 提议中 |
| branch-strategy.md | [../contributing/branch-strategy.md](../contributing/branch-strategy.md) | 已写 |
| jetpack6-ros2-humble.md | [../migration/jetpack6-ros2-humble.md](../migration/jetpack6-ros2-humble.md) | 已写 |
| hardware-port-mapping.md | [../hardware-port-mapping.md](../hardware-port-mapping.md) | **已写 (PR #5 合并)** — URDF 生成的 source of truth |
| ros-analysis.md | [../ros-analysis.md](../ros-analysis.md) | 已写 (本文与其一致的"不迁移 ROS2"立场**相反**: 本文是 **不迁移但 sidecar** 的折中方案, 因为有仿真和远程调试需求) |
| known-issues.md | [../known-issues.md](../known-issues.md) | 必读 (eval/裸 except/sleep 模式严禁重现) |

### 与 ADR-001 的关系

ADR-001 是 "要不要 ROS" 的决策 (答: 渐进式轻量集成 — 方案 C)。**本文是 ADR-001 的细化与扩展**: 从 ROS Noetic 演进到 ROS2 Humble + sidecar 模型, 引入仿真回路与远程调试两个新需求, 不破坏 ADR-001 的核心原则 (渐进式、零侵入、可回滚)。

### 决策触发器 (何时升级到 ROS2 全量)

| 触发条件 | 当前 ADR-001 立场 | 本文立场 |
|---------|:---:|:---:|
| 多机器人协调 | 不需要 | 同左 |
| SLAM / nav2 | 不需要 | 同左 |
| moveit 运动规划 | 不需要 | 同左 |
| **Gazebo 仿真 (赛后调试)** | 不需要 | **需要 → 触发 ROS2** |
| **远程多人调试** | 1-2 周足够 | **sidecar 已满足, 不升级** |
| 多摄像头同步 | 不需要 | sidecar 内 image_transport 解决 |
| 30+ 节点规模 | 当前 ~6 | sidecar 已足够 |

### 必须先补的文档

1. `docs/hardware-port-mapping.md` — URDF 生成的 source of truth, 表格需含:
   - 4 个底盘电机 (M1-M4) + 2 个辅助 (M5-M6)
   - 6 个机械臂关节 (含步进电机 stepper_1, stepper_3)
   - 3 个舵机 (S2, S3, S7)
   - 8 个 IO 口 (P1-P8，部分复用：P2 同时接 LED+真空泵) + 板载 Beep
   - 2 个摄像头位置 (前/侧)
   - 底盘尺寸 (麦轮直径、轴距、轮距)
2. `docs/contributing/branch-strategy.md` — 4 人并行包开发的分支约定
3. `docs/migration/jetpack6-ros2-humble.md` — phase 1→4 任务分解为可抢 issue

---

## 附录 A: 仓库结构 (最终)

```
/home/jetson/workspace/vehicle_wbt/
├── (现有 main/ car_wrap.py infer_cs/ vehicle/ ... 完全不动)
├── ros2_ws/                            # 新建 colcon 工作空间
│   └── src/
│       ├── vehicle_wbt_msgs/           # 自定义消息 (§6)
│       │   ├── msg/
│       │   │   ├── LaneResult.msg
│       │   │   ├── Detection.msg
│       │   │   ├── DetectionArray.msg
│       │   │   ├── TaskEvent.msg
│       │   │   ├── TaskState.msg
│       │   │   ├── ArmState.msg
│       │   │   ├── WheelEncoders.msg
│       │   │   ├── OcrResult.msg
│       │   │   ├── Track.msg
│       │   │   └── TrackArray.msg
│       │   ├── package.xml
│       │   └── setup.py
│       │
│       ├── vehicle_wbt_bringup/        # 主入口 + launch
│       │   ├── vehicle_wbt_bringup/
│       │   │   ├── sidecar_main.py     # 多节点聚合入口
│       │   │   └── __init__.py
│       │   ├── launch/
│       │   │   ├── vehicle_wbt.launch.py        # 真机
│       │   │   └── vehicle_wbt_sim.launch.py    # 仿真
│       │   ├── rviz/
│       │   │   └── vehicle_wbt.rviz
│       │   └── package.xml
│       │
│       ├── vehicle_wbt_camera_node/     # Phase 1
│       │   └── vehicle_wbt_camera_node/
│       │       └── camera_node.py
│       │
│       ├── vehicle_wbt_car_status_node/ # Phase 1
│       │   └── vehicle_wbt_car_status_node/
│       │       ├── car_status_node.py
│       │       └── tf_broadcaster.py
│       │
│       ├── vehicle_wbt_inference_bridge/ # Phase 1
│       │   └── vehicle_wbt_inference_bridge/
│       │       └── inference_bridge_node.py
│       │
│       ├── vehicle_wbt_safety_gate/     # Phase 3
│       │   └── vehicle_wbt_safety_gate/
│       │       ├── safety_gate_node.py
│       │       ├── mode_state_machine.py
│       │       └── deadman.py
│       │
│       ├── vehicle_wbt_control_bridge/  # Phase 3
│       │   └── vehicle_wbt_control_bridge/
│       │       └── control_bridge_node.py
│       │
│       ├── vehicle_wbt_sim_mock/        # Phase 1
│       │   └── vehicle_wbt_sim_mock/
│       │       ├── mock_camera_node.py
│       │       ├── mock_serial_node.py
│       │       └── mock_inference_node.py
│       │
│       └── vehicle_wbt_gz/              # Phase 2
│           ├── vehicle_wbt_gz/
│           ├── worlds/
│           │   ├── competition_track.sdf
│           │   └── lab_test.sdf
│           ├── urdf/
│           │   ├── vehicle_wbt.urdf.xacro
│           │   └── chassis.urdf.xacro
│           ├── controllers/
│           │   ├── chassis_controller.yaml
│           │   └── arm_controller.yaml
│           └── launch/
│               ├── gz_sim.launch.py
│               ├── spawn_robot.launch.py
│               └── bringup_sim.launch.py
│
├── scripts/                            # 新建辅助脚本
│   ├── urdf_gen.py                     # 从 hardware-port-mapping.md 生成 xacro
│   └── post_flash_setup.sh             # 刷机后一键安装 ROS2
│
└── car_wrap.py / main/qqq.py           # **修改一行**:
                                        #   if os.getenv('ENABLE_ROS2') == '1':
                                        #       self._sidecar = SidecarAdapter()
                                        # 其他完全不变
```

---

## 附录 B: SidecarAdapter 主进程适配层 (骨架)

文件: `car_wrap.py` 顶部 patch (DO_NOT_MODIFY 之外)

```python
# 不修改 car_wrap.py 现有类, 仅新增独立模块
# 文件: car_wrap_sidecar_adapter.py (新文件)
import os
import multiprocessing as mp
from multiprocessing import shared_memory

class SidecarAdapter:
    """主进程侧 ROS2 适配层: 不替换任何现有控制路径, 仅旁路 publish 数据"""

    def __init__(self, car: 'MyCar', config: dict):
        if os.getenv('ENABLE_ROS2') != '1':
            self._enabled = False
            return
        self._enabled = True
        self._car = car
        self._shm = self._setup_shared_memory()
        self._ros_proc = mp.Process(
            target=_sidecar_main,
            args=(self._shm.name,),
            daemon=True,
        )
        self._ros_proc.start()
        self._install_hooks()

    def _setup_shared_memory(self) -> shared_memory.SharedMemory:
        # 256MB ringbuffer: 头部 4KB 元数据, 其余图像帧
        size = 256 * 1024 * 1024
        try:
            shm = shared_memory.SharedMemory(
                name='vehicle_wbt_sidecar', create=True, size=size
            )
        except FileExistsError:
            shm = shared_memory.SharedMemory(name='vehicle_wbt_sidecar')
        return shm

    def _install_hooks(self) -> None:
        """仅挂监听器, 不修改任何控制逻辑"""
        # 1. 轮询 Camera.frame (daemon 线程已写入 self.frame)
        # 2. 轮询 CarBase.get_pose() 合成 odometry
        # 3. 监听 ClintInterface 推理结果
        # 4. 监听 MyCar.fsm_state 变化, publish TaskEvent
        # 5. 接收 /safety/heartbeat 超时 → 触发 E-stop callback
        raise NotImplementedError  # Phase 1 实现

    def shutdown(self) -> None:
        if not self._enabled:
            return
        if self._ros_proc.is_alive():
            self._ros_proc.terminate()
            self._ros_proc.join(timeout=5)


def _sidecar_main(shm_name: str) -> None:
    """sidecar 子进程入口 — 启动 ROS2 节点集合"""
    import rclpy
    from vehicle_wbt_bringup.sidecar_main import build_nodes

    rclpy.init()
    nodes = build_nodes(shm_name=shm_name)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()
```

关键不变量: `car_wrap.py` 主体 0 行修改, `MyCar.__init__` 末尾追加一行 `self._sidecar = SidecarAdapter(self, cfg) if os.getenv('ENABLE_ROS2')=='1' else None` (单行)。

---

> 本 spec 是 2026-07-05 启动 ROS2 sidecar 项目的设计基线。任何 phase 启动前的 PR 必须 review 是否仍符合本文 § 主题架构、§ 安全设计、§ 不可修改清单; 若偏离, 先更新本 spec 再写代码。