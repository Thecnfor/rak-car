
# ROS2 Sidecar 架构设计 (2026-07-05)

> 状态: 设计稿 (Draft)
> 作者: 架构组
> 关联 ADR: [ADR-001](../adr/ADR-001-ros-noetic-integration.md), [ADR-003](../adr/ADR-003-ros2-sidecar-integration.md), [branch-strategy](../contributing/branch-strategy.md), [migration-plan](../migration/jetpack6-ros2-humble.md)

> **本文是 vehicle_wbt 平台级 ROS2 sidecar 设计**,而非本次比赛专用。任何未来机器人形态(不同 chassis / 机械臂 / 传感器组合)只需按本文 §Component 模型 / §硬件接口插件 / §Chassis 抽象 / §机械臂抽象 / §Camera 抽象 增加配置和 adapter,**不修改业务代码**。

---

## 设计原则

1. **URDF 是规范** — 机器人的物理形态(连杆、关节、传感器位置、传动比)由 URDF/xacro 描述;业务代码不写死 chassis/arm/sensor 拓扑,所有几何参数从 URDF 读取。
2. **配置驱动** — 增加新硬件 = 在 YAML 注册表中加 1 个条目,不修改任何业务代码。配置文件是 single source of truth。
3. **版本化 topic namespace** — 所有 topic 在 `/vehicle_wbt/v1/` 下;v1 协议升级到 v2 时旧 topic 仍可订阅,过渡期允许 v1/v2 共存。
4. **硬件接口插件化** — MC601 / MC602 / 未来 MCxxx 都是同一个 `BaseControllerHardwareInterface` 协议的不同 adapter。新增控制器 = 写 1 个 adapter 类,sidecar 主体不动。
5. **多实例组件** — 摄像头/机械臂/传感器都支持 N 个实例(YAML 数组),不绑死 1 摄像头或 1 机械臂;实例 ID 是 topic namespace 的一部分。
6. **Chassis-agnostic** — Mecanum / Diff2 / Diff4 / Tricycle / Quadricycle 都是 `BaseChassis` 子类,由 `cfg_vehicle.yaml::chassis_type` 选择;`MyCar` 不感知 chassis 类型。
7. **Reserved namespaces** — 把未用硬件显式 reserve 到 topic namespace 中(`/vehicle_wbt/v1/reserved/`),物理接入时只需打开 `enabled: true` 开关,无需新增代码路径。

---

## 平台边界

### 本平台(v1)支持

- **控制器**: MC601 (380400 baud) / MC602 (1000000 baud) / MC602 无线 (115200 baud) 通过 CH340 USB 转串口
- **Chassis**: 5 种 — Mecanum / Diff2 / Diff4 / Tricycle / Quadricycle(由 `vehicle/driver/cfg_vehicle.yaml::chassis_type` 选择)
- **机械臂**: 单机械臂(v1);2 步进 + 2 舵机 + 真空泵 + 阀作为 v1 默认拓扑;7DOF / 双臂是 v2+ 扩展路径(见 §机械臂抽象)
- **摄像头**: 最多 4 路 USB 摄像头(`/dev/cam1-cam4`),通过 V4L2
- **串口扩展**: CH340 USB hub 可级联多控制器,每个 `/dev/ttyUSB*` 实例化为一个 controller adapter
- **URDF 描述的所有传感器**: IR 测距、超声波(预留)、触摸(预留)、环境光(预留)、IMU(预留,需外挂)
- **DDS 通信**: Jetson 本机 + 开发者笔记本(LAN/VPN) + 离线 Gazebo 仿真

### 本平台(v1)**不**支持

- **Jetson 直驱 GPIO**: v1 所有 IO 经 MC602 P口 中转,Jetson 40-pin GPIO header 完全保留给 v2
- **Jetson 直驱 CSI/MIPI 摄像头**: v1 仅 USB 摄像头,CSI 留 v2(需 Jetson 载板 spec 确认)
- **Jetson 直驱 `/dev/ttyTHS*`**: v1 仅 USB 串口,集成 UART 留 v2(需 Jetson 载板 spec 确认)
- **moveit / nav2 / SLAM**: 不在 sidecar 职责内;sidecar 仅做 observe + route,不替代主进程决策

### 待 Jetson 载板 spec 确认(v2 决策依据)

- CSI/MIPI 通道数(2 / 4 取决于载板)
- `/dev/ttyTHS*` 数量及复用情况
- GPIO 扩展芯片是否在板(如 TXS0108E 之类)
- PCIe M.2 槽位是否预留(可用于 SSD 录 rosbag)

**强制**: 上述 4 项**不硬编码**,运行时通过 `sysfs` + `udev` 检测,缺失时 sidecar 自动降级并不报错。

---

## 未用硬件清单 (Reserved Hardware Inventory)

### MC602 控制器(物理已部署,v1 不启用)

| 端口/类 | 类型 | 当前状态 | reserved for |
|---------|------|:--------:|--------------|
| S1 | 舵机 | 未接线 | 末端执行器(夹爪旋转 / 腕部 yaw) |
| S4 | 舵机 | 未接线 | 摄像头云台 / 第二夹爪 |
| S5 | 舵机 | 未接线 | 备用 |
| S6 | 舵机 | 未接线 | 备用 |
| P5 | 数字 IO | 未接线 | 超声波触发(配合 P5 trigger / P6 echo) |
| Stepper 2 | 步进电机 | 未接线 | 第二垂直轴(若双臂) / 旋转台 |
| `BluetoothPad` (dev_id 0x09) | 蓝牙手柄 | 类已定义未实例化 | 远程手动控制(mode=MANUAL 时使用) |
| `BoardKey` (dev_id 0x0D) | 板载按键 | 类已定义未实例化 | 物理按键扩展(模式切换 / 标定触发) |
| `NixieTube` (dev_id 0x0F) | 数码管 | 类已定义未实例化 | 状态显示(电量 / 当前 mode / 任务进度) |
| `Battry` (dev_id 0x0C) | 电池管理 | 类已定义未实例化 | 真实电量上报(目前用占位) |
| `ScreenShow` (dev_id 0x0B) | 屏幕 | 类已定义仅 `test()` | HRI 屏幕显示(主任务显示分数 / 提示) |
| dev_id 0x07 mode 2 | Touch | 协议支持未接 | 机械臂末端触觉 / 防撞 |
| dev_id 0x07 mode 3 | Ultrasonic | 协议支持未接 | 前向避障(冗余于 IR) |
| dev_id 0x07 mode 4 | Ambient light | 协议支持未接 | 自动灯光 / 亮度反馈 |

### Jetson Orin NX(板载资源,v1 不使用)

| 资源 | 数量 | 当前占用 | reserved for |
|------|:---:|:--------:|--------------|
| USB-A 端口 | 4 | 2 (CH340 + 2 USB cam) | 2 备用(可接 USB hub / 外接 SSD / 4G 模块) |
| USB 摄像头槽 | 4 (`/dev/cam1-4`) | 2 (cam1=side, cam2=front) | 2 备用(cam3, cam4,需 `/dev/cam3` `enabled: true`) |
| 40-pin GPIO header | 整片 | 0 | 留 v2(v1 走 MC602 P口) |
| HDMI/DP | 1 | 1 (HRI) | 0 备用 |
| CSI/MIPI | 待 spec 确认 | 0 | v2 启用(待载板 spec) |
| `/dev/ttyTHS*` | 待 spec 确认 | 0 | v2 启用(待载板 spec) |

### 启用流程

任何 reserved 资源启用只需 3 步(不改业务代码):

1. **物理接线** — 接到对应端口
2. **YAML 加条目** — 在 `config_sensors.yml` 写 1 行,设 `enabled: true`
3. **URDF 加 link** — 在 `urdf/vehicle_wbt.urdf.xacro` 加 `<link>` + `<joint>`,描述物理位置

sidecar 启动时扫描配置,自动 spawn 对应 ROS2 node,**无需重写 sidecar 代码**。

---

## 预留 Topic Namespace

`/vehicle_wbt/v1/` 下的 reserved/aux 命名空间预先定义好,启用新硬件只需 `enabled: true`,**不重新规划 topic**:

```
/vehicle_wbt/v1/
├── sensors/
│   ├── camera/<id>/image_raw        # N 路摄像头 (id=front|side|arm|...)
│   ├── imu/data                     # IMU(预留,需硬件)
│   ├── ir/<id>                      # 红外测距(id=left|right|front|rear|...)
│   ├── ultrasonic/<id>              # 超声波(预留)
│   ├── touch/<id>                   # 触摸(预留)
│   └── ambient_light/<id>           # 环境光(预留)
├── actuators/
│   ├── motor/<id>/state             # 电机状态(id=M1-M6)
│   ├── servo/<id>/state             # 舵机状态(id=S1-S7)
│   ├── stepper/<id>/state           # 步进电机(id=1-3)
│   ├── io/<id>/state                # IO 口(id=P1-P8)
│   └── vacuum/<id>/state            # 真空泵 / 阀
├── perception/
│   ├── lane                         # 车道线
│   ├── detections/<model_id>        # 检测结果(按模型分)
│   ├── ocr                          # OCR
│   └── tracks                       # MOT 跟踪
├── state/
│   ├── odom
│   ├── tf
│   ├── tf_static
│   ├── joint_states
│   ├── battery
│   └── system/<key>                 # 通用系统状态(ctl_id / uptime / ...)
├── task/
│   ├── state
│   └── event
├── safety/
│   ├── mode_cmd
│   ├── heartbeat
│   ├── estop
│   └── deadman
├── diagnostics/
│   ├── array
│   └── <component>                  # 每个 component 一条 DiagnosticStatus
├── cmd/
│   ├── vel_safe                     # 远程手动(仅 simulator / MANUAL 模式)
│   └── arm/<arm_id>/trajectory      # 机械臂轨迹(预留,sim 用)
├── reserved/                        # 物理未接,topic 预占
│   ├── nixie/<id>                   # 数码管
│   ├── bluetooth_pad/<id>           # 蓝牙手柄
│   ├── board_key/<id>               # 板载按键
│   ├── screen/<id>                  # 屏幕
│   └── csi/<id>                     # CSI/MIPI 摄像头(v2)
└── aux/                             # 已规划但 v1 未启用
    ├── arm/<arm_id>/joint_states    # 多机械臂扩展(v2)
    ├── second_chassis/odom          # 拖车 / 双车协调(v2)
    └── slam/<key>                   # SLAM / nav2 桥(v2,需评估)
```

**约定**: `/reserved/` 表示"硬件未接但协议已定义";`/aux/` 表示"未来可能要做,协议未定义"。两者均不报错;sidecar 启动时无对应 publisher 即不创建 node。

---

## 概述

本文定义 `vehicle_wbt` 平台在保留现有 ZMQ + 串口 + 直接函数调用架构的前提下,叠加一个**独立 ROS2 sidecar 节点**的完整设计。sidecar 是只读观察者与命令路由器 (observe + route),不取代现有控制路径;通过 DDS 在同一 Jetson 上运行,也允许开发者笔记本、远程调试机参与订阅。系统需满足三条底线:

- **(a) 不修改现有运动学、推理、机械臂、任务调度任一核心文件** (见 CLAUDE.md "DO_NOT_MODIFY")
- **(b) ENABLE_ROS2=0 时 sidecar 完全停摆**,主程序行为字节级一致
- **(c) 仿真回路可在无硬件笔记本上独立运行**,便于 4-6 人并行开发

---

## 背景与约束

| 维度 | 现实约束 |
|------|---------|
| 团队规模 | 4-6 人并行工作;ROS2 技能有但偏 ROS Noetic 经验 |
| 硬件 | Jetson 系列(具体型号运行时检测,通过 `sysfs` 读 `/proc/device-tree/model`) |
| 控制器 | MC601 / MC602 / MC602 无线,通过 CH340 USB 转串口(详见 hardware-port-mapping.md) |
| 编码器 | MC601: velocity × time 积分(仿真);MC602: 真实硬件编码器 |
| 操作系统 | Linux for Tegra (L4T),Ubuntu 20.04/22.04(由载板决定),ROS2 Humble LTS |
| 当前 ROS 状态 | Noetic 预装(零代码引用),ROS2 未装(colcon 工具有) |
| 不可破坏 | systemd `py_boot.service`、`main/qqq.py` 启动目标、MC601/MC602 双协议派发、ZMQ REP 端口 5001-5004、Camera daemon 线程、`sys.path.append` 约定、`import vehicle` 硬件副作用、global 状态(`serial_wrap` / `ctl_id` / `serial_mc601` / `serial_mc602` / `encoder_motor_all_sim1`) |
| 不可新增 | 裸 `except:`、`while True: time.sleep(1)` 代替错误处理、硬编码密钥、`eval(chassis_type)`、`eval()` 解析 LLM 输出 |
| 协作模式 | 单 Jetson + 开发者笔记本并行;VPN 偶尔;4-6 个独立 sidecar 包可分配给 4-6 人 |

**机器人形态配置**(运行时由 YAML + URDF 决定,**不硬编码**):
- Chassis: 由 `vehicle/driver/cfg_vehicle.yaml::chassis_type` 决定(Mecanum / Diff2 / Diff4 / Tricycle / Quadricycle)
- 机械臂: N 关节,v1 假设 2 步进 + 2 舵机 + 真空 + 阀;扩展性见 §机械臂抽象
- 摄像头: N 路,v1 假设 2 路 USB(`/dev/cam1`, `/dev/cam2`);最多 4 路
- 全部接口清单(已用 + reserved):见 §未用硬件清单

---

## 架构

sidecar 不在主进程中嵌入,而是由 `main/qqq.py` 在 `ENABLE_ROS2=1` 时 fork 一个独立 ROS2 Python 进程。两者通过 DDS 通信 + 文件(共享内存 / 命名管道)共享感知快照,无侵入式耦合。

**核心抽象**: sidecar 是 **Component 集合**,不是 monolithic node。每个 Component = 1 个 HardwareInterface + 1 个 ROS2 node + 1 个 YAML 注册条目;sidecar 启动时扫描 `config_sensors.yml`,按需 spawn 对应 node。

### 完整拓扑

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                         Jetson (L4T, 型号运行时检测)                          │
│                                                                                │
│  ┌────────────────────────────────────────┐    ┌────────────────────────┐    │
│  │  Main Process (main/qqq.py)            │    │  ROS2 Sidecar          │    │
│  │  ──────────────────────────            │    │  (subprocess.Popen)    │    │
│  │  MyCar (car_wrap.py, 1438 行)          │    │                        │    │
│  │    ├── ClintInterface ─┐               │    │  ros2_ws/src/          │    │
│  │    ├── CarBase         │ ZMQ (loopback │    │  ├── camera_node       │    │
│  │    │   (BaseChassis:  │   127.0.0.1)  │    │  ├── car_status_node   │    │
│  │    │    Mecanum/Diff/ │               │    │  ├── inference_bridge  │    │
│  │    │    Tricycle/...) │               │    │  ├── tf_static_pub     │    │
│  │    ├── ArmBase         │               │    │  ├── safety_gate       │    │
│  │    │   (URDF 驱动)    │               │    │  ├── chassis_bridge    │    │
│  │    └── MyTask          │               │    │  ├── arm_bridge        │    │
│  │         │              │               │    │  ├── sensor_bridge/*   │    │
│  │         ▼              ▼               │    │  └── mock_node (sim)   │    │
│  │  Camera (daemon thread)                │    │         │              │    │
│  │  ──────────────────────                │    │         ▼              │    │
│  │  ┌────────────────────────────────┐    │    │  ┌─────────────────────┐│    │
│  │  │ SidecarAdapter (singleton)     │──publish-shared-fd──┐          ││    │
│  │  │  - ringbuffer frame stream     │    │    │  │ ROS2 DDS Domain     ││    │
│  │  │  - hooks into CarBase events   │    │    │  │ (cyclonedds, v1)    ││    │
│  │  │  - emits on every Ctrl/Vel     │    │    │  └─────────────────────┘│    │
│  │  └────────────────────────────────┘    │    └────────────────────────┘    │
│  └────────────────────────────────────────┘                                    │
│           │                                          │                         │
│           ▼                                          ▼                         │
│  ┌─────────────────────┐                  ┌─────────────────────┐             │
│  │ MC601/MC602 (CH340) │                  │ /dev/cam1..N (V4L2) │             │
│  │ ControllerAdapter   │                  │ 摄像头 (USB)         │             │
│  │ (BaseController 子类)│                  └─────────────────────┘             │
│  └─────────────────────┘                                                      │
│                                                                                │
│  ┌─────────────────────────────────────────────────────────────────┐         │
│  │  InferServer (subprocess, N ZMQ REP, 端口由 infer.yaml)          │         │
│  │  lane:5001 / task:5002 / front:5003 / ocr:5004 ...              │         │
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
| ROS2 `/cmd/vel_safe` (外部) | **不直接驱动电机** — 仅作 simulator feed | ROS2 DDS | 20 Hz |
| `/diagnostics` (sidecar) | 主进程读 | shared memory fd | 1 Hz |

---

## C++ 核心 (Phase 1.5 增量 — 性能/实时层)

> **v1 协议层架构决策**: 本平台采用 **nav2 / moveit 风格 C++/Python 混合架构**——C++ 写性能/实时/硬件层,Python 写编排/配置/胶水。这是 ROS2 主流包(nav2, moveit, ros2_control, image_transport, tf2_ros)的事实标准,而非 "用 C++ 重写 Python"。

### 为什么不纯 Python

| 维度 | Python (Phase 1) | C++ (Phase 1.5+) |
|------|-------------------|-------------------|
| 控制循环延迟 | ~1 ms (有 GIL) | ~100 µs (无 GIL) |
| 实时调度 | best-effort | `SCHED_FIFO` 可选 |
| ros2_control 集成 | 不可用 (官方仅 C++) | 一等公民 |
| 图像处理吞吐 | 受 GIL 限制 | 多核并行 |
| 调试门槛 | 低 (REPL) | 中 (gdb / sanitizers) |
| 团队学习 | 已有 | 需 C++ + CMake |

**结论**: Python 适合 glue (config, orchestrator, lifecycle management),C++ 适合 core (controller adapters, kinematics, hardware interfaces)。两套在同一 `colcon_ws` build,共享 `.msg` 消息类型。

### 包拆分

```
ros2_ws/src/
├── vehicle_wbt_platform_cpp/        # ament_cmake (C++)  — Phase 1.5
│   ├── include/vehicle_wbt_platform_cpp/
│   │   ├── base_controller.hpp     # 纯虚接口
│   │   ├── mc602_adapter.hpp       # MC602 串口 I/O
│   │   ├── base_chassis.hpp        # 纯虚底盘 + Pose2D
│   │   ├── mecanum_chassis.hpp     # 4 轮 O 布局运动学
│   │   └── mc602_hardware_interface.hpp  # ros2_control SystemInterface
│   ├── src/                         # 对应 .cpp 实现
│   ├── msg/                         # 3 个 .msg 消息 (跨语言共享)
│   │   ├── LaneResult.msg
│   │   ├── DetectionArray.msg
│   │   └── ActuatorState.msg
│   ├── plugin.xml                   # pluginlib descriptor
│   └── test/                        # gtest (不依赖 rclpy)
│
└── vehicle_wbt_platform_py/         # ament_python (Python) — Phase 1
    ├── vehicle_wbt_platform/       # config_loader / orchestrator / __main__
    └── test/                        # pytest
```

### 调用方式 (nav2 风格)

1. **Python orchestrator** 读 `config_sensors.yml`,按 type 决定要 spawn 哪个 C++ 节点
2. **C++ node** 通过 `rclcpp` publish 消息到 `/vehicle_wbt/v1/...` topic
3. **Python** 通过 `rclpy` 订阅 topic 处理 high-level logic (任务编排, AI 集成)
4. **跨语言消息** 通过 `from vehicle_wbt_platform_cpp.msg import LaneResult` 共享 — `colcon build` 自动生成 Python binding

### 关键不变量 (从 Phase 1 继承)

- `main/qqq.py` 仍然零修改
- `ENABLE_ROS2=0` 仍然零影响
- `config_sensors.yml` schema 不变 (Python 解析, C++ 节点按 type 加载)
- Topic 命名空间 `/vehicle_wbt/v1/` 不变
- 现有 Python adapter/chassis 代码保留作为 **Type 1 / Prototype**(失败回退路径),正式运行时用 C++ 节点

### ros2_control 集成 (Phase 1.5 关键收益)

```xml
<!-- urdf/vehicle_wbt.urdf.xacro -->
<ros2_control name="MC602" type="system">
  <hardware>
    <plugin>vehicle_wbt_platform_cpp/MC602HardwareInterface</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="baud">1000000</param>
  </hardware>
  <joint name="wheel_m1"><command_interface name="velocity"/>
                         <state_interface name="position"/>
                         <state_interface name="velocity"/></joint>
  ...  <!-- 4 wheels -->
</ros2_control>
```

加载后 `controller_manager` 自动接管控制循环,`diff_drive_controller` / `mecanum_steering_controller` 等可直接接管。**这是纯 Python 架构无法实现的能力**。

### v1 暂不做的 C++ 工作 (Plan B 范围)

- 真实 MC602 协议 (CRC + frame parsing) — Phase 1.5 stub
- Mecanum/Diff/Tricycle 全部 5 个 chassis 子类的 C++ 实现 — Phase 1.5 只有 Mecanum
- 完整 ros2_control controller (差速 / 全向) — Plan B
- Camera C++ component (`image_transport` 集成) — Plan B
- 4 个 C++ 节点 (MecanumChassisNode / CameraNode / IRNode / EjectionNode) — Plan B
- 性能基准 (latency / jitter 测量) — Plan D

---

## Topic Schema

所有 topic 使用 `/vehicle_wbt/v1/<category>/<sub>/...` 命名空间。`<category>` 是顶层分类(sensors/actuators/perception/state/task/safety/diagnostics/cmd),实例 ID 嵌在路径中(便于多实例)。

### 1. Sensors (主进程 → sidecar → 外部)

| Topic 名 | 类型 | 频率 | QoS | 用途 |
|---------|------|:----:|-----|------|
| `/vehicle_wbt/v1/sensors/camera/front/image_raw` | `sensor_msgs/Image` | 10 Hz | BEST_EFFORT, depth=2 | 前向 V4L2 摄像头(与下游 10 Hz 推理匹配) |
| `/vehicle_wbt/v1/sensors/camera/side/image_raw` | `sensor_msgs/Image` | 10 Hz | BEST_EFFORT, depth=2 | 侧向摄像头(可选) |
| `/vehicle_wbt/v1/sensors/camera/<id>/image_raw` | `sensor_msgs/Image` | 10 Hz | BEST_EFFORT, depth=2 | 第 N 路摄像头(`config_sensors.yml` 启用后自动出现) |
| `/vehicle_wbt/v1/sensors/imu/data` | `sensor_msgs/Imu` | 100 Hz | BEST_EFFORT | 9-DOF IMU(预留,需外挂硬件) |
| `/vehicle_wbt/v1/sensors/ir/front` | `std_msgs/Int32MultiArray` | 20 Hz | RELIABLE | 红外测距(实例 ID 由 YAML 决定) |
| `/vehicle_wbt/v1/sensors/encoders/wheel` | `vehicle_wbt_msgs/WheelEncoders` | 50 Hz | BEST_EFFORT | 编码器(数量由 chassis 决定,Mecanum=4,Diff2=2 等) |

### 2. Actuators (主进程 → sidecar → 外部)

| Topic 名 | 类型 | 频率 | 用途 |
|---------|------|:----:|------|
| `/vehicle_wbt/v1/actuators/motor/<id>/state` | `vehicle_wbt_msgs/ActuatorState` | 50 Hz | 电机状态(id=M1-M6) |
| `/vehicle_wbt/v1/actuators/servo/<id>/state` | `vehicle_wbt_msgs/ActuatorState` | 50 Hz | 舵机状态(id=S1-S7) |
| `/vehicle_wbt/v1/actuators/stepper/<id>/state` | `vehicle_wbt_msgs/ActuatorState` | 50 Hz | 步进电机状态(id=1-3) |
| `/vehicle_wbt/v1/actuators/io/<id>/state` | `std_msgs/Bool` | 10 Hz | IO 口状态(id=P1-P8) |
| `/vehicle_wbt/v1/actuators/vacuum/<id>/state` | `std_msgs/Bool` | 10 Hz | 真空泵 / 阀 |

### 3. Perception (推理结果)

| Topic 名 | 类型 | 频率 | 用途 |
|---------|------|:----:|------|
| `/vehicle_wbt/v1/perception/lane` | `vehicle_wbt_msgs/LaneResult` | 20 Hz | 车道线分割结果 |
| `/vehicle_wbt/v1/perception/detections/task` | `vehicle_wbt_msgs/DetectionArray` | 10 Hz | task_wbt2025 模型输出 |
| `/vehicle_wbt/v1/perception/detections/front` | `vehicle_wbt_msgs/DetectionArray` | 10 Hz | front_model2 输出 |
| `/vehicle_wbt/v1/perception/detections/<model_id>` | `vehicle_wbt_msgs/DetectionArray` | 10 Hz | 第 N 个检测模型(`infer.yaml` 注册) |
| `/vehicle_wbt/v1/perception/ocr` | `vehicle_wbt_msgs/OcrResult` | 5 Hz (按需) | OCR 文本识别 |
| `/vehicle_wbt/v1/perception/tracks` | `vehicle_wbt_msgs/TrackArray` | 20 Hz | MOT 跟踪 ID |

### 4. State (主进程 → sidecar)

| Topic 名 | 类型 | 频率 | 用途 |
|---------|------|:----:|------|
| `/vehicle_wbt/v1/state/odom` | `nav_msgs/Odometry` | 50 Hz | 里程计(base_link → odom) |
| `/vehicle_wbt/v1/state/tf` | `tf2_msgs/TFMessage` | 50 Hz | 动态 TF(base_link → camera/arm_link) |
| `/vehicle_wbt/v1/state/tf_static` | `tf2_msgs/TFMessage` | latched | 静态 TF(底盘尺寸 → URDF) |
| `/vehicle_wbt/v1/state/joint_states` | `sensor_msgs/JointState` | 50 Hz | 所有关节(机械臂关节 + 步进 + 舵机,N 维) |
| `/vehicle_wbt/v1/state/battery` | `sensor_msgs/BatteryState` | 1 Hz | 电池电量(预留,目前占位) |
| `/vehicle_wbt/v1/state/system/ctl_id` | `std_msgs/Int32` | latched | 当前 ctl_id(0=MC601, 1=MC602) |
| `/vehicle_wbt/v1/state/system/chassis_type` | `std_msgs/String` | latched | 当前 chassis 类型(Mecanum/Diff2/...) |

### 5. Task / Control Signals

| Topic 名 | 类型 | 频率 | 方向 | 用途 |
|---------|------|:----:|:----:|------|
| `/vehicle_wbt/v1/task/state` | `vehicle_wbt_msgs/TaskState` | 10 Hz | 主→sidecar | 当前任务 FSM 状态 |
| `/vehicle_wbt/v1/task/event` | `vehicle_wbt_msgs/TaskEvent` | event | 主→sidecar | 关键事件流(拾起/放下/失败) |
| `/vehicle_wbt/v1/safety/mode_cmd` | `std_msgs/String` | 10 Hz | 外部→safety_gate | AUTO/MANUAL/ESTOP/SIM |
| `/vehicle_wbt/v1/safety/heartbeat` | `std_msgs/Int32` | 5 Hz | safety_gate→主 | 5s 心跳(失联自动 E-stop) |
| `/vehicle_wbt/v1/safety/estop` | `std_msgs/String` | event | sidecar→主 | E-stop 事件(reason 字符串) |
| `/vehicle_wbt/v1/cmd/vel_safe` | `geometry_msgs/Twist` | 20 Hz | 外部→sidecar | 仅 simulator 用,**不驱动真机** |

### 6. Diagnostics

| Topic 名 | 类型 | 频率 | 用途 |
|---------|------|:----:|------|
| `/vehicle_wbt/v1/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | 全组件健康 |
| `/vehicle_wbt/v1/diagnostics/<component>` | `diagnostic_msgs/DiagnosticStatus` | 1 Hz | 单组件健康(component=`serial`/`infer`/`camera_front`/...) |
| `/vehicle_wbt/v1/diagnostics/<component>/<instance>` | `vehicle_wbt_msgs/ComponentStatus` | 1 Hz | Component 自定义状态(实例级) |

### 7. Reserved / Aux 命名空间

详见 §预留 Topic Namespace。`/vehicle_wbt/v1/reserved/*` 物理未接时不创建 publisher;`/vehicle_wbt/v1/aux/*` 仅在 v2+ 启用。

### 命名约定

- 所有 topic 以 `/vehicle_wbt/v1/` 前缀,与 `ROS_DOMAIN_ID=42` 配合
- 实例 ID 嵌在路径中:`/camera/<id>/image_raw`,`/motor/<id>/state`,`/arm/<arm_id>/...`
- 子分类用单数(`camera/`, `motor/`, `ir/`),实例用复数只在明确是多实例集合时(`tracks`, `detections`)
- 任务事件以 `task/<verb>` 子命名空间
- 安全相关一律 `safety/`,避免与控制命令混淆
- 预留功能用 `reserved/` 或 `aux/` 子命名空间,不占用生产路径

---

## 自定义消息

文件位于 `ros2_ws/src/vehicle_wbt_msgs/msg/`。

### `LaneResult.msg`

```
# 车道线分割结果(来自 lane 推理服务)
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

### `DetectionArray.msg` / `Detection.msg`

```
================================================================================
DetectionArray.msg
================================================================================
std_msgs/Header header
Detection[] detections

================================================================================
Detection.msg
================================================================================
string class_name            # 模型类别名(由模型定义决定,不绑死)
int32 class_id
float32 score                # 置信度
float32[4] bbox              # [x1, y1, x2, y2]
float32[3] center_3d         # 相机坐标系 (X, Y, Z) 米
int32 track_id               # MOT ID (-1 = 无跟踪)
```

### `TaskEvent.msg`

```
std_msgs/Header header

# FSM 事件类型(强类型枚举)
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
string task_name             # 任务名(自由字符串,不绑死枚举)
string detail                # 自由文本(失败原因等)
float32 confidence
```

### `ActuatorState.msg` (通用,替代原 `ArmState.msg`)

```
std_msgs/Header header

# 通用执行器状态(电机/舵机/步进都用)
string actuator_id           # "M1", "S2", "stepper_1" 等(由 component id 决定)
string actuator_type         # "motor" | "servo" | "stepper" | "io" | "vacuum"

# 关节/执行器位置/速度/力矩 — 用 N 维数组,不绑死 6 关节
float32[] positions          # 弧度 / 编码器计数(由类型决定)
float32[] velocities         # RPM / rad·s⁻¹
float32[] efforts            # 力矩 / 电流

# 真空泵 / 夹爪(通用执行器特有的辅助字段)
bool[] binary_states         # 真空开 / 夹爪闭 / IO 高电平等

# 任务进度
int8 task_phase              # 0=idle, 1=moving, 2=picking, 3=placed, 4=failed
string current_target        # 物体类别名(任意任务)

# 限位状态
bool[] limit_hit             # 各执行器是否触发限位
```

> **设计意图**: `ActuatorState` 用 N 维数组,适配 2 步进 + 2 舵机(小臂)、6 关节机械臂、7DOF 机械臂、双臂(v2)。`actuator_type` 字段让订阅者区分语义。

### `ComponentStatus.msg` (新增,通用组件健康)

```
std_msgs/Header header

# 组件唯一标识
string component_name        # "camera_front", "motor_M1", "serial_mc602"
string component_type        # "camera" | "motor" | "serial" | "infer" | "arm" | ...

# 健康状态
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_OK = 1
uint8 STATUS_WARN = 2
uint8 STATUS_ERROR = 3
uint8 STATUS_STALE = 4       # 数据超时
uint8 status

# 详细信息
string message               # 自由文本("ZMQ 5003 connection refused")
int32 error_code             # 0=无错误,其它由组件定义

# 运行时长
float32 uptime_s

# 组件特有 key-value 数据(扩展点)
string[] kv_keys
string[] kv_values
```

### `WheelEncoders.msg`

```
std_msgs/Header header
# 编码器原始计数(数量由 chassis 决定 — Mecanum=4, Diff2=2, Diff4=4, Tricycle=2 等)
int32[] raw_count
float32[] velocity_rpm
uint8 ctl_id                 # 0=MC601, 1=MC602
bool encoder_is_simulated    # MC601 总是 true
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
string active_task           # 任务名(自由字符串)
float32 progress             # 0.0-1.0
string last_error
```

### `TrackArray.msg` / `Track.msg`

```
================================================================================
TrackArray.msg
================================================================================
std_msgs/Header header
Track[] tracks

================================================================================
Track.msg
================================================================================
int32 track_id
string class_name
float32[8] bbox_history      # 最近 8 帧 bbox [x1,y1,x2,y2]
float32[3] velocity_3d
int32 age                    # 跟踪帧数
bool lost
```

---

## Component 模型

sidecar 是 Component 集合,不是 monolithic node。每个 Component = 1 个 HardwareInterface + 1 个 ROS2 node + 1 个 YAML 注册条目。

### Component 接口契约

任何 Component 必须实现以下契约(以伪代码表示):

```python
class BaseComponent(ABC):
    """所有 sidecar component 的基类"""

    @abstractmethod
    def register(self, config: dict) -> None:
        """从 YAML 读取配置,初始化自身"""
        ...

    @abstractmethod
    def init(self, ros_node: rclpy.node.Node) -> None:
        """创建 publisher/subscriber,绑定 ROS2 接口"""
        ...

    @abstractmethod
    def read(self) -> Any:
        """从硬件读数据(轮询或回调)"""
        ...

    @abstractmethod
    def write(self, cmd: Any) -> None:
        """写数据到硬件(仅 control component 需要)"""
        ...

    @abstractmethod
    def cleanup(self) -> None:
        """释放资源"""
        ...

    @abstractmethod
    def status(self) -> ComponentStatus:
        """上报健康状态"""
        ...
```

### Component Lifecycle

```
register → init → [read/write loop] → cleanup
                          │
                          └── status() 每 1Hz 上报
```

### Component 注册表 (`config_sensors.yml`)

```yaml
# config_sensors.yml — 传感器/执行器注册表(sidecar 启动时扫描)
sensors:
  - id: camera_front
    type: camera
    enabled: true
    device: /dev/cam2
    topic: /vehicle_wbt/v1/sensors/camera/front/image_raw
    rate_hz: 10

  - id: camera_side
    type: camera
    enabled: true
    device: /dev/cam1
    topic: /vehicle_wbt/v1/sensors/camera/side/image_raw
    rate_hz: 10

  - id: camera_arm_wrist  # 预留,v1 不启用
    type: camera
    enabled: false
    device: /dev/cam3
    topic: /vehicle_wbt/v1/sensors/camera/arm_wrist/image_raw
    rate_hz: 10

  - id: ir_front
    type: ir
    enabled: true
    controller: mc602
    io_pin: P6
    topic: /vehicle_wbt/v1/sensors/ir/front

actuators:
  - id: motor_M1
    type: motor
    enabled: true
    controller: mc602
    motor_port: 1
    topic: /vehicle_wbt/v1/actuators/motor/M1/state

  - id: vacuum_main
    type: vacuum
    enabled: true
    controller: mc602
    io_pin: P1
    topic: /vehicle_wbt/v1/actuators/vacuum/main/state

  - id: stepper_2  # 预留
    type: stepper
    enabled: false
    controller: mc602
    stepper_id: 2
    topic: /vehicle_wbt/v1/actuators/stepper/2/state
```

### Component 加载流程

```
sidecar 启动
  │
  ▼
读取 config_sensors.yml
  │
  ▼
对每个 enabled=true 的 entry:
  │
  ├── 动态 import plugins/<type>.py (如 plugins/camera.py)
  ├── 实例化对应 Component 子类
  ├── Component.register(entry)
  ├── Component.init(ros_node)
  └── 加入 executor spin
```

**关键**: 启用新硬件 = 1 行 YAML,不写业务代码。

---

## 硬件接口插件

`BaseControllerHardwareInterface` 是所有电机控制器的统一抽象。MC601 / MC602 / 未来 MCxxx 都是这个接口的 adapter。

### BaseController 接口

```python
# vehicle/controller/base_controller.py (新文件, 不修改现有 controller_wrap.py)
from abc import ABC, abstractmethod
from typing import Tuple

class BaseControllerHardwareInterface(ABC):
    """所有电机控制器的统一接口"""

    @abstractmethod
    def connect(self, port: str) -> bool:
        """打开串口,探测协议版本"""
        ...

    @abstractmethod
    def read_encoders(self) -> Tuple[int, ...]:
        """读所有编码器(数量由 chassis 决定)"""
        ...

    @abstractmethod
    def set_motor_speed(self, motor_id: int, rpm: float) -> None:
        """单电机速度控制"""
        ...

    @abstractmethod
    def set_servo_angle(self, servo_id: int, angle_deg: float) -> None:
        """单舵机角度控制"""
        ...

    @abstractmethod
    def set_stepper_position(self, stepper_id: int, steps: int) -> None:
        """单步进位置控制"""
        ...

    @abstractmethod
    def read_io(self, io_id: str) -> bool:
        """读 IO 电平"""
        ...

    @abstractmethod
    def write_io(self, io_id: str, value: bool) -> None:
        """写 IO 电平"""
        ...

    @abstractmethod
    def probe_device_id(self) -> int:
        """探测控制器类型(0x06=MC601, 0x16=MC602, ...)"""
        ...

    @abstractmethod
    def status(self) -> 'ComponentStatus':
        """健康上报"""
        ...
```

### MC601Adapter / MC602Adapter

```python
# vehicle/controller/mc601_adapter.py
class MC601Adapter(BaseControllerHardwareInterface):
    """MC601 @ 380400 baud — 编码器由 velocity × time 仿真"""

    def connect(self, port: str) -> bool:
        # 调用现有 serial_wrap.Motor_1 的探测逻辑(不重写)
        ...

    def read_encoders(self) -> Tuple[int, ...]:
        # MC601 编码器是仿真值 — 标记 encoder_is_simulated=true
        return self._simulate_encoders_from_velocity()

# vehicle/controller/mc602_adapter.py
class MC602Adapter(BaseControllerHardwareInterface):
    """MC602 @ 1000000 baud — 真实编码器"""

    def connect(self, port: str) -> bool:
        # 调用现有 serial_wrap.Motor_2 的探测逻辑
        ...
```

### pluginlib 加载约定

```python
# vehicle/controller/__init__.py
import importlib
import os

def load_controller_adapter(dev_id: int, port: str) -> BaseControllerHardwareInterface:
    """根据 dev_id 动态加载 adapter — 加新控制器 = 在 plugins/ 加 1 个文件"""
    plugin_dir = os.path.join(os.path.dirname(__file__), 'plugins')
    for fname in os.listdir(plugin_dir):
        if fname.endswith('_adapter.py'):
            mod = importlib.import_module(f'vehicle.controller.plugins.{fname[:-3]}')
            if mod.SUPPORTED_DEV_ID == dev_id:
                adapter = mod.Adapter(port=port)
                if adapter.probe_device_id() == dev_id:
                    return adapter
    raise RuntimeError(f"No adapter found for dev_id={dev_id:#x}")
```

**约定**: 新控制器 = 在 `vehicle/controller/plugins/` 加 1 个 `mcXXX_adapter.py`,不动 sidecar。

---

## Chassis 抽象

`BaseChassis` 是所有底盘类型的统一接口。`MyCar` 不感知 chassis 类型,只调用 `BaseChassis` 接口。

### BaseChassis 接口

```python
# vehicle/driver/chassis/base_chassis.py (新文件,不修改 vehicle_base.py)
from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np

@dataclass
class ChassisGeometry:
    wheel_radius_m: float
    wheelbase_m: float
    track_m: float
    wheel_positions: list  # [(x, y, theta), ...] — 来自 URDF

class BaseChassis(ABC):
    """所有底盘类型的统一接口"""

    def __init__(self, geometry: ChassisGeometry):
        self.geometry = geometry

    @abstractmethod
    def forward_kinematics(self, wheel_speeds: np.ndarray) -> Tuple[float, float, float]:
        """轮速 → (vx, vy, wz)"""
        ...

    @abstractmethod
    def inverse_kinematics(self, vx: float, vy: float, wz: float) -> np.ndarray:
        """(vx, vy, wz) → 轮速"""
        ...

    @abstractmethod
    def update_odometry(self, current_pose, wheel_deltas, dt) -> 'Pose2D':
        """轮位移 → 底盘位姿更新"""
        ...

    @abstractmethod
    def num_wheels(self) -> int:
        """轮子数量(Mecanum=4, Diff2=2, Diff4=4, Tricycle=2, Quadricycle=4)"""
        ...
```

### 5 个子类

| 类 | 轮数 | 适用场景 |
|----|:---:|---------|
| `MecanumChassis` | 4 | 全向移动(竞赛现用) |
| `Diff2Chassis` | 2 | 差速两轮(轻量机器人) |
| `Diff4Chassis` | 4 | 差速四轮(越野 / 重载) |
| `TricycleChassis` | 2+1 | 三轮(前舵后驱,简单控制) |
| `QuadricycleChassis` | 4 | 四轮独立转向(汽车形态) |

### YAML 切换

```yaml
# vehicle/driver/cfg_vehicle.yaml
chassis_type: mecanum  # 可选: mecanum | diff2 | diff4 | tricycle | quadricycle

# 几何参数(由 URDF 自动生成,这里只覆盖特殊值)
geometry:
  wheel_radius_m: 0.05
  wheelbase_m: 0.30
  track_m: 0.30
```

### 加载逻辑

```python
# vehicle/driver/chassis/__init__.py
_CHASSIS_REGISTRY = {
    'mecanum': 'MecanumChassis',
    'diff2': 'Diff2Chassis',
    'diff4': 'Diff4Chassis',
    'tricycle': 'TricycleChassis',
    'quadricycle': 'QuadricycleChassis',
}

def load_chassis(chassis_type: str, geometry: ChassisGeometry) -> BaseChassis:
    cls_name = _CHASSIS_REGISTRY.get(chassis_type)
    if not cls_name:
        raise ValueError(f"Unknown chassis_type: {chassis_type}")
    mod = importlib.import_module(f'vehicle.driver.chassis.{chassis_type}_chassis')
    return getattr(mod, cls_name)(geometry)
```

**约束**: 加新 chassis = 写 1 个 `vehicle/driver/chassis/<type>_chassis.py`,实现 4 个抽象方法,不修改 `MyCar` 或 `vehicle_base.py`。

---

## 机械臂抽象

机械臂作为 Component,通过 YAML 配置,URDF 描述关节链。

### v1 默认拓扑

- 2 步进电机(stepper_1, stepper_3)— 垂直轴
- 2 舵机(S2, S3 或 S7)— 旋转 / 夹爪
- 真空泵(P1)
- 阀(P2 复用或独立 IO)

### YAML 配置 (`vehicle/arm/arm_cfg.yaml`,已存在,仅扩展)

```yaml
arm:
  id: arm_main
  type: robotic_arm
  enabled: true

  joints:
    - id: stepper_1
      type: stepper
      controller: mc602
      stepper_id: 1
      range_rad: [-3.14, 3.14]

    - id: stepper_3
      type: stepper
      controller: mc602
      stepper_id: 3
      range_rad: [-3.14, 3.14]

    - id: S2
      type: servo
      controller: mc602
      servo_id: 2
      range_deg: [0, 180]

    - id: S3
      type: servo
      controller: mc602
      servo_id: 3
      range_deg: [0, 180]

  end_effector:
    - id: vacuum_main
      type: vacuum
      controller: mc602
      io_pin: P1

    - id: valve_main
      type: io
      controller: mc602
      io_pin: P2

  urdf: vehicle_wbt.urdf.xacro::arm_main
  topic_state: /vehicle_wbt/v1/state/joint_states
```

### v2+ 扩展路径(仅文档,不做实现)

- **7DOF 单臂**: YAML 中加 5 个 joint 条目,URDF 描述 7 连杆,`ArmState` 用 N 维数组自动适配
- **双臂**: 加 `arm_2` block,topic 加 `/arm/<id>/...` 前缀(已在 §预留 Topic Namespace 中预留)
- **第二夹爪**: 加 `end_effector: gripper_2`,URDF 加 link

**约束**: 7DOF / 双臂的 `ActuatorState` 不需要重新定义,N 维数组天然适配。

---

## Camera 抽象

Camera 作为 Component,支持多实例。

### v1 支持

- USB 摄像头,通过 V4L2 读取
- 槽位:`/dev/cam1`, `/dev/cam2`(已用),`/dev/cam3`, `/dev/cam4`(预留)
- **运行时检测**: 启动时扫描 `/dev/v4l/by-id/`,按物理路径匹配 config 中的 `device`

### v2+ 计划(待 Jetson 载板 spec 确认)

- CSI/MIPI 摄像头,通过 `nvarguscamerasrc` 或 `libcamera`
- `/dev/csi0`, `/dev/csi1`(槽位预留,topic 已在 §预留 Topic Namespace 中)
- **运行时检测**: `/proc/device-tree/csi` 节点存在才启用 CSI topic

### YAML 配置(`config_sensors.yml` 中 cameras 块)

```yaml
sensors:
  - id: camera_front
    type: camera
    enabled: true
    device: /dev/cam2        # 启动时验证存在;不存在则 disabled
    transport: usb            # usb | csi (v2+)
    topic: /vehicle_wbt/v1/sensors/camera/front/image_raw
    rate_hz: 10
    resolution: [640, 480]
    frame_id: camera_front_optical

  - id: camera_arm_wrist     # 预留,v1 disabled
    type: camera
    enabled: false
    device: /dev/cam3
    transport: usb
    topic: /vehicle_wbt/v1/sensors/camera/arm_wrist/image_raw
    rate_hz: 10
    frame_id: camera_wrist_optical
```

**约定**: 启用新摄像头 = 物理插入 + YAML 加条目 + URDF 加 link,不改 sidecar。

---

## 配置系统

### 配置文件家族

| 文件 | 职责 | 状态 | 修改规则 |
|------|------|:---:|---------|
| `config_sensors.yml` | **新增**:传感器/执行器 Component 注册表 | v1 引入 | 加新硬件改这个 |
| `vehicle/driver/cfg_vehicle.yaml` | chassis 类型 + 几何参数 + 速度 PID | 已存在 | 切 chassis 改这个 |
| `vehicle/arm/arm_cfg.yaml` | 机械臂关节配置 + 限位 + PID | 已存在 | 改机械臂拓扑改这个 |
| `vehicle/base/mc602_cfg.yaml` | MC602 校准参数 | 已存在 | 校准改这个 |
| `config_car.yml` | 摄像头索引(legacy)+ IO 引脚 + 速度限制 + lane/detect PID | 已存在 | 不再添加摄像头字段 |
| `infer_cs/base/infer.yaml` | 推理服务定义(端口 + 模型路径) | 已存在 | 加新推理服务改这个 |

### 加载优先级

```
URDF(物理形态)
  ↓ 读几何参数
cfg_vehicle.yaml(chassis_type + 几何覆盖)
  ↓ 读关节定义
arm_cfg.yaml(机械臂拓扑)
  ↓ 读 IO / 端口
mc602_cfg.yaml(控制器校准)
  ↓ 读 Component 实例
config_sensors.yml(传感器/执行器实例)
  ↓ 读推理服务
infer.yaml(模型 + 端口)
```

### 加新硬件流程(端到端,4 步)

1. **物理接线** — 接到对应端口(MC602 P口 / 舵机口 / USB)
2. **`config_sensors.yml` 加条目** — 1 行 YAML,设 `enabled: true`
3. **`urdf/vehicle_wbt.urdf.xacro` 加 link** — 描述物理位置 + 光学参数
4. **启用 reserved topic** — §未用硬件清单 中的 reserved 项,把对应 `enabled: false` 改 `true`

**不修改任何业务代码**。sidecar 重启即生效。

---

## 安全设计

sidecar **绝不**直接驱动电机,所有 `/cmd/*` 在 safety_gate 中经过 4 层闸门。安全模式状态机是独立模块,可单测。

### 4 层安全闸门

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 0: 物理急停                                                            │
│  ─────────────                                                               │
│  物理急停按键(对接 controller_wrap 中通用按键抽象)拉低 → 主进程轮询线程检测    │
│  → MyCar.set_speed(0,0,0,0) 直接走 Motors.set_speed(MC601/MC602)              │
│  → 不经过 ROS、不经过 safety_gate、不经过 ArmBase                              │
│  → 优先级最高 (CLAUDE.md: "never replace error conditions with sleep")        │
│                                                                              │
│  注: 物理急停按键的电气连接不绑死 BUTTON3,由 cfg_safety.yaml::estop_pin 指定 │
└─────────────────────────────────────────────────────────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 1: 心跳闸门 (safety_gate)                                              │
│  ──────────────────                                                          │
│  safety_gate 每 1s publish /safety/heartbeat                                 │
│  主进程 (SidecarAdapter) 5s 未收到 → 触发 E-stop callback                    │
│  E-stop 动作: car.stop() + arm.emergency_release() + 通知 main/qqq.py        │
│  同时 publish /vehicle_wbt/v1/safety/estop 事件 (供其他 sidecar 节点知晓)     │
└─────────────────────────────────────────────────────────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 2: 模式闸门                                                            │
│  ────────────                                                                │
│  收到 /safety/mode_cmd 必须为枚举:                                            │
│    "AUTO"     → 允许主进程自主决策                                            │
│    "MANUAL"   → 主进程必须收到 /cmd/vel_safe 才能移动 (但仍写真机)            │
│    "SIM"      → /cmd/vel_safe 仅驱动 Gazebo mock,真机电机必须为 0            │
│    "ESTOP"    → 不论其他输入,硬停                                            │
│  状态机非法转移 → reject + log error                                         │
└─────────────────────────────────────────────────────────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 3: 数据完整性闸门                                                       │
│  ────────────────────                                                        │
│  检查上游 topic 质量:                                                          │
│    /state/odom 时间戳 > 0.5s 旧 → 拒绝 /cmd/vel_safe                          │
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
│  远程手动控制 (/cmd/vel_safe) 必须每 200ms 重新发送               │
│  ──────────────────────────────────────────────────             │
│  safety_gate 维护: last_cmd_vel_ts                              │
│  每 250ms 检查: now - last_cmd_vel_ts > 250ms                  │
│  超时 → 自动停 0 速度 + 通知主进程                              │
│  这是 `/cmd/vel_safe` 在 MANUAL/SIM 下的硬要求                   │
└───────────────────────────────────────────────────────────────┘
```

### 物理急停按键 优先级

- 物理急停按键的电气接线由 `cfg_safety.yaml::estop_pin` 指定(不绑死 `Key4Btn.BUTTON3`,v1 默认走 `BUTTON3`)
- 主进程中按键后台线程检测到按下 → 直接调 `Motors.set_speed(0)` 绕过一切
- sidecar 通过 `/vehicle_wbt/v1/safety/estop` 收到事件,仅作日志/可视化
- **绝不允许 ROS 关闭时物理急停失效** — 测试用例必须包含:sidecar 进程 kill -9 时物理急停仍能停机

---

## 仿真回路

采用 A+C 组合策略:开发/调试用 mock + rosbag2 (轻量、纯 Python);最终验证用 Gazebo + ros2_control (含物理、可跑 motion)。

### A. Mock + rosbag2 (笔记本, 95% 调试)

适用:单测、集成测试、CI、4 人并行开发。

```
┌───────────────────────────────────────────────────────────────┐
│  仿真包: vehicle_wbt_sim_mock                                  │
│  ──────────────────────────────                                │
│  nodes/                                                        │
│    ├── mock_camera_node.py      # 读视频文件 / 合成图           │
│    ├── mock_serial_node.py      # 合成编码器 / odometry(按 chassis)│
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
from vehicle.driver.chassis import load_chassis

class MockSerialNode(Node):
    """合成编码器 + chassis odometry,模拟 Jetson 真机行为"""

    def __init__(self):
        super().__init__('mock_serial_node')
        self.declare_parameter('ctl_id', 1)
        self.declare_parameter('chassis_type', 'mecanum')
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('update_rate_hz', 50.0)

        self._chassis = load_chassis(
            chassis_type=self.get_parameter('chassis_type').value,
            geometry=...,
        )

        self._odom_pub = self.create_publisher(Odometry, '/vehicle_wbt/v1/state/odom', 10)
        self._joint_pub = self.create_publisher(JointState, '/vehicle_wbt/v1/state/joint_states', 10)

        self._bridge = CvBridge()
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/vehicle_wbt/v1/cmd/vel_safe', self._on_cmd_vel, 10
        )

        period = 1.0 / self.get_parameter('update_rate_hz').value
        self._timer = self.create_timer(period, self._tick)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._vx = msg.linear.x
        self._wz = msg.angular.z

    def _tick(self) -> None:
        # 用 BaseChassis.forward_kinematics 合成 odom
        wheel_speeds = self._chassis.inverse_kinematics(self._vx, 0.0, self._wz)
        vx, vy, wz = self._chassis.forward_kinematics(wheel_speeds)
        # publish odom ...

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MockSerialNode())
    rclpy.shutdown()
```

### C. Gazebo Sim + ros2_control (桌面级验证, 赛前最终)

适用:完整物理仿真,验证 motion planning、碰撞、传感噪声。

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  仿真包: vehicle_wbt_gz                                                          │
│  ────────────────────                                                            │
│  worlds/                                                                          │
│    ├── competition_track.sdf       # 复刻竞赛场地                              │
│    └── lab_test.sdf                # 实验室测试                                │
│  urdf/                                                                            │
│    ├── vehicle_wbt.urdf.xacro      # 从 URDF source-of-truth 自动生成           │
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

依赖包:`ros_gz`, `ros2_control`, `controller_manager`, `gz_ros2_control_plugins`, `xacro`, `joint_state_publisher_gui`。

#### URDF 来源

```
config_sensors.yml + cfg_vehicle.yaml + arm_cfg.yaml
            │
            ▼
scripts/urdf_gen.py (从 YAML 合成 xacro)
            │
            ▼
urdf/vehicle_wbt.urdf.xacro (机器可读)
            │
            ▼
Gazebo / RViz
```

**v1 演进**: 早期版本曾从 `hardware-port-mapping.md` 生成;v1 改用 YAML 三件套作为 source of truth,避免 markdown 表格解析的脆弱性。

### sim↔real 切换

```
ENABLE_ROS2=1 ENABLE_SIM=1   → /cmd/vel_safe → Gazebo chassis
ENABLE_ROS2=1 ENABLE_SIM=0   → /cmd/vel_safe → 主进程不订阅, 仅 RViz 可视化
ENABLE_ROS2=0                → sidecar 不启动, 主程序保持原样
```

`safety_gate` 在 `mode_cmd == "SIM"` 时自动拒绝一切非 Gazebo 节点的 `/cmd/vel_safe` 注入。

---

## 远程接入

> **重要**: 本项目采用 **dev/target 双机架构**。Dev 桌面机装完整 ROS2 desktop (RViz2 + Gazebo + 工具链);Jetson Orin Nano 4GB 只装 ROS2 Humble base (rclcpp + rclpy, 无 GUI)。两者通过 ROS_DOMAIN_ID=42 自动发现,DDS 跨机器通信。详细 setup 见 [../../development/README.md](../../development/README.md)。

### 部署拓扑 (dev + target)

```
┌─────────────────────────────────────────────────────┐
│  Dev 桌面机 (Ubuntu 22.04/24.04/26.04 + ROS2 desktop) │
│  ┌────────────────────────────────────────────────┐ │
│  │ rclcpp + rclpy + RViz2 + ros2 bag + Gazebo     │ │
│  │ 编辑代码 + 跑测试 + 仿真 + 可视化              │ │
│  └────────────────────────────────────────────────┘ │
│                  │                                  │
│                  │ DDS over LAN (cyclonedds)        │
│                  │ ROS_DOMAIN_ID=42                 │
│                  ▼                                  │
│  ┌────────────────────────────────────────────────┐ │
│  │ Jetson Orin Nano 4GB (JetPack 6 + Humble base) │ │
│  │ rclcpp + rclpy only                            │ │
│  │ ssh xrak@orin 跑 sidecar 节点                 │ │
│  │ 发布真实传感器数据 + 接收控制指令              │ │
│  └────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
```

**关键原则**:
- dev 装 **desktop** 版 (含 RViz/Gazebo);Jetson 装 **base** 版 (无 GUI)
- dev 可以装 Jazzy/Humble/Kilted 任意桌面版 (享受新功能);Jetson 锁 Humble LTS (生产稳定性)
- dev 跑 80% 的工作 (单元测试、lint、仿真、可视化);Jetson 跑 20% (真硬件冒烟、生产部署)
- 同一份代码在 dev + Jetson 都能跑,差别只是 dev 用 mock / Gazebo,Jetson 用真硬件

### DDS 配置

| 场景 | 部署 | ROS_DOMAIN_ID | DDS 实现 | RMW |
|------|------|:---:|---------|-----|
| 单 Jetson 本机 | Jetson 单独跑 sidecar | 42 | cyclonedds | rmw_cyclonedds_cpp |
| **dev + target 协作 (推荐)** | **dev 桌面 ROS2 desktop + Jetson Humble base** | **42** | **cyclonedds** | **rmw_cyclonedds_cpp** |
| VPN 跨网 | Jetson + 异地开发机 | 42 | cyclonedds | rmw_cyclonedds_cpp |
| Gazebo 离线 | dev 桌面独立运行 | 43 (避开主 domain) | fastrtps | rmw_fastrtps_cpp |

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
| 5001-5004 | TCP | ZMQ 推理(本地, 不暴露) |
| 22 | TCP | SSH 调 Jetson |

---

## 生命周期

sidecar 由 `main/qqq.py` 在启动时通过 `subprocess.Popen` fork,独立 ROS2 Python 进程(`ros2_ws/src/` 下的可执行节点)。**主进程任何崩溃不应影响 sidecar 反之亦然**。

### 启动顺序

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  systemd 启动 main/qqq.py                                                     │
│  ──────────────────────────                                                   │
│  1. main/qqq.py 启动                                                          │
│     ├── 加载 config_car.yml, config_sensors.yml                              │
│     ├── import vehicle.* (触发串口扫描, ctl_id 探测)                          │
│     ├── 启动 InferServer (subprocess, 端口由 infer.yaml)                      │
│     ├── 启动 Camera daemon 线程                                                │
│     └── 实例化 MyCar                                                            │
│                                                                                │
│  2. if ENABLE_ROS2 == "1":                                                     │
│     ├── spawn ros2_daemon: ros2 run vehicle_wbt_bringup sidecar_main          │
│     │     ├── 扫描 config_sensors.yml,按 enabled=true 加载 Component          │
│     │     ├── wait for /vehicle_wbt/v1/state/system/ctl_id (latched)          │
│     │     ├── wait for /vehicle_wbt/v1/state/odom first msg                   │
│     │     └── ready=publish /safety/heartbeat                                  │
│     │                                                                          │
│     └── 主进程 SidecarAdapter 启动 publish 线程                                │
│            ├── Camera.frame → /v1/sensors/camera/<id>/image_raw                │
│            ├── ClintInterface result → /v1/perception/detections/*             │
│            ├── CarBase.get_pose() → /v1/state/odom + /v1/state/tf              │
│            └── MyCar.fsm_state → /v1/task/state + /v1/task/event               │
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
  │           ├── publish /v1/safety/estop (reason=shutdown)
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

切换 ROS2 时:`sudo systemctl edit py_boot.service` 添加 `Environment="ENABLE_ROS2=1"`,不修改 systemd 主单元文件(符合 "DO_NOT_MODIFY" 中对 systemd service 的隐含保护)。

### ENABLE_ROS2=0 行为验证

测试用例必须包含:
- `ENABLE_ROS2=0` 启动 main/qqq.py → 行为与 2026-07-04 之前的部署字节级一致
- 不引入 `ros2` 进程 (ps -ef | grep ros2 = empty)
- 不引入 cyclonedds 端口 (netstat | grep 7400 = empty)
- `/vehicle_wbt/v1/*` 所有 topic 无人 publish,无人订阅
- main/qqq.py exit code = 0 与是否启用 ROS2 无关

---

## 实施阶段

| Phase | 周次 | 工作量 | 主要交付 | 负责人 |
|:----:|:---:|:---:|------|:---:|
| 1 | W1 | 3-4 天 | config_sensors.yml + Component 框架 + rosbag2 录放 | 4 人并行 |
| 2 | W2 | 4-5 天 | BaseController + BaseChassis adapter + URDF + Gazebo | 2 人 |
| 3 | W3 | 4-5 天 | control bridge + safety_gate + Component 集成 | 2 人 |
| 4 | W4 | 3-4 天 | 多机验证 + 刷机后部署 | 1 人 |

### Phase 1: Component 框架 + Mock + rosbag2 (W1)

- 安装 ROS2 Humble + 创建 `vehicle_wbt_msgs` (LaneResult / Detection / **ActuatorState** / **ComponentStatus** / TaskEvent / OcrResult / TrackArray / TaskState / WheelEncoders)
- 创建 `vehicle_wbt_component_base/` 定义 `BaseComponent` 抽象
- 创建 `config_sensors.yml`,注册 camera + 电机 + arm + IR
- mock 节点:`mock_camera_node.py` / `mock_serial_node.py`(用 `BaseChassis.forward_kinematics`)/ `mock_inference_node.py`
- 主进程 `SidecarAdapter` 在 `ENABLE_ROS2=1` 时启动,publish Camera/CarBase/ClintInterface 数据
- **验收**: `ros2 topic list` 含所有 `/vehicle_wbt/v1/*`;rosbag2 record/play 60s 通过

### Phase 2: 硬件抽象 + URDF + Gazebo (W2)

- 写 `vehicle/controller/base_controller.py` + `mc601_adapter.py` + `mc602_adapter.py`(复用现有 `serial_wrap` / `controller_wrap`)
- 写 `vehicle/driver/chassis/base_chassis.py`,把现有 4 种 chassis 实现迁入子类
- 验证:`MyCar` 加载 Diff2 后跑通(单元测试,不动真车)
- `scripts/urdf_gen.py` 从 `config_sensors.yml` + `cfg_vehicle.yaml` + `arm_cfg.yaml` 合成 `vehicle_wbt.urdf.xacro`
- Gazebo `competition_track.sdf` + ros2_control chassis + arm controllers
- **验收**: Gazebo 30s 内显示机器人;RViz TF 树完整;ros2_control controllers list 全部 active

### Phase 3: Control Bridge + safety_gate (W3)

- `control_bridge` 订阅 `/cmd/vel_safe` → 调 `MyCar.move()` / `BaseChassis.forward(vx, vy, wz)`(**ENABLE_SIM=0 时此订阅器不创建**)
- `safety_gate` 实现 4 层闸门 + Mode 状态机 (IDLE/AUTO/MANUAL/SIM/ESTOP) + Deadman switch (250ms)
- pytest 覆盖状态机所有转移;mock safety_gate 在 mode=SIM 下拒绝外部 `/cmd/vel_safe`
- **验收**: `pytest tests/test_safety_gate.py` 100% 通过;kill -9 sidecar → 5s 内主进程 E-stop;物理急停按下立即停(不依赖 ROS)

### Phase 4: 多机验证 + 刷机部署 (W4)

- 2 台开发笔记本 + Jetson 同时订阅 `/vehicle_wbt/v1/*` 验证 DDS 发现
- WireGuard 隧道下 DDS 跨网工作
- `scripts/post_flash_setup.sh` 一键安装 ROS2 + colcon build + cyclonedds 配置
- 验证 `ENABLE_ROS2=0/1` 切换无 main/qqq.py 修改
- **验收**: 刷机后 ≤ 30 分钟 sidecar 可用

---

## 测试策略

### 仿真中测什么

| 测试类型 | 在哪跑 | 工具 | 频率 |
|---------|--------|------|:---:|
| 单元测试 (safety_gate / 状态机 / chassis FK/IK) | 笔记本 | pytest | 每次 commit |
| 集成测试 (mock + rosbag2 replay) | 笔记本 | ros2 bag + RViz | PR 前 |
| 物理仿真 (motion / collision) | 笔记本 | Gazebo + ros2_control | 每周 |
| 系统集成 (主进程 + sidecar) | Jetson 真机 | ros2 bag + rqt | W3/W4 |
| 真实机验收 | Jetson 真机 | 手动 + rosbag | 赛前 |

### Sim-to-Real Gap 清单

| 项 | Sim | Real | 风险 |
|------|-----|------|------|
| 摄像头延迟 | 0ms | 30-80ms (V4L2 + USB) | 🟡 |
| 编码器噪声 | 无 | MC602 1-2 计数抖动 | 🟡 |
| 串口往返 | 0ms | 2-5ms | 🟢 |
| ArmBase 步进电机 | 物理仿真 | 真实步进(非线性) | 🔴 — 真机标定 |
| 真空泵响应 | 理想 | 30-50ms 真空建立 | 🟡 |
| 红外测距 | 合成 | 模拟量 + 噪声 | 🟢 |
| IMU (若装) | 无噪声 | 温漂 + 振动 | 🟡 |

**必须真机标定**: arm stepper PID (`arm_cfg.yaml`)、lane-follow PID (`config_car.yml` lane_pid)。

### 真实机验收清单

W4 末必须全绿:

- [ ] **启动**: `ENABLE_ROS2=1` 启动 main/qqq.py + sidecar,5s 内 sidecar `/safety/heartbeat` 出现
- [ ] **topic 完整**: `ros2 topic list` 包含所有 §Topic Schema 中定义的 `/vehicle_wbt/v1/*` topic
- [ ] **/odom 50Hz**: `ros2 topic hz /vehicle_wbt/v1/state/odom` ≈ 50Hz
- [ ] **/camera 30Hz**: `ros2 topic hz /vehicle_wbt/v1/sensors/camera/front/image_raw` ≈ 30Hz
- [ ] **/tf 完整**: `ros2 run tf2_tools view_frames.py` 生成完整 TF 树(base_link → camera_link, arm_link)
- [ ] **rosbag 录制**: `ros2 bag record -a -o test.db3` 录制 60s 成功,文件 > 100MB
- [ ] **rosbag 回放**: `ros2 bag play test.db3` 在 RViz 显示一致
- [ ] **物理急停**: 按急停键 → 0.2s 内车速归零(不依赖 ROS)
- [ ] **sidecar 杀死**: `kill -9 $(pgrep -f vehicle_wbt_bringup)` → 5s 内主进程进入 E-stop
- [ ] **E-stop 恢复**: safety_gate 重启 → 自动恢复 AUTO 或人工 ACK
- [ ] **mode 切换**: `/safety/mode_cmd: "ESTOP"` → 立即停;`"AUTO"` → 恢复
- [ ] **deadman**: 远程 teleop 停发 → 250ms 内车速归零
- [ ] **ENABLE_ROS2=0**: `systemctl edit py_boot.service` 改 ENABLE_ROS2=0 → 无 ROS 进程、无 DDS 端口、行为字节级一致
- [ ] **CLI**: `ros2 run vehicle_wbt_bringup sidecar_main` 单跑也能启动
- [ ] **Component 动态启用**: 启用 `config_sensors.yml` 中 `stepper_2`,sidecar 重启后 `/actuators/stepper/2/state` 自动出现
- [ ] **Chassis 切换**: 改 `cfg_vehicle.yaml::chassis_type=diff2`,odom 编码器数量从 4 变 2
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
| 6 | URDF 与 config_sensors.yml 不一致 | 中 | 🟡 | urdf_gen.py 是 YAML → xacro 单向; config_sensors.yml 是 source of truth, lint check in CI |
| 7 | 团队不熟悉 ROS2 ament_python | 中 | 🟡 | Phase 1 优先做 1 个最小节点跑通; 配 PR 模板 |
| 8 | ros2_control 与现有 ArmBase 步进电机协议冲突 | 中 | 🔴 | Gazebo 模式只用 ros2_control; 真机模式只用 ArmBase, 互斥 |
| 9 | 物理急停与 ROS 紧耦合 (CLAUDE.md 明确禁止) | 低 | 🔴 | 物理急停走独立线程, ROS 完全旁路; 测试用例强制验证 |
| 10 | 刷机后 ROS2 装错版本 (humble vs iron) | 中 | 🟡 | post_flash_setup.sh 固定 humble; CI 验证 |
| 11 | 多个 Component 包并行开发冲突 | 低 | 🟡 | 每个包独立 package.xml, 不共享文件; 集成只在 launch 层 |
| 12 | ZMQ REP 与 sidecar publish 资源竞争 | 低 | 🟡 | 主进程 SidecarAdapter 在独立线程, 不阻塞 motion control loop |
| 13 | DDS 发现在网络切换 (WiFi/LAN) 中断 | 中 | 🟡 | cyclonedds 配置 NetworkInterface autodetermine; 重连后自动恢复 |
| 14 | rosbag2 录制文件过大 (摄像头 + odom) | 中 | 🟡 | image_transport compressed; 录制时启动 jpeg 压缩订阅器 |
| 15 | 竞赛现场不允许带笔记本跑 RViz | — | 🔴 | RViz/Gazebo 仅在开发期用; 比赛当天 ENABLE_ROS2=0 |
| 16 | arm stepper PID 在 sim 中表现完美, real 不行 | 高 | 🔴 | Phase 4 真机标定; sim 仅做 motion 验证 |
| 17 | 加新 chassis 时改坏现有 FK/IK | 中 | 🟡 | BaseChassis 子类必须有完整 pytest 覆盖;CI 跑全部 5 种 chassis |
| 18 | 加新控制器 (MC603) 时与 ctl_id 派发冲突 | 中 | 🟡 | 新 adapter 必须实现 `probe_device_id()`,pluginlib 按 dev_id 加载,不修改 ctl_id 全局 |
| 19 | v1 namespace 升级到 v2 时旧订阅者断裂 | 中 | 🟡 | v1/v2 共存期保留双 publisher;deprecation 时间表提前公告 |

---

## 决策记录

### 决策摘要

1. **保留现有架构**: 主进程不重构 (CLAUDE.md "DO_NOT_MODIFY" 列表保持完整)
2. **sidecar 是只读观察者 + 命令路由器**: 不取代 ZMQ/串口/直接调用
3. **ROS2 Humble (LTS)**: 对应 Ubuntu 22.04 (jammy) + JetPack 6.x — Humble 支持到 2027-05
4. **DDS 用 cyclonedds**: 比 fastdds 占用更小, Jetson 友好
5. **仿真 A+C**: mock + rosbag2 (主) + Gazebo + ros2_control (最终)
6. **ROS_DOMAIN_ID=42** 真机 / **43** 仿真,避免冲突
7. **URDF 从 YAML 三件套(config_sensors.yml + cfg_vehicle.yaml + arm_cfg.yaml)生成**: YAML 是 source of truth
8. **safety_gate 4 层闸门**: 物理急停 > 心跳 > 模式 > 数据完整性
9. **ENABLE_ROS2=0 必须零影响**: CI 强制验证两套行为一致
10. **物理急停永不经 ROS**: 后台线程独立路径, 测试用例覆盖
11. **Component 模型**: sidecar 是 Component 集合,不是 monolithic node
12. **插件化硬件接口**: MC601/MC602/MCxxx 都是 `BaseControllerHardwareInterface` 子类
13. **Chassis-agnostic**: `MyCar` 不感知 chassis 类型,只调用 `BaseChassis` 接口
14. **versioned namespace**: `/vehicle_wbt/v1/`,v2 升级时 v1 保留
15. **reserved/aux namespaces**: 未用硬件不报错,只占 topic 不占代码路径

### 关联文档

| 文档 | 链接 | 状态 |
|------|------|:---:|
| ADR-001 ROS Noetic 集成方案 | [../adr/ADR-001-ros-noetic-integration.md](../adr/ADR-001-ros-noetic-integration.md) | 提议中(与本文互补, 本文是其 ROS2 演化方向) |
| ADR-002 Python 环境管理 | [../adr/ADR-002-python-environment.md](../adr/ADR-002-python-environment.md) | 提议中 |
| ADR-003 ROS2 Sidecar 集成 | [../adr/ADR-003-ros2-sidecar-integration.md](../adr/ADR-003-ros2-sidecar-integration.md) | 提议中 |
| branch-strategy.md | [../contributing/branch-strategy.md](../contributing/branch-strategy.md) | 已写 |
| jetpack6-ros2-humble.md | [../migration/jetpack6-ros2-humble.md](../migration/jetpack6-ros2-humble.md) | 已写 |
| hardware-port-mapping.md | [../hardware-port-mapping.md](../hardware-port-mapping.md) | **已写 (PR #5 合并)** — 物理端口参考 |
| ros-analysis.md | [../ros-analysis.md](../ros-analysis.md) | 已写(本文与其一致的"不迁移 ROS2"立场**相反**: 本文是 **不迁移但 sidecar** 的折中方案, 因为有仿真和远程调试需求) |
| known-issues.md | [../known-issues.md](../known-issues.md) | 必读(eval/裸 except/sleep 模式严禁重现) |

### 与 ADR-001 的关系

ADR-001 是 "要不要 ROS" 的决策(答: 渐进式轻量集成 — 方案 C)。**本文是 ADR-001 的细化与扩展**: 从 ROS Noetic 演进到 ROS2 Humble + sidecar 模型, 引入仿真回路与远程调试两个新需求, 不破坏 ADR-001 的核心原则(渐进式、零侵入、可回滚)。

### 决策触发器 (何时升级到 ROS2 全量 / v2)

| 触发条件 | 当前 ADR-001 立场 | 本文立场 |
|---------|:---:|:---:|
| 多机器人协调 | 不需要 | 同左 |
| SLAM / nav2 | 不需要 | 同左 |
| moveit 运动规划 | 不需要 | 同左 |
| **Gazebo 仿真 (赛后调试)** | 不需要 | **需要 → 触发 ROS2** |
| **远程多人调试** | 1-2 周足够 | **sidecar 已满足, 不升级** |
| 多摄像头同步 | 不需要 | sidecar 内 image_transport 解决 |
| 30+ 节点规模 | 当前 ~6 | sidecar 已足够 |
| **第二台机械臂 (双臂)** | 预留 | **v2: 启用 /aux/arm/2/** |
| **CSI/MIPI 摄像头** | 不需要 | **v2: 启用 /reserved/csi/** (待载板 spec) |
| **Jetson 直驱 GPIO** | 不需要 | **v2: 启用 /aux/gpio/** (需新 adapter) |
| **更多 chassis (6 轮/8 轮)** | 不需要 | **v2: 在 _CHASSIS_REGISTRY 加条目** |

### 必须先补的文档

1. `docs/hardware-port-mapping.md` — 物理端口参考(已存在)
2. `docs/contributing/branch-strategy.md` — 4 人并行包开发分支约定
3. `docs/migration/jetpack6-ros2-humble.md` — phase 1→4 任务分解
4. `docs/component-contract.md` — BaseComponent 接口契约(由 §Component 模型提炼)
5. `docs/chassis-plugin-howto.md` — 加新 chassis 步骤(由 §Chassis 抽象提炼)
6. `docs/controller-adapter-howto.md` — 加新控制器 MC603 步骤(由 §硬件接口插件提炼)

---

## 附录 A: 仓库结构 (最终)

```
/home/jetson/workspace/vehicle_wbt/
├── (现有 main/ car_wrap.py infer_cs/ vehicle/ ... 完全不动)
├── config_sensors.yml                # 新建: 传感器/执行器 Component 注册表(§配置系统)
│
├── vehicle/                          # 扩展, 不修改现有模块
│   ├── controller/                   # 新建: 控制器抽象层
│   │   ├── base_controller.py        # BaseControllerHardwareInterface
│   │   ├── mc601_adapter.py          # MC601 adapter
│   │   ├── mc602_adapter.py          # MC602 adapter
│   │   └── plugins/                  # 加新控制器加文件
│   ├── driver/
│   │   └── chassis/                  # 新建: chassis 抽象层
│   │       ├── base_chassis.py       # BaseChassis
│   │       ├── mecanum_chassis.py
│   │       ├── diff2_chassis.py
│   │       ├── diff4_chassis.py
│   │       ├── tricycle_chassis.py
│   │       └── quadricycle_chassis.py
│   └── arm/                          # 已有, 不动
│
├── ros2_ws/                          # 新建 colcon 工作空间
│   └── src/
│       ├── vehicle_wbt_msgs/         # 自定义消息 (§Topic Schema)
│       │   ├── msg/
│       │   │   ├── LaneResult.msg
│       │   │   ├── Detection.msg
│       │   │   ├── DetectionArray.msg
│       │   │   ├── TaskEvent.msg
│       │   │   ├── TaskState.msg
│       │   │   ├── ActuatorState.msg       # 通用执行器(替代 ArmState)
│       │   │   ├── ComponentStatus.msg     # 通用组件健康
│       │   │   ├── WheelEncoders.msg
│       │   │   ├── OcrResult.msg
│       │   │   ├── Track.msg
│       │   │   └── TrackArray.msg
│       │   ├── package.xml
│       │   └── setup.py
│       │
│       ├── vehicle_wbt_bringup/      # 主入口 + launch
│       │   ├── vehicle_wbt_bringup/
│       │   │   ├── sidecar_main.py   # Component 加载入口(§Component 模型)
│       │   │   └── __init__.py
│       │   ├── launch/
│       │   │   ├── vehicle_wbt.launch.py        # 真机
│       │   │   └── vehicle_wbt_sim.launch.py    # 仿真
│       │   ├── rviz/
│       │   │   └── vehicle_wbt.rviz
│       │   └── package.xml
│       │
│       ├── vehicle_wbt_component_base/  # 新建: Component 抽象基类
│       │   └── vehicle_wbt_component_base/
│       │       └── base_component.py
│       │
│       ├── vehicle_wbt_camera_node/   # Phase 1
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
│       ├── vehicle_wbt_safety_gate/   # Phase 3
│       │   └── vehicle_wbt_safety_gate/
│       │       ├── safety_gate_node.py
│       │       ├── mode_state_machine.py
│       │       └── deadman.py
│       │
│       ├── vehicle_wbt_control_bridge/ # Phase 3
│       │   └── vehicle_wbt_control_bridge/
│       │       └── control_bridge_node.py
│       │
│       ├── vehicle_wbt_sim_mock/      # Phase 1
│       │   └── vehicle_wbt_sim_mock/
│       │       ├── mock_camera_node.py
│       │       ├── mock_serial_node.py
│       │       └── mock_inference_node.py
│       │
│       └── vehicle_wbt_gz/            # Phase 2
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
├── scripts/                          # 新建辅助脚本
│   ├── urdf_gen.py                   # 从 config_sensors.yml + cfg_vehicle.yaml + arm_cfg.yaml 生成 xacro
│   └── post_flash_setup.sh           # 刷机后一键安装 ROS2
│
└── car_wrap.py / main/qqq.py         # **修改一行**:
                                      #   if os.getenv('ENABLE_ROS2') == '1':
                                      #       self._sidecar = SidecarAdapter()
                                      # 其他完全不变
```

---

## 附录 B: SidecarAdapter 主进程适配层 (骨架)

文件: `car_wrap_sidecar_adapter.py` (新文件,**不修改 car_wrap.py**)

```python
import os
import multiprocessing as mp
from multiprocessing import shared_memory
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from car_wrap import MyCar

class SidecarAdapter:
    """主进程侧 ROS2 适配层: 不替换任何现有控制路径, 仅旁路 publish 数据"""

    def __init__(self, car: 'MyCar', config: dict):
        if os.getenv('ENABLE_ROS2') != '1':
            self._enabled = False
            return
        self._enabled = True
        self._car = car
        self._component_registry = self._load_components(config)
        self._shm = self._setup_shared_memory()
        self._ros_proc = mp.Process(
            target=_sidecar_main,
            args=(self._shm.name, list(self._component_registry.keys())),
            daemon=True,
        )
        self._ros_proc.start()
        self._install_hooks()

    @staticmethod
    def _load_components(config: dict) -> dict:
        """扫描 config_sensors.yml, 实例化 Component 子类(pluginlib 风格)"""
        import importlib
        components = {}
        for entry in config.get('sensors', []) + config.get('actuators', []):
            if not entry.get('enabled', False):
                continue
            comp_type = entry['type']
            mod = importlib.import_module(f'vehicle_wbt_components.{comp_type}_component')
            comp = mod.Component(entry)
            components[entry['id']] = comp
        return components

    def _setup_shared_memory(self) -> shared_memory.SharedMemory:
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

def _sidecar_main(shm_name: str, component_ids: list) -> None:
    """sidecar 子进程入口 — 按 component_ids 启动 ROS2 节点集合"""
    import rclpy
    from vehicle_wbt_bringup.sidecar_main import build_nodes

    rclpy.init()
    nodes = build_nodes(shm_name=shm_name, component_ids=component_ids)
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

关键不变量: `car_wrap.py` 主体 0 行修改, `MyCar.__init__` 末尾追加一行 `self._sidecar = SidecarAdapter(self, cfg) if os.getenv('ENABLE_ROS2')=='1' else None`。

---

> 本 spec 是 2026-07-05 ROS2 sidecar 项目的设计基线。任何 phase 启动前的 PR 必须 review 是否仍符合 § 设计原则 / § 平台边界 / § 安全设计 / § 不可修改清单; 若偏离, 先更新本 spec 再写代码。
