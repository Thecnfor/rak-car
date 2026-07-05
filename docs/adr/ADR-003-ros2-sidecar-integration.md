# ADR-003: ROS2 Sidecar 架构 + 仿真回路（A+C 路线）

## 状态

**提议中** — 等待团队评审

> **编号说明**: 仓库内已存在 `ADR-002-python-environment.md`（Python 环境管理策略，2026-06-27）。为避免编号冲突，本 ADR 落盘时直接使用 **ADR-003**。后续如需合并 `ADR-002-python-environment.md`，可在 PR 阶段重排。

## 日期

2026-07-05

## 背景

### ADR-001 的遗留问题

ADR-001（2026-06-27，提议中）在三个候选方案中推荐了**方案 C：渐进式轻量集成**（1-2 周，ROS Noetic 作为观察者叠加在现有架构之上），并明确否决了 ROS2 迁移：

> ADR-001 原文（"不做的事"小节）：
> - ❌ 不迁移到 ROS2（成本高，收益不明确）
> - ❌ 不迁移到 ROS2（成本高，收益不明确）

ADR-001 同时给出了 ROS2 迁移触发条件：

> 未来若需多机器人协调 / SLAM 导航 / moveit / Gazebo 才考虑 ROS2。

### 触发再讨论的事件

2026-07-05 团队脑暴（4-6 人参与，1 人有 ROS2 Humble 实操经验，2 人有 Gazebo 基础认知），对 ADR-001 形成了三条新输入：

1. **赛后窗口已确定**：`docs/migration/jetpack6-ros2-humble.md` 已规划 2026-08-13（比赛后第一天）开始刷 JetPack 6.1 + 装 ROS2 Humble，缓冲期到 2026-08-25。下赛季起 ROS2 Humble 必然在场。
2. **仿真回路收益明显**：当前调试完全靠实机跑 + 草稿 print。脑暴现场演示的 Gazebo Garden + ros-humble-ros-gz 桥接能让我们在 host PC（Ubuntu 22.04，无 Jetson）离线仿真底盘 + 机械臂，1 人离线迭代 + 1 人实机验证，效率翻倍。
3. **ROS1 Noetic 已 EOL**（2025-05），JetPack 5.x + Ubuntu 20.04 也进入 ESM。即便 ADR-001 方案 C 跑起来，下赛季前也必须迁移到 ROS2，ROS1 工作是沉没成本。

### 当前架构不变性约束

参考 ADR-001 与 CLAUDE.md"DO_NOT_MODIFY / DO_NOT_BREAK"清单：

- `vehicle/base/serial_wrap.py`、`mc601_ctl2.py`、`mc602_ctl2.py`、`controller_wrap.py`（`ctl_id` 全局派发）— **不动**
- `vehicle/arm/arm_base.py`、`vehicle/driver/vehicle_base.py`（底盘运动学）— **不动**
- ZMQ 推理端口 5001-5004 + `ClintInterface` 契约 — **不动**
- `main/qqq.py` systemd 启动路径 — **不动**
- `sys.path.append` 导入约定 — **不重构**
- `import vehicle` 时的硬件副作用 — **不解决**

这些约束决定了：ROS2 必须是 **sidecar（旁挂）**，不是 in-process replacement。

### 硬件端口映射就绪

`docs/hardware-port-mapping.md` 已经记录 M1-M6 驱动电机、S2/S3/S7 舵机总线、stepper_1/stepper_3 机械臂关节、IO 引脚 P1-P8 的物理映射（来源：CLAUDE.md + `controller_wrap.py` + `vehicle/arm/arm_cfg.yaml`），可直接作为 URDF / xacro 模型的链接定义来源，无需重新勘察硬件。

## 决策

**采用方案 3（Sidecar + 仿真回路，A+C 路线）**：

- 在比赛后（2026-08-13 后）刷 JetPack 6.1 + 装 ROS2 Humble
- 现有 6 层架构（CLAUDE.md）保持不变，ROS2 作为**旁挂进程**启动一组 publisher / service 节点，把现有数据流以 ROS2 Topic / Service 形式暴露出来
- 同步部署 Gazebo Garden 仿真回路（URDF 来源：`docs/hardware-port-mapping.md`），实现"host PC 离线仿真 + Jetson 实机回归"双轨开发
- **比赛前 4 周内（2026-07-13 → 2026-08-12）冻结所有 ROS2 相关代码改动**，与 `docs/migration/jetpack6-ros2-humble.md` 时间表对齐

> ADR-001 方案 C（Noetic sidecar）作为**短期临时桥梁**继续执行至 8.10 比赛结束。比赛结束后本 ADR-003 接管，ROS Noetic 节点下线，ROS2 Humble 节点接替。ADR-001 不撤销，但其"ROS1 Noetic"路径在赛后被本 ADR 取代。

## 方案对比

### 方案 1：保持 ADR-001（ROS Noetic sidecar）至退役

```
继续执行 ADR-001 方案 C，比赛后回滚到 JetPack 5
下赛季再决定 ROS1 vs ROS2
```

| 优点 | 缺点 |
|------|------|
| 零迁移成本（已经做了） | ROS Noetic 2025.05 已 EOL，无 CVE 补丁 |
| 团队已熟悉 rospy | 比赛后必然还要迁移（Ubuntu 20.04 ESM） |
| 比赛脚本零风险 | Gazebo 不能用（gazebo_ros 未装），仿真回路收益为零 |
| | 仿真能力缺失导致下赛季仍只能 print 调试 |
| | URDF / xacro / TF2_ros 仍可用，但失去生命周期管理 |

### 方案 2：全量 ROS2 迁移（拒绝 B 方案）

```
所有层重写为 ROS2 节点，6-10 周功能冻结
```

| 优点 | 缺点 |
|------|------|
| 完整 ROS2 生态（Lifecycle / Component / QoS） | **6-10 周工作量** |
| 标准化架构 | 比赛期间功能冻结 |
| rclpy 现代化 API | 团队需要重学 rclpy / launch_xml |
| ros_gz + Nav2 + MoveIt2 完整 | 现有代码（1438 行 MyCar God Object）大改风险极高 |
| | 与 CLAUDE.md 的"DO_NOT_MODIFY"清单正面冲突 |

### 方案 3：ROS2 Sidecar + 仿真回路（A+C 路线）⭐ 推荐

```
现有 6 层架构（CLAUDE.md）不动
新增 ROS2 sidecar：节点订阅现有数据 → 发布为 ROS2 Topic / Service
新增 Gazebo Garden 仿真回路：URDF + ros_gz bridge
比赛后接管 ADR-001 的 ROS1 sidecar 工作
```

```
┌─────────────────────────────────────────────────────────────┐
│  ROS2 能力层（新增，赛后部署）                                  │
│  ─────────────────────────────────────────                  │
│  ROS2 sidecar 节点：                                          │
│    car_status_node        /odom  /joint_states  /arm/state   │
│    camera_node            /camera/front/image_raw             │
│    inference_bridge_node  /detection/results  /lane/result   │
│    rviz_markers_node      /visualization_marker               │
│    tf_broadcaster         /tf  /tf_static                     │
│    lifecycle_manager      管理节点生命周期                     │
│  ─────────────────────────────────────────                  │
│  仿真回路（host PC）：                                          │
│    Gazebo Garden (ros-humble-ros-gz)                        │
│    URDF/xacro ← docs/hardware-port-mapping.md                │
│    ros_gz bridge 模拟串口硬件                                  │
│  ─────────────────────────────────────────                  │
│  现有 6 层（CLAUDE.md，完全不动）                               │
│  main/qqq.py → MyCar → MyTask → CarBase → controller_wrap    │
│  ZMQ 推理 (5001-5004) + 串口硬件 + 直接函数调用                │
└─────────────────────────────────────────────────────────────┘
```

| 优点 | 缺点 |
|------|------|
| **现有架构零修改**（满足 CLAUDE.md DO_NOT_MODIFY） | 需要维护 ROS2 节点（赛后 ~2-3 周工作量） |
| 与 ADR-001 衔接平滑（sidecar 节点名字一一对应） | rclpy + rclcpp 学习成本（团队 1 人熟，1-2 人需培训） |
| 赛后才能部署（避免竞赛风险） | URDF 需基于 hardware-port-mapping.md 重建（半天） |
| Gazebo 仿真回路可在 host PC 离线运行 | PaddlePaddle / erniebot 在仿真中不跑（只跑运动学） |
| 双轨开发：1 人离线仿真 + 1 人实机验证 | 仿真 ≠ 实机（电机响应、相机噪声、机械臂间隙） |
| 渐进式：sidecar 节点可逐个替换 ADR-001 的 ROS1 节点 | |
| 保留回滚：删除 ROS2 节点即可回到纯 Python | |

## 详细设计

完整规范见 `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md`。本节总结关键架构决策。

### Sidecar 进程模型

ROS2 节点作为**独立进程组**启动，与现有 `main/qqq.py` 通过**共享内存 + 共享文件**解耦：

```
Jetson 上同时跑两个进程组：
┌─────────────────────────────────────────┐
│  进程组 A（现有，不动）                     │
│  systemd py_boot.service → main/qqq.py   │
│  └─ MyCar (1438 lines)                   │
│     ├─ CarBase                           │
│     ├─ ArmBase                           │
│     ├─ Camera daemon → /tmp/cam_front.jpg │
│     ├─ ClintInterface → ZMQ 5001-5004    │
│     └─ controller_wrap → /dev/ttyUSB*    │
└─────────────────────────────────────────┘
                  ↕ 共享文件 / 共享内存
┌─────────────────────────────────────────┐
│  进程组 B（新增，ROS2 sidecar）            │
│  systemd ros2_sidecar.service            │
│  └─ ros2 launch ros2_sidecar bringup.xml │
│     ├─ camera_node (read /tmp/cam_front.jpg) │
│     ├─ car_status_node (read odom dump)  │
│     ├─ inference_bridge_node (proxy ZMQ)  │
│     ├─ tf_broadcaster                    │
│     ├─ rviz_markers_node                 │
│     └─ lifecycle_manager                 │
└─────────────────────────────────────────┘
```

**关键：sidecar 节点从不写入现有进程**，只读 + 通过 ROS2 Topic 发布出去。即便 ROS2 崩了，`main/qqq.py` 完全无感知。

### 共享数据接口（sidecar → 现有）

| 数据 | 现有存储位置 | sidecar 读取方式 |
|------|--------------|------------------|
| 摄像头帧 | `/tmp/cam_front.jpg` (Camera daemon 写入) | sidecar 节点读取文件 + cv_bridge 转 sensor_msgs/Image |
| 里程计 | `CarBase.odom` 字段 | sidecar 周期 dump 到 `/tmp/odom.jsonl`，节点 tail -F |
| 机械臂状态 | `ArmBase.angles` 字段 | sidecar 节点读 `/tmp/arm_state.json` |
| 检测结果 | ZMQ 5002 端口 | sidecar 节点自己起 ZMQ subscriber，不干扰 ClintInterface |
| 车道线结果 | ZMQ 5001 端口 | 同上 |
| TF 变换 | 计算 | sidecar 节点读机械臂关节角 + 底盘位姿，自行计算 TF |

### 仿真回路（Gazebo Garden）

部署在 host PC（Ubuntu 22.04，非 Jetson）：

```
host PC:
  ros-humble-desktop
  ros-humble-ros-gz (Gazebo Garden 桥接)
  ign gazebo vehicle_wbt.sdf
      ↑ ros_gz bridge
  ros2 topic pub /cmd_vel geometry_msgs/Twist ...
      ↓ ros2 topic echo /odom nav_msgs/Odometry
  rviz2 (可视化)
```

URDF 来源：`docs/hardware-port-mapping.md` 已记录 M1-M6 / S2-S3-S7 / stepper_1 / stepper_3 / 真空泵 / 红外 / IO 引脚 P1-P8 的物理映射，xacro 模型直接基于此构建：

```xml
<!-- urdf/vehicle_wbt.urdf.xacro -->
<robot name="vehicle_wbt">
  <link name="base_link"/>
  <link name="arm_link"/>      <!-- stepper_1 + stepper_3 -->
  <link name="camera_link"/>   <!-- /dev/cam0 安装位置 -->
  <joint name="arm_joint" type="revolute">...</joint>
  <!-- 详见 spec 文档 §4.2 -->
</robot>
```

仿真只跑**运动学 + 视觉**（OpenCV 注入合成图像），不跑 PaddlePaddle / erniebot（这两个仍然在 Jetson 上跑实机）。仿真目标：机械臂轨迹验证 + 底盘运动学 PID 调参 + 摄像头视野估算。

### 主题列表（与 ADR-001 对齐命名空间）

| Topic | Type | 发布节点 | 频率 |
|-------|------|----------|------|
| `/camera/front/image_raw` | sensor_msgs/Image | camera_node | 10 Hz (限频) |
| `/camera/side/image_raw` | sensor_msgs/Image | camera_node | 10 Hz |
| `/odom` | nav_msgs/Odometry | car_status_node | 30 Hz |
| `/joint_states` | sensor_msgs/JointState | car_status_node | 30 Hz |
| `/arm/state` | ros2_sidecar_msgs/ArmState | car_status_node | 10 Hz |
| `/detection/results` | ros2_sidecar_msgs/DetectionArray | inference_bridge_node | 5 Hz |
| `/lane/result` | ros2_sidecar_msgs/LaneResult | inference_bridge_node | 20 Hz |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | tf_broadcaster | 30 Hz |
| `/visualization_marker` | visualization_msgs/Marker | rviz_markers_node | 10 Hz |

### 安全约束（与 CLAUDE.md 一致）

- **sidecar 节点不得调用任何 `controller_wrap` 写方法**（电机 / 舵机 / 步进 / 真空泵 / IO）。违反此约束将破坏现有运动控制不变量。
- **sidecar 节点不得改写 `/tmp/cam_front.jpg` 等共享文件**。
- **sidecar 节点写频率受 rclpy QoS 限频**：图像 10 Hz、状态 30 Hz、检测 5 Hz（避免拖慢 Jetson）。
- **Gazebo 仿真回路默认在 host PC**，不在 Jetson 上跑（Gazebo 资源消耗大）。
- **systemd 服务分层**：`py_boot.service`（现有，依赖 `infer_back_end.service`）+ `ros2_sidecar.service`（新增，独立启动，ros2_sidecar.service 不依赖 py_boot.service，反之亦然）。

### 回滚机制

sidecar 是纯附加层，回滚只需：

```bash
sudo systemctl stop ros2_sidecar.service
sudo systemctl disable ros2_sidecar.service
# main/qqq.py 与所有现有代码零修改
```

Gazebo 仿真回路在 host PC，与 Jetson 完全隔离，回滚 = `apt remove ros-humble-ros-gz`。

## 实施计划

### 时间表（与 jetpack6-ros2-humble.md 对齐）

| 日期 | 里程碑 | 负责 |
|------|--------|------|
| 2026-07-05 | 本 ADR 评审 + spec 文档定稿 | 全员 |
| 2026-07-05 → 08-10 | **冻结 ROS2 代码改动**（专注比赛） | — |
| 2026-08-10 → 08-12 | 比赛（ROS2 工作继续冻结） | — |
| 2026-08-13 | dd 备份 Jetson + SDK Manager 烧 JetPack 6.1 | Thecnfor |
| 2026-08-14 | 装 ROS2 Humble + 项目 pip 依赖 | Thecnfor |
| 2026-08-15 | 还原 systemd service + 冒烟测试 | Thecnfor |
| 2026-08-16 → 08-18 | ros2_sidecar 包骨架 + 6 个节点最小可用版 | ROS2 熟手 1 人 |
| 2026-08-19 → 08-22 | URDF/xacro + Gazebo Garden + ros_gz bridge | 仿真 1 人 |
| 2026-08-23 → 08-25 | rviz2 验证 + rosbag 工作流 + 团队培训 | 全员 |
| 2026-08-25 | 里程碑：与 ADR-001 ROS1 sidecar 命名空间对齐 | 全员 |
| 2026-09-01 | ROS1 sidecar 下线（如果已部署），纯 ROS2 sidecar 接管 | Thecnfor |

### Phase 1：基础设施（赛后第 3-6 天）

```
ros2_sidecar/
├── src/
│   └── ros2_sidecar_nodes/
│       ├── package.xml
│       ├── setup.py
│       ├── ros2_sidecar_nodes/
│       │   ├── camera_node.py            # 读 /tmp/cam_*.jpg
│       │   ├── car_status_node.py        # tail -F /tmp/odom.jsonl
│       │   ├── inference_bridge_node.py  # ZMQ subscriber
│       │   ├── tf_broadcaster.py
│       │   ├── rviz_markers_node.py
│       │   └── lifecycle_manager.py
│       └── msg/
│           ├── Detection.msg
│           ├── LaneResult.msg
│           └── ArmState.msg
├── launch/
│   └── bringup.xml
├── systemd/
│   └── ros2_sidecar.service
├── urdf/
│   └── vehicle_wbt.urdf.xacro            # 来源: hardware-port-mapping.md
└── rviz/
    └── vehicle_wbt.rviz
```

### Phase 2：Gazebo 仿真回路（赛后第 7-10 天）

```
host PC:
  ign gazebo worlds/empty.sdf -r
  ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist
  ros2 launch ros2_sidecar sim_bringup.xml
  rviz2 -d rviz/sim.rviz
```

### Phase 3：迁移 ADR-001 ROS1 sidecar（赛后第 11-14 天）

| ADR-001 ROS1 节点 | ADR-003 ROS2 节点 | 迁移方式 |
|-------------------|-------------------|----------|
| `ros_bridge/nodes/camera_node.py` | `camera_node.py` | 翻译 `rospy` → `rclpy`，API 对齐 |
| `ros_bridge/nodes/car_status_node.py` | `car_status_node.py` | 同上 |
| `ros_bridge/nodes/inference_node.py` | `inference_bridge_node.py` | rospy Service → rclpy Service |
| `ros_bridge/nodes/tf_broadcaster.py` | `tf_broadcaster.py` | `tf2_ros` API 基本相同 |
| `ros_bridge/nodes/rviz_markers_node.py` | `rviz_markers_node.py` | markers 同构 |

### 并行工作（脑暴会议结论）

4 人并行（2-3 周内可完成 Phase 1 + 2）：

| 人员 | 责任 |
|------|------|
| Thecnfor | 刷机 + systemd service + 冒烟测试 |
| 成员 A（ROS2 熟手）| 6 个 sidecar 节点 |
| 成员 B（仿真经验）| URDF + Gazebo Garden + ros_gz |
| 成员 C/D | 验收测试 + rviz 配置 + rosbag workflow 文档 |

> 单 Jetson 测试目标：节点只在 Jetson 上跑集成测试。Gazebo 在 host PC 跑，避免 Jetson 资源竞争。

## 风险与缓解

| 风险 | 概率 | 影响 | 缓解措施 |
|------|:---:|:---:|---------|
| ROS2 节点写共享文件与 Camera daemon 冲突 | 低 | 🔴 | sidecar 节点**只读**，且用 `O_RDONLY` open + `inotify` 监听文件 mtime 而非直接 read+write |
| rclpy QoS 不匹配 subscriber | 中 | 🟡 | sidecar 节点默认 `SensorDataQoS`（latest 模式），文档明确 |
| JetPack 6.1 烧录失败（线缆 / 断电） | 中 | 🔴 | 备份 eMMC 镜像 dd 写回 5-10 分钟恢复（jetpack6-ros2-humble.md §风险与回滚） |
| Gazebo Garden 与 PaddlePaddle CUDA 冲突 | 低 | 🟡 | 仿真在 host PC（无 CUDA），不冲突 |
| PaddlePaddle 2.6 在 Python 3.10 wheel 缺失 | 中 | 🔴 | 改 conda-forge 或源码编译（jetpack6-ros2-humble.md 已记录） |
| systemd 服务路径变更 | 低 | 🟡 | backup py_boot.service 直接还原 |
| 团队 rclpy 经验不足导致 Phase 1 延期 | 中 | 🟡 | 1 人熟手主导 + 2 人跟随；推迟仿真回路（Phase 2）优先保证 sidecar |
| ROS2 节点误调用 controller_wrap 写方法 | 极低 | 🔴🔴 | 代码 review checklist 强制 + 单测覆盖（sidecar 节点不得 import 任何写类） |
| Gazebo 仿真与实机响应差异导致误导 | 高 | 🟡 | 仿真结果只用于初版验证，最终上 Jetson 实机回归 |
| ros2_sidecar.service 与 py_boot.service 启动顺序错位 | 低 | 🟡 | systemd `After=` + `Wants=` 依赖，且两服务互不依赖（设计原则） |

## 不做的事

- ❌ 不把 ZMQ 推理迁移到 ROS2 Service（ZMQ 已稳定，迁移收益不抵风险）
- ❌ 不把 `/cmd_vel` 作为运动控制入口（CLAUDE.md DO_NOT_BREAK：现有运动控制仍走直接函数调用，ROS2 只读不写）
- ❌ 不用 ROS2 launch 替换 systemd（双 systemd 并存：py_boot.service + ros2_sidecar.service）
- ❌ 不在 Jetson 上跑 Gazebo（资源不足，仿真在 host PC）
- ❌ 不迁移 PaddlePaddle 推理路径（仿真只跑运动学，Jetson 跑实机）
- ❌ 不在比赛前 4 周内（2026-07-13 → 08-12）写任何 ROS2 代码（与 jetpack6-ros2-humble.md 时间表对齐）
- ❌ 不动 `vehicle/base/serial_wrap.py` / `controller_wrap.py` / `vehicle/driver/vehicle_base.py` / `vehicle/arm/arm_base.py`（CLAUDE.md DO_NOT_MODIFY）
- ❌ 不重写 MyCar 的 1438 行 God Object（属于另一个 ADR 议题）

## 回滚方案

sidecar 是纯附加层，回滚零成本：

```bash
# Jetson 上
sudo systemctl stop ros2_sidecar.service
sudo systemctl disable ros2_sidecar.service
rm -rf /opt/ros/humble  # 完全卸载 ROS2
# main/qqq.py 与所有现有代码零修改

# host PC 上
sudo apt remove ros-humble-*
rm -rf ~/vehicle_wbt_ros2_ws
```

**回滚后系统状态**：与 ADR-001 阶段（ROS1 Noetic sidecar）或零 ROS 状态完全等价。

## 结论

| 方案 | 工作量 | 现有代码风险 | 仿真能力 | 推荐度 |
|------|:---:|:---:|:---:|:---:|
| 1: 保持 ROS1 sidecar | 0 | 无 | ❌ | ⭐⭐ |
| 2: 全量 ROS2 迁移 | 6-10 周 | 🔴 极高 | ✅ | ⭐ |
| **3: ROS2 Sidecar + 仿真** | **2-3 周** | **🟢 零** | **✅ Gazebo** | **⭐⭐⭐⭐⭐** |

**推荐方案 3（ROS2 Sidecar + 仿真回路）。** 工作量集中在赛后（2026-08-13 后），比赛前完全冻结，零现有代码修改风险。Gazebo 仿真回路带来的双轨开发能力（host PC 离线 + Jetson 实机）是下赛季效率提升的关键杠杆。URDF 直接基于 `docs/hardware-port-mapping.md` 构建，硬件映射无重复勘察成本。

ADR-001 方案 C 不撤销，但其 ROS Noetic 路径在赛后被本 ADR 取代。ADR-001 方案 C 的 sidecar 节点代码可作为本 ADR-003 的起点（API 一对一翻译，命名空间对齐），避免双倍工作量。

**下一步行动**：
1. 本周内（2026-07-05 → 07-10）团队评审本 ADR 与 `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md`
2. 编号冲突解决：已采用 ADR-003 编号（ADR-002 由 Python 环境 ADR 占用）。`ADR-002-python-environment.md` 后续可在团队评审时重排
3. 2026-07-13 前完成评审与决议
4. 2026-08-13 起按时间表执行