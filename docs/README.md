# vehicle_wbt 项目文档

> 自主机器人竞赛平台 — NVIDIA Jetson + MC601/MC602 单片机

## 文档索引

| 文档 | 内容 | 重要程度 |
|------|------|:---:|
| [architecture.md](architecture.md) | 整体架构、分层设计、依赖关系 | ⭐⭐⭐ |
| [hardware-comm.md](hardware-comm.md) | Jetson ↔ 单片机通信协议（MC601/MC602 帧格式、命令表） | ⭐⭐⭐ |
| [hardware-port-mapping.md](hardware-port-mapping.md) | 代码文件与 MC602 硬件接口对应表（M口/S口/P口/步进电机） | ⭐⭐⭐ |
| [vehicle-system.md](vehicle-system.md) | 底盘运动学、里程计、麦轮/差速/三轮运动学公式 | ⭐⭐⭐ |
| [arm-system.md](arm-system.md) | 机械臂子系统、步进电机、舵机、真空泵 | ⭐⭐ |
| [inference-system.md](inference-system.md) | ZMQ 推理服务、模型部署、检测/分割/OCR 流水线 | ⭐⭐⭐ |
| [camera-system.md](camera-system.md) | 摄像头采集、线程模型、设备约定 | ⭐⭐ |
| [llm-integration.md](llm-integration.md) | 大模型集成（ErnieBot/DeepSeek/Qwen）、Prompt 工程 | ⭐⭐ |
| [task-system.md](task-system.md) | 任务原语 MyTask、竞赛脚本组织、任务流程 | ⭐⭐⭐ |
| [qqq-control-layers.md](qqq-control-layers.md) | qqq.py 机器控制层次分析（6 层架构自底向上） | ⭐⭐⭐ |
| [config-reference.md](config-reference.md) | 所有配置文件详解、参数说明 | ⭐⭐ |
| [file-structure.md](file-structure.md) | 目录结构、文件清单、可删除文件 | ⭐ |
| [known-issues.md](known-issues.md) | 已知 Bug、安全问题、技术债务 | ⭐⭐⭐ |
| [system-environment.md](system-environment.md) | 系统环境依赖、刷机恢复指南、udev 规则、systemd 服务 | ⭐⭐⭐ |
| [ros-analysis.md](ros-analysis.md) | ROS 状态分析、ROS2 迁移评估、替代方案 | ⭐⭐ |
| [branch-strategy.md](contributing/branch-strategy.md) | 分支策略与协作工作流（main / develop/ros2-sidecar） | ⭐⭐⭐ |
| [jetpack6-ros2-humble.md](migration/jetpack6-ros2-humble.md) | JetPack 6 / Ubuntu 22.04 / ROS2 Humble 迁移计划（赛后） | ⭐⭐ |
| [2026-07-05-ros2-sidecar-design.md](superpowers/specs/2026-07-05-ros2-sidecar-design.md) | ROS2 Sidecar 架构完整设计 spec（A+C 仿真路线） | ⭐⭐⭐ |

### 开发工作流（Dev 桌面 + Jetson Orin 双机架构）

| 文档 | 内容 | 重要程度 |
|------|------|:---:|
| [development/README.md](development/README.md) | 双机开发架构总览（thick client + thin server） | ⭐⭐⭐ |
| [development/dev-machine-setup.md](development/dev-machine-setup.md) | 桌面开发机 ROS2 全安装（Jazzy/Humble 容器或裸机） | ⭐⭐⭐ |
| [development/jetson-target-setup.md](development/jetson-target-setup.md) | Jetson Orin Nano 4GB 最小安装（Humble base） | ⭐⭐⭐ |
| [development/ssh-workflow.md](development/ssh-workflow.md) | SSH 到 orin + 文件同步 + 远程 build | ⭐⭐⭐ |
| [development/test-matrix.md](development/test-matrix.md) | dev vs target 测试分工矩阵 | ⭐⭐⭐ |
| [development/no-hw-dev.md](development/no-hw-dev.md) | 无真机开发（dev-sidecar stub 节点） | ⭐⭐⭐ |
| [development/lan-rviz-camera.md](development/lan-rviz-camera.md) | LAN 上 RViz 看 Jetson cameras（DDS / CycloneDDS） | ⭐⭐⭐ |

### 团队成员 Onboarding（新成员必看）

| 文档 | 内容 | 重要程度 |
|------|------|:---:|
| [onboarding/README.md](onboarding/README.md) | 30 秒 TL;DR + 文档地图 + 工具速查 | ⭐⭐⭐ |
| [onboarding/day-one.md](onboarding/day-one.md) | Day 1 详细 7 步（装 ROS2 → onboard → 看到 cameras） | ⭐⭐⭐ |

### 运维 & 现场（Operations）

| 文档 | 内容 | 重要程度 |
|------|------|:---:|
| [operations/troubleshooting.md](operations/troubleshooting.md) | 症状速查表（**RViz 连不上 Jetson 怎么办**） | ⭐⭐⭐ |

### 架构决策记录 (ADR)

| ADR | 状态 | 决策 |
|-----|:---:|------|
| [ADR-001](adr/ADR-001-ros-noetic-integration.md) | 提议中 | 是否集成 ROS Noetic — 推荐渐进式轻量集成（方案 C） |
| [ADR-002](adr/ADR-002-python-environment.md) | 提议中 | Python 环境管理 — 推荐 venv + system-site-packages |
| [ADR-003](adr/ADR-003-ros2-sidecar-integration.md) | 提议中 | ROS2 Sidecar 架构 + 仿真回路（A+C 路线，赛后执行） |

ADR 模板：[adr/TEMPLATE.md](adr/TEMPLATE.md)

## 快速上手

```bash
# 1. 启动推理服务（必须先运行）
python infer_cs/base/infer_back_end.py

# 2. 启动主程序
python main/qqq.py

# 3. 或者通过 systemd 自动启动
sudo bash main/boot_py.sh
```

## 硬件拓扑

```
┌──────────────┐    USB/CH340    ┌──────────────┐
│   Jetson     │ ─────────────── │  MC601/MC602 │
│   (上位机)    │   串口通信       │   (下位机)    │
│              │                 │              │
│  - 摄像头 ×2 │                 │  - 电机 ×4   │
│  - GPU 推理  │                 │  - 舵机 ×N   │
│  - AI 决策   │                 │  - 传感器    │
│  - 显示屏    │                 │  - 步进电机   │
└──────────────┘                 └──────────────┘
```

## ⚠️ 不可修改的底层代码

以下文件是硬件驱动层，经过长期调试稳定，**不要修改**：

- `vehicle/base/serial_wrap.py` — 串口通信、控制器探测
- `vehicle/base/mc601_ctl2.py` — MC601 协议实现
- `vehicle/base/mc602_ctl2.py` — MC602 协议实现
- `vehicle/base/controller_wrap.py` — 统一硬件抽象（可重构，但协议细节不动）
- `vehicle/arm/arm_base.py` — 机械臂控制
- `vehicle/driver/vehicle_base.py` — 底盘运动学核心

## 可重构范围

- `car_wrap.py` — 可拆分（God Object）
- `main/*.py` — 可统一（重复代码）
- `ernie_bot/` — 可整理（接口不统一）
- `infer_cs/` — 可增强（错误恢复）
- 根目录草稿文件 — 可删除
