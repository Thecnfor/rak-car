# Development Workflow — Dev Machine vs Jetson Target

> **本目录定义 vehicle_wbt 项目的开发架构：thick client (dev 桌面) + thin server (Jetson Orin Nano 4GB)**。这是 ROS2 生态最成熟的开发模式（nav2 / moveit / TurtleBot 都用类似架构）。

## 架构总览

```
┌────────────────────────────────────────────────────────────────────┐
│  Dev Machine (开发机 — Ubuntu 22.04/24.04/26.04 + ROS2 desktop)    │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │ • 完整 ROS2 desktop (rclcpp + rclpy + RViz2 + ros2 bag)     │  │
│  │ • Gazebo Harmonic + ros_gz_bridge (仿真)                     │  │
│  │ • colcon / ament_cmake / ament_python / clang / gdb          │  │
│  │ • 编辑器 (VSCode / vim) + git + Docker                       │  │
│  │ • 跑测试: 单元测试 + 集成测试 (mock HW) + RViz 实时可视化    │  │
│  │ • 跑仿真: Gazebo + ros2_control + ros2 bag record/replay     │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                  │                                  │
│                                  │ ROS_DOMAIN_ID=42                  │
│                                  │ DDS over LAN (or VPN)             │
│                                  ▼                                  │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │ Target Machine: Jetson Orin Nano 4GB (JetPack 6)             │  │
│  │  • Ubuntu 22.04 + ROS2 Humble BASE (rclcpp + rclpy only)     │  │
│  │  • 4GB RAM, aarch64                                          │  │
│  │  • 跑 sidecar 节点 (PUBLISH 传感器 + 接收控制)               │  │
│  │  • 不跑 GUI/Gazebo/RViz (内存不够)                            │  │
│  │  • 不跑单元测试 (cycle 宝贵)                                  │  │
│  │  • SSH 接入: ssh xrak@192.168.3.69                            │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

## 文档索引

| 文档 | 内容 |
|------|------|
| [dev-machine-setup.md](dev-machine-setup.md) | Dev 桌面 ROS2 全安装（Jazzy/Humble + RViz + Gazebo + 工具链） |
| [jetson-target-setup.md](jetson-target-setup.md) | Jetson Orin Nano 最小安装（Humble base + 验证已装） |
| [ssh-workflow.md](ssh-workflow.md) | SSH 到 Jetson (192.168.3.69) + 文件同步 + 远程 colcon build + 远程测试 |
| [test-matrix.md](test-matrix.md) | 哪些测试在 dev 跑、哪些在 target 跑、原因 |

## 关键设计原则

1. **dev 跑 80% 的工作**：单元测试、lint、集成测试（mock HW）、仿真、可视化、调试
2. **target 只跑 2 件事**：发布真实传感器数据 + 接收控制指令（其余都让 dev 做）
3. **同一 ROS_DOMAIN_ID 通信**：dev 和 target 在同一 LAN/VPN 下用 `ROS_DOMAIN_ID=42` 自动发现，**RViz 跑在 dev 上订阅 target 的 topic**
4. **不绑死 ROS2 版本**：dev 可以 Jazzy / Humble / Kilted 任一桌面版（用于测试新功能），target 锁 Humble LTS（生产稳定性）
5. **代码一次写两边跑**：因为 ROS2 协议稳定，**同一份代码在 dev 和 target 都能跑**，区别只是 dev 用 mock / Gazebo，target 用真硬件

## 与现有 spec 的关系

本文档把 `docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §C++ 核心` 的"代码在哪里跑"问题**显式化**：
- 之前隐含假设 "sidecar 跑在 Jetson，开发者 SSH 进去调试"
- 现在改为 "sidecar 跑在 Jetson（rclcpp 节点），开发者跑 RViz/Gazebo/colcon test 在 dev 桌面，跨机器通过 DDS 通信"

## 快速链接

- 刷机相关: [../migration/jetpack6-ros2-humble.md](../migration/jetpack6-ros2-humble.md) (Jetson 当前已 JetPack 6 + Humble，**不需要刷机**)
- 分支策略: [../contributing/branch-strategy.md](../contributing/branch-strategy.md)
- 决策记录: [../adr/ADR-003-ros2-sidecar-integration.md](../adr/ADR-003-ros2-sidecar-integration.md)
