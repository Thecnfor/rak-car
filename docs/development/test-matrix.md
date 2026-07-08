# Test Matrix — Dev vs Target

> **明确每种测试在哪台机器上跑、为什么**。避免"在 Jetson 上跑单元测试"这种浪费 cycles 的事。

## 完整测试矩阵

| 测试类型 | Dev 桌面 | Jetson Orin | 原因 |
|----------|----------|-------------|------|
| **单元测试 - Python (pytest)** | ✅ 跑 | ❌ 不跑 | dev 有更强 CPU + 完整 pytest；Jetson cycles 宝贵 |
| **单元测试 - C++ (gtest)** | ✅ 跑（如装了 rclcpp） | ✅ 必跑 | gtest 必须在目标 ROS2 distro 跑；Jetson 装 Humble |
| **Linting (flake8 / clang-format)** | ✅ 跑 | ❌ 不跑 | 格式检查是 dev 的工作流一部分 |
| **Type checking (mypy)** | ✅ 跑 | ❌ 不跑 | 编译器无关 |
| **集成测试 - mock HW** | ✅ 跑 | ❌ 不跑 | dev 用 fake vehicle module；Jetson 跑会触发真串口 |
| **集成测试 - 真 HW** | ❌ 不跑 | ✅ 必跑 | dev 没接传感器；只有 Jetson 有真硬件 |
| **Gazebo 仿真** | ✅ 跑 | ❌ 不跑 | Gazebo 需要 2-4GB RAM + GPU，Jetson 4GB 跑不动 |
| **RViz 可视化** | ✅ 跑 | ❌ 不跑 | Jetson 没装 rviz2（4GB 内存不允许） |
| **ros2 bag record (长任务)** | ✅ 跑（开发机有 SSD 空间）| ⚠️ 只录 < 5 min | Jetson SSD 184G 宝贵；长时间录 dev 上做 |
| **ros2 bag replay** | ✅ 跑 | ⚠️ 偶尔跑 | 调试时 dev 跑，集成验证 Jetson 跑 |
| **性能基准 (latency / jitter)** | ⚠️ 仅供参考 | ✅ 必跑 | 真硬件性能数据才有意义 |
| **真硬件控制指令 (MANUAL mode)** | ❌ 不可 | ✅ 必跑 | 安全：dev 不能直接控制 Jetson 电机 |
| **systemd 服务 (`py_boot.service`)** | ❌ 不跑 | ✅ 必跑 | dev 是桌面机，无 systemd 启动目标 |
| **多机 DDS 发现** | ✅ 跑 | ✅ 跑 | 必须两台都参与 |

---

## 详细说明

### ✅ dev 跑（不需要 Jetson）

```bash
# Python 单元测试
cd ~/work/rak-car/ros2_ws/src/vehicle_wbt_platform
PYTHONPATH=. python3 -m pytest test/ -v
# → 0.07s 跑完 45 tests

# C++ 单元测试 (在 dev Jazzy 容器内)
cd ~/work/rak-car/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select vehicle_wbt_platform_cpp
# → gtest 跑 ~ 25 cases

# Lint
cd ~/work/rak-car
flake8 ros2_ws/src/vehicle_wbt_platform/vehicle_wbt_platform/  # Python
clang-format --dry-run --Werror ros2_ws/src/vehicle_wbt_platform_cpp/  # C++

# Gazebo 仿真
cd ~/work/rak-car/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch vehicle_wbt_platform_cpp gz_sim.launch.py

# RViz
ros2 run rviz2 rviz2 -d config/sidecar.rviz
```

### ✅ Jetson 跑（必须真硬件）

```bash
ssh xrak@192.168.3.69
cd ~/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# 真实传感器发布 (代替真硬件集成测试)
export ROS_DOMAIN_ID=42
ros2 launch vehicle_wbt_platform_cpp vehicle_wbt_platform.launch.py
# → Jetson 跑 sidecar 节点, 发布 /vehicle_wbt/v1/sensors/ir/left 等
# → dev 端 RViz 订阅这些 topic 看到真实数据

# 性能基准 (latency 测量)
ros2 topic delay /vehicle_wbt/v1/state/odom
ros2 topic hz /vehicle_wbt/v1/sensors/camera/front/image_raw

# MANUAL mode 真硬件控制
# 物理按钮 3 切到 MANUAL, dev 上:
ros2 topic pub /vehicle_wbt/v1/cmd/vel_safe geometry_msgs/Twist "{...}"
# → Jetson 接收并驱动电机
```

### ⚠️ 偶尔跑（看情况）

```bash
# 短时 ros2 bag (1-2 分钟)
ssh xrak@192.168.3.69
ros2 bag record -a -o debug_run -b 100  # 100 MB 限制
# → 拷到 dev 上回放
rsync -avz xrak@192.168.3.69:~/ros2_ws/debug_run/ ~/work/debug_run/
# dev 上:
ros2 bag play debug_run/  # RViz 重放

# 集成测试 (端到端)
ssh xrak@192.168.3.69
cd ~/ros2_ws && colcon test --packages-select vehicle_wbt_platform_cpp --event-handlers console_direct+
# → gtest 在 Jetson 跑,验证 rclcpp 节点正常工作
```

### ❌ 不要跑

```bash
# ❌ 在 Jetson 上跑 Python 单元测试 (浪费)
ssh xrak@192.168.3.69 "cd ~/ros2_ws/src/vehicle_wbt_platform && PYTHONPATH=. python3 -m pytest test/"
# 错误理由: Jetson 跑得一样慢(甚至更慢),但占用了真机 cycles
# 正解: dev 跑,Jetson 不跑

# ❌ 在 Jetson 上装 RViz (内存不够)
ssh xrak@192.168.3.69 "sudo apt install -y ros-humble-rviz2"
# 错误理由: rviz2 + ogre + gl 至少 1.5GB,Jetson 4GB 跑会频繁 OOM
# 正解: dev 桌面跑 RViz,通过 DDS 订阅 Jetson 的 topic

# ❌ 在 Jetson 上跑 Gazebo
ssh xrak@192.168.3.69 "sudo apt install -y ros-humble-ros-gz"
# 错误理由: Gazebo + physics + rendering 至少 2-4GB RAM + 独立 GPU
# 正解: dev 桌面跑 Gazebo,Jetson 不参与仿真
```

---

## CI 集成（未来）

未来加 GitHub Actions 时（推 `develop/ros2-sidecar` 时自动跑）：

| CI Job | Runner | 跑什么 |
|--------|--------|-------|
| `lint` | ubuntu-latest | flake8 + clang-format + mypy |
| `unit-pytest` | ubuntu-latest + Jazzy container | pytest 45 cases |
| `unit-gtest` | ubuntu-latest + Humble container | colcon test 25 gtest cases |
| `integration-mock` | ubuntu-latest + Jazzy + Humble 容器互通 | DDS 多机 mock 集成 |

Jetson **不进 CI**（4GB 内存跑 CI job 太慢、太贵）。Jetson 只在 PR 合 main 后做"真硬件冒烟测试"。

---

## 日常 dev 工作流（推荐节奏）

```
┌──────────────────────────────────────────────────────────────┐
│  本地开发 (dev 容器)                                          │
│  ┌────────────────────────────────────────────────────────┐  │
│  │ 1. 改代码                                                │  │
│  │ 2. pytest (0.07s)                                       │  │
│  │ 3. (C++ 改) gtest (~5s)                                  │  │
│  │ 4. flake8 / clang-format / mypy                          │  │
│  │ 5. (集成) Gazebo + RViz                                  │  │
│  └────────────────────────────────────────────────────────┘  │
│                          ↓ 一切通过 commit 前再 push           │
├──────────────────────────────────────────────────────────────┤
│  git push origin develop/ros2-sidecar                          │
├──────────────────────────────────────────────────────────────┤
│  真硬件验证 (Jetson)                                          │
│  ┌────────────────────────────────────────────────────────┐  │
│  │ ssh xrak@192.168.3.69 "cd ~/ros2_ws && git pull"                      │  │
│  │ ssh xrak@192.168.3.69 "colcon build --packages-up-to vehicle_wbt_*"   │  │
│  │ ssh xrak@192.168.3.69 "ros2 launch ... vehicle_wbt_platform.launch.py"│  │
│  │ dev 上: RViz 看真实数据                                  │  │
│  │ ssh xrak@192.168.3.69 "ros2 topic hz ..."  # 性能基准                  │  │
│  └────────────────────────────────────────────────────────┘  │
│                          ↓ 都通过                            │
├──────────────────────────────────────────────────────────────┤
│  PR → main → 比赛 (2026-08-10)                                 │
└──────────────────────────────────────────────────────────────┘
```

**核心原则**：dev 跑 80%（开发、测试、仿真、可视化），Jetson 跑 20%（真硬件冒烟、性能测量、生产部署）。**dev 上看不到真硬件时，Jetson 上看不到 UI**。
