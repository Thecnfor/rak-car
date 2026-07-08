# No-Hardware Development Workflow

> 在没有 Jetson 真机的情况下，用 dev 桌面机 + URDF + dev-sidecar stub 节点
> 进行完整的端到端开发。**stub 节点不是 production code 的 mock**——它们是
> 仅存在于 `dev_all.launch.py` 启动路径里的开发辅助节点，**没有 production
> 触发路径**。

## 与新 CLAUDE.md "no mocks in production" 规则的关系

新 CLAUDE.md 禁止的 **production code 里的 mock** 指：
- `camera_node` 等生产节点拿不到真实设备时，**静默 publish 合成帧 / 看似真实的假数据**
- 这是欺骗上层消费者（包括 AI 模型）

dev-sidecar stub 是**完全不同**的事：
- stub 节点**只在 `dev_all.launch.py` 启动时存在**，跟 `full_system.launch.py` 完全隔离
- 没有任何 production 启动路径会运行 stub 节点
- stub 数据是**显式标注的开发数据**（黑色图像、零速度），不会跟真实数据混淆
- CI / 比赛 / 真实运行都不依赖 stub

详细规则见 `memory/coding-rules-no-mocks.md`。

## 一行命令启动

```bash
bash scripts/dev.sh
```

**这会启动**：
- `robot_state_publisher` 加载 URDF（dev 上的 `/opt/ros/lyrical` 或 `/opt/ros/humble`），发布 `/tf` + `/robot_description`
- 5 个 dev-sidecar stub 节点（camera/IR/chassis/arm/safety_gate）发布 stub 数据到 `/vehicle_wbt/v1/...`
- （可选）RViz2 看到 3D 姿态

不需要 Jetson。不需要仿真器（Gazebo）。不需要真硬件。

## 你能看到的（topic 列表）

启动后跑 `ros2 topic list`，会看到：

```
/parameter_events
/robot_description
/tf
/tf_static
/vehicle_wbt/v1/cmd/vel_safe                  (sub by chassis)
/vehicle_wbt/v1/sensors/camera/front/image_raw
/vehicle_wbt/v1/sensors/camera/side/image_raw
/vehicle_wbt/v1/sensors/ir/left
/vehicle_wbt/v1/sensors/ir/right
/vehicle_wbt/v1/state/odom
/vehicle_wbt/v1/state/actuators/main
/vehicle_wbt/v1/safety/heartbeat
/vehicle_wbt/v1/safety/estop
/vehicle_wbt/v1/safety/mode_cmd
/vehicle_wbt/v1/state/mission_progress        (if mission running)
```

## 跟它交互

### 看 odom 数据流

```bash
ros2 topic echo /vehicle_wbt/v1/state/odom
# header, pose.pose.position, pose.pose.orientation
```

### 让 chassis 移动（发 vel 命令）

```bash
ros2 topic pub --once /vehicle_wbt/v1/cmd/vel_safe geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
# 1 秒后 odom 会有 x=0.5, theta=0.3
```

### 切换模式（启用 safety_gate）

```bash
ros2 topic pub --once /vehicle_wbt/v1/safety/mode_cmd std_msgs/msg/String "{data: MANUAL}"
# 之后 /cmd/vel_safe 命令会通过；之前 mode=AUTO 会被 drop
```

### 触发物理急停

```bash
ros2 topic pub --once /vehicle_wbt/v1/safety/estop std_msgs/msg/Bool "{data: true}"
# 所有 vel 命令立刻失效（zero twist）
```

### 跑一个任务

```bash
bash scripts/dev.sh --with-mission '[seeding]'
# 启动 dev + MissionRunnerNode 跑 seeding
ros2 topic echo /vehicle_wbt/v1/state/mission_progress
# 会看到: started: 1 tasks → seeding: 1/3 → ... → completed
```

## 跑 RViz2 看 3D

```bash
# 在有 DISPLAY 的机器上（不是 ssh 远程）
bash scripts/dev.sh --with-rviz
# 自动启动 RViz2 + 加载 urdf/vehicle_wbt.rviz 配置
# 你会看到:
#   - RobotModel: 4 个麦轮 + 4 段机械臂
#   - TF: odom → base_link → 各个 wheel/arm link
#   - Image (Front/Side Camera): 黑色图片（stub data）
#   - Range (IR Left/Right): 红外 marker
#   - Path: chassis 走过的轨迹
#   - Mission Progress: 文本视图
```

## 改任务代码 → 立即见效

```bash
# 1. 改 src/seeding_task.cpp
# 2. colcon build --packages-select vehicle_wbt_platform_cpp
# 3. 重启 dev.sh（Ctrl-C 旧进程，bash scripts/dev.sh --with-mission '[seeding]'）
# 4. 改完立即生效
```

整个循环 **< 30 秒**（build 5 秒 + 重启 1 秒 + 看效果）。

## 在 CI 中跑

CI 已经有 4 个 job 验证 Phase 1.5：
- `lint-py` — Python flake8
- `test-py` — 45 个 pytest
- `test-cpp` — colcon build + gtest（base_task / safety_gate / mc602_adapter / mecanum_chassis / base_controller_iface）
- `cpp-lint` — ament_lint_common
- `xacro-check` — URDF 展开验证

**新增（待加）**：
- `test-no-hw-dev` — 在 `ros:humble-ros-base` 容器里跑 `dev_all.launch.py`，验证 5 个 dev-sidecar stub 节点都正常 publish

## 它不是什么

- **不是 Gazebo** — 没有物理仿真（无重力、无摩擦、无碰撞）
- **不是数字孪生** — chassis 不会因为发 vel cmd 真的在 3D 世界里移动（除了 odom 在 /tf 里"移动"）
- **不能验证控制算法** — 比如 PID 调参、Mecanum 力学性能
- **不能验证视觉感知** — 摄像头发的黑图，没法训练/测试模型

如果你需要以上，得上 **Gazebo + ros2_control + 真传感器模型**（Plan D 范围，2-3 周）。

## 何时用它

| 任务 | 用无真机 | 用 Gazebo | 用真机 |
|------|----------|-----------|--------|
| 加新比赛任务（seeding/harvest/...） | ✅ 主战场 | 后期校验 | 比赛前 1 天 |
| 改任务编排逻辑（mission runner） | ✅ 主战场 | ❌ | 仅性能测试 |
| 写 / 调 / 重构 ROS2 节点 | ✅ 主战场 | 后期校验 | 仅集成测试 |
| 改 URDF / TF 树 | ✅ 主战场 | 后期校验 | 仅精度测试 |
| 视觉感知 (检测 / OCR) | ❌ 需 Gazebo | 后期 | 比赛调试 |
| 控制算法 (PID / 运动学) | ⚠️ 仅逻辑 | ✅ 主战场 | 比赛调试 |
| 端到端 8.10 模拟跑 | ❌ | ✅ | 比赛当天 |

## 故障排查

| 症状 | 原因 | 解决 |
|------|------|------|
| `colcon: command not found` | 没装 colcon | `pip install colcon-common-extensions` |
| `Package 'vehicle_wbt_platform_cpp' not found` | 没 colcon build | `cd ros2_ws && colcon build --packages-up-to vehicle_wbt_platform_cpp` |
| `xacro: command not found` | 没装 xacro | `apt install ros-humble-xacro` 或 `pip install xacro` |
| RViz2 不显示 / 黑屏 | 没有 DISPLAY (ssh 远程) | 本地有 GUI 的机器跑，或 `--no-rviz` |
| 5 个节点没起来 | workspace 没 build | 看 `ros2_ws/install/` 有没有可执行 |
| `ModuleNotFoundError: vehicle_wbt_platform_cpp` | 没 source install/setup.bash | dev.sh 自动 source |

## 文件清单

```
ros2_ws/src/vehicle_wbt_platform_cpp/
├── launch/
│   ├── full_system.launch.py    # Jetson 真硬件
│   ├── mock_system.launch.py    # 5 个 dev-sidecar stub 节点（无 URDF）
│   └── dev_all.launch.py        # ⭐ 一键无真机开发（含 URDF + mission runner）
├── urdf/
│   ├── vehicle_wbt.urdf.xacro   # 14 links / 21 joints
│   ├── vehicle_wbt.rviz         # ⭐ RViz2 配置（全车概览）
│   └── README.md
└── src/
    ├── camera_node.cpp
    ├── infrared_node.cpp
    ├── mecanum_chassis_node.cpp
    ├── arm_node.cpp
    ├── safety_gate_node.cpp
    ├── mission_runner_node.cpp
    └── seeding_task.cpp

scripts/
├── dev.sh                       # ⭐ 一键启动脚本
├── start_team_rviz.sh           # 远程 team RViz 工具（远程新增）
├── calibrate_camera.py          # 相机标定工具（远程新增）
└── README.md                    # scripts/ 索引（远程新增）
```

> 📝 **注意**：`mock_system.launch.py` 文件名是历史遗留（远程已有），暂不重命名。
> 其作用是提供 dev-sidecar stub 节点，未来可考虑重命名为 `dev_stubs.launch.py`。
> **production code（`full_system.launch.py` / 各 rclcpp 节点源码）完全不依赖**
> **这个 launch 文件**——没有任何 production 路径会运行 stub 节点。

## 下一步

- **加更多任务**（pest_scout/shoot_pest/harvest 等）—— 跟 seeding_task 同模式，每个 50 行
- **写 e2e 集成测试**（launch_test）—— 自动验证 mission 跑通
- **集成到 CI**（test-no-hw-dev job）—— PR 合并前自动跑完整 dev-sidecar mission
- **可选：重命名** `mock_system.launch.py` → `dev_stubs.launch.py`（独立小 PR）