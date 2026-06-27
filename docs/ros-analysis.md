# ROS 分析与 ROS2 迁移评估

## 当前 ROS 状态

### 系统安装了 ROS Noetic，但项目完全不用

```
系统环境:
  ROS_DISTRO = noetic
  /opt/ros/noetic/ 存在
  rospy 1.17.4, roslaunch 1.17.4, catkin 0.8.12 已安装
  colcon (ROS2 构建工具) 也已安装

项目代码:
  grep "import rospy" → 0 结果
  grep "import rclpy" → 0 结果
  *.launch 文件 → 0 个
  package.xml → 0 个
  CMakeLists.txt → 仅 paddle_jetson/base/deploy/ 中的第三方代码
```

**结论：ROS Noetic 是 JetPack 镜像预装的，本项目从未使用过 ROS。**

### 项目用什么替代了 ROS？

| ROS 功能 | 本项目的替代方案 |
|---------|---------------|
| 节点间通信 (Topic/Service) | **ZMQ REQ/REP** — `infer_cs/` 推理服务 |
| 进程管理 (roslaunch) | **systemd** — `py_boot.service` |
| 参数服务器 (rosparam) | **YAML 文件** — `config_car.yml` 等 |
| TF 坐标变换 | **自写里程计** — `vehicle_base.py` 中的 `OdometryBase` |
| 消息定义 (msg/srv) | **JSON** — 推理结果用 JSON 编码 |
| 包管理 (catkin) | **sys.path.append** — 手动管理路径 |
| 日志 (roslog) | **自写 logger** — `log_info/log_wrap.py` |
| 机器人模型 (URDF) | **无** — 直接写死参数 |

### 通信架构对比

```
ROS 方式:
  Node A --[Topic]--> Node B --[Topic]--> Node C
  (发布/订阅，松耦合，支持多对多)

本项目方式:
  主进程 --[ZMQ REQ/REP]--> InferServer (4 个端口)
  主进程 --[串口]--> MC601/MC602 控制器
  主进程 --[直接函数调用]--> MyTask / ArmBase
  (请求/应答，紧耦合，一对一)
```

---

## 要不要迁移到 ROS2？

### 不建议迁移的理由

| 理由 | 详细说明 |
|------|---------|
| **项目性质** | 竞赛机器人，不是通用机器人平台。功能固定，不需要 ROS 的松耦合 |
| **当前架构够用** | ZMQ 推理 + 串口通信 + 直接函数调用，简单可靠 |
| **迁移成本巨大** | 需要重写：推理服务(4个)、硬件驱动层、运动学、任务系统 |
| **无多机需求** | 单机器人，不需要 ROS2 的分布式通信 |
| **无导航栈需求** | 不用 move_base / SLAM / nav2，纯 PID 跟随 |
| **无仿真需求** | 不用 Gazebo / RViz，直接实物调试 |
| **竞赛时间压力** | 迁移期间无法比赛，风险极高 |
| **团队学习成本** | ROS2 的 DDS / QoS / 生命周期管理 学习曲线陡峭 |

### ROS2 能带来什么好处？

| 好处 | 对本项目的价值 |
|------|:---:|
| 标准化通信 (DDS) | ⭐ — 当前 ZMQ 够用 |
| 组件复用 (nav2, moveit) | ⭐ — 不用导航和运动规划 |
| 生命周期管理 | ⭐⭐ — 节点状态监控，但 systemd 也能做 |
| 多机器人协调 | — — 不需要 |
| 实时性 | ⭐ — 当前延迟可接受 |
| 社区/工具链 | ⭐⭐ — 调试工具更好，但不是刚需 |
| 传感器驱动复用 | ⭐ — 摄像头已经自己封装了 |

### 如果真要迁移，怎么做？

**不建议，但如果决定迁移，以下是路线图：**

```
Phase 1: 基础设施 (1-2 周)
├── 安装 ROS2 Humble (对应 JetPack 6.x) 或 Iron
├── 创建 vehicle_wbt ROS2 包 (ament_python)
├── 定义消息类型 (msg/): Bbox.msg, LaneResult.msg, ArmCommand.msg
└── 设置 colcon 工作空间

Phase 2: 通信层迁移 (2-3 周)
├── infer_cs/ → ROS2 Service 或 Action
│   ├── /inference/lane (sensor_msgs/Image → LaneResult)
│   ├── /inference/task (sensor_msgs/Image → BboxArray)
│   ├── /inference/front (sensor_msgs/Image → BboxArray)
│   └── /inference/ocr (sensor_msgs/Image → OCRResult)
├── 串口通信 → 硬件驱动节点
│   └── /motor_driver (geometry_msgs/Twist → 电机指令)
└── 传感器 → 传感器节点
    └── /infrared, /encoder, /button

Phase 3: 运动控制 (1-2 周)
├── CarBase → ROS2 节点，订阅 /cmd_vel
├── 里程计 → nav_msgs/Odometry 发布
└── PID 控制 → controller_manager 集成

Phase 4: 任务层 (2-3 周)
├── MyTask → ROS2 Action (长时间任务)
├── 竞赛脚本 → launch 文件编排
└── AI 集成 → 独立服务节点

Phase 5: 工具链 (1 周)
├── RViz2 可视化
├── rosbag2 数据录制
└── launch 文件替代 systemd
```

**估计总工作量：6-10 周（1人全职），期间功能冻结。**

### 替代方案：渐进式改进

与其全量迁移到 ROS2，不如渐进式改进当前架构：

| 改进项 | 工作量 | 收益 |
|--------|:---:|------|
| 推理服务加错误恢复 | 2 天 | 可靠性大幅提升 |
| 拆分 MyCar God Object | 3 天 | 可维护性提升 |
| 统一配置管理 | 1 天 | 调参效率提升 |
| 删除草稿文件 | 半天 | 代码库清晰 |
| 加日志和监控 | 2 天 | 调试效率提升 |
| ZMQ 改为 PUB/SUB 模式 | 1 天 | 支持多客户端 |
| **总计** | **~10 天** | **接近 ROS2 的核心收益，零迁移成本** |

---

## 总结

| 问题 | 回答 |
|------|------|
| 项目用了 ROS 吗？ | **没有。** 系统装了 ROS Noetic，但项目一行 ROS 代码都没有 |
| 刷机后需要装 ROS 吗？ | **不需要。** 项目不依赖 ROS，刷机后可以不装 |
| 要不要迁移到 ROS2？ | **不建议。** 成本高收益低，渐进式改进更划算 |
| 什么时候该考虑 ROS2？ | 如果未来需要：多机器人协调、SLAM 导航、moveit 运动规划、Gazebo 仿真 |
