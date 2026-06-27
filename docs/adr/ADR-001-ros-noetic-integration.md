# ADR-001: 是否集成 ROS Noetic

## 状态

**提议中** — 待团队讨论决定

## 日期

2026-06-27

## 背景

当前项目状态：
- 项目是一个自主机器人竞赛平台，运行在 NVIDIA Jetson (L4T R35.3.1) 上
- 通信架构：ZMQ 推理服务（4端口）+ 串口硬件控制（MC601/MC602）+ 直接函数调用
- 项目**从未使用过 ROS**，代码中零 ROS 引用
- 系统已安装 ROS Noetic（207 个包），所有核心库可用
- ROS Noetic 是 JetPack 镜像预装的，不是团队主动安装的
- ROS2 未安装（`/opt/ros/` 下只有 noetic）
- 竞赛日期紧迫，时间是最大约束

### ROS Noetic 当前可用资源

```
已验证可用:
  ✅ rospy              — ROS Python 客户端库
  ✅ sensor_msgs.msg    — 传感器消息（Image, LaserScan, Imu...）
  ✅ geometry_msgs.msg  — 几何消息（Twist, Pose, Transform...）
  ✅ nav_msgs.msg       — 导航消息（Odometry, Path...）
  ✅ std_msgs.msg       — 标准消息（String, Int32, Float32...）
  ✅ tf / tf2_ros       — 坐标变换
  ✅ cv_bridge          — OpenCV ↔ ROS Image 转换
  ✅ rosbag             — 数据录制与回放
  ✅ roscore/roslaunch  — 节点管理
  ✅ rostopic/rosnode   — 调试工具

未安装:
  ❌ image_transport    — 图像传输优化
  ❌ move_base / nav2   — 导航栈
  ❌ moveit             — 运动规划
  ❌ gazebo_ros         — 仿真（Gazebo 本体未装）
```

## 决策

**采用渐进式轻量集成方案（混合架构）。**

不全量迁移到 ROS，而是在现有架构上叠加 ROS 能力层。

## 方案对比

### 方案 A：不集成 ROS（现状）

```
保持当前 ZMQ + 串口 + 直接调用架构
```

| 优点 | 缺点 |
|------|------|
| 零迁移成本 | 无可视化调试工具 |
| 团队已熟悉 | 无数据录制回放 |
| 竞赛脚本稳定 | 坐标变换散落在代码中 |
| 无额外依赖 | 调试全靠 print + print |

### 方案 B：全量迁移到 ROS

```
所有模块重写为 ROS 节点，通信全部走 ROS Topic/Service
```

| 优点 | 缺点 |
|------|------|
| 完整 ROS 生态 | **6-10 周工作量** |
| 标准化架构 | 竞赛期间功能冻结 |
| RViz/rqt 全套工具 | ROS1 Noetic 2025.5 EOL |
| rosbag 数据驱动开发 | 团队学习成本高 |
| 可复用 ROS 包 | 重构风险极高 |

### 方案 C：渐进式轻量集成（推荐）⭐

```
现有架构不变，叠加 ROS 发布层用于可视化和调试
```

```
┌─────────────────────────────────────────────────────┐
│  ROS 可视化层（新增）                                  │
│  机器人状态发布 → RViz2 实时可视化                      │
│  rosbag 录制 → 回放调试                                │
│  tf2 坐标广播 → 统一坐标管理                            │
├─────────────────────────────────────────────────────┤
│  现有应用层（不动）                                     │
│  main/qqq.py → MyCar → MyTask                        │
├─────────────────────────────────────────────────────┤
│  现有通信层（不动）                                     │
│  ZMQ 推理 + 串口硬件 + 直接函数调用                      │
└─────────────────────────────────────────────────────┘
```

| 优点 | 缺点 |
|------|------|
| **1-2 周工作量** | 只获得部分 ROS 能力 |
| 竞赛脚本零修改 | 需要维护两套通信（ZMQ + ROS） |
| 获得 RViz 可视化 | ROS1 不是长期方向 |
| 获得 rosbag 数据录制 | |
| 获得 tf2 坐标管理 | |
| 渐进式，可随时回退 | |

## 方案 C 详细设计

### 第一阶段：ROS 节点包装器（3-5 天）

创建一个**独立的 ROS 桥接节点**，不影响现有代码：

```
ros_bridge/
├── launch/
│   └── vehicle_wbt.launch          # 启动所有节点
├── nodes/
│   ├── car_status_node.py          # 发布车辆状态
│   ├── camera_node.py              # 发布摄像头图像
│   ├── inference_node.py           # 包装 ZMQ 推理为 ROS Service
│   ├── tf_broadcaster.py           # 广播坐标变换
│   └── rviz_markers_node.py        # RViz 可视化标记
├── msg/
│   ├── Detection.msg               # 检测结果消息
│   ├── LaneResult.msg              # 车道线结果消息
│   └── ArmState.msg                # 机械臂状态消息
└── CMakeLists.txt / package.xml
```

**关键原则：ROS 节点是观察者，不是控制者。** 现有的运动控制和任务执行仍然走直接函数调用，ROS 只负责"看"和"录"。

### 第二阶段：数据录制与回放（2-3 天）

```bash
# 录制所有传感器数据
rosbag record -a -O debug_session.bag

# 回放
rosbag play debug_session.bag

# 在 RViz 中可视化回放
rviz
```

可录制的数据流：
- `/camera/front/image_raw` — 前方摄像头
- `/camera/side/image_raw` — 侧方摄像头
- `/detection/results` — 检测结果
- `/lane/result` — 车道线结果
- `/odom` — 里程计
- `/arm/state` — 机械臂状态
- `/tf` — 坐标变换

### 第三阶段：RViz 可视化面板（2-3 天）

```
RViz 显示内容:
├── 摄像头实时画面（/camera/front, /camera/side）
├── 检测框叠加显示（/detection/markers）
├── 车道线可视化（/lane/markers）
├── 机器人位姿（/tf → base_link）
├── 里程计轨迹（/odom → nav_msgs/Path）
├── 机械臂状态（/arm/markers）
└── 任务进度（/task/status → std_msgs/String）
```

### 数据流设计

```
现有代码                          ROS 桥接节点
   │                                  │
   │ camera.read()                    │
   ├─────────────────────→ image_msg ──→ /camera/front/image_raw
   │                                  │
   │ task_det.get_infer(frame)        │
   ├─────────────────────→ bbox_list ──→ /detection/results
   │                                  │
   │ CarBase.get_pose()               │
   ├─────────────────────→ odom_msg ──→ /odom
   │                                  │
   │ task.arm.get_angle()             │
   ├─────────────────────→ joint_msg ──→ /joint_states
   │                                  │
   │                              tf_broadcaster
   │                                  │
   │                              base_link → camera_link
   │                              base_link → arm_link
```

**写入方向：现有代码 → ROS（单向）。** ROS 不控制机器人，只观察。

### rosbag 调试工作流

```
1. 正常运行竞赛脚本，同时 rosrecord 录制所有数据
2. 出现问题后，停止脚本
3. rosbag play 回放数据
4. 在 RViz 中逐步回放，定位问题帧
5. 修改参数，重新录制验证
```

这是**目前最缺的调试能力**。当前调试方式是 `print()` + 猜测。

## 实施计划

### Phase 1: 基础设施（第 1 周）

```
Day 1: 创建 ros_bridge 包结构
       写 package.xml + CMakeLists.txt
       定义自定义消息类型

Day 2: 实现 camera_node.py
       订阅 Camera.frame → 发布 sensor_msgs/Image

Day 3: 实现 car_status_node.py
       读取 MyCar 状态 → 发布 Odometry + Transform

Day 4: 实现 inference_node.py
       包装 ClintInterface → 发布 Detection 结果

Day 5: 实现 tf_broadcaster.py
       广播 base_link → camera_link → arm_link
```

### Phase 2: 可视化与录制（第 2 周）

```
Day 1: 实现 rviz_markers_node.py
       检测框、车道线、机械臂的 RViz 标记

Day 2: 配置 RViz 保存文件 (vehicle_wbt.rviz)
       自动化 launch 文件

Day 3: 测试 rosbag 录制+回放工作流
       验证数据完整性

Day 4-5: 团队培训 + 文档
       RViz 使用教程
       rosbag 调试工作流教程
```

### Phase 3: 增强（后续，按需）

- 参数动态调参（`dynamic_reconfigure`）
- 多摄像头同步录制
- 竞赛回放分析工具

## 风险与缓解

| 风险 | 概率 | 影响 | 缓解措施 |
|------|:---:|:---:|---------|
| roscore 启动增加延迟 | 中 | 🟡 | systemd 服务中独立启动 roscore |
| ROS 发布增加 CPU 开销 | 低 | 🟡 | 限制发布频率（10Hz 而非 30Hz） |
| rospy 和 ZMQ 线程冲突 | 低 | 🟡 | ROS 发布在独立线程 |
| 竞赛现场不允许跑 roscore | 低 | 🔴 | ROS 层可完全关闭，不影响主功能 |
| ROS1 Noetic 已 EOL | — | — | 仅用于调试，不作为核心依赖 |

## 不做的事

- ❌ 不把推理服务迁移到 ROS Service（ZMQ 更简单更快）
- ❌ 不把运动控制迁移到 `/cmd_vel`（直接调用更可靠）
- ❌ 不用 ROS launch 替代 systemd（竞赛场景不需要）
- ❌ 不装 move_base / nav2（不需要路径规划）
- ❌ 不迁移到 ROS2（成本高，收益不明确）

## 回滚方案

如果 ROS 集成引入问题，只需：
1. 停止 roscore 和桥接节点
2. 竞赛脚本完全不受影响（不依赖 ROS）

**零回滚成本**，因为 ROS 层是纯观察者。

## 结论

| 方案 | 工作量 | 收益 | 风险 | 推荐度 |
|------|:---:|------|:---:|:---:|
| A: 不集成 | 0 | 无调试工具 | 无 | ⭐⭐ |
| B: 全量迁移 | 6-10 周 | 完整 ROS | 极高 | ⭐ |
| **C: 轻量集成** | **1-2 周** | **RViz + rosbag + tf** | **低** | **⭐⭐⭐⭐** |

**推荐方案 C。** 用 1-2 周获得 RViz 可视化和 rosbag 数据录制能力，这是当前项目最缺的调试工具。ROS 层是纯观察者，不影响竞赛脚本，可随时关闭。
