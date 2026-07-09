# Interface Snapshot — 2026-07-09 Freeze

> **状态**:Jetson 端冻结。3 个 launch 在跑,接口不再变化,4 同事 dev box 可以放心对接。
> 任何对底层接口的改动需先解冻(改此文件 + git commit)。

## 跑着的 launch

| Launch | 节点 | 串口/相机 | 角色 |
|---|---|---|---|
| `mc602.launch.py` | `mc602_io`, `chassis_kinematics` | `/dev/ttyUSB0` | **Phase 1 唯一串口主控** |
| `full_system.launch.py` | 7 个 C++ 节点(见下) | `/dev/cam3`, `/dev/cam4` | legacy 7 节点栈 |
| `smartcar_bridge.launch.py` | `smartcar_bridge_node` | — | legacy MyCar API(空跑) |

## 节点(11 个)

| 节点 | 类型 | 来源 |
|---|---|---|
| `mc602_io` | Python, Phase 1 | bridge |
| `chassis_kinematics` | Python, Phase 1 + | bridge |
| `camera_front` | C++, legacy | platform_cpp |
| `camera_arm` | C++, legacy | platform_cpp |
| `ir_left` | C++, legacy | platform_cpp |
| `ir_right` | C++, legacy | platform_cpp |
| `mecanum_chassis` | C++, legacy | platform_cpp |
| `arm_main` | C++, legacy | platform_cpp |
| `safety_gate` | C++, legacy | platform_cpp |
| `mission_runner` | C++, legacy | platform_cpp |
| `smartcar_bridge_node` | Python, legacy | bridge(无串口依赖) |

## Topic 总览(25 个)

### Phase 1 — 5 个
| Topic | 方向 | 类型 | 频率 | 说明 |
|---|---|---|---|---|
| `/vehicle_wbt/v1/cmd_vel` | 同事 → Jetson | `geometry_msgs/Twist` | 业务方 | 期望车体速度 |
| `/vehicle_wbt/v1/odom` | Jetson → 同事 | `nav_msgs/Odometry` | 50 Hz | 麦纳姆累计 odom |
| `/tf` | Jetson → 同事 | `tf2_msgs/TFMessage` | 50 Hz | odom→base_link |
| `/vehicle_wbt/v1/mc602/state/raw` | Jetson → 同事 | `vehicle_wbt_smartcar_msgs/msg/RawState` | 20 Hz | 完整传感器快照 |
| `/vehicle_wbt/v1/mc602/heartbeat` | Jetson → 同事 | `std_msgs/Header` | 1 Hz | 心跳 |
| `/vehicle_wbt/v1/mc602/board/button_events` | Jetson → 同事 | `vehicle_wbt_smartcar_msgs/msg/ButtonEvent` | 边沿 | 板载按键 |
| `/vehicle_wbt/v1/mc602/play_melody` | 同事 → Jetson | `vehicle_wbt_smartcar_msgs/msg/Melody` | 业务方 | 触发旋律 |

### Legacy C++ 7 节点 — 16 个
- `/vehicle_wbt/v1/cmd/vel_raw` — 原始 cmd_vel
- `/vehicle_wbt/v1/cmd/vel_safe` — 安全过滤后
- `/vehicle_wbt/v1/state/odom` — 旧底盘 odom
- `/vehicle_wbt/v1/state/actuators/main` — 旧臂状态
- `/vehicle_wbt/v1/cmd/arm/main/trajectory` — 旧臂轨迹输入
- `/vehicle_wbt/v1/safety/{estop,heartbeat,mode_cmd}` — 安全通道
- `/vehicle_wbt/v1/sensors/ir/{left,right}` — 红外(Float32)
- `/vehicle_wbt/v1/sensors/camera/{front,arm}/{image_raw,image_compressed,camera_status,camera_meta}` — 2 相机 × 4 stream = 8

### ROS2 系统 — 2 个
- `/parameter_events`, `/rosout`

## Service 总览(64 个)

### Phase 1 业务 — 21 个(`/vehicle_wbt/v1/mc602/*`)

| 类别 | Service | 字段 | 字段 |
|---|---|---|---|
| 底盘 | `set_wheels` | `int8 v0, v1, v2, v3` | (4 轮速度) |
| 底盘 | `read_encoders` | — | `int32[4] v` + `bool success` |
| 底盘 | `reset_encoders` | — | `bool success` |
| 底盘 | `set_dc_motor` | `uint8 port, int8 speed` | (单 DC 电机) |
| 臂 | `set_servo_pwm` | `uint8 port, int16 angle` | (PWM 舵机) |
| 臂 | `set_servo_bus` | `uint8 port, int16 angle, int8 speed` | (总线舵机) |
| 臂 | `set_stepper` | `uint8 port, int32 pwm` | (步进电机) |
| IO | `set_pout` | `uint8 port, bool state` | (数字输出) |
| IO | `set_led` | `uint8 port, uint8 r, g, b` | (RGB LED) |
| IO | `set_nixie` | `uint8 port, int32 number` | (数码管) |
| IO | `show_screen` | `string text` | (屏显) |
| 读 | `read_analog` | `uint8 port` | (通用模拟输入) |
| 读 | `read_touch` | `uint8 port` | (触摸) |
| 读 | `read_ultrasonic` | `uint8 port` | (超声) |
| 读 | `read_ambient` | `uint8 port` | (环境光) |
| 读 | `read_ir` | `uint8 port` | (红外测距) |
| 读 | `read_key4` | — | (4 键按键) |
| 读 | `read_battery` | — | (电池电压) |
| 读 | `read_bluetooth` | — | (蓝牙手柄) |
| 音频 | `buzzer` | `uint16 freq, float32 duration` | (蜂鸣器) |
| 音频 | `play_predefined` | `uint8 melody_id` | (预存旋律) |

### Legacy MyCar API — 19 个(`/vehicle_wbt/v1/cmd/*`)

```
chassis:    lane_base / lane_dis / lane_dis_offset / lane_time
            move_distance / move_for / move_time / move_to_position
            reset_odom
arm:        grasp / move_x / move_y / reset_position
            set_arm_angle / set_hand_angle / set_pose
注:peripheral_node 已永久删除(2026-07-09),buzzer / storage / shoot
   请改用 Phase 1 接口 /vehicle_wbt/v1/mc602/buzzer + /set_pout
```

### 系统 — 1 个
- `/set_camera_info`

### 参数服务 — 42 个(7 个 node × 6 个)
每个节点自动暴露:`/describe_parameters`, `/get_parameters`, `/get_parameter_types`, `/list_parameters`, `/set_parameters`, `/set_parameters_atomically`

## Action 总览

**0 个 action**

设计选型:Phase 1 全是同步 service + 流式 topic,没有长任务状态机反馈。
如果未来需要 action(机械臂轨迹插值 / 开火倒计时),需写新 action server 包,调 Phase 1 service 拼装。

## LAN 协作环境变量(dev box 必设)

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 设备节点状态(测试已通过)

| 测试项 | 结果 |
|---|---|
| M1-M4 麦轮 4 轮 | ✅ 编码器 Δ=[30, 6, 35, 30] (0.5s, v=15) |
| M6 机械臂 X 轴 (set_dc_motor) | ✅ success |
| P1-P5 P 口 (read_ir) | ✅ 5 端口全 success |
| S1 总线舵机 (Z 轴) | ✅ set_servo_bus success |
| S2 总线舵机 (Y 轴) | ✅ set_servo_bus success |
| IR 模拟输入 port 7, 8 | ✅ read_ir success |
| D1 PWM 舵机(跷跷板) | ✅ set_servo_pwm success |
| front 相机 (image_compressed) | ✅ 29.8 Hz |
| arm 相机 (image_compressed) | ✅ 30.0 Hz |

## dev box 5 行 onboarding

```bash
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
ros2 topic list | wc -l    # 应该 25+ topics
ros2 service list | wc -l  # 应该 60+ services
./scripts/quick_beep.sh    # 听到 beep → 全链路通
```

详细 dev box 教程: `docs/integration/LOWLEVEL_API.md` + `DEV_QUICKSTART.md` + `CHASSIS_TOPICS.md`

## 解冻流程

1. 改本文件描述新接口
2. `git commit -m "unfreeze: <reason>"`
3. 改代码 + 测试
4. `git commit -m "freeze: 2026-MM-DD 接口冻结"`
5. 更新本文件描述新冻结状态
6. 推送到 origin/robot-stable

## 已知限制(2026-07-09)

- 没有 action(机械臂轨迹 / 开火倒计时需自建)
- chassis_kinematics sign_flip 默认值:`vx=+1, vy=-1, omega=-1`(真机验证后定的)
- 实际速度 ≈ cmd_vel × 0.5-0.7(电机响应 + 50Hz 调度离散化)
- mc602_node 有 sensor tick warning(int() not list — IR sensor 类型不匹配)但不影响功能
