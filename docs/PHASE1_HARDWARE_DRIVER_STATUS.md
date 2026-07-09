# Phase 1 MC602 驱动 + 硬件状态(2026-07-09)

> 给同事的 5 分钟 onboarding:本仓库现在的 MC602 IO 网关是什么、能做什么、还差什么。

## TL;DR

Jetson 端 mc602_io 节点独占 `/dev/ttyUSB0` (CH340),通过 `/vehicle_wbt/v1/mc602/*` 暴露
**21 个 service + 4 个 topic**,同事在 dev box 上 `ros2 service call` / `ros2 topic pub` 就能
控制真实硬件,无需 SSH Jetson。

同事 4 人在 LAN 上并行开发自己的业务包,`colcon build --packages-up-to vehicle_wbt_smartcar_msgs`
+ `ros2 service call /vehicle_wbt/v1/mc602/...` 就够。

## 实硬件清单(2026-07-09 验证)

| 设备 | 物理位置 | dev_id / port | 工作状态 | 备注 |
|------|----------|----------------|----------|------|
| 麦轮电机 (M1) | M口 port 1 | dev_id 0x02, port 1 | ✅ **work,看到转动** | encoder 累到 5764 |
| 蜂鸣器 (Buzzer) | dev_id 0x0a | — | ✅ **work,可闻** | rings 262/440Hz |
| 电池 (Battery) | dev_id 0x0c | — | ✅ 12.4V (3S LiPo) | read 返回 V |
| 4 键按键板 (Key4Btn) | **P1** | dev_id 0x07, mode=0 | ✅ **work,你按了 1/2/3/4** | ADC 358/1391/2020/3025 |
| IR 传感器 (左, A1) | A口 port 1 | dev_id 0x08 (Sensor_Analog2_2) | ✅ raw ADC ~1465 | **应用层需校准公式** |
| IR 传感器 (右, A2) | A口 port 2 | dev_id 0x08 | ✅ raw ADC ~200 | 同上 |
| PWM 舵机 (hand) | P1 | dev_id 0x05, port 1 | ✅ **work,看到动** | set_angle SDK None 但 servo 真动 |
| 智能总线舵机 (wrist) | bus ID 1 | dev_id 0x06, port 1 | ✅ **work,看到动** | 同 protocol 现象 |
| 板载按钮 (on-board button) | dev_id 0x0d | — | ✅ **连上了**(待再测) | 单按钮 0/1 状态 |

**未连接的 P/M 口**:M2-M4 麦轮没接电机,P2-P8 没接传感器。同事业务包用到时再调 `mc602_ports.yaml`。

## 21 个 service 列表(同事 dev box 端调用)

```bash
# === 写类 actuator ===
/vehicle_wbt/v1/mc602/set_wheels       # 4 路麦轮 v0..v3 int8 [-100,100]
/vehicle_wbt/v1/mc602/set_servo_pwm    # PWM 舵机 port, angle, speed
/vehicle_wbt/v1/mc602/set_servo_bus    # 总线舵机 port, angle, speed
/vehicle_wbt/v1/mc602/set_stepper      # 步进电机 port, freq
/vehicle_wbt/v1/mc602/set_dc_motor     # 直流电机 port, speed
/vehicle_wbt/v1/mc602/set_pout         # 数字输出 port, state (0/1)

# === 读类 sensor ===
/vehicle_wbt/v1/mc602/read_encoders    # 4 路编码器 int32
/vehicle_wbt/v1/mc602/reset_encoders   # 清零
/vehicle_wbt/v1/mc602/read_ir          # IR 距离 m(实际是 raw ADC,见下)
/vehicle_wbt/v1/mc602/read_battery     # V
/vehicle_wbt/v1/mc602/read_analog      # ADC 0-4095
/vehicle_wbt/v1/mc602/read_touch       # 0/1
/vehicle_wbt/v1/mc602/read_ultrasonic  # mm
/vehicle_wbt/v1/mc602/read_ambient     # 0-4095
/vehicle_wbt/v1/mc602/read_bluetooth   # [lx,ly,rx,ry,btn]
/vehicle_wbt/v1/mc602/read_key4        # 1/2/3/4 (按了哪个键)

# === 蜂鸣器 ===
/vehicle_wbt/v1/mc602/buzzer           # 单音 freq, duration_ms
/vehicle_wbt/v1/mc602/play_predefined  # name: "twinkle"/"mary"/"birthday"

# === 新增 actuator ===
/vehicle_wbt/v1/mc602/set_led          # led_id, r, g, b, port
/vehicle_wbt/v1/mc602/set_nixie        # port, num
/vehicle_wbt/v1/mc602/show_screen      # text
```

## 4 个 topic(20Hz 持续 stream + 边沿事件)

```bash
# 20Hz 持续发布
ros2 topic echo /vehicle_wbt/v1/mc602/state/raw
# 字段: encoders[4] | ir_left_m | ir_right_m | ir_left_adc | ir_right_adc
#        | battery_v | arm_y_pos | arm_x_pos | pump_on | valve_on | board_key

# 板载按钮边沿(按下/松开)
ros2 topic echo /vehicle_wbt/v1/mc602/board/button_events
# 字段: pressed: bool (true=按下, false=松开)

# 任意旋律(自定义频率+时长)
ros2 topic pub /vehicle_wbt/v1/mc602/play_melody \
    vehicle_wbt_smartcar_msgs/msg/Melody \
    "{notes: [{freq_hz: 262, duration_ms: 400}, ...]}"

# 1Hz alive signal
ros2 topic echo /vehicle_wbt/v1/mc602/heartbeat
```

## IR 校准(重要!)

IR 传感器接在 A1/A2 (Sensor_Analog2_2, dev_id 0x08),返回 **raw ADC 0-4095**,**不是米**。
同事应用层需要校准公式。典型 Sharp IR 公式:
```python
# distance_cm ≈ 12000 / adc_value  (粗略)
def adc_to_cm(adc):
    if adc < 100:
        return float('inf')  # too close
    return 12000.0 / adc
```

`/vehicle_wbt/v1/mc602/state/raw` 同时暴露 `ir_left_m`(暂为 0)和 `ir_left_adc`(raw)。
**应用层**读 adc 字段 + 用 sensor-specific 校准公式 → 距离。

## 5 个 shell 脚本(本仓库 `scripts/`)

```bash
# Jetson 端安装(一次性,sudo 跑)
sudo bash deploy/install.sh

# 信任验证:一行 beep 听 Jetson 是否工作
./scripts/quick_beep.sh

# 5 步诊断 LAN 连通性(开发机)
./scripts/check_link.sh

# 综合硬件 probe(11 设备,~60s,SDK-direct 绕开 wrapper state 问题)
./scripts/probe_all.sh

# 按键诊断:扫 P1-P8 + on-board button,5s/口
./scripts/probe_p_ports.sh

# 持续监听 Key4Btn 4 键按键(30s 默认)
./scripts/watch_key4.sh
```

## 已知协议限制(给同事的注意点)

**read 类 device** (Buzzer/Motor/Battery/IR/Key4Btn/Encoders/BoardKey/Stepper get/Bus no_act):
- MC602 firmware 回包,SDK 工作 ✓
- 直接调 service 拿数据

**write 类 device** (PWM/Bus set/Stepper set_pwm/PoutD set):
- MC602 firmware **不回包**!SDK 的 `get_anwser` 等超时 → 返回 `None`
- **但实际指令被处理**(用户已确认 PWM/Bus 动)
- 同事写业务时**别用 response 验证**——直接相信 success=True,加 read-after-write 验证(读 sensor 确认状态)

## Phase 1 同事工作分配建议

| 同事 | 业务 | 用的 service / topic |
|------|------|----------------------|
| 底盘 | 麦轮 PID + odom | `read_encoders`(20Hz)/`set_wheels` + 算 `MecanumChassis` (hw/odometry.py) |
| 机械臂 | 舵机/电机控制 + PID | `set_servo_pwm`/`set_servo_bus`/`set_stepper` + 读 analog 反锁 |
| 枪 | 数字输出控制发射 | `set_pout` (shooter.barrel) |
| 红外避障 | IR 读 + 校准 | `state/raw.ir_left_adc`/`ir_right_adc` + 应用层 Sharp IR 公式 |
| 蜂鸣器提示 | 单音/旋律 | `buzzer` service 或 `play_melody` topic |

## 仓库结构(Phase 1 关键路径)

```
rak-car/
├── ros2_ws/
│   ├── src/
│   │   ├── vehicle_wbt_smartcar_hw/         # 协议层 23 device class + odometry + arm
│   │   │   ├── mc602_ctl2.py                # 14 + 9 个 device class
│   │   │   ├── serial.py                    # MC602 serial wrapper
│   │   │   ├── odometry.py                  # MecanumChassis + Odometry (numpy 1e-17 OK)
│   │   │   └── arm.py                       # ArmController(无 PID)
│   │   │
│   │   ├── vehicle_wbt_smartcar_msgs/       # 接口契约
│   │   │   ├── msg/RawState.msg             # 20Hz state
│   │   │   ├── msg/Melody.msg + MelodyNote.msg
│   │   │   ├── msg/ButtonEvent.msg
│   │   │   └── srv/SetWheels, ReadEncoders, ResetEncoders, ...
│   │   │       ReadTouch, ReadUltrasonic, ReadAmbient, ReadBluetooth, ReadKey4
│   │   │       SetLed, SetNixie, ShowScreen, Buzzer, PlayPredefined
│   │   │
│   │   └── vehicle_wbt_smartcar_bridge/     # mc602_node + launch
│   │       ├── mc602_node.py                # 21 service + 4 topic
│   │       ├── launch/mc602.launch.py
│   │       └── config/mc602_ports.yaml      # 硬件端口路由表
│   │
├── deploy/                                  # 部署件
│   ├── systemd/vehicle-wbt-mc602.service
│   ├── ros_env.sh, cyclonedds/cyclonedds.xml, udev/99-usbvideo.rules
│   └── install.sh                           # sudo 跑,自动 enable + start
│
├── scripts/                                 # Jetson 端工具
│   ├── quick_beep.sh                        # 一行验证
│   ├── check_link.sh                        # 5 步诊断
│   ├── probe_all.sh                         # 11 设备综合 probe
│   ├── probe_p_ports.sh                     # P 口扫描
│   └── watch_key4.sh                        # 4 键持续监听
│
└── docs/
    ├── HARDWARE_AND_DRIVER_STATUS.md        # (本文件)
    ├── integration/LOWLEVEL_API.md          # 21 service API
    ├── integration/MC602_HARDWARE_REFERENCE.md  # 协议帧格式
    ├── integration/DEV_QUICKSTART.md        # 同事 5 分钟 onboarding
    └── superpowers/
        ├── specs/2026-07-09-rak-car-mc602-phase1-design.md
        └── plans/2026-07-09-rak-car-mc602-phase1-plan.md
```

## Phase 1 commit 链(20+ commits)

```
4e7378b feat(scripts): probe_all.sh — comprehensive hardware probe
2d7986c fix(bridge): IR sensors on A1/A2 use Sensor_Analog2_2 (dev_id 0x08)
01fe5d1 fix(bridge): update mc602_ports.yaml (P1 IR + d1 PWM + bus_id 1)
69ea184 feat(scripts): watch_key4.sh
2953518 feat(scripts): probe_p_ports.sh
46c83aa fix(bridge): board_key [mode,value] 2-tuple + watch script
2ccd5c0 fix(hw+bridge): SDK no_act()/get() returns int (value field)
2104749 fix(bridge): Key4Btn port 4 -> 1
66bea75 docs: copy 硬件接口设置.md + update LOWLEVEL_API
aeb1f7a feat(hw+bridge): 9 new device classes + 8 new services + fix parsing
b7f2b08 feat(bridge): board button in /state/raw + /board/button_events edge
f24432b feat(bridge): melody playback via /play_melody topic + /play_predefined
8701731 fix(hw): MC602.get_anwser auto-recovery
3b07d56 fix(hw): get_anwser kwarg bug
... 17+ total commits
tag: phase1-mc602-driver-ready
```

## 后续 TODO(Phase 1+ / Phase 2)

1. **MC602 firmware 升级** — 让 set 类也回包,SDK 不再 timeout
2. **MC602 wrapper 重写** — 当前的 `vehicle_wbt_smartcar_hw.serial.MC602` 在密集测试中 state pollution;
   同事生产用 mc602_node 没事但脚本/ad-hoc 测试会卡
3. **业务包就位** — chassis / arm / shooter / perception 各自的 pkg
4. **加 Key4Btn + 数字 IO read-after-write 验证** — 让同事业务能确认指令真生效
5. **Phase 4 退役老 C++ `vehicle_wbt_platform_cpp`** — 现在两条栈并行

## 同事第一步

```bash
# 1. clone + colcon build
cd ~/workspace/rak-car
git pull
cd ros2_ws
colcon build --packages-up-to vehicle_wbt_smartcar_msgs
source install/setup.bash

# 2. export 环境(dev box)
export ROS_DOMAIN_ID=42
export RMW_OVERRIDE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # Jetson 端有 CycloneDDS;没装用 fastdds 跑通

# 3. 测试 LAN 连通
cd ..
./scripts/check_link.sh

# 4. 听 beep
./scripts/quick_beep.sh

# 5. 开始写业务
# (同事各自建 vehicle_wbt_smartcar_{chassis,arm,shooter,perception} pkg)
```
