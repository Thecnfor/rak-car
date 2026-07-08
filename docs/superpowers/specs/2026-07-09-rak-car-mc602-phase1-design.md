# rak-car Phase 1 — 通用 MC602 底层驱动设计

**Date:** 2026-07-09
**Branch:** `robot-stable`
**Status:** Approved (待用户最后核阅)
**Author:** Claude (brainstorming session)
**Goal:** Jetson 端的 MC602 协议层 + 唯一串口所有者 + 通用 ROS2 接口,让上层 4 个同事在 develop 分支并行开发业务节点。

---

## 1. 背景 & 动机

`rak-car` 当前从老 C++ `ros2_control` 栈迁移到 Python + MC602 下位机链路。
新协议层 `vehicle_wbt_smartcar_hw` 已开始,但只移植了 `Buzzer_2/ServoPwm/PoutD` 三个只写设备类,且字节格式与 SDK 实际帧不一致(评审已确认)。

目标架构:Jetson 上 **唯一一个进程** 独占 `/dev/ttyUSB*` 通向下位机 MC602,通过 ROS2 service/topic 暴露通用接口给 LAN 上的 dev 机器团队开发上层。

**Ground truth**: `baidu_smartcar_2026/smartcar/whalesbot/` SDK,所有协议字节、设备类、底盘运动学、机械臂 PID 都从那里 1:1 抄过来。

---

## 2. 团队分工

| 人 | 负责 | 工作分支 | 包 |
|---|---|---|---|
| 我(底层) | 协议层 + 唯一串口所有者 + 部署件 + API 文档 | `robot-stable` | `vehicle_wbt_smartcar_{hw,msgs,bridge}` |
| 同事 A | 底盘业务节点(订阅 cmd_vel、计算 odom、发布 TF、PID move_to_position) | `develop` | `vehicle_wbt_smartcar_chassis` |
| 同事 B | 机械臂业务节点(订阅 cmd_arm_pose、PID 闭环、发布 arm_state) | `develop` | `vehicle_wbt_smartcar_arm` |
| 同事 C | 枪业务节点(订阅视觉/事件、调 PoutD 控制发射/泵/阀) | `develop` | `vehicle_wbt_smartcar_shooter` |
| 同事 D | LLM/ERNIE 推理(订阅摄像头、任务规划) | `develop` | `vehicle_wbt_smartcar_perception` |

**约束**:上层同事只调 `/mc602/*` service + 订阅 `/mc602/state/raw`,**不 SSH Jetson**,**不碰 MC602 协议代码**。他们写完自己的节点,在开发机上 colcon build,启动后通过 LAN DDS 自动发现 Jetson 端,跨机器协作。

---

## 3. ROS2 Packages 划分

```
ros2_ws/src/
├── vehicle_wbt_smartcar_hw/         ← 我(协议层,纯 Python 库,无 ROS 依赖)
├── vehicle_wbt_smartcar_msgs/        ← 我(共享接口契约,rosidl)
├── vehicle_wbt_smartcar_bridge/      ← 我(rclpy 节点 mc602_node.py + launch)
├── vehicle_wbt_smartcar_chassis/     ← 同事 A(底盘)
├── vehicle_wbt_smartcar_arm/         ← 同事 B(机械臂)
├── vehicle_wbt_smartcar_shooter/     ← 同事 C(枪)
└── vehicle_wbt_smartcar_perception/  ← 同事 D(LLM/视觉)
```

每个 package 独立 `package.xml` / `setup.py`,独立 `colcon build`。上层只 `depend` `vehicle_wbt_smartcar_msgs`。

---

## 4. 通用 MC602 接口契约(`vehicle_wbt_smartcar_msgs/`)

### 4.1 Service 接口(12 个)

| Service | 类型 | 字段 | 谁用 |
|---|---|---|---|
| `/mc602/set_wheels` | `SetWheels.srv` | `v0,v1,v2,v3: int8[-100..100]` | 底盘 |
| `/mc602/read_encoders` | `ReadEncoders.srv` | → `{v: int32[4]}` | 底盘 |
| `/mc602/reset_encoders` | `Trigger.srv` | → `{success: bool}` | 底盘 |
| `/mc602/set_servo_pwm` | `SetServoPwm.srv` | `{port: uint8, angle: int16}` | 机械臂(手爪) |
| `/mc602/set_servo_bus` | `SetServoBus.srv` | `{port: uint8, angle: int16, speed: int16}` | 机械臂(腕部) |
| `/mc602/set_stepper` | `SetStepper.srv` | `{port: uint8, freq: int16}` | 机械臂(Y 轴) |
| `/mc602/set_dc_motor` | `SetDcMotor.srv` | `{port: uint8, speed: int8}` | 机械臂(X 轴) |
| `/mc602/set_pout` | `SetDout.srv` | `{port: uint8, state: bool}` | 枪/机械臂(grasp) |
| `/mc602/read_ir` | `ReadIR.srv` | `{port: uint8} → {distance_m: float32}` | 底盘/枪 |
| `/mc602/read_battery` | `ReadBattery.srv` | `{} → {voltage_v: float32}` | 任何人 |
| `/mc602/read_analog` | `ReadAnalog.srv` | `{port: uint8} → {value: int16}` | 机械臂(限位) |
| `/mc602/buzzer` | `Buzzer.srv` | `{freq_hz: uint16, duration_ms: uint16}` | 任何人 |

### 4.2 Topic 接口(2 个)

| Topic | 类型 | Rate | 字段 |
|---|---|---|---|
| `/mc602/state/raw` | `RawState.msg` | 50 Hz | `encoders: int32[4]`, `ir_left_m: float32`, `ir_right_m: float32`, `battery_v: float32`, `arm_y_pos: int32`, `arm_x_pos: int32`, `pump_on: bool`, `valve_on: bool` |
| `/mc602/heartbeat` | `std_msgs/Header` | 1 Hz | `stamp` |

### 4.3 设计原则

- **单字段 .srv,简单 1-3 字段**,不预先组合上层业务逻辑
- **不暴露 MC602 协议细节**(dev_id、port 字节等),用人类可读的英文 port(底盘同事不知道硬件 dev_id)
- **失败语义**:Service 调用永不抛异常,失败返回 `success: false` + 字段值不变;非阻塞,**不**保证在下一次调用前 frame 完成
- **50 Hz 高频数据走 topic**(raw),单次按需查询走 service

---

## 5. 协议层设计(`vehicle_wbt_smartcar_hw/`)

### 5.1 模块结构

```
vehicle_wbt_smartcar_hw/
├── package.xml                # ament_python, depends: pyserial
├── setup.py
└── vehicle_wbt_smartcar_hw/
    ├── __init__.py            # 重导出
    ├── serial.py              # MC602 class(抄 SDK serial_wrap.py 的 MC602 部分)
    ├── mc602_ctl2.py          # DevCmdInterface + DevListWrap + 20 设备类(1:1 抄 SDK)
    ├── odometry.py            # Odometry + MecanumChassis 纯计算(抄 SDK,去掉 Driver)
    └── arm.py                 # 设备引用 + set_angle/grasp 工具方法(简化抄 SDK)
```

### 5.2 关键决定

- **协议层是无 ROS 的纯 Python 库**,同事业务节点的代码可以 `from vehicle_wbt_smartcar_hw import Motors_2`(虽然原则上不推荐,但允许用于单元测试)
- **每个设备类的字节格式严格 1:1 对齐 SDK `mc602_ctl2.py`**,dev_id、mode、port_id、struct fmt 都抄过来
- **`port_id=None` vs `port_id=0` 区分**:前者不发 port 字节(Buzzer_2 的行为),后者发 0x00;通过 SDK `get_bytes` 函数 `mc602_ctl2.py:124-128` 的逻辑保留
- **`get_anwser` 完整抄 SDK `serial_wrap.py:222-245`**(读 3 字节→ length 字节解 dst_len → drain → 校验头尾 → 返回 res[3:-1])
- **`MC602` 是单例**(模块级 `serial_mc602 = MC602()`),跟 SDK 风格一致,所有设备类共享
- **`ping_port` 抄 SDK `serial_wrap.py:113-142`**,启动时自动扫 CH340/USB 端口

### 5.3 设备类清单(Phase 1 包含的 14 个,SDK 全 20 个减 4 个)

| 类 | dev_id | 状态 |
|---|---|---|
| `Buzzer_2` | 0x0a | ✓ 已有,只修字节 |
| `Motor_2` | 0x02 | 抄 |
| `Motor4_2` | 0x01 | 抄 |
| `Motors_2` | — | 抄(用 DevListWrap) |
| `EncoderMotor_2` | 0x04 | 抄 |
| `EncoderMotors4_2` | 0x03 | 抄 |
| `ServoPwm_2` | 0x05 | 抄(注意:fmt='bbBB' 速度+角度,不是 0..9000 映射) |
| `ServoBus_2` | 0x06 | 抄 |
| `AnalogInput_2` | 0x07 m=0 | 抄 |
| `Infrared_2` | 0x07 m=1 | 抄(读后 /1000.0 转米) |
| `Battry_2` | 0x0c | 抄(读后 /1000 转伏) |
| `BoardKey_2` | 0x0d | 抄 |
| `PoutD_2` | 0x10 | 抄(注意:只用 1 字节 state,SDK format='bbb' padding) |
| `Stepper_2` | 0x11 | 抄 |

**Phase 5+ 再加**:`BluetoothPad_2`(0x09)、`LedLight_2`(0x0e)、`NixieTube_2`(0x0f)、`ScreenShow_2`(0x0b)。

### 5.4 不做的

- ❌ LifecycleNode / MultiThreadedExecutor(底层用普通 Node)
- ❌ 自定义异常类(Style 与 SDK 一致,失败返回 None)
- ❌ MockSerial / 单元测试(真硬件验证)
- ❌ 重构 SDK 代码(直接抄)
- ❌ 任何业务逻辑(move_to_position PID、set_arm_pose 闭环等)→ 这些留给上层同事

---

## 6. 桥接节点设计(`vehicle_wbt_smartcar_bridge/`)

### 6.1 单节点 `mc602_node.py` (~300 行)

```python
class MC602Node(Node):
    def __init__(self):
        super().__init__('mc602_io')
        # 1. 声明参数: serial_port, baud, control_rate_hz, sensor_rate_hz
        # 2. 打开 MC602Serial
        # 3. 实例化设备类(Ports 编号见 config/mc602_ports.yaml)
        # 4. 创建 12 个 service server
        # 5. 创建 50Hz timer + 1Hz timer
        # 6. 创建 2 个 publisher

    def _on_set_wheels(self, req, resp): ...      # 调 Motors_2.set_speed
    def _on_read_encoders(self, req, resp): ...   # 调 EncoderMotors4_2.get
    def _on_reset_encoders(self, req, resp): ...  # 调 Motors_2.reset_encoder
    def _on_set_servo_pwm(self, req, resp): ...   # 路由到 self._servo_pwm_map[req.port]
    def _on_set_servo_bus(self, req, resp): ...   # 路由到 self._servo_bus_map[req.port]
    def _on_set_stepper(self, req, resp): ...
    def _on_set_dc_motor(self, req, resp): ...
    def _on_set_pout(self, req, resp): ...
    def _on_read_ir(self, req, resp): ...
    def _on_read_battery(self, req, resp): ...
    def _on_read_analog(self, req, resp): ...
    def _on_buzzer(self, req, resp): ...

    def _tick_50hz(self):
        # 读 encoders/IR/battery/analog,发布 /mc602/state/raw
    def _tick_1hz(self):
        # 发布 /mc602/heartbeat
```

### 6.2 设备路由表(config/mc602_ports.yaml)

底盘/臂/枪的硬件端口号集中在一个 YAML 里,同事需要时查(不写死):

```yaml
chassis:
  motors: [1, 2, 3, 4]
  encoders_dev: 0x03  # EncoderMotors4_2
arm:
  stepper_y:
    port: 1
    dev_id: 0x11
  dc_motor_x:
    port: 2
    dev_id: 0x02
  servo_pwm_hand:
    port: 3
    dev_id: 0x05
  servo_bus_wrist:
    port: 4
    dev_id: 0x06
  pump:
    port: 1
    dev_id: 0x10
  valve:
    port: 2
    dev_id: 0x10
  limit_switch_y:
    port: 6
    dev_id: 0x07 mode=0
shooter:
  barrel:
    port: 4
    dev_id: 0x10
ir:
  left: 7
  right: 8
```

### 6.3 失败语义

- 每个 service handler 调 SDK 设备方法,**失败返回 None 时**:`success: false`,**不抛异常**
- `resp.value` 字段(读类)在失败时填默认(0/0.0/false)
- 写类 service 在失败时只填 `success: false`,不动硬件

---

## 7. 部署设计(Jetson 端)

### 7.1 systemd unit

`deploy/systemd/vehicle-wbt-mc602.service`:
```ini
[Unit]
Description=RAK-Car MC602 IO Node
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=jetson
EnvironmentFile=/etc/vehicle-wbt/ros.env
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/jetson/workspace/rak-car/ros2_ws/install/setup.bash && exec ros2 run vehicle_wbt_smartcar_bridge mc602_node --ros-args -p serial_port:=/dev/ttyUSB1 -p baud:=1000000'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 7.2 env file

`deploy/ros_env.sh` → `/etc/vehicle-wbt/ros.env`:
```bash
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/cyclonedds.xml
VEHICLE_WBT_SERIAL=/dev/ttyUSB1
```

### 7.3 CycloneDDS 配置

`deploy/cyclonedds/cyclonedds.xml` → `/etc/cyclonedds.xml`(抄 platform_cpp/ 已有的,微调)。

### 7.4 udev 规则(摄像头)

`deploy/udev/99-usbvideo.rules` → `/etc/udev/rules.d/`:
```
# Aveo SP2812 cameras (vendor 1871:0110)
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1871", ATTRS{idProduct}=="0110", ATTR{devpath}=="*4.4*", SYMLINK+="cam4"
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1871", ATTRS{idProduct}=="0110", ATTR{devpath}=="*3.3*", SYMLINK+="cam3"
```

---

## 8. API 文档(`docs/integration/LOWLEVEL_API.md`)

给上层 4 个同事看的 API 文档,内容包括:

1. **总览**:Jetson 端服务部署在 `192.168.3.69`,ROS_DOMAIN_ID=42,通过 LAN DDS 自动发现
2. **快速验证**:在开发机上跑两条命令确认连通
3. **每个 service 的字段说明 + Python 调用示例**(基于 rclpy)
4. **每个 topic 的字段说明 + 订阅示例**
5. **常用模式**:
   - "底盘 50Hz 读编码器后积分 odom"伪代码
   - "机械臂发 pose → 等到达 → 反馈"伪代码
6. **错误处理**:`success: false` 时的处理

---

## 9. CLAUDE.md 更新要点

- 新架构说明(3 个底层 package + 上层 4 个 package 的边界)
- Jetson 部署步骤(`systemctl enable --now vehicle-wbt-mc602`)
- 开发机使用步骤(`colcon build --packages-up-to vehicle_wbt_smartcar_bridge && ros2 service call ...`)
- 移除/标注过时的内容(老 C++ ros2_control 节点即将退役)

---

## 10. 端到端验证(Jetson 端,真硬件)

冒烟测试步骤(我自己跑):

```bash
# 1. colcon build
cd ~/workspace/rak-car/ros2_ws
colcon build --packages-up-to vehicle_wbt_smartcar_bridge

# 2. 启动节点
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 run vehicle_wbt_smartcar_bridge mc602_node \
    --ros-args -p serial_port:=/dev/ttyUSB1 -p baud:=1000000

# 3. CLI 验证
ROS_DOMAIN_ID=42 ros2 service call /mc602/buzzer \
    vehicle_wbt_smartcar_msgs/srv/Buzzer "{freq_hz: 440, duration_ms: 200}"
# → 应该听到 0.2 秒 440Hz 蜂鸣

ROS_DOMAIN_ID=42 ros2 topic echo /mc602/state/raw --once
# → 应该有合理的 encoders (可能全是 0)、battery_v (如 11.8V)、ir_*_m (如 1.2m)

ROS_DOMAIN_ID=42 ros2 service call /mc602/read_battery \
    vehicle_wbt_smartcar_msgs/srv/ReadBattery {}
# → voltage_v: 11.85 左右

ROS_DOMAIN_ID=42 ros2 service call /mc602/set_wheels \
    vehicle_wbt_smartcar_msgs/srv/SetWheels "{v0: 30, v1: 30, v2: 30, v3: 30}"
# → 4 轮应该以中等速度转;推一下车
ROS_DOMAIN_ID=42 ros2 service call /mc602/read_encoders \
    vehicle_wbt_smartcar_msgs/srv/ReadEncoders {}
# → encoder 数值应该有变化
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_wheels \
    vehicle_wbt_smartcar_msgs/srv/SetWheels "{v0: 0, v1: 0, v2: 0, v3: 0}"
```

---

## 11. 不做的事(明确边界)

- ❌ 老 C++ `ros2_control` 栈退役(留给 Phase 4)
- ❌ 上层业务节点(底盘/臂/枪/LLM 的业务逻辑)→ 同事做
- ❌ 单元测试 / MockSerial(只接真硬件)
- ❌ LifecycleNode / MultiThreadedExecutor / 自定义错误类型
- ❌ BluetoothPad_2 / LedLight_2 / NixieTube_2 / ScreenShow_2(Phase 5+)
- ❌ 自动刷 `Run.bin`(`SerialWrap.download_bin` 太危险,Phase 5+)
- ❌ IR/电池/按键之外的传感器扩展

---

## 12. 风险 & 决策记录

- **R1**:字节必须 1:1 对齐 SDK(评审已发现现有实现错位)。**决策**:抄 SDK 原文,不重写。
- **R2**:真硬件 24h 内冒烟必须全过。**缓解**:今晚跑通所有 service;Phase 1 失败的设备类不进 bridge。
- **R3**:上层同事的需求可能变化(如 `set_wheels` 想带 PID 模式)。**缓解**:Service 接口预留 `mode` 字段;Phase 1 先无 PID,Phase 2 加。
- **R4**:多个同事同时调用 `/mc602/set_wheels` 会冲突。**缓解**:`MC602Node` 内部串行化(sdk MC602 单例 + lock 已保证);**上层**负责节流(不要 1kHz 调)。
- **R5**:CycloneDDS 在 Wi-Fi (`wlP1p1s0`) 上发现慢。**缓解**:ROS_DOMAIN_ID=42 已硬编码;Network 文档说明端口放行。

---

## 13. 关键文件清单

```
新建:
- ros2_ws/src/vehicle_wbt_smartcar_hw/                        (~1300 行,抄 SDK)
- ros2_ws/src/vehicle_wbt_smartcar_msgs/                       (~150 行)
- ros2_ws/src/vehicle_wbt_smartcar_bridge/                     (~400 行)
- deploy/systemd/vehicle-wbt-mc602.service                     (~25 行)
- deploy/ros_env.sh                                           (~15 行)
- deploy/cyclonedds/cyclonedds.xml                            (~30 行)
- deploy/udev/99-usbvideo.rules                               (~10 行)
- docs/integration/LOWLEVEL_API.md                            (~250 行)
- docs/superpowers/plans/2026-07-09-rak-car-mc602-phase1-plan.md (本文件)

修改:
- CLAUDE.md                                                   (新架构 + 部署说明)
- ros2_ws/src/vehicle_wbt_smartcar_bridge/launch/mc602.launch.py (新建, ~30 行)
- ros2_ws/src/vehicle_wbt_smartcar_bridge/setup.py            (修改)
- ros2_ws/src/vehicle_wbt_smartcar_bridge/package.xml          (修改)
```

---

## 14. 给上层 4 个同事的"开发上手 5 分钟"指南

放在 `docs/integration/LOWLEVEL_API.md` 末尾:

```bash
# 在你自己的 dev 机器上(假设已经 clone 了 rak-car)
cd ~/workspace/rak-car/ros2_ws
colcon build --packages-up-to vehicle_wbt_smartcar_bridge
source install/setup.bash

# 1. 验证能连通 Jetson
ROS_DOMAIN_ID=42 ros2 service call /mc602/read_battery \
    vehicle_wbt_smartcar_msgs/srv/ReadBattery {}

# 2. 看数据流
ROS_DOMAIN_ID=42 ros2 topic echo /mc602/state/raw

# 3. 试着发个指令
ROS_DOMAIN_ID=42 ros2 service call /mc602/buzzer \
    vehicle_wbt_smartcar_msgs/srv/Buzzer "{freq_hz: 440, duration_ms: 200}"

# 4. 创建你自己的 package
ros2 pkg create --build-type ament_python vehicle_wbt_smartcar_chassis \
    --dependencies vehicle_wbt_smartcar_msgs rclpy

# 5. 在 setup.py 里加 entry_point,在包目录下写你的节点,调 /mc602/* service
```

---

**Spec end.** 用户确认后进入 writing-plans 写详细 TODO list。