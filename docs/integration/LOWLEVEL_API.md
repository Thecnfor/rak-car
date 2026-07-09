# MC602 Low-Level API

Jetson 端 MC602 节点暴露的通用 ROS2 接口,供上层业务节点(底盘/机械臂/枪/LLM)远程调用。

## 环境

- **Jetson IP**: `192.168.3.69`(wi-fi `wlP1p1s0`)
- **ROS_DOMAIN_ID**: `42`(Jetson 启动文件硬编码,开发机需 export)
- **RMW**: CycloneDDS(开发机也建议装 `rmw-cyclonedds-cpp` 保持一致)

## 快速验证(开发机)

```bash
# 1. 确保 ROS_DOMAIN_ID 一致
export ROS_DOMAIN_ID=42

# 2. 拉数据
ros2 topic echo /vehicle_wbt/v1/mc602/state/raw
ros2 topic echo /vehicle_wbt/v1/mc602/heartbeat

# 3. 调服务
ros2 service list | grep /vehicle_wbt/v1/mc602
ros2 service call /vehicle_wbt/v1/mc602/read_battery vehicle_wbt_smartcar_msgs/srv/ReadBattery
ros2 service call /vehicle_wbt/v1/mc602/buzzer vehicle_wbt_smartcar_msgs/srv/Buzzer "{freq_hz: 440, duration_ms: 200}"
ros2 service call /vehicle_wbt/v1/mc602/set_wheels vehicle_wbt_smartcar_msgs/srv/SetWheels "{v0: 0, v1: 0, v2: 0, v3: 0}"
```

## Service 接口

### `/vehicle_wbt/v1/mc602/set_wheels` (SetWheels)

```python
req = SetWheels.Request(v0=30, v1=30, v2=30, v3=30)  # -100..100
# 4 个麦轮速度,内部用 Motors_2.set_speed 转发
```

### `/vehicle_wbt/v1/mc602/read_encoders` (ReadEncoders)

返回 4 个 int32 编码器值(增量式,从下位机读取,推一下车数字会变)。

```python
req = ReadEncoders.Request()
resp = await client.call_async(req)
print(resp.v)  # [v0, v1, v2, v3]
```

### `/vehicle_wbt/v1/mc602/reset_encoders` (ResetEncoders)

清零所有编码器。建议在 chassis_node 启动时调用一次。

### `/vehicle_wbt/v1/mc602/set_servo_pwm` (SetServoPwm)

PWM 舵机(机械臂手爪)。`port` 来自 `mc602_ports.yaml::arm::servo_pwm_hand::port`(默认 3)。

```python
req = SetServoPwm.Request(port=3, angle=60)  # 0..180
```

### `/vehicle_wbt/v1/mc602/set_servo_bus` (SetServoBus)

总线舵机(机械臂腕部)。`port=4`,`angle` + `speed`(速度,1..100)。

```python
req = SetServoBus.Request(port=4, angle=90, speed=60)
```

### `/vehicle_wbt/v1/mc602/set_stepper` (SetStepper)

步进电机速度(机械臂 Y 轴)。`port=1`,`freq` 是 -100..100 的速度。

### `/vehicle_wbt/v1/mc602/set_dc_motor` (SetDcMotor)

直流电机速度(机械臂 X 轴)。`port=2`,`speed` -100..100。

### `/vehicle_wbt/v1/mc602/set_pout` (SetDout)

通用数字输出。`port` 来自 `mc602_ports.yaml`:

| port 含义 | 设备 |
|---|---|
| 1 | 机械臂真空泵 |
| 2 | 机械臂真空阀 |
| 4 | 枪发射器 |

```python
req = SetDout.Request(port=1, state=True)  # 开泵
req = SetDout.Request(port=2, state=False)  # 关阀
req = SetDout.Request(port=4, state=True)  # 发射
```

### `/vehicle_wbt/v1/mc602/read_ir` (ReadIR)

红外测距。`port` 7=左, 8=右。返回 `distance_m`(米,float)。

### `/vehicle_wbt/v1/mc602/read_battery` (ReadBattery)

电池电压(伏)。返回 `voltage_v`。

### `/vehicle_wbt/v1/mc602/read_analog` (ReadAnalog)

模拟输入。`port=6` 是机械臂 Y 轴磁限位。返回原始 0..4095。

### `/vehicle_wbt/v1/mc602/buzzer` (Buzzer)

蜂鸣器。`freq_hz` 频率,`duration_ms` 持续时间。

## Topic 接口

### `/vehicle_wbt/v1/mc602/state/raw` (RawState.msg)

20 Hz 发布。字段:

| 字段 | 类型 | 含义 |
|---|---|---|
| `header.stamp` | time | 时间戳 |
| `encoders` | int32[4] | 4 轮编码器 |
| `ir_left_m` | float32 | 左红外距离(米) |
| `ir_right_m` | float32 | 右红外距离(米) |
| `battery_v` | float32 | 电池电压 |
| `arm_y_pos` | int32 | 臂 Y 轴(Phase 1 固定 0) |
| `arm_x_pos` | int32 | 臂 X 轴(Phase 1 固定 0) |
| `pump_on` | bool | 泵状态 |
| `valve_on` | bool | 阀状态 |

### `/vehicle_wbt/v1/mc602/heartbeat` (std_msgs/Header)

1 Hz 发布。检测 Jetson 端存活。

## 常用模式

### 底盘 50Hz 读编码器 + 算 odom

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vehicle_wbt_smartcar_msgs.msg import RawState
from vehicle_wbt_smartcar_hw import Odometry, MecanumChassis  # 可在 chassis 包里间接依赖

class ChassisNode(Node):
    def __init__(self):
        super().__init__('chassis')
        self._odom = Odometry()
        self._chassis = MecanumChassis(track=0.30, wheel_base=0.28)
        self._prev_encs = [0, 0, 0, 0]
        self.create_subscription(RawState, '/vehicle_wbt/v1/mc602/state/raw', self._on_raw, 10)
        self._odom_pub = self.create_publisher(Odometry, '/chassis/odom', 10)

    def _on_raw(self, msg: RawState):
        # 把编码器 delta 积分到 odom
        ...
```

### 机械臂发 pose + 等反馈

```python
from vehicle_wbt_smartcar_msgs.srv import SetServoPwm, SetServoBus

client_pwm = self.create_client(SetServoPwm, '/vehicle_wbt/v1/mc602/set_servo_pwm')
client_bus = self.create_client(SetServoBus, '/vehicle_wbt/v1/mc602/set_servo_bus')

# 手爪角度
req = SetServoPwm.Request(port=3, angle=90)
await client_pwm.call_async(req)

# 腕部角度 + 速度
req = SetServoBus.Request(port=4, angle=120, speed=80)
await client_bus.call_async(req)
```

## 错误处理

- 每个 service 调用失败时 `success: false`,**不会抛异常**(跨进程边界)
- 读类 service 失败时填默认值(0 / 0.0 / false)
- 写类 service 失败时**不保证硬件状态**;建议上层用 `/vehicle_wbt/v1/mc602/read_ir` 之类读回做验证
- 多个 client 并发调同一 service:内部 SDK 单例 + lock 保证串口帧不交错,但**应用层应节流**(50Hz 调一次足够)

## 限制(Phase 1)

- ❌ 底盘 PID `move_to_position`:Phase 1 不实现,底盘同事自己写
- ❌ 机械臂 PID 闭环:Phase 1 不实现,机械臂同事自己写
- ❌ 老 C++ `ros2_control` 栈仍在运行:Phase 4 退役,**注意 chassis/arm 服务可能被双向消费**——同事开发时暂时不用老 C++ 的话题(`/cmd/vel_safe` 等),只调 `/vehicle_wbt/v1/mc602/*`

## 完整 21 个 service 列表

| Service | 字段 in/out | 物理意义 |
|---|---|---|
| `/set_wheels` | v0,v1,v2,v3 (int8) → success | 麦纳姆轮 4 路速度 |
| `/read_encoders` | → v[4] (int32) + success | 4 路编码器原始脉冲 |
| `/reset_encoders` | → success | 编码器清零 |
| `/set_servo_pwm` | port, angle → success | PWM 舵机(机械臂手爪) |
| `/set_servo_bus` | port, angle, speed → success | 总线舵机(机械臂腕部) |
| `/set_stepper` | port, freq → success | 步进电机(机械臂 Y 轴) |
| `/set_dc_motor` | port, speed → success | 直流电机(机械臂 X 轴) |
| `/set_pout` | port, state → success | 数字输出(泵/阀/枪) |
| `/read_ir` | port → distance_m + success | 红外测距 (m) |
| `/read_battery` | → voltage_v + success | 电池电压 (V) |
| `/read_analog` | port → value + success | 通用 ADC (0~4095) |
| `/buzzer` | freq_hz, duration_ms → success | 蜂鸣器单音 |
| `/read_touch` | port → pressed + success | 触摸按键 (P 口 mode=2) |
| `/read_ultrasonic` | port → distance_m + success | 超声波测距 (P 口 mode=3) |
| `/read_ambient` | port → value + success | 环境光 (P 口 mode=4) |
| `/read_bluetooth` | → lx,ly,rx,ry,btn + success | 蓝牙手柄(dev_id=0x09) |
| `/read_key4` | port → key (1-4/0) + success | 4 键按键板 (P 口) |
| `/set_led` | led_id,r,g,b,port → success | RGB LED 灯条(dev_id=0x0e) |
| `/set_nixie` | port, num → success | 数码管整数显示(dev_id=0x0f) |
| `/show_screen` | text → success | 屏幕 ASCII(dev_id=0x0b) |
| `/play_predefined` | name → success + message | 预置旋律 (twinkle/mary/birthday) |

**Topic**:
- `/play_melody` (Melody.msg): 任意 melody pub
- `/state/raw` (RawState.msg, 20Hz): encoders + IR + battery + arm pose + pump/valve + board_key
- `/board/button_events` (ButtonEvent.msg, edge): 板载按钮按下/松开
- `/heartbeat` (std_msgs/Header, 1Hz): alive

## 硬件协议参考

完整 MC602 帧格式 + dev_id/mode/port_id 速查表,见 [MC602_HARDWARE_REFERENCE.md](MC602_HARDWARE_REFERENCE.md)。
