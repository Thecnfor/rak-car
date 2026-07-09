# WhalesBot 智能车硬件接口控制方法详解

本文档基于 [baidu_smartcar_2026](file:///home/xrak/workspace/scratch/baidu_smartcar_2026) 项目源码，详述下位机 MC602 控制器各硬件接口的上层控制方法。所有 P/M/S 口的协议帧格式统一为：

```
[77 68] [len] [dev_id mode port_id arg1 arg2 ...] [0A]
   头      长度         payload (struct packed)           尾
```

帧打包与发送由 [DevCmdInterface](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L89-L182) 完成，串口握手与回包解析由 [SerialWrap](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/serial_wrap.py) 实现。

---

## 1. P 口（传感器输入）

P 口是 WhalesBot 主板的传感器接口。下位机协议上统一由 `dev_id=0x07`（部分第二路由 `0x08`）代表，靠 `mode` 子类型区分具体传感器。

### 1.1 协议层设备清单

定义在 [mc602_ctl2.py ctl602_dev_list](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L26-L48)：

| 上层类 | dev_id | mode | struct 格式 | 物理意义 |
|--------|--------|------|------------|----------|
| `AnalogInput_2` | 0x07 | 0 | `<b b H` | 通用模拟量输入（0~4095 ADC） |
| `Infrared_2` | 0x07 | 1 | `<b b H` | 红外测距（返回 mm） |
| `Touch_2` | 0x07 | 2 | `<b b H` | 触摸按键（0/1） |
| `Ultrasonic_2` | 0x07 | 3 | `<b b H` | 超声波测距 |
| `Ambient_2` | 0x07 | 4 | `<b b H` | 环境光传感器 |
| `Sensor_Analog2_2` | 0x08 | 0 | `<b b H` | 第二路模拟输入（A 组） |

P 口读取的核心调用是 `DevCmdInterface.no_act(port_id=...)`，组帧形如：

```
77 68 03 07 01 0A           # 读 port=1 的模拟输入
77 68 03 07 81 01 0A         # mode=1(port红外), port=1
```

回包时 `serial_wrap.MC602.get_anwser` 会返回剥掉头尾和 dev_id 后的 `res[3:-1]`，即 `[mode, port, value]` 三字节。

### 1.2 通用模拟输入 `AnalogInput`

源码：[controller_wrap.py#L188-L195](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L188-L195)

```python
from smartcar.whalesbot.vehicle import AnalogInput

sensor = AnalogInput(port_id=1)
value = sensor.read()        # 返回浮点 ADC 值 0.0~4095.0
```

机械臂里用模拟输入做限位检测：[arm_base.py#L121](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/arm/arm_base.py#L121)：

```python
self.y_limit_sensor = AnalogInput(limit_port)
# 磁敏传感器值 > 1000 认为到达限位
return self.y_limit_sensor.read() > 1000
```

### 1.3 红外测距 `Infrared`

源码：[controller_wrap.py#L207-L215](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L207-L215)

```python
from smartcar.whalesbot.vehicle import Infrared

ir = Infrared(port_id=1)
distance_m = ir.read()       # 单位 m（内部 /1000 把 mm 转 m）
```

### 1.4 第二路模拟输入 `AnalogInput2`

源码：[controller_wrap.py#L197-L204](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L197-L204)

```python
from smartcar.whalesbot.vehicle import AnalogInput2

sensor2 = AnalogInput2(port_id=1)
value = sensor2.read()       # float, dev_id=0x08 那一路
```

### 1.5 4 键按键 `Key4Btn`（P 口模拟复用）

源码：[mc602_ctl2.py#L317-L402](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L317-L402) + [controller_wrap.py#L274-L286](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L274-L286)

P 口读出来是个模拟电压（电阻分压），下位机返回 [0~4095]，上层用查表区分按键：

```python
key_map = {3:355, 1:1366, 2:2137, 4:2988}  # 4 个按键对应的 ADC 中值
```

```python
from smartcar.whalesbot.vehicle import Key4Btn

key = Key4Btn(port_id=4)        # 接 4 键按键板（采集板上默认 port=4）
btn = key.read()                # 阻塞读一次，返回按键编号 1~4，松开返回 0
# 返回值: 1/2/3/4 为按键，0 为松开
# 内部已做长按/短按状态机
```

机械臂里用 `Key4Btn(4)` 实现手动调试模式（[arm_base.py#L402-L420](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/arm/arm_base.py#L402-L420)）：按键 1/2/3/4 分别控制机械臂上/下/左/右。

### 1.6 板载按键 `BoardKey`

源码：[mc602_ctl2.py#L300-L305](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L300-L305)

dev_id=0x0d，结构 `<b b b`（dev_id, mode, value）。**板载按键不需要端口号**，直接读：

```python
from smartcar.whalesbot.vehicle.base.mc602_ctl2 import BoardKey_2
key = BoardKey_2()
state = key.no_act()           # 返回 [key1, key2] 状态 0/1
```

### 1.7 数码管 `NixieTube`

源码：[mc602_ctl2.py#L405-L410](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L405-L410)

dev_id=0x0f，`<b b i`：

```python
from smartcar.whalesbot.vehicle import NixieTube
nixie = NixieTube(port_id=1)
nixie.set_number(1234)         # 显示 1234
```

---

## 2. M 口（电机输出）

M 口是带编码器的电机接口。协议层分两类：编码电机（直流 + 编码器）和步进电机（开环脉冲）。

### 2.1 协议层设备清单

| 类 | dev_id | mode | struct 格式 | 物理意义 |
|----|--------|------|------------|----------|
| `Motor_2` | 0x02 | 2 | `<b b b` | 单路 M 口电机（dev, port, speed） |
| `Motor4_2` | 0x01 | 2 | `<b b b b b` | 四路 M 口电机（dev, m1, m2, m3, m4） |
| `EncoderMotor_2` | 0x04 | 1 | `<b b i` | 单路编码器读（dev, port, value） |
| `EncoderMotors4_2` | 0x03 | 1 | `<b i i i i` | 四路编码器一次读 |
| `Stepper_2` | 0x11 | 2 | `<b b i` | 步进电机 PWM 频率 |
| `Stepper_2` | 0x11 | 1 | `<b b i` | 步进电机当前步数 |

### 2.2 单路 M 口电机 `Motor`

源码：[mc602_ctl2.py#L223-L237](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L223-L237)

M 口电机直接下发的速度单位是 **-100~100**（MC602 内部换算成 PWM 占空比）：

```python
from smartcar.whalesbot.vehicle.base.mc602_ctl2 import Motor_2

motor = Motor_2(port_id=1, reverse=1)    # reverse=-1 可反转方向
motor.set_speed(50)                      # 50% 占空比正转
# 内部组帧: 77 68 04 02 02 01 32 0A
# 03 02 02 01 32: dev=0x02, mode=2(set), port=1, speed=50

motor.set_speed(0)                       # 停
motor.set_dir(-1)                        # 动态切换方向
```

`Motor` 单电机上层封装（带编码器 + 物理量转换）：[controller_wrap.py#L321-L361](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L321-L361)

```python
from smartcar.whalesbot.vehicle import Motor

m = Motor(port_id=1, reverse=1, type="motor_280")
# type 选项: motor_280 (减速比 (28/11)^4)
#            motor_280_0 (48*46)

m.set_sp(50)                  # 直接设 -100~100 虚拟速度
m.set_angular(math.pi)        # 设角速度 rad/s，自动换算
m.get_encoder()               # 读编码器原始脉冲
m.get_rad()                   # 编码器 → 弧度
m.reset()                     # 清零编码器
```

### 2.3 四路 M 口电机 `Motors`（麦克纳姆轮专用）

源码：[mc602_ctl2.py#L419-L460](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L419-L460)

四路 M 口通过 [DevListWrap.get_all](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L194-L210) **一次发一长帧**，下位机按 size 切分回包，提升实时性：

```python
from smartcar.whalesbot.vehicle.base.mc602_ctl2 import Motors_2

motors = Motors_2(ports=[1, 2, 3, 4], reverse=False)
# 一次下发 4 路速度，效率是单电机的 4 倍
motors.set_speed([30, 30, 30, 30])

# 一次读 4 路编码器
enc = motors.get_encoder()    # [e1, e2, e3, e4]

motors.reset_encoder()        # 全部清零
motors.reset()                # 同时清电机状态 + 编码器
```

`Motors` 上层封装（[controller_wrap.py#L375-L416](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L375-L416)）：

```python
from smartcar.whalesbot.vehicle import Motors

wheels = Motors(port_list=[1, 2, 3, 4], reverse=False, type="motor_280")
wheels.set_speed([20, 20, 20, 20])              # 4 路虚拟速度
wheels.set_angular([math.pi]*4)                 # 4 路角速度
wheels.get_encoder()                            # 4 路编码器
wheels.get_rad()                                # 4 路弧度
wheels.reset()                                  # 编码器清零
```

### 2.4 单电机速度闭环封装 `MotorWrap`

源码：[controller_wrap.py#L559-L585](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L559-L585)

把"虚拟速度 -100~100"和"线速度 m/s"互转，给机械臂 X 轴电机用：

```python
from smartcar.whalesbot.vehicle import MotorWrap

arm_x = MotorWrap(id=1, reverse=-1, type="motor_280", perimeter=0.06*math.pi)
arm_x.set_linear(0.1)         # 0.1 m/s
arm_x.set_angular(math.pi)    # π rad/s
arm_x.get_dis()               # 当前位移 m
arm_x.get_rad()               # 当前弧度
arm_x.reset()                 # 编码器清零
```

### 2.5 步进电机 `StepperWrap`

源码：[controller_wrap.py#L587-L636](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L587-L636)

步进电机走 `dev_id=0x11`。机械臂 Y 轴（竖直方向）就是用它：

```python
from smartcar.whalesbot.vehicle import StepperWrap

y_motor = StepperWrap(id=1, reverse=1, perimeter=0.008)
# 步进值 1.8 度, 16 细分, 2 相位
# stepper2rad = pi/180 * 1.8/16

y_motor.set_angular(math.pi/4)    # 设角速度 rad/s
y_motor.set_velocity(0.05)        # 设线速度 m/s
y_motor.set_rad(math.pi/5*2)      # 走到指定弧度（PID 闭环）
y_motor.get_rad()                 # 当前弧度
y_motor.get_dis()                 # 当前位移 m
y_motor.reset()                   # 清零
```

`set_rad()` 内部用 PID 把当前步数闭环到目标位置，是机械臂安全升降的关键。

### 2.6 速度单位换算工具 `MotorConvert`

源码：[controller_wrap.py#L42-L86](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L42-L86)

```python
# 编码器一圈脉冲 = 12 栅格 × 4 倍频 × 减速比(28/11)^4 ≈ 2015.13
encoder_resolution = 2015.12792842019
speed_rate = 100                   # 虚拟速度 100 → 编码器 1 圈/秒

# virtual_speed (-100~100) → encoder 速度 → m/s
sp2virtual(): virtual → encoder/s
sp2true():     virtual → m/s
dis2encoder(): m → encoder
encoder2dis(): encoder → m
```

---

## 3. 智能舵机（总线舵机）

源码：[mc602_ctl2.py#L481-L490](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L481-L490) + [controller_wrap.py#L453-L465](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L453-L465)

智能总线舵机走 `dev_id=0x06`，`<b b b b h>`，可以同时控制位置 + 速度。

### 3.1 协议层

| mode | cmd | 含义 |
|------|-----|------|
| 1 | 2 | 设置角度（位置模式） |
| 2 | 2 | 设置速度（速度模式） |
| 1 | 1 | 读角度 |

### 3.2 上层类 `ServoBus`

```python
from smartcar.whalesbot.vehicle import ServoBus

servo = ServoBus(port_id=1)       # port_id = 总线上的舵机 ID（1/2/3/4...）
# 设置角度，速度 80
servo.set_angle(angle=90, speed=80)    # angle 单位 0~300 度
# 设置速度模式
servo.set_speed(50)                    # 速度 -300~300
```

组帧示例（舵机 1 转 90°，速度 80）：
```
77 68 06 06 01 02 01 50 5A 0A
# 06 06 01 02 01 50 5A:
#   dev_id=0x06, port=1, mode=2(set), act=1(angle), speed=80, angle=90
```

机械臂用 `ServoBus` 控制手臂摆向（[arm_base.py#L318](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/arm/arm_base.py#L318)）：

```python
self.arm_servo = ServoBus(hand["port"])
self.arm_servo.set_angle(angle, speed)
# 预设位置: "LEFT" / "MID" / "RIGHT" → 从 arm_cfg.yaml 的 angle_list 查
```

### 3.3 预设位置

`arm_cfg.yaml` 里的 `hand_cfg.hand.angle_list` 提供了语义化预设：

```yaml
hand:
  port: 1
  angle_list:
    LEFT: 30
    MID: 90
    RIGHT: 150
```

调用 `set_arm_angle("LEFT")` 就会取 30 度下发。

---

## 4. PWM 舵机（普通舵机）

源码：[mc602_ctl2.py#L474-L479](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L474-L479) + [controller_wrap.py#L442-L451](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L442-L451)

普通 PWM 舵机走 `dev_id=0x05`，`<b b B B>`：

### 4.1 协议层

| mode | 含义 |
|------|------|
| 1 | 读 |
| 2 | 设置（speed, angle） |

### 4.2 上层类 `ServoPwm`

```python
from smartcar.whalesbot.vehicle import ServoPwm

# mode=180: 输入 0~180 度，内部转成 0~255 PWM 占空比
# mode=90:  输入 0~90 度
servo = ServoPwm(port_id=2, mode=180)
servo.set_angle(angle=90, speed=100)    # 速度 100，角度 90°
# 内部换算: angle_pwm = int(90/180*180+90) = 180
# 帧: 77 68 05 05 02 02 64 B4 0A
#     05 05 02 02 64 B4: dev=0x05, port=2, mode=2, speed=100, angle_pwm=180
```

机械臂用 `ServoPwm` 控制手部俯仰（[arm_base.py#L316](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/arm/arm_base.py#L316)）：

```python
self.hand_servo = ServoPwm(hand2["port"], mode=hand2["mode"])
self.hand_servo.set_angle(angle, speed)
# 预设: "UP" / "MID" / "DOWN" → 从 angle_list2 查
```

---

## 5. PWM 输出 / 数字 IO

### 5.1 通用数字输出 `PoutD`

源码：[mc602_ctl2.py#L512-L517](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L512-L517) + [controller_wrap.py#L467-L473](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/controller_wrap.py#L467-L473)

dev_id=0x10，`<b b b>`，直接拉高/拉低一个 GPIO：

```python
from smartcar.whalesbot.vehicle import PoutD

pwm = PoutD(port=2)             # 接在 P 端口号 port=2 上
pwm.set(1)                       # 高电平（PWM 持续输出）
pwm.set(0)                       # 低电平（关闭）
# 帧: 77 68 04 10 02 02 01 0A  (set 1)
#     77 68 04 10 02 02 00 0A  (set 0)
```

机械臂用它控制气泵和阀门（[arm_base.py#L320-L331](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/arm/arm_base.py#L320-L331)）：

```python
self.pump = PoutD(grap["port_pump"])      # 气泵 P 口
self.valve = PoutD(grap["port_valve"])    # 阀门 P 口

def grasp(self, value: bool):
    self.pump.set(not value)    # True(抓取)=吸气: pump=0, valve=1
    self.valve.set(value)       # False(释放)=放气: pump=1, valve=0
```

### 5.2 步进电机的 PWM 输出

步进电机 `Stepper_2` 内部就是用 PWM 频率驱动步进电机驱动器：

```python
from smartcar.whalesbot.vehicle.base.mc602_ctl2 import Stepper_2
stepper = Stepper_2(port_id=1)
stepper.set_pwm(2000)            # 设频率 2000Hz
stepper.get_step()               # 读当前步数
stepper.set(0)                   # 停
```

### 5.3 屏幕/灯阵输出 `ScreenShow` `LedLight`

```python
from smartcar.whalesbot.vehicle.base.mc602_ctl2 import ScreenShow_2, LedLight_2
# 屏幕: dev_id=0x0b, 101 字节
screen = ScreenShow_2()
screen.show("Hello, 2026!")

# LED 灯条: dev_id=0x0e, 5 字节 (id, r, g, b)
led = LedLight_2(port_id=1)
led.set_light(led_id=1, r=255, g=0, b=0)    # 红色
```

---

## 6. 模拟输入（汇总）

所有模拟输入走 `dev_id=0x07`（mode 区分类型）或 `dev_id=0x08`（第二路）。

```python
from smartcar.whalesbot.vehicle import AnalogInput, AnalogInput2, Infrared

# 通用 ADC
a1 = AnalogInput(port_id=1)
v1 = a1.read()        # float 0.0~4095.0

# 红外测距（自动 /1000 转 m）
ir = Infrared(port_id=1)
d = ir.read()         # 单位 m

# 第二路 ADC（dev_id=0x08）
a2 = AnalogInput2(port_id=1)
v2 = a2.read()
```

读取都走 `no_act()` 模式，下位机返回 `[mode, port, value]` 三字节，`controller_wrap` 解包后返回 `value` 字段。

---

## 7. 其他常用 IO

### 7.1 蜂鸣器 `Beep`

源码：[mc602_ctl2.py#L214-L221](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L214-L221)

dev_id=0x0a，`<b B B>`，下发的两个参数是 `freq//2` 和 `duration*20`：

```python
from smartcar.whalesbot.vehicle import Beep
beep = Beep()
beep.rings(freq=262, duration=0.2)    # 262Hz 持续 0.2s
# 帧: 77 68 04 0a 02 83 04 0A
#     0a 02 83 04: dev=0x0a, mode=2, freq//2=131, dur*20=4
```

### 7.2 电池电压 `Battry`

dev_id=0x0c，`<b i>`：

```python
from smartcar.whalesbot.vehicle import Battry
bat = Battry()
v = bat.read()        # 单位 V
```

### 7.3 蓝牙手柄 `BluetoothPad`

dev_id=0x09，`<b B B B B i>`：

```python
from smartcar.whalesbot.vehicle import BluetoothPad
pad = BluetoothPad()
sticks, btn = pad.read()    # [左x, 左y, 右x, 右y], 按键 bitmask
# 按键 bitmask 编码：bit n 表示第 n 个键按下
```

### 7.4 4 路编码器读 `EncoderMotors4_2`

dev_id=0x03，`<b i i i i>`：

```python
from smartcar.whalesbot.vehicle.base.mc602_ctl2 import EncoderMotors4_2
enc4 = EncoderMotors4_2()
vals = enc4.get()           # [e1, e2, e3, e4]
```

---

## 8. 速查表

| 物理口 | dev_id | mode | struct | 上层类 | 关键方法 |
|--------|--------|------|--------|--------|---------|
| M 口单电机 | 0x02 | 2 | `<b b b` | `Motor_2(port).set_speed(v)` | 速度 -100~100 |
| M 口四电机 | 0x01 | 2 | `<b b b b b` | `Motors_2(ports).set_speed([v1,v2,v3,v4])` | 一次发 4 路 |
| 编码器单 | 0x04 | 1 | `<b b i` | `EncoderMotor_2(port).get_encoder()` | 原始脉冲 |
| 编码器四 | 0x03 | 1 | `<b i i i i` | `EncoderMotors4_2().get()` | 一次取 4 路 |
| 步进电机 | 0x11 | 2 | `<b b i` | `Stepper_2(port).set_pwm(freq)` | PWM 频率 |
| 步进位置 | 0x11 | 1 | `<b b i` | `Stepper_2(port).get_step()` | 当前步数 |
| 智能舵机 | 0x06 | 2 | `<b b b b h` | `ServoBus(port).set_angle(a, s)` | 位置 + 速度 |
| PWM 舵机 | 0x05 | 2 | `<b b B B` | `ServoPwm(port).set_angle(a, s)` | 0~180° + 速度 |
| P 口模拟 | 0x07 | 0 | `<b b H` | `AnalogInput(port).read()` | 0~4095 |
| P 口红外 | 0x07 | 1 | `<b b H` | `Infrared(port).read()` | mm |
| P 口触摸 | 0x07 | 2 | `<b b H` | `Touch(port).read()` | 0/1 |
| P 口超声 | 0x07 | 3 | `<b b H` | `Ultrasonic(port).read()` | mm |
| P 口环境光 | 0x07 | 4 | `<b b H` | `Ambient(port).read()` | 亮度 |
| 第二路模拟 | 0x08 | 0 | `<b b H` | `AnalogInput2(port).read()` | 0~4095 |
| 4 键按键 | 0x07 | 0 | `<b b H` | `Key4Btn(port).read()` | 1~4/0 |
| 数字输出 | 0x10 | 2 | `<b b b` | `PoutD(port).set(v)` | 0/1 高低电平 |
| 蜂鸣器 | 0x0a | 2 | `<b B B` | `Buzzer_2().rings(f, d)` | freq, dur |
| 电池 | 0x0c | 1 | `<b i` | `Battry_2().read()` | V |
| 蓝牙手柄 | 0x09 | 1 | `<b B B B B i` | `BluetoothPad_2().get_stick()` | [x,y,x,y,key] |
| 板载按键 | 0x0d | 1 | `<b b b` | `BoardKey_2().no_act()` | [k1, k2] |
| LED 灯 | 0x0e | 2 | `<b b B B B B` | `LedLight_2(port).set_light(...)` | (id,r,g,b) |
| 数码管 | 0x0f | 2 | `<b b i` | `NixieTube_2(port).set_number(n)` | int |
| 屏幕 | 0x0b | 2 | `<b b ×101` | `ScreenShow_2().show(str)` | ASCII |

---

## 9. 同类口的区分方法

> 本节回答：**多个相同类型的口（M 口电机、P 口传感器、舵机、数字输出）如何区分是哪一个？**

### 9.1 核心机制：`port_id`

所有"同类口"在协议帧里共用同一个 `dev_id`（设备类型），靠第 3 个字节 `port_id`（物理端口号）区分。

源码：[mc602_ctl2.py#L89-L142](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L89-L142) `DevCmdInterface.__init__` / `get_bytes`：

```python
class DevCmdInterface:
    def __init__(self, dev_id, mode, port_id, format='bb'):
        self.dev_id  = dev_id    # 设备类型（同一类口都一样）
        self.port_id = port_id   # 物理端口号 ← 同类多口的唯一区分
```

**组帧逻辑**（`get_bytes` 第 109-141 行）：

```python
data = [self.dev_id]               # 1. 设备类型（同类口都同一个）
if self.mode is not None:
    data.append(self.mode)         # 2. 操作模式 get/set/reset
if self.port_id is not None:
    data.append(self.port_id)      # 3. 物理端口号
data += args                       # 4. 参数
```

完整协议帧：

```
[77 68] [len] [dev_id] [mode] [port_id] [arg1 arg2 ...] [0A]
                              ^^^^^^^^
                              这一个字节就是同类多口的区分码
```

### 9.2 方式一：每口一个对象，构造时指定 `port_id`

```python
from smartcar.whalesbot.vehicle import AnalogInput, Motor, ServoPwm, PoutD

# 3 个 P 口模拟传感器（都是 dev_id=0x07，靠 port_id 区分）
ir_front = AnalogInput(port_id=1)    # 1 号 P 口
ir_left  = AnalogInput(port_id=2)    # 2 号 P 口
ir_right = AnalogInput(port_id=3)    # 3 号 P 口

# 4 个 M 口电机（都是 dev_id=0x02，靠 port_id 区分）
m1 = Motor(port_id=1)
m2 = Motor(port_id=2)
m3 = Motor(port_id=3)
m4 = Motor(port_id=4)

# 3 个 PWM 舵机（都是 dev_id=0x05，靠 port_id 区分）
servo1 = ServoPwm(port_id=1, mode=180)
servo2 = ServoPwm(port_id=2, mode=180)
servo3 = ServoPwm(port_id=3, mode=180)

# 2 个数字输出（都是 dev_id=0x10，靠 port_id 区分）
pump  = PoutD(port=2)   # 气泵
valve = PoutD(port=3)   # 阀门
```

**动态切换**（每个对象有 `set_port()`，[mc602_ctl2.py#L106-L107](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L106-L107)）：

```python
dev = AnalogInput(port_id=1)   # 默认 1 号
dev.set_port(3)                 # 动态改到 3 号
v = dev.read()                  # 读 3 号
```

**也可以在调用时临时指定 `port_id`**（`DevCmdInterface.set/get/no_act` 都有 `port_id=None` 形参）：

```python
dev = AnalogInput(port_id=1)   # 默认 1 号
v1 = dev.read()                 # 读 1 号
v3 = dev.no_act(port_id=3)      # 临时读 3 号
```

### 9.3 方式二：`Motors_2(ports=[...])` 批量端口

当需要"同时读写多个同种口"（典型如麦克纳姆轮 4 个编码电机），用 `Motors_2` / `EncoderMotors4_2` 等"批量类"。源码：[mc602_ctl2.py#L419-L460](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L419-L460) + [DevListWrap](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L187-L212)。

**原理**：`Motors_2(ports=[1,2,3,4])` 内部为每个 port 自动建一个 `Motor_2(port_id=i)` 子对象，然后由 `DevListWrap.get_all` 把它们拼成一条长帧发出，回包再按 `data_struct.size` 切回：

```python
class Motors_2:
    def __init__(self, ports, reverse=False):
        self.ports = ports
        self.motors = [Motor_2(i) for i in ports]   # 给每个 port 建子对象
        self.motors_wrap = DevListWrap(self.motors)

    def set_speed(self, speeds):
        return self.motors_wrap.get_all(speeds, mode=2)
```

`DevListWrap.get_all`（[L194-L210](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L194-L210)）：

```python
def get_all(self, args, mode=1):
    bytes_all = b''
    for i in range(len(self.dev_list)):
        # 每个 dev 自己的 dev_id + port_id + arg[i]
        bytes_all += self.dev_list[i].get_bytes(args[i], mode=mode)
    res = serial_mc602.get_anwser(bytes_all)
    # 按 data_struct.size 切回包，分别给每个 dev
    index = 0
    for i in range(len(self.dev_list)):
        data = self.dev_list[i].get_result(res, index)
        index += self.dev_list[i].data_struct.size
        data_ret.append(data)
    return data_ret
```

**用法**：

```python
from smartcar.whalesbot.vehicle.base.mc602_ctl2 import Motors_2, EncoderMotors4_2

# 4 个 M 口电机，ports 顺序就是 0/1/2/3 号
wheels = Motors_2(ports=[1, 2, 3, 4])
wheels.set_speed([m1_speed, m2_speed, m3_speed, m4_speed])
encs = wheels.get_encoder()        # [enc1, enc2, enc3, enc4]
```

下发的实际帧（4 个 `<b b b>` 拼起来）：

```
77 68 0D 02 02 01 V1  02 02 02 V2  02 02 03 V3  02 02 04 V4  0A
        |________|  |________|  |________|  |________|
         port=1      port=2      port=3      port=4
```

回包也是 4 个 `<b b b>` 拼起来，按 size 切，依次放进 `data_ret[0..3]`。

### 9.4 方式三：智能总线舵机用 `port_id` = 总线 ID

智能舵机 [ServoBus](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py#L481-L490) 走 `dev_id=0x06`，它的 `port_id` 是舵机在总线上的 ID（每个舵机出厂烧的 ID，通常 1/2/3/4）：

```python
from smartcar.whalesbot.vehicle import ServoBus

arm  = ServoBus(port_id=1)   # 总线上 ID=1 的舵机（手臂摆向）
hand = ServoBus(port_id=2)   # 总线上 ID=2 的舵机（手部）
arm.set_angle(90, 80)         # 发给 ID=1 那个
hand.set_angle(45, 80)        # 发给 ID=2 那个
```

### 9.5 方式对比

| 方式 | 适用场景 | 区分字段 | 例子 |
|------|----------|----------|------|
| 每口一个对象 | 任意口，单口操作灵活 | `port_id`（构造时或运行时设置） | `AnalogInput(port_id=1)`、`Motor(port_id=2)` |
| `Motors_2(ports=[1,2,3,4])` | 4 个同种口，需要同时读写（如麦克纳姆轮） | `ports` 列表 + 速度/编码器值按位对应 | `Motors_2([1,2,3,4]).set_speed([v1,v2,v3,v4])` |
| 智能总线舵机 `ServoBus(port_id=ID)` | 多个舵机挂在同一条物理总线上 | 总线 ID | `ServoBus(port_id=1)`、`ServoBus(port_id=2)` |
| 数字输出 `PoutD(port=N)` | 多个 GPIO | 输出口号 | `PoutD(2)`、`PoutD(3)` |
| 步进电机 `StepperWrap(id=N)` | 多个步进电机 | `id` | `StepperWrap(id=1)`、`StepperWrap(id=2)` |

### 9.6 总结

**一句话**：协议里有一个固定的 `port_id` 字节，**同类口下多设备的唯一区分就是这个字节**。

- 单口操作灵活：每口一个对象，每个对象带自己的 `port_id`。
- 批量操作高效：用 `Motors_2(ports=[...])` / `EncoderMotors4_2()` 一次性批量访问（提升实时性，代价是灵活性差一些）。
- 动态切换：每个对象都有 `set_port()` 和 `set/get/no_act(port_id=...)` 临时覆盖。

---

## 10. 统一调用流程

所有上层类的调用，最终都走同一条链路：

```
Python 类 .set_xxx() / .read()
   ↓
DevCmdInterface.send_get(bytes_payload)
   ↓
serial_mc602.get_anwser(payload, timeout)
   ↓
SerialWrap → pyserial.write(payload) → CH340 → MC602
   ↓
MC602 解析 [77 68 len payload 0A] → 执行 → 返回
   ↓
SerialWrap.MC602.get_anwser() 读回包、剥头尾
   ↓
DevCmdInterface.get_result() 解 struct
   ↓
上层类返回数值
```

协议帧由 [MC602.send_cmd](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/serial_wrap.py#L215-L219) 自动加 `[77 68] [len] [...] [0A]`，回包由 [MC602.get_anwser](file:///home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/serial_wrap.py#L222-L245) 验证头尾并剥壳，使用者只需要关心 `dev_id / mode / port_id / 参数值`。
