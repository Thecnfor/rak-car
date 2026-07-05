# MC602 硬件接口物理映射

> **本文聚焦物理接线：哪个端口接了什么东西、在代码的哪个位置初始化。**
>
> 📡 协议层（帧格式、命令表、struct 定义、操作模式）请参阅 [hardware-comm.md](hardware-comm.md)。
> 两者互补 — 本文回答"M5 接了什么"，`hardware-comm.md` 回答"M5 的 `02 02 05 [speed]` 帧怎么构造"。

---

## 快速定位表

| 端口 | 物理接线 | 代码类 | 初始化位置 |
|:----:|---------|--------|-----------|
| M1 | 右前轮（麦克纳姆） | `WheelWrap([1,2,3,4])` | `cfg_vehicle.yaml` → `vehicle_base.py:312` |
| M2 | 左前轮（麦克纳姆） | 同上 | 同上 |
| M3 | 左后轮（麦克纳姆） | 同上 | 同上 |
| M4 | 右后轮（麦克纳姆） | 同上 | 同上 |
| M5 | 弹射推进电机 | `MotorWrap(5, reverse=-1)` | `task_func.py:9` |
| M6 | 机械臂水平丝杆 | `MotorWrap(6, reverse=-1)` | `arm_cfg.yaml:17` → `arm_base.py:112` |
| S2 | 展示牌舵机（总线） | `ServoBus(2)` | `task_func.py:58` |
| S3 | 手爪旋转舵机（总线） | `ServoBus(3)` | `arm_cfg.yaml:5-8` |
| S7 | 手爪开合舵机（PWM, 270°） | `ServoPwm(7, mode=270)` | `arm_cfg.yaml:9-13` |
| P1 | 4按键菜单 | `Key4Btn(1)` | `config_car.yml:7` |
| P2 | LED 灯带 | `LedLight(2)` | `config_car.yml:8` |
| P2 | 真空泵（数字输出） | `PoutD(2)` | `arm_cfg.yaml:15` |
| P3 | 电磁阀（数字输出） | `PoutD(3)` | `arm_cfg.yaml:16` |
| P4 | 弹射气阀（数字输出） | `PoutD(4)` | `task_func.py:10` |
| P6 | 竖直限位传感器（模拟） | `AnalogInput(6)` | `arm_cfg.yaml:44` → `arm_base.py:54` |
| P7 | 右侧红外测距 | `Infrared(7)` | `config_car.yml:7` |
| P8 | 左侧红外测距 | `Infrared(8)` | `config_car.yml:6` |
| 步进1 | 弹射角度调节 | `StepperWrap(1)` | `task_func.py:12` |
| 步进3 | 机械臂竖直丝杆 | `StepperWrap(3, reverse=-1)` | `arm_cfg.yaml:40` → `arm_base.py:113` |

---

## M 口 — 直流电机（dev_id = 0x01 / 0x02）

6 路直流电机接口，支持编码器反馈。

### M1–M4：底盘驱动

```
代码：WheelWrap([1, 2, 3, 4], raduis=0.03)
文件：cfg_vehicle.yaml → vehicle_base.py:312
底盘：MecanumChassis（麦克纳姆轮 O 形布局）

轮子分布（俯视，车头朝前）：
    M2(左前) ←──→ M1(右前)
    M3(左后) ←──→ M4(右后)
```

- 每个轮子半径 3cm
- 电机类型 `motor_280`，减速比 `(28/11)^4 ≈ 41.98`
- 编码器分辨率 2015.13 counts/输出轴圈

### M5：弹射推进电机

```
代码：MotorWrap(5, reverse=-1)
文件：task_func.py:9 → class Ejection
功能：驱动弹射机构的橡皮筋/弹簧蓄力
```

- `Ejection.eject(x, vel)` — 先回拉 0.105m → 再释放到位置 x
- `x` 不同值对应不同弹射区域（1/2/3 号靶位）

### M6：机械臂水平丝杆电机

```
代码：MotorWrap(6, reverse=-1, perimeter=0.032)
文件：arm_cfg.yaml:17 → arm_base.py:112 → self.horiz_motor
功能：驱动机械臂水平方向伸缩
```

- 行程范围：0 ~ 0.3 m
- PID 参数：Kp=6, Ki=0, Kd=0
- 最大速度：±0.2 m/s

---

## S 口 — 舵机（dev_id = 0x05 / 0x06）

3 路舵机接口（2 路总线舵机 + 1 路 PWM 舵机）。注意 **S1 未使用**。

### S2：展示牌舵机（总线舵机）

```
代码：ServoBus(2)
文件：task_func.py:58 → self.servo_weather
功能：驱动天气展示牌指针旋转到对应天气图标
```

- `weather_set(num)` — 设置指针到 0-4 位置
- `bmi_set(num)` — 设置 BMI 指针

### S3：手爪旋转舵机（总线舵机）

```
代码：ServoBus(3)
文件：arm_cfg.yaml:5-8 → arm_base.py:57 → self.arm_servo
功能：切换机械臂在车身的左右侧
```

- 角度映射：`{-1: -93°, 0: 0°, 1: +93°}`
- `switch_side(-1)` → 转到右边
- `switch_side(1)` → 转到左边

### S7：手爪开合舵机（PWM 舵机, 270° 模式）

```
代码：ServoPwm(7, mode=270)
文件：arm_cfg.yaml:9-13 → arm_base.py:52 → self.hand_servo
功能：控制手爪的倾斜角度（掌心向下 / 掌心向前）
```

- 角度映射（270° 模式下）：`{-1: -45°, 1: +46°}`
- `set_hand_angle(60)` → 掌心向下（默认）
- `set_hand_angle(-80)` → 掌心向前（抓取姿态）

---

## P 口 — 多功能 IO（dev_id = 0x07 / 0x0E / 0x10）

8 路 RJ11 多功能接口，可复用作模拟输入、数字输入、数字输出等。

> ⚠️ **P2 和 P5 各承载两个设备** — 同一物理端口上挂了不同功能的硬件，通过 dev_id 区分。P2 同时接 LED 灯带（dev_id=0x0E）和真空泵（dev_id=0x10），P5 未在本车使用。

### P1：4 按键菜单

```
代码：Key4Btn(1)
文件：config_car.yml:7 → car_wrap.py
功能：4 个物理按键，用于菜单导航和紧急停止
```

- 按键映射（模拟值）：键3=355, 键1=1366, 键2=2137, 键4=2988
- 短按：触发事件 1-4
- 长按（>0.7s）：触发连续事件 5-8
- **按键3 在 `car_wrap.py` 中用作紧急停止**

### P2-A：LED 灯带

```
代码：LedLight(2)
文件：config_car.yml:8 → car_wrap.py
功能：RGB 可编程灯带，状态指示
```

### P2-B：真空泵

```
代码：PoutD(2)
文件：arm_cfg.yaml:15 → arm_base.py:48 → self.pump
功能：吸盘真空泵 — 数字输出 1=ON（吸气）, 0=OFF
```

### P3：电磁阀

```
代码：PoutD(3)
文件：arm_cfg.yaml:16 → arm_base.py:55 → self.valve
功能：吸盘破真空阀 — 数字输出 1=关闭（保持真空）, 0=打开（释放）
```

### P4：弹射气阀

```
代码：PoutD(4)
文件：task_func.py:10 → class Ejection
功能：控制弹射气路通断
```

### P6：竖直限位传感器（模拟输入）

```
代码：AnalogInput(6)
文件：arm_cfg.yaml:44 → arm_base.py:54 → self.vert_limit_sensor
功能：检测机械臂竖直方向是否到达最低点
```

- 传感器型号：S64B5090005（白色长条，模拟位置传感器）
- 当前状态：已接 P6 有信号，但传感器本体**未固定安装**到机械臂
- 归零逻辑：`vert_reset_check()` → 读值 > 1000 判定触底
- 装好后可做精准归零和位置闭环

### P7：右侧红外测距

```
代码：Infrared(7)
文件：config_car.yml:7 → car_wrap.py
功能：车身右侧红外距离传感器，用于定位停车点
```

- 返回值单位：米（原始值 ÷ 1000）
- 典型触发阈值：`value_h=0.2~0.6`

### P8：左侧红外测距

```
代码：Infrared(8)
文件：config_car.yml:6 → car_wrap.py
功能：车身左侧红外距离传感器

- `lane_sensor(speed, value_h, sides=1)` → 左侧红外触发停车
- `lane_sensor(speed, value_h, sides=-1)` → 右侧红外触发停车
```

---

## 步进电机口（dev_id = 0x11）

2 路步进电机接口。注意**步进 2 未使用**。

### 步进 1：弹射角度调节

```
代码：StepperWrap(1)
文件：task_func.py:12 → class Ejection
功能：调节弹射管的俯仰角度，控制弹道
```

- 步距角 1.8°，16 细分
- 丝杆周长 0.008m（假设值）

### 步进 3：机械臂竖直丝杆

```
代码：StepperWrap(3, reverse=-1, perimeter=0.008)
文件：arm_cfg.yaml:40 → arm_base.py:113 → self.vert_motor
功能：驱动机械臂竖直升降
```

- 行程范围：0 ~ 0.3 m
- PID 参数：Kp=4, Ki=0.1, Kd=0.1
- 最大速度：±0.04 m/s
- 归零时往下走直到 P6 限位传感器触发

---

## 板载设备

### 蜂鸣器

```
代码：Beep()
dev_id = 0x0A
功能：声音反馈（启动提示、状态告警）
```

- `beep.rings(freq=262, duration=0.4)` — 频率 Hz，时长秒

### 电池

```
代码：Battry()
dev_id = 0x0C
功能：读取电池电压
```

- 返回值单位：V（原始 mV ÷ 1000）
- 低电量：< 11V 需充电

### 显示屏

```
代码：ScreenShow()（仅 MC602）
dev_id = 0x0B
功能：LED 点阵屏文字显示
```

---

## 代码初始化链路

### 底盘电机

```
cfg_vehicle.yaml                    → 配置参数
  ↓
vehicle_base.py:299 chassis_init()  → 读取配置，创建 MecanumChassis + WheelWrap
  ↓
controller_wrap.py WheelWrap()      → Motors() → Motors_2() → 4× Motor_2()
  ↓
mc602_ctl2.py Motor_2(port_id)      → DevCmdInterface(dev_id=0x02, port_id=1..4)
  ↓
serial_wrap.py                      → USB/CH340 → MC602
```

### 机械臂

```
arm_cfg.yaml                        → 配置参数
  ↓
arm_base.py:112 self.horiz_motor    → MotorWrap(6)  → M6 水平丝杆
arm_base.py:113 self.vert_motor     → StepperWrap(3) → 步进3 竖直丝杆
arm_base.py:57 self.arm_servo       → ServoBus(3)    → S3 旋转舵机
arm_base.py:52 self.hand_servo      → ServoPwm(7)    → S7 手爪舵机
arm_base.py:48 self.pump            → PoutD(2)       → P2 真空泵
arm_base.py:55 self.valve           → PoutD(3)       → P3 电磁阀
arm_base.py:54 self.vert_limit      → AnalogInput(6) → P6 限位传感器
```

### 弹射系统

```
task_func.py class Ejection:
  self.motor_eject   → MotorWrap(5)  → M5 推进电机
  self.pout_eject    → PoutD(4)      → P4 气阀
  self.stepper_eject → StepperWrap(1) → 步进1 角度调节
```

---

## 相关文档

| 文档 | 内容 |
|------|------|
| [hardware-comm.md](hardware-comm.md) | 协议层：帧格式、命令表、struct 定义、速度换算公式 |
| [vehicle-system.md](vehicle-system.md) | 底盘运动学、里程计算法、PID 控制 |
| [arm-system.md](arm-system.md) | 机械臂子系统完整说明 |
| [config-reference.md](config-reference.md) | 所有配置文件的参数详解 |
