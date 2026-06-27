# Jetson ↔ 单片机通信协议

> **底层代码，不可修改。** 本文档详细记录协议细节，供全队理解。

## 物理连接

```
Jetson Orin Nano
    │
    │ USB
    ▼
CH340 USB转串口芯片
    │
    │ TTL 串口 (TX/RX)
    ▼
MC601 或 MC602 控制器
```

- **接口：** `/dev/ttyUSB*`（Linux 自动识别 CH340）
- **线程安全：** `SerialWrap.get_anwser()` 用 `threading.Lock` 保护，同一时刻只有一个命令在收发

## 控制器自动探测

`SerialWrap.__init__()` 启动时自动扫描：

```python
# 1. 扫描所有 CH340 串口
serial_list = [port for port in list_ports.comports() if "CH340" in port[1]]

# 2. 依次尝试三种协议 ping
for ctl_dev in [MC601(), MC602(), MC602Wireness()]:
    if ctl_dev.ping_rx(serial):
        return ctl_dev  # 匹配成功

# 3. 如果 ping 失败，尝试下载固件（MC602 特有）
for ctl_dev in [MC601(), MC602(), MC602Wireness()]:
    if ctl_dev.download_bin(serial):
        return ctl_dev
```

**⚠️ 问题：** 找不到控制器时无限 `while True: time.sleep(1)`，不会退出。

## 控制器类型与参数

| 控制器 | 波特率 | ping 命令 | 特点 |
|--------|--------|----------|------|
| MC601 | 380400 | `77 68 04 00 01 CA 01 0A` | 简单帧格式，编码器为模拟值 |
| MC602 (USB) | 1000000 | `02 01 10`（自动加帧头尾） | 结构化帧格式，真编码器 |
| MC602 (无线) | 115200 | `02 01 10`（加地址路由） | 2.4G 无线透传 |

---

## MC601 协议详解

### 帧格式

**发送帧：直接发送原始命令字节**（MC601 模式下 `send_cmd` 不加帧头尾）

**应答帧：**
```
┌──────┬────────┬──────────────────┬──────┐
│ 0x77 │ 0x68   │ 长度 (含头尾)     │ 数据  │ ... │ 0x0A │
│ 头   │ 头     │ dst_len = N+7   │      │     │ 尾   │
└──────┴────────┴──────────────────┴──────┘
```

- 头：`77 68`（2 字节）
- 长度：第 3 字节，`总帧长 = 长度字段值 + 7`
- 尾：`0A`（1 字节）

### MC601 命令表

#### 探测与控制

| 功能 | 命令 (hex) | 应答 |
|------|-----------|------|
| Ping | `77 68 04 00 01 CA 01 0A` | 有应答 = MC601 存在 |
| 关省电模式 | `77 68 03 00 02 67 0A` | 无 |

#### 电机控制

| 功能 | 命令格式 | 说明 |
|------|---------|------|
| 单电机转动 | `77 68 06 00 02 0C [驱动ID] [端口] [速度] 0A` | 速度: signed int8 (-128~127) |
| 四电机同时 | `77 68 0C 00 02 7A 01 [P1] [P2] [P3] [P4] 0A` | P1~P4: signed int8 |

**速度单位：** 虚拟速度值，范围 ±100。`100` 对应编码器每秒 100 个脉冲。

#### 舵机控制

| 功能 | 命令格式 | 说明 |
|------|---------|------|
| 总线舵机(角度) | `77 68 08 00 02 36 [端口] [速度] [角度4B小端] 0A` | 角度: signed int32, 小端序 |
| 总线舵机(速度) | `77 68 06 00 02 37 [端口] [速度] [1] 0A` | 持续转动 |
| 总线舵机复位 | `77 68 04 00 02 64 0A` | 所有舵机归零 |
| PWM舵机 | `77 68 06 00 02 0B [端口] [速度] [角度] 0A` | 角度 0-180, 连发5次 |

#### 传感器读取

| 功能 | 命令 | 应答数据位置 | 数据格式 |
|------|------|------------|---------|
| 红外传感器 | `77 68 04 00 01 D4 [端口] 0A` | [3:7] | `<i` (signed int32, 小端) |
| 超声波 | `77 68 04 00 01 D1 [端口] 0A` | [3:7] | `<f` (float32, 小端) |
| 模拟输入 | `77 68 04 00 01 E1 [端口] 0A` | [3:5] | `<h` (signed int16) |
| 磁传感器 | `77 68 04 00 01 CF [端口] 0A` | [3:7] | `<i` (signed int32) |
| 按键(单) | `77 68 05 00 01 DB [端口] [按钮号] 0A` | [3] | 0x01=按下 |
| 按键(全部) | `77 68 05 00 01 E1 [端口] 00 0A` | [3:5] | 模拟值→映射按键 |

**按键模拟值映射（MC601）：**
```
0x80~0x28F  → 按键3
0x300~0x48F → 按键1
0x501~0x6FF → 按键2
0x78F~0x9FF → 按键4
```

#### 输出设备

| 功能 | 命令格式 | 说明 |
|------|---------|------|
| LED灯 | `77 68 08 00 02 3B [端口] [灯ID] [R] [G] [B] 0A` | 灯ID: 0=全亮, 1~4=单个 |
| 数字输出 | `77 68 05 00 02 1E [端口] [值] 0A` | 1=断开, 2=导通 |
| 端口输出 | `77 68 05 00 02 3A [端口] [值] 0A` | 同上 |
| 蜂鸣器 | `77 68 05 00 02 3D 03 02 0A` | 固定命令 |
| 数码管 | `77 68 06 00 02 38 [端口] [低字节] [高字节] 0A` | 值 = 低 + 高×256, 0~9999 |

#### AI 摄像头

| 功能 | 命令 | 应答 |
|------|------|------|
| AI摄像头读取 | `77 68 06 00 01 E9 [端口] 54 18 0A` | 18字节数据, 8个值 |

### MC601 编码器（⚠️ 模拟值）

MC601 **没有真实编码器反馈**。编码器值在 Jetson 端通过速度积分估算：

```python
# mc601_ctl2.py - Motor_1 类
def get_encoder(self):
    encoder = self.encoder + self.speed * self.speed_rate * (time.time() - self.last_time)
    return int(encoder)
```

- `speed_rate = 100`（虚拟速度 1 = 每秒 100 个编码脉冲）
- **误差会累积**，长时间运行后里程计漂移

---

## MC602 协议详解

### 帧格式

**发送帧：**
```
┌──────┬──────┬────────┬──────────────────────────┬──────┐
│ 0x77 │ 0x68 │ 长度   │ dev_id mode port 参数...  │ 0x0A │
│ 头   │ 头   │ len+4  │ 命令载荷                   │ 尾   │
└──────┴──────┴────────┴──────────────────────────┴──────┘
```

**应答帧：**
```
┌──────┬──────┬────────┬──────────────────────────┬──────┐
│ 0x77 │ 0x68 │ 长度   │ dev_id mode port 数据...  │ 0x0A │
│ 头   │ 头   │ 总帧长  │ 应答载荷 [3:-1]            │ 尾   │
└──────┴──────┴────────┴──────────────────────────┴──────┘
```

- `get_anwser()` 返回 `res[3:-1]`，即去掉头尾和长度字段的纯载荷

### MC602 设备 ID 与格式

| 设备名 | dev_id | mode | struct 格式 | 说明 |
|--------|:------:|:----:|------------|------|
| motor4 | 0x01 | — | `<bbbbb` | 4电机同时控制 |
| motor | 0x02 | — | `<bbb` | 单电机 |
| encoder4 | 0x03 | — | `<biiii` | 4编码器（**真实硬件编码器**） |
| encoder | 0x04 | — | `<bbi` | 单编码器 |
| servo_pwm | 0x05 | — | `<bbBB` | PWM 舵机 |
| servo_bus | 0x06 | — | `<bbbbh` | 总线舵机 |
| sensor_analog | 0x07 | 0 | `<bbH` | 模拟传感器 |
| sensor_infrared | 0x07 | 1 | `<bbH` | 红外传感器 |
| sensor_touch | 0x07 | 2 | `<bbH` | 触摸传感器 |
| sensor_ultrasonic | 0x07 | 3 | `<bbH` | 超声波传感器 |
| sensor_ambient_light | 0x07 | 4 | `<bbH` | 环境光传感器 |
| sensor_analog_a | 0x08 | 0 | `<bbH` | 模拟传感器(A口) |
| bluetooth | 0x09 | — | `<BBBBi` | 蓝牙手柄 |
| beep | 0x0A | — | `<BBB` | 蜂鸣器 |
| led_show | 0x0B | — | `<b`×101 | LED 点阵屏 |
| power | 0x0C | — | `<bi` | 电池电压 (mV) |
| board_key | 0x0D | — | `<bbb` | 板载按键 |
| led_light | 0x0E | — | `<bbBBBB` | RGB LED |
| nixietube | 0x0F | — | `<bbi` | 数码管 |
| dout | 0x10 | — | `<bbb` | 数字输出 |
| stepper | 0x11 | — | `<bbii` | 步进电机 |

### MC602 操作模式

| mode 值 | 含义 | 说明 |
|:------:|------|------|
| 1 | get | 读取设备状态/传感器值 |
| 2 | set | 设置设备参数 |
| 3 | reset | 复位设备 |

### MC602 命令构造

`DevCmdInterface.get_bytes()` 自动构造命令：

```python
def get_bytes(self, *args, mode=None, port_id=None):
    data = [self.dev_id]           # 设备 ID
    if mode is not None:
        data.append(mode)          # 操作模式
    if port_id is not None:
        data.append(port_id)       # 端口号
    data = data + list(args)       # 参数
    return struct.pack('<b' + format, *data)
```

### MC602 批量读取

`DevListWrap` 支持一次串口往返读取多个设备：

```python
# 构造: 多个设备的命令拼接
bytes_all = dev1.get_bytes(args1, mode=1) + dev2.get_bytes(args2, mode=1)

# 发送: 一次发送
res = serial_mc602.get_anwser(bytes_all)

# 解析: 按各设备的 struct 大小依次解包
index = 0
for dev in dev_list:
    data = dev.get_result(res, index)
    index += dev.data_struct.size
```

### MC602 编码器（✅ 真实值）

MC602 的编码器从单片机读取**真实硬件编码器计数值**：

```python
# mc602_ctl2.py - EncoderMotor_2 类
def get_encoder(self):
    return self.get() * self.reverse  # 从单片机读取真实值
```

**编码器参数：**
- 编码器一圈：48 脉冲（12 栅格 × 4 倍频）
- 减速比：`(28/11)^4 = 41.98`
- 输出一圈总脉冲：`48 × 41.98 = 2015.13`

---

## MC602 无线协议

### 帧格式

```
┌──────┬──────┬────────┬──────┬──────┬──────────┬────────────┬──────┐
│ 0xFE │      │ 长度   │ src  │ dst  │ target   │ 命令载荷    │ 0xFF │
│ 头   │      │        │ port │ port │ id (2B)  │            │ 尾   │
└──────┴──────┴────────┴──────┴──────┴──────────┴────────────┴──────┘
```

- `src_port = 0x90`, `dst_port = 0x91`
- `target_id` = 2 字节目标地址（默认 `5D 3D`）

### 转义处理

```
0xFE → 0xFE 0xFC  (头转义)
0xFF → 0xFE 0xFD  (尾转义)
```

应答时反转义后取 `res[6:-1]` 为有效载荷。

---

## 统一抽象层 — controller_wrap.py

### ctl_id 分发模式

每个硬件类同时持有 MC601 和 MC602 的实例，用全局 `ctl_id` 选择：

```python
ctl_id = get_devid()  # import 时确定: 0=MC601, 1=MC602

class Motors():
    def __init__(self, port_id, id=1, reverse=1):
        self.motor_1 = Motor_1(driver_id=id, port=port_id)  # MC601
        self.motor_2 = Motor_2(port_id=port_id)               # MC602
        self.encoder_2 = EncoderMotor_2(port_id=port_id)      # MC602 编码器

    def set_speed(self, speed):
        fucs = [self.motor_1.rotate, self.motor_2.set_speed]
        fucs[ctl_id](speed)  # 根据 ctl_id 选择实现

    def get_encoder(self):
        fucs = [self.motor_1.get_encoder, self.encoder_2.get]
        return fucs[ctl_id]() * self.reverse
```

### 完整设备抽象列表

| 抽象类 | MC601 实现 | MC602 实现 | 功能 |
|--------|-----------|-----------|------|
| `Motors` | `Motor_1` | `Motor_2` + `EncoderMotor_2` | 单电机+编码器 |
| `Motor4` | `Motor4_1` | `Motor4_2` + `EncoderMotors4_2` | 四电机同时控制 |
| `Motor` | `Motor_1` | `Motor_2` + `EncoderMotor_2` | 电机+弧度换算 |
| `MotorWrap` | `Motor` | `Motor` | 电机+距离换算 |
| `Motors` (第二版) | `NoneDev` | `Motors_2` | 多电机批量控制 |
| `WheelWrap` | `Motors` | `Motors` | 轮子=电机+半径换算 |
| `ServoPwm` | `ServoPwm_1` | `ServoPwm_2` | PWM 舵机 |
| `ServoBus` | `ServoBus_1` | `ServoBus_2` | 总线舵机 |
| `Infrared` | `Infrared_1` | `Infrared_2` | 红外传感器 |
| `AnalogInput` | `AnalogInput_1` | `AnalogInput_2` | 模拟输入 |
| `LedLight` | `LedLight_1` | `LedLight_2` | RGB LED |
| `Key4Btn` | `ButtonAll_1` | `Key4Btn_2` | 四按键 |
| `Beep` | `Buzzer_1` | `Buzzer_2` | 蜂鸣器 |
| `ScreenShow` | `NoneDev` | `ScreenShow_2` | LED 显示屏 |
| `Battry` | `NoneDev` | `Battry_2` | 电池电压 |
| `PoutD` | `PortOut_1` | `PoutD_2` | 数字输出 |
| `StepperWrap` | — | `Stepper_2` | 步进电机 |
| `NixieTube` | `NixieTube_1` | `NixieTube_2` | 数码管 |
| `BluetoothPad` | `NoneDev` | `BluetoothPad_2` | 蓝牙手柄 |
| `BoardKey` | `NoneDev` | `BoardKey_2` | 板载按键 |

### NoneDev 占位

MC601 不支持的设备用 `NoneDev` 占位。调用其方法会**无限挂起**：

```python
class NoneDev:
    def not_support(self):
        logger.info("dev not support")
        while True:
            time.sleep(1)  # ⚠️ 永远挂起
```

---

## 速度/距离换算公式

### 正向换算（速度 → 单片机指令）

```
线速度 v (m/s)
    │
    │ ÷ radius (轮子半径)
    ▼
角速度 ω (rad/s)
    │
    │ × rad2virtual = encoder_resolution / (2π × speed_rate)
    │   = 2015.13 / (2π × 100) = 3.207
    ▼
虚拟速度 (int8, 范围 ±100)
    │
    │ 串口发送
    ▼
单片机 PWM 输出
```

### 反向换算（编码器 → 距离）

```
编码器原始值 (int32)
    │
    │ × encoder2rad = 2π / encoder_resolution
    │   = 2π / 2015.13 = 0.003118
    ▼
弧度 (rad)
    │
    │ × radius (轮子半径)
    ▼
距离 (m)
```

### 关键常量

| 常量 | 值 | 含义 |
|------|---|------|
| `encoder_resolution` | 2015.128 | 减速后输出一圈的编码脉冲数 |
| `speed_rate` | 100 | 虚拟速度 1 = 每秒 100 个编码脉冲 |
| 编码器原始分辨率 | 48 | 12栅格 × 4倍频 |
| 减速比 | (28/11)^4 ≈ 41.98 | 行星齿轮减速 |
| 步进电机步距角 | 1.8° | 标准两相步进 |
| 步进细分 | 16 | 驱动器细分设置 |

### StepperWrap 换算

```python
stepper2rad = π/180 × 1.8 / 16 = 0.001963 rad/step
rad2pwm = 16 × 180 / 1.8 / π = 509.3 step/rad
```
