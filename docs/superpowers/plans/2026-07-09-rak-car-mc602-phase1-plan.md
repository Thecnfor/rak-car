# rak-car Phase 1 — 通用 MC602 底层驱动 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 今晚 2026-07-09 完成 Jetson 端 MC602 通用驱动 + 部署件,明天同事就能在 dev 机器上远程调 `/vehicle_wbt/v1/mc602/*` service 和订阅 `/vehicle_wbt/v1/mc602/state/raw` 并行开发底盘/臂/枪/LLM。

**Architecture:** 3 个 ament_python 包(`hw` 纯库 / `msgs` 接口契约 / `bridge` 唯一节点)+ 部署件 + API 文档。所有 MC602 协议字节 1:1 抄 `baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py` 和 `serial_wrap.py`,不复刻不重写。

**Topology (Jetson vs dev box):**
- **Jetson 端**(本机 `192.168.3.69`):跑 `mc602_node.py` 独占 `/dev/ttyUSB*`,systemd 常驻。
- **Dev box 端**(4 同事):clone 仓库 → `colcon build --packages-up-to vehicle_wbt_smartcar_msgs` → `export ROS_DOMAIN_ID=42` → `scripts/quick_beep.sh` → DDS 自动发现 Jetson → 听到蜂鸣 → 信任建立 → 写业务节点。
- **同事不 SSH Jetson、不部署代码到 Jetson**;唯一例外是我紧急 debug。

**Tech Stack:** ROS2 Humble + Python 3.10 + pyserial + rclpy + rosidl(generate .srv/.msg)+ CycloneDDS + systemd + udev

## Global Constraints

- **Ground truth**: `baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/{mc602_ctl2.py, serial_wrap.py, controller_wrap.py}` 是协议字节的唯一参照。每个设备类的 dev_id / mode / port_id / struct format 必须 1:1 对齐。
- **port_id=None ≠ port_id=0**:前者不发 port 字节(Buzzer_2 的行为),后者发 0x00。通过 SDK `mc602_ctl2.py:120-129` 的 `get_bytes` 逻辑保留(`arg_reg` 控制发不发 mode/port)。
- **错误风格**:SDK 失败返回 `None`,上层 `if x is None: 处理失败`;不抛异常,不写自定义错误类。
- **真硬件验证**取代单元测试:每个 task 完成后必须有 Jetson 端冒烟步骤(用户决策,2026-07-09)。
- **代码风格**:Python 类型注解齐全,文件头 `from __future__ import annotations`,中文 docstring 风格(对齐 SDK),public 类/方法必须有 docstring。
- **Commit 规范**:feat/fix/chore/docs/refactor(scope): 简述。`robot-stable` 分支(HEAD 当前就在这)。
- **ROS_DOMAIN_ID=42** 硬编码在 launch 文件里(`SetEnvironmentVariable`)。
- **ROS2 接口命名空间**:`/vehicle_wbt/v1/mc602/*`(service 和 topic 一致),不暴露 SDK 内部 dev_id。
- **Port 路由**:`config/mc602_ports.yaml` 集中定义,服务 handler 路由到这里查表;同事不需要知道硬件 dev_id。

## File Structure (新建/修改)

```
新建:
ros2_ws/src/vehicle_wbt_smartcar_hw/                          # Task 1-6
├── package.xml
├── setup.py
├── resource/vehicle_wbt_smartcar_hw/
└── vehicle_wbt_smartcar_hw/
    ├── __init__.py                                           # Task 6
    ├── serial.py                                             # Task 2
    ├── mc602_ctl2.py                                         # Task 3
    ├── odometry.py                                           # Task 4
    └── arm.py                                                # Task 5

ros2_ws/src/vehicle_wbt_smartcar_msgs/                         # Task 7
├── CMakeLists.txt
├── package.xml
├── resource/vehicle_wbt_smartcar_msgs/
├── msg/RawState.msg
└── srv/
    ├── SetWheels.srv
    ├── ReadEncoders.srv
    ├── ResetEncoders.srv          # 用 trigger_msgs 或自定义
    ├── SetServoPwm.srv
    ├── SetServoBus.srv
    ├── SetStepper.srv
    ├── SetDcMotor.srv
    ├── SetDout.srv
    ├── ReadIR.srv
    ├── ReadBattery.srv
    ├── ReadAnalog.srv
    └── Buzzer.srv

ros2_ws/src/vehicle_wbt_smartcar_bridge/                       # Task 8-10
├── package.xml
├── setup.py
├── resource/vehicle_wbt_smartcar_bridge/
├── config/mc602_ports.yaml
├── launch/mc602.launch.py
└── vehicle_wbt_smartcar_bridge/
    ├── __init__.py
    └── mc602_node.py                                          # Task 9

deploy/                                                       # Task 11-14
├── systemd/vehicle-wbt-mc602.service
├── ros_env.sh
├── cyclonedds/cyclonedds.xml
└── udev/99-usbvideo.rules

docs/integration/LOWLEVEL_API.md                              # Task 15

修改:
CLAUDE.md                                                     # Task 16
```

---

## Task 1: 创建 vehicle_wbt_smartcar_hw 包骨架

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_hw/package.xml`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_hw/setup.py`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_hw/resource/vehicle_wbt_smartcar_hw/marker`

- [ ] **Step 1: 创建目录结构**

```bash
cd /home/xrak/workspace/rak-car
mkdir -p ros2_ws/src/vehicle_wbt_smartcar_hw/resource/vehicle_wbt_smartcar_hw
mkdir -p ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw
```

- [ ] **Step 2: 写 `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model
  href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vehicle_wbt_smartcar_hw</name>
  <version>0.1.0</version>
  <description>
    MC602 下位机协议层。pyserial 包装 + 20 个设备类 + 里程计纯计算。
    所有字节 1:1 对齐 baidu_smartcar_2026 SDK。
  </description>
  <maintainer email="rak-car@todo.todo">RAK-Car Team</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>  <!-- 只是 buildtool 依赖,运行时无 ROS 依赖;保留以便跨进程共享时方便 -->

  <exec_depend>pyserial</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: 写 `setup.py`**

```python
from setuptools import setup

package_name = 'vehicle_wbt_smartcar_hw'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['pyserial'],
    zip_safe=True,
    maintainer='RAK-Car Team',
    maintainer_email='rak-car@todo.todo',
    description='MC602 下位机协议层,SDK 字节 1:1 对齐',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
```

- [ ] **Step 4: 写 resource marker**

```bash
touch /home/xrak/workspace/rak-car/ros2_ws/src/vehicle_wbt_smartcar_hw/resource/vehicle_wbt_smartcar_hw/marker
```

- [ ] **Step 5: 写空 `__init__.py`(占位,Task 6 重写)**

```python
"""vehicle_wbt_smartcar_hw - MC602 下位机协议层(占位,Task 6 重写)。"""
```

- [ ] **Step 6: 验证 ament_python 能找到包**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vehicle_wbt_smartcar_hw
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 7: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_hw/
git commit -m "feat(hw): scaffold vehicle_wbt_smartcar_hw package skeleton"
```

---

## Task 2: 抄 SDK `serial.py`(MC602 class)

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/serial.py`

**Interfaces:**
- Consumes: pyserial (system)
- Produces: `MC602` 类(全局单例 `serial_mc602`)+ `MC602.send_cmd(payload)` + `MC602.get_anwser(time_out)` + `MC602.ping_port()`

- [ ] **Step 1: 写 `serial.py`(完整复制 SDK `serial_wrap.py` 中 MC602 相关部分,删除 MC601 / MC602Wireness)**

```python
"""MC602 串口封装。

源码 1:1 对齐 baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/serial_wrap.py
的 MC602 class + ping_port 逻辑。删除 MC601/MC602Wireness 不需要的部分,
保留 SDK 风格的全局单例 + lock + 自动端口扫描。
"""
from __future__ import annotations

import time
from threading import Lock

import serial
from serial.tools import list_ports


class MC602(serial.Serial):
    """MC602 串口包装,SDK 风格(继承 pyserial)。

    Args:
        port: 串口设备路径,默认 None(由 ping_port 自动扫描)。
        baud: 波特率,默认 1_000_000(SDK 默认)。
        timeout: 串口读超时,默认 0.03s。
    """

    HEADER = bytes.fromhex('77 68')
    TAIL = bytes.fromhex('0A')

    def __init__(self, port=None, baud=1_000_000, timeout=0.03):
        super().__init__(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=timeout,
            xonxoff=False, rtscts=False, dsrdtr=False,
        )
        self.lock = Lock()
        self.connect_flag = False

    def send_cmd(self, cmd: bytes) -> None:
        """发送完整帧(已含头尾)。SDK 行为:写一次不读返回(由 get_anwser 单独读)。"""
        self.write(cmd)

    def get_anwser(self, time_out: float = 0.1) -> bytes | None:
        """读 MC602 响应。

        协议:读 3 字节 → 解 dst_len = res[2] → 读剩余 → 校验头尾 → 返回 res[3:-1]。
        完全对齐 SDK `serial_wrap.py:222-245`。
        """
        time_start = time.time()
        res = self.read(3)
        if len(res) != 3:
            return None
        dst_len = res[2]
        res = res + self.read(dst_len - 3)
        while True:
            if time.time() - time_start > time_out:
                return None
            if len(res) == dst_len:
                if res[0] == self.HEADER[0] and res[-1] == self.TAIL[0]:
                    return res[3:-1]
                return None
            res = res + self.read(dst_len - len(res))

    def ping_rx(self, time_out: float = 0.05) -> bool:
        """探测 MC602 是否在响应。SDK `serial_wrap.py:248-257`。

        用 `02 01 10`(dev=0x10, mode=0, port=0)试探(SDK 用的内置 ping)。
        """
        time_start = time.time()
        while time.time() - time_start < time_out:
            self.reset_input_buffer()
            self.reset_output_buffer()
            # SDK 风格的 ping 帧(已含头尾和长度)
            self.write(bytes.fromhex('77 68 04 00 01 CA 01 0A'))  # SDK 头部用 0xCA01 探测
            res = self.get_anwser(0.02)
            if res is not None:
                return True
        return False

    def ping_port(self, baud: int = 1_000_000) -> str | None:
        """扫描 CH340/USB 串口,找到第一个能 ping 通的端口并打开。

        对齐 SDK `serial_wrap.py:113-142`。返回打开的端口名,失败 None。
        """
        port_list = list_ports.comports()
        port_list = [p for p in port_list if 'CH340' in p[1] or 'USB' in p[1]]
        port_list.sort(key=lambda x: 'CH340' not in x[1])  # CH340 优先
        for p in port_list:
            try:
                self.port = p[0]
                self.baudrate = baud
                self.open()
                self.connect_flag = True
                if self.ping_rx(time_out=0.05):
                    return p[0]
                self.close()
                self.connect_flag = False
            except Exception:
                try:
                    self.close()
                except Exception:
                    pass
                self.connect_flag = False
        return None

    def open(self) -> bool:
        """打开串口,带状态记录。"""
        try:
            if self.port is None:
                return False
            self.connect_flag = True
            super().open()
            return True
        except Exception:
            self.connect_flag = False
            return False


# 全局单例(SDK 风格,所有设备类共享)
serial_mc602 = MC602()
```

- [ ] **Step 2: 验证 import**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source install/setup.bash
python3 -c "from vehicle_wbt_smartcar_hw.serial import MC602, serial_mc602; print(type(serial_mc602).__name__)"
```

Expected: `MC602`

- [ ] **Step 3: 真硬件冒烟(Jetson 端):确认能 ping 通下位机**

```bash
# Jetson 端
python3 -c "
from vehicle_wbt_smartcar_hw.serial import serial_mc602
port = serial_mc602.ping_port(baud=1_000_000)
print(f'ping port = {port}')
"
```

Expected: `ping port = /dev/ttyUSB1` (或类似)。如果返回 None,检查 USB 连接 + udev 规则。

- [ ] **Step 4: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/serial.py
git commit -m "feat(hw): MC602 serial wrapper (SDK-aligned)"
```

---

## Task 3: 抄 SDK `mc602_ctl2.py`(全部 20 个设备类)

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/mc602_ctl2.py`

**Interfaces:**
- Consumes: `serial.serial_mc602`(Task 2)
- Produces: 14 个设备类 + `DevCmdInterface` + `DevListWrap`(全部 SDK 同名)

- [ ] **Step 1: 直接复制 SDK 文件到目标位置**

```bash
cp /home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py \
   /home/xrak/workspace/rak-car/ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/mc602_ctl2.py
```

- [ ] **Step 2: 删掉不需要的设备类(`BluetoothPad_2`、`LedLight_2`、`NixieTube_2`、`ScreenShow_2`、`Key4Btn_2`)**

在 `mc602_ctl2.py` 中:
- 删除 `class BluetoothPad_2`(line ~255-297)
- 删除 `class LedLight_2`(line ~307-315)
- 删除 `class NixieTube_2`(line ~405-410)
- 删除 `class ScreenShow_2`(line ~492-501)
- 删除 `class Key4Btn_2`(line ~317-402)

- [ ] **Step 3: 修改 `serial_mc602` 引用为相对路径**

`mc602_ctl2.py` 第 19 行原是:
```python
from smartcar.whalesbot.vehicle.base.serial_wrap import serial_wrap
```

改为:
```python
from .serial import serial_mc602
```

第 26 行的 `serial_mc602 = serial_wrap` 删除(已经在 `serial.py` 定义单例)。

- [ ] **Step 4: 修复 `from ...tools import logger`**

原 mc602_ctl2.py 第 17 行 `from ...tools import logger` 在 rak-car 项目里没有 tools 包。**全部 logger 调用替换为 `print`**(简单粗暴,Phase 1 不引 logging 框架):

```bash
# 用 sed 替换(在 mc602_ctl2.py 顶部和 logger.info/error 调用)
sed -i 's|from ...tools import logger|pass  # logger via print|g' \
    /home/xrak/workspace/rak-car/ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/mc602_ctl2.py
sed -i 's|logger\.\(info\|error\|warning\|debug\|critical\)(\(.*\))|print(\2)|g' \
    /home/xrak/workspace/rak-car/ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/mc602_ctl2.py
```

**警告**: 这个 sed 会把 `logger.xxx(...)` 替换成 `print(...)`。如果有嵌套括号或字符串里有右括号,需要手动校对。运行后 grep 确认没有遗漏的 `logger.` 引用。

- [ ] **Step 5: 验证 import 全部设备类**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source install/setup.bash
python3 -c "
from vehicle_wbt_smartcar_hw.mc602_ctl2 import (
    Buzzer_2, Motor_2, Motor4_2, Motors_2,
    EncoderMotor_2, EncoderMotors4_2,
    ServoPwm_2, ServoBus_2,
    AnalogInput_2, Infrared_2,
    Battry_2, BoardKey_2, PoutD_2, Stepper_2,
    DevCmdInterface, DevListWrap,
)
print('all 14 device classes imported OK')
"
```

Expected: `all 14 device classes imported OK`

- [ ] **Step 6: 真硬件冒烟:Buzzer + 电池读取**

```bash
# Jetson 端
python3 -c "
from vehicle_wbt_smartcar_hw.mc602_ctl2 import Buzzer_2, Battry_2
b = Buzzer_2()
b.rings(440, 0.2)  # 听到 0.2 秒 440Hz 蜂鸣
print('buzzer OK')
bat = Battry_2()
v = bat.read()
print(f'battery = {v} V')
"
```

Expected: 听到蜂鸣 + `battery = 11.x` 之类合理值(锂电 ~11-12V)。

- [ ] **Step 7: 真硬件冒烟:编码器读取**

```bash
python3 -c "
from vehicle_wbt_smartcar_hw.mc602_ctl2 import EncoderMotors4_2
e = EncoderMotors4_2()
print('encoders =', e.get())
"
```

Expected: `[0, 0, 0, 0]` (静止时);推一下车再读应该有变化。

- [ ] **Step 8: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/mc602_ctl2.py
git commit -m "feat(hw): 14 MC602 device classes (SDK 1:1 port)"
```

---

## Task 4: 抄 SDK `odometry.py`(只 Odometry + MecanumChassis 纯计算,不要 MecanumDriver)

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/odometry.py`

**Interfaces:**
- Consumes: numpy
- Produces: `Odometry` 类 + `MecanumChassis` 类(纯计算,无串口依赖)

- [ ] **Step 1: 复制并裁剪 SDK `mecanum.py`**

```bash
cp /home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/driver/mecanum.py \
   /home/xrak/workspace/rak-car/ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/odometry.py
```

- [ ] **Step 2: 删掉 `MecanumDriver` 类(line 297-676)和它的导入/使用**

在 `odometry.py` 中:
- 删除 `class MecanumDriver`(整段约 380 行)
- 删除 `from ..base.controller_wrap import WheelWrap` 导入
- 删除 `from ...tools import PID` 导入
- 删除 `from ...tools.log_wrap import logger` 导入
- 删除 `import yaml` 如果不再使用
- 删除 `_OffsetGroup` / `_Offset` 类如果只被 MecanumDriver 用
- 删除 `import threading` 如果不再使用

**注意**:MecanumChassis.forward_kinematics + inverse_kinematics + Odometry.update 是核心,保留。

- [ ] **Step 3: 把 logger 引用改为 pass 或 print(同 Task 3 Step 4)**

```bash
sed -i 's|from ...tools.log_wrap import logger|pass  # logger|g' /home/xrak/workspace/rak-car/ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/odometry.py
sed -i 's|logger\.\(info\|error\|warning\|debug\|critical\)(\(.*\))|print(\2)|g' /home/xrak/workspace/rak-car/ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/odometry.py
```

- [ ] **Step 4: 验证 import + 正逆解计算**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source install/setup.bash
python3 -c "
import numpy as np
from vehicle_wbt_smartcar_hw.odometry import Odometry, MecanumChassis
chassis = MecanumChassis(track=0.30, wheel_base=0.28, wheel_radius=0.03)
# 逆解: 车身速度 (0.2, 0, 0) → 4 轮速度
wheel_v = chassis.inverse_kinematics(np.array([0.2, 0.0, 0.0]))
print('wheel_v =', wheel_v)
# 正解: 回到车身速度
car_v = chassis.forward_kinematics(np.array(wheel_v))
print('car_v =', car_v)
"
```

Expected: `wheel_v = [v0 v1 v2 v3]`(4 个相同值,纯 x 方向);`car_v ≈ [0.2, 0, 0]`。

- [ ] **Step 5: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/odometry.py
git commit -m "feat(hw): Odometry + MecanumChassis (SDK-aligned, pure compute)"
```

---

## Task 5: 简化抄 SDK `arm_base.py`(只要设备引用,不要 PID Driver)

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/arm.py`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/arm_cfg.yaml`

**Interfaces:**
- Consumes: `Stepper_2` / `Motor_2` / `ServoPwm_2` / `ServoBus_2` / `PoutD_2` / `AnalogInput_2` / `arm_cfg.yaml`
- Produces: `ArmController` 类(简化版:持设备引用 + `set_arm_pose` 非闭环 + `grasp` + `reset_position`)

- [ ] **Step 1: 写 `arm_cfg.yaml`(从 SDK arm_base.py 推断)**

```yaml
# 机械臂配置(简化版,从 SDK arm_base.py 的 config 块提取)
arm_length: 0.30

vert_cfg:
  motor:
    name: "stepper_y"
    dev_id: 0x11
    port_id: 1
    reverse: 1
  limit_port: 6  # AnalogInput 端口
  pid:
    Kp: 5
    Ki: 0.1
    Kd: 0.05
    setpoint: 0
    output_limits: [-0.05, 0.05]
  threshold: [-0.05, 0.30]

horiz_cfg:
  motor:
    name: "dc_motor_x"
    dev_id: 0x02
    port_id: 2
    reverse: 1
  pid:
    Kp: 5
    Ki: 0.1
    Kd: 0.05
    setpoint: 0
    output_limits: [-0.06, 0.06]
  threshold: [-0.35, 0.0]

hand_cfg:
  hand:  # 手臂舵机(总线舵机, dev_id=0x06)
    port: 4
    angle_list:
      LEFT: 60
      MID: 90
      RIGHT: 120
  hand2:  # 手部舵机(PWM, dev_id=0x05)
    port: 3
    mode: 1
    angle_list:
      UP: 30
      MID: 60
      DOWN: 90
  grap:
    port_pump: 1
    port_valve: 2

pos_cfg:
  pose_enable: true
  pose_horiz: 0.0
  pose_vert: 0.0
  side: "MID"
```

- [ ] **Step 2: 写 `arm.py`(简化版 ArmController)**

```python
"""机械臂控制类(简化版,Phase 1 不含 PID 闭环)。

完整 PID 闭环逻辑留给上层机械臂同事实现。本类只提供:
- 设备引用
- set_arm_angle(角度或方向字符串)
- set_hand_angle(角度或方向字符串)
- grasp(bool)(控制泵+阀)
- reset_position()(简单粗暴 reset)

源码对齐:baidu_smartcar_2026/smartcar/whalesbot/vehicle/arm/arm_base.py
"""
from __future__ import annotations

import os
import time

import yaml

from .mc602_ctl2 import (
    AnalogInput_2,
    Motor_2,
    PoutD_2,
    ServoBus_2,
    ServoPwm_2,
    Stepper_2,
)


class ArmController:
    """机械臂设备容器。Phase 1 简化版,只暴露直接调用,无 PID 闭环。"""

    def __init__(self, config_path: str | None = None) -> None:
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), 'arm_cfg.yaml')
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.arm_length: float = cfg['arm_length']
        self.config = cfg

        # 垂直轴
        vc = cfg['vert_cfg']
        self.motor_y = Stepper_2(port_id=vc['motor']['port_id'])
        self.y_limit_sensor = AnalogInput_2(port_id=vc['limit_port'])
        self.y_threshold = vc['threshold']

        # 水平轴
        hc = cfg['horiz_cfg']
        self.motor_x = Motor_2(port_id=hc['motor']['port_id'])
        self.x_threshold = hc['threshold']

        # 手部
        hc_hand = cfg['hand_cfg']
        self.hand_servo = ServoPwm_2(port_id=hc_hand['hand2']['port'])
        self.hand_angle_list = hc_hand['hand2']['angle_list']
        self.arm_servo = ServoBus_2(port_id=hc_hand['hand']['port'])
        self.arm_angle_list = hc_hand['hand']['angle_list']

        # 泵 + 阀
        gc = hc_hand['grap']
        self.pump = PoutD_2(port_id=gc['port_pump'])
        self.valve = PoutD_2(port_id=gc['port_valve'])

        # 当前位置(由上层通过 set_x_mm / set_y_mm 更新;Phase 1 不做编码器回读)
        self.x_pose_now = 0.0
        self.y_pose_now = 0.0

    def set_arm_angle(self, angle, speed: int = 80):
        """设置手臂舵机角度(总线舵机)。angle 可为字符串 'LEFT'/'MID'/'RIGHT' 或数字。"""
        if isinstance(angle, str):
            assert angle in ('LEFT', 'MID', 'RIGHT'), f"bad angle: {angle}"
            angle = self.arm_angle_list[angle]
        self.arm_servo.set_angle(int(angle), int(speed))

    def set_hand_angle(self, angle, speed: int = 80):
        """设置手部舵机角度(PWM)。angle 可为字符串 'UP'/'MID'/'DOWN' 或数字。"""
        if isinstance(angle, str):
            assert angle in ('UP', 'MID', 'DOWN'), f"bad angle: {angle}"
            angle = self.hand_angle_list[angle]
        self.hand_servo.set_angle(int(angle), int(speed))

    def grasp(self, value: bool):
        """抓取/释放。True = 抓(泵开,阀关);False = 放(泵关,阀开)。"""
        self.pump.set(1 if value else 0)
        self.valve.set(0 if value else 1)

    def set_x_velocity(self, v: float):
        """设置水平轴电机速度(直接调,不闭环)。"""
        self.motor_x.set_speed(int(v))

    def set_y_velocity(self, v: float):
        """设置垂直轴步进电机速度(直接调,不闭环)。"""
        self.motor_y.set_pwm(int(v))

    def stop(self):
        """紧急停止所有电机。"""
        self.set_x_velocity(0)
        self.set_y_velocity(0)

    def reset_position(self):
        """简单 reset:手朝上 + 臂朝右 + 释放抓取。完整 PID reset 留给上层。"""
        self.set_hand_angle('UP')
        time.sleep(0.5)
        self.set_arm_angle('RIGHT')
        time.sleep(0.5)
        self.grasp(False)
```

- [ ] **Step 3: 验证 import**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source install/setup.bash
python3 -c "
from vehicle_wbt_smartcar_hw.arm import ArmController
arm = ArmController()
print(f'arm_length={arm.arm_length}')
print(f'motor_y port=1, motor_x port=2')
print(f'pump port=1, valve port=2')
"
```

Expected: 打印三个属性值,不报错。

- [ ] **Step 4: 真硬件冒烟(可选,仅当机械臂硬件已接好)**

```bash
python3 -c "
from vehicle_wbt_smartcar_hw.arm import ArmController
arm = ArmController()
arm.set_hand_angle('UP')  # 听到/看到舵机摆动
import time; time.sleep(1)
arm.set_hand_angle('MID')
arm.grasp(False)
"
```

Expected: 舵机摆动 + 抓取机构释放(可听到气泵声)。

- [ ] **Step 5: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/arm.py \
        ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/arm_cfg.yaml
git commit -m "feat(hw): simplified ArmController (device refs + non-closed-loop)"
```

---

## Task 6: 写 `__init__.py` 重导出

**Files:**
- Modify: `ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/__init__.py`

- [ ] **Step 1: 重写 `__init__.py`**

```python
"""vehicle_wbt_smartcar_hw — MC602 下位机协议层。

Ground truth: baidu_smartcar_2026 SDK。字节格式 1:1 对齐,
错误风格遵循 SDK(失败返回 None)。

模块布局:
    serial      - MC602 串口包装(SDK serial_wrap.py 简化版)
    mc602_ctl2  - 14 个设备类 + DevCmdInterface + DevListWrap(SDK 1:1)
    odometry    - Odometry + MecanumChassis 纯计算(SDK 简化版)
    arm         - ArmController 设备容器(简化版,无 PID 闭环)
"""
from __future__ import annotations

from .serial import MC602, serial_mc602
from .mc602_ctl2 import (
    AnalogInput_2,
    Battry_2,
    BoardKey_2,
    Buzzer_2,
    DevCmdInterface,
    DevListWrap,
    EncoderMotor_2,
    EncoderMotors4_2,
    Infrared_2,
    Motor_2,
    Motor4_2,
    Motors_2,
    PoutD_2,
    ServoBus_2,
    ServoPwm_2,
    Stepper_2,
)
from .odometry import MecanumChassis, Odometry
from .arm import ArmController

__all__ = [
    'MC602',
    'serial_mc602',
    'AnalogInput_2',
    'Battry_2',
    'BoardKey_2',
    'Buzzer_2',
    'DevCmdInterface',
    'DevListWrap',
    'EncoderMotor_2',
    'EncoderMotors4_2',
    'Infrared_2',
    'Motor_2',
    'Motor4_2',
    'Motors_2',
    'PoutD_2',
    'ServoBus_2',
    'ServoPwm_2',
    'Stepper_2',
    'MecanumChassis',
    'Odometry',
    'ArmController',
]
```

- [ ] **Step 2: 验证完整 import 链**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source install/setup.bash
python3 -c "
import vehicle_wbt_smartcar_hw as hw
print(sorted(hw.__all__))
"
```

Expected: 输出 21 个名字的排序列表。

- [ ] **Step 3: 重新 colcon build**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
colcon build --packages-select vehicle_wbt_smartcar_hw
```

Expected: BUILD SUCCESSFUL。

- [ ] **Step 4: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/__init__.py
git commit -m "feat(hw): __init__.py re-export public API"
```

---

## Task 7: 创建 vehicle_wbt_smartcar_msgs 包(12 .srv + 1 .msg)

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_msgs/package.xml`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_msgs/CMakeLists.txt`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_msgs/resource/.../marker`
- Create: 12 个 .srv 文件
- Create: 1 个 .msg 文件

**Interfaces:**
- Produces: ROS2 接口契约,被 `bridge` 和所有上层同事使用

- [ ] **Step 1: 创建目录**

```bash
cd /home/xrak/workspace/rak-car
mkdir -p ros2_ws/src/vehicle_wbt_smartcar_msgs/{resource/vehicle_wbt_smartcar_msgs,srv,msg}
touch ros2_ws/src/vehicle_wbt_smartcar_msgs/resource/vehicle_wbt_smartcar_msgs/marker
```

- [ ] **Step 2: 写 `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vehicle_wbt_smartcar_msgs</name>
  <version>0.1.0</version>
  <description>
    MC602 通用 ROS2 接口契约。12 service + 1 msg。
    上层业务节点(chassis/arm/shooter/perception)只 depend 这个包。
  </description>
  <maintainer email="rak-car@todo.todo">RAK-Car Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: 写 `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(vehicle_wbt_smartcar_msgs LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(SRV_FILES
  "srv/SetWheels.srv"
  "srv/ReadEncoders.srv"
  "srv/ResetEncoders.srv"
  "srv/SetServoPwm.srv"
  "srv/SetServoBus.srv"
  "srv/SetStepper.srv"
  "srv/SetDcMotor.srv"
  "srv/SetDout.srv"
  "srv/ReadIR.srv"
  "srv/ReadBattery.srv"
  "srv/ReadAnalog.srv"
  "srv/Buzzer.srv"
)

set(MSG_FILES
  "msg/RawState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

- [ ] **Step 4: 写所有 12 个 .srv 文件**

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/SetWheels.srv`:
```
int8 v0
int8 v1
int8 v2
int8 v3
---
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/ReadEncoders.srv`:
```
---
int32[4] v
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/ResetEncoders.srv`:
```
---
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/SetServoPwm.srv`:
```
uint8 port
int16 angle
---
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/SetServoBus.srv`:
```
uint8 port
int16 angle
int16 speed
---
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/SetStepper.srv`:
```
uint8 port
int16 freq
---
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/SetDcMotor.srv`:
```
uint8 port
int8 speed
---
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/SetDout.srv`:
```
uint8 port
bool state
---
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/ReadIR.srv`:
```
uint8 port
---
float32 distance_m
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/ReadBattery.srv`:
```
---
float32 voltage_v
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/ReadAnalog.srv`:
```
uint8 port
---
int16 value
bool success
```

`ros2_ws/src/vehicle_wbt_smartcar_msgs/srv/Buzzer.srv`:
```
uint16 freq_hz
uint16 duration_ms
---
bool success
```

- [ ] **Step 5: 写 `RawState.msg`**

`ros2_ws/src/vehicle_wbt_smartcar_msgs/msg/RawState.msg`:
```
std_msgs/Header header

int32[4] encoders
float32 ir_left_m
float32 ir_right_m
float32 battery_v
int32 arm_y_pos
int32 arm_x_pos
bool pump_on
bool valve_on
```

- [ ] **Step 6: colcon build 验证接口生成**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vehicle_wbt_smartcar_msgs
```

Expected: BUILD SUCCESSFUL,生成 `install/vehicle_wbt_smartcar_msgs/include/vehicle_wbt_smartcar_msgs/...` 头文件 + Python 接口。

- [ ] **Step 7: 验证 import**

```bash
source install/setup.bash
python3 -c "
from vehicle_wbt_smartcar_msgs.srv import SetWheels, Buzzer, ReadBattery
from vehicle_wbt_smartcar_msgs.msg import RawState
print('all imports OK')
ros2 service list -t 2>/dev/null | head
"
```

Expected: `all imports OK` + ros2 CLI 能看到 msg/srv 类型。

- [ ] **Step 8: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_msgs/
git commit -m "feat(msgs): 12 services + RawState.msg interface contract"
```

---

## Task 8: 创建 vehicle_wbt_smartcar_bridge 包骨架 + config

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_bridge/package.xml`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_bridge/setup.py`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_bridge/resource/.../marker`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_bridge/config/mc602_ports.yaml`
- Create: `ros2_ws/src/vehicle_wbt_smartcar_bridge/vehicle_wbt_smartcar_bridge/__init__.py`(空)

- [ ] **Step 1: 创建目录**

```bash
cd /home/xrak/workspace/rak-car
mkdir -p ros2_ws/src/vehicle_wbt_smartcar_bridge/{resource/vehicle_wbt_smartcar_bridge,config,launch,vehicle_wbt_smartcar_bridge}
touch ros2_ws/src/vehicle_wbt_smartcar_bridge/resource/vehicle_wbt_smartcar_bridge/marker
```

- [ ] **Step 2: 写 `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vehicle_wbt_smartcar_bridge</name>
  <version>0.1.0</version>
  <description>
    MC602 通用 ROS2 节点。唯一串口所有者,12 service + 2 topic,
    转发到底层 vehicle_wbt_smartcar_hw。
  </description>
  <maintainer email="rak-car@todo.todo">RAK-Car Team</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>vehicle_wbt_smartcar_hw</depend>
  <depend>vehicle_wbt_smartcar_msgs</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: 写 `setup.py`**

```python
from setuptools import setup
import os

package_name = 'vehicle_wbt_smartcar_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            ['config/mc602_ports.yaml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/mc602.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial', 'pyyaml'],
    zip_safe=True,
    maintainer='RAK-Car Team',
    maintainer_email='rak-car@todo.todo',
    description='MC602 通用 ROS2 桥接节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mc602_node = vehicle_wbt_smartcar_bridge.mc602_node:main',
        ],
    },
)
```

- [ ] **Step 4: 写 `config/mc602_ports.yaml`**

```yaml
# MC602 硬件端口路由表(集中管理,服务 handler 路由到这里查表)
# 底盘同事不知道硬件 dev_id,只通过 service 字段 port

chassis:
  motors: [1, 2, 3, 4]      # Motor_2 端口列表(用 Motors_2 包装)
  encoders_dev: 0x03          # EncoderMotors4_2 的 dev_id

arm:
  stepper_y:                  # Y 轴步进电机
    dev_id: 0x11
    port: 1
  dc_motor_x:                 # X 轴直流电机
    dev_id: 0x02
    port: 2
  servo_pwm_hand:             # 手部 PWM 舵机
    dev_id: 0x05
    port: 3
  servo_bus_wrist:            # 腕部总线舵机
    dev_id: 0x06
    port: 4
  pump:                       # 真空泵
    dev_id: 0x10
    port: 1
  valve:                      # 真空阀
    dev_id: 0x10
    port: 2
  limit_switch_y:             # Y 轴磁限位
    dev_id: 0x07
    mode: 0
    port: 6

shooter:
  barrel:                     # 发射器
    dev_id: 0x10
    port: 4
  pump_reload:                # 装填泵(可选,Phase 5+)
    dev_id: 0x10
    port: 5

ir:
  left: 7
  right: 8

beep:
  dev_id: 0x0a                # Buzzer_2,无 port
```

- [ ] **Step 5: 写 `vehicle_wbt_smartcar_bridge/__init__.py`**

```python
"""vehicle_wbt_smartcar_bridge - MC602 通用 ROS2 桥接节点。"""
```

- [ ] **Step 6: colcon build 验证包骨架**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vehicle_wbt_smartcar_bridge
```

Expected: BUILD SUCCESSFUL(虽然 mc602_node.py 还没写,但 setup.py 引用了它,可能失败;**预期失败**,等 Task 9 写完再 build)

- [ ] **Step 7: Commit(如果有可提交的)**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_bridge/
git commit -m "feat(bridge): scaffold + mc602_ports.yaml config"
```

---

## Task 9: 写 `mc602_node.py` 主节点(~300 行)

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_bridge/vehicle_wbt_smartcar_bridge/mc602_node.py`

**Interfaces:**
- Consumes: `vehicle_wbt_smartcar_hw`(Task 1-6)+ `vehicle_wbt_smartcar_msgs`(Task 7)+ `mc602_ports.yaml`(Task 8)
- Produces: 12 service server + 2 topic publisher + 2 timer callback

- [ ] **Step 1: 写完整的 `mc602_node.py`**

```python
"""MC602 通用 ROS2 节点。

唯一串口所有者。12 service + 2 topic,把 ROS2 请求转发到底层 SDK 设备类。

源码对齐:baidu_smartcar_2026 SDK (mc602_ctl2.py + serial_wrap.py)。
错误风格:SDK 失败返回 None → service 响应 success: false。
"""
from __future__ import annotations

import os
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Header

from vehicle_wbt_smartcar_hw import (
    ArmController,
    Battry_2,
    Buzzer_2,
    EncoderMotors4_2,
    Infrared_2,
    Motor_2,
    Motors_2,
    PoutD_2,
    ServoBus_2,
    ServoPwm_2,
    Stepper_2,
    serial_mc602,
)
from vehicle_wbt_smartcar_msgs.msg import RawState
from vehicle_wbt_smartcar_msgs.srv import (
    Buzzer,
    ReadAnalog,
    ReadBattery,
    ReadEncoders,
    ReadIR,
    ResetEncoders,
    SetDcMotor,
    SetDout,
    SetServoBus,
    SetServoPwm,
    SetStepper,
    SetWheels,
)

DEFAULT_PORTS_YAML = Path(__file__).parent.parent / 'config' / 'mc602_ports.yaml'


class MC602Node(Node):
    """MC602 通用 IO 节点,独占 /dev/ttyUSB*。"""

    def __init__(self) -> None:
        super().__init__('mc602_io')

        # 参数
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 1_000_000)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('sensor_rate_hz', 20.0)
        self.declare_parameter('ports_config', str(DEFAULT_PORTS_YAML))
        self.declare_parameter('auto_ping', True)

        # 加载端口路由表
        cfg_path = self.get_parameter('ports_config').value
        with open(cfg_path, 'r') as f:
            self.ports_cfg = yaml.safe_load(f)

        # 打开串口
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value
        auto_ping = self.get_parameter('auto_ping').value

        if auto_ping and port == 'auto':
            found = serial_mc602.ping_port(baud=baud)
            if found is None:
                raise RuntimeError('No MC602 found on any USB port')
            self.get_logger().info(f'auto-discovered MC602 at {found}')
        else:
            serial_mc602.port = port
            serial_mc602.baudrate = baud
            if not serial_mc602.open():
                raise RuntimeError(f'Cannot open {port}')
            self.get_logger().info(f'opened {port} @ {baud}')

        # 实例化所有设备类(端口由 ports_cfg 决定)
        self._motors = Motors_2(self.ports_cfg['chassis']['motors'])
        self._encoder4 = EncoderMotors4_2()

        arm_cfg = self.ports_cfg['arm']
        self._stepper_y = Stepper_2(port_id=arm_cfg['stepper_y']['port'])
        self._dc_motor_x = Motor_2(port_id=arm_cfg['dc_motor_x']['port'])
        self._servo_pwm_hand = ServoPwm_2(port_id=arm_cfg['servo_pwm_hand']['port'])
        self._servo_bus_wrist = ServoBus_2(port_id=arm_cfg['servo_bus_wrist']['port'])
        self._pump = PoutD_2(port_id=arm_cfg['pump']['port'])
        self._valve = PoutD_2(port_id=arm_cfg['valve']['port'])

        shooter_cfg = self.ports_cfg['shooter']
        self._shooter = PoutD_2(port_id=shooter_cfg['barrel']['port'])

        self._ir_left = Infrared_2(port_id=self.ports_cfg['ir']['left'])
        self._ir_right = Infrared_2(port_id=self.ports_cfg['ir']['right'])

        self._battery = Battry_2()
        self._buzzer = Buzzer_2()

        # 状态缓存(用于 RawState topic 发布)
        self._last_encoders = [0, 0, 0, 0]
        self._last_ir_left = 0.0
        self._last_ir_right = 0.0
        self._last_battery = 0.0
        self._pump_state = False
        self._valve_state = False

        # 服务
        self.create_service(SetWheels, '/mc602/set_wheels', self._on_set_wheels)
        self.create_service(ReadEncoders, '/mc602/read_encoders', self._on_read_encoders)
        self.create_service(ResetEncoders, '/mc602/reset_encoders', self._on_reset_encoders)
        self.create_service(SetServoPwm, '/mc602/set_servo_pwm', self._on_set_servo_pwm)
        self.create_service(SetServoBus, '/mc602/set_servo_bus', self._on_set_servo_bus)
        self.create_service(SetStepper, '/mc602/set_stepper', self._on_set_stepper)
        self.create_service(SetDcMotor, '/mc602/set_dc_motor', self._on_set_dc_motor)
        self.create_service(SetDout, '/mc602/set_pout', self._on_set_pout)
        self.create_service(ReadIR, '/mc602/read_ir', self._on_read_ir)
        self.create_service(ReadBattery, '/mc602/read_battery', self._on_read_battery)
        self.create_service(ReadAnalog, '/mc602/read_analog', self._on_read_analog)
        self.create_service(Buzzer, '/vehicle_wbt/v1/mc602/buzzer', self._on_buzzer)

        # 话题
        self._raw_pub = self.create_publisher(RawState, '/vehicle_wbt/v1/mc602/state/raw', 10)
        self._heartbeat_pub = self.create_publisher(Header, '/vehicle_wbt/v1/mc602/heartbeat', 10)

        # 定时器
        sensor_dt = 1.0 / self.get_parameter('sensor_rate_hz').value
        self.create_timer(sensor_dt, self._tick_sensor)

        control_dt = 1.0 / self.get_parameter('control_rate_hz').value
        self.create_timer(control_dt, self._tick_control)

        self.create_timer(1.0, self._tick_heartbeat)

        self.get_logger().info('MC602 node started')

    # ---- service handlers ----

    def _on_set_wheels(self, req, resp):
        try:
            self._motors.set_speed([req.v0, req.v1, req.v2, req.v3])
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_wheels: {e}')
            resp.success = False
        return resp

    def _on_read_encoders(self, req, resp):
        try:
            encs = self._encoder4.get() or [0, 0, 0, 0]
            if isinstance(encs, int):
                encs = [encs, 0, 0, 0]  # 退化保护
            resp.v = list(encs)[:4] + [0] * (4 - min(4, len(encs)))
            self._last_encoders = list(resp.v)
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_encoders: {e}')
            resp.v = [0, 0, 0, 0]
            resp.success = False
        return resp

    def _on_reset_encoders(self, req, resp):
        try:
            self._motors.reset_encoder()
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'reset_encoders: {e}')
            resp.success = False
        return resp

    def _on_set_servo_pwm(self, req, resp):
        try:
            self._servo_pwm_hand.set_angle(int(req.angle))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_servo_pwm: {e}')
            resp.success = False
        return resp

    def _on_set_servo_bus(self, req, resp):
        try:
            self._servo_bus_wrist.set_angle(int(req.angle), int(req.speed))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_servo_bus: {e}')
            resp.success = False
        return resp

    def _on_set_stepper(self, req, resp):
        try:
            self._stepper_y.set_pwm(int(req.freq))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_stepper: {e}')
            resp.success = False
        return resp

    def _on_set_dc_motor(self, req, resp):
        try:
            self._dc_motor_x.set_speed(int(req.speed))
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_dc_motor: {e}')
            resp.success = False
        return resp

    def _on_set_pout(self, req, resp):
        try:
            port = int(req.port)
            state = bool(req.state)
            if port == self.ports_cfg['arm']['pump']['port']:
                self._pump.set(1 if state else 0)
                self._pump_state = state
            elif port == self.ports_cfg['arm']['valve']['port']:
                self._valve.set(1 if state else 0)
                self._valve_state = state
            elif port == self.ports_cfg['shooter']['barrel']['port']:
                self._shooter.set(1 if state else 0)
            else:
                self.get_logger().warn(f'unknown pout port: {port}')
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'set_pout: {e}')
            resp.success = False
        return resp

    def _on_read_ir(self, req, resp):
        try:
            port = int(req.port)
            if port == self.ports_cfg['ir']['left']:
                val = self._ir_left.no_act() or [0]
            elif port == self.ports_cfg['ir']['right']:
                val = self._ir_right.no_act() or [0]
            else:
                val = [0]
            raw = val[0] if isinstance(val, list) and val else 0
            resp.distance_m = float(raw) / 1000.0
            if port == self.ports_cfg['ir']['left']:
                self._last_ir_left = resp.distance_m
            else:
                self._last_ir_right = resp.distance_m
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_ir: {e}')
            resp.distance_m = 0.0
            resp.success = False
        return resp

    def _on_read_battery(self, req, resp):
        try:
            v = self._battery.read()
            resp.voltage_v = float(v) if v is not None else 0.0
            self._last_battery = resp.voltage_v
            resp.success = v is not None
        except Exception as e:
            self.get_logger().error(f'read_battery: {e}')
            resp.voltage_v = 0.0
            resp.success = False
        return resp

    def _on_read_analog(self, req, resp):
        try:
            # 简化:用 Infrared_2 在不同 mode 切换,或复用 Stepper_2 的位置
            # Phase 1 只支持 Y 轴磁限位(port=6),其他返回 0
            from vehicle_wbt_smartcar_hw import AnalogInput_2
            sensor = AnalogInput_2(port_id=int(req.port))
            val = sensor.no_act() or [0]
            resp.value = int(val[0]) if isinstance(val, list) and val else 0
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'read_analog: {e}')
            resp.value = 0
            resp.success = False
        return resp

    def _on_buzzer(self, req, resp):
        try:
            self._buzzer.rings(int(req.freq_hz), int(req.duration_ms) / 1000.0)
            resp.success = True
        except Exception as e:
            self.get_logger().error(f'buzzer: {e}')
            resp.success = False
        return resp

    # ---- timers ----

    def _tick_sensor(self):
        """20Hz:读 IR + battery + encoder,发布 RawState。"""
        try:
            # 读编码器(轻量,batch)
            encs = self._encoder4.get()
            if isinstance(encs, list) and len(encs) >= 4:
                self._last_encoders = list(encs[:4])
            # 读电池
            v = self._battery.read()
            if v is not None:
                self._last_battery = float(v)
            # 读红外(各自 no_act)
            ir_l = self._ir_left.no_act()
            if ir_l and isinstance(ir_l, list):
                self._last_ir_left = float(ir_l[0]) / 1000.0
            ir_r = self._ir_right.no_act()
            if ir_r and isinstance(ir_r, list):
                self._last_ir_right = float(ir_r[0]) / 1000.0
        except Exception as e:
            self.get_logger().warn(f'sensor tick: {e}')

        msg = RawState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.encoders = self._last_encoders
        msg.ir_left_m = self._last_ir_left
        msg.ir_right_m = self._last_ir_right
        msg.battery_v = self._last_battery
        msg.arm_y_pos = 0  # Phase 1 不做编码器回读
        msg.arm_x_pos = 0
        msg.pump_on = self._pump_state
        msg.valve_on = self._valve_state
        self._raw_pub.publish(msg)

    def _tick_control(self):
        """50Hz 占位:Phase 1 不做闭环控制,只保活。"""
        pass

    def _tick_heartbeat(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self._heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MC602Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: colcon build 验证**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
colcon build --packages-up-to vehicle_wbt_smartcar_bridge
```

Expected: BUILD SUCCESSFUL。

- [ ] **Step 3: import 验证(不启动节点)**

```bash
source install/setup.bash
python3 -c "
from vehicle_wbt_smartcar_bridge.mc602_node import MC602Node
print('MC602Node class OK')
"
```

Expected: `MC602Node class OK`

- [ ] **Step 4: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_bridge/vehicle_wbt_smartcar_bridge/mc602_node.py
git commit -m "feat(bridge): MC602Node with 12 services + 2 topics + 3 timers"
```

---

## Task 10: 写 launch 文件

**Files:**
- Create: `ros2_ws/src/vehicle_wbt_smartcar_bridge/launch/mc602.launch.py`

- [ ] **Step 1: 写 `launch/mc602.launch.py`**

```python
"""MC602 节点启动文件。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),

        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB1',
            description='MC602 串口设备路径(auto 表示自动扫描)'),
        DeclareLaunchArgument(
            'baud', default_value='1000000',
            description='MC602 波特率'),
        DeclareLaunchArgument(
            'control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument(
            'sensor_rate_hz', default_value='20.0'),

        Node(
            package='vehicle_wbt_smartcar_bridge',
            executable='mc602_node',
            name='mc602_io',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud': LaunchConfiguration('baud'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'sensor_rate_hz': LaunchConfiguration('sensor_rate_hz'),
            }],
        ),
    ])
```

- [ ] **Step 2: colcon build 重新构建(让 launch 文件被安装)**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
colcon build --packages-up-to vehicle_wbt_smartcar_bridge
```

Expected: BUILD SUCCESSFUL + `install/.../share/vehicle_wbt_smartcar_bridge/launch/mc602.launch.py` 存在。

- [ ] **Step 3: 验证 launch 文件可发现**

```bash
source install/setup.bash
ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py --show-args
```

Expected: 列出所有 DeclareLaunchArgument。

- [ ] **Step 4: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add ros2_ws/src/vehicle_wbt_smartcar_bridge/launch/
git commit -m "feat(bridge): mc602.launch.py with serial_port/baud/rate args"
```

---

## Task 11: 部署件 — systemd unit

**Files:**
- Create: `deploy/systemd/vehicle-wbt-mc602.service`

- [ ] **Step 1: 写 systemd unit**

```bash
mkdir -p /home/xrak/workspace/rak-car/deploy/systemd
```

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
ExecStartPre=/bin/bash -c 'until ls /dev/ttyUSB* 2>/dev/null | head -1; do sleep 1; done'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/jetson/workspace/rak-car/ros2_ws/install/setup.bash && exec ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py serial_port:=/dev/ttyUSB1 baud:=1000000'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 2: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add deploy/systemd/
git commit -m "deploy(systemd): vehicle-wbt-mc602.service with restart on failure"
```

---

## Task 12: 部署件 — env file + DDS config + udev

**Files:**
- Create: `deploy/ros_env.sh`
- Create: `deploy/cyclonedds/cyclonedds.xml`
- Create: `deploy/udev/99-usbvideo.rules`

- [ ] **Step 1: 写 env file**

`deploy/ros_env.sh`:
```bash
# /etc/vehicle-wbt/ros.env - Jetson 端 MC602 节点环境变量
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/cyclonedds.xml
VEHICLE_WBT_SERIAL=/dev/ttyUSB1
```

- [ ] **Step 2: 写 CycloneDDS 配置**

`deploy/cyclonedds/cyclonedds.xml`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="42">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize>2MiB</SocketReceiveBufferSize>
      <Watermark>256KiB</Watermark>
    </Internal>
  </Domain>
</CycloneDDS>
```

- [ ] **Step 3: 写 udev 规则(摄像头)**

`deploy/udev/99-usbvideo.rules`:
```
# Aveo SP2812 cameras (vendor 1871:0110) - 通过 devpath 区分前后
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1871", ATTRS{idProduct}=="0110", ENV{ID_PATH}=="*usb-3*", SYMLINK+="cam3"
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1871", ATTRS{idProduct}=="0110", ENV{ID_PATH}=="*usb-4*", SYMLINK+="cam4"
```

**注意**:USB devpath 在 Jetson 上需实际确认,先用这个占位,Jetson 端调试时根据 `udevadm info /dev/videoX` 调整。

- [ ] **Step 4: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add deploy/
git commit -m "deploy: ros_env.sh + cyclonedds.xml + udev rules"
```

---

## Task 13: 写 API 文档(`docs/integration/LOWLEVEL_API.md`)

**Files:**
- Create: `docs/integration/LOWLEVEL_API.md`
- Create: `docs/integration/DEV_QUICKSTART.md`

- [ ] **Step 1: 创建目录**

```bash
mkdir -p /home/xrak/workspace/rak-car/docs/integration
```

- [ ] **Step 2: 写 `LOWLEVEL_API.md`**

```markdown
# MC602 Low-Level API

Jetson 端 MC602 节点暴露的通用 ROS2 接口,供上层业务节点(底盘/臂/枪/LLM)远程调用。

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
ros2 service list | grep /mc602
ros2 service call /mc602/read_battery vehicle_wbt_smartcar_msgs/srv/ReadBattery
ros2 service call /vehicle_wbt/v1/mc602/buzzer vehicle_wbt_smartcar_msgs/srv/Buzzer "{freq_hz: 440, duration_ms: 200}"
ros2 service call /mc602/set_wheels vehicle_wbt_smartcar_msgs/srv/SetWheels "{v0: 0, v1: 0, v2: 0, v3: 0}"
```

## Service 接口

### `/mc602/set_wheels` (SetWheels)

```python
req = SetWheels.Request(v0=30, v1=30, v2=30, v3=30)  # -100..100
# 4 个麦轮速度,内部用 Motors_2.set_speed 转发
```

### `/mc602/read_encoders` (ReadEncoders)

返回 4 个 int32 编码器值(增量式,从下位机读取,推一下车数字会变)。

```python
req = ReadEncoders.Request()
resp = await client.call_async(req)
print(resp.v)  # [v0, v1, v2, v3]
```

### `/mc602/reset_encoders` (ResetEncoders)

清零所有编码器。建议在 chassis_node 启动时调用一次。

### `/mc602/set_servo_pwm` (SetServoPwm)

PWM 舵机(机械臂手爪)。`port` 来自 `mc602_ports.yaml::arm::servo_pwm_hand::port`(默认 3)。

```python
req = SetServoPwm.Request(port=3, angle=60)  # 0..180
```

### `/mc602/set_servo_bus` (SetServoBus)

总线舵机(机械臂腕部)。`port=4`,`angle` + `speed`(速度,1..100)。

```python
req = SetServoBus.Request(port=4, angle=90, speed=60)
```

### `/mc602/set_stepper` (SetStepper)

步进电机速度(机械臂 Y 轴)。`port=1`,`freq` 是 -100..100 的速度。

### `/mc602/set_dc_motor` (SetDcMotor)

直流电机速度(机械臂 X 轴)。`port=2`,`speed` -100..100。

### `/mc602/set_pout` (SetDout)

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

### `/mc602/read_ir` (ReadIR)

红外测距。`port` 7=左, 8=右。返回 `distance_m`(米,float)。

### `/mc602/read_battery` (ReadBattery)

电池电压(伏)。返回 `voltage_v`。

### `/mc602/read_analog` (ReadAnalog)

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

client_pwm = self.create_client(SetServoPwm, '/mc602/set_servo_pwm')
client_bus = self.create_client(SetServoBus, '/mc602/set_servo_bus')

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
- 写类 service 失败时**不保证硬件状态**;建议上层用 `/mc602/read_ir` 之类读回做验证
- 多个 client 并发调同一 service:内部 SDK 单例 + lock 保证串口帧不交错,但**应用层应节流**(50Hz 调一次足够)

## 限制(Phase 1)

- ❌ 底盘 PID `move_to_position`:Phase 1 不实现,底盘同事自己写
- ❌ 机械臂 PID 闭环:Phase 1 不实现,机械臂同事自己写
- ❌ LED 屏 / 蓝牙手柄 / 数码管:Phase 5+
- ❌ 老 C++ `ros2_control` 栈仍在运行:Phase 4 退役,**注意 chassis/arm 服务可能被双向消费**——同事开发时暂时不用老 C++ 的话题(`/cmd/vel_safe` 等),只调 `/vehicle_wbt/v1/mc602/*`
```

- [ ] **Step 3: 写 `DEV_QUICKSTART.md`(给同事的 5 分钟指南)**

```markdown
# 同事开发上手 5 分钟

你是上层同事(底盘/机械臂/枪/LLM),只需要:

## 1. 在你的开发机上拉 repo + colcon build

```bash
cd ~/workspace/rak-car/ros2_ws
git pull
colcon build --packages-up-to vehicle_wbt_smartcar_bridge
source install/setup.bash
```

## 2. 验证能连通 Jetson

```bash
export ROS_DOMAIN_ID=42
ros2 service call /mc602/read_battery vehicle_wbt_smartcar_msgs/srv/ReadBattery
# → 应看到 voltage_v: ~11.x
```

## 3. 创建你自己的 package

```bash
cd ~/workspace/rak-car/ros2_ws/src
ros2 pkg create --build-type ament_python vehicle_wbt_smartcar_<你的> \
    --dependencies vehicle_wbt_smartcar_msgs rclpy
```

## 4. 写你的节点,只调 `/vehicle_wbt/v1/mc602/*` service

API 文档:`docs/integration/LOWLEVEL_API.md`

## 5. 在 dev 机器上启动你的节点

```bash
cd ~/workspace/rak-car/ros2_ws
colcon build --packages-select vehicle_wbt_smartcar_<你的>
source install/setup.bash
ros2 run vehicle_wbt_smartcar_<你的> <节点名>
```

你的节点会通过 LAN DDS 自动发现 Jetson 端的 `/vehicle_wbt/v1/mc602/*`,**不需要 SSH Jetson**。

## 6. 调试技巧

```bash
# 看 Jetson 端发的数据流
ros2 topic echo /vehicle_wbt/v1/mc602/state/raw

# 看 Jetson 端有什么 service/topic
ros2 service list | grep mc602
ros2 topic list | grep mc602

# 看 Jetson 节点日志(从你的开发机)
ros2 node info /mc602_io
```
```

- [ ] **Step 4: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add docs/integration/
git commit -m "docs: LOWLEVEL_API.md + DEV_QUICKSTART.md"
```

---

## Task 14: 更新 CLAUDE.md

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: 在 CLAUDE.md 顶部"Repository layout"加新包**

在 `## Repository layout (this branch)` 后面加:

```
- vehicle_wbt_smartcar_hw/         # 协议层(纯库,1:1 对齐 SDK)
- vehicle_wbt_smartcar_msgs/        # ROS2 接口契约
- vehicle_wbt_smartcar_bridge/      # 唯一 mc602_io 节点 + launch
```

- [ ] **Step 2: 在 CLAUDE.md "Build / run on this robot" 加新 launch**

```bash
# MC602 通用驱动(Jetson 自启)
ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py serial_port:=/dev/ttyUSB1 baud:=1000000
```

- [ ] **Step 3: 加 "API for dev boxes" 章节**

```markdown
## API for dev boxes (上层开发)

Jetson 端通过 ROS2 service/topic 暴露 MC602 接口给 LAN 上的开发机:

- 12 service: `/mc602/set_wheels`、`/mc602/read_encoders`、`/mc602/reset_encoders`、
  `/mc602/set_servo_pwm`、`/mc602/set_servo_bus`、`/mc602/set_stepper`、
  `/mc602/set_dc_motor`、`/mc602/set_pout`、`/mc602/read_ir`、`/mc602/read_battery`、
  `/mc602/read_analog`、`/vehicle_wbt/v1/mc602/buzzer`
- 2 topic: `/vehicle_wbt/v1/mc602/state/raw`(50 Hz RawState.msg)、`/vehicle_wbt/v1/mc602/heartbeat`(1 Hz)

完整文档:`docs/integration/LOWLEVEL_API.md`
同事上手:`docs/integration/DEV_QUICKSTART.md`

**关键约束**: 开发机必须 `export ROS_DOMAIN_ID=42`。
```

- [ ] **Step 4: Commit**

```bash
cd /home/xrak/workspace/rak-car
git add CLAUDE.md
git commit -m "docs(claude-md): document Phase 1 MC602 architecture + API"
```

---

## Task 15: Jetson 端真硬件冒烟测试

**Files:** 无,纯验证。

- [ ] **Step 1: Jetson 端 colcon build 完整 workspace**

```bash
cd /home/xrak/workspace/rak-car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_wbt_smartcar_bridge
```

Expected: BUILD SUCCESSFUL。

- [ ] **Step 2: 启动 mc602 节点(单独终端,前台跑方便看日志)**

```bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch vehicle_wbt_smartcar_bridge mc602.launch.py \
    serial_port:=/dev/ttyUSB1 baud:=1000000
```

Expected: 日志 `MC602 node started`,节点持续运行。

- [ ] **Step 3: 验证 `/vehicle_wbt/v1/mc602/state/raw` 数据流**

```bash
source install/setup.bash
ROS_DOMAIN_ID=42 timeout 3 ros2 topic echo /vehicle_wbt/v1/mc602/state/raw
```

Expected: 看到 RawState 消息,每行 20Hz,字段值合理(encoders 全 0 静止时,ir_*_m 0.1~2.0m,battery_v 11~12V)。

- [ ] **Step 4: 验证 buzzer**

```bash
ROS_DOMAIN_ID=42 ros2 service call /vehicle_wbt/v1/mc602/buzzer \
    vehicle_wbt_smartcar_msgs/srv/Buzzer "{freq_hz: 440, duration_ms: 200}"
# 听到 0.2 秒 440Hz 蜂鸣 → success: True
```

- [ ] **Step 5: 验证 read_battery**

```bash
ROS_DOMAIN_ID=42 ros2 service call /mc602/read_battery \
    vehicle_wbt_smartcar_msgs/srv/ReadBattery {}
# voltage_v: 11.x (锂电池),success: True
```

- [ ] **Step 6: 验证 read_encoders + set_wheels + reset_encoders**

```bash
# 静止时读
ROS_DOMAIN_ID=42 ros2 service call /mc602/read_encoders \
    vehicle_wbt_smartcar_msgs/srv/ReadEncoders {}
# v: [0, 0, 0, 0]

# 清零
ROS_DOMAIN_ID=42 ros2 service call /mc602/reset_encoders \
    vehicle_wbt_smartcar_msgs/srv/ResetEncoders {}

# 缓慢转 1 秒
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_wheels \
    vehicle_wbt_smartcar_msgs/srv/SetWheels "{v0: 20, v1: 20, v2: 20, v3: 20}"

# 1 秒后再读编码器
sleep 1
ROS_DOMAIN_ID=42 ros2 service call /mc602/read_encoders \
    vehicle_wbt_smartcar_msgs/srv/ReadEncoders {}
# v: [有变化的数, ...]

# 立即停止
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_wheels \
    vehicle_wbt_smartcar_msgs/srv/SetWheels "{v0: 0, v1: 0, v2: 0, v3: 0}"
```

Expected: 4 轮应该缓慢同向转,1 秒后停止;编码器数字有明显变化。

- [ ] **Step 7: 验证 read_ir**

```bash
ROS_DOMAIN_ID=42 ros2 service call /mc602/read_ir \
    vehicle_wbt_smartcar_msgs/srv/ReadIR "{port: 7}"
# distance_m: ~0.5 (或合理的红外距离值)

# 用手遮住左红外,再读
ROS_DOMAIN_ID=42 ros2 service call /mc602/read_ir \
    vehicle_wbt_smartcar_msgs/srv/ReadIR "{port: 7}"
# distance_m: 接近 0
```

- [ ] **Step 8: 验证 set_pout(发射器或泵)**

```bash
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_pout \
    vehicle_wbt_smartcar_msgs/srv/SetDout "{port: 4, state: true}"
# 听到发射器/继电器咔嗒声
sleep 1
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_pout \
    vehicle_wbt_smartcar_msgs/srv/SetDout "{port: 4, state: false}"
```

- [ ] **Step 9: 验证 set_servo_pwm / set_servo_bus(仅当舵机已接)**

```bash
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_servo_pwm \
    vehicle_wbt_smartcar_msgs/srv/SetServoPwm "{port: 3, angle: 90}"
# 看到/听到手爪舵机摆动到 90°

ROS_DOMAIN_ID=42 ros2 service call /mc602/set_servo_bus \
    vehicle_wbt_smartcar_msgs/srv/SetServoBus "{port: 4, angle: 90, speed: 60}"
# 看到/听到腕部总线舵机摆动
```

- [ ] **Step 10: 验证 read_analog(限位开关)**

```bash
ROS_DOMAIN_ID=42 ros2 service call /mc602/read_analog \
    vehicle_wbt_smartcar_msgs/srv/ReadAnalog "{port: 6}"
# value: 一些原始 ADC 值 (0..4095)
# 手动触发磁限位,值应该有变化
```

- [ ] **Step 11: 验证 set_dc_motor / set_stepper(机械臂电机)**

```bash
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_dc_motor \
    vehicle_wbt_smartcar_msgs/srv/SetDcMotor "{port: 2, speed: 20}"
# 看到 X 轴电机慢动
sleep 1
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_dc_motor \
    vehicle_wbt_smartcar_msgs/srv/SetDcMotor "{port: 2, speed: 0}"

ROS_DOMAIN_ID=42 ros2 service call /mc602/set_stepper \
    vehicle_wbt_smartcar_msgs/srv/SetStepper "{port: 1, freq: 50}"
# 看到 Y 轴步进电机慢动
sleep 1
ROS_DOMAIN_ID=42 ros2 service call /mc602/set_stepper \
    vehicle_wbt_smartcar_msgs/srv/SetStepper "{port: 1, freq: 0}"
```

- [ ] **Step 12: 如果冒烟测试有任何失败,记录到 README_DEBUG_NOTES.md**

如果有失败的 service,记录到 `docs/integration/DEBUG_NOTES.md` 供后续排查。**不阻塞其他同事开发**——失败的 service 在他们的 package 里可以临时不用。

- [ ] **Step 13: 停止前台节点(Ctrl+C)**

- [ ] **Step 14: Commit(如果有 debug notes)**

```bash
cd /home/xrak/workspace/rak-car
git add docs/integration/DEBUG_NOTES.md  # 如果创建
git commit -m "docs: Phase 1 hardware smoke test results"
```

---

## Task 16: 部署 Jetson 端 systemd(可选,Phase 1 末尾)

**Files:** 无,纯部署。

- [ ] **Step 1: Jetson 端复制部署件**

```bash
# Jetson 端
sudo cp /home/xrak/workspace/rak-car/deploy/systemd/vehicle-wbt-mc602.service \
    /etc/systemd/system/
sudo mkdir -p /etc/vehicle-wbt
sudo cp /home/xrak/workspace/rak-car/deploy/ros_env.sh /etc/vehicle-wbt/ros.env
sudo cp /home/xrak/workspace/rak-car/deploy/cyclonedds/cyclonedds.xml /etc/cyclonedds.xml
sudo cp /home/xrak/workspace/rak-car/deploy/udev/99-usbvideo.rules \
    /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

- [ ] **Step 2: 启用 systemd**

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now vehicle-wbt-mc602
sudo systemctl status vehicle-wbt-mc602
```

Expected: active (running)。

- [ ] **Step 3: 从开发机验证 systemd 启动的节点可达**

```bash
# 开发机
export ROS_DOMAIN_ID=42
ros2 service list | grep mc602
ros2 service call /mc602/read_battery vehicle_wbt_smartcar_msgs/srv/ReadBattery
```

Expected: 看到 service + 读到电压。

- [ ] **Step 4: 测试重启**

```bash
# Jetson 端
sudo reboot
# 重启后等 30s
# 开发机
ros2 service list | grep mc602
```

Expected: 重启后 systemd 自动拉起节点,开发机仍能看到 service。

- [ ] **Step 5: Commit(如果有调试) + 打 tag**

```bash
cd /home/xrak/workspace/rak-car
git tag phase1-mc602-driver-ready
git push origin robot-stable --tags
```

---

## Self-Review(写完 plan 后自检)

### Spec coverage

| Spec 章节 | Plan Task |
|---|---|
| §2 团队分工 | Task 8 (config) + Task 13 (docs/integration) |
| §3 ROS2 packages 划分 | Task 1 (hw) + Task 7 (msgs) + Task 8 (bridge) |
| §4.1 Service 接口 | Task 7 (12 srv) + Task 9 (handlers) |
| §4.2 Topic 接口 | Task 7 (RawState.msg) + Task 9 (publishers) |
| §4.3 设计原则 | Task 9 (失败语义 + 路由表) |
| §5 协议层模块结构 | Task 1-6 |
| §5.2 关键决定 | Task 2 (serial) + Task 3 (port_id=None 处理) + Task 5 (logger 替换) |
| §5.3 设备类清单 | Task 3 (14 个类) |
| §5.4 不做的 | Task 9 (无 LifecycleNode) |
| §6.1 单节点结构 | Task 9 |
| §6.2 设备路由表 | Task 8 (mc602_ports.yaml) + Task 9 (路由) |
| §6.3 失败语义 | Task 9 (每个 handler try/except) |
| §7.1 systemd | Task 11 |
| §7.2 env file | Task 12 |
| §7.3 CycloneDDS | Task 12 |
| §7.4 udev | Task 12 |
| §8 API 文档 | Task 13 |
| §9 CLAUDE.md | Task 14 |
| §10 真硬件冒烟 | Task 15 |
| §11 不做的 | 各 Task 明确注释 |

### Placeholder scan

✓ 无 TBD/TODO/"implement later"。

### Type consistency

- Task 3 设备类名 → Task 6 重导出 → Task 9 import:全部对齐。
- Task 7 srv 字段 → Task 9 handler req.v0 等:对齐。
- Task 8 ports_cfg → Task 9 路由:对齐(mc602_ports.yaml 的 key 在 mc602_node.py 里一致)。

### Spec gap

- ✓ Spec §10 提到的 "冒烟测试 CLI" 在 Task 15 全部覆盖。
- ✓ Spec §4.3 的"失败返回 success: false"在 Task 9 每个 handler 实现。
- ✓ Spec §11 的"Phase 1 不做的"在每个 Task 的"不做的"明确注释。

---

## Plan 结束

**Plan 包含 16 个 Task**,预计执行时间:
- Task 1-8: ~2 小时(主要是抄 SDK + 写 boilerplate)
- Task 9: ~1.5 小时(主节点 ~300 行)
- Task 10-14: ~1 小时(部署件 + 文档)
- Task 15: ~1 小时(真硬件冒烟,可能反复调试)
- Task 16: ~0.5 小时(systemd 部署)
- **总计 ~6 小时**(今晚完成)

执行完成后,4 个同事明天即可在 dev 机器上:
```bash
export ROS_DOMAIN_ID=42
ros2 service call /mc602/read_battery vehicle_wbt_smartcar_msgs/srv/ReadBattery
```