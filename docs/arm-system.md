# 机械臂子系统

> **底层代码不可修改。** `vehicle/arm/arm_base.py` 是机械臂控制核心。

## 硬件组成

```
ArmBase
    ├── 步进电机 × 2 (关节旋转)
    │   └── StepperWrap → Stepper_2 (MC602 步进驱动)
    ├── 总线舵机 × N (末端执行器)
    │   └── ServoBus → ServoBus_2 (MC602 总线舵机)
    ├── 真空泵 (吸取)
    │   └── PoutD → PoutD_2 (MC602 数字输出)
    └── 电机 (伸缩/夹爪)
        └── MotorWrap → Motor_2 (MC602 电机)
```

## 配置文件

`vehicle/arm/arm_cfg.yaml`：

```yaml
arm:
  stepper1:                    # 第一个步进电机
    id: 1                      # 端口号
    reverse: 1                 # 方向修正
    perimeter: 0.008           # 导程 (m/圈)
  stepper2:                    # 第二个步进电机
    id: 2
    reverse: 1
    perimeter: 0.008
  servo_bus:                   # 总线舵机
    id: 2                      # 端口号
  motor:                       # 伸缩电机
    id: 5
    reverse: -1
    perimeter: 0.06            # 周长 (m)
  vacuum_pump:                 # 真空泵
    port: 4                    # 数字输出端口
```

## ArmBase API

### 关节控制

| 方法 | 参数 | 功能 |
|------|------|------|
| `set(angle1, angle2)` | 弧度 | 设置两个步进电机目标角度 |
| `set_angle(joint, angle)` | 关节号, 弧度 | 设置单个关节 |
| `get_angle(joint)` | 关节号 | 读取当前关节角度 (rad) |
| `reset()` | — | 所有关节归零 |

### 末端执行器

| 方法 | 参数 | 功能 |
|------|------|------|
| `grap(val)` | 1=吸取, 0=释放 | 真空泵控制 |
| `set_servo(angle)` | 角度 | 舵机角度控制 |
| `set_servo_speed(speed)` | 速度 | 舵机连续转动 |

### 步进电机控制细节

`StepperWrap` 的换算：

```python
# 步进参数
步距角 = 1.8°
细分 = 16
每步角度 = 1.8° / 16 = 0.1125°

# 换算系数
stepper2rad = π/180 × 1.8/16 = 0.001963 rad/step
rad2pwm = 16 × 180 / (1.8 × π) = 509.3 step/rad
```

`set_rad()` 使用 PID 控制定位：

```python
def set_rad(self, rad, time=1.5):
    pid = PID(5, 0, 0)
    pid.setpoint = rad
    while abs(self.get_rad() - rad) > 0.1:
        self.set_angular(pid(self.get_rad()))
    self.set_angular(0)  # 到位后停止
```

## MyTask 任务原语

`task_func.py` 中的 `MyTask` 类封装了所有任务级的机械臂操作。

### 两阶段执行模式 (arm_set)

大多数任务方法支持 `arm_set=True` 参数：

```
第一阶段 (arm_set=True):
    移动机械臂到观测位置 → 返回目标坐标
    (此时不执行实际动作，只做视觉检测)

第二阶段 (arm_set=False):
    根据导航结果调整位置 → 执行实际动作
    (抓取/放置/浇水等)
```

### MyTask 方法一览

| 方法 | 参数 | 功能 | 两阶段 |
|------|------|------|:---:|
| `pick_up_cylinder(radius, side, arm_set)` | 0/1/2=小/中/大, 左/右 | 抓取圆柱体 | ✅ |
| `put_down_cylinder(radius, side)` | 同上 | 放置圆柱体 | ❌ |
| `planting(side, radius, arm_set)` | 左/右, 0=灯/1=蜂鸣/2=浇水 | 植物护理 | ✅ |
| `weather_set(num, arm_set)` | 0~4 | 天气显示舵机 | ✅ |
| `bmi_set(num, arm_set)` | 1~4 | BMI 结果显示 | ✅ |
| `get_ingredients(side, ocr_mode, arm_set)` | 左/右, OCR模式 | 食材识别位置 | ✅ |
| `pick_ingredients(num, row, arm_set)` | 编号, 行号 | 从货架取食材 | ✅ |
| `get_answer(arm_set)` | — | 答题面板位置 | ✅ |
| `set_food(num, row, arm_set)` | 编号, 行号 | 食材放到货架 | ✅ |
| `eject(area)` | 区域编号 | 弹射 | ❌ |
| `help_peo(arm_set)` | — | 帮人伸臂 | ✅ |
| `beep()` | — | 蜂鸣器 | ❌ |
| `reset()` | — | 机械臂归零 | ❌ |

### 圆柱体尺寸

| radius | 直径 | 对应常量 |
|:------:|------|---------|
| 0 | 小 | `dis_list = {0: 0.080}` |
| 1 | 中 | `dis_list = {1: 0.0535}` |
| 2 | 大 | `dis_list = {2: 0.050}` |

### Ejection 弹射机构

```python
class Ejection:
    def reset(self):
        # 电机回退直到堵转（编码器不再变化）
        while encoder 变化:
            motor.set_speed(-50)

    def eject(self, x, vel):
        self.reset()           # 1. 回退
        pout.set(2)            # 2. 打开电磁阀
        motor.set_speed(-vel)  # 3. 回退蓄力
        stepper.set(x)         # 4. 旋转弹仓
        motor.set_speed(vel)   # 5. 发射
        pout.set(1)            # 6. 关闭电磁阀
```

## 机械臂运动范围

> **⚠️ 这些值是调好的安全范围，不要随意修改！**

关节角度限制在 `arm_cfg.yaml` 中配置。超出范围可能导致机械臂碰撞或损坏。

## 舵机端口分配

| 端口 | 用途 |
|------|------|
| 1 | 机械臂关节舵机 |
| 2 | 天气显示舵机 / BMI 显示舵机 |
| 3 | 其他辅助舵机 |

具体端口分配以 `task_func.py` 中的初始化为准。
