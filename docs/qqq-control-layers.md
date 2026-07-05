# qqq.py 机器控制层次分析

> 自底向上梳理 `main/qqq.py` 如何通过 6 层架构控制自主小车完成竞赛任务

## 概述

`qqq.py` 是竞赛任务的总调度脚本，运行在 NVIDIA Jetson 嵌入式计算平台上。它控制一辆**麦克纳姆轮自主小车 + 多自由度机械臂**，通过串口与 MC601/MC602 单片机通信，通过 ZMQ 调用 AI 推理服务（PaddlePaddle），通过 HTTP API 调用 LLM（文心一言）。

初始化入口只有一行代码，但会**级联初始化**全部 6 层：

```python
my_car = MyCar()  # 串口扫描 → 控制器检测 → 电机/舵机/传感器 → 运动学 → 摄像头 → ZMQ推理 → 机械臂归零
```

---

## 第 1 层：硬件驱动层

**涉及模块：** `vehicle/base/controller_wrap.py`, `vehicle/base/serial_wrap.py`

最底层，直接与物理硬件通信。Jetson（上位机）通过 USB/CH340 串口连接 MC601 或 MC602 单片机（下位机），单片机再驱动各类执行器和传感器。

### 硬件清单

| 硬件 | 驱动类 | 通信方式 | qqq.py 中的间接调用 |
|------|--------|----------|---------------------|
| 直流电机 ×4 | `Motors` | 串口 → MC601/MC602 PWM | `set_pose_offset()` → `set_velocity()` |
| 舵机总线 | `ServoBus` | 串口 PWM 指令 | `arm.set_hand_angle(60)` |
| 步进电机 | `StepperWrap` | 串口脉冲指令 | `ejection.eject()` |
| 红外传感器 | `Infrared` | 串口 ADC 读取 | `lane_sensor()` |
| 真空泵 | `PoutD` | GPIO 控制 | `arm.grap(1)`（吸）/ `arm.grap(0)`（松） |
| 蜂鸣器 | `Beep` | 串口指令 | `beep()` |
| LED 灯 | `LedLight` | 串口指令 | 状态指示 |

### 关键机制：`ctl_id` 全局分发

系统启动时自动检测控制器型号（MC601 波特率 380400，MC602 波特率 1000000），设置全局变量 `ctl_id`。所有硬件类内部同时持有 MC601 和 MC602 两套实现，通过 `ctl_id` 分发：

```python
ctl_id = get_devid()  # 0=MC601, 1=MC602

class Motors:
    def __init__(self, port_id):
        self.motor_1 = Motor_1(port=port_id)   # MC601 实现
        self.motor_2 = Motor_2(port_id=port_id) # MC602 实现

    def set_speed(self, speed):
        fucs = [self.motor_1.rotate, self.motor_2.set_speed]
        fucs[ctl_id](speed)  # 全局分发
```

> **注意：** MC601 编码器为软件模拟（速度 × 时间积分），MC602 编码器为真实硬件值。里程计精度依赖 MC602。

---

## 第 2 层：车辆运动学层

**涉及模块：** `vehicle/driver/vehicle_base.py` → `CarBase`, `OdometryBase`, `ChassisBase`

将电机的原始转速转换为有意义的车辆运动指令。

### 底盘模型

- **类型：** 麦克纳姆轮（Mecanum），支持全向移动
- **能力：** 前行/后退、左右横移、原地旋转、任意组合运动
- **配置：** `vehicle/driver/cfg_vehicle.yaml`（轮径、轴距、PID 参数）

### 里程计

- 从电机编码器数据通过**死推算（dead reckoning）**积分出世界坐标系下的位姿 `[x, y, θ]`
- 更新公式（`OdometryBase.odom_update`）：通过旋转矩阵将车身坐标系位移转换到世界坐标系

### PID 速度控制

- 每个车轮独立速度 PID，参数可配置
- 车道跟随使用 `LanePidCal`（组合 Y 方向偏移 PID + 角度偏移 PID）

### qqq.py 中的运动控制调用模式

```python
# 相对位姿移动（最常用）—— [x前移, y横移, θ旋转] 米/弧度
my_car.set_pose_offset([0.18, 0, 0], 1)        # 前移 0.18m，超时 1s
my_car.set_pose_offset([0, 0, math.pi/4], 1)   # 原地旋转 45°
my_car.set_pose_offset([0, 0.05, 0], 0.5)      # 横移 0.05m

# 速度-时间控制
my_car.set_vel_time(vx, vy, vω, t)             # 以指定速度运行 t 秒

# 车道线 PID 巡航
my_car.lane_dis_offset(v, dis)                 # 沿车道线前进 dis 米

# 获取当前位姿
pose = my_car.get_odometry()                   # 返回 [x, y, θ]
```

---

## 第 3 层：感知层

**涉及模块：** `camera/`, `infer_cs/`, `paddle_jetson/`

为小车提供"眼睛"和"大脑"，通过 ZMQ 客户端-服务器架构调用 PaddlePaddle AI 模型。

### 摄像头

- 两个 USB 摄像头（`/dev/cam*`），分别用于前方和侧面
- `Camera` 类以 daemon 线程 30fps 采集，无锁读取 `self.frame`

### 推理服务（ZMQ 架构）

| 服务名 | 类型 | 端口 | 模型 | 用途 |
|--------|------|------|------|------|
| `lane` | LaneInfer | 5001 | 内置算法 | 车道线分割 |
| `task` | YoloeInfer | 5002 | task_wbt2025 | 任务目标检测 |
| `front` | YoloeInfer | 5003 | front_model2 | 前方目标检测 |
| `ocr` | OCRReco | 5004 | ch_PP-OCRv3 | 文字识别 |

### 推理流程

```
Camera(30fps线程) → ZMQ Client → ZMQ Server → PaddlePaddle 模型 → 返回结果
```

`ClintInterface` 自动管理连接：首次调用时自动启动 `infer_back_end.py` 服务进程。

### qqq.py 中的感知调用

```python
# 车道线跟随 + 目标检测定位（种植任务）
pose_dict = my_car.lane_det_location_plant(speed=0.10, targets=tar_list, side=side, dis_out=1.8)

# 多目标定位（汉诺塔大/中/小圆柱）
pose_dict = my_car.lane_det_location_v8_multi(0.15, pts, side=-1, dis_out=1.9)

# 单目标检测定位（方向牌文字）
flag, offset = my_car.lane_det_location_v4(0.2, tar, side=-1, dis_out=1)

# 垂直方向定位（食材抓取）
flag, offset = my_car.lane_det_location_vert(0.15, tar, side=-1, dis_out=1.2)

# OCR 文字识别
text = my_car.get_ocr_list()[0]
text = my_car.get_ocr_list_plus()[0]

# 前方目标检测
dets_ret = my_car.task_det(img_side)

# 方向牌识别
side = my_car.get_card_side()  # -1=左, 1=右
```

---

## 第 4 层：任务执行层

将基础能力（运动 + 感知）组合为可复用的任务原语。

### 4a. MyCar — 车辆控制原语

`MyCar` 继承自 `CarBase`，是系统的**中央调度器**（约 1438 行），封装了所有高层控制方法。

**初始化顺序（`MyCar.__init__`）：**

```
1. CarBase.__init__()     → 底盘运动学、电机
2. MyTask.__init__()     → 机械臂、弹射装置、蜂鸣器、舵机
3. ScreenShow()          → 显示屏
4. 加载 config_car.yml   → PID 参数、IO 引脚、摄像头索引
5. Key4Btn, LedLight     → 按键、LED
6. Infrared ×2           → 左右红外传感器
7. PidCal2 ×2            → 车道跟随 PID、检测定位 PID
8. Camera ×2             → 前方/侧面摄像头
9. ClintInterface ×4     → 4 个推理客户端
10. 按键监控线程启动
```

**核心控制方法（qqq.py 中调用频率）：**

| 方法 | 功能 | 调用次数 |
|------|------|:---:|
| `set_pose_offset([x,y,θ], t)` | 相对位姿移动 | 80+ |
| `lane_dis_offset(v, dis)` | PID 车道线巡航前进 | 20+ |
| `lane_sensor(v, value_h, sides)` | 红外传感器触发导航 | 15+ |
| `set_vel_time(vx,vy,vω, t)` | 速度-时间控制 | 10+ |
| `lane_det_location_plant(...)` | 车道+检测定位（植物） | 1 |
| `lane_det_location_v8_multi(...)` | 多目标定位（汉诺塔） | 1 |
| `lane_det_location_v4(...)` | 单目标检测定位 | 3 |
| `lane_det_location_vert(...)` | 目标检测+垂直位置 | 2 |
| `get_ocr_list()` / `get_ocr_list_plus()` | OCR 文字识别 | 4 |
| `get_card_side()` | 方向牌识别 | 2 |
| `set_pose(pose, t)` | 绝对位姿移动 | 2 |
| `calculation_dis(p1, p2)` | 两点距离计算 | 2 |

### 4b. MyTask — 机械臂与装置原语

`MyTask`（`task_func.py`）聚合了所有非底盘的执行装置：

```python
class MyTask:
    arm: ArmBase           # 机械臂（步进电机 ×2 + 舵机 ×2 + 真空泵）
    ejection: Ejection     # 弹射装置（推杆电机 + 转盘步进电机 + 电磁铁）
    servo_weather: ServoBus # 天气指示舵机
    ring: Beep             # 蜂鸣器
```

**机械臂控制链（`ArmBase`）：**

| 方法 | 功能 | 参数说明 |
|------|------|----------|
| `arm.set(x, z)` | 末端 XY 直角坐标定位 | x=前后伸缩, z=上下高度 |
| `arm.set_hand_angle(θ)` | 手爪俯仰角 | 60°=朝下, -80°=朝前 |
| `arm.set_arm_angle(θ)` | 大臂旋转角 | -115°=内弯, -96°=外展 |
| `arm.switch_side(±1)` | 左右侧切换 | 1=左侧, -1=右侧 |
| `arm.grap(1/0)` | 真空泵吸/松 | 1=吸, 0=松 |

**典型抓取-放置序列：**

```python
my_car.task.arm.switch_side(1)       # 1. 臂转到左侧
my_car.task.arm.set(0.13, 0)         # 2. 升到识别高度
my_car.task.arm.set_hand_angle(60)   # 3. 手爪朝下
my_car.task.arm.grap(1)              # 4. 吸取物体
my_car.task.arm.set(0.26, 0.07)      # 5. 抬起到运输高度
my_car.task.arm.switch_side(-1)      # 6. 转到右侧
my_car.task.arm.set(0.26, 0)         # 7. 下降到放置高度
my_car.task.arm.grap(0)              # 8. 释放
```

**弹射装置（`Ejection`）：**

```python
my_car.task.eject(n)  # n=1,2,3 对应不同弹射挡位
```

内部执行序列：`推杆复位 → 电磁铁吸合 → 推杆后拉上膛 → 转盘旋转到位 → 推杆弹射`

---

## 第 5 层：应用脚本层（qqq.py 任务函数）

将任务原语编排为**完整的竞赛任务流程**。每个函数 = 一个子任务的状态机，阻塞式顺序执行。

### 5.1 汉诺塔 — `hanoi()`（行 159–262）

```
手臂初始化(手爪60°)
  → 前进 0.7m 到方向牌
  → 识别方向(card_side)
  → 分支:
    side=-1(左): 转-45° → 前进 → 转+18° → ... → 到达汉诺塔区
    side=+1(右): 转+45° → 前进 → 微调 → 到达汉诺塔区
  → lane_det_location_v8_multi 定位大/中/小圆柱
  → 循环 3 次:
      移动到圆柱位置(set_pose)
      抓取圆盘(pick_up_cylinder)
      移动到放置区
      放下圆盘(put_down_cylinder)
```

### 5.2 BMI 体质分析 — `bmi()`（行 265–367）

```
手臂转左、升到识别高度
  → 车道线巡航(lane_dis_offset)
  → 红外传感器定位推杆位置(lane_sensor)
  → 横移推杆 → 回位
  → 前进到文字识别位置
  → OCR 识别身高体重
  → 调用 ErnieBot.ask3 计算 BMI 等级
  → 舵机指示 BMI 结果(bmi_set)
```

### 5.3 食材抓取 — `get_food()`（行 577–785）

```
手臂向右、定位文字区域
  → OCR 识别 → ErnieBot.ask1 解析第一个食物名
  → lane_det_location_vert 垂直定位食材位置
  → 根据食材高度 y 值选择抓取策略:
    y≤0(高位): 吸取 → 上抬 → 缩回 → 翻转手腕 → 放入篮子
    y>0(低位): 吸取 → 下降 → 前伸 → 缩回 → 放入篮子
  → 手臂转左、重复流程获取第二个食物
  → 返回 [food1, food2]
```

### 5.4 种植任务 — `plants(side)`（行 54–89）

```
planting(side, arm_set=True) 获取目标列表
  → lane_det_location_plant 定位目标
  → 根据目标 ID 判断圆柱类型:
    ID=100(cylinder3) → radius=2 → 浇水
    ID=80 (cylinder2) → radius=1 → 蜂鸣示意
    ID=60 (cylinder1) → radius=0 → 补光
  → planting(side, radius=radius) 执行操作
```

### 5.5 天气识别 — `weather_action()`（行 97–156）

```
手臂初始化 → 前进到方向牌 → 识别方向
  → 转弯进入天气区
  → Weather API 获取东莞天气
  → weather_set 设置指示舵机
  → 返回 side 值
```

### 5.6 食材放置 — `put_food(food1, food2)`（行 936–1092）

```
红外导航到放置区
  → 在左右两列分别 OCR 识别菜品描述文字
  → ErnieBot.ask2 匹配食物与描述 → 确定放置位置(1-4)
  → 根据答案选择策略:
    1或3(上栏): 吸 → 抬手 → 伸臂 → 松开 → 缩回
    2或4(下栏): 吸 → 压手 → 伸臂 → 松开 → 缩回
  → 近/远位置微调(back偏移量不同)
```

### 5.7 答题操作 — `answer1-4()` / `answer_real()`（行 787–922）

```
车道线巡航到答题区
  → 红外定位选项位置
  → 前进不同距离到达对应选项:
    answer1: 0.15m, answer2: 0.235m, answer3: 0.32m, answer4: 0.43m
  → 吸取 → 伸臂推倒答案方块
```

### 5.8 辅助任务函数

| 函数 | 行 | 功能概述 |
|------|:---:|------|
| `plant_action1/2/3(side)` | 91–517 | 种植任务的三种入口变体 |
| `camp()` | 458–500 | 营地任务：弧线运动到目标点 |
| `magic()` | 521–544 | 魔法任务：抓取道具并转移 |
| `eject1()` | 547–574 | 弹射装置 1：巡航 → 对正 → 弹射 |
| `eject2()` | 925–932 | 弹射装置 2 |
| `medicine()` | 1132–1148 | 药品任务：横向移动后旋转弹射 |
| `old_people()` | 1153–1176 | 敬老任务：推杆前后左右操作 |
| `answer_real()` | 878–922 | AI 答题：OCR 识别题目 → LLM 推理 → 推倒对应选项 |

---

## 第 6 层：调度层

通过 4 个预设路径 `push_A`、`push_b`、`push_C`、`push_D` 组合子任务，对应竞赛的 A/B/C/D 四个赛道。还有一个 `all_all_all()` 作为随机组合路径。

### 任务组合矩阵

```
任务          push_A  push_b  push_C  push_D
──────────    ──────  ──────  ──────  ──────
hanoi           ✔               ✔       ✔
weather_action  ✔       ✔
bmi                                     ✔       ✔
camp            ✔       ✔       ✔       ✔
plant_action              ✔
magic                           ✔       ✔
plant_action3             ✔
eject1          ✔       ✔       ✔       ✔
get_food        ✔       ✔       ✔       ✔
answer1         ✔
answer2                 ✔
answer3                         ✔
answer4                                 ✔
eject2          ✔               ✔       ✔
put_food        ✔       ✔       ✔       ✔
medicine                ✔
old_people      ✔       ✔       ✔       ✔
```

### 主入口

```python
if __name__ == "__main__":
    my_car = MyCar()           # 实例化全部硬件栈（6层级联初始化）
    my_car.STOP_PARAM = False  # 清除急停标志
    my_car.beep()              # 启动提示音
    push_b()                   # 执行 B 赛道
```

### 选择机制

被注释的代码（行 1315–1318）展示了原本的交互式选择逻辑：

```python
# functions = [push_A, push_b, push_C, push_D, all_all_all]
# my_car.manage(functions, 5)  # 通过按键从 5 个路径中选择
```

当前版本直接硬编码执行 `push_b()`。

---

## 完整控制流总览

```
调度层  push_A / push_b / push_C / push_D
  │
  ├─► 应用任务函数 (hanoi, bmi, get_food, plants, ...)
  │     │
  │     ├─► MyCar (car_wrap.py) ─── 车辆控制原语
  │     │     │
  │     │     ├─► CarBase ─── 运动学层
  │     │     │     ├─► OdometryBase ─── 里程计
  │     │     │     └─► Motors ─── 电机驱动
  │     │     │           └─► Serial (CH340) ─── MC601/MC602 单片机
  │     │     │
  │     │     ├─► Camera + ClintInterface ─── 感知层
  │     │     │     └─► ZMQ ─── InferServer (PaddlePaddle)
  │     │     │
  │     │     └─► PID (simple_pid) ─── 车道跟随 / 检测定位
  │     │
  │     ├─► MyTask (task_func.py) ─── 装置控制原语
  │     │     │
  │     │     ├─► ArmBase ─── 机械臂
  │     │     │     ├─► StepperWrap ×2 ─── 大臂/小臂步进电机
  │     │     │     ├─► ServoBus ─── 手爪俯仰/旋转舵机
  │     │     │     └─► PoutD ─── 真空泵
  │     │     │
  │     │     ├─► Ejection ─── 弹射装置
  │     │     │     ├─► MotorWrap ─── 推杆电机
  │     │     │     ├─► StepperWrap ─── 转盘步进电机
  │     │     │     └─► PoutD ─── 电磁铁
  │     │     │
  │     │     └─► ServoBus ─── 天气/BMI 指示舵机
  │     │
  │     └─► ErnieBot (ernie_bot/) ─── LLM 自然语言理解
  │           └─► HTTP API ─── 文心一言
  │
  └─► 每个任务结束后返回调度层，继续执行下一个任务
```

### 关键设计特征

1. **同步阻塞式编排**：所有任务函数都是同步的，前一步完成才进入下一步，没有使用 ROS 或异步框架
2. **硬编码参数**：位移量、速度、超时时间等参数直接写死在代码中（如 `0.18`, `0.3`, `1.5`），而非从配置文件读取
3. **多版本函数共存**：`bmi()` 有 2 个版本、`old_people()` 有新旧版本，旧版本被注释保留
4. **容错设计**：关键操作（OCR、LLM 调用）包裹在 `try/except` 中，失败时使用硬编码默认值
5. **LLM 集成**：5 个 AI 问答接口覆盖食物识别(`ask1`)、菜品匹配(`ask2`)、BMI 计算(`ask3`)、选择题(`ask4`)
6. **全局实例**：`my_car` 作为模块级全局变量，所有函数直接访问
