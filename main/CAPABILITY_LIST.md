# 功能清单

这份清单只回答一件事：

- 这台车现在到底有哪些能力
- 哪些已经能直接通过 runtime API 调
- 哪些底层有能力，但目前还没注册到 API

判断标准：

- “已 API 暴露” 以 [actions.py](file:///home/jetson/workspace/rak-car/runtime/core/actions.py) 为准
- “底层已具备” 以 `car_wrap_2026.py`、`mecanum.py`、`arm_base.py`、`controller_wrap.py`、`mc602_ctl2.py` 实际代码为准

## 1. 底盘

### 1.1 已 API 暴露

- `car.beep`
  - 蜂鸣一声
- `car.stop`
  - 立即停车
- `car.reset_position`
  - 重置底盘里程计原点
- `car.move_for`
  - 按相对位姿移动，支持前进、横移、旋转、麦轮斜走
- `car.move_to_position`
  - 按绝对位姿移动
- `car.move_time`
  - 按速度跑固定时间
- `car.move_distance`
  - 按速度跑到累计距离阈值
- `car.lane_time`
  - 前摄巡线跑固定时间
- `car.lane_dis`
  - 前摄巡线跑到目标距离
- `car.lane_dis_offset`
  - 前摄巡线跑到目标距离，并带偏移逻辑
- `car.move_to_detection_target`
  - 侧摄视觉对齐目标
- `car.adjust_arm_position`
  - 依据机械臂左右侧做末端微调
- `car.get_odometry`
  - 读取底盘当前位姿 `[x, y, theta]`
- `car.get_distance`
  - 读取累计行驶距离

### 1.2 底层已具备，但当前未直接暴露为 API

- 原始速度控制 `set_velocity(x, y, z)`
- 四轮线速度/角速度直接下发
- 单电机速度控制
- 编码器原始读数
- 步进电机控制

### 1.3 麦轮特色能力

- 纯前后运动
- 纯横移
- 原地旋转
- 前进和横移组合斜走
- 基于里程计的相对位姿闭环
- 基于里程计的绝对位姿闭环
- 基于前摄车道线的智能巡线
- 基于侧摄检测结果的视觉对齐

## 2. 机械臂

### 2.1 已 API 暴露

- `arm.reset_position`
  - 机械臂整体回零
- `arm.reset_x`
  - 机械臂横轴回零
- `arm.move_x_position`
  - 横轴移动到指定位置
- `arm.move_y_position`
  - 竖轴移动到指定位置
- `arm.set_arm_angle`
  - 设置大臂方向，常用 `LEFT` / `MID` / `RIGHT`
- `arm.set_hand_angle`
  - 设置手爪角度，常用 `UP` / `MID` / `DOWN`
- `arm.set_arm_pose`
  - 一次性设置 `x / y / arm / hand`
- `arm.grasp`
  - 吸盘吸取/释放
- `arm.x_get_position`
  - 读取横轴当前位置

### 2.2 机械臂组成

- `y` 轴
  - 步进电机，支持回零、定位、速度控制
- `x` 轴
  - 直流电机，支持回零、定位、速度控制
- 吸盘
  - 真空泵 + 破真空阀，统一由 `grasp(True/False)` 控制
- 大臂舵机
  - 总线舵机，负责左右侧方向
- 手爪舵机
  - PWM 舵机，负责上下角度

### 2.3 底层已具备，但当前未直接暴露为 API

- `goto_position(x, y)`
- `go_for(x_offset, y_offset)`
- 原始 `x_speed()` / `y_speed()`
- 机械臂当前 `y` 位置读取
- 当前 `side`、最近角度状态等内部状态

## 3. 枪

### 3.1 已 API 暴露

- `car.shooting`
  - 单次射击触发
  - 当前逻辑是固定脉冲：先拉低，再高电平触发，再强制拉低收尾

### 3.2 底层实现

- 使用数字输出口 `PoutD(4)`
- 本质是继电器/数字口触发型枪口

### 3.3 当前限制

- 现在只暴露“单次触发”
- 还没有开放“原始数字口 set/reset API”

## 4. PWM 舵机

### 4.1 已 API 暴露

- `car.set_storage`
  - 控储物仓 PWM 舵机开合
- `arm.set_hand_angle`
  - 控机械臂末端 PWM 舵机角度

### 4.2 已在代码中使用，但以业务动作形式暴露

- 机械臂大臂角度 `arm.set_arm_angle`
  - 这是总线舵机，不是普通 PWM
- 机械臂组合姿态 `arm.set_arm_pose`
  - 会同时联动舵机和电机

### 4.3 底层已具备，但当前未直接暴露为 API

- 原始 `ServoPwm(port).set_angle(angle)`
- 原始 `ServoBus(port).set_angle(angle, speed)`
- 任意端口 PWM 舵机通用控制

## 5. 其他可检测/可交互能力

### 5.1 已 API 暴露

- `car.get_detection_results`
  - 侧摄目标检测结果
- `car.get_ocr`
  - OCR 识别结果
- `car.get_odometry`
  - 底盘位姿
- `car.get_distance`
  - 累计距离
- `car.beep`
  - 蜂鸣器

### 5.2 底层已具备，但当前未直接暴露为 API

- `IR / 红外`
  - 有底层封装，但当前 `MyCar.sensor_init()` 没启用
- `BluetoothPad`
  - 蓝牙手柄可读摇杆和按键
- `Key4Btn`
  - 物理四键输入
- `Battry.read()`
  - 电池电压读取
- `BoardKey`
  - 板载按键读取
- `AnalogInput / AnalogInput2`
  - 模拟量读取
- `sensor_touch`
  - 触碰类输入，协议层已有
- `sensor_ultrasonic`
  - 超声类输入，协议层已有
- `sensor_ambient_light`
  - 环境光输入，协议层已有
- `get_lane_results()`
  - 前摄巡线结果，当前未注册到 runtime API

### 5.3 可控输出，但当前未直接暴露为 API

- `LedLight`
  - 灯带/灯光
- `NixieTube`
  - 数码管
- `ScreenShow`
  - 屏幕显示
- `PoutD`
  - 任意数字输出
- `ServoPwm`
  - 任意 PWM 舵机

## 6. 直接给我提需求时怎么说

以后如果你要提需求，建议直接按这张清单说：

- 底盘
  - 纯麦轮直控
  - 巡线导航
  - 视觉对齐
- 机械臂
  - `x`
  - `y`
  - 吸盘
  - 末端 PWM
  - 大臂角度
- 枪
  - 单发
  - 连发
  - 原始数字口控制
- PWM
  - 储物仓
  - 机械臂末端
  - 任意端口 PWM
- 检测/反馈
  - 位姿
  - 距离
  - OCR
  - 检测
  - IR
  - 电池
  - 蜂鸣器
  - 蓝牙手柄
  - 按键

## 7. 现在最重要的一句

这台车当前已经稳定具备：

- 底盘麦轮控制
- 机械臂 `y / x / 吸盘 / 舵机`
- 枪
- 储物仓 PWM
- 蜂鸣器
- 位姿/距离
- 检测/OCR

这几个已经足够支撑大部分真实业务开发。
