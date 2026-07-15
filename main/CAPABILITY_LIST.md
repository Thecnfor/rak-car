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
- `car.set_chassis_velocity`
  - 原始底盘速度控制，适合高频上层闭环
- `car.get_lane_results`
  - 读取前摄巡线误差和角度误差
- `POST /v1/realtime/wheels/speeds`
  - 4 轮线速度直达，绕过 set_chassis_velocity 的里程计耦合路径（`_realtime_gate` 微秒级瞬持锁，**不被 arm 长动作挡住**——是「巡线 + 机械臂」并发的关键）
- `GET /v1/realtime/wheels/encoders`
  - 4 轮编码器弧度累计值
- `POST /v1/realtime/motor/speed`
  - 单电机原始速度（开环，不进 PID）
- `GET /v1/realtime/encoder?port=N`
  - 单电机编码器原始累计值
- `POST /v1/realtime/stepper/rad`
  - 底盘步进电机弧度定位

### 1.2 底层已具备，但当前未直接暴露为 API

- 四轮角速度（rad/s）直接下发（线速度已通过 `/v1/realtime/wheels/speeds` 暴露）
- 四轮虚拟速度（PWM [-100,100]，mc602 `Motor4_2.set_speed`）—— 业务侧一般无需直发原始 PWM

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
- `arm.y_get_position`
  - 读取竖轴当前位置
- `arm.goto_position`
  - 直接按 `x / y` 移动到目标点
- `arm.go_for`
  - 按相对偏移移动机械臂
- `arm.x_speed`
  - 横轴原始速度控制
- `arm.y_speed`
  - 竖轴原始速度控制

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

### 2.3 已 API 暴露的机械臂状态

- `car.get_arm_state`
  - 返回 `x / y / side / arm_angle / hand_angle / y_limit`

## 3. 枪

### 3.1 已 API 暴露

- `car.shooting`
  - 单次射击触发
  - 当前逻辑是固定脉冲：先拉低，再高电平触发，再强制拉低收尾
- `car.set_shoot_state`
  - 原始枪口数字输出控制

### 3.2 底层实现

- 使用数字输出口 `PoutD(4)`
- 本质是继电器/数字口触发型枪口

### 3.3 已扩展的原始控制

- `car.set_digital_output`
  - 任意数字输出口 set/reset

## 4. PWM 舵机

### 4.1 已 API 暴露

- `car.set_storage`
  - 控储物仓 PWM 舵机开合
- `car.set_storage_angle`
  - 直接设置储物仓 PWM 角度
- `arm.set_hand_angle`
  - 控机械臂末端 PWM 舵机角度
- `car.set_pwm_servo_angle`
  - 任意端口 PWM 舵机通用控制
- `POST /v1/realtime/bus-servo/angle`
  - 任意端口总线舵机（ServoBus）角度下发，绕过 `arm.set_arm_angle` 的业务封装
- `GET /v1/realtime/bus-servo/angle?port=N`
  - 读取总线舵机当前角度（mc602 实现，mc601 暂不支持）

### 4.2 已在代码中使用，但以业务动作形式暴露

- 机械臂大臂角度 `arm.set_arm_angle`
  - 这是总线舵机，不是普通 PWM
- 机械臂组合姿态 `arm.set_arm_pose`
  - 会同时联动舵机和电机

### 4.3 底层仍未直接暴露为 API

- 原始 `ServoBus(port).set_speed(speed)`（速度模式，mc602 协议层支持；本轮只接了位置模式）

## 5. 其他可检测/可交互能力

### 5.1 已 API 暴露

- `car.get_detection_results`
  - 侧摄目标检测结果
- `car.get_ocr`
  - OCR 识别结果
- `car.get_det_ocr`
  - 对指定检测框做 OCR
- `car.get_odometry`
  - 底盘位姿
- `car.get_distance`
  - 累计距离
- `car.beep`
  - 蜂鸣器
- `car.get_ir_distance`
  - 读取单侧 IR
- `car.get_all_ir_distance`
  - 同时读取左右 IR
- `car.get_bluetooth_pad`
  - 读取蓝牙手柄
- `car.get_key_event`
  - 读取物理按键事件
- `car.get_key_state`
  - 读取物理按键当前状态
- `car.get_battery_voltage`
  - 读取电池电压
- `car.set_light_color`
  - 控制灯带颜色
- `car.show_text`
  - 屏幕显示文本
- `GET /v1/realtime/analog?port=N`
  - 单路模拟量（mc602 走 `Sensor_Analog2_2`，对应第二路 `dev_id=0x08`）
- `GET /v1/realtime/analog2?port=N`
  - 第二路模拟量（mc602 走 `AnalogInput`，对应 `dev_id=0x07 mode=0`）

### 5.2 底层已具备，但当前未直接暴露为 API

- `BoardKey`
  - 板载按键读取
- `sensor_touch`
  - 触碰类输入，协议层已有
- `sensor_ultrasonic`
  - 超声类输入，协议层已有
- `sensor_ambient_light`
  - 环境光输入，协议层已有

### 5.3 可控输出，但当前未直接暴露为 API

- `NixieTube`
  - 数码管

## 6. 八个任务

这 8 个比赛任务已经可以直接走 `target=task` 调用：

- `auto_lane_tracing`
- `auto_seeding`
- `target_shooting_detection`
- `water_tower_task`
- `target_shooting`
- `crop_harvesting`
- `sort_and_store`
- `get_order`
- `order_delivery`

其中两个任务会返回可继续编排的数据：

- `target_shooting_detection`
  - 返回 `animal_list`
- `get_order`
  - 返回 `order_list`

这意味着上层业务已经可以这样编排：

- 先调 `task.target_shooting_detection`
- 再把返回值传给 `task.target_shooting`
- 先调 `task.get_order`
- 再把返回值传给 `task.order_delivery`

## 7. WebSocket

现在除了 HTTP，还支持 WebSocket 长连接：

- `ws://192.168.6.231:5050/v1/ws`

适合：

- 高频底盘速度控制
- 长连接状态轮询
- 远端业务编排

## 8. 直接给我提需求时怎么说

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

## 9. 现在最重要的一句

这台车当前已经稳定具备：

- 底盘麦轮控制
- 机械臂 `y / x / 吸盘 / 舵机`
- 枪
- 储物仓 PWM
- 蜂鸣器
- IR / 电池 / 手柄 / 按键 / 灯 / 屏幕
- 位姿/距离
- 检测/OCR
- WebSocket 长连接控制
- 八任务 API 编排

这几个已经足够支撑大部分真实业务开发。
