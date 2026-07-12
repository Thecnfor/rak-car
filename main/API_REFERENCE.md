# main 接口速查手册

这份文档按“能直接抄请求体”的方式写，面向真实业务开发。

如果你想先看完整能力边界，不想每次重新口头描述，直接看 [CAPABILITY_LIST.md](file:///home/jetson/workspace/rak-car/main/CAPABILITY_LIST.md)。

## 1. 先记住这一个接口

推荐统一使用：

- `POST /v1/execute`

请求体固定长这样：

```json
{
  "target": "car",
  "name": "beep",
  "args": [],
  "kwargs": {},
  "timeout": 40
}
```

字段含义：

- `target`: `car` / `arm` / `task`
- `name`: 动作名
- `args`: 位置参数列表
- `kwargs`: 关键字参数
- `timeout`: 本次动作超时秒数

特点：

- 一次 `POST` 直接拿最终结果
- 服务端自动等待初始化
- 服务端自动做控制器恢复

## 2. Python 最小用法

最简单的代码只需要 [api_client.py](file:///home/jetson/workspace/rak-car/main/api_client.py)：

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
print(client.call("car", "beep", timeout=40))
```

也可以显式写：

```python
client.execute("car", "beep", timeout=40)
```

## 3. 底盘坐标和单位

先统一几个概念，不然后面参数容易写反：

- `x`: 前后方向，`+` 前进，`-` 后退
- `y`: 左右横移，`+` 向左，`-` 向右
- `theta` / `z`: 旋转角，单位是弧度，`+` 逆时针，`-` 顺时针
- 底盘位置单位：
  - `move_for` / `move_to_position` 用米和弧度
  - `move_time` 用速度向量和秒
  - `lane_*` 的 `speed` 用前进速度，单位近似米每秒

## 4. 底盘直控速查

这类接口是“纯底盘控制”，不依赖巡线。

### 4.1 `beep`

- `target`: `car`
- `name`: `beep`
- 参数：无
- 用途：蜂鸣一声

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"car","name":"beep","timeout":20}'
```

### 4.2 `shooting`

- `target`: `car`
- `name`: `shooting`
- 参数：无
- 用途：继电器枪单次触发
- 当前逻辑：先拉低，再高电平脉冲，再强制拉低

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"car","name":"shooting","timeout":50}'
```

### 4.3 `move_for`

- `target`: `car`
- `name`: `move_for`
- 语义：相对当前姿态移动
- 适合：
  - 纯前进
  - 纯横移
  - 原地转角
  - 麦轮组合位移

参数：

- `args[0] = [x, y, theta]`
- 可选 `kwargs.duration`
- 可选 `kwargs.max_velocities = [vx_max, vy_max, wz_max]`
- 可选 `kwargs.tolerance = [x_tol, y_tol, theta_tol]`

例子 1，纯前进 5cm：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_for",
    "args":[[0.05, 0.0, 0.0]],
    "timeout":60
  }'
```

例子 2，纯横移到左边 3cm：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_for",
    "args":[[0.0, 0.03, 0.0]],
    "timeout":60
  }'
```

例子 3，原地左转约 90 度：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_for",
    "args":[[0.0, 0.0, 1.5708]],
    "timeout":60
  }'
```

例子 4，麦轮斜着走：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_for",
    "args":[[0.10, 0.05, 0.0]],
    "timeout":80
  }'
```

说明：

- 这是最推荐的“纯纯往前走/横移/旋转”接口
- 它走的是里程计闭环，不是视觉巡线

### 4.4 `move_to_position`

- `target`: `car`
- `name`: `move_to_position`
- 语义：移动到绝对位姿

参数：

- `args[0] = [x, y, theta]`
- 可选 `kwargs.duration`
- 可选 `kwargs.max_velocities`
- 可选 `kwargs.tolerance`
- 可选 `kwargs.timeout`

例子，回到原点：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_to_position",
    "args":[[0.0, 0.0, 0.0]],
    "timeout":80
  }'
```

适合：

- 已记录过里程计位置
- 想回某个点位
- 不想走巡线

### 4.5 `move_time`

- `target`: `car`
- `name`: `move_time`
- 语义：按速度跑固定时间

参数：

- `args[0] = [vx, vy, wz]`
- 可选 `kwargs.dur_time`
- 可选 `kwargs.stop`

例子，前进 1 秒：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_time",
    "args":[[0.10, 0.0, 0.0]],
    "kwargs":{"dur_time":1.0},
    "timeout":20
  }'
```

适合：

- 临时联调
- 粗调速度
- 不适合要求高精度的位置控制

### 4.6 `move_distance`

- `target`: `car`
- `name`: `move_distance`
- 语义：按给定速度，走到累计距离达到阈值

参数：

- `args[0] = [vx, vy, wz]`
- 可选 `kwargs.dis`
- 可选 `kwargs.stop`

例子，前进累计 10cm：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_distance",
    "args":[[0.10, 0.0, 0.0]],
    "kwargs":{"dis":0.10},
    "timeout":30
  }'
```

说明：

- 它还是“按速度跑”，只是结束条件换成累计距离
- 精度和鲁棒性一般不如 `move_for`

## 5. 智能导航速查

这类接口是“用前摄巡线”的智能导航。

### 5.1 `lane_time`

- `target`: `car`
- `name`: `lane_time`
- 语义：跟着车道线跑固定时间

参数：

- `kwargs.speed`
- `kwargs.time_dur`
- 可选 `kwargs.stop`

例子：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"lane_time",
    "kwargs":{"speed":0.3,"time_dur":1.5},
    "timeout":40
  }'
```

### 5.2 `lane_dis`

- `target`: `car`
- `name`: `lane_dis`
- 语义：巡线直到累计距离超过目标值

参数：

- `kwargs.speed`
- `kwargs.dis_end`

例子：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"lane_dis",
    "kwargs":{"speed":0.3,"dis_end":1.0},
    "timeout":80
  }'
```

### 5.3 `lane_dis_offset`

- `target`: `car`
- `name`: `lane_dis_offset`
- 语义：从当前累计距离继续巡线一段增量
- 这是最常用的“智能导航往前走”接口

参数：

- `kwargs.speed`
- `kwargs.dis_hold`

例子，巡线前进 20cm：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"lane_dis_offset",
    "kwargs":{"speed":0.3,"dis_hold":0.2},
    "timeout":80
  }'
```

怎么选：

- 想“纯纯前进/横移/旋转”：用 `move_for`
- 想“沿赛道自动往前走”：用 `lane_dis_offset`

## 6. 视觉对齐与识别速查

### 6.1 `get_detection_results`

- `target`: `car`
- `name`: `get_detection_results`
- 语义：获取侧摄目标检测结果

参数：

- 可选 `kwargs.sort_pos = [x_ref, y_ref]`
- 可选 `kwargs.limit_x`
- 可选 `kwargs.limit_y`

返回每项格式：

- `[cls_id, det_id, label, score, x_c, y_c, w, h]`

例子：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"get_detection_results",
    "kwargs":{"sort_pos":[0,0],"limit_x":0.5,"limit_y":1.0},
    "timeout":20
  }'
```

### 6.2 `move_to_detection_target`

- `target`: `car`
- `name`: `move_to_detection_target`
- 语义：根据侧摄识别结果对齐到目标

参数：

- 可选 `kwargs.delta_x`
- 可选 `kwargs.delta_y`
- 可选 `kwargs.label`
- 可选 `kwargs.time_out`
- 可选 `kwargs.sort_pos`
- 可选 `kwargs.num`

常用场景：

- `delta_y=None`: 只做横向/前向视觉对准，不强制某个 y 偏差
- `label="order"`: 只盯某类标签

例子，对齐最近目标：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_to_detection_target",
    "kwargs":{"delta_x":0.0,"delta_y":null,"time_out":3.0},
    "timeout":50
  }'
```

### 6.3 `adjust_arm_position`

- `target`: `car`
- `name`: `adjust_arm_position`
- 语义：根据机械臂当前朝向，微调 X 轴位置

参数：

- 可选 `kwargs.dis`，默认 `0.05`

例子：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"adjust_arm_position",
    "kwargs":{"dis":0.03},
    "timeout":20
  }'
```

### 6.4 `get_ocr`

- `target`: `car`
- `name`: `get_ocr`
- 语义：侧摄检测文字区域并做 OCR

参数：

- 可选 `kwargs.label`
- 可选 `kwargs.time_out`

例子：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"get_ocr",
    "kwargs":{"label":"order","time_out":3.0},
    "timeout":20
  }'
```

## 7. 机械臂速查

机械臂位置单位统一按米来写。

### 7.1 `reset_position`

- `target`: `arm`
- `name`: `reset_position`
- 语义：整体复位
- 动作内容：
  - 手爪向上
  - 大臂向右
  - X/Y 回零

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"reset_position","timeout":40}'
```

### 7.2 `reset_x`

- `target`: `arm`
- `name`: `reset_x`
- 语义：只复位 X 轴

### 7.3 `move_x_position`

- `target`: `arm`
- `name`: `move_x_position`
- 语义：把机械臂水平轴移到目标位置

参数：

- `args[0] = target`
- 可选 `kwargs.out_time`

例子，X 轴移动到 `0.20m`：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"arm",
    "name":"move_x_position",
    "args":[0.20],
    "kwargs":{"out_time":6.0},
    "timeout":20
  }'
```

### 7.4 `move_y_position`

- `target`: `arm`
- `name`: `move_y_position`
- 语义：把机械臂竖直轴移到目标高度

参数：

- `args[0] = target`

例子，Y 轴移动到 `0.10m`：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"arm",
    "name":"move_y_position",
    "args":[0.10],
    "timeout":20
  }'
```

### 7.5 `set_arm_angle`

- `target`: `arm`
- `name`: `set_arm_angle`
- 语义：大臂总成角度

参数：

- `kwargs.angle`: `"LEFT"` / `"MID"` / `"RIGHT"` 或数字角度
- 可选 `kwargs.speed`

例子 1，用枚举：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"arm",
    "name":"set_arm_angle",
    "kwargs":{"angle":"LEFT","speed":80},
    "timeout":20
  }'
```

例子 2，直接给角度：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"arm",
    "name":"set_arm_angle",
    "kwargs":{"angle":94,"speed":80},
    "timeout":20
  }'
```

### 7.6 `set_hand_angle`

- `target`: `arm`
- `name`: `set_hand_angle`
- 语义：手爪舵机角度

参数：

- `kwargs.angle`: `"UP"` / `"MID"` / `"DOWN"` 或数字角度
- 可选 `kwargs.speed`

例子：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"arm",
    "name":"set_hand_angle",
    "kwargs":{"angle":"DOWN","speed":80},
    "timeout":20
  }'
```

### 7.7 `set_arm_pose`

- `target`: `arm`
- `name`: `set_arm_pose`
- 语义：一次同时设位置和姿态

参数：

- 可选 `kwargs.x`
- 可选 `kwargs.y`
- 可选 `kwargs.arm`
- 可选 `kwargs.hand`

例子 1，完整设置：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"arm",
    "name":"set_arm_pose",
    "kwargs":{"x":0.05,"y":0.05,"arm":"LEFT","hand":"UP"},
    "timeout":30
  }'
```

例子 2，只改姿态不改 X/Y：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"arm",
    "name":"set_arm_pose",
    "kwargs":{"arm":"RIGHT","hand":"DOWN"},
    "timeout":30
  }'
```

### 7.8 `grasp`

- `target`: `arm`
- `name`: `grasp`
- 语义：气泵抓取/释放

参数：

- `args[0] = true` 抓取
- `args[0] = false` 释放

例子，抓取：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"arm",
    "name":"grasp",
    "args":[true],
    "timeout":20
  }'
```

### 7.9 `x_get_position`

- `target`: `arm`
- `name`: `x_get_position`
- 语义：读取当前 X 轴位置

## 8. 系统接口速查

### 8.1 `GET /v1/health`

用途：

- 看服务是否在线
- 看是否初始化完成
- 看控制器探测状态

关键字段：

- `state.initialized`
- `state.initializing`
- `state.last_error`
- `state.controller_probe.ready`
- `state.controller_probe.detail`

### 8.2 `GET /v1/actions`

用途：

- 查看当前支持哪些动作

### 8.3 `GET /v1/runtime`

用途：

- 看里程计、累计距离、stop 标记

### 8.4 `POST /v1/control/emergency-stop`

用途：

- 立即停车

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/control/emergency-stop
```

### 8.5 `POST /v1/control/close`

用途：

- 关闭当前 runtime 内部实例
- 下次动作会重新自动初始化

### 8.6 `POST /v1/control/init`

用途：

- 手动初始化

例子：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/control/init \
  -H 'Content-Type: application/json' \
  -d '{"force":true,"reset_arm":false,"reset_position":true}'
```

## 9. 任务接口速查

这类是完整业务任务，不是单步控制。

常见任务名：

- `auto_lane_tracing`
- `auto_seeding`
- `target_shooting_detection`
- `water_tower_task`
- `target_shooting`
- `crop_harvesting`
- `sort_and_store`
- `get_order`
- `order_delivery`

例子：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"task",
    "name":"auto_lane_tracing",
    "kwargs":{"speed":0.3,"dis_hold":0.2},
    "timeout":120
  }'
```

说明：

- 单步业务开发优先用 `car` / `arm`
- 成熟流程再收敛成 `task`

### 9.1 两个可继续编排的任务返回值

- `target_shooting_detection`
  - 返回 `animal_list`
- `get_order`
  - 返回 `order_list`

上层业务可以这样串：

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()

animal_job = client.execute("task", "target_shooting_detection", timeout=240)
animal_list = animal_job["result"]
client.execute("task", "target_shooting", kwargs={"animal_list": animal_list}, timeout=240)

order_job = client.execute("task", "get_order", timeout=300)
order_list = order_job["result"]
client.execute("task", "order_delivery", kwargs={"order_list": order_list}, timeout=300)
```

### 9.2 原始控制和传感接口

现在已经支持这些更底层的业务接口：

- `car.set_chassis_velocity`
- `car.get_lane_results`
- `car.get_key_event`
- `car.get_key_state`
- `car.get_bluetooth_pad`
- `car.get_battery_voltage`
- `car.get_ir_distance`
- `car.get_all_ir_distance`
- `car.set_light_color`
- `car.show_text`
- `car.set_storage_angle`
- `car.set_pwm_servo_angle`
- `car.set_digital_output`
- `car.set_shoot_state`
- `car.get_arm_state`
- `arm.goto_position`
- `arm.go_for`
- `arm.x_speed`
- `arm.y_speed`
- `arm.y_get_position`

例子，原始底盘速度控制：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"set_chassis_velocity",
    "kwargs":{"x":0.10,"y":0.00,"z":0.00,"duration":0.20},
    "timeout":20
  }'
```

例子，读取左右 IR：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"get_all_ir_distance",
    "timeout":20
  }'
```

例子，控制任意数字口：

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"set_digital_output",
    "kwargs":{"port":4,"value":true},
    "timeout":20
  }'
```

### 9.3 WebSocket 长连接

如果上层要做高频控制、持续轮询或长连接编排，直接用：

- `ws://192.168.3.60:5050/v1/ws`

连接成功后，发送 JSON 消息即可。

最常用 3 类消息：

```json
{"op":"health","request_id":"h1"}
{"op":"execute","request_id":"e1","target":"car","name":"beep","timeout":20}
{"op":"execute","request_id":"e2","target":"car","name":"set_chassis_velocity","kwargs":{"x":0.1,"y":0.0,"z":0.0}}
```

WebSocket 支持的 `op`：

- `ping`
- `health`
- `runtime`
- `actions`
- `config`
- `execute`
- `create_job`
- `get_job`
- `init`
- `stop_mode`
- `reset_stop`
- `close`
- `emergency_stop`

## 10. 推荐写法

### 10.1 最推荐的 Python 写法

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()

client.call("car", "beep", timeout=40)
client.call("car", "move_for", [0.05, 0.0, 0.0], timeout=60)
client.call("arm", "move_x_position", 0.20, timeout=20)
client.call("arm", "set_arm_pose", timeout=30, x=0.05, y=0.05, arm="LEFT", hand="UP")
```

### 10.2 最推荐的 curl 写法

```bash
curl -sS -X POST http://192.168.3.60:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{
    "target":"car",
    "name":"move_for",
    "args":[[0.05, 0.0, 0.0]],
    "timeout":60
  }'
```

## 11. 选型建议

- 想直走 5cm、横移 3cm、转 90 度：`move_for`
- 想回到某个已记录的位置：`move_to_position`
- 想粗略跑一会儿：`move_time`
- 想跟着赛道自动走：`lane_dis_offset`
- 想靠视觉贴近目标：`move_to_detection_target`
- 想单独调机械臂某个电机：`move_x_position` / `move_y_position`
- 想一次设机械臂整体姿态：`set_arm_pose`

## 12. 一句话结论

现在 `main/` 业务层可以按最简单的方式使用：

- 配置在 [settings.py](file:///home/jetson/workspace/rak-car/main/settings.py)
- 调用在 [api_client.py](file:///home/jetson/workspace/rak-car/main/api_client.py)
- 详细参数直接抄这份文档里的请求体即可
