# main 业务 API 速查

面向 `main/` 业务层：只列可调接口、参数、返回，不讲驱动和实现细节。

## 1. 约定

### 1.1 三个 target

| target | 作用 | 入口 |
| --- | --- | --- |
| `car` | 底盘动作、传感器、视觉、外围（PWM/灯/屏/枪/数字口） | `POST /v1/execute` 或 WebSocket `execute` |
| `arm` | 机械臂位置、姿态、抓取 | 同上 |
| `task` | 已封装好的比赛级任务流程 | 同上 |

约束：

- `car` / `arm` / `task` 是接口的 `target` 名字，不是 Python 对象
- 业务代码不允许直接 `from ... import car` 或 `MyCar()`
- 业务代码不允许直接调 `car_wrap_2026.py` / `car_task_function.py` / `smartcar/` / `runtime/` 内部
- 一切控制统一走 `POST /v1/execute`

### 1.2 请求体

```json
{
  "target": "car",
  "name": "move_for",
  "args": [[0.05, 0.0, 0.0]],
  "kwargs": {},
  "timeout": 60
}
```

| 字段 | 说明 |
| --- | --- |
| `target` | `car` / `arm` / `task` |
| `name` | 动作名（见下文速查表） |
| `args` | 位置参数列表 |
| `kwargs` | 关键字参数 |
| `timeout` | 本次动作超时秒数 |

Python 侧推荐：

```python
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
client.wait_until_ready()

client.call("car", "beep", timeout=20)
client.call("car", "move_for", [0.05, 0.0, 0.0], timeout=60)
print(client.call("car", "get_odometry", timeout=20)["result"])
```

### 1.3 坐标约定

| 项 | 说明 |
| --- | --- |
| `x` | 前后，`+` 前进，`-` 后退 |
| `y` | 左右，`+` 左移，`-` 右移 |
| `theta` / `z` | 旋转，弧度，`+` 逆时针，`-` 顺时针 |

## 2. 状态与元信息

| 接口 | 用途 | 关键返回 |
| --- | --- | --- |
| `GET /v1/health` | 服务是否在线、是否初始化完成 | `state.initialized` `state.initializing` `state.last_error` |
| `GET /v1/runtime` | 当前运行时快照 | `runtime.odometry` `runtime.distance` `runtime.stop_after_action` `runtime.stop_flag` |
| `GET /v1/actions` | 当前支持哪些动作 | `actions.car` `actions.arm` `actions.task` `actions.system` |
| `GET /v1/config` | runtime 配置 | `config` |

```json
// GET /v1/runtime
{
  "ok": true,
  "runtime": {
    "odometry": [0.0, 0.0, 0.0],
    "distance": 0.0,
    "stop_after_action": false,
    "stop_flag": false
  }
}
```

## 3. 作业接口

| 接口 | 用途 |
| --- | --- |
| `POST /v1/execute` | 同步执行一个动作并等待结果（最常用） |
| `POST /v1/jobs` | 创建异步任务，返回 `job.id` |
| `GET /v1/jobs` | 查看任务列表 |
| `GET /v1/jobs/{job_id}` | 查看单个任务状态，看 `status/result/error` |

`POST /v1/execute` 返回结构：

```json
{
  "ok": true,
  "job": {
    "id": "xxxx",
    "target": "car",
    "name": "get_odometry",
    "status": "succeeded",
    "result": [0.0, 0.0, 0.0],
    "error": null
  }
}
```

## 4. 控制接口

| 接口 | 用途 | 关键参数 |
| --- | --- | --- |
| `POST /v1/control/init` | 手动初始化 runtime | `force` `reset_arm` `reset_position` |
| `POST /v1/control/stop-mode` | 设置动作后是否自动停 | `enabled` |
| `POST /v1/control/reset-stop` | 清掉 stop 标记 | 无 |
| `POST /v1/control/close` | 关闭当前 runtime 实例 | 无 |
| `POST /v1/control/emergency-stop` | 立即停车 | 无 |

## 5. 实时硬件控制（50Hz 直达路径）

不走 `/v1/execute`，不进 `job_queue`，在 `car_lock` 内同步执行，单次 RTT 毫秒级，适合 50Hz 闭环。
错误码：409（car 未初始化）/ 400（参数错）。

| 接口 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `POST /v1/realtime/wheels/speeds` | 4 轮线速度直达 | `speeds=[v1,v2,v3,v4]` | `{"speeds":[...]}` |
| `GET /v1/realtime/wheels/encoders` | 读 4 轮编码器 | 无 | `{"encoders":[r1,r2,r3,r4]}` |
| `POST /v1/realtime/motor/speed` | 单电机速度（开环，不进 PID） | `port` `speed` `reverse?` | `{"port","speed","reverse"}` |
| `GET /v1/realtime/encoder?port=N` | 单电机编码器 | `port` `reverse?` | `{"encoder":int}` |
| `POST /v1/realtime/stepper/rad` | 步进弧度定位 | `port` `rad` `time?` `reverse?` `perimeter?` | `{"port","rad","time"}` |
| `POST /v1/realtime/bus-servo/angle` | 总线舵机角度下发 | `port` `angle` `speed?` | `{"port","angle","speed"}` |
| `GET /v1/realtime/bus-servo/angle?port=N` | 读总线舵机角度 | `port` | `{"angle":int}` |
| `GET /v1/realtime/analog?port=N` | 单路模拟量 | `port` | `{"value":float}` |
| `GET /v1/realtime/analog2?port=N` | 第二路模拟量 | `port` | `{"value":float}` |

注意：

- `realtime/stepper_rad` 的 `port` 不要跟 yaml 里配置的机械臂 y 轴端口重复，否则两路会互踩串口
- `realtime/bus_servo_read` 在 mc601 控制板上暂不支持（协议层未实现）
- `realtime/motor_speed` 是开环速度下发（不进 PID），调用方负责上层闭环

## 6. `car` 接口

### 6.1 底盘与执行

| 接口名 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `car.beep` | 蜂鸣一声 | 无 | — |
| `car.stop` | 立即停车 | 无 | — |
| `car.reset_position` | 重置底盘里程计原点 | 无 | — |
| `car.move_for` | 相对位姿移动 | `args[0]=[x,y,theta]` `duration?` `max_velocities?` `tolerance?` | 动作完成 |
| `car.move_to_position` | 绝对位姿移动 | `args[0]=[x,y,theta]` `duration?` `max_velocities?` `tolerance?` | 动作完成 |
| `car.move_time` | 按速度跑固定时间 | `args[0]=[vx,vy,wz]` `dur_time?` `stop?` | 动作完成 |
| `car.move_distance` | 按速度跑到指定累计距离 | `args[0]=[vx,vy,wz]` `dis?` `stop?` | 动作完成 |
| `car.set_chassis_velocity` | 原始底盘速度控制 | `x` `y` `z` `duration?` | `{"x","y","z","duration"}` |

### 6.2 巡线

| 接口名 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `task.auto_lane_tracing` | 推荐，自动巡航到目标距离 | `speed` `dis_hold` | 任务结果 |
| `car.lane_time` | 底盘级巡线固定时间 | `speed` `time_dur` `stop?` | 动作完成 |
| `car.lane_dis` | 底盘级巡线到目标累计距离 | `speed` `dis_end` `stop?` | 动作完成 |
| `car.lane_dis_offset` | 从当前距离继续巡线一段增量 | `speed` `dis_hold` `stop?` | 动作完成 |
| `car.get_lane_results` | 单次读取巡线误差 | 无 | `(error_y, error_angle)` |
| `GET /v1/vision/lane/state` | 读 runtime 内最新 lane 状态 | 无 | `active` `error_y` `error_angle` `distance` 等 |

> `car.get_lane_results` 只是单次取样，不等于“持续巡航”。要看持续状态用 `/v1/vision/lane/state`。

### 6.3 视觉与 OCR

| 接口名 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `car.get_detection_results` | 读取侧摄检测结果 | `sort_pos?` `limit_x?` `limit_y?` | `[[cls_id,det_id,label,score,x_c,y_c,w,h], ...]` |
| `car.move_to_detection_target` | 按检测结果做视觉对齐 | `delta_x?` `delta_y?` `label?` `time_out?` `sort_pos?` `num?` | `(cls_id,label)` 或 `(None,None)` |
| `car.adjust_arm_position` | 根据机械臂朝向微调机械臂 X | `dis?` | 动作完成 |
| `car.get_ocr` | 对侧摄文本目标做 OCR | `label?` `time_out?` | `text` 或 `None` |
| `car.get_det_ocr` | 对指定检测框做 OCR | `det` `label?` `time_out?` | `text` 或 `None` |

```json
// car.get_detection_results
[
  [0, 12, "order", 0.98, 0.02, -0.15, 0.30, 0.22]
]
```

### 6.4 枪 / 数字口 / 灯 / 屏

| 接口名 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `car.shooting` | 单次射击触发 | 无 | 动作完成 |
| `car.set_shoot_state` | 直接控制枪口输出 | `value` | `true/false` |
| `car.set_digital_output` | 控任意数字输出口 | `port` `value` | `{"port","value"}` |
| `car.set_light_color` | 控灯带颜色 | `led_id` `r` `g` `b` | `{"led_id","r","g","b"}` |
| `car.show_text` | 屏幕显示文本 | `args[0]=text` | 返回显示内容 |

### 6.5 PWM / 储物仓

| 接口名 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `car.set_storage` | 储物仓开合 | 业务参数 | 动作完成 |
| `car.set_storage_angle` | 直接设储物仓角度 | `angle` `speed?` | `angle` |
| `car.set_pwm_servo_angle` | 控任意 PWM 舵机 | `port` `angle` `mode?` `speed?` | `{"port","angle","mode","speed"}` |

### 6.6 状态读取

| 接口名 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `car.get_odometry` | 读取当前位姿 | `show_info?` | `[x,y,theta]` |
| `car.get_distance` | 读取累计行驶距离 | `show_info?` | `float` |
| `car.get_battery_voltage` | 读取电池电压 | 无 | `float/int` |
| `car.get_ir_distance` | 读取单侧 IR | `side="left/right"` | 单侧距离值 |
| `car.get_all_ir_distance` | 同时读取左右 IR | 无 | `{"left","right"}` |
| `car.get_key_event` | 读取按键事件 | 无 | 按键事件值 |
| `car.get_key_state` | 读取按键状态 | 无 | 按键状态值 |
| `car.get_bluetooth_pad` | 读取蓝牙手柄状态 | 无 | 手柄状态数组 |
| `car.get_arm_state` | 读取机械臂状态 | 无 | `{"x","y","side","arm_angle","hand_angle","y_limit"}` |

## 7. `arm` 接口

| 接口名 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `arm.reset_position` | 机械臂整体复位 | 无 | 动作完成 |
| `arm.reset_x` | 只复位 X 轴 | 无 | 动作完成 |
| `arm.set_arm_pose` | 一次设置 `x/y/arm/hand` | `x?` `y?` `arm?` `hand?` | 动作完成 |
| `arm.set_hand_angle` | 设置手爪角度 | `angle` `speed?` | 动作完成 |
| `arm.set_arm_angle` | 设置大臂角度 | `angle` `speed?` | 动作完成 |
| `arm.move_x_position` | X 轴定位 | `args[0]=target` `out_time?` | 动作完成 |
| `arm.move_y_position` | Y 轴定位 | `args[0]=target` | 动作完成 |
| `arm.goto_position` | 直接移动到指定 `x/y` | `x?` `y?` `time_run?` `speed?` | 动作完成 |
| `arm.go_for` | 相对位移 | 相对位移参数 | 动作完成 |
| `arm.x_speed` | X 轴原始速度控制 | 速度参数 | 动作完成 |
| `arm.y_speed` | Y 轴原始速度控制 | 速度参数 | 动作完成 |
| `arm.grasp` | 吸盘抓取/释放 | `args[0]=true/false` | 动作完成 |
| `arm.x_get_position` | 读当前 X 位置 | 无 | `float` |
| `arm.y_get_position` | 读当前 Y 位置 | 无 | `float` |

## 8. `task` 接口

| 接口名 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `task.auto_lane_tracing` | 巡线测试 / 任务流程中的巡线 | `speed` `dis_hold` | 任务结果 |
| `task.auto_seeding` | 播种任务 | 任务内部参数 | 任务结果 |
| `task.target_shooting_detection` | 识别虫害目标 | — | `animal_list` |
| `task.water_tower_task` | 灌溉任务 | — | 任务结果 |
| `task.target_shooting` | 射击除害 | `animal_list` | 任务结果 |
| `task.crop_harvesting` | 作物收集 | — | 任务结果 |
| `task.sort_and_store` | 作物分拣入仓 | — | 任务结果 |
| `task.get_order` | 读取订单 | — | `order_list` |
| `task.order_delivery` | 按订单配送 | `order_list` | 任务结果 |

可继续编排：

- `task.target_shooting_detection` → `task.target_shooting(animal_list=...)`
- `task.get_order` → `task.order_delivery(order_list=...)`

## 9. WebSocket

`ws://<ip>:5050/v1/ws`

| `op` | 用途 |
| --- | --- |
| `ping` | 连通性检查 |
| `health` | 查询健康状态 |
| `runtime` | 查询运行时快照 |
| `actions` | 查询动作清单 |
| `config` | 查询配置 |
| `execute` | 直接执行动作 |
| `create_job` | 创建异步任务 |
| `get_job` | 查询任务 |
| `init` | 初始化 |
| `stop_mode` | 设置 stop mode |
| `reset_stop` | 清 stop 标记 |
| `close` | 关闭 runtime |
| `emergency_stop` | 立即停车 |
| `realtime/wheel_speeds` | 4 轮线速度 |
| `realtime/wheel_encoders` | 4 轮编码器 |
| `realtime/motor_speed` | 单电机速度 |
| `realtime/encoder` | 单电机编码器 |
| `realtime/stepper_rad` | 步进弧度定位 |
| `realtime/bus_servo_angle` | 总线舵机角度下发 |
| `realtime/bus_servo_read` | 总线舵机读角度 |
| `realtime/analog` | 单路模拟量 |
| `realtime/analog2` | 第二路模拟量 |

## 10. 一句话选型

| 需求 | 接口 |
| --- | --- |
| 直走 / 横移 / 转角 | `car.move_for` |
| 回到某个位置 | `car.move_to_position` |
| 沿赛道自动往前 | `car.lane_dis_offset` |
| 靠视觉对齐目标 | `car.move_to_detection_target` |
| 读取位姿 | `car.get_odometry` |
| 读取检测框 | `car.get_detection_results` |
| 读取 OCR | `car.get_ocr` |
| 控制机械臂姿态 | `arm.set_arm_pose` |
| 读取机械臂状态 | `car.get_arm_state` |
| 跑完整任务 | `task.*` |
| 50Hz 闭环 | `/v1/realtime/*` |
