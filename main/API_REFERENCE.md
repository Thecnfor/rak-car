# main API 速查手册

只看这个文件就行。

补充文档：

- 双摄流、截图、保存下载： [STREAM_API.md](file:///home/jetson/workspace/rak-car/runtime/STREAM_API.md)
- `lane/task/ocr` 结构化推理结果： [VISION_API.md](file:///home/jetson/workspace/rak-car/runtime/VISION_API.md)

用途：

- 把当前可调接口全部列出来
- 只写接口名、用途、关键参数、返回
- 不讲驱动，不讲业务废话

最重要的约束：

- `car` 和 `arm` 不是让你在业务代码里直接 import 的 Python 对象
- `car` / `arm` / `task` 只是接口里的 `target` 名字
- `main/` 里的业务代码只能通过 API 调用它们
- 不允许业务层直接 import `main/` 以外的任何源码
- 不允许业务层直接调用 `car_wrap_2026.py`、`car_task_function.py`、`smartcar/`、`runtime/` 里的内部实现
- 一切控制统一走接口

## 通用调用格式

统一入口：

- `POST /v1/execute`

你要这样理解：

| 名字     | 身份                        | 业务层该怎么用                                      |
| ------ | ------------------------- | -------------------------------------------- |
| `car`  | 底盘/传感/视觉这一组接口的 `target` 名 | 用 `POST /v1/execute` 或 WebSocket `execute` 调 |
| `arm`  | 机械臂这一组接口的 `target` 名      | 用 `POST /v1/execute` 或 WebSocket `execute` 调 |
| `task` | 已封装任务流程的 `target` 名       | 用 `POST /v1/execute` 或 WebSocket `execute` 调 |

不是这样用：

- 不是 `from xxx import car`
- 不是 `from xxx import arm`
- 不是业务脚本直接调 `car_wrap_2026.MyCar()`
- 不是业务脚本直接调 `car_task_function.*`

请求体：

```json
{
  "target": "car",
  "name": "move_for",
  "args": [[0.05, 0.0, 0.0]],
  "kwargs": {},
  "timeout": 60
}
```

字段：

| 字段        | 说明                     |
| --------- | ---------------------- |
| `target`  | `car` / `arm` / `task` |
| `name`    | 动作名                    |
| `args`    | 位置参数列表                 |
| `kwargs`  | 关键字参数                  |
| `timeout` | 本次动作超时秒数               |

坐标约定：

| 项             | 说明                    |
| ------------- | --------------------- |
| `x`           | 前后，`+` 前进，`-` 后退      |
| `y`           | 左右，`+` 左移，`-` 右移      |
| `theta` / `z` | 旋转，弧度，`+` 逆时针，`-` 顺时针 |

## HTTP 接口

### 状态与元信息

| 接口                | 用途              | 关键返回                                                                                  |
| ----------------- | --------------- | ------------------------------------------------------------------------------------- |
| `GET /v1/health`  | 看服务是否在线、是否初始化完成 | `state.initialized` `state.initializing` `state.last_error`                           |
| `GET /v1/runtime` | 看当前运行时状态        | `runtime.odometry` `runtime.distance` `runtime.stop_after_action` `runtime.stop_flag` |
| `GET /v1/actions` | 看当前支持哪些动作       | `actions.car` `actions.arm` `actions.task` `actions.system`                           |
| `GET /v1/config`  | 看 runtime 配置    | `config`                                                                              |

### 作业接口

| 接口                      | 用途            | 说明                      |
| ----------------------- | ------------- | ----------------------- |
| `POST /v1/execute`      | 同步执行一个动作并等待结果 | 最常用                     |
| `POST /v1/jobs`         | 创建异步任务        | 返回 `job.id`             |
| `GET /v1/jobs`          | 查看任务列表        | 返回全部 jobs               |
| `GET /v1/jobs/{job_id}` | 查看单个任务状态      | 看 `status/result/error` |

### 控制接口

| 接口                                | 用途              | 关键参数                                 |
| --------------------------------- | --------------- | ------------------------------------ |
| `POST /v1/control/init`           | 手动初始化 runtime   | `force` `reset_arm` `reset_position` |
| `POST /v1/control/stop-mode`      | 设置动作后是否自动停      | `enabled`                            |
| `POST /v1/control/reset-stop`     | 清掉 stop 标记      | 无                                    |
| `POST /v1/control/close`          | 关闭当前 runtime 实例 | 无                                    |
| `POST /v1/control/emergency-stop` | 立即停车            | 无                                    |

## WebSocket

地址：

- `ws://<ip>:5050/v1/ws`

支持的 `op`：

| `op`             | 用途           |
| ---------------- | ------------ |
| `ping`           | 连通性检查        |
| `health`         | 查询健康状态       |
| `runtime`        | 查询运行时快照      |
| `actions`        | 查询动作清单       |
| `config`         | 查询配置         |
| `execute`        | 直接执行动作       |
| `create_job`     | 创建异步任务       |
| `get_job`        | 查询任务         |
| `init`           | 初始化          |
| `stop_mode`      | 设置 stop mode |
| `reset_stop`     | 清 stop 标记    |
| `close`          | 关闭 runtime   |
| `emergency_stop` | 立即停车         |

## 实时硬件控制（50Hz 直达路径）

这一组跟 `car.*` 不一样：

- 不进 `job_queue`，不走 `POST /v1/execute`
- 同步在 `car_lock` 内执行，绕过 50Hz 业务循环的 FIFO 排队
- 单次 RTT 在毫秒级，适合 50Hz 闭环
- 出错返回 409（car 未初始化）或 400（参数错）

### HTTP 端点

| 接口 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `POST /v1/realtime/wheels/speeds` | 4 轮线速度直达 | `speeds=[v1,v2,v3,v4]` | `{"speeds":[...]}` |
| `GET /v1/realtime/wheels/encoders` | 读 4 轮编码器 | 无 | `{"encoders":[r1,r2,r3,r4]}` |
| `POST /v1/realtime/motor/speed` | 单电机速度 | `port` `speed` `reverse?` | `{"port","speed","reverse"}` |
| `GET /v1/realtime/encoder?port=N` | 单电机编码器 | `port` `reverse?` | `{"encoder":int}` |
| `POST /v1/realtime/stepper/rad` | 步进弧度定位 | `port` `rad` `time?` `reverse?` `perimeter?` | `{"port","rad","time"}` |
| `POST /v1/realtime/bus-servo/angle` | 总线舵机角度下发 | `port` `angle` `speed?` | `{"port","angle","speed"}` |
| `GET /v1/realtime/bus-servo/angle?port=N` | 总线舵机读角度 | `port` | `{"angle":int}` |
| `GET /v1/realtime/analog?port=N` | 单路模拟量 | `port` | `{"value":float}` |
| `GET /v1/realtime/analog2?port=N` | 第二路模拟量 | `port` | `{"value":float}` |

### WebSocket op

| `op` | 用途 |
| --- | --- |
| `realtime/wheel_speeds` | 4 轮线速度 |
| `realtime/wheel_encoders` | 4 轮编码器 |
| `realtime/motor_speed` | 单电机速度 |
| `realtime/encoder` | 单电机编码器 |
| `realtime/stepper_rad` | 步进弧度定位 |
| `realtime/bus_servo_angle` | 总线舵机角度下发 |
| `realtime/bus_servo_read` | 总线舵机读角度 |
| `realtime/analog` | 单路模拟量 |
| `realtime/analog2` | 第二路模拟量 |

### 注意事项

- `realtime/stepper_rad` 的 port 不要跟 yaml 里配置的机械臂 y 轴端口重复，否则两路会互踩串口
- `realtime/bus_servo_read` 在 mc601 控制板上暂不支持（协议层未实现）
- `realtime/motor_speed` 是开环速度下发（不进 PID），调用方负责上层闭环

## `car` 接口

### 底盘与执行

| 接口名                        | 用途          | 关键参数                                                             | 关键返回                       |
| -------------------------- | ----------- | ---------------------------------------------------------------- | -------------------------- |
| `car.beep`                 | 蜂鸣一声        | 无                                                                | 通常无关键返回                    |
| `car.stop`                 | 立即停车        | 无                                                                | 通常无关键返回                    |
| `car.reset_position`       | 重置底盘里程计原点   | 无                                                                | 通常无关键返回                    |
| `car.move_for`             | 相对位姿移动      | `args[0]=[x,y,theta]` `duration?` `max_velocities?` `tolerance?` | 动作完成                       |
| `car.move_to_position`     | 绝对位姿移动      | `args[0]=[x,y,theta]` `duration?` `max_velocities?` `tolerance?` | 动作完成                       |
| `car.move_time`            | 按速度跑固定时间    | `args[0]=[vx,vy,wz]` `dur_time?` `stop?`                         | 动作完成                       |
| `car.move_distance`        | 按速度跑到指定累计距离 | `args[0]=[vx,vy,wz]` `dis?` `stop?`                              | 动作完成                       |
| `car.set_chassis_velocity` | 原始底盘速度控制    | `x` `y` `z` `duration?`                                          | `{"x","y","z","duration"}` |

### 巡线

推荐这样理解：

- 要让车自己沿线持续跑：优先用 `task.auto_lane_tracing`
- 要做底盘级巡线控制：用 `car.lane_*`
- 要看当前实时巡线误差：用 `GET /v1/vision/lane/state`
- `car.get_lane_results` 只是单次取样，不等于“持续巡航”

| 接口名                         | 用途                   | 关键参数                       | 关键返回                                          |
| --------------------------- | -------------------- | -------------------------- | --------------------------------------------- |
| `task.auto_lane_tracing`    | 推荐，自动巡航到目标距离         | `speed` `dis_hold`         | 任务结果                                          |
| `car.lane_time`             | 底盘级巡线固定时间            | `speed` `time_dur` `stop?` | 动作完成                                          |
| `car.lane_dis`              | 底盘级巡线到目标累计距离         | `speed` `dis_end` `stop?`  | 动作完成                                          |
| `car.lane_dis_offset`       | 从当前距离继续巡线一段增量        | `speed` `dis_hold` `stop?` | 动作完成                                          |
| `car.get_lane_results`      | 单次读取巡线误差             | 无                          | `(error_y, error_angle)`                      |
| `GET /v1/vision/lane/state` | 读取 runtime 内最新 lane 状态 | 无                          | `active` `error_y` `error_angle` `distance` 等 |

### 视觉与 OCR

| 接口名                            | 用途             | 关键参数                                                          | 关键返回                                             |
| ------------------------------ | -------------- | ------------------------------------------------------------- | ------------------------------------------------ |
| `car.get_detection_results`    | 读取侧摄检测结果       | `sort_pos?` `limit_x?` `limit_y?`                             | `[[cls_id,det_id,label,score,x_c,y_c,w,h], ...]` |
| `car.move_to_detection_target` | 按检测结果做视觉对齐     | `delta_x?` `delta_y?` `label?` `time_out?` `sort_pos?` `num?` | `(cls_id,label)` 或 `(None,None)`                 |
| `car.adjust_arm_position`      | 根据机械臂朝向微调机械臂 X | `dis?`                                                        | 动作完成                                             |
| `car.get_ocr`                  | 对侧摄文本目标做 OCR   | `label?` `time_out?`                                          | `text` 或 `None`                                  |
| `car.get_det_ocr`              | 对指定检测框做 OCR    | `det` `label?` `time_out?`                                    | `text` 或 `None`                                  |

### 枪 / 数字口 / 灯 / 屏

| 接口名                      | 用途       | 关键参数                 | 关键返回                     |
| ------------------------ | -------- | -------------------- | ------------------------ |
| `car.shooting`           | 单次射击触发   | 无                    | 动作完成                     |
| `car.set_shoot_state`    | 直接控制枪口输出 | `value`              | `true/false`             |
| `car.set_digital_output` | 控任意数字输出口 | `port` `value`       | `{"port","value"}`       |
| `car.set_light_color`    | 控灯带颜色    | `led_id` `r` `g` `b` | `{"led_id","r","g","b"}` |
| `car.show_text`          | 屏幕显示文本   | `args[0]=text`       | 返回显示内容                   |

### PWM / 储物仓

| 接口名                       | 用途         | 关键参数                            | 关键返回                              |
| ------------------------- | ---------- | ------------------------------- | --------------------------------- |
| `car.set_storage`         | 储物仓开合      | 业务参数                            | 动作完成                              |
| `car.set_storage_angle`   | 直接设储物仓角度   | `angle` `speed?`                | `angle`                           |
| `car.set_pwm_servo_angle` | 控任意 PWM 舵机 | `port` `angle` `mode?` `speed?` | `{"port","angle","mode","speed"}` |

### 状态读取

| 接口名                       | 用途        | 关键参数                | 关键返回                                                  |
| ------------------------- | --------- | ------------------- | ----------------------------------------------------- |
| `car.get_odometry`        | 读取当前位姿    | `show_info?`        | `[x,y,theta]`                                         |
| `car.get_distance`        | 读取累计行驶距离  | `show_info?`        | `float`                                               |
| `car.get_battery_voltage` | 读取电池电压    | 无                   | `float/int`                                           |
| `car.get_ir_distance`     | 读取单侧 IR   | `side="left/right"` | 单侧距离值                                                 |
| `car.get_all_ir_distance` | 同时读取左右 IR | 无                   | `{"left","right"}`                                    |
| `car.get_key_event`       | 读取按键事件    | 无                   | 按键事件值                                                 |
| `car.get_key_state`       | 读取按键状态    | 无                   | 按键状态值                                                 |
| `car.get_bluetooth_pad`   | 读取蓝牙手柄状态  | 无                   | 手柄状态数组                                                |
| `car.get_arm_state`       | 读取机械臂状态   | 无                   | `{"x","y","side","arm_angle","hand_angle","y_limit"}` |

## `arm` 接口

### 位置 / 姿态 / 抓取

| 接口名                   | 用途                  | 关键参数                           | 关键返回    |
| --------------------- | ------------------- | ------------------------------ | ------- |
| `arm.reset_position`  | 机械臂整体复位             | 无                              | 动作完成    |
| `arm.reset_x`         | 只复位 X 轴             | 无                              | 动作完成    |
| `arm.set_arm_pose`    | 一次设置 `x/y/arm/hand` | `x?` `y?` `arm?` `hand?`       | 动作完成    |
| `arm.set_hand_angle`  | 设置手爪角度              | `angle` `speed?`               | 动作完成    |
| `arm.set_arm_angle`   | 设置大臂角度              | `angle` `speed?`               | 动作完成    |
| `arm.move_x_position` | X 轴定位               | `args[0]=target` `out_time?`   | 动作完成    |
| `arm.move_y_position` | Y 轴定位               | `args[0]=target`               | 动作完成    |
| `arm.goto_position`   | 直接移动到指定 `x/y`       | `x?` `y?` `time_run?` `speed?` | 动作完成    |
| `arm.go_for`          | 相对位移                | 相对位移参数                         | 动作完成    |
| `arm.x_speed`         | X 轴原始速度控制           | 速度参数                           | 动作完成    |
| `arm.y_speed`         | Y 轴原始速度控制           | 速度参数                           | 动作完成    |
| `arm.grasp`           | 吸盘抓取/释放             | `args[0]=true/false`           | 动作完成    |
| `arm.x_get_position`  | 读当前 X 位置            | 无                              | `float` |
| `arm.y_get_position`  | 读当前 Y 位置            | 无                              | `float` |

## `task` 接口

| 接口名                              | 用途              | 关键参数                  | 关键返回          |
| -------------------------------- | --------------- | --------------------- | ------------- |
| `task.auto_lane_tracing`         | 巡线测试 / 任务流程中的巡线 | 常用 `speed` `dis_hold` | 任务结果          |
| `task.auto_seeding`              | 播种任务            | 无或任务内部参数              | 任务结果          |
| `task.target_shooting_detection` | 识别虫害目标          | 无                     | `animal_list` |
| `task.water_tower_task`          | 灌溉任务            | 无                     | 任务结果          |
| `task.target_shooting`           | 射击除害            | 常用 `animal_list`      | 任务结果          |
| `task.crop_harvesting`           | 作物收集            | 无                     | 任务结果          |
| `task.sort_and_store`            | 作物分拣入仓          | 无                     | 任务结果          |
| `task.get_order`                 | 读取订单            | 无                     | `order_list`  |
| `task.order_delivery`            | 按订单配送           | 常用 `order_list`       | 任务结果          |

## 最常用返回结构

### `POST /v1/execute`

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

### `GET /v1/runtime`

```json
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

### `car.get_detection_results`

```json
[
  [0, 12, "order", 0.98, 0.02, -0.15, 0.30, 0.22]
]
```

## 一句话选型

| 需求           | 接口                             |
| ------------ | ------------------------------ |
| 直走 / 横移 / 转角 | `car.move_for`                 |
| 回到某个位置       | `car.move_to_position`         |
| 沿赛道自动往前      | `car.lane_dis_offset`          |
| 靠视觉对齐目标      | `car.move_to_detection_target` |
| 读取位姿         | `car.get_odometry`             |
| 读取检测框        | `car.get_detection_results`    |
| 读取 OCR       | `car.get_ocr`                  |
| 控制机械臂姿态      | `arm.set_arm_pose`             |
| 读取机械臂状态      | `car.get_arm_state`            |
| 跑完整任务        | `task.*`                       |
