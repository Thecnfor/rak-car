# rak-car Runtime API 全量手册

> 这份是**全量级** API 文档：runtime 暴露的所有 HTTP / WebSocket 端点、注册到 runtime 的全部 `car` / `arm` / `task` / `system` 动作、字段语义、错误码、curl / WS 示例。
>
> **基础地址**：`http://<host>:<port>`（默认 `5050`，详见 `runtime/core/settings.py`）。
> **路径前缀**：
> - `v1`  = `RAK_CAR_API_PREFIX` = `/v1`（默认，推荐）
> - `legacy` = `RAK_CAR_LEGACY_API_PREFIX` = `/api`（保留兼容老客户端）
>
> 三层组织：
> - §1 — **动作注册表**（`arm` / `car` / `task` / `system`），是所有 `execute` / `create_job` 端点的"菜单"
> - §2 — **HTTP 端点**（按用途分组：作业、控制、视觉、realtime、stream）
> - §3 — **WebSocket 端点**（`/v1/ws` 的 op 清单）
> - §4 — **HTTP / WS 共用字段**（payload 模板、错误码、状态字段）
> - §5 — **关键约定 & 已知限制**

---

## §1. 动作注册表

`POST /v1/execute` / `POST /v1/jobs` / WS `op=execute` / `op=create_job` 用的"菜单"。

通过 `GET /v1/actions` 可拿到运行时版本（注册表可能在不同 commit 之间增减）。

### 1.1 `arm` 动作（15 个）— `target="arm"`

| 动作 | 参数（`args`, `kwargs`） | 用途 | 备注 |
| --- | --- | --- | --- |
| `reset_position` | — | y 触底 + x 撞墙整体复位 | `ArmClient.reset_origin` 走这条 + 写 yaml |
| `reset_y` | — | 仅 y 触底复位（不动 x） | 磁感独证；失败不伪归零 |
| `reset_x` | — | 仅 x 撞墙复位（不动 y） | **需 y < -100**（`ArmClient.reset_x` 安全门） |
| `set_arm_pose` | `x=?, y=?, arm=?, hand=?` | 一次设 4 个轴，None 不动 | 越界抛 `ValueError` |
| `set_hand_angle` | `angle="UP"/"MID"/"DOWN"/int, speed=80` | 手爪 PWM 舵机 | — |
| `set_arm_angle` | `angle="LEFT"/"MID"/"RIGHT"/int, speed=80` | 大臂总线舵机 | — |
| `move_x_position` | `target=m, out_time=6.0` | x 轴定位（带软限位 + 丢步核对） | target 单位**米** |
| `move_y_position` | `target=m` | y 轴定位（带软限位 + 丢步核对） | target 单位**米** |
| `goto_position` | `x=?, y=?, time_run=?, speed=[0.15, 0.04]` | 双轴定位（PID 闭环） | — |
| `go_for` | `x_offset, y_offset, time_run=?, speed=[0.15, 0.04]` | 相对位移 | — |
| `x_speed` | `velocity=m/s` | x 轴开环速度 | — |
| `y_speed` | `velocity=m/s` | y 轴开环速度 | — |
| `grasp` | `value=bool` | 吸盘（真空泵） | — |
| `x_get_position` | — | 读 x（米） | — |
| `y_get_position` | — | 读 y（米） | — |

> ⚠️ `arm.*` 业务层常用单位 mm。`ArmClient` 自动换算；直调 `/v1/execute` 时单位是**米**。
> ⚠️ `move_x_position` / `move_y_position` / `reset_x` / `set_arm_pose` 内部已做软限位 + 丢步核对。

### 1.2 `car` 动作（36 个）— `target="car"`

| 动作 | 参数 | 用途 | 备注 |
| --- | --- | --- | --- |
| `beep` | — | 蜂鸣器响一下 | 调试用 |
| `stop` | — | 立即停三轴 | 等价 `emergency_stop` 但无协作退出 |
| `reset_position` | — | 整车 odometry 归零 | — |
| `set_storage` | `state=bool` | 存储仓二选一档位（False=LEFT, True=RIGHT） | **需 y < -100**（ArmClient 安全门） |
| `set_storage_angle` | `angle=int, speed=100` | 存储仓任意角度 | ⚠️ 业务层禁用（绕开官方标定） |
| `shooting` | — | 射击：继电器高电平脉冲 | — |
| `set_shoot_state` | `value=bool` | 持续设置枪口高低电平 | — |
| `move_for` | `(x, y, z), timeout=?, speed=?` | 相对位移（m, m, rad） | z 是 yaw 角速度 |
| `move_time` | `(x, y, z, time), timeout=?` | 相对位移跑 time 秒 | — |
| `move_distance` | `(distance, theta, speed=?)` | 直线跑 distance 米 | — |
| `move_to_position` | `(x, y, theta=0), speed=?, timeout=?` | 走点到点（带 PID） | x/y 单位 m |
| `set_chassis_velocity` | `(vx, vy, wz, duration=?)` | 设置底盘 3 轴速度（持续 duration 秒） | m/s, m/s, rad/s |
| `lane_time` | `(time, speed=0.3), ...` | 巡线跑 time 秒 | 内环 PID |
| `lane_dis` | `(dis_hold), ...` | 巡线跑 dis_hold 米 | — |
| `lane_dis_offset` | `(dis_hold, speed=0.3), ...` | 巡线+横向补偿跑 dis_hold 米 | — |
| `start_lane_feed` | `hz=20.0` | 启动 `lane_feed` 守护线程 | 幂等 |
| `stop_lane_feed` | — | 停 `lane_feed` | runtime init 默认启，**不调** |
| `move_to_detection_target` | `(label=?, delta_x=0, delta_y=None, time_out=3), ...` | 视觉终点微调 | — |
| `adjust_arm_position` | — | 末端左右微调 | 比赛用 |
| `get_detection_results` | `sort_pos=[0,0], limit_x=1, limit_y=1, timeout=?` | 读侧视检测框 | cam2 |
| `get_lane_results` | — | 读前视巡线误差 | cam1 |
| `get_odometry` | `timeout=?` | 读 (x, y, theta) | m, m, rad |
| `get_distance` | `timeout=?` | 读累计行驶距离 | m |
| `get_ocr` | `timeout=?` | 读 OCR 文本 | — |
| `get_det_ocr` | `(det), label=?, time_out=?` | 读指定检测框的 OCR | — |
| `get_key_event` | — | 读 4 键事件（不消费） | Key4Btn |
| `get_key_state` | — | 读 4 键状态 | — |
| `get_bluetooth_pad` | — | 读蓝牙手柄 | — |
| `get_battery_voltage` | — | 读电池电压 | — |
| `get_ir_distance` | `(port=?)` | 单路红外 | — |
| `get_all_ir_distance` | — | 全部红外 | — |
| `set_light_color` | `(led_id, r, g, b)` | LED 颜色 | 0~255 |
| `show_text` | `text` | 屏显示文本 | 数字屏 |
| `set_pwm_servo_angle` | `(port, angle, mode=180, speed=100)` | 任意 PWM 舵机任意角度 | 业务层慎用 |
| `set_digital_output` | `(port, value=bool)` | 数字口输出 | 枪口/继电器也走这 |
| `get_arm_state` | — | 读舵机状态（side/arm_angle/hand_angle/y_limit） | — |

### 1.3 `task` 动作（9 个）— `target="task"`

8 任务官方流程 + 1 个 OCR order 任务。**业务层不直接调**（用 `car_start_2026.py` 模板）：

| 动作 | 用途 |
| --- | --- |
| `auto_lane_tracing` | 巡线行驶 |
| `auto_seeding` | 自动播种 |
| `target_shooting_detection` | 射击目标检测 |
| `water_tower_task` | 水塔任务 |
| `target_shooting` | 射击 |
| `crop_harvesting` | 收获 |
| `sort_and_store` | 分类存储 |
| `get_order` | OCR 订单 |
| `order_delivery` | 送货 |

### 1.4 `system` 动作（5 个）— `target="system"`

走 `system_job` 通道，**不进** arm/car queue；用于 runtime 生命周期管理。

| 动作 | 参数 | 用途 |
| --- | --- | --- |
| `init` | `reset_arm=bool, force=bool, reset_position=bool` | 手动初始化 runtime（重建 `MyCar()`） |
| `close` | — | 关闭 runtime 实例 |
| `set_stop_mode` | `enabled=bool` | 动作后是否自动 stop |
| `reset_stop_flag` | — | 清 `_stop_flag`（恢复 `lane_feed` / `arm_feed`） |
| `emergency_stop` | — | 急停（写 `_stop_flag=True`，停 `lane_feed`） |

> system 走的是 `_dispatch_system` 而非 worker 队列，所以是"插队"运行，**慎用**。

---

## §2. HTTP 端点

### 2.1 总览

| 路径 | 方法 | 用途 | 鉴权 | 备注 |
| --- | --- | --- | --- | --- |
| `/v1/health` | GET | 整体健康 | 否 | 包含 controller/infer/camera 子状态 |
| `/v1/runtime` | GET | runtime 快照（odometry + stop_flag） | 否 | — |
| `/v1/actions` | GET | 列出注册表 | 否 | 见 §1 |
| `/v1/config` | GET | 运行时配置（env 派生） | 否 | — |
| `/v1/infer/state` | GET | 推理后端状态 | 否 | lane/task/ocr 各 port ready |
| `/v1/estop` | POST | 软件急停（无锁直达） | 否 | 100ms 内停三轴 |
| `/v1/estop/clear` | POST | 解除急停 | 否 | — |
| `/v1/vision/models` | GET | 列出 vision 模型 | 否 | — |
| `/v1/vision/lane` | POST | 触发前视巡线推理（同步） | 否 | — |
| `/v1/vision/lane/state` | GET | 读 lane_feed 缓存的 lane_state | 否 | meta_lock 路径，50Hz 友好 |
| `/v1/vision/lane/preview.jpg` | GET | cam1 + lane 误差 overlay JPEG | 否 | — |
| `/v1/vision/task` | POST | 触发侧视检测（同步） | 否 | — |
| `/v1/vision/ocr` | POST | 触发 OCR（同步） | 否 | — |
| `/v1/jobs` | GET | 任务列表（最近 N 条） | 否 | N=100 (`JOB_HISTORY_LIMIT`) |
| `/v1/jobs` | POST | 提交异步任务 | 否 | 返回 `job` dict（status=queued） |
| `/v1/jobs/{job_id}` | GET | 单个任务状态 | 否 | — |
| `/v1/jobs/{job_id}/stop` | POST | 协作取消任务 | 否 | 立即返回 `cancelled: true` |
| `/v1/execute` | POST | 提交任务（默认异步 / `sync=true` 同步） | 否 | — |
| `/v1/control/init` | POST | 手动 init（同 system.init） | 否 | 异步 202 |
| `/v1/control/stop-mode` | POST | 设置 stop mode | 否 | — |
| `/v1/control/reset-stop` | POST | 清 stop flag | 否 | — |
| `/v1/control/close` | POST | 关闭 runtime | 否 | — |
| `/v1/control/emergency-stop` | POST | 急停（无锁） | 否 | — |
| `/v1/realtime/wheels/speeds` | POST | 4 轮线速度直达 | 否 | _realtime_gate 50Hz 友好 |
| `/v1/realtime/wheels/encoders` | GET | 4 轮编码器 | 否 | — |
| `/v1/realtime/lane/state` | GET | lane_feed 缓存 | 否 | 同 `/v1/vision/lane/state` |
| `/v1/realtime/arm/state` | GET | arm_feed 缓存 | 否 | — |
| `/v1/realtime/chassis-velocity` | POST | (vx, vy, wz) 直发，IK 反算 4 轮 | 否 | 外环 50Hz 用 |
| `/v1/realtime/motor/speed` | POST | 单电机速度 | 否 | — |
| `/v1/realtime/encoder` | GET | 单电机编码器 | 否 | `?port=N` |
| `/v1/realtime/stepper/rad` | POST | 步进电机弧度 | 否 | — |
| `/v1/realtime/bus-servo/angle` | POST / GET | 总线舵机写/读 | 否 | — |
| `/v1/realtime/analog` | GET | 模拟量 1 | 否 | `?port=N` |
| `/v1/realtime/analog2` | GET | 模拟量 2 | 否 | `?port=N` |
| `/v1/ws` | WebSocket | WS 端点 | 否 | 见 §3 |
| `/stream/` | GET | 视频流页面（HTML） | 否 | — |
| `/video_feed/{cam_id}` | GET | MJPEG 长连接 | 否 | cam1 / cam2 |
| `/stream/frame/{cam_id}.jpg` | GET | 单帧 JPEG（带 ETag/304） | 否 | — |
| `/stream/health` | GET | 流服务健康 | 否 | — |
| `/stream/info` | GET | 流信息 | 否 | — |
| `/stream/clear` | GET | 清空帧缓存 | 否 | — |
| `/stream/capture` | POST | 保存单帧到磁盘 | 否 | — |
| `/stream/capture/{cam_id}/download` | POST | 捕获+下载 | 否 | — |
| `/stream/captures/{cam_id}/{filename}` | GET | 下载已保存的捕获 | 否 | — |
| `/keypress` | POST | 模拟 4 键按键 | 否 | — |
| `/health` | GET | legacy 等价 `/v1/health` | 否 | — |
| `/meta` | GET | legacy 等价 `/v1/actions` | 否 | — |
| `/runtime` | GET | legacy 等价 `/v1/runtime` | 否 | — |
| `/jobs` | GET/POST | legacy 等价 `/v1/jobs` | 否 | — |
| `/execute` | POST | legacy 等价 `/v1/execute` | 否 | — |
| `/system/init` | POST | legacy 等价 `/v1/control/init` | 否 | — |
| `/system/stop-mode` | POST | legacy 等价 | 否 | — |
| `/system/reset-stop` | POST | legacy 等价 | 否 | — |
| `/system/close` | POST | legacy 等价 | 否 | — |
| `/system/emergency-stop` | POST | legacy 等价 | 否 | — |

### 2.2 `GET /v1/health`

```bash
curl http://localhost:5050/v1/health
```

```json
{
  "ok": true,
  "state": {
    "initialized": true,
    "initializing": false,
    "last_error": null,
    "current_job_id": "abc123",
    "queued_jobs": 0,
    "stop_after_action": false,
    "stop_flag": false,
    "components": {
      "controller": {"ready": true, "state": "PROGRAM_READY", "port": "/dev/ttyUSB0", "controller": "mc602"},
      "infer":      {"ready": true, "state": "ready", "detail": null},
      "camera":     {"ready": true, "state": "ok",    "detail": null}
    }
  },
  "snapshot": {...},
  "links": {...}
}
```

| 字段 | 含义 |
| --- | --- |
| `state.initialized` | `MyCar()` 是否已构造 |
| `state.components.controller.ready` | MC602 串口在线 + 在 program 模式 |
| `state.components.infer.ready` | 至少一个 inference port 响应 |
| `state.components.camera.ready` | 双摄至少一帧活跃 |
| `state.stop_flag` | `_stop_flag`，true 时 `lane_feed` 守护线程 break |

### 2.3 `GET /v1/actions`

```json
{
  "task": ["auto_lane_tracing", "auto_seeding", ...],
  "car":  ["beep", "stop", "reset_position", ...],
  "arm":  ["reset_position", "reset_y", "reset_x", ...],
  "system": ["init", "close", "set_stop_mode", "reset_stop_flag", "emergency_stop"]
}
```

### 2.4 `POST /v1/execute`

**默认异步**（立即返回 `job_id`）；传 `"sync": true` 同步阻塞到完成。

请求体：

```json
{
  "target": "arm",
  "name": "goto_position",
  "args": [],
  "kwargs": {"x": 0.05, "y": -0.05},
  "sync": false,
  "timeout": 30
}
```

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `target` | `"car"` / `"arm"` / `"task"` / `"system"` | 是 | 注册表 target |
| `name` | str | 是 | 动作名（见 §1） |
| `args` | list | 否 | 位置参数 |
| `kwargs` | dict | 否 | keyword 参数 |
| `sync` | bool | 否 | 默认 `false`（异步）；`true` 阻塞到 done/failed |
| `timeout` | float | 否 | `sync=true` 时最大等待秒数 |

异步返回：

```json
{"ok": true, "async": true, "job": {"id": "abc123", "status": "queued", ...}}
```

同步返回：

```json
{"ok": true, "job": {"id": "abc123", "status": "succeeded", "result": ..., "error": null}}
```

> `arm` 走 `arm_queue`（独立 worker），`car` / `task` / `system` 走 `car_queue`。
> **arm 长动作不挡住 car 短动作**（这是 runtime 并发改造的核心）。

### 2.5 `POST /v1/jobs` / `GET /v1/jobs/{id}` / `POST /v1/jobs/{id}/stop`

`POST /v1/jobs` 接受同 §2.4 的 payload（去掉 `sync`），返回：

```json
{"ok": true, "job": {"id": "abc123", "status": "queued", ...}}
```

`GET /v1/jobs/{id}` 返回 job 完整 dict：

```json
{
  "ok": true,
  "job": {
    "id": "abc123",
    "target": "arm",
    "name": "goto_position",
    "args": [],
    "kwargs": {"x": 0.05, "y": -0.05},
    "status": "succeeded",         // queued | running | succeeded | failed
    "submitted_at": 1784099908.66,
    "started_at": 1784099908.66,
    "finished_at": 1784099909.20,
    "result": null,                 // 成功结果
    "error": null                   // 失败 traceback
  }
}
```

`POST /v1/jobs/{id}/stop` 立即返回 `cancelled: true`（协作取消）：

```json
{"ok": true, "cancelled": true, "job_id": "abc123"}
```

> **SDK 限制**：`arm_base.py` 的 PID 循环不查 `_stop_flag` / `stop_event`，所以 cancel 后 arm 动作会自然跑完；不立刻中断。需要 SDK 配合（独立 PR）。

### 2.6 `POST /v1/control/emergency-stop` 与 `/v1/estop`

两条等价（HTTP 入口和 control 入口都进同一个无锁 `service.emergency_stop()`）：

```bash
curl -X POST http://localhost:5050/v1/estop
# {"ok": true, "stopped": true}
```

立即返回，**不持 car_lock**。副作用：写 `car._stop_flag = True`，让 `lane_feed` / `arm_feed` 守护线程 break 退出。

### 2.7 视觉端点

#### `POST /v1/vision/lane`

```bash
curl -X POST http://localhost:5050/v1/vision/lane -H 'Content-Type: application/json' -d '{}'
```

```json
{
  "ok": true,
  "model": "lane",
  "camera": "cam1",
  "frame_url": "/stream/frame/cam1.jpg",
  "preview_url": "/stream/",
  "state_url": "/v1/vision/lane/state",
  "result": {"error": 0.00008, "angle": -0.435},
  "frame_shape": [480, 640, 3]
}
```

#### `GET /v1/vision/lane/state`

lane_feed 守护线程缓存（默认 20Hz 刷新）：

```json
{
  "ok": true,
  "active": true,
  "mode": "tracking",                  // tracking | external_feed | idle | stopped
  "error_y": 0.0012,
  "error_angle": -0.4354,
  "forward_speed": 0.3,
  "lateral_speed": -0.012,
  "angular_speed": 0.086,
  "distance": 1.284,
  "frame_shape": [480, 640, 3],
  "updated_at": 1784099908.66,
  "frame_url": "/stream/frame/cam1.jpg",
  "preview_url": "/stream/"
}
```

#### `POST /v1/vision/task`

```json
{
  "ok": true,
  "model": "task",
  "camera": "cam2",
  "frame_url": "/stream/frame/cam2.jpg",
  "preview_url": "/stream/",
  "filters": {"sort_pos": [0, 0], "limit_x": 1, "limit_y": 1},
  "count": 1,
  "detections": [
    {
      "index": 0,
      "class_id": 3,
      "track_id": 0,
      "label": "order",
      "score": 0.98,
      "bbox_norm": {"x_center": 0.11, "y_center": -0.03, "width": 0.22, "height": 0.15},
      "bbox_pixels": {"x1": 289, "y1": 201, "x2": 359, "y2": 237, "width": 70, "height": 36}
    }
  ],
  "frame_shape": [480, 640, 3]
}
```

#### `POST /v1/vision/ocr`

```json
{
  "ok": true,
  "model": "ocr",
  "label": "order",
  "text": "苹果两个",
  "matched_detection": {...},
  "detections": [...],
  "frame_url": "/stream/frame/cam1.jpg",
  "source_frame_url": "/stream/frame/cam2.jpg"
}
```

### 2.8 Realtime 端点（50Hz 直达，**不进 job_queue**，**不持 car_lock**）

走 `_realtime_gate` 微秒级瞬持锁，硬件层字节流由 SDK `serial_mc602.lock` 串行。

| 端点 | payload | 返回 | 量纲 |
| --- | --- | --- | --- |
| `POST /v1/realtime/wheels/speeds` | `{"speeds": [v1, v2, v3, v4]}` | `{"speeds": [...]}` | m/s |
| `GET /v1/realtime/wheels/encoders` | — | `{"encoders": [r1, r2, r3, r4]}` | rad 累计 |
| `POST /v1/realtime/chassis-velocity` | `{"vx": 0.3, "vy": 0.0, "wz": 0.0, "duration": null}` | `{"vx": ..., "vy": ..., "wz": ..., "wheel_speeds": [...]}` | m/s, m/s, rad/s |
| `POST /v1/realtime/motor/speed` | `{"port": N, "speed": v, "reverse": 1}` | `{"port": ..., "speed": ..., "reverse": ...}` | — |
| `GET /v1/realtime/encoder?port=N&reverse=1` | — | `{"encoder": int}` | — |
| `POST /v1/realtime/stepper/rad` | `{"port": N, "rad": r, "time": 0.5, "reverse": 1, "perimeter": 0.008}` | `{"port": ..., "rad": ..., "time": ...}` | rad |
| `POST /v1/realtime/bus-servo/angle` | `{"port": N, "angle": a, "speed": 100}` | `{"result": ...}` | 度 |
| `GET /v1/realtime/bus-servo/angle?port=N` | — | `{"angle": int}` | 度 |
| `GET /v1/realtime/analog?port=N` | — | `{"value": int}` | — |
| `GET /v1/realtime/analog2?port=N` | — | `{"value": int}` | — |
| `GET /v1/realtime/lane/state` | — | 同 `/v1/vision/lane/state` | — |
| `GET /v1/realtime/arm/state` | — | 见下 | — |

#### `GET /v1/realtime/arm/state`

arm_feed 守护线程缓存（20Hz）：

```json
{
  "ok": true,
  "arm_state": {
    "active": true,
    "mode": "arm_feed",
    "y_m": 0.0,             // SDK 原始（米）
    "x_m": 0.0,
    "y_mm": 0.0,            // 业务坐标（毫米）
    "x_mm": 0.0,            // 负=向左，正=向右
    "ref_encoder": 0.0115,  // 最近 reset_y 触底时的编码器值
    "updated_at": 1784099908.66
  }
}
```

### 2.9 视频流端点

| 端点 | 用途 | 备注 |
| --- | --- | --- |
| `GET /stream/` | 视频流页面（HTML） | 双摄实时预览 |
| `GET /video_feed/{cam_id}` | MJPEG 长连接 | `Content-Type: multipart/x-mixed-replace` |
| `GET /stream/frame/{cam_id}.jpg` | 单帧 JPEG | 带 ETag/304 缓存 |
| `GET /stream/health` | 流服务健康 | — |
| `POST /stream/capture` | 保存单帧到 `.remember/captures/{cam_id}/` | — |
| `GET /stream/captures/{cam_id}/{filename}` | 下载已保存帧 | — |
| `POST /keypress` | 模拟 4 键按键（`{"key": N}`） | 调试 |

### 2.10 Control 端点（异步 202）

| 端点 | payload | 用途 |
| --- | --- | --- |
| `POST /v1/control/init` | `{"reset_arm": false, "force": false, "reset_position": true}` | 手动 init MyCar |
| `POST /v1/control/stop-mode` | `{"enabled": true}` | 动作后是否自动 stop |
| `POST /v1/control/reset-stop` | — | 清 `_stop_flag`（恢复 lane/arm_feed） |
| `POST /v1/control/close` | — | 关闭 runtime |
| `POST /v1/control/emergency-stop` | — | 急停 |

legacy 路径下同名同语义，路径前缀 `/api/...` 而非 `/v1/...`。

---

## §3. WebSocket 端点

### 3.1 `WS /v1/ws`

#### 握手

```
ws://localhost:5050/v1/ws
```

服务端**不要求鉴权**。握手成功后立即推一条 welcome：

```json
{
  "ok": true,
  "op": "welcome",
  "links": {...},   // 所有公开 URL
  "usage": {...}    // 用法示例
}
```

#### 通用 op 清单（25 个）

| op | 方向 | 等价 HTTP | 说明 |
| --- | --- | --- | --- |
| `ping` | c→s | — | 连通性测试 |
| `health` | c→s | `GET /v1/health` | 可选 `payload.snapshot=1` |
| `runtime` | c→s | `GET /v1/runtime` | — |
| `actions` | c→s | `GET /v1/actions` | — |
| `config` | c→s | `GET /v1/config` | — |
| `infer_state` | c→s | `GET /v1/infer/state` | — |
| `create_job` | c→s | `POST /v1/jobs` | 异步提交 |
| `get_job` | c→s | `GET /v1/jobs/{id}` | payload: `{"job_id": "..."}` |
| `execute` | c→s | `POST /v1/execute` | 接受 `sync` 字段，行为同 HTTP |
| `init` | c→s | `POST /v1/control/init` | — |
| `stop_mode` | c→s | `POST /v1/control/stop-mode` | payload: `{"enabled": bool}` |
| `reset_stop` | c→s | `POST /v1/control/reset-stop` | — |
| `close` | c→s | `POST /v1/control/close` | — |
| `emergency_stop` | c→s | `POST /v1/control/emergency-stop` | — |

#### Realtime op（9 个，等价 HTTP 端点）

| op | 方向 | 等价 HTTP | payload |
| --- | --- | --- | --- |
| `realtime/wheel_speeds` | c→s | `POST /v1/realtime/wheels/speeds` | `{"speeds": [v1, v2, v3, v4]}` |
| `realtime/wheel_encoders` | c→s (请求-响应) | `GET /v1/realtime/wheels/encoders` | `{}` |
| `realtime/chassis_velocity` | c→s | `POST /v1/realtime/chassis-velocity` | `{"vx": 0.3, "vy": 0.0, "wz": 0.0}` |
| `realtime/motor_speed` | c→s | `POST /v1/realtime/motor/speed` | `{"port": N, "speed": v, "reverse": 1}` |
| `realtime/encoder` | c→s (请求-响应) | `GET /v1/realtime/encoder?port=N` | `{"port": N}` |
| `realtime/stepper_rad` | c→s | `POST /v1/realtime/stepper/rad` | `{"port": N, "rad": r, ...}` |
| `realtime/bus_servo_angle` | c→s | `POST /v1/realtime/bus-servo/angle` | `{"port": N, "angle": a, "speed": 100}` |
| `realtime/analog` | c→s (请求-响应) | `GET /v1/realtime/analog?port=N` | `{"port": N}` |
| `realtime/analog2` | c→s (请求-响应) | `GET /v1/realtime/analog2?port=N` | `{"port": N}` |

#### 订阅型 op（持续推送）

| op | 方向 | 行为 |
| --- | --- | --- |
| `subscribe_lane` | c→s | 订阅 lane_state 推送（默认 20Hz，按 `updated_at` 变化才推） |
| `unsubscribe_lane` | c→s | 取消订阅，关闭后台 task |
| `subscribe_arm_state` | c→s | 订阅 arm_state 推送（20Hz） |
| `unsubscribe_arm_state` | c→s | 取消订阅 |
| `realtime/lane_state` | c→s | 一次性读 lane_state（等价 HTTP） |
| `realtime/arm_state` | c→s | 一次性读 arm_state（等价 HTTP） |

#### WS 用法示例（Python）

```python
import json, websocket

ws = websocket.create_connection("ws://localhost:5050/v1/ws")
print(ws.recv())   # welcome

# 1) 同步执行
ws.send(json.dumps({
    "op": "execute",
    "target": "arm",
    "name": "y_get_position",
    "kwargs": {},
    "sync": True,
    "request_id": "r1",
}))
print(ws.recv())
# {"ok": true, "op": "execute", "job": {...}, "request_id": "r1"}

# 2) 异步执行 + 轮询
ws.send(json.dumps({"op": "execute", "target": "arm", "name": "goto_position",
                     "kwargs": {"x": 0.05, "y": -0.05}}))
ack = json.loads(ws.recv())
job_id = ack["job"]["id"]
while True:
    ws.send(json.dumps({"op": "get_job", "job_id": job_id}))
    s = json.loads(ws.recv())["data"]["job"]
    if s["status"] in ("succeeded", "failed"):
        break
import time; time.sleep(0.05)

# 3) 订阅 lane_state 持续推送
ws.send(json.dumps({"op": "subscribe_lane"}))
print(ws.recv())  # subscribe ack
while True:
    msg = json.loads(ws.recv())
    if msg.get("op") == "lane_state":
        d = msg["data"]
        # 外环 50Hz 控制
        ey, ea = d["error_y"], d["error_angle"]
        ws.send(json.dumps({"op": "realtime/chassis_velocity",
                            "vx": 0.3, "vy": -0.5 * ey, "wz": -0.8 * ea}))
```

---

## §4. 公共字段与错误码

### 4.1 状态字段

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `status` | `"queued" / "running" / "succeeded" / "failed"` | job 状态机 |
| `updated_at` | unix float | 守护线程最后一次刷新时间（用于推送去重） |
| `active` | bool | 守护线程是否在跑 |
| `mode` | str | 守护线程当前模式（"lane_feed" / "arm_feed" / "tracking" / "external_feed" / "idle" / "stopped"） |
| `error_y` / `error_angle` | float | 车道误差（m, rad） |
| `forward_speed` / `lateral_speed` / `angular_speed` | float | 当前车体速度（m/s, m/s, rad/s） |
| `distance` | float | 累计行驶距离（m） |
| `stop_flag` | bool | `_stop_flag`（true 时 `lane_feed` 守护线程退出） |
| `ref_encoder` | float | 最近 `reset_y` 触底时的编码器零点（丢步核对） |

### 4.2 错误码

| 状态码 | 含义 | 触发场景 |
| --- | --- | --- |
| `200` | OK | 正常 |
| `202` | Accepted | 异步任务已入队（`POST /v1/jobs`、`/v1/control/init` 等） |
| `400` | Bad Request | 参数缺失、`target` / `name` 不在注册表、action 必填字段未传 |
| `404` | Not Found | job_id / capture file 不存在 |
| `409` | Conflict | car 未初始化；stream frame 暂不可用 |
| `500` | Internal Server Error | action 抛异常 / 车端 I/O 错 |
| `503` | Service Unavailable | 推理后端未就绪 |
| `504` | Gateway Timeout | `sync=true` 时 action 超时未完成 |

### 4.3 错误响应

```json
{"detail": "action 失败原因..."}
```

`POST /v1/execute` 同步失败时：

```json
{"ok": false, "job": {"status": "failed", "error": "Traceback..."}}
```

### 4.4 通用 payload 模板

| 模板 | 用途 |
| --- | --- |
| `{"target": "...", "name": "...", "args": [...], "kwargs": {...}, "sync": bool, "timeout": float}` | `/v1/execute` / `/v1/jobs` |
| `{"vx": 0.3, "vy": 0.0, "wz": 0.0}` | realtime 底盘速度 |
| `{"speeds": [v1, v2, v3, v4]}` | realtime 4 轮速度 |
| `{"port": N, "speed": v, "reverse": 1}` | realtime 单电机 |
| `{"port": N, "angle": a, "speed": 100}` | realtime 总线舵机 |
| `{"sort_pos": [x, y], "limit_x": 1, "limit_y": 1, "timeout": 20}` | vision/task 过滤 |
| `{"label": "order", ...}` | vision/ocr 标签 |

### 4.5 配置开关（env vars）

| 变量 | 默认 | 用途 |
| --- | --- | --- |
| `RAK_CAR_BIND_HOST` | `0.0.0.0` | API listen |
| `RAK_CAR_BIND_PORT` | `5050` | API listen port |
| `RAK_CAR_PUBLIC_HOST` | `192.168.6.231` | LAN 返回地址 |
| `RAK_CAR_PUBLIC_STREAM_PORT` | = BIND_PORT | 视频流对外端口 |
| `RAK_CAR_PUBLIC_STREAM_PATH` | `/stream/` | 视频流页面路径 |
| `RAK_CAR_AUTO_INIT` | `1` | MC602 重启时自动 rebuild MyCar |
| `RAK_CAR_RESET_ARM` | `0` | init 时跑 `arm.reset_position`（y+x 都归） |
| `RAK_CAR_RESET_POSITION_ON_INIT` | `1` | init 时清 odometry |
| `RAK_CAR_STOP_AFTER_ACTION` | `0` | 动作后自动 stop |
| `RAK_CAR_INFER_AUTO_START` | `1` | runtime 启动后托管 infer_back_end |
| `RAK_CAR_INFER_POLL_INTERVAL` | `1.0` | 推理健康轮询 |
| `RAK_CAR_INFER_READY_TIMEOUT` | `45` | 推理启动最长等待 |
| `RAK_CAR_INFER_HEALTH_TIMEOUT` | `2.0` | 单次推理探测超时 |
| `RAK_CAR_API_PREFIX` | `/v1` | v1 路由前缀 |
| `RAK_CAR_LEGACY_API_PREFIX` | `/api` | legacy 路由前缀 |

---

## §5. 关键约定 & 已知限制

### 5.1 runtime 并发模型（2026-07 改造）

- `car_lock` 已拆为 `_ref_lock`（保护 self.car 引用替换）+ `_realtime_gate`（realtime 端点入口微秒级）。旧 `car_lock` 改成抛 `RuntimeError` 的 property，漏改的代码路径立即崩。
- `job_queue` 拆为 `arm_queue` + `car_queue` 两个 daemon worker。`arm.goto_position` 1-3s 闭环不挡住 `car.*` 短动作。
- 字节流仍由 SDK `serial_mc602.lock` 串行（**物理约束，不可消除**）。
- 详见 `runtime/README.md §并发任务模型`。

### 5.2 守护线程依赖 `_stop_flag`

`lane_feed` / `arm_feed` 循环检查 `self._stop_flag`，true 时 break 并把 `active=False`。**触发场景**：
- `POST /v1/control/emergency-stop` 或 `POST /v1/estop`
- `POST /v1/jobs/{id}/stop`（`cancel_job` 会写 `car._stop_flag=True`）

**恢复**：

```bash
curl -X POST http://localhost:5050/v1/control/reset-stop
curl -X POST http://localhost:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"car","name":"start_lane_feed","kwargs":{"hz":20}}'
```

`arm_feed` 通常由 runtime init 自动启，**业务层不需要手动控制**。

### 5.3 存储仓角度（对齐官方）

`car.set_storage` 的 `servo_1_angle_list = [-42, 165]`，**严格对齐** `baidu_smartcar_2026/car_wrap_2026.py:389`。**不要改这两个角度**——改了物理位置就和官方对不上。

- `LEFT = -42°` → 协议值 48（合法）
- `RIGHT = 165°` → 协议值 255（**超 0~180**，mc602 协议层不识别，舵机实际行为由 mc602 固件决定）
- 业务层**只允许 LEFT/RIGHT 二选一**（`set_storage_angle` 任意角度已注册但业务层禁用）

**y < -100 硬件安全门**：set_storage / reset_x / reset_origin 在 y >= -100 时**直接 raise ValueError**，不下发硬件命令。详见 `main/arm/api.py:_check_y_safe_for_storage`。

### 5.4 `cancel_job` 已知 SDK 限制

`POST /v1/jobs/{id}/stop` 立即返回 `cancelled: true`，但 `arm_base.py` 的 PID 循环不查 `_stop_flag` / `stop_event`，arm 动作会自然跑完（不能立刻中断）。需要 SDK 配合（独立 PR）。急停 `emergency_stop` 走另一条路径，立刻停三轴。

### 5.5 同步/异步 API 选型

- **需要等结果**（链式编排、读位姿、获取 odometry）：`client.execute(..., sync=True)` 或 `client.execute_task(..., timeout=...)`。
- **不需要等**（fire-and-forget、并行编排、机械臂长动作）：`client.execute(..., sync=False)`（默认）拿 `job_id`，后用 `client.get_job` 轮询或 `client.cancel_job` 取消。
- 业务层 `ArmClient` 长动作默认 `sync=True`（保留旧行为）；`ArmClient._call_car` 默认 `sync=False`（短动作异步）。

### 5.6 推理后端

- lane / task / ocr 三个模型走 ZMQ 端口 5001 / 5002 / 5004
- 推理未就绪时 `vision/*` 返回 `503`
- `front` 模型在 `config_car.yml` 但未接入 `MyCar`，所以 `vision/models` 标 `enabled=false`

### 5.7 SDK 锁（物理约束）

所有下位机 byte round-trip 都要过 `serial_mc602.lock`（`smartcar/whalesbot/vehicle/base/serial_wrap.py:84`）。runtime 层并发只能让 Python 层不再阻塞，**字节流依然由 SDK 串行**。两个 worker 并发跑 → 但下发的串口命令在硬件层按顺序处理。
