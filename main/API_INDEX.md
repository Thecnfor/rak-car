# main/ API 总表（单一权威源）

> 所有 runtime HTTP / WS / 客户端方法的**唯一权威清单**。
> 旧版分散在 `API.md` / `API_REFERENCE.md` / `CAPABILITY_LIST.md` / `BUSINESS_API_GUIDE.md` 已合并；如发现与本文档冲突，**以本文档为准**。

---

## 0. 入口与配置

| 项 | 值 | 说明 |
| --- | --- | --- |
| Runtime 地址 | `http://192.168.5.230:5050` | 改 `RAK_CAR_SERVER_ORIGIN` / `RAK_CAR_API_PORT` |
| API 前缀 | `/v1` | 默认；改 `RAK_CAR_API_PREFIX` |
| HTTP / WS 默认超时 | 10s | `RAK_CAR_REQUEST_TIMEOUT` |
| 同步等待上限 | 300s | `RAK_CAR_WAIT_TIMEOUT` |
| 轮询间隔 | 0.5s | `RAK_CAR_POLL_INTERVAL` |
| Stream 地址 | `http://192.168.5.230:5050/stream/` | 改 `RAK_CAR_STREAM_PORT` / `RAK_CAR_STREAM_PATH` |

**Python 客户端**：

| 类 | 来源 | 用途 |
| --- | --- | --- |
| `RuntimeApiClient` | [main/api_client.py](../api_client.py) | 同步 HTTP 请求 + job 异步/同步 |
| `RuntimeWsClient` | [main/ws_client.py](../ws_client.py) | WebSocket 单点调用 + 推送订阅 |

WS 推送订阅：`subscribe_lane` / `subscribe_arm_state` / `subscribe_task_detection` / `subscribe_ir` / `subscribe_odom`。

---

## 1. System / Health

| Method | Path | WS op | 说明 |
| --- | --- | --- | --- |
| GET | `/v1/health` | `health` | `state.initialized` / `state.initializing` / `state.last_error` |
| GET | `/v1/runtime` | `runtime` | runtime 元信息（端口 / PID / 在线摄像头） |
| GET | `/v1/actions` | `actions` | `CAR_ACTIONS` + `ARM_ACTIONS` + `SYSTEM_ACTIONS` 全部注册名 |
| GET | `/v1/config` | — | 当前生效的 runtime 配置 |
| GET | `/v1/infer/state` | — | 推理后端 `ready` 状态（task / lane / ocr） |

**Python**：`client.get_health()` / `client.get_runtime()` / `client.get_actions()` / `client.get_config()`。

**`/v1/health` 字段**：

```json
{
  "ok": true,
  "state": {
    "initialized": true,
    "initializing": false,
    "last_error": null,
    "stop_flag": false
  },
  "inference": {"task": {"ready": true}, "lane": {"ready": true}, "ocr": {"ready": true}},
  "cameras": {"cam1": {"fps": 30.2}, "cam2": {"fps": 25.0}}
}
```

---

## 2. Vision（cam1 车道 + cam2 侧摄）

| Method | Path | 走哪个路径 | 说明 |
| --- | --- | --- | --- |
| POST | `/v1/vision/lane` | car_queue, sync | 单次 cam1 车道识别（20s 阻塞） |
| GET | `/v1/vision/lane/state` | — | **推荐**：读 lane_feed 守护线程缓存（默认 50Hz） |
| GET | `/v1/vision/lane/preview.jpg?cam_id=cam1` | — | cam1 + `d_e/d_a` overlay 一次性 JPEG |
| POST | `/v1/vision/task` | car_queue, sync | 单次 cam2 目标检测（5-15s 阻塞） |
| GET | `/v1/realtime/vision/task` | — | **推荐**：读 task_feed 守护线程缓存（默认 10Hz） |
| GET | `/v1/vision/task/preview.jpg?cam_id=cam2` | — | cam2 + bbox overlay 一次性 JPEG |
| POST | `/v1/vision/ocr` | car_queue, sync | cam2 + OCR（label=`order` / `name`） |
| GET | `/v1/vision/models` | — | 模型清单（含类别标签） |

**Python**：

```python
client.realtime_lane_state()        # 外环 50Hz 轮询
client.get_task_state()             # 边走边看 10Hz
client.execute_car_action("get_detection_results", sync=True, timeout=20,
                          sort_pos=[0,0], limit_x=1, limit_y=1)
```

**`lane_state` 字段**：`{error_y, error_angle, active, mode, updated_at}`，为 None → feed 未运行。
**`task_state.detections`**：`[{cls_id, det_id, label, score, bbox_norm: {x_center, y_center, width, height}}]`。
**`/v1/vision/task` filters**：`sort_pos`（xy 排序方向）、`limit_x` / `limit_y`（只保留 ±limit 范围内的目标）。

> ⚠️ `/v1/vision/task` 现在与 task_feed 守护线程共享同一 ZMQ REQ socket。
> 2026-07-31 修复：`ClintInterface` 内部加了 `threading.Lock()` 串行化 send/recv，**可以并发**。
> 同 socket 的 lane_feed / arm_feed / ir_feed / odom_feed 也都受锁保护。

---

## 3. Jobs & Execute（主入口）

| Method | Path | WS op | 说明 |
| --- | --- | --- | --- |
| GET | `/v1/jobs` | — | 当前所有 job（active + recent history） |
| POST | `/v1/jobs` | `create_job` | 提交 job，202 + `job_id`，不阻塞 |
| GET | `/v1/jobs/{job_id}` | — | 查 job 状态（`queued` / `running` / `succeeded` / `failed`） |
| POST | `/v1/jobs/{job_id}/stop` | — | 协作取消（不等 SDK 完成） |
| POST | `/v1/execute` | `execute` | 单次 action 调用；默认 async |

**`/v1/execute` payload**：

```json
{
  "target": "car | arm | system",
  "name": "动作名（见 §6）",
  "args": [...],
  "kwargs": {...},
  "sync": false,
  "timeout": 30
}
```

- `sync=false`（默认）：立即返回 `job_id`，调用方轮询 `/v1/jobs/{id}`。
- `sync=true`：阻塞轮询到 `succeeded` / `failed`。
- 响应包装：
  ```json
  {"ok": true, "async": true, "job": {"id":"...", "target":"...", "name":"...", "status":"queued|running|succeeded|failed", "result":..., "error":...}}
  ```
  job_id 从 `response.job.id` 取（不是顶层 `id`）。`async` 字段仅 `sync=false` 时出现。
- `target=car` → 进 `car_queue` worker；`target=arm` → 进 `arm_queue` worker；两者物理串口锁仍是公共瓶颈。

**Python**：

```python
# 异步（默认）
job = client.execute_car_action("move_to_position", x=1.0, y=0.5)
client.wait_job(job["id"], timeout=60)

# 同步（一行式）
client.execute_car_action("move_to_position", x=1.0, y=0.5, sync=True, timeout=60)

# 异步链 + 显式 cancel
job_id = client.create_job("car", "move_for", kwargs={"speeds":[0,0,0,0], "time": 5})["id"]
...
client.cancel_job(job_id)
```

---

## 4. Control（系统控制）

| Method | Path | 说明 |
| --- | --- | --- |
| POST | `/v1/control/init` | 触发 auto-init（MC602 重启后自调用，也可手调 `force=true`） |
| POST | `/v1/control/stop-mode` | 切换硬停模式（`{"enabled": true}`） |
| POST | `/v1/control/reset-stop` | 清 `_stop_flag` + 重启 lane_feed（**清 stop 必须调用**） |
| POST | `/v1/control/emergency-stop` | 软件急停（无锁直达，worker 跑长动作也能立刻抢占） |
| POST | `/v1/control/close` | 关闭 runtime，释放 MC602 串口 |
| POST | `/v1/estop` | 同 `emergency-stop`（v1 alias） |
| POST | `/v1/estop/clear` | 同 `reset-stop`（v1 alias） |

**Python**：`client.init_runtime(force=True)` / `client.emergency_stop()` / `client.reset_stop_flag()` / `client.close_runtime()`。

---

## 5. Realtime（车锁同步路径，**不进 job_queue**）

> 设计目的：外环 50Hz 轮询 / 直发轮速 / 直读编码器，全部绕开 job_queue。
> 物理串口锁（`serial_mc602.lock`）仍是公共瓶颈——多线程并发会自然排队。

### 5.1 底盘

| Method | Path | 说明 |
| --- | --- | --- |
| POST | `/v1/realtime/wheels/speeds` | 直发 4 路轮速（`{"speeds": [v0,v1,v2,v3]}`） |
| GET | `/v1/realtime/wheels/encoders` | 读 4 路编码器 |
| POST | `/v1/realtime/chassis-velocity` | `(vx, vy, wz)` 直发，绕开里程计耦合 |
| POST | `/v1/realtime/motor/speed` | 单电机测速（`port, speed, reverse`） |
| GET | `/v1/realtime/encoder?port=&reverse=` | 单编码器读 |
| POST | `/v1/realtime/stepper/rad` | 步进电机按 rad 转（`port, rad, time, reverse, perimeter`） |

### 5.2 机械臂/总线舵机/PWM

| Method | Path | 说明 |
| --- | --- | --- |
| POST | `/v1/realtime/bus-servo/angle` | 总线舵机写角度（`port, angle, speed`） |
| GET | `/v1/realtime/bus-servo/angle?port=` | 总线舵机读角度 |

### 5.3 模拟口

| Method | Path | 说明 |
| --- | --- | --- |
| GET | `/v1/realtime/analog?port=` | 单 ADC 读 |
| GET | `/v1/realtime/analog2?port=` | 第二 ADC 读 |

### 5.4 守护线程缓存（**最快路径**）

| Method | Path | 守护线程 / 频率 | 字段 |
| --- | --- | --- | --- |
| GET | `/v1/realtime/lane/state` | lane_feed / 50Hz | `error_y`, `error_angle`, `active`, `updated_at` |
| GET | `/v1/realtime/arm/state` | arm_feed / 20Hz | `y_m`, `x_m`, `y_mm`, `x_mm`, `ref_encoder`, `active` |
| GET | `/v1/realtime/ir/state` | ir_feed / 50Hz | `active`, `mode`, `left`, `right`, `updated_at` |
| GET | `/v1/realtime/odom/state` | odom_feed / 50Hz | `active`, `mode`, `x`, `y`, `theta`, `distance`, `updated_at` |
| GET | `/v1/realtime/vision/task` | task_feed / 10Hz | `active`, `mode`, `detections`, `count`, `updated_at` |

**Python**：

```python
client.realtime_wheel_speeds([0.1, 0.1, 0.1, 0.1])
client.realtime_lane_state()      # 50Hz 轮询
client.get_ir_state()             # 触发判定
client.get_odom_state()           # 里程计
```

---

## 6. Actions（`runtime/core/actions.py` 全部注册）

### 6.1 底盘 CAR_ACTIONS（`target="car"`）

| Action | 用途 |
| --- | --- |
| `beep` | 蜂鸣 |
| `stop` | 急停 |
| `reset_position` | 里程计清零 |
| `set_storage` / `set_storage_angle` | 存储仓档位 |
| `shooting` | 单次击发脉冲（约 500ms） |
| `set_shoot_state` | 直写数字口（**不要**连发） |
| `move_for` / `move_time` / `move_distance` | 4 轮直行/旋转/距离 |
| `move_to_position` | 里程计点到点（带 location_pid） |
| `set_chassis_velocity` | (vx, vy, wz) 闭环 |
| `lane_time` / `lane_dis` / `lane_dis_offset` | 巡线 + 距离补偿 |
| `start_lane_feed` / `stop_lane_feed` | lane_feed 守护线程开关 |
| `start_ir_feed` / `stop_ir_feed` / `restart_ir_feed` | ir_feed 守护线程开关 / 强制 restart 切档（hz 不同自动 stop → start） |
| `start_odom_feed` / `stop_odom_feed` / `restart_odom_feed` | odom_feed 守护线程开关 / 强制 restart 切档 |
| `move_to_detection_target` | 视觉对齐到目标点 |
| `adjust_arm_position` | 调臂位姿 |
| `get_detection_results` | cam2 目标检测（带 sort/limit） |
| `get_lane_results` | cam1 车道识别（与 vision/lane 同源） |
| `get_odometry` | 里程计读 |
| `get_distance` | 测距 |
| `get_ocr` / `get_det_ocr` | cam2 + OCR |
| `get_bluetooth_pad` | 蓝牙手柄读 |
| `get_battery_voltage` | 电池电压 |
| `get_ir_distance` / `get_all_ir_distance` | 单/全 IR 距离 |
| `set_light_color` | 灯带颜色 |
| `show_text` | 屏幕显示 |
| `set_pwm_servo_angle` | PWM 舵机 |
| `set_digital_output` | 数字输出口 |
| `get_arm_state` | 机械臂 y/x 位置（`{x, y, side, arm_angle, hand_angle, y_limit}`，与 arm_feed 同源走 SDK 直读，不带 `ref_encoder` / `active` 字段；如需守护线程缓存请用 `/v1/realtime/arm/state`） |

### 6.2 机械臂 ARM_ACTIONS（`target="arm"`）

| Action | 用途 |
| --- | --- |
| `reset_position` | y 触底 + x 撞墙，**首次定原点** |
| `reset_y` / `reset_x` / `reset_all` | 单/双轴 opt-in 复位 |
| `composite_pick` / `composite_release` / `composite_go_home` | PR#13 复合动作（2-3 路电机真并发） |
| `set_arm_pose` / `set_arm_angle` / `set_hand_angle` | 一次/单次位姿 |
| `move_x_position` / `move_y_position` / `goto_position` / `go_for` | 单/双轴位移 |
| `x_speed` / `y_speed` | 速度模式 |
| `grasp` | 真空泵 on/off |
| `x_get_position` / `y_get_position` | 读单轴位置 |

### 6.3 System（`target="system"`）

| Action | 用途 |
| --- | --- |
| `init` | 触发 auto-init |
| `reset_stop_flag` | 清 `_stop_flag` + 重启 lane_feed |
| `close` | 关闭 runtime |

---

## 7. Stream（视频流与录制）

| Method | Path | 说明 |
| --- | --- | --- |
| GET | `/stream/info` | 流元信息 |
| GET | `/stream/` | 浏览器预览页 |
| GET | `/video_feed/{cam_id}` | MJPEG 流（cam1 / cam2） |
| GET | `/stream/health` | 流服务健康 |
| GET | `/stream/frame/{cam_id}.jpg` | 单帧 JPEG |
| POST | `/stream/capture` | 触发截图 |
| POST | `/stream/capture/{cam_id}/download` | 触发下载截图 |
| GET | `/stream/captures/{cam_id}/{filename}` | 下载历史截图 |
| GET | `/stream/clear` | 清空截图缓存 |
| POST | `/keypress` | 模拟按键（调相机） |

---

## 8. WebSocket 推送订阅（`/v1/ws`）

| 订阅 op | 推送 op | 频率 | 字段 |
| --- | --- | --- | --- |
| `subscribe_lane` | `lane_state` | 50Hz | 同 `/v1/realtime/lane/state` |
| `subscribe_arm_state` | `arm_state` | 20Hz | 同 `/v1/realtime/arm/state` |
| `subscribe_task_detection` | `task_state` | 10Hz | 同 `/v1/realtime/vision/task` |
| `subscribe_ir` | `ir_state` | 50Hz | 左右 IR 距离 |
| `subscribe_odom` | `odom_state` | 50Hz | 底盘里程计 |

**Python**：

```python
client = RuntimeWsClient()
client.connect()

stop = client.subscribe_lane(lambda s: ctrl_loop(s["error_y"], s["error_angle"]))
# ...
stop()  # 断开订阅连接
```

每个 `subscribe_*` 内部独立 WS 连接，主连接跑 req/rep，零干扰。

---

## 9. 错误码

| HTTP | 含义 |
| --- | --- |
| 202 | job 提交成功（异步路径），response.body 含 `job_id` |
| 400 | payload 缺字段 / 类型错误 |
| 409 | runtime 未初始化 / runtime 主动拒绝（`/v1/realtime/*`） |
| 503 | 守护线程未运行 / cv2 不可用 |
| 500 | 内部错误 |

`{"ok": false, "detail": "..."}` 错误体。

---

## 10. 客户端方法速查（`RuntimeApiClient`）

| 方法 | 端点 |
| --- | --- |
| `get_health()` | `GET /v1/health` |
| `get_actions()` | `GET /v1/actions` |
| `get_config()` | `GET /v1/config` |
| `get_runtime()` | `GET /v1/runtime` |
| `list_jobs()` | `GET /v1/jobs` |
| `create_job(target, name, args, kwargs)` | `POST /v1/jobs` |
| `execute(target, name, args, kwargs, timeout, sync)` | `POST /v1/execute` |
| `cancel_job(job_id)` | `POST /v1/jobs/{id}/stop` |
| `get_job(job_id)` / `wait_job(job_id, timeout, poll_interval)` | `GET /v1/jobs/{id}` |
| `call(target, name, *args, **kwargs)` | `execute(...)` 旧调用方 |
| `wait_until_ready(timeout, poll_interval)` | `GET /v1/health` polling |
| `init_runtime(force, reset_arm, reset_position)` | `POST /v1/control/init` |
| `emergency_stop()` / `reset_stop_flag()` / `close_runtime()` | `/v1/control/*` |
| `realtime_wheel_speeds(speeds)` | `POST /v1/realtime/wheels/speeds` |
| `realtime_wheel_encoders()` | `GET /v1/realtime/wheels/encoders` |
| `realtime_motor_speed(port, speed, reverse)` | `POST /v1/realtime/motor/speed` |
| `realtime_encoder(port, reverse)` | `GET /v1/realtime/encoder` |
| `realtime_stepper_rad(...)` | `POST /v1/realtime/stepper/rad` |
| `restart_ir_feed()` / `restart_odom_feed()` | `execute_car_action("restart_ir_feed" / "restart_odom_feed")` |
| `realtime_bus_servo_angle(...)` / `realtime_bus_servo_read(port)` | `/v1/realtime/bus-servo/angle` |
| `realtime_analog(port)` / `realtime_analog2(port)` | `/v1/realtime/analog*` |
| `realtime_lane_state()` | `GET /v1/realtime/lane/state` |
| `get_arm_state()` | `GET /v1/realtime/arm/state` |
| `get_task_state()` | `GET /v1/realtime/vision/task` |
| `get_ir_state()` | `GET /v1/realtime/ir/state` |
| `get_odom_state()` | `GET /v1/realtime/odom/state` |
| `run_task(name, *a, **kw)` / `run_car_action` / `run_arm_action` | `POST /v1/jobs` |
| `execute_task(name, *a, timeout, sync, **kw)` | `POST /v1/execute` |
| `execute_car_action` / `execute_arm_action` | `POST /v1/execute` |

WS 同名方法略（见 §8）。