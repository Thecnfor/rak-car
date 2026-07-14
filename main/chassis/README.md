# main/chassis —— 底盘组独享子包

> 这份文档只回答一件事：**底盘组跑自动寻线（client 端外环）需要知道哪些 API、怎么搭起来**。
> 完整 API 总表见 `main/API_REFERENCE.md`，本文件不再重复。

## 一句话定位

`main/chassis/` = **底盘组专属子包**：自动寻线（lane 推理→客户端控制律→轮速下发）的主循环在这里；底盘只需要新写控律、组合任务，其他全部走 API。

## 双环怎么搭

- **外环（客户端，50Hz）**
  - 订阅 `GET /v1/vision/lane/state` 拿 `(error_y, error_angle, distance, mode)`。
  - 客户端按自己算法（Stanley / Pure Pursuit / P）算 4 轮线速度 `[v1, v2, v3, v4]`（m/s）。
  - 通过 `POST /v1/realtime/wheels/speeds`（或 ws `realtime/wheel_speeds`）直接下发到 MC602，绕开车端 job_queue。
- **内环（车端，车端已有 PID）**
  - `car.move_to_position` / `car.move_for` / `car.lane_*` / `car.move_to_detection_target` 都在车端跑 PID。
  - **重要**：client 端外环跑起来后，**不要再开 `car.lane_*`**，否则会和外环互斥排队。

## 10 行起步

```python
from main.chassis import ChassisClient, DoubleLoopRunner, POuterLoop

api = ChassisClient.connect()             # 默认走 env: RAK_CAR_SERVER_ORIGIN
api.start_lane_feed(hz=20.0)              # 车端开 lane 误差缓存线程（不动轮速）
runner = DoubleLoopRunner(api=api, outer=POuterLoop(vx=0.3), hz=50.0)
runner.run(max_seconds=15.0)               # 内置 zero out 退出
api.stop_lane_feed()
```

完整示例在 `examples/01_minimal_p_lane.py`。

## 底盘组专用 API 子集

| 用途 | 接口 | 推荐方式 | 推荐频率 |
| --- | --- | --- | --- |
| **喂 lane 误差源**（客户端外环必备） | `car.start_lane_feed` / `car.stop_lane_feed` | `ChassisClient.start_lane_feed(hz)` | 启动一次 |
| **读 lane 误差**（外环每帧必读） | `GET /v1/vision/lane/state` | `ChassisClient.get_lane_state()` | ≤50Hz |
| **下发轮速**（外环每帧必写） | `POST /v1/realtime/wheels/speeds` 或 ws `realtime/wheel_speeds` | `ChassisClient.set_wheel_speeds([v1,v2,v3,v4])` | ≤50Hz |
| **读里程计**（点到点/航位推算用） | `car.get_odometry` | `ChassisClient.get_odometry()` | ≤50Hz |
| **读编码器**（高级控律用） | `GET /v1/realtime/wheels/encoders` | `ChassisClient.get_wheel_encoders()` | ≤50Hz |
| **急停** | `POST /v1/control/emergency-stop` | `ChassisClient.emergency_stop()` | 任意 |
| **健康检查** | `GET /v1/health` 或 ws `ping` | `ChassisClient.ping()` | 1Hz |
| **视觉对齐（内环触发）** | `car.move_to_detection_target` | `tasks.track_target(...)` 包装 | 1–2Hz |
| **巡线内环（兜底用）** | `car.lane_dis_offset` | 直接调 | 1–2Hz |

### 1. `car.start_lane_feed(hz=20.0)`

- **做什么**：车端起一个守护线程，**只**跑 `get_lane_results()` 并把 `(error_y, error_angle, distance)` 写到 `/v1/vision/lane/state`。
- **不做什么**：不调用任何 `set_velocity` / `set_wheel_speeds`，不会和你的外环抢锁。
- **同一时刻只允许一个实例**，重复调用 noop。
- 参数：`hz` 默认 20（外环 50Hz 完全够用；上限受 lane 推理 ≈5–10ms 限制）。

### 2. `GET /v1/vision/lane/state`

返回示例：

```json
{
  "active": true,
  "mode": "external_feed",
  "error_y": -0.0234,
  "error_angle": 0.087,
  "forward_speed": null,
  "lateral_speed": null,
  "angular_speed": null,
  "distance": 1.234,
  "updated_at": 1761234567.890,
  "frame_url": "/stream/frame/cam1.jpg",
  "preview_url": "/stream/"
}
```

`LaneState.from_lane_state_payload(...)` 会把它转成 dataclass，自动算 `age_ms`；超过 500ms 没刷新 `DoubleLoopRunner` 会自动 `emergency_stop`。

### 3. 下发轮速

- **WebSocket 优先**：底层 `ChassisClient.set_wheel_speeds(...)` 优先走 ws `realtime/wheel_speeds`，失败回退 http。
- **量纲**：4 个 m/s 线速度，`v1 v2 v3 v4` 分别对应车端 `wheels_chassis.set_linear(...)` 的 4 个轮；逆解由车端完成。
- **典型前向 0.3 m/s + 横移 ±0.1 m/s + 角速度 ±0.5 rad/s** 是这辆麦轮的常用域。

### 4. 客户端控制律（controllers/）

| 名字 | 含义 | 行数 |
| --- | --- | --- |
| `POuterLoop` | 最简 P：`-kp_y * error_y` 横移 / `-kp_theta * error_angle` 角速度 | <40 |
| `StanleyOuterLoop` | Stanley：误差角度 + arctan(k*error_y/v) | <50 |
| `PurePursuitOuterLoop` | Pure Pursuit 占位（按 LaneState 模拟前方目标点） | <60 |

新增控律：继承 `controllers/base.py:OuterLoop`，实现 `step(state, dt) -> [v1,v2,v3,v4]`。`state.has_error / state.error_y / state.error_angle / state.distance` 是稳定字段。

### 5. 安全与兜底（loops/safety.py）

- `EmergencyWatchdog(threshold_ms)`：`lane_state.age_ms` 超阈值 → `emergency_stop`。
- `LostLineDetector(stable_ms, zero_eps)`：误差值齐 0 持续 `stable_ms` → 视作丢线报警。

`DoubleLoopRunner` 已默认接入两者，底盘同学可直接用。

## 目录结构

```
main/chassis/
├── README.md                 ← 你正在看
├── __init__.py
├── api.py                    ← ChassisClient：薄封装 main.api_client / main.ws_client
├── state.py                  ← LaneState / OdometryState / WheelsState
├── loops/
│   ├── closed_loop.py        ← DoubleLoopRunner：50Hz 外环主循环
│   └── safety.py             ← EmergencyWatchdog / LostLineDetector
├── controllers/
│   ├── base.py               ← OuterLoop ABC + mecanum_inverse helper
│   ├── p_controller.py
│   ├── stanley.py
│   └── pure_pursuit.py
├── tasks/                    ← 高层组合（外环 + 内环事件）
│   ├── follow_lane.py        ← 起 lane feed + 外环跑 N 秒
│   ├── track_target.py       ← car.move_to_detection_target 包装
│   └── back_to_line.py       ← 丢线恢复（直走 straight_seconds）
└── examples/
    ├── 01_minimal_p_lane.py  ← README 中引用的 10 行起步
    ├── 02_stanley_lane.py
    └── 03_p2p_with_vision.py
```

## 三条红线

1. **`start_lane_feed` 跑起来后，不要再调 `car.lane_*`**——会撞 car_lock，让外环下发被串行化。
2. **50Hz 闭环只走 `/v1/realtime/wheel_speeds`**（或 ws `realtime/wheel_speeds`）；不要走 `POST /v1/execute`——会进 job queue 排队。
3. **任何脚本入口都要 `try/finally: api.stop_wheel_speeds()`**。`DoubleLoopRunner.run` 已经做了，遇到自己手写的循环请保留同样习惯。

## 在哪查 API

- 底盘专用接口子集：本文档（上面那张表）
- 全部接口速查：[main/API_REFERENCE.md](../API_REFERENCE.md)
- 完整能力清单：[main/CAPABILITY_LIST.md](../CAPABILITY_LIST.md)
- 怎么改控律：直接看 [controllers/base.py](controllers/base.py)，所有控律都是同样的接口
- 出问题了看：[debug-controller-download-stuck.md](../../debug-controller-download-stuck.md)、[debug-mc602-download-stuck.md](../../debug-mc602-download-stuck.md)、[debug-runtime-init-queue.md](../../debug-runtime-init-queue.md)
