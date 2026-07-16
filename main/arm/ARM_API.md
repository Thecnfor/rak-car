# 机械臂业务 API 速查

面向 `main/arm/` 业务层。

> 这份文档讲**四层**：
> 1. **业务层入口**（`ArmClient` / `ArmRunner` / `tasks/` / `examples/`）
> 2. **HTTP/WS 端点**（runtime 暴露的 `/v1/execute` / `/v1/realtime/arm/state` / `subscribe_arm_state` 等）
> 3. **动作注册表**（`runtime/core/actions.py` 里的 `ARM_ACTIONS` / `CAR_ACTIONS`，车端 action）
> 4. **底层 SDK**（`smartcar/whalesbot/vehicle/arm/arm_base.py`，业务层一般不要直调）
>
> 从下到上：先看 §1 业务层用哪个接口 → 不够用再翻 §2 端点 → §3 动作表 → §4 SDK。

---

## 0. 坐标系约定（贯穿四层，禁改）

| 量 | 单位 | 语义 |
| --- | --- | --- |
| `x_mm` | mm | 水平位移，**初始化位置=0**，远离为正。**x 轴无软件软限位**（用户原话"灵活使用就好"），业务层 `_check_safe` 不再校验 x；物理墙 ≈ 0.34m 由 `move_x_position` 的 `x_stop_check` 触发 calibrate 兜底 |
| `y_mm` | mm | 垂直位移，**触底=0**。`y<0` 向上（远离触底），`y>0` 向下（被安全门拦） |
| `side` | enum | `LEFT` / `MID` / `RIGHT` —— 大臂总线舵机 |
| `hand` | enum | `UP` / `MID` / `DOWN` —— 手爪 PWM 舵机 |
| `grasping` | bool | 真空泵（只读） |
| `storage_side` | enum | `LEFT` / `RIGHT` —— 车体存储仓独立 PWM 舵机（port=1） |

默认软限位（与 `state.ArmOrigin` / `arm_origin.yaml` 对齐）：

| 项 | 默认值 | 来源 |
| --- | --- | --- |
| `soft_y_max_mm` | **200** | `state.ArmOrigin.soft_y_max_m = 0.20` |
| `soft_x_min_mm` | **None** | x 轴软限位已取消（2026-07-16），`ArmOrigin.soft_x_min_m` 字段保留兼容读 yaml |
| `soft_x_max_mm` | **None** | 同上，`soft_x_max_m` 保留为 None |

---

## 1. 业务层入口

```python
from main.arm import (
    ArmClient,            # 薄封装 HTTP/WS
    ArmRunner,            # 业务编排（move_xy + dry-run + 状态读）
    ArmState,             # 位姿 dataclass（mm + 枚举）
    ArmOrigin,            # 业务坐标系软限位
    TrajectoryGenerator,  # S 曲线 dry-run
    TrajectoryPlan, TrajectorySample,  # dry-run 结果
    OriginCalibrator, run_calibrator,  # 重新定原点（漂移后手调用）
    SIDES, HANDS, STORAGE_SIDES,       # 合法枚举
    STORAGE_DEFAULT_LEFT_ANGLE,        # -42°
    STORAGE_DEFAULT_RIGHT_ANGLE,       # 90°
)
```

### 1.1 `ArmClient` 业务动作

| 方法 | 用途 | 关键参数 | 底层 action / 端点 | 备注 |
| --- | --- | --- | --- | --- |
| `connect(load_origin=True)` | 建一个 client（HTTP 自动连、WS 选连） | — | — | 同步加载 `arm_origin.yaml` |
| `set_pose(x_mm, y_mm, timeout=30)` | 一次设 x/y（**side/hand 已删 2026-07-16**） | mm | `POST /v1/execute {target:arm, name:set_arm_pose}` | 保护区拦截 |
| `move_xy(x_mm, y_mm, v_max=150, a_max=400, timeout=None)` | 双轴同步 | mm, mm/s, mm/s² | `arm.goto_position` | 客户端 S 曲线 dry-run 算 `plan.T`，自动超时 = `max(5, T*2+1)` |
| `move_y(y_mm, v_max=80, timeout=20)` | 单轴 y | mm | `arm.move_y_position` | 完成后做丢步核对（驱动层 + 上层） |
| `move_x(x_mm, v_max=150, timeout=20)` | 单轴 x | mm | `arm.move_x_position` | 编码器闭环，正常不丢步 |
| `set_arm_angle(angle, speed, timeout)` | 大臂角度（**业务硬限 [0, -150]° + y 保护区**） | float（**必填**） | `arm.set_arm_angle` | angle > 0 / < -150 报 ValueError；0° (MID) 是 init 位置（保护区允许） |
| `set_hand_angle(angle, speed, timeout)` | 手爪角度（**业务硬限 [-90, 0]° + y 保护区**） | float（**必填**） | `arm.set_hand_angle` | angle > 0 / < -90 报 ValueError；-90° (UP) 是 init 位置（保护区允许） |
| `grasp(on, timeout=10)` | 吸盘抓/放 | bool | `arm.grasp` | — |
| `set_storage(side, timeout=10)` | 存储仓档位（写死 -42°/90°） | enum | **`car.set_storage`（注意是 car）** | 实际角度走 `ServoPwm` wrapper，参见 §6 |
| `get_storage()` | 读当前档位（客户端缓存，**不下发舵机**） | — | — | 重建 client 后回 "UNKNOWN" |
| `reset_y(timeout=30)` | **仅**归 y（磁感触底） | — | `arm.reset_y` | 不动 x，详见 §7 |
| `reset_origin(x_wall="left", timeout=60)` | 仅 y 触底定原点 + 写 `arm_origin.yaml` | `"left"`/`"right"` | `arm.reset_position` | `x_origin_m` 固定 0（x 无撞墙校准） |
| `save_origin(origin)` | 单独把 `ArmOrigin` 写盘 | `ArmOrigin` | — | — |

```python
# 一次成功调用的返回结构（move_xy 内部）
{
  "ok": True,
  "job": {
    "id": "xxxx",
    "target": "arm",
    "name": "goto_position",
    "status": "succeeded",
    "result": None,
    "error": None
  }
}
```

### 1.2 `ArmClient` 状态读取

| 方法 | 用途 | 关键返回 |
| --- | --- | --- |
| `get_state()` | 一次拿全 `ArmState` | `ArmState` |
| `get_pose_mm()` | 简版位姿 | `(x_mm, y_mm, side, hand)` |
| `get_x_mm()` / `get_y_mm()` | 单值 | float |
| `emergency_stop()` | 车端软件急停 | job dict |
| `ping()` | runtime 在线 | bool |

```python
# ArmState（ArmClient.get_state 返回）
@dataclass
class ArmState:
    x_mm: float
    y_mm: float
    side: str            # LEFT/MID/RIGHT
    hand: str            # UP/MID/DOWN
    grasping: bool       # 恒为 False（车端没暴露）
    storage_side: str    # 恒为 "LEFT"（ArmClient.get_state 不填，请用 ArmClient.get_storage()）
    y_origin_valid: bool # 来自 car.get_arm_state.y_limit
    x_origin_valid: bool # 恒为 False（车端没暴露）
    soft_y_max_mm: float  # 默认 200
    soft_x_min_mm: float  # 默认 -320（v3 双边）
    soft_x_max_mm: float  # 默认 320
    raw_x_m: float
    raw_y_m: float
    arm_angle: int | None
    hand_angle: int | None
    fetched_at: float
```

`ArmState.in_safe_box(x, y)` / `ArmState.is_ready()` 给上层做预校验。

### 1.3 `ArmRunner`（业务编排入口）

`ArmRunner.move_xy / move_x / move_y` 在 `ArmClient` 基础上加：**自动 dry-run 日志 + 超时**，移动完成后调用 `_verify_*` 做上层丢步核对。

| 方法 | 用途 | 备注 |
| --- | --- | --- |
| `move_xy(x_mm, y_mm, v_max=150, a_max=400)` | 同 `ArmClient`，带 dry-run 日志 | 默认超时 = `max(default_timeout_s, T*2+1)` |
| `move_x(x_mm, verify=True)` | 单轴 | `verify=True` 时上层丢步 warn |
| `move_y(y_mm, verify=True)` | 单轴 | `verify=True` 时上层丢步 warn |
| `set_side(side)` | 大臂方向 | — |
| `set_hand(hand)` | 手爪角度 | — |
| `set_storage(side)` | 存储仓档位 | — |
| `get_storage()` | 读档位（不下发） | — |
| `grasp(on)` | 吸盘 | — |
| `go_home()` | 回 y=0, x=0, hand=UP, side=MID | — |
| `pick(side, x_mm, y_mm)` | `set_side` + `move_xy` + `grasp(True)` | — |
| `release(drop_x_mm=0, drop_y_mm=30)` | `set_hand(DOWN)` + `move_xy` + `grasp(False)` | — |
| `reset_y(timeout=30)` | **仅** y 触底 | 走 `arm.reset_y`，**不动 x** |

### 1.4 `tasks/`（高层组合，单函数入口）

| 接口 | 用途 |
| --- | --- |
| `go_home()` | 回原点 + 安全姿态 |
| `pick_left(x_mm, y_mm)` | 左侧抓取 |
| `pick_right(x_mm, y_mm)` | 右侧抓取 |
| `release(drop_x_mm=0, drop_y_mm=30)` | 释放到指定位置 |

### 1.5 `OriginCalibrator`（重新定原点）

```python
from main.arm import OriginCalibrator
from main.api_client import RuntimeApiClient

http = RuntimeApiClient()
origin = OriginCalibrator(http).run(x_wall="left")
# 走 arm.reset_position（车端 y 触底 + x 撞墙）→ 读 y_get_position / x_get_position
# → 把当前编码器值作为新原点写到 arm_origin.yaml
# 写盘用的 soft_* 默认值 = state.ArmOrigin 默认值（200 / -320 / +320 mm），不再覆盖成 v1 单边值
```

什么时候用：

| 场景 | 用法 |
| --- | --- |
| 首次上电 | **不需要手动调** —— runtime 启动时若 `RAK_CAR_RESET_ARM=1` 会自动跑一次 |
| 漂移严重 / PID 卡死 / 编码器读数明显不对 | 手跑 `examples/01_calibrate_origin.py left`（或 `right`） |
| 业务代码里临时复位 | `ArmClient.reset_origin(x_wall="left")` |

> 历史说明：旧版 `OriginCalibrator` 需要在车端按 4 键（1=y 下，3=y 上，2=x 左，4=x 右）手动 jog，再 `1+3` 同时按 1 秒保存 —— **该流程已删除**。当前实现直接下发一次 `arm.reset_position`，由车端 PID 闭环完成触底 / 撞墙，不再监听任何按键。

### 1.6 `examples/`

| 文件 | 用途 |
| --- | --- |
| `examples/01_calibrate_origin.py` | 触发车端 `arm.reset_position` 重新定原点 |
| `examples/02_trajectory_preview.py` | 不下硬件，只算 S 曲线 |
| `examples/03_move_xy_basic.py` | 双轴同步移动 + dry-run |
| `examples/04_grasp_template.py` | 完整 pick-and-place |

### 1.7 `TrajectoryGenerator`（S 曲线 dry-run）

```python
from main.arm import TrajectoryGenerator

gen = TrajectoryGenerator(v_max=150, a_max=400, j_max=2000)
plan = gen.plan_xy(0, 0, 100, 80, sample_hz=50)
print(plan.describe())           # TrajectoryPlan((0.0,0.0) -> (100.0,80.0) mm, T=2.30s, ...)
print(gen.total_time(plan))      # 2.30
print(gen.sample(plan, t_s=1.0).x_mm)
```

返回的 `TrajectoryPlan`：

```python
@dataclass
class TrajectoryPlan:
    x0: float
    y0: float
    x1: float
    y1: float
    T: float          # 总时间（s）
    T_x: float        # x 轴单独走需要的时间
    T_y: float        # y 轴单独走需要的时间
    peak_vx: float    # x 轴峰值速度（mm/s）
    peak_vy: float
    v_max: float      # 规格
    a_max: float
    j_max: float
    samples: list[TrajectorySample]
```

> 注意：**S 曲线只用于客户端 dry-run / 日志 / 超时，不下发到硬件**。硬件仍然走车端 PID。

### 1.8 一句话选型

| 需求 | 接口 |
| --- | --- |
| 首次上电定原点 | 不用手动 —— runtime 在 `RAK_CAR_RESET_ARM=1` 时自动跑；漂移后再手跑 `examples/01_calibrate_origin.py` |
| 之后重置原点 | `ArmClient.reset_origin("left")`（仅 y 触底定原点，x 固定 0） |
| 只重置 y | `ArmClient.reset_y()` / `ArmRunner.reset_y()` |
| ~~只重置 x~~ | ❌ reset_x 已删除（2026-07-16）；x 位置由视觉闭环控制，无软件复位 |
| 双轴同步移动 | `ArmClient.move_xy(...)` / `ArmRunner.move_xy(...)` |
| 单轴移动 | `ArmClient.move_x/move_y` |
| 改大臂角度 | `ArmClient.set_arm_angle(-90, speed=80, timeout=10)` |
| 改手爪角度 | `ArmClient.set_hand_angle(-30, speed=80, timeout=10)` |
| 切存储仓档位 | `ArmClient.set_storage("RIGHT")` |
| 读存储仓档位 | `ArmClient.get_storage()` |
| 抓取 | `ArmClient.grasp(True)` |
| 释放 | `ArmClient.grasp(False)` |
| 读位姿 | `ArmClient.get_state()` |
| 读 y/x（20Hz，**不抢 car_lock**） | `RuntimeApiClient.get_arm_state()` 或 WS `subscribe_arm_state`（§2.3） |
| 算 S 曲线（不下发） | `TrajectoryGenerator().plan_xy(...)` |
| 完整 pick-and-place | `examples/04_grasp_template.py` |

---

## 2. runtime HTTP / WS 端点

> runtime 在 `runtime/api/routes.py` 暴露。**业务层一般不要直调**，通过 `main.api_client.RuntimeApiClient` / `main.ws_client.RuntimeWsClient` 转。

### 2.1 执行动作（car_lock 同步路径，进 job_queue）

```bash
# 走 arm action
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"goto_position","args":[],"kwargs":{"x":0.1,"y":0.04}}'

# 走 car action（注意 set_storage 是 car 不是 arm）
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"car","name":"set_storage","args":[],"kwargs":{"state":true}}'

# 读 car.get_arm_state
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"car","name":"get_arm_state","args":[],"kwargs":{}}'
```

可用 `target`：`task` / `car` / `arm` / `system`。完整 action 列表见 §3。

### 2.2 实时臂位 HTTP 端点（meta_lock 路径，**不抢 car_lock**）

runtime 启动时由 `arm_feed` 守护线程（默认 20Hz）持续把机械臂 y/x 位置刷到 `streamer.arm_state`。`/v1/realtime/arm/state` 直接读这份缓存，**不进 job_queue、不打 ZMQ、不抢 car_lock**，调试/UI 轮询 20Hz+ 安全。

```bash
curl http://192.168.6.231:5050/v1/realtime/arm/state
```

返回：

```json
{
  "ok": true,
  "arm_state": {
    "active": true,
    "mode": "arm_feed",
    "y_m": 0.0,            // SDK 原始坐标（米）
    "x_m": 0.0,
    "y_mm": 0.0,           // 业务坐标（毫米）
    "x_mm": 0.0,           // 业务坐标，负=向左，正=向右
    "ref_encoder": 0.0115, // 最近一次 reset_y 后的编码器零点（丢步核对用）
    "updated_at": 1784099908.66
  }
}
```

字段语义：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `y_m` / `x_m` | float / None | SDK 内部坐标，**米**，与 `arm.y_get_position()` / `arm.x_get_position()` 一致 |
| `y_mm` / `x_mm` | float / None | 业务坐标，**毫米**，`= y_m*1000` / `= x_m*1000` |
| `ref_encoder` | float / None | 最近 `reset_y` 触发磁感时的编码器值，丢步核对用 |
| `active` | bool | `arm_feed` 守护线程是否在跑 |
| `mode` | str | `"arm_feed"`（运行中）/ `"idle"`（已停） |
| `updated_at` | float / None | unix 时间戳，WS 推送用它判"是否新数据" |

> 业务坐标系与 SDK 完全一致：触底/撞墙=0，负=向上/向左（远离触底/远离墙）。

### 2.3 实时臂位 WebSocket（推荐 UI 用）

`/v1/ws` 端点新增两个 op，与 `subscribe_lane` 完全同构：

| op | 方向 | 说明 |
| --- | --- | --- |
| `subscribe_arm_state` | client → server | 订阅 arm_state 推送，服务端按 `updated_at` 变化推 |
| `unsubscribe_arm_state` | client → server | 取消订阅，server 关闭推送 task |
| `arm_state` | server → client | 推送数据，字段同 HTTP 响应 |
| `realtime/arm_state` | client → server | 一次性读取（等价于 HTTP） |

```python
from main.ws_client import RuntimeWsClient
client = RuntimeWsClient("ws://192.168.6.231:5050/v1/ws")
client.connect()

def on_arm(state):
    y_mm = state.get("y_mm")
    x_mm = state.get("x_mm")
    print(f"y={y_mm:.1f}mm  x={x_mm:.1f}mm  ref={state.get('ref_encoder'):.4f}")

stop = client.subscribe_arm_state(on_arm, hz=20.0)
# ... 跑机械臂 ...
stop()  # 断开订阅
```

等价 Python HTTP：

```python
from main.api_client import RuntimeApiClient
client = RuntimeApiClient("http://192.168.6.231:5050")
state = client.get_arm_state()["arm_state"]
print(state["y_mm"], state["x_mm"])
```

### 2.4 HTTP/WS 速查

```
GET  /v1/realtime/arm/state              一次性读 (curl 调试)

WS   /v1/ws
  -> {"op": "subscribe_arm_state", "hz": 20.0}
  <- {"ok": true, "op": "subscribe_arm_state", "subscribed": true, "hz": 20.0}
  <- {"ok": true, "op": "arm_state", "data": {...}}   # 持续推送
  -> {"op": "unsubscribe_arm_state"}
  <- {"ok": true, "op": "unsubscribe_arm_state", "subscribed": false}

WS   /v1/ws
  -> {"op": "realtime/arm_state"}
  <- {"ok": true, "op": "realtime/arm_state", "data": {"arm_state": {...}}}
```

### 2.5 `arm_feed` 守护线程

`arm_feed` 是 `MyCar` 上的一个守护线程（默认 20Hz 拉取 `arm.x_get_position` / `arm.y_get_position`，写 `streamer.arm_state`），由 runtime `_create_car_locked` / `ensure_initialized` 在 init 阶段**默认启动**，idempotent。

行为约束：

- **不抢 car_lock**：取数据走 `meta_lock`，机械臂长动作期间不卡帧
- **不污染主帧流**：`arm_feed` 不读摄像头、不写 `frames`，仅写 `arm_state` meta
- **init 时自动启**：`runtime _create_car_locked` 默认起 `arm_feed`（20Hz），除非 `arm_feed 启动失败` 才会缺失
- **disconnect 自动清理**：WS 断连时 `arm_push_task` 自动 cancel

> 当前 `arm_feed` 没有运行时启停 HTTP 端点（也没有 arm action），业务层不需要手动管。如果想观察是否在跑：看 `arm_state.active=True` / `mode="arm_feed"`。

---

## 3. 动作注册表（runtime `core/actions.py`）

### 3.1 `ARM_ACTIONS`（target = `"arm"`）

| action | 底层 `car.arm` 方法 | 关键参数 |
| --- | --- | --- |
| `reset_position` | `arm.reset_position()` | — |
| `reset_y` | `arm.reset_y()` | — |
| ~~`reset_x`~~ | ~~`arm.reset_x()`~~ | ❌ 已删除（2026-07-16） |
| `set_arm_pose` | `arm.set_arm_pose(x=None, y=None, arm=None, hand=None)` | x?, y?, arm?, hand? |
| `set_hand_angle` | `arm.set_hand_angle(angle, speed=80)` | `angle` (UP/MID/DOWN/int), `speed?` |
| `set_arm_angle` | `arm.set_arm_angle(angle, speed=80)` | `angle` (LEFT/MID/RIGHT/int), `speed?` |
| `move_x_position` | `arm.move_x_position(target, out_time=6.0)` | `target` (m), `out_time?` |
| `move_y_position` | `arm.move_y_position(target)` | `target` (m) |
| `goto_position` | `arm.goto_position(x=None, y=None, time_run=None, speed=[0.15, 0.04])` | `x?`, `y?` (m) |
| `go_for` | `arm.go_for(x_offset, y_offset, time_run=None, speed=[0.15, 0.04])` | 偏移 (m) |
| `x_speed` | `arm.x_speed(velocity)` | velocity (m/s) |
| `y_speed` | `arm.y_speed(velocity)` | velocity (m/s) |
| `grasp` | `arm.grasp(value)` | bool |
| `x_get_position` | `arm.x_get_position()` | 返回 m |
| `y_get_position` | `arm.y_get_position()` | 返回 m |

### 3.2 机械臂相关的 `CAR_ACTIONS`（target = `"car"`）

| action | 底层 `car` 方法 | 用途 |
| --- | --- | --- |
| `set_storage` | `car.set_storage(state: bool)` | 存储仓二选一档位（state=True→RIGHT / state=False→LEFT） |
| `set_storage_angle` | `car.set_storage_angle(angle, speed=100)` | 存储仓任意角度（**业务层慎用**，会绕开协议校验） |
| `get_arm_state` | `car.get_arm_state()` | 读舵机反馈（`side` / `arm_angle` / `hand_angle` / `y_limit`） |

### 3.3 动作分发的对外入口

```python
from main.api_client import RuntimeApiClient
http = RuntimeApiClient()

# 阻塞执行 arm action
http.execute_arm_action("goto_position", x=0.1, y=0.04, timeout=30)

# 阻塞执行 car action（set_storage 是 car 不是 arm！）
http.execute_car_action("set_storage", state=True, timeout=10)
http.execute_car_action("get_arm_state", timeout=10)

# 异步提交
http.create_job("arm", "goto_position", args=[], kwargs={"x": 0.1, "y": 0.04})
```

完整 HTTP/WS 接口和错误码见 [main/API.md](../API.md)。

---

## 4. 底层 SDK（业务层不要直调）

| 接口 | 用途 | 关键参数 |
| --- | --- | --- |
| `arm.reset_position` | 车端整体复位（仅 y 触底；reset_x 已删除） | — |
| `arm.reset_y` | 仅归 y（磁感 + 50ms dwell，不动 x） | — |
| `arm.set_arm_pose` | 一次设置 x/y/arm/hand | `x?` `y?` `arm?` `hand?` |
| `arm.set_hand_angle` | 手爪角度 | `angle` `speed?` |
| `arm.set_arm_angle` | 大臂角度 | `angle` `speed?` |
| `arm.move_x_position` | x 轴定位 | `target` (m) `out_time?` |
| `arm.move_y_position` | y 轴定位 | `target` (m) |
| `arm.goto_position` | 双轴定位 | `x?` `y?` (m) |
| `arm.go_for` | 相对位移 | 偏移 (m) |
| `arm.x_speed` | x 轴开环速度 | velocity (m/s) |
| `arm.y_speed` | y 轴开环速度 | velocity (m/s) |
| `arm.grasp` | 吸盘 | bool |
| `arm.x_get_position` | 读 x | m |
| `arm.y_get_position` | 读 y | m |

完整 SDK 注释见 [smartcar/whalesbot/vehicle/arm/arm_base.py](../../smartcar/whalesbot/vehicle/arm/arm_base.py)。

---

## 5. 启动归零（init 流程）

`runtime _create_car_locked` 每次创建 `MyCar` 后：

1. `reset_arm=False`（默认）：**只调 `arm.reset_y` 触底定原点**，**不**调 `arm.reset_position`（不动角度 / 底盘）
2. `reset_arm=True`：调 `arm.reset_position` 完整复位（仅 `reset_y`，`reset_x` 已删除）

为何默认只归 y？

- y 轴有磁感触底，3-10s 可信定原点；x 轴无磁感/限位传感器，撞墙会空转/编码器漂移
- 不归零 y 的话后续 `move_y` 会在错误基准上跑，导致距离全错
- x 位置由视觉闭环控制（`move_to_detection_target` + `subscribe_task_detection`），不需要软件复位
- 失败不抛 init（避免 1 次瞬态故障阻断整个 runtime），仅记 `last_error`

复用现有 `MyCar` 时（`ensure_initialized` 走 reused 分支），同样会幂等调一次 `start_arm_feed(hz=20)`。

---

## 6. 存储仓 `set_storage` 角度范围

存储仓是独立 PWM 舵机（`ServoPwm` wrapper，`port=1`），LEFT/RIGHT 角度写死：

- `STORAGE_DEFAULT_LEFT_ANGLE = -42°`
- `STORAGE_DEFAULT_RIGHT_ANGLE = 90°`

下发时 `ServoPwm` wrapper 把 `angle` 转成 `int(angle/180*180 + 90)` = `angle + 90`：

- LEFT → 协议值 `48`
- RIGHT → 协议值 `180`（临界）

协议层说明：

- `mc601` 自动 clamp 到 0~180，安全
- `mc602` **不 clamp**，超出 0~180 会触发舵机瞬间回中 / 回弹
- 历史事故：`RIGHT=165° → 协议值 255`，舵机"摆一下就回弹"

> **改角度常量时务必保证 `angle + 90 ∈ [0, 180]`**。否则 `car.set_storage` 直接抛 `ValueError`（在 `car_wrap_2026.set_storage:480`）。

如果确实要任意角度，调 `car.set_storage_angle(angle, speed=100)`（CAR_ACTIONS 里有 `set_storage_angle`），但**业务层慎用**——它会绕开协议校验。

---

## 7. 软限位与行程

### 7.1 y 轴

- **业务 y 区间**：`y ∈ [-soft_y_max_mm, 0]` = `[-200, 0]` mm。
- **保护区 y ∈ [0, -80]mm**（2026-07-16 加）：舵机摆动会撞车。
  - **允许**：set_hand('UP') / set_arm_angle('MID'/0)（init 姿态）；move_y 任意值
  - **拦截**：move_xy / move_x / set_pose / set_hand（非 UP）/ set_hand_angle（非 -90）/ set_arm_angle（非 0）
- **SDK y_threshold**：`arm_cfg.yaml` 的 `vert_cfg.threshold = [-0.20, 0.0]`，与业务层一致。
- **硬件实测定**：从 0 走到 `-0.20` 仍有余量，未撞顶部机械硬限位。
- **末段减速带**（`y >= -0.015`）：PWM 限幅 `0.02 m/s`，防过冲。
- **顶段减速带**（`y <= -0.035`）：PWM 限幅 `0.03 m/s`，防失步。**仅 `move_y_position` 往上走时生效**；`reset_y` 永不下行，不走此分支。

### 7.2 x 轴

> **x 轴软限位已取消（2026-07-16）**：用户原话"灵活使用就好，一般不会超"。
> 业务层 `ArmClient._check_safe` 不再校验 x；SDK `move_x_position` / `x_speed` / `goto_position` 不再 clamp。
> PID 主限幅 `horiz_cfg.pid.output_limits = [-0.4, 0.4]` m/s 仍生效，作为速度上限。
> 物理墙 ≈ 0.34m 由 `move_x_position` 的 `x_stop_check` 触发 calibrate 兜底。

- **业务 x 区间**：无软件上限；灵活使用。
- **SDK x 限位**：无。`arm_cfg.yaml:horiz_cfg` 已删除 `threshold` / `slow_band_m` / `slow_velocity` / `top_slow_m` / `top_slow_velocity` / `reset_*` / `wall_*`。
- **撞墙判据**（x 无传感器）：`move_x_position` 中 `x_stop_check`（`STOP_CHECK_THRESHOLD` 控制）+ 100ms dwell 后自动 calibrate `x_pose_start`。
- **历史错配**：`x_threshold` 旧值 `[0, 0.315]`（单调正方向）已取消；`arm_origin.soft_x_min/max_m` 字段保留但固定 None。

> 历史：旧版 `soft_y_max_mm=180` + `threshold=[0, 0.2]`（错配，正方向）已统一改为 `200` + `[-0.20, 0.0]`。

---

## 8. `reset_y` 行为（**磁感是唯一到底凭证**）

> 这是与过去所有版本最大的区别。务必读懂再用。

### 8.1 触发场景

| 入口 | 行为 |
| --- | --- |
| `ArmClient.reset_y()` / `ArmRunner.reset_y()` | 调 `arm.reset_y` action，**只动 y**，不动 x |
| `ArmClient.reset_origin(x_wall)` | 调 `arm.reset_position`（仅 y 触底），x 固定 0 |
| HTTP `POST /v1/execute {"target":"arm","name":"reset_y"}` | 同上 `arm.reset_y` |
| `car.reset_position()` | 内部串行调 `reset_y`（`reset_x` 已删除） |
| `runtime _create_car_locked`（默认 `reset_arm=False`） | 每次 init 默认调 `arm.reset_y`（**不**调 `reset_position` 也不调 `reset_x`） |

### 8.2 算法

```
_y_seeking_bottom = True        # 让 y_speed 磁感门对正速度放行
while 没超时:
  if 急停: 中止
  if 磁感触发:
    if 首次触发: 记 triggered_at
    elif dwell(50ms) 通过: 归零 + 记 ref_encoder + return True
  else:
    按当前 y 选档:
      cur >= -slow_band(15mm): v = +0.02 m/s  # 末段极慢
      else:                    v = +0.08 m/s  # 远段快
    y_speed(v)                 # 包含末段限幅
```

关键不变量：

- 找底期间**只有正向速度**（向下），`v > 0`。
- 成功条件**只有真磁感触发 + 50ms dwell**；`y_stop_check` 已被新实现**完全忽略**（旧实现被它误判导致"假到底"）。
- 失败（超时 / 急停 / 卡死）**不伪归零**——返回 `False`，`y_pose_start` 保持原值，后续 `move_y_position` 会发现偏差并 warn。
- 收工**必 `y_speed(0)`**——绝不残留速度。

### 8.3 返回值

- `True`  = 磁感触发 + dwell 通过，已归零
- `False` = 超时 / 急停 / 卡死，未归零（`y_pose_now` 保持搜索前值）

### 8.4 失败时的诊断

| 现象 | 看哪 |
| --- | --- |
| 永远 10s 超时 | `arm_base.py reset_y: 找底 %.1fs 超时未触发磁感` → 磁感线 / 磁铁松脱 |
| 编码器持续不动 | `reset_y: 编码器持续 2.0s 不动, 疑似失步 / 卡死` → 撞顶失步或机械卡死 |
| 收工 y ≠ 0 | 物理磁感触发点比机械底高 ~10-15mm，属正常，需要绝对精度时看 `ref_encoder` |

---

## 9. ~~`reset_x`~~ 已删除（2026-07-16）

> **x 轴无软件复位**。原因：x 没有磁感 / 限位传感器，撞墙会空转 / 编码器漂移；
> 之前 `reset_x` 触发 25s 超时 + auto_init 反复重建 → pm2 疯转（commit `fb24b1a` 已规避）。
>
> x 位置由视觉闭环控制：`move_to_detection_target` + `subscribe_task_detection`。
> 物理墙由 `move_x_position` 的 `x_stop_check` 触发 calibrate 兜底（`x_pose_start = 当前 dis, _x_wall = left/right`）。
>
> 删除内容：
> - SDK `arm_base.reset_x()` 方法（~140 行，含三层撞墙判据 + 物理清零 + 滑窗 dwell）
> - SDK `arm_base.x_speed` 边界自然停 + 末段 / 顶段减速带（依赖 `x_reset_target_m`）
> - Runtime `ARM_ACTIONS["reset_x"]` 注册
> - 业务 `ArmClient.reset_x()` / `ArmRunner.reset_x()`
> - `arm_cfg.yaml:horiz_cfg` 中 `reset_*` / `wall_*` / `enable_encoder_reset` 字段
>
> 顺带删除的 bug：`MIN_PRE_TRIGGER_DISP` NameError（`arm_base.py:581` 引用未定义常量）。

> **历史参考**：以下内容已删除，仅供回溯。原 §9.1-§9.9 见 git history（commit `fb24b1a` 之前）。
>
> x 与 y 关键区别：**x 只有编码器**，没有磁感 / 限位传感器，撞墙靠**模型驱动 + 物理清零**判定（v2）。
>
> 三层撞墙判据（任一持续 200ms 命中即认定）：
> - 位移比 = 实际位移 / 期望位移 < 0.20（主）
> - 速度比 = 滑窗实测速度 / 命令速度 < 0.30（辅，防丢步误判）
> - 100ms dwell 确认撞墙 → `motor_x.motor.reset()` 物理清零编码器
>
> 与 v1 对比：v1 用 5×0.5mm stall 简单堵转判据 + 数学补偿 `x_pose_start = get_dis() - 0.31`；
> v2 用位移比+速度比 200ms 滑窗 + 物理清零（MC602 ctl_id=2 走 `encoder_2.reset`）。
>
> 修复记录（v3）：旧 `x_speed` 用 `_x_wall` 撞墙门，撞墙 calibrate 设 `_x_wall="right"` 后任何 `velocity>0` 被钳 0 → motor 不动 → `x_stop_check` 命中 → calibrate → 又设 `_x_wall="right"` 死循环。修复：撞墙门改为软限位边界自然停（`cur >= x_hi - 5mm & v>0 → velocity=0`），2026-07-16 随软限位取消一并删除。

---

## 10. 丢步核对

`move_y_position` 完成后会自动核对：

- 用 `reset_y` 记录的 `ref_encoder` + 累积命令位移 vs 编码器实际位移
- 偏差 > 5mm → `move_y_position 疑似丢步` warn（不抛错，业务层可订阅日志）
- 偏差 > 2mm × N 次 → 建议手动重置原点（`ArmClient.reset_y()`）

阈值可在 `ArmOrigin` 里覆盖：

```python
origin = ArmOrigin(step_loss_y_mm=2.0, step_loss_x_mm=5.0)  # 默认
```

---

## 11. 相关文档

- 子包总览：[README.md](./README.md)
- 10 行起步：[QUICKSTART.md](./QUICKSTART.md)
- 业务 API 总览（包含 chassis）：[../API.md](../API.md)
- runtime 端点：[../../runtime/README.md](../../runtime/README.md) · [../../runtime/VISION_API.md](../../runtime/VISION_API.md)
- SDK 注释：[../../smartcar/whalesbot/vehicle/arm/arm_base.py](../../smartcar/whalesbot/vehicle/arm/arm_base.py)
- 软件急停 / `reset_y` 找底方向 / HTTP 急停端点：[SOFTWARE_ESTOP.md](./SOFTWARE_ESTOP.md)