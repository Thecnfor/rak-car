# 机械臂业务 API 速查

面向 `main/arm/` 业务层。只列可调接口、参数、返回。

## 1. 入口类

```python
from main.arm import (
    ArmClient,        # 薄封装 HTTP/WS
    ArmRunner,        # 业务编排（move_xy + dry-run + 状态读）
    ArmState,         # 位姿 dataclass（mm + 枚举）
    ArmOrigin,        # 业务坐标系软限位
    TrajectoryGenerator,   # S 曲线 dry-run
    TrajectoryPlan,   # 一次 plan 的结果
    OriginCalibrator, # 4 键手动定原点
    SIDES, HANDS,     # 合法枚举
)
```

约定：

- 单位：**mm**（API 层进车端时换算 m）
- 坐标系：`x` 水平，`y` 垂直；`y=0` 为触底
- 合法枚举：`side ∈ {LEFT, MID, RIGHT}`，`hand ∈ {UP, MID, DOWN}`

## 2. ArmClient 业务动作

| 接口 | 用途 | 关键参数 | 关键返回 |
| --- | --- | --- | --- |
| `connect(load_origin=True)` | 建一个 client（自动连 HTTP/WS，加载 arm_origin.yaml） | — | `ArmClient` |
| `set_pose(x_mm=None, y_mm=None, side=None, hand=None)` | 一次设置 4 个轴，None=不动 | 4 个轴，单位 mm + 枚举 | job dict |
| `move_xy(x_mm, y_mm, v_max=150, a_max=400)` | 双轴同步移动（dry-run + 走车端 PID） | 单位 mm，速度 mm/s，加速度 mm/s² | job dict |
| `move_x(x_mm, v_max=150)` | 单轴 x 移动 | mm | job dict |
| `move_y(y_mm, v_max=80)` | 单轴 y 移动 | mm | job dict |
| `set_side("LEFT"/"MID"/"RIGHT")` | 大臂方向（总线舵机） | enum, speed? | job dict |
| `set_hand("UP"/"MID"/"DOWN")` | 手爪角度（PWM 舵机） | enum, speed? | job dict |
| `grasp(True/False)` | 吸盘抓取/释放 | bool | job dict |
| `reset_y()` | 仅复位 y（车端 `reset_position`） | — | job dict |
| `reset_x()` | 仅复位 x（车端 `reset_x`） | — | job dict |
| `reset_origin(x_wall="left")` | 主动撞一侧墙 + 触底，落盘 `arm_origin.yaml` | `"left"`/`"right"` | job dict |

```json
// move_xy 内部调用结构（参考）
{
  "ok": true,
  "job": {
    "id": "xxxx",
    "target": "arm",
    "name": "goto_position",
    "status": "succeeded",
    "result": null,
    "error": null
  }
}
```

## 3. ArmClient 状态读取

| 接口 | 用途 | 关键返回 |
| --- | --- | --- |
| `get_state()` | 一次拿全 `ArmState`（x_mm, y_mm, side, hand, ...） | `ArmState` |
| `get_pose_mm()` | 简版位姿 | `(x_mm, y_mm, side, hand)` |
| `get_x_mm()` | 单独读 x | `float` |
| `get_y_mm()` | 单独读 y | `float` |
| `get_arm_state()`（走 `car.get_arm_state`） | 角状态 | `{"x","y","side","arm_angle","hand_angle","y_limit"}` |
| `ping()` | runtime 在线 | `bool` |
| `emergency_stop()` | 车端急停 | job dict |

```python
# ArmState 结构
@dataclass
class ArmState:
    x_mm: float
    y_mm: float
    side: str            # LEFT/MID/RIGHT
    hand: str            # UP/MID/DOWN
    grasping: bool
    y_origin_valid: bool
    x_origin_valid: bool
    soft_y_max_mm: float
    soft_x_min_mm: float
    soft_x_max_mm: float
    raw_x_m: float       # 车端原始读数（m，调试用）
    raw_y_m: float
    arm_angle: int | None
    hand_angle: int | None
    fetched_at: float
```

## 4. ArmRunner（业务编排入口）

| 接口 | 用途 | 关键参数 |
| --- | --- | --- |
| `move_xy(x_mm, y_mm, v_max=150, a_max=400)` | 同 ArmClient，但带 dry-run 日志 + 自动超时 | mm, mm/s, mm/s² |
| `move_x(x_mm)` | 单轴 | mm |
| `move_y(y_mm)` | 单轴 | mm |
| `set_side(side)` | 大臂方向 | enum |
| `set_hand(hand)` | 手爪角度 | enum |
| `set_storage(side)` | 存储仓档位（LEFT/RIGHT，写死角度） | enum |
| `get_storage()` | 只读当前存储仓档位（客户端缓存，不会下发舵机动作） | — |
| `grasp(on)` | 吸盘 | bool |
| `go_home()` | 回 y=0, x=0, hand=UP, side=MID | — |
| `pick(side, x_mm, y_mm)` | set_side + move_xy + grasp(True) | enum, mm, mm |
| `release(drop_x=0, drop_y=30)` | set_hand(DOWN) + move_xy + grasp(False) | mm, mm |

## 5. tasks/（高层组合）

| 接口 | 用途 |
| --- | --- |
| `go_home()` | 回到原点 + 安全姿态 |
| `pick_left(x_mm, y_mm)` | 左侧抓取 |
| `pick_right(x_mm, y_mm)` | 右侧抓取 |
| `release(drop_x_mm=0, drop_y_mm=30)` | 释放到指定位置 |

## 6. OriginCalibrator（4 键手动定原点）

```python
from main.arm import OriginCalibrator
from main.api_client import RuntimeApiClient

http = RuntimeApiClient()
OriginCalibrator(http).run(x_wall="left")
```

按键映射：

| 键 | 行为 |
| --- | --- |
| 1 | y 下降 |
| 3 | y 上升 |
| 2 | x 左移 |
| 4 | x 右移 |
| 1+3 同时按 1s | 保存原点到 `arm_origin.yaml` 并退出 |
| 1+3 同时按 3s | 强制退出（不保存） |
| Ctrl-C | 中断（不保存） |

## 7. TrajectoryGenerator（S 曲线 dry-run）

```python
from main.arm import TrajectoryGenerator

gen = TrajectoryGenerator(v_max=150, a_max=400, j_max=2000)
plan = gen.plan_xy(0, 0, 100, 80, sample_hz=50)
print(plan.describe())  # TrajectoryPlan((0.0,0.0) -> (100.0,80.0) mm, T=2.30s, ...)
print(gen.total_time(plan))
print(gen.sample(plan, t_s=1.0).x_mm)  # 任意时刻位姿
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

注意：**S 曲线只用于客户端 dry-run / 日志 / 超时，不下发到硬件**。硬件仍然走车端 PID。

## 8. 底层 `arm.*` 速查（不推荐业务层直调）

| 接口名 | 用途 | 关键参数 |
| --- | --- | --- |
| `arm.reset_position` | 车端整体复位（y 触底 + x 堵转） | — |
| `arm.reset_x` | 车端仅复位 x | — |
| `arm.set_arm_pose` | 一次设置 x/y/arm/hand | `x?` `y?` `arm?` `hand?` |
| `arm.set_hand_angle` | 手爪角度 | `angle` `speed?` |
| `arm.set_arm_angle` | 大臂角度 | `angle` `speed?` |
| `arm.move_x_position` | x 轴定位 | `target`（m） `out_time?` |
| `arm.move_y_position` | y 轴定位 | `target`（m） |
| `arm.goto_position` | 双轴定位 | `x?` `y?`（m） |
| `arm.go_for` | 相对位移 | 偏移（m） |
| `arm.x_speed` | x 轴开环速度 | velocity（m/s） |
| `arm.y_speed` | y 轴开环速度 | velocity（m/s） |
| `arm.grasp` | 吸盘 | bool |
| `arm.x_get_position` | 读 x | m |
| `arm.y_get_position` | 读 y | m |

完整 HTTP/WS 接口和错误码见 [main/API.md](../API.md)。

## 9. 一句话选型

| 需求 | 接口 |
| --- | --- |
| 首次上电手动定原点 | `examples/01_calibrate_origin.py` |
| 之后 reset 原点 | `ArmClient.reset_origin("left")` |
| 双轴同步移动 | `ArmClient.move_xy(...)` / `ArmRunner.move_xy(...)` |
| 单轴移动 | `ArmClient.move_x/move_y` |
| 改大臂方向 | `ArmClient.set_side("LEFT")` |
| 改手爪角度 | `ArmClient.set_hand("DOWN")` |
| 切存储仓档位 | `ArmClient.set_storage("RIGHT")` |
| 读存储仓档位 | `ArmClient.get_storage()` |
| 抓取 | `ArmClient.grasp(True)` |
| 释放 | `ArmClient.grasp(False)` |
| 读位姿 | `ArmClient.get_state()` |
| 算 S 曲线（不下发） | `TrajectoryGenerator().plan_xy(...)` |
| 完整 pick-and-place | `examples/04_grasp_template.py` |

## 10. 存储仓 set_storage 角度范围

存储仓是独立 PWM 舵机（port=1），LEFT/RIGHT 角度写死：

- `STORAGE_DEFAULT_LEFT_ANGLE = -42°`
- `STORAGE_DEFAULT_RIGHT_ANGLE = 90°`

下发时 `ServoPwm` wrapper 把 `angle` 转成 `int(angle/180*180 + 90) = angle + 90`：

- LEFT → 协议值 `48`
- RIGHT → 协议值 `180`（临界）

协议层说明：

- `mc601` 自动 clamp 到 0~180，安全
- `mc602` **不 clamp**，超出 0~180 会触发舵机瞬间回中/回弹
- 历史事故：`RIGHT=165° → 协议值 255`，舵机"摆一下就回弹"

**改角度常量时务必保证 `angle + 90 ∈ [0, 180]`**。否则 `car_wrap_2026.set_storage` 直接抛 `ValueError`。
