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
    OriginCalibrator, # 调车端 reset_position 重新定原点（漂移后手调用）
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
| `reset_y()` | 复位 y（底层走 `arm.reset_position`，会**同时复位 x**） | — | job dict |
| `reset_x()` | 仅复位 x（车端 `arm.reset_x`，不动 y） | — | job dict |
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
    # 注意：以下字段当前未被 ArmClient.get_state() 填充，恒为默认值，使用时务必看注释。
    #  - storage_side     恒为 "LEFT"。请改用 ArmClient.get_storage() 拿真实档位。
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

## 6. OriginCalibrator（调车端 reset_position 重新定原点）

```python
from main.arm import OriginCalibrator
from main.api_client import RuntimeApiClient

http = RuntimeApiClient()
OriginCalibrator(http).run(x_wall="left")
# 底层走车端 arm.reset_position（y 触底 + x 撞墙），完成后读 y_get_position / x_get_position
# 把当前编码器值作为新原点写到 arm_origin.yaml。
```

什么时候用：

| 场景 | 用法 |
| --- | --- |
| 首次上电 | **不需要手动调** —— runtime 启动时若 `RAK_CAR_RESET_ARM=1` 会自动跑一次 |
| 漂移严重 / PID 卡死 / 编码器读数明显不对 | 手跑 `examples/01_calibrate_origin.py left`（或 `right`） |
| 业务代码里临时复位 | `ArmClient.reset_origin(x_wall="left")`（见 [README.md](./README.md) §3） |

> 历史说明：旧版 `OriginCalibrator` 需要在车端按 4 键（1=y 下，3=y 上，2=x 左，4=x 右）手动 jog，再 `1+3` 同时按 1 秒保存 —— **该流程已删除**。当前实现直接下发一次 `arm.reset_position`，由车端 PID 闭环完成触底 / 撞墙，不再监听任何按键。

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
| 首次上电定原点 | 不用手动 —— runtime 在 `RAK_CAR_RESET_ARM=1` 时自动跑；漂移后再手跑 `examples/01_calibrate_origin.py` |
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

## 软限位与行程

### y 轴

- **业务 y 上限**：`soft_y_max_mm` = **200 mm**（默认 200.0），即 `y ∈ [-200, 0] mm`。
- **SDK y_threshold**：`arm_cfg.yaml` 的 `vert_cfg.threshold = [-0.20, 0.0]`,与业务层一致。
- **硬件实测定**:从 0 走到 `-0.20` 仍有余量,未撞顶部机械硬限位。
- **末段减速带**（`y >= -0.015`）:PWM 限幅 `0.02 m/s`,防过冲。

### x 轴

- **业务 x 区间**：`soft_x_min_mm` = **-320 mm**,`soft_x_max_mm` = **+320 mm**,即 `x ∈ [-320, +320] mm`(撞墙=0,远离为正)。
- **SDK x_threshold**：`arm_cfg.yaml` 的 `horiz_cfg.threshold = [-0.32, 0.32]`,与业务层一致。
- **末段减速带**（`x >= wall_pos - 0.02 = 0.32`）:PWM 限幅 `0.04 m/s`,防过冲。
- **顶段减速带**（`x <= -0.02`）:PWM 限幅 `0.06 m/s`,防失步。
- **撞墙判据**(x 无传感器):连续 5 次编码器变化 < 0.5mm 判到墙 + 100ms dwell 确认。
- ⚠️ **历史错配**:`x_threshold` 旧值 `[0, 0.315]`(单调正方向)已修正为 `[-0.32, 0.32]`,同步 `arm_origin.soft_x_min/max_m`。
- **顶段减速带**（`y <= -0.035`）:PWM 限幅 `0.03 m/s`,防失步。**仅 `move_y_position` 往上走时生效**;`reset_y` 永不下行,不走此分支。

> 历史:旧版 `soft_y_max_mm=180` + `threshold=[0, 0.2]`（错配,正方向）已统一改为 `200` + `[-0.20, 0.0]`。

## `reset_y` 行为（**磁感是唯一到底凭证**）

> 这是与过去所有版本最大的区别。务必读懂再用。

### 触发场景

| 入口 | 行为 |
| --- | --- |
| HTTP `POST /v1/execute {"target":"arm","name":"reset_y"}` | 直接调,绕开 `reset_position`（不重启 x/角度） |
| `car.reset_position()` | 内部开线程调 `reset_y` + `reset_x` |
| **`runtime _create_car_locked`** | 每次 init 默认调 `reset_y`（除非 `reset_arm=True` 此时会跑 `reset_position` 包含 `reset_y`） |

### 算法

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

**关键不变量**:
- 找底期间**只有正向速度**(向下),`v > 0`。
- 成功条件**只有真磁感触发 + 50ms dwell**;`y_stop_check` 已被新实现**完全忽略**(旧实现被它误判导致"假到底")。
- 失败(超时/急停/卡死)**不伪归零**——返回 `False`,`y_pose_start` 保持原值,后续 `move_y_position` 会发现偏差并 warn。
- 收工**必 `y_speed(0)`**——绝不残留速度。

### 返回值

- `True`  = 磁感触发 + dwell 通过,已归零
- `False` = 超时/急停/卡死,未归零(y_pose_now 保持搜索前值)

### 失败时的诊断

| 现象 | 看哪 |
| --- | --- |
| 永远 10s 超时 | `arm_base.py reset_y: 找底 %.1fs 超时未触发磁感` → 磁感线/磁铁松脱 |
| 编码器持续不动 | `reset_y: 编码器持续 2.0s 不动, 疑似失步/卡死` → 撞顶失步或机械卡死 |
| 收工 y ≠ 0 | 物理磁感触发点比机械底高 ~10-15mm,属正常,需要绝对精度时看 `ref_encoder` |

## `reset_x` 行为(**撞墙是唯一到墙凭证,v2 模型驱动**)

> x 与 y 关键区别:**x 只有编码器**,没有磁感/限位传感器,撞墙靠**模型驱动 + 物理清零**判定。

### v2 撞墙判据(三层组合,任一持续 200ms 命中即认定)

```
loop:
  cur = x_get_position()
 滑窗 samples(过去 200ms 的 [t, pos])
 期望位移 = |velocity| × dt
 实际位移 = |pos[-1] - pos[0]|

判定条件(主):
  actual_disp / expected_disp < 0.20   # 位移比 = 实测/期望
  # 撞墙/失步 → 实际几乎不动 → 比值→0

判定条件(辅):
  actual_speed / |velocity| < 0.30     # 速度比
  # 防丢步:失步时编码器可能"追着命令走",位移比接近 1.0
  # 但滑窗速度低于命令 → 速度对照异常

一旦触发 + 100ms dwell → 撞墙确认
  → 调 motor_x.motor.reset() 物理清零编码器
  → x_pose_start = 0, x_pose_now = 0
  → ref_encoder = 0
```

### 与 v1 对比

| | v1 (旧) | v2 (现) |
| --- | --- | --- |
| 撞墙判据 | 5×0.5mm stall | 位移比 0.20 + 速度比 0.30,200ms 滑窗 |
| 撞墙后归零 | `x_pose_start = get_dis() - 0.31`(数学补偿) | `motor_x.motor.reset()`(MC602 ctl_id=2 走 encoder_2.reset) |
| 失步误判防护 | ❌ 无 | ✅ 位移比 + 速度对照 |
| 失败行为 | 强制归零 | **不伪归零**,返回 False |

### 可调参数(`arm_cfg.yaml → horiz_cfg → reset_*`)

```yaml
reset_target_m: 0.34          # 撞墙目标距离(应略 > 软上限)
reset_velocity: 0.05          # 撞墙速度(m/s),必须慢
reset_dwell_time: 0.10        # 撞墙后 dwell(秒)
reset_timeout: 8.0            # 总超时
wall_ratio_threshold: 0.20    # 位移比阈值
min_velocity_ratio: 0.30      # 速度比阈值
wall_window_ms: 200           # 滑窗时长
enable_encoder_reset: true    # 撞墙后是否物理清零编码器
```

### 返回值

- `True`  = 撞墙 + dwell 通过 + 编码器物理清零
- `False` = 超时/急停/撞墙条件不满足

### 硬件实测日志

```
INFO: reset_x: 撞右墙+model判据(ratio=0.00,speed=0.0000)+dwell通过,耗时0.17s,enc_物理清零
```

### ⚠️ 修复记录:撞墙门死循环(v3)

**症状**:第一次 `reset_x` 后 `move_x_position` 立刻卡死,任何方向都推不动。

**根因**:旧 `x_speed` 用 `_x_wall` 撞墙门:一旦撞墙 calibrate 设了 `_x_wall="right"`,后续任何 `velocity>0` 都被钳 0 → motor 不动 → `x_stop_check` 命中(编码器不动) → 触发 calibrate → 又设 `_x_wall="right"` → **死循环**。

**修复**(v3):`x_speed` 撞墙门改为**软限位边界自然停**(用 `x_threshold`):
```python
if cur >= x_hi - BOUNDARY and velocity > 0: velocity = 0
elif cur <= x_lo + BOUNDARY and velocity < 0: velocity = 0
```

效果:
- `move_x` 能自由推,到软限位边界 5mm 内自动停
- `reset_x` 设 `target=0.34` 大于软限位 0.32 → 不被 boundary 拦截,继续推到物理右墙
- 不再有死循环,`x_get_position` 真实反映位移

### `move_x_position` 已知问题

`move_x_position` 走 PID + `x_stop_check`,当撞墙/PID 收敛时会 calibrate `x_pose_start`,可能导致 `x_get_position` 短期归零(R2/R5 因为已经在 boundary 内,move_x 不动)。**v4 改造方向:开环 time-based**(同 reset_x 模式)。

### 已知移动问题诊断

| 现象 | 可能原因 |
| --- | --- |
| `move_x(+200)` 完成后 x=0 | 已在 boundary 内,软限位自然停(move_x 没真的推) |
| `move_x(+200)` 6.52s 推 200mm | 正常 ✓ |
| 修复 | 当前 v3 移除了撞墙门死循环,move_x 在 boundary 外可自由走 |

### 5 轮实测日志(R5 关键)

```
INFO: reset_x: 开始,起点 x=0.0000 m (ref=0.2006),_x_wall=right
INFO: reset_x: 撞右墙+model判据(ratio=0.04,speed=0.0021)+dwell通过,
               耗时 0.44s,推了 206.2mm,ref_encoder=0.206153
```

**reset_x 真的能撞到最边**(推 206.2mm)。

## 启动归零（init 流程）

`runtime _create_car_locked` 每次创建 `MyCar` 后:

1. `reset_arm=False`(默认):**只调 `reset_y` + `reset_x`**,不调 `reset_position`(不动 x/角度/底盘)
2. `reset_arm=True`:调 `reset_position` 完整复位(含 `reset_y` + `reset_x`)

为何默认 y/x 归零?
- 控制器重启/USB 重连后 y/x 编码器基准点丢失
- 不归零的话后续 `move_y` / `move_x` 会在错误基准上跑,导致距离全错
- 失败不抛 init(避免 1 次瞬态故障阻断整个 runtime),仅记 `last_error`

## 丢步核对

`move_y_position` 完成后会自动核对:

- 用 `reset_y` 记录的 `ref_encoder` + 累积命令位移 vs 编码器实际位移
- 偏差 > 5mm → `move_y_position 疑似丢步` warn(不抛错,业务层可订阅日志)
- 偏差 > 2mm × N 次 → 建议手动重置原点(`reset_y`)

## 相关文档

- **软件急停 / `reset_y` 找底方向 / HTTP 急停端点**：[SOFTWARE_ESTOP.md](./SOFTWARE_ESTOP.md)
- 子包总览：[README.md](./README.md) · 快速起步：[QUICKSTART.md](./QUICKSTART.md)

## 实时 y/x 位置接口

机械臂 y/x 位置由 runtime 启动时**默认**开启的 `arm_feed` 守护线程持续刷新到 `streamer.arm_state`,然后通过 HTTP / WS 暴露。**所有路径都不抢 car_lock**,调试/UI 轮询 20Hz+ 安全。

### HTTP 一次性读取

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
    "y_m": 0.0,            // SDK 原始坐标(米)
    "x_m": 0.0,
    "y_mm": 0.0,           // 业务坐标(毫米),与 main.arm 一致
    "x_mm": 0.0,           // 业务坐标,负=向左,正=向右
    "ref_encoder": 0.0115, // 最近一次 reset_y 后的编码器零点
    "updated_at": 1784099908.66
  }
}
```

### WebSocket 实时推送(推荐用于调试/UI)

`/v1/ws` 端点新增两个 op,与 `subscribe_lane` 完全同构:

| op | 方向 | 说明 |
| --- | --- | --- |
| `subscribe_arm_state` | client → server | 订阅 arm_state 推送,服务端按 `updated_at` 变化推 |
| `unsubscribe_arm_state` | client → server | 取消订阅,server 关闭推送 task |
| `arm_state` | server → client | 推送数据,字段同 HTTP 响应 |
| `realtime/arm_state` | client → server | 一次性读取(等价于 HTTP) |

### `RuntimeWsClient.subscribe_arm_state` 用法

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

### `RuntimeApiClient.get_arm_state` 一次性查询

```python
from main.api_client import RuntimeApiClient
client = RuntimeApiClient("http://192.168.6.231:5050")
state = client.get_arm_state()["arm_state"]
print(state["y_mm"], state["x_mm"])
```

### 字段语义

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `y_m` / `x_m` | float / None | SDK 内部坐标,**米**,与 `arm.y_get_position()` 一致 |
| `y_mm` / `x_mm` | float / None | 业务坐标,**毫米**,=`y_m*1000` / `x_m*1000` |
| `ref_encoder` | float / None | 最近 `reset_y` 触发磁感时的编码器值,丢步核对用 |
| `active` | bool | `arm_feed` 守护线程是否在跑 |
| `mode` | str | `"arm_feed"`(运行中)/ `"idle"`(已停) |
| `updated_at` | float / None | unix 时间戳,WS 推送用它判"是否新数据" |

> 业务坐标系与 SDK 完全一致:触底/撞墙=0,负=向上/向左(远离触底/远离墙)。

### HTTP/WS 速查

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

### 行为约束

- **不抢 car_lock**:取数据走 `meta_lock`,机械臂长动作期间不卡帧
- **不污染主帧流**:`arm_feed` 不读摄像头、不写 `frames`,仅写 `arm_state` meta
- **init 时自动启**:`runtime _create_car_locked` 默认起 `arm_feed`(20Hz),除非 `arm_feed 启动失败` 才会缺失
- **手动启停**:`car.start_arm_feed(hz=20)` / `car.stop_arm_feed()`,幂等
- **disconnect 自动清理**:WS 断连时 `arm_push_task` 自动 cancel

## 相关文档

- **软件急停 / `reset_y` 找底方向 / HTTP 急停端点**：[SOFTWARE_ESTOP.md](./SOFTWARE_ESTOP.md)
- 子包总览：[README.md](./README.md) · 快速起步：[QUICKSTART.md](./QUICKSTART.md)
