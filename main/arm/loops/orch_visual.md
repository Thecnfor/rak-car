# 视觉联调编排器（orch_visual.py）

> `VisualOrchestrator` — chassis 追踪 + arm 4-DOF + 抓取，一条龙。
> 2026-08-02 新增。

## 坐标系映射（2026-08-02 现场标定）

| 层 | 画面轴 | 车/臂运动 | 误差含义 |
|---|---|---|---|
| **Chassis** | cx（横向） | 车前后 vx | cx 负（画面左）→ vx 负（后退）|
| **Chassis** | cy（纵向） | 车横向 vy | cy 负（画面上）→ vy 正（右移）|
| **Arm 4-DOF** | cx（横向） | 大臂角度 arm_angle | dx>0 → arm 更负（sign_arm=+1）|
| **Arm 4-DOF** | cy（纵向） | x 十字位置 | dy>0 → x 往左（sign_x=-1）|

## 三阶段流水线

```
Stage 1:  track_chassis()         → 把目标拉到画面中心（chassis 两自由度）
Stage 2:  track_velocity_pick()    → 臂结构映射，4-DOF 精准对齐到吸嘴 setpoint
Stage 3:  y 降到 0 → grasp(True)  → 吸气
```

## 快速开始

```python
from main.arm.loops import VisualOrchestrator

orch = VisualOrchestrator()

# 方式 A：一条龙（chassis → arm → grasp）
result = orch.track_and_grasp(
    "h_tu_dou",               # 目标 label
    chassis_max_seconds=15.0,  # Stage 1 超时
    arm_timeout=30.0,         # Stage 2 超时
)
print(result.arrived_chassis, result.arrived_arm, result.grasp_ok)

# 方式 B：单步用
orch.chassis_only("h_tu_dou")        # 只 Stage 1
orch.arm_only("h_tu_dou")            # 只 Stage 2
orch.grasp(y_mm=0.0)                  # 只 Stage 3
```

## Stage 1 — chassis 追踪

```python
from main.arm.loops import VisualOrchestrator
orch = VisualOrchestrator()

r = orch.align_chassis(
    "h_tu_dou",
    sign_vx=-1, sign_vy=+1,   # 2026-08-02 现场标定
    kp=0.20, v_max=0.12,
    deadband=0.08, hold_frames=5,
    max_seconds=15.0,
    dry_run=True,               # True=只看不发车
)
print(r.arrived, r.reason, r.final_frame.cx, r.final_frame.cy)
```

**方向反了？**

| 现场问题 | 改参数 |
|---|---|
| 画面左/右反了 | `sign_vx=+1` |
| 画面上/下反了 | `sign_vy=-1` |

## Stage 2 — arm 4-DOF

```python
r = orch.align_arm(
    "h_tu_dou",
    x_start=0.0, y_start=-180.0,   # 起始姿态（y=-180=吸嘴朝前）
    arm_start=-90.0, hand_start=0.0,
    sign_arm=1.0, sign_x=-1.0,    # 2026-08-02 现场标定
    gain_arm=0.4, gain_x=0.08,
    deadzone=0.02,
    settle_hits=3,                 # 连续 3 帧命中 → 认为对齐
    timeout=30.0,
)
print(r["ok"], r["reason"], r["trace_hits"])
```

**吸嘴 setpoint**：自动读 `origin.nozzle_offset_for(label)`，按 label 查表补偿。

**方向反了？**

| 现场问题 | 改参数 |
|---|---|
| arm 旋转方向反了 | `sign_arm=-1.0` |
| x 十字移动方向反了 | `sign_x=+1.0` |

## Stage 3 — 抓取

```python
# 自动在 Stage 2 完成后调用（track_and_grasp 内部）
orch.grasp(y_mm=0.0)          # y 降到 0 → 吸气
orch.grasp(y_mm=0.0, dry_run=True)  # 只看不真动作
```

## OrchResult 返回值

```python
@dataclass
class OrchResult:
    arrived_chassis: bool              # Stage 1 chassis 到达
    reason_chassis: str              # arrived / timeout / no_target / watchdog
    arrived_arm: bool              # Stage 2 arm 对齐
    reason_arm: str                # ok / timeout / failed
    grasp_ok: bool                # Stage 3 吸气成功
    trace: List[OrchFrame]         # 每帧状态（chassis+arm）
    elapsed_s: float               # 总耗时
    chassis_result: TrackChassisResult
    arm_result: dict
```

## 按场景拆用

```python
# 只做 chassis 追踪（不抓）
orch.chassis_only("h_tu_dou")

# arm 4-DOF 对齐（车已经到位）
orch.arm_only("h_tu_dou")

# chassis 追踪 + arm 对齐，不抓
result = orch.track_and_grasp(
    "h_tu_dou",
    skip_grasp=True,
    arm_timeout=30.0,
)
# result.arrived_chassis / arrived_arm 都有了

# 只做 grasp（arm 已经在位）
orch.grasp(y_mm=0.0)
```

## 调试技巧

```python
# Stage 1 dry-run（不发车）
orch.track_and_grasp("h_tu_dou", chassis_dry_run=True)

# Stage 2 dry-run（不下发 grasp）
orch.track_and_grasp("h_tu_dou", skip_chassis=True, arm_dry_run=True)

# 两个都 dry
orch.track_and_grasp("h_tu_dou", chassis_dry_run=True, skip_grasp=True)
```
