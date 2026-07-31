# debug-task4-rebuild-2026-07-28.md

**会话日期**：2026-07-28
**用户硬约束**：只能改 `main/**` 业务层；`smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py` **暂不能动**
**分支**：`am` (HEAD `ac1413b`)
**主要工作**：task4 重建 v3 — target1/target2/target3/target4 全套改 + 闭环控制 + BALL_VERIFIED 重新校准

---

## 1. 主要调用的文档 / 源码

### 文档 (CLAUDE.md 提到的入口 + 这次新确认的)
- `CLAUDE.md` — 项目主入口文档
- `main/arm/README.md` + `main/arm/ARM_API.md` + `main/arm/QUICKSTART.md` — 机械臂业务层 API
- `runtime/README.md` — runtime 服务（含并发任务模型）
- `runtime/VISION_API.md` — 视觉相关端点
- `debug-x-axis-rollout-v3.md` — x 轴 2026-07-17 session 总结
- `debug-task5-rebuild-2026-07-22.md` — task5 重建总结
- `debug-belt-slip-checklist.md` — x 轴同步带打滑
- `main/arm/each_task/task4/__init__.py` — task4 业务流图 (a/b1/b2/b3/c)

### 源码 (按调用频率)
- `main/arm/api.py` — `ArmClient` (1144 行)
- `main/arm/each_task/task4/constants.py` — 业务常量 (含 BALL_VERIFIED_*)
- `main/arm/each_task/task4/target1.py` / `target2.py` / `target3.py` / `target4.py` — 4 个任务脚本
- `runtime/services/my_car.py` — `MyCar`, `start_task_feed`, `grasp` 底层
- `runtime/services/camera_stream_service.py` — cam2 帧缓存 + 任务检测结果
- `runtime/services/runtime_service.py` — runtime service + auto-init + job queue
- `runtime/core/actions.py` — `ARM_ACTIONS["grasp"]` 注册 (lambda 透传 kwargs)
- `smartcar/whalesbot/vehicle/arm/arm_base.py:780` — `grasp(value: bool)` 签名
- `smartcar/whalesbot/vehicle/base/controller_wrap.py:501` — `PoutD` 数字输出

---

## 2. 主要改动汇总

### 2.1 `main/arm/api.py` — 修了 2 个 bug

**Bug 1: `ArmClient.grasp` 静默失败** (api.py:732-747)
- **症状**：业务层 `runner.grasp(True)` 返回 failed dict 不抛异常，球没吸起来
- **根因**：原来 `_call_arm("grasp", on=bool(on), ...)` 让 `kwargs={on, sync, timeout}` 整个透传到 `arm_base.grasp(self, value)`，签名只接 `value`，三个 kwarg 全 TypeError
- **修法**：直接调 `self.http.execute_arm_action("grasp", bool(on), timeout=..., sync=True)`，绕开 `_call_arm`（其 timeout 位置参陷阱）和 keyword 透传陷阱
- **踩坑**：之前试 `_call_arm("grasp", bool(on), ...)` 也不行 —— `_call_arm` 签名是 `(name, timeout=20.0, *args, ...)`，`bool(on)` 位置参被 timeout 抓住报 `multiple values for argument 'timeout'`

**Bug 2 (未修，留给下个 session)**: `set_storage_angle` 缺 `job =` (api.py:720-729)
```python
self._call_car("set_storage_angle", timeout=timeout, angle=angle, speed=speed, sync=True)
# ↑ 缺 job = 赋值
self._storage_side_cache = "UNKNOWN"
return {
    "ok": bool(isinstance(job, dict) and job.get("status") == "succeeded"),
    "raw_job": job,  # ← job 未定义 → NameError
}
```
HEAD `ca896c2` 引入时就有，调一次崩一次。**今天没动**，待修。

**顺带加的**：
- `class ArmSafetyError(ValueError)` — 业务层安全门拦截细分异常类
- `pre_init_close_storage()` — 任何 init 入口前预关仓 (y gate 干扰磁感找底)
- 修复 `move_x` `v_max_mms` 透传 (之前被吞，target1.x 限速不生效)

### 2.2 `main/arm/each_task/task4/constants.py` — BALL_VERIFIED_* 重新校准

**6 次黄色球实测 (target1 位姿下, y=-150)**：

| 字段 | 旧 (5 次) | 新 (6 次含现场补测) | 改 |
|---|---|---|---|
| cx | [0.05, 0.18] | 同 | — |
| **cy** | [-0.67, -0.58] | **[-0.68, -0.58]** | CY_MIN -0.67→-0.68 |
| w | [0.40, 0.44] | 同 | — |
| h | [0.59, 0.62] | 同 | — |
| area | [0.24, 0.27] | 同 | — |
| **score** | ≥ 0.92 | **≥ 0.80** | SCORE 0.92→0.80 |
| aspect | [0.60, 0.80] | 同 | — |

**现场补测数据**：cx=+0.087, cy=-0.675, w=0.423, h=0.611, area=0.258, score=0.924, aspect=0.692

**TARGET_* 球过滤阈值 (4 项, 收紧)**：
- `TARGET_SCORE_MIN`: 0.5 → 0.85
- `TARGET_ASPECT_TOL`: 0.4 → 0.8
- `TARGET_AREA_MIN`: 0.003 → 0.20
- `TARGET_AREA_MAX`: 0.20 → 0.30

### 2.3 `main/arm/each_task/task4/target1.py` — y 几次调整

| 时间 | y 值 | 备注 |
|---|---|---|
| 早上 | -200 | 默认 arm_base y 软限位 |
| 中午 | -175 | 25mm buffer |
| 下午 | -150 | 50mm buffer |
| **傍晚 (现在)** | **-125** | "用户实测球检测在 y=-125 落点更稳" |

⚠️ **target1 改 y=-125 但 BALL_VERIFIED_* 还是按 y=-150 校准的**。建议现场用 target1 (新 y=-125) → target2 重新测球位置，更新 BALL_VERIFIED_* 基线。

### 2.4 `main/arm/each_task/task4/target3.py` — return y 改

- `RETURN_Y_MM`: -200.0 → -150.0
- docstring + argparse description 同步更新

### 2.5 `main/arm/each_task/task4/target2.py` — verify_target1_pose 默认翻转

- `fetch_balls` / `step_target2_once` / `step_target2_loop` 默认 `verify_target1_pose=True → False`
- CLI flag: `--no-verify-target1-pose` → `--verify-target1-pose` (正向 opt-in)
- main() 去掉 `not` 取反

**根因**：默认 True 太严，target2 是通用检测查看器，多数场景不在 target1 位姿，开启 BALL_VERIFIED_* 过滤会误伤（现场球在 cam2 右上方 cy=-0.20 不在 [-0.67, -0.58] 直接被滤掉）

### 2.6 `main/arm/each_task/task4/target4.py` — 状态机重构 + ADJUST 闭环 (大头)

**新增 13 个常量**：
```python
DEFAULT_CHASSIS_OSCILLATE: bool = False  # SEARCH 只前进, 后退只给 ADJUST
DEFAULT_VERIFY_TARGET1_POSE: bool = True
ADJUST_X_STEP_MM: float = 10.0
ADJUST_CHASSIS_STEP_M: float = 0.02
ADJUST_DEADBAND_X: float = 0.01
ADJUST_DEADBAND_Y: float = 0.01
ADJUST_MAX_NO_PROGRESS: int = 6
ADJUST_MAX_ROUNDS: int = 40
ADJUST_X_SIGN: int = +1     # ⚠️ 旧 -1 推球方向反了
ADJUST_CHASSIS_SIGN: int = +1
DEFAULT_SCAN_MODE: str = "sweep"   # 旧 "oneway" 端点 break
DEFAULT_MAX_X_CYCLES: int = 5
```

**三段式状态机** (2026-07-28 v2):
```
SEARCH   (没识别到球)    → 底盘前移 5cm + x 扫描
ADJUST   (识别到颜色但没进 BALL_VERIFIED_*) → PD 微调底盘+x 把球移到抓取位
LOCKED   (球进 BALL_VERIFIED_*) → 连续 confirm_frames 帧命中 → 退出
```

**ADJUST 公式 (现场确认方向)**:
```python
target_cx = (BALL_VERIFIED_CX_MIN + BALL_VERIFIED_CX_MAX) / 2  # 0.115
target_cy = (BALL_VERIFIED_CY_MIN + BALL_VERIFIED_CY_MAX) / 2  # -0.63

cx_err = ball.cx - target_cx   # 正: 球在目标右侧
cy_err = ball.cy - target_cy   # 正: 球在目标下方

# 推导 (现场验证):
#   arm_x 更负 → 相机更靠左 → 球在画面里更靠右 → cx ↑
#   球偏右 (cx_err>0) → arm_x 朝正向 (向 0) → x_delta > 0
#   球偏左 (cx_err<0) → arm_x 朝负向 (向 -280) → x_delta < 0
x_delta = ADJUST_X_SIGN * sign(cx_err) * ADJUST_X_STEP_MM   # ADJUST_X_SIGN=+1

# 底盘: 前移 → 球相对往后 (cy ↑); 后退 → 球相对往前 (cy ↓)
#   球偏下 (cy_err>0) → 底盘后退 → c_delta < 0
#   球偏上 (cy_err<0) → 底盘前进 → c_delta > 0
c_delta = ADJUST_CHASSIS_SIGN * (-sign(cy_err)) * ADJUST_CHASSIS_STEP_M  # CHASSIS_SIGN=+1
```

**x 扫描修复**:
- 旧 `oneway` 模式到端点 `break` → 现到端点翻转方向 + `n_x_cycles++`，满 5 圈才放弃
- 默认改 `sweep` 模式自动 bounce

**移动后立即重检测** (LOCKED 兜底):
```python
# ADJUST 移动一步后 0.15s 立即重 fetch + verify
# 防止球穿过 BALL_VERIFIED_* 一帧 (0.5s step_interval) 来不及命中就跳出
```

**修的 2 个调用错误**:
1. `runner.move_x(x_mm, v_max_mms=30.0, ...)` → `ArmRunner.move_x` 不接 `v_max_mms` → 改成 `runner.move_x(x_mm, timeout=30.0)`
2. `chassis_client.move_for(...)` → `ChassisClient` 没有 → 改成 `chassis_client.http.execute_car_action("move_for", [step_m, 0, 0], sync=True, timeout=10.0)`

**修的 1 个 import 漏**:
- 加 `BALL_VERIFIED_CX_MIN/MAX, BALL_VERIFIED_CY_MIN/MAX` 到 from .constants import (之前 L411 `NameError`)

**新增 CLI flags**:
- `--chassis-oscillate` / `--no-chassis-oscillate`
- `--no-verify-pose`
- `--flip-cx-sign` / `--flip-cy-sign`
- `--max-x-cycles`

---

## 3. 当前状态 (今天 2026-07-28 改完)

| 文件 | 状态 |
|---|---|
| `main/arm/api.py` | grasp 修好, set_storage_angle 仍坏 (没改) |
| `main/arm/each_task/task4/constants.py` | BALL_VERIFIED_* 已校准 (按 y=-150), 待按 y=-125 重新校准 |
| `main/arm/each_task/task4/target1.py` | y=-125 (最新) |
| `main/arm/each_task/task4/target2.py` | verify_target1_pose 默认 False |
| `main/arm/each_task/task4/target3.py` | return y=-150 |
| `main/arm/each_task/task4/target4.py` | 状态机 SEARCH/ADJUST/LOCKED, 13 个新常量, x 扫描不 break |
| `main/arm/each_task/task4/test_blue.py` / `test_yellow.py` | 没动 |

---

## 4. 已知 / 面临问题 (按优先级)

### 🔴 P0：target1 改 y=-125 后 BALL_VERIFIED_* 还没重校准
- 当前 BALL_VERIFIED_* 13 个常量基于 y=-150 6 次实测
- target1 已用 y=-125，球在画面里的 cx/cy/w/h 都可能偏移
- target4 ADJUST 阶段按这套基线找"抓取位"，可能球到了 y=-125 实际位置但不在 BALL_VERIFIED_* 区间内
- **修法**：现场跑 target1 (新 y=-125) → target2 重新测，更新 BALL_VERIFIED_*

### 🟡 P1：set_storage_angle 在 api.py:720-729 缺 job = 赋值
- HEAD `ca896c2` 引入时就有
- 调一次崩一次 (NameError)
- 业务层没用到所以没人发现
- **修法**：1 行加 `job = self._call_car(...)`

### 🟡 P2：ChassisClient 没 move_for 包装
- task4 forward 模式必须走 `chassis_client.http.execute_car_action("move_for", ...)`
- 业务层用着别扭，但能跑
- **修法**：在 `main/chassis/api.py` 加 `move_for(displacement, timeout)` 包装，调内部 http

### 🟢 P3：grasp 一致性
- 现在 api.py 用 `http.execute_arm_action` 直调绕开 _call_arm
- 跟代码其他地方风格不一致
- **修法**：可选地把 _call_arm 签名改成 timeout keyword-only（动架构层）

### 🟢 P4：verify_target1_pose=True 时机不明
- target2 CLI 默认 False
- target4 ADJUST LOCKED 判定本地用 `arm_client.verify_ball(b)`，相当于 verify_target1_pose=True
- 没有显式 CLI 入口跑"在 target1 位姿下批量验证"
- **修法**：可选加 `target2 --verify-target1-pose` 跑 + 把 task2 末轮改成"先跑到 target1 位姿再 verify"

---

## 5. 准备在解决的 (下一步)

1. **BALL_VERIFIED_* 按 y=-125 重校准** (P0, 现场跑)
   - 跑 `target1.py` (y=-125) → `target2.py` 测黄球 cx/cy/w/h
   - 改 `constants.py` 13 个常量
   - ADJUST 的 `target_cx/target_cy` 自动重算

2. **修 set_storage_angle 缺 job =** (P1, 1 行)
   - 直接加 `job = self._call_car(...)`

3. **现场调试 ADJUST 方向** (验证 ADJUST_X_SIGN=+1 正确)
   - 跑 `target4.py --dry-run --no-verify-pose` 看 ADJUST 是否收敛
   - 跑 `target4.py --color yellow` 真动
   - 如果还不收敛就用 `--flip-cx-sign` / `--flip-cy-sign` 现场调

4. **task4 完整流程联调** (P?)
   - a_approach → target1 → target2 → target3 → target4 (找球) → target3 (抓) → b3_store
   - 用 target4 ADJUST + LOCKED 验证 SEARCH→ADJUST→LOCKED 状态机跑通

---

## 6. 注意事项 (下次 session 必读)

### 用户硬约束
- ⚠️ **只能改 `main/**` 业务层**；`smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py` 暂不能动
- 任何改 API / 业务逻辑都行；改硬件 SDK / runtime 服务 / 顶层入口都禁止

### 几个被踩过的坑 (避免重蹈)
1. **`ArmRunner.move_x(x_mm, v_max_mms=...)` 会 TypeError** — `v_max_mms` 是 `ArmClient.move_x` 参不是 `ArmRunner.move_x`
2. **`ChassisClient` 没有 `move_for`** — 走 `chassis_client.http.execute_car_action("move_for", ...)`
3. **`ArmClient._call_arm` 的 timeout 是位置参** — 不能 `_call_arm("grasp", bool(on), ...)`，会 multiple values
4. **`runtime ARM_ACTIONS["grasp"]` 透传所有 kwargs** — 只能位置参 `bool(on)`，不能关键字
5. **`x_get_position` 走坏掉的 calibrate 框架** — 业务层必须走 `_read_x_mm_realtime()` (arm_feed 20Hz)
6. **`_call_car` 默认 sync=False, `_call_arm` 默认 sync=True** — 写新方法时注意区分
7. **BALL_VERIFIED_* 是位姿特定的** — 改 target1 y 之后必须重新校准
8. **x 扫描默认 sweep 不要改 oneway** — oneway 端点会 break 提前退出

### 几个常用模式
- **走 realtime 读 x/y 真值**：`client._read_x_mm_realtime()` / `client._read_y_mm_realtime()`
- **走 task_feed 读球检测**：`http.get_task_state() → task_state.detections`
- **改 ball 颜色映射**：`_label_to_color` 在 target2.py
- **改过滤阈值**：`constants.py` 顶部 TARGET_* + BALL_VERIFIED_*
- **走 runtime CAR_ACTIONS**：`chassis_client.http.execute_car_action(name, args, kwargs, sync, timeout)`
- **走 runtime ARM_ACTIONS**：`client._call_arm(name, timeout, *args, sync, **kwargs)`（grasp 除外）

### 重要调试入口
- 现场 trace: `pm2 logs rak-car-api --lines 200 | grep -iE "task_feed|cap_side|frame missing"`
- task_feed 状态: `curl http://192.168.6.231:5050/v1/realtime/vision/task`
- 整体健康: `curl http://192.168.6.231:5050/v1/health`
- task_state 缓存: `curl http://192.168.6.231:5050/v1/state | python -m json.tool`
- 视觉预览: `http://192.168.6.231:5050/stream/`

---

## 7. 内存 / 文档同步建议

下次 session 开始时建议先:
1. 读本文件 + `debug-x-axis-rollout-v3.md` (2026-07-17) + `debug-task5-rebuild-2026-07-22.md` (2026-07-22)
2. 跑 `git log --oneline -5` 看 HEAD
3. 跑 `git status --short main/arm/each_task/task4/` 看未提交改动
4. 如果要做 P0 (BALL_VERIFIED 重校准) 直接按 §5 步骤走
