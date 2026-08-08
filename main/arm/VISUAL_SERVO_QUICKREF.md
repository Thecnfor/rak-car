# 视觉伺服算法速查（1 页业务版）

> 接手 5 分钟读完。能讲清楚：**算法在做什么、为什么这样做、不能做什么、还能怎么改**。
> 本文是视觉伺服的**唯一概述**（早期 DESIGN/PLAN/REALTIME 设计文档已随实现完成而删除，git 历史可查）。

---

## 1. 硬件 ↔ 算法 映射（一图说清）

```
机械结构                                 算法层只看到
─────────────────────                   ──────────────
xy 十字滑台（垂直于大地）  ──►  x_mm, y_mm ∈ mm           ← 视觉伺服闭环**只动这两个**
大臂电机 -90°~+90°           ──►  arm_angle ∈ 度（连续）  ← composite_run 入口快照，PID 期间冻结
手抓 0°~180°                 ──►  hand ∈ 度（连续）        ← 同上；"UP/MID/DOWN"是业务 enum 别名
```

**关键事实**：算法层**没有 IK**（没有反运动学）。`(arm_angle, x, y, hand)` 是 4 个独立的目标值，**打包发给 SDK**（`composite_run` 内部用 ThreadPoolExecutor 让 4 个 motor 并行）。比赛场景不需要 IK 解算，避免奇异点失败。

---

## 2. 算法栈（5 层 + 2 个传输路径）

```
L5:  MC602 SDK ──── goto_position(arm, x, y, hand) ──► 4 个 motor 真并发
L4:  Composite ──── composite_run(...) + _check_safe ─► 软限位网
L3:  S 曲线 Dry ── plan_xy(x0,y0 → x1,y1) ─► 给 L4 估 timeout
L2:  视觉伺服 ──── PID + depth-aware + 4-DOF ──► (dx_mm, dy_mm) ∈ mm
L1:  解析选择 ──── bbox parse + TargetSelector ──► Detection {bbox_norm ∈ [-1,+1], bbox_px}

L2 的传输有四条：
  · HTTP /v1/vision/task cache（30Hz 轮询） ──► find_target / find_target_pid / find_target_legacy
  · WS subscribe_task_detection（推流 ~25-60ms）─► find_target_realtime / find_target_track
  · velocity 模式（推荐高频追踪, 免 arm_queue）─► find_target_velocity (XY) / find_target_4dof
    （2026-08-02 封装, 原 07/08 示例抽成 VelocityLoop; ArmRunner.track_velocity / track_4dof 编排）
  · 进程内闭环（2026-08-09 闭环下沉, **零每帧网络**）──► runtime 读 task_feed 缓存 + 直调
    arm.x_speed/set_arm_angle, main 只发一次 `run_arm_servo` 等结果
    （task2 水立方: `task_config.yml → pick_vision.local_servo: true` 走这条;
      `false` = 退回上面的 velocity 网络闭环, 每条任务只加这一个开关, 旧路径原样保留）

切换说明（2026-08-09）:
  - **arm**: `task_config.yml` `pick_vision.local_servo: true/false` → true=进程内闭环(runtime),
    false=旧每帧网络闭环(track_velocity_pick)。判断在 `task2_water_tower.py::_pick_cube`。
  - **底盘**: `track_chassis()` 函数内自动路由——**不传 `sense_fn`** 走 runtime 闭环
    (`POST /v1/realtime/chassis-align`, 一次 HTTP), **传了 `sense_fn`**（task6 LLM-as-servo）
    走旧 client 闭环。task 侧零改动。
  - ⚠️ 前置: 两个闭环都读 `task_feed` 检测缓存（init 默认 30Hz 启动）, cam2 必须活着、
    画面里有目标 label, 否则伺服直接找不到目标 → 超时。
```

详细函数签名看 `ARM_API.md §0-§10`，单测看 `tests/test_*.py`。

---

## 3. 当前算法（3 块数学）

### 3.1 像素深度（单目针孔）

```python
D = target_real_height_m × focal_length_px / bbox_height_px
         ↑                       ↑
         来自 task_config.yml    DEFAULT_FOCAL_LENGTH_PX = 600.0  ⚠️ 见 §4
```

代码：`vision/__init__.py:44` `ArmVisionClient.compute_depth()`。

### 3.2 深度自适应 PID 增益

```python
mm_per_norm_eff = mm_per_norm_base × (D / ref_depth_m)
                  ↑ 30mm/单位          ↑ 0.30m 参考深度

out = kp·err + ki·∫err + kd·derr   (饱和限幅 ±1.0)
dx_mm = -out × mm_per_norm_eff      (负号: bbox 偏右 → 末端向左追)
```

直觉：**目标越远 bbox 越小** → `D` 越大 → `mm_per_norm_eff` 自动放大 → 步长自动匹配"远距离需要走更多 mm 才能让 bbox 同样变小"。

默认增益（`vision/servo.py:110`）：`kp=1.0, ki=0.05, kd=0.2`（`ki≠0` 是为了消除稳态误差；`kd=0.2` 抑制超调）。

**收敛判定**：连续 `settle_stable_frames=3` 帧满足 `|x_center|, |y_center| ≤ 0.05` 才算收敛（防单帧抖动假报）。

### 3.3 4-DOF 策略（decouple 设计）

```python
if abs(dx_norm) > arm_dx_threshold_norm:    # 默认 0.3
    on_strategic_4dof("arm_rotate", pick)   # 回调,不直接调 arm
```

**算法层不直接动 arm/hand**——而是触发回调让业务层决定。这是解耦：急弯场景下大臂要怎么跟（角度多少、速度多少、是否需要先回安全位）都是场地几何问题。

代码：`vision/servo.py:204` 的 `find_target_pid`；入口在 `vision/__init__.py:25` 的 `ArmVisionClient`。

---

## 4. 算法能做 / 不能做（**这是接手时必须看的一节**）

| ✅ 能做 | 算法 | 工程化 |
|---|---|---|
| bbox 居中 + settle | PID + depth-aware gain | 6cm 果实与 30cm 水桶指示牌都能 N 帧内收敛 |
| 多目标选一 | 8 种 `SelectionStrategy` | `highest_score` / `closest_to_center` / `lock_first_seen` / ... |
| 4 自由度混合 | `composite_run(arm, x, y, hand)` 并行 | ThreadPoolExecutor 真并发 |
| 软限位网 | `_check_safe(y_mm) + _check_y_protected()` | 不通过直接 raise，不下发 |
| 持续追踪不收敛 | `find_target_track` | 仅 timeout 后返回，trace 整段可回放 |

| ❌ 不能做（**当前没实现**） | 原因 |
|---|---|
| **相机焦距自标定** | `focal_length_px=600` 是启发式默认；实际值未在线标定（PR 见 §5） |
| **多目标深度排序** | 用单个 bbox 算 depth；多目标之间的相对深度没有算法 |
| **运动视差 / 光流深度** | 需要场景+相机相对运动，工程量大；当前任务都是相机静态 |
| **MonoDepth 神经估计** | 没有引入 PaddleDetection 的 monodepth 模型 |
| **IK 解算** | 故意不做；4 自由度直接下发，避免奇异点 |
| **arm_angle 在 PID 期间连续跟随** | 当前冻结，由 `on_strategic_4dof` 异步回调解耦（业务层在 PID 间隙手动调） |

### 4.1 `focal_length_px=600` 的精度影响 ⚠️

| 真实焦距 | 报距 0.30m 处 6cm 目标时 | 实际 `mm_per_norm_eff` | 影响 |
|---|---|---|---|
| 400px（实际更广角） | 真距 ≈ 0.45m | 比预期大 50% | 增益过大 → **震荡超调** |
| 600px（默认启发值） | 真距 0.30m | 标准 | 标定后会比这好或差 |
| 800px（实际更长焦） | 真距 ≈ 0.225m | 比预期小 25% | 增益过小 → **收敛慢** |

**实装前必做**：拿一个已知尺寸物体放在已知距离，单帧 snap 读 `bbox_pixels.height`，反算 `f_real`，写回 `arm_origin.yaml` 或业务参数。

---

## 5. 改进路线（PR 级体量，独立可拆）

| 方向 | 体量 | 依赖 | 价值 |
|---|---|---|---|
| **C1: 相机自标定任务** | 0.5 day | 现有 `snap` API + yaml 读写 | 立竿见影：消除 §4.1 误差，比赛上场前必做 |
| **C2: 卡尔曼滤波 `depth_m`** | 0.5 day | `compute_depth` 加 `SimpleKalman` | 单帧 bbox 抖动被平滑；远目标收敛更稳 |
| **C3: 4-DOF PID 重构** | 1-2 days | 新写 `find_target_4dof_pid`，4 维误差向量 | 大幅改进急弯场景下大臂跟随质量（需要重写 composite_run 的"快照"模型） |
| **C4: MonoDepth 神经估计** | 3-5 days | 引入 PaddleDetection monodepth 模型 | 不再需要每任务先验 `target_real_height_m`；通用测距 |
| **C5: 多目标深度排序** | 1 day | C2 + 检测器输出多目标 | "前景果实 vs 背景储物架"场景下的优先级排序 |

**推荐路径**：比赛前先做 **C1**（半天平），比赛后做 **C2**（提升稳定性），长期做 **C4**（降先验依赖）。

---

## 6. 关键文件指针

| 文件 | 看什么 |
|---|---|
| `vision/__init__.py:25` | `ArmVisionClient` 聚合类 |
| `vision/servo.py:98-110` | `find_target` 自动路由 legacy ↔ PID |
| `vision/servo.py:215-238` | depth-aware gain + PID 完整数学 |
| `vision/realtime.py:55+` | WS 路径；`find_target_realtime` vs `find_target_track` 在尾部 |
| `api/vis_servo.py:25-47` | `_make_vision_with_move()` 安全 wrap |
| `loops/runner.py:198-309` | 高层 `move_to_vision_target_*` / `pick_by_vision_*` / `track_vision_target` |
| `trajectory.py:148-285` | S 曲线规划（dry-run） |
| `api/composite.py:56` | `composite_run` 4 路并行入口 |

## 7. 单测覆盖（验证用）

```bash
/usr/bin/python3 -m unittest discover -s main/arm/tests -p "test_*.py" -v
```

**19 个测试文件 / 141 用例，关键 4 个**：
- `test_servo_depth.py` — `compute_depth` 边界（bbox=0、target=0、bbox=None、正常）
- `test_servo_pid.py` — PID 响应
- `test_servo_4dof.py` — 急弯策略
- `test_vision_realtime_safety.py` — `_make_vision_with_move` 必须 wrap 两个方法（PR#13 回归）

## 8. 详见其他文档

| 你想看 | 看哪里 |
|---|---|
| 完整业务 API | `ARM_API.md` §0-§10 |
| WS 实时推送端点 | `ARM_API.md` §10.7 |
| chassis→arm 联调标定 | `loops/orch_visual.md` |
| 子包总览 + 10 行上手 | `README.md` |
| 实机 checklist | `TEST_PREFLIGHT.md` |
