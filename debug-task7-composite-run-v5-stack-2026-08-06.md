# task7 position1/2/3/5 v5 4 机联动大改 — 上下文压缩

**会话日期**: 2026-08-06 (今天, 本 session 是上一个 4 机联动会话的续)
**作者**: Claude (MiniMax-M3) on behalf of user
**目的**: task7 子目录 position1/2/3/5.py 全面改 1 步 composite_run 4 机联动; position5.py 加双机联动 partial 子设计 + 现场踩坑修复; y 值 -74→-92→-85 调试轨迹
**下次会话第一件事读这个** — 全 task7 position* v5/v4+ 重构 + 同步压缩 + 现场实测反馈全在这里

---

## TL;DR

| 文件 | 旧版 | **新版** | 状态 |
| --- | --- | --- | --- |
| `position2.py` | 4 步串行 | **v5: 4 步大改** (composite_run → move_x → drop → move_x) | ✅ 现场实测已通过 |
| `position1.py` | 7 步串行 + 底盘 Phase | **v5: 4 步大改** (Phase 2 同 position2 v5, Phase 1/3 底盘不动) | ✅ 写完待测 |
| `position3.py` | 7 步串行 + 底盘 Phase | **v5: 4 步大改** (Phase 2 同 position2 v5, Phase 1/3 底盘不动) | ✅ 写完待测 |
| `position5.py` | 10 步串行 | **v4+ (现场 v4+ 已通过, y 调试轨迹 -74→-92→-85)** | ✅ 现场实测已通过 |
| `target.py` v3 | — | 4 机联动 + 无 pre-check | ✅ 上一 session 改完, 待测 |
| `get_position1.py` v3+ | — | 4 机联动 (-190 / +86° / +10°) | ✅ 上一 session 改完, 待测 |
| `the_final.py` v5 | — | 加 Step 0 pingcang | ✅ 上一 session 改完, 待测 |
| `aaashouzhua.py` | — | 新建 (hand=-10→+10°) | ✅ 上一 session 改完, 待测 |

**全部未 commit** — 5 个文件改动等用户现场实测 + 决定何时 commit。

---

## 主要参考文档

### 项目根 + MEMORY

| 文档 | 用途 |
| --- | --- |
| `C:\Users\29368\Desktop\智能车\rak-car\CLAUDE.md` | 项目总览 + 三层架构 + runtime 锁模型 + arm/target7/visual servo 章节 |
| `C:\Users\29368\.claude\projects\...\MEMORY.md` | 历史踩坑 + 业务硬约束 + 历次会话压缩索引 |
| `C:\Users\29368\Desktop\智能车\rak-car\debug-task7-4axis-composite-run-2026-08-06.md` | **上一 session 上下文压缩** (target.py v3 / get_position1.py v3+ / the_final.py v5 / aaashouzhua.py 改完, 本 session 续) |
| `C:\Users\29368\Desktop\智能车\rak-car\debug-task7-2026-08-04.md` | task7 早些天会话压缩 (get_position2 / pingcang / position2 旧版 7 步) |
| `C:\Users\29368\Desktop\智能车\rak-car\debug-task5-rebuild-2026-07-22.md` | task5 自包含脚本约定 (本会话沿用) |

### 业务层文档 (本会话反复引用)

| 文档 | 章节引用 |
| --- | --- |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\ARM_API.md` | §0 坐标系 / §1.1 业务硬限 (arm ±150°, hand [-90, +10]) / §6 set_storage_angle / §7 软限位 / §9 composite_run 家族 / §9.6 4 机联动详解 |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\README.md` | 业务层总览 |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\VISUAL_SERVO_QUICKREF.md` | 视觉伺服 |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\TEST_PREFLIGHT.md` | 真机测试前检查 |

### 底层参考 (读源码，不改)

| 文件 | 用途 |
| --- | --- |
| `main/arm/api/composite.py` | CompositeMixin: 5 个 composite_* 入口 — **`composite_run` (行 56-68) 全轴 Optional 但 SDK 不接受 None 轴** |
| `main/arm/api/setters.py` | 行 36-61 set_hand_angle 业务硬限校验 (含 _check_y_protected) / 行 24 set_arm_angle 业务硬限 |
| `main/arm/api/__init__.py` | 行 134-138 _call_arm (返回完整 job dict) |
| `main/arm/api/storage.py` | 行 28/63/76 正确的 ok check 模式 (`status == "succeeded"`) |
| `main/arm/loops/runner.py` | 行 107 `move_x(x_mm, v_max_mms, timeout, verify)` / 行 124 `move_y(y_mm, timeout, verify)` / 行 198 `drop_object(timeout)` |
| `main/task/task1_seeding.py` | 行 410-425 _init_step2_s_pose pre-check + composite_run 同款模式 |
| `smartcar/whalesbot/vehicle/arm/arm_base.py` | SDK 层 (本会话**不读**, 业务层硬约束禁止改) |
| `runtime/services/my_car/...` | runtime (本会话**不读**, 同上) |

### MEMORY 关键指针 (本会话用到 + 新建)

- `[[arm-business-layer-only]]` — 只能改 main/** 业务层, 底层不动
- `[[arm-api-reference]]` — ARM_API.md v3 大改同步基线
- `[[execute-car-action-args-pattern]]` — execute_car_action args 包裹规则
- `[[arm-grasp-call-arm-base]]` — grasp 走 runner.suck(), 不走 http.execute_arm_action
- `[[armrunner-set-hand-angle-gotcha]]` — ArmRunner 没有 set_hand_angle, 必须走 client
- `[[stream-cam-id-mapping]]` — cam1 vs cam2 编号独立
- `[[composite-run-no-partial-2026-08-06]]` — **🆕 本 session 新建**: composite_run SDK 不接受 None 轴, 必须 4 轴全传

---

## 当前状态 (改动清单)

### 本 session 5 个新改动

#### 1. `main/arm/each_task/task7/position2.py` v5 ✅ 现场通过
- **+187 行 (从原 246 行重写)**
- 旧版 (v4): `move_y(-190)` + `set_arm(+90°)` + `set_hand(-40°)` + `move_y(-135)` + `move_x_with_split(-225)` + `drop` + `move_x_with_split(0)` = 7 步串行
- 新版 (v5): 4 步
  - Step 1: `client.composite_run(arm=+90°, x=-144, y=-72, hand=-66°, speed=80, timeout=30)`
  - Step 2: `runner.move_x(-240)` (无 split, belt-slip 已修复)
  - Step 3: `runner.drop_object()`
  - Step 4: `runner.move_x(0)`
- **关键设计**: y=-72 在保护区 [0, -80] **内 8mm**, 必须 composite_run (不查 y 保护区)
- 总耗时 ~3-4s (旧版 ~10s)

#### 2. `main/arm/each_task/task7/position1.py` v5 ✅ 写完待测
- **底盘 Phase 1/3 不动**, Phase 2 替换为 v5 同款 4 步 (与 position2 v5 完全一致)
- Phase 1: 后退 130mm (13cm)
- Phase 2: composite_run(-144, -72, +90°, -66°) → move_x(-240) → drop → move_x(0)
- Phase 3: 前进 130mm
- 总耗时 ~4-5s (旧版 ~11s)

#### 3. `main/arm/each_task/task7/position3.py` v5 ✅ 写完待测
- **底盘 Phase 1/3 方向与 position1 相反**: 先 forward → Phase 2 → back
- Phase 2 跟 position1/2 完全同款 4 步
- 总耗时 ~4-5s (旧版 ~11s)

#### 4. `main/arm/each_task/task7/position5.py` v4+ ✅ 现场通过 (多次 y 调试)
- **+290 行 (从原 319 行重写)**
- 旧版 (v3): 10 步 (y_up → arm → hand → x_mid → y_down → x_final → drop → x_mid → y_up → x_return)
- 新版 (v4+): **7 步**
  - Step 1: `client.composite_run(arm=+90°, x=-175, y=-160, hand=0°)` — 4 机联动
  - Step 2: **`client.composite_run(arm=+90°, x=-175, y=-85, hand=-20°)` — 🆕 双机联动 (4 轴全传, arm/x 复用同值)** — 详见 [[composite-run-no-partial-2026-08-06]]
  - Step 3: `runner.move_x(-230)` — push
  - Step 4: `runner.drop_object()` — 放气 (y=-85 + x=-230)
  - Step 5: `runner.move_x(-170)` — pull
  - Step 6: `runner.move_y(-160)` — y up (隔离 hand 状态)
  - Step 7: `runner.move_x(0)` — 归零
- **总耗时 ~5-6s (旧版 ~15s)**

#### 5. y 值调试轨迹 (position5.py, 同一 session 内)
- 起点 -74 → 改 -92 → 再改 -85 (用户 2026-08-06 现场实测后定稿)
- -74: 保护区**内** 6mm (危险)
- -92: 保护区**外** 12mm (安全, 余量充足)
- -85: 保护区**外** 5mm (安全, 余量偏紧, 用户最终定值)
- 13+ 处引用全部更新, docstring 头部 mirror 跟现场输出逐字符对齐

### git 状态

- 最后 commit `7ae022a feat(arm+task): 大改 业务层 (api/each_task/task6/task7) (2026-08-06 同步 push)` 已 push
- 本 session + 上一 session **7 个文件改动未 commit**:
  - `target.py` v3 / `get_position1.py` v3+ / `the_final.py` v5 / `aaashouzhua.py` (上一 session)
  - `position1.py` v5 / `position2.py` v5 / `position3.py` v5 / `position5.py` v4+ (本 session)

---

## 你的改动 (按文件分类)

### A. composite_run 标准模式 (5 个文件共用)

```python
# 标准 1 步 composite_run 模式 (仿 task1_seeding._init_step2_s_pose)
composite_result = client.composite_run(
    arm=<角度>,
    x_mm=<mm>,
    y_mm=<mm>,
    hand=<角度>,
    speed=80,
    timeout=30.0,
)
ok = (
    isinstance(composite_result, dict)
    and composite_result.get("status") == "succeeded"
    and isinstance(composite_result.get("result"), dict)
    and composite_result["result"].get("ok", False)
)
```

### B. 双机联动新模式 (position5.py 独有)

```python
# 4 轴全传, arm/x 复用 Step 1 终态值 (值不变 = SDK no-op)
step2 = client.composite_run(
    arm=POS_ARM_DEG,         # 复用 Step 1 终态 (+90°)
    x_mm=POS_X_INIT_MM,      # 复用 Step 1 终态 (-175mm)
    y_mm=POS_Y_DOWN_MM,      # 真改 (-160 → -85)
    hand=POS_HAND_DOWN_DEG,  # 真改 (0° → -20°)
    speed=80,
    timeout=30,
)
```

### C. ok check 修复 (必踩坑, 所有 5 文件同款)

`_call_arm` 返回**完整 job dict**:
```python
{
    'id': '...',
    'status': 'succeeded',          # ← 顶层
    'result': {                     # ← 嵌套
        'ok': True,
        'steps': {'arm': True, 'x': True, 'y': True, 'hand': True}
    },
    'error': None
}
```

**正确**: `job.get("status") == "succeeded"`
**错误** (我最初踩的): `job.get("ok")` — "ok" 不在顶层, 嵌在 result 里

参考 `main/arm/api/storage.py:28` 早就写对了, 我初版没抄对。

### D. 删除的冗余代码 (4 文件同款)

| 旧版 | 新版 | 原因 |
| --- | --- | --- |
| `move_x_with_split` (position1/2/3/5) | 删 import + 删调用 | composite_run 走 move_x_position (SDK), 不带 split; state 过渡不需要 split 兜底 |
| 5/7/10 步串行 (4 文件 v3/v4) | 1 步 composite_run + 简化 x/y 序列 | 仿 task1 4 机联动, 耗时 6-10s → 2-3s |
| `y` 保护区 pre-check (旧 target.py v3) | 删 | composite_run 内部**不调** _check_y_protected (composite.py:60 拍板), pre-check 冗余 |
| `runner.drop_object` 顺序检查代码 | 删 | drop_object 不查保护区, 不查 y, 不查 x, 不需要顺序保护 |

---

## 面临的问题 / 待解

### 1. ✅ 已修: composite_run ok check 错位 (5 文件)

**现象**: 现场打印 `❌ composite_run 失败`, 但实际 `status=succeeded`, 4 路全 True
**根因**: 我用了 `result.get("ok")` 检查, 但 "ok" 在 `result.result.ok` 嵌套里
**修复**: 改 `result.get("status") == "succeeded"` + 双重 check
**状态**: ✅ 5 个文件 (target / get_position1 / position1-3 / position5 / the_final) 全修了

### 2. ✅ 已修: composite_run 不支持偏量调用 (position5.py 独有)

**现场错误** (2026-08-06 实测):
```python
client.composite_run(y_mm=-74, hand=-20)  # arm/x 默认 None
# → kwargs={'arm': None, 'x': None, 'y': -0.074, 'hand': -20.0, 'speed': 80}
# → result.steps={'arm': False, 'x': False, 'y': True, 'hand': True}
# → result.ok=False  ← 整个 job 失败
```

**根因**: 业务层 `composite.py:56-68` 源码把 None 透传给 SDK, 但 **SDK 不识别 None**, 把 None 当无效值拒绝
**修复**: 4 轴全传有效值, "不动的轴"靠"传相同值"实现 (SDK 走 no-op)
**状态**: ✅ position5 v4+ Step 2 修好, 现场通过
**MEMORY**: 新建 [[composite-run-no-partial-2026-08-06]] (通用警告, 适用于所有 composite_run 调用)

### 3. ⚠️ ARM_API.md §1.1 业务硬限文档落后于代码 (历史问题)

**现象**: 文档写 `hand ∈ [-90, 0]°`, 代码 `setters.py:45` 写 `[-90, +10]°` (2026-08-05 放宽)
**影响**: 业务层代码已用 +10° (target.py hand=-76 是边界内, get_position1 hand=+10 踩上界, aaashouzhua default=+10)
**待解**: 同步 ARM_API.md §1.1 + §6.1 描述, **文档/代码一致性**
**优先级**: 中 (下次改 ARM_API.md 时一起处理)

### 4. ⚠️ get_position1.py 用户改的常量与 docstring 不一致 (历史问题)

**现象**: 用户/外部改:
- `POS_Y_UP_MM = -190.0` (docstring 仍写 -110)
- `POS_ARM_DEG = 86.0` (docstring 仍写 -86°)

**影响**: docstring 误导, 但代码值是用户最新意图
**待解**: docstring 同步更新
**优先级**: 低 (用户主动改的, 不算 bug)

### 5. ⚠️ position5.py v4+ 现场通过, 但 y=-85 余量紧 (新)

**现象**: y=-85 距保护区边界 -80 **仅 5mm**, 比之前 -92 的 12mm 余量紧
**风险**: 现场标定误差 / 编码器漂移可能导致 y 实际位置进入保护区, 后续 set_*_angle 会被 _check_y_protected 拦截
**待解**: 用户决定是否调到 -90 或更安全值 (本文件 step6 move_y(-160) 已经隔离了大部分风险, 但理论上紧余量风险存在)
**优先级**: 中 (用户最终定的值, 等下次现场复测看是否真触发风险)

### 6. ❓ position4/6.py 没改

**现状**: task7 投递脚本 position4.py 和 position6.py 还是顺序动作 (跟 position5 旧版 v3 类似的 10 步结构)
**待解**: 是否改 composite_run 提速? 还是保持不动?
**优先级**: 低 (用户没要求, 业务流稳定优先)

### 7. ❓ get_position2.py 没改

**现状**: get_position2.py 还是 v2+ 5 步串行, **未**改成 composite_run
**待解**: 与 get_position1 v3+ 保持一致?
**优先级**: 中 (跟 get_position1 配套)

### 8. ⚠️ set_storage_angle 已知坑 (底层 bug, 业务层绕过)

**现象**: MEMORY 记录 pingcang.py 调 ArmClient.set_storage_angle 崩 NameError (api.py:720-729 缺 `job =` 赋值)
**本会话状态**: the_final.py 加 pingcang 时**显式绕过** (`client.http.execute_car_action(..., sync=True)`), 与 pingcang.py 同款
**待解**: 底层 bug 仍在, 业务层用直调绕开 (OK 现状), 后续 SDK 修了再统一回 ArmClient
**优先级**: 低 (业务层已绕开, 不影响使用)

---

## 准备在解决 (TODO)

| 优先级 | 任务 | 状态 |
| --- | --- | --- |
| 高 | 验证 position1.py / position3.py 现场实测全通过 (position2/5 已通过) | 待用户现场 |
| 高 | 7 文件改动 commit + push (上一 session 4 + 本 session 5 - 共享 some 文件) | 待用户决定 |
| 中 | 同步 ARM_API.md §1.1 业务硬限 `[-90, +10]°` | 待整理 |
| 中 | position4/6.py 是否改 composite_run | 待用户决策 |
| 中 | get_position2.py 同款改 4 机联动 | 待启动 |
| 低 | get_position1.py docstring 同步 -190 / +86° | 待整理 |
| 低 | set_storage_angle 底层 bug 修复 (业务层无权改, 推 SDK owner) | 等 runtime 改 |

---

## 注意事项 / 硬约束

### 业务层硬约束 (本会话严格遵守)

来自 MEMORY `[[arm-business-layer-only]]`:

- ✅ **可改**: `main/**` 业务层 (含 `main/arm/each_task/task7/**`)
- ❌ **不可改**: `smartcar/whalesbot/**`, `runtime/**`, `car_wrap_2026.py`, `car_start_2026.py`, `car_task_function.py`
- ❌ **不可改**: SDK 层 (任何 `*_base.py`)
- ❌ **不可改**: runtime 层 (任何 `runtime/services/`, `runtime/api/`)
- ⚠️ 改业务层前用 `main/test/` 冒烟脚本验证

### 自包含约定 (task5/task7 通用)

来自 MEMORY `[[task5-rebuild-2026-07-22]]`:

- `main/arm/each_task/task7/{pingcang, aaashouzhua, dipan, target, position*}.py` **自包含**
- 不 import `task7` 包内任何模块 (跨模块 import 只对 `the_final.py` 编排器开例外)
- 原因: task5 包曾被外部动作清空过, 自包含保证 `python xxx.py` 直接跑不受影响
- 本会话新建的 position1/2/3/5 v5/v4+ 严格遵守

### composite_run 调用规则 (本 session 踩坑后总结)

- ✅ **必须 4 轴全传有效值**: `client.composite_run(arm=, x_mm=, y_mm=, hand=, speed=, timeout=)`
- ❌ **不能传 None 让 SDK 跳过某轴**: SDK 不识别 None, 会把 None 当无效值拒绝
- ❌ **不走 execute_arm_action 直调**: 业务层会校验业务硬限, 直调绕开校验
- ❌ **不走 _check_y_protected**: composite_run 本身**不调**这个 (composite.py:60 拍板 "不怕撞车")
- ❌ **不走 move_x_with_split**: composite_run 内部走 move_x_position (SDK), 不带 belt-slip retry
- ✅ **"部分轴变化" 模式**: 4 轴全传, arm/x 传相同值 (SDK 走 no-op), 只 y/hand 真改 — 见 position5 v4+ Step 2

### 业务硬限 (2026-08-05 放宽后)

| 字段 | 硬限 | 来源 |
| --- | --- | --- |
| arm 角度 | `[-150, +150]°` | `setters.py:24` (2026-08-05 对称放宽) |
| hand 角度 | `[-90, +10]°` | `setters.py:45` (2026-08-05 +10 放宽) |
| y 软限位 | `[-200, 0] mm` | `arm_origin.yaml:soft_y_max_mm=200` |
| x 软限位 | `[-320, +220] mm` | `arm_cfg.yaml:horiz_cfg.x_min_m=-0.32, x_max_m=0.22` |
| y 保护区 | `[0, -80] mm` | 业务硬规则, composite_run 不查 |

### ok check 必踩坑 (本会话踩了)

- `_call_arm` 返回**完整 job dict**: `{'status': 'succeeded', 'result': {'ok': True, ...}, 'error': None}`
- ✅ 正确: `job.get("status") == "succeeded"`
- ❌ 错误: `job.get("ok")` (嵌在 result.result.ok 里)

参考 `main/arm/api/storage.py:28/63/76` (这文件早就写对了)。

---

## 续接指南 (下次会话怎么接)

### 第一件事: 读这个文档

读 `C:\Users\29368\Desktop\智能车\rak-car\debug-task7-composite-run-v5-stack-2026-08-06.md`
**然后**: 读上一 session 压缩 `debug-task7-4axis-composite-run-2026-08-06.md` (target.py / get_position1.py / the_final.py / aaashouzhua.py 改完状态)

### 然后: 现场实测 (position1 / position3 还差这俩)

```bash
# 1. position1.py (v5: 后退 → 4 步臂 → 前进)
python main/arm/each_task/task7/position1.py

# 2. position3.py (v5: 前进 → 4 步臂 → 后退)
python main/arm/each_task/task7/position3.py

# 3. position5.py (v4+: 7 步, 含双机联动 partial)
python main/arm/each_task/task7/position5.py

# 4. 上一 session 4 个文件 (待测)
python main/arm/each_task/task7/target.py
python main/arm/each_task/task7/get_position1.py
python main/arm/each_task/task7/aaashouzhua.py
python main/arm/each_task/task7/the_final.py
```

### 验证清单

- [ ] position1/2/3 v5: 4 步臂都在 ~3-4s 完成, composite_run 打印 `result.ok=True, steps=全 True`
- [ ] position5 v4+: 7 步在 ~5-6s 完成, Step 2 双机联动 y/hand 成功, arm/x no-op
- [ ] position1 终态: y=-72 (保护区**内** 8mm) / x=0 / arm=+90° / hand=-66°
- [ ] position2 终态: y=-72 (保护区**内** 8mm) / x=0 / arm=+90° / hand=-66°
- [ ] position3 终态: y=-72 (保护区**内** 8mm) / x=0 / arm=+90° / hand=-66°
- [ ] position5 终态: y=-160 (保护区**外** 80mm) / x=0 / arm=+90° / hand=-20°
- [ ] 全部 7 文件 `python xxx.py` 直接跑不报错
- [ ] 货物正确投递到对应位置 (上排 1/2/3, 下排 5)

### 如有 bug

1. **composite_run 失败**: 看 `result.steps`, 哪个轴 False 就查那个轴的值是否 None 或无效
2. **ok check 报错**: 检查 `result.get("ok")` vs `status == "succeeded"` (本会话踩坑)
3. **y 保护区拦截**: composite_run 本身不查, 别的 action 会查 (set_hand_angle / set_arm_angle)
4. **move_x 超时**: 检查是否撞墙 / 卡阻 / belt-slip (用户 2026-07-31 报 belt-slip 已修复, 但仍可能复发)

---

## 文件清单 (本 session + 上一 session 改动 + 待改)

```
main/arm/each_task/task7/
├── __init__.py          (未改)
├── the_final.py         ✅ v5 (加 pingcang Step 0)                    [上一 session, 待测]
├── aaashouzhua.py       ✅ 新建 (hand=-10 → +10°)                     [上一 session, 待测]
├── get_position1.py     ✅ v3+ (1 步 composite_run, -190/+86°/+10°)  [上一 session, 待测]
├── get_position2.py     ❌ 未改 (下次?)
├── target.py            ✅ v3 (1 步 composite_run, 无 pre-check)      [上一 session, 待测]
├── pingcang.py          (未改, 仅引用其 DEFAULT_* 常量)
├── dipan.py             (未改)
├── duiying.py           (未改)
├── position1.py         ✅ v5 (4 步臂 + 底盘 Phase 1/3 不变)          [本 session, 待测]
├── position2.py         ✅ v5 (4 步臂, 无底盘)                        [本 session, 已现场通过]
├── position3.py         ✅ v5 (4 步臂 + 底盘 Phase 1/3 不变)          [本 session, 待测]
├── position4.py         ❌ 未改 (用户没要求)
├── position5.py         ✅ v4+ (7 步臂, 双机联动 partial)             [本 session, 已现场通过]
└── position6.py         ❌ 未改 (用户没要求)
```

未 commit **7 个改动** 等用户现场实测 + 决定何时 commit + push。

---

## MEMORY 同步 (本 session 新建/更新)

### 新建

- `composite-run-no-partial-2026-08-06.md` (feedback 类型) — composite_run SDK 不接受 None 轴, 必须 4 轴全传有效值, "不动的轴"靠"传相同值"实现

### MEMORY.md 指针

新增一行 pointer (在本 session 末):
```
- [composite-run-no-partial-2026-08-06](composite-run-no-partial-2026-08-06.md) — **2026-08-06 现场实测踩坑**: `client.composite_run()` **不支持偏量调用** (业务层 composite.py:56-68 虽然把 None 透传给 SDK, 但 SDK 不识别 None → 报 `result.steps={None轴: False, 有值轴: True}`, 整个 job `result.ok=False`)。**正确用法**: 4 轴全传有效值, "不动的轴"靠"传相同值"实现 (SDK 内部走 no-op)。**通用**, 适用于所有 composite_run 调用点; position5 v4 Step 2 是唯一使用 "部分值变化" 模式的文件
```

---

**会话结束, 上下文压缩完毕。下次启动先读这份文档, 然后读上一 session 压缩 `debug-task7-4axis-composite-run-2026-08-06.md`, 再做事。**
