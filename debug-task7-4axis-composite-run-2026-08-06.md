# task7 4 机联动 (composite_run) 重构 — 上下文压缩

**会话日期**: 2026-08-06 (今天)
**作者**: Claude (MiniMax-M3) on behalf of user
**目的**: task7 子目录 4 个文件改成 1 步 composite_run (4 机联动)，仿 task1_seeding 模式
**下次会话第一件事读这个** — 全 task7 业务层 v3/v3+ 重构 + 同步压缩 + 现场实测反馈全在这里

---

## TL;DR

task7 子目录的 4 个文件改成了 1 步 `composite_run` (4 机联动):

| 文件 | 旧版 | **新版** | 状态 |
| --- | --- | --- | --- |
| `the_final.py` (orchestrator) | 顶层无 pingcang | **run() 开头 Step 0 自动 pingcang** | ✅ done |
| `aaashouzhua.py` (新文件) | — | **新建, hand=-10°** (后被用户改为 +10°) | ✅ done |
| `get_position1.py` | 5 步串行 | **1 步 composite_run** (后被用户改为 -190 / +86°) | ✅ done |
| `target.py` | 4 步串行 | **1 步 composite_run** (pre-check 已删) | ✅ done |

现场实测反馈: composite_run **实际成功** (status=succeeded, 4 步全 True), 但我最初写的 ok check 是 bug → 已修。

---

## 主要参考文档

### 项目根 + MEMORY

| 文档 | 用途 |
| --- | --- |
| `C:\Users\29368\Desktop\智能车\rak-car\CLAUDE.md` | 项目总览 + 三层架构 + runtime 锁模型 + arm/target7/visual servo 章节 |
| `C:\Users\29368\.claude\projects\...\MEMORY.md` | 历史踩坑 + 业务硬约束 + 历次会话压缩索引 |
| `C:\Users\29368\Desktop\智能车\rak-car\debug-task7-2026-08-04.md` | task7 上次会话压缩 (get_position1 v2/v2+ / position2 pingcang args pattern) |
| `C:\Users\29368\Desktop\智能车\rak-car\debug-task5-rebuild-2026-07-22.md` | task5 自包含脚本约定 (本会话沿用) |

### 业务层文档 (本会话反复引用)

| 文档 | 章节引用 |
| --- | --- |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\ARM_API.md` | §0 坐标系 / §1.1 业务硬限 (arm ±150°, hand [-90, +10]) / §1.8 选型速查 / §6 set_storage_angle / §7 软限位 / §9 composite_run 家族 / §9.6 4 机联动详解 |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\README.md` | 业务层总览 |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\VISUAL_SERVO_QUICKREF.md` | 视觉伺服 |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\TEST_PREFLIGHT.md` | 真机测试前检查 |
| `C:\Users\29368\Desktop\智能车\rak-car\main\arm\loops\orch_visual.md` | chassis→arm 联调 |

### 底层参考 (读源码，不改)

| 文件 | 用途 |
| --- | --- |
| `main/arm/api/composite.py` | CompositeMixin: 5 个 composite_* 入口 (composite_run 行 56-68) |
| `main/arm/api/setters.py` | 行 36-61 set_hand_angle 业务硬限校验 (含 _check_y_protected) |
| `main/arm/api/__init__.py` | 行 134-138 _call_arm (返回完整 job dict) |
| `main/arm/api/storage.py` | 行 28/63/76 正确的 ok check 模式 (status == "succeeded") |
| `main/task/task1_seeding.py` | 行 410-425 _init_step2_s_pose pre-check + composite_run 同款模式 |
| `smartcar/whalesbot/vehicle/arm/arm_base.py` | SDK 层 (本会话**不读**, 业务层硬约束禁止改) |
| `runtime/services/my_car/...` | runtime (本会话**不读**, 同上) |

### MEMORY 关键指针 (本会话用到)

- `[[arm-business-layer-only]]` — 只能改 main/** 业务层, 底层不动
- `[[arm-api-reference]]` — ARM_API.md v3 大改同步基线
- `[[execute-car-action-args-pattern]]` — execute_car_action args 包裹规则
- `[[arm-grasp-call-arm-base]]` — grasp 走 runner.suck(), 不走 http.execute_arm_action
- `[[armrunner-set-hand-angle-gotcha]]` — ArmRunner 没有 set_hand_angle, 必须走 client
- `[[stream-cam-id-mapping]]` — cam1 vs cam2 编号独立

---

## 当前状态 (改动清单)

### 已完成改动 (本会话)

#### 1. `main/arm/each_task/task7/the_final.py` (orchestrator) ✅

- **+74/-3 行**
- 加 `pingcang as pingcang_mod` import (第 100-110 行)
- 加 4 个 `DEFAULT_PINGCANG_*` 常量 (第 135-150 行, 复用 pingcang.py 默认值)
- `run()` 签名 + 4 个新参数 (第 220-227 行)
- 主循环前加 **Step 0 pingcang 调用** + 错误处理 (第 271-300 行)
- 成功返回 dict 加 `pingcang_result` 字段 (第 451 行)
- CLI 加 4 个 flag: `--skip-pingcang / --pingcang-angle / --pingcang-speed / --pingcang-timeout`
- 改版标记: v4 → **v5 (pingcang 预备)**

#### 2. `main/arm/each_task/task7/aaashouzhua.py` (新建文件) ✅

- **新建, 156 行** (后被用户/linter 改: `DEFAULT_ANGLE_DEG = -10` → **+10**)
- 命名约定: `aaa` 前缀 = 任务起点预备动作 (与 `the_final.py` 的 `the` 前缀命名类比)
- 默认 **-10°** (用户指定) → 后续被改 +10°
- 走 `client.set_hand_angle(angle, speed, timeout)` 业务层 (与 pingcang 不同, 无已知 bug)
- ⚠️ **业务硬限已放宽**: `setters.py:45` 显示 `[-90, +10]°` (2026-08-05), 但 `ARM_API.md` §1.1 还是 `[-90, 0]`, **文档落后于代码**
- 自包含: 只依赖 `main.arm.ArmClient`

#### 3. `main/arm/each_task/task7/get_position1.py` ✅

- 改动 1 (v3 重写): 5 步串行 → **4 步串行**
  - y=-110 / x=0 / arm=-86° / hand=+10°
  - 用户改: `POS_Y_UP_MM=-110` → **-190**, `POS_ARM_DEG=-86` → **+86** (其他保留)
- 改动 2 (v3+ 重写): 4 步串行 → **1 步 composite_run**
  - 仿 task1_seeding.py 同款 4 机联动
  - `client.composite_run(arm=+86°, x=0, y=-190, hand=+10°, speed=80, timeout=30.0)`
  - 删 `move_x_with_split` import
- 改动 3 (bug 修): ok check `result.get("ok")` → `status == "succeeded"`
- 改版标记: v1 → v2 → v2+ → v3 → **v3+**

#### 4. `main/arm/each_task/task7/target.py` ✅

- 改动 1 (v3): 4 步串行 (move_y / set_arm / set_hand / move_x_with_split) → **1 步 composite_run**
  - 仿 task1_seeding 同款
  - `client.composite_run(arm=+90°, x=0, y=-120, hand=-76°, speed=80, timeout=30.0)`
  - 删 `move_x_with_split` import
- 改动 2 (pre-check 删): 用户拍板**不要 pre-check** (composite.py:60 注释 "23:31 用户拍板: 不怕撞车! _check_y_protected 去掉! 要速度!")
  - 删 `if state.y_mm > Y_PRECHECK_THRESHOLD_MM: runner.move_y(...)` 整块
  - 删 `Y_PRECHECK_THRESHOLD_MM` 常量
  - 更新 docstring 解释为什么 pre-check 冗余
- 改动 3 (bug 修): ok check 同 target.py
- 用户/linter 后续改 `DEFAULT_OCR_PROMPT` (本会话**没改**, 用户/linter 行为, 不revert)

### git 状态

- 最后一次 commit `7ae022a feat(arm+task): 大改 业务层 (api/each_task/task6/task7) (2026-08-06 同步 push)` 已 push origin
- 本会话 4 个文件改动**未 commit** (target.py / get_position1.py / aaashouzhua.py / the_final.py)

---

## 你的改动 (按改动类型分类)

### A. composite_run 改写 (4 文件共用模式)

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

### B. ok check 修复 (必踩坑)

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

**正确 check**: `job.get("status") == "succeeded"`
**错误 check** (我最初踩的): `job.get("ok")` — "ok" 不在顶层, 嵌在 result 里

参考: `main/arm/api/storage.py:28` 早就写对了, 我的代码初版没抄对。

### C. 删除的冗余代码

| 旧版 | 新版 | 原因 |
| --- | --- | --- |
| `move_x_with_split` (get_position1 / target) | 删 import + 删调用 | composite_run 走 move_x_position (SDK), 不带 split; 状态过渡不需要 split 兜底 |
| y pre-check (target) | 删 `if state.y_mm > -80: move_y()` | composite_run 内部**不调用** _check_y_protected (composite.py:60 拍板), pre-check 冗余 |
| 5 步串行 (get_position1 v2+/v3) | 1 步 composite_run | 仿 task1 4 机联动, 耗时 6-8s → 2-3s |
| 4 步串行 (target v2) | 1 步 composite_run | 同上 |

---

## 面临的问题 / 待解

### 1. bug 复盘 — composite_run ok check 错位

**现象**: 现场打印 `❌ composite_run 失败`, 但实际 `status=succeeded`, 4 路全 True
**根因**: 我用了 `result.get("ok")` 检查, 但 "ok" 在 `result.result.ok` 嵌套里
**修复**: 改 `result.get("status") == "succeeded"` + 双重 check
**状态**: ✅ target.py + get_position1.py 都修了

### 2. ⚠️ `ARM_API.md` 文档落后于代码

**现象**: 文档写 `hand ∈ [-90, 0]°`, 代码 `setters.py:45` 写 `[-90, +10]°` (2026-08-05 放宽)
**影响**: 业务层代码已用 +10° (target.py hand=-76 是边界内, get_position1 hand=+10 踩上界, aaashouzhua default=+10)
**待解**: 同步 ARM_API.md §1.1 + §6.1 描述, **文档/代码一致性**
**优先级**: 中 (下次改 ARM_API.md 时一起处理)

### 3. ⚠️ `get_position1.py` 用户改的常量与 docstring 不一致

**现象**: 用户/外部改:
- `POS_Y_UP_MM = -190.0` (docstring 仍写 -110)
- `POS_ARM_DEG = 86.0` (docstring 仍写 -86°)

**影响**: docstring 误导, 但代码值是用户最新意图
**待解**: docstring 同步更新
**优先级**: 低 (用户主动改的, 不算 bug)

### 4. ⚠️ MEMORY 已知坑 — `api.py:720-729` set_storage_angle 缺 `job =` 赋值

**现象**: MEMORY 记录 pingcang.py 调 ArmClient.set_storage_angle 崩 NameError
**本会话状态**: the_final.py 加 pingcang 时**显式绕过** (`client.http.execute_car_action(..., sync=True)`), 与 pingcang.py 同款
**待解**: 底层 bug 仍在, 业务层用直调绕开 (OK 现状), 后续 SDK 修了再统一回 ArmClient
**优先级**: 低 (业务层已绕开)

### 5. ⚠️ `target.py` 默认 prompt 被用户/linter 改

**现象**: 用户/linter 改 `DEFAULT_OCR_PROMPT` 加 "**严格按从左到右、从上到下顺序**" 和 "每行 3 个名字用半角空格分隔" + "2×3 网格布局"
**影响**: prompt 更具体, row-major 解析更稳
**待解**: 无 (用户主动改, 不 revert)
**优先级**: 无

### 6. ❓ get_position2.py 没改

**现状**: get_position2.py 还是 v2+ 5 步串行, **未**改成 composite_run
**待解**: 与 get_position1 保持一致?
**优先级**: 中 (跟 get_position1 配套)

### 7. ❓ position1-6.py 没改

**现状**: task7 投递脚本 (position1.py ~ position6.py) 还是顺序动作
**待解**: 是否改 composite_run 提速?
**优先级**: 低 (用户没要求, 业务流稳定优先)

### 8. ❓ composite_run 在 belt-slip 场景的安全性

**现状**: composite_run 走 move_x_position (SDK), **不带 belt-slip retry**
**已知 belt-slip 修复**: 用户 2026-07-31 说 "belt-slip 已修复"
**当前文件影响**:
- get_position1 / target 是**状态过渡**, 不需要 split 兜底
- 真要 belt-slip 兜底: 业务层单独调 `runner.move_x_with_split(...)`
**待解**: 实测如果 belt-slip 复发, 是否需要 split 兜底 (目前不复发, 暂不动)
**优先级**: 低

---

## 准备在解决 (TODO)

| 优先级 | 任务 | 状态 |
| --- | --- | --- |
| 高 | 验证 4 文件 composite_run 现场实测全通过 | 待用户现场 |
| 中 | 同步 ARM_API.md §1.1 业务硬限 `[-90, +10]°` | 待整理 |
| 中 | get_position2.py 同款改 4 机联动 | 待启动 |
| 低 | get_position1.py docstring 同步 -190 / +86° | 待整理 |
| 低 | position1-6.py 是否改 composite_run | 待用户决策 |

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
- 本会话新建的 `aaashouzhua.py` 严格遵守

### composite_run 调用规则

- ✅ **走业务层**: `client.composite_run(arm=, x_mm=, y_mm=, hand=, speed=, timeout=)`
- ❌ **不走 execute_arm_action 直调**: 业务层会校验业务硬限, 直调绕开校验
- ❌ **不走 _check_y_protected**: composite_run 本身**不调**这个 (composite.py:60 拍板 "不怕撞车")
- ❌ **不走 move_x_with_split**: composite_run 内部走 move_x_position (SDK), 不带 belt-slip retry

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
- ❌ 错误: `job.get("ok")` (嵌在 result 里)

参考 `main/arm/api/storage.py:28/63/76` (这文件早就写对了)。

---

## 续接指南 (下次会话怎么接)

### 第一件事: 读这个文档

读 `C:\Users\29368\Desktop\智能车\rak-car\debug-task7-4axis-composite-run-2026-08-06.md`

### 然后: 现场实测

```bash
# 1. get_position1.py (v3+ 1 步 composite_run)
python main/arm/each_task/task7/get_position1.py

# 2. target.py (v3 1 步 composite_run, 无 pre-check)
python main/arm/each_task/task7/target.py

# 3. aaashouzhua.py (新建, hand=+10°)
python main/arm/each_task/task7/aaashouzhua.py

# 4. the_final.py (v5 加 Step 0 pingcang)
python main/arm/each_task/task7/the_final.py
```

### 验证清单

- [ ] 4 文件 `python xxx.py` 直接跑不报错
- [ ] composite_run job dict 看到 `status=succeeded, result.steps={arm, x, y, hand: 全 True}`
- [ ] get_position1 终态: y=-190 / x=0 / arm=+86° / hand=+10°
- [ ] target 终态: y=-120 / x=0 / arm=+90° / hand=-76°
- [ ] aaashouzhua: 手爪到 +10° (UP 不是, P 姿态上界)
- [ ] the_final.py: 开头有 pingcang 调用, 然后进主循环

### 如有 bug

1. 先看 job dict 实际结构 (`print(composite_result)` 调试)
2. 检查 `result.get("ok")` vs `status == "succeeded"` (本会话踩坑)
3. 检查 `_check_y_protected` 是否被错误调用 (composite_run 本身不调, 别的 action 会调)

---

## 文件清单 (本会话改动 + 待改)

```
main/arm/each_task/task7/
├── __init__.py          (未改)
├── the_final.py         ✅ v5 (加 pingcang Step 0)  ← 未 commit
├── aaashouzhua.py       ✅ 新建 (hand=-10 → +10°)
├── get_position1.py     ✅ v3+ (1 步 composite_run)  ← 未 commit
├── get_position2.py     ❌ 未改 (下次?)
├── target.py            ✅ v3 (1 步 composite_run, 无 pre-check)  ← 未 commit
├── pingcang.py          (未改, 仅引用其 DEFAULT_* 常量)
├── dipan.py             (未改)
├── duiying.py           (未改)
├── position1.py         ❌ 未改 (用户没要求)
├── position2.py         ❌ 未改
├── position3.py         ❌ 未改
├── position4.py         ❌ 未改
├── position5.py         ❌ 未改
├── position6.py         ❌ 未改
```

未 commit 4 个改动待用户决定何时 commit + push。

---

**会话结束, 上下文压缩完毕。下次启动先读这份文档。**