# task5 / last_* batch 整合会话压缩文档

**日期**: 2026-07-29
**范围**: task5 取放球系列脚本 (last_yellow_to_high / last_blue_to_high / last_yellow_to_low / last_blue_to_low) 的整套整合、摆臂标准化、阈值标定、命名整理
**会话状态**: 改完等现场跑一次确认 prep_pose 不撞车; 文档化后压缩上下文

---

## 0. 这次会话**主要调用/依赖**的文档清单

下次回来要查的入口文件（按优先级）:

| 路径 | 用途 |
|---|---|
| `CLAUDE.md` | 项目总入口 (架构 / 三层 / runtime API / 业务层 client) |
| `main/arm/README.md` + `ARM_API.md` + `QUICKSTART.md` | 机械臂业务层说明 |
| `main/API_REFERENCE.md` / `API.md` / `CAPABILITY_LIST.md` | 业务层 client API 速查 |
| `debug-task4-rebuild-2026-07-28.md` (repo root) | task4 重建基线 (BALL_VERIFIED_* 来源, BALL_VERIFIED_* 当前按 task4 target1 y=-150 标定) |
| `debug-task5-rebuild-2026-07-22.md` (repo root) | task5 重建基线 (high_tower / low_tower / get_blue / get_yellow / grasp_5) |
| `main/arm/api.py` | 业务层 ArmClient (set_arm_angle, set_hand_angle, x/y 安全门) |
| `main/arm/each_task/task4/constants.py` | task4 BALL_VERIFIED_* / TARGET_* (业务层**不能复用** TARGET_AREA_* 阈值) |
| `main/arm/each_task/task4/target2.py` | `fetch_balls` / `_label_to_color` / `_verify_ball_in_target1_pose` (球检测复用入口) |
| `main/arm/each_task/task5/_last_loop.py` | 共享骨架 (prep_pose + detect + 循环) |
| `main/arm/each_task/task5/get_blue.py` / `get_yellow.py` | 取物位姿唯一定义源 |
| `main/arm/each_task/task5/target_blue.py` / `target_yellow.py` | 观察位姿 + task5 专属 DETECT_* 阈值 |
| `main/arm/each_task/task5/test1..4_from_*_to_*.py` | 整合吸气+放塔脚本 |

---

## 1. 主要场景与文件角色

### 取放球全流程 (8 步走完一颗)

```
[prep_pose] y/x/arm/hand 摆到观察位 ──> [detect_balls] task_feed 拿球列表 + 颜色过滤
                                              │
                                              ▼
                                       N = 检测到几颗
                                              │
            ┌──────── for round in 1..N ────────────────┐
            │                                            │
            ▼                                            ▼
   testN_run(client, runner)                  testN_run(client, runner)
   ├─ get_*.run  (5 步: y_down→x→arm→hand→y_pickup)
   ├─ grasp(True) 吸气
   ├─ sleep(5s) 独立保持
   ├─ high_tower.run / low_tower.run  (4 步: y→arm→hand→x)
   ├─ grasp(False) 放气
   └─ reset_x 撞墙归零
```

### 四个 last_* 的接线矩阵 (2026-07-29 最终态)

| 脚本 | prep_pose (用户指定 2026-07-29: 统一 target_blue) | 检测 | 循环 |
|---|---|---|---|
| `last_blue_to_high.py` | target_blue (y=-200/x=-40/arm=90/hand=0) | `target_blue.detect_balls(color_filter=blue)` | `test2_run` (get_blue→high_tower) |
| `last_blue_to_low.py` | target_blue (同) | `target_blue.detect_balls(color_filter=blue)` | `test4_run` (get_blue→low_tower) |
| `last_yellow_to_high.py` | target_blue (同) | `target_yellow.detect_balls(color_filter=yellow)` | `test1_run` (get_yellow→high_tower) |
| `last_yellow_to_low.py` | target_blue (同) | `target_yellow.detect_balls(color_filter=yellow)` | `test3_run` (get_yellow→low_tower) |

**关键点**: 所有 4 个 last_* 的 prep_pose 都**统一用 target_blue.TARGET1_*** (用户明确要求)。prep 与 grab 解耦: prep 在观察位姿 (target_blue), 进 test_run 后被 get_*.run 推到抓取位姿 (get_blue/get_yellow)。

---

## 2. 本次会话所有改动 (按文件分类)

### 2.1 `main/arm/each_task/task5/get_blue.py` / `get_yellow.py`

| 项 | 旧 | 新 | 备注 |
|---|---|---|---|
| 大臂 `GET_*_ARM_DEG` | -5° / -6° | **85°** | 业务硬限 [+90, -150] 内, ≥+30 安全姿态带外 |
| 抬回位 y `GET_*_Y_MM` | -85 / -82 | **-70** | 沿革 -88 → -85/-82 → -75 → -70 |
| get_yellow x `GET_YELLOW_X_MM` | -72 | **-68** | 今天第二次改; 与 target_yellow 同步 |
| step 4 `set_hand_angle` 走底层 | (保留) | (保留) | 大臂 85° 后 wrapper 已不拦, 保留直调仅为跟兄弟脚本一致 |

### 2.2 `main/arm/api.py` — 修了一个必崩 bug

`set_storage_angle()` 第 720 行漏 `job =`:
```python
# 旧
self._call_car("set_storage_angle", timeout=timeout,
               angle=angle, speed=speed, sync=True)
# 新
job = self._call_car("set_storage_angle", timeout=timeout,
                     angle=angle, speed=speed, sync=True)
```
崩溃发生在 `sync=True` 已返回 (舵机动作已下) 之后拼返回 dict 时 → **硬件动了但脚本红了**。从 commit `ca896c2` 引入起就一直坏, 业务层凡调过 `set_storage_angle` 都被 try/except 吞掉当成"舵机没反应"。

### 2.3 `main/arm/each_task/task5/target_blue.py` / `target_yellow.py` — 新建

从 `target1.py` 改名/复制:
- `target_blue.py` 由 target1.py 改名, `DETECT_COLOR_FILTER="blue"`
- `target_yellow.py` 由 target_blue.py 复制, x=-68, `DETECT_COLOR_FILTER="yellow"`
- 不提供 `--color` CLI 开关 (文件名与颜色绑定)

加 **task5 专属 DETECT_* 阈值** (2026-07-29 实测, target_blue 位姿):
| 阈值 | 值 | 备注 |
|---|---|---|
| `DETECT_SCORE_MIN` | 0.60 | 实测 0.683~0.916, 留 0.08 余量保住边缘球 |
| `DETECT_AREA_MIN` | 0.10 | ⚠️ **关键差异**: task4 的 0.20 在此位姿会把球全筛光 (实测 area 0.148~0.168) |
| `DETECT_AREA_MAX` | 0.24 | 留余量 |
| `DETECT_ASPECT_TOL` | 0.8 | 沿用 task4 (aspect 0.67~0.71 与 task4 实测 0.70 一致) |
| `verify_target1_pose` | 永远 False | BALL_VERIFIED_* 是 task4 target1 位姿标定, 本位姿不同 |

⚠️ **绝对不能复用** `task4/constants.py.TARGET_*` (那是 task4 target1 近景标定), 否则 0 个球。

### 2.4 `main/arm/each_task/task5/test1..4_from_*_to_*.py` — 文档同步

- 跟随 `GET_*_Y_MM` 从 -85/-82 → -75 → -70, 全文 -82 替换为 -75, 再到 -70
- y 抬回位描述、grasp_window 字段、阶段 1 描述、吸气期间吸盘位姿描述全部更新
- "离地 85mm" 改为 "离地高度待实测 (y 已改 -70)" (高度实测跟具体 y 有关)
- `test1/test3` 里 `get_yellow (move_x -72)` → `get_yellow (move_x -68)` (test2/test4 同款)

### 2.5 `main/arm/each_task/task5/last_yellow_to_high.py` — 重构为 thin wrapper

- 删掉原内联 `_prep_pose_for_detect` / `run` / `build_parser` (200+ 行)
- 改为 import `_last_loop` 共享骨架 (~50 行)
- prep_pose 参数最终态: `target_module.TARGET1_*` (target_blue 统一)

### 2.6 `main/arm/each_task/task5/last_blue_to_high.py` / `last_blue_to_low.py` / `last_yellow_to_low.py` — 新建

- 三个 thin wrapper, ~50 行/个, 通过 `_last_loop` 共享骨架
- 同样 prep_pose 参数最终态: `target_module.TARGET1_*` (target_blue 统一)

### 2.7 `main/arm/each_task/task5/_last_loop.py` — 新建共享骨架

```python
def _prep_pose(client, runner, target_module, log_prefix,
               y_mm=None, x_mm=None, arm_deg=None, hand_deg=None):
    """默认从 target_module.TARGET1_* 拿参数, 显式传 y_mm/x_mm/arm_deg/hand_deg 时覆盖"""

def run_last_loop(..., prep_pose=True, prep_y_mm=None, prep_x_mm=None,
                  prep_arm_deg=None, prep_hand_deg=None, ...):
    """prep_pose → detect → 循环 test_run_fn"""

def build_last_parser(log_prefix, color_label, test_log_prefix): ...
def main_with_args(args, log_prefix, target_module, test_run_fn,
                   test_log_prefix, color_label,
                   prep_y_mm=None, prep_x_mm=None,
                   prep_arm_deg=None, prep_hand_deg=None): ...
```

CLI 阈值默认 None, `_resolve()` 运行时从 `target_module.DETECT_*` 取 (target_* 阈值改 wrapper 自动跟)。sentinel default 替代之前 last_yellow_to_high 里的脆弱 "硬编码默认值对比" 写法。

### 2.8 `main/arm/test/test_storage_close.py` — 同步修 bug

`set_storage_angle` wrapper 返回的是业务 dict (`{"ok", "angle", "raw_job"}`), 测试却在读 `job.get("status")` / `job.get("error")`, 改从 `raw_job` 里取 `status`/`error`, `ok` 用业务字段。

---

## 3. 当前 task5 目录文件图 (2026-07-29)

```
main/arm/each_task/task5/
├── __init__.py                       (文档级, 已过时, 仍指向旧的 a/b1/b2/b3/c 六阶段)
├── constants.py                      (若存在; 现在 task5 自包含, 业务层不依赖)
│
├── get_blue.py                       ★ 蓝球取物位姿唯一定义源 (GET_BLUE_*)
├── get_yellow.py                     ★ 黄球取物位姿唯一定义源 (GET_YELLOW_*)
├── high_tower.py                     高储存仓放球 (4 步: y→arm→hand→x)
├── low_tower.py                      低储存仓放球 (同款 4 步)
├── grasp_5.py                        单独吸气测试
│
├── target_blue.py                    ★ 蓝球观察位姿 (TARGET1_*) + task5 DETECT_* 阈值
├── target_yellow.py                  ★ 黄球观察位姿 (TARGET1_*) + task5 DETECT_* 阈值
│
├── test1_from_yellow_to_high.py      ★ get_yellow+high_tower 整合 (吸气 v5)
├── test2_from_blue_to_high.py        ★ get_blue+high_tower 整合
├── test3_from_yellow_to_low.py       ★ get_yellow+low_tower 整合
├── test4_from_blue_to_low.py         ★ get_blue+low_tower 整合
│
├── _last_loop.py                     ★ 共享骨架 (prep_pose + detect + 循环 + CLI)
├── last_yellow_to_high.py            ★ thin wrapper (test1)
├── last_blue_to_high.py              ★ thin wrapper (test2)
├── last_yellow_to_low.py             ★ thin wrapper (test3)
├── last_blue_to_low.py               ★ thin wrapper (test4)
│
└── .target4_v6.bak / .target4_v7-final.bak   (历史备份, task4 的, 误放这里)
```

★ = 本次会话涉及/新建/重构

---

## 4. 已验证/实测过的功能

| 测试 | 结果 |
|---|---|
| `get_yellow.x -72 → -68` 语法 + --help | ✓ |
| `target_yellow.x -72 → -68` 同步文档 | ✓ |
| `target_yellow.detect_balls(timeout=2.0)` 连真车 | ✓ (0 个球因 score 0.542 < 0.60; `--score-min 0.5` → 1 个黄球) |
| `target_blue.detect_balls(timeout=2.0)` 连真车 | ✓ (3 raw detections → 2 个蓝球) |
| `set_storage_angle` 修复后 `test_storage_close.py` | ✓ EXIT=0 |
| 4 个 last_* `build_parser().parse_args(['--balls','2','--no-prep'])` | ✓ |
| 4 个 last_* `import` + `LOG_PREFIX`/`COLOR_LABEL` | ✓ |
| 5 个新/改文件语法 (`_last_loop` + 4 个 wrapper) | ✓ |

**未实测** (需要上电跑):
- 实际机械臂运动 (target_blue prep 4 步 + testN_run 5/4 步)
- 球检测在 target_blue prep 位姿下 (相对 task4 target1 位姿) 的实际识别率
- 2 轮 test1_run 的总耗时 (预估 ~110-130s, 含 prep)

---

## 5. 面临的问题 / 待解决

### 5.1 球检测阈值的"目标位姿依赖"问题

- `BALL_VERIFIED_*` (task4) 是 task4 target1 位姿 (y=-150) 标定
- `DETECT_*` (task5, target_blue/target_yellow) 是 target_blue 位姿 (y=-200, arm=90°) 标定
- prep_pose 改后 (现在 target_blue, 之前 get_*, 再之前 target_*), 球在画面里的 cx/cy/w/h 都变
- **当前 prep 在 target_blue 位姿, 应使用 task5 的 DETECT_* 阈值** (已对齐, 不动)
- **若改 prep 位姿, 必须重测并标定新一套阈值**

### 5.2 当前画面黄色球 score 偏低

实测当前画面黄球 score=0.542, 默认 `DETECT_SCORE_MIN=0.60` 过不了。
- 解决方案 A: 把两个 target_*.py 的 `DETECT_SCORE_MIN` 降到 0.50 (用户拍)
- 解决方案 B: 提高画面质量/光照 (不动代码)

### 5.3 prep→grab 的过渡动作 (轻微)

prep 在 (y=-200, x=-40, arm=90°), testN_run 内 get_*.run 推到 (y=-130, x=0/-68, arm=85°)。
- 中间多了 4 步运动过渡 (~0.5-1s)
- arm 差 5° (90→85) 单独舵机动作, ~0.5s
- 接受, 不动

### 5.4 grab_module import 是死代码 (cosmetic)

4 个 last_* wrapper 里都 `import main.arm.each_task.task5.get_blue/yellow as grab_module`, 但 prep 改用 target_module.* 后 grab_module 没人用。
- 留着无害, 1 行 import
- 注释里"退回 get_* prep 怎么改"那段也过期了, 但作为回退路径仍有文档价值
- 决定: 留着, 不动

### 5.5 task5/__init__.py 文档级过时

仍描述旧的 a/b1/b2/b3/c 六阶段设计, 不影响运行 (没有 __all__ 也没 import)。
- 不影响功能, 不动

### 5.6 reset_x 撞墙后位置读数偏差

实测撞墙后 `realtime x=-1.175mm` (定义应为 0)。calibrate 框架已知坏 (ARM_API §11), 这是预期行为, 不动。

---

## 6. 准备做的事 (下次会话第一件事读这个)

### 6.1 现场实测 last_blue_to_high / last_yellow_to_high

```bash
# 1) 单球冒烟 (1 轮, 不预跑 prep)
python main/arm/each_task/task5/last_blue_to_high.py --no-prep --balls 1
python main/arm/each_task/task5/last_yellow_to_high.py --no-prep --balls 1

# 2) 单球冒烟 (带 prep)
python main/arm/each_task/task5/last_blue_to_high.py --balls 1
python main/arm/each_task/task5/last_yellow_to_high.py --balls 1

# 3) 双球实测
python main/arm/each_task/task5/last_blue_to_high.py
python main/arm/each_task/task5/last_yellow_to_high.py
```

观察:
- prep_pose 4 步是否能完成 (target_blue: y=-200 / x=-40 / arm=90° / hand=0°)
- 检测到几颗球 (当前画面 1 黄 + 2 蓝, 默认阈值应能识别 3 颗; 但黄球 score 0.542 < 0.60 可能漏)
- 球数与运行轮数是否匹配
- 总耗时 (预估 ~110-130s / 2 轮)

### 6.2 若球检测漏球, 调阈值

```bash
# 看 raw detections
curl -s http://10.253.70.20:5050/v1/realtime/vision/task | python -m json.tool

# 临时放宽 score 阈值
python main/arm/each_task/task5/last_yellow_to_high.py --score-min 0.50 --balls 1

# 现场定档后, 同步修改 target_blue.py / target_yellow.py 的 DETECT_SCORE_MIN
```

### 6.3 待办 (优先级排序)

| # | 待办 | 优先级 |
|---|---|---|
| 1 | 现场跑一次 4 个 last_*, 确认 prep_pose 不撞车 | P0 (本次会话结束前应跑) |
| 2 | 黄色球 score 阈值降到 0.50 (若现场实测仍漏) | P1 |
| 3 | `task5/__init__.py` 文档级更新 | P3 (不影响运行) |
| 4 | 清理 4 个 wrapper 里 dead `grab_module` import | P3 (cosmetic) |
| 5 | 业务层**扫一遍**有没有别的 `set_storage_angle`-like 调用方被 try/except 吞掉 (api.py 的 bug 揭示业务层可能有同类盲点) | P2 |

---

## 7. 注意事项 / 硬约束 (下次会话务必遵守)

### 7.1 业务层边界

- ✅ **可改**: `main/**` 全部 (`main/arm/`, `main/chassis/`, `main/misc/`, `main/tasks/`, `main/api_client.py`, `main/ws_client.py`, `main/settings.py`)
- ❌ **不可改**:
  - `smartcar/whalesbot/**` (SDK, 业务层无权)
  - `runtime/**` (运行时, 业务层无权)
  - `car_wrap_2026.py` (MyCar 类, 业务层无权)
  - `car_start_2026.py` / `car_task_function.py` (顶层 monolith)
  - `config_car.yml` (赛道标定)

### 7.2 位置读取 (容易踩坑)

- **x/y 一律走 realtime**: `_read_x_mm_realtime()` / `_read_y_mm_realtime()` (走 20Hz arm_feed)
- **x_get_position / y_get_position 已坏**: calibrate 框架坏, 实测同位置不同时间读数飘 0.3/22.5/46.9mm (ARM_API §11)
- `get_state()` 内部仍调坏路径 (api.py:1082 `_read_raw_state`), 丢步检查 (`move_x`/`move_y`) 是假报风险

### 7.3 task_feed (容易误判)

- task_feed 守护线程 runtime 默认 30Hz 常开, **不需要手动启**
- 检测阈值用 task5 专属 (`DETECT_*` in target_blue.py / target_yellow.py), **绝不**用 task4 TARGET_*
- `verify_target1_pose=False` 永远写死 (task4 BALL_VERIFIED_* 不能在本位姿用)

### 7.4 安全门 (硬限)

- 大臂业务硬限: `[+90, -150]` (2026-07-27 v3 重定义)
- 大臂"安全姿态"带: `[-30, +30]` (带外可跳 y 保护区)
- y 保护区: `[0, -30]` (move_x / set_arm_angle 会被拦; move_y 不会)
- set_storage_angle **无软限** (用户原话: "这个存储仓舵机不要任何软限制")
- soft_y_max_m = 0.2 (arm_origin.yaml), -200 边界闭区间通过

### 7.5 x 轴同步带打滑 (老问题)

- 单次有效行程 24-46mm, 跨 bin 必分段 (用 `_move_x_with_split` test_x_to_150.py 模式)
- x_speed_with_safety watchdog: 2s 内 x_mm 无变化自动停机 (memory [[x-speed-safety-watchdog]])
- get_blue 用 `reset_x` 撞墙定原点 (不走 move_x 分段, belt-slip 兼容)
- get_yellow / target_* 用 `_move_x_with_split` 分段

### 7.6 grab (吸盘) 调用 (易踩坑)

- **必须**用 `http.execute_arm_action("grasp", bool(on), timeout=..., sync=True)` 直调
- **不能**走 `ArmClient._call_arm("grasp", ...)` (timeout 位置参陷阱 + kwargs 透传到 arm_base.grasp TypeError 静默失败)
- 详见 `arm-grasp-call-arm-base` memory

### 7.7 api.py 已知 bug 模式

- `set_storage_angle` 第 720 行漏 `job =` (已修)
- 业务层用 `try/except Exception` 吞掉的代码路径, 可能遮蔽同类 NameError
- 排查模式: 业务层代码"舵机没反应"但日志无异常 → 高度怀疑被 try/except 吞掉

---

## 8. 一句话总结 (TL;DR for 上下文压缩)

> task5 取放球 batch 整合完成: 4 个 `last_*_to_{high,low}` thin wrapper 通过共享骨架 `_last_loop.py` 跑"prep → 检测 → 循环调 testN_run"; prep_pose 用户指定统一用 `target_blue.TARGET1_*` (y=-200/x=-40/arm=90°/hand=0°); 颜色靠 `DETECT_COLOR_FILTER` 区分; task5 专属 `DETECT_*` 阈值 (`area∈[0.10,0.24]`, `score≥0.60`) 在 target_blue 位姿下实测; **业务层唯一可改的入口**; 下次回来先现场跑 4 个 last_* 确认 prep_pose 不撞车, 再决定是否降 score 阈值。